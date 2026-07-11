/**
 ****************************************************************************
 * @file    pedal_logic.c
 * @brief   Pure accelerator-pedal plausibility pipeline (see pedal_logic.h).
 *
 *          This is the SINGLE implementation of the pedal plausibility logic.
 *          sensor_manager.c calls it from Pedal_Update() after reading the
 *          ADC, and the host unit tests call it directly — there is no second
 *          copy of the algorithm to drift out of sync.
 ****************************************************************************
 */

#include "pedal_logic.h"

void Pedal_StateInit(PedalState *st, uint16_t adc_min, uint16_t adc_max)
{
    st->adc_min        = adc_min;
    st->adc_max        = adc_max;
    st->raw_adc        = 0U;
    st->raw_adc2       = 0U;
    st->pct            = 0.0f;
    st->pct_raw        = 0.0f;
    st->pct_prev       = 0.0f;
    st->ema            = 0.0f;
    st->plausible      = true;
    st->contradict     = false;
    st->ema_primed     = false;
    st->rate_fault     = false;
    st->recovery_count = 0U;
}

static float Pedal_RawToPercent(const PedalState *st, uint16_t raw)
{
    if (raw <= st->adc_min) return 0.0f;
    if (raw >= st->adc_max) return 100.0f;
    return (float)(raw - st->adc_min) * 100.0f
         / (float)(st->adc_max - st->adc_min);
}

void Pedal_ProcessSamples(PedalState *st, uint16_t adc1, uint16_t adc2)
{
    st->raw_adc  = adc1;
    st->raw_adc2 = adc2;

    /* 2. Dual-sample consistency check */
    uint16_t diff_raw = (adc1 >= adc2) ? (uint16_t)(adc1 - adc2)
                                       : (uint16_t)(adc2 - adc1);
    if (diff_raw > PEDAL_SAMPLE_TOLERANCE) {
        /* Consecutive samples disagree — noise or transient fault.
         * Zero safe output; if a rate-fault recovery was in progress reset
         * the consecutive counter (samples are unreliable).  rate_fault is
         * intentionally NOT cleared — the contradiction is a separate fault
         * type that supersedes it but recovery must restart from zero.     */
        st->contradict      = true;
        st->plausible       = false;
        st->pct             = 0.0f;
        st->recovery_count  = 0;
        return;
    }
    st->contradict = false;

    /* 3. Use average of both samples for best accuracy */
    uint16_t avg_raw = (uint16_t)(((uint32_t)adc1 + adc2) / 2U);

    /* 4. Range validation — detect open/short circuit. */
#if PEDAL_ADC_FAULT_LO > 0U
    if (avg_raw < PEDAL_ADC_FAULT_LO || avg_raw > PEDAL_ADC_FAULT_HI) {
#else
    if (avg_raw > PEDAL_ADC_FAULT_HI) {
#endif
        st->plausible       = false;
        st->pct             = 0.0f;
        st->pct_prev        = 0.0f;
        st->ema             = 0.0f;
        st->ema_primed      = false;
        st->rate_fault      = false;
        st->recovery_count  = 0;
        return;
    }

    /* 5. Convert to percentage */
    st->pct_raw = Pedal_RawToPercent(st, avg_raw);

    /* 6. Rate-fault recovery state machine.
     *    Runs BEFORE the rate check so a downward transition during recovery
     *    does NOT re-fire the rapid-release path — recovery ends from 0 %.  */
    if (st->rate_fault) {
        if (st->pct_raw <= PEDAL_RELEASE_ZONE_PCT) {
            st->recovery_count++;
            if (st->recovery_count >= PEDAL_RECOVERY_CYCLES) {
                st->rate_fault      = false;
                st->recovery_count  = 0;
                st->plausible       = true;
                st->pct             = 0.0f;
                st->pct_prev        = 0.0f;
                st->ema             = 0.0f;
                st->ema_primed      = false;
                return; /* next cycle processes normally from 0 % */
            }
        } else {
            st->recovery_count = 0;  /* pedal above release zone — reset */
        }
        st->plausible = false;
        st->pct       = 0.0f;
        return;
    }

    /* 7. Rate-of-change check — direction aware. */
    float delta = st->pct_raw - st->pct_prev;

    if (delta < -(PEDAL_MAX_RATE_PCT)) {
        /* Rapid RELEASE — safe downward transition (torque reduction).
         *
         * §5 partial-release fix: only force the history to 0 % when the
         * pedal actually settles in the rest zone.  A fast release that ends
         * ABOVE the rest zone (100→60, 80→40, 60→20) is a legitimate demand
         * reduction; zeroing pct_prev made the next cycle compute e.g.
         * 60−0 = +60 % and fire a FALSE rapid-application rate fault.       */
        if (st->pct_raw <= PEDAL_RELEASE_ZONE_PCT) {
            /* A1. Settled in rest zone → clean full reset to 0. */
            st->pct             = 0.0f;
            st->pct_prev        = 0.0f;
            st->ema             = 0.0f;
            st->ema_primed      = false;
            st->plausible       = true;
            st->rate_fault      = false;
            st->recovery_count  = 0;
            return;
        }
        /* A2. Settled ABOVE the rest zone → legitimate demand reduction.
         *     Re-baseline history to the current raw reading (no false rise
         *     next cycle) and align the EMA to the current value so the
         *     output tracks the reduced physical pedal and never exceeds it. */
        st->ema             = st->pct_raw;
        st->ema_primed      = true;
        st->pct             = st->pct_raw;
        st->pct_prev        = st->pct_raw;
        st->plausible       = true;
        st->rate_fault      = false;
        st->recovery_count  = 0;
        return;
    }

    if (delta > PEDAL_MAX_RATE_PCT) {
        /* Rapid APPLICATION — fault.  Zero safe output and re-baseline
         * pct_prev to the current raw reading to break the permanent
         * lockout, then require PEDAL_RECOVERY_CYCLES released cycles.      */
        st->plausible       = false;
        st->rate_fault      = true;
        st->pct             = 0.0f;
        st->pct_prev        = st->pct_raw;
        st->ema             = 0.0f;
        st->ema_primed      = false;
        st->recovery_count  = 0;
        return;
    }

    /* 8. EMA filter — only updated when the rate check has passed. */
    if (!st->ema_primed) {
        st->ema        = st->pct_raw;
        st->ema_primed = true;
    } else {
        st->ema = PEDAL_EMA_ALPHA * st->pct_raw
                + (1.0f - PEDAL_EMA_ALPHA) * st->ema;
    }

    /* 9. All checks passed — publish output and update history */
    st->plausible = true;
    st->pct       = st->ema;
    st->pct_prev  = st->pct_raw;
}
