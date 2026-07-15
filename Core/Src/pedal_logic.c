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
    st->adc_min          = adc_min;
    st->adc_max          = adc_max;
    st->raw_adc          = 0U;
    st->raw_adc2         = 0U;
    st->pct              = 0.0f;
    st->pct_raw          = 0.0f;
    st->pct_prev         = 0.0f;
    st->ema              = 0.0f;
    st->plausible        = true;
    st->contradict       = false;
    st->ema_primed       = false;
    st->rate_limited     = false;
    st->contradict_count = 0U;
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

    /* 1. Dual-sample consistency with persistence debounce.
     *
     *    A disagreement between the two consecutive ADC reads is only a REAL
     *    fault (pedal position genuinely indeterminate) when it PERSISTS.  A
     *    single/short-lived disagreement is transient electrical noise and
     *    must NOT immobilise the vehicle (§C): the last validated demand is
     *    held and plausibility is preserved so no global SENSOR_FAULT fires.
     *    Only after PEDAL_CONTRADICT_DEBOUNCE_CYCLES consecutive contradictory
     *    cycles do we latch the contradiction fault and force zero torque (§D). */
    uint16_t diff_raw = (adc1 >= adc2) ? (uint16_t)(adc1 - adc2)
                                       : (uint16_t)(adc2 - adc1);
    if (diff_raw > PEDAL_SAMPLE_TOLERANCE) {
        if (st->contradict_count < 0xFFU) {
            st->contradict_count++;
        }
        st->rate_limited = false;
        if (st->contradict_count >= PEDAL_CONTRADICT_DEBOUNCE_CYCLES) {
            /* Persistent contradiction — position indeterminate → no torque. */
            st->contradict = true;
            st->plausible  = false;
            st->pct        = 0.0f;
            return;
        }
        /* Transient noise — hold last validated demand, stay drivable.
         * st->pct / st->ema / st->pct_prev are intentionally left unchanged. */
        st->contradict = false;
        st->plausible  = true;
        return;
    }
    st->contradict       = false;
    st->contradict_count = 0U;

    /* 2. Use average of both samples for best accuracy. */
    uint16_t avg_raw = (uint16_t)(((uint32_t)adc1 + adc2) / 2U);

    /* 3. Range validation — detect stuck/impossible reading (rail short). This
     *    remains a REAL fault: torque must never be produced from a signal
     *    whose physical value cannot be known (§E). */
#if PEDAL_ADC_FAULT_LO > 0U
    if (avg_raw < PEDAL_ADC_FAULT_LO || avg_raw > PEDAL_ADC_FAULT_HI) {
#else
    if (avg_raw > PEDAL_ADC_FAULT_HI) {
#endif
        st->plausible    = false;
        st->pct          = 0.0f;
        st->pct_raw      = 0.0f;
        st->pct_prev     = 0.0f;
        st->ema          = 0.0f;
        st->ema_primed   = false;
        st->rate_limited = false;
        return;
    }

    /* 4. Convert to percentage. */
    st->pct_raw = Pedal_RawToPercent(st, avg_raw);

    /* 5. Asymmetric EMA: snap DOWN immediately on release (torque reduces
     *    without lag), smooth on application (noise filter / natural ramp). */
    if (!st->ema_primed) {
        st->ema        = st->pct_raw;
        st->ema_primed = true;
    } else if (st->pct_raw < st->ema) {
        st->ema = st->pct_raw;                       /* fast release */
    } else {
        st->ema = PEDAL_EMA_ALPHA * st->pct_raw
                + (1.0f - PEDAL_EMA_ALPHA) * st->ema; /* smooth application */
    }

    /* 6. Direction-aware output update.
     *    - A rise larger than PEDAL_MAX_RATE_PCT is clamped to that safe ramp
     *      (§A): a fast, electrically coherent stab is VALID intent — the
     *      demand keeps climbing over the next cycles, it is NOT a fault and
     *      never zeroes the output.  rate_limited is a diagnostic advisory.
     *    - A decrease (or small rise) passes through immediately (§B): fast
     *      release reduces torque at once and never trips a fault.            */
    float rise = st->ema - st->pct;
    if (rise > PEDAL_MAX_RATE_PCT) {
        st->pct          = st->pct + PEDAL_MAX_RATE_PCT;
        st->rate_limited = true;
    } else {
        st->pct          = st->ema;
        st->rate_limited = false;
    }

    /* 7. All checks passed — signal is plausible. */
    st->plausible = true;
    st->pct_prev  = st->pct_raw;
}
