/**
 ****************************************************************************
 * @file    pedal_logic.h
 * @brief   Pure, host-testable accelerator-pedal plausibility pipeline.
 *
 *          Extracted from sensor_manager.c so that BOTH production firmware
 *          and host unit tests exercise the SAME code (§5 requirement: no
 *          hand-written re-implementation of Pedal_Update() in the test).
 *
 *          The module is deliberately free of any HAL/hardware dependency:
 *          the caller supplies the two raw ADC samples and this pipeline
 *          performs dual-sample consistency, rail-fault, calibration,
 *          direction-aware rate limiting, recovery state machine and EMA
 *          filtering, operating entirely on an explicit PedalState struct.
 ****************************************************************************
 */
#ifndef PEDAL_LOGIC_H
#define PEDAL_LOGIC_H

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * Pipeline constants — single source of truth (shared by firmware + tests).
 * ------------------------------------------------------------------------- */

/* Dual-sample consistency tolerance (ADC counts). */
#define PEDAL_SAMPLE_TOLERANCE   30U

/* Rail-fault band.  Rest ≈ 0 counts is valid (pedal wired directly to ADC),
 * so only a high-side rail short pinned at 4095 is flagged. */
#define PEDAL_ADC_FAULT_LO        0U
#define PEDAL_ADC_FAULT_HI     4094U

/* Rate-of-change limit: max % change per 50 ms update cycle. */
#define PEDAL_MAX_RATE_PCT     35.0f

/* EMA filter coefficient (0 < α ≤ 1). */
#define PEDAL_EMA_ALPHA         0.3f

/* Upward rate-fault recovery: released zone and required consecutive cycles. */
#define PEDAL_RELEASE_ZONE_PCT  3.0f
#define PEDAL_RECOVERY_CYCLES     3U

/* -------------------------------------------------------------------------
 * Explicit pipeline state (was a set of file-scope statics in sensor_manager).
 * ------------------------------------------------------------------------- */
typedef struct {
    /* Runtime calibration endpoints (raw ADC counts). */
    uint16_t adc_min;
    uint16_t adc_max;
    /* Last raw samples (diagnostics / accessors). */
    uint16_t raw_adc;
    uint16_t raw_adc2;
    /* Pipeline outputs / accumulated state. */
    float    pct;             /* published control output (EMA), 0–100 %      */
    float    pct_raw;         /* instantaneous %, pre-EMA                     */
    float    pct_prev;        /* previous cycle % (for rate check)            */
    float    ema;             /* EMA-filtered %                               */
    bool     plausible;       /* software plausibility result                 */
    bool     contradict;      /* dual samples disagree                        */
    bool     ema_primed;      /* EMA initialised after first read             */
    bool     rate_fault;      /* upward (application) rate fault latched       */
    uint8_t  recovery_count;  /* consecutive released cycles during recovery   */
} PedalState;

/**
 * @brief Initialise a PedalState to a clean released (0 %) baseline.
 */
void Pedal_StateInit(PedalState *st, uint16_t adc_min, uint16_t adc_max);

/**
 * @brief Run the plausibility pipeline for one cycle on two raw ADC samples.
 *
 *        Mirrors steps 2–9 of the original Pedal_Update():
 *          2. dual-sample consistency
 *          4. rail-fault range validation
 *          5. raw→%
 *          6. rate-fault recovery state machine
 *          7. direction-aware rate check
 *          8. EMA filter
 *          9. publish
 *
 *        On return st->pct / st->plausible / st->contradict reflect the
 *        result of this cycle.
 */
void Pedal_ProcessSamples(PedalState *st, uint16_t adc1, uint16_t adc2);

#endif /* PEDAL_LOGIC_H */
