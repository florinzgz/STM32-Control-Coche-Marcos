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

/* Safe demand ramp: maximum UPWARD change of the published output per 50 ms
 * cycle.  This is NOT a fault threshold — a fast but electrically coherent
 * stab of the pedal is valid driver intent.  When the raw signal rises faster
 * than this the demand is rate-limited (a diagnostic "rate_limited" advisory)
 * to protect the motors/transmission, but the vehicle keeps responding and
 * plausibility is preserved.  Downward changes (release) are NOT limited so
 * torque drops immediately. */
#define PEDAL_MAX_RATE_PCT     35.0f

/* EMA filter coefficient (0 < α ≤ 1).  Applied only while the demand rises
 * (noise smoothing / natural ramp); a release snaps the filter down so torque
 * is reduced without lag. */
#define PEDAL_EMA_ALPHA         0.3f

/* Released ("rest") zone used by callers to detect a fully lifted pedal. */
#define PEDAL_RELEASE_ZONE_PCT  3.0f

/* Dual-sample contradiction debounce: a disagreement between the two ADC reads
 * must PERSIST for this many consecutive cycles before it is treated as a real
 * (position-indeterminate) fault that zeroes torque.  A shorter/one-shot
 * disagreement is transient noise: the last validated demand is held and the
 * vehicle is NOT immobilised. */
#define PEDAL_CONTRADICT_DEBOUNCE_CYCLES  3U

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
    bool     contradict;      /* dual samples disagree PERSISTENTLY (fault)   */
    bool     ema_primed;      /* EMA initialised after first read             */
    bool     rate_limited;    /* output ramp active this cycle (advisory only) */
    uint8_t  contradict_count;/* consecutive contradictory cycles (debounce)   */
} PedalState;

/**
 * @brief Initialise a PedalState to a clean released (0 %) baseline.
 */
void Pedal_StateInit(PedalState *st, uint16_t adc_min, uint16_t adc_max);

/**
 * @brief Run the plausibility pipeline for one cycle on two raw ADC samples.
 *
 *        Pipeline:
 *          1. dual-sample consistency with persistence debounce
 *             (transient noise holds last demand; only a PERSISTENT
 *              disagreement latches a contradiction fault → zero torque)
 *          2. rail-fault range validation (stuck/impossible → zero torque)
 *          3. raw→%
 *          4. asymmetric EMA (snap down on release, smooth on application)
 *          5. direction-aware output: upward change limited to a safe ramp
 *             (fast coherent stab is VALID intent, never a fault); downward
 *             change passes through immediately
 *
 *        On return st->pct / st->plausible / st->contradict reflect the
 *        result of this cycle; st->rate_limited is an advisory diagnostic.
 */
void Pedal_ProcessSamples(PedalState *st, uint16_t adc1, uint16_t adc2);

#endif /* PEDAL_LOGIC_H */
