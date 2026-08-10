/**
  ****************************************************************************
  * @file    wheel_equality_test.c
  * @brief   Implementation of the wheel-equality / BTS7960 health self-test
  *          pure decision core (Hito 2, PR #445).
  *
  * Pure logic only — no HAL, no flash, no delays, no direct actuator access.
  * See wheel_equality_test.h for the full contract.
  ****************************************************************************
  */

#include "wheel_equality_test.h"

#include <math.h>
#include <string.h>

/* ---- Fase 1 step table: step_index (0..15) -> wheel/level/dir ----------
 * Per wheel, in order: 25%FWD, 25%REV, 50%FWD, 50%REV (spec order: "a 25 %
 * y a 50 %, en AMBOS sentidos").  Grouping by wheel first lets a technician
 * watch one wheel complete its full 4-sample sequence before the next. */
static void step_to_wheel_level_dir(uint8_t step_index, uint8_t *wheel,
                                     uint8_t *level, uint8_t *dir)
{
    uint8_t w   = step_index / WHEQ_PHASE1_STEPS_PER_WHEEL;
    uint8_t sub = step_index % WHEQ_PHASE1_STEPS_PER_WHEEL;
    *wheel = w;
    *level = (sub < WHEQ_NUM_DIRS) ? 0U : 1U;         /* 0=25%, 1=50% */
    *dir   = sub % WHEQ_NUM_DIRS;                      /* 0=fwd, 1=rev */
}

static uint8_t level_pwm_pct(uint8_t level)
{
    return (level == 0U) ? (uint8_t)WHEQ_PWM_LEVEL_25_PCT : (uint8_t)WHEQ_PWM_LEVEL_50_PCT;
}

/* ---- Median of 4 floats (standard even-count median: mean of the two
 * middle values after sorting).  No qsort dependency — a 4-element sorting
 * network is cheap and deterministic. ------------------------------------- */
static float median4(const float v[WHEQ_NUM_WHEELS])
{
    float a[WHEQ_NUM_WHEELS];
    memcpy(a, v, sizeof(a));
    for (uint8_t i = 0; i < WHEQ_NUM_WHEELS - 1U; i++) {
        for (uint8_t j = 0; j < WHEQ_NUM_WHEELS - 1U - i; j++) {
            if (a[j] > a[j + 1U]) {
                float tmp = a[j]; a[j] = a[j + 1U]; a[j + 1U] = tmp;
            }
        }
    }
    return 0.5f * (a[1] + a[2]);
}

/* Percent deviation of value from a (non-zero) reference; 0 if ref==0. */
static float pct_deviation(float value, float ref)
{
    if (fabsf(ref) < 1.0e-9f) return 0.0f;
    return fabsf(value - ref) / fabsf(ref) * 100.0f;
}

static void reset_accumulator(WheelEqTest *t)
{
    t->acc_pulses_ps = 0.0f;
    t->acc_current_a = 0.0f;
    t->acc_battery_v = 0.0f;
    t->acc_count      = 0U;
}

static void coast_and_idle_actuation(WheelEqTest *t)
{
    /* Nothing to clear on the struct itself — WheelEqTest_GetActuation()
     * derives a zero mask from state alone; kept as a named no-op for
     * call-site clarity at abort/finish points. */
    (void)t;
}

static void enter_aborted(WheelEqTest *t, WheelEqReason_t reason)
{
    t->state  = WHEQ_STATE_ABORTED;
    t->reason = reason;
    coast_and_idle_actuation(t);
}

/* ------------------------------------------------------------------ */
void WheelEqTest_Init(WheelEqTest *t)
{
    if (t == NULL) return;
    memset(t, 0, sizeof(*t));
    t->state  = WHEQ_STATE_IDLE;
    t->reason = WHEQ_REASON_NONE;
    for (uint8_t i = 0; i < WHEQ_NUM_WHEELS; i++) {
        t->result[i].wheel_verdict      = WHEQ_WHEEL_VERDICT_PENDING;
        t->result[i].halfbridge_verdict = WHEQ_HALFBRIDGE_PASS;
        t->result[i].driver_verdict     = WHEQ_DRIVER_PENDING;
    }
}

/* ------------------------------------------------------------------ */
bool WheelEqTest_Begin(WheelEqTest *t, const WheelEqConds *c)
{
    if (t == NULL || c == NULL) return false;

    if (WheelEqTest_Active(t)) {
        t->reason = WHEQ_REASON_ALREADY_ACTIVE;
        return false;
    }

    if (!c->svc_armed) {
        /* Hito 1 session must already be ARMED — every one of its entry
         * gates passed.  Not our own precondition to duplicate.          */
        t->state  = WHEQ_STATE_IDLE;
        t->reason = WHEQ_REASON_ENVELOPE_ABORT;
        return false;
    }

    /* Precondition 1: steering centred (spec: "CENTRAR VOLANTE" screen). */
    if (!c->steering_centered) {
        t->state = WHEQ_STATE_BLOCKED_DEADBAND;
        t->reason = WHEQ_REASON_NONE;
        t->steering_angle_at_block_deg = c->steering_angle_deg;
        return false;
    }

    /* Precondition 2: Ackermann differential must be exactly 1.000 on all
     * four wheels with the steering centred — any offset is a genuine
     * defect to report (FAIL_ACKERMANN_OFFSET), never silently ignored. */
    uint8_t offending = 0U;
    for (uint8_t i = 0; i < WHEQ_NUM_WHEELS; i++) {
        if (fabsf(c->ackermann_diff[i] - 1.0f) > TRACTION_OUTPUT_UNITY_EPSILON) {
            offending |= (uint8_t)(1U << i);
        }
    }
    if (offending != 0U) {
        t->state = WHEQ_STATE_BLOCKED_ACKERMANN;
        t->reason = WHEQ_REASON_NONE;
        t->ackermann_offending_mask = offending;
        for (uint8_t i = 0; i < WHEQ_NUM_WHEELS; i++) {
            t->result[i].wheel_verdict = (offending & (1U << i))
                ? WHEQ_WHEEL_VERDICT_FAIL_ACKERMANN_OFFSET
                : WHEQ_WHEEL_VERDICT_PENDING;
        }
        return false;
    }

    /* All clear — start Fase 1. */
    WheelEqState_t prev_state = t->state;
    WheelEqReason_t prev_reason = t->reason;
    (void)prev_state; (void)prev_reason;

    memset(t->pulses_ps,  0, sizeof(t->pulses_ps));
    memset(t->current_a,  0, sizeof(t->current_a));
    memset(t->battery_v,  0, sizeof(t->battery_v));
    t->phase2_ran = false;
    memset(t->phase2_pulses_ps, 0, sizeof(t->phase2_pulses_ps));
    memset(t->phase2_current_a, 0, sizeof(t->phase2_current_a));
    t->ackermann_offending_mask = 0U;

    for (uint8_t i = 0; i < WHEQ_NUM_WHEELS; i++) {
        t->temp_present[i]  = c->wheel_temp_present[i];
        t->temp_before_c[i] = c->wheel_temp_c[i];
        t->temp_after_c[i]  = c->wheel_temp_c[i];
        memset(&t->result[i], 0, sizeof(t->result[i]));
        t->result[i].wheel_verdict      = WHEQ_WHEEL_VERDICT_PENDING;
        t->result[i].halfbridge_verdict = WHEQ_HALFBRIDGE_PASS;
        t->result[i].driver_verdict     = WHEQ_DRIVER_PENDING;
    }

    t->state       = WHEQ_STATE_PHASE1_RUNNING;
    t->reason      = WHEQ_REASON_NONE;
    t->step_index  = 0U;
    t->step_start_ms = c->now_ms;
    t->retry_count = 0U;
    reset_accumulator(t);
    t->last_update_ms   = c->now_ms;
    t->have_last_update = true;
    return true;
}

/* ---- Verdict computation (called once, at Fase 1 completion) ----------- */
static void compute_verdicts(WheelEqTest *t)
{
    float normalized[WHEQ_NUM_WHEELS];
    float current50fwd[WHEQ_NUM_WHEELS];
    float slope[WHEQ_NUM_WHEELS];

    for (uint8_t w = 0; w < WHEQ_NUM_WHEELS; w++) {
        if (t->result[w].wheel_verdict == WHEQ_WHEEL_VERDICT_FAIL_ACKERMANN_OFFSET) {
            /* Should not happen (Begin() blocks before Fase 1 starts if any
             * wheel offends), kept defensive only.                        */
            normalized[w] = 0.0f;
            current50fwd[w] = 0.0f;
            slope[w] = 0.0f;
            continue;
        }

        float p25f = t->pulses_ps[w][0][0], p25r = t->pulses_ps[w][0][1];
        float p50f = t->pulses_ps[w][1][0], p50r = t->pulses_ps[w][1][1];
        float v25f = t->battery_v[w][0][0], v25r = t->battery_v[w][0][1];
        float v50f = t->battery_v[w][1][0], v50r = t->battery_v[w][1][1];

        float ns25f = (v25f > 0.0f) ? (p25f / (WHEQ_PWM_LEVEL_25_PCT * v25f)) : 0.0f;
        float ns25r = (v25r > 0.0f) ? (p25r / (WHEQ_PWM_LEVEL_25_PCT * v25r)) : 0.0f;
        float ns50f = (v50f > 0.0f) ? (p50f / (WHEQ_PWM_LEVEL_50_PCT * v50f)) : 0.0f;
        float ns50r = (v50r > 0.0f) ? (p50r / (WHEQ_PWM_LEVEL_50_PCT * v50r)) : 0.0f;

        normalized[w]   = 0.25f * (ns25f + ns25r + ns50f + ns50r);
        current50fwd[w] = t->current_a[w][1][0];
        slope[w] = (t->current_a[w][1][0] - t->current_a[w][0][0]) /
                   (float)(WHEQ_PWM_LEVEL_50_PCT - WHEQ_PWM_LEVEL_25_PCT);

        t->result[w].pulses_ps_25     = p25f;
        t->result[w].pulses_ps_50     = p50f;
        t->result[w].normalized_speed = normalized[w];
        t->result[w].current_a        = current50fwd[w];
        t->result[w].slope_a_per_pct  = slope[w];

        float speed_fwd = ns50f, speed_rev = ns50r;
        float speed_avg = 0.5f * (speed_fwd + speed_rev);
        float speed_asym = (fabsf(speed_avg) > 1.0e-9f)
            ? fabsf(speed_fwd - speed_rev) / fabsf(speed_avg) * 100.0f : 0.0f;
        float cur_avg = 0.5f * (t->current_a[w][1][0] + t->current_a[w][1][1]);
        float cur_asym = (fabsf(cur_avg) > 1.0e-9f)
            ? fabsf(t->current_a[w][1][0] - t->current_a[w][1][1]) / fabsf(cur_avg) * 100.0f : 0.0f;
        float asym = (speed_asym > cur_asym) ? speed_asym : cur_asym;
        t->result[w].halfbridge_asym_pct = asym;

        if (asym > WHEQ_HALFBRIDGE_FAIL_PCT) {
            t->result[w].halfbridge_verdict = WHEQ_HALFBRIDGE_FAIL;
        } else if (asym > WHEQ_HALFBRIDGE_WARN_PCT) {
            t->result[w].halfbridge_verdict = WHEQ_HALFBRIDGE_WARN;
        } else {
            t->result[w].halfbridge_verdict = WHEQ_HALFBRIDGE_PASS;
        }

        t->result[w].delta_temp_c = t->temp_present[w]
            ? (t->temp_after_c[w] - t->temp_before_c[w]) : 0.0f;
        t->result[w].temp_present = t->temp_present[w];
    }

    float median_speed   = median4(normalized);
    float median_current = median4(current50fwd);
    float median_slope    = median4(slope);

    float delta_temp[WHEQ_NUM_WHEELS];
    for (uint8_t w = 0; w < WHEQ_NUM_WHEELS; w++) {
        delta_temp[w] = t->result[w].delta_temp_c;
    }
    float median_delta_temp = median4(delta_temp);

    for (uint8_t w = 0; w < WHEQ_NUM_WHEELS; w++) {
        if (t->result[w].wheel_verdict == WHEQ_WHEEL_VERDICT_FAIL_ACKERMANN_OFFSET) {
            continue;  /* precondition failure already recorded */
        }

        float dev_pct = pct_deviation(normalized[w], median_speed);
        t->result[w].deviation_pct = dev_pct;

        WheelEqWheelVerdict_t verdict;
        if (dev_pct > WHEQ_EQUALITY_FAIL_PCT) {
            verdict = WHEQ_WHEEL_VERDICT_FAIL;
        } else if (dev_pct > WHEQ_EQUALITY_WARN_PCT) {
            verdict = WHEQ_WHEEL_VERDICT_WARN;
        } else {
            verdict = WHEQ_WHEEL_VERDICT_PASS;
        }
        t->result[w].wheel_verdict = verdict;

        /* Probable cause — spec's diagnostic cross-reference table. */
        WheelEqCause_t cause = WHEQ_CAUSE_NONE;
        if (verdict != WHEQ_WHEEL_VERDICT_PASS) {
            bool faster_than_median = normalized[w] > median_speed;
            if (faster_than_median) {
                cause = WHEQ_CAUSE_OTHERS_BRAKED;
            } else {
                float cur_dev_pct = pct_deviation(current50fwd[w], median_current);
                if (cur_dev_pct <= WHEQ_EQUALITY_WARN_PCT) {
                    cause = WHEQ_CAUSE_SENSOR;
                } else if (current50fwd[w] > median_current) {
                    cause = WHEQ_CAUSE_MECHANICAL;
                } else {
                    cause = WHEQ_CAUSE_ELECTRICAL;
                }
            }
        }
        t->result[w].cause = cause;

        /* Driver (BTS7960) verdict — combine half-bridge asymmetry, I/PWM
         * slope anomaly, electrical-cause wheel result and thermal drift,
         * each contributing a NAMED bit so the "concrete criterion" that
         * motivated the verdict is always traceable.                     */
        uint8_t reason_mask = 0U;
        bool degraded = false, suspicious = false;

        if (t->result[w].halfbridge_verdict == WHEQ_HALFBRIDGE_FAIL) {
            degraded = true;
            reason_mask |= WHEQ_DRIVER_REASON_HALFBRIDGE;
        } else if (t->result[w].halfbridge_verdict == WHEQ_HALFBRIDGE_WARN) {
            suspicious = true;
            reason_mask |= WHEQ_DRIVER_REASON_HALFBRIDGE;
        }

        float slope_dev_pct = pct_deviation(slope[w], median_slope);
        if (slope_dev_pct > WHEQ_SLOPE_FAIL_PCT) {
            degraded = true;
            reason_mask |= WHEQ_DRIVER_REASON_SLOPE;
        } else if (slope_dev_pct > WHEQ_SLOPE_WARN_PCT) {
            suspicious = true;
            reason_mask |= WHEQ_DRIVER_REASON_SLOPE;
        }

        if (cause == WHEQ_CAUSE_ELECTRICAL) {
            suspicious = true;
            reason_mask |= WHEQ_DRIVER_REASON_ELECTRICAL;
        }

        if (t->temp_present[w]) {
            float dt_dev = t->result[w].delta_temp_c - median_delta_temp;
            if (dt_dev > WHEQ_THERMAL_DRIFT_WARN_DELTA_C) {
                suspicious = true;
                reason_mask |= WHEQ_DRIVER_REASON_THERMAL;
            }
        }

        t->result[w].driver_reason_mask = reason_mask;
        t->result[w].driver_verdict = degraded ? WHEQ_DRIVER_DEGRADADO
                                    : (suspicious ? WHEQ_DRIVER_SOSPECHOSO
                                                  : WHEQ_DRIVER_SANO);
    }
}

/* ---- Fase 1 running: discard startup, accumulate, retry on interference */
static void step_phase1_running(WheelEqTest *t, const WheelEqConds *c)
{
    uint8_t wheel, level, dir;
    step_to_wheel_level_dir(t->step_index, &wheel, &level, &dir);

    uint32_t elapsed = c->now_ms - t->step_start_ms;

    if (elapsed < WHEQ_STARTUP_DISCARD_MS) {
        return;  /* still discarding the spin-up transient */
    }

    if (c->abs_or_tcs_active) {
        /* ABS/TCS intervening during the measurement window invalidates
         * whatever has been accumulated so far — restart this same step
         * from scratch, bounded by WHEQ_STEP_MAX_RETRIES.                 */
        t->retry_count++;
        if (t->retry_count > WHEQ_STEP_MAX_RETRIES) {
            enter_aborted(t, WHEQ_REASON_TCS_ABS_INTERFERENCE);
            return;
        }
        t->step_start_ms = c->now_ms;
        reset_accumulator(t);
        return;
    }

    t->acc_pulses_ps += c->wheel_pulses_ps[wheel];
    t->acc_current_a += c->wheel_current_a[wheel];
    t->acc_battery_v += c->battery_v;
    t->acc_count++;

    if (elapsed >= WHEQ_STEP_MEASURE_TOTAL_MS) {
        float n = (t->acc_count > 0U) ? (float)t->acc_count : 1.0f;
        t->pulses_ps[wheel][level][dir] = t->acc_pulses_ps / n;
        t->current_a[wheel][level][dir] = t->acc_current_a / n;
        t->battery_v[wheel][level][dir] = t->acc_battery_v / n;

        t->step_index++;
        t->retry_count = 0U;
        reset_accumulator(t);

        if (t->step_index >= WHEQ_PHASE1_TOTAL_STEPS) {
            for (uint8_t i = 0; i < WHEQ_NUM_WHEELS; i++) {
                t->temp_after_c[i] = c->wheel_temp_c[i];
            }
            compute_verdicts(t);
            t->state = WHEQ_STATE_PHASE1_DONE;
        } else {
            t->state = WHEQ_STATE_PHASE1_DEADTIME;
            t->deadtime_start_ms = c->now_ms;
        }
    }
}

static void step_phase1_deadtime(WheelEqTest *t, const WheelEqConds *c)
{
    if ((c->now_ms - t->deadtime_start_ms) >= WHEQ_STEP_DEADTIME_MS) {
        t->state = WHEQ_STATE_PHASE1_RUNNING;
        t->step_start_ms = c->now_ms;
        reset_accumulator(t);
    }
}

static void step_phase2_running(WheelEqTest *t, const WheelEqConds *c)
{
    uint32_t elapsed = c->now_ms - t->step_start_ms;

    if (elapsed < WHEQ_STARTUP_DISCARD_MS) {
        return;
    }

    if (c->abs_or_tcs_active) {
        t->retry_count++;
        if (t->retry_count > WHEQ_STEP_MAX_RETRIES) {
            enter_aborted(t, WHEQ_REASON_TCS_ABS_INTERFERENCE);
            return;
        }
        t->step_start_ms = c->now_ms;
        reset_accumulator(t);
        return;
    }

    /* Fase 2 drives all four wheels at once — accumulate a per-wheel sum
     * using the shared counter (all four sampled every tick together). */
    for (uint8_t w = 0; w < WHEQ_NUM_WHEELS; w++) {
        t->phase2_pulses_ps[w] += c->wheel_pulses_ps[w];
        t->phase2_current_a[w] += c->wheel_current_a[w];
    }
    t->acc_count++;

    if (elapsed >= WHEQ_PHASE2_PLATEAU_MS) {
        float n = (t->acc_count > 0U) ? (float)t->acc_count : 1.0f;
        for (uint8_t w = 0; w < WHEQ_NUM_WHEELS; w++) {
            t->phase2_pulses_ps[w] /= n;
            t->phase2_current_a[w] /= n;
        }
        t->phase2_ran = true;
        t->state = WHEQ_STATE_PHASE2_DONE;
    }
}

/* ------------------------------------------------------------------ */
WheelEqState_t WheelEqTest_Update(WheelEqTest *t, const WheelEqConds *c)
{
    if (t == NULL) return WHEQ_STATE_IDLE;
    if (c == NULL) return t->state;

    if (!WheelEqTest_Active(t)) {
        return t->state;  /* nothing to do until Begin()/BeginPhase2() */
    }

    /* Watchdog — a stale gap since the last Update() forces an abort
     * instead of silently resuming a possibly-unsafe stale command.    */
    if (t->have_last_update) {
        uint32_t gap = c->now_ms - t->last_update_ms;
        if (gap > WHEQ_WATCHDOG_MAX_GAP_MS) {
            enter_aborted(t, WHEQ_REASON_WATCHDOG);
            t->last_update_ms = c->now_ms;
            return t->state;
        }
    }
    t->last_update_ms   = c->now_ms;
    t->have_last_update = true;

    /* Always-on: the underlying Hito 1 session must remain active — the
     * instant ANY of its own abort conditions fire, we abort too.        */
    if (!c->svc_active) {
        enter_aborted(t, WHEQ_REASON_ENVELOPE_ABORT);
        return t->state;
    }

    switch (t->state) {
    case WHEQ_STATE_PHASE1_RUNNING:  step_phase1_running(t, c);  break;
    case WHEQ_STATE_PHASE1_DEADTIME: step_phase1_deadtime(t, c); break;
    case WHEQ_STATE_PHASE2_RUNNING:  step_phase2_running(t, c);  break;
    default: break;  /* PHASE1_DONE / PHASE2_DONE: awaiting operator */
    }
    return t->state;
}

/* ------------------------------------------------------------------ */
bool WheelEqTest_Phase1AllPass(const WheelEqTest *t)
{
    if (t == NULL) return false;
    if (t->state != WHEQ_STATE_PHASE1_DONE && t->state != WHEQ_STATE_PHASE2_DONE) {
        return false;
    }
    for (uint8_t i = 0; i < WHEQ_NUM_WHEELS; i++) {
        WheelEqWheelVerdict_t v = t->result[i].wheel_verdict;
        if (v == WHEQ_WHEEL_VERDICT_FAIL || v == WHEQ_WHEEL_VERDICT_FAIL_ACKERMANN_OFFSET) {
            return false;
        }
        if (t->result[i].halfbridge_verdict == WHEQ_HALFBRIDGE_FAIL) {
            return false;
        }
    }
    return true;
}

bool WheelEqTest_BeginPhase2(WheelEqTest *t, const WheelEqConds *c)
{
    if (t == NULL || c == NULL) return false;

    if (t->state != WHEQ_STATE_PHASE1_DONE) {
        t->reason = WHEQ_REASON_ALREADY_ACTIVE;
        return false;
    }
    if (!c->svc_active) {
        t->reason = WHEQ_REASON_ENVELOPE_ABORT;
        return false;
    }
    if (!WheelEqTest_Phase1AllPass(t)) {
        t->reason = WHEQ_REASON_PHASE1_NOT_CLEAN;
        return false;
    }

    t->state = WHEQ_STATE_PHASE2_RUNNING;
    t->reason = WHEQ_REASON_NONE;
    t->step_start_ms = c->now_ms;
    t->retry_count = 0U;
    reset_accumulator(t);
    memset(t->phase2_pulses_ps, 0, sizeof(t->phase2_pulses_ps));
    memset(t->phase2_current_a, 0, sizeof(t->phase2_current_a));
    t->last_update_ms = c->now_ms;
    t->have_last_update = true;
    return true;
}

void WheelEqTest_Abort(WheelEqTest *t, WheelEqReason_t reason)
{
    if (t == NULL) return;
    if (t->state == WHEQ_STATE_IDLE) return;  /* idempotent, no-op */
    enter_aborted(t, reason);
}

/* ------------------------------------------------------------------ */
WheelEqActuation_t WheelEqTest_GetActuation(const WheelEqTest *t)
{
    WheelEqActuation_t a;
    a.wheel_mask = 0U;
    a.pwm_pct    = 0U;
    a.direction  = WHEQ_DIR_FORWARD;

    if (t == NULL) return a;

    if (t->state == WHEQ_STATE_PHASE1_RUNNING) {
        uint8_t wheel, level, dir;
        step_to_wheel_level_dir(t->step_index, &wheel, &level, &dir);
        /* Do not command a step until its warm-up discard AND validity
         * gates have started accumulating — always safe to drive from the
         * moment we enter the state (dead-time already coasted before).  */
        a.wheel_mask = (uint8_t)(1U << wheel);
        a.pwm_pct    = level_pwm_pct(level);
        a.direction  = (WheelEqDirection_t)dir;
    } else if (t->state == WHEQ_STATE_PHASE2_RUNNING) {
        a.wheel_mask = 0x0FU;  /* all four wheels simultaneously */
        a.pwm_pct    = (uint8_t)WHEQ_PHASE2_PWM_CEILING_PCT;
        a.direction  = WHEQ_DIR_FORWARD;
    }
    return a;
}

/* ------------------------------------------------------------------ */
const char *WheelEqTest_StateText(WheelEqState_t st)
{
    switch (st) {
    case WHEQ_STATE_IDLE:               return "IDLE";
    case WHEQ_STATE_BLOCKED_DEADBAND:   return "CENTRAR VOLANTE";
    case WHEQ_STATE_BLOCKED_ACKERMANN:  return "FALLO ACKERMANN";
    case WHEQ_STATE_PHASE1_RUNNING:     return "FASE 1: MIDIENDO";
    case WHEQ_STATE_PHASE1_DEADTIME:    return "FASE 1: PAUSA";
    case WHEQ_STATE_PHASE1_DONE:        return "FASE 1: COMPLETA";
    case WHEQ_STATE_PHASE2_RUNNING:     return "FASE 2: MIDIENDO";
    case WHEQ_STATE_PHASE2_DONE:        return "FASE 2: COMPLETA";
    case WHEQ_STATE_ABORTED:            return "ABORTADO";
    default:                            return "?";
    }
}

const char *WheelEqTest_ReasonText(WheelEqReason_t r)
{
    switch (r) {
    case WHEQ_REASON_NONE:                  return "-";
    case WHEQ_REASON_OPERATOR:              return "ABORTADO POR OPERADOR";
    case WHEQ_REASON_ALREADY_ACTIVE:        return "TEST YA ACTIVO";
    case WHEQ_REASON_ENVELOPE_ABORT:        return "SESION HITO1 INACTIVA";
    case WHEQ_REASON_TCS_ABS_INTERFERENCE:  return "INTERFERENCIA TCS/ABS";
    case WHEQ_REASON_WATCHDOG:              return "WATCHDOG";
    case WHEQ_REASON_PHASE1_NOT_CLEAN:      return "FASE 1 CON FALLOS";
    default:                                return "?";
    }
}

const char *WheelEqTest_WheelVerdictText(WheelEqWheelVerdict_t v)
{
    switch (v) {
    case WHEQ_WHEEL_VERDICT_PENDING:               return "PENDIENTE";
    case WHEQ_WHEEL_VERDICT_PASS:                   return "PASS";
    case WHEQ_WHEEL_VERDICT_WARN:                   return "WARN";
    case WHEQ_WHEEL_VERDICT_FAIL:                   return "FAIL";
    case WHEQ_WHEEL_VERDICT_FAIL_ACKERMANN_OFFSET:  return "FAIL_ACKERMANN_OFFSET";
    default:                                        return "?";
    }
}

const char *WheelEqTest_CauseText(WheelEqCause_t c)
{
    switch (c) {
    case WHEQ_CAUSE_NONE:          return "-";
    case WHEQ_CAUSE_MECHANICAL:    return "RESISTENCIA MECANICA (rodamiento/freno/reductora/rozamiento)";
    case WHEQ_CAUSE_ELECTRICAL:    return "FALTA DE PAR ELECTRICO (escobillas/conexion/driver/caida tension)";
    case WHEQ_CAUSE_SENSOR:        return "SENSOR (no la rueda)";
    case WHEQ_CAUSE_OTHERS_BRAKED: return "REVISAR SI LAS OTRAS TRES ESTAN FRENADAS";
    default:                       return "?";
    }
}

const char *WheelEqTest_HalfBridgeVerdictText(WheelEqHalfBridgeVerdict_t v)
{
    switch (v) {
    case WHEQ_HALFBRIDGE_PASS: return "PASS";
    case WHEQ_HALFBRIDGE_WARN: return "WARN_HALFBRIDGE";
    case WHEQ_HALFBRIDGE_FAIL: return "FAIL_HALFBRIDGE";
    default:                   return "?";
    }
}

const char *WheelEqTest_DriverVerdictText(WheelEqDriverVerdict_t v)
{
    switch (v) {
    case WHEQ_DRIVER_PENDING:     return "PENDIENTE";
    case WHEQ_DRIVER_SANO:        return "SANO";
    case WHEQ_DRIVER_SOSPECHOSO:  return "SOSPECHOSO";
    case WHEQ_DRIVER_DEGRADADO:   return "DEGRADADO";
    default:                      return "?";
    }
}
