/**
  ****************************************************************************
  * @file    pedal_cal_session.c
  * @brief   Implementation of the explicit pedal-calibration session FSM.
  *
  * Pure logic only — no HAL, no flash, no delays.  See pedal_cal_session.h
  * for the full contract.  Persistence / apply / readback are injected via
  * PedalCalSessionHooks so the whole module is host-testable.
  ****************************************************************************
  */

#include "pedal_cal_session.h"

#include <string.h>

/* ------------------------------------------------------------------ */
PedalCalSessionCfg PedalCalSession_DefaultCfg(void)
{
    PedalCalSessionCfg cfg;
    cfg.stable_samples     = 8U;    /* audit §5: 8 stable samples          */
    cfg.stable_tol         = 8U;    /* same spread tolerance as before     */
    cfg.range_min          = 800U;  /* audit §5: range >= 800              */
    cfg.phase_timeout_ms   = 30000U;/* 30 s of operator inactivity aborts  */
    cfg.capture_timeout_ms = 5000U; /* a single 8-sample lock must be quick*/
    return cfg;
}

void PedalCalSession_Init(PedalCalSession *s,
                          const PedalCalSessionCfg *cfg,
                          const PedalCalSessionHooks *hooks)
{
    if (s == NULL) return;
    memset(s, 0, sizeof(*s));
    s->cfg   = (cfg != NULL)   ? *cfg   : PedalCalSession_DefaultCfg();
    if (hooks != NULL) {
        s->hooks = *hooks;
    }
    s->state  = PEDAL_CAL_IDLE;
    s->reason = PEDAL_CAL_SESS_OK;
    if (s->cfg.stable_samples == 0U) {
        s->cfg.stable_samples = 1U;
    }
    if (s->cfg.stable_samples > (uint8_t)(sizeof(s->samples) / sizeof(s->samples[0]))) {
        s->cfg.stable_samples = (uint8_t)(sizeof(s->samples) / sizeof(s->samples[0]));
    }
}

/* ---- Entry guards (audit §5) ------------------------------------- */
static uint32_t entry_block_mask(const PedalCalConds *c)
{
    uint32_t bits = PEDAL_CAL_SESS_OK;
    if (!c->in_standby)           bits |= PEDAL_CAL_BLOCK_NOT_STANDBY;
    if (!c->gear_park_or_neutral) bits |= PEDAL_CAL_BLOCK_GEAR;
    if (c->wheels_moving)         bits |= PEDAL_CAL_BLOCK_WHEELS_MOVING;
    if (!c->pedal_plausible)      bits |= PEDAL_CAL_BLOCK_PEDAL_IMPLAUSIBLE;
    if (c->critical_error)        bits |= PEDAL_CAL_BLOCK_CRITICAL_ERROR;
    if (!c->traction_inhibited)   bits |= PEDAL_CAL_BLOCK_TRACTION_LIVE;
    /* audit P5 final — require the real movement lock (demand 0 / PWM 0 / EN
     * LOW / relay OFF) to be CONFIRMED before the session may start.  The
     * caller enforces Traction_CalibrationLock() and checks the relay OFF, then
     * reports the combined result via traction_locked. */
    if (!c->traction_locked)      bits |= PEDAL_CAL_BLOCK_LOCK_NOT_CONFIRMED;
    return bits;
}

/* ---- Always-on abort conditions (audit §5) ----------------------- */
static uint32_t abort_mask(const PedalCalConds *c)
{
    uint32_t bits = PEDAL_CAL_SESS_OK;
    if (c->safe_state)        bits |= PEDAL_CAL_ABORT_SAFE;
    if (c->critical_error)    bits |= PEDAL_CAL_ABORT_ERROR;
    if (c->emergency)         bits |= PEDAL_CAL_ABORT_EMERGENCY;
    if (c->wheels_moving)     bits |= PEDAL_CAL_ABORT_MOVEMENT;
    if (c->can_loss)          bits |= PEDAL_CAL_ABORT_CAN_LOSS;
    /* audit P5: the real movement lock (demand 0 / PWM 0 / EN LOW / relay OFF)
     * is verified by the caller every tick and reported via traction_locked.
     * If it is ever lost while a session runs, abort immediately. */
    if (!c->traction_locked)  bits |= PEDAL_CAL_ABORT_LOCK_LOST;
    return bits;
}

static void enter_aborted(PedalCalSession *s, uint32_t reason_bit)
{
    s->state        = PEDAL_CAL_ABORTED;
    s->reason       = reason_bit;
    s->sample_count = 0U;
    s->max_armed    = false;
}

static void reset_capture(PedalCalSession *s, uint32_t now_ms)
{
    s->sample_count   = 0U;
    s->phase_start_ms = now_ms;
}

/* Feed one raw sample into the rolling window; returns true and writes the
 * mean when `stable_samples` consecutive in-tolerance samples are locked. */
static bool capture_feed(PedalCalSession *s, uint16_t raw, uint16_t *out_mean)
{
    const uint8_t n = s->cfg.stable_samples;
    if (s->sample_count < n) {
        s->samples[s->sample_count++] = raw;
    } else {
        /* Slide the window forward by one. */
        for (uint8_t i = 1U; i < n; i++) {
            s->samples[i - 1U] = s->samples[i];
        }
        s->samples[n - 1U] = raw;
    }
    if (s->sample_count < n) {
        return false;
    }
    uint16_t mn = s->samples[0];
    uint16_t mx = s->samples[0];
    uint32_t sum = s->samples[0];
    for (uint8_t i = 1U; i < n; i++) {
        uint16_t v = s->samples[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
    }
    if ((uint32_t)(mx - mn) > s->cfg.stable_tol) {
        return false;  /* not stable yet — keep sliding                     */
    }
    *out_mean = (uint16_t)((sum + (n / 2U)) / n);
    return true;
}

/* ------------------------------------------------------------------ */
bool PedalCalSession_Begin(PedalCalSession *s, const PedalCalConds *c)
{
    if (s == NULL || c == NULL) return false;
    if (s->state != PEDAL_CAL_IDLE &&
        s->state != PEDAL_CAL_COMPLETED &&
        s->state != PEDAL_CAL_ABORTED) {
        return false;  /* already running                                    */
    }
    uint32_t block = entry_block_mask(c);
    if (block != PEDAL_CAL_SESS_OK) {
        s->state  = PEDAL_CAL_IDLE;
        s->reason = block;
        return false;
    }
    s->state            = PEDAL_CAL_ENTERING;
    s->reason           = PEDAL_CAL_SESS_OK;
    s->have_min         = false;
    s->have_max         = false;
    s->max_armed        = false;
    s->adc_min          = 0U;
    s->adc_max          = 0U;
    s->sample_count     = 0U;
    s->session_start_ms = c->now_ms;
    s->phase_start_ms   = c->now_ms;
    return true;
}

void PedalCalSession_ArmCaptureMax(PedalCalSession *s)
{
    if (s == NULL) return;
    /* Only arming from WAIT_FULL_PRESS is meaningful; ignore otherwise so a
     * stray button press cannot skip a phase or re-trigger a locked capture. */
    if (s->state == PEDAL_CAL_WAIT_FULL_PRESS) {
        s->max_armed = true;
    }
}

void PedalCalSession_Abort(PedalCalSession *s, uint32_t reason_bit)
{
    if (s == NULL) return;
    if (s->state == PEDAL_CAL_IDLE || s->state == PEDAL_CAL_COMPLETED) {
        return;
    }
    enter_aborted(s, reason_bit == 0U ? PEDAL_CAL_ABORT_OPERATOR : reason_bit);
}

PedalCalState PedalCalSession_Update(PedalCalSession *s, const PedalCalConds *c)
{
    if (s == NULL || c == NULL) return PEDAL_CAL_IDLE;

    /* Nothing to do in terminal / idle states. */
    if (!PedalCalSession_Active(s)) {
        return s->state;
    }

    /* 1. Always-on aborts take priority (SAFE/ERROR/emergency/movement/CAN). */
    uint32_t ab = abort_mask(c);
    if (ab != PEDAL_CAL_SESS_OK) {
        enter_aborted(s, ab);
        return s->state;
    }

    /* 2. Session-level timeout guard. */
    if ((uint32_t)(c->now_ms - s->phase_start_ms) > s->cfg.phase_timeout_ms) {
        enter_aborted(s, PEDAL_CAL_ABORT_TIMEOUT);
        return s->state;
    }

    switch (s->state) {
    case PEDAL_CAL_ENTERING:
        /* Safe outputs are enforced by the caller; move to the release wait. */
        s->state          = PEDAL_CAL_WAIT_RELEASED;
        s->phase_start_ms = c->now_ms;
        reset_capture(s, c->now_ms);
        break;

    case PEDAL_CAL_WAIT_RELEASED:
        /* Auto-start MIN capture once the pedal is released and plausible. */
        if (c->pedal_released && c->pedal_plausible) {
            s->state = PEDAL_CAL_CAPTURING_MIN;
            reset_capture(s, c->now_ms);
        }
        break;

    case PEDAL_CAL_CAPTURING_MIN: {
        /* MIN requires the pedal to stay released for the whole window. */
        if (!c->pedal_released) {
            s->state = PEDAL_CAL_WAIT_RELEASED;  /* operator lifted off — retry */
            reset_capture(s, c->now_ms);
            break;
        }
        if ((uint32_t)(c->now_ms - s->phase_start_ms) > s->cfg.capture_timeout_ms) {
            enter_aborted(s, PEDAL_CAL_FAIL_UNSTABLE);
            break;
        }
        uint16_t mean = 0U;
        if (capture_feed(s, c->pedal_raw, &mean)) {
            s->adc_min        = mean;
            s->have_min       = true;
            s->state          = PEDAL_CAL_WAIT_FULL_PRESS;
            s->phase_start_ms = c->now_ms;
            reset_capture(s, c->now_ms);
        }
        break;
    }

    case PEDAL_CAL_WAIT_FULL_PRESS:
        /* Wait for the operator to ARM the capture with the CAPTURE MAX button
         * (audit P5).  The transition no longer depends on a percent threshold
         * (Pedal_GetPercent >= 80 %) derived from a possibly-wrong old
         * calibration; the pressed pedal is proven from the RAW ADC below. */
        if (s->max_armed) {
            s->state          = PEDAL_CAL_CAPTURING_MAX;
            s->phase_start_ms = c->now_ms;
            reset_capture(s, c->now_ms);
        }
        break;

    case PEDAL_CAL_CAPTURING_MAX: {
        /* MAX is captured purely from the RAW ADC (audit P5): 8 stable samples
         * whose mean is above MIN by at least the required span.  We do NOT
         * gate on pedal_pressed_full, so a wrong old calibration (whose 100 %
         * the real pedal can never reach) cannot block a fresh calibration.
         * The operator keeps pressing until the raw mean clears the threshold;
         * a genuinely stuck / insufficient reading aborts on the phase timeout. */
        uint16_t mean = 0U;
        if (capture_feed(s, c->pedal_raw, &mean)) {
            if (mean > s->adc_min &&
                (uint32_t)(mean - s->adc_min) >= s->cfg.range_min) {
                s->adc_max        = mean;
                s->have_max       = true;
                s->max_armed      = false;
                s->state          = PEDAL_CAL_WAIT_RELEASE_FOR_SAVE;
                s->phase_start_ms = c->now_ms;
                reset_capture(s, c->now_ms);
            }
            /* else: stable but not pressed far enough yet — keep the rolling
             * window sliding so a harder press re-locks at a higher value. */
        }
        break;
    }

    case PEDAL_CAL_WAIT_RELEASE_FOR_SAVE:
        /* SAVE demands the pedal be released again before persisting. */
        if (c->pedal_released && c->pedal_plausible) {
            /* Validate the captured pair before offering SAVE. */
            if (!s->have_min || !s->have_max || s->adc_max <= s->adc_min) {
                enter_aborted(s, PEDAL_CAL_FAIL_MIN_GE_MAX);
                break;
            }
            if ((uint32_t)(s->adc_max - s->adc_min) < s->cfg.range_min) {
                enter_aborted(s, PEDAL_CAL_FAIL_RANGE_SMALL);
                break;
            }
            s->state          = PEDAL_CAL_READY_TO_SAVE;
            s->phase_start_ms = c->now_ms;
        }
        break;

    case PEDAL_CAL_READY_TO_SAVE:
        /* Wait here for the explicit SAVE request; re-guard pedal release. */
        if (!c->pedal_released) {
            s->state          = PEDAL_CAL_WAIT_RELEASE_FOR_SAVE;
            s->phase_start_ms = c->now_ms;
        }
        break;

    case PEDAL_CAL_SAVING:
        /* SAVING is driven synchronously by PedalCalSession_RequestSave();
         * a lingering SAVING here means the caller never finished it. */
        break;

    default:
        break;
    }

    return s->state;
}

void PedalCalSession_RequestSave(PedalCalSession *s, const PedalCalConds *c)
{
    if (s == NULL || c == NULL) return;
    if (s->state != PEDAL_CAL_READY_TO_SAVE) {
        return;  /* SAVE only valid once released and the pair is validated */
    }
    /* Re-guard: pedal released, no abort condition, pair still valid. */
    uint32_t ab = abort_mask(c);
    if (ab != PEDAL_CAL_SESS_OK) { enter_aborted(s, ab); return; }
    if (!c->pedal_released || !c->pedal_plausible) {
        s->state          = PEDAL_CAL_WAIT_RELEASE_FOR_SAVE;
        s->phase_start_ms = c->now_ms;
        return;
    }
    if (!s->have_min || !s->have_max || s->adc_max <= s->adc_min) {
        enter_aborted(s, PEDAL_CAL_FAIL_MIN_GE_MAX);
        return;
    }
    if ((uint32_t)(s->adc_max - s->adc_min) < s->cfg.range_min) {
        enter_aborted(s, PEDAL_CAL_FAIL_RANGE_SMALL);
        return;
    }
    if (s->hooks.validate != NULL && !s->hooks.validate(s->adc_min, s->adc_max)) {
        enter_aborted(s, PEDAL_CAL_FAIL_MIN_GE_MAX);
        return;
    }

    s->state = PEDAL_CAL_SAVING;

    /* Persist. */
    if (s->hooks.persist != NULL && !s->hooks.persist(s->adc_min, s->adc_max)) {
        enter_aborted(s, PEDAL_CAL_FAIL_READBACK);
        return;
    }
    /* Apply to the live pipeline. */
    if (s->hooks.apply != NULL) {
        s->hooks.apply(s->adc_min, s->adc_max);
    }
    /* Read back and verify what was actually stored. */
    if (s->hooks.readback != NULL) {
        uint16_t rb_min = 0U, rb_max = 0U;
        if (!s->hooks.readback(&rb_min, &rb_max) ||
            rb_min != s->adc_min || rb_max != s->adc_max) {
            enter_aborted(s, PEDAL_CAL_FAIL_READBACK);
            return;
        }
    }

    s->state        = PEDAL_CAL_COMPLETED;
    s->reason       = PEDAL_CAL_SESS_OK;
    s->sample_count = 0U;
}

/* ------------------------------------------------------------------ */
const char *PedalCalSession_StateText(PedalCalState st)
{
    switch (st) {
    case PEDAL_CAL_IDLE:                 return "IDLE";
    case PEDAL_CAL_ENTERING:             return "ENTERING";
    case PEDAL_CAL_WAIT_RELEASED:        return "SUELTA EL PEDAL";
    case PEDAL_CAL_CAPTURING_MIN:        return "CAPTURANDO MIN";
    case PEDAL_CAL_WAIT_FULL_PRESS:      return "PISA A FONDO";
    case PEDAL_CAL_CAPTURING_MAX:        return "CAPTURANDO MAX";
    case PEDAL_CAL_WAIT_RELEASE_FOR_SAVE:return "SUELTA PARA GUARDAR";
    case PEDAL_CAL_READY_TO_SAVE:        return "LISTO PARA GUARDAR";
    case PEDAL_CAL_SAVING:               return "GUARDANDO";
    case PEDAL_CAL_COMPLETED:            return "COMPLETADO";
    case PEDAL_CAL_ABORTED:              return "CANCELADO";
    default:                             return "?";
    }
}

const char *PedalCalSession_ReasonText(uint32_t reason_mask)
{
    /* Highest-priority (most safety-relevant) cause first. */
    if (reason_mask == PEDAL_CAL_SESS_OK)               return "OK";
    if (reason_mask & PEDAL_CAL_ABORT_EMERGENCY)        return "EMERGENCIA";
    if (reason_mask & PEDAL_CAL_ABORT_SAFE)             return "MODO SEGURO";
    if (reason_mask & PEDAL_CAL_ABORT_ERROR)            return "ERROR CRITICO";
    if (reason_mask & PEDAL_CAL_ABORT_LOCK_LOST)        return "BLOQUEO PERDIDO";
    if (reason_mask & PEDAL_CAL_BLOCK_LOCK_NOT_CONFIRMED) return "BLOQUEO NO CONFIRMADO";
    if (reason_mask & PEDAL_CAL_ABORT_MOVEMENT)         return "VEHICULO EN MOVIMIENTO";
    if (reason_mask & PEDAL_CAL_ABORT_CAN_LOSS)         return "SIN CAN";
    if (reason_mask & PEDAL_CAL_ABORT_TIMEOUT)          return "TIEMPO AGOTADO";
    if (reason_mask & PEDAL_CAL_ABORT_OPERATOR)         return "CANCELADO POR OPERADOR";
    if (reason_mask & PEDAL_CAL_BLOCK_NOT_STANDBY)      return "NO EN STANDBY";
    if (reason_mask & PEDAL_CAL_BLOCK_GEAR)             return "PON P O N";
    if (reason_mask & PEDAL_CAL_BLOCK_WHEELS_MOVING)    return "RUEDAS EN MOVIMIENTO";
    if (reason_mask & PEDAL_CAL_BLOCK_TRACTION_LIVE)    return "TRACCION ACTIVA";
    if (reason_mask & PEDAL_CAL_BLOCK_CRITICAL_ERROR)   return "ERROR CRITICO";
    if (reason_mask & PEDAL_CAL_BLOCK_PEDAL_IMPLAUSIBLE)return "PEDAL NO FIABLE";
    if (reason_mask & PEDAL_CAL_FAIL_MIN_GE_MAX)        return "MIN >= MAX";
    if (reason_mask & PEDAL_CAL_FAIL_RANGE_SMALL)       return "RANGO INSUFICIENTE";
    if (reason_mask & PEDAL_CAL_FAIL_UNSTABLE)          return "LECTURA INESTABLE";
    if (reason_mask & PEDAL_CAL_FAIL_READBACK)          return "FALLO AL GUARDAR";
    return "BLOQUEADO";
}
