/**
  ****************************************************************************
  * @file    service_diag_session.c
  * @brief   Implementation of the SERVICE_DIAG session pure decision core.
  *
  * Pure logic only — no HAL, no flash, no delays, no direct actuator access.
  * See service_diag_session.h for the full contract.
  ****************************************************************************
  */

#include "service_diag_session.h"

#include <string.h>

/* ------------------------------------------------------------------ */
ServiceDiagCfg ServiceDiagSession_DefaultCfg(void)
{
    ServiceDiagCfg cfg;
    cfg.wheel_pwm_ceiling_pct = SVCDIAG_PWM_DEFAULT_CEILING_PCT;
    return cfg;
}

void ServiceDiagSession_Init(ServiceDiagSession *s, const ServiceDiagCfg *cfg)
{
    if (s == NULL) return;
    memset(s, 0, sizeof(*s));
    s->cfg = (cfg != NULL) ? *cfg : ServiceDiagSession_DefaultCfg();
    /* Defense in depth: the absolute ceiling is never negotiable, no matter
     * what the caller passes in. */
    if (s->cfg.wheel_pwm_ceiling_pct > SVCDIAG_PWM_ABS_MAX_PCT) {
        s->cfg.wheel_pwm_ceiling_pct = SVCDIAG_PWM_ABS_MAX_PCT;
    }
    s->state             = SVCDIAG_STATE_IDLE;
    s->reason            = SVCDIAG_REASON_NONE;
    s->active_channel    = SVCDIAG_CH_NONE;
    s->active_direction  = SVCDIAG_DIR_FORWARD;
    s->step_verdict      = SVCDIAG_STEP_NONE;
}

/* ---- Entry guards (spec Bloque A) --------------------------------- */
static ServiceDiagReason_t entry_block(const ServiceDiagConds *c)
{
    if (c->boot_state)             return SVCDIAG_REASON_BOOT_STATE;
    if (!c->gear_park_or_neutral)  return SVCDIAG_REASON_GEAR;
    if (!c->wheels_stationary_1s)  return SVCDIAG_REASON_WHEELS_MOVING;
    if (!c->confirm_token_ok)      return SVCDIAG_REASON_NOT_CONFIRMED;
    if (!c->battery_above_cutoff)  return SVCDIAG_REASON_BATTERY_LOW;
    return SVCDIAG_REASON_NONE;
}

/* ---- Always-on abort conditions (spec Bloque A) -------------------
 * Single, highest-priority cause; test-overcurrent and the watchdog are
 * handled separately in Update() because they need extra side effects
 * (step verdict, forced PWM=0) beyond a plain abort. */
static ServiceDiagReason_t abort_mask(const ServiceDiagConds *c, uint8_t origin_state_raw)
{
    if (c->system_state_raw != origin_state_raw)         return SVCDIAG_REASON_STATE_CHANGED;
    if (!c->gear_park_or_neutral)                         return SVCDIAG_REASON_GEAR;
    if (c->can_rx_age_ms > SVCDIAG_CAN_HEARTBEAT_ABORT_MS) return SVCDIAG_REASON_CAN_LOSS;
    if (!c->battery_above_warning)                        return SVCDIAG_REASON_BATTERY_WARN;
    if (c->grounded_wheel_pulse)                           return SVCDIAG_REASON_GROUNDED_WHEEL;
    return SVCDIAG_REASON_NONE;
}

static void enter_aborted(ServiceDiagSession *s, ServiceDiagReason_t reason)
{
    s->state          = SVCDIAG_STATE_ABORTED;
    s->reason         = reason;
    s->active_channel = SVCDIAG_CH_NONE;
    s->active_pwm_pct = 0U;
}

/* ------------------------------------------------------------------ */
bool ServiceDiagSession_Begin(ServiceDiagSession *s, const ServiceDiagConds *c)
{
    if (s == NULL || c == NULL) return false;

    if (s->state != SVCDIAG_STATE_IDLE && s->state != SVCDIAG_STATE_ABORTED) {
        s->reason = SVCDIAG_REASON_ALREADY_ACTIVE;
        return false;  /* already running — do not disturb it */
    }

    ServiceDiagReason_t block = entry_block(c);
    if (block != SVCDIAG_REASON_NONE) {
        s->state  = SVCDIAG_STATE_IDLE;
        s->reason = block;
        return false;
    }

    s->state             = SVCDIAG_STATE_ENTERING;
    s->reason            = SVCDIAG_REASON_NONE;
    s->origin_state_raw  = c->system_state_raw;
    s->active_channel    = SVCDIAG_CH_NONE;
    s->active_direction  = SVCDIAG_DIR_FORWARD;
    s->active_pwm_pct    = 0U;
    s->active_plateau_ms = 0U;
    s->step_verdict      = SVCDIAG_STEP_NONE;
    s->step_index        = 0U;
    s->session_start_ms  = c->now_ms;
    s->step_start_ms     = c->now_ms;
    s->deadtime_start_ms = c->now_ms;
    s->last_update_ms    = c->now_ms;
    s->have_last_update  = true;
    return true;
}

ServiceDiagState_t ServiceDiagSession_Update(ServiceDiagSession *s, const ServiceDiagConds *c)
{
    if (s == NULL) return SVCDIAG_STATE_IDLE;
    if (c == NULL) return s->state;

    if (!ServiceDiagSession_Active(s)) {
        return s->state;
    }

    /* 1. Watchdog: a stale gap while a channel is actually being driven
     *    means the actuator may have kept running unattended in between —
     *    force it to coast and abort instead of silently resuming. */
    if (s->state == SVCDIAG_STATE_STEPPING && s->have_last_update &&
        (uint32_t)(c->now_ms - s->last_update_ms) > SVCDIAG_WATCHDOG_MAX_GAP_MS) {
        s->active_pwm_pct = 0U;
        enter_aborted(s, SVCDIAG_REASON_WATCHDOG);
        s->last_update_ms   = c->now_ms;
        s->have_last_update = true;
        return s->state;
    }
    s->last_update_ms   = c->now_ms;
    s->have_last_update = true;

    /* 2. Test overcurrent: coast the channel, FAIL the step, abort the
     *    session — never touches Safety_SetError()/Safety_SetState(). */
    if (s->state == SVCDIAG_STATE_STEPPING && c->test_overcurrent) {
        s->active_pwm_pct = 0U;
        s->step_verdict   = SVCDIAG_STEP_FAIL_OVERCURRENT;
        enter_aborted(s, SVCDIAG_REASON_OVERCURRENT);
        return s->state;
    }

    /* 3. Generic always-on aborts (state changed, gear, CAN, battery,
     *    grounded wheel). */
    ServiceDiagReason_t ab = abort_mask(c, s->origin_state_raw);
    if (ab != SVCDIAG_REASON_NONE) {
        s->active_pwm_pct = 0U;
        enter_aborted(s, ab);
        return s->state;
    }

    /* 4. Session-wide timeout (10 min). */
    if ((uint32_t)(c->now_ms - s->session_start_ms) > SVCDIAG_SESSION_TIMEOUT_MS) {
        s->active_pwm_pct = 0U;
        enter_aborted(s, SVCDIAG_REASON_SESSION_TIMEOUT);
        return s->state;
    }

    switch (s->state) {
    case SVCDIAG_STATE_ENTERING:
        s->state = SVCDIAG_STATE_ARMED;
        break;

    case SVCDIAG_STATE_ARMED:
        /* Waiting for RequestStep(); nothing to do here. */
        break;

    case SVCDIAG_STATE_STEPPING:
        if ((uint32_t)(c->now_ms - s->step_start_ms) > SVCDIAG_STEP_TIMEOUT_MS) {
            s->active_pwm_pct = 0U;
            enter_aborted(s, SVCDIAG_REASON_STEP_TIMEOUT);
            break;
        }
        if ((uint32_t)(c->now_ms - s->step_start_ms) >= s->active_plateau_ms) {
            s->step_verdict      = SVCDIAG_STEP_PASS;
            s->active_pwm_pct    = 0U;      /* coast — never active braking */
            s->state             = SVCDIAG_STATE_DEADTIME;
            s->deadtime_start_ms = c->now_ms;
        }
        break;

    case SVCDIAG_STATE_DEADTIME:
        if ((uint32_t)(c->now_ms - s->deadtime_start_ms) >= SVCDIAG_DEAD_TIME_MS) {
            s->state = SVCDIAG_STATE_ARMED;
        }
        break;

    default:
        break;
    }

    return s->state;
}

bool ServiceDiagSession_RequestStep(ServiceDiagSession *s, const ServiceDiagConds *c,
                                     ServiceDiagChannel_t channel,
                                     ServiceDiagDirection_t direction,
                                     uint8_t requested_pwm_pct,
                                     uint16_t requested_plateau_ms)
{
    if (s == NULL || c == NULL) return false;
    /* Only from ARMED — never while STEPPING (single actuator at a time,
     * structurally) and never while DEADTIME has not yet elapsed (mandatory
     * coast dead-time between steps). */
    if (s->state != SVCDIAG_STATE_ARMED) return false;
    if (channel > SVCDIAG_CH_STEERING) return false;
    if (direction != SVCDIAG_DIR_FORWARD && direction != SVCDIAG_DIR_REVERSE) return false;

    uint8_t ceiling = (channel == SVCDIAG_CH_STEERING)
                          ? c->steering_pwm_ceiling_pct
                          : s->cfg.wheel_pwm_ceiling_pct;
    /* Defense in depth: no channel's ceiling may ever exceed the absolute
     * maximum, regardless of what Cfg or the store computed. */
    if (ceiling > SVCDIAG_PWM_ABS_MAX_PCT) ceiling = SVCDIAG_PWM_ABS_MAX_PCT;

    uint8_t pwm = requested_pwm_pct;
    if (pwm > ceiling) pwm = ceiling;

    uint16_t plateau = requested_plateau_ms;
    if (plateau < SVCDIAG_STEP_PLATEAU_MIN_MS) plateau = SVCDIAG_STEP_PLATEAU_MIN_MS;
    if (plateau > SVCDIAG_STEP_PLATEAU_MAX_MS) plateau = SVCDIAG_STEP_PLATEAU_MAX_MS;

    s->active_channel    = channel;
    s->active_direction   = direction;
    s->active_pwm_pct     = pwm;
    s->active_plateau_ms  = plateau;
    s->step_verdict       = SVCDIAG_STEP_RUNNING;
    s->step_index         = (uint8_t)(s->step_index + 1U);
    s->step_start_ms      = c->now_ms;
    s->state              = SVCDIAG_STATE_STEPPING;
    return true;
}

void ServiceDiagSession_Abort(ServiceDiagSession *s, ServiceDiagReason_t reason)
{
    if (s == NULL) return;
    if (s->state == SVCDIAG_STATE_IDLE || s->state == SVCDIAG_STATE_ABORTED) return;
    enter_aborted(s, reason == SVCDIAG_REASON_NONE ? SVCDIAG_REASON_OPERATOR : reason);
}

/* ------------------------------------------------------------------ */
uint8_t ServiceDiagSession_ProgressPct(const ServiceDiagSession *s, uint32_t now_ms)
{
    if (s == NULL || s->state != SVCDIAG_STATE_STEPPING || s->active_plateau_ms == 0U) {
        return 0U;
    }
    uint32_t elapsed = (uint32_t)(now_ms - s->step_start_ms);
    if (elapsed >= s->active_plateau_ms) return 100U;
    return (uint8_t)((elapsed * 100U) / s->active_plateau_ms);
}

uint8_t ServiceDiagSession_ElapsedSec(const ServiceDiagSession *s, uint32_t now_ms)
{
    if (s == NULL || !ServiceDiagSession_Active(s)) return 0U;
    uint32_t ms  = (uint32_t)(now_ms - s->session_start_ms);
    uint32_t sec = ms / 1000U;
    return (sec > 255U) ? 255U : (uint8_t)sec;
}

const char *ServiceDiagSession_StateText(ServiceDiagState_t st)
{
    switch (st) {
    case SVCDIAG_STATE_IDLE:     return "INACTIVO";
    case SVCDIAG_STATE_ENTERING: return "ENTRANDO";
    case SVCDIAG_STATE_ARMED:    return "LISTO";
    case SVCDIAG_STATE_STEPPING: return "EJECUTANDO";
    case SVCDIAG_STATE_DEADTIME: return "PAUSA";
    case SVCDIAG_STATE_ABORTED:  return "ABORTADO";
    default:                     return "?";
    }
}

const char *ServiceDiagSession_ReasonText(ServiceDiagReason_t r)
{
    switch (r) {
    case SVCDIAG_REASON_NONE:              return "OK";
    case SVCDIAG_REASON_OPERATOR:          return "CANCELADO POR OPERADOR";
    case SVCDIAG_REASON_BOOT_STATE:        return "SISTEMA ARRANCANDO";
    case SVCDIAG_REASON_GEAR:              return "PON P O N";
    case SVCDIAG_REASON_WHEELS_MOVING:     return "RUEDAS EN MOVIMIENTO";
    case SVCDIAG_REASON_NOT_CONFIRMED:     return "FALTA CONFIRMACION";
    case SVCDIAG_REASON_BATTERY_LOW:       return "BATERIA BAJO CORTE";
    case SVCDIAG_REASON_BATTERY_WARN:      return "BATERIA BAJO AVISO";
    case SVCDIAG_REASON_ALREADY_ACTIVE:    return "SESION YA ACTIVA";
    case SVCDIAG_REASON_STATE_CHANGED:     return "ESTADO DEL VEHICULO CAMBIO";
    case SVCDIAG_REASON_CAN_LOSS:          return "SIN CAN";
    case SVCDIAG_REASON_GROUNDED_WHEEL:    return "COCHE APOYADO";
    case SVCDIAG_REASON_OVERCURRENT:       return "SOBRECORRIENTE DE PRUEBA";
    case SVCDIAG_REASON_SESSION_TIMEOUT:   return "TIEMPO DE SESION AGOTADO";
    case SVCDIAG_REASON_STEP_TIMEOUT:      return "TIEMPO DE PASO AGOTADO";
    case SVCDIAG_REASON_WATCHDOG:          return "WATCHDOG";
    default:                               return "?";
    }
}
