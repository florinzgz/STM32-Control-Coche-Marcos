/**
  ****************************************************************************
  * @file    service_diag_session.h
  * @brief   SERVICE_DIAG session — pure decision core (Bloque A, PR #445).
  *
  * Engineering self-test session that lets a technician command a SINGLE
  * actuator at a bounded, reduced PWM while the vehicle is suspended (all
  * four wheels off the ground), without clearing or masking any latched
  * fault and without touching the real vehicle safety state machine
  * (Safety_GetState() / Safety_SetState()).
  *
  * DESIGN — orthogonal session layer, NOT a new SystemState_t value:
  *   This module owns its OWN state machine (ServiceDiagState_t) layered
  *   ON TOP of whichever SystemState_t the vehicle is already in.  Entry
  *   snapshots the current state opaquely (origin_state_raw, never
  *   interpreted as an enum by this module); exit (finish/abort/timeout)
  *   simply stops commanding the test actuator and lets the real state
  *   machine keep running exactly as it would have without this module —
  *   there is nothing to "restore" because nothing was ever mutated.
  *   This also gives a robust, free abort trigger: if the live system
  *   state ever differs from the recorded origin (in EITHER direction —
  *   e.g. a real fault escalates DEGRADED->SAFE, or a state recovers),
  *   the session aborts (SVCDIAG_REASON_STATE_CHANGED).
  *
  * Like pedal_cal_session.c, this is a PURE decision core: no HAL, no
  * flash, no delays, no direct sensor/actuator access.  The caller (main.c /
  * can_handler.c) fills a ServiceDiagConds snapshot every cycle from the
  * live safety/sensor state and applies whatever PWM/channel/direction the
  * accessors report; this module never reads or writes hardware itself,
  * which is what makes it host-testable (test_service_diag_session.c).
  *
  * Safety envelope (spec, Bloque A) — enforced unconditionally, every tick:
  *   - PWM ceiling: SVCDIAG_PWM_DEFAULT_CEILING_PCT (50%) by default, hard
  *     absolute ceiling SVCDIAG_PWM_ABS_MAX_PCT (60%) that NOTHING can
  *     exceed, regardless of what is requested or configured.
  *   - Exactly one actuator (one channel) active at a time — structural:
  *     the session carries a single active_channel field, not an array.
  *   - Plateau <= SVCDIAG_STEP_PLATEAU_MAX_MS (2000 ms); independent hard
  *     per-step timeout SVCDIAG_STEP_TIMEOUT_MS (4000 ms); session timeout
  *     SVCDIAG_SESSION_TIMEOUT_MS (10 min).
  *   - Test overcurrent (caller-detected: active channel current > 60% of
  *     its NORMAL limit) coasts the channel, marks the step
  *     SVCDIAG_STEP_FAIL_OVERCURRENT, and ABORTS the session — it never
  *     calls Safety_SetError()/Safety_SetState(SAFE); "no lleva a SAFE" is
  *     honoured by this module never touching the real safety machine.
  *   - Stops are always coast (the caller must never brake-actively when
  *     applying active_pwm_pct==0 after an abort/step-end); a fixed
  *     SVCDIAG_DEAD_TIME_MS elapses (PWM already 0) before the next step
  *     may begin.
  *   - Watchdog: if Update() is not called for longer than
  *     SVCDIAG_WATCHDOG_MAX_GAP_MS while a channel is live, the NEXT call
  *     detects the stale gap, forces active_pwm_pct to 0 and aborts
  *     (SVCDIAG_REASON_WATCHDOG) instead of resuming the stale command.
  *
  * Entry gates (Begin()) — ALL required:
  *   - NOT in SYS_STATE_BOOT (spec explicitly allows STANDBY, ACTIVE,
  *     DEGRADED L1/L2/L3, LIMP_HOME, SAFE and ERROR — i.e. every state
  *     except BOOT, which means peripherals are still initialising).
  *   - Gear in Park or Neutral.
  *   - All four wheel speeds have read below SVCDIAG_WHEEL_STATIONARY_KMH
  *     for at least SVCDIAG_STATIONARY_MIN_MS (pre-digested by the caller
  *     into wheels_stationary_1s — the 1 s debounce timer is bookkeeping
  *     the caller owns, exactly like pedal_cal_session.c's wheels_moving).
  *   - Explicit operator confirmation token (the ESP32 dedicated dialog
  *     "VEHICULO SUSPENDIDO - 4 RUEDAS EN EL AIRE"), not an ordinary button.
  *   - Battery above the CUTOFF threshold (the hardest of the 3 battery
  *     thresholds; the softer WARNING threshold is instead an always-on
  *     abort trigger below, per spec — entry is deliberately looser than
  *     the running abort so a marginal battery still gets a clear,
  *     immediate abort rather than being silently refused entry).
  *
  * The PIN gate is intentionally NOT part of this module: it is checked by
  * the ESP32's PIN-gated engineering menu before SERVICE_ACTION_SELF_TEST
  * is ever sent, exactly like every other SERVICE_CMD action in this repo.
  ****************************************************************************
  */

#ifndef SERVICE_DIAG_SESSION_H
#define SERVICE_DIAG_SESSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---- Safety envelope constants (spec-mandated ceilings, not tunables) --- */
#define SVCDIAG_PWM_DEFAULT_CEILING_PCT   50U      /* default per-channel ceiling      */
#define SVCDIAG_PWM_ABS_MAX_PCT           60U      /* absolute, non-negotiable ceiling */
#define SVCDIAG_STEP_PLATEAU_MIN_MS       100U
#define SVCDIAG_STEP_PLATEAU_MAX_MS       2000U
#define SVCDIAG_STEP_TIMEOUT_MS           4000U
#define SVCDIAG_SESSION_TIMEOUT_MS        600000U  /* 10 minutes                       */
#define SVCDIAG_DEAD_TIME_MS              300U
#define SVCDIAG_WATCHDOG_MAX_GAP_MS       100U      /* 10x the nominal 10 ms cycle      */
#define SVCDIAG_STATIONARY_MIN_MS         1000U
#define SVCDIAG_WHEEL_STATIONARY_KMH      0.5f
#define SVCDIAG_TEST_OVERCURRENT_FACTOR   0.60f     /* 60% of the channel normal limit  */
#define SVCDIAG_CAN_HEARTBEAT_ABORT_MS    500U
#define SVCDIAG_CONFIRM_TOKEN             0xA5U

_Static_assert(SVCDIAG_PWM_DEFAULT_CEILING_PCT <= SVCDIAG_PWM_ABS_MAX_PCT,
               "default PWM ceiling must not exceed the absolute maximum");
_Static_assert(SVCDIAG_STEP_PLATEAU_MAX_MS <= SVCDIAG_STEP_TIMEOUT_MS,
               "plateau ceiling must fit inside the per-step hard timeout");
_Static_assert(SVCDIAG_STEP_PLATEAU_MIN_MS < SVCDIAG_STEP_PLATEAU_MAX_MS,
               "plateau bounds must be a non-empty range");

/* ---- Session FSM state (stable, CAN-transportable; append only) -------- */
typedef enum {
    SVCDIAG_STATE_IDLE = 0,     /* no session                                */
    SVCDIAG_STATE_ENTERING,     /* transient: entry accepted, arming         */
    SVCDIAG_STATE_ARMED,        /* session live, no actuator commanded       */
    SVCDIAG_STATE_STEPPING,     /* one channel commanded at a clamped PWM    */
    SVCDIAG_STATE_DEADTIME,     /* PWM 0, mandatory coast dead-time          */
    SVCDIAG_STATE_ABORTED,      /* terminal until a new Begin()              */
} ServiceDiagState_t;

/* ---- Test channel (stable, CAN-transportable) --------------------------- */
typedef enum {
    SVCDIAG_CH_FL        = 0,
    SVCDIAG_CH_FR        = 1,
    SVCDIAG_CH_RL        = 2,
    SVCDIAG_CH_RR        = 3,
    SVCDIAG_CH_STEERING  = 4,
    SVCDIAG_CH_NONE      = 5,
} ServiceDiagChannel_t;

typedef enum {
    SVCDIAG_DIR_FORWARD = 0,
    SVCDIAG_DIR_REVERSE = 1,
} ServiceDiagDirection_t;

typedef enum {
    SVCDIAG_STEP_NONE = 0,
    SVCDIAG_STEP_RUNNING,
    SVCDIAG_STEP_PASS,
    SVCDIAG_STEP_FAIL_OVERCURRENT,
} ServiceDiagStepVerdict_t;

/* ---- Abort/reject reason (single, highest-priority cause; stable,
 * CAN-transportable — mirrors the "motivo de aborto" 0x31B byte). --------- */
typedef enum {
    SVCDIAG_REASON_NONE = 0,
    SVCDIAG_REASON_OPERATOR,          /* explicit ABORT (incl. HMI touch/exit)    */
    SVCDIAG_REASON_BOOT_STATE,        /* entry block: system in BOOT              */
    SVCDIAG_REASON_GEAR,              /* entry block or mid-session: not P/N      */
    SVCDIAG_REASON_WHEELS_MOVING,     /* entry block: not stationary >= 1 s       */
    SVCDIAG_REASON_NOT_CONFIRMED,     /* entry block: confirm token missing/wrong */
    SVCDIAG_REASON_BATTERY_LOW,       /* entry block: battery <= cutoff           */
    SVCDIAG_REASON_BATTERY_WARN,      /* mid-session: battery < warning threshold */
    SVCDIAG_REASON_ALREADY_ACTIVE,    /* entry block: a session is already running*/
    SVCDIAG_REASON_STATE_CHANGED,     /* mid-session: origin state no longer holds*/
    SVCDIAG_REASON_CAN_LOSS,          /* mid-session: CAN heartbeat > 500 ms      */
    SVCDIAG_REASON_GROUNDED_WHEEL,    /* mid-session: pulses on a non-driven wheel*/
    SVCDIAG_REASON_OVERCURRENT,       /* mid-session: test overcurrent, 1 sample  */
    SVCDIAG_REASON_SESSION_TIMEOUT,   /* mid-session: 10 minute session ceiling   */
    SVCDIAG_REASON_STEP_TIMEOUT,      /* mid-session: 4000 ms per-step hard cap   */
    SVCDIAG_REASON_WATCHDOG,          /* mid-session: stale Update() tick detected*/
} ServiceDiagReason_t;

/* ---- Per-tick condition snapshot (all facts the FSM needs) ---------------
 * The caller fills this from the live safety/sensor state every cycle; this
 * module never reads hardware itself. ------------------------------------- */
typedef struct {
    uint32_t now_ms;

    /* Entry gates — read only by Begin(). */
    bool     boot_state;              /* Safety_GetState() == SYS_STATE_BOOT      */
    bool     gear_park_or_neutral;
    bool     wheels_stationary_1s;    /* all 4 wheels < epsilon for >= 1000 ms     */
    bool     confirm_token_ok;        /* operator confirmed the suspended-vehicle
                                        * dialog (START payload confirm byte)      */
    bool     battery_above_cutoff;

    /* Always-on facts — read every Update(). */
    uint8_t  system_state_raw;        /* opaque Safety_GetState() snapshot         */
    bool     battery_above_warning;
    uint32_t can_rx_age_ms;
    bool     grounded_wheel_pulse;    /* pulses detected on a wheel OTHER than the
                                        * currently active channel                 */
    bool     test_overcurrent;        /* active channel current > 60% of its
                                        * NORMAL (non-test) limit, caller-computed  */
    uint8_t  steering_pwm_ceiling_pct;/* from SteeringServiceStore_GetEffectiveClamped(),
                                        * converted to a % of full PWM scale by the
                                        * caller; ignored for wheel channels        */
} ServiceDiagConds;

/* ---- Tunables (genuinely configurable; safety ceilings above are NOT
 * exposed here on purpose — they are non-negotiable #defines). ------------ */
typedef struct {
    uint8_t wheel_pwm_ceiling_pct;    /* default SVCDIAG_PWM_DEFAULT_CEILING_PCT;
                                       * Init() hard-clamps to SVCDIAG_PWM_ABS_MAX_PCT
                                       * regardless of what is passed in.           */
} ServiceDiagCfg;

/* ---- Session object (opaque-ish POD so tests can inspect it) ------------ */
typedef struct {
    ServiceDiagCfg           cfg;
    ServiceDiagState_t       state;
    ServiceDiagReason_t      reason;
    uint8_t                  origin_state_raw;   /* snapshot of system_state_raw at entry */

    ServiceDiagChannel_t     active_channel;
    ServiceDiagDirection_t   active_direction;
    uint8_t                  active_pwm_pct;     /* clamped value currently applied  */
    uint16_t                 active_plateau_ms;
    ServiceDiagStepVerdict_t step_verdict;
    uint8_t                  step_index;

    uint32_t session_start_ms;
    uint32_t step_start_ms;
    uint32_t deadtime_start_ms;

    uint32_t last_update_ms;
    bool     have_last_update;       /* false until the first Update() after Begin()*/
} ServiceDiagSession;

/* Default configuration (50% default ceiling). */
ServiceDiagCfg ServiceDiagSession_DefaultCfg(void);

/* Initialise a session to IDLE with the given config (NULL -> defaults). */
void ServiceDiagSession_Init(ServiceDiagSession *s, const ServiceDiagCfg *cfg);

/* Operator request to start a session (SERVICE_ACTION_SELF_TEST START).
 * Returns true if accepted (IDLE/ABORTED -> ARMED); on rejection the block
 * reason is latched, state stays IDLE, and false is returned. */
bool ServiceDiagSession_Begin(ServiceDiagSession *s, const ServiceDiagConds *c);

/* Periodic tick — call EVERY cycle while state != IDLE. Enforces the
 * always-on abort conditions, the watchdog, plateau/timeouts, and dead-time.
 * Returns the new state. */
ServiceDiagState_t ServiceDiagSession_Update(ServiceDiagSession *s, const ServiceDiagConds *c);

/* Operator request to run one step (SERVICE_ACTION_SELF_TEST NEXT). Only
 * accepted from ARMED or DEADTIME-elapsed (i.e. NOT while STEPPING — this
 * is what makes "one actuator at a time" and "dead-time between steps"
 * structural). requested_pwm_pct/plateau_ms are clamped to the safety
 * envelope before being applied; returns true if the step was accepted. */
bool ServiceDiagSession_RequestStep(ServiceDiagSession *s, const ServiceDiagConds *c,
                                     ServiceDiagChannel_t channel,
                                     ServiceDiagDirection_t direction,
                                     uint8_t requested_pwm_pct,
                                     uint16_t requested_plateau_ms);

/* Operator/host abort at any time (SERVICE_ACTION_SELF_TEST ABORT). Also the
 * target of ESP32-side "touch during test" / "left the submenu" triggers,
 * which map to this same call with SVCDIAG_REASON_OPERATOR. */
void ServiceDiagSession_Abort(ServiceDiagSession *s, ServiceDiagReason_t reason);

/* ---- Accessors ------------------------------------------------------- */
static inline ServiceDiagState_t ServiceDiagSession_State(const ServiceDiagSession *s)
{
    return s->state;
}
static inline ServiceDiagReason_t ServiceDiagSession_Reason(const ServiceDiagSession *s)
{
    return s->reason;
}
static inline bool ServiceDiagSession_Active(const ServiceDiagSession *s)
{
    return s->state != SVCDIAG_STATE_IDLE && s->state != SVCDIAG_STATE_ABORTED;
}
/* The channel/PWM/direction to actually apply THIS cycle — NONE/0 unless
 * genuinely STEPPING, so the caller never needs its own extra "are we
 * stepping" check to decide whether to write the actuator. */
static inline ServiceDiagChannel_t ServiceDiagSession_ActiveChannel(const ServiceDiagSession *s)
{
    return (s->state == SVCDIAG_STATE_STEPPING) ? s->active_channel : SVCDIAG_CH_NONE;
}
static inline uint8_t ServiceDiagSession_ActivePwmPct(const ServiceDiagSession *s)
{
    return (s->state == SVCDIAG_STATE_STEPPING) ? s->active_pwm_pct : 0U;
}
static inline ServiceDiagDirection_t ServiceDiagSession_ActiveDirection(const ServiceDiagSession *s)
{
    return s->active_direction;
}
static inline uint8_t ServiceDiagSession_StepIndex(const ServiceDiagSession *s)
{
    return s->step_index;
}
static inline ServiceDiagStepVerdict_t ServiceDiagSession_StepVerdict(const ServiceDiagSession *s)
{
    return s->step_verdict;
}
static inline uint8_t ServiceDiagSession_OriginState(const ServiceDiagSession *s)
{
    return s->origin_state_raw;
}

/* Progress within the current step, 0-100 (0 outside STEPPING). */
uint8_t ServiceDiagSession_ProgressPct(const ServiceDiagSession *s, uint32_t now_ms);

/* Elapsed session time in whole seconds, saturated at 255 (0 if IDLE). */
uint8_t ServiceDiagSession_ElapsedSec(const ServiceDiagSession *s, uint32_t now_ms);

/* Human-readable label for a state / reason (HMI). */
const char *ServiceDiagSession_StateText(ServiceDiagState_t st);
const char *ServiceDiagSession_ReasonText(ServiceDiagReason_t r);

#ifdef __cplusplus
}
#endif

#endif /* SERVICE_DIAG_SESSION_H */
