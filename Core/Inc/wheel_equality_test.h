/**
  ****************************************************************************
  * @file    wheel_equality_test.h
  * @brief   Wheel-equality / BTS7960 health self-test — pure decision core
  *          (Hito 2, PR #445).
  *
  * Engineering self-test that commands RAW, IDENTICAL PWM to the four
  * traction wheels (one at a time in Fase 1, all four at once in the
  * optional Fase 2) and compares the resulting normalized speed and current
  * to reveal mechanical, electrical, sensor or BTS7960 driver defects.  The
  * four wheels are identical hardware (same motor, same wheel diameter,
  * same pulses/rev, same BTS7960 model); with the steering wheel centred
  * and the same PWM applied, all four MUST spin at the same speed.  Any
  * deviation is a defect to diagnose — this module is FORBIDDEN from
  * applying any per-wheel trim/compensation; its only job is to REVEAL
  * differences, never to mask them.
  *
  * DESIGN — leans on the Hito 1 ServiceDiagSession (service_diag_session.h)
  * without modifying it:
  *   - Begin() requires the caller to report the Hito 1 session as ARMED
  *     (`WheelEqConds.svc_armed`) — every one of Hito 1's entry gates
  *     (not BOOT, gear P/N, wheels stationary 1 s + confirmed suspended
  *     vehicle, battery above cutoff) must already hold, with ZERO
  *     duplication of that logic here.
  *   - Update() requires the caller to keep reporting the Hito 1 session as
  *     Active() (`WheelEqConds.svc_active`) every cycle; the instant Hito 1's
  *     OWN always-on abort conditions fire (state changed, gear left P/N,
  *     CAN loss, battery warning, watchdog, session timeout) and its session
  *     goes inactive, this module aborts too (WHEQ_REASON_ENVELOPE_ABORT).
  *     This is how "Mantiene TODOS los demás abortos de la envolvente" is
  *     satisfied without copying a single threshold.
  *   - The safety-envelope ceilings actually used here (PWM levels, dead
  *     time) are deliberately far inside Hito 1's own limits: 25 %/50 %
  *     (Fase 1) and 40 % (Fase 2) are all <= SVCDIAG_PWM_ABS_MAX_PCT (60 %);
  *     the per-step measurement window reuses SVCDIAG_STEP_PLATEAU_MAX_MS
  *     and the inter-step coast reuses SVCDIAG_DEAD_TIME_MS verbatim (see
  *     the _Static_assert block below).  Fase 2's 3000 ms plateau is the
  *     ONE explicit, spec-mandated exception (see WHEQ_PHASE2_PLATEAU_MS).
  *
  * Like service_diag_session.c, this is a PURE decision core: no HAL, no
  * flash, no delays, no direct sensor/actuator access.  The caller
  * (can_handler.c) fills a WheelEqConds snapshot every cycle from the live
  * safety/sensor state and applies whatever wheel_mask/pwm_pct/direction
  * WheelEqTest_GetActuation() reports; this module never reads or writes
  * hardware itself, which is what makes it host-testable
  * (test_wheel_equality_test.c).
  *
  * PROHIBITED: this module — and the actuation path that applies its
  * commanded PWM — must NEVER call TractionOutput_Resolve4x4() or
  * TractionOutput_Resolve4x2Rear() (traction_output_policy.h).  Those
  * functions equalise PWM to the minimum across wheels in straight-line
  * conditions specifically to make the vehicle drive straight; applying
  * them here would silently erase the very asymmetry this test exists to
  * reveal.  The actuation glue (motor_control.c) documents this bypass at
  * the call site.
  *
  * Fase 1 — SEQUENTIAL (precise measurement, one actuator at a time, exactly
  * like Hito 1's structural "one channel" rule): 4 wheels x {25 %, 50 %} x
  * {FORWARD, REVERSE} = 16 auto-sequenced steps.  Each step commands ONE
  * wheel (the other three coast), discards the first
  * WHEQ_STARTUP_DISCARD_MS of the plateau (motor spin-up transient) and
  * averages pulses/s, current and battery voltage over the remainder.  Any
  * sample where ABS or TCS is intervening is discarded and the step is
  * retried (bounded by WHEQ_STEP_MAX_RETRIES) because a real ABS/TCS scale
  * factor would corrupt the measured PWM->speed relationship.
  *
  * Fase 2 — SIMULTANEOUS (optional, explicit exception to "one actuator at
  * a time"): all four wheels at once, PWM ceiling WHEQ_PHASE2_PWM_CEILING_PCT
  * (40 %), plateau WHEQ_PHASE2_PLATEAU_MS (3000 ms).  Only reachable from
  * WHEQ_STATE_PHASE1_DONE when Fase 1 produced ZERO FAIL of any kind
  * (wheel equality, half-bridge, or Ackermann-offset) — see
  * WheelEqTest_Phase1AllPass().  Every other Hito 1 abort still applies.
  ****************************************************************************
  */

#ifndef WHEEL_EQUALITY_TEST_H
#define WHEEL_EQUALITY_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "traction_output_policy.h"  /* TRACTION_OUTPUT_UNITY_EPSILON (pure header, no HAL) */
#include "service_diag_session.h"    /* SVCDIAG_* envelope constants to reuse (pure header)  */

/* ---- Sizing ------------------------------------------------------------ */
#define WHEQ_NUM_WHEELS              4U   /* FL, FR, RL, RR                */
#define WHEQ_NUM_LEVELS               2U   /* 25 %, 50 %                    */
#define WHEQ_NUM_DIRS                  2U   /* forward, reverse              */
#define WHEQ_PHASE1_STEPS_PER_WHEEL   (WHEQ_NUM_LEVELS * WHEQ_NUM_DIRS)  /* 4 */
#define WHEQ_PHASE1_TOTAL_STEPS       (WHEQ_NUM_WHEELS * WHEQ_PHASE1_STEPS_PER_WHEEL) /* 16 */

/* ---- PWM levels used by Fase 1 (named, not literals in the .c file) ---- */
#define WHEQ_PWM_LEVEL_25_PCT   25U
#define WHEQ_PWM_LEVEL_50_PCT   50U

/* ---- Fase 2 (simultaneous) envelope — spec-mandated exception ---------- */
#define WHEQ_PHASE2_PWM_CEILING_PCT   40U
#define WHEQ_PHASE2_PLATEAU_MS        3000U  /* explicit exception to
                                               * SVCDIAG_STEP_PLATEAU_MAX_MS
                                               * (2000 ms) — spec literal.   */

/* ---- Timing reused verbatim from the Hito 1 envelope -------------------- */
#define WHEQ_STARTUP_DISCARD_MS       500U   /* discard motor spin-up (spec) */
#define WHEQ_STEP_MEASURE_TOTAL_MS    SVCDIAG_STEP_PLATEAU_MAX_MS  /* 2000ms */
#define WHEQ_STEP_DEADTIME_MS         SVCDIAG_DEAD_TIME_MS         /* 300ms  */
#define WHEQ_WATCHDOG_MAX_GAP_MS      SVCDIAG_WATCHDOG_MAX_GAP_MS  /* 100ms  */
#define WHEQ_STEP_MAX_RETRIES         3U     /* bounded ABS/TCS-interference
                                               * retries before aborting     */

_Static_assert(WHEQ_STARTUP_DISCARD_MS < WHEQ_STEP_MEASURE_TOTAL_MS,
               "discard window must fit inside the measurement plateau");
_Static_assert(WHEQ_PWM_LEVEL_50_PCT <= SVCDIAG_PWM_DEFAULT_CEILING_PCT,
               "Fase 1 PWM levels must stay inside Hito 1's default ceiling");
_Static_assert(WHEQ_PHASE2_PWM_CEILING_PCT <= SVCDIAG_PWM_ABS_MAX_PCT,
               "Fase 2 PWM ceiling must never exceed Hito 1's absolute max");

/* ---- Verdict thresholds (named constants, spec-mandated numbers) ------- */
#define WHEQ_EQUALITY_WARN_PCT        5.0f
#define WHEQ_EQUALITY_FAIL_PCT        15.0f
#define WHEQ_HALFBRIDGE_WARN_PCT      5.0f
#define WHEQ_HALFBRIDGE_FAIL_PCT      20.0f
/* Slope-vs-median anomaly: the spec gives no numeric threshold for the
 * I/PWM slope comparison (section b), only "compárala con la mediana".
 * Reuse the equality thresholds for consistency (same "deviation from a
 * 4-channel median" philosophy, same order of magnitude of engineering
 * judgement) rather than invent an unrelated magic number.               */
#define WHEQ_SLOPE_WARN_PCT           WHEQ_EQUALITY_WARN_PCT
#define WHEQ_SLOPE_FAIL_PCT           WHEQ_EQUALITY_FAIL_PCT
/* Thermal drift: absolute degrees C by which one driver's temperature rise
 * may exceed the median rise of the other three before it contributes to
 * a SOSPECHOSO verdict.  No numeric value is given by the spec; documented
 * engineering judgement call (DS18B20 sensors are +-0.5 degC typical, so a
 * 5 degC gap is well outside sensor noise).                               */
#define WHEQ_THERMAL_DRIFT_WARN_DELTA_C  5.0f

/* ---- Driver-verdict "concrete criterion" bitmask (named, not literal) -- */
#define WHEQ_DRIVER_REASON_HALFBRIDGE  (1U << 0)  /* F/R asymmetry WARN/FAIL */
#define WHEQ_DRIVER_REASON_SLOPE       (1U << 1)  /* I/PWM slope vs median   */
#define WHEQ_DRIVER_REASON_ELECTRICAL  (1U << 2)  /* electrical-cause wheel  */
#define WHEQ_DRIVER_REASON_THERMAL     (1U << 3)  /* ΔT vs median            */

/* ---- FSM state (stable, CAN-transportable; append only) ---------------- */
typedef enum {
    WHEQ_STATE_IDLE = 0,
    WHEQ_STATE_BLOCKED_DEADBAND,    /* rejected: steering not centred        */
    WHEQ_STATE_BLOCKED_ACKERMANN,   /* rejected: Ackermann diff != 1.000     */
    WHEQ_STATE_PHASE1_RUNNING,      /* sequential 16-step measurement        */
    WHEQ_STATE_PHASE1_DEADTIME,     /* coast between Fase 1 steps            */
    WHEQ_STATE_PHASE1_DONE,         /* verdicts computed; Fase 2 optional    */
    WHEQ_STATE_PHASE2_RUNNING,      /* simultaneous 4-wheel measurement      */
    WHEQ_STATE_PHASE2_DONE,
    WHEQ_STATE_ABORTED,
} WheelEqState_t;

/* ---- Abort/reject reason (single, highest-priority cause) --------------- */
typedef enum {
    WHEQ_REASON_NONE = 0,
    WHEQ_REASON_OPERATOR,             /* explicit ABORT                     */
    WHEQ_REASON_ALREADY_ACTIVE,       /* Begin() while already running      */
    WHEQ_REASON_ENVELOPE_ABORT,       /* underlying Hito1 session went
                                        * inactive (any of its own reasons)  */
    WHEQ_REASON_TCS_ABS_INTERFERENCE, /* exceeded retry bound, sample never
                                        * validated                          */
    WHEQ_REASON_WATCHDOG,             /* stale Update() tick detected        */
    WHEQ_REASON_PHASE1_NOT_CLEAN,     /* Phase2 requested but Fase 1 had a
                                        * FAIL somewhere                     */
} WheelEqReason_t;

typedef enum {
    WHEQ_WHEEL_FL = 0,
    WHEQ_WHEEL_FR = 1,
    WHEQ_WHEEL_RL = 2,
    WHEQ_WHEEL_RR = 3,
} WheelEqWheel_t;

typedef enum {
    WHEQ_DIR_FORWARD = 0,
    WHEQ_DIR_REVERSE = 1,
} WheelEqDirection_t;

/* ---- Per-wheel equality verdict ----------------------------------------- */
typedef enum {
    WHEQ_WHEEL_VERDICT_PENDING = 0,        /* not yet measured               */
    WHEQ_WHEEL_VERDICT_PASS,
    WHEQ_WHEEL_VERDICT_WARN,
    WHEQ_WHEEL_VERDICT_FAIL,
    WHEQ_WHEEL_VERDICT_FAIL_ACKERMANN_OFFSET, /* precondition failure, not a
                                                 * speed/current deviation    */
} WheelEqWheelVerdict_t;

/* ---- Probable-cause text, derived from speed deviation x current -------- */
typedef enum {
    WHEQ_CAUSE_NONE = 0,           /* PASS — no significant deviation        */
    WHEQ_CAUSE_MECHANICAL,         /* gira MENOS, consume MÁS                */
    WHEQ_CAUSE_ELECTRICAL,         /* gira MENOS, consume MENOS              */
    WHEQ_CAUSE_SENSOR,             /* gira ~igual (corriente normal) pero
                                     * cuenta MENOS pulsos                    */
    WHEQ_CAUSE_OTHERS_BRAKED,      /* gira MÁS que la mediana                */
} WheelEqCause_t;

/* ---- Half-bridge (forward vs reverse) asymmetry verdict ----------------- */
typedef enum {
    WHEQ_HALFBRIDGE_PASS = 0,
    WHEQ_HALFBRIDGE_WARN,
    WHEQ_HALFBRIDGE_FAIL,
} WheelEqHalfBridgeVerdict_t;

/* ---- Driver (BTS7960) health verdict ------------------------------------ */
typedef enum {
    WHEQ_DRIVER_PENDING = 0,
    WHEQ_DRIVER_SANO,
    WHEQ_DRIVER_SOSPECHOSO,
    WHEQ_DRIVER_DEGRADADO,
} WheelEqDriverVerdict_t;

/* ---- Per-tick condition snapshot (all facts the FSM needs) --------------
 * The caller fills this from the live safety/sensor state every cycle; this
 * module never reads hardware itself. ------------------------------------- */
typedef struct {
    uint32_t now_ms;

    /* Hito 1 session gate — read every cycle. */
    bool     svc_armed;    /* ServiceDiagSession_State() == ARMED (Begin() gate) */
    bool     svc_active;   /* ServiceDiagSession_Active() (continuous run gate)  */

    /* Preconditions — read only by Begin(). */
    bool     steering_centered;       /* caller: |Steering_GetCurrentAngle()|
                                        * <= Traction_GetAckermannDeadbandDeg() */
    float    steering_angle_deg;      /* raw angle, HMI "CENTRAR VOLANTE" only */
    float    ackermann_diff[WHEQ_NUM_WHEELS]; /* Traction_GetAckermannDiff(i) */

    /* Always-on facts — read every Update(). */
    bool     abs_or_tcs_active;       /* ABS_IsActive() || TCS_IsActive()      */

    /* Live sensor samples — read every Update() while measuring. */
    float    battery_v;
    float    wheel_pulses_ps[WHEQ_NUM_WHEELS];
    float    wheel_current_a[WHEQ_NUM_WHEELS];

    /* DS18B20 thermal — read at Begin() (before) and at Fase 1 completion
     * (after); NaN-safe: temp_present false means "no sensor mapped".      */
    bool     wheel_temp_present[WHEQ_NUM_WHEELS];
    float    wheel_temp_c[WHEQ_NUM_WHEELS];
} WheelEqConds;

/* ---- Actuation the caller must apply THIS cycle -------------------------
 * wheel_mask bit i set => wheel i must be driven at pwm_pct/direction;
 * every other wheel must coast.  Zero mask => coast all four (idle,
 * dead-time, done, aborted, or blocked). ----------------------------------*/
typedef struct {
    uint8_t             wheel_mask;
    uint8_t             pwm_pct;
    WheelEqDirection_t  direction;
} WheelEqActuation_t;

/* ---- Fully-computed per-wheel result (valid once state has reached
 * PHASE1_DONE or later). --------------------------------------------------*/
typedef struct {
    float    pulses_ps_25;              /* forward-direction, for the HMI   */
    float    pulses_ps_50;
    float    normalized_speed;          /* pulses/s per (pwm% x battery_V),
                                          * averaged over all 4 Fase1 samples*/
    float    deviation_pct;             /* |value-median|/median x 100      */
    float    current_a;                 /* forward-direction @ 50 % (HMI)   */
    float    slope_a_per_pct;           /* (I50-I25)/(50-25), forward dir   */
    float    halfbridge_asym_pct;       /* fwd-vs-rev asymmetry @ 50 %      */
    float    delta_temp_c;              /* after - before, 0 if not present */
    bool     temp_present;

    WheelEqWheelVerdict_t      wheel_verdict;
    WheelEqCause_t             cause;
    WheelEqHalfBridgeVerdict_t halfbridge_verdict;
    WheelEqDriverVerdict_t     driver_verdict;
    uint8_t                    driver_reason_mask;  /* WHEQ_DRIVER_REASON_*  */
} WheelEqWheelResult;

/* ---- Session object (opaque-ish POD so tests can inspect it) ------------ */
typedef struct {
    WheelEqState_t   state;
    WheelEqReason_t  reason;

    uint8_t          step_index;         /* 0..15 during Fase 1             */
    uint32_t         step_start_ms;
    uint32_t         deadtime_start_ms;
    uint8_t          retry_count;

    /* Running accumulator for the CURRENT step's measurement window. */
    float            acc_pulses_ps;
    float            acc_current_a;
    float            acc_battery_v;
    uint32_t         acc_count;

    uint32_t         last_update_ms;
    bool             have_last_update;

    /* Precondition-block telemetry (HMI). */
    float            steering_angle_at_block_deg;
    uint8_t          ackermann_offending_mask;

    /* Raw Fase 1 samples: [wheel][level 0=25%,1=50%][dir 0=fwd,1=rev]. */
    float  pulses_ps[WHEQ_NUM_WHEELS][WHEQ_NUM_LEVELS][WHEQ_NUM_DIRS];
    float  current_a[WHEQ_NUM_WHEELS][WHEQ_NUM_LEVELS][WHEQ_NUM_DIRS];
    float  battery_v[WHEQ_NUM_WHEELS][WHEQ_NUM_LEVELS][WHEQ_NUM_DIRS];

    /* Raw Fase 2 samples (optional). */
    bool     phase2_ran;
    float    phase2_pulses_ps[WHEQ_NUM_WHEELS];
    float    phase2_current_a[WHEQ_NUM_WHEELS];

    /* Thermal before/after (captured at Begin() / Fase1 completion). */
    bool     temp_present[WHEQ_NUM_WHEELS];
    float    temp_before_c[WHEQ_NUM_WHEELS];
    float    temp_after_c[WHEQ_NUM_WHEELS];

    /* Computed results, valid once state >= PHASE1_DONE. */
    WheelEqWheelResult result[WHEQ_NUM_WHEELS];
} WheelEqTest;

/* ---- API ----------------------------------------------------------------*/
void WheelEqTest_Init(WheelEqTest *t);

/* Operator request to start Fase 1 (16-step sequential measurement).
 * Returns true if accepted; on rejection the state/reason record why
 * (BLOCKED_DEADBAND / BLOCKED_ACKERMANN / stays IDLE with a reason). */
bool WheelEqTest_Begin(WheelEqTest *t, const WheelEqConds *c);

/* Periodic tick — call EVERY cycle while state != IDLE/ABORTED/BLOCKED_*.
 * Enforces the always-on envelope-abort condition, the watchdog, the
 * discard/measure/dead-time sequencing, and (at Fase1 completion) computes
 * every verdict.  Returns the new state. */
WheelEqState_t WheelEqTest_Update(WheelEqTest *t, const WheelEqConds *c);

/* Operator request to start Fase 2 (optional, simultaneous). Only accepted
 * from PHASE1_DONE with zero FAIL of any kind (WheelEqTest_Phase1AllPass()).*/
bool WheelEqTest_BeginPhase2(WheelEqTest *t, const WheelEqConds *c);

/* Operator/host abort at any time. */
void WheelEqTest_Abort(WheelEqTest *t, WheelEqReason_t reason);

/* True once Fase 1 has completed with no FAIL/FAIL_ACKERMANN_OFFSET/
 * FAIL_HALFBRIDGE verdict anywhere — the sole Fase 2 eligibility gate. */
bool WheelEqTest_Phase1AllPass(const WheelEqTest *t);

/* The wheel_mask/pwm_pct/direction to actually apply THIS cycle. */
WheelEqActuation_t WheelEqTest_GetActuation(const WheelEqTest *t);

/* ---- Accessors ------------------------------------------------------- */
static inline WheelEqState_t WheelEqTest_State(const WheelEqTest *t)
{
    return t->state;
}
static inline WheelEqReason_t WheelEqTest_Reason(const WheelEqTest *t)
{
    return t->reason;
}
static inline bool WheelEqTest_Active(const WheelEqTest *t)
{
    return t->state != WHEQ_STATE_IDLE &&
           t->state != WHEQ_STATE_ABORTED &&
           t->state != WHEQ_STATE_BLOCKED_DEADBAND &&
           t->state != WHEQ_STATE_BLOCKED_ACKERMANN;
}
static inline const WheelEqWheelResult *WheelEqTest_Result(const WheelEqTest *t, uint8_t wheel)
{
    return &t->result[wheel & 0x03U];
}
static inline uint8_t WheelEqTest_StepIndex(const WheelEqTest *t)
{
    return t->step_index;
}
static inline bool WheelEqTest_Phase2Ran(const WheelEqTest *t)
{
    return t->phase2_ran;
}

/* Bitmask (bit i = wheel i, 0=FL..3=RR) of the wheel(s) THIS module is
 * actively commanding right now — 0 whenever WheelEqTest_Active() is false
 * (IDLE/ABORTED/BLOCKED_*) or the FSM is between steps (PHASE1_DEADTIME) or
 * awaiting an operator decision (PHASE1_DONE/PHASE2_DONE), matching
 * WheelEqTest_GetActuation()'s own "zero mask => coast all four" contract.
 *
 * Pure convenience so can_handler.c's SERVICE_DIAG grounded-wheel-pulse
 * guard (svcdiag_build_conds()) can OR this into its own single-channel
 * exemption without duplicating this module's actuation-mask logic — the
 * exemption always tracks exactly which wheel(s) are really being driven,
 * never a global permission, and automatically covers all four during
 * Fase 2 only while that phase is actually running. */
static inline uint8_t WheelEqTest_ActiveWheelMask(const WheelEqTest *t)
{
    if (t == NULL || !WheelEqTest_Active(t)) return 0U;
    return WheelEqTest_GetActuation(t).wheel_mask;
}

/* Human-readable label for a state / reason / verdict / cause (HMI). */
const char *WheelEqTest_StateText(WheelEqState_t st);
const char *WheelEqTest_ReasonText(WheelEqReason_t r);
const char *WheelEqTest_WheelVerdictText(WheelEqWheelVerdict_t v);
const char *WheelEqTest_CauseText(WheelEqCause_t c);
const char *WheelEqTest_HalfBridgeVerdictText(WheelEqHalfBridgeVerdict_t v);
const char *WheelEqTest_DriverVerdictText(WheelEqDriverVerdict_t v);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_EQUALITY_TEST_H */
