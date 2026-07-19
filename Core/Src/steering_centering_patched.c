/**
  ****************************************************************************
  * @file    steering_centering_patched.c
  * @brief   Effective steering-homing policy for the RS390 + 1:50 gearbox.
  *
  * The production centering FSM (steering_centering.c) remains the source of
  * truth for centre detection, encoder/range checks, ownership and isolation.
  * This narrow wrapper adapts the physical homing attempt to the installed
  * actuator without weakening any real EPS safety protection:
  *
  *   - a non-zero homing command is applied at 60 % PWM (2550 / 4249) so the
  *     RS390 and 1:50 reduction can overcome static friction and backlash;
  *   - each sweep receives a limited 300 ms gearbox take-up grace before the
  *     original 300 ms encoder-stall timer advances.  The maximum blind push
  *     against an undetected end stop is therefore 600 ms, not 1.2 s;
  *   - a faster end-stop detector combines no encoder motion with two fresh
  *     high-current CH5 samples.  It stops PWM immediately, waits 100 ms, then
  *     reverses LEFT->RIGHT; a second end stop on RIGHT ends the search safely;
  *   - flash is never treated as an absolute steering position after power-off.
  *     Every boot requires a fresh physical PB5 confirmation.  If PB5 is already
  *     over the centre metal, three stable samples complete without moving;
  *   - PB5 is accepted both as the normal debounced falling-edge event and as
  *     a stable active-LOW level for three 100 Hz cycles.  This prevents a unit
  *     already over the centre metal—or an edge missed during boot—from first
  *     sweeping away and travelling almost the whole steering range;
  *   - PB5 not seeing metal starts/continues the search and does NOT itself
  *     disconnect PC12.  PC12 stays authorised while CENTERING owns the motor;
  *   - completion transfers ownership to EPS immediately after the centering
  *     writer has neutralised the motor.  This closes the one-cycle
  *     STANDBY->ACTIVE hand-off window in which the relay supervisor could see
  *     the stale CENTERING owner and momentarily open PC12;
  *   - genuine encoder faults, range/total timeout, both-direction failure,
  *     CH5 faults and overcurrent still isolate the EPS exactly as before;
  *   - the diagnostic reports the effective requested PWM (2550), not the
  *     historical 425-count request inside the base FSM.
  *
  * A failed physical-centre search remains an ISOLABLE ASSIST fault:
  * PA6=PA7=0, PC4=LOW, PC12=OFF, owner=NONE, EPS mechanical-only, while the
  * vehicle remains ACTIVE with full traction.
  ****************************************************************************
  */

#include "steering_centering.h"
#include "steering_eps.h"
#include "steering_supervisor.h"
#include "sensor_manager.h"
#include "safety_system.h"
#include "motor_control.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* TIM3 is configured with ARR=4249.  60 % rounds to 2550 timer counts. */
#define PR434_HOMING_PWM_PERCENT            60U
#define PR434_HOMING_PWM_COUNTS             2550U
#define PR434_GEARBOX_TAKEUP_GRACE_MS       300U
#define PR434_CENTER_RAW_CONFIRM_CYCLES     3U

/* End-stop pressure detector.  It never relies on current alone: the encoder
 * must also remain stationary.  Half the proven 25 A active overcurrent ceiling
 * is high enough to reject normal no-load motion while detecting a hard stop
 * before the global overcurrent protection has to isolate the EPS. */
#define PR434_ENDSTOP_CURRENT_MA            (STEERING_ACTIVE_OVERCURRENT_MA / 2)
#define PR434_ENDSTOP_CONFIRM_SAMPLES       2U
#define PR434_ENDSTOP_NO_MOTION_MS          100U
#define PR434_ENDSTOP_SAMPLE_MAX_AGE_MS     250U
#define PR434_ENDSTOP_ENCODER_DELTA_COUNTS  2
#define PR434_REVERSE_DEADTIME_MS           100U

_Static_assert(PR434_HOMING_PWM_PERCENT == 60U,
               "RS390 homing profile must remain at the requested 60 percent");
_Static_assert(PR434_HOMING_PWM_COUNTS < 4250U,
               "Steering homing PWM must fit the TIM3 period");
_Static_assert(PR434_CENTER_RAW_CONFIRM_CYCLES >= 2U,
               "PB5 level fallback must reject a one-cycle EMI transient");
_Static_assert(PR434_ENDSTOP_CURRENT_MA > 0,
               "End-stop current threshold must be positive");
_Static_assert(PR434_ENDSTOP_CONFIRM_SAMPLES >= 2U,
               "A single CH5 sample must never declare an end stop");

/* Prototypes consumed by the included production FSM after macro routing. */
static uint32_t PR434_HomingTick(void);
static void PR434_HomingMotorSetPwm(uint16_t requested_pwm, bool reverse);
void PR434_SteeringCentering_Init_Base(void);
void PR434_SteeringCentering_Step_Base(void);
void PR434_SteeringCentering_UpdateDiag_Base(void);
void PR434_SteeringCentering_MarkRestoredFromFlash_Base(int32_t stored_center);

static bool PR434_SteeringHomingPowerReady(void)
{
    const CenteringState_t state = SteeringCentering_GetState();
    const bool sweeping = (state == CENTERING_SWEEP_LEFT) ||
                          (state == CENTERING_SWEEP_RIGHT);

    /* WAIT_RAIL intentionally remains not-ready.  Once the FSM enters an
     * active sweep, PC12 HIGH is the real local power evidence. */
    return sweeping &&
        (HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR) == GPIO_PIN_SET);
}

/* Route only this translation unit through the installed-actuator policy. */
#define Safety_IsPowerReady                         PR434_SteeringHomingPowerReady
#define Motor_SetPWM_Steering                       PR434_HomingMotorSetPwm
#define HAL_GetTick                                 PR434_HomingTick
#define SteeringCentering_Init                      PR434_SteeringCentering_Init_Base
#define SteeringCentering_Step                      PR434_SteeringCentering_Step_Base
#define SteeringCentering_UpdateDiag                PR434_SteeringCentering_UpdateDiag_Base
#define SteeringCentering_MarkRestoredFromFlash     PR434_SteeringCentering_MarkRestoredFromFlash_Base
#include "steering_centering.c"
#undef SteeringCentering_MarkRestoredFromFlash
#undef SteeringCentering_UpdateDiag
#undef SteeringCentering_Step
#undef SteeringCentering_Init
#undef HAL_GetTick
#undef Motor_SetPWM_Steering
#undef Safety_IsPowerReady

/* -------------------------------------------------------------------------
 * 60 % physical homing command
 * ------------------------------------------------------------------------- */
static void PR434_HomingMotorSetPwm(uint16_t requested_pwm, bool reverse)
{
    /* Preserve an explicit zero command.  Every non-zero centering request is
     * the controlled search command and is resolved to exactly 60 %. */
    const uint16_t applied_pwm =
        (requested_pwm == 0U) ? 0U : (uint16_t)PR434_HOMING_PWM_COUNTS;
    Motor_SetPWM_Steering(applied_pwm, reverse);
}

/* -------------------------------------------------------------------------
 * Gearbox take-up clock
 * -------------------------------------------------------------------------
 * The base FSM has a 300 ms no-encoder-change stall detector.  Freeze only the
 * centering FSM's private time for the first 300 real milliseconds of each
 * sweep, allowing backlash take-up without permitting a 1.2 s blind push.
 * Rail settle remains 50 real ms.  All other firmware uses the real HAL tick.
 */
typedef struct {
    bool             initialized;
    CenteringState_t observed_state;
    uint32_t         sweep_entry_real_ms;
    uint32_t         completed_grace_ms;
} PR434_HomingClock_t;

static PR434_HomingClock_t s_pr434_homing_clock;
static uint8_t s_pr434_center_raw_cycles;

static bool PR434_IsSweep(CenteringState_t state)
{
    return state == CENTERING_SWEEP_LEFT ||
           state == CENTERING_SWEEP_RIGHT;
}

static uint32_t PR434_MinU32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

static void PR434_ResetHomingClock(void)
{
    s_pr434_homing_clock.initialized = false;
    s_pr434_homing_clock.observed_state = CENTERING_IDLE;
    s_pr434_homing_clock.sweep_entry_real_ms = 0U;
    s_pr434_homing_clock.completed_grace_ms = 0U;
}

static uint32_t PR434_HomingTick(void)
{
    const uint32_t real_now = HAL_GetTick();
    const CenteringState_t state = centering_state;

    if (!s_pr434_homing_clock.initialized) {
        s_pr434_homing_clock.initialized = true;
        s_pr434_homing_clock.observed_state = state;
        s_pr434_homing_clock.sweep_entry_real_ms = real_now;
    } else if (state != s_pr434_homing_clock.observed_state) {
        /* Finalise the grace consumed by the sweep we just left. */
        if (PR434_IsSweep(s_pr434_homing_clock.observed_state)) {
            const uint32_t elapsed =
                real_now - s_pr434_homing_clock.sweep_entry_real_ms;
            s_pr434_homing_clock.completed_grace_ms +=
                PR434_MinU32(elapsed, PR434_GEARBOX_TAKEUP_GRACE_MS);
        }

        s_pr434_homing_clock.observed_state = state;
        s_pr434_homing_clock.sweep_entry_real_ms = real_now;
    }

    uint32_t active_grace_ms = 0U;
    if (PR434_IsSweep(state)) {
        const uint32_t elapsed =
            real_now - s_pr434_homing_clock.sweep_entry_real_ms;
        active_grace_ms =
            PR434_MinU32(elapsed, PR434_GEARBOX_TAKEUP_GRACE_MS);
    }

    return real_now - s_pr434_homing_clock.completed_grace_ms -
           active_grace_ms;
}

/* Stable level fallback for an active-LOW LJ12A3 center sensor.  The normal
 * EXTI/debounced event remains authoritative and is still checked inside the
 * base FSM.  This second path closes the boot race where PB5 was already LOW
 * before the falling-edge interrupt was armed or its flag was cleared. */
static bool PR434_CenterRawStable(void)
{
    const bool active = ServiceMode_IsEnabled(MODULE_STEER_CENTER) &&
        (HAL_GPIO_ReadPin(GPIOB, PIN_STEER_CENTER) == GPIO_PIN_RESET);

    if (!active) {
        s_pr434_center_raw_cycles = 0U;
        return false;
    }

    if (s_pr434_center_raw_cycles < PR434_CENTER_RAW_CONFIRM_CYCLES) {
        s_pr434_center_raw_cycles++;
    }
    return s_pr434_center_raw_cycles >= PR434_CENTER_RAW_CONFIRM_CYCLES;
}

/* -------------------------------------------------------------------------
 * End-stop pressure detector
 * ------------------------------------------------------------------------- */
typedef struct {
    bool             initialized;
    CenteringState_t observed_state;
    int32_t          motion_reference_count;
    uint32_t         last_motion_ms;
    uint32_t         last_sample_sequence;
    uint8_t          high_current_samples;
    bool             reverse_pending;
    uint32_t         reverse_started_ms;
} PR434_EndstopGuard_t;

static PR434_EndstopGuard_t s_pr434_endstop;

static uint32_t PR434_CurrentMagnitudeMa(int32_t current_ma)
{
    return (current_ma < 0)
        ? (uint32_t)(-(int64_t)current_ma)
        : (uint32_t)current_ma;
}

static int32_t PR434_AbsDeltaCounts(int32_t a, int32_t b)
{
    const int64_t d = (int64_t)a - (int64_t)b;
    return (int32_t)((d < 0) ? -d : d);
}

static void PR434_ResetEndstopObservation(CenteringState_t state)
{
    s_pr434_endstop.initialized = true;
    s_pr434_endstop.observed_state = state;
    s_pr434_endstop.motion_reference_count =
        (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    s_pr434_endstop.last_motion_ms = HAL_GetTick();
    s_pr434_endstop.last_sample_sequence = 0U;
    s_pr434_endstop.high_current_samples = 0U;
}

static void PR434_ResetEndstopGuard(void)
{
    s_pr434_endstop.initialized = false;
    s_pr434_endstop.observed_state = CENTERING_IDLE;
    s_pr434_endstop.motion_reference_count = 0;
    s_pr434_endstop.last_motion_ms = 0U;
    s_pr434_endstop.last_sample_sequence = 0U;
    s_pr434_endstop.high_current_samples = 0U;
    s_pr434_endstop.reverse_pending = false;
    s_pr434_endstop.reverse_started_ms = 0U;
}

static bool PR434_EndstopPressureDetected(void)
{
    const CenteringState_t state = centering_state;
    if (!PR434_IsSweep(state)) {
        s_pr434_endstop.initialized = false;
        s_pr434_endstop.high_current_samples = 0U;
        return false;
    }

    const uint32_t now = HAL_GetTick();
    const int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

    if (!s_pr434_endstop.initialized ||
        s_pr434_endstop.observed_state != state) {
        PR434_ResetEndstopObservation(state);
        return false;
    }

    if (PR434_AbsDeltaCounts(count,
            s_pr434_endstop.motion_reference_count) >=
            PR434_ENDSTOP_ENCODER_DELTA_COUNTS) {
        s_pr434_endstop.motion_reference_count = count;
        s_pr434_endstop.last_motion_ms = now;
        s_pr434_endstop.high_current_samples = 0U;
        return false;
    }

    const Ina226ChannelDiag *diag = Sensor_GetChannel5Diag();
    if (diag == NULL ||
        !diag->i2c_ack ||
        !diag->shunt_read_ok ||
        diag->sample_age_ms > PR434_ENDSTOP_SAMPLE_MAX_AGE_MS) {
        /* Missing or stale current evidence cannot prove an end stop.  The base
         * encoder stall/range/timeout protections remain as the fallback. */
        s_pr434_endstop.high_current_samples = 0U;
        return false;
    }

    if (diag->sample_sequence == s_pr434_endstop.last_sample_sequence) {
        return false;  /* Count each real 20 Hz acquisition only once. */
    }
    s_pr434_endstop.last_sample_sequence = diag->sample_sequence;

    const bool no_motion_long_enough =
        (uint32_t)(now - s_pr434_endstop.last_motion_ms) >=
        PR434_ENDSTOP_NO_MOTION_MS;
    const bool high_current =
        PR434_CurrentMagnitudeMa(diag->signed_current_ma) >=
        (uint32_t)PR434_ENDSTOP_CURRENT_MA;

    if (no_motion_long_enough && high_current) {
        if (s_pr434_endstop.high_current_samples < UINT8_MAX) {
            s_pr434_endstop.high_current_samples++;
        }
    } else {
        s_pr434_endstop.high_current_samples = 0U;
    }

    return s_pr434_endstop.high_current_samples >=
           PR434_ENDSTOP_CONFIRM_SAMPLES;
}

static bool PR434_ServiceReverseDeadtime(void)
{
    if (!s_pr434_endstop.reverse_pending) {
        return false;
    }

    /* Keep both PWM channels at zero throughout the direction-change deadtime. */
    PR434_HomingMotorSetPwm(0U, false);

    if ((uint32_t)(HAL_GetTick() - s_pr434_endstop.reverse_started_ms) <
        PR434_REVERSE_DEADTIME_MS) {
        return true;
    }

    const int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    sweep_origin_count = count;
    stall_prev_count = count;
    stall_last_change_tick = PR434_HomingTick();
    centering_state = CENTERING_SWEEP_RIGHT;
    s_pr434_endstop.reverse_pending = false;
    PR434_ResetEndstopObservation(CENTERING_SWEEP_RIGHT);
    PR434_HomingMotorSetPwm(CENTERING_PWM, false);
    return true;
}

static bool PR434_HandleEndstopPressure(void)
{
    if (!PR434_EndstopPressureDetected()) {
        return false;
    }

    /* Remove torque before changing state or direction. */
    PR434_HomingMotorSetPwm(0U, false);

    if (centering_state == CENTERING_SWEEP_LEFT) {
        s_pr434_endstop.reverse_pending = true;
        s_pr434_endstop.reverse_started_ms = HAL_GetTick();
        s_pr434_endstop.high_current_samples = 0U;
        return true;
    }

    /* The right-hand stop was also reached without PB5: centre cannot be found.
     * Use the existing isolable-assist abort path; traction remains available. */
    Centering_Abort(EPS_FAULT_CENTER_NOT_FOUND);
    return true;
}

/* The main loop decides the writer before stepping the FSM.  On the cycle that
 * finds PB5, that local decision is still CENTERING even though
 * Centering_Complete() has already stopped the motor and made the calibration
 * valid.  Transfer the stored authority immediately and explicitly through
 * NONE so the following cycle's relay supervisor sees EPS+calibrated and never
 * tears PC12 down during the STANDBY->ACTIVE transition. */
static void PR434_TransferCompletedHomingToEps(void)
{
    if (centering_state != CENTERING_DONE ||
        !Steering_IsCalibrated() ||
        Steering_IsAssistLatchedOff()) {
        return;
    }

    Steering_EpsSetOwner(STEER_OWNER_NONE);
    Steering_EpsSetOwner(STEER_OWNER_EPS);
}

/* Public init wrapper: reset every private adaptation on a new centering
 * session, then run the unchanged production init. */
void SteeringCentering_Init(void)
{
    PR434_ResetHomingClock();
    PR434_ResetEndstopGuard();
    s_pr434_center_raw_cycles = 0U;
    PR434_SteeringCentering_Init_Base();
}

/* Persistent flash is useful for integrity/history and the optional Z offset,
 * but an incremental encoder has no knowledge of movement while powered off.
 * Therefore a flash restore must NEVER mark the steering calibrated or skip the
 * physical PB5 search.  Keeping the FSM in IDLE makes the next Step either:
 *   - accept three stable PB5-active samples without moving, or
 *   - energise PC12 and perform the guarded LEFT->RIGHT physical search. */
void SteeringCentering_MarkRestoredFromFlash(int32_t stored_center)
{
    (void)stored_center;
    /* Deliberately do not call PR434_SteeringCentering_MarkRestoredFromFlash_Base. */
}

/* Public step wrapper: complete immediately after a stable PB5 active level,
 * even when no new falling edge can occur because the sensor was already over
 * metal at startup.  Otherwise apply end-stop protection and delegate the full
 * two-direction base FSM. */
void SteeringCentering_Step(void)
{
    if (centering_state != CENTERING_DONE &&
        centering_state != CENTERING_FAULT &&
        PR434_CenterRawStable()) {
        Centering_Complete();
        PR434_TransferCompletedHomingToEps();
        PR434_ResetEndstopGuard();
        s_pr434_center_raw_cycles = 0U;
        return;
    }

    if (PR434_ServiceReverseDeadtime()) {
        return;
    }

    if (PR434_HandleEndstopPressure()) {
        return;
    }

    const CenteringState_t before = centering_state;
    PR434_SteeringCentering_Step_Base();

    if (before != CENTERING_DONE && centering_state == CENTERING_DONE) {
        PR434_TransferCompletedHomingToEps();
    }

    if (centering_state == CENTERING_DONE ||
        centering_state == CENTERING_FAULT) {
        PR434_ResetEndstopGuard();
        s_pr434_center_raw_cycles = 0U;
    }
}

/* Public diagnostic wrapper: publish the command that reaches TIM3. */
void SteeringCentering_UpdateDiag(void)
{
    PR434_SteeringCentering_UpdateDiag_Base();

    if (PR434_IsSweep(centering_state)) {
        s_diag.pwm_requested = (uint16_t)PR434_HOMING_PWM_COUNTS;
    }
}
