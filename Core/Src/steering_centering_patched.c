/**
  ****************************************************************************
  * @file    steering_centering_patched.c
  * @brief   Effective steering-homing policy for the installed RS390 gearbox.
  *
  * The productive FSM remains in steering_centering.c.  This adapter provides:
  *   - a full-authority search command (100 % PWM) to overcome the installed
  *     gearbox static friction;
  *   - a fast encoder+CH5 end-stop guard;
  *   - mandatory zero-PWM dead-time before LEFT->RIGHT reversal;
  *   - stable active-low PB5 level acceptance, including boot over the target;
  *   - a fresh physical PB5 search on every power cycle;
  *   - immediate CENTERING->NONE->EPS owner transfer at completion.
  *
  * Full PWM does not mean sustained force.  A fresh high-current sample or a
  * no-motion/current combination removes torque first.  The normal encoder
  * stall/range/timeout protections remain active as independent fallbacks.
  ****************************************************************************
  */

#include "steering_centering.h"
#include "steering_eps.h"
#include "steering_supervisor.h"
#include "sensor_manager.h"
#include "safety_system.h"
#include "service_mode.h"
#include "motor_control.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <limits.h>

/* TIM3 ARR=4249. */
#define HOMING_PWM_COUNTS                4249U
#define CENTER_RAW_CONFIRM_CYCLES        3U
#define REVERSE_DEADTIME_MS              100U
#define ENDSTOP_NO_MOTION_MS             100U
#define ENDSTOP_SAMPLE_MAX_AGE_MS        250U
#define ENDSTOP_ENCODER_DELTA_COUNTS     2
/* A hard-current sample cuts immediately.  The lower threshold additionally
 * requires no encoder movement, rejecting normal running current. */
#define ENDSTOP_HARD_CURRENT_MA          20000U
#define ENDSTOP_STALL_CURRENT_MA          8000U

_Static_assert(HOMING_PWM_COUNTS == 4249U,
               "homing full authority must match TIM3 period");
_Static_assert(CENTER_RAW_CONFIRM_CYCLES >= 2U,
               "PB5 level fallback must debounce EMI");

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

static void Homing_SetPwm(uint16_t requested_pwm, bool reverse);
void Homing_BaseInit(void);
void Homing_BaseStep(void);
void Homing_BaseUpdateDiag(void);
void Homing_BaseMarkRestored(int32_t stored_center);

static bool HomingPowerReady(void)
{
    CenteringState_t state = SteeringCentering_GetState();
    bool sweeping = state == CENTERING_SWEEP_LEFT ||
                    state == CENTERING_SWEEP_RIGHT;
    return sweeping &&
        HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR) == GPIO_PIN_SET;
}

#define Safety_IsPowerReady                     HomingPowerReady
#define Motor_SetPWM_Steering                   Homing_SetPwm
#define SteeringCentering_Init                  Homing_BaseInit
#define SteeringCentering_Step                  Homing_BaseStep
#define SteeringCentering_UpdateDiag            Homing_BaseUpdateDiag
#define SteeringCentering_MarkRestoredFromFlash Homing_BaseMarkRestored
#include "steering_centering.c"
#undef SteeringCentering_MarkRestoredFromFlash
#undef SteeringCentering_UpdateDiag
#undef SteeringCentering_Step
#undef SteeringCentering_Init
#undef Motor_SetPWM_Steering
#undef Safety_IsPowerReady

static void Homing_SetPwm(uint16_t requested_pwm, bool reverse)
{
    const uint16_t applied = requested_pwm == 0U ? 0U : HOMING_PWM_COUNTS;
    Motor_SetPWM_Steering(applied, reverse);
}

static uint8_t s_center_raw_cycles;

static bool center_raw_stable(void)
{
    const bool active = ServiceMode_IsEnabled(MODULE_STEER_CENTER) &&
        HAL_GPIO_ReadPin(GPIOB, PIN_STEER_CENTER) == GPIO_PIN_RESET;

    if (!active) {
        s_center_raw_cycles = 0U;
        return false;
    }

    if (s_center_raw_cycles < CENTER_RAW_CONFIRM_CYCLES) {
        s_center_raw_cycles++;
    }
    return s_center_raw_cycles >= CENTER_RAW_CONFIRM_CYCLES;
}

static bool is_sweep(CenteringState_t state)
{
    return state == CENTERING_SWEEP_LEFT || state == CENTERING_SWEEP_RIGHT;
}

static uint32_t current_magnitude_ma(int32_t current_ma)
{
    return current_ma < 0
        ? (uint32_t)(-(int64_t)current_ma)
        : (uint32_t)current_ma;
}

static int32_t abs_count_delta(int32_t a, int32_t b)
{
    int64_t d = (int64_t)a - (int64_t)b;
    if (d < 0) d = -d;
    if (d > INT32_MAX) return INT32_MAX;
    return (int32_t)d;
}

typedef struct {
    bool initialized;
    CenteringState_t state;
    int32_t reference_count;
    uint32_t last_motion_ms;
    uint32_t last_sample_sequence;
    bool reverse_pending;
    uint32_t reverse_started_ms;
} HomingGuard;

static HomingGuard s_guard;

static void reset_observation(CenteringState_t state)
{
    s_guard.initialized = true;
    s_guard.state = state;
    s_guard.reference_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    s_guard.last_motion_ms = HAL_GetTick();
    s_guard.last_sample_sequence = 0U;
}

static void reset_guard(void)
{
    s_guard.initialized = false;
    s_guard.state = CENTERING_IDLE;
    s_guard.reference_count = 0;
    s_guard.last_motion_ms = 0U;
    s_guard.last_sample_sequence = 0U;
    s_guard.reverse_pending = false;
    s_guard.reverse_started_ms = 0U;
}

static bool endstop_pressure_detected(void)
{
    const CenteringState_t state = centering_state;
    if (!is_sweep(state)) {
        s_guard.initialized = false;
        return false;
    }

    const uint32_t now = HAL_GetTick();
    const int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

    if (!s_guard.initialized || s_guard.state != state) {
        reset_observation(state);
        return false;
    }

    if (abs_count_delta(count, s_guard.reference_count) >=
        ENDSTOP_ENCODER_DELTA_COUNTS) {
        s_guard.reference_count = count;
        s_guard.last_motion_ms = now;
    }

    const Ina226ChannelDiag *diag = Sensor_GetChannel5Diag();
    if (diag == NULL || !diag->i2c_ack || !diag->shunt_read_ok ||
        diag->sample_age_ms > ENDSTOP_SAMPLE_MAX_AGE_MS ||
        diag->sample_sequence == s_guard.last_sample_sequence) {
        return false;
    }

    s_guard.last_sample_sequence = diag->sample_sequence;
    const uint32_t current = current_magnitude_ma(diag->signed_current_ma);
    const bool no_motion =
        (uint32_t)(now - s_guard.last_motion_ms) >= ENDSTOP_NO_MOTION_MS;

    return current >= ENDSTOP_HARD_CURRENT_MA ||
           (no_motion && current >= ENDSTOP_STALL_CURRENT_MA);
}

static void begin_right_reverse(void)
{
    /* Torque must disappear before the direction changes. */
    Homing_SetPwm(0U, false);
    s_guard.reverse_pending = true;
    s_guard.reverse_started_ms = HAL_GetTick();
}

static bool service_reverse_deadtime(void)
{
    if (!s_guard.reverse_pending) return false;

    Homing_SetPwm(0U, false);
    if ((uint32_t)(HAL_GetTick() - s_guard.reverse_started_ms) <
        REVERSE_DEADTIME_MS) {
        return true;
    }

    const int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    sweep_origin_count = count;
    stall_prev_count = count;
    stall_last_change_tick = HAL_GetTick();
    centering_state = CENTERING_SWEEP_RIGHT;
    s_guard.reverse_pending = false;
    reset_observation(CENTERING_SWEEP_RIGHT);
    Homing_SetPwm(CENTERING_PWM, false);
    return true;
}

static bool handle_endstop_pressure(void)
{
    if (!endstop_pressure_detected()) return false;

    Homing_SetPwm(0U, false);
    if (centering_state == CENTERING_SWEEP_LEFT) {
        begin_right_reverse();
    } else {
        Centering_Abort(EPS_FAULT_CENTER_NOT_FOUND);
    }
    return true;
}

static void transfer_completed_homing_to_eps(void)
{
    if (centering_state != CENTERING_DONE ||
        !Steering_IsCalibrated() || Steering_IsAssistLatchedOff()) {
        return;
    }

    Steering_EpsSetOwner(STEER_OWNER_NONE);
    Steering_EpsSetOwner(STEER_OWNER_EPS);
}

void SteeringCentering_Init(void)
{
    reset_guard();
    s_center_raw_cycles = 0U;
    Homing_BaseInit();
}

void SteeringCentering_MarkRestoredFromFlash(int32_t stored_center)
{
    /* An incremental encoder cannot observe movement while powered off.
     * Stored data remains diagnostic only; every boot must confirm PB5. */
    (void)stored_center;
}

void SteeringCentering_Step(void)
{
    if (centering_state != CENTERING_DONE &&
        centering_state != CENTERING_FAULT && center_raw_stable()) {
        Centering_Complete();
        transfer_completed_homing_to_eps();
        reset_guard();
        s_center_raw_cycles = 0U;
        return;
    }

    if (service_reverse_deadtime()) return;
    if (handle_endstop_pressure()) return;

    const CenteringState_t before = centering_state;
    Homing_BaseStep();

    /* The base encoder-stall fallback also detects the left stop.  It normally
     * writes right PWM immediately; override that transition with the same
     * zero-output dead-time used by the CH5 fast path. */
    if (before == CENTERING_SWEEP_LEFT &&
        centering_state == CENTERING_SWEEP_RIGHT) {
        begin_right_reverse();
        return;
    }

    if (before != CENTERING_DONE && centering_state == CENTERING_DONE) {
        transfer_completed_homing_to_eps();
    }

    if (centering_state == CENTERING_DONE ||
        centering_state == CENTERING_FAULT) {
        reset_guard();
        s_center_raw_cycles = 0U;
    }
}

void SteeringCentering_UpdateDiag(void)
{
    Homing_BaseUpdateDiag();
    if (is_sweep(centering_state)) {
        s_diag.pwm_requested = HOMING_PWM_COUNTS;
    }
}
