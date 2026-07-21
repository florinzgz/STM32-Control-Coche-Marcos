/**
  ****************************************************************************
  * @file    motor_control_patched.c
  * @brief   Physical drivetrain policy adapter.
  *
  * The base controller remains the sole source for demand filtering, ramps,
  * safety limits, ABS/TCS, Ackermann and DRIVE/BRAKE/COAST decisions.  It is
  * evaluated against RAM-backed shadow timer/GPIO registers; only the resolved
  * final decision is then written to the real BTS7960 outputs.
  *
  * Installed policies:
  *   - 4x2 drives RL/RR and keeps FL/FR in coast.
  *   - 4x4 applies all four resolved outputs.
  *   - while the steering angle is inside the Ackermann deadband and no
  *     per-wheel safety scaling is active, all four 4x4 outputs are explicitly
  *     forced to identical PWM and direction.  A FR+RL diagonal can therefore
  *     no longer originate from the software command path.
  *   - tank turn keeps the installed rear-polarity correction and momentary
  *     pedal-release behaviour.
  *   - the encoder health monitor is rebaselined exactly once when physical
  *     PB5 homing enters DONE, so TIM2 being intentionally zeroed is not
  *     misclassified as an impossible encoder jump.
  ****************************************************************************
  */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "tank_turn_policy.h"
#include "steering_centering.h"
#include "traction_output_policy.h"

#define Traction_Update          Traction_Update_Base
#define Traction_SetAxisRotation Traction_SetAxisRotation_Base
#define Encoder_CheckHealth      Encoder_CheckHealth_Base
#include "motor_control.c"
#undef Encoder_CheckHealth
#undef Traction_SetAxisRotation
#undef Traction_Update

/* The base health monitor intentionally latches real faults, but its previous
 * count lives outside the centering module.  Centering_Complete() deliberately
 * writes TIM2=0 at PB5.  Without this one-shot state-aware rebaseline, the next
 * health cycle compares zero with the pre-zero center count and can latch a
 * false ENC_MAX_JUMP fault, immediately isolating EPS/PC12 despite a healthy
 * encoder.  No existing fault is cleared here. */
static CenteringState_t s_encoder_health_centering_state = CENTERING_IDLE;

void Encoder_CheckHealth(void)
{
    const CenteringState_t state = SteeringCentering_GetState();

    if (state == CENTERING_DONE &&
        s_encoder_health_centering_state != CENTERING_DONE &&
        !Encoder_HasFault()) {
        enc_prev_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        enc_last_change_tick = HAL_GetTick();
        s_encoder_health_centering_state = state;
        return;
    }

    s_encoder_health_centering_state = state;
    Encoder_CheckHealth_Base();
}

bool Traction_IsWheelDriven(uint8_t wheel)
{
    if (wheel >= 4U) return false;
    if (traction_state.mode4x4 || traction_state.axisRotation) return true;
    return wheel == MOTOR_RL || wheel == MOTOR_RR;
}

typedef struct {
    TIM_TypeDef       rpwm_regs;
    TIM_TypeDef       lpwm_regs;
    TIM_HandleTypeDef rpwm_handle;
    TIM_HandleTypeDef lpwm_handle;
    GPIO_TypeDef      gpio_regs;
} ShadowMotorHw;

static ShadowMotorHw s_shadow[4];
static bool s_shadow_ready;
static TankTurnReleaseState_t s_tank_release = { false, false };

static void init_shadow(void)
{
    if (s_shadow_ready) return;
    memset(s_shadow, 0, sizeof(s_shadow));
    for (uint8_t i = 0U; i < 4U; ++i) {
        s_shadow[i].rpwm_handle.Instance = &s_shadow[i].rpwm_regs;
        s_shadow[i].lpwm_handle.Instance = &s_shadow[i].lpwm_regs;
    }
    s_shadow_ready = true;
}

static Motor_t make_shadow(const Motor_t *real, uint8_t index)
{
    Motor_t m = *real;
    m.rpwm_timer = &s_shadow[index].rpwm_handle;
    m.lpwm_timer = &s_shadow[index].lpwm_handle;
    m.en_port = &s_shadow[index].gpio_regs;
    m.direction = 0;
    m.current_mode = MOTOR_MODE_COAST;
    return m;
}

/* Read the duty actually resolved into the shadow timer registers.
 * Unlike the legacy wheel telemetry, this remains truthful during the
 * bounded Neutral ramp and therefore becomes the source routed to the
 * real BTS7960 outputs. */
static uint16_t shadow_resolved_pwm(const Motor_t *motor)
{
    const uint32_t rpwm = __HAL_TIM_GET_COMPARE(motor->rpwm_timer,
                                                motor->rpwm_channel);
    const uint32_t lpwm = __HAL_TIM_GET_COMPARE(motor->lpwm_timer,
                                                motor->lpwm_channel);
    const uint32_t resolved = (rpwm > lpwm) ? rpwm : lpwm;
    return TractionOutput_ClampPwm(resolved, PWM_PERIOD);
}

static bool drive_state(SystemState_t state)
{
    return state == SYS_STATE_ACTIVE ||
           state == SYS_STATE_DEGRADED ||
           state == SYS_STATE_LIMP_HOME;
}

static void coast_all(void)
{
    Motor_SetMode(&motor_fl, MOTOR_MODE_COAST, 0);
    Motor_SetMode(&motor_fr, MOTOR_MODE_COAST, 0);
    Motor_SetMode(&motor_rl, MOTOR_MODE_COAST, 0);
    Motor_SetMode(&motor_rr, MOTOR_MODE_COAST, 0);
}

static void zero_output_telemetry(void)
{
    for (uint8_t i = 0U; i < 4U; ++i) {
        traction_state.wheels[i].pwm = 0U;
        traction_state.wheels[i].reverse = false;
    }
    Traction_UpdateMotionInhibit(motion_effective_demand, 0U);
}

static void apply_motor(Motor_t *motor, uint8_t wheel, motor_mode_t mode,
                        uint16_t pwm, int8_t direction)
{
    if (mode == MOTOR_MODE_BRAKE) {
        Motor_SetMode(motor, MOTOR_MODE_BRAKE, 0);
        traction_state.wheels[wheel].pwm = 0U;
        traction_state.wheels[wheel].reverse = false;
        return;
    }
    if (mode == MOTOR_MODE_COAST || pwm == 0U) {
        Motor_SetMode(motor, MOTOR_MODE_COAST, 0);
        traction_state.wheels[wheel].pwm = 0U;
        traction_state.wheels[wheel].reverse = false;
        return;
    }

    if (pwm > PWM_PERIOD) pwm = PWM_PERIOD;
    int16_t signed_pwm = direction < 0 ? -(int16_t)pwm : (int16_t)pwm;
    Motor_SetMode(motor, MOTOR_MODE_DRIVE, signed_pwm);
    traction_state.wheels[wheel].pwm = pwm;
    traction_state.wheels[wheel].reverse = direction < 0;
}

void Traction_SetAxisRotation(bool enable)
{
    if (!enable) {
        Traction_SetAxisRotation_Base(false);
        TankTurnRelease_Reset(&s_tank_release);
        return;
    }

    if (s_tank_release.release_latched) return;
    if (!traction_state.axisRotation) s_tank_release.pedal_seen = false;
    Traction_SetAxisRotation_Base(true);
}

static bool auto_release_tank(void)
{
    if (!traction_state.axisRotation) {
        s_tank_release.pedal_seen = false;
        return false;
    }

    float pedal = sanitize_float(Pedal_GetPercent(), 0.0f);
    if (!TankTurnRelease_Update(&s_tank_release, true, pedal)) return false;

    Traction_SetAxisRotation_Base(false);
    pedal_ema = 0.0f;
    pedal_ramped = 0.0f;
    traction_state.demandPct = 0.0f;
    prev_demand_pct = 0.0f;
    dynbrake_pct = 0.0f;
    brake_release_pct = 0.0f;
    creep_smooth_pct = 0.0f;
    creep_smooth_init = 0U;
    creep_demand_active = false;
    creep_demand_start_tick = 0U;
    neutral_ramp_pct = 0.0f;
    neutral_ramp_active = 0U;
    for (uint8_t i = 0U; i < 4U; ++i) prev_output_pwm[i] = 0U;
    trac_phase = TRAC_PHASE_COAST;
    coast_all();
    zero_output_telemetry();
    return true;
}

static bool straight_line_unlimited(void)
{
    if (fabsf(Steering_GetCurrentAngle()) >= ACKERMANN_DEADBAND_DEG) {
        return false;
    }
    for (uint8_t i = 0U; i < 4U; ++i) {
        if (safety_status.wheel_scale[i] < (1.0f - 1.0e-6f)) return false;
    }
    return true;
}

void Traction_Update(void)
{
    if (auto_release_tank()) return;

    init_shadow();

    Motor_t real[4] = { motor_fl, motor_fr, motor_rl, motor_rr };
    motor_fl = make_shadow(&real[MOTOR_FL], MOTOR_FL);
    motor_fr = make_shadow(&real[MOTOR_FR], MOTOR_FR);
    motor_rl = make_shadow(&real[MOTOR_RL], MOTOR_RL);
    motor_rr = make_shadow(&real[MOTOR_RR], MOTOR_RR);

    Traction_Update_Base();

    const motor_mode_t mode[4] = {
        motor_fl.current_mode, motor_fr.current_mode,
        motor_rl.current_mode, motor_rr.current_mode
    };
    const int8_t direction[4] = {
        motor_fl.direction, motor_fr.direction,
        motor_rl.direction, motor_rr.direction
    };
        /* Capture the effective shadow hardware, not the legacy telemetry.
     * This preserves Neutral's bounded ramp and any per-cycle jerk limit. */
    uint16_t pwm[4] = {
        shadow_resolved_pwm(&motor_fl),
        shadow_resolved_pwm(&motor_fr),
        shadow_resolved_pwm(&motor_rl),
        shadow_resolved_pwm(&motor_rr)
    };

    motor_fl = real[MOTOR_FL];
    motor_fr = real[MOTOR_FR];
    motor_rl = real[MOTOR_RL];
    motor_rr = real[MOTOR_RR];

    if (!drive_state(Safety_GetState()) || !Safety_IsPowerReady()) {
        coast_all();
        zero_output_telemetry();
        return;
    }

    if (traction_state.axisRotation) {
        const int8_t hw_dir[4] = {
            TankTurn_ResolveHardwareDirection(MOTOR_FL, direction[MOTOR_FL]),
            TankTurn_ResolveHardwareDirection(MOTOR_FR, direction[MOTOR_FR]),
            TankTurn_ResolveHardwareDirection(MOTOR_RL, direction[MOTOR_RL]),
            TankTurn_ResolveHardwareDirection(MOTOR_RR, direction[MOTOR_RR])
        };
        apply_motor(&motor_fl, MOTOR_FL, mode[MOTOR_FL], pwm[MOTOR_FL], hw_dir[MOTOR_FL]);
        apply_motor(&motor_fr, MOTOR_FR, mode[MOTOR_FR], pwm[MOTOR_FR], hw_dir[MOTOR_FR]);
        apply_motor(&motor_rl, MOTOR_RL, mode[MOTOR_RL], pwm[MOTOR_RL], hw_dir[MOTOR_RL]);
        apply_motor(&motor_rr, MOTOR_RR, mode[MOTOR_RR], pwm[MOTOR_RR], hw_dir[MOTOR_RR]);
    } else if (traction_state.mode4x4) {
        /* Ackermann already returns unity below 2 degrees.  Reinforce that
         * contract at the physical output layer: when no wheel limiter is active
         * and every resolved motor is in DRIVE, all four channels get exactly
         * the same duty and direction. */
        const bool all_drive = mode[0] == MOTOR_MODE_DRIVE &&
                               mode[1] == MOTOR_MODE_DRIVE &&
                               mode[2] == MOTOR_MODE_DRIVE &&
                               mode[3] == MOTOR_MODE_DRIVE;
        const bool same_direction =
            TractionOutput_SameNonzeroDirection(direction);
        if (all_drive && same_direction && straight_line_unlimited()) {
            /* Use the MINIMUM resolved duty: symmetry may reduce stronger
             * channels, but can never raise a wheel over its jerk/safety cap. */
            const uint16_t equal_pwm = TractionOutput_MinPwm(pwm);
            const int8_t equal_dir = direction[MOTOR_FL] < 0 ? -1 : 1;
            apply_motor(&motor_fl, MOTOR_FL, MOTOR_MODE_DRIVE, equal_pwm, equal_dir);
            apply_motor(&motor_fr, MOTOR_FR, MOTOR_MODE_DRIVE, equal_pwm, equal_dir);
            apply_motor(&motor_rl, MOTOR_RL, MOTOR_MODE_DRIVE, equal_pwm, equal_dir);
            apply_motor(&motor_rr, MOTOR_RR, MOTOR_MODE_DRIVE, equal_pwm, equal_dir);
        } else {
            apply_motor(&motor_fl, MOTOR_FL, mode[MOTOR_FL], pwm[MOTOR_FL], direction[MOTOR_FL]);
            apply_motor(&motor_fr, MOTOR_FR, mode[MOTOR_FR], pwm[MOTOR_FR], direction[MOTOR_FR]);
            apply_motor(&motor_rl, MOTOR_RL, mode[MOTOR_RL], pwm[MOTOR_RL], direction[MOTOR_RL]);
            apply_motor(&motor_rr, MOTOR_RR, mode[MOTOR_RR], pwm[MOTOR_RR], direction[MOTOR_RR]);
        }
    } else {
        /* Base 4x2 computes a complete left/right command on FL/FR.  Route that
         * already-ramped and safety-limited result to the installed rear axle. */
        if (mode[MOTOR_FL] == MOTOR_MODE_BRAKE &&
            mode[MOTOR_FR] == MOTOR_MODE_BRAKE) {
            /* Logical FL/FR are the installed rear axle in 4x2.  Brake
             * RL/RR only and keep the non-driven front axle in coast. */
            Motor_SetMode(&motor_fl, MOTOR_MODE_COAST, 0);
            Motor_SetMode(&motor_fr, MOTOR_MODE_COAST, 0);
            Motor_SetMode(&motor_rl, MOTOR_MODE_BRAKE, 0);
            Motor_SetMode(&motor_rr, MOTOR_MODE_BRAKE, 0);
            zero_output_telemetry();
            return;
        }
        if (mode[MOTOR_FL] == MOTOR_MODE_COAST &&
            mode[MOTOR_FR] == MOTOR_MODE_COAST) {
            coast_all();
            zero_output_telemetry();
            return;
        }

        Motor_SetMode(&motor_fl, MOTOR_MODE_COAST, 0);
        Motor_SetMode(&motor_fr, MOTOR_MODE_COAST, 0);
        traction_state.wheels[MOTOR_FL].pwm = 0U;
        traction_state.wheels[MOTOR_FR].pwm = 0U;
        traction_state.wheels[MOTOR_FL].reverse = false;
        traction_state.wheels[MOTOR_FR].reverse = false;

        /* The base 4x2 decision is logical-left/logical-right.  Apply
         * the PHYSICAL rear-wheel safety caps again at the final routing
         * boundary so RL/RR temperature/TCS/ABS cutoffs can never be
         * bypassed by the FL/FR shadow calculation. */
        const uint16_t rear_left_pwm = TractionOutput_CapPwmByScale(
            pwm[MOTOR_FL], safety_status.wheel_scale[MOTOR_RL], PWM_PERIOD);
        const uint16_t rear_right_pwm = TractionOutput_CapPwmByScale(
            pwm[MOTOR_FR], safety_status.wheel_scale[MOTOR_RR], PWM_PERIOD);
        apply_motor(&motor_rl, MOTOR_RL, mode[MOTOR_FL], rear_left_pwm,
                    direction[MOTOR_FL]);
        apply_motor(&motor_rr, MOTOR_RR, mode[MOTOR_FR], rear_right_pwm,
                    direction[MOTOR_FR]);
    }

    uint16_t final_max = 0U;
    for (uint8_t i = 0U; i < 4U; ++i) {
        if (traction_state.wheels[i].pwm > final_max) {
            final_max = traction_state.wheels[i].pwm;
        }
    }
    Traction_UpdateMotionInhibit(motion_effective_demand, final_max);
}
