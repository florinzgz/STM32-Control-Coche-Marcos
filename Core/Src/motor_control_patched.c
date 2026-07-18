/**
  ****************************************************************************
  * @file    motor_control_patched.c
  * @brief   Rear-wheel-drive 4x2 policy layered over motor_control.c.
  *
  * 4x2 policy:
  *   - RL/RR are the only driven wheels.
  *   - FL/FR remain in COAST while traction or dynamic braking is applied.
  *   - COAST releases all four wheels.
  *   - PARK / low-speed hold BRAKE remains symmetric on all four wheels.
  *   - 4x4 and tank-turn behaviour remain byte-for-byte in the production
  *     implementation included below.
  *
  * The production traction pipeline is still the single source for pedal
  * filtering, ramps, obstacle limiting, Ackermann, current/temperature limits,
  * ABS/TCS wheel scales and telemetry.  During 4x2 only, it is evaluated in a
  * shadow 4x4 pass so the rear-wheel safety inputs are used; no shadow write can
  * reach the real timer/GPIO registers.  The resolved rear commands are then
  * applied to RL/RR at full 4x2 authority, while FL/FR are explicitly coasted.
  ****************************************************************************
  */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Keep the complete productive implementation and rename only its public
 * update entry point.  All module-static state and helpers remain visible in
 * this translation unit to the rear-drive adapter below. */
#define Traction_Update Traction_Update_Base
#include "motor_control.c"
#undef Traction_Update

/* Single source of truth used by traction, wheel diagnostics, ABS and TCS. */
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
} RearDriveShadowHw_t;

static RearDriveShadowHw_t pr_shadow_hw[4];
static bool                 pr_shadow_ready = false;
static uint16_t             pr_rear_actual_pwm[2] = {0U, 0U};
static bool                 pr_rear_drive_active = false;

static void PR_InitShadowHardware(void)
{
    if (pr_shadow_ready) return;

    memset(pr_shadow_hw, 0, sizeof(pr_shadow_hw));
    for (uint8_t i = 0U; i < 4U; ++i) {
        pr_shadow_hw[i].rpwm_handle.Instance = &pr_shadow_hw[i].rpwm_regs;
        pr_shadow_hw[i].lpwm_handle.Instance = &pr_shadow_hw[i].lpwm_regs;
    }
    pr_shadow_ready = true;
}

static Motor_t PR_MakeShadowMotor(const Motor_t *real, uint8_t index)
{
    Motor_t shadow = *real;
    shadow.rpwm_timer = &pr_shadow_hw[index].rpwm_handle;
    shadow.lpwm_timer = &pr_shadow_hw[index].lpwm_handle;
    shadow.en_port    = &pr_shadow_hw[index].gpio_regs;
    shadow.direction  = 0;
    shadow.current_mode = MOTOR_MODE_COAST;
    return shadow;
}

static bool PR_IsDriveCapableState(SystemState_t state)
{
    return state == SYS_STATE_ACTIVE ||
           state == SYS_STATE_DEGRADED ||
           state == SYS_STATE_LIMP_HOME;
}

static bool PR_ShouldRunRearDrive4x2(void)
{
    if (traction_state.mode4x4 || traction_state.axisRotation) return false;
    if (current_gear != GEAR_FORWARD &&
        current_gear != GEAR_FORWARD_D2 &&
        current_gear != GEAR_REVERSE) return false;
    if (!PR_IsDriveCapableState(Safety_GetState())) return false;
    return Safety_IsPowerReady();
}

static uint16_t PR_RampRearPwm(uint16_t previous, uint16_t target)
{
    if (target > PWM_PERIOD) target = PWM_PERIOD;

    if (target > previous) {
        uint16_t step = (uint16_t)(target - previous);
        if (step > MAX_PWM_DELTA_PER_CYCLE) step = MAX_PWM_DELTA_PER_CYCLE;
        return (uint16_t)(previous + step);
    }

    uint16_t step = (uint16_t)(previous - target);
    if (step > MAX_PWM_DELTA_PER_CYCLE) step = MAX_PWM_DELTA_PER_CYCLE;
    return (uint16_t)(previous - step);
}

static void PR_CoastAllPhysical(void)
{
    Motor_SetMode(&motor_fl, MOTOR_MODE_COAST, 0);
    Motor_SetMode(&motor_fr, MOTOR_MODE_COAST, 0);
    Motor_SetMode(&motor_rl, MOTOR_MODE_COAST, 0);
    Motor_SetMode(&motor_rr, MOTOR_MODE_COAST, 0);
}

static void PR_BrakeAllPhysical(void)
{
    Motor_SetMode(&motor_fl, MOTOR_MODE_BRAKE, 0);
    Motor_SetMode(&motor_fr, MOTOR_MODE_BRAKE, 0);
    Motor_SetMode(&motor_rl, MOTOR_MODE_BRAKE, 0);
    Motor_SetMode(&motor_rr, MOTOR_MODE_BRAKE, 0);
}

static void PR_ZeroTractionTelemetry(void)
{
    for (uint8_t i = 0U; i < 4U; ++i) {
        traction_state.wheels[i].pwm = 0U;
        traction_state.wheels[i].reverse = false;
    }
    pr_rear_actual_pwm[0] = 0U;
    pr_rear_actual_pwm[1] = 0U;
    Traction_UpdateMotionInhibit(motion_effective_demand, 0U);
}

void Traction_Update(void)
{
    if (!PR_ShouldRunRearDrive4x2()) {
        pr_rear_drive_active = false;
        pr_rear_actual_pwm[0] = 0U;
        pr_rear_actual_pwm[1] = 0U;
        Traction_Update_Base();
        return;
    }

    PR_InitShadowHardware();

    /* Preserve the actual rear duty on entry from 4x4 so the dedicated rear
     * ramp starts from the physically applied value rather than from zero. */
    if (!pr_rear_drive_active) {
        pr_rear_actual_pwm[0] = traction_state.wheels[MOTOR_RL].pwm;
        pr_rear_actual_pwm[1] = traction_state.wheels[MOTOR_RR].pwm;
        pr_rear_drive_active = true;
    }

    Motor_t real_fl = motor_fl;
    Motor_t real_fr = motor_fr;
    Motor_t real_rl = motor_rl;
    Motor_t real_rr = motor_rr;

    /* Replace every physical endpoint with RAM-backed timer/GPIO registers.
     * This guarantees the virtual 4x4 calculation cannot produce even a brief
     * pulse on the non-driven front axle. */
    motor_fl = PR_MakeShadowMotor(&real_fl, MOTOR_FL);
    motor_fr = PR_MakeShadowMotor(&real_fr, MOTOR_FR);
    motor_rl = PR_MakeShadowMotor(&real_rl, MOTOR_RL);
    motor_rr = PR_MakeShadowMotor(&real_rr, MOTOR_RR);

    traction_state.mode4x4 = true;
    Traction_Update_Base();
    traction_state.mode4x4 = false;

    motor_fl = real_fl;
    motor_fr = real_fr;
    motor_rl = real_rl;
    motor_rr = real_rr;

    /* A real safety transition that occurred during the calculation always
     * wins.  Do not apply a cached drive command after SAFE/ERROR/power loss. */
    if (!PR_IsDriveCapableState(Safety_GetState()) || !Safety_IsPowerReady()) {
        PR_CoastAllPhysical();
        PR_ZeroTractionTelemetry();
        return;
    }

    const uint16_t virtual_rl = traction_state.wheels[MOTOR_RL].pwm;
    const uint16_t virtual_rr = traction_state.wheels[MOTOR_RR].pwm;
    const bool reverse_rl = traction_state.wheels[MOTOR_RL].reverse;
    const bool reverse_rr = traction_state.wheels[MOTOR_RR].reverse;

    /* The base 4x4 path divides the axle command by two.  Reconstruct full 4x2
     * authority on the rear axle, then apply the original per-cycle jerk limit
     * once more so the physical rear output never changes abruptly. */
    uint32_t target_rl32 = (uint32_t)virtual_rl * 2U;
    uint32_t target_rr32 = (uint32_t)virtual_rr * 2U;
    if (target_rl32 > PWM_PERIOD) target_rl32 = PWM_PERIOD;
    if (target_rr32 > PWM_PERIOD) target_rr32 = PWM_PERIOD;

    if (trac_phase == TRAC_PHASE_BRAKE &&
        virtual_rl == 0U && virtual_rr == 0U) {
        /* Intentional low-speed hold / PARK-like brake remains symmetric. */
        PR_BrakeAllPhysical();
        PR_ZeroTractionTelemetry();
        return;
    }

    if (trac_phase == TRAC_PHASE_COAST &&
        virtual_rl == 0U && virtual_rr == 0U) {
        /* Four-wheel free roll: the non-driven axle is never left braked. */
        PR_CoastAllPhysical();
        PR_ZeroTractionTelemetry();
        return;
    }

    pr_rear_actual_pwm[0] = PR_RampRearPwm(pr_rear_actual_pwm[0],
                                            (uint16_t)target_rl32);
    pr_rear_actual_pwm[1] = PR_RampRearPwm(pr_rear_actual_pwm[1],
                                            (uint16_t)target_rr32);

    /* Front axle is always Hi-Z in 4x2 DRIVE and dynamic braking. */
    Motor_SetMode(&motor_fl, MOTOR_MODE_COAST, 0);
    Motor_SetMode(&motor_fr, MOTOR_MODE_COAST, 0);

    if (pr_rear_actual_pwm[0] == 0U) {
        Motor_SetMode(&motor_rl, MOTOR_MODE_COAST, 0);
    } else {
        int16_t signed_pwm = reverse_rl ? -(int16_t)pr_rear_actual_pwm[0]
                                        :  (int16_t)pr_rear_actual_pwm[0];
        Motor_SetMode(&motor_rl, MOTOR_MODE_DRIVE, signed_pwm);
    }

    if (pr_rear_actual_pwm[1] == 0U) {
        Motor_SetMode(&motor_rr, MOTOR_MODE_COAST, 0);
    } else {
        int16_t signed_pwm = reverse_rr ? -(int16_t)pr_rear_actual_pwm[1]
                                        :  (int16_t)pr_rear_actual_pwm[1];
        Motor_SetMode(&motor_rr, MOTOR_MODE_DRIVE, signed_pwm);
    }

    traction_state.wheels[MOTOR_FL].pwm = 0U;
    traction_state.wheels[MOTOR_FR].pwm = 0U;
    traction_state.wheels[MOTOR_FL].reverse = false;
    traction_state.wheels[MOTOR_FR].reverse = false;
    traction_state.wheels[MOTOR_RL].pwm = pr_rear_actual_pwm[0];
    traction_state.wheels[MOTOR_RR].pwm = pr_rear_actual_pwm[1];
    traction_state.wheels[MOTOR_RL].reverse = reverse_rl;
    traction_state.wheels[MOTOR_RR].reverse = reverse_rr;

    uint16_t final_max = pr_rear_actual_pwm[0] > pr_rear_actual_pwm[1]
                       ? pr_rear_actual_pwm[0] : pr_rear_actual_pwm[1];
    Traction_UpdateMotionInhibit(motion_effective_demand, final_max);
}
