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
#include "drive_dynamics_policy.h"
#include "eps_assist_policy.h"
#include "can_handler.h"

#ifdef HOST_TEST
extern bool CAN_PedalCalServiceActive(void) __attribute__((weak));
static bool PR_MotorPedalCalServiceActive(void)
{
    return CAN_PedalCalServiceActive != 0 &&
           CAN_PedalCalServiceActive();
}
#else
static bool PR_MotorPedalCalServiceActive(void)
{
    return CAN_PedalCalServiceActive();
}
#endif

#define Traction_Update          Traction_Update_Base
#define Traction_SetAxisRotation Traction_SetAxisRotation_Base
#define Encoder_CheckHealth      Encoder_CheckHealth_Base
#define Steering_ControlLoop     Steering_ControlLoop_Base
#include "motor_control.c"
#undef Steering_ControlLoop
#undef Encoder_CheckHealth
#undef Traction_SetAxisRotation
#undef Traction_Update

_Static_assert((uint8_t)MOTOR_MODE_COAST == TRACTION_OUTPUT_MODE_COAST,
               "traction policy COAST mode must match motor_mode_t");
_Static_assert((uint8_t)MOTOR_MODE_DRIVE == TRACTION_OUTPUT_MODE_DRIVE,
               "traction policy DRIVE mode must match motor_mode_t");
_Static_assert((uint8_t)MOTOR_MODE_BRAKE == TRACTION_OUTPUT_MODE_BRAKE,
               "traction policy BRAKE mode must match motor_mode_t");

/* The base health monitor intentionally latches real faults, but its previous
 * count lives outside the centering module.  Centering_Complete() deliberately
 * writes TIM2=0 at PB5.  Without this one-shot state-aware rebaseline, the next
 * health cycle compares zero with the pre-zero center count and can latch a
 * false ENC_MAX_JUMP fault, immediately isolating EPS/PC12 despite a healthy
 * encoder.  No existing fault is cleared here. */
static CenteringState_t s_encoder_health_centering_state = CENTERING_IDLE;
static uint8_t s_encoder_frozen_confirm_cycles;
static EpsAssistState_t s_eps_assist_state;
static uint8_t s_eps_raw_reversal_cycles;

#define PR_ENC_FROZEN_TIMEOUT_MS       500U
#define PR_ENC_MOTOR_ACTIVE_PCT        20.0f
#define PR_ENC_FROZEN_CONFIRM_CYCLES   2U

void Encoder_CheckHealth(void)
{
    const CenteringState_t state = SteeringCentering_GetState();

    if (state == CENTERING_DONE &&
        s_encoder_health_centering_state != CENTERING_DONE &&
        !Encoder_HasFault()) {
        enc_prev_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        enc_last_change_tick = HAL_GetTick();
        s_encoder_frozen_confirm_cycles = 0U;
        s_encoder_health_centering_state = state;
        return;
    }

    s_encoder_health_centering_state = state;
    if (enc_fault) return;

    const int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    const uint32_t now = HAL_GetTick();

    if (count > ENC_MAX_COUNTS || count < -ENC_MAX_COUNTS) {
        enc_fault = 1;
        return;
    }

    int32_t delta = count - enc_prev_count;
    if (delta < 0) delta = -delta;
    if (delta > ENC_MAX_JUMP) {
        enc_fault = 1;
        return;
    }

    if (count != enc_prev_count) {
        enc_last_change_tick = now;
        s_encoder_frozen_confirm_cycles = 0U;
    } else {
        const float motor_pct = fabsf(eps_motor_effort);
        if (motor_pct <= PR_ENC_MOTOR_ACTIVE_PCT) {
            /* Start the frozen window only after meaningful effort begins. */
            enc_last_change_tick = now;
            s_encoder_frozen_confirm_cycles = 0U;
        } else if ((now - enc_last_change_tick) > PR_ENC_FROZEN_TIMEOUT_MS) {
            if (s_encoder_frozen_confirm_cycles < UINT8_MAX) {
                ++s_encoder_frozen_confirm_cycles;
            }
            if (s_encoder_frozen_confirm_cycles >=
                PR_ENC_FROZEN_CONFIRM_CYCLES) {
                enc_fault = 1;
                return;
            }
        }
    }

    enc_prev_count = count;
}

/* Normal COAST is not a safety reset.  Keep the angle/velocity observer alive
 * across encoder quantisation gaps so slow manual steering remains continuous.
 * Full Steering_Neutralize() is reserved for disabled, unpowered and faulted
 * states where clearing estimator history is intentional. */
static void Steering_CoastPreserveEstimator(void)
{
    Motor_SetSigned(&motor_steer, 0);
    eps_prev_pwm_raw = 0;
    eps_motor_effort = 0.0f;
}

static void Steering_FullNeutralize(void)
{
    EpsAssist_Reset(&s_eps_assist_state);
    s_eps_raw_reversal_cycles = 0U;
    Steering_Neutralize();
}

/* EPS intent must be derived from the real road-wheel angle.  Applying the
 * mechanical deadband before differentiating erased the driver's initial
 * movement and generated a false velocity spike at the deadband edge. */
void Steering_ControlLoop(void)
{
    if (Steering_IsAssistLatchedOff()) {
        Steering_FullNeutralize();
        return;
    }
    if (!steering_calibrated) {
        Steering_FullNeutralize();
        return;
    }

    /* The Makefile compiles this wrapper, not motor_control.c directly.  Keep
     * the effective EPS writer electrically gated by the completed relay
     * sequence and by the commanded state of the steering power relay. */
    if (!Safety_IsPowerReady() ||
        HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR) != GPIO_PIN_SET) {
        Steering_FullNeutralize();
        return;
    }

    if (enc_fault) {
        Steering_FullNeutralize();
        return;
    }

    {
        const SystemState_t st = Safety_GetState();
        if (st == SYS_STATE_SAFE || st == SYS_STATE_ERROR) {
            Steering_FullNeutralize();
            return;
        }
    }

    const eps_params_t *p = EPS_Params_Get();
    static uint32_t last_time;
    const uint32_t now = HAL_GetTick();

    if (last_time == 0U || (now - last_time) > 500U) {
        last_time = now;
        eps_prev_angle_deg = sanitize_float(
            Steering_GetCurrentAngle() - p->center_offset_deg, 0.0f);
        Steering_FullNeutralize();
        return;
    }

    const float dt = (float)(now - last_time) / 1000.0f;
    if (dt < 0.001f) return;
    last_time = now;

    const float theta = sanitize_float(Steering_GetCurrentAngle(), 0.0f);
    const float theta_real = theta - p->center_offset_deg;

    float omega_raw = (theta_real - eps_prev_angle_deg) / dt;
    omega_raw = sanitize_float(omega_raw, 0.0f);
    eps_prev_angle_deg = theta_real;
    eps_omega_filt += 0.3f * (omega_raw - eps_omega_filt);
    const bool raw_reversal_confirmed = EpsAssist_UpdateRawReversal(
        &s_eps_raw_reversal_cycles, omega_raw, eps_omega_filt);

    float eff_theta = theta_real;
    if (fabsf(eff_theta) < p->deadband_deg) eff_theta = 0.0f;

    float v_kmh = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                   Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) * 0.25f;
    v_kmh = sanitize_float(v_kmh, 0.0f);
    if (v_kmh < 0.0f) v_kmh = 0.0f;

    const float abs_omega = fabsf(eps_omega_filt);
    const float lambda = DriveDynamics_EpsLambda(abs_omega);
    const float g_v = 1.0f / (1.0f + v_kmh / p->assist_vs_speed);
    const float h_v = 0.3f + v_kmh / p->return_vs_speed;

    float fric_sign = 0.0f;
    if (abs_omega < 2.0f && fabsf(eff_theta) > 1.0f) {
        fric_sign = (eff_theta > 0.0f) ? -1.0f : 1.0f;
    }

    const float assist_tau =
        lambda * p->assist_strength * g_v * eps_omega_filt;
    const float damped_assist_tau = EpsAssist_ApplyDamping(
        assist_tau, p->damping, eps_omega_filt);
    const float return_tau =
        -(1.0f - lambda) * p->center_strength * h_v * eff_theta
        - p->damping * eps_omega_filt
        + p->friction_comp * fric_sign;

    const int8_t previous_intent_direction = s_eps_assist_state.direction;
    EpsAssistDecision_t assist = EpsAssist_Resolve(
        &s_eps_assist_state, now, eps_omega_filt, v_kmh,
        damped_assist_tau, return_tau, p->coast_band_pct);
    const bool intent_direction_changed = EpsAssist_DirectionChanged(
        previous_intent_direction, &s_eps_assist_state,
        assist.driver_intent);

    /* Never slew through zero while still applying the previous direction.
     * A confirmed raw/EMA disagreement is also treated as direction
     * uncertainty.  Both paths insert physical COAST, reset PWM history,
     * and preserve the angle/velocity estimator. */
    if (intent_direction_changed || raw_reversal_confirmed) {
        Steering_CoastPreserveEstimator();
        return;
    }

    float tau = assist.pwm_pct;

    if (v_kmh > EPS_HS_FADE_START_KMH) {
        const float hs_range = EPS_HS_FADE_END_KMH - EPS_HS_FADE_START_KMH;
        float hs_fade = 1.0f - (1.0f - EPS_HS_FADE_MIN_FACTOR) *
                        (v_kmh - EPS_HS_FADE_START_KMH) / hs_range;
        if (hs_fade < EPS_HS_FADE_MIN_FACTOR) {
            hs_fade = EPS_HS_FADE_MIN_FACTOR;
        }
        tau *= hs_fade;
    }

    if (Safety_IsDegraded()) tau *= Safety_GetSteeringLimitFactor();
    tau = sanitize_float(tau, 0.0f);

    float pwm_pct = tau;
    if (pwm_pct > p->max_pwm_pct) pwm_pct = p->max_pwm_pct;
    if (pwm_pct < -p->max_pwm_pct) pwm_pct = -p->max_pwm_pct;

    /* Resolve COAST before minimum-drive compensation.  A tiny control
     * residue must not be promoted to the configured breakaway PWM and keep
     * the BTS7960 fighting the driver near centre. */
    if (assist.coast) {
        Steering_CoastPreserveEstimator();
        return;
    }

    const float effective_coast_band = EpsAssist_EffectiveCoastBand(
        p->coast_band_pct, assist.driver_intent);
    const float effective_min_drive = EpsAssist_EffectiveMinDrive(
        p->min_drive_pct, assist.driver_intent);
    const EpsOutputDecision_t output = EpsOutput_Resolve(
        pwm_pct, effective_coast_band, effective_min_drive);
    if (output.coast) {
        Steering_CoastPreserveEstimator();
        return;
    }
    pwm_pct = output.pwm_pct;

    int16_t pwm_raw = (int16_t)(pwm_pct * (float)PWM_PERIOD / 100.0f);
    const float effective_slew_rate = EpsAssist_EffectiveSlewRate(
        p->slew_rate_pct, assist.driver_intent);
    int16_t slew_counts =
        (int16_t)(effective_slew_rate * (float)PWM_PERIOD / 100.0f);
    if (slew_counts < 1) slew_counts = 1;

    const int16_t delta_pwm = pwm_raw - eps_prev_pwm_raw;
    if (delta_pwm > slew_counts) pwm_raw = eps_prev_pwm_raw + slew_counts;
    if (delta_pwm < -slew_counts) pwm_raw = eps_prev_pwm_raw - slew_counts;
    eps_prev_pwm_raw = pwm_raw;

    const uint16_t pwm_abs =
        (uint16_t)((pwm_raw >= 0) ? pwm_raw : -pwm_raw);
    eps_motor_effort = (float)pwm_abs * 100.0f / (float)PWM_PERIOD;
    Motor_SetSigned(&motor_steer, pwm_raw);
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

static void apply_plan(const TractionOutputPlan *plan)
{
    if (plan == NULL) {
        coast_all();
        zero_output_telemetry();
        return;
    }

    Motor_t *const motors[4] = { &motor_fl, &motor_fr, &motor_rl, &motor_rr };
    for (uint8_t i = 0U; i < 4U; ++i) {
        apply_motor(motors[i], i, (motor_mode_t)plan->mode[i],
                    plan->pwm[i], plan->direction[i]);
    }
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

void Traction_Update(void)
{
    /* Pedal-calibration service lock is an independent,
     * highest-priority physical motion inhibit.  Do not evaluate ramps,
     * tank turn, Ackermann, ABS/TCS or base demand while it owns the vehicle. */
    if (PR_MotorPedalCalServiceActive()) {
        (void)Traction_CalibrationLock();
        coast_all();
        zero_output_telemetry();
        Traction_UpdateMotionInhibit(0.0f, 0U);
        return;
    }

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
    const uint8_t policy_mode[4] = {
        (uint8_t)mode[MOTOR_FL], (uint8_t)mode[MOTOR_FR],
        (uint8_t)mode[MOTOR_RL], (uint8_t)mode[MOTOR_RR]
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

    /* The base dynamic brake was tuned at 0.5 with a 60 % ceiling.  Preserve
     * its direction/state decisions but reduce the final opposing torque to
     * the field-safe 0.2-equivalent curve, capped at 30 %. */
    if (dynbrake_pct > DYNBRAKE_ACTIVE_THRESHOLD) {
        const float original_dynbrake = dynbrake_pct;
        const float limited_dynbrake =
            DriveDynamics_DynbrakeLimitedPct(original_dynbrake);
        const float scale = (original_dynbrake > 0.0f)
                          ? limited_dynbrake / original_dynbrake : 0.0f;
        dynbrake_pct = limited_dynbrake;
        if (motion_effective_demand < 0.0f) motion_effective_demand *= scale;
        for (uint8_t i = 0U; i < 4U; ++i) {
            pwm[i] = (uint16_t)((float)pwm[i] * scale);
        }
    }

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
        TractionOutputPlan plan;
        TractionOutput_Resolve4x4(policy_mode, direction, pwm,
                                  Steering_GetCurrentAngle(),
                                  (float)ACKERMANN_DEADBAND_DEG,
                                  safety_status.wheel_scale, &plan);
        apply_plan(&plan);
    } else {
        TractionOutputPlan plan;
        if (!TractionOutput_Resolve4x2Rear(policy_mode, direction, pwm,
                                           Steering_GetCurrentAngle(),
                                           (float)ACKERMANN_DEADBAND_DEG,
                                           safety_status.wheel_scale,
                                           PWM_PERIOD, &plan)) {
            coast_all();
            zero_output_telemetry();
            return;
        }
        apply_plan(&plan);
    }

    uint16_t final_max = 0U;
    for (uint8_t i = 0U; i < 4U; ++i) {
        if (traction_state.wheels[i].pwm > final_max) {
            final_max = traction_state.wheels[i].pwm;
        }
    }
    Traction_UpdateMotionInhibit(motion_effective_demand, final_max);
}
