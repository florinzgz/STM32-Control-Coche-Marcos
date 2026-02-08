/**
  ****************************************************************************
  * @file           : motor_control.c
  * @brief          : Motor control implementation with PWM, PID, and Ackermann
  ****************************************************************************
  */

#include "motor_control.h"
#include "ackermann.h"
#include "main.h"
#include "sensor_manager.h"
#include <math.h>

/* Constants */
#define PWM_PERIOD     8499
#define PWM_FREQUENCY  20000

/* Motor structures */
typedef struct {
    TIM_HandleTypeDef *timer;
    uint32_t channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef *en_port;
    uint16_t en_pin;
    int16_t power;
} Motor_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float setpoint;
    float output;
} PID_t;

/* Global variables */
static Motor_t motor_fl, motor_fr, motor_rl, motor_rr, motor_steer;
/* steering_motor.cpp: kp = 1.2 in degree-space.
 * STM32 PID operates in encoder-count-space (4800 CPR).
 * Equivalent kp: 1.2 / (4800/360) = 0.09.
 * Base firmware uses P-only control (no I or D terms). */
static PID_t steering_pid = {0.09f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static TractionState_t traction_state = {0};
static float ackermann_wheelbase = WHEELBASE_M;
static float ackermann_track     = TRACK_WIDTH_M;
static float ackermann_max_inner = MAX_STEER_DEG;
static uint8_t steering_calibrated = 0;

/* Ackermann-computed individual wheel angle setpoints (degrees).
 * Updated every time Steering_SetAngle() is called.               */
static float steer_fl_deg = 0.0f;
static float steer_fr_deg = 0.0f;

/* ---- Encoder fault detection state ----
 * The E6B2-CWZ6C encoder Z-index pulse (PB4/EXTI4) is intentionally NOT used:
 *   1. No EXTI4 hardware initialisation exists in MX_GPIO_Init().
 *   2. Steering uses relative positioning zeroed at Steering_Init(); an
 *      absolute index reference would require a known mechanical alignment
 *      that is not guaranteed by the chassis design.
 *   3. Fault detection is achieved through range, jump and frozen-value
 *      checks on the A/B quadrature channels, which are sufficient for
 *      safety without the Z pulse.
 */

/* Steering deadband in encoder counts (steering_motor.cpp: kDeadbandDeg = 0.5f)
 * 0.5° × 4800 counts/360° ≈ 6.67 counts */
#define STEERING_DEADBAND_COUNTS  (0.5f * (float)ENCODER_CPR / 360.0f)
#define ENC_MAX_COUNTS       ((int16_t)((STEERING_WHEEL_MAX_DEG + 20.0f) * (float)ENCODER_CPR / 360.0f))
        /* ±370° of steering wheel travel (350° + 20° margin) → ±4933
         * counts.  Any reading beyond this is mechanically impossible.   */
#define ENC_MAX_JUMP         100
        /* Maximum plausible count change per 10 ms control cycle.
         * At 200 °/s steering rate: 200/360*4800*0.01 ≈ 27 counts.
         * 100 counts/cycle ≈ 750 °/s — well beyond any physical rate.   */
#define ENC_FROZEN_TIMEOUT_MS 200
        /* If the motor is driving above ENC_MOTOR_ACTIVE_PCT and the
         * encoder has not changed for this long, declare frozen fault.   */
#define ENC_MOTOR_ACTIVE_PCT  10.0f
        /* Minimum |PID output| (%) to consider the motor actively
         * driving.  Below this the motor may legitimately be at rest.    */

static int16_t  enc_prev_count       = 0;
static uint32_t enc_last_change_tick = 0;
static uint8_t  enc_fault            = 0;   /* 0 = healthy, 1 = faulted */

extern TIM_HandleTypeDef htim1, htim2, htim8;

/* Private function prototypes */
static void Motor_SetPWM(Motor_t *motor, uint16_t pwm);
static void Motor_SetDirection(Motor_t *motor, int8_t direction);
static void Motor_Enable(Motor_t *motor, uint8_t enable);
static float PID_Compute(PID_t *pid, float measured, float dt);

/* ==================================================================
 *  Initialization
 * ================================================================== */

void Motor_Init(void)
{
    motor_fl.timer = &htim1;  motor_fl.channel = TIM_CHANNEL_1;
    motor_fl.dir_port = GPIOC; motor_fl.dir_pin = PIN_DIR_FL;
    motor_fl.en_port  = GPIOC; motor_fl.en_pin  = PIN_EN_FL;

    motor_fr.timer = &htim1;  motor_fr.channel = TIM_CHANNEL_2;
    motor_fr.dir_port = GPIOC; motor_fr.dir_pin = PIN_DIR_FR;
    motor_fr.en_port  = GPIOC; motor_fr.en_pin  = PIN_EN_FR;

    motor_rl.timer = &htim1;  motor_rl.channel = TIM_CHANNEL_3;
    motor_rl.dir_port = GPIOC; motor_rl.dir_pin = PIN_DIR_RL;
    motor_rl.en_port  = GPIOC; motor_rl.en_pin  = PIN_EN_RL;

    motor_rr.timer = &htim1;  motor_rr.channel = TIM_CHANNEL_4;
    motor_rr.dir_port = GPIOC; motor_rr.dir_pin = PIN_DIR_RR;
    motor_rr.en_port  = GPIOC; motor_rr.en_pin  = PIN_EN_RR;

    motor_steer.timer = &htim8;  motor_steer.channel = TIM_CHANNEL_3;
    motor_steer.dir_port = GPIOC; motor_steer.dir_pin = PIN_DIR_STEER;
    motor_steer.en_port  = GPIOC; motor_steer.en_pin  = PIN_EN_STEER;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

void Traction_Init(void)
{
    traction_state.mode4x4      = false;
    traction_state.demandPct    = 0.0f;
    traction_state.axisRotation = false;
    for (uint8_t i = 0; i < 4; i++) {
        traction_state.wheels[i].demandPct = 0.0f;
        traction_state.wheels[i].pwm       = 0;
        traction_state.wheels[i].reverse   = false;
    }
}

void Steering_Init(void)
{
    steering_pid.integral   = 0.0f;
    steering_pid.prev_error = 0.0f;
    steering_pid.setpoint   = 0.0f;
    steering_pid.output     = 0.0f;
    __HAL_TIM_SET_COUNTER(&htim2, 0);  /* Zero encoder at current position */
    steering_calibrated = 1;

    /* Initialise encoder health tracking */
    enc_prev_count       = 0;
    enc_last_change_tick = HAL_GetTick();
    enc_fault            = 0;
}

/* ==================================================================
 *  Traction Control
 * ================================================================== */

void Traction_SetDemand(float throttlePct)
{
    if (throttlePct < -100.0f) throttlePct = -100.0f;
    if (throttlePct >  100.0f) throttlePct =  100.0f;
    traction_state.demandPct = throttlePct;
}

void Traction_SetMode4x4(bool enable)
{
    traction_state.mode4x4 = enable;
}

void Traction_SetAxisRotation(bool enable)
{
    traction_state.axisRotation = enable;
}

void Traction_Update(void)
{
    float demand = traction_state.demandPct;
    uint16_t pwm = (uint16_t)(fabs(demand) * PWM_PERIOD / 100.0f);
    int8_t dir   = (demand >= 0) ? 1 : -1;

    if (traction_state.axisRotation) {
        /* Tank turn: left wheels reverse, right wheels forward (or vice versa) */
        Motor_SetPWM(&motor_fl, pwm); Motor_SetDirection(&motor_fl, (int8_t)-dir);
        Motor_SetPWM(&motor_fr, pwm); Motor_SetDirection(&motor_fr, dir);
        Motor_SetPWM(&motor_rl, pwm); Motor_SetDirection(&motor_rl, (int8_t)-dir);
        Motor_SetPWM(&motor_rr, pwm); Motor_SetDirection(&motor_rr, dir);
    } else if (traction_state.mode4x4) {
        /* 4x4: All wheels same demand */
        Motor_SetPWM(&motor_fl, pwm); Motor_SetDirection(&motor_fl, dir);
        Motor_SetPWM(&motor_fr, pwm); Motor_SetDirection(&motor_fr, dir);
        Motor_SetPWM(&motor_rl, pwm); Motor_SetDirection(&motor_rl, dir);
        Motor_SetPWM(&motor_rr, pwm); Motor_SetDirection(&motor_rr, dir);
    } else {
        /* 4x2: Front wheels only */
        Motor_SetPWM(&motor_fl, pwm); Motor_SetDirection(&motor_fl, dir);
        Motor_SetPWM(&motor_fr, pwm); Motor_SetDirection(&motor_fr, dir);
        Motor_SetPWM(&motor_rl, 0);   Motor_Enable(&motor_rl, 0);
        Motor_SetPWM(&motor_rr, 0);   Motor_Enable(&motor_rr, 0);
    }

    /* Enable/disable motors */
    uint8_t active = (fabs(demand) > 0.5f) ? 1 : 0;
    Motor_Enable(&motor_fl, active);
    Motor_Enable(&motor_fr, active);
    if (traction_state.mode4x4 || traction_state.axisRotation) {
        Motor_Enable(&motor_rl, active);
        Motor_Enable(&motor_rr, active);
    }

    /* Update state with sensor readings */
    traction_state.wheels[0].speedKmh = Wheel_GetSpeed_FL();
    traction_state.wheels[1].speedKmh = Wheel_GetSpeed_FR();
    traction_state.wheels[2].speedKmh = Wheel_GetSpeed_RL();
    traction_state.wheels[3].speedKmh = Wheel_GetSpeed_RR();
    for (uint8_t i = 0; i < 4; i++) {
        traction_state.wheels[i].currentA = Current_GetAmps(i);
        traction_state.wheels[i].tempC    = Temperature_Get(i);
        traction_state.wheels[i].pwm      = pwm;
        traction_state.wheels[i].reverse  = (dir < 0);
    }
}

void Traction_EmergencyStop(void)
{
    Motor_Enable(&motor_fl, 0);
    Motor_Enable(&motor_fr, 0);
    Motor_Enable(&motor_rl, 0);
    Motor_Enable(&motor_rr, 0);
    Motor_Enable(&motor_steer, 0);
    Motor_SetPWM(&motor_fl, 0);
    Motor_SetPWM(&motor_fr, 0);
    Motor_SetPWM(&motor_rl, 0);
    Motor_SetPWM(&motor_rr, 0);
    Motor_SetPWM(&motor_steer, 0);
    traction_state.demandPct = 0.0f;
}

const TractionState_t* Traction_GetState(void)
{
    return &traction_state;
}

/* ==================================================================
 *  Steering Control (PID + Encoder)
 * ================================================================== */

void Steering_SetAngle(float angle_deg)
{
    if (angle_deg < -MAX_STEER_DEG) angle_deg = -MAX_STEER_DEG;
    if (angle_deg >  MAX_STEER_DEG) angle_deg =  MAX_STEER_DEG;

    /* --- Ackermann integration ---
     * After clamping the road angle, compute individual front-left and
     * front-right wheel angles using the pure Ackermann module.
     * steer_fl_deg / steer_fr_deg are the per-wheel setpoints.
     *
     * The single steering motor PID targets the road angle because the
     * encoder measures the steering column position; the mechanical
     * linkage translates it into the Ackermann FL/FR geometry.         */
    Ackermann_ComputeWheelAngles(angle_deg, &steer_fl_deg, &steer_fr_deg);

    /* Convert road angle to encoder counts for steering motor PID */
    steering_pid.setpoint = angle_deg * (float)ENCODER_CPR / 360.0f;
}

void Steering_ControlLoop(void)
{
    /* If the encoder is faulted, do not run PID — we have no reliable
     * position feedback.  Neutralise the motor to prevent uncontrolled
     * steering.  The safety system handles the state transition.       */
    if (enc_fault) {
        Steering_Neutralize();
        return;
    }

    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();

    /* On first call (or if last_time was never set), seed the timestamp
     * and skip the PID iteration to avoid a huge dt spike.              */
    if (last_time == 0) {
        last_time = now;
        return;
    }

    float dt = (float)(now - last_time) / 1000.0f;
    if (dt < 0.001f) return;

    int16_t encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    float measured = (float)encoder_count;

    /* steering_motor.cpp: kDeadbandDeg = 0.5f */
    float error = steering_pid.setpoint - measured;
    float absError = fabsf(error);
    if (absError < STEERING_DEADBAND_COUNTS) {
        Motor_SetPWM(&motor_steer, 0);
        Motor_Enable(&motor_steer, 0);
        last_time = now;
        return;
    }

    float output   = PID_Compute(&steering_pid, measured, dt);

    if (output < -100.0f) output = -100.0f;
    if (output >  100.0f) output =  100.0f;

    uint16_t pwm = (uint16_t)(fabs(output) * PWM_PERIOD / 100.0f);
    int8_t direction = (output >= 0) ? 1 : -1;

    Motor_SetPWM(&motor_steer, pwm);
    Motor_SetDirection(&motor_steer, direction);
    Motor_Enable(&motor_steer, (fabs(output) > 1.0f));
    last_time = now;
}

float Steering_GetCurrentAngle(void)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    return (float)cnt * 360.0f / (float)ENCODER_CPR;
}

bool Steering_IsCalibrated(void)
{
    return (steering_calibrated != 0);
}

void Steering_GetWheelAngles(float *out_fl_deg, float *out_fr_deg)
{
    if (out_fl_deg) *out_fl_deg = steer_fl_deg;
    if (out_fr_deg) *out_fr_deg = steer_fr_deg;
}

/* ==================================================================
 *  Encoder Health Monitoring
 *
 *  Detects three classes of encoder fault:
 *    1. Out-of-range  – counter exceeds mechanical travel (±50°)
 *    2. Implausible jump – large count change between reads
 *    3. Frozen value  – no change while motor is actively driving
 *
 *  On any fault enc_fault is latched; the safety system must handle
 *  the transition to SAFE state and steering neutralisation.
 * ================================================================== */

void Encoder_CheckHealth(void)
{
    /* Fault is latched intentionally: in a safety-critical steering
     * system, transient encoder faults (noise, loose connector) must
     * not auto-recover.  The vehicle must come to a stop and be
     * inspected.  Only a full system reset clears the latch.          */
    if (enc_fault) return;

    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint32_t now  = HAL_GetTick();

    /* --- 1. Out-of-range check ---
     * If the counter is outside the mechanically possible range the
     * encoder signal is corrupt or disconnected (counter wrapped).    */
    if (count > ENC_MAX_COUNTS || count < -ENC_MAX_COUNTS) {
        enc_fault = 1;
        return;
    }

    /* --- 2. Implausible jump check ---
     * A sudden large delta between consecutive reads indicates noise,
     * wiring fault, or encoder disconnect/reconnect.                  */
    int16_t delta = count - enc_prev_count;
    if (delta < 0) delta = (int16_t)(-delta);  /* abs — overflow-safe because
                                                 * both values are bounded by
                                                 * ENC_MAX_COUNTS (667).      */
    if (delta > ENC_MAX_JUMP) {
        enc_fault = 1;
        return;
    }

    /* --- 3. Frozen value check ---
     * If the PID is commanding significant motor output but the
     * encoder count has not changed, the sensor is likely
     * disconnected or mechanically decoupled.                         */
    if (count != enc_prev_count) {
        enc_last_change_tick = now;
    } else {
        float motor_pct = fabsf(steering_pid.output);
        if (motor_pct > ENC_MOTOR_ACTIVE_PCT) {
            if ((now - enc_last_change_tick) > ENC_FROZEN_TIMEOUT_MS) {
                enc_fault = 1;
                return;
            }
        }
    }

    enc_prev_count = count;
}

bool Encoder_HasFault(void)
{
    return (enc_fault != 0);
}

/**
 * @brief  Safely disable steering motor output.
 *
 * Used when the encoder is faulted: we must NOT drive the motor
 * toward any position because we have no reliable feedback.
 * Instead, cut PWM and disable the H-bridge enable pin.
 */
void Steering_Neutralize(void)
{
    Motor_SetPWM(&motor_steer, 0);
    Motor_Enable(&motor_steer, 0);
    steering_pid.integral   = 0.0f;
    steering_pid.prev_error = 0.0f;
    steering_pid.output     = 0.0f;
}

/* ==================================================================
 *  Ackermann Geometry
 * ================================================================== */

AckermannResult_t Ackermann_Compute(float wheelAngleDeg)
{
    AckermannResult_t result = {0};
    if (fabs(wheelAngleDeg) < 0.01f) {
        result.innerDeg = 0.0f;
        result.outerDeg = 0.0f;
        return result;
    }

    float angle_rad = wheelAngleDeg * (float)M_PI / 180.0f;
    float turn_radius = ackermann_wheelbase / tanf(fabs(angle_rad));

    float inner_radius = turn_radius - ackermann_track / 2.0f;
    float outer_radius = turn_radius + ackermann_track / 2.0f;

    float inner_rad = atanf(ackermann_wheelbase / inner_radius);
    float outer_rad = atanf(ackermann_wheelbase / outer_radius);

    result.innerDeg = inner_rad * 180.0f / (float)M_PI;
    result.outerDeg = outer_rad * 180.0f / (float)M_PI;

    if (result.innerDeg > ackermann_max_inner)
        result.innerDeg = ackermann_max_inner;

    return result;
}

void Ackermann_SetGeometry(float wheelbase_m, float track_m, float maxInnerDeg)
{
    ackermann_wheelbase  = wheelbase_m;
    ackermann_track      = track_m;
    ackermann_max_inner  = maxInnerDeg;
}

/* ==================================================================
 *  Low-level per-wheel PWM wrappers
 * ================================================================== */

void Motor_SetPWM_FL(uint16_t pwm, bool reverse) {
    Motor_SetPWM(&motor_fl, pwm);
    Motor_SetDirection(&motor_fl, reverse ? -1 : 1);
    Motor_Enable(&motor_fl, (pwm > 0));
}

void Motor_SetPWM_FR(uint16_t pwm, bool reverse) {
    Motor_SetPWM(&motor_fr, pwm);
    Motor_SetDirection(&motor_fr, reverse ? -1 : 1);
    Motor_Enable(&motor_fr, (pwm > 0));
}

void Motor_SetPWM_RL(uint16_t pwm, bool reverse) {
    Motor_SetPWM(&motor_rl, pwm);
    Motor_SetDirection(&motor_rl, reverse ? -1 : 1);
    Motor_Enable(&motor_rl, (pwm > 0));
}

void Motor_SetPWM_RR(uint16_t pwm, bool reverse) {
    Motor_SetPWM(&motor_rr, pwm);
    Motor_SetDirection(&motor_rr, reverse ? -1 : 1);
    Motor_Enable(&motor_rr, (pwm > 0));
}

void Motor_SetPWM_Steering(uint16_t pwm, bool reverse) {
    Motor_SetPWM(&motor_steer, pwm);
    Motor_SetDirection(&motor_steer, reverse ? -1 : 1);
    Motor_Enable(&motor_steer, (pwm > 0));
}

/* ==================================================================
 *  Private helpers
 * ================================================================== */

static void Motor_SetPWM(Motor_t *motor, uint16_t pwm)
{
    if (pwm > PWM_PERIOD) pwm = PWM_PERIOD;
    __HAL_TIM_SET_COMPARE(motor->timer, motor->channel, pwm);
}

static void Motor_SetDirection(Motor_t *motor, int8_t direction)
{
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin,
                      (direction > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Motor_Enable(Motor_t *motor, uint8_t enable)
{
    HAL_GPIO_WritePin(motor->en_port, motor->en_pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static float PID_Compute(PID_t *pid, float measured, float dt)
{
    if (dt <= 0.0001f) return pid->output;
    float error = pid->setpoint - measured;
    pid->integral += error * dt;
    if (pid->integral >  1000.0f) pid->integral =  1000.0f;
    if (pid->integral < -1000.0f) pid->integral = -1000.0f;
    float derivative = (error - pid->prev_error) / dt;
    pid->output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    pid->prev_error = error;
    return pid->output;
}