/**
  ****************************************************************************
  * @file           : motor_control.c
  * @brief          : Motor control implementation with PWM, PID, and Ackermann
  ****************************************************************************
  */

#include "motor_control.h"
#include "ackermann.h"
#include "main.h"
#include "safety_system.h"
#include "sensor_manager.h"
#include "service_mode.h"
#include <math.h>

/* ---- NaN / Inf float validation ----
 *
 * Security hardening: NaN and Inf float values bypass normal C float
 * comparisons (NaN != NaN, NaN < x is always false, etc.) and can
 * propagate through the traction pipeline into PWM registers.
 *
 * This helper forces any non-finite value to a safe default (0.0f)
 * and raises SAFETY_ERROR_SENSOR_FAULT so the safety state machine
 * can react.  Called on every float input that affects torque/PWM.
 *
 * Reference: TECHNICAL_AUDIT_REPORT.md risk R1.                       */
static inline float sanitize_float(float val, float safe_default)
{
    if (isnan(val) || isinf(val)) {
        Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
        return safe_default;
    }
    return val;
}

/* Constants */
#define PWM_PERIOD     4249   /* Center-aligned ARR: 170 MHz / (2 × 4250) = 20 kHz */
#define PWM_FREQUENCY  20000

/* ---- Pedal signal conditioning ----
 *
 * A) EMA noise filter
 *    Coefficient from reference firmware (pedal.cpp: EMA_ALPHA = 0.15f).
 *    At the 20 Hz update rate used by main.c the –3 dB cutoff is ≈ 0.5 Hz,
 *    which rejects ADC / EMI noise while keeping pedal feel responsive.
 *
 * B) Ramp rate limiter
 *    MOTOR_CONTROL.md documents MAX_ACCEL_RATE = 50 %/s.
 *    Deceleration (ramp-down) uses 2× the rate (100 %/s) so the driver
 *    can lift off the pedal quickly.
 *    dt is computed from HAL_GetTick() exactly as Steering_ControlLoop does. */
#define PEDAL_EMA_ALPHA        0.15f     /* EMA coefficient (reference repo) */
#define PEDAL_RAMP_UP_PCT_S    50.0f     /* Max rise   rate (%/s) */
#define PEDAL_RAMP_DOWN_PCT_S  100.0f    /* Max fall   rate (%/s) */

/* ---- Demand anomaly detection (Security Hardening Phase 2) ----
 *
 * Defence-in-depth layer that detects implausible throttle demand
 * patterns that could indicate sensor faults, CAN corruption, or
 * injection attacks.  Traced to reference firmware traction.cpp
 * demand anomaly detection.
 *
 * Three independent checks:
 *   1) Step-rate:  raw demand jump > MAX_THROTTLE_STEP_PER_10MS
 *                  → clamp to allowed rate, DEGRADED
 *   2) Range:      effective_demand outside [0, 100] after pipeline
 *                  → force to 0, DEGRADED
 *   3) Frozen:     pedal value identical for > FROZEN_PEDAL_TIMEOUT_MS
 *                  while vehicle speed changes significantly
 *                  → raise warning, DEGRADED
 *
 * All checks are non-blocking (timestamp-based, no HAL_Delay).
 * Reuses SAFETY_ERROR_SENSOR_FAULT — no new CAN error codes.        */
#define MAX_THROTTLE_STEP_PER_10MS  15.0f  /* Max allowed raw jump (%/10ms)  */
#define FROZEN_PEDAL_TIMEOUT_MS     5000   /* Frozen pedal timeout (ms)      */
#define FROZEN_PEDAL_SPEED_DELTA_KMH 3.0f  /* Speed change threshold (km/h)  */
#define FROZEN_PEDAL_TOLERANCE_PCT   0.5f  /* Pedal movement tolerance (%)   */
#define DYNBRAKE_ACTIVE_THRESHOLD    0.5f  /* dynbrake_pct above = active    */
#define TRACTION_ZERO_DEMAND_PCT     0.5f  /* Demand below this = zero (%)   */

/* ---- Dynamic braking configuration ----
 *
 * When the driver releases the throttle rapidly, the vehicle decelerates
 * smoothly via H-bridge active braking (short-brake mode) instead of
 * coasting.  This is dynamic braking only — no energy is fed back to the
 * battery (the H-bridge dissipates it as heat in the motor windings).
 *
 * The brake effort is proportional to the throttle decrease rate:
 *   brake_pct = |throttle_rate| * DYNBRAKE_FACTOR
 * clamped to DYNBRAKE_MAX_PCT.
 *
 * The brake is disabled:
 *   - Below DYNBRAKE_MIN_SPEED_KMH (wheels nearly stationary)
 *   - In SYS_STATE_SAFE, SYS_STATE_ERROR, emergency stop, CAN timeout
 *   - When ABS is active (any wheel_scale < 1.0)
 * The brake is reduced in DEGRADED mode (scaled by power limit factor).  */
#define DYNBRAKE_FACTOR          0.5f    /* Brake %  per  throttle-%/s     */
#define DYNBRAKE_MAX_PCT         60.0f   /* Maximum dynamic brake (%)      */
#define DYNBRAKE_MIN_SPEED_KMH   3.0f    /* Disable below this speed       */
#define DYNBRAKE_RAMP_DOWN_PCT_S 80.0f   /* Max brake release rate (%/s)   */

/* ---- BTS7960 active brake ----
 *
 * Chinese BTS7960 modules have a known design defect: when EN=HIGH
 * and PWM=0, the motor is not braked — it floats.  This causes
 * unintended rolling on slopes and steering drift.
 *
 * Active brake on the BTS7960 is achieved by driving PWM to 100 %
 * (full duty, both FETs ON) which shorts the motor terminals through
 * the H-bridge, producing electromagnetic braking torque.
 *
 * This constant is used wherever the firmware needs a motor to hold
 * position rather than coast.  It replaces the previous PWM=0 + EN=0
 * pattern that left motors floating.                                  */
#define BTS7960_BRAKE_PWM        PWM_PERIOD   /* 100 % duty = active brake */

/* ---- Park hold configuration ----
 *
 * In gear P the STM32 applies a controlled active motor brake to
 * simulate a parking lock.  The H-bridge shorts the motor terminals
 * (brake mode), producing a holding torque proportional to PWM duty.
 *
 * Current and temperature are monitored; braking is progressively
 * reduced or disabled to protect the motors during long-duration hold. */
#define PARK_HOLD_PWM_PCT        30.0f   /* Default hold duty (%)          */
#define PARK_HOLD_CURRENT_WARN_A 15.0f   /* Reduce braking above this (A)  */
#define PARK_HOLD_CURRENT_MAX_A  20.0f   /* Disable braking above this (A) */
#define PARK_HOLD_TEMP_WARN_C    70.0f   /* Reduce braking above (°C)      */
#define PARK_HOLD_TEMP_CRIT_C    85.0f   /* Disable braking above (°C)     */

/* ---- Gear-based power scaling ----
 *
 * Gear determines direction; power mode sets the max power fraction.
 * Applied once at the final demand stage in Traction_Update() so it
 * does NOT interact with ABS/TCS wheel_scale[] or the ramp limiter.
 *
 *   GEAR_FORWARD  (D1) = 60 % max power — default forward mode
 *   GEAR_FORWARD_D2    = 100 % max power — full performance
 *   GEAR_REVERSE       = 60 % max power
 *
 * Safety_GetPowerLimitFactor() is applied separately upstream and
 * is NOT modified by this scaling.                                     */
#define GEAR_POWER_FORWARD_PCT   0.60f   /* D1: 60 % max power            */
#define GEAR_POWER_FORWARD_D2_PCT 1.00f /* D2: 100 % max power           */
#define GEAR_POWER_REVERSE_PCT   0.60f  /* R:  60 % max power            */

/* ---- Per-motor emergency temperature cutoff ----
 *
 * Hardware protection layer independent from Safety_CheckTemperature().
 * Traced to reference firmware traction.cpp:
 *   TEMP_EMERGENCY_SHUTDOWN = 130°C → immediate per-motor stop
 *
 * When a motor reaches 130°C, wheel_scale[i] is forced to 0.0 for
 * that motor ONLY.  Other motors are NOT affected.  This does NOT
 * trigger global SAFE state — it coexists with the existing 90°C
 * SAFE trigger in safety_system.c.
 *
 * 15°C hysteresis: motor re-enabled below 115°C.                      */
#define MOTOR_TEMP_CUTOFF_C      130.0f  /* Per-motor emergency cutoff   */
#define MOTOR_TEMP_RECOVERY_C    115.0f  /* Hysteresis recovery point    */

/* ---- Ackermann differential torque correction ----
 *
 * Simplified Ackermann geometry: when the vehicle is turning, the
 * inside wheels trace a smaller radius arc than the outside wheels.
 * To prevent inside wheel scrubbing (understeer) and improve
 * cornering stability, the torque is biased toward the outside
 * wheels and reduced on the inside wheels.
 *
 * Geometry:
 *   R = wheelbase / tan(|steering_angle|)   (turn center radius)
 *   left_ratio  = (R - track/2) / R
 *   right_ratio = (R + track/2) / R
 *   (swapped for right turns)
 *
 * The correction is bounded to ±15% maximum differential to avoid
 * aggressive torque imbalance.  Below a 2° deadband, no correction
 * is applied (straight-line driving).
 *
 * Pipeline position:
 *   base_pwm → axle_split → degraded_limit → obstacle_scale
 *   → ackermann_diff[i] → wheel_scale[i] (ABS/TCS) → final PWM
 *
 * Coexists with ABS, TCS, obstacle_scale, and degraded mode.          */
#define ACKERMANN_DEADBAND_DEG   2.0f    /* No correction below this    */
#define ACKERMANN_MAX_DIFF       0.15f   /* Max ±15% differential       */

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

/* ---- Pedal filter / ramp state ---- */
static float    pedal_ema         = 0.0f;   /* EMA-filtered pedal value      */
static float    pedal_ramped      = 0.0f;   /* Output after ramp limiting    */
static uint8_t  pedal_filter_init = 0;      /* 0 = first sample pending      */
static uint32_t pedal_last_tick   = 0;

/* ---- Demand anomaly detection state ---- */
static float    prev_raw_demand      = 0.0f;   /* Previous raw throttle input   */
static uint32_t prev_raw_demand_tick = 0;       /* Timestamp of previous input   */
static float    frozen_pedal_value   = 0.0f;   /* Last distinct pedal value      */
static uint32_t frozen_pedal_tick    = 0;       /* When pedal last changed        */
static float    frozen_pedal_speed   = 0.0f;   /* Vehicle speed when pedal froze */
static uint8_t  anomaly_init         = 0;       /* 0 = first sample pending       */

/* ---- Dynamic braking state ---- */
static float    dynbrake_pct      = 0.0f;   /* Current dynamic brake effort  */
static float    prev_demand_pct   = 0.0f;   /* Previous demand for rate calc */
static uint32_t dynbrake_last_tick = 0;

/* ---- Gear position state ---- */
static GearPosition_t current_gear = GEAR_FORWARD;

/* ---- Per-motor overtemp cutoff state ---- */
static bool motor_overtemp_cutoff[4] = {false, false, false, false};

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
#define ENC_MAX_COUNTS       ((int32_t)((MAX_STEER_DEG + 20.0f) * (float)ENCODER_CPR / 360.0f))
        /* Encoder is 1:1 on the steering output shaft.
         * ±74° (54° max road-wheel + 20° margin) → ±987 counts.
         * Any reading beyond this is mechanically impossible.            */
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

static int32_t  enc_prev_count       = 0;
static uint32_t enc_last_change_tick = 0;
static uint8_t  enc_fault            = 0;   /* 0 = healthy, 1 = faulted */

extern TIM_HandleTypeDef htim1, htim2, htim8;

/* Private function prototypes */
static void Motor_SetPWM(Motor_t *motor, uint16_t pwm);
static void Motor_SetDirection(Motor_t *motor, int8_t direction);
static void Motor_Enable(Motor_t *motor, uint8_t enable);
static float PID_Compute(PID_t *pid, float measured, float dt);
static void compute_ackermann_differential(float steer_deg, float diff_out[4]);

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

    /* Reset pedal filter / ramp state */
    pedal_ema         = 0.0f;
    pedal_ramped      = 0.0f;
    pedal_filter_init = 0;

    /* Reset dynamic braking state */
    dynbrake_pct       = 0.0f;
    prev_demand_pct    = 0.0f;
    dynbrake_last_tick = 0;

    /* Reset demand anomaly detection state */
    prev_raw_demand      = 0.0f;
    prev_raw_demand_tick = 0;
    frozen_pedal_value   = 0.0f;
    frozen_pedal_tick    = 0;
    frozen_pedal_speed   = 0.0f;
    anomaly_init         = 0;

    /* Default gear to FORWARD */
    current_gear = GEAR_FORWARD;
}

void Steering_Init(void)
{
    steering_pid.integral   = 0.0f;
    steering_pid.prev_error = 0.0f;
    steering_pid.setpoint   = 0.0f;
    steering_pid.output     = 0.0f;
    __HAL_TIM_SET_COUNTER(&htim2, 0);  /* Zero encoder at current position */
    /* Calibration flag is intentionally NOT set here.
     * The automatic centering module (steering_centering.c) must detect
     * the physical center reference before the encoder zero is valid.
     * Until centering completes, steering commands are rejected.         */
    steering_calibrated = 0;

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
    /* NaN/Inf guard — reject corrupt throttle demand (security hardening) */
    throttlePct = sanitize_float(throttlePct, 0.0f);

    /* Clamp raw input to the existing ±100 % range (unchanged) */
    if (throttlePct < -100.0f) throttlePct = -100.0f;
    if (throttlePct >  100.0f) throttlePct =  100.0f;

    /* ---- Demand anomaly: throttle step-rate validation ----
     * Detect unrealistic demand jumps (e.g. 0% → 100% in < 10ms)
     * that could indicate sensor fault or CAN injection.
     * Applied to the RAW input before EMA/ramp filtering.
     * Clamp to allowed rate and raise DEGRADED (not SAFE).             */
    uint32_t now_anom = HAL_GetTick();
    if (!anomaly_init) {
        prev_raw_demand      = throttlePct;
        prev_raw_demand_tick = now_anom;
        frozen_pedal_value   = throttlePct;
        frozen_pedal_tick    = now_anom;
        frozen_pedal_speed   = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                                Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;
        anomaly_init         = 1;
    } else {
        float dt_anom = (float)(now_anom - prev_raw_demand_tick);
        if (dt_anom < 1.0f) dt_anom = 1.0f;  /* minimum 1 ms */
        float allowed_step = MAX_THROTTLE_STEP_PER_10MS * (dt_anom / 10.0f);
        float raw_diff = throttlePct - prev_raw_demand;

        if (fabsf(raw_diff) > allowed_step) {
            /* Anomalous jump detected — clamp to max allowed step */
            if (raw_diff > 0.0f) {
                throttlePct = prev_raw_demand + allowed_step;
            } else {
                throttlePct = prev_raw_demand - allowed_step;
            }
            Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
            if (Safety_GetState() == SYS_STATE_ACTIVE) {
                Safety_SetState(SYS_STATE_DEGRADED);
                Safety_SetDegradedLevel(DEGRADED_L1,
                                        DEGRADED_REASON_DEMAND_ANOMALY);
            }
        }
        prev_raw_demand      = throttlePct;
        prev_raw_demand_tick = now_anom;
    }

    /* ---- Demand anomaly: frozen pedal detection ----
     * If pedal value remains identical for > FROZEN_PEDAL_TIMEOUT_MS
     * while vehicle speed changes significantly, raise warning.
     * Non-blocking, timestamp-based.                                    */
    if (fabsf(throttlePct - frozen_pedal_value) > FROZEN_PEDAL_TOLERANCE_PCT) {
        /* Pedal moved — reset frozen tracking */
        frozen_pedal_value = throttlePct;
        frozen_pedal_tick  = now_anom;
        frozen_pedal_speed = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                              Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;
    } else if ((now_anom - frozen_pedal_tick) > FROZEN_PEDAL_TIMEOUT_MS) {
        /* Pedal has been frozen — check speed divergence */
        float current_speed = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                               Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;
        if (fabsf(current_speed - frozen_pedal_speed) > FROZEN_PEDAL_SPEED_DELTA_KMH) {
            /* Speed changed significantly while pedal frozen → anomaly */
            Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
            if (Safety_GetState() == SYS_STATE_ACTIVE) {
                Safety_SetState(SYS_STATE_DEGRADED);
                Safety_SetDegradedLevel(DEGRADED_L1,
                                        DEGRADED_REASON_DEMAND_ANOMALY);
            }
            /* Reset tick to avoid re-triggering every cycle */
            frozen_pedal_tick = now_anom;
            frozen_pedal_speed = current_speed;
        }
    }

    /* ---- A) EMA noise filter ---- */
    if (!pedal_filter_init) {
        pedal_ema       = throttlePct;
        pedal_ramped    = throttlePct;
        pedal_last_tick = HAL_GetTick();
        pedal_filter_init = 1;
    } else {
        pedal_ema = PEDAL_EMA_ALPHA * throttlePct
                  + (1.0f - PEDAL_EMA_ALPHA) * pedal_ema;
    }

    /* ---- B) Ramp rate limiter (applied after EMA) ---- */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - pedal_last_tick) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;   /* guard against zero / tiny dt */
    pedal_last_tick = now;

    float target = pedal_ema;
    float diff   = target - pedal_ramped;

    if (diff > 0.0f) {
        /* Accelerating: slower ramp */
        float max_up = PEDAL_RAMP_UP_PCT_S * dt;
        if (diff > max_up) diff = max_up;
    } else {
        /* Decelerating: faster ramp */
        float max_down = PEDAL_RAMP_DOWN_PCT_S * dt;
        if (diff < -max_down) diff = -max_down;
    }
    pedal_ramped += diff;

    /* Final clamp — never exceed 0–100 % (or ±100 % for reverse) */
    if (pedal_ramped < -100.0f) pedal_ramped = -100.0f;
    if (pedal_ramped >  100.0f) pedal_ramped =  100.0f;

    traction_state.demandPct = pedal_ramped;
}

void Traction_SetMode4x4(bool enable)
{
    traction_state.mode4x4 = enable;
}

void Traction_SetAxisRotation(bool enable)
{
    traction_state.axisRotation = enable;
}

void Traction_SetGear(GearPosition_t gear)
{
    current_gear = gear;
}

GearPosition_t Traction_GetGear(void)
{
    return current_gear;
}

/* ==================================================================
 *  Ackermann Differential Torque Correction
 *
 *  Computes per-wheel torque multipliers based on the current
 *  steering angle using simplified Ackermann geometry.
 *
 *  For a turn of radius R (measured to the vehicle centre):
 *    left_wheel_radius  = R - track/2
 *    right_wheel_radius = R + track/2
 *
 *  The linear velocity of each wheel is proportional to its radius:
 *    left_ratio  = (R - track/2) / R = 1 - track/(2R)
 *    right_ratio = (R + track/2) / R = 1 + track/(2R)
 *
 *  For a left turn (positive angle), left wheels are inside;
 *  for a right turn (negative angle), right wheels are inside.
 *
 *  The correction is bounded to ±ACKERMANN_MAX_DIFF (15%) and
 *  each multiplier is clamped to [0, 1].
 *
 *  diff_out[4]: FL, FR, RL, RR multipliers (1.0 = no change)
 * ================================================================== */

static void compute_ackermann_differential(float steer_deg, float diff_out[4])
{
    /* Default: no correction (straight line or below deadband) */
    diff_out[MOTOR_FL] = 1.0f;
    diff_out[MOTOR_FR] = 1.0f;
    diff_out[MOTOR_RL] = 1.0f;
    diff_out[MOTOR_RR] = 1.0f;

    float abs_angle = fabsf(steer_deg);
    if (abs_angle < ACKERMANN_DEADBAND_DEG) return;

    /* Compute turn radius from Ackermann geometry:
     * R = wheelbase / tan(|angle|) — distance from turn center
     * to the midpoint of the rear axle.                            */
    float angle_rad = abs_angle * (float)M_PI / 180.0f;
    float tan_angle = tanf(angle_rad);

    /* Guard against very small tan (near-zero angle already
     * filtered by deadband, but protect against float edge cases) */
    if (tan_angle < 0.001f) return;

    float R = WHEELBASE_M / tan_angle;

    /* Compute correction term: half_track / R.
     * This is the fractional velocity difference between inside
     * and outside wheels relative to the vehicle center speed.     */
    float half_track = TRACK_WIDTH_M / 2.0f;
    float correction = half_track / R;

    /* Bound correction to maximum differential */
    if (correction > ACKERMANN_MAX_DIFF)
        correction = ACKERMANN_MAX_DIFF;

    /* Apply correction:
     *   Positive steer_deg = left turn → left wheels inside (slower)
     *   Negative steer_deg = right turn → right wheels inside (slower)
     *
     * inside_mult  = 1.0 - correction  (reduce inside wheels)
     * outside_mult = 1.0 + correction  (increase outside wheels)
     * Then clamp outside to 1.0 to never exceed base torque.       */
    float inside_mult  = 1.0f - correction;
    float outside_mult = 1.0f + correction;

    /* Clamp: never exceed 1.0 per wheel */
    if (outside_mult > 1.0f) outside_mult = 1.0f;
    if (inside_mult  < 0.0f) inside_mult  = 0.0f;

    if (steer_deg > 0.0f) {
        /* Left turn: left wheels are inside */
        diff_out[MOTOR_FL] = inside_mult;
        diff_out[MOTOR_FR] = outside_mult;
        diff_out[MOTOR_RL] = inside_mult;
        diff_out[MOTOR_RR] = outside_mult;
    } else {
        /* Right turn: right wheels are inside */
        diff_out[MOTOR_FL] = outside_mult;
        diff_out[MOTOR_FR] = inside_mult;
        diff_out[MOTOR_RL] = outside_mult;
        diff_out[MOTOR_RR] = inside_mult;
    }

    /* Sanitize all outputs */
    for (uint8_t i = 0; i < 4; i++) {
        diff_out[i] = sanitize_float(diff_out[i], 1.0f);
    }
}

void Traction_Update(void)
{
    /* --- Gear P: Park Hold ---
     * Apply controlled active brake via H-bridge to simulate a parking
     * lock.  No throttle demand is accepted.  Current and temperature
     * are monitored to prevent overheating during long-duration hold.
     * Park hold is released in SAFE/ERROR states (safety override).    */
    if (current_gear == GEAR_PARK) {
        SystemState_t st = Safety_GetState();
        if (st == SYS_STATE_SAFE || st == SYS_STATE_ERROR) {
            /* Safety override — release park hold */
            Motor_SetPWM(&motor_fl, 0); Motor_Enable(&motor_fl, 0);
            Motor_SetPWM(&motor_fr, 0); Motor_Enable(&motor_fr, 0);
            Motor_SetPWM(&motor_rl, 0); Motor_Enable(&motor_rl, 0);
            Motor_SetPWM(&motor_rr, 0); Motor_Enable(&motor_rr, 0);
        } else {
            /* Compute park hold PWM with current/temp derating */
            float hold_pct = PARK_HOLD_PWM_PCT;

            /* Check per-motor current and temperature; use worst case */
            float max_current = 0.0f;
            float max_temp    = 0.0f;
            for (uint8_t i = 0; i < 4; i++) {
                float a = Current_GetAmps(i);
                float t = Temperature_Get(i);
                if (a > max_current) max_current = a;
                if (t > max_temp)    max_temp    = t;
            }

            /* Current derating */
            if (max_current > PARK_HOLD_CURRENT_MAX_A) {
                hold_pct = 0.0f;   /* Disable braking entirely */
            } else if (max_current > PARK_HOLD_CURRENT_WARN_A) {
                float ratio = (PARK_HOLD_CURRENT_MAX_A - max_current)
                            / (PARK_HOLD_CURRENT_MAX_A - PARK_HOLD_CURRENT_WARN_A);
                hold_pct *= ratio;
            }

            /* Temperature derating */
            if (max_temp > PARK_HOLD_TEMP_CRIT_C) {
                hold_pct = 0.0f;   /* Disable braking entirely */
            } else if (max_temp > PARK_HOLD_TEMP_WARN_C) {
                float ratio = (PARK_HOLD_TEMP_CRIT_C - max_temp)
                            / (PARK_HOLD_TEMP_CRIT_C - PARK_HOLD_TEMP_WARN_C);
                hold_pct *= ratio;
            }

            uint16_t hold_pwm = (uint16_t)(hold_pct * PWM_PERIOD / 100.0f);

            /* Apply hold brake to all four motors.
             * Direction is set to forward; the H-bridge "brake" effect
             * comes from driving all motors at low duty with enable ON.
             * This dissipates energy as heat — no regeneration.         */
            Motor_SetPWM(&motor_fl, hold_pwm);
            Motor_SetPWM(&motor_fr, hold_pwm);
            Motor_SetPWM(&motor_rl, hold_pwm);
            Motor_SetPWM(&motor_rr, hold_pwm);
            Motor_SetDirection(&motor_fl, 1);
            Motor_SetDirection(&motor_fr, 1);
            Motor_SetDirection(&motor_rl, 1);
            Motor_SetDirection(&motor_rr, 1);
            Motor_Enable(&motor_fl, (hold_pwm > 0) ? 1 : 0);
            Motor_Enable(&motor_fr, (hold_pwm > 0) ? 1 : 0);
            Motor_Enable(&motor_rl, (hold_pwm > 0) ? 1 : 0);
            Motor_Enable(&motor_rr, (hold_pwm > 0) ? 1 : 0);
        }

        /* Update sensor readings even in park */
        traction_state.wheels[0].speedKmh = Wheel_GetSpeed_FL();
        traction_state.wheels[1].speedKmh = Wheel_GetSpeed_FR();
        traction_state.wheels[2].speedKmh = Wheel_GetSpeed_RL();
        traction_state.wheels[3].speedKmh = Wheel_GetSpeed_RR();
        for (uint8_t i = 0; i < 4; i++) {
            traction_state.wheels[i].currentA = Current_GetAmps(i);
            traction_state.wheels[i].tempC    = Temperature_Get(i);
        }
        return;
    }

    /* --- Gear N: Neutral / Coast ---
     * All motors enter full coast mode: PWM = 0, H-bridge disabled.
     * No active braking, no holding torque — wheels spin freely.
     * Dynamic braking is also disabled in Neutral.                     */
    if (current_gear == GEAR_NEUTRAL) {
        Motor_SetPWM(&motor_fl, 0); Motor_Enable(&motor_fl, 0);
        Motor_SetPWM(&motor_fr, 0); Motor_Enable(&motor_fr, 0);
        Motor_SetPWM(&motor_rl, 0); Motor_Enable(&motor_rl, 0);
        Motor_SetPWM(&motor_rr, 0); Motor_Enable(&motor_rr, 0);

        /* Reset dynamic braking state so it doesn't spike on gear change */
        dynbrake_pct    = 0.0f;
        prev_demand_pct = 0.0f;

        /* Update sensor readings */
        traction_state.wheels[0].speedKmh = Wheel_GetSpeed_FL();
        traction_state.wheels[1].speedKmh = Wheel_GetSpeed_FR();
        traction_state.wheels[2].speedKmh = Wheel_GetSpeed_RL();
        traction_state.wheels[3].speedKmh = Wheel_GetSpeed_RR();
        for (uint8_t i = 0; i < 4; i++) {
            traction_state.wheels[i].currentA = Current_GetAmps(i);
            traction_state.wheels[i].tempC    = Temperature_Get(i);
            traction_state.wheels[i].pwm      = 0;
            traction_state.wheels[i].reverse  = false;
        }
        return;
    }

    /* --- Gear F/D1/D2/R: Normal traction with dynamic braking ---       */
    float demand = traction_state.demandPct;

    /* ---- Dynamic braking computation ----
     * Detect throttle decrease rate and generate proportional braking.
     * The brake effort ramps progressively and respects existing limits. */
    uint32_t now_db = HAL_GetTick();
    float dt_db = (float)(now_db - dynbrake_last_tick) / 1000.0f;
    if (dt_db < 0.001f) dt_db = 0.001f;
    dynbrake_last_tick = now_db;

    float demand_rate = (demand - prev_demand_pct) / dt_db;  /* %/s */
    prev_demand_pct = demand;

    /* Determine if dynamic braking should be active */
    bool dynbrake_allowed = true;
    SystemState_t sys_st = Safety_GetState();

    /* Disable in non-driveable states */
    if (sys_st == SYS_STATE_SAFE  || sys_st == SYS_STATE_ERROR ||
        sys_st == SYS_STATE_BOOT  || sys_st == SYS_STATE_STANDBY) {
        dynbrake_allowed = false;
    }

    /* Disable if ABS is active on any wheel (wheel_scale < 1.0) */
    if (dynbrake_allowed) {
        for (uint8_t i = 0; i < 4; i++) {
            if (safety_status.wheel_scale[i] < 1.0f) {
                dynbrake_allowed = false;
                break;
            }
        }
    }

    /* Disable below minimum speed */
    if (dynbrake_allowed) {
        float avg_speed = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                           Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;
        if (avg_speed < DYNBRAKE_MIN_SPEED_KMH) {
            dynbrake_allowed = false;
        }
    }

    if (dynbrake_allowed && demand_rate < 0.0f) {
        /* Throttle decreasing — apply proportional brake */
        float target_brake = fabsf(demand_rate) * DYNBRAKE_FACTOR;
        if (target_brake > DYNBRAKE_MAX_PCT) target_brake = DYNBRAKE_MAX_PCT;

        /* Limit in DEGRADED mode — uses per-level power limit (Phase 12) */
        if (sys_st == SYS_STATE_DEGRADED) {
            target_brake *= Safety_GetPowerLimitFactor();
        }

        /* Progressive ramp toward target (never jump instantly) */
        if (target_brake > dynbrake_pct) {
            float brake_ramp = PEDAL_RAMP_DOWN_PCT_S * dt_db;
            float diff_b = target_brake - dynbrake_pct;
            if (diff_b > brake_ramp) diff_b = brake_ramp;
            dynbrake_pct += diff_b;
        } else {
            dynbrake_pct = target_brake;
        }
    } else {
        /* No braking needed — ramp down smoothly */
        if (dynbrake_pct > 0.0f) {
            float release = DYNBRAKE_RAMP_DOWN_PCT_S * dt_db;
            dynbrake_pct -= release;
            if (dynbrake_pct < 0.0f) dynbrake_pct = 0.0f;
        }
    }

    /* When dynamic braking is active and throttle demand is near zero,
     * the brake PWM is applied with the motor direction reversed relative
     * to the travel direction.  This creates an opposing torque that
     * decelerates the vehicle.  Energy is dissipated as heat in the motor
     * windings — the battery is NOT charged.                             */
    float effective_demand = demand;

    if (dynbrake_pct > DYNBRAKE_ACTIVE_THRESHOLD && fabsf(demand) < 1.0f) {
        /* Use dynamic braking — set negative demand (opposing torque) */
        effective_demand = -dynbrake_pct;
    }

    /* ---- Gear-based power scaling (applied once at final demand) ----
     * Scale the positive traction demand by the gear power fraction.
     * Dynamic braking demand is NOT scaled (braking effort is
     * independent of power mode).  ABS/TCS wheel_scale[] is applied
     * separately per-wheel below and is not affected.                   */
    if (effective_demand > 0.0f) {
        float gear_scale;
        if (current_gear == GEAR_FORWARD_D2) {
            gear_scale = GEAR_POWER_FORWARD_D2_PCT;
        } else if (current_gear == GEAR_REVERSE) {
            gear_scale = GEAR_POWER_REVERSE_PCT;
        } else {
            gear_scale = GEAR_POWER_FORWARD_PCT;
        }
        effective_demand *= gear_scale;
    }

    /* ---- Demand anomaly: negative / out-of-range validation ----
     * After gear scaling (positive demands only), effective_demand
     * should be in [–dynbrake_max, 100].  If it falls outside [0, 100]
     * when positive traction is expected (not dynamic braking), force
     * to 0 and raise DEGRADED.  Dynamic braking legitimately produces
     * negative effective_demand — do not flag that.                     */
    if (effective_demand > 100.0f) {
        effective_demand = 0.0f;
        Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
        if (Safety_GetState() == SYS_STATE_ACTIVE) {
            Safety_SetState(SYS_STATE_DEGRADED);
            Safety_SetDegradedLevel(DEGRADED_L1,
                                    DEGRADED_REASON_DEMAND_ANOMALY);
        }
    } else if (effective_demand < 0.0f && dynbrake_pct < DYNBRAKE_ACTIVE_THRESHOLD) {
        /* Negative demand without dynamic braking → anomaly */
        effective_demand = 0.0f;
        Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
        if (Safety_GetState() == SYS_STATE_ACTIVE) {
            Safety_SetState(SYS_STATE_DEGRADED);
            Safety_SetDegradedLevel(DEGRADED_L1,
                                    DEGRADED_REASON_DEMAND_ANOMALY);
        }
    }

    /* ---- Per-motor emergency temperature cutoff ----
     * Hardware protection layer independent from Safety_CheckTemperature().
     * Traced to reference firmware traction.cpp:
     *   TEMP_EMERGENCY_SHUTDOWN = 130°C per-motor immediate stop.
     *
     * For each traction motor (FL, FR, RL, RR):
     *   >= 130°C → force wheel_scale[i] = 0.0 (that motor only)
     *   <  115°C → allow wheel_scale[i] to return to normal (15°C hysteresis)
     *
     * Does NOT force global SAFE state.
     * Does NOT modify global demand.
     * Does NOT affect other wheels.
     * Coexists with ABS/TCS modulation (most restrictive wins).           */
    for (uint8_t i = 0; i < 4; i++) {
        float motor_temp = Temperature_Get(i);
        ModuleID_t temp_mod = (ModuleID_t)(MODULE_TEMP_SENSOR_0 + i);

        if (motor_temp >= MOTOR_TEMP_CUTOFF_C) {
            /* Emergency cutoff — force this motor off */
            safety_status.wheel_scale[i] = 0.0f;
            motor_overtemp_cutoff[i] = true;
            ServiceMode_SetFault(temp_mod, MODULE_FAULT_ERROR);
            if (Safety_GetError() != SAFETY_ERROR_OVERTEMP) {
                Safety_SetError(SAFETY_ERROR_OVERTEMP);
            }
        } else if (motor_overtemp_cutoff[i] && motor_temp < MOTOR_TEMP_RECOVERY_C) {
            /* Hysteresis recovery — allow normal operation */
            motor_overtemp_cutoff[i] = false;
            ServiceMode_ClearFault(temp_mod);
        } else if (motor_overtemp_cutoff[i]) {
            /* Still in cutoff band (115–130°C) — maintain cutoff */
            safety_status.wheel_scale[i] = 0.0f;
        }
    }

    /* ---- NaN/Inf validation (security hardening) ----
     * Validate all float inputs that affect PWM before they reach the
     * hardware.  NaN bypasses C float comparisons and would propagate
     * into Motor_SetPWM() producing unpredictable duty cycles.
     * Reference: TECHNICAL_AUDIT_REPORT.md risk R1.                    */
    effective_demand = sanitize_float(effective_demand, 0.0f);
    safety_status.obstacle_scale = sanitize_float(safety_status.obstacle_scale, 0.0f);
    for (uint8_t i = 0; i < 4; i++) {
        safety_status.wheel_scale[i] = sanitize_float(safety_status.wheel_scale[i], 0.0f);
    }

    uint16_t base_pwm = (uint16_t)(fabsf(effective_demand) * PWM_PERIOD / 100.0f);

    /* Apply obstacle scale uniformly to all wheels.  This multiplier
     * is set by Obstacle_Update() from CAN-received distance data.
     * Applied before per-wheel ABS/TCS wheel_scale[] so that obstacle
     * reduction and ABS/TCS modulation are multiplicative (most
     * restrictive wins).  Same approach as the reference monolithic
     * firmware's obstacleFactor in traction.cpp.                       */
    base_pwm = (uint16_t)(base_pwm * safety_status.obstacle_scale);

    /* ---- Traction cap (Phase 12) ----
     * Apply per-level traction cap in DEGRADED mode.  This limits
     * maximum PWM output independently of the power limit applied
     * to the demand upstream.  Acts as a defence-in-depth speed cap.
     * 1.0 in ACTIVE, L1=80%, L2=60%, L3=50% in DEGRADED.             */
    {
        float traction_cap = Safety_GetTractionCapFactor();
        if (traction_cap < 1.0f) {
            base_pwm = (uint16_t)(base_pwm * traction_cap);
        }
    }

    /* ---- Ackermann differential torque correction ----
     * Compute per-wheel multipliers based on current steering angle.
     * Applied after obstacle_scale and before wheel_scale[i] (ABS/TCS).
     * Skipped during tank turn (axisRotation) — differential is
     * meaningless when wheels on each side spin in opposite directions. */
    float acker_diff[4];
    compute_ackermann_differential(Steering_GetCurrentAngle(), acker_diff);

    int8_t dir   = (effective_demand >= 0) ? 1 : -1;

    /* GEAR_REVERSE: invert motor direction for reverse travel.
     * This flips both traction (positive demand → backward travel)
     * and dynamic braking (negative demand → forward opposing torque
     * becomes backward opposing torque, correct for a vehicle
     * decelerating while traveling in reverse).                         */
    if (current_gear == GEAR_REVERSE) {
        dir = -dir;
    }

    if (traction_state.axisRotation) {
        /* Tank turn: left wheels reverse, right wheels forward (or vice versa).
         * Per-wheel scale still applies for safety (slip on one side).        */
        for (uint8_t i = 0; i < 4; i++) {
            uint16_t pwm = (uint16_t)(base_pwm * safety_status.wheel_scale[i]);
            int8_t d = ((i == MOTOR_FL) || (i == MOTOR_RL)) ? (int8_t)-dir : dir;
            Motor_t *m = (i == MOTOR_FL) ? &motor_fl :
                         (i == MOTOR_FR) ? &motor_fr :
                         (i == MOTOR_RL) ? &motor_rl : &motor_rr;
            Motor_SetPWM(m, pwm);
            Motor_SetDirection(m, d);
        }
    } else if (traction_state.mode4x4) {
        /* 4x4: 50/50 axle torque split — distribute base torque equally
         * between front and rear axles, then apply Ackermann differential
         * and per-wheel ABS/TCS scale.
         * Aligned with reference firmware traction.cpp 50/50 split.
         * Each axle receives half the base_pwm; Ackermann differential
         * and per-wheel wheel_scale[] are applied after the split so
         * individual wheel interventions remain effective.  Obstacle
         * scale is already applied to base_pwm upstream.                */
        uint16_t axle_pwm = base_pwm / 2;

        Motor_SetPWM(&motor_fl, (uint16_t)(axle_pwm * acker_diff[MOTOR_FL] * safety_status.wheel_scale[MOTOR_FL]));
        Motor_SetDirection(&motor_fl, dir);
        Motor_SetPWM(&motor_fr, (uint16_t)(axle_pwm * acker_diff[MOTOR_FR] * safety_status.wheel_scale[MOTOR_FR]));
        Motor_SetDirection(&motor_fr, dir);
        Motor_SetPWM(&motor_rl, (uint16_t)(axle_pwm * acker_diff[MOTOR_RL] * safety_status.wheel_scale[MOTOR_RL]));
        Motor_SetDirection(&motor_rl, dir);
        Motor_SetPWM(&motor_rr, (uint16_t)(axle_pwm * acker_diff[MOTOR_RR] * safety_status.wheel_scale[MOTOR_RR]));
        Motor_SetDirection(&motor_rr, dir);
    } else {
        /* 4x2: Front wheels only, Ackermann differential + per-wheel scale */
        Motor_SetPWM(&motor_fl, (uint16_t)(base_pwm * acker_diff[MOTOR_FL] * safety_status.wheel_scale[MOTOR_FL]));
        Motor_SetDirection(&motor_fl, dir);
        Motor_SetPWM(&motor_fr, (uint16_t)(base_pwm * acker_diff[MOTOR_FR] * safety_status.wheel_scale[MOTOR_FR]));
        Motor_SetDirection(&motor_fr, dir);
        /* 4x2: rear motors not driven — apply BTS7960 active brake
         * to prevent free-rolling on slopes.  Chinese BTS7960 modules
         * float the motor when EN=HIGH + PWM=0, so we must use full
         * duty to engage the brake FETs.                              */
        Motor_SetPWM(&motor_rl, BTS7960_BRAKE_PWM);  Motor_Enable(&motor_rl, 1);
        Motor_SetPWM(&motor_rr, BTS7960_BRAKE_PWM);  Motor_Enable(&motor_rr, 1);
    }

    /* ---- BTS7960 active brake when demand is zero ----
     *
     * Chinese BTS7960 modules float the motor when EN=HIGH and PWM=0.
     * When demand drops to zero, we must apply active brake (PWM=100 %,
     * EN=HIGH) to prevent the vehicle from rolling on slopes.
     *
     * Active brake is NOT applied in Neutral gear (handled above) or
     * during ABS intervention (wheel_scale < 1.0 — ABS needs to
     * modulate braking per-wheel independently).
     *
     * Motors that are actively driving (demand > TRACTION_ZERO_DEMAND_PCT)
     * keep their computed PWM values unchanged.                        */
    bool rear_active = traction_state.mode4x4 || traction_state.axisRotation;

    if (fabs(effective_demand) <= TRACTION_ZERO_DEMAND_PCT) {
        /* Zero demand — apply BTS7960 active brake to hold vehicle */
        Motor_SetPWM(&motor_fl, BTS7960_BRAKE_PWM);
        Motor_SetPWM(&motor_fr, BTS7960_BRAKE_PWM);
        Motor_Enable(&motor_fl, 1);
        Motor_Enable(&motor_fr, 1);
        if (rear_active) {
            Motor_SetPWM(&motor_rl, BTS7960_BRAKE_PWM);
            Motor_SetPWM(&motor_rr, BTS7960_BRAKE_PWM);
            Motor_Enable(&motor_rl, 1);
            Motor_Enable(&motor_rr, 1);
        }
    } else {
        /* Non-zero demand — enable motors normally.
         * Do NOT disable based on wheel_scale: ABS sets scale=0.0
         * but the motor must stay enabled so the H-bridge can
         * actively brake (coast mode would lose control).             */
        Motor_Enable(&motor_fl, 1);
        Motor_Enable(&motor_fr, 1);
        if (rear_active) {
            Motor_Enable(&motor_rl, 1);
            Motor_Enable(&motor_rr, 1);
        }
    }

    /* Update state with sensor readings */
    traction_state.wheels[0].speedKmh = Wheel_GetSpeed_FL();
    traction_state.wheels[1].speedKmh = Wheel_GetSpeed_FR();
    traction_state.wheels[2].speedKmh = Wheel_GetSpeed_RL();
    traction_state.wheels[3].speedKmh = Wheel_GetSpeed_RR();
    for (uint8_t i = 0; i < 4; i++) {
        traction_state.wheels[i].currentA = Current_GetAmps(i);
        traction_state.wheels[i].tempC    = Temperature_Get(i);
        /* In 4x4 mode, per-wheel PWM uses 50/50 axle split (base_pwm/2) */
        uint16_t per_wheel_base = traction_state.mode4x4 ? (base_pwm / 2) : base_pwm;
        traction_state.wheels[i].pwm      = (uint16_t)(per_wheel_base * acker_diff[i] * safety_status.wheel_scale[i]);
        traction_state.wheels[i].reverse  = (dir < 0);
    }
}

void Traction_EmergencyStop(void)
{
    /* Emergency stop — cut all power immediately.
     *
     * We intentionally use EN=0 (coast) rather than BTS7960 active
     * brake here because in an emergency (overcurrent, overtemp,
     * short circuit) we must completely de-energise the H-bridges.
     * Active brake would keep the FETs conducting, which is unsafe
     * when the fault condition involves the power stage itself.
     * The relay shutdown sequence (safety_system.c) will also
     * physically disconnect motor power.                              */
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

    /* Reset pedal filter so emergency stop is immediate */
    pedal_ema         = 0.0f;
    pedal_ramped      = 0.0f;
    pedal_filter_init = 0;

    /* Reset dynamic braking */
    dynbrake_pct    = 0.0f;
    prev_demand_pct = 0.0f;
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
    /* Reject commands until Steering_Init() has established the relative
     * zero reference.  Without calibration the encoder position is
     * undefined and driving to any setpoint would be unsafe.            */
    if (!steering_calibrated) return;

    /* NaN/Inf guard — reject corrupt steering angle (security hardening) */
    angle_deg = sanitize_float(angle_deg, 0.0f);

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
    /* Steering_Init() has not been called yet — no encoder reference.
     * Keep the motor safely off until calibration is established.      */
    if (!steering_calibrated) {
        Steering_Neutralize();
        return;
    }

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

    int32_t encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    float measured = (float)encoder_count;

    /* steering_motor.cpp: kDeadbandDeg = 0.5f */
    float error = steering_pid.setpoint - measured;
    float absError = fabsf(error);
    if (absError < STEERING_DEADBAND_COUNTS) {
        /* Position within deadband — apply BTS7960 active brake to
         * hold the steering in place.  Chinese BTS7960 modules float
         * the motor when EN=HIGH + PWM=0, so PWM must be set to 100 %
         * to engage the brake FETs and prevent steering drift.        */
        Motor_SetPWM(&motor_steer, BTS7960_BRAKE_PWM);
        Motor_Enable(&motor_steer, 1);
        last_time = now;
        return;
    }

    float output   = PID_Compute(&steering_pid, measured, dt);

    if (output < -100.0f) output = -100.0f;
    if (output >  100.0f) output =  100.0f;

    /* Reduce steering aggressiveness in DEGRADED state.
     * Phase 12: per-level scaling via Safety_GetSteeringLimitFactor().
     * L1=85%, L2=70%, L3=60% (legacy was fixed 60%).
     * Traced to limp_mode.cpp drive-home philosophy: lower steering
     * torque reduces risk while preserving controllability.             */
    if (Safety_IsDegraded()) {
        output *= Safety_GetSteeringLimitFactor();
    }
    output = sanitize_float(output, 0.0f);

    uint16_t pwm = (uint16_t)(fabs(output) * PWM_PERIOD / 100.0f);
    int8_t direction = (output >= 0) ? 1 : -1;

    Motor_SetPWM(&motor_steer, pwm);
    Motor_SetDirection(&motor_steer, direction);
    Motor_Enable(&motor_steer, (fabs(output) > 1.0f));
    last_time = now;
}

float Steering_GetCurrentAngle(void)
{
    int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    return (float)cnt * 360.0f / (float)ENCODER_CPR;
}

bool Steering_IsCalibrated(void)
{
    return (steering_calibrated != 0);
}

/**
 * @brief  Mark the steering system as calibrated.
 *
 * Called by the automatic centering module (steering_centering.c) once
 * the physical center reference has been detected and the encoder has
 * been zeroed at that position.  After this call, Steering_SetAngle()
 * and Steering_ControlLoop() will accept commands.
 */
void Steering_SetCalibrated(void)
{
    steering_calibrated = 1;
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

    int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
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
    int32_t delta = count - enc_prev_count;
    if (delta < 0) delta = -delta;
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
 * @brief  Safely stop steering motor and apply active brake.
 *
 * Used when the encoder is faulted or calibration is not yet done:
 * we must NOT drive the motor toward any position because we have
 * no reliable feedback.  Apply BTS7960 active brake (PWM=100 %,
 * EN=HIGH) to hold the steering in place and prevent drift.
 * Chinese BTS7960 modules float the motor when PWM=0 + EN=HIGH,
 * so PWM must be at full duty to engage the brake FETs.
 */
void Steering_Neutralize(void)
{
    Motor_SetPWM(&motor_steer, BTS7960_BRAKE_PWM);
    Motor_Enable(&motor_steer, 1);
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