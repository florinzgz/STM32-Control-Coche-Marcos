/**
  ****************************************************************************
  * @file           : motor_control.c
  * @brief          : Motor control implementation with PWM, PID, and Ackermann
  ****************************************************************************
  */

#include "motor_control.h"
#include "ackermann.h"
#include "eps_params.h"
#include "steering_eps.h"
#include "gear_limits_store.h"
#include "drive_tuning_store.h"
#include "main.h"
#include "safety_system.h"
#include "sensor_manager.h"
#include "service_mode.h"
#include "motion_inhibit.h"
#include <math.h>
#include <stdbool.h>

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

/* ---- Smooth Driving Parameters (Tuning Section) ----
 *
 * These constants control the smooth-driving strategy for jerk-free,
 * vibration-free low-speed operation.  They do NOT affect the safety
 * architecture, CAN protocol, or hardware.
 *
 * A) Start/Stop Hysteresis
 *    DRIVE_ENTER_PCT: pedal must exceed this to transition from brake to drive.
 *    DRIVE_EXIT_PCT:  pedal must drop below this to return from drive to brake.
 *    The gap prevents rapid toggling at the brake/drive boundary.
 *
 * B) Motor Dead-Zone Compensation (Creep)
 *    MOTOR_DEADZONE_PCT: minimum PWM% output when drive is active.
 *    The motor has a physical dead zone below which it stalls.  When
 *    throttle demand is above DRIVE_ENTER_PCT, PWM jumps directly to
 *    MOTOR_DEADZONE_PCT and then maps linearly from there to 100%.
 *    This ensures the motor always receives enough torque to move.
 *
 * C) Brake→Drive Transition Ramp
 *    BRAKE_RELEASE_RAMP_PCT_S: max PWM rise rate when transitioning
 *    from brake (BTS7960 hold) to drive.  Prevents the abrupt jump
 *    from 100% brake duty to low drive duty.
 *
 * D) Coast vs Hold
 *    COAST_SPEED_THRESHOLD_KMH: above this speed, releasing the pedal
 *    enters coast (motors disabled, free-rolling) instead of immediate
 *    hold brake.  Below this speed, hold brake engages for hill hold.
 *    COAST_TIMEOUT_MS: maximum coast duration before hold brake engages.
 *
 * E) Jerk Limiter
 *    MAX_PWM_DELTA_PER_CYCLE: maximum absolute change in PWM counts
 *    per 10 ms control cycle.  This is a second-order rate limiter
 *    (limits acceleration of the demand ramp) to prevent mechanical
 *    jerk in the drivetrain.
 *
 * F) Low-Speed Stability
 *    CREEP_DEMAND_SMOOTHING_ALPHA: additional EMA coefficient applied
 *    only when demand is below CREEP_ZONE_PCT.  Smooths out ADC noise
 *    and ramp quantisation that cause micro-accelerations at creep speed.
 *    CREEP_ZONE_PCT: demand threshold below which extra smoothing applies.
 */
#define DRIVE_ENTER_PCT              3.0f   /* Pedal% to enter drive mode     */
#define DRIVE_EXIT_PCT               1.0f   /* Pedal% to return to brake/coast*/
#define MOTOR_DEADZONE_PCT           8.0f   /* Min PWM% when driving (creep)  */
#define BRAKE_RELEASE_RAMP_PCT_S     40.0f  /* Brake→drive transition rate    */
#define COAST_SPEED_THRESHOLD_KMH    2.0f   /* Coast above this speed (km/h)  */
#define COAST_SPEED_HYSTERESIS_KMH   0.5f   /* Hysteresis band for coast↔brake
                                              * transition: enter coast at
                                              * COAST_SPEED_THRESHOLD_KMH, exit
                                              * at THRESHOLD - HYSTERESIS.  This
                                              * prevents oscillation when speed
                                              * hovers near the threshold.      */
#define COAST_TIMEOUT_MS             3000U  /* Max coast duration (ms)        */
#define MAX_PWM_DELTA_PER_CYCLE      80U    /* Max PWM change per 10 ms cycle */
#define CREEP_DEMAND_SMOOTHING_ALPHA 0.08f  /* Extra EMA below creep zone     */
#define CREEP_ZONE_PCT               15.0f  /* Demand% threshold for creep    */

/* ---- Axis-rotation dynamic force distribution ----
 *
 * Applies a per-wheel PWM reduction factor in tank-turn mode only.
 * Update cadence is 50 ms (5 × 10 ms control cycles) to match INA226
 * current refresh and avoid oscillations from stale current samples.
 *
 * The scale is reduction-only:
 *   1.0  = no reduction
 *   0.75 = max reduction for a "free-spinning" wheel
 */
#define AXIS_ROT_UPDATE_CYCLES   5U
#define AXIS_ROT_SCALE_MIN       0.75f
#define AXIS_ROT_SCALE_MED       0.90f
#define AXIS_ROT_EMA_ALPHA       0.15f
#define AXIS_ROT_EMA_COMPLEMENT  (1.0f - AXIS_ROT_EMA_ALPHA)
#define AXIS_ROT_MIN_CURRENT_A   0.5f
#define AXIS_ROT_MIN_CUR_RATIO   0.2f
#define AXIS_ROT_MIN_RPM_AVG     0.01f
#define AXIS_ROT_SLIP_HI         1.35f
#define AXIS_ROT_SLIP_MED        1.15f
#define AXIS_ROT_WHEEL_COUNT     4U

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

/* ---- BTS7960 passive brake (design decision) ----
 *
 * BRAKE is implemented as EN=HIGH, RPWM=0, LPWM=0 (passive EM brake).
 * The BTS7960 internal logic then holds both motor terminals at the
 * same potential (low-side FETs ON), producing electromagnetic braking
 * by short-circuiting the motor windings.
 *
 * DECISION: Passive brake is sufficient for this application because:
 *   1) The vehicle mass is low (~50 kg + driver)
 *   2) The back-EMF at typical speeds (5-10 km/h) produces adequate
 *      braking torque through the short-circuit current
 *   3) Active brake (applying small opposing PWM) would generate heat
 *      in both the FETs and motor windings during sustained holds
 *
 * FALLBACK: If passive brake proves insufficient on steep slopes,
 * enable BRAKE_ACTIVE_FALLBACK and tune BRAKE_ACTIVE_MIN_PWM.  This
 * applies a small opposing duty on one channel to increase the braking
 * current.  Must be field-tested to determine the minimum effective
 * value (~5-10% is typical).
 *
 * Previous design (74HC external logic) used 100% PWM + DIR=1 for
 * brake; that is no longer needed because direction is encoded as
 * the choice of RPWM vs LPWM channel.                                */
#define BTS7960_BRAKE_PWM        0U   /* Both RPWM=0, LPWM=0 = passive brake */

/* Optional active brake fallback — disabled by default.
 * Enable by defining BRAKE_ACTIVE_FALLBACK=1 if passive brake is
 * insufficient in field testing.  BRAKE_ACTIVE_MIN_PWM is the duty
 * (in timer ticks, max PWM_PERIOD=4249) applied to LPWM during brake
 * to increase braking current.  RPWM remains 0.
 * Typical value: 5-10% of PWM_PERIOD (212-425 ticks).
 *
 * Runtime override: brake_active_override can be set at runtime (e.g.
 * via CAN service command) to enable active braking without
 * recompiling.  When non-zero AND BRAKE_ACTIVE_FALLBACK==0, the
 * runtime override value is used as the LPWM duty during BRAKE mode.
 * Set to 0 to disable; typical field-test value: 212-425 ticks.      */
#ifndef BRAKE_ACTIVE_FALLBACK
#define BRAKE_ACTIVE_FALLBACK    0
#endif
#if BRAKE_ACTIVE_FALLBACK
#define BRAKE_ACTIVE_MIN_PWM     212U  /* ~5% of 4249 — tune in field  */
#endif

/* ---- BTS7960 coast/brake behaviour (all motors symmetric) ----
 *
 * All five BTS7960 modules now have dedicated GPIO EN pins on GPIOC.
 * Motor_SetMode() selects the physical state explicitly:
 *
 *   MOTOR_MODE_COAST: PWM=0, EN=LOW → Hi-Z, motor free-spinning
 *   MOTOR_MODE_BRAKE: PWM=0, EN=HIGH → motor terminals shorted (hold)
 *   MOTOR_MODE_DRIVE: EN=HIGH, PWM=duty → motor driven
 *
 * Motor_SetSigned(0) always produces coast (EN=LOW) — used for
 * emergency stop and neutral ramp completion where de-energising
 * the H-bridge is the correct fail-safe action.
 *
 * Traction_Update() uses Motor_SetMode() for the traction phase
 * state machine to correctly distinguish brake from coast.
 *
 * No asymmetry compensation is needed.                                */
#define NEUTRAL_RAMP_DOWN_PCT_S 100.0f  /* Neutral entry ramp-down rate (%/s)  */

/* ---- Safe transition dead-time ----
 *
 * Minimum delay (µs) enforced when Motor_SetMode() changes the EN/PWM
 * state of a BTS7960 half-bridge.  This ensures the FETs in the driver
 * have fully settled before the new drive state is applied, preventing
 * transient shoot-through during direction changes or brake↔drive
 * transitions.
 *
 * At 20 kHz centre-aligned PWM the period is 50 µs.  5 µs (10 % of
 * one period) is well above the BTS7960 t_d(on)/t_d(off) spec of
 * ~0.2 µs and provides generous margin for the 74HC244 buffer
 * propagation delay (~15 ns).                                         */
#define SAFE_DEADTIME_US  5U

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
 *   GEAR_FORWARD  (D1) = 60 % max power — drive forward (low gear)
 *   GEAR_FORWARD_D2    = 100 % max power — full performance
 *
 * Boot default is GEAR_NEUTRAL (fail-safe) — see current_gear init.
 *   GEAR_REVERSE       = 60 % max power
 *
 * Safety_GetPowerLimitFactor() is applied separately upstream and
 * is NOT modified by this scaling.
 *
 * RUNTIME-CONFIGURABLE (R-2): the three fractions below are no longer
 * hard-coded.  They are seeded from the GEAR_LIMIT_*_DEFAULT_PCT macros
 * (which equal the historic compile-time values, so default behaviour is
 * unchanged) and may be overridden at runtime via Traction_SetGearLimits()
 * — sourced either from the gear_limits_store.c flash slot at boot or from
 * the CAN Engineering-menu service command.  The macros below are kept as
 * the single compile-time default source for RESTORE DEFAULTS.            */
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

/* ---- EPS high-speed assist fade ----
 * Steering assist is gradually reduced at higher speeds for safety.
 * Linear fade from 100% at EPS_HS_FADE_START_KMH to
 * EPS_HS_FADE_MIN_FACTOR at EPS_HS_FADE_END_KMH.                     */
#define EPS_HS_FADE_START_KMH    20.0f   /* Begin assist fade (km/h)    */
#define EPS_HS_FADE_END_KMH      30.0f   /* Full fade reached (km/h)    */
#define EPS_HS_FADE_MIN_FACTOR   0.5f    /* Minimum assist fraction      */

/* Motor structures */
typedef struct {
    TIM_HandleTypeDef *rpwm_timer;   /* Timer for RPWM (forward direction) */
    uint32_t           rpwm_channel;
    TIM_HandleTypeDef *lpwm_timer;   /* Timer for LPWM (reverse direction) */
    uint32_t           lpwm_channel;
    GPIO_TypeDef      *en_port;      /* Hardware EN GPIO port (GPIOC for all)    */
    uint16_t           en_pin;
    int8_t             direction;    /* Stored direction: 1=forward, -1=reverse  */
    int16_t            power;        /* Retained for ABI compatibility            */
    motor_mode_t       current_mode; /* Last mode set by Motor_SetMode() — used
                                      * for safe transition detection.  Initialised
                                      * to MOTOR_MODE_COAST (0) by zero-init.     */
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

/* ---- EPS torque-assist state ---- */
static float   eps_omega_filt     = 0.0f;   /* EMA-filtered angular velocity  */
static float   eps_prev_angle_deg = 0.0f;   /* Previous angle for derivative  */
static int16_t eps_prev_pwm_raw   = 0;      /* Previous PWM for slew-rate     */
static float   eps_motor_effort   = 0.0f;   /* Current motor output (%) for
                                              * encoder health frozen check    */

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

/* ---- Smooth driving state ---- */
typedef enum {
    TRAC_PHASE_BRAKE,   /* BTS7960 active brake — motors holding           */
    TRAC_PHASE_COAST,   /* Motors disabled — vehicle rolling freely         */
    TRAC_PHASE_DRIVE    /* Throttle active — motors driving                 */
} TractionPhase_t;

static TractionPhase_t trac_phase        = TRAC_PHASE_BRAKE;
static uint32_t        coast_start_tick  = 0;     /* When coast phase began */

/* ---- MOTION_INHIBIT_REASON instrumentation state ----
 * Latest bitfield + the effective demand / final PWM snapshot behind it.
 * Updated once per Traction_Update() at every exit path so telemetry
 * always reflects the current cycle.  Instrumentation only.              */
static uint16_t motion_inhibit_reason   = MOTION_INHIBIT_NONE;
static float    motion_effective_demand = 0.0f;
static uint8_t  motion_final_pwm_pct    = 0;
static uint16_t        prev_output_pwm[4] = {0};  /* Previous PWM per motor
                                                    * for jerk limiting      */
static float           brake_release_pct = 0.0f;  /* Ramp during brake→drive */
static float           creep_smooth_pct  = 0.0f;  /* Extra EMA for creep zone*/
static uint8_t         creep_smooth_init = 0;      /* First sample flag       */
static float           axis_rot_scale[AXIS_ROT_WHEEL_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
static uint8_t         axis_rot_update_ctr = 0;

/* ---- Neutral ramp-down state ----
 * When entering GEAR_NEUTRAL, PWM ramps down gradually instead of
 * cutting immediately to zero.  This prevents abrupt torque removal
 * and associated vehicle jerk.  The ramp is applied to all motors
 * and uses the same time-base as the pedal ramp limiter.            */
static float           neutral_ramp_pct   = 0.0f; /* Current ramp level (%)  */
static uint32_t        neutral_ramp_tick  = 0;     /* Timestamp for dt calc   */
static uint8_t         neutral_ramp_active = 0;    /* 1 = ramping down        */
static int8_t          neutral_ramp_dir   = 1;     /* Captured travel direction*/

/* ---- Gear position state ----
 * Safety default: boot in NEUTRAL.  The ESP32 shifter driver also boots
 * in NEUTRAL (esp32/src/shifter_input.cpp).  Booting both nodes in the
 * same fail-safe state eliminates the cross-node mismatch window that
 * existed when the STM32 defaulted to FORWARD before shifter resync.
 * While current_gear == GEAR_NEUTRAL, Traction_Update() and main.c
 * already force the traction demand to 0 (see main.c:483-484), so this
 * is the strictest fail-safe boot state.                                */
static GearPosition_t current_gear = GEAR_NEUTRAL;

/* ---- Runtime-configurable gear power limits (R-2) ----
 * Stored as percent (0..100) for an exact match with the Engineering-menu
 * UI and the CAN/flash representation; converted to a fraction at the
 * point of use.  Seeded from the compile-time defaults so a unit with no
 * valid flash slot behaves identically to the historic firmware.        */
static uint8_t gear_limit_d2_pct = GEAR_LIMIT_D2_DEFAULT_PCT;
static uint8_t gear_limit_d1_pct = GEAR_LIMIT_D1_DEFAULT_PCT;
static uint8_t gear_limit_r_pct  = GEAR_LIMIT_R_DEFAULT_PCT;

/* ---- Runtime-configurable per-gear acceleration RESPONSE profile ----
 * Stored as percent (0..100); applied in Traction_SetDemand() after EMA#2
 * and before the global ramp limiter.  Softens the demand target per gear
 * (positive demand only, clamped to <=100 % so it can never amplify).
 * Seeded from the compile-time response defaults so a unit with no valid
 * (or only a legacy power-only) flash slot has well-defined behaviour.   */
static uint8_t gear_response_d2_pct = GEAR_RESPONSE_D2_DEFAULT_PCT;
static uint8_t gear_response_d1_pct = GEAR_RESPONSE_D1_DEFAULT_PCT;
static uint8_t gear_response_r_pct  = GEAR_RESPONSE_R_DEFAULT_PCT;

/* ---- Runtime-configurable drive tuning (FASE 2) ----
 * Pedal ramp rates and motor dead-zone (creep) compensation, seeded with
 * the compile-time values so a unit with no valid flash slot behaves
 * EXACTLY like the historic firmware:
 *   accel_ramp   = PEDAL_RAMP_UP_PCT_S   (50  %/s)
 *   brake_ramp   = PEDAL_RAMP_DOWN_PCT_S (100 %/s)
 *   reverse_ramp = accel_ramp            (50  %/s, applied in GEAR_REVERSE)
 *   creep_enable = 1, creep_power = MOTOR_DEADZONE_PCT (8 %), creep_delay = 0 ms
 * The ramp rates feed the rate limiter in Traction_SetDemand(); the creep
 * fields feed the dead-zone compensation in Traction_Update().  Replacing
 * only the constants (not the fórmulas) keeps every other behaviour intact. */
static DriveTuning_t drive_tuning = {
    .accel_ramp   = DRIVE_ACCEL_RAMP_DEFAULT,
    .brake_ramp   = DRIVE_BRAKE_RAMP_DEFAULT,
    .reverse_ramp = DRIVE_REVERSE_RAMP_DEFAULT,
    .creep_enable = DRIVE_CREEP_ENABLE_DEFAULT,
    .creep_power  = DRIVE_CREEP_POWER_DEFAULT,
    .creep_delay  = DRIVE_CREEP_DELAY_DEFAULT,
};

/* ---- Creep-delay timer ----
 * Tracks how long the (positive) demand has been continuously above zero.
 * The dead-zone floor (creep_power) is applied only once this has exceeded
 * creep_delay ms; before that the output uses a plain proportional mapping
 * with no minimum floor.  With creep_delay == 0 (default) the floor is
 * applied immediately on the first positive sample, exactly as today.    */
static uint32_t creep_demand_start_tick = 0;
static bool     creep_demand_active     = false;

/* ---- Per-motor overtemp cutoff state ---- */
static bool motor_overtemp_cutoff[4] = {false, false, false, false};

/* ---- Runtime active-brake override ----
 * When BRAKE_ACTIVE_FALLBACK is 0 (compile-time passive brake), this
 * variable allows field testing of active braking without reflashing.
 * Set via Motor_SetBrakeActiveOverride().  When non-zero, the value
 * is applied as LPWM duty (timer ticks, max PWM_PERIOD) during
 * MOTOR_MODE_BRAKE.  RPWM remains 0, EN=HIGH.
 * Typical test values: 212 (~5%) to 425 (~10%) ticks.
 * 0 = passive brake (default).                                        */
static uint16_t brake_active_override = 0;

/* Ackermann-computed individual wheel angle setpoints (degrees).
 * Updated every time Steering_SetAngle() is called.               */
static float steer_fl_deg = 0.0f;
static float steer_fr_deg = 0.0f;

/* ---- Encoder fault detection state ----
 * The E6B2-CWZ6C encoder Z-index pulse (PB4/EXTI4) is used for
 * inter-revolution drift detection (see encoder_reader.c).
 * It does NOT control or override steering centering, which uses the
 * inductive LJ12A3 sensor on PB5.  The Z channel:
 *   1. Detects encoder slip / A/B noise in high-EMI environments.
 *   2. Records TIM2 position at each index pulse for diagnostics.
 *   3. Flags Encoder_Z_HasSlipped() when the inter-pulse delta deviates
 *      from ±ENCODER_CPR by more than ENC_Z_SLIP_THRESHOLD counts.
 * Steering control continues to use relative positioning zeroed at
 * Steering_Init(); no mechanical alignment constraint is assumed.
 */

/* Steering deadband in encoder counts (steering_motor.cpp: kDeadbandDeg = 0.5f)
 * 0.5° × 4800 counts/360° ≈ 6.67 counts
 * NOTE: kept for historical reference only; control-loop deadband is now
 *       applied in road-wheel degrees via STEERING_DEADBAND_DEG below.    */
#define STEERING_DEADBAND_COUNTS  (0.5f * (float)ENCODER_CPR / 360.0f)

/* Steering deadband in road-wheel degrees — absorbs ≈3° of mechanical
 * backlash downstream of the motor/reductor (1.8° ≈ 60 % of the slop).
 * Tuning: raise to 2.0° if chatter persists, lower to 1.5° if the
 * response feels dead; do NOT go below 1.0° (below backlash = unstable). */
#define STEERING_DEADBAND_DEG     1.8f
#define ENC_MAX_COUNTS       ((int32_t)((MAX_STEER_DEG * STEERING_GEAR_RATIO + 20.0f) * (float)ENCODER_CPR / 360.0f))
        /* Encoder is mounted on the steering column (volante), not on
         * the road-wheel output shaft.  Plausible range is the full
         * volante travel (MAX_STEER_DEG·STEERING_GEAR_RATIO ≈ ±350°)
         * plus 20° margin.  Any reading beyond this is mechanically
         * impossible and indicates corrupt counter / disconnect.        */
#define ENC_MAX_JUMP         ((int32_t)(100 * STEERING_GEAR_RATIO))
        /* Maximum plausible count change per 10 ms control cycle.
         * Original design intent was ~750 °/s at the controlled axis;
         * with the encoder mounted on the volante we scale by
         * STEERING_GEAR_RATIO so the same physical rate limit holds
         * (~648 counts/cycle ≈ 750 °/s volante, covers 500–1000 °/s
         * manual steering while still flagging impossible glitches).   */
#define ENC_FROZEN_TIMEOUT_MS 200
        /* If the motor is driving above ENC_MOTOR_ACTIVE_PCT and the
         * encoder has not changed for this long, declare frozen fault.   */
#define ENC_MOTOR_ACTIVE_PCT  10.0f
        /* Minimum |PID output| (%) to consider the motor actively
         * driving.  Below this the motor may legitimately be at rest.    */

static int32_t  enc_prev_count       = 0;
static uint32_t enc_last_change_tick = 0;
static uint8_t  enc_fault            = 0;   /* 0 = healthy, 1 = faulted */

extern TIM_HandleTypeDef htim1, htim2, htim3, htim8;

/* Private function prototypes */
static void Motor_SetSigned(Motor_t *motor, int16_t signed_pwm);
static void Motor_SetMode(Motor_t *motor, motor_mode_t mode, int16_t signed_pwm);
static float __attribute__((unused)) PID_Compute(PID_t *pid, float measured, float dt);
static void compute_ackermann_differential(float steer_deg, float diff_out[4]);
static void axis_rotation_reset_scale(void);
static void axis_rotation_update_scale(void);

/* ---- Microsecond busy-wait helper ----
 * Uses the Cortex-M4 DWT cycle counter (CYCCNT) for deterministic,
 * compiler-independent and clock-frequency-independent timing.
 *
 * DWT is initialised by Motor_Init() → DWT_Init().
 *
 * DEFENCE-IN-DEPTH — DWT guard flag:
 *   If delay_us() is ever called before DWT_Init() (e.g. after a warm
 *   reset where the call graph is re-entered unexpectedly), the guard
 *   flag falls through to a conservative NOP-loop fallback calibrated
 *   for 170 MHz / -O2.  This avoids a zero-delay or infinite-loop if
 *   CYCCNT is not yet running.
 *
 * OVERFLOW PROTECTION:
 *   delay_us() clamps the delay to MAX_DELAY_US (25 000 µs ≈ 25 ms)
 *   to prevent uint32_t overflow of (us * dwt_cycles_per_us).
 *   At 170 MHz: 25 000 × 170 = 4 250 000 — well within uint32_t range.
 *   Production code only uses SAFE_DEADTIME_US (5 µs), so this clamp
 *   is pure defence-in-depth.
 *
 * On HOST_TEST builds this is a no-op (no real hardware).
 * Only used for SAFE_DEADTIME_US (5 µs) — negligible CPU cost.     */

#define MAX_DELAY_US  25000U  /* Max delay: 25 ms — prevents overflow */

#ifndef HOST_TEST

/* Precomputed cycles-per-microsecond — set once by DWT_Init().
 * Avoids division on every delay_us() call.                         */
static uint32_t dwt_cycles_per_us = 170U;  /* Safe default for 170 MHz */

/* Guard flag: set to 1 by DWT_Init().  If delay_us() is called before
 * DWT_Init(), the NOP-loop fallback executes instead of reading an
 * uninitialised or stopped CYCCNT.                                  */
static volatile uint8_t dwt_initialized = 0U;

/* DWT initialisation — called once from Motor_Init().
 * Enables the trace unit and starts the cycle counter.
 * Safe to call multiple times (idempotent).                         */
static inline void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* Enable trace */
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;             /* Start counter */
    __DSB();  /* Ensure CYCCNTENA write completes before first CYCCNT read */
    dwt_cycles_per_us = SystemCoreClock / 1000000U;
    dwt_initialized = 1U;                              /* Arm the guard */
}

static inline void delay_us(uint32_t us)
{
    /* Clamp to prevent uint32_t overflow in cycle calculation */
    if (us > MAX_DELAY_US) us = MAX_DELAY_US;

    if (dwt_initialized) {
        /* Primary path: DWT cycle-counter busy-wait (deterministic) */
        uint32_t start  = DWT->CYCCNT;
        uint32_t cycles = us * dwt_cycles_per_us;
        while ((DWT->CYCCNT - start) < cycles) {
            __NOP();  /* Explicit busy-wait — prevents compiler elision */
        }
    } else {
        /* Fallback: conservative NOP loop for 170 MHz / -O2.
         * Less accurate than DWT but guarantees a non-zero delay.
         * 30 NOPs ≈ 1 µs at 170 MHz with -O2 (same calibration as
         * the original pre-DWT implementation).                     */
        for (volatile uint32_t i = 0; i < us * 30U; i++) {
            __NOP();
        }
    }
}
#else
static volatile uint8_t dwt_initialized = 0U;
static inline void DWT_Init(void)  { dwt_initialized = 1U; }
static inline void delay_us(uint32_t us) { (void)us; }
#endif

/* ==================================================================
 *  Initialization
 * ================================================================== */

void Motor_Init(void)
{
    /* ---- Initialise DWT cycle counter for delay_us() ----
     * Must be called before any Motor_SetMode() / Motor_SetSigned()
     * that uses delay_us() for dead-time enforcement.                  */
    DWT_Init();

    /* ---- motor_fl: RPWM = TIM1_CH1 (PA8), LPWM = TIM1_CH2 (PA9) ---- */
    /* Both channels on TIM1 → same UEV → overlap = 0                    */
    motor_fl.rpwm_timer   = &htim1;  motor_fl.rpwm_channel = TIM_CHANNEL_1;
    motor_fl.lpwm_timer   = &htim1;  motor_fl.lpwm_channel = TIM_CHANNEL_2;
    motor_fl.en_port      = GPIOC;   motor_fl.en_pin       = PIN_EN_FL;  /* PC5 */
    motor_fl.direction    = 0;

    /* ---- motor_fr: RPWM = TIM1_CH3 (PA10), LPWM = TIM1_CH4 (PC3) ---- */
    /* Both channels on TIM1 → same UEV → overlap = 0.
     * LPWM_FR moved from TIM15_CH1/PB14 to TIM1_CH4/PC3, eliminating
     * the cross-timer arrangement.  PB14 freed for LED_DIAG.            */
    motor_fr.rpwm_timer   = &htim1;  motor_fr.rpwm_channel = TIM_CHANNEL_3;
    motor_fr.lpwm_timer   = &htim1;  motor_fr.lpwm_channel = TIM_CHANNEL_4;
    motor_fr.en_port      = GPIOC;   motor_fr.en_pin       = PIN_EN_FR;  /* PC0 */
    motor_fr.direction    = 0;

    /* ---- motor_rl: RPWM = TIM8_CH1 (PC6), LPWM = TIM8_CH2 (PC7) ---- */
    /* Both channels on TIM8 → same UEV → overlap = 0                    */
    motor_rl.rpwm_timer   = &htim8;  motor_rl.rpwm_channel = TIM_CHANNEL_1;
    motor_rl.lpwm_timer   = &htim8;  motor_rl.lpwm_channel = TIM_CHANNEL_2;
    motor_rl.en_port      = GPIOC;   motor_rl.en_pin       = PIN_EN_RL;  /* PC1 */
    motor_rl.direction    = 0;

    /* ---- motor_rr: RPWM = TIM8_CH3 (PC8), LPWM = TIM8_CH4 (PC9) ---- */
    /* Both channels on TIM8 → same UEV → overlap = 0                    */
    motor_rr.rpwm_timer   = &htim8;  motor_rr.rpwm_channel = TIM_CHANNEL_3;
    motor_rr.lpwm_timer   = &htim8;  motor_rr.lpwm_channel = TIM_CHANNEL_4;
    motor_rr.en_port      = GPIOC;   motor_rr.en_pin       = PIN_EN_RR; /* PC2 */
    motor_rr.direction    = 0;

    /* ---- motor_steer: RPWM = TIM3_CH1 (PA6), LPWM = TIM3_CH2 (PA7) ---- */
    /* Both channels on TIM3 → same UEV → overlap = 0                      */
    motor_steer.rpwm_timer  = &htim3; motor_steer.rpwm_channel = TIM_CHANNEL_1;
    motor_steer.lpwm_timer  = &htim3; motor_steer.lpwm_channel = TIM_CHANNEL_2;
    motor_steer.en_port     = GPIOC;  motor_steer.en_pin       = PIN_EN_STEER; /* PC4 */
    motor_steer.direction   = 0;

    /* C2 HARDENING: Force all EN pins LOW before starting any PWM timer.
     * This is defence-in-depth: MX_GPIO_Init() already set them LOW,
     * but between GPIO init and here, MX_TIM*_Init() may have enabled
     * timer clocks.  Reconfirming EN=LOW here guarantees no motor can
     * receive torque until Motor_SetSigned/Motor_SetMode explicitly
     * commands it after the timers are started.                         */
    HAL_GPIO_WritePin(GPIOC, PIN_EN_FL | PIN_EN_FR | PIN_EN_RL
                                       | PIN_EN_RR | PIN_EN_STEER,
                      GPIO_PIN_RESET);

    /* ---- Start TIM1 channels: FL (CH1/CH2) and FR (CH3/CH4) ---- */
    /* HAL_TIM_PWM_Start re-enables MOE (TIM1 is advanced; MOE was cleared
     * by BREAK2 config with AutomaticOutput=DISABLE).                  */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  /* RPWM_FL  */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  /* LPWM_FL  */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  /* RPWM_FR  */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  /* LPWM_FR  */

    /* ---- Start TIM8 channels: RL (CH1/CH2) and RR (CH3/CH4) ---- */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  /* RPWM_RL  */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  /* LPWM_RL  */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);  /* RPWM_RR  */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);  /* LPWM_RR  */

    /* ---- Start TIM3 channels: STEER (CH1/CH2) ---- */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  /* RPWM_STEER */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  /* LPWM_STEER */

    /* ---- Quadrature encoder for steering ---- */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    /* ---- Safe initial state: all motors in coast (RPWM=0, LPWM=0, EN=LOW) ----
     * Coast is chosen (not brake) because at startup we want all H-bridges
     * fully de-energised until the traction state machine commands otherwise. */
    Motor_SetMode(&motor_fl,    MOTOR_MODE_COAST, 0);
    Motor_SetMode(&motor_fr,    MOTOR_MODE_COAST, 0);
    Motor_SetMode(&motor_rl,    MOTOR_MODE_COAST, 0);
    Motor_SetMode(&motor_rr,    MOTOR_MODE_COAST, 0);
    Motor_SetMode(&motor_steer, MOTOR_MODE_COAST, 0);
}

static void axis_rotation_reset_scale(void)
{
    axis_rot_update_ctr = 0U;
    for (uint8_t i = 0; i < AXIS_ROT_WHEEL_COUNT; i++) {
        axis_rot_scale[i] = 1.0f;
    }
}

static void axis_rotation_update_scale(void)
{
    axis_rot_update_ctr++;
    if (axis_rot_update_ctr < AXIS_ROT_UPDATE_CYCLES) {
        return;
    }
    axis_rot_update_ctr = 0U;

    /* ---- Fail-safe: if any wheel sensor is stale, do NOT compute partial
     * averages that would bias the reference.  Instead relax all scales
     * smoothly toward 1.0 via EMA and return early.  This prevents a stale
     * wheel's frozen scale from creating persistent lateral torque asymmetry. */
    for (uint8_t i = 0; i < AXIS_ROT_WHEEL_COUNT; i++) {
        if (Wheel_IsStale(i)) {
            for (uint8_t j = 0; j < AXIS_ROT_WHEEL_COUNT; j++) {
                axis_rot_scale[j] = AXIS_ROT_EMA_ALPHA * 1.0f
                                  + AXIS_ROT_EMA_COMPLEMENT * axis_rot_scale[j];
            }
            return;
        }
    }

    float rpm[AXIS_ROT_WHEEL_COUNT];
    float cur[AXIS_ROT_WHEEL_COUNT];
    float rpm_sum = 0.0f;
    float cur_sum = 0.0f;

    rpm[MOTOR_FL] = Wheel_GetSpeed_FL();
    rpm[MOTOR_FR] = Wheel_GetSpeed_FR();
    rpm[MOTOR_RL] = Wheel_GetSpeed_RL();
    rpm[MOTOR_RR] = Wheel_GetSpeed_RR();

    for (uint8_t i = 0; i < AXIS_ROT_WHEEL_COUNT; i++) {
        cur[i] = Current_GetAmps(i);
        rpm_sum += rpm[i];
        cur_sum += cur[i];
    }

    float inv_count = 1.0f / (float)AXIS_ROT_WHEEL_COUNT;
    float rpm_avg = rpm_sum * inv_count;
    float cur_avg = cur_sum * inv_count;

    if (rpm_avg < AXIS_ROT_MIN_RPM_AVG || cur_avg < AXIS_ROT_MIN_CURRENT_A) {
        /* Low-demand state: relax all scales smoothly toward 1.0 via EMA
         * instead of returning without update.  This prevents scales from
         * freezing at a reduced value (e.g. 0.75) when current or RPM drops
         * during throttle easing, slow rotation entry, or load transitions. */
        for (uint8_t i = 0; i < AXIS_ROT_WHEEL_COUNT; i++) {
            axis_rot_scale[i] = AXIS_ROT_EMA_ALPHA * 1.0f
                              + AXIS_ROT_EMA_COMPLEMENT * axis_rot_scale[i];
        }
        return;
    }

    for (uint8_t i = 0; i < AXIS_ROT_WHEEL_COUNT; i++) {
        float rpm_ratio = rpm[i] / rpm_avg;
        float cur_ratio = cur[i] / cur_avg;
        /* Higher slip_metric => wheel spins faster relative to load current
         * (free-spinning / lower traction), therefore reduce PWM scale. */
        float slip_metric = rpm_ratio / fmaxf(cur_ratio, AXIS_ROT_MIN_CUR_RATIO);

        float target_scale = 1.0f;
        if (slip_metric > AXIS_ROT_SLIP_HI) {
            target_scale = AXIS_ROT_SCALE_MIN;
        } else if (slip_metric > AXIS_ROT_SLIP_MED) {
            target_scale = AXIS_ROT_SCALE_MED;
        }

        axis_rot_scale[i] = AXIS_ROT_EMA_ALPHA * target_scale
                          + AXIS_ROT_EMA_COMPLEMENT * axis_rot_scale[i];

        axis_rot_scale[i] = fminf(fmaxf(axis_rot_scale[i], AXIS_ROT_SCALE_MIN), 1.0f);
    }
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
    dynbrake_last_tick = HAL_GetTick();

    /* Reset smooth driving state */
    trac_phase        = TRAC_PHASE_BRAKE;
    coast_start_tick  = 0;
    brake_release_pct = 0.0f;
    creep_smooth_pct  = 0.0f;
    creep_smooth_init = 0;
    creep_demand_active     = false;
    creep_demand_start_tick = 0;
    neutral_ramp_pct   = 0.0f;
    neutral_ramp_active = 0;
    neutral_ramp_dir   = 1;
    for (uint8_t j = 0; j < 4; j++) {
        prev_output_pwm[j] = 0;
    }
    axis_rotation_reset_scale();

    /* Reset demand anomaly detection state */
    prev_raw_demand      = 0.0f;
    prev_raw_demand_tick = 0;
    frozen_pedal_value   = 0.0f;
    frozen_pedal_tick    = 0;
    frozen_pedal_speed   = 0.0f;
    anomaly_init         = 0;

    /* Default gear to NEUTRAL (fail-safe).
     * The ESP32 shifter driver also boots in NEUTRAL, so both nodes
     * start in the same state and no forward assumption exists before
     * the first CMD_MODE gear update arrives over CAN.                 */
    current_gear = GEAR_NEUTRAL;
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

    /* Initialise EPS torque-assist state */
    eps_omega_filt     = 0.0f;
    eps_prev_angle_deg = 0.0f;
    eps_prev_pwm_raw   = 0;
    eps_motor_effort   = 0.0f;
    EPS_Params_Init();

    /* Initialise encoder health tracking */
    enc_prev_count       = 0;
    enc_last_change_tick = HAL_GetTick();
    enc_fault            = 0;

    /* Initialise the EPS local state authority (STARTING, owner NONE). */
    Steering_EpsInit();
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
        frozen_pedal_speed   = sanitize_float(
                                (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                                 Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f, 0.0f);
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
            SystemState_t st_anom = Safety_GetState();
            if (st_anom == SYS_STATE_ACTIVE) {
                Safety_SetState(SYS_STATE_DEGRADED);
                Safety_SetDegradedLevel(DEGRADED_L1,
                                        DEGRADED_REASON_DEMAND_ANOMALY);
            } else if (st_anom == SYS_STATE_DEGRADED) {
                Safety_SetDegradedLevel(DEGRADED_L2,
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
        frozen_pedal_speed = sanitize_float(
                              (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                               Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f, 0.0f);
    } else if ((now_anom - frozen_pedal_tick) > FROZEN_PEDAL_TIMEOUT_MS) {
        /* Pedal has been frozen — check speed divergence */
        float current_speed = sanitize_float(
                                (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                                 Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f, 0.0f);
        if (fabsf(current_speed - frozen_pedal_speed) > FROZEN_PEDAL_SPEED_DELTA_KMH) {
            /* Speed changed significantly while pedal frozen → anomaly */
            Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
            SystemState_t st_frz = Safety_GetState();
            if (st_frz == SYS_STATE_ACTIVE) {
                Safety_SetState(SYS_STATE_DEGRADED);
                Safety_SetDegradedLevel(DEGRADED_L1,
                                        DEGRADED_REASON_DEMAND_ANOMALY);
            } else if (st_frz == SYS_STATE_DEGRADED) {
                Safety_SetDegradedLevel(DEGRADED_L2,
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
        /* Seed ramp to 0 (not to throttlePct) so the ramp limiter
         * smoothly brings up demand after emergency stop / recovery.
         * Without this, a driver holding the pedal during SAFE→ACTIVE
         * recovery would cause an instant torque spike (P1 fix).      */
        pedal_ramped    = 0.0f;
        pedal_last_tick = HAL_GetTick();
        pedal_filter_init = 1;
    } else {
        pedal_ema = PEDAL_EMA_ALPHA * throttlePct
                  + (1.0f - PEDAL_EMA_ALPHA) * pedal_ema;
    }

    /* ---- A.1) Per-gear acceleration RESPONSE profile ----
     * Applied to the EMA output, AFTER EMA#2 and BEFORE the ramp limiter
     * (the exact point recommended by the pedal→PWM audit).  The factor
     * softens the demand target the ramp chases, making acceleration more
     * progressive in the selected gear.  Strictly a "soften only" stage:
     *   - applied to POSITIVE demand only (reverse direction is handled
     *     downstream in Traction_Update(); demand here is always positive);
     *   - the factor is clamped to <= 1.0 so it can NEVER amplify demand;
     *   - the EMA state (pedal_ema) is NOT mutated, only the local target,
     *     so the noise filter and frozen/step anomaly detection upstream
     *     are completely unaffected.
     * This stage does not touch dynamic braking (negative demand), ABS/TCS
     * wheel_scale[], obstacle_scale, the power limits, SAFE or LIMP_HOME. */
    float target = pedal_ema;
    if (target > 0.0f) {
        float resp;
        if (current_gear == GEAR_FORWARD_D2) {
            resp = (float)gear_response_d2_pct * 0.01f;
        } else if (current_gear == GEAR_REVERSE) {
            resp = (float)gear_response_r_pct * 0.01f;
        } else {
            resp = (float)gear_response_d1_pct * 0.01f;
        }
        if (resp > 1.0f) resp = 1.0f;   /* never amplify */
        if (resp < 0.0f) resp = 0.0f;   /* defensive floor */
        target *= resp;
    }

    /* ---- B) Ramp rate limiter (applied after EMA + response profile) ----
     * Runtime-configurable rates (FASE 2).  Ramp-up uses accel_ramp, except
     * in GEAR_REVERSE where reverse_ramp is used instead; ramp-down always
     * uses brake_ramp.  Defaults (50 / 100 / 50) reproduce the historic
     * PEDAL_RAMP_UP_PCT_S / PEDAL_RAMP_DOWN_PCT_S behaviour exactly.        */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - pedal_last_tick) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;   /* guard against zero / tiny dt */
    pedal_last_tick = now;

    float diff   = target - pedal_ramped;

    if (diff > 0.0f) {
        /* Accelerating: slower ramp.  Reverse gear uses its own ramp-up. */
        float up_rate = (current_gear == GEAR_REVERSE)
                          ? (float)drive_tuning.reverse_ramp
                          : (float)drive_tuning.accel_ramp;
        float max_up = up_rate * dt;
        if (diff > max_up) diff = max_up;
    } else {
        /* Decelerating: faster ramp */
        float max_down = (float)drive_tuning.brake_ramp * dt;
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
    if (!enable && traction_state.axisRotation) {
        axis_rotation_reset_scale();
    }
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

/* ---- Runtime-configurable gear power limits (R-2) ---------------- */

bool Traction_ValidateGearLimits(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct)
{
    /* Delegate to the single source of truth in gear_limits_store so the
     * CAN handler, the flash loader and the motor controller all agree.  */
    return GearLimitsStore_Validate(d2_pct, d1_pct, r_pct);
}

bool Traction_SetGearLimits(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct)
{
    if (!Traction_ValidateGearLimits(d2_pct, d1_pct, r_pct))
        return false;
    /* Single-byte writes are atomic on Cortex-M4; the next Traction_Update()
     * cycle picks up the new fractions.  No effect on ramp limiter or the
     * ABS/TCS per-wheel scaling applied downstream.                        */
    gear_limit_d2_pct = d2_pct;
    gear_limit_d1_pct = d1_pct;
    gear_limit_r_pct  = r_pct;
    return true;
}

void Traction_GetGearLimits(uint8_t *d2_pct, uint8_t *d1_pct, uint8_t *r_pct)
{
    if (d2_pct) *d2_pct = gear_limit_d2_pct;
    if (d1_pct) *d1_pct = gear_limit_d1_pct;
    if (r_pct)  *r_pct  = gear_limit_r_pct;
}

/* ---- Runtime-configurable per-gear acceleration RESPONSE profile ---- */

bool Traction_ValidateGearResponse(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct)
{
    /* Delegate to the single source of truth in gear_limits_store so the
     * CAN handler, the flash loader and the motor controller all agree.  */
    return GearLimitsStore_ValidateResponse(d2_pct, d1_pct, r_pct);
}

bool Traction_SetGearResponse(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct)
{
    if (!Traction_ValidateGearResponse(d2_pct, d1_pct, r_pct))
        return false;
    /* Single-byte writes are atomic on Cortex-M4; the next Traction_SetDemand()
     * call picks up the new factors.  No effect on the EMA state, the ramp
     * limiter constants, dynamic braking, or the ABS/TCS scaling.          */
    gear_response_d2_pct = d2_pct;
    gear_response_d1_pct = d1_pct;
    gear_response_r_pct  = r_pct;
    return true;
}

void Traction_GetGearResponse(uint8_t *d2_pct, uint8_t *d1_pct, uint8_t *r_pct)
{
    if (d2_pct) *d2_pct = gear_response_d2_pct;
    if (d1_pct) *d1_pct = gear_response_d1_pct;
    if (r_pct)  *r_pct  = gear_response_r_pct;
}

/* ---- Runtime-configurable drive tuning (FASE 2) ------------------- */

bool Traction_ValidateDriveTuning(const DriveTuning_t *t)
{
    /* Delegate to the single source of truth in drive_tuning_store so the
     * CAN handler, the flash loader and the motor controller all agree.   */
    return DriveTuningStore_Validate(t);
}

bool Traction_SetDriveTuning(const DriveTuning_t *t)
{
    /* Reject out-of-range sets and KEEP the previous values (FASE 4). */
    if (!Traction_ValidateDriveTuning(t))
        return false;
    /* The struct is small POD; the next Traction_SetDemand()/Update() cycle
     * picks up the new values.  No effect on EMA state, ABS/TCS scaling or
     * the gear power/response pipeline.                                     */
    drive_tuning = *t;
    return true;
}

void Traction_GetDriveTuning(DriveTuning_t *out)
{
    if (out) *out = drive_tuning;
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

    /* Sanitize input: NaN/Inf would bypass all comparisons below */
    steer_deg = sanitize_float(steer_deg, 0.0f);

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

/* ---- MOTION_INHIBIT_REASON snapshot helper (instrumentation only) ----
 * Builds a MotionInhibitInputs snapshot from the values the traction/safety
 * pipeline has already computed and stores the resulting bitfield.  Called
 * at every exit path of Traction_Update() so 0x315 telemetry reflects the
 * current cycle.  Never alters control state.                             */
static void Traction_UpdateMotionInhibit(float effective_demand_pct,
                                         uint16_t final_pwm_max_ticks)
{
    SystemState_t st = Safety_GetState();

    MotionInhibitInputs mi;
    mi.state        = (uint8_t)st;
    mi.power_ready  = Safety_IsPowerReady();
    mi.gear         = (uint8_t)current_gear;
    mi.gear_park    = (uint8_t)GEAR_PARK;
    mi.gear_neutral = (uint8_t)GEAR_NEUTRAL;
    mi.state_safe   = (uint8_t)SYS_STATE_SAFE;
    mi.state_error  = (uint8_t)SYS_STATE_ERROR;
    mi.obstacle_forward_blocked = Obstacle_IsForwardBlocked();
    mi.forward_gear = (current_gear == GEAR_FORWARD ||
                       current_gear == GEAR_FORWARD_D2);
    mi.demand_pct           = traction_state.demandPct;
    mi.effective_demand_pct = effective_demand_pct;
    mi.final_pwm_max        = final_pwm_max_ticks;
    mi.degraded_level       = (st == SYS_STATE_DEGRADED)
                                ? (uint8_t)Safety_GetDegradedLevel() : 0U;

    /* ---- Additional observability signals (read-only, no policy) --------
     * All are pre-computed elsewhere; we only classify a snapshot.  The
     * per-wheel "safety scale zero" is derived from the safety_status the
     * pipeline already applied — a value of ~0 on every wheel means the
     * ABS/TCS/driver-enable/safety scaling collapsed traction to zero.     */
    mi.startup_inhibit   = Startup_IsInhibited();
    mi.pedal_fault       = !Pedal_IsPlausible();
    bool all_scale_zero = true;
    for (uint8_t i = 0; i < 4; i++) {
        if (safety_status.wheel_scale[i] > 0.001f) { all_scale_zero = false; break; }
    }
    mi.safety_scale_zero = all_scale_zero;
    Safety_Error_t err   = Safety_GetError();
    mi.battery_cutoff    = (err == SAFETY_ERROR_BATTERY_UV_WARNING  ||
                            err == SAFETY_ERROR_BATTERY_UV_CRITICAL ||
                            err == SAFETY_ERROR_BATTERY_OV_WARNING  ||
                            err == SAFETY_ERROR_BATTERY_OV_CRITICAL);
    mi.thermal_overcurrent = (err == SAFETY_ERROR_OVERTEMP ||
                              err == SAFETY_ERROR_OVERCURRENT);
    mi.service_disabled  = !ServiceMode_IsEnabled(MODULE_RELAY_TRAC);

    motion_inhibit_reason   = MotionInhibit_Evaluate(&mi);
    motion_effective_demand = effective_demand_pct;
    motion_final_pwm_pct    = (uint8_t)((final_pwm_max_ticks * 100U) / PWM_PERIOD);
}

uint16_t Traction_GetMotionInhibit(void)
{
    return motion_inhibit_reason;
}

float Traction_GetEffectiveDemandPct(void)
{
    return motion_effective_demand;
}

uint8_t Traction_GetFinalPwmPct(void)
{
    return motion_final_pwm_pct;
}

/* AUDIT A: per-wheel FINAL PWM duty actually written to the BTS7960 this
 * cycle, expressed 0..100 %.  Single source of truth is
 * traction_state.wheels[i].pwm, which is set from the resolved
 * desired_en[]/desired_pwm[] (post jerk limiter + DRIVE/BRAKE/COAST) in
 * Traction_Update() and forced to 0 in Park/Neutral/coast paths.  Used by
 * CAN 0x20C (STATUS_WHEEL_EFFORT).  Returns 0 for an out-of-range index. */
uint8_t Traction_GetWheelFinalPwmPct(uint8_t wheel)
{
    if (wheel >= 4U) return 0U;
    uint32_t pct = ((uint32_t)traction_state.wheels[wheel].pwm * 100U) / PWM_PERIOD;
    if (pct > 100U) pct = 100U;
    return (uint8_t)pct;
}

void Traction_Update(void)
{
    /* --- Power-ready gate ---
     * Do NOT drive any motor output until the relay power-up sequence
     * has completed.  During the ~70 ms window between entering ACTIVE
     * and RELAY_SEQ_COMPLETE, motors would attempt to run with partial
     * or no supply voltage — causing unpredictable BTS7960 behaviour,
     * possible overcurrent spikes, and false relay-health alarms.
     *
     * This check is a pre-condition gate: all downstream motor output
     * (park hold, traction, coast, brake) is suppressed until
     * Safety_IsPowerReady() returns true.  The relay health check
     * (Safety_CheckRelayHealth) still runs independently and provides
     * post-hoc fault detection after power is established.
     *
     * SAFE/ERROR states bypass this gate because they already force
     * all outputs to zero and call Relay_PowerDown().                  */
    {
        SystemState_t st = Safety_GetState();
        if (st != SYS_STATE_SAFE && st != SYS_STATE_ERROR &&
            !Safety_IsPowerReady()) {
            Traction_UpdateMotionInhibit(0.0f, 0);  /* instrumentation */
            return;  /* Relays not yet settled — suppress all motor output */
        }
    }

    /* --- Gear P: Park Hold ---
     * Apply a PASSIVE electromagnetic brake via the H-bridge to simulate a
     * parking lock: both PWM channels at 0 with EN asserted causes the
     * BTS7960 to short the motor terminals (MOTOR_MODE_BRAKE).  This holds
     * the vehicle against rolling WITHOUT injecting any driving torque, so
     * the wheels are never actively driven while in Park.  No throttle
     * demand is accepted.  Because no drive current is forced through the
     * motor at standstill, the current/temperature derating that a forward
     * active-hold required is unnecessary here.
     * Park hold is released (coast) in SAFE/ERROR states (safety override). */
    if (current_gear == GEAR_PARK) {
        SystemState_t st = Safety_GetState();
        if (st == SYS_STATE_SAFE || st == SYS_STATE_ERROR) {
            /* Safety override — release park hold (free coast) */
            Motor_SetMode(&motor_fl, MOTOR_MODE_COAST, 0);
            Motor_SetMode(&motor_fr, MOTOR_MODE_COAST, 0);
            Motor_SetMode(&motor_rl, MOTOR_MODE_COAST, 0);
            Motor_SetMode(&motor_rr, MOTOR_MODE_COAST, 0);
        } else {
            /* Passive brake: short the motor terminals (RPWM=LPWM=0,
             * EN=HIGH).  No forward/active torque is applied.            */
            Motor_SetMode(&motor_fl, MOTOR_MODE_BRAKE, 0);
            Motor_SetMode(&motor_fr, MOTOR_MODE_BRAKE, 0);
            Motor_SetMode(&motor_rl, MOTOR_MODE_BRAKE, 0);
            Motor_SetMode(&motor_rr, MOTOR_MODE_BRAKE, 0);
        }

        /* Update sensor readings even in park */
        traction_state.wheels[0].speedKmh = Wheel_GetSpeed_FL();
        traction_state.wheels[1].speedKmh = Wheel_GetSpeed_FR();
        traction_state.wheels[2].speedKmh = Wheel_GetSpeed_RL();
        traction_state.wheels[3].speedKmh = Wheel_GetSpeed_RR();
        for (uint8_t i = 0; i < 4; i++) {
            traction_state.wheels[i].currentA = Current_GetAmps(i);
            traction_state.wheels[i].tempC    = Temperature_Get(i);
            traction_state.wheels[i].pwm      = 0U;  /* Park = BRAKE/COAST → 0 % push */
        }
        Traction_UpdateMotionInhibit(0.0f, 0);  /* Park — no drive */
        return;
    }

    /* --- Gear N: Neutral / Coast ---
     * All motors transition to full coast mode with a smooth ramp-down.
     * No active braking, no holding torque — wheels spin freely once
     * the ramp completes.  Dynamic braking is also disabled.
     *
     * The ramp-down prevents abrupt torque removal when shifting from
     * a driving gear to Neutral while the vehicle is in motion.        */
    if (current_gear == GEAR_NEUTRAL) {
        /* Initialise ramp on first cycle in Neutral */
        if (!neutral_ramp_active) {
            /* Capture the highest current motor output as ramp start.
             * This prevents a ramp from 100% when motors were at 10%. */
            float max_pct = 0.0f;
            for (uint8_t j = 0; j < 4; j++) {
                float pct = (float)prev_output_pwm[j] * 100.0f / (float)PWM_PERIOD;
                if (pct > max_pct) max_pct = pct;
            }
            neutral_ramp_pct   = max_pct;
            neutral_ramp_tick  = HAL_GetTick();
            neutral_ramp_active = 1;
            /* Capture travel direction from the motor hardware state at
             * the instant of gear change.  motor_fl.direction reflects
             * the most recent Motor_SetSigned write (1=fwd, -1=rev, 0=stop).
             * Using the motor struct avoids relying on a single wheel's
             * telemetry reverse flag which may not represent all motors.  */
            neutral_ramp_dir = (motor_fl.direction != 0) ? motor_fl.direction : 1;
        }

        /* Compute dt and ramp down */
        uint32_t now_n = HAL_GetTick();
        float dt_n = (float)(now_n - neutral_ramp_tick) / 1000.0f;
        if (dt_n < 0.001f) dt_n = 0.001f;
        neutral_ramp_tick = now_n;

        if (neutral_ramp_pct > 0.0f) {
            neutral_ramp_pct -= NEUTRAL_RAMP_DOWN_PCT_S * dt_n;
            if (neutral_ramp_pct < 0.0f) neutral_ramp_pct = 0.0f;
        }

        if (neutral_ramp_pct > 0.0f) {
            /* Still ramping — apply diminishing PWM to all motors.
             * Direction uses the captured travel direction so torque
             * matches the vehicle's inertia during the entire ramp.  */
            uint16_t ramp_pwm = (uint16_t)(neutral_ramp_pct * (float)PWM_PERIOD / 100.0f);
            Motor_SetSigned(&motor_fl, (int16_t)((int32_t)neutral_ramp_dir * ramp_pwm));
            Motor_SetSigned(&motor_fr, (int16_t)((int32_t)neutral_ramp_dir * ramp_pwm));
            Motor_SetSigned(&motor_rl, (int16_t)((int32_t)neutral_ramp_dir * ramp_pwm));
            Motor_SetSigned(&motor_rr, (int16_t)((int32_t)neutral_ramp_dir * ramp_pwm));
        } else {
            /* Ramp complete — full coast */
            Motor_SetSigned(&motor_fl, 0);
            Motor_SetSigned(&motor_fr, 0);
            Motor_SetSigned(&motor_rl, 0);
            Motor_SetSigned(&motor_rr, 0);
        }

        /* Reset dynamic braking state so it doesn't spike on gear change */
        dynbrake_pct    = 0.0f;
        prev_demand_pct = 0.0f;

        /* Reset smooth driving state so phase doesn't carry over */
        trac_phase        = TRAC_PHASE_BRAKE;
        brake_release_pct = 0.0f;
        creep_smooth_init = 0;
        for (uint8_t j = 0; j < 4; j++) prev_output_pwm[j] = 0;

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
        Traction_UpdateMotionInhibit(0.0f, 0);  /* Neutral — coast */
        return;
    }

    /* Leaving Neutral — clear ramp state */
    neutral_ramp_active = 0;

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

    /* In LIMP_HOME: limited regen braking (reduced effort) */
    bool limp_home_braking = (sys_st == SYS_STATE_LIMP_HOME);

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
        float avg_speed = sanitize_float(
                            (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                             Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f, 0.0f);
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

        /* Limited regen braking in LIMP_HOME — cap at LIMP_HOME limit */
        if (limp_home_braking) {
            target_brake *= LIMP_HOME_TORQUE_LIMIT_FACTOR;
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
            gear_scale = (float)gear_limit_d2_pct * 0.01f;
        } else if (current_gear == GEAR_REVERSE) {
            gear_scale = (float)gear_limit_r_pct * 0.01f;
        } else {
            gear_scale = (float)gear_limit_d1_pct * 0.01f;
        }
        effective_demand *= gear_scale;
    }

    /* ---- LIMP_HOME speed cap enforcement ----
     * Hard speed limit at walking pace.  When current speed exceeds
     * LIMP_HOME_SPEED_LIMIT_KMH, reduce demand toward zero to prevent
     * further acceleration.  This is a defence-in-depth layer on top
     * of the 20% torque limit applied upstream.                        */
    if (sys_st == SYS_STATE_LIMP_HOME && effective_demand > 0.0f) {
        float avg_spd = sanitize_float(
                          (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                           Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f, 0.0f);
        if (avg_spd > LIMP_HOME_SPEED_LIMIT_KMH) {
            effective_demand = 0.0f;
        } else if (avg_spd > (LIMP_HOME_SPEED_LIMIT_KMH * 0.8f)) {
            /* Progressive reduction as speed approaches limit */
            float headroom = (LIMP_HOME_SPEED_LIMIT_KMH - avg_spd)
                           / (LIMP_HOME_SPEED_LIMIT_KMH * 0.2f);
            if (headroom < 0.0f) headroom = 0.0f;
            if (headroom > 1.0f) headroom = 1.0f;
            effective_demand *= headroom;
        }
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
        {
            SystemState_t st_rng = Safety_GetState();
            if (st_rng == SYS_STATE_ACTIVE) {
                Safety_SetState(SYS_STATE_DEGRADED);
                Safety_SetDegradedLevel(DEGRADED_L1,
                                        DEGRADED_REASON_DEMAND_ANOMALY);
            } else if (st_rng == SYS_STATE_DEGRADED) {
                Safety_SetDegradedLevel(DEGRADED_L2,
                                        DEGRADED_REASON_DEMAND_ANOMALY);
            }
        }
    } else if (effective_demand < 0.0f && dynbrake_pct < DYNBRAKE_ACTIVE_THRESHOLD) {
        /* Negative demand without dynamic braking → anomaly */
        effective_demand = 0.0f;
        Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
        {
            SystemState_t st_rng = Safety_GetState();
            if (st_rng == SYS_STATE_ACTIVE) {
                Safety_SetState(SYS_STATE_DEGRADED);
                Safety_SetDegradedLevel(DEGRADED_L1,
                                        DEGRADED_REASON_DEMAND_ANOMALY);
            } else if (st_rng == SYS_STATE_DEGRADED) {
                Safety_SetDegradedLevel(DEGRADED_L2,
                                        DEGRADED_REASON_DEMAND_ANOMALY);
            }
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
     * into Motor_SetSigned() producing unpredictable duty cycles.
     * Reference: TECHNICAL_AUDIT_REPORT.md risk R1.                    */
    effective_demand = sanitize_float(effective_demand, 0.0f);
    safety_status.obstacle_scale = sanitize_float(safety_status.obstacle_scale, 0.0f);
    for (uint8_t i = 0; i < 4; i++) {
        safety_status.wheel_scale[i] = sanitize_float(safety_status.wheel_scale[i], 0.0f);
    }

    uint16_t base_pwm;

    /* ---- B) Motor dead-zone compensation (creep) ----
     * When positive traction demand is above the drive-enter threshold,
     * remap it so the motor jumps over its physical dead zone.
     * Linear mapping: [0, 100] -> [creep_floor, 100].
     * Dynamic braking (negative demand) is not remapped.
     *
     * Runtime-configurable (FASE 2):
     *   - creep_enable == 0 disables the floor entirely (linear mapping).
     *   - creep_power is the floor PWM% (default 8 == MOTOR_DEADZONE_PCT).
     *   - creep_delay (ms) suppresses the floor until the demand has been
     *     continuously positive for that long; the timer below tracks it.
     * Defaults (enable=1, power=8, delay=0) reproduce the historic
     * MOTOR_DEADZONE_PCT behaviour exactly (floor applied immediately).    */
    if (effective_demand > 0.0f) {
        uint32_t now_creep = HAL_GetTick();
        if (!creep_demand_active) {
            creep_demand_active     = true;
            creep_demand_start_tick = now_creep;
        }
        bool delay_elapsed =
            ((uint32_t)(now_creep - creep_demand_start_tick) >=
             (uint32_t)drive_tuning.creep_delay);
        float creep_floor = 0.0f;
        if (drive_tuning.creep_enable && delay_elapsed) {
            creep_floor = (float)drive_tuning.creep_power;
        }
        float mapped = creep_floor
                     + effective_demand * (100.0f - creep_floor) / 100.0f;
        base_pwm = (uint16_t)(mapped * PWM_PERIOD / 100.0f);
    } else {
        creep_demand_active = false;
        base_pwm = (uint16_t)(fabsf(effective_demand) * PWM_PERIOD / 100.0f);
    }

    /* ---- F) Low-speed stability: extra smoothing in creep zone ----
     * When demand is low, ADC quantisation and ramp stepping cause
     * periodic micro-accelerations.  An additional heavy EMA smooths
     * the base_pwm value only below CREEP_ZONE_PCT to eliminate
     * these oscillations without slowing normal throttle response.     */
    if (effective_demand > 0.0f && effective_demand < CREEP_ZONE_PCT) {
        float pwm_f = (float)base_pwm;
        if (!creep_smooth_init) {
            creep_smooth_pct = pwm_f;
            creep_smooth_init = 1;
        } else {
            creep_smooth_pct = CREEP_DEMAND_SMOOTHING_ALPHA * pwm_f
                             + (1.0f - CREEP_DEMAND_SMOOTHING_ALPHA) * creep_smooth_pct;
        }
        base_pwm = (uint16_t)(creep_smooth_pct + 0.5f);
    } else {
        /* Outside creep zone — track current value so re-entry is seamless */
        creep_smooth_pct  = (float)base_pwm;
        creep_smooth_init = 1;
    }

    /* Apply obstacle scale to all wheels.  This multiplier is set by
     * Obstacle_Update() — STM32 primary safety controller.  CAN data
     * from ESP32 is advisory; the local state machine with plausibility
     * validation, stuck-sensor detection, speed-dependent thresholds,
     * and temporal hysteresis determines the actual scale.
     * Applied before per-wheel ABS/TCS wheel_scale[] so that obstacle
     * reduction and ABS/TCS modulation are multiplicative (most
     * restrictive wins).
     *
     * Reverse escape: when obstacle_forward_blocked is set, forward
     * motion is blocked (scale = 0.0) but reverse is allowed.         */
    {
        float effective_obstacle = safety_status.obstacle_scale;
        if (effective_obstacle < 0.01f &&
            current_gear == GEAR_REVERSE &&
            Obstacle_IsForwardBlocked()) {
            /* Reverse escape: obstacle blocks forward, allow reverse */
            effective_obstacle = 1.0f;
        }
        base_pwm = (uint16_t)(base_pwm * effective_obstacle);
    }

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
     * meaningless when wheels on each side spin in opposite directions.
     * Also skipped in LIMP_HOME — no torque vectoring in degraded
     * autonomous mode.  Each motor operates independently.             */
    float acker_diff[4];
    if (sys_st == SYS_STATE_LIMP_HOME) {
        /* LIMP_HOME: no torque vectoring — all wheels equal */
        acker_diff[0] = 1.0f;
        acker_diff[1] = 1.0f;
        acker_diff[2] = 1.0f;
        acker_diff[3] = 1.0f;
    } else {
        compute_ackermann_differential(Steering_GetCurrentAngle(), acker_diff);
        /* Safety: if the steering encoder has faulted, the angle read
         * by Steering_GetCurrentAngle() is no longer trustworthy (it
         * may be latched at the value that triggered the fault or
         * drifting from EMI).  Neutralise the differential so traction
         * stays symmetric while DEGRADED_L1 "drive-home" is active.   */
        if (Encoder_HasFault()) {
            for (int i = 0; i < 4; i++) acker_diff[i] = 1.0f;
        }
    }

    int8_t dir   = (effective_demand >= 0) ? 1 : -1;

    /* GEAR_REVERSE: invert motor direction for reverse travel.
     * This flips both traction (positive demand → backward travel)
     * and dynamic braking (negative demand → forward opposing torque
     * becomes backward opposing torque, correct for a vehicle
     * decelerating while traveling in reverse).                         */
    if (current_gear == GEAR_REVERSE) {
        dir = -dir;
    }

    /* ---- A/C/D) Traction phase state machine (BRAKE ↔ COAST ↔ DRIVE) ----
     *
     * Replaces the previous binary zero-demand check with a three-phase
     * state machine that eliminates the abrupt 100%→low-duty PWM jump:
     *
     *   BRAKE: BTS7960 active brake (PWM=100%, EN=1).  Vehicle held.
     *          Transition to DRIVE when effective_demand > DRIVE_ENTER_PCT.
     *
     *   DRIVE: Motors driven at base_pwm.
     *          Transition to COAST when effective_demand < DRIVE_EXIT_PCT
     *          and vehicle speed > COAST_SPEED_THRESHOLD_KMH.
     *          Transition to BRAKE when effective_demand < DRIVE_EXIT_PCT
     *          and vehicle speed <= COAST_SPEED_THRESHOLD_KMH.
     *
     *   COAST: Motors disabled (EN=0, PWM=0).  Vehicle rolls freely.
     *          Transition to DRIVE when effective_demand > DRIVE_ENTER_PCT.
     *          Transition to BRAKE after COAST_TIMEOUT_MS or when speed
     *          drops below COAST_SPEED_THRESHOLD_KMH.
     *
     * Dynamic braking (negative effective_demand) bypasses this state
     * machine and directly commands motors — it has its own ramp logic.
     *
     * Safety override: SAFE/ERROR states handled upstream (early return).
     * ABS intervention (wheel_scale < 1.0): motors stay enabled so the
     * H-bridge can modulate braking per-wheel.                           */
    bool rear_active = traction_state.mode4x4 || traction_state.axisRotation;
    bool is_dynbrake = (effective_demand < 0.0f &&
                        dynbrake_pct > DYNBRAKE_ACTIVE_THRESHOLD);

    float avg_speed = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                       Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;
    avg_speed = sanitize_float(avg_speed, 0.0f);
    if (avg_speed < 0.0f) avg_speed = 0.0f;

    /* Phase transitions */
    switch (trac_phase) {
    case TRAC_PHASE_BRAKE:
        if (is_dynbrake) {
            /* Dynamic braking uses motor directly — stay in brake concept */
        } else if (effective_demand > DRIVE_ENTER_PCT) {
            trac_phase = TRAC_PHASE_DRIVE;
            brake_release_pct = 0.0f;  /* Start brake→drive ramp from 0 */
        }
        break;

    case TRAC_PHASE_DRIVE:
        if (is_dynbrake) {
            /* Dynamic braking takes over — allow it */
        } else if (effective_demand <= DRIVE_EXIT_PCT) {
            if (avg_speed > COAST_SPEED_THRESHOLD_KMH) {
                trac_phase = TRAC_PHASE_COAST;
                coast_start_tick = HAL_GetTick();
            } else {
                trac_phase = TRAC_PHASE_BRAKE;
            }
        }
        break;

    case TRAC_PHASE_COAST:
        if (effective_demand > DRIVE_ENTER_PCT) {
            trac_phase = TRAC_PHASE_DRIVE;
            brake_release_pct = 0.0f;
        } else if (avg_speed <= (COAST_SPEED_THRESHOLD_KMH
                                 - COAST_SPEED_HYSTERESIS_KMH)) {
            /* Speed dropped below hysteresis band — engage hold brake.
             * The hysteresis prevents oscillation when speed hovers
             * near COAST_SPEED_THRESHOLD_KMH.                         */
            trac_phase = TRAC_PHASE_BRAKE;
        } else if ((HAL_GetTick() - coast_start_tick) >= COAST_TIMEOUT_MS) {
            trac_phase = TRAC_PHASE_BRAKE;
        }
        break;
    }

    /* ---- C) Brake→Drive transition ramp ----
     * When transitioning from brake to drive, ramp up the PWM gradually
     * instead of jumping from 100% brake to a low drive duty.  This
     * eliminates the mechanical jerk at the brake/drive boundary.       */
    if (trac_phase == TRAC_PHASE_DRIVE && !is_dynbrake) {
        if (brake_release_pct < 100.0f) {
            brake_release_pct += BRAKE_RELEASE_RAMP_PCT_S * dt_db;
            if (brake_release_pct > 100.0f) brake_release_pct = 100.0f;
            /* Scale base_pwm by the ramp progress */
            base_pwm = (uint16_t)((float)base_pwm * brake_release_pct / 100.0f);
        }
    }

    /* Apply phase to hardware */
    Motor_t *motors[4] = {&motor_fl, &motor_fr, &motor_rl, &motor_rr};

    /* Compute desired per-motor PWM into an intermediate array so the
     * jerk limiter can compare desired vs previous before writing.     */
    uint16_t desired_pwm[4] = {0, 0, 0, 0};
    int8_t   desired_dir[4] = {dir, dir, dir, dir};
    uint8_t  desired_en[4]  = {0, 0, 0, 0};

    if (is_dynbrake) {
        /* Dynamic braking — apply opposing torque via normal motor path.
         * Phase state machine does not override dynamic braking.        */
        for (uint8_t i = 0; i < 4; i++) {
            bool is_active_motor = (i <= MOTOR_FR) || rear_active;
            if (is_active_motor) {
                desired_pwm[i] = (uint16_t)(base_pwm * acker_diff[i] * safety_status.wheel_scale[i]);
                desired_dir[i] = dir;
                desired_en[i]  = 1;
            } else {
                /* Non-driven rear wheels in 4x2: keep brake */
                desired_pwm[i] = BTS7960_BRAKE_PWM;
                desired_en[i]  = 1;
            }
        }
    } else if (trac_phase == TRAC_PHASE_BRAKE) {
        /* Hold brake — all motors now have GPIO EN, so PWM=0 + EN=HIGH
         * produces symmetric passive brake on every wheel.               */
        for (uint8_t i = 0; i < 4; i++) {
            desired_pwm[i] = BTS7960_BRAKE_PWM;
            desired_en[i]  = 1;
        }
    } else if (trac_phase == TRAC_PHASE_COAST) {
        /* Coast — all motors disabled for symmetric free rolling.
         * With all EN pins under GPIO control, PWM=0 + EN=LOW gives
         * true coast (Hi-Z) on every motor — no bias needed.          */
        for (uint8_t i = 0; i < 4; i++) {
            if (!rear_active && (i == MOTOR_RL || i == MOTOR_RR)) {
                /* 4x2: rear axle stays braked even in coast */
                desired_pwm[i] = BTS7960_BRAKE_PWM;
                desired_en[i]  = 1;
            } else {
                desired_pwm[i] = 0;
                desired_en[i]  = 0;
            }
        }
    } else {
        /* DRIVE phase — normal traction with smooth driving features */
        if (traction_state.axisRotation) {
            axis_rotation_update_scale();
            for (uint8_t i = 0; i < 4; i++) {
                desired_pwm[i] = (uint16_t)(base_pwm * axis_rot_scale[i]
                                            * safety_status.wheel_scale[i]);
                desired_dir[i] = ((i == MOTOR_FL) || (i == MOTOR_RL)) ? (int8_t)-dir : dir;
                desired_en[i]  = 1;
            }
        } else if (traction_state.mode4x4) {
            uint16_t axle_pwm = base_pwm / 2;
            for (uint8_t i = 0; i < 4; i++) {
                desired_pwm[i] = (uint16_t)(axle_pwm * acker_diff[i] * safety_status.wheel_scale[i]);
                desired_en[i]  = 1;
            }
        } else {
            /* 4x2: front wheels driven */
            desired_pwm[MOTOR_FL] = (uint16_t)(base_pwm * acker_diff[MOTOR_FL] * safety_status.wheel_scale[MOTOR_FL]);
            desired_pwm[MOTOR_FR] = (uint16_t)(base_pwm * acker_diff[MOTOR_FR] * safety_status.wheel_scale[MOTOR_FR]);
            desired_en[MOTOR_FL]  = 1;
            desired_en[MOTOR_FR]  = 1;
            /* Rear: brake */
            desired_pwm[MOTOR_RL] = BTS7960_BRAKE_PWM; desired_en[MOTOR_RL] = 1;
            desired_pwm[MOTOR_RR] = BTS7960_BRAKE_PWM; desired_en[MOTOR_RR] = 1;
        }
    }

    /* ---- E) Jerk limiter ----
     * Limit the per-cycle PWM change on each motor to prevent mechanical
     * jerk in the drivetrain.  Compares the desired PWM against the
     * previous cycle's output and clamps the step to ±MAX_PWM_DELTA.
     *
     * This does NOT affect emergency stop (Traction_EmergencyStop resets
     * prev_output_pwm[]) or safety overrides (handled upstream).
     *
     * Skip jerk limiting during brake and coast phases (PWM values are
     * either BTS7960_BRAKE_PWM or 0, which must be applied immediately).*/
    if (trac_phase == TRAC_PHASE_DRIVE || is_dynbrake) {
        for (uint8_t i = 0; i < 4; i++) {
            int32_t delta_j = (int32_t)desired_pwm[i] - (int32_t)prev_output_pwm[i];
            if (delta_j > (int32_t)MAX_PWM_DELTA_PER_CYCLE) {
                desired_pwm[i] = prev_output_pwm[i] + MAX_PWM_DELTA_PER_CYCLE;
            } else if (delta_j < -(int32_t)MAX_PWM_DELTA_PER_CYCLE) {
                desired_pwm[i] = (prev_output_pwm[i] > MAX_PWM_DELTA_PER_CYCLE)
                               ? prev_output_pwm[i] - MAX_PWM_DELTA_PER_CYCLE
                               : 0;
            }
            prev_output_pwm[i] = desired_pwm[i];
        }
    } else {
        /* In brake/coast, reset prev_output_pwm to 0 so the jerk
         * limiter sees a smooth transition from "stopped" to "driving"
         * when entering DRIVE phase.  Without this, prev_output_pwm
         * would track BTS7960_BRAKE_PWM (0), and the jerk limiter
         * would fight the brake_release_pct ramp by keeping PWM locked
         * near full brake for ~530ms (P2 fix).                         */
        for (uint8_t i = 0; i < 4; i++) {
            prev_output_pwm[i] = 0;
        }
    }

    /* ---- Defence-in-depth: clamp per-wheel PWM to PWM_PERIOD (audit fix) ----
     * Float-to-uint16 casts from upstream multiplications (acker_diff,
     * wheel_scale, obstacle_scale) could theoretically exceed PWM_PERIOD
     * due to float rounding.  Motor_SetSigned also clamps internally,
     * but an explicit clamp here ensures the telemetry PWM values and
     * jerk limiter comparisons are never corrupted.                      */
    for (uint8_t i = 0; i < 4; i++) {
        if (desired_pwm[i] > PWM_PERIOD) desired_pwm[i] = PWM_PERIOD;
    }

    /* Write final motor state to hardware using Motor_SetMode().
     *
     * M4 fix: The previous code always called Motor_SetSigned(0) when
     * desired_pwm==0, which set EN=LOW (coast) regardless of whether
     * brake was intended.  Now the desired_en[] flag explicitly selects:
     *
     *   desired_en=1, desired_pwm=0 → MOTOR_MODE_BRAKE (EN=HIGH, PWM=0)
     *   desired_en=0, desired_pwm=0 → MOTOR_MODE_COAST (EN=LOW,  PWM=0)
     *   desired_en=1, desired_pwm>0 → MOTOR_MODE_DRIVE (EN=HIGH, PWM=duty)
     *   desired_en=0, desired_pwm>0 → MOTOR_MODE_COAST (safety: EN=LOW)
     *
     * This ensures TRAC_PHASE_BRAKE produces real passive braking
     * (motor terminals shorted) instead of unintended coast.            */
    for (uint8_t i = 0; i < 4; i++) {
        if (!desired_en[i]) {
            /* Coast — motor Hi-Z, free rolling */
            Motor_SetMode(motors[i], MOTOR_MODE_COAST, 0);
        } else if (desired_pwm[i] == 0) {
            /* Brake — EN=HIGH, PWM=0, motor terminals shorted */
            Motor_SetMode(motors[i], MOTOR_MODE_BRAKE, 0);
        } else {
            /* Drive — EN=HIGH, apply signed duty cycle */
            int16_t sp = (int16_t)((int32_t)desired_dir[i] * (int32_t)desired_pwm[i]);
            Motor_SetMode(motors[i], MOTOR_MODE_DRIVE, sp);
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
        /* AUDIT A (empuje real por rueda): store the PWM value ACTUALLY
         * written to the BTS7960 this cycle, not an intermediate stage.
         * desired_pwm[i]/desired_en[i] are the FINAL hardware decision made
         * above (after obstacle, traction cap, Ackermann, ABS/TCS wheel_scale,
         * the jerk limiter and the DRIVE/BRAKE/COAST state machine):
         *   COAST (EN low)        → 0  (free rolling, no push)
         *   BRAKE (EN high, PWM0) → 0  (holding, no forward push)
         *   DRIVE                 → desired_pwm[i] (the real duty applied)
         * Never reconstruct it from base_pwm/acker_diff/wheel_scale — that is
         * the ABS/TCS-permitted (0x205) view, which reads 100 % whenever the
         * limiter is idle even if the real duty is far lower.               */
        if (!desired_en[i] || desired_pwm[i] == 0U) {
            traction_state.wheels[i].pwm = 0U;
        } else {
            traction_state.wheels[i].pwm = desired_pwm[i];
        }
        traction_state.wheels[i].reverse  = (dir < 0);
    }

    /* ---- MOTION_INHIBIT_REASON instrumentation (normal drive path) ----
     * Capture the effective demand and the maximum final PWM duty actually
     * written to the BTS7960 drivers (EN asserted), so the classifier can
     * distinguish "demand zeroed before PWM" from "demand survived but PWM
     * is zero".  Instrumentation only — no control effect.               */
    {
        uint16_t final_pwm_max = 0;
        for (uint8_t i = 0; i < 4; i++) {
            if (desired_en[i] && desired_pwm[i] > final_pwm_max) {
                final_pwm_max = desired_pwm[i];
            }
        }
        Traction_UpdateMotionInhibit(effective_demand, final_pwm_max);
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
    Motor_SetSigned(&motor_fl,    0);
    Motor_SetSigned(&motor_fr,    0);
    Motor_SetSigned(&motor_rl,    0);
    Motor_SetSigned(&motor_rr,    0);
    Motor_SetSigned(&motor_steer, 0);
    traction_state.demandPct = 0.0f;

    /* Reset pedal filter so emergency stop is immediate */
    pedal_ema         = 0.0f;
    pedal_ramped      = 0.0f;
    pedal_filter_init = 0;

    /* Reset dynamic braking */
    dynbrake_pct       = 0.0f;
    prev_demand_pct    = 0.0f;
    dynbrake_last_tick = HAL_GetTick();

    /* Reset demand anomaly detection — prevent stale state from
     * causing false anomaly triggers after SAFE → ACTIVE recovery. */
    prev_raw_demand      = 0.0f;
    prev_raw_demand_tick = 0;
    frozen_pedal_value   = 0.0f;
    frozen_pedal_tick    = 0;
    frozen_pedal_speed   = 0.0f;
    anomaly_init         = 0;

    /* Reset smooth driving state — emergency stop is immediate,
     * no ramp, no coast, no jerk limit.  Motors are de-energised. */
    trac_phase        = TRAC_PHASE_BRAKE;
    coast_start_tick  = 0;
    brake_release_pct = 0.0f;
    creep_smooth_pct  = 0.0f;
    creep_smooth_init = 0;
    creep_demand_active     = false;
    creep_demand_start_tick = 0;
    neutral_ramp_pct   = 0.0f;
    neutral_ramp_active = 0;
    neutral_ramp_dir   = 1;
    for (uint8_t i = 0; i < 4; i++) {
        prev_output_pwm[i] = 0;
    }
    axis_rotation_reset_scale();
}

const TractionState_t* Traction_GetState(void)
{
    return &traction_state;
}

/* Calibration movement lock (audit P5.4).
 *
 * Enforces — every call — the real "cannot move" state required while a
 * pedal-calibration session runs, then VERIFIES it and returns the result:
 *   - traction demand forced to 0;
 *   - the four traction H-bridges driven to COAST (PWM = 0, EN = LOW);
 *   - readback confirms every traction EN pin is physically LOW and the
 *     resolved final PWM duty is 0.
 *
 * The traction relay is owned by safety_system.c; the caller additionally
 * verifies the relay is de-energised.  Returns true only when the enable
 * lines and PWM are confirmed safe, so the session can abort (LOCK_LOST) the
 * instant the lock is lost instead of assuming STANDBY implies inhibition. */
bool Traction_CalibrationLock(void)
{
    /* 1. Force zero demand and kill the smooth-drive state so nothing can
     *    ramp the motors back up on the next Traction_Update(). */
    traction_state.demandPct = 0.0f;
    pedal_ema         = 0.0f;
    pedal_ramped      = 0.0f;
    pedal_filter_init = 0;

    /* 2. De-energise the four traction H-bridges (EN LOW, PWM 0). */
    Motor_SetMode(&motor_fl, MOTOR_MODE_COAST, 0);
    Motor_SetMode(&motor_fr, MOTOR_MODE_COAST, 0);
    Motor_SetMode(&motor_rl, MOTOR_MODE_COAST, 0);
    Motor_SetMode(&motor_rr, MOTOR_MODE_COAST, 0);

    /* 3. Verify the enables really read LOW and the resolved duty is 0. */
    bool en_low = (HAL_GPIO_ReadPin(motor_fl.en_port, motor_fl.en_pin) == GPIO_PIN_RESET) &&
                  (HAL_GPIO_ReadPin(motor_fr.en_port, motor_fr.en_pin) == GPIO_PIN_RESET) &&
                  (HAL_GPIO_ReadPin(motor_rl.en_port, motor_rl.en_pin) == GPIO_PIN_RESET) &&
                  (HAL_GPIO_ReadPin(motor_rr.en_port, motor_rr.en_pin) == GPIO_PIN_RESET);
    return en_low && (Traction_GetFinalPwmPct() == 0U);
}

/* Read-only confirmation of the calibration movement lock (audit fix).
 *
 * Unlike Traction_CalibrationLock(), this NEVER modifies any output: it only
 * READS the effective traction demand, the resolved final PWM duty, the four
 * traction enable lines and the traction relay state, and reports whether the
 * "cannot move" condition currently holds.  This is the ONLY check that may be
 * used from telemetry / diagnostic / QUERY paths (e.g. the 0x319 "entry OK"
 * bit), so that simply reading calibration status can never force demand 0,
 * PWM 0 or the traction enables LOW outside an actual calibration session. */
bool Traction_IsCalibrationLockConfirmed(void)
{
    bool demand_zero = (fabsf(traction_state.demandPct) < 0.01f);
    bool en_low = (HAL_GPIO_ReadPin(motor_fl.en_port, motor_fl.en_pin) == GPIO_PIN_RESET) &&
                  (HAL_GPIO_ReadPin(motor_fr.en_port, motor_fr.en_pin) == GPIO_PIN_RESET) &&
                  (HAL_GPIO_ReadPin(motor_rl.en_port, motor_rl.en_pin) == GPIO_PIN_RESET) &&
                  (HAL_GPIO_ReadPin(motor_rr.en_port, motor_rr.en_pin) == GPIO_PIN_RESET);
    bool pwm_zero  = (Traction_GetFinalPwmPct() == 0U);
    bool relay_off = ((Safety_GetRelayStatusByte() & (1U << 1)) == 0U);
    return demand_zero && en_low && pwm_zero && relay_off;
}

/* ==================================================================
 *  Steering Control — EPS Torque-Assist
 *
 *  Replaces the former position-PID approach.  The motor NEVER applies
 *  active brake near center.  Close to center → coast (EN=LOW).
 *
 *  Torque equation (all gains from eps_params_t):
 *    τ = + λ(ω) · assist_strength · g(v) · ω_filt          (assist)
 *        − (1−λ(ω)) · center_strength · h(v) · θ           (return)
 *        − damping · ω_filt                                 (damp)
 *        + friction_comp · sign_when_stopped(θ,ω)           (friction)
 *
 *  λ(ω) = smoothstep(|ω|, 2°/s → 12°/s)   (driver intent)
 *  g(v)  = 1 / (1 + v / assist_vs_speed)    (speed-dep assist)
 *  h(v)  = 0.3 + v / return_vs_speed        (speed-dep return)
 * ================================================================== */

void Steering_SetAngle(float angle_deg)
{
    /* In EPS torque-assist mode Steering_SetAngle is kept for Ackermann
     * wheel-angle computation only.  There is no position setpoint.    */
    if (!steering_calibrated) return;

    angle_deg = sanitize_float(angle_deg, 0.0f);
    if (angle_deg < -MAX_STEER_DEG) angle_deg = -MAX_STEER_DEG;
    if (angle_deg >  MAX_STEER_DEG) angle_deg =  MAX_STEER_DEG;

    Ackermann_ComputeWheelAngles(angle_deg, &steer_fl_deg, &steer_fr_deg);
}

void Steering_ControlLoop(void)
{
    /* ---- Guard: assist isolated → keep motor disconnected (mechanical) ----
     * A latched isolable EPS fault means the assist is permanently off for
     * this power cycle.  Coast the motor (PA6=PA7=0, PC4=LOW) every cycle;
     * PC12 stays OFF via the relay sequencer.  Never re-drive the motor.   */
    if (Steering_IsMechanicalOnly()) {
        Steering_Neutralize();
        return;
    }

    /* ---- Guard: not calibrated → coast ---- */
    if (!steering_calibrated) {
        Steering_Neutralize();
        return;
    }

    /* ---- Guard: encoder fault → disable motor ---- */
    if (enc_fault) {
        Motor_SetSigned(&motor_steer, 0);
        eps_motor_effort = 0.0f;
        return;
    }

    /* ---- Guard: SAFE / ERROR → coast immediately ---- */
    {
        SystemState_t st = Safety_GetState();
        if (st == SYS_STATE_SAFE || st == SYS_STATE_ERROR) {
            Steering_Neutralize();
            return;
        }
    }

    /* ---- Time delta ---- */
    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();

    if (last_time == 0 || (now - last_time) > 500) {
        /* First call or returning from SAFE/ERROR — re-seed to avoid
         * a huge dt that would spike the angular velocity estimate.   */
        last_time = now;
        eps_prev_angle_deg = Steering_GetCurrentAngle();
        return;
    }

    float dt = (float)(now - last_time) / 1000.0f;
    if (dt < 0.001f) return;
    last_time = now;

    /* ---- Read encoder: angle θ (road-wheel degrees) ----
     * Use Steering_GetCurrentAngle() so the STEERING_GEAR_RATIO
     * conversion is applied in one place and EPS operates on the
     * same scale as Ackermann and safety plausibility checks.        */
    float theta = Steering_GetCurrentAngle();
    theta = sanitize_float(theta, 0.0f);

    /* ---- Mechanical-backlash deadband (road-wheel degrees) ----
     * Ignore sub-deadband angles so the control loop does not chase
     * the ~3° of slop between the encoder/motor side and the road
     * wheels.  Applied in deg (the global unit) after sanitisation,
     * before the ω estimate so ω also sees theta clamped to 0.     */
    const eps_params_t *p = EPS_Params_Get();

    float eff_theta = theta - p->center_offset_deg;
    if (fabsf(eff_theta) < p->deadband_deg) {
        eff_theta = 0.0f;
    }

    /* ---- Angular velocity ω (°/s) ---- */
    float omega_raw = (eff_theta - eps_prev_angle_deg) / dt;
    omega_raw = sanitize_float(omega_raw, 0.0f);
    eps_prev_angle_deg = eff_theta;

    /* EMA filter: α = 0.3 */
    eps_omega_filt = eps_omega_filt + 0.3f * (omega_raw - eps_omega_filt);

    /* ---- Vehicle speed (average of 4 wheels, km/h) ---- */
    float v_kmh = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                   Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) * 0.25f;
    v_kmh = sanitize_float(v_kmh, 0.0f);
    if (v_kmh < 0.0f) v_kmh = 0.0f;

    /* ---- Smoothstep: driver intention λ(ω) ---- */
    float abs_omega = fabsf(eps_omega_filt);
    float lambda;
    {
        /* smoothstep mapping |ω| from [2, 12] → [0, 1] */
        float lo = 2.0f, hi = 12.0f;
        if (abs_omega <= lo)      lambda = 0.0f;
        else if (abs_omega >= hi) lambda = 1.0f;
        else {
            float t = (abs_omega - lo) / (hi - lo);
            lambda = t * t * (3.0f - 2.0f * t);
        }
    }

    /* ---- Speed-dependent gains ---- */
    float g_v = 1.0f / (1.0f + v_kmh / p->assist_vs_speed);
    float h_v = 0.3f + v_kmh / p->return_vs_speed;

    /* ---- Friction compensation sign ----
     * Helps overcome static friction when the wheel is displaced and
     * nearly stopped (return-to-center scenario).                     */
    float fric_sign = 0.0f;
    if (abs_omega < 2.0f && fabsf(eff_theta) > 1.0f) {
        fric_sign = (eff_theta > 0.0f) ? -1.0f : 1.0f;
    }

    /* ---- Torque equation ---- */
    float tau = 0.0f;
    tau += lambda * p->assist_strength * g_v * eps_omega_filt;          /* assist  */
    tau -= (1.0f - lambda) * p->center_strength * h_v * eff_theta;      /* return  */
    tau -= p->damping * eps_omega_filt;                                  /* damp    */
    tau += p->friction_comp * fric_sign;                                 /* friction*/

    /* ---- High-speed safety: gradually reduce assist above 20 km/h ----
     * Previous implementation used a hard step at 25 km/h (tau *= 0.5),
     * which created an abrupt discontinuity in steering feel — the
     * driver would feel the wheel suddenly stiffen at exactly 25 km/h.
     * Replaced with a linear fade from 100% at 20 km/h to 50% at
     * 30 km/h for a smooth, progressive weight-up (P3 fix).           */
    if (v_kmh > EPS_HS_FADE_START_KMH) {
        float hs_range = EPS_HS_FADE_END_KMH - EPS_HS_FADE_START_KMH;
        float hs_fade = 1.0f - (1.0f - EPS_HS_FADE_MIN_FACTOR)
                       * (v_kmh - EPS_HS_FADE_START_KMH) / hs_range;
        if (hs_fade < EPS_HS_FADE_MIN_FACTOR) hs_fade = EPS_HS_FADE_MIN_FACTOR;
        tau *= hs_fade;
    }

    /* ---- Degraded-mode scaling ---- */
    if (Safety_IsDegraded()) {
        tau *= Safety_GetSteeringLimitFactor();
    }
    tau = sanitize_float(tau, 0.0f);

    /* ---- Torque → PWM% (clamp to ±max_pwm_pct) ---- */
    float pwm_pct = tau;
    if (pwm_pct >  p->max_pwm_pct) pwm_pct =  p->max_pwm_pct;
    if (pwm_pct < -p->max_pwm_pct) pwm_pct = -p->max_pwm_pct;

    /* ---- Dead-zone compensation: jump to min_drive_pct ---- */
    float abs_pct = fabsf(pwm_pct);
    if (abs_pct > 0.01f && abs_pct < p->min_drive_pct) {
        pwm_pct = (pwm_pct > 0.0f) ? p->min_drive_pct : -p->min_drive_pct;
        abs_pct = p->min_drive_pct;
    }

    /* ---- Coast band: below coast_band_pct → motor off (EN=LOW) ---- */
    if (abs_pct < p->coast_band_pct) {
        Steering_Neutralize();
        return;
    }

    /* ---- Convert to PWM counts ---- */
    int16_t pwm_raw = (int16_t)(pwm_pct * (float)PWM_PERIOD / 100.0f);

    /* ---- Slew-rate limit (from params: slew_rate_pct × PWM_PERIOD / 100) ---- */
    int16_t slew_counts = (int16_t)(p->slew_rate_pct * (float)PWM_PERIOD / 100.0f);
    if (slew_counts < 1) slew_counts = 1;
    int16_t delta_pwm = pwm_raw - eps_prev_pwm_raw;
    if (delta_pwm >  slew_counts) pwm_raw = eps_prev_pwm_raw + slew_counts;
    if (delta_pwm < -slew_counts) pwm_raw = eps_prev_pwm_raw - slew_counts;
    eps_prev_pwm_raw = pwm_raw;

    /* ---- Direction + absolute PWM ---- */
    int8_t  direction = (pwm_raw >= 0) ? 1 : -1;
    uint16_t pwm_abs  = (uint16_t)((pwm_raw >= 0) ? pwm_raw : -pwm_raw);

    /* Store motor effort for encoder health monitoring */
    eps_motor_effort = (float)pwm_abs * 100.0f / (float)PWM_PERIOD;

    /* ---- Apply to motor hardware ---- */
    Motor_SetSigned(&motor_steer,
                    direction > 0 ? (int16_t)pwm_abs : -(int16_t)pwm_abs);
}

float Steering_GetCurrentAngle(void)
{
    int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    /* Encoder mounted on steering column: divide by STEERING_GEAR_RATIO
     * to return road-wheel degrees (single conversion point).          */
    return ((float)cnt * 360.0f / (float)ENCODER_CPR) / STEERING_GEAR_RATIO;
}

float Steering_GetMotorEffortPct(void)
{
    return eps_motor_effort;
}

int32_t Steering_GetEncoderRaw(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
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
    /* Homing succeeded → the assist may become active (unless a fault has
     * already latched the EPS to mechanical-only, in which case this is a
     * no-op inside the EPS authority).                                     */
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);
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
     * If the EPS is commanding significant motor output but the
     * encoder count has not changed, the sensor is likely
     * disconnected or mechanically decoupled.                         */
    if (count != enc_prev_count) {
        enc_last_change_tick = now;
    } else {
        float motor_pct = fabsf(eps_motor_effort);
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
 * @brief  Safely stop steering motor — coast mode (EN=LOW, PWM=0).
 *
 * EPS philosophy: the motor NEVER applies active brake near center.
 * Coast mode lets the steering float freely, avoiding vibration and
 * fighting the driver.  Used on encoder fault, pre-calibration, and
 * SAFE/ERROR state transitions.
 */
void Steering_Neutralize(void)
{
    Motor_SetSigned(&motor_steer, 0);
    eps_omega_filt   = 0.0f;
    eps_prev_pwm_raw = 0;
    eps_motor_effort = 0.0f;
}

/* ==================================================================
 *  Ackermann Geometry
 * ================================================================== */

AckermannResult_t Ackermann_Compute(float wheelAngleDeg)
{
    AckermannResult_t result = {0};
    if (fabsf(wheelAngleDeg) < 0.01f) {
        result.innerDeg = 0.0f;
        result.outerDeg = 0.0f;
        return result;
    }

    float angle_rad = wheelAngleDeg * (float)M_PI / 180.0f;
    float turn_radius = ackermann_wheelbase / tanf(fabsf(angle_rad));

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

/* ==================================================================
 *  Low-level per-wheel PWM wrappers  (legacy API — preserved)
 *  Internally delegate to Motor_SetSigned so RPWM/LPWM is always
 *  consistent.  pwm=0 maps to coast regardless of reverse flag.
 * ================================================================== */

void Motor_SetPWM_FL(uint16_t pwm, bool reverse) {
    Motor_SetSigned(&motor_fl, reverse ? -(int16_t)pwm : (int16_t)pwm);
}

void Motor_SetPWM_FR(uint16_t pwm, bool reverse) {
    Motor_SetSigned(&motor_fr, reverse ? -(int16_t)pwm : (int16_t)pwm);
}

void Motor_SetPWM_RL(uint16_t pwm, bool reverse) {
    Motor_SetSigned(&motor_rl, reverse ? -(int16_t)pwm : (int16_t)pwm);
}

void Motor_SetPWM_RR(uint16_t pwm, bool reverse) {
    Motor_SetSigned(&motor_rr, reverse ? -(int16_t)pwm : (int16_t)pwm);
}

void Motor_SetPWM_Steering(uint16_t pwm, bool reverse) {
    Motor_SetSigned(&motor_steer, reverse ? -(int16_t)pwm : (int16_t)pwm);
}

/* ==================================================================
 *  Signed-speed API  (new — preferred for direct BTS7960 control)
 *
 *  speed > 0 → RPWM = |speed|, LPWM = 0   (forward)
 *  speed < 0 → RPWM = 0,       LPWM = |speed| (reverse)
 *  speed = 0 → RPWM = 0,       LPWM = 0   (coast / passive brake)
 *
 *  Guarantees RPWM and LPWM are never simultaneously non-zero.
 *  |speed| is clamped to PWM_PERIOD (4249) before writing.
 * ================================================================== */

void Motor_SetSignedPWM_FL(int16_t speed) {
    Motor_SetSigned(&motor_fl, speed);
}

void Motor_SetSignedPWM_FR(int16_t speed) {
    Motor_SetSigned(&motor_fr, speed);
}

void Motor_SetSignedPWM_RL(int16_t speed) {
    Motor_SetSigned(&motor_rl, speed);
}

void Motor_SetSignedPWM_RR(int16_t speed) {
    Motor_SetSigned(&motor_rr, speed);
}

void Motor_SetSignedPWM_Steering(int16_t speed) {
    Motor_SetSigned(&motor_steer, speed);
}

/* ---- Runtime active-brake override ---- */

void Motor_SetBrakeActiveOverride(uint16_t pwm_ticks)
{
    if (pwm_ticks > PWM_PERIOD) pwm_ticks = PWM_PERIOD;
    brake_active_override = pwm_ticks;
}

uint16_t Motor_GetBrakeActiveOverride(void)
{
    return brake_active_override;
}

/* ==================================================================
 *  Private helpers
 * ================================================================== */

/**
 * @brief  Core RPWM/LPWM driver — the only function that writes to
 *         hardware timer CCR registers and the optional GPIO EN pin.
 *
 *  signed_pwm > 0 → Forward:  RPWM = |signed_pwm|, LPWM = 0
 *  signed_pwm < 0 → Reverse:  RPWM = 0,            LPWM = |signed_pwm|
 *  signed_pwm = 0 → Stop:     RPWM = 0,            LPWM = 0
 *
 *  LPWM is always written to zero BEFORE RPWM is set (and vice versa)
 *  to guarantee the two channels are never simultaneously non-zero,
 *  even during the brief window between two CCR register writes.
 *  Both CCRs are double-buffered (OCPreload enabled) so the actual
 *  output only changes at the next timer period boundary, making the
 *  simultaneous-active risk negligible in hardware.
 *
 *  @note THREAD SAFETY: Same restriction as Motor_SetMode() — must
 *        only be called from the main loop.  Not re-entrant.
 *
 *  @note MODE TRACKING: This function does NOT update current_mode.
 *        Callers that need explicit mode tracking (COAST/BRAKE/DRIVE)
 *        should use Motor_SetMode() instead.  Direct Motor_SetSigned()
 *        calls (Emergency stop, Park hold, Neutral ramp) intentionally
 *        bypass mode tracking because they are transitional states that
 *        will be followed by a proper Motor_SetMode() call.
 */
static void Motor_SetSigned(Motor_t *motor, int16_t signed_pwm)
{
    /* Guard against INT16_MIN: negation of −32768 is undefined behaviour
     * in two's complement.  Clamp to −32767 which negates safely.       */
    if (signed_pwm == INT16_MIN) signed_pwm = INT16_MIN + 1;

    uint16_t duty = (signed_pwm >= 0) ? (uint16_t)signed_pwm
                                      : (uint16_t)(-signed_pwm);
    if (duty > PWM_PERIOD) duty = PWM_PERIOD;

    /* ---- Direction-change dead-state enforcement (audit fix) ----
     * When the requested direction differs from the current direction
     * AND the motor is not stopping (duty > 0), force both channels
     * to zero first and enforce a software dead-time delay.
     *
     * This guarantees a zero-torque intermediate state during direction
     * reversal, preventing any possibility of transient shoot-through
     * even if OCPreload were misconfigured.
     *
     * With OCPreload enabled (as configured in MX_TIM*_Init), the
     * actual outputs only change at the next UEV (50 µs), but the
     * software delay provides defence-in-depth for the BTS7960 FET
     * switching times (~0.2 µs t_d(on)/t_d(off)).                     */
    int8_t new_dir = (signed_pwm > 0) ? 1 : ((signed_pwm < 0) ? -1 : 0);
    if (duty > 0U && motor->direction != 0 && new_dir != 0 &&
        new_dir != motor->direction) {
        /* Transitioning between forward and reverse — insert zero state */
        __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);
        __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, 0U);
        delay_us(SAFE_DEADTIME_US);
    }

    if (signed_pwm > 0) {
        /* Forward: clear LPWM first, then set RPWM */
        __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, 0U);
        __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, duty);
        motor->direction = 1;
    } else if (signed_pwm < 0) {
        /* Reverse: clear RPWM first, then set LPWM */
        __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);
        __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, duty);
        motor->direction = -1;
    } else {
        /* Stop: both channels to zero, EN → LOW (coast).
         * For explicit brake (EN=HIGH, PWM=0), use Motor_SetMode()
         * with MOTOR_MODE_BRAKE instead.  Motor_SetSigned(0) always
         * produces coast — safe default for emergency stop paths.     */
        __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);
        __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, 0U);
        motor->direction = 0;
    }

    /* Assert / deassert GPIO EN.  All five motors now have dedicated
     * GPIO EN pins, giving symmetric coast/brake behaviour.             */
    if (motor->en_port != NULL) {
        HAL_GPIO_WritePin(motor->en_port, motor->en_pin,
                          (duty > 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/**
 * @brief  Mode-aware motor control — selects BTS7960 physical state.
 *
 *  This function replaces the implicit duty==0→coast assumption of
 *  Motor_SetSigned() with an explicit three-mode API:
 *
 *    MOTOR_MODE_COAST: PWM=0, EN=LOW  → Hi-Z, motor free-spinning
 *    MOTOR_MODE_BRAKE: PWM=0, EN=HIGH → motor terminals shorted (hold)
 *    MOTOR_MODE_DRIVE: EN=HIGH, then apply signed PWM
 *
 *  Safe transition: PWM is always zeroed and a dead-time is enforced
 *  before any EN or direction change, preventing transient shoot-through.
 *
 * @param motor      Pointer to the motor structure.
 * @param mode       Desired physical operating mode.
 * @param signed_pwm Signed duty: >0 forward, <0 reverse, 0 ignored
 *                   in COAST and BRAKE modes (used only in DRIVE).
 *
 * @note THREAD SAFETY: This function must ONLY be called from the
 *       main loop context (10 ms tick).  It is NOT re-entrant and
 *       NOT safe to call from ISR, CAN callback, or any other
 *       interrupt context.  The delay_us() busy-wait and multi-step
 *       register writes are not atomic.  All callers in this firmware
 *       are confirmed main-loop-only (Traction_Update, Motor_Init,
 *       Traction_EmergencyStop).
 */
static void Motor_SetMode(Motor_t *motor, motor_mode_t mode, int16_t signed_pwm)
{
    /* ---- Universal safe-transition pre-step ----
     *
     * Before ANY mode change, zero all PWM outputs and enforce a
     * dead-time delay.  This guarantees that the FETs in the BTS7960
     * have fully settled before the new drive state is applied,
     * preventing transient shoot-through during:
     *   - DRIVE → BRAKE  (high duty → shorted terminals)
     *   - DRIVE → COAST  (high duty → Hi-Z)
     *   - BRAKE → DRIVE  (shorted terminals → active drive)
     *   - COAST → DRIVE  (Hi-Z → active drive)
     *
     * The pre-step is skipped when the mode is NOT changing (e.g.
     * consecutive DRIVE calls with same direction) to avoid
     * unnecessary PWM glitches during normal operation.  The
     * direction-change case within DRIVE is handled separately
     * by Motor_SetSigned()'s own dead-state enforcement.
     *
     * OCPRELOAD INTERACTION: With OCPreload enabled, the CCR=0 writes
     * above go to the preload (shadow) register and only take effect
     * at the next timer UEV (~25 µs).  If the new-mode CCR writes
     * (below) happen before UEV, the preload is overwritten and the
     * zero-duty state never appears on the physical outputs.  This
     * is acceptable because:
     *   a) All RPWM/LPWM pairs share the same timer → UEV updates
     *      both CCRs atomically, preventing any window where both
     *      channels are simultaneously non-zero.
     *   b) The BTS7960 integrates its own internal FET dead-time.
     *   c) The delay primarily covers BTS7960 INH (EN) settling time
     *      (~0.2 µs t_d(on)/t_d(off)) and 74HC244 buffer propagation
     *      (~15 ns), both of which are satisfied by the 5 µs wait.
     *
     * CRITICAL SECTION: The pre-step + mode change must be atomic
     * with respect to interrupts.  An ISR preempting between the
     * PWM-zero write and the EN state change could leave the
     * H-bridge in an undefined intermediate state.  __disable_irq()
     * ensures the entire transition completes without interruption.
     *
     * Cost: 5 µs busy-wait per mode transition — negligible at the
     * 10 ms control loop rate.                                        */

    /* Disable interrupts for the entire mode transition to prevent
     * an ISR from seeing an intermediate EN/PWM state.  This is
     * defence-in-depth: all current callers are main-loop-only, but
     * this protects against future call sites from ISR context.
     *
     * INTERRUPT LATENCY: When the pre-step fires, interrupts are
     * held for ~5 µs (SAFE_DEADTIME_US × dwt_cycles_per_us = 850
     * cycles at 170 MHz).  This is well within the tolerance of all
     * ISRs in this system (FDCAN @ 100 ms period, SysTick @ 1 ms,
     * TIM UEV @ 50 µs).  If sub-10 µs ISR response were ever
     * required, the critical section could be narrowed to protect
     * only the EN GPIO write.                                         */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (mode != motor->current_mode) {
        __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);
        __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, 0U);
        delay_us(SAFE_DEADTIME_US);
    }

    switch (mode) {

    case MOTOR_MODE_COAST:
        /* Safe order: zero PWM (already done above), then deassert EN.
         * This ensures no current flows when EN transitions.            */
        __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);
        __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, 0U);
        motor->direction = 0;
        if (motor->en_port != NULL) {
            HAL_GPIO_WritePin(motor->en_port, motor->en_pin, GPIO_PIN_RESET);
        }
        break;

    case MOTOR_MODE_BRAKE:
        /* Safe order: zero PWM (already done above + dead-time), then
         * assert EN.  With RPWM=0 and LPWM=0 and EN=HIGH, the BTS7960
         * shorts the motor terminals → passive electromagnetic braking.
         *
         * If BRAKE_ACTIVE_FALLBACK is enabled at compile time, a small
         * duty is applied on LPWM to increase braking current.
         * Otherwise, brake_active_override provides runtime control
         * for field testing without recompilation.                      */
        __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);
#if BRAKE_ACTIVE_FALLBACK
        __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel,
                              BRAKE_ACTIVE_MIN_PWM);
#else
        {
            uint16_t brake_lpwm = brake_active_override;
            if (brake_lpwm > PWM_PERIOD) brake_lpwm = PWM_PERIOD;
            __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel,
                                  brake_lpwm);
        }
#endif
        motor->direction = 0;
        if (motor->en_port != NULL) {
            HAL_GPIO_WritePin(motor->en_port, motor->en_pin, GPIO_PIN_SET);
        }
        break;

    case MOTOR_MODE_DRIVE:
        /* Delegate to Motor_SetSigned which handles direction-change
         * dead-state, RPWM/LPWM sequencing, and EN assertion.
         * If signed_pwm == 0, this degrades to coast (EN=LOW) which
         * is the correct fallback for zero-duty drive requests.         */
        Motor_SetSigned(motor, signed_pwm);
        break;
    }

    /* Track current mode for transition detection */
    motor->current_mode = mode;

    /* Restore interrupt state — if IRQs were already disabled by the
     * caller (e.g. Error_Handler), they stay disabled.                */
    __set_PRIMASK(primask);
}

static float __attribute__((unused)) PID_Compute(PID_t *pid, float measured, float dt)
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
