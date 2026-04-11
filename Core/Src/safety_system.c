/**
  ****************************************************************************
  * @file    safety_system.c
  * @brief   Safety: ABS, TCS, overcurrent, overtemp, CAN timeout, fail-safe
  *
  *          Implements the STM32 safety-authority role:
  *            – System state machine (BOOT→STANDBY→ACTIVE→SAFE→ERROR)
  *            – Command validation gate for ESP32 requests
  *            – Relay power sequencing
  *            – Sensor plausibility checks
  ****************************************************************************
  */

#include "safety_system.h"
#include "main.h"
#include "sensor_manager.h"
#include "motor_control.h"
#include "service_mode.h"
#include "boot_validation.h"
#include "error_log.h"
#include "can_handler.h"
#include "math_safety.h"
#include <math.h>       /* isnan(), isinf() — NaN/Inf hardening */

/* ---- Thresholds (from base firmware) ---- */
#define ABS_SLIP_THRESHOLD   15   /* abs_system.cpp: slipThreshold = 15.0f */
#define TCS_SLIP_THRESHOLD   15   /* tcs_system.cpp: slipThreshold = 15.0f */
#define MAX_CURRENT_A        25.0f   /* per-motor overcurrent threshold      */
#define MAX_CURRENT_BATT_A   100.0f  /* battery channel aggregates all loads  */
#define CAN_TIMEOUT_MS       250

/* Saturating uint32_t increment — prevents wrap to 0 */
static inline void sat_inc_u32(uint32_t *counter) {
    if (*counter < UINT32_MAX) (*counter)++;
}

/* Battery undervoltage thresholds (24 V system).
 *
 * Rationale for chosen voltages:
 *   24 V nominal → six 18650 cells in 6S configuration (25.2 V full, 18.0 V empty).
 *   20.0 V warning ≈ 3.33 V/cell — cells are nearly depleted, reduce load to
 *       prolong remaining capacity and prevent deep-discharge damage.
 *   18.0 V critical ≈ 3.00 V/cell — absolute minimum safe cell voltage.
 *       Below this, cell chemistry degrades irreversibly and motor controller
 *       behaviour becomes unpredictable (brown-out risk).
 *
 * Hysteresis (0.5 V) prevents oscillation when voltage sags under load
 * then recovers during coast.  This is a common pattern in automotive BMS.
 *
 * Recovery from SAFE is intentionally blocked (non-auto-recovery):
 *   A critically depleted battery cannot reliably power actuators.
 *   Operator must recharge and reset the system to clear the fault.
 *   This follows the fail-safe philosophy used for CAN timeout and
 *   emergency stop in the existing safety architecture.                  */
#define BATTERY_UV_WARNING_V    20.0f   /* Enter DEGRADED below this      */
#define BATTERY_UV_CRITICAL_V   18.0f   /* Enter SAFE below this          */
#define BATTERY_UV_HYST_V       0.5f    /* Hysteresis band for recovery   */

/* Battery overvoltage thresholds (24 V system).
 *
 * Overvoltage can occur from:
 *   - Charger malfunction (no BMS cutoff)
 *   - Regenerative braking feeding energy back to battery
 *   - Voltage regulator failure on auxiliary power
 *
 * 30.0 V warning ≈ 5.0 V/cell — above max charge voltage (4.2 V/cell = 25.2 V).
 *   Indicates abnormal condition; reduce load and alert operator.
 * 35.0 V critical — risk of damage to 24 V electronics (capacitors, ICs).
 *   Immediate shutdown to protect components.
 *
 * Hysteresis (0.5 V) prevents oscillation during transient spikes.       */
#define BATTERY_OV_WARNING_V    30.0f   /* Enter DEGRADED above this      */
#define BATTERY_OV_CRITICAL_V   35.0f   /* Enter SAFE above this          */
#define BATTERY_OV_HYST_V       0.5f    /* Hysteresis band for recovery   */

/* Command-validation constants */
#define THROTTLE_MIN         0.0f
#define THROTTLE_MAX         100.0f
#define STEERING_RATE_MAX_DEG_PER_S  200.0f  /* max steering rate          */
#define STEERING_RATE_MIN_DT_S       0.001f /* ignore dt below 1 ms       */
#define MODE_CHANGE_MAX_SPEED_KMH 1.0f       /* speed below which mode OK  */

/* Steering command timeout — if no new steering command (0x101) is received
 * for this duration while CAN is otherwise alive, gradually return steering
 * to center.  Prevents stale steering angle if ESP32 stops sending 0x101
 * but keeps heartbeat alive (e.g., HMI crash, ESP32 task starvation).     */
#define STEERING_CMD_TIMEOUT_MS  500

/* Relay power sequencing delays (milliseconds) */
#define RELAY_MAIN_SETTLE_MS     50   /* inrush current settling time      */
#define RELAY_TRACTION_SETTLE_MS 20   /* contactor arc suppression delay   */

/* Relay health check thresholds (post-ACTIVE runtime validation).
 * Detects relay-not-closing condition using throttle demand vs motor
 * current observation.  Runs only in ACTIVE / DEGRADED / LIMP_HOME.
 *
 * Algorithm:
 *   1. Activation: throttle > RELAY_CHK_THROTTLE_MIN_PCT (primary) OR
 *      throttle > RELAY_CHK_THROTTLE_SPEED_PCT with speed > SPEED_MIN
 *      (secondary — catches relay failure while vehicle is in motion).
 *   2. After RELAY_CHK_DELAY_MS, compute TOTAL absolute motor current
 *      (INA226 ch 0-3 sum, not per-wheel average) and compare against
 *      RELAY_CHK_CURRENT_MIN_TOTAL_A.
 *   3. If below threshold for RELAY_CHK_DEBOUNCE_CYCLES consecutive
 *      calls, trigger SAFETY_ERROR_RELAY_OPEN → DEGRADED L3.
 *   4. Voltage correlation: skip check if battery voltage below
 *      RELAY_CHK_VOLTAGE_MIN_V (a depleted battery cannot deliver
 *      current regardless of relay state — false positive protection).
 *   5. Recovery hysteresis: fault clears ONLY after RELAY_CHK_RECOVERY_MS
 *      of sustained healthy current — prevents DEGRADED ↔ ACTIVE
 *      oscillation.
 *
 * Debounce: fault condition must persist for RELAY_CHK_DEBOUNCE_CYCLES
 * consecutive 10 ms checks to trigger — rejects transient glitches.
 *
 * Threshold rationale:
 *   Total current ≈ 3.0 A chosen because 4 × RS775 at 24 V under
 *   20 % throttle draw ≈ 1.5–2.5 A each → total ≈ 6–10 A.
 *   3.0 A total sum leaves ~50 % margin for single-wheel scenarios
 *   while remaining well above INA226 noise floor (~0.1 A per ch).    */
#define RELAY_CHK_THROTTLE_MIN_PCT  20.0f   /* Primary: min throttle %      */
#define RELAY_CHK_THROTTLE_SPEED_PCT 10.0f  /* Secondary: lower floor when
                                             * speed confirms motion        */
#define RELAY_CHK_SPEED_MIN_KMH     2.0f    /* Min speed for secondary
                                             * activation (km/h)            */
#define RELAY_CHK_DELAY_MS          300U    /* Settle time after activation  */
#define RELAY_CHK_CURRENT_MIN_TOTAL_A 3.0f  /* Total sum below this → relay
                                             * suspect (was 0.5 per-wheel)  */
#define RELAY_CHK_DEBOUNCE_CYCLES   3U      /* Consecutive fails to trigger */
#define RELAY_CHK_RECOVERY_MS       1500U   /* Hysteresis: sustain healthy
                                             * current this long to clear   */
#define RELAY_CHK_VOLTAGE_MIN_V     18.0f   /* Skip check below this battery
                                             * voltage (false positive guard)*/
#define SENSOR_TEMP_MIN_C    (-40.0f)
#define SENSOR_TEMP_MAX_C    125.0f   /* DS18B20 absolute range */
#define SENSOR_CURRENT_MAX_A       50.0f    /* motor channel plausibility ceiling  */
#define SENSOR_CURRENT_MAX_BATT_A 100.0f   /* battery channel: 100A sensor range  */
#define SENSOR_SPEED_MAX_KMH 25.0f    /* RS775 20000RPM / 1:75 gear → ~266 wheel RPM
                                        * × 1.1m circumf → ~17.6 km/h max.
                                        * 25 km/h gives ~40 % plausibility margin. */

/* ---- Module state ---- */
SafetyStatus_t safety_status = {0};
Safety_Error_t safety_error  = SAFETY_ERROR_NONE;

static SystemState_t system_state       = SYS_STATE_BOOT;
static volatile uint32_t last_can_rx_time = 0;  /* Updated from CAN_ProcessMessages() main loop */
static uint8_t  emergency_stopped       = 0;
static float    last_steering_cmd   = 0.0f;
static uint32_t last_steering_tick  = 0;

/* SAFETY FIX: Steering validity flag.
 * Set to 1 when steering timeout fires — prevents stale steering data
 * from being used by Safety_ValidateSteering().  Cleared when a new
 * valid steering command arrives via CAN.                              */
static uint8_t  steering_timed_out  = 0;

/* Consecutive-error counter for DEGRADED → SAFE escalation.
 * Traced to base firmware relays.cpp: consecutiveErrors.
 * Only modified from the main-loop safety checks (never from ISR).    */
static uint8_t  consecutive_errors      = 0;
static uint32_t last_error_tick         = 0;

/* ---- Non-blocking relay sequencer state machine ---- */
typedef enum {
    RELAY_SEQ_IDLE = 0,     /* All relays off, no sequence in progress   */
    RELAY_SEQ_MAIN_ON,      /* Main relay energised, waiting settle      */
    RELAY_SEQ_TRACTION_ON,  /* Traction relay energised, waiting settle  */
    RELAY_SEQ_COMPLETE       /* All relays on, sequence finished          */
} RelaySeqState_t;

static RelaySeqState_t relay_seq_state     = RELAY_SEQ_IDLE;
static uint32_t        relay_seq_timestamp = 0;

/* ---- Relay health check runtime state ----
 * Tracks whether the relay-open detection window is active and how
 * many consecutive 10 ms cycles the fault condition has persisted.
 * relay_chk_recovery_tick tracks the recovery hysteresis window —
 * the fault is only cleared after sustained healthy current.          */
static uint8_t  relay_chk_active       = 0;     /* 1 = timing window running  */
static uint32_t relay_chk_start_tick   = 0;     /* HAL_GetTick() at window start */
static uint8_t  relay_chk_debounce     = 0;     /* Consecutive fail counter   */
static uint32_t relay_chk_recovery_tick = 0;    /* 0 = not recovering; else
                                                  * HAL_GetTick() at start of
                                                  * healthy current window      */

/* Recovery debounce: require RECOVERY_HOLD_MS of clean operation
 * before transitioning DEGRADED → ACTIVE.  Prevents rapid state
 * oscillation when a sensor value fluctuates near a threshold.
 * Traced to limp_mode.cpp: STATE_HYSTERESIS_MS = 500.                 */
#define RECOVERY_HOLD_MS  500
static uint32_t recovery_clean_since    = 0;
static uint8_t  recovery_pending        = 0;  /* 1 = waiting for debounce */

/* LIMP_HOME → ACTIVE recovery debounce (CAN restoration).
 * Require sustained heartbeat presence for RECOVERY_HOLD_MS before
 * re-entering ACTIVE after CAN timeout or bus-off.  Prevents premature
 * reactivation from a single heartbeat in a flapping CAN bus.            */
static uint32_t limphome_recovery_since = 0;
static uint8_t  limphome_recovery_pending = 0;

/* ---- Granular degradation internal state (Phase 12) ----
 * These variables track the current degradation level and reason
 * internally.  The external CAN representation remains SYS_STATE_DEGRADED
 * for all levels — no CAN contract changes.                              */
static DegradedLevel_t  degraded_level   = DEGRADED_LEVEL_NONE;
static DegradedReason_t degraded_reason  = DEGRADED_REASON_NONE;
static uint32_t         degraded_telemetry_count = 0;  /* Total entries into degraded */

/* Per-wheel TCS reduction accumulator (persistent across calls).
 * Mirrors tcs_system.cpp WheelTCSState::powerReduction.
 * Declared here (module state section) so Safety_Init can reset them. */
static float tcs_reduction[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint32_t tcs_last_tick = 0;

/* Per-wheel ABS pulse state (non-blocking, timestamp-based).
 * Declared here so Safety_Init can reset them. */
static uint32_t abs_pulse_timer[4];     /* HAL_GetTick() at phase start   */
static uint8_t  abs_pulse_phase[4];     /* 1 = ON (reduced), 0 = OFF      */

/* ---- Obstacle safety — STM32 primary safety controller ---------- */
/* CAN obstacle frames from ESP32 are advisory only — never mandatory
 * for motion.  The STM32 implements a full autonomous obstacle safety
 * module with:
 *   - Physical plausibility validation (speed-based change limits)
 *   - Stuck sensor detection (distance static while vehicle moves)
 *   - Speed-dependent stopping distance (thresholds ∝ v²)
 *   - Temporal hysteresis (confirm presence and clearance over time)
 *   - Local state machine independent of CAN availability
 *   - Reverse escape preserved
 *   - Conservative fallback when sensor is invalid
 *
 * The vehicle remains safely drivable if ESP32, CAN bus, or HMI
 * completely disappear.  No motion immobilization — only controlled
 * slowdown.                                                            */

/* Fixed distance thresholds (floor values — dynamic thresholds may
 * be larger at higher speeds due to stopping-distance calculation).
 * 5 zones: emergency, critical, warning, caution, alert.              */
#define OBSTACLE_EMERGENCY_MM       200     /* < 200 mm → scale = 0.0         */
#define OBSTACLE_CRITICAL_MM        500     /* 200–500 mm → scale = 0.3       */
#define OBSTACLE_WARNING_MM         1000    /* 500–1000 mm → scale = 0.7      */
#define OBSTACLE_CAUTION_MM         1500    /* 1000–1500 mm → scale = 0.85    */
#define OBSTACLE_ALERT_MM           4000    /* 1500–4000 mm → scale = 0.95    */

/* Temporal hysteresis */
#define OBSTACLE_CONFIRM_MS         200     /* Confirm obstacle before acting  */
#define OBSTACLE_CLEAR_MS           1000    /* Confirm clearance before reset  */
#define OBSTACLE_RECOVERY_MM        500     /* Min distance to start clearing  */

/* CAN timeout — advisory.  When exceeded:
 *   - If obstacle was active: hold last scale (≤ OBSTACLE_FAULT_SCALE)
 *   - If no obstacle was active: scale → 1.0 (LIMP_HOME speed cap)
 * CAN loss alone is NOT a hazard, but an active obstacle must not be
 * forgotten just because CAN frames stopped arriving.                 */
#define OBSTACLE_CAN_TIMEOUT_MS     500

/* Physical plausibility: max approach rate between sensor and vehicle.
 * vehicle_speed + max_obstacle_speed.  A child + vehicle combined ≈
 * 8 m/s (29 km/h).  Anything faster is noise or sensor fault.        */
#define OBSTACLE_MAX_APPROACH_MMS   8000    /* 8 m/s in mm/s                  */

/* Stuck-sensor detection: if vehicle moves > 1 km/h but distance
 * has not changed by more than STUCK_THRESHOLD for STUCK_DURATION,
 * the sensor is declared stuck.                                       */
#define OBSTACLE_STUCK_THRESHOLD_MM 10
#define OBSTACLE_STUCK_DURATION_MS  1000
#define OBSTACLE_STUCK_MIN_SPEED_KMH 1.0f

/* Speed-dependent stopping distance: d = v² / (2·a) + margin.
 * Assumed deceleration 3 m/s² (gentle for child vehicle).             */
#define OBSTACLE_STOP_DECEL_MMS2    3000.0f /* 3 m/s² in mm/s²               */
#define OBSTACLE_STOP_MARGIN_MM     200     /* Safety margin added            */

/* Sensor-fault conservative scale — vehicle remains mobile.           */
#define OBSTACLE_FAULT_SCALE        0.3f

/* Preemptive scale floor during CONFIRMING / CLEARING states.
 * Limits reduction to at most 0.7 (gentle) while temporal
 * confirmation is still pending.                                      */
#define OBSTACLE_PREEMPTIVE_FLOOR   0.7f
static inline float obstacle_preemptive_scale(float target)
{
    return (target < OBSTACLE_PREEMPTIVE_FLOOR) ? OBSTACLE_PREEMPTIVE_FLOOR
                                                 : target;
}

/* ---- CAN advisory state ---- */
static uint32_t obstacle_last_rx_tick    = 0;      /* Last 0x208 reception     */
static uint16_t obstacle_distance_mm     = 0xFFFF; /* Last reported distance   */
static uint8_t  obstacle_zone            = 0;      /* Last zone (0–5)          */
static uint8_t  obstacle_sensor_healthy  = 0;      /* Health flag from ESP32   */
static uint8_t  obstacle_last_counter    = 0;      /* Rolling counter          */
static uint8_t  obstacle_stale_count     = 0;      /* Consecutive stale frames */
static uint8_t  obstacle_data_valid      = 0;      /* 1 = ≥1 msg received      */

/* ---- Local state machine (independent of CAN) ---- */
static ObstacleState_t obstacle_state    = OBS_STATE_NO_SENSOR;
static uint32_t obstacle_confirm_tick    = 0;      /* Confirmation start time  */
static uint32_t obstacle_clear_tick      = 0;      /* Clearance start time     */
static uint8_t  obstacle_forward_blocked = 0;      /* Forward blocked flag     */

/* ---- Plausibility & stuck detection ---- */
static uint16_t obstacle_prev_distance   = 0xFFFF; /* Previous valid distance  */
static uint32_t obstacle_prev_dist_tick  = 0;      /* Time of prev distance    */
static uint32_t obstacle_stuck_since     = 0;      /* When distance froze      */
static uint8_t  obstacle_plausible       = 1;      /* Current data plausible   */

/* ---- Validated distance used by state machine ---- */
static uint16_t obstacle_validated_mm    = 0xFFFF; /* After plausibility check */

/* ---- ESP32 obstacle safety state (0x209, cross-validation) ---- */
static uint8_t  esp32_obs_zone           = 0;      /* Zone reported by ESP32   */
static uint8_t  esp32_obs_sensor_status  = 0;      /* 0=WAIT, 1=INVALID, 2=OK */
static uint8_t  esp32_obs_stuck          = 0;      /* ESP32 stuck flag         */
static uint32_t esp32_obs_last_rx_tick   = 0;      /* Last 0x209 reception     */

/* ---- Child reaction detection ----
 * When the child rapidly releases the pedal (drop > THRESHOLD in WINDOW),
 * the obstacle safety factors in warning/caution zones are tightened.
 * This detects an instinctive reaction to a perceived obstacle.           */
#define CHILD_REACTION_THRESHOLD    10.0f   /* Pedal drop > 10% triggers    */
#define CHILD_REACTION_MIN_PEDAL    10.0f   /* Min pedal % before detection */
#define CHILD_REACTION_WINDOW_MS    500     /* Detection window (ms)        */
#define CHILD_REACTION_BOOST_MS     2000    /* How long tighter factors last */
#define CHILD_REACTION_WARNING_SCALE  0.5f  /* Warning zone during reaction */
#define CHILD_REACTION_CAUTION_SCALE  0.7f  /* Caution zone during reaction */

static float    child_prev_pedal_pct     = 0.0f;  /* Pedal % at window start */
static uint32_t child_pedal_sample_tick  = 0;      /* When sample was taken   */
static uint8_t  child_reaction_active    = 0;      /* 1 = reaction detected   */
static uint32_t child_reaction_start     = 0;      /* When reaction started   */

/* ================================================================== */
/*  State Machine                                                      */
/* ================================================================== */

SystemState_t Safety_GetState(void) { return system_state; }

void Safety_SetState(SystemState_t state)
{
    if (state == system_state) return;

    /* Only allow forward transitions and recovery transitions:
     *   SAFE→ACTIVE, DEGRADED→ACTIVE, LIMP_HOME→ACTIVE               */
    switch (state) {
        case SYS_STATE_STANDBY:
            if (system_state == SYS_STATE_BOOT)
                system_state = SYS_STATE_STANDBY;
            break;

        case SYS_STATE_ACTIVE:
            if (system_state == SYS_STATE_STANDBY ||
                system_state == SYS_STATE_SAFE    ||
                system_state == SYS_STATE_DEGRADED ||
                system_state == SYS_STATE_LIMP_HOME) {
                /* Require no active faults to enter ACTIVE */
                if (safety_error == SAFETY_ERROR_NONE) {
                    system_state = SYS_STATE_ACTIVE;
                    consecutive_errors = 0;
                    /* Clear internal degradation level on recovery */
                    degraded_level  = DEGRADED_LEVEL_NONE;
                    degraded_reason = DEGRADED_REASON_NONE;
                    Relay_PowerUp();
                }
            }
            break;

        /* DEGRADED: limp / reduced-power mode.  Vehicle can still
         * "drive home".  Traced to base firmware limp_mode.cpp.
         * Unlike SAFE, relays stay ON and commands are accepted
         * (with power/speed limits applied by Safety_ValidateThrottle). */
        case SYS_STATE_DEGRADED:
            if (system_state == SYS_STATE_ACTIVE ||
                system_state == SYS_STATE_STANDBY) {
                system_state = SYS_STATE_DEGRADED;
                /* Do NOT call Safety_FailSafe() — keep relays on.
                 * Traction demand is limited via Safety_ValidateThrottle(). */
            }
            break;

        case SYS_STATE_SAFE:
            if (system_state == SYS_STATE_ACTIVE  ||
                system_state == SYS_STATE_STANDBY  ||
                system_state == SYS_STATE_DEGRADED ||
                system_state == SYS_STATE_LIMP_HOME) {
                system_state = SYS_STATE_SAFE;
                degraded_level  = DEGRADED_LEVEL_NONE;
                degraded_reason = DEGRADED_REASON_NONE;
                Safety_FailSafe();
            }
            break;

        case SYS_STATE_ERROR:
            system_state = SYS_STATE_ERROR;
            degraded_level  = DEGRADED_LEVEL_NONE;
            degraded_reason = DEGRADED_REASON_NONE;
            Safety_PowerDown();
            break;

        /* LIMP_HOME: CAN-loss degraded mode — minimal drivable.
         * Relays stay ON.  Local pedal input accepted with strong
         * clamps.  Vehicle remains mobile without CAN/ESP32.
         * Communication loss is NOT a hazard.                          */
        case SYS_STATE_LIMP_HOME:
            if (system_state == SYS_STATE_STANDBY  ||
                system_state == SYS_STATE_ACTIVE   ||
                system_state == SYS_STATE_DEGRADED ||
                system_state == SYS_STATE_SAFE) {
                system_state = SYS_STATE_LIMP_HOME;
                degraded_level  = DEGRADED_LEVEL_NONE;
                degraded_reason = DEGRADED_REASON_NONE;
                /* Keep relays on — vehicle must remain drivable */
                Relay_PowerUp();
            }
            break;

        default:
            break;
    }
}

bool Safety_IsCommandAllowed(void)
{
    return (system_state == SYS_STATE_ACTIVE ||
            system_state == SYS_STATE_DEGRADED);
}

/* Motion is allowed in ACTIVE, DEGRADED, and LIMP_HOME.
 * In LIMP_HOME, only local pedal input drives the motors —
 * CAN commands are rejected (Safety_IsCommandAllowed returns false).   */
bool Safety_IsMotionAllowed(void)
{
    return (system_state == SYS_STATE_ACTIVE   ||
            system_state == SYS_STATE_DEGRADED ||
            system_state == SYS_STATE_LIMP_HOME);
}

bool Safety_IsDegraded(void)
{
    return (system_state == SYS_STATE_DEGRADED);
}

bool Safety_IsLimpHome(void)
{
    return (system_state == SYS_STATE_LIMP_HOME);
}

/* Return power-limit multiplier for the current state.
 * ACTIVE    → 1.0 (100 %)
 * DEGRADED  → per-level scaling (L1=70%, L2=50%, L3=40%)
 *             Falls back to DEGRADED_POWER_LIMIT_PCT if level not set.
 * LIMP_HOME → 0.20 (20 %) — strong clamp for walking-speed safety
 * Others    → 0.0 (commands rejected upstream)                          */
float Safety_GetPowerLimitFactor(void)
{
    if (system_state == SYS_STATE_ACTIVE)    return 1.0f;
    if (system_state == SYS_STATE_LIMP_HOME) return LIMP_HOME_TORQUE_LIMIT_FACTOR;
    if (system_state == SYS_STATE_DEGRADED) {
        switch (degraded_level) {
            case DEGRADED_L1: return DEGRADED_L1_POWER_PCT / 100.0f;
            case DEGRADED_L2: return DEGRADED_L2_POWER_PCT / 100.0f;
            case DEGRADED_L3: return DEGRADED_L3_POWER_PCT / 100.0f;
            default:          return DEGRADED_POWER_LIMIT_PCT / 100.0f;
        }
    }
    return 0.0f;
}

/* Return steering-assist multiplier for the current state (Phase 12).
 * ACTIVE    → 1.0 (100 %)
 * DEGRADED  → per-level scaling (L1=85%, L2=70%, L3=60%)
 * LIMP_HOME → 1.0 (steering fully operational — critical for safety)
 * Others    → 0.0                                                       */
float Safety_GetSteeringLimitFactor(void)
{
    if (system_state == SYS_STATE_ACTIVE)    return 1.0f;
    if (system_state == SYS_STATE_LIMP_HOME) return 1.0f;
    if (system_state == SYS_STATE_DEGRADED) {
        switch (degraded_level) {
            case DEGRADED_L1: return DEGRADED_L1_STEERING_PCT / 100.0f;
            case DEGRADED_L2: return DEGRADED_L2_STEERING_PCT / 100.0f;
            case DEGRADED_L3: return DEGRADED_L3_STEERING_PCT / 100.0f;
            default:          return 0.6f;  /* Legacy 40% reduction */
        }
    }
    return 0.0f;
}

/* Return traction-cap multiplier for the current state (Phase 12).
 * ACTIVE    → 1.0 (100 %)
 * DEGRADED  → per-level scaling (L1=80%, L2=60%, L3=50%)
 * LIMP_HOME → 0.20 (20 %) — matches power limit for walking speed
 * Others    → 0.0                                                       */
float Safety_GetTractionCapFactor(void)
{
    if (system_state == SYS_STATE_ACTIVE)    return 1.0f;
    if (system_state == SYS_STATE_LIMP_HOME) return LIMP_HOME_TORQUE_LIMIT_FACTOR;
    if (system_state == SYS_STATE_DEGRADED) {
        switch (degraded_level) {
            case DEGRADED_L1: return DEGRADED_L1_TRACTION_PCT / 100.0f;
            case DEGRADED_L2: return DEGRADED_L2_TRACTION_PCT / 100.0f;
            case DEGRADED_L3: return DEGRADED_L3_TRACTION_PCT / 100.0f;
            default:          return DEGRADED_SPEED_LIMIT_PCT / 100.0f;
        }
    }
    return 0.0f;
}

/* Granular degradation accessors (Phase 12) */
DegradedLevel_t  Safety_GetDegradedLevel(void)          { return degraded_level;  }
DegradedReason_t Safety_GetDegradedReason(void)         { return degraded_reason; }
uint32_t         Safety_GetDegradedTelemetryCount(void)  { return degraded_telemetry_count; }

void Safety_SetDegradedLevel(DegradedLevel_t level, DegradedReason_t reason)
{
    /* Only escalate — never reduce level while still in DEGRADED.
     * Recovery to ACTIVE resets the level via Safety_SetState().
     * Telemetry counter increments only on initial entry (NONE→Lx),
     * not on intra-degraded escalation (L1→L2).                        */
    if (level > degraded_level) {
        if (degraded_level == DEGRADED_LEVEL_NONE) {
            sat_inc_u32(&degraded_telemetry_count);
        }
        degraded_level  = level;
        degraded_reason = reason;
    }
}

uint8_t Safety_GetFaultFlags(void)
{
    uint8_t flags = 0;
    if (safety_error == SAFETY_ERROR_CAN_TIMEOUT)  flags |= FAULT_CAN_TIMEOUT;
    if (safety_error == SAFETY_ERROR_OVERTEMP)      flags |= FAULT_TEMP_OVERLOAD;
    if (safety_error == SAFETY_ERROR_OVERCURRENT)   flags |= FAULT_CURRENT_OVERLOAD;
    if (safety_error == SAFETY_ERROR_CENTERING)     flags |= FAULT_CENTERING;

    /* Use service mode per-module fault tracking for more granular
     * fault flags (encoder vs wheel speed differentiation) */
    if (ServiceMode_GetFault(MODULE_STEER_ENCODER) != MODULE_FAULT_NONE)
        flags |= FAULT_ENCODER_ERROR;
    for (uint8_t i = 0; i < 4; i++) {
        if (ServiceMode_GetFault((ModuleID_t)(MODULE_WHEEL_SPEED_FL + i)) != MODULE_FAULT_NONE) {
            flags |= FAULT_WHEEL_SENSOR;
            break;
        }
    }
    if (safety_status.abs_active)                   flags |= FAULT_ABS_ACTIVE;
    if (safety_status.tcs_active)                   flags |= FAULT_TCS_ACTIVE;
    return flags;
}

/* ================================================================== */
/*  Relay Power Sequencing                                             */
/* ================================================================== */

void Relay_PowerUp(void)
{
    /* Initiate non-blocking power-up sequence.
     * The actual relay transitions are driven by Relay_SequencerUpdate()
     * called from the 10 ms safety loop.
     * Re-entry safe: if already sequencing or complete, do nothing.     */
    if (relay_seq_state != RELAY_SEQ_IDLE) {
        return;  /* Sequence already in progress or complete */
    }

    /* Step 1: Energise main relay and record timestamp */
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_MAIN, GPIO_PIN_SET);
    relay_seq_state     = RELAY_SEQ_MAIN_ON;
    relay_seq_timestamp = HAL_GetTick();
}

void Relay_SequencerUpdate(void)
{
    /* Non-blocking relay sequencer — call from the 10 ms safety loop.
     * Progresses through the power-up sequence using timestamps:
     *   MAIN_ON  →  (50 ms)  →  TRACTION_ON  →  (20 ms)  →  COMPLETE
     * IDLE and COMPLETE are no-ops.                                     */
    uint32_t now = HAL_GetTick();

    switch (relay_seq_state) {
        case RELAY_SEQ_MAIN_ON:
            if ((now - relay_seq_timestamp) >= RELAY_MAIN_SETTLE_MS) {
                HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_SET);
                relay_seq_state     = RELAY_SEQ_TRACTION_ON;
                relay_seq_timestamp = now;
            }
            break;

        case RELAY_SEQ_TRACTION_ON:
            if ((now - relay_seq_timestamp) >= RELAY_TRACTION_SETTLE_MS) {
                HAL_GPIO_WritePin(GPIOC, PIN_RELAY_DIR, GPIO_PIN_SET);
                relay_seq_state = RELAY_SEQ_COMPLETE;
            }
            break;

        case RELAY_SEQ_IDLE:
        case RELAY_SEQ_COMPLETE:
        default:
            break;
    }
}

void Relay_PowerDown(void)
{
    /* Reverse order: Direction → Traction → Main.
     * Cancels any in-progress power-up sequence immediately.            */
    relay_seq_state = RELAY_SEQ_IDLE;
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_DIR,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_MAIN, GPIO_PIN_RESET);
}

/* ================================================================== */
/*  Command Validation Gate                                            */
/* ================================================================== */

float Safety_ValidateThrottle(float requested_pct)
{
    /* Reject commands when not ACTIVE or DEGRADED */
    if (!Safety_IsCommandAllowed()) return 0.0f;

    /* Clamp to valid range */
    if (requested_pct < THROTTLE_MIN) requested_pct = THROTTLE_MIN;
    if (requested_pct > THROTTLE_MAX) requested_pct = THROTTLE_MAX;

    /* Per-wheel ABS/TCS modulation is now handled in Traction_Update()
     * via safety_status.wheel_scale[].  No global override here.
     * Aligned with base firmware: abs_system.cpp modulateBrake() and
     * tcs_system.cpp modulatePower() operate per-wheel, not globally.
     *
     * Global fallback (all 4 wheels slipping) is handled directly in
     * ABS_Update / TCS_Update via Traction_SetDemand().               */

    /* Degraded-mode power limit (limp_mode.cpp: POWER_LIMP = 0.4) */
    float limit = Safety_GetPowerLimitFactor();
    return requested_pct * limit;
}

float Safety_ValidateSteering(float requested_deg)
{
    /* Reject commands when not ACTIVE or DEGRADED */
    if (!Safety_IsCommandAllowed()) return Steering_GetCurrentAngle();

    /* SAFETY FIX: A new valid steering command clears the timeout flag.
     * This ensures that a fresh CAN 0x101 resumes normal steering
     * after a previous timeout forced center.                          */
    steering_timed_out = 0;

    /* Clamp to mechanical limits */
    if (requested_deg < -MAX_STEER_DEG) requested_deg = -MAX_STEER_DEG;
    if (requested_deg >  MAX_STEER_DEG) requested_deg =  MAX_STEER_DEG;

    /* Rate-limit to prevent violent movements */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - last_steering_tick) / 1000.0f;
    if (dt > STEERING_RATE_MIN_DT_S) {
        float max_delta = STEERING_RATE_MAX_DEG_PER_S * dt;
        float delta = requested_deg - last_steering_cmd;
        if (delta >  max_delta) requested_deg = last_steering_cmd + max_delta;
        if (delta < -max_delta) requested_deg = last_steering_cmd - max_delta;
    }
    last_steering_cmd  = requested_deg;
    last_steering_tick = now;

    return requested_deg;
}

/**
 * @brief  Check for stale steering commands.
 *
 * Called from the 10 ms safety loop.  If the last steering command
 * (CAN 0x101) is older than STEERING_CMD_TIMEOUT_MS and the system is
 * in ACTIVE or DEGRADED state, gradually return steering to center.
 *
 * This catches the scenario where the ESP32 stops sending steering
 * updates (HMI crash, task starvation) but keeps the heartbeat alive,
 * so the CAN timeout does NOT fire.  Without this check, the last
 * steering angle persists indefinitely while throttle continues.
 */
void Safety_CheckSteeringTimeout(void)
{
    if (!Safety_IsCommandAllowed()) return;

    uint32_t now = HAL_GetTick();
    if (last_steering_tick > 0 &&
        (now - last_steering_tick) > STEERING_CMD_TIMEOUT_MS) {
        /* Steering commands have gone stale — return to center.
         * Use the existing rate-limited path to avoid a sudden jerk.
         *
         * SAFETY FIX: Also mark steering as timed-out so that
         * Safety_ValidateSteering() rejects any use of the stale
         * last_steering_cmd value.  This flag is cleared when a
         * new valid CAN 0x101 arrives.                              */
        steering_timed_out = 1;
        last_steering_cmd  = 0.0f;
        Steering_SetAngle(0.0f);
        /* Update the tick so this fires once per timeout period,
         * not continuously every 10 ms.                              */
        last_steering_tick = now;
    }
}

bool Safety_IsSteeringTimedOut(void)
{
    return (steering_timed_out != 0);
}

bool Safety_ValidateModeChange(bool enable_4x4, bool tank_turn)
{
    (void)enable_4x4;  /* Reserved for future 4×4 mode validation */
    (void)tank_turn;    /* Reserved for future tank-turn validation */

    /* Reject commands when not ACTIVE or DEGRADED */
    if (!Safety_IsCommandAllowed()) return false;

    /* Mode change only allowed at very low speed */
    float avg_speed = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                       Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;
    avg_speed = sanitize_float(avg_speed, MODE_CHANGE_MAX_SPEED_KMH + 1.0f);
    if (avg_speed > MODE_CHANGE_MAX_SPEED_KMH) return false;

    return true;
}

/* ================================================================== */
/*  Initialization                                                     */
/* ================================================================== */

void Safety_Init(void)
{
    safety_status.abs_active = false;
    safety_status.tcs_active = false;
    safety_status.abs_wheel_mask = 0;
    safety_status.tcs_wheel_mask = 0;
    safety_status.abs_activation_count = 0;
    safety_status.tcs_activation_count = 0;
    safety_status.obstacle_scale = 1.0f;
    for (uint8_t i = 0; i < 4; i++) {
        safety_status.wheel_scale[i] = 1.0f;
        tcs_reduction[i] = 0.0f;
        abs_pulse_timer[i] = HAL_GetTick();
        abs_pulse_phase[i] = 0U;
    }
    tcs_last_tick    = HAL_GetTick();
    safety_error     = SAFETY_ERROR_NONE;
    emergency_stopped = 0;
    last_can_rx_time  = HAL_GetTick();
    last_steering_cmd = 0.0f;
    last_steering_tick = HAL_GetTick();
    steering_timed_out = 0;
    consecutive_errors = 0;
    last_error_tick    = 0;
    recovery_clean_since = 0;
    recovery_pending     = 0;
    obstacle_last_rx_tick    = 0;
    obstacle_distance_mm     = 0xFFFF;
    obstacle_zone            = 0;
    obstacle_sensor_healthy  = 0;
    obstacle_last_counter    = 0;
    obstacle_stale_count     = 0;
    obstacle_data_valid      = 0;
    obstacle_state           = OBS_STATE_NO_SENSOR;
    obstacle_confirm_tick    = 0;
    obstacle_clear_tick      = 0;
    obstacle_forward_blocked = 0;
    obstacle_prev_distance   = 0xFFFF;
    obstacle_prev_dist_tick  = 0;
    obstacle_stuck_since     = 0;
    obstacle_plausible       = 1;
    obstacle_validated_mm    = 0xFFFF;
    esp32_obs_zone           = 0;
    esp32_obs_sensor_status  = 0;
    esp32_obs_stuck          = 0;
    esp32_obs_last_rx_tick   = 0;
    child_prev_pedal_pct     = 0.0f;
    child_pedal_sample_tick  = 0;
    child_reaction_active    = 0;
    child_reaction_start     = 0;
    relay_chk_active         = 0;
    relay_chk_start_tick     = 0;
    relay_chk_debounce       = 0;
    relay_chk_recovery_tick  = 0;
    system_state      = SYS_STATE_BOOT;
}

/* ---- ABS --------------------------------------------------------- */

/* ---- ABS Pulse Modulation ----------------------------------------
 *
 * Previous behaviour: wheel_scale = 0.0 (full torque cut) whenever
 * slip exceeded the threshold.  This was too aggressive — a 100 %
 * cut removes all motor torque instantly, preventing the tyre from
 * recovering traction smoothly.  On real vehicles ABS works by
 * rapidly cycling brake pressure (pulse modulation) so the wheel
 * alternates between a reduced-torque phase and a recovery phase,
 * maintaining directional control and shortening stopping distance.
 *
 * New behaviour (aligned with reference firmware abs_system.cpp):
 *   pressureReduction = 0.30 → 30 % reduction during the ON phase.
 *   Pulse period       = 80 ms  (square-wave cycle).
 *   ON ratio           = 60 %   → 48 ms reduced, 32 ms recovery.
 *
 * During the ON phase wheel_scale = 0.70 (1.0 − 0.30), allowing
 * the motor to provide 70 % torque.  During the OFF phase
 * wheel_scale = 1.0 (full torque) so the wheel can spin back up.
 * The rapid cycling improves grip recovery compared to a sustained
 * full cut while still limiting lock-up torque.
 *
 * 30 % reduction was chosen because:
 *   1. It matches the reference firmware abs_system.cpp value.
 *   2. It is aggressive enough to break the lock-up cycle.
 *   3. It is mild enough to preserve steering authority.
 *
 * Implementation uses HAL_GetTick() for non-blocking timing.
 * Per-wheel state (abs_pulse_timer[], abs_pulse_phase[]) ensures
 * independent modulation on each corner.
 * ----------------------------------------------------------------- */

#define ABS_BASE_REDUCTION      0.30f   /* 30 % reduction (ref firmware)  */
#define ABS_PULSE_PERIOD_MS     80      /* total pulse cycle in ms        */
#define ABS_PULSE_ON_RATIO      0.6f    /* 60 % of period = reduced phase */

void ABS_Update(void)
{
    /* Skip if ABS module is disabled (service mode) */
    if (!ServiceMode_IsEnabled(MODULE_ABS)) {
        safety_status.abs_active = false;
        safety_status.abs_wheel_mask = 0;
        return;
    }

    /* Implicit dependency: ABS requires wheel speed sensors.
     * If wheel speed modules are disabled (ServiceMode), GetSpeed returns 0.0,
     * causing avg < 10.0 → ABS self-deactivates gracefully.
     * The speed gate in CAN_ID_SERVICE_CMD (can_handler.c) prevents disabling
     * wheel speed sensors while the vehicle is in motion (avg_spd > 0.5),
     * so ABS is always available when it matters.                            */
    float spd[4];
    spd[0] = Wheel_GetSpeed_FL();
    spd[1] = Wheel_GetSpeed_FR();
    spd[2] = Wheel_GetSpeed_RL();
    spd[3] = Wheel_GetSpeed_RR();

    float avg = (spd[0] + spd[1] + spd[2] + spd[3]) / 4.0f;
    if (avg < 10.0f) {         /* abs_system.cpp: minSpeedKmh = 10.0f */
        safety_status.abs_active = false;
        safety_status.abs_wheel_mask = 0;
        /* Do NOT reset wheel_scale here — TCS_Update may have active
         * reductions that must not be overwritten.  ABS-specific
         * scales are restored in the per-wheel loop below.            */
        return;
    }

    uint32_t now = HAL_GetTick();
    uint32_t on_duration  = (uint32_t)(ABS_PULSE_PERIOD_MS * ABS_PULSE_ON_RATIO);
    uint32_t off_duration = (uint32_t)(ABS_PULSE_PERIOD_MS) - on_duration;

    uint8_t mask = 0;
    for (uint8_t i = 0; i < 4; i++) {
        float slip = ((avg - spd[i]) * 100.0f) / avg;
        if (slip > (float)ABS_SLIP_THRESHOLD) {
            mask |= (1U << i);

            /* Advance pulse state machine (non-blocking square-wave).
             * ON phase  → wheel_scale = 1.0 − ABS_BASE_REDUCTION (70 %)
             * OFF phase → wheel_scale = 1.0 (full torque recovery)      */
            uint32_t elapsed = now - abs_pulse_timer[i];
            if (abs_pulse_phase[i]) {
                /* Currently in ON (reduced) phase */
                if (elapsed >= on_duration) {
                    abs_pulse_phase[i] = 0U;
                    abs_pulse_timer[i] = now;
                }
            } else {
                /* Currently in OFF (recovery) phase */
                if (elapsed >= off_duration) {
                    abs_pulse_phase[i] = 1U;
                    abs_pulse_timer[i] = now;
                }
            }

            if (abs_pulse_phase[i]) {
                safety_status.wheel_scale[i] = 1.0f - ABS_BASE_REDUCTION;
            } else {
                safety_status.wheel_scale[i] = 1.0f;
            }
        } else {
            /* Wheel not locking — restore full power and reset pulse.
             * TCS_Update runs after ABS_Update and may further reduce
             * this value if the wheel is spinning (TCS takes the min). */
            safety_status.wheel_scale[i] = 1.0f;
            abs_pulse_timer[i] = now;
            abs_pulse_phase[i] = 0U;
        }
    }

    if (mask) {
        safety_status.abs_active = true;
        safety_status.abs_wheel_mask = mask;
        sat_inc_u32(&safety_status.abs_activation_count);
        /* Global fallback: if ALL wheels lock, apply global throttle
         * cut as a last-resort safety measure (vehicle is on ice or
         * sensors are unreliable).                                    */
        if (mask == 0x0F) {
            Traction_SetDemand(0);
        }
        /* Otherwise: per-wheel scale is applied in Traction_Update(). */
    } else {
        safety_status.abs_active = false;
        safety_status.abs_wheel_mask = 0;
    }
}

bool ABS_IsActive(void)  { return safety_status.abs_active; }
void ABS_Reset(void)
{
    safety_status.abs_active = false;
    safety_status.abs_wheel_mask = 0;
    for (uint8_t i = 0; i < 4; i++) {
        safety_status.wheel_scale[i] = 1.0f;
        abs_pulse_timer[i] = HAL_GetTick();
        abs_pulse_phase[i] = 0U;
    }
}

/* ---- TCS --------------------------------------------------------- */

/* Per-wheel TCS power reduction, aligned with base firmware
 * tcs_system.cpp:
 *   aggressiveReduction = 40.0f  → initial 40 % cut (scale = 0.6)
 *   smoothReduction     =  5.0f  → +5 % per cycle while still slipping
 *   max reduction       = 80.0f  → floor scale = 0.2
 *   recoveryRatePerSec  = 25.0f  → 25 %/s recovery when slip clears
 *
 * These are NOT new thresholds — they are taken directly from the
 * base firmware TCSSystem::Config defaults.                          */
#define TCS_INITIAL_REDUCTION   0.40f   /* 40 % power cut on activation */
#define TCS_SMOOTH_REDUCTION    0.05f   /* 5 % additional per cycle     */
#define TCS_MAX_REDUCTION       0.80f   /* Maximum 80 % power cut       */
#define TCS_RECOVERY_RATE_PER_S 0.25f   /* 25 %/s recovery rate         */

void TCS_Update(void)
{
    /* Skip if TCS module is disabled (service mode) */
    if (!ServiceMode_IsEnabled(MODULE_TCS)) {
        safety_status.tcs_active = false;
        safety_status.tcs_wheel_mask = 0;
        for (uint8_t i = 0; i < 4; i++)
            tcs_reduction[i] = 0.0f;
        return;
    }

    /* Implicit dependency: TCS requires wheel speed sensors.
     * If wheel speed modules are disabled (ServiceMode), GetSpeed returns 0.0,
     * causing avg < 3.0 → TCS self-deactivates gracefully.
     * The speed gate in CAN_ID_SERVICE_CMD (can_handler.c) prevents disabling
     * wheel speed sensors while the vehicle is in motion (avg_spd > 0.5),
     * so TCS is always available when it matters.                            */
    float spd[4];
    spd[0] = Wheel_GetSpeed_FL();
    spd[1] = Wheel_GetSpeed_FR();
    spd[2] = Wheel_GetSpeed_RL();
    spd[3] = Wheel_GetSpeed_RR();

    float avg = (spd[0] + spd[1] + spd[2] + spd[3]) / 4.0f;
    if (avg < 3.0f) {          /* tcs_system.cpp: minSpeedKmh = 3.0f */
        safety_status.tcs_active = false;
        safety_status.tcs_wheel_mask = 0;
        for (uint8_t i = 0; i < 4; i++)
            tcs_reduction[i] = 0.0f;
        return;
    }

    /* Delta time for recovery ramp (tcs_system.cpp uses millis delta) */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - tcs_last_tick) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;  /* Guard, same as base */
    tcs_last_tick = now;

    uint8_t mask = 0;
    for (uint8_t i = 0; i < 4; i++) {
        float slip = ((spd[i] - avg) * 100.0f) / avg;
        if (slip > (float)TCS_SLIP_THRESHOLD) {
            mask |= (1U << i);

            if (tcs_reduction[i] < 0.01f) {
                /* First activation — aggressive initial cut.
                 * tcs_system.cpp: aggressiveReduction = 40.0f          */
                tcs_reduction[i] = TCS_INITIAL_REDUCTION;
            } else {
                /* Already active — smooth progressive reduction.
                 * tcs_system.cpp: smoothReduction = 5.0f               */
                tcs_reduction[i] += TCS_SMOOTH_REDUCTION;
            }
            /* Clamp to maximum reduction (tcs_system.cpp: 80 %).       */
            if (tcs_reduction[i] > TCS_MAX_REDUCTION)
                tcs_reduction[i] = TCS_MAX_REDUCTION;
        } else {
            /* Slip under control — gradually recover power.
             * tcs_system.cpp: recoveryRatePerSec = 25.0f               */
            if (tcs_reduction[i] > 0.0f) {
                tcs_reduction[i] -= TCS_RECOVERY_RATE_PER_S * dt;
                if (tcs_reduction[i] < 0.0f)
                    tcs_reduction[i] = 0.0f;
            }
        }

        /* Compute per-wheel scale.  ABS_Update runs first and may have
         * already set wheel_scale[i] < 1.0.  Take the minimum of ABS
         * and TCS so the most restrictive intervention wins.           */
        float tcs_scale = 1.0f - tcs_reduction[i];
        if (tcs_scale < safety_status.wheel_scale[i])
            safety_status.wheel_scale[i] = tcs_scale;
    }

    if (mask) {
        safety_status.tcs_active = true;
        safety_status.tcs_wheel_mask = mask;
        sat_inc_u32(&safety_status.tcs_activation_count);
        /* Global fallback: if ALL wheels spin, apply global limit as
         * last-resort safety (all traction lost).                     */
        if (mask == 0x0F) {
            Traction_SetDemand(Pedal_GetPercent() * (1.0f - TCS_MAX_REDUCTION));
        }
        /* Otherwise: per-wheel scale is applied in Traction_Update(). */
    } else {
        safety_status.tcs_active = false;
        safety_status.tcs_wheel_mask = 0;
    }
}

bool TCS_IsActive(void) { return safety_status.tcs_active; }
void TCS_Reset(void)    { safety_status.tcs_active = false; safety_status.tcs_wheel_mask = 0; for (uint8_t i = 0; i < 4; i++) tcs_reduction[i] = 0.0f; }

/* ---- Overcurrent ------------------------------------------------- */

/* Overcurrent reaction path:
 *   1. Single event  → DEGRADED (power-limited via Safety_ValidateThrottle)
 *   2. ≥2 consecutive → DEGRADED L3 (more restrictive limits)
 *   3. ≥3 consecutive → SAFE → Safety_FailSafe():
 *        – Traction_EmergencyStop() disables all motor PWM + H-bridge EN
 *        – Relay_PowerDown() de-energises main/traction/direction relays
 *        – Steering centred or neutralised
 *   Hardware fuses and BTS7960 internal current limiting handle sub-ms
 *   transients; this software path protects against sustained faults.  */

void Safety_CheckCurrent(void)
{
    for (uint8_t i = 0; i < NUM_INA226; i++) {
        /* Skip disabled current sensors (service mode).
         * Traced to base firmware car_sensors.cpp:
         *   if (!cfg.currentSensorsEnabled) { return 0.0f; }         */
        ModuleID_t mod = (ModuleID_t)(MODULE_CURRENT_SENSOR_0 + i);
        if (!ServiceMode_IsEnabled(mod)) continue;

        float amps = Current_GetAmps(i);
        /* Use per-channel overcurrent threshold: battery channel carries
         * total system current (motors + steering + electronics) and must
         * allow up to 100 A.                                               */
        float limit = (i == INA226_CHANNEL_BATTERY)
                    ? MAX_CURRENT_BATT_A : MAX_CURRENT_A;
        /* Check absolute overcurrent — reverse current through the shunt
         * (regenerative braking, back-EMF) must also be detected.
         * NaN/Inf hardening: invalid ADC reading → treat as overcurrent
         * (safe side — prefer false positive over silent pass-through).   */
        if (isnan(amps) || isinf(amps) || amps > limit || amps < -limit) {
            ServiceMode_SetFault(mod, MODULE_FAULT_ERROR);
            Safety_SetError(SAFETY_ERROR_OVERCURRENT);
            /* Count consecutive errors — escalate to SAFE only after
             * CONSECUTIVE_ERROR_THRESHOLD (traced to relays.cpp:
             * consecutiveErrors >= 3).  Single overcurrent events
             * enter DEGRADED to allow "drive home".                   */
            if (consecutive_errors < 255) consecutive_errors++;
            last_error_tick = HAL_GetTick();
            if (consecutive_errors >= CONSECUTIVE_ERROR_THRESHOLD) {
                Safety_SetState(SYS_STATE_SAFE);
            } else {
                Safety_SetState(SYS_STATE_DEGRADED);
                /* Phase 12: escalate degradation level with each
                 * consecutive overcurrent event.                      */
                if (consecutive_errors >= 2) {
                    Safety_SetDegradedLevel(DEGRADED_L3,
                                            DEGRADED_REASON_PERSISTENT);
                } else {
                    Safety_SetDegradedLevel(DEGRADED_L1,
                                            DEGRADED_REASON_OVERCURRENT);
                }
            }
            return;
        } else {
            ServiceMode_ClearFault(mod);
        }
    }
    /* No overcurrent — decay consecutive error counter after 1 s of
     * clean operation (traced to relays.cpp: lastErrorMs > 1000).     */
    if ((HAL_GetTick() - last_error_tick) > 1000 && consecutive_errors > 0) {
        consecutive_errors = 0;
    }
    /* Clear overcurrent error once current is back to normal,
     * allowing DEGRADED → ACTIVE recovery.                            */
    if (safety_error == SAFETY_ERROR_OVERCURRENT &&
        system_state == SYS_STATE_DEGRADED &&
        consecutive_errors == 0) {
        Safety_ClearError(SAFETY_ERROR_OVERCURRENT);
    }
}

/* ---- Overtemperature ---------------------------------------------- */

/* Temperature warning threshold — enter DEGRADED, not SAFE.
 * Traced to limp_mode.cpp: Thresholds::TEMP_WARNING = 80.0f            */
#define TEMP_WARNING_C    80.0f
/* Temperature critical threshold — enter SAFE (actuators off).
 * Traced to limp_mode.cpp: Thresholds::TEMP_CRITICAL = 90.0f           */
#define TEMP_CRITICAL_C   90.0f

void Safety_CheckTemperature(void)
{
    for (uint8_t i = 0; i < NUM_DS18B20; i++) {
        /* Skip disabled temperature sensors (service mode).
         * Traced to base firmware car_sensors.cpp:
         *   if (!cfg.tempSensorsEnabled) { return 0.0f; }
         * and temperature.cpp: sensorOk[] per-sensor tracking         */
        ModuleID_t mod = (ModuleID_t)(MODULE_TEMP_SENSOR_0 + i);
        if (!ServiceMode_IsEnabled(mod)) continue;

        float t = Temperature_Get(i);
        /* NaN/Inf hardening: invalid reading → treat as critical overtemp
         * (safe side — prefer actuator shutdown over silent pass-through). */
        if (isnan(t) || isinf(t) || t > TEMP_CRITICAL_C) {
            ServiceMode_SetFault(mod, MODULE_FAULT_ERROR);
            Safety_SetError(SAFETY_ERROR_OVERTEMP);
            Safety_SetState(SYS_STATE_SAFE);
            return;
        }
        if (t > TEMP_WARNING_C) {
            ServiceMode_SetFault(mod, MODULE_FAULT_WARNING);
            Safety_SetError(SAFETY_ERROR_OVERTEMP);
            Safety_SetState(SYS_STATE_DEGRADED);
            Safety_SetDegradedLevel(DEGRADED_L2,
                                    DEGRADED_REASON_THERMAL_WARN);
            return;
        }
        ServiceMode_ClearFault(mod);
    }
    /* All temperatures OK — clear overtemp error if it was set,
     * allowing DEGRADED → ACTIVE recovery via Safety_CheckCANTimeout.
     * Apply 5 °C hysteresis below TEMP_WARNING_C to prevent
     * oscillation when a motor temp hovers near the threshold.        */
    if (safety_error == SAFETY_ERROR_OVERTEMP &&
        system_state == SYS_STATE_DEGRADED) {
        bool all_below_hysteresis = true;
        for (uint8_t i = 0; i < NUM_DS18B20; i++) {
            ModuleID_t mod = (ModuleID_t)(MODULE_TEMP_SENSOR_0 + i);
            if (!ServiceMode_IsEnabled(mod)) continue;
            float ht = Temperature_Get(i);
            /* NaN/Inf blocks hysteresis recovery (safe side) */
            if (isnan(ht) || isinf(ht) || ht > (TEMP_WARNING_C - 5.0f)) {
                all_below_hysteresis = false;
                break;
            }
        }
        if (all_below_hysteresis) {
            Safety_ClearError(SAFETY_ERROR_OVERTEMP);
        }
    }
}

/* ---- CAN Heartbeat Timeout --------------------------------------- */

void Safety_CheckCANTimeout(void)
{
    /* SAFETY FIX: Global CAN watchdog — detect total bus silence.
     * If no CAN frames of any kind have arrived within
     * CAN_GLOBAL_SILENCE_MS the bus is completely dead (hardware
     * fault, failed transceiver, severed wiring).  This is more
     * severe than a heartbeat timeout (which only means the ESP32
     * stopped sending) and gets caught even if other periodic
     * messages are expected to mask it.                             */
    if (CAN_IsGlobalSilent() &&
        (system_state == SYS_STATE_ACTIVE ||
         system_state == SYS_STATE_DEGRADED)) {
        Safety_SetState(SYS_STATE_LIMP_HOME);
    }

    if ((HAL_GetTick() - last_can_rx_time) > CAN_TIMEOUT_MS) {
        ServiceMode_SetFault(MODULE_CAN_TIMEOUT, MODULE_FAULT_ERROR);
        Safety_SetError(SAFETY_ERROR_CAN_TIMEOUT);

        /* Reset LIMP_HOME recovery debounce — CAN is down, no recovery */
        limphome_recovery_pending = 0;

        /* CAN loss → LIMP_HOME (not SAFE).
         * Communication loss is NOT a hazard — the vehicle can still
         * operate at reduced capability with local pedal input.
         * SAFE is reserved for real hardware danger (overcurrent,
         * inverter fault, watchdog, electrical hazard).                 */
        if (system_state == SYS_STATE_ACTIVE ||
            system_state == SYS_STATE_DEGRADED) {
            Safety_SetState(SYS_STATE_LIMP_HOME);
        }

        /* STANDBY → LIMP_HOME if boot validation passed.
         * Vehicle must always be capable of moving at low speed
         * without CAN or ESP32.  Loss of communication must never
         * immobilize the vehicle.
         * Steering calibration is NOT required for LIMP_HOME —
         * uncalibrated steering means no assisted steering, but
         * the vehicle can still move at walking speed.  The fault
         * is still reported via SAFETY_ERROR_CENTERING / FAULT_CENTERING. */
        if (system_state == SYS_STATE_STANDBY &&
            BootValidation_IsPassed()) {
            Safety_SetState(SYS_STATE_LIMP_HOME);
        }
    } else {
        ServiceMode_ClearFault(MODULE_CAN_TIMEOUT);
        /* ESP32 alive – if we were in STANDBY, transition to ACTIVE
         * only when steering centering has completed successfully
         * AND the boot validation checklist has passed.                 */
        if (system_state == SYS_STATE_STANDBY &&
            safety_error == SAFETY_ERROR_NONE &&
            Steering_IsCalibrated() &&
            BootValidation_IsPassed()) {
            Safety_SetState(SYS_STATE_ACTIVE);
        }
        /* CAN restored from LIMP_HOME → attempt ACTIVE.
         * Heartbeat has appeared and system is healthy.
         * ACTIVE still requires steering calibration — if centering
         * never completed, the vehicle stays in LIMP_HOME.
         * Clear both CAN_TIMEOUT and CAN_BUSOFF: if the bus recovered
         * on its own (auto-recovery) without CAN_CheckBusOff() completing
         * its reinit sequence, the residual BUSOFF error would otherwise
         * persist and block future state transitions.
         *
         * SAFETY: Debounce the transition — require RECOVERY_HOLD_MS of
         * sustained heartbeat presence before re-entering ACTIVE.  This
         * prevents premature reactivation from a single heartbeat on a
         * flapping CAN bus (e.g. intermittent short, bad termination).  */
        if (system_state == SYS_STATE_LIMP_HOME &&
            Steering_IsCalibrated()) {
            if (!limphome_recovery_pending) {
                limphome_recovery_pending = 1;
                limphome_recovery_since   = HAL_GetTick();
            } else if ((HAL_GetTick() - limphome_recovery_since) >= RECOVERY_HOLD_MS) {
                /* SAFETY FIX: Before transitioning, verify that the CAN
                 * bus has remained continuously healthy for the entire
                 * debounce window.  Check that no non-CAN errors exist
                 * and that the heartbeat is still alive right now.
                 * If any condition fails, reset the window.             */
                if (safety_error != SAFETY_ERROR_NONE &&
                    safety_error != SAFETY_ERROR_CAN_TIMEOUT &&
                    safety_error != SAFETY_ERROR_CAN_BUSOFF) {
                    /* A non-CAN error appeared during the window.
                     * Cannot enter ACTIVE; stay in LIMP_HOME.           */
                    limphome_recovery_pending = 0;
                } else {
                    limphome_recovery_pending = 0;
                    Safety_ClearError(SAFETY_ERROR_CAN_TIMEOUT);
                    Safety_ClearError(SAFETY_ERROR_CAN_BUSOFF);
                    Safety_SetState(SYS_STATE_ACTIVE);
                }
            }
        } else if (system_state != SYS_STATE_LIMP_HOME) {
            limphome_recovery_pending = 0;
        }
        /* If in SAFE due to CAN timeout/busoff and heartbeat restored, try recovery.
         * NOTE: SAFE should no longer be triggered by CAN timeout alone,
         * but this path remains for backward-compatible recovery from
         * earlier firmware versions or manual SAFE entry.
         *
         * SAFETY: Use the same LIMP_HOME debounce mechanism — the bus
         * must be stable for RECOVERY_HOLD_MS before reactivation.
         * First transition to LIMP_HOME (limited power), then the
         * LIMP_HOME→ACTIVE debounce path handles full reactivation.    */
        if (system_state == SYS_STATE_SAFE &&
            (safety_error == SAFETY_ERROR_CAN_TIMEOUT ||
             safety_error == SAFETY_ERROR_CAN_BUSOFF)) {
            Safety_ClearError(SAFETY_ERROR_CAN_TIMEOUT);
            Safety_ClearError(SAFETY_ERROR_CAN_BUSOFF);
            Safety_SetState(SYS_STATE_LIMP_HOME);
        }
        /* DEGRADED recovery: if fault has been cleared while in DEGRADED,
         * attempt to return to ACTIVE after a debounce period.
         * Traced to limp_mode.cpp: STATE_HYSTERESIS_MS = 500.
         * The debounce prevents rapid state oscillation when sensor
         * values fluctuate near thresholds.                              */
        if (system_state == SYS_STATE_DEGRADED &&
            safety_error == SAFETY_ERROR_NONE) {
            if (!recovery_pending) {
                recovery_pending   = 1;
                recovery_clean_since = HAL_GetTick();
            } else if ((HAL_GetTick() - recovery_clean_since) >= RECOVERY_HOLD_MS) {
                recovery_pending = 0;
                Safety_SetState(SYS_STATE_ACTIVE);
            }
        } else {
            recovery_pending = 0;  /* Reset debounce if fault reappears */
        }
    }
}

/* Called by CAN RX handler to refresh watchdog */
void Safety_UpdateCANRxTime(void)
{
    last_can_rx_time = HAL_GetTick();
}

/* ---- Sensor plausibility checks ---------------------------------- */

void Safety_CheckSensors(void)
{
    /* Temperature plausibility: values must be within DS18B20 range.
     * Non-critical sensor fault → DEGRADED (not SAFE) to allow
     * "drive home".  Traced to base firmware system.cpp selfTest:
     * temperature sensors are OPTIONAL and use MODE_DEGRADED.           */
    uint8_t fault_count = 0;
    for (uint8_t i = 0; i < NUM_DS18B20; i++) {
        ModuleID_t mod = (ModuleID_t)(MODULE_TEMP_SENSOR_0 + i);
        if (!ServiceMode_IsEnabled(mod)) continue;
        float t = Temperature_Get(i);
        /* NaN/Inf hardening: invalid sensor reading → plausibility fault */
        if (isnan(t) || isinf(t) || t < SENSOR_TEMP_MIN_C || t > SENSOR_TEMP_MAX_C) {
            ServiceMode_SetFault(mod, MODULE_FAULT_ERROR);
            fault_count++;
        }
    }

    /* Current plausibility: significantly negative or extremely high = fault.
     * A small negative reading (> −1 A) is expected due to INA226 offset
     * and inductive motor flyback during deceleration — not a fault.
     * Traced to base firmware system.cpp selfTest: current sensors
     * are OPTIONAL and use MODE_DEGRADED.                               */
    for (uint8_t i = 0; i < NUM_INA226; i++) {
        ModuleID_t mod = (ModuleID_t)(MODULE_CURRENT_SENSOR_0 + i);
        if (!ServiceMode_IsEnabled(mod)) continue;
        float a = Current_GetAmps(i);
        float ceil = (i == INA226_CHANNEL_BATTERY)
                   ? SENSOR_CURRENT_MAX_BATT_A : SENSOR_CURRENT_MAX_A;
        /* NaN/Inf hardening: invalid sensor reading → plausibility fault */
        if (isnan(a) || isinf(a) || a < -1.0f || a > ceil) {
            ServiceMode_SetFault(mod, MODULE_FAULT_ERROR);
            fault_count++;
        }
    }

    /* Wheel speed plausibility: no single wheel wildly out of range.
     * Traced to base firmware system.cpp selfTest: wheel sensors are
     * OPTIONAL and use MODE_DEGRADED.                                   */
    float spd[4];
    spd[0] = Wheel_GetSpeed_FL();
    spd[1] = Wheel_GetSpeed_FR();
    spd[2] = Wheel_GetSpeed_RL();
    spd[3] = Wheel_GetSpeed_RR();
    for (uint8_t i = 0; i < 4; i++) {
        ModuleID_t mod = (ModuleID_t)(MODULE_WHEEL_SPEED_FL + i);
        if (!ServiceMode_IsEnabled(mod)) continue;
        /* NaN/Inf hardening: invalid speed reading → plausibility fault */
        if (isnan(spd[i]) || isinf(spd[i]) ||
            spd[i] < 0.0f || spd[i] > SENSOR_SPEED_MAX_KMH) {
            ServiceMode_SetFault(mod, MODULE_FAULT_ERROR);
            fault_count++;
        }
        /* Stale detection: flag sensor if no pulses within timeout.
         * A stale wheel while other wheels report speed indicates a
         * disconnected or failed sensor — triggers DEGRADED via the
         * fault_count accumulator below.                                */
        if (Wheel_IsStale(i) && spd[i] == 0.0f) {
            /* Only flag if at least one other wheel has nonzero speed.
             * All wheels stale at zero = vehicle stopped (normal).      */
            uint8_t any_moving = 0;
            for (uint8_t j = 0; j < 4; j++) {
                if (j != i && spd[j] > 1.0f) { any_moving = 1; break; }
            }
            if (any_moving) {
                ServiceMode_SetFault(mod, MODULE_FAULT_WARNING);
                fault_count++;
            }
        }
    }

    /* Pedal plausibility: dual-channel cross-validation.
     * ACTIVE state requires fully valid pedal plausibility — any
     * plausibility failure exits ACTIVE immediately.
     *
     * Two distinct failure modes:
     *   1. Contradictory: dual ADC samples disagree.
     *      → No torque allowed (ADC fault / noise risk).
     *      → Demand forced to zero in ALL states.
     *   2. Implausible: range or rate-of-change check failed.
     *      → Cross-validation impossible, but ADC may be valid.
     *      → LIMP_HOME allows limited throttle via primary ADC
     *        with 20 % torque cap and 5 km/h speed limit.
     *
     * In both cases: ACTIVE → LIMP_HOME (fail-operational).
     * Recovery to ACTIVE requires full pedal plausibility restored.    */
    if (!Pedal_IsPlausible()) {
        /* Safety invariant: contradictory samples → zero torque always.
         * When not contradictory (implausible) and already in LIMP_HOME,
         * main.c provides limited throttle from primary ADC + clamp.     */
        if (Pedal_IsContradictory()) {
            Traction_SetDemand(0.0f);
        } else if (system_state != SYS_STATE_LIMP_HOME) {
            /* Safe transition: zero torque until LIMP_HOME is reached */
            Traction_SetDemand(0.0f);
        }

        Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);

        /* Pedal fault: ACTIVE/DEGRADED → LIMP_HOME (not SAFE).
         * STANDBY → LIMP_HOME if boot validation passed.
         * Already in LIMP_HOME → stay (no state change needed).         */
        if (system_state == SYS_STATE_ACTIVE ||
            system_state == SYS_STATE_DEGRADED) {
            Safety_SetState(SYS_STATE_LIMP_HOME);
        } else if (system_state == SYS_STATE_STANDBY &&
                   BootValidation_IsPassed()) {
            Safety_SetState(SYS_STATE_LIMP_HOME);
        }
        return;
    }

    /* If any enabled sensor has a plausibility fault, enter DEGRADED */
    if (fault_count > 0) {
        Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
        Safety_SetState(SYS_STATE_DEGRADED);
        /* Phase 12: multiple sensor faults → higher degradation level */
        if (fault_count >= 3) {
            Safety_SetDegradedLevel(DEGRADED_L3,
                                    DEGRADED_REASON_PERSISTENT);
        } else {
            Safety_SetDegradedLevel(DEGRADED_L1,
                                    DEGRADED_REASON_SENSOR_FAULT);
        }
        return;
    }

    /* All sensor checks passed (including pedal plausibility) —
     * clear sensor fault to allow recovery.
     * DEGRADED: CAN timeout handler can recover to ACTIVE.
     * LIMP_HOME: pedal plausibility restored → allow ACTIVE recovery
     * via Safety_CheckCANTimeout() when CAN is alive.                   */
    if (safety_error == SAFETY_ERROR_SENSOR_FAULT &&
        (system_state == SYS_STATE_DEGRADED ||
         system_state == SYS_STATE_LIMP_HOME)) {
        Safety_ClearError(SAFETY_ERROR_SENSOR_FAULT);
    }
}

/* ---- Steering encoder health ----------------------------------------- */

/**
 * @brief  Check the steering encoder for faults.
 *
 * Delegates the actual detection to Encoder_CheckHealth() in
 * motor_control.c (which monitors range, jumps and frozen values).
 * If a fault is detected, raise SAFETY_ERROR_SENSOR_FAULT and
 * transition to DEGRADED state.  Steering is neutralised (no PID
 * without encoder feedback), but traction remains operational at
 * reduced power so the vehicle can "drive home".
 *
 * Traced to base firmware limp_mode.cpp: steering not centered →
 * LimpState::LIMP (40 % power, 50 % speed).
 */
void Safety_CheckEncoder(void)
{
    /* Skip if steering encoder module is disabled (service mode).
     * The user has acknowledged the fault and wants to drive without
     * encoder-based steering assist. */
    if (!ServiceMode_IsEnabled(MODULE_STEER_ENCODER)) return;

    Encoder_CheckHealth();

    if (Encoder_HasFault()) {
        ServiceMode_SetFault(MODULE_STEER_ENCODER, MODULE_FAULT_ERROR);
        /* Only set the error once to avoid overwriting a different
         * existing fault code.                                      */
        if (safety_error == SAFETY_ERROR_NONE) {
            Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
        }
        /* Neutralise steering (no PID without encoder) but keep
         * traction alive in DEGRADED mode for "drive home".         */
        Steering_Neutralize();
        Safety_SetState(SYS_STATE_DEGRADED);
        Safety_SetDegradedLevel(DEGRADED_L1,
                                DEGRADED_REASON_ENCODER_FAULT);
    } else {
        ServiceMode_ClearFault(MODULE_STEER_ENCODER);
    }
}

/* ---- Battery undervoltage protection ----------------------------- */

/**
 * @brief  Check battery bus voltage and enforce undervoltage protection.
 *
 * Reads the INA226 on channel INA226_CHANNEL_BATTERY (24 V bus).
 * Voltage_GetBus() returns the last value sampled by Current_ReadAll()
 * in the 50 ms tier.  This function runs in the 100 ms tier.
 *
 * Warning (< 20.0 V):
 *   - Transition to SYS_STATE_DEGRADED
 *   - Power limited via DEGRADED_POWER_LIMIT_PCT (40 %)
 *   - Dynamic braking, park hold, and traction demand are reduced
 *     through the existing degraded-mode power limiter.
 *   - Recovery: voltage must exceed 20.5 V (0.5 V hysteresis).
 *
 * Critical (< 18.0 V or sensor failure):
 *   - Transition to SYS_STATE_SAFE
 *   - All traction outputs disabled (Safety_FailSafe)
 *   - Dynamic braking and park hold disabled (SAFE state inhibits)
 *   - Steering centering preserved if encoder is healthy
 *   - NO auto-recovery: operator must recharge and reset
 *
 * Sensor failure (0.0 V reading indicates I2C / multiplexer fault):
 *   - Treated as CRITICAL (fail-safe default)
 *   - The battery INA226 is placed BEFORE the main relay (directly at the
 *     battery terminal), so relay-off states do NOT cause 0 V readings.
 *     A 0 V reading truly means the sensor/I2C bus has failed.
 *
 * Works alongside overcurrent / overtemperature / CAN timeout:
 *   - Does not interfere with existing fault escalation
 *   - Only sets error if no higher-priority fault is active
 */
void Safety_CheckBatteryVoltage(void)
{
    float voltage = Voltage_GetBus(INA226_CHANNEL_BATTERY);

    /* Sensor failure: 0.0 V means TCA9548A channel select failed or
     * INA226 returned zero — treat as critical (fail-safe).
     * NaN/Inf hardening: invalid ADC reading → critical sensor failure. */
    if (isnan(voltage) || isinf(voltage) || voltage <= 0.0f) {
        Safety_SetError(SAFETY_ERROR_BATTERY_UV_CRITICAL);
        Safety_SetState(SYS_STATE_SAFE);
        return;
    }

    /* Critical undervoltage — SAFE state, no auto-recovery */
    if (voltage < BATTERY_UV_CRITICAL_V) {
        Safety_SetError(SAFETY_ERROR_BATTERY_UV_CRITICAL);
        Safety_SetState(SYS_STATE_SAFE);
        return;
    }

    /* Warning undervoltage — DEGRADED state with power limiting */
    if (voltage < BATTERY_UV_WARNING_V) {
        Safety_SetError(SAFETY_ERROR_BATTERY_UV_WARNING);
        Safety_SetState(SYS_STATE_DEGRADED);
        Safety_SetDegradedLevel(DEGRADED_L2,
                                DEGRADED_REASON_BATTERY_UV);
        return;
    }

    /* Voltage OK — attempt recovery from DEGRADED if hysteresis met.
     * Recovery from DEGRADED requires voltage > WARNING + HYSTERESIS
     * (20.5 V) to prevent oscillation under load transients.
     *
     * Recovery from SAFE is intentionally NOT attempted here:
     * a critically depleted battery cannot reliably power actuators.
     * The operator must recharge and reset the system.                 */
    if (system_state == SYS_STATE_DEGRADED &&
        safety_error == SAFETY_ERROR_BATTERY_UV_WARNING &&
        voltage > (BATTERY_UV_WARNING_V + BATTERY_UV_HYST_V)) {
        Safety_ClearError(SAFETY_ERROR_BATTERY_UV_WARNING);
    }
}

/**
 * @brief  Check battery bus overvoltage and enforce protection.
 *
 * Overvoltage can damage 24 V electronics (capacitors, ICs, sensors).
 * Sources: charger malfunction, regen braking, regulator failure.
 *
 * Warning (> 30.0 V):
 *   - Transition to SYS_STATE_DEGRADED (reduce load)
 *
 * Critical (> 35.0 V):
 *   - Transition to SYS_STATE_SAFE (disable actuators immediately)
 *   - NO auto-recovery: operator must investigate overvoltage source
 *
 * Recovery from DEGRADED: voltage < WARNING − HYSTERESIS (29.5 V).
 */
void Safety_CheckBatteryOvervoltage(void)
{
    float voltage = Voltage_GetBus(INA226_CHANNEL_BATTERY);

    /* Ignore 0 V readings (sensor failure handled by undervoltage check).
     * NaN/Inf hardening: invalid ADC → skip OV check (UV handles it). */
    if (isnan(voltage) || isinf(voltage) || voltage <= 0.0f) return;

    /* Critical overvoltage — SAFE state, no auto-recovery */
    if (voltage > BATTERY_OV_CRITICAL_V) {
        Safety_SetError(SAFETY_ERROR_BATTERY_OV_CRITICAL);
        Safety_SetState(SYS_STATE_SAFE);
        return;
    }

    /* Warning overvoltage — DEGRADED state with power limiting */
    if (voltage > BATTERY_OV_WARNING_V) {
        Safety_SetError(SAFETY_ERROR_BATTERY_OV_WARNING);
        Safety_SetState(SYS_STATE_DEGRADED);
        Safety_SetDegradedLevel(DEGRADED_L2,
                                DEGRADED_REASON_BATTERY_OV);
        return;
    }

    /* Voltage OK — attempt recovery from DEGRADED if hysteresis met */
    if (system_state == SYS_STATE_DEGRADED &&
        safety_error == SAFETY_ERROR_BATTERY_OV_WARNING &&
        voltage < (BATTERY_OV_WARNING_V - BATTERY_OV_HYST_V)) {
        Safety_ClearError(SAFETY_ERROR_BATTERY_OV_WARNING);
    }
}

/* ---- Relay health check (post-ACTIVE runtime validation) ----------
 *
 * Detects relay-not-closing using existing signals (throttle + INA226
 * motor current + wheel speed + battery voltage).
 * Active ONLY in ACTIVE / DEGRADED / LIMP_HOME.
 *
 * Algorithm:
 *   1. Activation: throttle > 20% (primary) OR throttle > 10% with
 *      vehicle speed > 2 km/h (secondary — catches relay failure
 *      while vehicle is in motion at lower demand).
 *   2. After 300 ms settle, compute TOTAL absolute motor current
 *      (INA226 ch 0-3 sum).
 *   3. Voltage guard: skip check if battery < 18 V (depleted battery
 *      cannot deliver current regardless of relay state).
 *   4. If total current < 3.0 A for 3 consecutive calls (30 ms),
 *      trigger SAFETY_ERROR_RELAY_OPEN → DEGRADED L3.
 *   5. Recovery: fault clears ONLY after 1500 ms of sustained healthy
 *      current — prevents DEGRADED ↔ ACTIVE oscillation.
 *
 * O(1) per call.  No division.  No blocking.  No CAN changes.
 * Called from the 10 ms safety loop (same tier as Safety_CheckCurrent). */

void Safety_CheckRelayHealth(void)
{
    /* Only active when motion is allowed (ACTIVE / DEGRADED / LIMP_HOME) */
    if (!Safety_IsMotionAllowed()) {
        relay_chk_active        = 0;
        relay_chk_debounce      = 0;
        relay_chk_recovery_tick = 0;
        return;
    }

    /* Skip if relay power-up sequence has not completed */
    if (relay_seq_state != RELAY_SEQ_COMPLETE) {
        relay_chk_active        = 0;
        relay_chk_debounce      = 0;
        relay_chk_recovery_tick = 0;
        return;
    }

    /* Read current throttle demand (post-validation, post-clamp).
     * Pedal_GetPercent returns the EMA-filtered 0-100% pedal value.
     * In LIMP_HOME the clamped demand is lower, but the raw pedal
     * still reflects driver intent — use raw percent for the check
     * so that a 25% raw pedal (→5% LIMP demand) still triggers
     * relay validation when the driver clearly wants to move.       */
    float throttle = Pedal_GetPercent();

    /* Speed-aware activation: compute average wheel speed (km/h).
     * Pattern reused from Safety_ValidateModeChange / ABS / TCS.
     * Multiply by 0.25 instead of dividing by 4 (no division).     */
    float avg_speed = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                       Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) * 0.25f;

    /* Activation condition:
     *   Primary:   throttle > 20 % (standing start / normal drive)
     *   Secondary: throttle > 10 % AND speed > 2 km/h (relay failure
     *              while vehicle is moving at lower demand).
     * At idle (throttle ≤ 10 %, speed ≤ 2 km/h): no check → no FP.  */
    uint8_t demand_present =
        (throttle > RELAY_CHK_THROTTLE_MIN_PCT) ||
        (throttle > RELAY_CHK_THROTTLE_SPEED_PCT &&
         avg_speed > RELAY_CHK_SPEED_MIN_KMH);

    if (demand_present) {
        uint32_t now = HAL_GetTick();

        if (!relay_chk_active) {
            /* Start timing window */
            relay_chk_active     = 1;
            relay_chk_start_tick = now;
            relay_chk_debounce   = 0;
            return;
        }

        /* Wait for settle delay before evaluating current */
        if ((now - relay_chk_start_tick) < RELAY_CHK_DELAY_MS) {
            return;
        }

        /* Voltage correlation: skip relay check if battery voltage is
         * below the critical threshold.  A depleted battery cannot
         * deliver motor current regardless of relay state — triggering
         * a relay fault here would be a false positive.  The existing
         * battery undervoltage checks handle that condition.          */
        float batt_v = Voltage_GetBus(INA226_CHANNEL_BATTERY);
        if (isnan(batt_v) || isinf(batt_v) ||
            batt_v < RELAY_CHK_VOLTAGE_MIN_V) {
            relay_chk_debounce = 0;
            return;
        }

        /* Compute TOTAL absolute motor current (ch 0-3).
         * Uses sum of all enabled motor channels instead of per-wheel
         * average — more robust against single-channel noise.
         * Skip disabled sensors (service mode).  Manual abs value
         * (no fabsf — avoid pulling in libm for a single call).      */
        float sum = 0.0f;
        uint8_t count = 0;
        for (uint8_t i = 0; i < NUM_WHEELS; i++) {
            ModuleID_t mod = (ModuleID_t)(MODULE_CURRENT_SENSOR_0 + i);
            if (!ServiceMode_IsEnabled(mod)) continue;
            float amps = Current_GetAmps(i);
            if (isnan(amps) || isinf(amps)) continue;
            /* Manual absolute value */
            if (amps < 0.0f) amps = -amps;
            sum += amps;
            count++;
        }

        /* If no enabled sensors, cannot determine relay state — bail */
        if (count == 0) {
            relay_chk_debounce = 0;
            return;
        }

        /* Compare total sum against fixed threshold.
         * No division needed — RELAY_CHK_CURRENT_MIN_TOTAL_A is the
         * minimum expected TOTAL current across all enabled channels
         * when throttle is above the activation threshold.
         *
         * Rationale: 4 × RS775 at 24 V under 20% throttle draw
         * ≈ 1.5–2.5 A each → total ≈ 6–10 A.  Threshold of 3.0 A
         * leaves ~50% margin while remaining well above INA226
         * noise floor (~0.1 A per channel).                          */
        if (sum < RELAY_CHK_CURRENT_MIN_TOTAL_A) {
            /* Fault condition: demand present but no motor current.
             * Reset recovery timer — fault condition persists.        */
            relay_chk_recovery_tick = 0;

            if (relay_chk_debounce < UINT8_MAX) relay_chk_debounce++;

            if (relay_chk_debounce >= RELAY_CHK_DEBOUNCE_CYCLES) {
                /* Confirmed relay-open fault */
                ServiceMode_SetFault(MODULE_RELAY_MAIN, MODULE_FAULT_ERROR);
                Safety_SetError(SAFETY_ERROR_RELAY_OPEN);
                Safety_SetState(SYS_STATE_DEGRADED);
                Safety_SetDegradedLevel(DEGRADED_L3,
                                        DEGRADED_REASON_SENSOR_FAULT);
            }
        } else {
            /* Current present — relay is healthy, reset debounce */
            relay_chk_debounce = 0;

            /* Recovery hysteresis: only clear the relay-open fault
             * after RELAY_CHK_RECOVERY_MS of sustained healthy current.
             * Prevents DEGRADED ↔ ACTIVE oscillation from noisy
             * current readings or intermittent relay contact.
             *
             * The recovery timer starts when current first exceeds the
             * threshold and resets if current drops below it again.
             * Fault is cleared only when the timer expires.           */
            if (safety_error == SAFETY_ERROR_RELAY_OPEN) {
                if (relay_chk_recovery_tick == 0) {
                    /* Start recovery window */
                    relay_chk_recovery_tick = now;
                } else if ((now - relay_chk_recovery_tick) >=
                           RELAY_CHK_RECOVERY_MS) {
                    /* Sustained healthy current confirmed — clear fault */
                    ServiceMode_ClearFault(MODULE_RELAY_MAIN);
                    Safety_ClearError(SAFETY_ERROR_RELAY_OPEN);
                    relay_chk_recovery_tick = 0;
                }
                /* else: still waiting for hysteresis window to expire */
            }
        }
    } else {
        /* Throttle and speed both below thresholds — reset check window.
         * Do NOT clear relay_chk_recovery_tick here: if a relay fault
         * was detected and throttle drops briefly, re-demanding throttle
         * must still sustain healthy current for the full recovery
         * period.  However, since we cannot measure current without
         * demand, reset the recovery timer — it will restart when
         * demand returns and current is observed.                     */
        relay_chk_active        = 0;
        relay_chk_debounce      = 0;
        relay_chk_recovery_tick = 0;
    }
}

/* ---- Emergency actions ------------------------------------------- */

void Safety_EmergencyStop(void)
{
    emergency_stopped = 1;
    Traction_EmergencyStop();
    /* Transition to ERROR which calls Safety_PowerDown → Relay_PowerDown.
     * Safety_PowerDown is safe to call after Traction_EmergencyStop
     * (actuators are already inhibited; relays are de-energised).       */
    system_state = SYS_STATE_ERROR;
    Relay_PowerDown();
}

void Safety_FailSafe(void)
{
    Traction_EmergencyStop();

    /* When the encoder is healthy, command the steering motor toward
     * centre (0°) so the vehicle tracks straight under inertia.
     * When the encoder is faulted, we have no reliable position
     * feedback — driving the motor blind could make things worse.
     * Instead, neutralise the motor (cut PWM + disable H-bridge)
     * and let mechanical return springs / friction slow the rack.    */
    if (Encoder_HasFault()) {
        Steering_Neutralize();
    } else {
        Steering_SetAngle(0.0f);
    }
}

void Safety_PowerDown(void)
{
    Traction_EmergencyStop();
    Relay_PowerDown();
}

/* ---- Error tracking ---------------------------------------------- */

void Safety_SetError(Safety_Error_t error)
{
    /* Only log transitions to a new error (avoid flooding the log
     * when the same error is re-asserted every check cycle).      */
    if (error != SAFETY_ERROR_NONE && error != safety_error) {
        ErrorLog_Record((uint8_t)error, 0 /* GLOBAL */,
                        (uint8_t)system_state,
                        Safety_GetFaultFlags());
    }
    safety_error = error;
}
void Safety_ClearError(Safety_Error_t error) { if (safety_error == error) safety_error = SAFETY_ERROR_NONE; }
Safety_Error_t Safety_GetError(void)         { return safety_error; }
bool Safety_IsError(void)                    { return (safety_error != SAFETY_ERROR_NONE); }

/* ================================================================== */
/*  Obstacle Safety — STM32 Primary Safety Controller                  */
/*  CAN frames from ESP32 are advisory only, never mandatory.          */
/* ================================================================== */

/**
 * @brief  Helper: compute average vehicle speed from wheel sensors.
 * @return Vehicle speed in km/h (average of all four wheels).
 */
static float Obstacle_GetVehicleSpeed(void)
{
    float avg = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                 Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) * 0.25f;
    avg = sanitize_float(avg, 0.0f);
    if (avg < 0.0f) avg = 0.0f;
    return avg;
}

/**
 * @brief  Helper: compute speed-dependent stopping distance.
 *
 * d_stop = v² / (2·a) + margin
 * Returns the dynamic emergency threshold in mm.  The result is
 * at least OBSTACLE_EMERGENCY_MM so the fixed floor always applies.
 *
 * @param speed_kmh  Vehicle speed in km/h.
 * @return Dynamic emergency threshold in mm.
 */
static uint16_t Obstacle_StoppingDistance(float speed_kmh)
{
    float v_ms = speed_kmh / 3.6f;                         /* km/h → m/s   */
    float v_mms = v_ms * 1000.0f;                          /* m/s → mm/s   */
    float d_mm = (v_mms * v_mms) / (2.0f * OBSTACLE_STOP_DECEL_MMS2)
                 + (float)OBSTACLE_STOP_MARGIN_MM;
    if (d_mm < (float)OBSTACLE_EMERGENCY_MM) d_mm = (float)OBSTACLE_EMERGENCY_MM;
    if (d_mm > 4000.0f) d_mm = 4000.0f;                   /* clamp to 4 m */
    return (uint16_t)d_mm;
}

/**
 * @brief  Process a CAN obstacle distance message (0x208) from ESP32.
 *
 * Called from CAN_ProcessMessages().  CAN data is advisory — the local
 * state machine decides whether to trust it.  Physical plausibility and
 * stuck-sensor detection are applied before the distance is accepted.
 *
 * Payload (DLC ≥ 5):
 *   Byte 0-1: minimum distance (mm, uint16 LE)
 *   Byte 2:   zone level (0–5)
 *   Byte 3:   sensor health (0/1)
 *   Byte 4:   rolling counter
 */
void Obstacle_ProcessCAN(const uint8_t *data, uint8_t len)
{
    if (len < 5) return;

    /* Skip if disabled via service mode */
    if (!ServiceMode_IsEnabled(MODULE_OBSTACLE_DETECT)) return;

    uint16_t dist    = (uint16_t)(data[0] | (data[1] << 8));
    uint8_t  zone    = data[2];
    uint8_t  health  = data[3];
    uint8_t  counter = data[4];

    /* Reject sentinel value 0xFFFF — means "no data" from the sensor.
     * Treat as implausible and do not update the validated distance.   */
    if (dist == 0xFFFF) {
        obstacle_plausible = 0;
        obstacle_last_rx_tick = HAL_GetTick();
        return;
    }

    /* ---- Stale-data detection (rolling counter) ---- */
    if (obstacle_data_valid && counter == obstacle_last_counter) {
        if (obstacle_stale_count < 255) obstacle_stale_count++;
    } else {
        obstacle_stale_count = 0;
    }
    obstacle_last_counter = counter;

    /* Zone plausibility: clamp to valid range (0–4, matching ESP32 distanceToZone).
     * Invalid zone → emergency (4): fail-safe — assume closest obstacle
     * rather than clear, so a corrupted zone byte never opens the way.   */
    if (zone > 4) zone = 4;

    uint32_t now = HAL_GetTick();

    /* ---- Physical plausibility validation ---- */
    if (obstacle_data_valid && obstacle_prev_dist_tick > 0 &&
        obstacle_prev_distance != 0xFFFF) {
        uint32_t dt_ms = now - obstacle_prev_dist_tick;
        if (dt_ms > 0 && dt_ms < 5000) {
            /* Max plausible change = (max_approach_rate) × dt */
            float max_change_mm = ((float)OBSTACLE_MAX_APPROACH_MMS *
                                   (float)dt_ms) / 1000.0f;
            int32_t actual_change = (int32_t)obstacle_prev_distance -
                                    (int32_t)dist;
            /* Only check approach (distance decreasing = positive change).
             * Rapid distance increase (object moved away) is benign.      */
            if (actual_change > (int32_t)max_change_mm &&
                max_change_mm > 0.0f) {
                /* Implausible jump — reject this reading */
                obstacle_plausible = 0;
                ServiceMode_SetFault(MODULE_OBSTACLE_DETECT,
                                     MODULE_FAULT_WARNING);
                /* Keep previous validated distance; don't update */
                obstacle_last_rx_tick = now;
                return;
            }
        }
    }
    obstacle_plausible = 1;

    /* ---- Stuck-sensor detection ---- */
    if (obstacle_data_valid && obstacle_prev_distance != 0xFFFF) {
        int32_t delta = (int32_t)dist - (int32_t)obstacle_prev_distance;
        if (delta < 0) delta = -delta;
        float speed = Obstacle_GetVehicleSpeed();

        if (speed > OBSTACLE_STUCK_MIN_SPEED_KMH &&
            delta <= OBSTACLE_STUCK_THRESHOLD_MM) {
            /* Distance hasn't changed while vehicle is moving */
            if (obstacle_stuck_since == 0) {
                obstacle_stuck_since = now;
            } else if ((now - obstacle_stuck_since) >=
                       OBSTACLE_STUCK_DURATION_MS) {
                /* Sensor declared stuck */
                obstacle_plausible = 0;
                ServiceMode_SetFault(MODULE_OBSTACLE_DETECT,
                                     MODULE_FAULT_WARNING);
            }
        } else {
            obstacle_stuck_since = 0;
        }
    }

    /* Store raw CAN values */
    obstacle_prev_distance  = obstacle_distance_mm;
    obstacle_prev_dist_tick = now;
    obstacle_distance_mm    = dist;
    obstacle_zone           = zone;
    obstacle_sensor_healthy = health;
    obstacle_last_rx_tick   = now;
    obstacle_data_valid     = 1;

    /* Update validated distance only if plausible */
    if (obstacle_plausible && obstacle_stale_count < 3 &&
        obstacle_sensor_healthy) {
        obstacle_validated_mm = dist;
    }
}

/**
 * @brief  Process CAN 0x209 (Obstacle Safety State) from ESP32.
 *
 * Informational only — the STM32 computes its own obstacle_scale from
 * the raw distance in 0x208.  This frame provides the ESP32's view of
 * the obstacle situation for cross-validation and diagnostic logging.
 *
 * If the ESP32 reports its sensor as stuck (byte 2 = 1) and the STM32's
 * own plausibility checks have not flagged an issue, a service-mode
 * warning is raised to indicate a potential sensor disagreement.
 *
 * Payload (DLC ≥ 3):
 *   Byte 0: zone (0–4)
 *   Byte 1: sensor_status (0=WAITING, 1=INVALID, 2=VALID)
 *   Byte 2: stuck flag (0=OK, 1=stuck)
 *   Byte 3: reserved
 */
void Obstacle_ProcessSafetyCAN(const uint8_t *data, uint8_t len)
{
    if (len < 3) return;

    /* Skip if disabled via service mode */
    if (!ServiceMode_IsEnabled(MODULE_OBSTACLE_DETECT)) return;

    esp32_obs_zone          = data[0];
    esp32_obs_sensor_status = data[1];
    esp32_obs_stuck         = data[2];
    esp32_obs_last_rx_tick  = HAL_GetTick();

    /* Clamp zone to valid range (0–4).
     * Invalid zone → emergency (4): fail-safe assumption.              */
    if (esp32_obs_zone > 4) esp32_obs_zone = 4;

    /* Cross-validation: if ESP32 reports stuck but STM32 sees plausible
     * data, flag a diagnostic warning — sensors may disagree.           */
    if (esp32_obs_stuck && obstacle_plausible && obstacle_data_valid) {
        ServiceMode_SetFault(MODULE_OBSTACLE_DETECT, MODULE_FAULT_WARNING);
    }
}

/**
 * @brief  Periodic obstacle safety update — called every 10 ms.
 *
 * Implements a local state machine that is independent of CAN:
 *
 *   NO_SENSOR → (first valid CAN frame) → NORMAL
 *   NORMAL    → (obstacle in range confirmed) → CONFIRMING
 *   CONFIRMING→ (sustained for CONFIRM_MS) → ACTIVE
 *   CONFIRMING→ (obstacle disappeared) → NORMAL
 *   ACTIVE    → (obstacle receded) → CLEARING
 *   CLEARING  → (sustained for CLEAR_MS) → NORMAL
 *   CLEARING  → (obstacle returned) → ACTIVE
 *   any       → (sensor fault) → SENSOR_FAULT
 *   SENSOR_FAULT → (valid data resumes) → NORMAL
 *   any       → (CAN timeout, no obstacle) → NO_SENSOR (scale = 1.0)
 *   ACTIVE/CONFIRMING → (CAN timeout) → SENSOR_FAULT (hold scale)
 *
 * Speed-dependent stopping distance adjusts thresholds dynamically.
 * CAN loss with no active obstacle → scale 1.0 (LIMP_HOME cap provides safety).
 * CAN loss with active obstacle → hold last scale (obstacle < stopping distance safe).
 * No motion immobilization — only controlled slowdown.
 * Reverse escape preserved when forward is blocked.
 */
void Obstacle_Update(void)
{
    /* Skip if disabled via service mode */
    if (!ServiceMode_IsEnabled(MODULE_OBSTACLE_DETECT)) {
        safety_status.obstacle_scale = 1.0f;
        obstacle_forward_blocked = 0;
        obstacle_state = OBS_STATE_NO_SENSOR;
        return;
    }

    uint32_t now = HAL_GetTick();

    /* ---- CAN timeout: advisory data lost ----
     * CAN frames are advisory only.  When lost, LIMP_HOME speed limits
     * provide a baseline safety net.
     *
     * However, if an obstacle was actively detected (ACTIVE or CONFIRMING
     * state) when CAN died, we must NOT instantly drop protection to 1.0.
     * Scenarios that rely on this:
     *   - Obstacle closer than stopping distance when CAN fails
     *   - Vehicle rolling downhill (speed cap limits demand, not gravity)
     *   - Pedal pressed continuously at 20% torque limit
     *   - Single-wheel traction understating average speed
     *
     * Policy: retain the last known obstacle_scale (or a conservative
     * OBSTACLE_FAULT_SCALE) when an obstacle was being tracked.
     * If no obstacle was active, allow scale = 1.0.                    */
    if (obstacle_data_valid &&
        (now - obstacle_last_rx_tick) > OBSTACLE_CAN_TIMEOUT_MS) {

        if (obstacle_state == OBS_STATE_ACTIVE ||
            obstacle_state == OBS_STATE_CONFIRMING) {
            /* Obstacle was being tracked — hold last scale or fall back
             * to conservative limit.  Never weaker than FAULT_SCALE.   */
            if (safety_status.obstacle_scale > OBSTACLE_FAULT_SCALE) {
                safety_status.obstacle_scale = OBSTACLE_FAULT_SCALE;
            }
            /* Keep forward_blocked if it was already set */
            obstacle_state = OBS_STATE_SENSOR_FAULT;
        } else {
            /* No active obstacle threat when CAN died — allow motion.
             * LIMP_HOME speed cap provides the safety net.             */
            safety_status.obstacle_scale = 1.0f;
            obstacle_forward_blocked = 0;
            obstacle_state = OBS_STATE_NO_SENSOR;
        }

        ServiceMode_SetFault(MODULE_OBSTACLE_DETECT, MODULE_FAULT_WARNING);
        return;
    }

    /* ---- No CAN data ever received ---- */
    if (!obstacle_data_valid) {
        safety_status.obstacle_scale = 1.0f;
        obstacle_forward_blocked = 0;
        obstacle_state = OBS_STATE_NO_SENSOR;
        return;
    }

    /* ---- Sensor fault detection ---- */
    uint8_t sensor_fault = 0;
    if (obstacle_stale_count >= 3)     sensor_fault = 1; /* Frozen counter */
    if (!obstacle_sensor_healthy)      sensor_fault = 1; /* ESP32 reports  */
    if (!obstacle_plausible)           sensor_fault = 1; /* Implausible    */

    if (sensor_fault) {
        /* Conservative fallback: vehicle remains mobile at reduced power.
         * No immobilization.  LIMP_HOME speed cap is additional net.    */
        safety_status.obstacle_scale = OBSTACLE_FAULT_SCALE;
        obstacle_forward_blocked = 0;
        obstacle_state = OBS_STATE_SENSOR_FAULT;
        ServiceMode_SetFault(MODULE_OBSTACLE_DETECT, MODULE_FAULT_WARNING);
        return;
    }

    /* ---- Compute speed-dependent thresholds ---- */
    float speed_kmh = Obstacle_GetVehicleSpeed();
    uint16_t dyn_emergency  = Obstacle_StoppingDistance(speed_kmh);
    /* Critical, warning, caution, alert thresholds scale proportionally
     * but keep a minimum floor from the static defines.                */
    uint16_t dyn_critical = dyn_emergency + (OBSTACLE_CRITICAL_MM -
                                              OBSTACLE_EMERGENCY_MM);
    uint16_t dyn_warning  = dyn_critical  + (OBSTACLE_WARNING_MM -
                                              OBSTACLE_CRITICAL_MM);
    uint16_t dyn_caution  = dyn_warning   + (OBSTACLE_CAUTION_MM -
                                              OBSTACLE_WARNING_MM);
    uint16_t dyn_alert    = dyn_caution   + (OBSTACLE_ALERT_MM -
                                              OBSTACLE_CAUTION_MM);
    if (dyn_critical < OBSTACLE_CRITICAL_MM) dyn_critical = OBSTACLE_CRITICAL_MM;
    if (dyn_warning  < OBSTACLE_WARNING_MM)  dyn_warning  = OBSTACLE_WARNING_MM;
    if (dyn_caution  < OBSTACLE_CAUTION_MM)  dyn_caution  = OBSTACLE_CAUTION_MM;
    if (dyn_alert    < OBSTACLE_ALERT_MM)    dyn_alert    = OBSTACLE_ALERT_MM;

    /* Use the validated (plausibility-checked) distance */
    uint16_t dist = obstacle_validated_mm;

    /* ---- Determine raw target scale from distance (5 zones) ---- */
    float target_scale;
    uint8_t target_blocked = 0;

    if (dist < dyn_emergency) {
        target_scale   = 0.0f;
        target_blocked = 1;
    } else if (dist < dyn_critical) {
        target_scale = 0.3f;
    } else if (dist < dyn_warning) {
        target_scale = 0.7f;
    } else if (dist < dyn_caution) {
        target_scale = 0.85f;
    } else if (dist < dyn_alert) {
        target_scale = 0.95f;
    } else {
        target_scale = 1.0f;
    }

    /* ---- Child reaction detection ----
     * Sample pedal at regular intervals.  If pedal drops by more than
     * CHILD_REACTION_THRESHOLD within CHILD_REACTION_WINDOW_MS, the
     * child is reacting to a perceived obstacle → tighten warning and
     * caution zone factors for CHILD_REACTION_BOOST_MS.                 */
    {
        float pedal_now = Pedal_GetPercent();

        if ((now - child_pedal_sample_tick) >= CHILD_REACTION_WINDOW_MS) {
            float drop = child_prev_pedal_pct - pedal_now;
            if (drop >= CHILD_REACTION_THRESHOLD &&
                child_prev_pedal_pct > CHILD_REACTION_MIN_PEDAL) {
                child_reaction_active = 1;
                child_reaction_start  = now;
            }
            child_prev_pedal_pct    = pedal_now;
            child_pedal_sample_tick = now;
        }

        /* Expire reaction boost after duration */
        if (child_reaction_active &&
            (now - child_reaction_start) >= CHILD_REACTION_BOOST_MS) {
            child_reaction_active = 0;
        }

        /* Apply tighter factors to warning/caution zones when active.
         * Use distance thresholds (not float scale) for robust matching. */
        if (child_reaction_active) {
            if (dist >= dyn_warning && dist < dyn_caution) {
                /* Warning zone: tighten from 0.7 → 0.5 */
                target_scale = CHILD_REACTION_WARNING_SCALE;
            } else if (dist >= dyn_caution && dist < dyn_alert) {
                /* Caution zone: tighten from 0.85 → 0.7 */
                target_scale = CHILD_REACTION_CAUTION_SCALE;
            }
        }
    }

    /* ---- State machine with temporal hysteresis ---- */
    switch (obstacle_state) {

    case OBS_STATE_NO_SENSOR:
        /* First valid data arrived — transition to NORMAL */
        obstacle_state = OBS_STATE_NORMAL;
        obstacle_confirm_tick = 0;
        obstacle_clear_tick   = 0;
        safety_status.obstacle_scale = 1.0f;
        obstacle_forward_blocked = 0;
        break;

    case OBS_STATE_NORMAL:
        if (target_scale < 1.0f) {
            /* Potential obstacle — start temporal confirmation */
            obstacle_state = OBS_STATE_CONFIRMING;
            obstacle_confirm_tick = now;
            /* During confirmation, start reducing gently (0.7)
             * to avoid sudden full-speed into obstacle.                */
            safety_status.obstacle_scale = obstacle_preemptive_scale(target_scale);
            obstacle_forward_blocked = 0;
        } else {
            safety_status.obstacle_scale = 1.0f;
            obstacle_forward_blocked = 0;
        }
        ServiceMode_ClearFault(MODULE_OBSTACLE_DETECT);
        if (safety_error == SAFETY_ERROR_OBSTACLE) {
            Safety_ClearError(SAFETY_ERROR_OBSTACLE);
        }
        break;

    case OBS_STATE_CONFIRMING:
        if (target_scale >= 1.0f) {
            /* Obstacle disappeared before confirmation — back to NORMAL */
            obstacle_state = OBS_STATE_NORMAL;
            safety_status.obstacle_scale = 1.0f;
            obstacle_forward_blocked = 0;
            obstacle_confirm_tick = 0;
        } else if ((now - obstacle_confirm_tick) >= OBSTACLE_CONFIRM_MS) {
            /* Obstacle confirmed — apply full reduction */
            obstacle_state = OBS_STATE_ACTIVE;
            safety_status.obstacle_scale = target_scale;
            obstacle_forward_blocked = target_blocked;
            if (target_blocked) {
                Safety_SetError(SAFETY_ERROR_OBSTACLE);
            }
        } else {
            /* Still confirming — apply gentle preemptive reduction */
            safety_status.obstacle_scale = obstacle_preemptive_scale(target_scale);
            obstacle_forward_blocked = 0;
        }
        break;

    case OBS_STATE_ACTIVE:
        if (target_scale >= 1.0f) {
            /* Obstacle receded — start clearance timer */
            obstacle_state = OBS_STATE_CLEARING;
            obstacle_clear_tick = now;
            /* Keep current restriction during clearance confirmation */
            obstacle_forward_blocked = 0;
            safety_status.obstacle_scale = 0.7f;
        } else if (dist >= OBSTACLE_RECOVERY_MM && target_blocked == 0) {
            /* Obstacle moved past emergency but still in range —
             * start clearance check from reduced level             */
            obstacle_state = OBS_STATE_CLEARING;
            obstacle_clear_tick = now;
            safety_status.obstacle_scale = target_scale;
            obstacle_forward_blocked = 0;
        } else {
            /* Still active — apply target scale */
            safety_status.obstacle_scale = target_scale;
            obstacle_forward_blocked = target_blocked;
            if (target_blocked) {
                Safety_SetError(SAFETY_ERROR_OBSTACLE);
            }
        }
        break;

    case OBS_STATE_CLEARING:
        if (target_scale <= 0.3f) {
            /* Obstacle returned close — back to ACTIVE immediately */
            obstacle_state = OBS_STATE_ACTIVE;
            obstacle_clear_tick = 0;
            safety_status.obstacle_scale = target_scale;
            obstacle_forward_blocked = target_blocked;
        } else if ((now - obstacle_clear_tick) >= OBSTACLE_CLEAR_MS) {
            /* Sustained clearance — return to NORMAL */
            obstacle_state = OBS_STATE_NORMAL;
            obstacle_clear_tick = 0;
            safety_status.obstacle_scale = 1.0f;
            obstacle_forward_blocked = 0;
            ServiceMode_ClearFault(MODULE_OBSTACLE_DETECT);
            Safety_ClearError(SAFETY_ERROR_OBSTACLE);
        } else {
            /* Still confirming clearance — keep moderate reduction */
            safety_status.obstacle_scale = obstacle_preemptive_scale(target_scale);
            obstacle_forward_blocked = 0;
        }
        break;

    case OBS_STATE_SENSOR_FAULT:
        /* Handled above — should not reach here.  Fallback: */
        safety_status.obstacle_scale = OBSTACLE_FAULT_SCALE;
        obstacle_forward_blocked = 0;
        break;

    default:
        /* Unknown state — reset to safe default */
        obstacle_state = OBS_STATE_NO_SENSOR;
        safety_status.obstacle_scale = 1.0f;
        obstacle_forward_blocked = 0;
        break;
    }
}

/**
 * @brief  Get the current obstacle torque scale factor.
 * @return 0.0–1.0 (0.0 = full stop, 1.0 = no obstacle reduction)
 */
float Obstacle_GetScale(void)
{
    return safety_status.obstacle_scale;
}

/**
 * @brief  Query whether forward motion is blocked by an obstacle.
 * @return true if forward blocked (reverse escape available).
 */
bool Obstacle_IsForwardBlocked(void)
{
    return (obstacle_forward_blocked != 0);
}

/**
 * @brief  Get the current obstacle state machine state.
 * @return Current ObstacleState_t value.
 */
ObstacleState_t Obstacle_GetState(void)
{
    return obstacle_state;
}