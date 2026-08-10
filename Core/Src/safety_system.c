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
#include "standby_mode_sync_policy.h"
#include "main.h"
#include "battery_limits_store.h"
#include "sensor_manager.h"
#include "motor_control.h"
#include "service_mode.h"
#include "boot_validation.h"
#include "error_log.h"
#include "can_handler.h"
#include "math_safety.h"
#include "current_plausibility.h"
#include "relay_health_diag.h"    /* Problem 3: evidence-graded relay/current diagnosis */
#include "steering_eps.h"         /* EPS local isolation authority (mechanical-only) */
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
 * Recovery from SAFE is gated, not blocked:
 *   A critically depleted pack stays in SAFE while the fault persists.
 *   When the pack voltage genuinely returns above the hysteresis
 *   threshold (CRITICAL + HYST = 18.5 V) and stays there for a stable
 *   window (BATTERY_UV_RECOVERY_STABLE_MS), the active fault is cleared
 *   and the system leaves SAFE to STANDBY (motion still disabled).  This
 *   lets a temporary battery disconnect/reconnect recover without a
 *   manual reset, while hysteresis + stable-time prevent flapping and an
 *   invalid / I2C-failed reading never auto-recovers (fail-safe).        */
#define BATTERY_UV_WARNING_V    20.0f   /* Default derate point (now runtime) */
#define BATTERY_UV_CRITICAL_V   18.0f   /* Default SAFE cutoff (now runtime)  */
#define BATTERY_UV_HYST_V       0.5f    /* Hysteresis band for recovery   */

/* ---- Runtime-configurable battery limits (FASE 3) ----
 * The under-voltage thresholds compared in Safety_CheckBatteryVoltage()
 * become runtime variables.  Seeded with battery_limits_store.h defaults,
 * which MIRROR the historic #define values above (Limit/Warning 20.0 V,
 * Cutoff 18.0 V, Recovery = Cutoff + HYST = 18.5 V, Filter 0 ms), so the
 * comparisons are byte-for-byte identical until the operator re-tunes.
 * Only the VALUES change here — the state machine is untouched.           */
static BatteryLimits_t batt_limits = {
    .warning_cv  = BATT_WARNING_DEFAULT_CV,
    .limit_cv    = BATT_LIMIT_DEFAULT_CV,
    .cutoff_cv   = BATT_CUTOFF_DEFAULT_CV,
    .recovery_cv = BATT_RECOVERY_DEFAULT_CV,
    .filter_ms   = BATT_FILTER_DEFAULT_MS,
};

/* ---- Optional voltage filter (EMA) state ----
 * With filter_ms == 0 (default) the filter is bypassed and the raw bus
 * voltage is used directly, so behaviour is identical to the original
 * firmware.  With filter_ms > 0 a first-order EMA whose alpha = dt/(tc+dt)
 * smooths the reading used for the threshold COMPARES only — the invalid /
 * zero-reading fail-safe check below always runs on the raw value.        */
static float    batt_v_filtered  = 0.0f;
static bool     batt_v_filt_init = false;
static uint32_t batt_v_filt_tick = 0U;

/* Convert a centivolt threshold to volts. */
static inline float batt_cv_to_v(uint16_t cv) { return (float)cv * 0.01f; }

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

/* ---- Relay power sequencing delays (milliseconds) ----
 *
 * ⚠ HARDWARE INTEGRATION CONTRACT — RELAY TIMING OWNERSHIP:
 *
 * The STM32 firmware owns ALL relay timing.  The non-blocking sequencer
 * (Relay_SequencerUpdate) is called every 10 ms from the main safety loop
 * and uses HAL_GetTick() timestamps to progress through the sequence:
 *
 *   TRACTION ON  ──(50 ms)──▸  DIRECTION ON
 *   (PC11)                      (PC12)
 *
 * NOTE: PC10 is a free GPIO (INPUT_PULLDOWN, not connected) — the 24 V
 * battery feeds RELAY_TRAC (PC11) directly.  Only two power relays exist.
 *
 * Total sequence: ~50 ms (deterministic, jitter ≤ 10 ms from loop cadence).
 *
 * ⚠ EXTERNAL DELAY RELAY MODULES (if present in the power path):
 *   External "delay relay" modules (5V/12V/24V timer relays) used to
 *   interface between the STM32 GPIO → módulo 4-ch opto relé → power relay
 *   MUST be configured with ZERO delay (minimum timer setting).
 *   The firmware relay sequencer already provides the required inrush
 *   settling and arc suppression timing.  Any additional delay from
 *   external timer modules would:
 *     - Extend the power-up window beyond the expected ~70 ms
 *     - Desynchronise relay health checks (300 ms post-activation)
 *     - Cause false RELAY_OPEN faults if motors are commanded before
 *       the external relay physically closes
 *
 * ⚠ POWER-READY GATE:
 *   No subsystem may assume relay power is available until
 *   Safety_IsPowerReady() returns true.  Traction_Update() gates
 *   all motor output on this condition.  The relay health check
 *   (Safety_CheckRelayHealth) provides post-hoc fault detection
 *   after power is established.
 *
 * ⚠ RELAY SEQUENCER DETERMINISM:
 *   Relay_SequencerUpdate() is a pure state machine with no blocking
 *   calls, no HAL_Delay, and no I2C/SPI transactions.  It executes
 *   in < 1 µs per call.  The 10 ms loop cadence introduces up to
 *   10 ms of jitter per stage (within specification for SRD-05VDC
 *   relays with 10 ms activation time).                                */
#define RELAY_TRACTION_SETTLE_MS 50   /* Traction relay arc suppression +
                                       * 4× BTS7960 inrush settling delay.
                                       * Bumped from 20 ms → 50 ms to cover
                                       * worst-case capacitive inrush on the
                                       * 24 V bus before the steering power relay
                                       * and steering electronics come up.  */

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
 *      an adaptive threshold that scales with throttle demand.
 *   3. If below threshold for RELAY_CHK_DEBOUNCE_CYCLES consecutive
 *      calls AND traction controller is actively commanding output,
 *      trigger SAFETY_ERROR_RELAY_OPEN → DEGRADED L3.
 *   4. Voltage correlation: below RELAY_CHK_VOLTAGE_MIN_V use relaxed
 *      thresholds (higher debounce + lower current floor) instead of
 *      skipping entirely.  Below RELAY_CHK_VOLTAGE_DEAD_V: skip
 *      (battery too depleted for any meaningful current measurement).
 *   5. Recovery hysteresis: fault clears ONLY after RELAY_CHK_RECOVERY_MS
 *      of sustained healthy current AND RELAY_CHK_FAULT_HOLD_MS minimum
 *      visibility — prevents DEGRADED ↔ ACTIVE oscillation and ensures
 *      the user sees the fault on HMI.
 *
 * Debounce: fault condition must persist for RELAY_CHK_DEBOUNCE_CYCLES
 * consecutive 10 ms checks to trigger — rejects transient glitches.
 *
 * Adaptive threshold rationale:
 *   threshold = clamp(BASE_A + K × throttle_pct, BASE_A, MAX_A)
 *   At 20 %: 1.5 + 0.02 × 20 = 1.9 A  (gentle — low-throttle creep)
 *   At 50 %: 1.5 + 0.02 × 50 = 2.5 A  (moderate drive)
 *   At 100%: 1.5 + 0.02 × 100= 3.5 A  (high confidence at full power)
 *   Clamped to [1.5, 4.0] A to prevent runaway from corrupted throttle.
 *   This avoids false positives at low throttle while maintaining
 *   tight fault detection under high demand.  One multiply + one add.
 *
 * Brownout handling rationale:
 *   Between VOLTAGE_DEAD_V (12 V) and VOLTAGE_MIN_V (18 V) the battery
 *   may be too weak to deliver full motor current even with relay closed.
 *   Instead of skipping entirely (blind zone), use relaxed params:
 *   lower current floor (1.0 A — any measurable flow confirms relay)
 *   and higher debounce (8 cycles / 80 ms — reject transient noise).    */
#define RELAY_CHK_THROTTLE_MIN_PCT  20.0f   /* Primary: min throttle %      */
#define RELAY_CHK_THROTTLE_SPEED_PCT 10.0f  /* Secondary: lower floor when
                                             * speed confirms motion        */
#define RELAY_CHK_SPEED_MIN_KMH     2.0f    /* Min speed for secondary
                                             * activation (km/h)            */
#define RELAY_CHK_DELAY_MS          300U    /* Settle time after activation  */
#define RELAY_CHK_CURRENT_BASE_A    1.5f    /* Adaptive threshold: base (A) */
#define RELAY_CHK_CURRENT_K         0.02f   /* Adaptive threshold: slope
                                             * (A per % throttle)           */
#define RELAY_CHK_CURRENT_MAX_A     4.0f    /* Adaptive threshold: ceiling.
                                             * Clamps result of BASE + K×thr
                                             * to prevent runaway from
                                             * corrupted throttle input.    */
#define RELAY_CHK_DEBOUNCE_CYCLES   3U      /* Consecutive fails to trigger */
#define RELAY_CHK_RECOVERY_MS       1500U   /* Hysteresis: sustain healthy
                                             * current this long to clear   */
#define RELAY_CHK_VOLTAGE_MIN_V     18.0f   /* Below this: brownout mode
                                             * (relaxed detection)          */
#define RELAY_CHK_VOLTAGE_DEAD_V    12.0f   /* Below this: skip entirely
                                             * (battery dead / invalid)     */
#define RELAY_CHK_BROWNOUT_CURRENT_A 1.0f   /* Relaxed current floor under
                                             * brownout: any flow = relay ok */
#define RELAY_CHK_BROWNOUT_DEBOUNCE 8U      /* Higher debounce under brownout
                                             * (80 ms at 10 ms loop rate)   */
#define RELAY_CHK_FAULT_HOLD_MS     2000U   /* Min fault visibility for UX.
                                             * Once triggered, error stays
                                             * visible for at least this
                                             * duration before clearing.    */
#define RELAY_CHK_STALL_SPEED_KMH  0.5f    /* Min avg wheel speed (km/h) to
                                             * confirm motion.  Below this,
                                             * motors may be stalled (wall /
                                             * obstacle) — relay fault is
                                             * suppressed to avoid FP.      */
#define SENSOR_TEMP_MIN_C    (-40.0f)
#define SENSOR_TEMP_MAX_C    125.0f   /* DS18B20 absolute range */
/* Roadmap 1.4 — temperature cross-validation: per-sensor deviation
 * (°C) from the median of the OTHER enabled, valid DS18B20s above
 * which the sensor is tagged MODULE_FAULT_WARNING.  Diagnostic only.   */
#define TEMP_CROSS_DEV_C      30.0f
/* Roadmap 1.4 — wheel-speed median outlier: minimum per-wheel speed
 * required for the median check to run.  Below this, pulse-period
 * extrapolation dominates and small absolute deltas inflate the
 * relative deviation.                                                  */
#define WHEEL_MEDIAN_MIN_KMH   5.0f
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

/* ---- Per-wheel speed-sensor diagnostics (report-only) ----
 * wheel_diag[i]         : latest WheelDiag_t reason for channel i.
 * wheel_mismatch_since[i]: HAL_GetTick() when a mismatch first appeared
 *                          while under traction (0 = no active mismatch).
 *                          Used for the WHEEL_FAULT_DEBOUNCE_MS filter.  */
static WheelDiag_t wheel_diag[NUM_WHEELS]         = {WHEEL_DIAG_OK};
static uint32_t    wheel_mismatch_since[NUM_WHEELS] = {0};

/* Reason captured at the instant a channel's service-mode fault was latched.
 * The live wheel_diag[i] self-heals back to OK as soon as the wheel produces
 * coherent pulses again, but the service-mode fault stays latched — so without
 * this the HMI showed "all OK" next to a red LATCH flag with no way to tell
 * WHICH channel aborted.  wheel_latched_reason[i] keeps the culprit reason for
 * as long as the fault is latched, cleared when the fault clears.            */
static WheelDiag_t wheel_latched_reason[NUM_WHEELS] = {WHEEL_DIAG_OK};

/* ---- Non-blocking relay sequencer state machine ----
 *
 * The 24 V battery has only ONE relay (traction, PC11).  The 12 V
 * battery feeds the steering actuator power relay (PC12).  Two-stage power-up sequence:
 *   TRACTION_ON → (RELAY_TRACTION_SETTLE_MS) → COMPLETE (direction on)
 * Power-down order is reversed: DIR off, then TRAC off.               */
typedef enum {
    RELAY_SEQ_IDLE = 0,      /* All relays off, no sequence in progress   */
    RELAY_SEQ_TRACTION_ON,   /* Traction relay energised, waiting settle  */
    RELAY_SEQ_COMPLETE       /* All relays on, sequence finished          */
} RelaySeqState_t;

static RelaySeqState_t relay_seq_state     = RELAY_SEQ_IDLE;
static uint32_t        relay_seq_timestamp = 0;

/* ---- Relay override (engineering diagnostic mode) ----
 * Manual relay GPIO control gated through strict safety conditions.
 * Override is ONLY effective in STANDBY with zero throttle, zero speed,
 * and no active safety errors.  Automatically disabled on any violation.
 *
 * relay_override_mask bit layout (3-bit, backward-compatible with the
 * SERVICE_CMD 0xE0 wire contract — bit 0 is reserved/ignored for
 * forward compatibility with rev 1.3 consumers):
 *   bit 0: reserved (always 0)
 *   bit 1: TRACTION  relay (PC11)
 *   bit 2: STEER_PWR relay (PC12, 12 V steering actuator power)         */
static bool    relay_override_enabled = false;
static uint8_t relay_override_mask    = 0;

/* ---- Relay health check runtime state ----
 * Tracks whether the relay-open detection window is active and how
 * many consecutive 10 ms cycles the fault condition has persisted.
 * relay_chk_recovery_tick tracks the recovery hysteresis window —
 * the fault is only cleared after sustained healthy current.
 * relay_chk_fault_set_tick tracks when the fault was first set —
 * ensures minimum visibility on HMI (UX anti-flicker).               */
static uint8_t  relay_chk_active       = 0;     /* 1 = timing window running  */
static uint32_t relay_chk_start_tick   = 0;     /* HAL_GetTick() at window start */
static uint8_t  relay_chk_debounce     = 0;     /* Consecutive fail counter   */
static uint32_t relay_chk_recovery_tick = 0;    /* 0 = not recovering; else
                                                  * HAL_GetTick() at start of
                                                  * healthy current window      */
static uint32_t relay_chk_fault_set_tick = 0;   /* 0 = no active fault; else
                                                  * HAL_GetTick() when fault
                                                  * first triggered (for min
                                                  * visibility hold)            */

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

/* Reason-tracking flag for LIMP_HOME entry.
 * Set to true when LIMP_HOME is entered from STANDBY via a sensor fault
 * (pedal implausibility), not from ACTIVE/DEGRADED via CAN loss.
 * Used to route the recovery back to STANDBY (not ACTIVE) once the
 * sensor fault clears, allowing the calibration gate conditions to be
 * satisfied again (STANDBY + startup_inhibit).                           */
static bool limp_entered_from_standby = false;

/* Battery critical-UV recovery debounce (SAFE → STANDBY).
 *
 * A critically depleted pack that is RECHARGED or RECONNECTED (the pack
 * voltage genuinely returns to a valid, stable range) must be able to
 * leave SAFE without a manual reset — otherwise an intermittent battery
 * disconnect latches the vehicle in SAFE until power-cycle.
 *
 * Recovery is intentionally conservative:
 *   - Voltage must rise above the hysteresis-corrected threshold
 *     (BATTERY_UV_CRITICAL_V + BATTERY_UV_HYST_V = 18.5 V), not merely
 *     back above the trip point, to avoid flapping at the boundary.
 *   - It must stay there continuously for BATTERY_UV_RECOVERY_STABLE_MS.
 *     Pack voltage rebounds slowly and can sag again under the first
 *     load, so this window is longer than RECOVERY_HOLD_MS.
 *   - The recovery target is STANDBY (motion disabled), NOT ACTIVE: the
 *     normal STANDBY → ACTIVE promotion (Safety_CheckCANTimeout) then
 *     re-enables drive only when every other precondition is met.
 *   - An invalid / I2C-failed reading never recovers here (it returns
 *     early and keeps the system in SAFE), preserving the distinction
 *     between a real undervoltage and a sensor/bus fault.
 * The historical DTC stays in the error log; only the *active* fault is
 * cleared so the main display can leave SAFE.                           */
#define BATTERY_UV_RECOVERY_STABLE_MS 2000U
static uint32_t batt_uv_recovery_since   = 0;
static uint8_t  batt_uv_recovery_pending = 0;

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
#define OBSTACLE_EMERGENCY_MM       500     /* < 500 mm (50 cm) → scale = 0.0 */
#define OBSTACLE_CRITICAL_MM        1000    /* 500–1000 mm → scale = 0.3      */
#define OBSTACLE_WARNING_MM         1500    /* 1000–1500 mm → scale = 0.7     */
#define OBSTACLE_CAUTION_MM         2000    /* 1500–2000 mm → scale = 0.85    */
#define OBSTACLE_ALERT_MM           4000    /* 2000–4000 mm → scale = 0.95    */

/* Temporal hysteresis */
#define OBSTACLE_CONFIRM_MS         200     /* Confirm obstacle before acting  */
#define OBSTACLE_CLEAR_MS           1000    /* Confirm clearance before reset  */
/* Spatial hysteresis: obstacle is considered cleared only above
 * OBSTACLE_RECOVERY_MM, which is deliberately set well beyond
 * OBSTACLE_EMERGENCY_MM (500 mm) to prevent zone oscillation at the
 * boundary under LiDAR noise or EMI.  Band = 250 mm. */
#define OBSTACLE_RECOVERY_MM        750     /* Min distance to start clearing  */

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
static uint8_t  obstacle_zone            = 0;      /* Last zone (0–4)          */
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

    /* Auto-disable relay override on any state transition.
     * Override is ONLY valid in STANDBY; leaving STANDBY (or entering
     * any other state) must immediately revoke the override.             */
    if (relay_override_enabled) {
        Safety_SetRelayOverride(false, 0);
    }

    /* Only allow forward transitions and recovery transitions:
     *   DEGRADED→ACTIVE, LIMP_HOME→ACTIVE, LIMP_HOME→STANDBY (sensor-fault
     *   recovery from a STANDBY-origin fault), and the fault-cleared
     *   SAFE→STANDBY recovery (see SYS_STATE_STANDBY case).
     * SAFE→ACTIVE direct transition is explicitly blocked: recovery from SAFE
     * MUST go through STANDBY first (SAFE→STANDBY→ACTIVE).               */
    switch (state) {
        case SYS_STATE_STANDBY:
            if (system_state == SYS_STATE_BOOT)
                system_state = SYS_STATE_STANDBY;
            /* Safe recovery target after a latched fault has cleared
             * (e.g. battery critical-UV that returned to a valid, stable
             * range).  Only allowed when no active fault remains, so we
             * never relax safety: relays stay down (Safety_FailSafe was
             * already invoked on SAFE entry) and motion stays disabled
             * until the normal STANDBY → ACTIVE promotion re-validates
             * every precondition.                                       */
            else if (system_state == SYS_STATE_SAFE &&
                     safety_error == SAFETY_ERROR_NONE) {
                system_state    = SYS_STATE_STANDBY;
                degraded_level  = DEGRADED_LEVEL_NONE;
                degraded_reason = DEGRADED_REASON_NONE;
            }
            /* Sensor-fault recovery: LIMP_HOME → STANDBY.
             * Only allowed when the system entered LIMP_HOME from STANDBY
             * (tracked by limp_entered_from_standby) AND no active fault
             * remains.  Relays stay in their current state (PowerUp was
             * called on STANDBY→LIMP_HOME; we go back to STANDBY which
             * gates traction demand independently).  Motion stays
             * disabled until normal STANDBY→ACTIVE promotion.           */
            else if (system_state == SYS_STATE_LIMP_HOME &&
                     limp_entered_from_standby &&
                     safety_error == SAFETY_ERROR_NONE) {
                limp_entered_from_standby = false;
                system_state    = SYS_STATE_STANDBY;
                degraded_level  = DEGRADED_LEVEL_NONE;
                degraded_reason = DEGRADED_REASON_NONE;
            }
            break;

        case SYS_STATE_ACTIVE:
            /* SAFE→ACTIVE is intentionally excluded: recovery from SAFE
             * requires passing through STANDBY (SAFE→STANDBY→ACTIVE).   */
            if (system_state == SYS_STATE_STANDBY ||
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
                SystemState_t prev_limp = system_state;
                system_state = SYS_STATE_LIMP_HOME;
                degraded_level  = DEGRADED_LEVEL_NONE;
                degraded_reason = DEGRADED_REASON_NONE;
                /* Track the entry source so Safety_CheckSensors() can
                 * route recovery back to STANDBY (not ACTIVE) when the
                 * sensor fault was the sole reason for LIMP_HOME and the
                 * vehicle had not yet entered ACTIVE operation.          */
                limp_entered_from_standby = (prev_limp == SYS_STATE_STANDBY);
                /* Keep relays on — vehicle must remain drivable */
                Relay_PowerUp();
                /* Boot-without-CAN path: gear is still at the NEUTRAL
                 * boot default because the ESP32 never sent a CMD_MODE.
                 * Default to GEAR_FORWARD (D1) so the vehicle can move
                 * at walking speed without any CAN/ESP32 connection.
                 * Not applied on ACTIVE/DEGRADED→LIMP_HOME transitions
                 * because those keep the last real gear from the ESP32. */
                if (prev_limp == SYS_STATE_STANDBY) {
                    GearPosition_t g = Traction_GetGear();
                    if (g == GEAR_NEUTRAL || g == GEAR_PARK)
                        Traction_SetGear(GEAR_FORWARD);
                }
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
 * LIMP_HOME → 0.40 (40 %) — used to scale dynamic-brake target only;
 *             traction demand is already clamped once in main.c
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
 * LIMP_HOME → 1.0 — the 40 % LIMP_HOME ceiling is applied ONCE at the
 *             pedal clamp in main.c (demand = pedal * LIMP_HOME_TORQUE_LIMIT_
 *             FACTOR).  Returning the factor here again would double-scale
 *             (0.40 * 0.40 = 16 %; previously 0.20 * 0.20 = 4 %).  AUDIT G.
 * Others    → 0.0                                                       */
float Safety_GetTractionCapFactor(void)
{
    if (system_state == SYS_STATE_ACTIVE)    return 1.0f;
    if (system_state == SYS_STATE_LIMP_HOME) return 1.0f;
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

    /* Step 1: Energise traction relay and record timestamp.
     * (The 24 V bus has only one switch — the traction relay
     *  is the first and only 24 V-side contact.)                       */
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_SET);
    relay_seq_state     = RELAY_SEQ_TRACTION_ON;
    relay_seq_timestamp = HAL_GetTick();
}

void Relay_SequencerUpdate(void)
{
    /* Non-blocking relay sequencer — call from the 10 ms safety loop.
     * Progresses through the power-up sequence using timestamps:
     *   TRACTION_ON  →  (RELAY_TRACTION_SETTLE_MS)  →  COMPLETE (DIR on)
     * IDLE and COMPLETE are no-ops.                                     */

    /* Hard gate: NEVER energise relays unless the system is in the
     * fully operational ACTIVE state.  This is a defence-in-depth
     * safeguard that prevents the steering power relay from being turned ON
     * by this 10 ms tick if a fault, SAFE, ERROR, or emergency stop
     * occurred between Relay_PowerUp() and the next sequencer tick.
     *
     * Edge case — mid-sequence state transition:
     *   If the system leaves ACTIVE while the sequencer is in
     *   RELAY_SEQ_TRACTION_ON (TRAC already energised, DIR still OFF,
     *   waiting for the 50 ms settle delay), a plain early-return would
     *   leave the hardware in an inconsistent partial state
     *   (TRAC=ON, DIR=OFF) until something else powers down.
     *   Force a full, atomic power-down here so the relays are always
     *   in a coherent state — either fully OFF or fully ON (TRAC+DIR).
     *
     * Degenerate cases handled safely:
     *   - SAFE / ERROR: Safety_FailSafe / Safety_PowerDown /
     *     Safety_EmergencyStop already called Relay_PowerDown() before
     *     this tick; relay_seq_state == IDLE, so the mid-sequence check
     *     below is a no-op and we simply return.
     *   - STANDBY / BOOT: sequencer is IDLE by construction; no-op.
     *   - DEGRADED / LIMP_HOME: Relay_PowerUp() is only called on the
     *     ACTIVE→{DEGRADED,LIMP_HOME} transition where the sequencer is
     *     already COMPLETE (re-entry-safe PowerUp is a no-op), so relays
     *     stay fully ON and this path returns without touching them.
     *     If the transition happens mid-sequence, we intentionally
     *     power-down to avoid a partial (TRAC=ON, DIR=OFF) state rather
     *     than silently leaving the vehicle half-energised.            */
    if (system_state != SYS_STATE_ACTIVE) {
        /* Never leave a partially-energised relay state behind. */
        if (relay_seq_state == RELAY_SEQ_TRACTION_ON) {
            Relay_PowerDown();
        }
        return;
    }

    uint32_t now = HAL_GetTick();

    switch (relay_seq_state) {
        case RELAY_SEQ_TRACTION_ON:
            if ((now - relay_seq_timestamp) >= RELAY_TRACTION_SETTLE_MS) {
                /* Honour a latched EPS isolation: keep the steering rail
                 * (PC12) OFF so an isolated assist fault never auto-
                 * reconnects the motor.  Traction still completes normally
                 * (relay_seq_state → COMPLETE, PC11 already ON), so
                 * Safety_IsPowerReady() stays valid for traction.          */
                if (!Steering_IsAssistLatchedOff()) {
                    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_STEER_PWR, GPIO_PIN_SET);
                }
                relay_seq_state = RELAY_SEQ_COMPLETE;
            }
            break;

        case RELAY_SEQ_IDLE:
        case RELAY_SEQ_COMPLETE:
        default:
            break;
    }
}

void Steering_SteerPowerOff(void)
{
    /* Atomic single-pin release of the steering actuator rail (PC12).
     * BSRR upper half clears the bit in one bus cycle.  The traction relay
     * (PC11) and relay_seq_state are intentionally NOT touched, so traction
     * power readiness is preserved.  Repeated calls are harmless.          */
    GPIOC->BSRR = (uint32_t)PIN_RELAY_STEER_PWR << 16U;
}

/**
 * @brief  Single authorisation gate for the steering-motor isolation relay
 *         (PC12).  See the header for the full rationale and topology.
 *
 * PHYSICAL TOPOLOGY (owner-confirmed, authoritative):
 *   12 V BATTERY → INA226 CH5 + SHUNT → [BTS7960 if in this tranche]
 *                → STEER MOTOR RELAY (PC12) → STEERING MOTOR
 *   The INA226/shunt (CH5) sit BEFORE the relay, so CH5 stays powered and keeps
 *   measuring (~12 V, ~0 A) even with PC12 OFF.  PC12 is only a RELAY COMMAND;
 *   with no independent contact feedback the RELAY ACTUAL state is UNKNOWN.
 *
 * PC12 may be closed in exactly two mutually-exclusive situations, and never
 * while the assist is latched isolated:
 *   - EPS assist owns the motor (STEER_OWNER_EPS): only once the steering is
 *     CALIBRATED, the EPS is available and NOT latched.
 *   - Centering owns the motor (STEER_OWNER_CENTERING): during BOOT/STANDBY
 *     homing, the EPS available and NOT latched.  Homing must energise PC12
 *     BEFORE it can find centre and calibrate, so calibration is deliberately
 *     NOT required in this branch — this is what lets Safety_SteerRelaySupervise
 *     (which runs BEFORE SteeringCentering_Step) keep PC12 ON during
 *     CENTERING_WAIT_RAIL instead of tearing the rail down every cycle.
 *   - MECHANICAL_ONLY / ELECTRICAL_HAZARD (latched) or no owner: always OFF.
 * Returning false forces PC12 OFF everywhere (sequencer, override, telemetry)
 * and never touches the traction relay (PC11), so traction is fully preserved.
 */
bool Steering_MotorRelayAllowed(void)
{
    /* A latched isolation (MECHANICAL_ONLY or ELECTRICAL_HAZARD) forces PC12
     * OFF regardless of the owner — steering stays purely mechanical for the
     * rest of the power cycle. */
    if (!Steering_EpsIsAvailable() || Steering_IsAssistLatchedOff()) {
        return false;
    }

    /* Owner-aware authorisation.  Steering_EpsGetOwner() already collapses to
     * STEER_OWNER_NONE while latched (handled above), so here it reflects the
     * live writer decided by SteeringCentering_DecideOwner(). */
    switch (Steering_EpsGetOwner()) {
        case STEER_OWNER_EPS:
            /* Normal assist: the motor may only be energised once calibrated. */
            return Steering_IsCalibrated();

        case STEER_OWNER_CENTERING: {
            /* Homing needs the rail BEFORE calibration, but only while the
             * system is in a homing-capable state (BOOT or STANDBY). */
            const SystemState_t st = Safety_GetState();
            return (st == SYS_STATE_BOOT) || (st == SYS_STATE_STANDBY);
        }

        case STEER_OWNER_NONE:
        default:
            return false;
    }
}

void Relay_PowerDown(void)
{
    /* Deterministic shutdown: single atomic BSRR write forces BOTH relay
     * GPIOs to RESET in one cycle, regardless of sequencer state.
     *
     * Mask contains ONLY the power-relay pins (TRAC + DIR).  PC10 is
     * a free GPIO (input + pull-down) and is NOT touched.  Other GPIOC
     * outputs (motor ENs, PWM channels) are untouched — those are
     * handled separately by Traction_EmergencyStop().
     *
     * BSRR upper half (bits 16-31) = reset bits; lower half = set bits.
     * Writing PIN_RELAY_TRAC|PIN_RELAY_STEER_PWR shifted left 16 atomically
     * clears only those two outputs.                                    */
    GPIOC->BSRR = (uint32_t)(PIN_RELAY_TRAC | PIN_RELAY_STEER_PWR) << 16U;

    /* Cancel any in-progress power-up sequence AFTER the hardware
     * outputs are guaranteed to be low.  Also clears override state so
     * Safety_RelayOverrideUpdate() cannot re-energise relays on the
     * next 10 ms tick.                                                  */
    relay_seq_state        = RELAY_SEQ_IDLE;
    relay_override_enabled = false;
    relay_override_mask    = 0;
}

bool Safety_IsPowerReady(void)
{
    return (relay_seq_state == RELAY_SEQ_COMPLETE);
}

bool Relay_IsSequenceInProgress(void)
{
    return (relay_seq_state == RELAY_SEQ_TRACTION_ON);
}

/* ================================================================== */
/*  Relay Telemetry                                                    */
/* ================================================================== */
/* ---- RELAY TELEMETRY MODEL ----
 *
 * This function assembles a single byte that represents the current
 * relay COMMAND state, not the physical relay contact state.
 *
 * Relay verification hierarchy:
 *   1. GPIO = command state (what the firmware told the relay to do)
 *   2. CAN relay byte = exported GPIO state (what the ESP32 receives)
 *   3. INA226 = physical validation under load (only with motor demand)
 *
 * There is NO GPIO feedback input from the relay contacts.  The
 * STM32 drives relay coils through a 4-channel optocoupler relay module
 * (SRD-12VDC-SL-C, 12V).  The module's contacts switch 12V to the coils
 * of the high-current power relays (two-stage architecture).
 * and reads back the GPIO output state only.  Physical relay failure
 * (coil open, contact weld) is detected indirectly via INA226 motor
 * current in Safety_CheckRelayHealth().
 *
 * Debug assertion: if the sequencer reports COMPLETE but no relay
 * GPIOs are set, something is inconsistent.  This is checked only
 * in DEBUG builds to avoid runtime cost.                              */

uint8_t Safety_GetRelayStatusByte(void)
{
    uint8_t status = 0;

    /* Read GPIO output register — reports commanded state.
     *
     * CAN wire layout (backward-compatible 3-bit, bit 0 reserved):
     *   bit 0 = 0 (reserved; PC10 is a free GPIO, always 0)
     *   bit 1 = TRACTION  (PC11, 24 V — the only 24 V-side switch)
     *   bit 2 = STEER_PWR (PC12, 12 V — steering actuator supply; legacy
     *                       name "DIRECTION relay")
     *   bit 7 = SEQ_COMPLETE
     *
     * This layout preserves the original 3-bit contract so existing
     * ESP32 consumers that decode bit 1 = TRAC and bit 2 = DIR continue
     * to work unchanged.                                                 */
    if (HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_TRAC)) status |= (1U << 1);

    /* PC12 (steering-motor isolation relay) — bit 2 reports the REAL GPIO
     * command (ODR) unconditionally.  It is deliberately NOT masked by the
     * authorisation policy: masking would hide a policy-violating GPIO order
     * (PC12 commanded ON while the assist is isolated) from telemetry, CAN and
     * the HMI.  A commanded-but-not-allowed state is surfaced separately as a
     * policy violation (Safety_GetSteerRelayDiag / Safety_SteerRelaySupervise)
     * and forced OFF by the supervisor, but it is never concealed here. */
    if (HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR)) {
        status |= (1U << 2);
    }

    /* Sequence complete flag — RELAY_SEQ_COMPLETE requires the traction relay
     * (PC11) to actually be ON.  Reporting COMPLETE without PC11 energised
     * would be incoherent, so gate bit 7 on the PC11 command state.          */
    if (relay_seq_state == RELAY_SEQ_COMPLETE &&
        HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_TRAC)) {
        status |= (1U << 7);
    }

#ifdef DEBUG
    /* Consistency assertion: COMPLETE implies the traction relay (PC11, bit 1)
     * is ON.  Independently, a PC12 command that is NOT authorised by
     * Steering_MotorRelayAllowed() is a policy violation and must trap so it is
     * caught in Debug rather than silently masked.  "PC11 ON, PC12 OFF" is a
     * coherent mechanical-only state and must NOT trip the assertion.        */
    {
        bool pc12_commanded = (status & (1U << 2)) != 0U;
        bool pc12_violation = pc12_commanded && !Steering_MotorRelayAllowed();
        bool complete_ok    = (relay_seq_state != RELAY_SEQ_COMPLETE) ||
                              ((status & (1U << 1)) != 0U);
        if (!complete_ok || pc12_violation) {
            /* Breakpoint trap for SWD debugger — NOP in release builds */
            __NOP();
        }
    }
#endif

    return status;
}

/**
 * @brief  Evidence-grade steering-motor relay (PC12) diagnostic.
 *
 * Reports the three distinct quantities without conflating them:
 *   - RELAY CMD     : the REAL GPIO command (ODR) driving PC12.
 *   - RELAY ALLOWED : the computed authorisation policy
 *                     (Steering_MotorRelayAllowed()).
 *   - POLICY VIOL.  : CMD ON while NOT ALLOWED — a real order that violates
 *                     the isolation policy.  Never hidden.
 *
 * RELAY ACTUAL (contact closed / open) is deliberately absent: the installed
 * hardware has no post-relay feedback, so the physical contact state is
 * UNKNOWN and must not be fabricated from the command.
 */
void Safety_GetSteerRelayDiag(SteerRelayDiag_t *out)
{
    if (out == NULL) {
        return;
    }
    out->pc12_commanded        = (HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR)
                                    == GPIO_PIN_SET);
    out->pc12_allowed          = Steering_MotorRelayAllowed();
    out->pc12_policy_violation = out->pc12_commanded && !out->pc12_allowed;
}

/**
 * @brief  Steering-motor relay (PC12) policy supervisor.
 *
 * Independent, always-run guard: if PC12 is physically commanded ON while the
 * authorisation policy forbids it (isolated assist), that is a policy
 * violation.  The violation:
 *   - is asserted (Debug breakpoint trap);
 *   - forces PC12 OFF via a single atomic BSRR write (PC11/traction and the
 *     relay sequencer state are untouched);
 *   - remains fully visible through Safety_GetSteerRelayDiag() and the relay
 *     status byte (it is never masked).
 *
 * @return true if a policy violation was detected (and corrected) this call.
 */
bool Safety_SteerRelaySupervise(void)
{
    bool commanded = (HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR)
                        == GPIO_PIN_SET);
    bool violation = commanded && !Steering_MotorRelayAllowed();

    if (violation) {
#ifdef DEBUG
        /* Breakpoint trap for SWD debugger — NOP in release builds. */
        __NOP();
#endif
        /* Force PC12 OFF atomically; PC11 (traction) is never touched. */
        GPIOC->BSRR = (uint32_t)PIN_RELAY_STEER_PWR << 16U;
    }

    return violation;
}

#ifdef HOST_TEST
/**
 * @brief  Test-only fault injection: drive the raw PC12 GPIO command ON from
 *         WITHIN this translation unit.  Host tests use per-TU GPIO models, so
 *         a policy-violating hardware state (PC12 commanded ON while the assist
 *         is isolated) — which production paths never create — can only be
 *         reproduced from inside the safety translation unit.  Compiled out of
 *         firmware builds.
 */
void Safety_TestInjectSteerRelayOn(void)
{
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_STEER_PWR, GPIO_PIN_SET);
}

/**
 * @brief  Test-only companion to Safety_TestInjectSteerRelayOn(): drive the
 *         raw PC12 GPIO command OFF from WITHIN this translation unit.  With
 *         the per-TU host GPIO model this lets an integration test mirror the
 *         steering-centering FSM's PC12 command (raised in its OWN TU) into
 *         the safety TU, so Safety_SteerRelaySupervise() reads back the SAME
 *         PC12 state the homing FSM commanded.  Compiled out of firmware.
 */
void Safety_TestInjectSteerRelayOff(void)
{
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_STEER_PWR, GPIO_PIN_RESET);
}
#endif

/* ================================================================== */
/*  Relay Override (Engineering Diagnostic Mode)                        */
/* ================================================================== */

void Safety_SetRelayOverride(bool enabled, uint8_t mask)
{
    if (enabled) {
        /* Safety gating: override ONLY allowed in strict conditions */
        if (system_state != SYS_STATE_STANDBY) {
            relay_override_enabled = false;
            relay_override_mask    = 0;
            return;
        }
        if (safety_error != SAFETY_ERROR_NONE) {
            relay_override_enabled = false;
            relay_override_mask    = 0;
            return;
        }
        /* Throttle must be at rest (≤ 1 % dead-zone for analog pedal noise).
         * Pedal_GetPercent() returns EMA-filtered 0–100 % value.
         * The 1 % threshold matches the pedal rest noise floor.          */
        float throttle = Pedal_GetPercent();
        if (throttle > 1.0f) {
            relay_override_enabled = false;
            relay_override_mask    = 0;
            return;
        }
        /* Speed must be 0 km/h */
        float avg_speed = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                           Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) * 0.25f;
        if (avg_speed > 0.5f) {
            relay_override_enabled = false;
            relay_override_mask    = 0;
            return;
        }
        /* Normal relay sequence must NOT be in progress */
        if (relay_seq_state != RELAY_SEQ_IDLE) {
            relay_override_enabled = false;
            relay_override_mask    = 0;
            return;
        }
        relay_override_enabled = true;
        relay_override_mask    = mask & 0x06U;  /* Only bits 1-2 valid (bit 0 reserved) */

        /* EPS isolation guard: never store the steering-motor relay bit
         * (0x04 / PC12) when the assist is latched MECHANICAL_ONLY or
         * ELECTRICAL_HAZARD.  A new engineering override must not be able
         * to re-close the steering-motor relay after isolation — not even
         * for a single tick.  Traction (bit 1 / PC11) is unaffected.       */
        if (Steering_IsAssistLatchedOff()) {
            relay_override_mask &= (uint8_t)~0x04U;
        }
    } else {
        /* Disable override — turn off all relay GPIOs that were set by override */
        if (relay_override_enabled) {
            HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, PIN_RELAY_STEER_PWR,  GPIO_PIN_RESET);
        }
        relay_override_enabled = false;
        relay_override_mask    = 0;
    }
}

bool Safety_IsRelayOverrideActive(void)
{
    return relay_override_enabled;
}

void Safety_RelayOverrideUpdate(void)
{
    /* Called from 10 ms safety loop — after normal relay sequencing.
     * Continuous safety re-validation: auto-disable on any violation.    */
    if (!relay_override_enabled) return;

    /* Re-check safety conditions every cycle */
    if (system_state != SYS_STATE_STANDBY ||
        safety_error != SAFETY_ERROR_NONE) {
        Safety_SetRelayOverride(false, 0);
        return;
    }
    float throttle = Pedal_GetPercent();
    if (throttle > 1.0f) {
        Safety_SetRelayOverride(false, 0);
        return;
    }
    float avg_speed = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                       Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) * 0.25f;
    if (avg_speed > 0.5f) {
        Safety_SetRelayOverride(false, 0);
        return;
    }
    /* Normal sequencer must remain idle while override is active */
    if (relay_seq_state != RELAY_SEQ_IDLE) {
        Safety_SetRelayOverride(false, 0);
        return;
    }

    /* Apply override mask to relay GPIOs (bit 0 reserved/ignored) */
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC,
                      (relay_override_mask & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_STEER_PWR,
                      (relay_override_mask & 0x04U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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

/* §3 (Option A) — see header for the full rationale.  Applies ONLY to the
 * logical mode flags; the caller must NOT change gear or energise motion when
 * this returns true.  Deliberately does NOT call Safety_IsCommandAllowed(): its
 * whole purpose is to permit a mode-flag sync in STANDBY, where command motion
 * is (correctly) still forbidden.                                             */
bool Safety_IsStandbyModeSyncAllowed(void)
{
    /* Gather all inputs and delegate the decision to the HAL-free policy
     * (standby_mode_sync_policy.c).  This is the ONLY place that calls
     * StandbyModeSync_Evaluate() in the firmware.                        */

    /* Wheel speeds → average km/h. */
    float avg_speed = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                       Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;

    /* Internal demand from the traction state. */
    const TractionState_t *ts = Traction_GetState();
    float demand_pct = (ts != NULL) ? ts->demandPct : 100.0f;

    StandbySyncInput_t in;
    in.state               = (uint8_t)system_state;
    in.pedalPct            = sanitize_float(Pedal_GetPercent(), 100.0f);
    in.demandPct           = sanitize_float(demand_pct, 100.0f);
    in.effectiveDemandPct  = sanitize_float(Traction_GetEffectiveDemandPct(), 100.0f);
    in.finalPwmPct         = Traction_GetFinalPwmPct();
    in.avgSpeedKmh         = sanitize_float(avg_speed, MODE_CHANGE_MAX_SPEED_KMH + 1.0f);
    in.errorOrHazardActive = false;   /* existing gate: state check covers this */

    return (StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY);
}

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
    batt_uv_recovery_since   = 0;
    batt_uv_recovery_pending = 0;
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
    relay_chk_fault_set_tick = 0;
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

/* ---- Powertrain-engaged gate -------------------------------------------
 * True only when the vehicle is in a drive-capable state AND actually
 * commanding traction above WHEEL_INTERVENTION_MIN_DEMAND_PCT.
 *
 * ABS/TCS and the wheel-mismatch plausibility fault are meaningless when
 * no torque is being applied: spinning a wheel by hand in STANDBY (or in
 * ACTIVE with the pedal released) is expected and must NOT trigger an
 * intervention or a SENSOR_FAULT.  When the powertrain is engaged a real
 * wheel/sensor problem still produces slip under load and is caught.      */
static bool Safety_PowertrainEngaged(void)
{
    const TractionState_t *ts = Traction_GetState();
    float demand = (ts != NULL) ? ts->demandPct : 0.0f;
    if (demand < 0.0f) demand = -demand;              /* reverse counts too */
    if (demand < WHEEL_INTERVENTION_MIN_DEMAND_PCT) return false;
    return (system_state == SYS_STATE_ACTIVE   ||
            system_state == SYS_STATE_DEGRADED ||
            system_state == SYS_STATE_LIMP_HOME);
}

WheelDiag_t Safety_GetWheelDiag(uint8_t idx)
{
    if (idx >= NUM_WHEELS) return WHEEL_DIAG_OK;
    return wheel_diag[idx];
}

WheelDiag_t Safety_GetWheelLatchedReason(uint8_t idx)
{
    if (idx >= NUM_WHEELS) return WHEEL_DIAG_OK;
    /* While a service-mode fault is latched on this channel, report the reason
     * captured at latch time so the culprit stays identifiable even after the
     * live wheel_diag[idx] self-heals back to OK.  When no fault is latched the
     * live reason is returned (may be OK / MANUAL_MOVEMENT / DISABLED_STATE).  */
    if (ServiceMode_GetFault((ModuleID_t)(MODULE_WHEEL_SPEED_FL + idx))
            != MODULE_FAULT_NONE &&
        wheel_latched_reason[idx] != WHEEL_DIAG_OK) {
        return wheel_latched_reason[idx];
    }
    return wheel_diag[idx];
}

/* Diagnostic reason for the STEER/CENTER channel (0x313 byte 4).  Report-only:
 * surfaces the steering encoder as DISABLED_STATE when its service module is
 * turned off (e.g. the E6B2-CWZ6C encoder is not yet wired) so the HMI shows
 * a clear "DESHAB." instead of a misleading OK/fault.  Genuine encoder faults
 * are reported through the existing FAULT_ENCODER_ERROR path, not here.       */
uint8_t Safety_GetSteerDiagReason(void)
{
    if (!ServiceMode_IsEnabled(MODULE_STEER_ENCODER)) {
        return (uint8_t)WHEEL_DIAG_DISABLED_STATE;
    }
    return (uint8_t)WHEEL_DIAG_OK;
}

uint8_t Safety_WheelDiagToCanReason(WheelDiag_t diag)
{
    /* WheelDiag_t codes 0-7 are transmitted verbatim; any value outside the
     * known enum range collapses to the stable UNKNOWN wire code (8).     */
    if ((unsigned)diag <= (unsigned)WHEEL_DIAG_DISABLED_STATE) {
        return (uint8_t)diag;
    }
    return WHEEL_DIAG_CAN_UNKNOWN;
}

bool Safety_IsPowertrainEngaged(void)
{
    return Safety_PowertrainEngaged();
}

uint8_t Safety_GetWheelFaultMask(void)
{
    uint8_t mask = 0U;
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        if (ServiceMode_GetFault((ModuleID_t)(MODULE_WHEEL_SPEED_FL + i))
                != MODULE_FAULT_NONE) {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

void ABS_Update(void)
{
    /* Skip if ABS module is disabled (service mode) */
    if (!ServiceMode_IsEnabled(MODULE_ABS)) {
        safety_status.abs_active = false;
        safety_status.abs_wheel_mask = 0;
        return;
    }

    /* Powertrain gate: ABS only intervenes when torque is being applied.
     * Spinning a wheel by hand (pedal released / STANDBY) must not raise
     * abs_active — otherwise the mismatch between the hand-spun wheel and
     * the three stationary wheels is misread as wheel lock-up (0x20).    */
    if (!Safety_PowertrainEngaged()) {
        safety_status.abs_active = false;
        safety_status.abs_wheel_mask = 0;
        for (uint8_t i = 0; i < 4; i++) {
            safety_status.wheel_scale[i] = 1.0f;
            abs_pulse_timer[i] = HAL_GetTick();
            abs_pulse_phase[i] = 0U;
        }
        return;
    }

    /* Implicit dependency: ABS requires wheel speed sensors.
     * Reference average is computed from HEALTHY wheels only (enabled and
     * fault-free).  A disabled or faulted sensor reporting 0 km/h would
     * otherwise corrupt the average and trigger false ABS intervention on
     * the dead wheel AND false TCS throttling on every healthy wheel.
     * The speed gate in CAN_ID_SERVICE_CMD (can_handler.c) prevents
     * disabling wheel speed sensors while the vehicle is in motion
     * (avg_spd > 0.5), so ABS is always available when it matters.       */
    float spd[4];
    spd[0] = Wheel_GetSpeed_FL();
    spd[1] = Wheel_GetSpeed_FR();
    spd[2] = Wheel_GetSpeed_RL();
    spd[3] = Wheel_GetSpeed_RR();

    /* Compute average from healthy (enabled + fault-free) sensors only.
     * In 4x2 mode exclude rear drag wheels: their braked state means
     * spd[2/3] ≈ 0 even when the front axle is moving, which would halve
     * the reference and cause false lock-up detection on the rear AND
     * mask real front-wheel lock-up by making front wheels appear fast. */
    bool abs_rear_driven = Traction_GetState()->mode4x4 ||
                           Traction_GetState()->axisRotation;
    float    sum_spd   = 0.0f;
    uint8_t  n_healthy = 0U;
    for (uint8_t i = 0; i < 4; i++) {
        ModuleID_t mod = (ModuleID_t)(MODULE_WHEEL_SPEED_FL + i);
        if (!ServiceMode_IsEnabled(mod)) continue;
        if (ServiceMode_GetFault(mod) != MODULE_FAULT_NONE) continue;
        if (!abs_rear_driven && i >= 2U) continue;  /* Skip drag wheels */
        sum_spd += spd[i];
        n_healthy++;
    }
    float avg = (n_healthy > 0U) ? (sum_spd / (float)n_healthy) : 0.0f;

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
        ModuleID_t mod = (ModuleID_t)(MODULE_WHEEL_SPEED_FL + i);
        /* Skip disabled or faulted wheel sensors — do not trigger ABS on
         * a dead sensor, and do not reduce its wheel_scale artificially. */
        if (!ServiceMode_IsEnabled(mod) ||
            ServiceMode_GetFault(mod) != MODULE_FAULT_NONE) {
            safety_status.wheel_scale[i] = 1.0f;
            abs_pulse_timer[i] = now;
            abs_pulse_phase[i] = 0U;
            continue;
        }
        /* Skip ABS intervention on non-driven rear drag wheels in 4x2.
         * Their braked state is intentional; not wheel lock-up.        */
        if (!abs_rear_driven && i >= 2U) {
            safety_status.wheel_scale[i] = 1.0f;
            abs_pulse_timer[i] = now;
            abs_pulse_phase[i] = 0U;
            continue;
        }

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
        /* Global fallback: if ALL healthy wheels lock, apply global throttle
         * cut as a last-resort safety measure (vehicle is on ice or
         * sensors are unreliable).  Compare against healthy-wheel mask only
         * so a dead sensor never forces a global cut by itself.           */
        uint8_t healthy_mask = 0U;
        for (uint8_t i = 0; i < 4; i++) {
            ModuleID_t mod = (ModuleID_t)(MODULE_WHEEL_SPEED_FL + i);
            if (!ServiceMode_IsEnabled(mod)) continue;
            if (ServiceMode_GetFault(mod) != MODULE_FAULT_NONE) continue;
            if (!abs_rear_driven && i >= 2U) continue;  /* Skip drag wheels */
            healthy_mask |= (1U << i);
        }
        if (healthy_mask != 0U && (mask & healthy_mask) == healthy_mask) {
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

    /* Powertrain gate: TCS only intervenes when torque is being applied.
     * A wheel spun by hand with the pedal released (or in STANDBY) is not
     * wheelspin — suppress activation so 0x40 is not raised on manual
     * movement.  Under real load a spinning wheel is still caught.        */
    if (!Safety_PowertrainEngaged()) {
        safety_status.tcs_active = false;
        safety_status.tcs_wheel_mask = 0;
        for (uint8_t i = 0; i < 4; i++)
            tcs_reduction[i] = 0.0f;
        tcs_last_tick = HAL_GetTick();
        return;
    }

    /* Implicit dependency: TCS requires wheel speed sensors.
     * Reference average is computed from HEALTHY wheels only (enabled and
     * fault-free).  A disabled or faulted sensor reporting 0 km/h would
     * otherwise corrupt the average and trigger false TCS throttling on
     * every healthy wheel (and false ABS on the dead wheel via ABS_Update).
     * The speed gate in CAN_ID_SERVICE_CMD (can_handler.c) prevents
     * disabling wheel speed sensors while the vehicle is in motion
     * (avg_spd > 0.5), so TCS is always available when it matters.       */
    float spd[4];
    spd[0] = Wheel_GetSpeed_FL();
    spd[1] = Wheel_GetSpeed_FR();
    spd[2] = Wheel_GetSpeed_RL();
    spd[3] = Wheel_GetSpeed_RR();

    /* Compute average from healthy (enabled + fault-free) sensors only.
     * In 4x2 mode exclude rear drag wheels: their braked state means
     * spd[2/3] ≈ 0 while the front axle is moving, which would halve
     * the reference and cause false TCS throttle cuts on the front
     * driven wheels (they would appear to be "spinning" vs. the low avg). */
    bool tcs_rear_driven = Traction_GetState()->mode4x4 ||
                           Traction_GetState()->axisRotation;
    float    sum_spd   = 0.0f;
    uint8_t  n_healthy = 0U;
    for (uint8_t i = 0; i < 4; i++) {
        ModuleID_t mod = (ModuleID_t)(MODULE_WHEEL_SPEED_FL + i);
        if (!ServiceMode_IsEnabled(mod)) continue;
        if (ServiceMode_GetFault(mod) != MODULE_FAULT_NONE) continue;
        if (!tcs_rear_driven && i >= 2U) continue;  /* Skip drag wheels */
        sum_spd += spd[i];
        n_healthy++;
    }
    float avg = (n_healthy > 0U) ? (sum_spd / (float)n_healthy) : 0.0f;

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
        ModuleID_t mod = (ModuleID_t)(MODULE_WHEEL_SPEED_FL + i);
        /* Skip disabled or faulted wheel sensors — a dead sensor must not
         * trigger TCS on itself, and must not corrupt the average used to
         * evaluate healthy wheels.  Clear any residual reduction so the
         * motor on this channel runs without artificial TCS restriction. */
        if (!ServiceMode_IsEnabled(mod) ||
            ServiceMode_GetFault(mod) != MODULE_FAULT_NONE) {
            tcs_reduction[i] = 0.0f;
            /* Do NOT touch wheel_scale here: ABS_Update has priority and
             * runs first; leave its result for Traction_Update.          */
            continue;
        }
        /* Skip TCS intervention on non-driven rear drag wheels in 4x2.
         * Their braked state is intentional; not wheelspin.             */
        if (!tcs_rear_driven && i >= 2U) {
            tcs_reduction[i] = 0.0f;
            continue;
        }

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
        /* Global fallback: if ALL healthy wheels spin, apply global limit
         * as last-resort safety (all traction lost).  Compare against the
         * healthy-wheel mask so a dead sensor never forces a global cut.  */
        uint8_t healthy_mask = 0U;
        for (uint8_t i = 0; i < 4; i++) {
            ModuleID_t mod = (ModuleID_t)(MODULE_WHEEL_SPEED_FL + i);
            if (!ServiceMode_IsEnabled(mod)) continue;
            if (ServiceMode_GetFault(mod) != MODULE_FAULT_NONE) continue;
            if (!tcs_rear_driven && i >= 2U) continue;  /* Skip drag wheels */
            healthy_mask |= (1U << i);
        }
        if (healthy_mask != 0U && (mask & healthy_mask) == healthy_mask) {
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
 *        – Relay_PowerDown() de-energises traction and steering-power relays
 *        – Steering centred or neutralised
 *   Hardware fuses and BTS7960 internal current limiting handle sub-ms
 *   transients; this software path protects against sustained faults.  */

void Safety_CheckCurrent(void)
{
    for (uint8_t i = 0; i < NUM_INA226; i++) {
        /* Steering (CH5) is EPS-owned: its overcurrent is handled exclusively
         * by the steering supervisor's dedicated FSM (isolate the assist relay
         * PC12, stay MECHANICAL_ONLY, keep the vehicle ACTIVE with full
         * traction; only a proven persistent hazard escalates).  Routing CH5
         * through this GLOBAL check would wrongly force DEGRADED/SAFE on an
         * isolable steering fault, so skip it here.                          */
        if (i == INA226_CHANNEL_STEER) continue;

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

        /* Roadmap 1.5: skip sensors whose most recent read failed CRC
         * or returned out-of-range data.  The per-sensor MODULE_FAULT
         * is already raised by Temperature_ReadAll(); evaluating an
         * invalid sample here would either mask other sensors with a
         * bogus reading or trigger spurious overtemp via stale data.  */
        if (!Temperature_IsValid(i)) continue;

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
            /* Roadmap 1.5: invalid sample cannot confirm hysteresis
             * recovery; treat as "not yet below" so the safety_error
             * remains latched until a valid reading is available.    */
            if (!Temperature_IsValid(i)) {
                all_below_hysteresis = false;
                break;
            }
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

/**
 * @brief  Single authority deciding whether the steering state permits the
 *         vehicle to enter (or return to) a drive-capable ACTIVE state.
 *
 * The steering is MECHANICAL; the EPS motor only assists.  A centering or
 * encoder fault during boot latches the EPS to MECHANICAL_ONLY, but the
 * vehicle must still gain traction — an isolated assist loss must never
 * immobilise the car in STANDBY/LIMP_HOME.  ACTIVE is therefore permitted
 * when the assist is calibrated OR when it has been cleanly isolated to
 * purely mechanical steering (EPS_STATE_MECHANICAL_ONLY).
 *
 * An ELECTRICAL_HAZARD must NEVER authorise ACTIVE.  It is guarded two ways:
 * an explicit Steering_IsElectricalHazard() veto (so it can never promote even
 * if the steering happens to still read "calibrated"), and — for the benign,
 * isolable case — a precise check against EPS_STATE_MECHANICAL_ONLY only.  The
 * hazard is handed to the safety subsystem for SAFE/ERROR escalation elsewhere;
 * meanwhile the vehicle may still crawl in LIMP_HOME but never enters ACTIVE.
 */
bool Steering_AllowsVehicleDrive(void)
{
    /* A genuine electrical danger vetoes ACTIVE unconditionally. */
    if (Steering_IsElectricalHazard()) {
        return false;
    }

    EpsState_t eps = Steering_GetEpsState();

    return Steering_IsCalibrated() ||
           (eps == EPS_STATE_MECHANICAL_ONLY);
}

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
         * only when steering centering has completed successfully,
         * the boot validation checklist has passed, AND the startup
         * inhibit latch has cleared (pedal held at rest).
         *
         * Requiring startup_inhibit == false keeps the system in
         * STANDBY while the driver confirms pedal release, giving a
         * practical calibration window at every (re-)arm cycle.     */
        if (system_state == SYS_STATE_STANDBY &&
            safety_error == SAFETY_ERROR_NONE &&
            Steering_AllowsVehicleDrive() &&
            BootValidation_IsPassed() &&
            !Startup_IsInhibited()) {
            Safety_SetState(SYS_STATE_ACTIVE);
        }
        /* CAN restored from LIMP_HOME → attempt ACTIVE.
         * Heartbeat has appeared and system is healthy.
         * ACTIVE requires the steering to permit driving — either the
         * assist is calibrated, or it has been cleanly isolated to
         * MECHANICAL_ONLY (an isolable EPS fault must never pin the
         * vehicle in LIMP_HOME).  If steering neither calibrated nor
         * mechanical-only, the vehicle stays in LIMP_HOME.
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
            Steering_AllowsVehicleDrive()) {
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
                    /* SAFETY (F2): force gear to NEUTRAL before re-enabling
                     * motion so that any stale `current_gear` latched during
                     * the outage is invalidated.  The ESP32 will retransmit
                     * the real shifter position on the next CMD_MODE update,
                     * which is the authoritative source.  While gear stays
                     * in NEUTRAL the demand pipeline in main.c/Traction_Update
                     * already clamps traction demand to 0.                 */
                    Traction_SetGear(GEAR_NEUTRAL);
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
            /* SAFETY (F2): also clear any stale gear latch on this path,
             * since SAFE→LIMP_HOME re-enables some motion (limited power).
             * The ESP32 will refresh the gear via CMD_MODE.               */
            Traction_SetGear(GEAR_NEUTRAL);
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

/* Bloque A (PR #445): read-only age-of-last-CAN-frame getter for the
 * service-diagnostic session, which applies its OWN stricter heartbeat
 * timeout (500 ms) independent of CAN_TIMEOUT_MS above. Purely additive,
 * no behaviour change to the existing CAN timeout state machine. */
uint32_t Safety_GetCanRxAgeMs(void)
{
    return HAL_GetTick() - last_can_rx_time;
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
        /* Roadmap 1.5: skip sensors with an invalid most-recent read.
         * The per-sensor MODULE_FAULT_ERROR is already set by
         * Temperature_ReadAll(); counting it again here would compound
         * fault_count and falsely escalate to DEGRADED L3.            */
        if (!Temperature_IsValid(i)) continue;
        float t = Temperature_Get(i);
        /* NaN/Inf hardening: invalid sensor reading → plausibility fault */
        if (isnan(t) || isinf(t) || t < SENSOR_TEMP_MIN_C || t > SENSOR_TEMP_MAX_C) {
            ServiceMode_SetFault(mod, MODULE_FAULT_ERROR);
            fault_count++;
        }
    }

    /* ------------------------------------------------------------------
     * Roadmap 1.4 (warning subset) — Temperature cross-validation.
     *
     * Compare each enabled, valid DS18B20 against the median of the
     * other enabled, valid DS18B20s.  When the absolute deviation
     * exceeds TEMP_CROSS_DEV_C (30 °C) the offending
     * MODULE_TEMP_SENSOR_i is tagged MODULE_FAULT_WARNING.  Diagnostic
     * only — does NOT increment fault_count and therefore CANNOT
     * escalate the system to DEGRADED.  The absolute thresholds in
     * Safety_CheckTemperature() (warning 80 °C, critical 90 °C) remain
     * the authoritative overtemp detector.
     *
     * Requires at least 3 enabled, valid sensors so the "other" set
     * after removing the candidate still has a meaningful median (≥2
     * samples).  With fewer than 3 there is no reliable reference.
     *
     * Severity ordering preserved: a pre-existing MODULE_FAULT_ERROR
     * is never downgraded to WARNING.
     * ------------------------------------------------------------------ */
    {
        float    samples[NUM_DS18B20];
        uint8_t  sample_idx[NUM_DS18B20];
        uint8_t  n = 0;
        for (uint8_t i = 0; i < NUM_DS18B20; i++) {
            ModuleID_t mod = (ModuleID_t)(MODULE_TEMP_SENSOR_0 + i);
            if (!ServiceMode_IsEnabled(mod)) continue;
            if (!Temperature_IsValid(i)) continue;
            float t = Temperature_Get(i);
            if (isnan(t) || isinf(t) ||
                t < SENSOR_TEMP_MIN_C || t > SENSOR_TEMP_MAX_C) continue;
            samples[n]    = t;
            sample_idx[n] = i;
            n++;
        }
        if (n >= 3) {
            for (uint8_t k = 0; k < n; k++) {
                /* Build the "other" set by copying every sample except k. */
                float others[NUM_DS18B20 - 1];
                uint8_t m = 0;
                for (uint8_t j = 0; j < n; j++) {
                    if (j != k) others[m++] = samples[j];
                }
                /* Insertion sort — m ≤ NUM_DS18B20-1 = 4, trivial cost. */
                for (uint8_t a = 1; a < m; a++) {
                    float key = others[a];
                    int8_t b = (int8_t)a - 1;
                    while (b >= 0 && others[b] > key) {
                        others[b + 1] = others[b];
                        b--;
                    }
                    others[b + 1] = key;
                }
                float median;
                if ((m & 1U) == 1U) {
                    median = others[m / 2U];
                } else {
                    median = 0.5f * (others[m / 2U - 1U] + others[m / 2U]);
                }
                float dev    = samples[k] - median;
                float absdev = (dev < 0.0f) ? -dev : dev;
                if (absdev > TEMP_CROSS_DEV_C) {
                    ModuleID_t mod = (ModuleID_t)(MODULE_TEMP_SENSOR_0 + sample_idx[k]);
                    if (ServiceMode_GetFault(mod) != MODULE_FAULT_ERROR) {
                        ServiceMode_SetFault(mod, MODULE_FAULT_WARNING);
                    }
                    /* fault_count NOT incremented — diagnostic only. */
                }
            }
        }
    }

    /* Current plausibility.  Motor channels are bidirectional, so
     * regeneration/back-EMF is validated by absolute magnitude rather than
     * by sign.  The battery channel keeps the legacy −1 A offset tolerance.
     * CH5 polarity is classified by its dedicated INA226 supervisor instead
     * of being collapsed into the global SENSOR_FAULT bucket.             */
    for (uint8_t i = 0; i < NUM_INA226; i++) {
        ModuleID_t mod = (ModuleID_t)(MODULE_CURRENT_SENSOR_0 + i);
        if (!ServiceMode_IsEnabled(mod)) continue;
        float a = Current_GetAmps(i);
        float ceil = (i == INA226_CHANNEL_BATTERY)
                   ? SENSOR_CURRENT_MAX_BATT_A : SENSOR_CURRENT_MAX_A;
        /* Motor channels accept either sign inside their physical range.
         * The battery channel remains unidirectional with −1 A tolerance. */
        if (CurrentPlausibility_IsFault(i == INA226_CHANNEL_BATTERY,
                                        a, ceil)) {
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
        /* Clear the persisted latch culprit once the service-mode fault has
         * cleared (e.g. after a service-screen restore).  Kept in sync so a
         * stale culprit reason never survives past its fault.               */
        if (ServiceMode_GetFault(mod) == MODULE_FAULT_NONE) {
            wheel_latched_reason[i] = WHEEL_DIAG_OK;
        }
        if (!ServiceMode_IsEnabled(mod)) {
            wheel_diag[i]          = WHEEL_DIAG_DISABLED_STATE;
            wheel_mismatch_since[i] = 0;
            continue;
        }
        /* NaN/Inf hardening: invalid speed reading → plausibility fault.
         * An impossible value is a genuine fault in any state.           */
        if (isnan(spd[i]) || isinf(spd[i]) ||
            spd[i] < 0.0f || spd[i] > SENSOR_SPEED_MAX_KMH) {
            ServiceMode_SetFault(mod, MODULE_FAULT_ERROR);
            fault_count++;
            wheel_diag[i]          = WHEEL_DIAG_IMPOSSIBLE_RATE;
            wheel_latched_reason[i] = WHEEL_DIAG_IMPOSSIBLE_RATE;
            wheel_mismatch_since[i] = 0;
            continue;
        }

        /* Stale-while-others-moving detection.
         *
         * A stale wheel (no pulses within timeout) reading 0 while other
         * wheels report speed CAN indicate a disconnected/failed sensor —
         * but it is ALSO exactly what happens when a single wheel is
         * turned by hand while the car is parked (the other three stay
         * still and a sensor sitting over a bolt emits no edge).
         *
         * To avoid a false SENSOR_FAULT / DEGRADED on manual movement we
         * only escalate this mismatch to a plausibility fault when:
         *   (a) the powertrain is actually applying torque, AND
         *   (b) the mismatch PERSISTS for WHEEL_FAULT_DEBOUNCE_MS.
         * Otherwise it is reported as MANUAL_MOVEMENT (diagnostic only).  */
        if (Wheel_IsStale(i) && spd[i] == 0.0f) {
            uint8_t any_moving = 0;
            for (uint8_t j = 0; j < 4; j++) {
                if (j != i && spd[j] > 1.0f) { any_moving = 1; break; }
            }
            if (!any_moving) {
                /* All-but-this stopped too → vehicle stopped, normal.    */
                wheel_diag[i]          = WHEEL_DIAG_OK;
                wheel_mismatch_since[i] = 0;
            } else if (!Safety_PowertrainEngaged()) {
                /* Others moving but no torque commanded → the motion is
                 * manual.  Never a fault; surface it for the operator.   */
                wheel_diag[i]          = WHEEL_DIAG_MANUAL_MOVEMENT;
                wheel_mismatch_since[i] = 0;
            } else {
                /* In 4x2 mode the rear wheels (RL=index 2, RR=index 3)
                 * are braked drag wheels — they are NOT expected to
                 * produce speed pulses matching the driven front axle.
                 * Treat a silent rear drag wheel as normal: not a fault. */
                const TractionState_t *ts = Traction_GetState();
                bool rear_drag = (i >= 2U) &&
                                 !(ts->mode4x4 || ts->axisRotation);
                if (rear_drag) {
                    wheel_diag[i]           = WHEEL_DIAG_OK;
                    wheel_mismatch_since[i] = 0;
                } else {
                    /* Under load and this wheel is silent → candidate fault.
                     * Require persistence before latching so a momentary
                     * difference (e.g. a wheel just starting to turn) does
                     * not force DEGRADED.                                   */
                    uint32_t now = HAL_GetTick();
                    if (wheel_mismatch_since[i] == 0) {
                        wheel_mismatch_since[i] = now;
                    }
                    if ((now - wheel_mismatch_since[i]) >= WHEEL_FAULT_DEBOUNCE_MS) {
                        ServiceMode_SetFault(mod, MODULE_FAULT_WARNING);
                        fault_count++;
                        /* Classify the silent channel using the raw pin level
                         * so the operator can tell a stuck sensor apart from
                         * a disconnected one.                               */
                        uint8_t lvl = Wheel_GetGpioLevel(i);
                        if      (lvl == 1U) wheel_diag[i] = WHEEL_DIAG_STUCK_HIGH;
                        else if (lvl == 0U) wheel_diag[i] = WHEEL_DIAG_STUCK_LOW;
                        else                wheel_diag[i] = WHEEL_DIAG_NO_PULSE;
                        /* Persist the classified reason so the culprit channel
                         * stays identifiable on the HMI after it self-heals. */
                        wheel_latched_reason[i] = wheel_diag[i];
                    } else {
                        /* Persisting but not yet latched — diagnostic only.  */
                        wheel_diag[i] = WHEEL_DIAG_MISMATCH;
                    }
                }
            }
        } else {
            /* Wheel is producing pulses (or vehicle stopped) — coherent. */
            wheel_diag[i]          = WHEEL_DIAG_OK;
            wheel_mismatch_since[i] = 0;
        }
    }

    /* ------------------------------------------------------------------
     * Roadmap 1.4 (warning subset) — Wheel-speed median outlier.
     *
     * When all four wheels are reporting genuine motion (> 5 km/h each),
     * a single wheel that deviates by more than 50 % from the median of
     * the OTHER three is flagged with MODULE_FAULT_WARNING.  This is a
     * pure-diagnostic signal: it does NOT increment fault_count and
     * therefore CANNOT escalate the system to DEGRADED.  Reasoning:
     *
     *   - The vehicle has independent reasons (TCS, ABS) for its
     *     control-loop response; this check is post-hoc and used to
     *     surface a flaky sensor to the operator before it fails hard.
     *   - The 5 km/h floor avoids the noisy quantisation regime where
     *     pulse-period extrapolation dominates and small absolute
     *     deltas inflate the relative deviation.
     *   - "All four > 5 km/h" prevents misfires on a stopped/dragging
     *     wheel — that case is already handled by the stale-detection
     *     path above.
     *
     * Auto-clear is intentionally left to ServiceMode (factory restore /
     * service screen).  The latch makes the warning visible across the
     * 1 Hz SERVICE_FAULTS frame even if the deviation is transient.
     * ------------------------------------------------------------------ */
    {
        bool all_moving = true;
        for (uint8_t i = 0; i < 4; i++) {
            if (isnan(spd[i]) || isinf(spd[i]) || spd[i] < WHEEL_MEDIAN_MIN_KMH) {
                all_moving = false;
                break;
            }
        }
        if (all_moving) {
            for (uint8_t i = 0; i < 4; i++) {
                ModuleID_t mod = (ModuleID_t)(MODULE_WHEEL_SPEED_FL + i);
                if (!ServiceMode_IsEnabled(mod)) continue;
                /* Median of the OTHER three wheels = the middle value
                 * of a 3-element set, i.e. (max+min) subtracted from
                 * the sum.                                              */
                float a = 0.0f, b = 0.0f, c = 0.0f;
                uint8_t k = 0;
                for (uint8_t j = 0; j < 4; j++) {
                    if (j == i) continue;
                    if (k == 0)      a = spd[j];
                    else if (k == 1) b = spd[j];
                    else             c = spd[j];
                    k++;
                }
                float min3 = (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c);
                float max3 = (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c);
                float med3 = (a + b + c) - (min3 + max3);
                if (med3 < WHEEL_MEDIAN_MIN_KMH) continue;  /* defensive */
                float dev    = spd[i] - med3;
                float absdev = (dev < 0.0f) ? -dev : dev;
                if ((absdev / med3) > 0.5f) {
                    /* Do NOT clobber a more severe ERROR potentially set
                     * by the loop above.  WARNING is strictly less.     */
                    if (ServiceMode_GetFault(mod) != MODULE_FAULT_ERROR) {
                        ServiceMode_SetFault(mod, MODULE_FAULT_WARNING);
                    }
                    /* Record the culprit reason for the HMI if nothing more
                     * specific was latched on this channel already.        */
                    if (wheel_latched_reason[i] == WHEEL_DIAG_OK) {
                        wheel_latched_reason[i] = WHEEL_DIAG_MISMATCH;
                    }
                    /* fault_count NOT incremented — diagnostic only.    */
                }
            }
        }
    }

    /* Pedal plausibility: dual-channel cross-validation.
     * ACTIVE state requires fully valid pedal plausibility — any
     * plausibility failure exits ACTIVE immediately.
     *
     * A plausibility failure now only occurs for GENUINELY indeterminate
     * signals (a fast but coherent stab is valid intent and is rate-limited
     * upstream, never faulted).  Two distinct failure modes remain:
     *   1. Contradictory: dual ADC samples disagree PERSISTENTLY.
     *      → No torque allowed (position indeterminate).
     *      → Demand forced to zero in ALL states.
     *   2. Implausible: rail/stuck-ADC range fault.
     *      → Cross-validation impossible, but primary ADC may be usable.
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
     * LIMP_HOME from ACTIVE/DEGRADED: CAN timeout handler recovers to ACTIVE.
     * LIMP_HOME from STANDBY: sensor fault was the sole cause → recover to
     *   STANDBY so calibration conditions can be re-established.
     *
     * Patch B (audit BUG-2): also allow clearing from SAFE.  Previously
     * SAFETY_ERROR_SENSOR_FAULT was only cleared in DEGRADED/LIMP_HOME, so
     * if a sensor/bus fault escalated to SAFE (e.g. battery disconnect that
     * also stalls the I2C sensor reads) the SENSOR_FAULT latch survived
     * forever and pinned the system in SAFE.  We reach this point only when
     * EVERY sensor check of the current cycle has passed (fault_count == 0
     * and pedal plausibility OK).  Because safety_error is single-slot,
     * safety_error == SAFETY_ERROR_SENSOR_FAULT also guarantees no other
     * critical fault is active.  The recovery target is STANDBY — never
     * ACTIVE — so motion stays disabled and the normal STANDBY -> ACTIVE
     * promotion re-validates every precondition.  The historical DTC stays
     * in the error log (only the active fault is cleared).                */
    if (safety_error == SAFETY_ERROR_SENSOR_FAULT &&
        system_state == SYS_STATE_LIMP_HOME) {
        Safety_ClearError(SAFETY_ERROR_SENSOR_FAULT);
        /* If the LIMP_HOME entry was from STANDBY (pedal fault before the
         * vehicle entered ACTIVE operation), recover back to STANDBY and
         * re-arm the startup inhibit latch so the calibration gate
         * conditions (STANDBY + startup_inhibit) can be satisfied again.
         * Safety_SetState() guards the transition with its own
         * limp_entered_from_standby check so no stale flag is consumed.  */
        if (limp_entered_from_standby && safety_error == SAFETY_ERROR_NONE) {
            Startup_Rearm();
            Safety_SetState(SYS_STATE_STANDBY);
        }
    } else if (safety_error == SAFETY_ERROR_SENSOR_FAULT &&
               system_state == SYS_STATE_DEGRADED) {
        Safety_ClearError(SAFETY_ERROR_SENSOR_FAULT);
    } else if (safety_error == SAFETY_ERROR_SENSOR_FAULT &&
               system_state == SYS_STATE_SAFE) {
        Safety_ClearError(SAFETY_ERROR_SENSOR_FAULT);
        /* Leave SAFE only if no other active fault remains.  If a different
         * fault latched meanwhile, Safety_SetState ignores the STANDBY
         * request and the system stays SAFE (fail-safe).                  */
        if (safety_error == SAFETY_ERROR_NONE) {
            Safety_SetState(SYS_STATE_STANDBY);
        }
    }
}

/* ---- Steering encoder health ----------------------------------------- */

/**
 * @brief  Check the steering encoder for faults.
 *
 * Delegates the actual detection to Encoder_CheckHealth() in
 * motor_control.c (which monitors range, jumps and frozen values).
 * A steering-encoder fault is an ISOLABLE ASSIST fault: the encoder is
 * only needed for the electric assist, not for the mechanical steering.
 * On fault the assist is disconnected via Steering_DisableAssistFault()
 * (motor isolated, steering purely mechanical) while the GLOBAL vehicle
 * state stays ACTIVE and traction is fully preserved.  No DEGRADED, no
 * LIMP_HOME, no traction reduction.
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

        /* Isolable assist fault: the steering encoder is required only for
         * the ELECTRIC ASSIST, never for the mechanical steering itself.
         * Disconnect the assist (PA6=PA7=0, PC4=LOW, PC12=OFF, owner=NONE)
         * and leave purely mechanical steering.  The global vehicle state
         * stays ACTIVE and traction is fully preserved — no DEGRADED, no
         * SENSOR_FAULT, no reduced power.  The fault is reported locally via
         * the EPS state / centering diagnostic (0x316) and the HMI.        */
        Steering_DisableAssistFault(EPS_FAULT_ENCODER_AB);
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
 *   - Gated auto-recovery: leaves SAFE to STANDBY once the pack stays
 *     above 18.5 V (CRITICAL + hysteresis) for BATTERY_UV_RECOVERY_STABLE_MS;
 *     a still-invalid / I2C-failed reading never auto-recovers (fail-safe)
 *
 * Sensor / I2C failure (0.0 V reading indicates I2C / multiplexer fault):
 *   - Treated as CRITICAL (fail-safe default); reported as Code 11
 *     (SAFETY_ERROR_I2C_FAILURE) when the mux/INA did not ACK, else as
 *     Code 10 (SAFETY_ERROR_BATTERY_UV_CRITICAL).
 *   - The battery INA226 normally sits at the battery terminal, but if the
 *     main battery connector is physically removed the whole I2C sensor bus
 *     loses power and stops ACKing — that legitimately raises Code 11.
 *   - Patch A: once the hardware physically recovers (mux present, battery
 *     INA ACKs + valid bus read) AND the voltage is back above 18.5 V and
 *     stays stable for BATTERY_UV_RECOVERY_STABLE_MS, the Code 11 latch is
 *     cleared and SAFE is left to STANDBY (never directly to ACTIVE).  An
 *     invalid reading or a missing mux/INA aborts the recovery window.
 *
 * Works alongside overcurrent / overtemperature / CAN timeout:
 *   - Does not interfere with existing fault escalation
 *   - Only sets error if no higher-priority fault is active
 */
void Safety_CheckBatteryVoltage(void)
{
    float raw_voltage = Voltage_GetBus(INA226_CHANNEL_BATTERY);

    /* Invalid reading (0.0 V / NaN / Inf): on this topology the battery
     * INA226 sits behind the TCA9548A multiplexer, so a 0 V reading almost
     * always means the mux channel-select or the INA226 read did NOT ACK on
     * I2C — NOT that the pack is critically depleted.  Mapping that case to
     * BATT UV CRIT (Code 10) masks the real cause and contradicts the I2C
     * topology diagnostic.  Distinguish the two with the mux-presence,
     * INA-ACK, and bus-voltage-read-valid bits published by Current_ReadAll():
     *   - battery INA/bus read did NOT ACK  → genuine I2C/sensor fault (Code 11)
     *   - battery INA + bus read both ACKed but still read ~0 V → real critical UV
     * Both still enter SAFE (fail-safe), only the reported error differs.
     * This fail-safe check ALWAYS runs on the raw reading — the optional
     * voltage filter below only smooths the numeric threshold compares.   */
    if (isnan(raw_voltage) || isinf(raw_voltage) || raw_voltage <= 0.0f) {
        bool batt_sample_ok = Sensor_GetMuxPresent() &&
                              ((Sensor_GetInaOkMask() & INA226_MASK_BATTERY) != 0U) &&
                              ((Sensor_GetInaBusOkMask() & INA226_MASK_BATTERY) != 0U);
        Safety_SetError(batt_sample_ok ? SAFETY_ERROR_BATTERY_UV_CRITICAL
                                       : SAFETY_ERROR_I2C_FAILURE);
        Safety_SetState(SYS_STATE_SAFE);
        batt_uv_recovery_pending = 0;   /* invalid reading aborts recovery */
        batt_v_filt_init = false;       /* invalid reading resets the filter */
        return;
    }

    /* A valid raw sample below the critical cutoff is an immediate
     * electrical protection event.  Never let the optional EMA delay SAFE;
     * the filter is only for non-critical bands and stable recovery. */
    const float uv_cutoff_v = batt_cv_to_v(batt_limits.cutoff_cv);
    if (raw_voltage < uv_cutoff_v) {
        Safety_SetError(SAFETY_ERROR_BATTERY_UV_CRITICAL);
        Safety_SetState(SYS_STATE_SAFE);
        batt_uv_recovery_pending = 0;
        batt_v_filt_init = false;
        return;
    }

    /* ---- Optional voltage filter (FASE 3) ----
     * filter_ms == 0 (default): bypass — voltage == raw, identical to today.
     * filter_ms  > 0: first-order EMA with alpha = dt/(tc+dt) so the time
     * constant is honoured regardless of the (variable) call cadence.      */
    float voltage;
    uint16_t filt_ms = batt_limits.filter_ms;
    if (filt_ms == 0U) {
        voltage = raw_voltage;
        batt_v_filt_init = false;
    } else {
        uint32_t nowf = HAL_GetTick();
        if (!batt_v_filt_init) {
            batt_v_filtered  = raw_voltage;
            batt_v_filt_init = true;
            batt_v_filt_tick = nowf;
        } else {
            float dt = (float)(nowf - batt_v_filt_tick);
            if (dt < 1.0f) dt = 1.0f;
            batt_v_filt_tick = nowf;
            float alpha = dt / ((float)filt_ms + dt);
            batt_v_filtered += alpha * (raw_voltage - batt_v_filtered);
        }
        voltage = batt_v_filtered;
    }

    /* Runtime thresholds (seeded == historic #define values). */
    const float uv_recovery_v = batt_cv_to_v(batt_limits.recovery_cv);
    const float uv_limit_v    = batt_cv_to_v(batt_limits.limit_cv);

    /* Critical undervoltage — SAFE state */
    if (voltage < uv_cutoff_v) {
        Safety_SetError(SAFETY_ERROR_BATTERY_UV_CRITICAL);
        Safety_SetState(SYS_STATE_SAFE);
        batt_uv_recovery_pending = 0;   /* still below trip: restart window */
        return;
    }

    /* Recovery from SAFE after a critical undervoltage (Code 10).
     *
     * Reached only while still latched in SAFE on a genuine battery UV
     * (voltage is already >= BATTERY_UV_CRITICAL_V here).  The pack must
     * return above the hysteresis threshold (CRITICAL + HYST = 18.5 V)
     * and stay there for BATTERY_UV_RECOVERY_STABLE_MS before we clear
     * the active fault and leave SAFE to STANDBY.  This handles the real
     * case of a temporary battery disconnect/reconnect without a manual
     * reset; hysteresis + stable-time prevent flapping.  Handling this
     * before the warning branch keeps the critical latch from being
     * overwritten by a warning-band reading while still in SAFE.
     *
     * We only act on a genuine battery UV latch — an I2C/sensor failure
     * (SAFETY_ERROR_I2C_FAILURE) is intentionally NOT cleared in THIS
     * block; it has its own dedicated recovery block below (Patch A) that
     * additionally requires the mux/INA to be physically back.  The
     * historical DTC remains in the error log; only
     * the active fault is cleared so the display can leave SAFE.  The
     * recovery target is STANDBY (motion stays disabled) — the normal
     * STANDBY -> ACTIVE promotion then re-validates every precondition
     * before re-enabling drive.                                          */
    if (system_state == SYS_STATE_SAFE &&
        safety_error == SAFETY_ERROR_BATTERY_UV_CRITICAL) {
        if (voltage > uv_recovery_v) {
            if (!batt_uv_recovery_pending) {
                batt_uv_recovery_pending = 1;
                batt_uv_recovery_since   = HAL_GetTick();
            } else if ((HAL_GetTick() - batt_uv_recovery_since) >=
                       BATTERY_UV_RECOVERY_STABLE_MS) {
                batt_uv_recovery_pending = 0;
                Safety_ClearError(SAFETY_ERROR_BATTERY_UV_CRITICAL);
                /* Leave SAFE only if no other active fault remains.  If a
                 * different fault latched meanwhile, Safety_SetState
                 * ignores the STANDBY request and the system stays SAFE. */
                if (safety_error == SAFETY_ERROR_NONE) {
                    Safety_SetState(SYS_STATE_STANDBY);
                }
            }
        } else {
            /* Above the trip point but still inside the hysteresis band:
             * keep the SAFE/critical latch and restart the stable window. */
            batt_uv_recovery_pending = 0;
        }
        return;
    }

    /* Patch A — Recovery from SAFE after an I2C / sensor-bus failure
     * (Code 11) once the hardware has physically recovered.
     *
     * Rationale (audit BUG-1): SAFETY_ERROR_I2C_FAILURE had NO clearing
     * path anywhere in the firmware, so a transient loss of the I2C bus
     * (e.g. the operator disconnects and reconnects the main battery while
     * the STM32 logic rail stays powered) latched the system in SAFE until
     * a full power-cycle.  The TCA9548A multiplexer and the battery INA226
     * recover on their own (Current_ReadAll resets its per-cycle failure
     * counters), yet safety_error/SAFE were never released.
     *
     * This block leaves SAFE *only* when every recovery precondition holds,
     * preserving fail-safe behaviour:
     *   - reached here only with a VALID battery voltage reading (the
     *     invalid-reading branch above re-asserts the fault and returns),
     *   - the TCA9548A mux is present again AND the battery INA226 both
     *     ACKs and returns a valid bus-voltage read (Sensor_Get*),
     *   - the pack is above the critical+hysteresis threshold (18.5 V),
     *   - and it stays stable for BATTERY_UV_RECOVERY_STABLE_MS.
     *
     * The historical DTC is kept in the error log (only the active fault is
     * cleared).  The recovery target is STANDBY — never ACTIVE — so motion
     * stays disabled and the normal STANDBY -> ACTIVE promotion re-validates
     * every precondition before drive is re-enabled.  The same stable-window
     * timer is reused: safety_error is single-slot, so the UV-critical and
     * I2C-failure recovery scenarios are mutually exclusive.               */
    if (system_state == SYS_STATE_SAFE &&
        safety_error == SAFETY_ERROR_I2C_FAILURE) {
        bool batt_sample_ok = Sensor_GetMuxPresent() &&
            ((Sensor_GetInaOkMask()    & INA226_MASK_BATTERY) != 0U) &&
            ((Sensor_GetInaBusOkMask() & INA226_MASK_BATTERY) != 0U);
        if (batt_sample_ok &&
            voltage > uv_recovery_v) {
            if (!batt_uv_recovery_pending) {
                batt_uv_recovery_pending = 1;
                batt_uv_recovery_since   = HAL_GetTick();
            } else if ((HAL_GetTick() - batt_uv_recovery_since) >=
                       BATTERY_UV_RECOVERY_STABLE_MS) {
                batt_uv_recovery_pending = 0;
                Safety_ClearError(SAFETY_ERROR_I2C_FAILURE);
                /* Leave SAFE only if no other active fault remains.  If a
                 * different fault latched meanwhile, Safety_SetState
                 * ignores the STANDBY request and the system stays SAFE. */
                if (safety_error == SAFETY_ERROR_NONE) {
                    Safety_SetState(SYS_STATE_STANDBY);
                }
            }
        } else {
            /* Bus not fully back or voltage not yet stable above the
             * hysteresis band: keep the SAFE latch, restart the window. */
            batt_uv_recovery_pending = 0;
        }
        return;
    }

    /* Not in the SAFE/critical-UV recovery scenario: make sure a stale
     * debounce window cannot survive to a later trip.                    */
    batt_uv_recovery_pending = 0;

    /* Warning undervoltage — DEGRADED state with power limiting.
     * Uses the runtime derate (Limit) threshold; default 20.0 V.          */
    if (voltage < uv_limit_v) {
        Safety_SetError(SAFETY_ERROR_BATTERY_UV_WARNING);
        Safety_SetState(SYS_STATE_DEGRADED);
        Safety_SetDegradedLevel(DEGRADED_L2,
                                DEGRADED_REASON_BATTERY_UV);
        return;
    }

    /* Voltage OK — attempt recovery from DEGRADED if hysteresis met.
     * Recovery from DEGRADED requires voltage > Limit + HYSTERESIS
     * (default 20.5 V) to prevent oscillation under load transients.      */
    if (system_state == SYS_STATE_DEGRADED &&
        safety_error == SAFETY_ERROR_BATTERY_UV_WARNING &&
        voltage > (uv_limit_v + BATTERY_UV_HYST_V)) {
        Safety_ClearError(SAFETY_ERROR_BATTERY_UV_WARNING);
    }
}

/* ---- Runtime-configurable battery limits (FASE 3) ---------------- */

bool Safety_ValidateBatteryLimits(const BatteryLimits_t *b)
{
    /* Delegate to the single source of truth in battery_limits_store so the
     * CAN handler, the flash loader and the safety system all agree.       */
    return BatteryLimitsStore_Validate(b);
}

bool Safety_SetBatteryLimits(const BatteryLimits_t *b)
{
    /* Reject out-of-range sets and KEEP the previous values (FASE 4). */
    if (!Safety_ValidateBatteryLimits(b))
        return false;
    batt_limits = *b;
    /* A new filter time-constant restarts the EMA so the next sample seeds
     * it cleanly (avoids a stale value biasing the first compare).        */
    batt_v_filt_init = false;
    return true;
}

void Safety_GetBatteryLimits(BatteryLimits_t *out)
{
    if (out) *out = batt_limits;
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
 * motor current + wheel speed + battery voltage + traction demand).
 * Active ONLY in ACTIVE / DEGRADED / LIMP_HOME.
 *
 * Algorithm:
 *   1. Activation: throttle > 20% (primary) OR throttle > 10% with
 *      vehicle speed > 2 km/h (secondary — catches relay failure
 *      while vehicle is in motion at lower demand).
 *   2. After 300 ms settle, compute TOTAL absolute motor current
 *      (INA226 ch 0-3 sum).
 *   3. Voltage correlation:
 *      - Battery < 12 V: skip entirely (dead/invalid sensor)
 *      - Battery < 18 V: brownout mode — relaxed threshold (1.0 A)
 *        and higher debounce (8 cycles) to avoid blind zone while
 *        still preventing false positives.
 *      - Battery ≥ 18 V: normal adaptive threshold.
 *   4. Adaptive threshold: BASE_A + K × throttle_pct scales with
 *      demand — gentle at low throttle, tight at high throttle.
 *   5. Motion intent: cross-check Traction_GetState()->demandPct > 0
 *      before triggering — ensures the traction controller is
 *      actually commanding motor output (not limited by ABS/TCS/etc).
 *   6. If total current < threshold for debounce consecutive calls,
 *      trigger SAFETY_ERROR_RELAY_OPEN → DEGRADED L3.
 *   7. Recovery: fault clears ONLY after FAULT_HOLD_MS minimum
 *      visibility AND RECOVERY_MS of sustained healthy current —
 *      prevents oscillation and ensures user sees the fault on HMI.
 *
 * O(1) per call.  No division.  No blocking.  No CAN changes.
 * Called from the 10 ms safety loop (same tier as Safety_CheckCurrent). */

/* ---- Problem 3: evidence-graded relay / current-sense diagnosis ----------
 * Snapshot classified by the pure Relay_ClassifyHealth() every cycle and
 * exposed to the CAN/HMI path.  Instrumentation only: it never sets a fault
 * or gates control — the legacy latch in Safety_CheckRelayHealth() is left
 * exactly as-is.  It exists so the operator sees the REAL cause (and the
 * numbers behind it) instead of a bare "RELAY OPEN" when the motors clearly
 * turn but the shunt reads 0 A.                                             */
static RelayHealthDiag s_relay_diag = { .diagnostic_reason = RELAY_DIAG_INCONCLUSIVE };

void Safety_UpdateRelayHealthDiag(void)
{
    RelayHealthDiag d = (RelayHealthDiag){0};

    /* Relay / power path. */
    d.relay_commanded         = (relay_seq_state != RELAY_SEQ_IDLE);
    d.relay_sequence_complete = (relay_seq_state == RELAY_SEQ_COMPLETE);
    d.power_ready             = Safety_IsPowerReady();

    /* Demand chain.  The three demand signals MUST come from three distinct
     * real sources, otherwise a request that is legitimately suppressed to
     * zero (SAFE / ERROR / startup inhibit / relay-not-ready / ABS / TCS /
     * LIMP_HOME / thermal / battery / obstacle / motion-inhibit / …) would be
     * mis-read as "output present" and produce false RELAY OPEN / CURRENT
     * SENSE INVALID verdicts.                                                */
    d.throttle_pct = Pedal_GetPercent();
    const TractionState_t *ts = Traction_GetState();
    float traction_demand = (ts != (void *)0) ? ts->demandPct : 0.0f;
    d.traction_demand_pct  = traction_demand;                        /* requested demand      */
    d.effective_demand_pct = Traction_GetEffectiveDemandPct();       /* after all limits      */
    d.final_pwm_pct        = (float)Traction_GetFinalPwmPct();       /* PWM actually emitted  */

    /* Motion. */
    d.wheel_speed[0] = Wheel_GetSpeed_FL();
    d.wheel_speed[1] = Wheel_GetSpeed_FR();
    d.wheel_speed[2] = Wheel_GetSpeed_RL();
    d.wheel_speed[3] = Wheel_GetSpeed_RR();
    d.average_speed  = (d.wheel_speed[0] + d.wheel_speed[1] +
                        d.wheel_speed[2] + d.wheel_speed[3]) * 0.25f;
    d.any_wheel_moving = (d.wheel_speed[0] > RELAY_CHK_STALL_SPEED_KMH) ||
                         (d.wheel_speed[1] > RELAY_CHK_STALL_SPEED_KMH) ||
                         (d.wheel_speed[2] > RELAY_CHK_STALL_SPEED_KMH) ||
                         (d.wheel_speed[3] > RELAY_CHK_STALL_SPEED_KMH);

    /* Current sense (INA226 CH0..3). */
    d.ina_ok_mask        = Sensor_GetInaOkMask();
    d.ina_expected_mask  = Sensor_GetInaExpectedMask();
    d.current_sample_age_ms = Current_GetSampleAgeMs();

    uint8_t valid_mask = 0;
    float   sum_abs = 0.0f, signed_sum = 0.0f;
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        float amps = Current_GetAmps(i);
        d.current_ch[i] = amps;
        bool finite = !isnan(amps) && !isinf(amps);
        float mag = (amps < 0.0f) ? -amps : amps;
        if (finite && mag <= SENSOR_CURRENT_MAX_A) {
            valid_mask |= (uint8_t)(1U << i);
            sum_abs    += mag;
            signed_sum += amps;
        }
    }
    d.ina_valid_mask     = valid_mask;
    d.current_sum_abs    = sum_abs;
    d.current_signed_sum = signed_sum;

    /* Same adaptive detection threshold the legacy latch uses, so the
     * classifier's "effectively zero" test matches the real trip point. */
    float thr = RELAY_CHK_CURRENT_BASE_A + RELAY_CHK_CURRENT_K * d.throttle_pct;
    if (thr < RELAY_CHK_CURRENT_BASE_A)      thr = RELAY_CHK_CURRENT_BASE_A;
    else if (thr > RELAY_CHK_CURRENT_MAX_A)  thr = RELAY_CHK_CURRENT_MAX_A;
    d.current_threshold = thr;

    d.battery_voltage = Voltage_GetBus(INA226_CHANNEL_BATTERY);

    /* Independent confirming evidence (rule C).  This platform has NO
     * post-relay voltage / contact-feedback sensor (the battery INA sits
     * BEFORE the traction relay, see project_config.h), so we cannot supply
     * the independent evidence the audit requires for RELAY_OPEN_CONFIRMED.
     * Leave both false (unknown) and downgrade any CONFIRMED verdict to
     * SUSPECTED below — never claim confirmation without real evidence.    */
    d.post_relay_voltage_present = false;
    d.battery_consumption_rising = false;
    d.scale_invalid    = false;
    d.polarity_reversed = false;

    RelayDiagReason_t reason = Relay_ClassifyHealth(&d);
    if (reason == RELAY_OPEN_CONFIRMED) {
        /* No independent tension/contact evidence available on this HW. */
        reason = RELAY_OPEN_SUSPECTED;
    }
    d.diagnostic_reason = reason;

    s_relay_diag = d;
}

const RelayHealthDiag *Safety_GetRelayHealthDiag(void)
{
    return &s_relay_diag;
}

void Safety_CheckRelayHealth(void)
{
    /* Build the evidence-graded relay/current-sense diagnosis first, every
     * cycle, so the HMI/CAN can show the real cause (and the numbers behind
     * it) independently of the legacy pass/fail latch below.  Read-only. */
    Safety_UpdateRelayHealthDiag();


    /* ---- Relay Failure Detection — Design Constraints ----
     *
     * ⚠ IMPORTANT: Relay fault detection REQUIRES active motor demand.
     *
     * This function detects a relay-not-closing condition by comparing
     * throttle demand against measured motor current (INA226 ch 0–3).
     * It CANNOT detect a stuck-open relay when the vehicle is idle:
     *   - Throttle < 10% AND speed < 2 km/h → check is inactive
     *   - ABS/TCS actively limiting → demandPct = 0 → fault suppressed
     *   - Obstacle system zeroed demand → no current expected → no fault
     *   - Vehicle stalled (wheels blocked) → current ambiguous → suppressed
     *
     * Detection latency from demand onset:
     *   300 ms (settle) + 30 ms (debounce, 3 × 10 ms) = ~330 ms typical
     *   300 ms (settle) + 80 ms (brownout debounce)    = ~380 ms brownout
     *
     * This is by design: relay health cannot be assessed without
     * current flow, and current flow requires motor demand.  The
     * relay health check provides POST-HOC detection — not a
     * pre-condition gate.  The pre-condition gate for relay power
     * is Safety_IsPowerReady() (checked in Traction_Update).
     *
     * Arithmetic: all threshold checks use only multiply and compare
     * (no division in the hot path).  NaN/Inf from INA226 is rejected
     * per-channel.  Readings > 50 A are rejected as implausible.
     * Debounce counter (uint8_t) is capped at UINT8_MAX (255) to
     * prevent wrap-around — cannot overflow.                           */

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
     * In LIMP_HOME the clamped traction demand is lower (20%),
     * but Pedal_GetPercent still reflects the full filtered pedal
     * position — e.g. 25% filtered pedal (→5% LIMP demand) still
     * triggers relay validation when the driver clearly wants to
     * move.  This is intentional: the relay check uses the pedal
     * position, not the post-limit traction demand.                 */
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

        /* ---- Voltage correlation ----
         * Instead of fully disabling detection under low voltage (which
         * creates a blind zone that may mask real relay faults), we use
         * a two-tier approach:
         *   - Dead battery (< 12 V): skip entirely — readings meaningless
         *   - Brownout (12–18 V): relaxed params — still detects open relay
         *     (zero current) but won't false-positive from weak supply
         *   - Normal (≥ 18 V): full adaptive detection                     */
        float batt_v = Voltage_GetBus(INA226_CHANNEL_BATTERY);
        if (isnan(batt_v) || isinf(batt_v) ||
            batt_v < RELAY_CHK_VOLTAGE_DEAD_V) {
            /* Dead battery or invalid sensor — cannot determine relay
             * state from current readings.  Existing UV checks cover
             * the battery condition itself.                              */
            relay_chk_debounce = 0;
            return;
        }

        uint8_t brownout = (batt_v < RELAY_CHK_VOLTAGE_MIN_V) ? 1 : 0;

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
            /* Plausibility ceiling: reject corrupt I2C reads that return
             * finite but wildly implausible values.  Without this, a
             * single 999 A glitch would inflate sum and mask a real relay
             * fault.  Uses existing SENSOR_CURRENT_MAX_A (50 A).         */
            if (amps > SENSOR_CURRENT_MAX_A) continue;
            sum += amps;
            count++;
        }

        /* If no enabled sensors, cannot determine relay state — bail */
        if (count == 0) {
            relay_chk_debounce = 0;
            return;
        }

        /* Adaptive current threshold and debounce, mode-dependent:
         *
         * Normal mode (batt ≥ 18 V):
         *   threshold = BASE_A + K × throttle_pct
         *   One multiply + one add.  Scales with demand so low-throttle
         *   creep doesn't false-positive while high demand tightens
         *   detection.  debounce = DEBOUNCE_CYCLES (3 × 10 ms = 30 ms).
         *
         * Brownout mode (12 V ≤ batt < 18 V):
         *   threshold = BROWNOUT_CURRENT_A (1.0 A fixed floor — any
         *   measurable flow confirms relay is passing current).
         *   debounce = BROWNOUT_DEBOUNCE (8 × 10 ms = 80 ms — higher
         *   bar rejects transient noise from weak supply).               */
        float    eff_threshold;
        uint8_t  eff_debounce;
        if (brownout) {
            eff_threshold = RELAY_CHK_BROWNOUT_CURRENT_A;
            eff_debounce  = RELAY_CHK_BROWNOUT_DEBOUNCE;
        } else {
            eff_threshold = RELAY_CHK_CURRENT_BASE_A +
                            RELAY_CHK_CURRENT_K * throttle;
            /* Safety clamp: bound adaptive threshold to [BASE, MAX].
             * Prevents runaway values from corrupted throttle input
             * (e.g. ADC glitch returning >100% or negative).  O(1). */
            if (eff_threshold < RELAY_CHK_CURRENT_BASE_A) {
                eff_threshold = RELAY_CHK_CURRENT_BASE_A;
            } else if (eff_threshold > RELAY_CHK_CURRENT_MAX_A) {
                eff_threshold = RELAY_CHK_CURRENT_MAX_A;
            }
            eff_debounce  = RELAY_CHK_DEBOUNCE_CYCLES;
        }

        if (sum < eff_threshold) {
            /* Fault condition: demand present but insufficient motor current.
             * Reset recovery timer — fault condition persists.             */
            relay_chk_recovery_tick = 0;

            if (relay_chk_debounce < UINT8_MAX) relay_chk_debounce++;

            if (relay_chk_debounce >= eff_debounce) {
                /* Motion intent cross-check: only trigger relay fault if
                 * the traction controller is actually commanding output.
                 * This prevents false positives when the motor is
                 * intentionally stopped by ABS, TCS, or other limiters
                 * even though the pedal is pressed.
                 *
                 * Traction_GetState()->demandPct is the global traction
                 * demand (post-safety limits).  If 0, no motor output
                 * is expected regardless of relay state.
                 *
                 * Stall guard (OEM hardening): when all four wheels show
                 * zero speed despite high throttle (vehicle blocked by
                 * wall / obstacle), motor current can be unstable or
                 * current-limited by the BTS7960 driver.  To avoid a
                 * false relay-open fault in this scenario, require at
                 * least one wheel above a minimum speed threshold.
                 * Without wheel motion, the low current is ambiguous
                 * (could be stall, not relay failure).                    */
                const TractionState_t *ts = Traction_GetState();
                uint8_t any_wheel_moving = (avg_speed > RELAY_CHK_STALL_SPEED_KMH) ? 1 : 0;
                if (ts != (void *)0 && ts->demandPct > 0.0f &&
                    any_wheel_moving) {
                    /* Confirmed relay-open fault */
                    ServiceMode_SetFault(MODULE_RELAY_TRAC,
                                         MODULE_FAULT_ERROR);
                    Safety_SetError(SAFETY_ERROR_RELAY_OPEN);
                    Safety_SetState(SYS_STATE_DEGRADED);
                    Safety_SetDegradedLevel(DEGRADED_L3,
                                            DEGRADED_REASON_SENSOR_FAULT);
                    /* Record fault onset for minimum visibility hold.
                     * Intentionally set-once: if fault re-triggers before
                     * recovery completes, the original timestamp persists
                     * so the hold counts from first occurrence.  Cleared
                     * only on full recovery (line ~1887).                */
                    if (relay_chk_fault_set_tick == 0) {
                        relay_chk_fault_set_tick = now;
                    }
                }
            }
        } else {
            /* Current present — relay is healthy, reset debounce */
            relay_chk_debounce = 0;

            /* Recovery hysteresis with minimum fault visibility:
             *
             * Two conditions must BOTH be met before clearing the fault:
             *   a) FAULT_HOLD_MS has elapsed since the fault was first set
             *      (UX anti-flicker: ensures the user sees the error).
             *   b) RECOVERY_MS of sustained healthy current has elapsed
             *      (stability: prevents DEGRADED ↔ ACTIVE oscillation).
             *
             * The recovery timer starts when current first exceeds the
             * threshold and resets if current drops below it again.
             * Neither condition alone is sufficient.                      */
            if (safety_error == SAFETY_ERROR_RELAY_OPEN) {
                /* Check minimum visibility hold first */
                if (relay_chk_fault_set_tick != 0 &&
                    (now - relay_chk_fault_set_tick) <
                        RELAY_CHK_FAULT_HOLD_MS) {
                    /* Still within minimum visibility — do not start
                     * recovery timer yet.  Keep fault visible.           */
                    relay_chk_recovery_tick = 0;
                } else if (relay_chk_recovery_tick == 0) {
                    /* Hold expired (or no hold) — start recovery window */
                    relay_chk_recovery_tick = now;
                } else if ((now - relay_chk_recovery_tick) >=
                           RELAY_CHK_RECOVERY_MS) {
                    /* Sustained healthy current confirmed — clear fault */
                    ServiceMode_ClearFault(MODULE_RELAY_TRAC);
                    Safety_ClearError(SAFETY_ERROR_RELAY_OPEN);
                    relay_chk_recovery_tick  = 0;
                    relay_chk_fault_set_tick = 0;
                }
                /* else: still waiting for hysteresis window to expire */
            }
        }
    } else {
        /* Throttle and speed both below thresholds — reset check window.
         * Do NOT clear relay_chk_fault_set_tick here: the minimum
         * visibility hold must persist even if throttle drops briefly.
         * Recovery timer resets because we cannot observe current
         * without demand — it will restart when demand returns.       */
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

    /* Deterministic relay shutdown: Relay_PowerDown() performs an atomic
     * BSRR write that forces TRAC and DIR OFF in a single cycle,
     * independent of relay_seq_state or any previous relay state. This
     * guarantees both 24 V and 12 V power rails are cut even if the
     * sequencer was mid-transition or override was active.              */
    Relay_PowerDown();

    /* Transition to ERROR AFTER actuators and relays are inhibited. */
    system_state = SYS_STATE_ERROR;
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

/**
 * @brief  Deterministic pre-power-cut safe state, requested by the ESP32
 *         via CAN_ID_CMD_SYSTEM_SHUTDOWN (0x130) when the ignition key is
 *         turned OFF, before the external delay relay physically removes
 *         power.
 *
 * Reuses already-validated primitives — no duplicated shutdown logic:
 *   - Traction_EmergencyStop() : zeroes all traction PWM, drops EN.
 *   - Steering_Neutralize()    : zeroes steering PWM + eps_motor_effort,
 *                                disables steering H-bridge.
 *   - Relay_PowerDown()        : atomic BSRR → TRAC + DIR relays OFF.
 *
 * Idempotent: safe to call multiple times.  Non-blocking: no delays,
 * no loops, no waits.  Does not alter the relay sequencer state machine
 * beyond what Relay_PowerDown() already does.
 *
 * If the ESP32 never sends the frame, system behaviour is unchanged —
 * the hardware delay-relay cutoff still works exactly as before.
 *
 * State is set to SYS_STATE_SAFE (not ERROR) so this is treated as a
 * commanded safe stop, not an unrecoverable fault.
 */
void Safety_RequestShutdown(void)
{
    /* Cut all motor outputs first (deterministic, no current draw). */
    Traction_EmergencyStop();
    Steering_Neutralize();

    /* Atomic BSRR-based relay shutdown — independent of sequencer state. */
    Relay_PowerDown();

    /* Mark the system as in a commanded safe state.  No further motion
     * commands will be honoured (Safety_IsMotionAllowed gates on state).
     * If we are already in SAFE/ERROR, leave the existing state alone
     * to preserve any earlier diagnostic context.
     *
     * Threading note: this function is only invoked from the CAN dispatch
     * inside CAN_ProcessMessages(), which runs in main-loop context (see
     * main.c:304 / main.c:620) — never from an ISR.  All other writers
     * of system_state (Safety_SetState, Safety_EmergencyStop, Safety_Init)
     * also run in main-loop context, so a single-word write on Cortex-M4
     * is safe here without an explicit critical section.                 */
    if (system_state != SYS_STATE_SAFE && system_state != SYS_STATE_ERROR) {
        system_state = SYS_STATE_SAFE;
    }
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
 *   Byte 2:   zone level (0–4)
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
        /* Mirror the CAN-timeout policy: when an obstacle was being
         * tracked (ACTIVE or CONFIRMING), preserve obstacle_forward_blocked
         * and retain the last obstacle_scale, capped at
         * OBSTACLE_FAULT_SCALE.  Keep that forward-block latch for the
         * full sensor-fault duration so an emergency-zone obstacle that
         * triggers invalid readings does NOT silently reopen forward
         * motion after one cycle.  Reverse escape remains available via
         * Obstacle_IsForwardBlocked() when scale < 0.01 (emergency zone
         * scale = 0.0 is preserved).                                   */
        if (obstacle_state == OBS_STATE_ACTIVE ||
            obstacle_state == OBS_STATE_CONFIRMING ||
            obstacle_forward_blocked != 0U) {
            if (safety_status.obstacle_scale > OBSTACLE_FAULT_SCALE) {
                safety_status.obstacle_scale = OBSTACLE_FAULT_SCALE;
            }
            /* Keep forward_blocked if it was already set */
        } else {
            safety_status.obstacle_scale = OBSTACLE_FAULT_SCALE;
            obstacle_forward_blocked = 0;
        }
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
        /* Sensor path recovered (sensor_fault == 0) — rejoin normal flow. */
        obstacle_confirm_tick = 0;
        obstacle_clear_tick   = 0;
        if (target_scale < 1.0f) {
            /* Valid sensor but obstacle still in range: confirm before ACTIVE. */
            obstacle_state = OBS_STATE_CONFIRMING;
            obstacle_confirm_tick = now;
            safety_status.obstacle_scale = obstacle_preemptive_scale(target_scale);
        } else {
            obstacle_state = OBS_STATE_NORMAL;
            safety_status.obstacle_scale = 1.0f;
        }
        obstacle_forward_blocked = 0;
        ServiceMode_ClearFault(MODULE_OBSTACLE_DETECT);
        if (safety_error == SAFETY_ERROR_OBSTACLE) {
            Safety_ClearError(SAFETY_ERROR_OBSTACLE);
        }
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
