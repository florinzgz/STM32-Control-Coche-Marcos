/**
  ****************************************************************************
  * @file    safety_system.h
  * @brief   Safety systems header - ABS, TCS, and fail-safe mechanisms
  ****************************************************************************
  */

#ifndef SAFETY_SYSTEM_H
#define SAFETY_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "battery_limits_store.h"
#include <stdbool.h>
#include <stdint.h>

/* Safety error codes */
typedef enum {
    SAFETY_ERROR_NONE = 0,
    SAFETY_ERROR_OVERCURRENT = 1,
    SAFETY_ERROR_OVERTEMP = 2,
    SAFETY_ERROR_CAN_TIMEOUT = 3,
    SAFETY_ERROR_SENSOR_FAULT = 4,
    SAFETY_ERROR_MOTOR_STALL = 5,
    SAFETY_ERROR_EMERGENCY_STOP = 6,
    SAFETY_ERROR_WATCHDOG = 7,
    SAFETY_ERROR_CENTERING = 8,             /* Steering centering failed */
    SAFETY_ERROR_BATTERY_UV_WARNING = 9,    /* Battery voltage < 20.0 V */
    SAFETY_ERROR_BATTERY_UV_CRITICAL = 10,  /* Battery voltage < 18.0 V */
    SAFETY_ERROR_I2C_FAILURE = 11,          /* I2C bus locked / unrecoverable */
    SAFETY_ERROR_OBSTACLE = 12,             /* Obstacle emergency or CAN timeout */
    SAFETY_ERROR_CAN_BUSOFF = 13,           /* FDCAN bus-off condition detected   */
    SAFETY_ERROR_BATTERY_OV_WARNING = 14,   /* Battery voltage > 30.0 V          */
    SAFETY_ERROR_BATTERY_OV_CRITICAL = 15,  /* Battery voltage > 35.0 V          */
    SAFETY_ERROR_RELAY_OPEN = 16,           /* Relay health: insufficient motor current */
} Safety_Error_t;

/* System operational state – the STM32 progresses through these states.
 * CAN commands are accepted in ACTIVE and DEGRADED (with limits).
 * Local pedal control is accepted in LIMP_HOME (walking speed only).
 *
 * ---- COMPLETE STATE TRANSITION DIAGRAM ----
 *
 *                    ┌─────────────────────────────────────────────────┐
 *                    │                                                 │
 *  BOOT ──▸ STANDBY ──┬──▸ ACTIVE ⇄ DEGRADED ──▸ SAFE ──▸ ERROR     │
 *                      │      │  ↑       │          │                 │
 *                      │      │  │       │          │                 │
 *                      │      ▼  │       ▼          ▼                 │
 *                      └──▸ LIMP_HOME ◂─────────────┘                 │
 *                                │                                    │
 *                                └───────▸ ACTIVE ◂───────────────────┘
 *
 * ⚠ STATE MACHINE INTEGRITY CONSTRAINTS:
 *
 *   1. BOOT → ACTIVE is IMPOSSIBLE.
 *      BOOT can only transition to STANDBY.  STANDBY requires ESP32
 *      heartbeat + steering calibrated + boot validation before ACTIVE.
 *      Safety_SetState() enforces this via the switch-case directed graph.
 *
 *   2. No double transitions per call.
 *      Safety_SetState() performs a single transition and returns.
 *      A second transition requires a new call from the main loop.
 *
 *   3. ACTIVE entry always calls Relay_PowerUp().
 *      The relay sequencer must complete (Safety_IsPowerReady() == true)
 *      before Traction_Update() permits motor output.  No subsystem
 *      assumes power without relay completion.
 *
 *   4. startup_inhibit prevents early torque.
 *      Active from boot until the pedal is held below 3% for 400 ms.
 *      Independent of state machine — applies in STANDBY, ACTIVE,
 *      DEGRADED, and LIMP_HOME.  SAFE already inhibits all torque.
 *
 *   5. ERROR is a terminal absorbing state.
 *      Once in ERROR, Safety_PowerDown() is called and only a physical
 *      reset can recover.  No automatic recovery path from ERROR.
 *
 * Transitions (authoritative — see Safety_SetState in safety_system.c):
 *   BOOT→STANDBY         : peripheral init complete (always succeeds)
 *   STANDBY→ACTIVE       : ESP32 heartbeat + calibrated + boot valid + no error
 *   STANDBY→LIMP_HOME    : boot validation passed but CAN heartbeat timeout
 *   ACTIVE→DEGRADED      : non-critical fault (sensor, temp warn, single OC)
 *   ACTIVE→LIMP_HOME     : CAN timeout (communication loss is NOT a hazard)
 *   ACTIVE→SAFE          : overcurrent, overtemp, electrical hazard, watchdog
 *   DEGRADED→ACTIVE      : fault cleared + 500 ms recovery debounce
 *   DEGRADED→LIMP_HOME   : CAN timeout while already degraded
 *   DEGRADED→SAFE        : critical fault while degraded, or ≥3 consecutive errors
 *   LIMP_HOME→ACTIVE     : CAN heartbeat restored + 500 ms debounce + calibrated
 *   SAFE→ACTIVE          : fault cleared AND ESP32 heartbeat restored
 *   SAFE→LIMP_HOME       : CAN timeout recovery (relays stay on)
 *   any→ERROR            : unrecoverable fault (watchdog, emergency stop)
 *
 * SAFE is reserved for real hardware danger — never triggered by
 * missing CAN frames.  Communication loss enters LIMP_HOME instead.
 */
typedef enum {
    SYS_STATE_BOOT      = 0,  /* Power-on, peripherals initialising        */
    SYS_STATE_STANDBY   = 1,  /* Ready, waiting for ESP32 heartbeat        */
    SYS_STATE_ACTIVE    = 2,  /* Normal operation – CAN commands accepted   */
    SYS_STATE_DEGRADED  = 3,  /* Limp / degraded – CAN commands accepted
                               * with reduced power/speed limits.           */
    SYS_STATE_SAFE      = 4,  /* Hardware danger – actuators inhibited.
                               * Only for overcurrent, inverter fault,
                               * watchdog, electrical hazard.               */
    SYS_STATE_ERROR     = 5,  /* Unrecoverable fault – power-down required  */
    SYS_STATE_LIMP_HOME = 6   /* CAN-loss degraded – minimal drivable mode.
                               * Local pedal control, walking speed cap,
                               * steering operational, no torque vectoring.
                               * Vehicle remains mobile without CAN/ESP32.  */
} SystemState_t;

/* Fault-flag bitmask transmitted in the heartbeat (byte 2) */
#define FAULT_CAN_TIMEOUT       (1U << 0)
#define FAULT_TEMP_OVERLOAD     (1U << 1)
#define FAULT_CURRENT_OVERLOAD  (1U << 2)
#define FAULT_ENCODER_ERROR     (1U << 3)
#define FAULT_WHEEL_SENSOR      (1U << 4)
#define FAULT_ABS_ACTIVE        (1U << 5)
#define FAULT_TCS_ACTIVE        (1U << 6)
#define FAULT_CENTERING         (1U << 7)

/* Extended fault flags (bits 8+).
 * These are NOT transmitted in the CAN heartbeat byte 2 (uint8_t)
 * but are tracked internally and reported via the safety error code
 * (STATUS_SAFETY 0x203 byte 2).  Document here for future CAN
 * contract extensions.                                              */
#define FAULT_BATT_UV_WARN      (1U << 8)   /* Battery < 20.0 V */
#define FAULT_BATT_UV_CRIT      (1U << 9)   /* Battery < 18.0 V */
#define FAULT_RELAY_OPEN        (1U << 10)  /* Relay health: insufficient motor current */

/* ABS/TCS status */
typedef struct {
    bool abs_active;
    bool tcs_active;
    uint8_t abs_wheel_mask;
    uint8_t tcs_wheel_mask;
    uint32_t abs_activation_count;
    uint32_t tcs_activation_count;
    /* Per-wheel torque scale factor (0.0–1.0).
     * 1.0 = full power, 0.0 = wheel fully inhibited.
     * Set by ABS_Update / TCS_Update per-wheel; consumed by
     * Traction_Update to modulate individual motor PWM.
     * Aligned with base firmware: abs_system.cpp modulateBrake()
     * and tcs_system.cpp modulatePower() per-wheel approach.        */
    float wheel_scale[4];
    /* Obstacle torque scale factor (0.0–1.0).
     * 1.0 = no obstacle reduction, 0.0 = full stop.
     * Set by Obstacle_Update() from CAN-received distance data.
     * Applied uniformly to all wheels in Traction_Update().          */
    float obstacle_scale;
} SafetyStatus_t;

/* Degraded-mode power / speed limits
 * Traced to base firmware limp_mode.cpp Limits namespace:
 *   DEGRADED → 70 % power, 80 % speed
 *   LIMP     → 40 % power, 50 % speed
 * The STM32 collapses these into a single DEGRADED state that applies
 * the LIMP (more conservative) limits so the vehicle can always
 * "drive home" safely.                                                  */
#define DEGRADED_POWER_LIMIT_PCT    40.0f   /* limp_mode.cpp POWER_LIMP */
#define DEGRADED_SPEED_LIMIT_PCT    50.0f   /* limp_mode.cpp SPEED_LIMP */

/* ---- Granular Degradation (Phase 12) ----
 * Internal degradation levels that refine the single SYS_STATE_DEGRADED
 * state into three severity tiers.  All levels still report
 * SYS_STATE_DEGRADED over CAN — no CAN contract changes.
 *
 * Traced to base firmware limp_mode.cpp multi-level approach:
 *   DEGRADED (L1) → minor fault, conservative limits
 *   LIMP     (L2) → thermal or moderate fault
 *   CRITICAL (L3) → persistent anomaly, most restrictive             */
typedef enum {
    DEGRADED_LEVEL_NONE = 0,   /* Not in degraded mode                 */
    DEGRADED_L1         = 1,   /* Minor sensor fault — least restrictive */
    DEGRADED_L2         = 2,   /* Thermal warning — moderate            */
    DEGRADED_L3         = 3    /* Persistent anomaly — most restrictive  */
} DegradedLevel_t;

/* ---- Per-wheel speed-sensor diagnostic reason codes ----
 * Surface WHY a wheel channel is (or is not) flagged, so the operator can
 * tell a genuine sensor fault apart from expected behaviour when a wheel is
 * turned by hand.  Only MISMATCH / IMPOSSIBLE_RATE / STUCK_* / NO_PULSE are
 * escalating (they increment the plausibility fault_count); MANUAL_MOVEMENT
 * and DISABLED_STATE are diagnostic-only and never raise WHEEL_SENSOR.      */
typedef enum {
    WHEEL_DIAG_OK              = 0,  /* Nominal — pulses coherent / vehicle stopped */
    WHEEL_DIAG_NO_PULSE        = 1,  /* Under power + others moving, this one silent */
    WHEEL_DIAG_STUCK_HIGH      = 2,  /* Silent under power, pin parked HIGH (on bolt) */
    WHEEL_DIAG_STUCK_LOW       = 3,  /* Silent under power, pin parked LOW (in gap)   */
    WHEEL_DIAG_MISMATCH        = 4,  /* Under power, wheel deviates from the others   */
    WHEEL_DIAG_IMPOSSIBLE_RATE = 5,  /* NaN/Inf/out-of-range speed value              */
    WHEEL_DIAG_MANUAL_MOVEMENT = 6,  /* Incoherent pulses but NOT under traction      */
    WHEEL_DIAG_DISABLED_STATE  = 7   /* Channel disabled in service mode              */
} WheelDiag_t;

/* Time a wheel mismatch must PERSIST while under traction before it is
 * latched as a WHEEL_SENSOR fault.  Prevents a single hand-spin or a
 * momentary difference while turning a wheel from forcing DEGRADED.        */
#define WHEEL_FAULT_DEBOUNCE_MS      1000U

/* Minimum |traction demand| (%) that counts as "vehicle under power".
 * Below this, ABS/TCS interventions and wheel-mismatch faults are
 * suppressed because any wheel motion is manual, not commanded.           */
#define WHEEL_INTERVENTION_MIN_DEMAND_PCT  3.0f

/* Reason for entering a degraded level (diagnostic / telemetry) */
typedef enum {
    DEGRADED_REASON_NONE            = 0,
    DEGRADED_REASON_SENSOR_FAULT    = 1,  /* Single sensor implausibility  */
    DEGRADED_REASON_THERMAL_WARN    = 2,  /* Temperature > warning thresh  */
    DEGRADED_REASON_OVERCURRENT     = 3,  /* Single overcurrent event      */
    DEGRADED_REASON_CENTERING_FAIL  = 4,  /* Steering centering failed     */
    DEGRADED_REASON_BATTERY_UV      = 5,  /* Battery undervoltage warning  */
    DEGRADED_REASON_DEMAND_ANOMALY  = 6,  /* Throttle demand anomaly       */
    DEGRADED_REASON_ENCODER_FAULT   = 7,  /* Steering encoder fault        */
    DEGRADED_REASON_PERSISTENT      = 8,  /* Multiple faults accumulated   */
    DEGRADED_REASON_BATTERY_OV      = 9   /* Battery overvoltage warning   */
} DegradedReason_t;

/* Per-level scaling factors (power, steering assist, traction cap).
 *
 *   Level | Power  | Steering | Traction cap | Description
 *   ------|--------|----------|--------------|----------------------------
 *   L1    | 70 %   | 85 %     | 80 %         | Minor: gentle limiting
 *   L2    | 50 %   | 70 %     | 60 %         | Thermal: moderate limits
 *   L3    | 40 %   | 60 %     | 50 %         | Persistent: most cautious
 *
 * DEGRADED_POWER_LIMIT_PCT (40 %) is preserved as the L3 floor to
 * maintain backward-compatible worst-case behaviour.                    */
#define DEGRADED_L1_POWER_PCT      70.0f
#define DEGRADED_L1_STEERING_PCT   85.0f
#define DEGRADED_L1_TRACTION_PCT   80.0f

#define DEGRADED_L2_POWER_PCT      50.0f
#define DEGRADED_L2_STEERING_PCT   70.0f
#define DEGRADED_L2_TRACTION_PCT   60.0f

#define DEGRADED_L3_POWER_PCT      40.0f   /* == DEGRADED_POWER_LIMIT_PCT */
#define DEGRADED_L3_STEERING_PCT   60.0f
#define DEGRADED_L3_TRACTION_PCT   50.0f   /* == DEGRADED_SPEED_LIMIT_PCT */

/* ---- LIMP_HOME mode parameters — minimal mobility without CAN ----
 * When CAN is lost, the vehicle enters LIMP_HOME instead of SAFE.
 * Communication loss is NOT a hazard.  The vehicle can still move
 * at walking speed using the local pedal sensor.
 *
 * Safety is maintained by:
 *   - 20% torque limit (strong clamp on pedal input)
 *   - 5 km/h speed cap (walking pace)
 *   - 10 %/s ramp rate (very slow acceleration)
 *   - No torque vectoring (Ackermann differential disabled)
 *   - Each motor operates independently
 *   - Limited regen braking
 *   - Obstacle scale still applied when CAN data is available        */
#define LIMP_HOME_TORQUE_LIMIT_FACTOR  0.20f   /* 20% max torque       */
#define LIMP_HOME_SPEED_LIMIT_KMH      5.0f    /* Walking speed cap    */
#define LIMP_HOME_RAMP_RATE_PCT_PER_S  10.0f   /* Very slow accel ramp */

/* Consecutive-error threshold before escalating DEGRADED → SAFE.
 * Traced to base firmware relays.cpp (consecutiveErrors >= 3).          */
#define CONSECUTIVE_ERROR_THRESHOLD  3

/* Function prototypes */
void Safety_Init(void);
void ABS_Update(void);
bool ABS_IsActive(void);
void ABS_Reset(void);
void TCS_Update(void);
bool TCS_IsActive(void);
void TCS_Reset(void);
void Safety_CheckCurrent(void);
void Safety_CheckTemperature(void);
void Safety_CheckCANTimeout(void);
void Safety_CheckSteeringTimeout(void);
bool Safety_IsSteeringTimedOut(void);
void Safety_CheckSensors(void);
void Safety_CheckEncoder(void);
void Safety_CheckBatteryVoltage(void);
void Safety_CheckBatteryOvervoltage(void);

/* ---- Runtime-configurable battery limits (FASE 3) ----
 * The low-voltage warning / derate / SAFE-cutoff / recovery thresholds and
 * an optional voltage filter become runtime variables seeded with the
 * historic compile-time values (Limit/Warning 20.0 V, Cutoff 18.0 V,
 * Recovery 18.5 V, Filter 0 ms).  Only the threshold VALUES used by
 * Safety_CheckBatteryVoltage() are replaced; the state machine itself is
 * unchanged, so with the defaults behaviour is byte-for-byte identical.
 * Ranges/defaults live in battery_limits_store.h (single source).        */
bool Safety_ValidateBatteryLimits(const BatteryLimits_t *b);
bool Safety_SetBatteryLimits(const BatteryLimits_t *b);
void Safety_GetBatteryLimits(BatteryLimits_t *out);

void Safety_CheckRelayHealth(void);
void Safety_EmergencyStop(void);
void Safety_FailSafe(void);
void Safety_PowerDown(void);
void Safety_RequestShutdown(void);   /* Deterministic pre-power-cut safe state.
                                       * Reuses Traction_EmergencyStop,
                                       * Steering_Neutralize and Relay_PowerDown.
                                       * Idempotent, non-blocking, no new logic. */
void Safety_SetError(Safety_Error_t error);
void Safety_ClearError(Safety_Error_t error);
Safety_Error_t Safety_GetError(void);
bool Safety_IsError(void);
void Safety_UpdateCANRxTime(void);

/* State machine */
SystemState_t Safety_GetState(void);
void          Safety_SetState(SystemState_t state);
bool          Safety_IsCommandAllowed(void);
bool          Safety_IsMotionAllowed(void);
bool          Safety_IsDegraded(void);
bool          Safety_IsLimpHome(void);
uint8_t       Safety_GetFaultFlags(void);

/* Per-wheel speed-sensor diagnostic reason (idx 0-3 = FL,FR,RL,RR).
 * Returns WHEEL_DIAG_OK for out-of-range idx.  Report-only.            */
WheelDiag_t   Safety_GetWheelDiag(uint8_t idx);

/* Degraded-mode throttle limit (returns multiplier 0.0–1.0) */
float         Safety_GetPowerLimitFactor(void);

/* Granular degradation (Phase 12) — per-level scaling factors */
float           Safety_GetSteeringLimitFactor(void);
float           Safety_GetTractionCapFactor(void);
DegradedLevel_t Safety_GetDegradedLevel(void);
DegradedReason_t Safety_GetDegradedReason(void);
uint32_t        Safety_GetDegradedTelemetryCount(void);
void            Safety_SetDegradedLevel(DegradedLevel_t level,
                                        DegradedReason_t reason);

/* Relay power sequencing */
void Relay_PowerUp(void);
void Relay_PowerDown(void);
void Relay_SequencerUpdate(void);

/* ---- Relay Override (Engineering / Diagnostic Mode) ----
 *
 * Allows manual relay GPIO control from the ESP32 engineering menu for
 * diagnostic purposes ONLY.  Override is strictly safety-gated:
 *   - System state must be STANDBY (no motion, relays normally off)
 *   - Throttle must be 0 %
 *   - Speed must be 0 km/h
 *   - No active safety errors
 *
 * If any condition fails, the override is immediately disabled and all
 * relay GPIOs are returned to their normal state (relay sequencer idle).
 *
 * Override does NOT:
 *   - Bypass Safety_IsPowerReady() — relay_seq_state stays IDLE
 *   - Affect normal relay sequencing logic
 *   - Work during ACTIVE, DEGRADED, LIMP_HOME, or ERROR states
 *   - Influence motor control (no torque possible without seq COMPLETE)
 *
 * CAN interface: SERVICE_CMD (0x110) action 0xE0, byte1 = relay mask:
 *   bit 0: override enable
 *   bit 1: reserved (always 0; PC10 not connected)
 *   bit 2: TRACTION relay
 *   bit 3: STEER_PWR relay (12 V steering actuator supply; legacy name
 *          "DIRECTION relay" — does NOT select drive direction)        */
void Safety_SetRelayOverride(bool enabled, uint8_t mask);
bool Safety_IsRelayOverrideActive(void);
void Safety_RelayOverrideUpdate(void);

/* Returns true only when the relay power-up sequence has completed
 * and both relays (TRACTION, STEER_PWR) are physically
 * closed.  Subsystems that depend on relay power being available
 * should gate on this instead of checking system state alone.
 *
 * During the ~70 ms window between entering ACTIVE and relay
 * sequence completion, this returns false.                          */
bool Safety_IsPowerReady(void);

/* Returns true while the relay sequencer is actively progressing
 * (between Relay_PowerUp() call and sequence completion).
 * Exposed for diagnostic / debug visibility only — no safety logic
 * should depend on this; use Safety_IsPowerReady() instead.         */
bool Relay_IsSequenceInProgress(void);

/* Relay telemetry byte — packed bitmask of relay COMMAND states.
 *
 *   Bit 0: reserved (always 0; PC10 is a free GPIO)
 *   Bit 1: TRACTION relay GPIO (PC11)  — 1 = commanded ON
 *   Bit 2: STEER_PWR relay GPIO (PC12) — 1 = commanded ON
 *          (12 V steering actuator supply; legacy name "DIRECTION relay")
 *   Bit 7: Sequence complete flag       — 1 = RELAY_SEQ_COMPLETE
 *   Bits 3-6: reserved (always 0)
 *
 * IMPORTANT: this reports the GPIO COMMAND state, not the physical
 * relay contact state.  There is no electrical feedback from the relay
 * coils — the STM32 does not have a relay-contact sense input.
 * Physical verification of relay closure is done indirectly via
 * INA226 current monitoring (Safety_CheckRelayHealth).
 *
 * Sent as heartbeat byte 5 (0x001, DLC 6).                           */
uint8_t Safety_GetRelayStatusByte(void);

/* Command validation – returns clamped/safe value */
float   Safety_ValidateThrottle(float requested_pct);
float   Safety_ValidateSteering(float requested_deg);
bool    Safety_ValidateModeChange(bool enable_4x4, bool tank_turn);

/* ---- Local obstacle state machine (STM32 is primary safety authority) ----
 * CAN obstacle frames from ESP32 are advisory only — never mandatory
 * for motion.  The STM32 runs a full autonomous obstacle safety module
 * with plausibility validation, stuck-sensor detection, speed-dependent
 * stopping distance, and temporal hysteresis.                              */

/* Obstacle distance zone thresholds (mm) — 5 zones */
#define OBSTACLE_CAUTION_MM_PUB     2000    /* 1500–2000 mm → scale = 0.85    */
#define OBSTACLE_ALERT_MM_PUB       4000    /* 2000–4000 mm → scale = 0.95    */

/* Child reaction detection — tighten obstacle factors when child
 * releases pedal rapidly (instinctive reaction to obstacle).               */
#define CHILD_REACTION_THRESHOLD_PUB   10.0f  /* Pedal drop threshold (%)     */
#define CHILD_REACTION_WINDOW_MS_PUB   500    /* Detection window (ms)        */

typedef enum {
    OBS_STATE_NO_SENSOR = 0,   /* No CAN data ever received — full motion    */
    OBS_STATE_NORMAL,          /* Sensor valid, no obstacle in range          */
    OBS_STATE_CONFIRMING,      /* Potential obstacle, temporal confirmation   */
    OBS_STATE_ACTIVE,          /* Confirmed obstacle, torque reduction active */
    OBS_STATE_CLEARING,        /* Obstacle receding, confirming clearance     */
    OBS_STATE_SENSOR_FAULT     /* Sensor data implausible — conservative mode */
} ObstacleState_t;

/* Obstacle safety (STM32 primary — CAN advisory from ESP32) */
void             Obstacle_Update(void);
void             Obstacle_ProcessCAN(const uint8_t *data, uint8_t len);
void             Obstacle_ProcessSafetyCAN(const uint8_t *data, uint8_t len);
float            Obstacle_GetScale(void);
bool             Obstacle_IsForwardBlocked(void);
ObstacleState_t  Obstacle_GetState(void);

extern SafetyStatus_t safety_status;
extern Safety_Error_t safety_error;

#ifdef __cplusplus
}
#endif

#endif
