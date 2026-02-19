/**
  ****************************************************************************
  * @file    safety_system.h
  * @brief   Safety systems header - ABS, TCS, and fail-safe mechanisms
  ****************************************************************************
  */

#ifndef __SAFETY_SYSTEM_H
#define __SAFETY_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
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
    SAFETY_ERROR_CAN_BUSOFF = 13            /* FDCAN bus-off condition detected   */
} Safety_Error_t;

/* System operational state – the STM32 progresses through these states.
 * CAN commands are accepted in ACTIVE and DEGRADED (with limits).
 * Local pedal control is accepted in LIMP_HOME (walking speed only).
 *
 *  BOOT → STANDBY → ACTIVE ⇄ DEGRADED → SAFE → ERROR
 *                 ↘ LIMP_HOME ↗
 *
 * Transitions:
 *   BOOT→STANDBY     : peripheral init complete
 *   STANDBY→ACTIVE   : ESP32 heartbeat received, sensors plausible
 *   STANDBY→LIMP_HOME: boot validation passed but no CAN heartbeat
 *   ACTIVE→DEGRADED  : non-critical fault (sensor glitch, temp warning,
 *                       centering fail, single overcurrent)
 *   ACTIVE→LIMP_HOME : CAN timeout (communication loss is NOT a hazard)
 *   DEGRADED→ACTIVE  : fault cleared (recovery — "drive home" philosophy)
 *   DEGRADED→LIMP_HOME: CAN timeout while already degraded
 *   DEGRADED→SAFE    : critical fault while already degraded, or
 *                       persistent fault (consecutive error count ≥ 3)
 *   LIMP_HOME→ACTIVE : CAN heartbeat restored AND system healthy
 *   ACTIVE→SAFE      : overcurrent, overtemp, inverter fault, watchdog,
 *                       electrical hazard, sensor incoherence
 *   SAFE→ACTIVE      : fault cleared AND ESP32 heartbeat restored
 *   any→ERROR        : unrecoverable fault (watchdog, emergency stop)
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
    DEGRADED_REASON_PERSISTENT      = 8   /* Multiple faults accumulated   */
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
void Safety_CheckSensors(void);
void Safety_CheckEncoder(void);
void Safety_CheckBatteryVoltage(void);
void Safety_EmergencyStop(void);
void Safety_FailSafe(void);
void Safety_PowerDown(void);
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

/* Command validation – returns clamped/safe value */
float   Safety_ValidateThrottle(float requested_pct);
float   Safety_ValidateSteering(float requested_deg);
bool    Safety_ValidateModeChange(bool enable_4x4, bool tank_turn);

/* ---- Local obstacle state machine (STM32 is primary safety authority) ----
 * CAN obstacle frames from ESP32 are advisory only — never mandatory
 * for motion.  The STM32 runs a full autonomous obstacle safety module
 * with plausibility validation, stuck-sensor detection, speed-dependent
 * stopping distance, and temporal hysteresis.                              */
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
float            Obstacle_GetScale(void);
bool             Obstacle_IsForwardBlocked(void);
ObstacleState_t  Obstacle_GetState(void);

extern SafetyStatus_t safety_status;
extern Safety_Error_t safety_error;

#ifdef __cplusplus
}
#endif

#endif