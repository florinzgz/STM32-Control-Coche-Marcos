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
    SAFETY_ERROR_OBSTACLE = 12              /* Obstacle emergency or CAN timeout */
} Safety_Error_t;

/* System operational state – the STM32 progresses through these states.
 * Commands are accepted in ACTIVE and DEGRADED (with limits).
 *
 *  BOOT → STANDBY → ACTIVE ⇄ DEGRADED → SAFE → ERROR
 *
 * Transitions (traced to base firmware limp_mode.cpp):
 *   BOOT→STANDBY     : peripheral init complete
 *   STANDBY→ACTIVE   : ESP32 heartbeat received, sensors plausible
 *   ACTIVE→DEGRADED  : non-critical fault (sensor glitch, temp warning,
 *                       centering fail, single overcurrent)
 *   DEGRADED→ACTIVE  : fault cleared (recovery — "drive home" philosophy)
 *   DEGRADED→SAFE    : critical fault while already degraded, or
 *                       persistent fault (consecutive error count ≥ 3)
 *   ACTIVE→SAFE      : CAN timeout, emergency stop
 *   SAFE→ACTIVE      : fault cleared AND ESP32 heartbeat restored
 *   any→ERROR        : unrecoverable fault (watchdog, emergency stop)
 */
typedef enum {
    SYS_STATE_BOOT     = 0,  /* Power-on, peripherals initialising        */
    SYS_STATE_STANDBY  = 1,  /* Ready, waiting for ESP32 heartbeat        */
    SYS_STATE_ACTIVE   = 2,  /* Normal operation – commands accepted       */
    SYS_STATE_DEGRADED = 3,  /* Limp / degraded – commands accepted with
                              * reduced power/speed limits.  The vehicle
                              * can still "drive home".  Traced to base
                              * firmware limp_mode.cpp states DEGRADED /
                              * LIMP / CRITICAL.                           */
    SYS_STATE_SAFE     = 4,  /* Severe fault – actuators inhibited         */
    SYS_STATE_ERROR    = 5   /* Unrecoverable fault – power-down required  */
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
bool          Safety_IsDegraded(void);
uint8_t       Safety_GetFaultFlags(void);

/* Degraded-mode throttle limit (returns multiplier 0.0–1.0) */
float         Safety_GetPowerLimitFactor(void);

/* Relay power sequencing */
void Relay_PowerUp(void);
void Relay_PowerDown(void);

/* Command validation – returns clamped/safe value */
float   Safety_ValidateThrottle(float requested_pct);
float   Safety_ValidateSteering(float requested_deg);
bool    Safety_ValidateModeChange(bool enable_4x4, bool tank_turn);

/* Obstacle safety (CAN-received from ESP32) */
void    Obstacle_Update(void);
void    Obstacle_ProcessCAN(const uint8_t *data, uint8_t len);
float   Obstacle_GetScale(void);

extern SafetyStatus_t safety_status;
extern Safety_Error_t safety_error;

#ifdef __cplusplus
}
#endif

#endif