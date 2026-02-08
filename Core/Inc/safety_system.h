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
    SAFETY_ERROR_WATCHDOG = 7
} Safety_Error_t;

/* System operational state – the STM32 progresses through these states
 * and only accepts actuator commands when in STATE_ACTIVE.
 *
 *  BOOT → STANDBY → ACTIVE ⇄ SAFE → ERROR
 *
 * Transitions:
 *   BOOT→STANDBY  : peripheral init complete
 *   STANDBY→ACTIVE: ESP32 heartbeat received, sensors plausible
 *   ACTIVE→SAFE   : fault detected (CAN timeout, overcurrent, overtemp…)
 *   SAFE→ACTIVE   : fault cleared AND ESP32 heartbeat restored
 *   any→ERROR     : unrecoverable fault (watchdog, emergency stop)
 */
typedef enum {
    SYS_STATE_BOOT    = 0,  /* Power-on, peripherals initialising        */
    SYS_STATE_STANDBY = 1,  /* Ready, waiting for ESP32 heartbeat        */
    SYS_STATE_ACTIVE  = 2,  /* Normal operation – commands accepted       */
    SYS_STATE_SAFE    = 3,  /* Fault detected – actuators inhibited       */
    SYS_STATE_ERROR   = 4   /* Unrecoverable fault – power-down required  */
} SystemState_t;

/* Fault-flag bitmask transmitted in the heartbeat (byte 2) */
#define FAULT_CAN_TIMEOUT       (1U << 0)
#define FAULT_TEMP_OVERLOAD     (1U << 1)
#define FAULT_CURRENT_OVERLOAD  (1U << 2)
#define FAULT_ENCODER_ERROR     (1U << 3)
#define FAULT_WHEEL_SENSOR      (1U << 4)
#define FAULT_ABS_ACTIVE        (1U << 5)
#define FAULT_TCS_ACTIVE        (1U << 6)

/* ABS/TCS status */
typedef struct {
    bool abs_active;
    bool tcs_active;
    uint8_t abs_wheel_mask;
    uint8_t tcs_wheel_mask;
    uint32_t abs_activation_count;
    uint32_t tcs_activation_count;
} SafetyStatus_t;

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
uint8_t       Safety_GetFaultFlags(void);

/* Relay power sequencing */
void Relay_PowerUp(void);
void Relay_PowerDown(void);

/* Command validation – returns clamped/safe value */
float   Safety_ValidateThrottle(float requested_pct);
float   Safety_ValidateSteering(float requested_deg);
bool    Safety_ValidateModeChange(bool enable_4x4, bool tank_turn);

extern SafetyStatus_t safety_status;
extern Safety_Error_t safety_error;

#ifdef __cplusplus
}
#endif

#endif