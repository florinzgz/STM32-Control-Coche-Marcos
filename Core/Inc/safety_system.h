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
void Safety_EmergencyStop(void);
void Safety_FailSafe(void);
void Safety_PowerDown(void);
void Safety_SetError(Safety_Error_t error);
void Safety_ClearError(Safety_Error_t error);
Safety_Error_t Safety_GetError(void);
bool Safety_IsError(void);
void Safety_UpdateCANRxTime(void);

extern SafetyStatus_t safety_status;
extern Safety_Error_t safety_error;

#ifdef __cplusplus
}
#endif

#endif