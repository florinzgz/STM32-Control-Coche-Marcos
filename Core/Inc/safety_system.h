/**
  ******************************************************************************
  * @file    safety_system.h
  * @brief   Safety systems interface (ABS/TCS, protections)
  * @author  florinzgz
  * @date    2026-02-01
  ******************************************************************************
  * @attention
  *
  * Sistemas de seguridad:
  * - ABS (Anti-lock Braking System)
  * - TCS (Traction Control System)
  * - Protecciones térmicas
  * - Protecciones de corriente
  * - Watchdog
  *
  ******************************************************************************
  */

#ifndef __SAFETY_SYSTEM_H
#define __SAFETY_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sensor_manager.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief ABS/TCS state structure
 */
typedef struct {
    uint8_t abs_active[4];      /* ABS active flags per wheel */
    uint8_t tcs_active[4];      /* TCS active flags per wheel */
    uint8_t slip_percent[4];    /* Slip percentage per wheel */
    uint8_t slip_max;           /* Maximum slip detected */
} Safety_ABS_TCS_t;

/**
 * @brief Fault flags structure
 */
typedef struct {
    uint8_t can_timeout;
    uint8_t temp_overload;
    uint8_t current_overload;
    uint8_t encoder_error;
    uint8_t wheel_sensor_error;
    uint8_t abs_active;
    uint8_t tcs_active;
} FaultFlags_t;

/**
 * @brief Error log structure
 */
typedef struct {
    ErrorCode_t last_error;
    uint32_t error_count;
    uint32_t watchdog_resets;
    uint32_t can_timeouts;
    uint32_t temp_overloads;
    uint32_t current_overloads;
} ErrorLog_t;

/* Exported constants --------------------------------------------------------*/

/* Power limits */
#define POWER_LIMIT_NORMAL      100     /* 100% normal operation */
#define POWER_LIMIT_WARNING     70      /* 70% when temp warning */
#define POWER_LIMIT_CRITICAL    30      /* 30% when temp critical */

/* Warning flags */
#define WARNING_NONE            0x00
#define WARNING_TEMP_HIGH       0x01
#define WARNING_TEMP_CRITICAL   0x02
#define WARNING_BATTERY_LOW     0x04
#define WARNING_UNKNOWN_ERROR   0x80

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Initialize safety systems
 * @retval HAL status
 */
HAL_StatusTypeDef Safety_Init(void);

/**
 * @brief Main safety task (call at 100 Hz)
 * @retval None
 */
void Safety_Task(void);

/**
 * @brief Update ABS system
 * @param speeds: Pointer to wheel speeds structure
 * @retval None
 */
void ABS_Update(WheelSpeed_t *speeds);

/**
 * @brief Update TCS system
 * @param speeds: Pointer to wheel speeds structure
 * @retval None
 */
void TCS_Update(WheelSpeed_t *speeds);

/**
 * @brief Detect wheel lock (ABS)
 * @param wheel_speed: Wheel speed (mm/s)
 * @param avg_speed: Average vehicle speed (mm/s)
 * @retval 1 if lock detected, 0 otherwise
 */
uint8_t ABS_DetectLock(uint16_t wheel_speed, uint16_t avg_speed);

/**
 * @brief Detect wheel spin (TCS)
 * @param wheel_speed: Wheel speed (mm/s)
 * @param avg_speed: Average vehicle speed (mm/s)
 * @retval 1 if spin detected, 0 otherwise
 */
uint8_t TCS_DetectSpin(uint16_t wheel_speed, uint16_t avg_speed);

/**
 * @brief Enable/disable TCS
 * @param enable: 1 to enable, 0 to disable
 * @retval None
 */
void TCS_SetEnabled(uint8_t enable);

/**
 * @brief Thermal protection (check temperatures and limit power)
 * @param temps: Pointer to temperatures structure
 * @retval None
 */
void Thermal_Protection(Temperature_t *temps);

/**
 * @brief Current protection (check currents and limit/disable)
 * @param currents: Pointer to currents structure
 * @retval None
 */
void Current_Protection(Current_t *currents);

/**
 * @brief Battery protection (check battery voltage)
 * @param currents: Pointer to currents structure (contains voltage)
 * @retval None
 */
void Battery_Protection(Current_t *currents);

/**
 * @brief Apply thermal power limit to requested power
 * @param requested_power: Requested power (-100 to +100)
 * @retval Limited power
 */
int8_t Safety_ApplyPowerLimit(int8_t requested_power);

/**
 * @brief Apply rate limiter (acceleration ramp)
 * @param current: Current power
 * @param target: Target power
 * @param dt: Delta time (seconds)
 * @retval New power with rate limit applied
 */
int8_t Safety_RateLimit(int8_t current, int8_t target, float dt);

/**
 * @brief Enter safe mode (emergency stop)
 * @retval None
 */
void Mode_Safe(void);

/**
 * @brief Enter normal mode (active control)
 * @retval None
 */
void Mode_Normal(void);

/**
 * @brief Check reset cause (watchdog, power-on, etc.)
 * @retval None
 */
void Safety_CheckResetCause(void);

/**
 * @brief Handle error based on severity
 * @param error: Error code
 * @retval None
 */
void Safety_HandleError(ErrorCode_t error);

/**
 * @brief Log error to error log
 * @param error: Error code
 * @param subsystem: Subsystem ID
 * @param data: Additional error data
 * @retval None
 */
void Safety_LogError(ErrorCode_t error, uint8_t subsystem, uint16_t data);

/**
 * @brief Set warning flag
 * @param flag: Warning flag to set
 * @retval None
 */
void Safety_SetWarning(uint8_t flag);

/**
 * @brief Clear warning flag
 * @param flag: Warning flag to clear
 * @retval None
 */
void Safety_ClearWarning(uint8_t flag);

/**
 * @brief Get current fault flags
 * @retval FaultFlags_t structure
 */
FaultFlags_t Safety_GetFaultFlags(void);

/* Exported variables --------------------------------------------------------*/

extern Safety_ABS_TCS_t abs_tcs_state;
extern ErrorLog_t error_log;
extern uint8_t power_limit_percent;
extern uint8_t warning_flags;
extern uint8_t tcs_enabled;

#ifdef __cplusplus
}
#endif

#endif /* __SAFETY_SYSTEM_H */
