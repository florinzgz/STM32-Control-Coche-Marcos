/**
  ******************************************************************************
  * @file    safety_system.c
  * @brief   Safety systems implementation (ABS/TCS, protections)
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
  * - Monitoreo de batería
  * - Watchdog
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "safety_system.h"
#include "motor_control.h"
#include "can_handler.h"
#include <string.h>
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define ACCEL_RAMP_RATE_PCT_S   200.0f  /* 200%/s acceleration limit */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Safety_ABS_TCS_t abs_tcs_state;
ErrorLog_t error_log;
uint8_t power_limit_percent = POWER_LIMIT_NORMAL;
uint8_t warning_flags = WARNING_NONE;
uint8_t tcs_enabled = 1;

static FaultFlags_t fault_flags;

/* Private function prototypes -----------------------------------------------*/
static uint8_t Safety_CalculateSlip(uint16_t wheel_speed, uint16_t avg_speed);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief Initialize safety systems
  * @retval HAL status
  */
HAL_StatusTypeDef Safety_Init(void)
{
  /* Clear all structures */
  memset(&abs_tcs_state, 0, sizeof(Safety_ABS_TCS_t));
  memset(&error_log, 0, sizeof(ErrorLog_t));
  memset(&fault_flags, 0, sizeof(FaultFlags_t));
  
  /* Set default values */
  power_limit_percent = POWER_LIMIT_NORMAL;
  warning_flags = WARNING_NONE;
  tcs_enabled = 1;
  
  error_log.last_error = ERR_NONE;
  
  return HAL_OK;
}

/**
  * @brief Main safety task (call at 100 Hz)
  * @retval None
  */
void Safety_Task(void)
{
  /* Update ABS/TCS */
  ABS_Update(&wheel_speed);
  TCS_Update(&wheel_speed);
  
  /* Thermal protection */
  Thermal_Protection(&temperature);
  
  /* Current protection */
  Current_Protection(&current);
  
  /* Battery protection */
  Battery_Protection(&current);
}

/**
  * @brief Update ABS system
  * @param speeds: Pointer to wheel speeds structure
  * @retval None
  */
void ABS_Update(WheelSpeed_t *speeds)
{
  uint8_t i;
  uint16_t wheel_speeds[4] = {speeds->speed_FL, speeds->speed_FR, 
                              speeds->speed_RL, speeds->speed_RR};
  
  /* Check each wheel for lock condition */
  for (i = 0; i < 4; i++) {
    if (ABS_DetectLock(wheel_speeds[i], speeds->speed_avg)) {
      abs_tcs_state.abs_active[i] = 1;
      abs_tcs_state.slip_percent[i] = Safety_CalculateSlip(wheel_speeds[i], speeds->speed_avg);
      
      /* Reduce motor power on locked wheel */
      Motor_t *motor = NULL;
      switch (i) {
        case 0: motor = &motor_FL; break;
        case 1: motor = &motor_FR; break;
        case 2: motor = &motor_RL; break;
        case 3: motor = &motor_RR; break;
      }
      
      if (motor) {
        int8_t reduced_power = motor->power_pct * 70 / 100;
        Motor_SetPower(motor, reduced_power);
      }
      
      fault_flags.abs_active = 1;
    } else {
      abs_tcs_state.abs_active[i] = 0;
      abs_tcs_state.slip_percent[i] = 0;
    }
  }
  
  /* Find maximum slip */
  abs_tcs_state.slip_max = abs_tcs_state.slip_percent[0];
  for (i = 1; i < 4; i++) {
    if (abs_tcs_state.slip_percent[i] > abs_tcs_state.slip_max) {
      abs_tcs_state.slip_max = abs_tcs_state.slip_percent[i];
    }
  }
}

/**
  * @brief Update TCS system
  * @param speeds: Pointer to wheel speeds structure
  * @retval None
  */
void TCS_Update(WheelSpeed_t *speeds)
{
  uint8_t i;
  uint16_t wheel_speeds[4] = {speeds->speed_FL, speeds->speed_FR, 
                              speeds->speed_RL, speeds->speed_RR};
  
  if (!tcs_enabled) {
    for (i = 0; i < 4; i++) {
      abs_tcs_state.tcs_active[i] = 0;
    }
    return;
  }
  
  /* Check each wheel for spin condition */
  for (i = 0; i < 4; i++) {
    if (TCS_DetectSpin(wheel_speeds[i], speeds->speed_avg)) {
      abs_tcs_state.tcs_active[i] = 1;
      abs_tcs_state.slip_percent[i] = Safety_CalculateSlip(wheel_speeds[i], speeds->speed_avg);
      
      /* Reduce motor power on spinning wheel */
      Motor_t *motor = NULL;
      switch (i) {
        case 0: motor = &motor_FL; break;
        case 1: motor = &motor_FR; break;
        case 2: motor = &motor_RL; break;
        case 3: motor = &motor_RR; break;
      }
      
      if (motor) {
        int8_t reduced_power = motor->power_pct * 80 / 100;
        Motor_SetPower(motor, reduced_power);
      }
      
      fault_flags.tcs_active = 1;
    } else {
      abs_tcs_state.tcs_active[i] = 0;
    }
  }
}

/**
  * @brief Detect wheel lock (ABS)
  * @param wheel_speed: Wheel speed (mm/s)
  * @param avg_speed: Average vehicle speed (mm/s)
  * @retval 1 if lock detected, 0 otherwise
  */
uint8_t ABS_DetectLock(uint16_t wheel_speed, uint16_t avg_speed)
{
  uint8_t slip;
  
  /* Only check if vehicle is moving */
  if (avg_speed < 100) return 0;
  
  /* Calculate slip percentage */
  slip = Safety_CalculateSlip(wheel_speed, avg_speed);
  
  /* Check if slip exceeds ABS threshold */
  if (wheel_speed < avg_speed && slip >= ABS_THRESHOLD_PERCENT) {
    return 1;
  }
  
  return 0;
}

/**
  * @brief Detect wheel spin (TCS)
  * @param wheel_speed: Wheel speed (mm/s)
  * @param avg_speed: Average vehicle speed (mm/s)
  * @retval 1 if spin detected, 0 otherwise
  */
uint8_t TCS_DetectSpin(uint16_t wheel_speed, uint16_t avg_speed)
{
  uint8_t slip;
  
  /* Only check if vehicle is accelerating */
  if (avg_speed < 50) return 0;
  
  /* Calculate slip percentage */
  slip = Safety_CalculateSlip(wheel_speed, avg_speed);
  
  /* Check if slip exceeds TCS threshold */
  if (wheel_speed > avg_speed && slip >= TCS_THRESHOLD_PERCENT) {
    return 1;
  }
  
  return 0;
}

/**
  * @brief Enable/disable TCS
  * @param enable: 1 to enable, 0 to disable
  * @retval None
  */
void TCS_SetEnabled(uint8_t enable)
{
  tcs_enabled = enable;
}

/**
  * @brief Thermal protection (check temperatures and limit power)
  * @param temps: Pointer to temperatures structure
  * @retval None
  */
void Thermal_Protection(Temperature_t *temps)
{
  /* Check for critical temperature */
  if (temps->temp_max >= TEMP_CRITICAL_C) {
    power_limit_percent = POWER_LIMIT_CRITICAL;
    Safety_SetWarning(WARNING_TEMP_CRITICAL);
    fault_flags.temp_overload = 1;
    Safety_HandleError(ERR_TEMP_CRITICAL);
  }
  /* Check for warning temperature */
  else if (temps->temp_max >= TEMP_WARNING_C) {
    power_limit_percent = POWER_LIMIT_WARNING;
    Safety_SetWarning(WARNING_TEMP_HIGH);
  }
  /* Normal temperature */
  else {
    power_limit_percent = POWER_LIMIT_NORMAL;
    Safety_ClearWarning(WARNING_TEMP_HIGH);
    Safety_ClearWarning(WARNING_TEMP_CRITICAL);
    fault_flags.temp_overload = 0;
  }
}

/**
  * @brief Current protection (check currents and limit/disable)
  * @param currents: Pointer to currents structure
  * @retval None
  */
void Current_Protection(Current_t *currents)
{
  uint32_t current_time = HAL_GetTick();
  uint8_t i;
  float motor_currents[4] = {currents->current_FL, currents->current_FR,
                             currents->current_RL, currents->current_RR};
  Motor_t *motors[4] = {&motor_FL, &motor_FR, &motor_RL, &motor_RR};
  
  for (i = 0; i < 4; i++) {
    /* Check for critical overcurrent */
    if (motor_currents[i] >= CURRENT_CRITICAL_A) {
      /* Immediately disable motor */
      Motor_SetPower(motors[i], 0);
      Motor_Enable(motors[i], 0);
      fault_flags.current_overload = 1;
      Safety_HandleError(ERR_CURRENT_OVERLOAD);
    }
    /* Check for peak overcurrent */
    else if (motor_currents[i] >= CURRENT_PEAK_A) {
      /* Start timer if not already running */
      if (currents->overcurrent_time[i] == 0) {
        currents->overcurrent_time[i] = current_time;
      }
      
      /* Check if overcurrent duration exceeds 2 seconds */
      if ((current_time - currents->overcurrent_time[i]) > 2000) {
        /* Reduce motor power */
        int8_t reduced_power = motors[i]->power_pct * 50 / 100;
        Motor_SetPower(motors[i], reduced_power);
        fault_flags.current_overload = 1;
      }
    }
    /* Normal current */
    else {
      currents->overcurrent_time[i] = 0;
    }
  }
}

/**
  * @brief Battery protection (check battery voltage)
  * @param currents: Pointer to currents structure (contains voltage)
  * @retval None
  */
void Battery_Protection(Current_t *currents)
{
  /* Check for critical battery voltage */
  if (currents->voltage_BATT <= BATTERY_CRITICAL_V) {
    Safety_HandleError(ERR_BATTERY_CRITICAL);
    Mode_Safe();
  }
  /* Check for low battery voltage */
  else if (currents->voltage_BATT <= BATTERY_LOW_V) {
    Safety_SetWarning(WARNING_BATTERY_LOW);
  }
  /* Normal voltage */
  else {
    Safety_ClearWarning(WARNING_BATTERY_LOW);
  }
}

/**
  * @brief Apply thermal power limit to requested power
  * @param requested_power: Requested power (-100 to +100)
  * @retval Limited power
  */
int8_t Safety_ApplyPowerLimit(int8_t requested_power)
{
  int16_t limited_power;
  
  /* Apply percentage limit */
  limited_power = (int16_t)requested_power * power_limit_percent / 100;
  
  /* Clamp to valid range */
  if (limited_power > 100) limited_power = 100;
  if (limited_power < -100) limited_power = -100;
  
  return (int8_t)limited_power;
}

/**
  * @brief Apply rate limiter (acceleration ramp)
  * @param current: Current power
  * @param target: Target power
  * @param dt: Delta time (seconds)
  * @retval New power with rate limit applied
  */
int8_t Safety_RateLimit(int8_t current, int8_t target, float dt)
{
  float max_change = ACCEL_RAMP_RATE_PCT_S * dt;
  float diff = (float)(target - current);
  
  if (diff > max_change) {
    return current + (int8_t)max_change;
  } else if (diff < -max_change) {
    return current - (int8_t)max_change;
  } else {
    return target;
  }
}

/**
  * @brief Enter safe mode (emergency stop)
  * @retval None
  */
void Mode_Safe(void)
{
  /* Disable all motors */
  Traction_SetUniform(0);
  Traction_Enable(0);
  
  Motor_SetPower(&steering.motor, 0);
  Motor_Enable(&steering.motor, 0);
  
  /* Turn off power relays */
  HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RELAY_TRAC_GPIO_Port, RELAY_TRAC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RELAY_DIR_GPIO_Port, RELAY_DIR_Pin, GPIO_PIN_RESET);
  
  /* Update system state */
  system_state = SYSTEM_STATE_SAFE;
}

/**
  * @brief Enter normal mode (active control)
  * @retval None
  */
void Mode_Normal(void)
{
  /* Enable power relays */
  HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RELAY_TRAC_GPIO_Port, RELAY_TRAC_Pin, GPIO_PIN_SET);
  
  /* Enable motors */
  Traction_Enable(1);
  Motor_Enable(&steering.motor, 1);
  
  /* Update system state */
  system_state = SYSTEM_STATE_ACTIVE;
}

/**
  * @brief Check reset cause (watchdog, power-on, etc.)
  * @retval None
  */
void Safety_CheckResetCause(void)
{
  /* Check RCC reset flags */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
    /* Independent watchdog reset */
    error_log.watchdog_resets++;
    Safety_LogError(ERR_WATCHDOG_RESET, 0, error_log.watchdog_resets);
    CAN_SendError(ERR_WATCHDOG_RESET, 0, error_log.watchdog_resets);
  }
  
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
    /* Software reset */
  }
  
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
    /* Power-on reset */
  }
  
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
    /* External pin reset */
  }
  
  /* Clear all reset flags */
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

/**
  * @brief Handle error based on severity
  * @param error: Error code
  * @retval None
  */
void Safety_HandleError(ErrorCode_t error)
{
  /* Log error */
  Safety_LogError(error, 0, 0);
  
  /* Send error via CAN */
  CAN_SendError(error, 0, 0);
  
  /* Take action based on error severity */
  switch (error) {
    case ERR_TEMP_CRITICAL:
      error_log.temp_overloads++;
      /* Power limiting already handled in Thermal_Protection */
      break;
      
    case ERR_CURRENT_OVERLOAD:
      error_log.current_overloads++;
      /* Motor disabling already handled in Current_Protection */
      break;
      
    case ERR_BATTERY_CRITICAL:
      /* Enter safe mode */
      Mode_Safe();
      break;
      
    case ERR_TIMEOUT_CAN:
      error_log.can_timeouts++;
      /* Enter safe mode */
      Mode_Safe();
      break;
      
    case ERR_WATCHDOG_RESET:
      /* Already logged in Safety_CheckResetCause */
      break;
      
    default:
      /* Unknown error */
      Safety_SetWarning(WARNING_UNKNOWN_ERROR);
      break;
  }
}

/**
  * @brief Log error to error log
  * @param error: Error code
  * @param subsystem: Subsystem ID
  * @param data: Additional error data
  * @retval None
  */
void Safety_LogError(ErrorCode_t error, uint8_t subsystem, uint16_t data)
{
  error_log.last_error = error;
  error_log.error_count++;
}

/**
  * @brief Set warning flag
  * @param flag: Warning flag to set
  * @retval None
  */
void Safety_SetWarning(uint8_t flag)
{
  warning_flags |= flag;
}

/**
  * @brief Clear warning flag
  * @param flag: Warning flag to clear
  * @retval None
  */
void Safety_ClearWarning(uint8_t flag)
{
  warning_flags &= ~flag;
}

/**
  * @brief Get current fault flags
  * @retval FaultFlags_t structure
  */
FaultFlags_t Safety_GetFaultFlags(void)
{
  return fault_flags;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Calculate slip percentage
  * @param wheel_speed: Wheel speed (mm/s)
  * @param avg_speed: Average vehicle speed (mm/s)
  * @retval Slip percentage (0-100)
  */
static uint8_t Safety_CalculateSlip(uint16_t wheel_speed, uint16_t avg_speed)
{
  int32_t diff;
  uint8_t slip;
  
  if (avg_speed == 0) return 0;
  
  diff = abs((int32_t)wheel_speed - (int32_t)avg_speed);
  slip = (uint8_t)((diff * 100) / avg_speed);
  
  if (slip > 100) slip = 100;
  
  return slip;
}
