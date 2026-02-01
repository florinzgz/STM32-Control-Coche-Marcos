/**
  ******************************************************************************
  * @file    safety_system.c
  * @brief   Safety systems implementation (ABS, TCS, protections)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "safety_system.h"
#include "motor_control.h"
#include "sensor_manager.h"
#include "can_handler.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SafetySystem_t safety_system;

/* Private function prototypes -----------------------------------------------*/
static float CalculateWheelSlip(uint8_t wheel_index);

/**
  * @brief  Initialize safety system
  * @retval None
  */
void Safety_Init(void)
{
  memset(&safety_system, 0, sizeof(SafetySystem_t));

  safety_system.level = SAFETY_OK;

  ABS_Init();
  TCS_Init();
  TempProtection_Init();
  CurrentProtection_Init();
  Watchdog_Init();

  safety_system.emergency_stop_active = false;
}

/**
  * @brief  Enable watchdog
  * @retval None
  */
void Safety_EnableWatchdog(void)
{
  Watchdog_Init();
  safety_system.watchdog.enabled = true;
}

/**
  * @brief  Update safety system
  * @retval None
  */
void Safety_Update(void)
{
  /* Update protection flags */
  Safety_UpdateProtectionFlags();

  /* Update subsystems */
  if (safety_system.abs.enabled)
  {
    ABS_Update();
  }

  if (safety_system.tcs.enabled)
  {
    TCS_Update();
  }

  TempProtection_Update();
  CurrentProtection_Update();

  /* Check all protections */
  Safety_CheckAllProtections();
}

/**
  * @brief  Get safety level
  * @retval Safety level
  */
SafetyLevel_t Safety_GetLevel(void)
{
  return safety_system.level;
}

/**
  * @brief  Check if system is OK
  * @retval true if OK, false otherwise
  */
bool Safety_IsSystemOK(void)
{
  return (safety_system.level == SAFETY_OK || safety_system.level == SAFETY_WARNING);
}

/**
  * @brief  Initialize ABS
  * @retval None
  */
void ABS_Init(void)
{
  safety_system.abs.enabled = true;
  safety_system.abs.slip_threshold = ABS_SLIP_THRESHOLD;
  safety_system.abs.speed_threshold_kmh = ABS_MIN_SPEED_KMH;

  for (int i = 0; i < 4; i++)
  {
    safety_system.abs.active[i] = false;
  }
}

/**
  * @brief  Enable or disable ABS
  * @param  enable: true to enable, false to disable
  * @retval None
  */
void ABS_Enable(bool enable)
{
  safety_system.abs.enabled = enable;
}

/**
  * @brief  Update ABS
  * @retval None
  */
void ABS_Update(void)
{
  /* Check each wheel for lock-up */
  for (int i = 0; i < 4; i++)
  {
    bool lock_detected = ABS_CheckWheelLock(i);
    ABS_ControlWheel(i, lock_detected);
  }
}

/**
  * @brief  Check if wheel is locked
  * @param  wheel_index: Wheel index
  * @retval true if locked, false otherwise
  */
bool ABS_CheckWheelLock(uint8_t wheel_index)
{
  if (wheel_index >= 4)
    return false;

  float wheel_speed = WheelSensor_GetSpeed(wheel_index);

  /* Only activate above minimum speed */
  if (wheel_speed < safety_system.abs.speed_threshold_kmh)
    return false;

  /* Calculate average speed of other wheels */
  float avg_speed = 0.0f;
  int count = 0;
  for (int i = 0; i < 4; i++)
  {
    if (i != wheel_index)
    {
      avg_speed += WheelSensor_GetSpeed(i);
      count++;
    }
  }
  if (count > 0)
    avg_speed /= count;
  else
    return false;

  /* Avoid division by zero */
  if (avg_speed < 0.1f)
    return false;

  /* Check if this wheel is significantly slower (indicating lock) */
  float slip = ((avg_speed - wheel_speed) / avg_speed) * 100.0f;

  return (slip > safety_system.abs.slip_threshold);
}

/**
  * @brief  Control wheel for ABS
  * @param  wheel_index: Wheel index
  * @param  lock_detected: true if lock detected
  * @retval None
  */
void ABS_ControlWheel(uint8_t wheel_index, bool lock_detected)
{
  if (wheel_index >= 4)
    return;

  safety_system.abs.active[wheel_index] = lock_detected;

  if (lock_detected)
  {
    /* Reduce brake force (in our case, reduce motor braking) */
    /* This would be implemented in braking control */
  }
}

/**
  * @brief  Initialize TCS
  * @retval None
  */
void TCS_Init(void)
{
  safety_system.tcs.enabled = true;
  safety_system.tcs.slip_threshold = TCS_SLIP_THRESHOLD;
  safety_system.tcs.max_torque_reduction = TCS_MAX_TORQUE_REDUCTION;

  for (int i = 0; i < 4; i++)
  {
    safety_system.tcs.active[i] = false;
  }
}

/**
  * @brief  Enable or disable TCS
  * @param  enable: true to enable, false to disable
  * @retval None
  */
void TCS_Enable(bool enable)
{
  safety_system.tcs.enabled = enable;
}

/**
  * @brief  Update TCS
  * @retval None
  */
void TCS_Update(void)
{
  /* Check each wheel for slip */
  for (int i = 0; i < 4; i++)
  {
    bool slip_detected = TCS_CheckWheelSlip(i);
    safety_system.tcs.active[i] = slip_detected;

    if (slip_detected)
    {
      float slip_percent = CalculateWheelSlip(i);
      float torque_reduction = TCS_CalculateTorqueReduction(i, slip_percent);

      /* Apply torque reduction to motor */
      if (i < MOTOR_STEER)
      {
        uint16_t reduced_duty = (uint16_t)(motors[i].pwm_duty *
                                           (1.0f - torque_reduction / 100.0f));
        Motor_SetSpeed(i, reduced_duty);
      }
    }
  }
}

/**
  * @brief  Check if wheel is slipping
  * @param  wheel_index: Wheel index
  * @retval true if slipping, false otherwise
  */
bool TCS_CheckWheelSlip(uint8_t wheel_index)
{
  if (wheel_index >= 4)
    return false;

  float slip = CalculateWheelSlip(wheel_index);
  return (slip > safety_system.tcs.slip_threshold);
}

/**
  * @brief  Calculate torque reduction for wheel
  * @param  wheel_index: Wheel index
  * @param  slip_percent: Slip percentage
  * @retval Torque reduction percentage
  */
float TCS_CalculateTorqueReduction(uint8_t wheel_index, float slip_percent)
{
  /* Linear reduction based on slip */
  float reduction = (slip_percent / safety_system.tcs.slip_threshold) *
                     safety_system.tcs.max_torque_reduction;

  if (reduction > safety_system.tcs.max_torque_reduction)
    reduction = safety_system.tcs.max_torque_reduction;

  return reduction;
}

/**
  * @brief  Initialize temperature protection
  * @retval None
  */
void TempProtection_Init(void)
{
  safety_system.temp_protection.warning_temp_c = TEMP_WARNING_C;
  safety_system.temp_protection.critical_temp_c = TEMP_CRITICAL_C;
  safety_system.temp_protection.max_temp_c = 0.0f;
  safety_system.temp_protection.throttle_limited = false;
  safety_system.temp_protection.throttle_limit_percent = 100;
}

/**
  * @brief  Update temperature protection
  * @retval None
  */
void TempProtection_Update(void)
{
  /* Find maximum temperature */
  float max_temp = 0.0f;
  for (int i = 0; i < 4; i++)
  {
    if (motors[i].temperature_c > max_temp)
      max_temp = motors[i].temperature_c;
  }

  safety_system.temp_protection.max_temp_c = max_temp;

  /* Update throttle limit based on temperature */
  if (max_temp >= TEMP_SHUTDOWN_C)
  {
    safety_system.temp_protection.throttle_limit_percent = 0;
    safety_system.level = SAFETY_SHUTDOWN;
  }
  else if (max_temp >= safety_system.temp_protection.critical_temp_c)
  {
    safety_system.temp_protection.throttle_limit_percent = 30;
    safety_system.temp_protection.throttle_limited = true;
    if (safety_system.level < SAFETY_CRITICAL)
      safety_system.level = SAFETY_CRITICAL;
  }
  else if (max_temp >= safety_system.temp_protection.warning_temp_c)
  {
    safety_system.temp_protection.throttle_limit_percent = 70;
    safety_system.temp_protection.throttle_limited = true;
    if (safety_system.level < SAFETY_WARNING)
      safety_system.level = SAFETY_WARNING;
  }
  else
  {
    safety_system.temp_protection.throttle_limit_percent = 100;
    safety_system.temp_protection.throttle_limited = false;
  }
}

/**
  * @brief  Get throttle limit from temperature protection
  * @retval Throttle limit percentage
  */
uint8_t TempProtection_GetThrottleLimit(void)
{
  return safety_system.temp_protection.throttle_limit_percent;
}

/**
  * @brief  Check if system is overheated
  * @retval true if overheated, false otherwise
  */
bool TempProtection_IsOverheated(void)
{
  return safety_system.temp_protection.throttle_limited;
}

/**
  * @brief  Initialize current protection
  * @retval None
  */
void CurrentProtection_Init(void)
{
  safety_system.current_protection.warning_current_a = CURRENT_WARNING_A;
  safety_system.current_protection.critical_current_a = CURRENT_CRITICAL_A;
  safety_system.current_protection.max_current_a = 0.0f;
  safety_system.current_protection.overcurrent_count = 0;
}

/**
  * @brief  Update current protection
  * @retval None
  */
void CurrentProtection_Update(void)
{
  /* Find maximum current */
  float max_current = 0.0f;
  for (int i = 0; i < 4; i++)
  {
    if (motors[i].current_amps > max_current)
      max_current = motors[i].current_amps;
  }

  safety_system.current_protection.max_current_a = max_current;

  /* Check thresholds */
  if (max_current >= CURRENT_SHUTDOWN_A)
  {
    CurrentProtection_HandleOvercurrent();
    safety_system.level = SAFETY_SHUTDOWN;
  }
  else if (max_current >= safety_system.current_protection.critical_current_a)
  {
    safety_system.current_protection.overcurrent_count++;
    if (safety_system.level < SAFETY_CRITICAL)
      safety_system.level = SAFETY_CRITICAL;
  }
  else if (max_current >= safety_system.current_protection.warning_current_a)
  {
    if (safety_system.level < SAFETY_WARNING)
      safety_system.level = SAFETY_WARNING;
  }
}

/**
  * @brief  Check if overcurrent condition exists
  * @retval true if overcurrent, false otherwise
  */
bool CurrentProtection_IsOvercurrent(void)
{
  return (safety_system.current_protection.max_current_a >=
          safety_system.current_protection.warning_current_a);
}

/**
  * @brief  Handle overcurrent condition
  * @retval None
  */
void CurrentProtection_HandleOvercurrent(void)
{
  /* Emergency stop all motors */
  Motor_EmergencyStop();

  /* Log error */
  CAN_LogError(0x0001, 0x01, 2);
}

/**
  * @brief  Emergency stop
  * @retval None
  */
void Safety_EmergencyStop(void)
{
  Motor_EmergencyStop();
  safety_system.emergency_stop_active = true;
  safety_system.level = SAFETY_SHUTDOWN;
}

/**
  * @brief  Enter safe mode
  * @retval None
  */
void Safety_EnterSafeMode(void)
{
  /* Reduce power to minimum */
  for (int i = 0; i < MOTOR_STEER; i++)
  {
    Motor_SetSpeed(i, 0);
  }

  safety_system.level = SAFETY_CRITICAL;
}

/**
  * @brief  Clear errors
  * @retval None
  */
void Safety_ClearErrors(void)
{
  memset(&safety_system.flags, 0, sizeof(ProtectionFlags_t));
  safety_system.emergency_stop_active = false;
  safety_system.level = SAFETY_OK;
}

/**
  * @brief  Initialize watchdog
  * @retval None
  */
void Watchdog_Init(void)
{
  safety_system.watchdog.enabled = false;
  safety_system.watchdog.timeout_ms = IWDG_TIMEOUT_MS;
  safety_system.watchdog.last_refresh_ms = HAL_GetTick();

  /* TODO: Initialize hardware IWDG */
}

/**
  * @brief  Refresh watchdog
  * @retval None
  */
void Watchdog_Refresh(void)
{
  if (!safety_system.watchdog.enabled)
    return;

  safety_system.watchdog.last_refresh_ms = HAL_GetTick();

  /* TODO: Refresh hardware IWDG */
}

/**
  * @brief  Check watchdog timeout
  * @retval true if timeout, false otherwise
  */
bool Watchdog_CheckTimeout(void)
{
  if (!safety_system.watchdog.enabled)
    return false;

  uint32_t now_ms = HAL_GetTick();
  uint32_t elapsed = now_ms - safety_system.watchdog.last_refresh_ms;

  return (elapsed > safety_system.watchdog.timeout_ms);
}

/**
  * @brief  Get error flags
  * @retval Error flags as 16-bit value
  */
uint16_t Safety_GetErrorFlags(void)
{
  uint16_t flags = 0;

  flags |= (safety_system.flags.overcurrent_fl << 0);
  flags |= (safety_system.flags.overcurrent_fr << 1);
  flags |= (safety_system.flags.overcurrent_rl << 2);
  flags |= (safety_system.flags.overcurrent_rr << 3);
  flags |= (safety_system.flags.overtemp_fl << 4);
  flags |= (safety_system.flags.overtemp_fr << 5);
  flags |= (safety_system.flags.overtemp_rl << 6);
  flags |= (safety_system.flags.overtemp_rr << 7);
  flags |= (safety_system.flags.can_timeout << 8);
  flags |= (safety_system.flags.encoder_fault << 9);
  flags |= (safety_system.flags.pedal_fault << 10);

  return flags;
}

/**
  * @brief  Get diagnostic information
  * @param  buffer: Output buffer
  * @param  buffer_size: Buffer size
  * @retval None
  */
void Safety_GetDiagnosticInfo(char *buffer, uint16_t buffer_size)
{
  snprintf(buffer, buffer_size,
           "Safety Level: %d\n"
           "Max Temp: %.1f C\n"
           "Max Current: %.1f A\n"
           "ABS Active: %d\n"
           "TCS Active: %d\n",
           safety_system.level,
           safety_system.temp_protection.max_temp_c,
           safety_system.current_protection.max_current_a,
           safety_system.abs.active[0] || safety_system.abs.active[1] ||
           safety_system.abs.active[2] || safety_system.abs.active[3],
           safety_system.tcs.active[0] || safety_system.tcs.active[1] ||
           safety_system.tcs.active[2] || safety_system.tcs.active[3]);
}

/**
  * @brief  Check all protections
  * @retval true if all OK, false otherwise
  */
bool Safety_CheckAllProtections(void)
{
  bool all_ok = true;

  /* Reset safety level */
  safety_system.level = SAFETY_OK;

  /* Check temperature */
  if (TempProtection_IsOverheated())
  {
    all_ok = false;
  }

  /* Check current */
  if (CurrentProtection_IsOvercurrent())
  {
    all_ok = false;
  }

  /* Check CAN timeout */
  if (safety_system.flags.can_timeout)
  {
    all_ok = false;
    if (safety_system.level < SAFETY_WARNING)
      safety_system.level = SAFETY_WARNING;
  }

  return all_ok;
}

/**
  * @brief  Update protection flags
  * @retval None
  */
void Safety_UpdateProtectionFlags(void)
{
  /* Update overcurrent flags */
  for (int i = 0; i < 4; i++)
  {
    bool overcurrent = (motors[i].current_amps > CURRENT_WARNING_A);
    bool overtemp = (motors[i].temperature_c > TEMP_WARNING_C);

    switch (i)
    {
      case 0:
        safety_system.flags.overcurrent_fl = overcurrent;
        safety_system.flags.overtemp_fl = overtemp;
        break;
      case 1:
        safety_system.flags.overcurrent_fr = overcurrent;
        safety_system.flags.overtemp_fr = overtemp;
        break;
      case 2:
        safety_system.flags.overcurrent_rl = overcurrent;
        safety_system.flags.overtemp_rl = overtemp;
        break;
      case 3:
        safety_system.flags.overcurrent_rr = overcurrent;
        safety_system.flags.overtemp_rr = overtemp;
        break;
    }
  }

  /* Update CAN timeout flag */
  safety_system.flags.can_timeout = !CAN_IsESP32Alive();
}

/**
  * @brief  Calculate wheel slip
  * @param  wheel_index: Wheel index
  * @retval Slip percentage
  */
static float CalculateWheelSlip(uint8_t wheel_index)
{
  if (wheel_index >= 4)
    return 0.0f;

  float wheel_speed = WheelSensor_GetSpeed(wheel_index);

  /* Calculate average speed of other wheels */
  float avg_speed = 0.0f;
  int count = 0;
  for (int i = 0; i < 4; i++)
  {
    if (i != wheel_index)
    {
      avg_speed += WheelSensor_GetSpeed(i);
      count++;
    }
  }
  if (count > 0)
    avg_speed /= count;
  else
    return 0.0f;

  /* Avoid division by zero */
  if (avg_speed < 0.1f)
    return 0.0f;

  /* Calculate slip percentage */
  float slip = fabsf((wheel_speed - avg_speed) / avg_speed) * 100.0f;

  return slip;
}
