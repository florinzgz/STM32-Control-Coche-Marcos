/**
  ******************************************************************************
  * @file    sensor_manager.c
  * @brief   Sensor management implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sensor_manager.h"
#include "motor_control.h"
#include <string.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/
WheelSensor_t wheel_sensors[4];
TempSensor_t temp_sensors[TEMP_SENSOR_COUNT];
CurrentSensor_t current_sensors[CURRENT_SENSOR_COUNT];
PedalSensor_t pedal_sensor;
ShifterState_t shifter_state;

/* Private function prototypes -----------------------------------------------*/
static void TCA9548A_SelectChannel(uint8_t channel);
static float LowPassFilter(float current, float previous, float alpha);

/**
  * @brief  Initialize all sensors
  * @retval None
  */
void Sensors_Init(void)
{
  WheelSensors_Init();
  TempSensors_Init();
  CurrentSensors_Init();
  Pedal_Init();
  Shifter_Init();
}

/**
  * @brief  Initialize wheel speed sensors
  * @retval None
  */
void WheelSensors_Init(void)
{
  for (int i = 0; i < 4; i++)
  {
    wheel_sensors[i].pulse_count = 0;
    wheel_sensors[i].last_pulse_ms = 0;
    wheel_sensors[i].rpm = 0.0f;
    wheel_sensors[i].speed_kmh = 0.0f;
    wheel_sensors[i].pulses_per_rev = WHEEL_PULSES_PER_REV;
  }
}

/**
  * @brief  Initialize temperature sensors
  * @retval None
  */
void TempSensors_Init(void)
{
  for (int i = 0; i < TEMP_SENSOR_COUNT; i++)
  {
    temp_sensors[i].temperature_c = 25.0f;
    temp_sensors[i].sensor_present = false;
    memset(temp_sensors[i].rom_code, 0, 8);
  }

  /* TODO: Implement OneWire initialization and sensor detection */
}

/**
  * @brief  Initialize current sensors
  * @retval None
  */
void CurrentSensors_Init(void)
{
  for (int i = 0; i < CURRENT_SENSOR_COUNT; i++)
  {
    current_sensors[i].current_ma = 0.0f;
    current_sensors[i].voltage_v = 0.0f;
    current_sensors[i].power_mw = 0.0f;
    current_sensors[i].sensor_ready = false;
    current_sensors[i].i2c_address = INA226_I2C_ADDR_BASE + i;
  }

  /* TODO: Implement INA226 initialization via I2C */
  /* Configure each INA226 sensor through TCA9548A multiplexer */
}

/**
  * @brief  Initialize pedal sensor
  * @retval None
  */
void Pedal_Init(void)
{
  pedal_sensor.adc_raw = 0;
  pedal_sensor.position_percent = 0;
  pedal_sensor.adc_min = 0;
  pedal_sensor.adc_max = PEDAL_ADC_RESOLUTION - 1;
  pedal_sensor.calibrated = false;
}

/**
  * @brief  Initialize shifter inputs
  * @retval None
  */
void Shifter_Init(void)
{
  shifter_state.current_gear = GEAR_NEUTRAL;
  shifter_state.fwd_pressed = false;
  shifter_state.neu_pressed = false;
  shifter_state.rev_pressed = false;
}

/**
  * @brief  Wheel sensor pulse callback (called from ISR)
  * @param  wheel_index: Wheel index (0-3)
  * @retval None
  */
void WheelSensor_PulseCallback(uint8_t wheel_index)
{
  if (wheel_index >= 4)
    return;

  wheel_sensors[wheel_index].pulse_count++;
  wheel_sensors[wheel_index].last_pulse_ms = HAL_GetTick();
}

/**
  * @brief  Update wheel sensor calculations
  * @retval None
  */
void WheelSensors_Update(void)
{
  uint32_t now_ms = HAL_GetTick();

  for (int i = 0; i < 4; i++)
  {
    uint32_t time_since_pulse = now_ms - wheel_sensors[i].last_pulse_ms;

    /* If timeout, set RPM to zero */
    if (time_since_pulse > WHEEL_TIMEOUT_MS)
    {
      wheel_sensors[i].rpm = 0.0f;
      wheel_sensors[i].speed_kmh = 0.0f;
    }
    else
    {
      /* Calculate RPM based on pulse frequency */
      /* This is a simplified calculation - real implementation would use timing */
      wheel_sensors[i].rpm = WheelSensor_GetRPM(i);
      wheel_sensors[i].speed_kmh = Sensors_RPMtoKMH(wheel_sensors[i].rpm,
                                                     WHEEL_DIAMETER_MM);
    }

    /* Update motor current RPM for control */
    if (i < MOTOR_STEER)
    {
      motors[i].current_rpm = wheel_sensors[i].rpm;
    }
  }
}

/**
  * @brief  Get wheel RPM
  * @param  wheel_index: Wheel index
  * @retval RPM value
  */
float WheelSensor_GetRPM(uint8_t wheel_index)
{
  if (wheel_index >= 4)
    return 0.0f;

  return wheel_sensors[wheel_index].rpm;
}

/**
  * @brief  Get wheel speed in km/h
  * @param  wheel_index: Wheel index
  * @retval Speed in km/h
  */
float WheelSensor_GetSpeed(uint8_t wheel_index)
{
  if (wheel_index >= 4)
    return 0.0f;

  return wheel_sensors[wheel_index].speed_kmh;
}

/**
  * @brief  Start temperature conversion
  * @retval None
  */
void TempSensors_StartConversion(void)
{
  /* TODO: Implement DS18B20 OneWire conversion start */
}

/**
  * @brief  Read temperatures from sensors
  * @retval None
  */
void TempSensors_ReadTemperatures(void)
{
  /* TODO: Implement DS18B20 OneWire temperature read */
  /* Update motors temperature */
  for (int i = 0; i < 4 && i < TEMP_SENSOR_COUNT; i++)
  {
    motors[i].temperature_c = temp_sensors[i].temperature_c;
  }
}

/**
  * @brief  Get temperature from sensor
  * @param  sensor_index: Sensor index
  * @retval Temperature in Celsius
  */
float TempSensor_GetTemperature(uint8_t sensor_index)
{
  if (sensor_index >= TEMP_SENSOR_COUNT)
    return 0.0f;

  return temp_sensors[sensor_index].temperature_c;
}

/**
  * @brief  Read all current sensors
  * @retval None
  */
void CurrentSensors_ReadAll(void)
{
  /* TODO: Implement INA226 reading via I2C */
  /* Iterate through TCA9548A channels */
  for (int i = 0; i < CURRENT_SENSOR_COUNT; i++)
  {
    /* Select I2C multiplexer channel */
    TCA9548A_SelectChannel(i);

    /* Read INA226 registers */
    /* current_sensors[i].current_ma = ... */
    /* current_sensors[i].voltage_v = ... */
    /* current_sensors[i].power_mw = ... */
  }

  /* Update motor currents */
  for (int i = 0; i < 4 && i < CURRENT_SENSOR_COUNT; i++)
  {
    motors[i].current_amps = current_sensors[i].current_ma / 1000.0f;
  }
}

/**
  * @brief  Get current from sensor
  * @param  sensor_index: Sensor index
  * @retval Current in mA
  */
float CurrentSensor_GetCurrent(uint8_t sensor_index)
{
  if (sensor_index >= CURRENT_SENSOR_COUNT)
    return 0.0f;

  return current_sensors[sensor_index].current_ma;
}

/**
  * @brief  Get voltage from sensor
  * @param  sensor_index: Sensor index
  * @retval Voltage in V
  */
float CurrentSensor_GetVoltage(uint8_t sensor_index)
{
  if (sensor_index >= CURRENT_SENSOR_COUNT)
    return 0.0f;

  return current_sensors[sensor_index].voltage_v;
}

/**
  * @brief  Get power from sensor
  * @param  sensor_index: Sensor index
  * @retval Power in mW
  */
float CurrentSensor_GetPower(uint8_t sensor_index)
{
  if (sensor_index >= CURRENT_SENSOR_COUNT)
    return 0.0f;

  return current_sensors[sensor_index].power_mw;
}

/**
  * @brief  Update pedal position
  * @retval None
  */
void Pedal_Update(void)
{
  /* ADC value updated via callback */
  static float filtered_value = 0.0f;

  /* Apply low-pass filter to ADC reading */
  filtered_value = LowPassFilter((float)pedal_sensor.adc_raw,
                                  filtered_value,
                                  PEDAL_FILTER_ALPHA);

  /* Convert to percentage */
  if (pedal_sensor.calibrated)
  {
    int32_t range = pedal_sensor.adc_max - pedal_sensor.adc_min;
    if (range > 0)
    {
      int32_t position = (int32_t)filtered_value - pedal_sensor.adc_min;
      pedal_sensor.position_percent = (uint8_t)((position * 100) / range);

      /* Clamp to 0-100 */
      if (pedal_sensor.position_percent > 100)
        pedal_sensor.position_percent = 100;

      /* Apply deadzone */
      if (pedal_sensor.position_percent < PEDAL_DEADZONE_PERCENT)
        pedal_sensor.position_percent = 0;
    }
  }
  else
  {
    /* If not calibrated, use full ADC range */
    pedal_sensor.position_percent = (uint8_t)((filtered_value * 100) /
                                               PEDAL_ADC_RESOLUTION);
  }
}

/**
  * @brief  Get pedal position
  * @retval Position in percent (0-100)
  */
uint8_t Pedal_GetPosition(void)
{
  return pedal_sensor.position_percent;
}

/**
  * @brief  Calibrate pedal sensor
  * @retval None
  */
void Pedal_Calibrate(void)
{
  /* TODO: Implement calibration procedure */
  /* Set adc_min and adc_max based on user input */
  pedal_sensor.calibrated = true;
}

/**
  * @brief  ADC conversion complete callback
  * @retval None
  */
void Pedal_ADCCallback(void)
{
  pedal_sensor.adc_raw = HAL_ADC_GetValue(&hadc1);
}

/**
  * @brief  Update shifter state
  * @retval None
  */
void Shifter_Update(void)
{
  /* Read GPIO pins (active low) */
  shifter_state.fwd_pressed = (HAL_GPIO_ReadPin(FWD_GPIO_Port, FWD_Pin) == GPIO_PIN_RESET);
  shifter_state.neu_pressed = (HAL_GPIO_ReadPin(NEU_GPIO_Port, NEU_Pin) == GPIO_PIN_RESET);
  shifter_state.rev_pressed = (HAL_GPIO_ReadPin(REV_GPIO_Port, REV_Pin) == GPIO_PIN_RESET);

  /* Determine gear based on button states */
  if (shifter_state.fwd_pressed)
  {
    shifter_state.current_gear = GEAR_FORWARD;
  }
  else if (shifter_state.rev_pressed)
  {
    shifter_state.current_gear = GEAR_REVERSE;
  }
  else
  {
    shifter_state.current_gear = GEAR_NEUTRAL;
  }
}

/**
  * @brief  Get current gear
  * @retval Current gear
  */
Gear_t Shifter_GetGear(void)
{
  return shifter_state.current_gear;
}

/**
  * @brief  Periodic sensor update
  * @retval None
  */
void Sensors_PeriodicUpdate(void)
{
  WheelSensors_Update();
  Pedal_Update();
  Shifter_Update();

  static uint32_t last_current_read_ms = 0;
  static uint32_t last_temp_read_ms = 0;

  uint32_t now_ms = HAL_GetTick();

  /* Read current sensors at 10 Hz */
  if (now_ms - last_current_read_ms >= 100)
  {
    last_current_read_ms = now_ms;
    CurrentSensors_ReadAll();
  }

  /* Read temperature sensors at 1 Hz */
  if (now_ms - last_temp_read_ms >= 1000)
  {
    last_temp_read_ms = now_ms;
    TempSensors_StartConversion();
  }

  /* Read temperatures after conversion time */
  static uint32_t conversion_started_ms = 0;
  if (conversion_started_ms > 0 &&
      now_ms - conversion_started_ms >= TEMP_CONVERSION_TIME_MS)
  {
    conversion_started_ms = 0;
    TempSensors_ReadTemperatures();
  }
}

/**
  * @brief  Calculate RPM from pulse count and time
  * @param  pulse_count: Number of pulses
  * @param  time_delta_ms: Time period in ms
  * @param  pulses_per_rev: Pulses per revolution
  * @retval RPM value
  */
float Sensors_CalculateRPM(uint32_t pulse_count, uint32_t time_delta_ms,
                            uint16_t pulses_per_rev)
{
  if (time_delta_ms < 10 || pulses_per_rev == 0)  /* Minimum 10ms for stability */
    return 0.0f;

  float revolutions = (float)pulse_count / (float)pulses_per_rev;
  float minutes = (float)time_delta_ms / 60000.0f;

  return revolutions / minutes;
}

/**
  * @brief  Convert RPM to km/h
  * @param  rpm: RPM value
  * @param  wheel_diameter_mm: Wheel diameter in mm
  * @retval Speed in km/h
  */
float Sensors_RPMtoKMH(float rpm, float wheel_diameter_mm)
{
  /* Circumference in meters */
  float circumference_m = (wheel_diameter_mm / 1000.0f) * M_PI;

  /* Distance per minute in meters */
  float distance_per_min = rpm * circumference_m;

  /* Convert to km/h */
  float speed_kmh = (distance_per_min * 60.0f) / 1000.0f;

  return speed_kmh;
}

/**
  * @brief  Select I2C multiplexer channel
  * @param  channel: Channel number (0-7)
  * @retval None
  */
static void TCA9548A_SelectChannel(uint8_t channel)
{
  if (channel > 7)
    return;

  uint8_t data = (1 << channel);
  HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDR << 1, &data, 1, 100);
}

/**
  * @brief  Low-pass filter
  * @param  current: Current value
  * @param  previous: Previous filtered value
  * @param  alpha: Filter coefficient (0-1)
  * @retval Filtered value
  */
static float LowPassFilter(float current, float previous, float alpha)
{
  return alpha * current + (1.0f - alpha) * previous;
}

/**
  * @brief  ADC conversion complete callback
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    Pedal_ADCCallback();
  }
}
