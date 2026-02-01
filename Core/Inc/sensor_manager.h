/**
  ******************************************************************************
  * @file    sensor_manager.h
  * @brief   Sensor management for wheels, temperature, current, and pedal
  ******************************************************************************
  */

#ifndef __SENSOR_MANAGER_H
#define __SENSOR_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/* Wheel sensor data */
typedef struct {
    volatile uint32_t pulse_count;   /* Total pulse count */
    volatile uint32_t last_pulse_ms; /* Timestamp of last pulse */
    float rpm;                       /* Calculated RPM */
    float speed_kmh;                 /* Speed in km/h */
    uint16_t pulses_per_rev;         /* Pulses per wheel revolution */
} WheelSensor_t;

/* Temperature sensor data */
typedef struct {
    float temperature_c;             /* Temperature in Celsius */
    bool sensor_present;             /* Sensor detected flag */
    uint8_t rom_code[8];            /* DS18B20 ROM code */
} TempSensor_t;

/* Current sensor data (INA226) */
typedef struct {
    float current_ma;                /* Current in milliamps */
    float voltage_v;                 /* Voltage in volts */
    float power_mw;                  /* Power in milliwatts */
    bool sensor_ready;               /* Sensor initialized flag */
    uint8_t i2c_address;            /* I2C address */
} CurrentSensor_t;

/* Pedal sensor data */
typedef struct {
    uint16_t adc_raw;                /* Raw ADC value (0-4095) */
    uint8_t position_percent;        /* Pedal position (0-100%) */
    uint16_t adc_min;                /* Calibration: minimum ADC */
    uint16_t adc_max;                /* Calibration: maximum ADC */
    bool calibrated;                 /* Calibration status */
} PedalSensor_t;

/* Shifter state */
typedef struct {
    Gear_t current_gear;             /* Current gear selection */
    bool fwd_pressed;                /* Forward button state */
    bool neu_pressed;                /* Neutral button state */
    bool rev_pressed;                /* Reverse button state */
} ShifterState_t;

/* Exported constants --------------------------------------------------------*/

/* Wheel sensor configuration */
#define WHEEL_PULSES_PER_REV 20      /* Hall pulses per wheel revolution */
#define WHEEL_DIAMETER_MM 200.0f     /* Wheel diameter in mm */
#define WHEEL_TIMEOUT_MS 1000        /* Timeout for RPM calculation */

/* Temperature sensor configuration */
#define TEMP_SENSOR_COUNT 4          /* Number of DS18B20 sensors */
#define TEMP_CONVERSION_TIME_MS 750  /* DS18B20 conversion time */

/* Current sensor configuration (INA226) */
#define CURRENT_SENSOR_COUNT 6       /* Number of INA226 sensors */
#define INA226_I2C_ADDR_BASE 0x40    /* Base I2C address */
#define TCA9548A_ADDR 0x70           /* I2C multiplexer address */

/* Pedal configuration */
#define PEDAL_ADC_RESOLUTION 4096    /* 12-bit ADC */
#define PEDAL_DEADZONE_PERCENT 2     /* Deadzone at 0% */
#define PEDAL_FILTER_ALPHA 0.1f      /* Low-pass filter coefficient */

/* Exported variables --------------------------------------------------------*/
extern WheelSensor_t wheel_sensors[4];      /* FL, FR, RL, RR */
extern TempSensor_t temp_sensors[TEMP_SENSOR_COUNT];
extern CurrentSensor_t current_sensors[CURRENT_SENSOR_COUNT];
extern PedalSensor_t pedal_sensor;
extern ShifterState_t shifter_state;

/* Exported function prototypes ----------------------------------------------*/

/* Initialization */
void Sensors_Init(void);
void WheelSensors_Init(void);
void TempSensors_Init(void);
void CurrentSensors_Init(void);
void Pedal_Init(void);
void Shifter_Init(void);

/* Wheel sensors */
void WheelSensor_PulseCallback(uint8_t wheel_index);
void WheelSensors_Update(void);
float WheelSensor_GetRPM(uint8_t wheel_index);
float WheelSensor_GetSpeed(uint8_t wheel_index);

/* Temperature sensors (DS18B20 - OneWire) */
void TempSensors_StartConversion(void);
void TempSensors_ReadTemperatures(void);
float TempSensor_GetTemperature(uint8_t sensor_index);

/* Current sensors (INA226 - I2C) */
void CurrentSensors_ReadAll(void);
float CurrentSensor_GetCurrent(uint8_t sensor_index);
float CurrentSensor_GetVoltage(uint8_t sensor_index);
float CurrentSensor_GetPower(uint8_t sensor_index);

/* Pedal sensor */
void Pedal_Update(void);
uint8_t Pedal_GetPosition(void);
void Pedal_Calibrate(void);
void Pedal_ADCCallback(void);

/* Shifter */
void Shifter_Update(void);
Gear_t Shifter_GetGear(void);

/* Periodic update */
void Sensors_PeriodicUpdate(void);

/* Utility functions */
float Sensors_CalculateRPM(uint32_t pulse_count, uint32_t time_delta_ms, uint16_t pulses_per_rev);
float Sensors_RPMtoKMH(float rpm, float wheel_diameter_mm);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_MANAGER_H */
