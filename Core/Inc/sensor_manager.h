/**
  ******************************************************************************
  * @file    sensor_manager.h
  * @brief   Sensor management interface
  * @author  florinzgz
  * @date    2026-02-01
  ******************************************************************************
  * @attention
  *
  * Gestión de sensores:
  * - 5 sensores de rueda (4 ruedas + 1 encoder dirección)
  * - 5 sensores de temperatura DS18B20
  * - 6 sensores de corriente INA226
  *
  ******************************************************************************
  */

#ifndef __SENSOR_MANAGER_H
#define __SENSOR_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Wheel speed structure
 */
typedef struct {
    uint16_t speed_FL;      /* Front left speed (mm/s) */
    uint16_t speed_FR;      /* Front right speed (mm/s) */
    uint16_t speed_RL;      /* Rear left speed (mm/s) */
    uint16_t speed_RR;      /* Rear right speed (mm/s) */
    uint16_t speed_avg;     /* Average speed (mm/s) */
    uint32_t pulse_count[4]; /* Pulse counters */
    uint32_t last_pulse_time[4]; /* Last pulse timestamp (ms) */
} WheelSpeed_t;

/**
 * @brief Temperature readings structure
 */
typedef struct {
    float temp_FL;          /* Motor FL temperature (°C) */
    float temp_FR;          /* Motor FR temperature (°C) */
    float temp_RL;          /* Motor RL temperature (°C) */
    float temp_RR;          /* Motor RR temperature (°C) */
    float temp_ambient;     /* Ambient temperature (°C) */
    float temp_max;         /* Maximum temperature (°C) */
} Temperature_t;

/**
 * @brief Current readings structure
 */
typedef struct {
    float current_FL;       /* Motor FL current (A) */
    float current_FR;       /* Motor FR current (A) */
    float current_RL;       /* Motor RL current (A) */
    float current_RR;       /* Motor RR current (A) */
    float current_STEER;    /* Steering motor current (A) */
    float current_BATT;     /* Battery current (A) */
    float voltage_BATT;     /* Battery voltage (V) */
    uint32_t overcurrent_time[4]; /* Overcurrent timestamp per motor */
} Current_t;

/**
 * @brief Pedal reading structure
 */
typedef struct {
    uint16_t adc_raw;       /* Raw ADC value (0-4095) */
    float voltage;          /* Voltage (0-3.3V) */
    uint8_t throttle_pct;   /* Throttle percentage (0-100%) */
} Pedal_t;

/* Exported constants --------------------------------------------------------*/

/* DS18B20 ROM addresses (64-bit, to be configured during calibration) */
#define DS18B20_ROM_FL      0x28XXXXXXXXXX01ULL
#define DS18B20_ROM_FR      0x28XXXXXXXXXX02ULL
#define DS18B20_ROM_RL      0x28XXXXXXXXXX03ULL
#define DS18B20_ROM_RR      0x28XXXXXXXXXX04ULL
#define DS18B20_ROM_AMB     0x28XXXXXXXXXX05ULL

/* INA226 Shunt Resistor */
#define INA226_SHUNT_OHM    0.001f      /* 1 mΩ shunt resistor */

/* Wheel circumference (mm) - adjust for actual wheel size */
#define WHEEL_CIRCUMFERENCE_MM  300.0f  /* Example: 300mm diameter wheel */
#define WHEEL_PPR               2       /* Pulses per revolution (adjust) */

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Initialize all sensors
 * @retval HAL status
 */
HAL_StatusTypeDef Sensors_Init(void);

/**
 * @brief Read all wheel speeds
 * @param speeds: Pointer to WheelSpeed_t structure
 * @retval None
 */
void Sensors_ReadWheelSpeeds(WheelSpeed_t *speeds);

/**
 * @brief Read all temperatures (DS18B20)
 * @param temps: Pointer to Temperature_t structure
 * @retval HAL status
 */
HAL_StatusTypeDef Sensors_ReadTemperatures(Temperature_t *temps);

/**
 * @brief Read all currents (INA226)
 * @param currents: Pointer to Current_t structure
 * @retval HAL status
 */
HAL_StatusTypeDef Sensors_ReadCurrents(Current_t *currents);

/**
 * @brief Read pedal analog value
 * @param pedal: Pointer to Pedal_t structure
 * @retval HAL status
 */
HAL_StatusTypeDef Sensors_ReadPedal(Pedal_t *pedal);

/**
 * @brief Read shifter position
 * @retval ShifterState_t (FORWARD, NEUTRAL, REVERSE, ERROR)
 */
ShifterState_t Sensors_ReadShifter(void);

/**
 * @brief Wheel sensor interrupt handler (EXTI)
 * @param wheel_index: Wheel index (0=FL, 1=FR, 2=RL, 3=RR)
 * @retval None
 */
void Sensors_WheelPulseCallback(uint8_t wheel_index);

/**
 * @brief Calculate wheel speed from pulse frequency
 * @param pulse_count: Number of pulses
 * @param time_ms: Time period (ms)
 * @retval Speed in mm/s
 */
uint16_t Sensors_CalculateWheelSpeed(uint32_t pulse_count, uint32_t time_ms);

/* OneWire functions for DS18B20 */

/**
 * @brief OneWire reset pulse
 * @retval 1 if device present, 0 if no device
 */
uint8_t OneWire_Reset(void);

/**
 * @brief Write byte to OneWire bus
 * @param byte: Byte to write
 * @retval None
 */
void OneWire_WriteByte(uint8_t byte);

/**
 * @brief Read byte from OneWire bus
 * @retval Read byte
 */
uint8_t OneWire_ReadByte(void);

/**
 * @brief Read temperature from DS18B20 with ROM addressing
 * @param rom_address: 64-bit ROM address
 * @retval Temperature in °C
 */
float DS18B20_ReadTemperature(uint64_t rom_address);

/* I2C functions for INA226 */

/**
 * @brief Select TCA9548A multiplexer channel
 * @param channel: Channel number (0-7)
 * @retval HAL status
 */
HAL_StatusTypeDef TCA9548A_SelectChannel(uint8_t channel);

/**
 * @brief Initialize INA226 sensor
 * @param i2c_addr: I2C address (7-bit)
 * @retval HAL status
 */
HAL_StatusTypeDef INA226_Init(uint8_t i2c_addr);

/**
 * @brief Read current from INA226
 * @param i2c_addr: I2C address (7-bit)
 * @retval Current in Amperes
 */
float INA226_ReadCurrent(uint8_t i2c_addr);

/**
 * @brief Read voltage from INA226
 * @param i2c_addr: I2C address (7-bit)
 * @retval Voltage in Volts
 */
float INA226_ReadVoltage(uint8_t i2c_addr);

/* Exported variables --------------------------------------------------------*/

extern WheelSpeed_t wheel_speed;
extern Temperature_t temperature;
extern Current_t current;
extern Pedal_t pedal;

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_MANAGER_H */
