# Sensor Manager Header

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdint.h>

// Structures for Wheel Sensors
typedef struct {
    uint8_t id;
    float speed;
    float distance;
} WheelSensor;

#define NUM_WHEEL_SENSORS 4
WheelSensor wheelSensors[NUM_WHEEL_SENSORS];

// Structures for DS18B20 Temperature Sensors
typedef struct {
    uint8_t id;
    float temperature;
} DS18B20Sensor;

#define NUM_DS18B20_SENSORS 5
DS18B20Sensor ds18B20Sensors[NUM_DS18B20_SENSORS];

// Structures for INA226 Current Sensors
typedef struct {
    uint8_t id;
    float current;
    float voltage;
} INA226Sensor;

#define NUM_INA226_SENSORS 6
INA226Sensor ina226Sensors[NUM_INA226_SENSORS];

// ADC Pedal Sensor
typedef struct {
    uint8_t id;
    uint16_t value;
} ADCPedalSensor;

ADCPedalSensor pedalSensor;

// Function Declarations
void initSensors();
void readWheelSensors();
void readDS18B20Sensors();
void readINA226Sensors();
void readPedalSensor();

#endif // SENSOR_MANAGER_H
