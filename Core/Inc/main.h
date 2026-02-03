#ifndef MAIN_H
#define MAIN_H

// ENCODER DE DIRECCIÓN (E6B2-CWZ6C - 1200 PPR - TIM2 Quadrature)
// Quadrature mode: 1200 PPR × 4 = 4800 counts/revolution
// Resolution: 360° / 4800 = 0.075° per count
#define ENCODER_PPR 1200   // Pulses per Revolution for the encoder

// GPIO Pin Definitions
// Direct Motor Control via TIM1/TIM8
#define MOTOR1_PWM_PIN      GPIO_PIN_0   // Example pin for Motor 1
#define MOTOR2_PWM_PIN      GPIO_PIN_1   // Example pin for Motor 2
#define MOTOR3_PWM_PIN      GPIO_PIN_2   // Example pin for Motor 3
#define MOTOR4_PWM_PIN      GPIO_PIN_3   // Example pin for Motor 4

// DS18B20 Temperature Sensors (4 motors + 1 steering wheel)
#define TEMP_SENSOR1_PIN    GPIO_PIN_4   // Sensor for Motor 1
#define TEMP_SENSOR2_PIN    GPIO_PIN_5   // Sensor for Motor 2
#define TEMP_SENSOR3_PIN    GPIO_PIN_6   // Sensor for Motor 3
#define TEMP_SENSOR4_PIN    GPIO_PIN_7   // Sensor for Motor 4
#define STEERING_WHEEL_TEMP_PIN GPIO_PIN_8 // Sensor for Steering Wheel

// LJ12A3-4-Z/BX Sensors (4 wheels + 1 encoder Z)
#define WHEEL_SENSOR1_PIN   GPIO_PIN_9   // Sensor for Wheel 1
#define WHEEL_SENSOR2_PIN   GPIO_PIN_10  // Sensor for Wheel 2
#define WHEEL_SENSOR3_PIN   GPIO_PIN_11  // Sensor for Wheel 3
#define WHEEL_SENSOR4_PIN   GPIO_PIN_12  // Sensor for Wheel 4
#define ENCODER_Z_SENSOR_PIN GPIO_PIN_13  // Encoder Z Sensor

// Additional GPIO configurations can be added here as needed.

#endif // MAIN_H
