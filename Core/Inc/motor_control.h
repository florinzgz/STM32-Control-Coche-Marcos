// motor_control.h

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

// Constants for Ackermann geometry
#define WHEELBASE 0.95 // in meters
#define TRACK 0.70 // in meters

// Structure for individual wheel state
typedef struct {
    uint8_t demandPct;  // Desired percentage demand for the motor
    float currentA;     // Current in Amperes
    float tempC;        // Temperature in Celsius
    float speedKmh;     // Speed in km/h
    uint8_t pwm;        // PWM value
    uint8_t reverse;     // Reverse flag (0 = forward, 1 = reverse)
} WheelState;

// Structure for traction state with mode support
typedef struct {
    WheelState wheels[4]; // Array of wheel states for 4 vehicles
    uint8_t mode;          // 0 = 4x4 mode, 1 = 4x2 mode
} TractionState;

// Function prototypes for motor control
void Traction_Init(void);
void Traction_Update(float steering_angle);
void Steering_Init(void);
float Steering_GetAngle(void);

// Safety functions
void Safety_Check(void);
void Safety_Engage(void);

#endif // MOTOR_CONTROL_H
