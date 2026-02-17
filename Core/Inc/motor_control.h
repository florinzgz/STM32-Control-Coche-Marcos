#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// Motor indices
#define MOTOR_FL  0  // Front Left
#define MOTOR_FR  1  // Front Right
#define MOTOR_RL  2  // Rear Left
#define MOTOR_RR  3  // Rear Right

/* Gear position — received from ESP32 via CAN_ID_CMD_MODE byte 1.
 * Traced to CAN_PROTOCOL.md: 0=PARK, 1=REVERSE, 2=NEUTRAL, 3=FORWARD.
 * Park (P) was implicit in the base firmware; it is now handled
 * explicitly as an active hold brake on the STM32 side.               */
typedef enum {
    GEAR_PARK    = 0,
    GEAR_REVERSE = 1,
    GEAR_NEUTRAL = 2,
    GEAR_FORWARD = 3,
    GEAR_FORWARD_D2 = 4   /* Full power (100 %) forward mode */
} GearPosition_t;

// Wheel state structure (per wheel)
typedef struct {
    float demandPct;      // Desired power 0-100%
    float currentA;       // Measured current (Amperes)
    float tempC;          // Temperature (Celsius)
    float speedKmh;       // Wheel speed (km/h)
    uint16_t pwm;         // PWM duty cycle (0-8499 for TIM1 @ 20kHz)
    bool reverse;         // Direction flag
    float effortPct;      // Motor effort percentage
} WheelState_t;

// Traction system state
typedef struct {
    WheelState_t wheels[4];  // FL, FR, RL, RR
    bool mode4x4;            // true = 4x4, false = 4x2 (front only)
    float demandPct;         // Global throttle demand 0-100%
    bool axisRotation;       // Tank turn mode
} TractionState_t;

// Ackermann calculation result
typedef struct {
    float innerDeg;    // Inner wheel angle
    float outerDeg;    // Outer wheel angle
} AckermannResult_t;

/* Motor Control Functions */
void Motor_Init(void);
void Traction_Init(void);
void Traction_SetDemand(float throttlePct);
void Traction_SetMode4x4(bool enable);
void Traction_SetAxisRotation(bool enable);
void Traction_SetGear(GearPosition_t gear);
GearPosition_t Traction_GetGear(void);
void Traction_Update(void);
void Traction_EmergencyStop(void);
const TractionState_t* Traction_GetState(void);

/* Steering Functions */
void Steering_Init(void);
void Steering_SetAngle(float angle_deg);
void Steering_ControlLoop(void);
float Steering_GetCurrentAngle(void);
bool Steering_IsCalibrated(void);
void Steering_SetCalibrated(void);
void Steering_Neutralize(void);
void Steering_GetWheelAngles(float *out_fl_deg, float *out_fr_deg);

/* Encoder Health */
void  Encoder_CheckHealth(void);
bool  Encoder_HasFault(void);

/* Low-level PWM control (TIM1/TIM8 direct) */
void Motor_SetPWM_FL(uint16_t pwm, bool reverse);
void Motor_SetPWM_FR(uint16_t pwm, bool reverse);
void Motor_SetPWM_RL(uint16_t pwm, bool reverse);
void Motor_SetPWM_RR(uint16_t pwm, bool reverse);
void Motor_SetPWM_Steering(uint16_t pwm, bool reverse);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
