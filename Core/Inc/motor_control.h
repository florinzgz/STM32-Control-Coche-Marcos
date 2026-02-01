#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* Motor control functions */
void Motor_Init(void);
void Traction_SetThrottle(int8_t throttle);
void Traction_SetSpeed_FL(int8_t speed);
void Traction_SetSpeed_FR(int8_t speed);
void Traction_SetSpeed_RL(int8_t speed);
void Traction_SetSpeed_RR(int8_t speed);
void Traction_EmergencyStop(void);
void Steering_SetAngle(float angle_deg);
void Steering_ControlLoop(void);
float Steering_GetCurrentAngle(void);
void Ackermann_ApplySteering(float angle_rad, int8_t throttle);

#ifdef __cplusplus
}
#endif

#endif