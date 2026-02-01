#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

#define MAX_SPEED 255
#define MIN_SPEED 0

// Motor PWM control structure
typedef struct {
    uint8_t speed;  // Current speed of the motor (0 to 255)
    uint8_t direction; // Direction (0: forward, 1: backward)
} MotorControl;

// Ackermann steering geometry functions
float calculateTurningRadius(float steeringAngle);
float calculateSteeringAngle(float turningRadius);

// PID controller structure
typedef struct {
    float kp;      // Proportional gain
    float ki;      // Integral gain
    float kd;      // Derivative gain
    float setPoint; // Desired value
    float lastError; // Last error value
    float integral; // Integral value
} PIDController;

// Function prototypes
void motor_init(MotorControl *motor);
void motor_setSpeed(MotorControl *motor, uint8_t speed);
void motor_setDirection(MotorControl *motor, uint8_t direction);
void motor_stop(MotorControl *motor);

// PID control function prototypes
void pid_init(PIDController *pid, float kp, float ki, float kd);
float pid_calculate(PIDController *pid, float currentValue);

// Encoder handling functions
void encoder_init(void);
uint32_t encoder_getPosition(void);

// Safety protection functions
void safety_check(void);

#endif // MOTOR_CONTROL_H
