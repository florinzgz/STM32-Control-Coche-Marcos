/**
  ******************************************************************************
  * @file    motor_control.h
  * @brief   Motor control header with PWM, PID and Ackermann geometry
  ******************************************************************************
  */

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/* Motor identifier */
typedef enum {
    MOTOR_FL = 0,  /* Front Left */
    MOTOR_FR = 1,  /* Front Right */
    MOTOR_RL = 2,  /* Rear Left */
    MOTOR_RR = 3,  /* Rear Right */
    MOTOR_STEER = 4, /* Steering */
    MOTOR_COUNT = 5
} MotorID_t;

/* Motor direction */
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_REVERSE = 1,
    MOTOR_DIR_BRAKE = 2
} MotorDirection_t;

/* PID Controller structure */
typedef struct {
    float Kp;          /* Proportional gain */
    float Ki;          /* Integral gain */
    float Kd;          /* Derivative gain */
    float setpoint;    /* Target value */
    float integral;    /* Integral accumulator */
    float prev_error;  /* Previous error for derivative */
    float output_min;  /* Output minimum limit */
    float output_max;  /* Output maximum limit */
    float dt;          /* Sample time in seconds */
} PID_Controller_t;

/* Motor state structure */
typedef struct {
    uint16_t pwm_duty;        /* PWM duty cycle (0-100) */
    MotorDirection_t direction;
    bool enabled;
    float current_rpm;        /* Current speed in RPM */
    float target_rpm;         /* Target speed in RPM */
    float current_amps;       /* Current consumption in Amps */
    float temperature_c;      /* Temperature in Celsius */
    PID_Controller_t pid;     /* PID controller for speed */
} MotorState_t;

/* Ackermann steering structure */
typedef struct {
    float wheelbase;          /* Distance between front and rear axles (mm) */
    float track_width;        /* Distance between left and right wheels (mm) */
    int16_t encoder_position; /* Current encoder position */
    int16_t encoder_zero;     /* Zero position */
    float steering_angle;     /* Current steering angle (degrees) */
    float target_angle;       /* Target steering angle (degrees) */
    float max_angle;          /* Maximum steering angle (degrees) */
} Ackermann_t;

/* Exported constants --------------------------------------------------------*/
#define PWM_PERIOD 1000         /* PWM period for 20 kHz @ 170 MHz */
#define MAX_PWM_DUTY 95         /* Maximum duty cycle (%) */
#define MIN_PWM_DUTY 5          /* Minimum duty cycle (%) */

#define ENCODER_PPR 360         /* Encoder pulses per revolution */
#define ENCODER_CPR (ENCODER_PPR * 4)  /* Counts per revolution (quadrature) */

#define WHEELBASE_MM 1200.0f    /* Example wheelbase in mm */
#define TRACK_WIDTH_MM 800.0f   /* Example track width in mm */
#define MAX_STEERING_ANGLE 30.0f /* Maximum steering angle in degrees */

/* PID default gains for speed control */
#define PID_KP_SPEED 1.0f
#define PID_KI_SPEED 0.5f
#define PID_KD_SPEED 0.1f

/* PID default gains for steering position */
#define PID_KP_STEER 2.0f
#define PID_KI_STEER 0.1f
#define PID_KD_STEER 0.5f

/* Exported variables --------------------------------------------------------*/
extern MotorState_t motors[MOTOR_COUNT];
extern Ackermann_t ackermann;
extern DriveMode_t drive_mode;

/* Exported function prototypes ----------------------------------------------*/

/* Initialization */
void MotorControl_Init(void);

/* Motor control */
void Motor_SetSpeed(MotorID_t motor_id, uint16_t duty_cycle);
void Motor_SetDirection(MotorID_t motor_id, MotorDirection_t direction);
void Motor_Enable(MotorID_t motor_id, bool enable);
void Motor_EmergencyStop(void);

/* Traction control */
void Traction_SetThrottle(uint8_t throttle_percent);
void Traction_SetGear(Gear_t gear);
void Traction_ApplyAckermann(float steering_angle);
void Traction_UpdateSpeeds(void);

/* Steering control */
void Steering_SetAngle(float angle_degrees);
void Steering_UpdatePosition(void);
int16_t Steering_GetEncoderPosition(void);
void Steering_ResetEncoder(void);

/* PID control */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float dt);
float PID_Update(PID_Controller_t *pid, float measured_value);
void PID_Reset(PID_Controller_t *pid);

/* Ackermann geometry calculations */
void Ackermann_Init(void);
void Ackermann_CalculateWheelSpeeds(float steering_angle, float base_speed,
                                     float *fl_speed, float *fr_speed,
                                     float *rl_speed, float *rr_speed);

/* PWM control (low-level) */
void PWM_SetDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t duty);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
