/**
  ****************************************************************************
  * @file           : motor_control.c
  * @brief          : Motor control implementation with PWM, PID, and Ackermann
  ****************************************************************************
  */

#include "motor_control.h"
#include "main.h"
#include <math.h>

/* Constants */
#define WHEELBASE_M 1.2f
#define TRACK_WIDTH_M 0.8f
#define PWM_PERIOD 8499
#define PWM_FREQUENCY 20000

/* Motor structures */
typedef struct {
    TIM_HandleTypeDef *timer;
    uint32_t channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef *en_port;
    uint16_t en_pin;
    int16_t power;
} Motor_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float setpoint;
    float output;
} PID_t;

/* Global variables */
Motor_t motor_fl, motor_fr, motor_rl, motor_rr, motor_steer;
PID_t steering_pid = {2.0f, 0.1f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f};
extern TIM_HandleTypeDef htim1, htim2, htim8;

/* Private function prototypes */
static void Motor_SetPWM(Motor_t *motor, uint16_t pwm);
static void Motor_SetDirection(Motor_t *motor, int8_t direction);
static void Motor_Enable(Motor_t *motor, uint8_t enable);
static float PID_Compute(PID_t *pid, float measured, float dt);

void Motor_Init(void)
{
    motor_fl.timer = &htim1;
    motor_fl.channel = TIM_CHANNEL_1;
    motor_fl.dir_port = GPIOC;
    motor_fl.dir_pin = GPIO_PIN_0;
    motor_fl.en_port = GPIOC;
    motor_fl.en_pin = GPIO_PIN_1;
    
    motor_fr.timer = &htim1;
    motor_fr.channel = TIM_CHANNEL_2;
    motor_fr.dir_port = GPIOC;
    motor_fr.dir_pin = GPIO_PIN_2;
    motor_fr.en_port = GPIOC;
    motor_fr.en_pin = GPIO_PIN_3;
    
    motor_rl.timer = &htim1;
    motor_rl.channel = TIM_CHANNEL_3;
    motor_rl.dir_port = GPIOC;
    motor_rl.dir_pin = GPIO_PIN_4;
    motor_rl.en_port = GPIOC;
    motor_rl.en_pin = GPIO_PIN_5;
    
    motor_rr.timer = &htim1;
    motor_rr.channel = TIM_CHANNEL_4;
    motor_rr.dir_port = GPIOC;
    motor_rr.dir_pin = GPIO_PIN_6;
    motor_rr.en_port = GPIOC;
    motor_rr.en_pin = GPIO_PIN_7;
    
    motor_steer.timer = &htim8;
    motor_steer.channel = TIM_CHANNEL_3;
    motor_steer.dir_port = GPIOC;
    motor_steer.dir_pin = GPIO_PIN_9;
    motor_steer.en_port = GPIOC;
    motor_steer.en_pin = GPIO_PIN_10;
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

void Traction_SetThrottle(int8_t throttle)
{
    if (throttle < -100) throttle = -100;
    if (throttle > 100) throttle = 100;
    
    uint16_t pwm = (uint16_t)(abs(throttle) * PWM_PERIOD / 100);
    int8_t direction = (throttle >= 0) ? 1 : -1;
    
    Motor_SetPWM(&motor_fl, pwm);
    Motor_SetDirection(&motor_fl, direction);
    Motor_Enable(&motor_fl, (throttle != 0));
    
    Motor_SetPWM(&motor_fr, pwm);
    Motor_SetDirection(&motor_fr, direction);
    Motor_Enable(&motor_fr, (throttle != 0));
    
    Motor_SetPWM(&motor_rl, pwm);
    Motor_SetDirection(&motor_rl, direction);
    Motor_Enable(&motor_rl, (throttle != 0));
    
    Motor_SetPWM(&motor_rr, pwm);
    Motor_SetDirection(&motor_rr, direction);
    Motor_Enable(&motor_rr, (throttle != 0));
}

void Traction_EmergencyStop(void)
{
    Motor_Enable(&motor_fl, 0);
    Motor_Enable(&motor_fr, 0);
    Motor_Enable(&motor_rl, 0);
    Motor_Enable(&motor_rr, 0);
    Motor_Enable(&motor_steer, 0);
}

void Steering_SetAngle(float angle_deg)
{
    if (angle_deg < -45.0f) angle_deg = -45.0f;
    if (angle_deg > 45.0f) angle_deg = 45.0f;
    steering_pid.setpoint = angle_deg * 360.0f / 45.0f;
}

void Steering_ControlLoop(void)
{
    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) / 1000.0f;
    if (dt < 0.001f) return;
    
    int16_t encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    float measured = (float)encoder_count;
    float output = PID_Compute(&steering_pid, measured, dt);
    
    if (output < -100.0f) output = -100.0f;
    if (output > 100.0f) output = 100.0f;
    
    uint16_t pwm = (uint16_t)(fabs(output) * PWM_PERIOD / 100.0f);
    int8_t direction = (output >= 0) ? 1 : -1;
    
    Motor_SetPWM(&motor_steer, pwm);
    Motor_SetDirection(&motor_steer, direction);
    Motor_Enable(&motor_steer, (fabs(output) > 1.0f));
    last_time = now;
}

static void Motor_SetPWM(Motor_t *motor, uint16_t pwm)
{
    if (pwm > PWM_PERIOD) pwm = PWM_PERIOD;
    __HAL_TIM_SET_COMPARE(motor->timer, motor->channel, pwm);
}

static void Motor_SetDirection(Motor_t *motor, int8_t direction)
{
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, (direction > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Motor_Enable(Motor_t *motor, uint8_t enable)
{
    HAL_GPIO_WritePin(motor->en_port, motor->en_pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static float PID_Compute(PID_t *pid, float measured, float dt)
{
    if (dt <= 0.0001f) return pid->output;
    float error = pid->setpoint - measured;
    pid->integral += error * dt;
    if (pid->integral > 1000.0f) pid->integral = 1000.0f;
    if (pid->integral < -1000.0f) pid->integral = -1000.0f;
    float derivative = (error - pid->prev_error) / dt;
    pid->output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    pid->prev_error = error;
    return pid->output;
}