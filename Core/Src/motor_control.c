/**
  ******************************************************************************
  * @file    motor_control.c
  * @brief   Motor control implementation - Direct PWM control
  * @author  florinzgz
  * @date    2026-02-01
  ******************************************************************************
  * @attention
  *
  * Control directo de motores mediante PWM (TIM1/TIM8)
  * - 4 motores de tracción (TIM1 CH1-4)
  * - 1 motor de dirección con encoder (TIM8 CH3, TIM2 encoder)
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"
#include <stdlib.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PID_OUTPUT_LIMIT    100

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Motor instances */
Motor_t motor_FL;
Motor_t motor_FR;
Motor_t motor_RL;
Motor_t motor_RR;
SteeringMotor_t steering;

/* Encoder Z pulse flag */
volatile uint8_t encoder_z_detected = 0;

/* PID state */
static float pid_integral = 0.0f;
static float pid_last_error = 0.0f;

/* Private function prototypes -----------------------------------------------*/
static void Motor_ConfigureInstance(Motor_t *motor, GPIO_TypeDef *dir_port, uint16_t dir_pin,
                                    GPIO_TypeDef *en_port, uint16_t en_pin,
                                    TIM_HandleTypeDef *htim, uint32_t channel);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief Initialize motor control system
  * @retval HAL status
  */
HAL_StatusTypeDef Motor_Init(void)
{
  /* Configure motor instances */
  Motor_ConfigureInstance(&motor_FL, DIR_FL_GPIO_Port, DIR_FL_Pin,
                         EN_FL_GPIO_Port, EN_FL_Pin,
                         &htim1, PWM_FL_TIM_CHANNEL);
  
  Motor_ConfigureInstance(&motor_FR, DIR_FR_GPIO_Port, DIR_FR_Pin,
                         EN_FR_GPIO_Port, EN_FR_Pin,
                         &htim1, PWM_FR_TIM_CHANNEL);
  
  Motor_ConfigureInstance(&motor_RL, DIR_RL_GPIO_Port, DIR_RL_Pin,
                         EN_RL_GPIO_Port, EN_RL_Pin,
                         &htim1, PWM_RL_TIM_CHANNEL);
  
  Motor_ConfigureInstance(&motor_RR, DIR_RR_GPIO_Port, DIR_RR_Pin,
                         EN_RR_GPIO_Port, EN_RR_Pin,
                         &htim1, PWM_RR_TIM_CHANNEL);
  
  /* Start PWM channels for traction motors */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  
  /* Set all motors to 0% power, disabled */
  Motor_SetPower(&motor_FL, 0);
  Motor_SetPower(&motor_FR, 0);
  Motor_SetPower(&motor_RL, 0);
  Motor_SetPower(&motor_RR, 0);
  
  Motor_Enable(&motor_FL, 0);
  Motor_Enable(&motor_FR, 0);
  Motor_Enable(&motor_RL, 0);
  Motor_Enable(&motor_RR, 0);
  
  return HAL_OK;
}

/**
  * @brief Initialize steering motor with encoder
  * @retval HAL status
  */
HAL_StatusTypeDef Steering_Init(void)
{
  /* Configure steering motor instance */
  Motor_ConfigureInstance(&steering.motor, DIR_STEER_GPIO_Port, DIR_STEER_Pin,
                         EN_STEER_GPIO_Port, EN_STEER_Pin,
                         &htim8, PWM_STEER_TIM_CHANNEL);
  
  /* Initialize steering state */
  steering.current_position = 0;
  steering.target_position = 0;
  steering.angle_deg = 0.0f;
  steering.at_limit = 0;
  
  /* Start PWM for steering motor */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  
  /* Start encoder in quadrature mode */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  
  /* Set encoder counter to center value */
  __HAL_TIM_SET_COUNTER(&htim2, ENCODER_CENTER_OFFSET);
  
  /* Set motor to 0% power, disabled */
  Motor_SetPower(&steering.motor, 0);
  Motor_Enable(&steering.motor, 0);
  
  /* Reset PID state */
  pid_integral = 0.0f;
  pid_last_error = 0.0f;
  
  return HAL_OK;
}

/**
  * @brief Calibrate steering motor (find center using Z pulse)
  * @retval HAL status
  */
HAL_StatusTypeDef Steering_Calibrate(void)
{
  uint32_t timeout_start = HAL_GetTick();
  uint32_t timeout_ms = 5000;
  
  encoder_z_detected = 0;
  
  /* Enable motor */
  Motor_Enable(&steering.motor, 1);
  
  /* Rotate slowly to find Z pulse */
  Motor_SetPower(&steering.motor, 20);
  
  /* Wait for Z pulse or timeout */
  while (!encoder_z_detected && (HAL_GetTick() - timeout_start) < timeout_ms) {
    HAL_Delay(10);
  }
  
  /* Stop motor */
  Motor_SetPower(&steering.motor, 0);
  Motor_Enable(&steering.motor, 0);
  
  if (encoder_z_detected) {
    /* Reset encoder counter to center */
    __HAL_TIM_SET_COUNTER(&htim2, ENCODER_CENTER_OFFSET);
    steering.current_position = 0;
    steering.angle_deg = 0.0f;
    return HAL_OK;
  } else {
    /* Calibration failed - use current position as center */
    __HAL_TIM_SET_COUNTER(&htim2, ENCODER_CENTER_OFFSET);
    steering.current_position = 0;
    steering.angle_deg = 0.0f;
    return HAL_ERROR;
  }
}

/**
  * @brief Set motor power with direction
  * @param motor: Pointer to Motor_t structure
  * @param power_pct: Power percentage (-100 to +100, negative = reverse)
  * @retval None
  */
void Motor_SetPower(Motor_t *motor, int8_t power_pct)
{
  uint16_t pwm_value;
  uint8_t abs_power;
  
  if (!motor) return;
  
  /* Clamp power to valid range */
  if (power_pct > MOTOR_POWER_MAX) power_pct = MOTOR_POWER_MAX;
  if (power_pct < MOTOR_POWER_MIN) power_pct = MOTOR_POWER_MIN;
  
  motor->power_pct = power_pct;
  
  /* Set direction */
  if (power_pct >= 0) {
    HAL_GPIO_WritePin(motor->DIR_PORT, motor->DIR_PIN, GPIO_PIN_RESET);
    abs_power = (uint8_t)power_pct;
  } else {
    HAL_GPIO_WritePin(motor->DIR_PORT, motor->DIR_PIN, GPIO_PIN_SET);
    abs_power = (uint8_t)(-power_pct);
  }
  
  /* Convert power percentage to PWM CCR value */
  pwm_value = Motor_PercentToCCR(abs_power);
  
  /* Set PWM duty cycle */
  __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pwm_value);
}

/**
  * @brief Enable or disable motor
  * @param motor: Pointer to Motor_t structure
  * @param enable: 1 to enable, 0 to disable
  * @retval None
  */
void Motor_Enable(Motor_t *motor, uint8_t enable)
{
  if (!motor) return;
  
  motor->enabled = enable;
  
  if (enable) {
    HAL_GPIO_WritePin(motor->EN_PORT, motor->EN_PIN, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(motor->EN_PORT, motor->EN_PIN, GPIO_PIN_RESET);
  }
}

/**
  * @brief Set power for all traction motors (uniform)
  * @param power_pct: Power percentage (-100 to +100)
  * @retval None
  */
void Traction_SetUniform(int8_t power_pct)
{
  Motor_SetPower(&motor_FL, power_pct);
  Motor_SetPower(&motor_FR, power_pct);
  Motor_SetPower(&motor_RL, power_pct);
  Motor_SetPower(&motor_RR, power_pct);
}

/**
  * @brief Set power for each traction motor individually
  * @param fl: Front left power
  * @param fr: Front right power
  * @param rl: Rear left power
  * @param rr: Rear right power
  * @retval None
  */
void Traction_SetIndividual(int8_t fl, int8_t fr, int8_t rl, int8_t rr)
{
  Motor_SetPower(&motor_FL, fl);
  Motor_SetPower(&motor_FR, fr);
  Motor_SetPower(&motor_RL, rl);
  Motor_SetPower(&motor_RR, rr);
}

/**
  * @brief Enable/disable all traction motors
  * @param enable: 1 to enable, 0 to disable
  * @retval None
  */
void Traction_Enable(uint8_t enable)
{
  Motor_Enable(&motor_FL, enable);
  Motor_Enable(&motor_FR, enable);
  Motor_Enable(&motor_RL, enable);
  Motor_Enable(&motor_RR, enable);
}

/**
  * @brief Apply electric brake (0% PWM with motors enabled)
  * @retval None
  */
void Traction_ElectricBrake(void)
{
  Traction_SetUniform(0);
  Traction_Enable(1);
}

/**
  * @brief Set steering angle in degrees
  * @param angle_deg: Desired angle (-45 to +45 degrees)
  * @retval None
  */
void Steering_SetAngle(float angle_deg)
{
  /* Clamp angle to limits */
  if (angle_deg > ENCODER_MAX_ANGLE) angle_deg = ENCODER_MAX_ANGLE;
  if (angle_deg < -ENCODER_MAX_ANGLE) angle_deg = -ENCODER_MAX_ANGLE;
  
  /* Convert angle to encoder counts */
  steering.target_position = (int16_t)(angle_deg / ENCODER_DEGREES_PER_CNT);
}

/**
  * @brief Update steering PID controller (call at 100 Hz)
  * @retval None
  */
void Steering_Update(void)
{
  int32_t encoder_count;
  float error, derivative, output;
  int8_t motor_power;
  
  /* Read encoder position */
  encoder_count = __HAL_TIM_GET_COUNTER(&htim2);
  steering.current_position = (int16_t)(encoder_count - ENCODER_CENTER_OFFSET);
  
  /* Convert position to angle */
  steering.angle_deg = steering.current_position * ENCODER_DEGREES_PER_CNT;
  
  /* Check if at mechanical limit */
  if (abs(steering.current_position) >= ENCODER_MAX_COUNTS) {
    steering.at_limit = 1;
  } else {
    steering.at_limit = 0;
  }
  
  /* Calculate PID error */
  error = (float)(steering.target_position - steering.current_position);
  
  /* PID controller */
  pid_integral += error * 0.01f; // dt = 10ms
  
  /* Anti-windup: clamp integral */
  if (pid_integral > 1000.0f) pid_integral = 1000.0f;
  if (pid_integral < -1000.0f) pid_integral = -1000.0f;
  
  derivative = (error - pid_last_error) / 0.01f;
  pid_last_error = error;
  
  /* Calculate PID output */
  output = STEERING_KP * error + STEERING_KI * pid_integral + STEERING_KD * derivative;
  
  /* Limit output */
  if (output > PID_OUTPUT_LIMIT) output = PID_OUTPUT_LIMIT;
  if (output < -PID_OUTPUT_LIMIT) output = -PID_OUTPUT_LIMIT;
  
  /* Dead zone for small errors */
  if (abs(error) < 2.0f) {
    output = 0.0f;
  }
  
  /* Convert to motor power */
  motor_power = (int8_t)output;
  
  /* Apply power to motor */
  Motor_SetPower(&steering.motor, motor_power);
  
  /* Enable motor if output is non-zero */
  if (motor_power != 0) {
    Motor_Enable(&steering.motor, 1);
  } else {
    Motor_Enable(&steering.motor, 0);
  }
}

/**
  * @brief Get current steering angle
  * @retval Current angle in degrees
  */
float Steering_GetAngle(void)
{
  return steering.angle_deg;
}

/**
  * @brief Get current steering encoder position
  * @retval Encoder position in counts
  */
int16_t Steering_GetPosition(void)
{
  return steering.current_position;
}

/**
  * @brief Convert percentage to PWM CCR value
  * @param percent: Percentage (0-100)
  * @retval CCR value (0-8499)
  */
uint16_t Motor_PercentToCCR(uint8_t percent)
{
  if (percent > 100) percent = 100;
  return (uint16_t)((uint32_t)percent * PWM_PERIOD / 100);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Configure motor instance structure
  * @param motor: Pointer to Motor_t structure
  * @param dir_port: Direction GPIO port
  * @param dir_pin: Direction GPIO pin
  * @param en_port: Enable GPIO port
  * @param en_pin: Enable GPIO pin
  * @param htim: Timer handle
  * @param channel: Timer channel
  * @retval None
  */
static void Motor_ConfigureInstance(Motor_t *motor, GPIO_TypeDef *dir_port, uint16_t dir_pin,
                                    GPIO_TypeDef *en_port, uint16_t en_pin,
                                    TIM_HandleTypeDef *htim, uint32_t channel)
{
  motor->DIR_PORT = dir_port;
  motor->DIR_PIN = dir_pin;
  motor->EN_PORT = en_port;
  motor->EN_PIN = en_pin;
  motor->htim = htim;
  motor->channel = channel;
  motor->power_pct = 0;
  motor->enabled = 0;
}
