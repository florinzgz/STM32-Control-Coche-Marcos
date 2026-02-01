/**
  ******************************************************************************
  * @file    motor_control.h
  * @brief   Motor control interface - Direct PWM control
  * @author  florinzgz
  * @date    2026-02-01
  ******************************************************************************
  * @attention
  *
  * Control directo de motores mediante PWM (TIM1/TIM8)
  * NO se usa PCA9685
  *
  ******************************************************************************
  */

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Motor structure
 */
typedef struct {
    GPIO_TypeDef *DIR_PORT;         /* Direction GPIO port */
    uint16_t      DIR_PIN;          /* Direction pin */
    GPIO_TypeDef *EN_PORT;          /* Enable GPIO port */
    uint16_t      EN_PIN;           /* Enable pin */
    TIM_HandleTypeDef *htim;        /* Timer handle */
    uint32_t      channel;          /* Timer channel */
    int8_t        power_pct;        /* Current power (-100 to +100) */
    uint8_t       enabled;          /* Motor enabled flag */
} Motor_t;

/**
 * @brief Steering motor structure (with encoder feedback)
 */
typedef struct {
    Motor_t motor;                  /* Base motor */
    int16_t current_position;       /* Current encoder position (-720 to +720) */
    int16_t target_position;        /* Target encoder position */
    float   angle_deg;              /* Current angle in degrees */
    uint8_t at_limit;               /* At mechanical limit flag */
} SteeringMotor_t;

/* Exported constants --------------------------------------------------------*/

/* Motor power limits */
#define MOTOR_POWER_MIN     -100
#define MOTOR_POWER_MAX     100

/* Steering PID constants */
#define STEERING_KP         2.0f
#define STEERING_KI         0.1f
#define STEERING_KD         0.5f

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Initialize motor control system
 * @retval HAL status
 */
HAL_StatusTypeDef Motor_Init(void);

/**
 * @brief Set motor power with direction
 * @param motor: Pointer to Motor_t structure
 * @param power_pct: Power percentage (-100 to +100, negative = reverse)
 * @retval None
 */
void Motor_SetPower(Motor_t *motor, int8_t power_pct);

/**
 * @brief Enable or disable motor
 * @param motor: Pointer to Motor_t structure
 * @param enable: 1 to enable, 0 to disable
 * @retval None
 */
void Motor_Enable(Motor_t *motor, uint8_t enable);

/**
 * @brief Set power for all traction motors (uniform)
 * @param power_pct: Power percentage (-100 to +100)
 * @retval None
 */
void Traction_SetUniform(int8_t power_pct);

/**
 * @brief Set power for each traction motor individually
 * @param fl: Front left power
 * @param fr: Front right power
 * @param rl: Rear left power
 * @param rr: Rear right power
 * @retval None
 */
void Traction_SetIndividual(int8_t fl, int8_t fr, int8_t rl, int8_t rr);

/**
 * @brief Enable/disable all traction motors
 * @param enable: 1 to enable, 0 to disable
 * @retval None
 */
void Traction_Enable(uint8_t enable);

/**
 * @brief Apply electric brake (0% PWM with motors enabled)
 * @retval None
 */
void Traction_ElectricBrake(void);

/**
 * @brief Initialize steering motor with encoder
 * @retval HAL status
 */
HAL_StatusTypeDef Steering_Init(void);

/**
 * @brief Calibrate steering motor (find center using Z pulse)
 * @retval HAL status
 */
HAL_StatusTypeDef Steering_Calibrate(void);

/**
 * @brief Set steering angle in degrees
 * @param angle_deg: Desired angle (-45 to +45 degrees)
 * @retval None
 */
void Steering_SetAngle(float angle_deg);

/**
 * @brief Update steering PID controller (call at 100 Hz)
 * @retval None
 */
void Steering_Update(void);

/**
 * @brief Get current steering angle
 * @retval Current angle in degrees
 */
float Steering_GetAngle(void);

/**
 * @brief Get current steering encoder position
 * @retval Encoder position in counts
 */
int16_t Steering_GetPosition(void);

/**
 * @brief Convert percentage to PWM CCR value
 * @param percent: Percentage (0-100)
 * @retval CCR value (0-8499)
 */
uint16_t Motor_PercentToCCR(uint8_t percent);

/* Exported variables --------------------------------------------------------*/

extern Motor_t motor_FL;
extern Motor_t motor_FR;
extern Motor_t motor_RL;
extern Motor_t motor_RR;
extern SteeringMotor_t steering;

/* Encoder Z pulse detection flag */
extern volatile uint8_t encoder_z_detected;

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
