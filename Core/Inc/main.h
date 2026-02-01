/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* GPIO Pin Definitions ------------------------------------------------------*/

/* Traction Motors (4x BTS7960) - TIM1 PWM @ 20 kHz */
#define PWM_FL_Pin GPIO_PIN_8
#define PWM_FL_GPIO_Port GPIOA
#define DIR_FL_Pin GPIO_PIN_0
#define DIR_FL_GPIO_Port GPIOC
#define EN_FL_Pin GPIO_PIN_1
#define EN_FL_GPIO_Port GPIOC

#define PWM_FR_Pin GPIO_PIN_9
#define PWM_FR_GPIO_Port GPIOA
#define DIR_FR_Pin GPIO_PIN_2
#define DIR_FR_GPIO_Port GPIOC
#define EN_FR_Pin GPIO_PIN_3
#define EN_FR_GPIO_Port GPIOC

#define PWM_RL_Pin GPIO_PIN_10
#define PWM_RL_GPIO_Port GPIOA
#define DIR_RL_Pin GPIO_PIN_4
#define DIR_RL_GPIO_Port GPIOC
#define EN_RL_Pin GPIO_PIN_5
#define EN_RL_GPIO_Port GPIOC

#define PWM_RR_Pin GPIO_PIN_11
#define PWM_RR_GPIO_Port GPIOA
#define DIR_RR_Pin GPIO_PIN_6
#define DIR_RR_GPIO_Port GPIOC
#define EN_RR_Pin GPIO_PIN_7
#define EN_RR_GPIO_Port GPIOC

/* Steering Motor (BTS7960) - TIM8 PWM @ 20 kHz */
#define PWM_STEER_Pin GPIO_PIN_8
#define PWM_STEER_GPIO_Port GPIOC
#define DIR_STEER_Pin GPIO_PIN_9
#define DIR_STEER_GPIO_Port GPIOC
#define EN_STEER_Pin GPIO_PIN_10
#define EN_STEER_GPIO_Port GPIOC

/* Steering Encoder (E6B2-CWZ6C 360 PPR) - TIM2 Quadrature */
#define ENC_A_Pin GPIO_PIN_15
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_3
#define ENC_B_GPIO_Port GPIOB
#define ENC_Z_Pin GPIO_PIN_4
#define ENC_Z_GPIO_Port GPIOB
#define ENC_Z_EXTI_IRQn EXTI4_IRQn

/* Wheel Speed Sensors (4x Hall) */
#define WHEEL_FL_Pin GPIO_PIN_0
#define WHEEL_FL_GPIO_Port GPIOB
#define WHEEL_FL_EXTI_IRQn EXTI0_IRQn

#define WHEEL_FR_Pin GPIO_PIN_1
#define WHEEL_FR_GPIO_Port GPIOB
#define WHEEL_FR_EXTI_IRQn EXTI1_IRQn

#define WHEEL_RL_Pin GPIO_PIN_2
#define WHEEL_RL_GPIO_Port GPIOB
#define WHEEL_RL_EXTI_IRQn EXTI2_IRQn

#define WHEEL_RR_Pin GPIO_PIN_10
#define WHEEL_RR_GPIO_Port GPIOB
#define WHEEL_RR_EXTI_IRQn EXTI15_10_IRQn

/* I2C Bus (INA226 x6 via TCA9548A) */
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB

/* CAN Bus (FDCAN1 @ 500 kbps) */
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_8
#define CAN_RX_GPIO_Port GPIOB

/* Pedal Analog Input (ADC1) */
#define PEDAL_Pin GPIO_PIN_0
#define PEDAL_GPIO_Port GPIOA

/* Shifter F/N/R (GPIO Input, Pull-up, Active Low) */
#define FWD_Pin GPIO_PIN_12
#define FWD_GPIO_Port GPIOB
#define NEU_Pin GPIO_PIN_13
#define NEU_GPIO_Port GPIOB
#define REV_Pin GPIO_PIN_14
#define REV_GPIO_Port GPIOB

/* Relays (GPIO Output, Default LOW) */
#define RELAY_MAIN_Pin GPIO_PIN_11
#define RELAY_MAIN_GPIO_Port GPIOC
#define RELAY_TRAC_Pin GPIO_PIN_12
#define RELAY_TRAC_GPIO_Port GPIOC
#define RELAY_DIR_Pin GPIO_PIN_2
#define RELAY_DIR_GPIO_Port GPIOD

/* Temperature Sensors (OneWire - DS18B20 x4) */
#define TEMP_Pin GPIO_PIN_5
#define TEMP_GPIO_Port GPIOB

/* System Constants ----------------------------------------------------------*/
#define SYSTEM_FREQUENCY 170000000  /* 170 MHz */
#define PWM_FREQUENCY 20000         /* 20 kHz */
#define ADC_SAMPLE_RATE 200         /* 200 Hz */

/* Drive Modes */
typedef enum {
    DRIVE_MODE_4X2 = 0,
    DRIVE_MODE_4X4 = 1
} DriveMode_t;

/* Gear Selection */
typedef enum {
    GEAR_NEUTRAL = 0,
    GEAR_FORWARD = 1,
    GEAR_REVERSE = 2
} Gear_t;

/* System State */
typedef enum {
    STATE_INIT = 0,
    STATE_IDLE = 1,
    STATE_READY = 2,
    STATE_RUNNING = 3,
    STATE_SAFETY = 4,
    STATE_ERROR = 5
} SystemState_t;

/* Exported variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;    /* Traction PWM */
extern TIM_HandleTypeDef htim2;    /* Encoder */
extern TIM_HandleTypeDef htim3;    /* ADC trigger */
extern TIM_HandleTypeDef htim8;    /* Steering PWM */
extern ADC_HandleTypeDef hadc1;    /* Pedal */
extern I2C_HandleTypeDef hi2c1;    /* Current sensors */
extern FDCAN_HandleTypeDef hfdcan1; /* CAN Bus */

/* Exported function prototypes ----------------------------------------------*/
void Error_Handler(void);
void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
