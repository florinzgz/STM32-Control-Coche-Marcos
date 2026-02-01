/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* ============================================================================ */
/*                         PIN DEFINITIONS - MOTORS                             */
/* ============================================================================ */

/* Motor Front Left (FL) - TIM1_CH1 */
#define PWM_FL_Pin GPIO_PIN_8
#define PWM_FL_GPIO_Port GPIOA
#define PWM_FL_TIM_CHANNEL TIM_CHANNEL_1
#define DIR_FL_Pin GPIO_PIN_0
#define DIR_FL_GPIO_Port GPIOC
#define EN_FL_Pin GPIO_PIN_1
#define EN_FL_GPIO_Port GPIOC

/* Motor Front Right (FR) - TIM1_CH2 */
#define PWM_FR_Pin GPIO_PIN_9
#define PWM_FR_GPIO_Port GPIOA
#define PWM_FR_TIM_CHANNEL TIM_CHANNEL_2
#define DIR_FR_Pin GPIO_PIN_2
#define DIR_FR_GPIO_Port GPIOC
#define EN_FR_Pin GPIO_PIN_3
#define EN_FR_GPIO_Port GPIOC

/* Motor Rear Left (RL) - TIM1_CH3 */
#define PWM_RL_Pin GPIO_PIN_10
#define PWM_RL_GPIO_Port GPIOA
#define PWM_RL_TIM_CHANNEL TIM_CHANNEL_3
#define DIR_RL_Pin GPIO_PIN_4
#define DIR_RL_GPIO_Port GPIOC
#define EN_RL_Pin GPIO_PIN_5
#define EN_RL_GPIO_Port GPIOC

/* Motor Rear Right (RR) - TIM1_CH4 */
#define PWM_RR_Pin GPIO_PIN_11
#define PWM_RR_GPIO_Port GPIOA
#define PWM_RR_TIM_CHANNEL TIM_CHANNEL_4
#define DIR_RR_Pin GPIO_PIN_6
#define DIR_RR_GPIO_Port GPIOC
#define EN_RR_Pin GPIO_PIN_7
#define EN_RR_GPIO_Port GPIOC

/* Motor Steering - TIM8_CH3 */
#define PWM_STEER_Pin GPIO_PIN_8
#define PWM_STEER_GPIO_Port GPIOC
#define PWM_STEER_TIM_CHANNEL TIM_CHANNEL_3
#define DIR_STEER_Pin GPIO_PIN_9
#define DIR_STEER_GPIO_Port GPIOC
#define EN_STEER_Pin GPIO_PIN_10
#define EN_STEER_GPIO_Port GPIOC

/* ============================================================================ */
/*                         PIN DEFINITIONS - SENSORS                            */
/* ============================================================================ */

/* Wheel Sensors (4 wheels) */
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

/* Encoder Steering (E6B2-CWZ6C) - TIM2 Quadrature */
#define ENC_A_Pin GPIO_PIN_15
#define ENC_A_GPIO_Port GPIOA
#define ENC_A_TIM_CHANNEL TIM_CHANNEL_1

#define ENC_B_Pin GPIO_PIN_3
#define ENC_B_GPIO_Port GPIOB
#define ENC_B_TIM_CHANNEL TIM_CHANNEL_2

#define ENC_Z_Pin GPIO_PIN_4
#define ENC_Z_GPIO_Port GPIOB
#define ENC_Z_EXTI_IRQn EXTI4_IRQn

/* Temperature Sensors (DS18B20 - OneWire) */
#define TEMP_ONEWIRE_Pin GPIO_PIN_5
#define TEMP_ONEWIRE_GPIO_Port GPIOB

/* I2C for INA226 Current Sensors */
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB

#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB

/* Pedal Hall Analog (ADC1_IN1) */
#define PEDAL_ANALOG_Pin GPIO_PIN_0
#define PEDAL_ANALOG_GPIO_Port GPIOA
#define PEDAL_ADC_CHANNEL ADC_CHANNEL_1

/* Shifter F/N/R */
#define SHIFTER_FWD_Pin GPIO_PIN_12
#define SHIFTER_FWD_GPIO_Port GPIOB

#define SHIFTER_NEU_Pin GPIO_PIN_13
#define SHIFTER_NEU_GPIO_Port GPIOB

#define SHIFTER_REV_Pin GPIO_PIN_14
#define SHIFTER_REV_GPIO_Port GPIOB

/* ============================================================================ */
/*                         PIN DEFINITIONS - COMMUNICATION                      */
/* ============================================================================ */

/* CAN Bus (FDCAN1) */
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB

#define CAN_RX_Pin GPIO_PIN_8
#define CAN_RX_GPIO_Port GPIOB

/* ============================================================================ */
/*                         PIN DEFINITIONS - RELAYS                             */
/* ============================================================================ */

/* Power Relays (Active HIGH, Fail-safe LOW) */
#define RELAY_MAIN_Pin GPIO_PIN_11
#define RELAY_MAIN_GPIO_Port GPIOC

#define RELAY_TRAC_Pin GPIO_PIN_12
#define RELAY_TRAC_GPIO_Port GPIOC

#define RELAY_DIR_Pin GPIO_PIN_2
#define RELAY_DIR_GPIO_Port GPIOD

/* ============================================================================ */
/*                         HARDWARE CONSTANTS                                   */
/* ============================================================================ */

/* PWM Configuration */
#define PWM_FREQUENCY_HZ        20000   /* 20 kHz PWM */
#define PWM_TIMER_CLOCK_HZ      170000000UL /* 170 MHz */
#define PWM_PRESCALER           0       /* No prescaler */
#define PWM_PERIOD              8499    /* ARR = (170MHz / 20kHz) - 1 */

/* Encoder Configuration */
#define ENCODER_PPR             360     /* Pulses Per Revolution */
#define ENCODER_COUNTS_PER_REV  1440    /* Quadrature mode: 360 × 4 */
#define ENCODER_DEGREES_PER_CNT 0.25f   /* 360° / 1440 */
#define ENCODER_CENTER_OFFSET   32768   /* TIM2 CNT center value */
#define ENCODER_MAX_ANGLE       45.0f   /* Maximum steering angle (degrees) */
#define ENCODER_MAX_COUNTS      720     /* Maximum counts from center */

/* ADC Configuration */
#define ADC_RESOLUTION          4096    /* 12-bit ADC */
#define ADC_VREF                3.3f    /* Reference voltage */
#define ADC_SAMPLE_FREQ_HZ      200     /* 200 Hz sampling rate */

/* I2C Addresses */
#define TCA9548A_ADDR           0x70    /* I2C multiplexer */
#define INA226_ADDR_FL          0x40    /* Motor FL current sensor */
#define INA226_ADDR_FR          0x41    /* Motor FR current sensor */
#define INA226_ADDR_RL          0x44    /* Motor RL current sensor */
#define INA226_ADDR_RR          0x45    /* Motor RR current sensor */
#define INA226_ADDR_STEER       0x48    /* Steering motor current sensor */
#define INA226_ADDR_BATT        0x49    /* Main battery current sensor */

/* CAN Configuration */
#define CAN_BITRATE_KBPS        500     /* 500 kbps */
#define CAN_PRESCALER           20      /* 170 MHz / 20 = 8.5 MHz */
#define CAN_TIME_SEG1           13      /* Time segment 1 */
#define CAN_TIME_SEG2           3       /* Time segment 2 */
#define CAN_SYNC_JUMP_WIDTH     1       /* SJW */

/* Safety Thresholds */
#define TEMP_WARNING_C          60.0f   /* Temperature warning threshold */
#define TEMP_CRITICAL_C         80.0f   /* Temperature critical threshold */
#define CURRENT_CONTINUOUS_A    20.0f   /* Continuous current limit */
#define CURRENT_PEAK_A          30.0f   /* Peak current limit (<2s) */
#define CURRENT_CRITICAL_A      35.0f   /* Critical current limit */
#define BATTERY_LOW_V           20.0f   /* Low battery voltage */
#define BATTERY_CRITICAL_V      18.0f   /* Critical battery voltage */

/* ABS/TCS Configuration */
#define ABS_THRESHOLD_PERCENT   20      /* 20% slip triggers ABS */
#define TCS_THRESHOLD_PERCENT   15      /* 15% slip triggers TCS */

/* Timing */
#define WATCHDOG_TIMEOUT_MS     500     /* IWDG timeout */
#define CAN_HEARTBEAT_TIMEOUT_MS 250    /* CAN heartbeat timeout */
#define MAIN_LOOP_FREQ_HZ       100     /* 100 Hz main loop */
#define MAIN_LOOP_PERIOD_MS     10      /* 10 ms period */

/* ============================================================================ */
/*                         SYSTEM STATE                                         */
/* ============================================================================ */

typedef enum {
    SYSTEM_STATE_BOOT = 0,
    SYSTEM_STATE_STANDBY,
    SYSTEM_STATE_ACTIVE,
    SYSTEM_STATE_SAFE,
    SYSTEM_STATE_ERROR
} SystemState_t;

typedef enum {
    SHIFTER_FORWARD = 2,
    SHIFTER_NEUTRAL = 1,
    SHIFTER_REVERSE = 0,
    SHIFTER_ERROR = 0xFF
} ShifterState_t;

/* ============================================================================ */
/*                         GLOBAL VARIABLES                                     */
/* ============================================================================ */

/* Timer Handles */
extern TIM_HandleTypeDef htim1;  /* TIM1: Traction motors PWM */
extern TIM_HandleTypeDef htim2;  /* TIM2: Encoder quadrature */
extern TIM_HandleTypeDef htim8;  /* TIM8: Steering motor PWM */

/* Communication Handles */
extern FDCAN_HandleTypeDef hfdcan1;  /* FDCAN1: CAN bus */
extern I2C_HandleTypeDef hi2c1;      /* I2C1: INA226 sensors */
extern ADC_HandleTypeDef hadc1;      /* ADC1: Pedal analog */

/* Watchdog Handle */
extern IWDG_HandleTypeDef hiwdg;

/* System State */
extern volatile SystemState_t system_state;
extern volatile uint32_t system_tick_counter;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
