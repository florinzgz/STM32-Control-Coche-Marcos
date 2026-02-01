/**
  ****************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ****************************************************************************
  * @attention
  *
  * Copyright (c) 2026 florinzgz - STM32-Control-Coche-Marcos
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  *
  ****************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

// Firmware Version
#define FIRMWARE_VERSION_MAJOR  1
#define FIRMWARE_VERSION_MINOR  0
#define FIRMWARE_VERSION_PATCH  0

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/

// ============================================================================
// MOTORES DE TRACCIÓN (4x - Control Directo TIM1)
// ============================================================================

// Motor FL (Front Left - Delantero Izquierdo)
#define PWM_FL_Pin       GPIO_PIN_8
#define PWM_FL_GPIO_Port GPIOA
#define DIR_FL_Pin       GPIO_PIN_0
#define DIR_FL_GPIO_Port GPIOC
#define EN_FL_Pin        GPIO_PIN_1
#define EN_FL_GPIO_Port  GPIOC

// Motor FR (Front Right - Delantero Derecho)
#define PWM_FR_Pin       GPIO_PIN_9
#define PWM_FR_GPIO_Port GPIOA
#define DIR_FR_Pin       GPIO_PIN_2
#define DIR_FR_GPIO_Port GPIOC
#define EN_FR_Pin        GPIO_PIN_3
#define EN_FR_GPIO_Port  GPIOC

// Motor RL (Rear Left - Trasero Izquierdo)
#define PWM_RL_Pin       GPIO_PIN_10
#define PWM_RL_GPIO_Port GPIOA
#define DIR_RL_Pin       GPIO_PIN_4
#define DIR_RL_GPIO_Port GPIOC
#define EN_RL_Pin        GPIO_PIN_5
#define EN_RL_GPIO_Port GPIOC

// Motor RR (Rear Right - Trasero Derecho)
#define PWM_RR_Pin       GPIO_PIN_11
#define PWM_RR_GPIO_Port GPIOA
#define DIR_RR_Pin       GPIO_PIN_6
#define DIR_RR_GPIO_Port GPIOC
#define EN_RR_Pin        GPIO_PIN_7
#define EN_RR_GPIO_Port  GPIOC

// ============================================================================
// MOTOR DE DIRECCIÓN (1x - Control Directo TIM8)
// ============================================================================

#define PWM_STEER_Pin       GPIO_PIN_8
#define PWM_STEER_GPIO_Port GPIOC
#define DIR_STEER_Pin       GPIO_PIN_9
#define DIR_STEER_GPIO_Port GPIOC
#define EN_STEER_Pin        GPIO_PIN_10
#define EN_STEER_GPIO_Port  GPIOC

// ============================================================================
// ENCODER DE DIRECCIÓN (E6B2-CWZ6C - 360 PPR - TIM2 Quadrature)
// ============================================================================

#define ENC_A_Pin       GPIO_PIN_15
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin       GPIO_PIN_3
#define ENC_B_GPIO_Port GPIOB
#define ENC_Z_Pin       GPIO_PIN_4
#define ENC_Z_GPIO_Port GPIOB
#define ENC_Z_EXTI_IRQn EXTI4_IRQn

// ============================================================================
// SENSORES DE RUEDA (4x - GPIO + EXTI)
// ============================================================================

#define WHEEL_FL_Pin       GPIO_PIN_0
#define WHEEL_FL_GPIO_Port GPIOB
#define WHEEL_FR_Pin       GPIO_PIN_1
#define WHEEL_FR_GPIO_Port GPIOB
#define WHEEL_RL_Pin       GPIO_PIN_2
#define WHEEL_RL_GPIO_Port GPIOB
#define WHEEL_RR_Pin       GPIO_PIN_10
#define WHEEL_RR_GPIO_Port GPIOB

// ============================================================================
// I2C (INA226 x6 vía TCA9548A)
// ============================================================================

#define I2C_SCL_Pin       GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin       GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB

// ============================================================================
// CAN BUS (FDCAN1 @ 500 kbps)
// ============================================================================

#define CAN_RX_Pin       GPIO_PIN_8
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin       GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB

// ============================================================================
// SHIFTER (F/N/R - 3 posiciones)
// ============================================================================

#define SHIFTER_FWD_Pin       GPIO_PIN_12
#define SHIFTER_FWD_GPIO_Port GPIOB
#define SHIFTER_NEU_Pin       GPIO_PIN_13
#define SHIFTER_NEU_GPIO_Port GPIOB
#define SHIFTER_REV_Pin       GPIO_PIN_14
#define SHIFTER_REV_GPIO_Port GPIOB

// ============================================================================
// LLAVE DE CONTACTO / POWER-HOLD
// ============================================================================

#define KEY_ON_Pin       GPIO_PIN_15
#define KEY_ON_GPIO_Port GPIOB
#define KEY_ON_EXTI_IRQn EXTI15_IRQn

// ============================================================================
// RELÉS DE POTENCIA (3x - Default LOW)
// ============================================================================

#define RELAY_MAIN_Pin       GPIO_PIN_11
#define RELAY_MAIN_GPIO_Port GPIOC
#define RELAY_TRAC_Pin       GPIO_PIN_12
#define RELAY_TRAC_GPIO_Port GPIOC
#define RELAY_DIR_Pin        GPIO_PIN_2
#define RELAY_DIR_GPIO_Port  GPIOD

// ============================================================================
// TEMPERATURA (OneWire - 5 DS18B20)
// ============================================================================

#define TEMP_ONEWIRE_Pin       GPIO_PIN_5
#define TEMP_ONEWIRE_GPIO_Port GPIOB

// ============================================================================
// PEDAL ANALÓGICO (ADC1)
// ============================================================================

#define PEDAL_ANALOG_Pin       GPIO_PIN_0
#define PEDAL_ANALOG_GPIO_Port GPIOA

// ============================================================================
// DEBUG (SWD - NO USAR PARA OTRO PROPÓSITO)
// ============================================================================

#define SWDIO_Pin       GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin       GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */