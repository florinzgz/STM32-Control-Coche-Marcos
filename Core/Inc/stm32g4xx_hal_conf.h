/* STM32G4 HAL configuration header file */

#ifndef STM32G4XX_HAL_CONF_H
#define STM32G4XX_HAL_CONF_H

/* HSE_VALUE and HSI_VALUE */
#define HSE_VALUE 8000000U
#define HSI_VALUE 16000000U

/* System Clock configuration */
#define SYSTEM_CLOCK 170000000U

/* Include HAL drivers */
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_can.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_i2c.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_dma.h"
#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_pwr.h"
#include "stm32g4xx_hal_cortex.h"

/* Module selections */
#define USE_HAL_ADC
#define USE_HAL_CAN
#define USE_HAL_TIM
#define USE_HAL_I2C
#define USE_HAL_UART
#define USE_HAL_GPIO
#define USE_HAL_DMA
#define USE_HAL_RCC
#define USE_HAL_FLASH
#define USE_HAL_PWR
#define USE_HAL_CORTEX

#endif /* STM32G4XX_HAL_CONF_H */