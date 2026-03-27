/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* USER CODE BEGIN Includes */
#include "project_config.h"
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
void SystemClock_Config(void);
uint8_t Boot_GetResetCause(void);
bool    Startup_IsInhibited(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* CubeMX regeneration note:
 * Pin definitions (STEER_CENTER_Pin, WHEEL_FL_Pin, etc.) are NOT placed here.
 * All project-specific pin macros live in Core/Inc/project_config.h, which is
 * included above via "USER CODE BEGIN Includes".  This strategy ensures that
 * a CubeMX code-generation run never overwrites the pin mapping.  CubeMX may
 * add its own *_Pin / *_GPIO_Port defines between this comment and the USER
 * CODE marker below; those are harmless but unused — the firmware references
 * the PIN_* macros from project_config.h exclusively.                        */
/* USER CODE BEGIN Private defines */

/* ---- Global HAL handles ---- */
extern ADC_HandleTypeDef hadc1;
extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1, htim2, htim3, htim8, htim15;
extern IWDG_HandleTypeDef hiwdg;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
