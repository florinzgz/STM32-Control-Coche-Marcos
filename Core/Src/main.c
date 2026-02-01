/**
  ****************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ****************************************************************************
  */

#include "main.h"
#include "motor_control.h"
#include "can_handler.h"
#include "sensor_manager.h"
#include "safety_system.h"
#include "stm32g4xx_it.h"

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;
ADC_HandleTypeDef hadc1;
IWDG_HandleTypeDef hiwdg;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  
  Motor_Init();
  CAN_Init();
  Sensor_Init();
  Safety_Init();
  
  HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(RELAY_TRAC_GPIO_Port, RELAY_TRAC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RELAY_DIR_GPIO_Port, RELAY_DIR_Pin, GPIO_PIN_SET);
  
  uint32_t last_heartbeat = 0;
  uint32_t last_sensor_read = 0;
  uint32_t last_safety_check = 0;
  
  while (1)
  {
    uint32_t now = HAL_GetTick();
    
    if (now - last_heartbeat >= 100) {
      CAN_SendHeartbeat();
      last_heartbeat = now;
    }
    
    if (now - last_sensor_read >= 50) {
      Pedal_Update();
      Current_ReadAll();
      last_sensor_read = now;
    }
    
    if (now - last_safety_check >= 10) {
      Safety_CheckCurrent();
      Safety_CheckTemperature();
      Safety_CheckCANTimeout();
      ABS_Update();
      TCS_Update();
      Steering_ControlLoop();
      last_safety_check = now;
    }
    
    CAN_ProcessMessages();
    HAL_IWDG_Refresh(&hiwdg);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  HAL_GPIO_WritePin(GPIOC, DIR_FL_Pin|DIR_FR_Pin|DIR_RL_Pin|DIR_RR_Pin|DIR_STEER_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, EN_FL_Pin|EN_FR_Pin|EN_RL_Pin|EN_RR_Pin|EN_STEER_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, RELAY_MAIN_Pin|RELAY_TRAC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RELAY_DIR_GPIO_Port, RELAY_DIR_Pin, GPIO_PIN_RESET);
  
  GPIO_InitStruct.Pin = DIR_FL_Pin|DIR_FR_Pin|DIR_RL_Pin|DIR_RR_Pin|DIR_STEER_Pin|EN_FL_Pin|EN_FR_Pin|EN_RL_Pin|EN_RR_Pin|EN_STEER_Pin|RELAY_MAIN_Pin|RELAY_TRAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = RELAY_DIR_Pin;
  HAL_GPIO_Init(RELAY_DIR_GPIO_Port, &GPIO_InitStruct);
}

static void MX_FDCAN1_Init(void) { }
static void MX_I2C1_Init(void) { }
static void MX_TIM1_Init(void) { }
static void MX_TIM2_Init(void) { }
static void MX_TIM8_Init(void) { }
static void MX_ADC1_Init(void) { }
static void MX_IWDG_Init(void) { }

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}