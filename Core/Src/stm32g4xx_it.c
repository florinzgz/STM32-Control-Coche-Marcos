/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_it.h"
#include "main.h"
#include "motor_control.h"
#include "sensor_manager.h"
#include "can_handler.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1) {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  
  /* Capture fault information for debugging */
  volatile uint32_t *hardfault_args;
  volatile uint32_t stacked_r0;
  volatile uint32_t stacked_r1;
  volatile uint32_t stacked_r2;
  volatile uint32_t stacked_r3;
  volatile uint32_t stacked_r12;
  volatile uint32_t stacked_lr;
  volatile uint32_t stacked_pc;
  volatile uint32_t stacked_psr;
  volatile uint32_t cfsr;
  volatile uint32_t hfsr;
  volatile uint32_t dfsr;
  volatile uint32_t afsr;
  volatile uint32_t bfar;
  volatile uint32_t mmar;
  
  /* Get stack pointer */
  __asm volatile (
    "TST LR, #4            \n"
    "ITE EQ                \n"
    "MRSEQ R0, MSP         \n"
    "MRSNE R0, PSP         \n"
    "B HardFault_Handler_C \n"
  );
  
  /* Read stacked registers */
  hardfault_args = (uint32_t *)__get_MSP();
  
  stacked_r0 = ((uint32_t)hardfault_args[0]);
  stacked_r1 = ((uint32_t)hardfault_args[1]);
  stacked_r2 = ((uint32_t)hardfault_args[2]);
  stacked_r3 = ((uint32_t)hardfault_args[3]);
  stacked_r12 = ((uint32_t)hardfault_args[4]);
  stacked_lr = ((uint32_t)hardfault_args[5]);
  stacked_pc = ((uint32_t)hardfault_args[6]);
  stacked_psr = ((uint32_t)hardfault_args[7]);
  
  /* Read fault status registers */
  cfsr = (*((volatile uint32_t *)(0xE000ED28)));
  hfsr = (*((volatile uint32_t *)(0xE000ED2C)));
  dfsr = (*((volatile uint32_t *)(0xE000ED30)));
  afsr = (*((volatile uint32_t *)(0xE000ED3C)));
  bfar = (*((volatile uint32_t *)(0xE000ED38)));
  mmar = (*((volatile uint32_t *)(0xE000ED34)));
  
  /* Prevent compiler optimization */
  (void)stacked_r0;
  (void)stacked_r1;
  (void)stacked_r2;
  (void)stacked_r3;
  (void)stacked_r12;
  (void)stacked_lr;
  (void)stacked_pc;
  (void)stacked_psr;
  (void)cfsr;
  (void)hfsr;
  (void)dfsr;
  (void)afsr;
  (void)bfar;
  (void)mmar;
  
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* Infinite loop in fault state */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt (Wheel FL sensor).
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(WHEEL_FL_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt (Wheel FR sensor).
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(WHEEL_FR_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt (Wheel RL sensor).
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(WHEEL_RL_Pin);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt (Encoder Z pulse).
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ENC_Z_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts (Wheel RR sensor).
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(WHEEL_RR_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt (Encoder update).
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0 (RX FIFO 0).
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief GPIO EXTI callback (called by HAL for all EXTI interrupts)
  * @param GPIO_Pin: Pin that triggered the interrupt
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Handle wheel sensor interrupts */
  if (GPIO_Pin == WHEEL_FL_Pin) {
    Sensors_WheelPulseCallback(0);  /* Front Left */
  }
  else if (GPIO_Pin == WHEEL_FR_Pin) {
    Sensors_WheelPulseCallback(1);  /* Front Right */
  }
  else if (GPIO_Pin == WHEEL_RL_Pin) {
    Sensors_WheelPulseCallback(2);  /* Rear Left */
  }
  else if (GPIO_Pin == WHEEL_RR_Pin) {
    Sensors_WheelPulseCallback(3);  /* Rear Right */
  }
  /* Handle encoder Z pulse interrupt */
  else if (GPIO_Pin == ENC_Z_Pin) {
    encoder_z_detected = 1;
    
    /* Reset encoder counter to center when Z pulse is detected */
    __HAL_TIM_SET_COUNTER(&htim2, ENCODER_CENTER_OFFSET);
  }
}

/* USER CODE END 1 */
