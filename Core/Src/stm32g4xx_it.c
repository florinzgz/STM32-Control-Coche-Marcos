/**
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
 */

#include "stm32g4xx_it.h"
#include "main.h"
#include "sensor_manager.h"
#include "safety_system.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim1, htim2;
extern I2C_HandleTypeDef hi2c1;

/* ---- Cortex-M4 core exceptions ---- */

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1) { }
}

void MemManage_Handler(void)
{
    while (1) { }
}

void BusFault_Handler(void)
{
    while (1) { }
}

void UsageFault_Handler(void)
{
    while (1) { }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ---- Peripheral interrupts ---- */

void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

void FDCAN1_IT1_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(PIN_WHEEL_FL);
    Wheel_FL_IRQHandler();
}

void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(PIN_WHEEL_FR);
    Wheel_FR_IRQHandler();
}

void EXTI2_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(PIN_WHEEL_RL);
    Wheel_RL_IRQHandler();
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(PIN_WHEEL_RR);
    Wheel_RR_IRQHandler();
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c1);
}

/* ---- HAL Callbacks ---- */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan)
{
    Safety_UpdateCANRxTime();
}