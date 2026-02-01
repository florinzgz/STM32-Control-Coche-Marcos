/**
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
 */

#include "stm32g4xx_it.h"
#include "main.h"
#include "Safety.h"

void NMI_Handler(void) {
    // Handle Non-Maskable Interrupt
}

void HardFault_Handler(void) {
    // Handle Hard Fault
    while (1) {}
}

void SysTick_Handler(void) {
    // Handle SysTick interrupt
    HAL_IncTick();
}

void FDCAN1_IT0_IRQHandler(void) {
    // Handle FDCAN1 Interrupt 0
}

void FDCAN1_IT1_IRQHandler(void) {
    // Handle FDCAN1 Interrupt 1
}

void EXTI0_IRQHandler(void) {
    // Handle external interrupt for wheel FL sensor
}

void EXTI1_IRQHandler(void) {
    // Handle external interrupt for wheel FR sensor
}

void EXTI2_IRQHandler(void) {
    // Handle external interrupt for wheel RL sensor
}

void EXTI3_IRQHandler(void) {
    // Handle external interrupt for wheel RR sensor
}

void TIM1_UP_TIM16_IRQHandler(void) {
    // Handle TIM1 Update and TIM16 interrupt
}

void TIM2_IRQHandler(void) {
    // Handle TIM2 interrupt for encoder
}

void I2C1_EV_IRQHandler(void) {
    // Handle I2C1 event interrupt
}

void I2C1_ER_IRQHandler(void) {
    // Handle I2C1 error interrupt
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan) {
    Safety_UpdateCANRxTime();
}