/* Interrupt Service Routines */

#include "stm32g4xx_it.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;
extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;

void NMI_Handler(void) { }
void HardFault_Handler(void) { while (1) { } }
void MemManage_Handler(void) { while (1) { } }
void BusFault_Handler(void) { while (1) { } }
void UsageFault_Handler(void) { while (1) { } }
void SVC_Handler(void) { }
void DebugMon_Handler(void) { }
void PendSV_Handler(void) { }
void SysTick_Handler(void) { HAL_IncTick(); }
void EXTI4_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4); }
void EXTI15_10_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15); HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10); }
void TIM1_UP_IRQHandler(void) { HAL_TIM_IRQHandler(&htim1); }
void TIM2_IRQHandler(void) { HAL_TIM_IRQHandler(&htim2); }
void TIM8_UP_IRQHandler(void) { HAL_TIM_IRQHandler(&htim8); }
void FDCAN1_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&hfdcan1); }
void FDCAN1_IT1_IRQHandler(void) { HAL_FDCAN_IRQHandler(&hfdcan1); }
void I2C1_EV_IRQHandler(void) { HAL_I2C_EV_IRQHandler(&hi2c1); }
void I2C1_ER_IRQHandler(void) { HAL_I2C_ER_IRQHandler(&hi2c1); }
void ADC1_2_IRQHandler(void) { HAL_ADC_IRQHandler(&hadc1); }