/**
  ****************************************************************************
  * @file    steering_output.c
  * @brief   Shared production shutdown of the physical steering-assist
  *          actuator.  See steering_output.h for the rationale.
  *
  * This is the single, canonical low-level coast of the steering H-bridge.
  * Keeping it in its own tiny translation unit lets both production callers
  * (motor_control.c, steering_eps.c) and the host tests link the EXACT same
  * register writes, so tests verify real production behaviour rather than a
  * re-implemented wrapper.
  ****************************************************************************
  */

#include "steering_output.h"
#include "main.h"            /* htim3 handle, HAL GPIO/TIM helpers */
#include "project_config.h"  /* PIN_EN_STEER (PC4) */

/* TIM3 drives the steering assist PWM: CH1 = PA6 (RPWM), CH2 = PA7 (LPWM). */
extern TIM_HandleTypeDef htim3;

void Steering_PhysicalOff(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);   /* PA6 CCR = 0 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);   /* PA7 CCR = 0 */
    HAL_GPIO_WritePin(GPIOC, PIN_EN_STEER, GPIO_PIN_RESET);   /* PC4 = LOW */
}
