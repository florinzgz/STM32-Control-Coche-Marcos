/**
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
 */

#include "stm32g4xx_it.h"
#include "main.h"
#include "sensor_manager.h"
#include "safety_system.h"
#include "can_handler.h"
#include "encoder_reader.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim1, htim2, htim3, htim8;
extern I2C_HandleTypeDef hi2c1;

/* ---- Fault-indication LED helper ----
 * Ensures PA5 (LD2) is driveable even if MX_GPIO_Init() has not run,
 * then blinks LD2 forever at a rate determined by delay_count.
 * Approximate timing at SYSCLK = 170 MHz:
 *   800 000 → ~50 ms toggle (~10 Hz blink)
 * 4 000 000 → ~250 ms toggle (~2 Hz blink)                            */
static void Fault_BlinkLD2(uint32_t delay_count)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    PORT_LD2->MODER = (PORT_LD2->MODER
                       & ~(3U << (PIN_LD2_N * 2)))
                    | (1U << (PIN_LD2_N * 2));           /* PA5 = output */
    while (1) {
        PORT_LD2->ODR ^= PIN_LD2;
        for (volatile uint32_t d = 0; d < delay_count; d++) { __NOP(); }
    }
}

/* ---- Cortex-M4 core exceptions ---- */

/* Emergency motor shutdown — called from all fault handlers.
 * EN-first order: BSRR → MOE → CCR.
 *
 * 1) BSRR: Force all motor EN pins and relays LOW.  This is the
 *    single fastest action to guarantee Hi-Z on ALL BTS7960 drivers
 *    (including TIM3/steering which has no BREAK input).  EN=LOW
 *    makes the BTS7960 ignore PWM inputs immediately, stopping all
 *    current flow — critical in overcurrent / short-circuit faults
 *    where keeping FETs conducting (passive brake) would be unsafe.
 *
 * 2) MOE: Clear MOE on advanced timers → hardware-level shutdown of
 *    TIM1/TIM8 PWM outputs.  Defence-in-depth: even if EN were
 *    somehow re-asserted, timer outputs are forced to idle (LOW).
 *
 * 3) CCR: Zero all preload registers.  Third layer of defence: if
 *    MOE were ever re-enabled (debugger, errant code), duty = 0 %.
 *
 * Extracted into a shared function to guarantee all fault handlers
 * use the identical shutdown sequence and order.                     */
static inline void Fault_MotorShutdown(void)
{
    GPIOC->BSRR = (uint32_t)(PIN_EN_FL | PIN_EN_FR | PIN_EN_RL | PIN_EN_RR
                  | PIN_EN_STEER
                  | PIN_RELAY_TRAC | PIN_RELAY_STEER_PWR) << 16U;
    GPIOB->BSRR = (uint32_t)(PIN_RELAY_LED | PIN_RELAY_LED_REAR) << 16U;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1  = 0U;  TIM1->CCR2  = 0U;  /* FL: RPWM, LPWM → 0 */
    TIM1->CCR3  = 0U;  TIM1->CCR4  = 0U;  /* FR: RPWM, LPWM → 0 */
    TIM8->CCR1  = 0U;  TIM8->CCR2  = 0U;  /* RL: RPWM, LPWM → 0 */
    TIM8->CCR3  = 0U;  TIM8->CCR4  = 0U;  /* RR: RPWM, LPWM → 0 */
    TIM3->CCR1  = 0U;               /* RPWM_STEER → 0 */
    TIM3->CCR2  = 0U;               /* LPWM_STEER → 0 */
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    Fault_MotorShutdown();

    /* Rapid-blink status LED (~10 Hz) — indicates a CPU fault (distinct
     * from Error_Handler's ~2 Hz slow blink and the main-loop heartbeat). */
    Fault_BlinkLD2(800000U);            /* never returns */
}

void MemManage_Handler(void)
{
    Fault_MotorShutdown();
    Fault_BlinkLD2(800000U);
}

void BusFault_Handler(void)
{
    Fault_MotorShutdown();
    Fault_BlinkLD2(800000U);
}

void UsageFault_Handler(void)
{
    Fault_MotorShutdown();
    Fault_BlinkLD2(800000U);
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

void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(PIN_ENC_Z);
    EncoderZ_IRQHandler();
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(PIN_WHEEL_RR);
    Wheel_RR_IRQHandler();
}

void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(PIN_STEER_CENTER);
    SteeringCenter_IRQHandler();
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);
}

void TIM8_UP_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim8);
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

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)hfdcan;  /* Single FDCAN instance — handle not used */

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        /* Visual feedback: toggle LD2 on every received CAN frame.
         * Note: LD2 is also toggled by the main-loop heartbeat;
         * on CAN-active buses the combined pattern will differ from the
         * steady heartbeat blink, providing a visual indication of bus
         * activity.
         *
         * IMPORTANT: Do NOT call HAL_FDCAN_GetRxMessage() here.
         * The message must remain in FIFO0 so that the main-loop
         * CAN_ProcessMessages() can read and process it (heartbeat
         * liveness, throttle commands, steering, etc.).  Reading
         * here would consume the message before the main loop sees
         * it, causing silent loss of all ESP32 commands.            */
        HAL_GPIO_TogglePin(PORT_LD2, PIN_LD2);
    }
}