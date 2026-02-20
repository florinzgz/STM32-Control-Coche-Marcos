#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "vehicle_physics.h"
#include <stdbool.h>

/* ---- ENCODER DE DIRECCIÓN (E6B2-CWZ6C - 1200 PPR - TIM2 Quadrature) ---- */
/* Quadrature mode: 1200 PPR × 4 = 4800 counts/revolution                    */
/* Resolution: 360° / 4800 = 0.075° per count                                */
#define ENCODER_PPR        1200
#define ENCODER_CPR        (ENCODER_PPR * 4)  /* 4800 counts/rev */

/* ---- PWM Motor Pins (TIM1 CH1-CH4: PA8-PA11, TIM8 CH3: PC8) ---- */
#define PIN_PWM_FL         GPIO_PIN_8   /* PA8  - TIM1_CH1 */
#define PIN_PWM_FR         GPIO_PIN_9   /* PA9  - TIM1_CH2 */
#define PIN_PWM_RL         GPIO_PIN_10  /* PA10 - TIM1_CH3 */
#define PIN_PWM_RR         GPIO_PIN_11  /* PA11 - TIM1_CH4 */
#define PIN_PWM_STEER      GPIO_PIN_8   /* PC8  - TIM8_CH3 */

/* ---- Direction Control (GPIOC) ---- */
#define PIN_DIR_FL         GPIO_PIN_0   /* PC0 */
#define PIN_DIR_FR         GPIO_PIN_1   /* PC1 */
#define PIN_DIR_RL         GPIO_PIN_2   /* PC2 */
#define PIN_DIR_RR         GPIO_PIN_3   /* PC3 */
#define PIN_DIR_STEER      GPIO_PIN_4   /* PC4 */

/* ---- Enable Signals (GPIOC) ---- */
#define PIN_EN_FL          GPIO_PIN_5   /* PC5 */
#define PIN_EN_FR          GPIO_PIN_6   /* PC6 */
#define PIN_EN_RL          GPIO_PIN_7   /* PC7 */
#define PIN_EN_RR          GPIO_PIN_13  /* PC13 */
#define PIN_EN_STEER       GPIO_PIN_9   /* PC9 */

/* ---- Relay Control (GPIOC) ---- */
#define PIN_RELAY_MAIN     GPIO_PIN_10  /* PC10 */
#define PIN_RELAY_TRAC     GPIO_PIN_11  /* PC11 */
#define PIN_RELAY_DIR      GPIO_PIN_12  /* PC12 */

/* ---- Wheel Speed Sensors (EXTI) ---- */
#define PIN_WHEEL_FL       GPIO_PIN_0   /* PA0 - EXTI0 */
#define PIN_WHEEL_FR       GPIO_PIN_1   /* PA1 - EXTI1 */
#define PIN_WHEEL_RL       GPIO_PIN_2   /* PA2 - EXTI2 */
#define PIN_WHEEL_RR       GPIO_PIN_15  /* PB15 - EXTI15 */

/* ---- Steering Encoder (TIM2 Quadrature) ---- */
#define PIN_ENC_A          GPIO_PIN_15  /* PA15 - TIM2_CH1 */
#define PIN_ENC_B          GPIO_PIN_3   /* PB3  - TIM2_CH2 */
#define PIN_ENC_Z          GPIO_PIN_4   /* PB4  - EXTI4 (index pulse) */

/* ---- Steering Center Inductive Sensor ---- */
/* LJ12A3-type inductive proximity sensor detecting a physical screw
 * placed at the mechanical center of the steering rack.               */
#define PIN_STEER_CENTER   GPIO_PIN_5   /* PB5 - EXTI5 */

/* ---- I2C Bus (INA226 via TCA9548A, DS18B20) ---- */
#define PIN_I2C_SCL        GPIO_PIN_6   /* PB6 - I2C1_SCL */
#define PIN_I2C_SDA        GPIO_PIN_7   /* PB7 - I2C1_SDA */

/* ---- OneWire Bus (DS18B20 temperatures) ---- */
#define PIN_ONEWIRE        GPIO_PIN_0   /* PB0 */

/* ---- Pedal Accelerator (dual-channel redundant reading) ---- */
/* Primary: internal ADC1 on PA3 via voltage divider (5V→3.3V)
 * Plausibility: ADS1115 16-bit I2C ADC reading full 5V range   */
#define PIN_PEDAL          GPIO_PIN_3   /* PA3 - ADC1_IN4 */

/* ---- ADS1115 Pedal ADC (I2C plausibility channel) ---- */
/* The Hall-effect pedal (SS1324LUA-T) operates at 5V (0.3V–4.8V output).
 * A voltage divider (10 kΩ + 6.8 kΩ) scales the signal to 0–3.3V for
 * the STM32 internal ADC (primary/fast channel).  The ADS1115 reads
 * the unscaled 5V signal as a plausibility cross-check.
 * ADS1115 ADDR pin → GND gives I2C address 0x48.                       */
#define I2C_ADDR_ADS1115   0x48

/* ---- CAN Bus (FDCAN1 on PB8/PB9 per HAL MSP) ---- */
#define PIN_CAN_TX         GPIO_PIN_9   /* PB9 - FDCAN1_TX (AF9) */
#define PIN_CAN_RX         GPIO_PIN_8   /* PB8 - FDCAN1_RX (AF9) */

/* ---- I2C Addresses ---- */
#define I2C_ADDR_TCA9548A  0x70
#define I2C_ADDR_INA226    0x40
#define NUM_INA226         6
#define NUM_DS18B20        5
#define NUM_WHEELS         4
#define WHEEL_PULSES_REV   6   /* 6 bolts per wheel revolution */

/* ---- Sensor Constants ---- */
/* INA226 shunt resistors (mΩ) — values per hardware BOM:
 * Channels 0-3 (motor wheels): 50A sensors with 1 mΩ shunt
 *   → placed BEFORE each BTS7960 driver (between relay output and B+ input)
 * Channel 4 (battery 24V):     100A sensor with 0.5 mΩ shunt
 *   → placed BEFORE the main relay (between battery + terminal and relay input)
 *     so that Voltage_GetBus() always reads battery voltage even when relay is open
 * Channel 5 (steering motor):  50A sensor with 1 mΩ shunt
 *   → placed BEFORE the steering BTS7960 driver (between relay output and B+ input) */
#define INA226_SHUNT_MOHM_MOTOR    1      /* 1 mΩ for 50A sensors    */
#define INA226_SHUNT_MOHM_BATTERY  0.5f  /* 0.5 mΩ for 100A sensor  */
#define INA226_CHANNEL_BATTERY     4    /* TCA9548A channel index  */

/* ---- Global HAL handles ---- */
extern ADC_HandleTypeDef hadc1;
extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1, htim2, htim8;
extern IWDG_HandleTypeDef hiwdg;

void Error_Handler(void);
void SystemClock_Config(void);
uint8_t Boot_GetResetCause(void);
bool    Startup_IsInhibited(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
