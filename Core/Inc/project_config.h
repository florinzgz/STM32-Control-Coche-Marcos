/**
  ******************************************************************************
  * @file    project_config.h
  * @brief   Central project configuration — NOT managed by STM32CubeMX.
  *
  *          This file collects every project-specific macro (pin mappings,
  *          sensor constants, I2C addresses, vehicle geometry, etc.) so that
  *          a CubeMX code re-generation never overwrites them.
  *
  *          Include chain:
  *            main.h  →  project_config.h  →  vehicle_physics.h
  *
  *          Any application source that includes main.h (directly or through
  *          its own header) automatically gets these definitions.
  ******************************************************************************
  */

#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

/* ---- Vehicle geometry & physics (separate header for reuse) ---- */
#include "vehicle_physics.h"

/* ========================================================================== */
/*                       ENCODER DE DIRECCIÓN                                 */
/* ========================================================================== */
/* E6B2-CWZ6C — 1200 PPR — TIM2 Quadrature                                   */
/* Quadrature mode: 1200 PPR × 4 = 4800 counts/revolution                    */
/* Resolution: 360° / 4800 = 0.075° per count                                */
#define ENCODER_PPR        1200
#define ENCODER_CPR        (ENCODER_PPR * 4)  /* 4800 counts/rev */

/* ========================================================================== */
/*                       MOTOR PWM PIN MAPPING                                */
/* ========================================================================== */

/* ---- TIM1 (advanced): FL motor CH1/CH2, FR motor RPWM CH3 + LPWM CH4 ----
 * RPWM and LPWM of FL share the SAME timer so both channels update
 * at the same UEV → overlap = 0.  TIM1 BREAK2 is armed to Cortex LOCKUP.
 * FR motor RPWM is on TIM1_CH3 (PA10); LPWM_FR is on TIM1_CH4 (PC3).
 * PA11 was reassigned to FDCAN1_RX, so LPWM_FR was moved off TIM1_CH4/PA11.
 * PC3 provides TIM1_CH4 via AF2 — both RPWM_FR and LPWM_FR are now on
 * the same timer (TIM1), ensuring perfect UEV synchronisation (overlap = 0)
 * and automatic BREAK2/LOCKUP protection.
 * Since only one of RPWM/LPWM is active at any time (BTS7960 H-bridge),
 * Motor_SetSigned() always zeroes the inactive channel first.
 *
 * ---- BTS7960 (IBT-2) Logic Level Compatibility ----
 * STM32G474RE outputs 3.3V GPIO / timer signals.  BTS7960 IC datasheet
 * specifies V_IH(min) = 2.0V for INH (EN) and IN (PWM) inputs, so 3.3V is
 * above the BTS7960 IC threshold.  However, the IBT-2 module includes a
 * 74HC244 octal buffer (confirmed by Handsontec official documentation) that
 * sits between the input header and the BTS7960 ICs.  The 74HC244 V_IH(min)
 * = 0.7 × VCC: at VCC=5V → 3.5V (3.3V is BELOW threshold); at VCC=3.3V →
 * 2.31V (3.3V is safely above).
 * ⚠ SOLUTION: This design powers IBT-2 VCC from the STM32 3.3V rail, NOT 5V.
 * This ensures the 74HC244 V_IH(min) = 2.31V, well below the 3.3V signal
 * level from the MCU.  If VCC must be 5V, add BSS138 level shifters or tie
 * EN directly to 5V and control direction via RPWM/LPWM only.
 *
 * ---- BTS7960 R_IS / L_IS Current Sense (NOT USED) ----
 * The BTS7960 provides analog current sense outputs (R_IS, L_IS) with a ratio
 * of ~8500:1 (I_IS = I_LOAD / 8500).  These are NOT connected to the STM32
 * ADC in this design.  Instead, external INA226 sensors on the I2C bus provide
 * current monitoring with higher accuracy and digital readout.
 * ⚠ NOTE: The INA226 read cycle (~2ms at 400kHz I2C) is slower than the
 * BTS7960 analog current sense (~1µs response).  For the fastest possible
 * hardware-level overcurrent protection, consider connecting R_IS/L_IS to
 * STM32 ADC channels with analog watchdog (AWD) interrupt for sub-microsecond
 * overcurrent detection as an additional defence layer.                       */
#define PIN_PWM_FL         GPIO_PIN_8   /* PA8  - TIM1_CH1 — RPWM_FL  */
#define PIN_LPWM_FL        GPIO_PIN_9   /* PA9  - TIM1_CH2 — LPWM_FL  */
#define PIN_PWM_FR         GPIO_PIN_10  /* PA10 - TIM1_CH3 — RPWM_FR  */
#define PIN_LPWM_FR        GPIO_PIN_3   /* PC3  - TIM1_CH4 — LPWM_FR */

/* ---- TIM8 (advanced): RL motor CH1/CH2, RR motor CH3/CH4 ----
 * Same-timer guarantee and BREAK2/LOCKUP protection as TIM1.                 */
#define PIN_PWM_RL         GPIO_PIN_6   /* PC6  - TIM8_CH1 — RPWM_RL  */
#define PIN_LPWM_RL        GPIO_PIN_7   /* PC7  - TIM8_CH2 — LPWM_RL  */
#define PIN_PWM_RR         GPIO_PIN_8   /* PC8  - TIM8_CH3 — RPWM_RR  */
#define PIN_LPWM_RR        GPIO_PIN_9   /* PC9  - TIM8_CH4 — LPWM_RR  */

/* ---- TIM3 (general-purpose): STEER motor CH1/CH2 ----
 * Same-timer guarantee.  TIM3 has no BREAK input; fault handlers zero
 * CCR1/CCR2 via direct register access.                                      */
#define PIN_PWM_STEER      GPIO_PIN_6   /* PA6  - TIM3_CH1 — RPWM_STEER */
#define PIN_LPWM_STEER     GPIO_PIN_7   /* PA7  - TIM3_CH2 — LPWM_STEER */

/* ========================================================================== */
/*                       ENABLE SIGNALS (GPIOC)                               */
/* ========================================================================== */
/* All five BTS7960 modules now have dedicated GPIO Enable pins.
 * PC0, PC1, PC4 were formerly direction-control outputs (PIN_DIR_*) that
 * became unused when RPWM/LPWM direct generation replaced DIR+PWM logic.
 * They are now repurposed as EN_FR, EN_RL, EN_STEER respectively, giving
 * every motor driver identical 5-wire control (RPWM, LPWM, EN, GND, VCC)
 * and eliminating the brake/coast asymmetry between GPIO-EN and tied-HIGH
 * motors.  PC2 and PC3 remain freed/unused.
 *
 * ⚠ INIT SAFETY: All EN pins are forced LOW in MX_GPIO_Init() (main.c)
 *   via GPIOC->BSRR atomic write BEFORE the GPIO is configured as output.
 *   This guarantees no transient motor activation after warm reset or
 *   watchdog reset where ODR may retain its previous value.
 *   No EN pin is left floating — all are configured as push-pull output
 *   with no pull resistor (the push-pull driver actively holds the level). */
#define PIN_EN_FR          GPIO_PIN_0   /* PC0  — GPIO output, active HIGH (was DIR_FL) */
#define PIN_EN_RL          GPIO_PIN_1   /* PC1  — GPIO output, active HIGH (was DIR_FR) */
#define PIN_EN_FL          GPIO_PIN_5   /* PC5  — GPIO output, active HIGH */
#define PIN_EN_RR          GPIO_PIN_13  /* PC13 — GPIO output, active HIGH */
#define PIN_EN_STEER       GPIO_PIN_4   /* PC4  — GPIO output, active HIGH (was DIR_STEER) */

/* ========================================================================== */
/*                       RELAY CONTROL (GPIOC)                                */
/* ========================================================================== */
/* Relay GPIO pins are configured as push-pull output, no pull, LOW at init.
 * All relays default OPEN (fail-safe).  Power-up is managed by the
 * non-blocking relay sequencer in safety_system.c.
 *
 * ⚠ See RELAY TIMING OWNERSHIP doc block in safety_system.c for details on
 *   external delay relay module constraints and Safety_IsPowerReady() gate. */
#define PIN_RELAY_MAIN     GPIO_PIN_10  /* PC10 */
#define PIN_RELAY_TRAC     GPIO_PIN_11  /* PC11 */
#define PIN_RELAY_DIR      GPIO_PIN_12  /* PC12 */

/* ========================================================================== */
/*                       LED POWER RELAYS (GPIOB)                             */
/* ========================================================================== */
/* Front relay (PB10): controls 5V supply to front WS2812B LED strip (28 LEDs).
 * Rear  relay (PB11): controls 5V supply to rear  WS2812B LED strip (16 LEDs).
 * The ESP32 drives the WS2812B data lines; the STM32 controls the
 * power relays for safety cutoff.  Toggled via CAN command 0x120.
 *   Byte 0 = front relay, Byte 1 = rear relay.                              */
#define PIN_RELAY_LED       GPIO_PIN_10  /* PB10 — front LED strip relay */
#define PIN_RELAY_LED_REAR  GPIO_PIN_11  /* PB11 — rear  LED strip relay */

/* ========================================================================== */
/*                       WHEEL SPEED SENSORS (EXTI)                           */
/* ========================================================================== */
#define PIN_WHEEL_FL       GPIO_PIN_0   /* PA0 - EXTI0 */
#define PIN_WHEEL_FR       GPIO_PIN_1   /* PA1 - EXTI1 */
#define PIN_WHEEL_RL       GPIO_PIN_2   /* PA2 - EXTI2 */
#define PIN_WHEEL_RR       GPIO_PIN_15  /* PB15 - EXTI15 */

/* Software debounce: minimum ms between accepted EXTI pulses per wheel.
 * At 25 km/h (max plausible speed), 1.1 m circumference, 6 pulses/rev:
 *   freq = (25/3.6)/1.1 × 6 ≈ 38 Hz → period ≈ 26 ms.
 * 1 ms blanking (HAL_GetTick resolution) safely rejects contact bounce
 * without attenuating valid pulses at any realistic speed.               */
#define WHEEL_MIN_PULSE_INTERVAL_MS  1U

/* Stale detection: if no new pulse arrives within this window the wheel
 * speed is forced to 0 and the channel is flagged stale.  500 ms means
 * the vehicle must be nearly stopped (< 0.4 km/h) to trigger.           */
#define WHEEL_STALE_TIMEOUT_MS       500U

/* ISR flood ceiling: maximum accepted pulse rate (Hz) per wheel.
 * Any rate above this is physically impossible and indicates sensor
 * noise or wiring fault — pulses exceeding this are silently dropped
 * and the wheel is flagged as saturated.
 * 200 Hz corresponds to ~55 km/h (well above 25 km/h plausibility).     */
#define WHEEL_MAX_FREQ_HZ            200U

/* Output clamp: computed speed is hard-limited to this value (km/h).
 * Prevents NaN, negative, or wildly impossible speed readings from
 * propagating to traction control, ABS, and CAN telemetry.
 * Set to 40 km/h — well above the 25 km/h safety plausibility
 * threshold in safety_system.c, but low enough to catch malfunction.     */
#define WHEEL_SPEED_CLAMP_KMH        40.0f

/* ========================================================================== */
/*                       STEERING ENCODER (TIM2 Quadrature)                   */
/* ========================================================================== */
#define PIN_ENC_A          GPIO_PIN_15  /* PA15 - TIM2_CH1 */
#define PIN_ENC_B          GPIO_PIN_3   /* PB3  - TIM2_CH2 */
#define PIN_ENC_Z          GPIO_PIN_4   /* PB4  - GPIO polled input (index pulse) */

/* ========================================================================== */
/*                       STEERING CENTER INDUCTIVE SENSOR                     */
/* ========================================================================== */
/* LJ12A3-type inductive proximity sensor detecting a physical screw
 * placed at the mechanical center of the steering rack.                      */
#define PIN_STEER_CENTER   GPIO_PIN_5   /* PB5 - EXTI5 */

/* ========================================================================== */
/*                       I2C BUS (INA226 via TCA9548A, DS18B20)               */
/* ========================================================================== */
#define PIN_I2C_SCL        GPIO_PIN_6   /* PB6 - I2C1_SCL */
#define PIN_I2C_SDA        GPIO_PIN_7   /* PB7 - I2C1_SDA */

/* ========================================================================== */
/*                       NUCLEO-64 USER LED  (LD2)                            */
/* ========================================================================== */
/* LD2 is a green LED soldered on the NUCLEO-G474RE board, directly connected
 * to PA5 (per UM2505 §6.5).  No external wiring or resistor is needed.
 * Used for boot indication, CAN status, heartbeat, and fault blink.         */
#define PIN_LD2            GPIO_PIN_5   /* PA5 — Nucleo-64 user LED (LD2)    */
#define PORT_LD2           GPIOA        /* PA5 port                          */
#define PIN_LD2_N          5U           /* bit number for direct MODER/ODR   */

/* ========================================================================== */
/*                       DIAGNOSTIC LED (PB14, external)                      */
/* ========================================================================== */
/* PB14 was freed from TIM15_CH1 (LPWM_FR moved to PC3/TIM1_CH4).
 * Now available as a dedicated diagnostic LED — not shared with ST-Link
 * (unlike PA5/LD2 which shares SB21 with SPI SCK).
 * Accessible on Morpho CN10 pin 28.  Requires external LED + 330Ω resistor.
 * Can be used for: boot, CAN status, heartbeat, errors — independently
 * of PA5/LD2 which may exhibit interference during debug sessions.          */
#define PIN_LED_DIAG       GPIO_PIN_14  /* PB14 — external diagnostic LED    */
#define PORT_LED_DIAG      GPIOB        /* PB14 port                         */
#define PIN_LED_DIAG_N     14U          /* bit number for direct MODER/ODR   */

/* ========================================================================== */
/*                       ONEWIRE BUS (DS18B20 temperatures)                   */
/* ========================================================================== */
/* IMPORTANT: PB0 MUST be initialised as GPIO_MODE_OUTPUT_OD (open-drain)
 * with GPIO_PULLUP and GPIO_SPEED_FREQ_HIGH.  The .ioc file uses
 * GPIO_ModeDefaultOutputPP=GPIO_MODE_OUTPUT_OD to encode this.
 * If CubeMX ever resets PB0 to push-pull, the OneWire bus will fail
 * because DS18B20 requires open-drain signalling with a pull-up resistor.
 * MX_GPIO_Init() in main.c sets this correctly; verify after regeneration. */
#define PIN_ONEWIRE        GPIO_PIN_0   /* PB0 — open-drain, pull-up, high speed */

/* ========================================================================== */
/*                       PEDAL ACCELERATOR (ADC)                              */
/* ========================================================================== */
/* The Hall-effect pedal (SS1324LUA-T) operates at 5V (0.3V–4.8V output).
 * A voltage divider (10 kΩ + 6.8 kΩ) scales the signal to 0–3.3V for
 * the STM32 internal ADC on PA3.  Plausibility is ensured by:
 *   - Dual consecutive ADC samples with consistency check
 *   - EMA (Exponential Moving Average) filtering
 *   - Range validation (stuck-high / stuck-low detection)
 *   - Rate-of-change limiting (sudden jump detection)                        */
#define PIN_PEDAL          GPIO_PIN_3   /* PA3 - ADC1_IN4 */

/* ========================================================================== */
/*                       CAN BUS (FDCAN1)                                     */
/* ========================================================================== */
#define PIN_CAN_TX         GPIO_PIN_12  /* PA12 - FDCAN1_TX (AF9) */
#define PIN_CAN_RX         GPIO_PIN_11  /* PA11 - FDCAN1_RX (AF9) */

/* ========================================================================== */
/*                       I2C ADDRESSES                                        */
/* ========================================================================== */
#define I2C_ADDR_TCA9548A  0x70
#define I2C_ADDR_INA226    0x40

/* ========================================================================== */
/*                       SENSOR COUNTS                                        */
/* ========================================================================== */
#define NUM_INA226         6
#define NUM_DS18B20        5
#define NUM_WHEELS         4
#define WHEEL_PULSES_REV   6   /* 6 bolts per wheel revolution */

/* ========================================================================== */
/*                       INA226 SENSOR CONSTANTS                              */
/* ========================================================================== */
/* INA226 shunt resistors (mΩ) — values per hardware BOM:
 *   External shunts rated at 75 mV full-scale drop:
 *     50A  shunt: R = 75 mV / 50 A  = 1.5 mΩ  (0.0015 Ω)
 *     100A shunt: R = 75 mV / 100 A = 0.75 mΩ  (0.00075 Ω)
 * Channels 0-3 (motor wheels): 50A / 75mV shunt → 1.5 mΩ
 *   → placed BEFORE each BTS7960 driver (between relay output and B+ input)
 * Channel 4 (battery 24V):     100A / 75mV shunt → 0.75 mΩ
 *   → placed BEFORE the main relay (between battery + terminal and relay input)
 *     so that Voltage_GetBus() always reads battery voltage even when relay is open
 * Channel 5 (steering motor):  50A / 75mV shunt → 1.5 mΩ
 *   → placed BEFORE the steering BTS7960 driver (between relay output and B+ input) */
#define INA226_SHUNT_MOHM_MOTOR    1.5f   /* 1.5 mΩ for 50A/75mV sensors  */
#define INA226_SHUNT_MOHM_BATTERY  0.75f  /* 0.75 mΩ for 100A/75mV sensor */
#define INA226_CHANNEL_BATTERY     4      /* TCA9548A channel index        */

/* ========================================================================== */
/*               SYSTEM-LEVEL ELECTRICAL INTEGRATION NOTES                    */
/* ========================================================================== */
/*
 * ---- §A: BTS7960 (IBT-2) LOGIC SUPPLY — 3.3V REQUIREMENT ----
 *
 * ⚠ CRITICAL: The IBT-2 VCC pin MUST be powered from the 3.3V rail.
 *
 * Reason: The IBT-2 module includes a 74HC244 octal buffer between the
 * input header pins and the BTS7960 power ICs.  The 74HC244 input-high
 * threshold V_IH(min) = 0.7 × VCC:
 *
 *   VCC = 5.0V → V_IH(min) = 3.50V → STM32 3.3V GPIO is BELOW threshold
 *                                      (unreliable, may be read as LOW)
 *
 *   VCC = 3.3V → V_IH(min) = 2.31V → STM32 3.3V GPIO is ABOVE threshold
 *                                      (reliable, correct operation)
 *
 * The BTS7960 IC itself accepts V_IH ≥ 2.0V on INH/IN pins, but the
 * 74HC244 buffer is the limiting factor on the IBT-2 module.
 *
 * ⚠ VARIANT WARNING:
 *   Some third-party IBT-2 clones use different buffer ICs (74HCT244,
 *   CD74HC244, SN74HC244).  The 74HCT244 variant has V_IH(min) = 2.0V
 *   regardless of VCC and works at 5V with 3.3V inputs.  If your module
 *   uses 74HC (not 74HCT), the 3.3V VCC solution is REQUIRED.
 *
 *   If instability is observed at 3.3V VCC (e.g., motor twitching,
 *   sporadic enable faults), the migration path is:
 *     1. Power IBT-2 VCC from 5V rail
 *     2. Add BSS138 bidirectional level shifters on ALL signal lines:
 *        RPWM, LPWM, R_EN, L_EN (4 shifters per module, 20 total)
 *     3. Alternatively: bypass the 74HC244 and drive BTS7960 INH/IN
 *        directly (requires PCB modification)
 *
 * ⚠ EN PIN SAFETY:
 *   All five EN pins (PC0, PC1, PC4, PC5, PC13) are forced LOW via
 *   GPIOC->BSRR atomic write at the TOP of MX_GPIO_Init(), before
 *   the GPIO mode is configured.  No floating inputs are possible —
 *   push-pull drivers actively hold the level.
 *
 * ---- §B: POWER RAIL ARCHITECTURE — 3.3V / 5V SEPARATION ----
 *
 * ⚠ The STM32 internal 3.3V regulator (on the Nucleo-G474RE board) has
 *   limited current capacity (~300 mA).  It is intended ONLY for the
 *   MCU core and minimal on-board logic.
 *
 *   DO NOT power external sensors or modules from the STM32 3.3V pin:
 *
 *     ✗ INA226 sensors (6×, ~1 mA each = ~6 mA)        — use external LDO
 *     ✗ TCA9548A multiplexor (~1 mA)                    — use external LDO
 *     ✗ DS18B20 temperature sensors (5×, ~1.5 mA each)  — use external LDO
 *     ✗ BTS7960 IBT-2 VCC (5×, ~10 mA each = ~50 mA)   — use external LDO
 *     ✗ I2C pull-ups (2× 4.7kΩ, ~0.7 mA each)         — use external LDO
 *     ✗ OneWire pull-up (4.7kΩ, ~0.7 mA)               — use external LDO
 *
 *   Total external 3.3V load: ~60–80 mA (well within AMS1117-3.3 capacity
 *   of 1A, but EXCEEDS safe continuous draw from the Nucleo on-board reg).
 *
 *   RECOMMENDED: Use a dedicated external 3.3V regulator (AMS1117-3.3 or
 *   equivalent buck converter) fed from the 5V DC-DC (LM2596).
 *
 *   ⚠ AMS1117-3.3 THERMAL NOTE: At 5V→3.3V with 80 mA load, the LDO
 *   dissipates (5.0 − 3.3) × 0.08 = 136 mW — well within SOT-223
 *   package thermal limits.  At higher loads (>500 mA), consider a
 *   buck converter (e.g., AP63203, MP2315) to avoid thermal runaway.
 *
 * 5V rail (from LM2596 24V→5V buck):
 *   ✓ CAN transceiver TJA1051T/3 VCC (requires 4.5–5.5V, NOT 3.3V)
 *   ✓ WS2812B LED strips (via LED power relays PB10/PB11)
 *   ✓ Relay coils (SRD-05VDC, via HY-M158 optoacopladors)
 *   ✓ Encoder E6B2-CWZ6C power (5–12V)
 *
 * ---- §C: GROUNDING STRATEGY — STAR GROUND TOPOLOGY ----
 *
 * ⚠ CRITICAL FOR INA226 MEASUREMENT ACCURACY AND EMI IMMUNITY:
 *
 * All ground returns MUST converge at a single STAR GROUND point.
 * Do NOT daisy-chain ground wires between subsystems.
 *
 *                            ★ STAR GROUND POINT
 *                            │
 *        ┌───────────────────┼───────────────────┬──────────────────┐
 *        │                   │                   │                  │
 *   GND Bat 24V         GND Bat 12V        GND Logic 3.3V     GND BTS7960
 *   (heavy gauge)       (heavy gauge)      (signal grade)      (motor return)
 *                                               │
 *                                          ┌────┼────┬────┐
 *                                          │    │    │    │
 *                                     INA226  TCA  DS18  CAN
 *                                     (×6)  9548A  B20   GND
 *                                                  (×5)
 *
 * MANDATORY RULES:
 *   1. Motor power return cables (2.5 mm² min) go DIRECTLY to star point.
 *      Motor GND must NEVER share a return path with logic/sensor GND.
 *      Motor switching noise (20 kHz PWM) on shared ground causes:
 *        - INA226 measurement offset/noise (ground loop)
 *        - I2C communication errors (ground bounce on SDA/SCL)
 *        - ADC pedal reading jitter (ground noise on VSSA reference)
 *
 *   2. Battery negative terminals connect to star point with dedicated
 *      cables (4 mm² min).  Do NOT daisy-chain: Bat24V− → BTS → star.
 *
 *   3. CAN bus GND connects to star point through galvanic isolation
 *      (if ADuM1201 is used) or through a single point (no loop).
 *
 *   4. Decoupling capacitors (100 nF ceramic) placed at EACH IC VCC/GND
 *      pin, returned to the star ground via shortest possible trace.
 *      Bulk capacitor (1000 µF / 35V) on 24V bus near relay cluster.
 *
 *   5. INA226 VBUS and shunt connections:
 *      INA226 measures differential shunt voltage (IN+ / IN−).
 *      The sensor GND must be connected to the SAME star point as the
 *      shunt resistor's low-side.  Ground offset between INA226 GND
 *      and shunt GND causes measurement error proportional to the offset.
 */

#endif /* PROJECT_CONFIG_H */
