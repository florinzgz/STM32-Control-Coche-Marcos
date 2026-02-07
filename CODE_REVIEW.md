# Code Review — STM32G474RE Vehicle Safety Authority Firmware

**Date:** 2026-02-07  
**Reviewer:** Automated senior firmware review  
**Scope:** All `.c` / `.h` source, ISRs, state machines, safety logic, CAN, sensors, motor control, build system

---

## ✅ COMPLETE

The following features are **fully implemented, integrated, and safe to consider done**:

- **SystemClock_Config** — HSI 16 MHz → PLL (PLLM/4, PLLN×85, PLLR/2) → 170 MHz SYSCLK with `PWR_REGULATOR_VOLTAGE_SCALE1_BOOST` and `FLASH_LATENCY_8`. AHB/APB1/APB2 all 170 MHz.
- **TIM1 PWM (20 kHz, 4 channels)** — PA8–PA11 for traction motors FL/FR/RL/RR. Period = 8499, prescaler = 0. Proper OC PWM1 config.
- **TIM8 PWM (20 kHz, CH3)** — PC8 for steering motor. Same period/prescaler.
- **TIM2 quadrature encoder** — PA15 (CH1) / PB3 (CH2), TI12 mode, 4800 CPR. MSP AF1 correct.
- **Steering PID** — Closed-loop with anti-windup (±1000), dt-based compute, output clamped ±100%.
- **Ackermann geometry** — `Ackermann_Compute()` properly calculates inner/outer wheel angles with turn radius derivation. Inner angle limit applied. Configurable via `Ackermann_SetGeometry()`.
- **Traction modes** — 4×4, 4×2 (front only), tank turn. Enable logic correctly inhibits rear wheels in 4×2. Tank turn reverses left side.
- **Emergency stop** — `Traction_EmergencyStop()` zeros all 5 PWM channels and disables all 5 enable GPIOs. Sets demand to 0.
- **FDCAN1 init** — Classic CAN @ 500 kbps: prescaler=17, seg1=14, seg2=5 → 170MHz/17/(1+14+5) = 500k. Auto-retransmission enabled.
- **CAN TX messages** — 8 message types: heartbeat (0x001, 4B), speed (0x200, 8B), current (0x201, 8B), temp (0x202, 5B), safety (0x203, 3B), steering (0x204, 3B), error (0x300, 2B). All properly packed little-endian where applicable.
- **CAN RX parsing** — Throttle (0x100), steering (0x101), mode (0x102), heartbeat (0x011). All commands pass through `Safety_Validate*()` before reaching actuators.
- **CAN hardware filters** — Filter 0: dual-match 0x011. Filter 1: range 0x100–0x102. Global reject for all non-matching standard and extended IDs.
- **Enriched heartbeat** — Alive counter (cyclic 0–255), system state (0–4), fault flags bitmask (7 bits).
- **Safety state machine** — BOOT → STANDBY → ACTIVE ⇄ SAFE → ERROR. Forward-only transitions. SAFE→ACTIVE recovery only when fault cleared + ESP32 heartbeat present. ACTIVE→SAFE on fault. Any→ERROR on emergency stop.
- **Command validation gate** — `Safety_ValidateThrottle()`: clamp 0–100%, ABS/TCS override. `Safety_ValidateSteering()`: clamp ±45°, rate-limit 200°/s. `Safety_ValidateModeChange()`: speed gate <1 km/h.
- **ABS per-wheel** — Slip detection >20% per wheel, bitmask, activation counter. Cuts throttle to 0 when active. Min speed threshold 2 km/h.
- **TCS per-wheel** — Slip detection >15% per wheel, bitmask, activation counter. Reduces throttle to 50% when active. Min speed threshold 1 km/h.
- **Overcurrent protection** — Iterates all 6 INA226 sensors, >25A → `SAFETY_ERROR_OVERCURRENT` + SAFE state.
- **Overtemperature protection** — Iterates all 5 DS18B20 sensors, >90°C → `SAFETY_ERROR_OVERTEMP` + SAFE state.
- **CAN timeout detection** — >250ms since last ESP32 heartbeat → `SAFETY_ERROR_CAN_TIMEOUT` + SAFE state. Auto-recovery when heartbeat restored.
- **Relay power sequencing** — `Relay_PowerUp()`: Main→50ms→Traction→20ms→Direction. `Relay_PowerDown()`: reverse order, immediate.
- **DS18B20 OneWire** — Full bit-bang (reset, write/read bit/byte), Search ROM (0xF0) per Maxim AN187, Match ROM (0x55), CRC-8/MAXIM validation on ROM and scratchpad. Fallback to Skip ROM if no devices found.
- **INA226 current via TCA9548A** — Channel selection, shunt voltage read (2.5µV LSB / 1mΩ = mA → A), bus voltage read (1.25mV LSB → V).
- **Wheel speed sensors** — 4× EXTI interrupt-driven pulse counting. Speed computation: pulses × circumference / dt. Proper `volatile` on pulse counters.
- **ADC pedal** — PA3/ADC1_IN4, 12-bit, single-conversion polling. 0–4095 → 0–100%.
- **ISRs** — EXTI0/1/2/15_10 → wheel handlers. FDCAN1_IT0/IT1. TIM1_UP, TIM2. I2C1_EV/ER. SysTick → HAL_IncTick. FDCAN RxFifo0Callback → Safety_UpdateCANRxTime.
- **HAL MSP** — All alternate functions correct: FDCAN1 PB8/PB9 AF9, I2C1 PB6/PB7 AF4, TIM1 PA8–PA11 AF6, TIM8 PC8 AF4, TIM2 PA15/PB3 AF1, ADC1 PA3 analog.
- **IWDG** — 500ms timeout (prescaler 32, reload 4095 @ 32kHz LSI).
- **Main loop scheduling** — 10ms (safety+PID+traction), 50ms (sensors+pedal), 100ms (CAN status), 1000ms (CAN temp). Continuous: CAN RX + IWDG refresh.
- **Build system** — Makefile with correct source list, ARM GCC cross-compiler flags, linker script (512KB Flash, 128KB RAM), startup assembly.

---

## ⚠️ PARTIAL

- **Sensor plausibility checks** — Logic exists and covers temperature range (−40°C to 125°C), current range (0–50A), and speed range (0–60 km/h). **However**, it lacks cross-sensor consistency checks (e.g., all 4 wheels suddenly reading 0 simultaneously should be flagged as a sensor bus failure, not just "all speeds are plausible").

- **Temperature protection granularity** — Documentation (`SAFETY_SYSTEMS.md`) describes 3-tier thermal derating (60°C → 70% power, 80°C → 30% power, >90°C → stop). Code implements only a single >90°C hard cutoff. No gradual derating exists.

- **Current protection granularity** — Documentation describes 3-tier current protection (20A continuous, 30A peak for 2s, 35A critical disconnect). Code implements only a single >25A hard cutoff. No time-based peak allowance exists.

- **Ackermann integration** — `Ackermann_Compute()` is fully implemented but **never called** from the traction or steering control paths. Differential wheel speed based on steering angle is not applied.

- **Error tracking** — `safety_error` is a single `enum`, not a bitmask. If multiple faults occur simultaneously (e.g., overcurrent AND overtemp), only the last one written is retained. `Safety_GetFaultFlags()` correctly maps to a bitmask, but only tests equality against the single stored error.

- **Watchdog reset detection** — Documentation (`SAFETY_SYSTEMS.md`) describes `Check_Reset_Cause()` using `RCC_FLAG_IWDGRST` to detect and log watchdog resets. Not implemented in actual code; the system simply restarts from boot.

---

## ❌ MISSING

- **Regenerative braking** — Referenced in `PROJECT_STATUS.md` (medium priority). Logic exists in the FULL-FIRMWARE reference (`regen_ai.cpp`) but has not been ported.
- **Limp mode** — Referenced in `PROJECT_STATUS.md` (medium priority). Expected for degraded operation under partial faults (e.g., one motor driver failure).
- **Adaptive cruise control** — Referenced in `PROJECT_STATUS.md` (medium priority). Requires CAN data integration from ESP32 obstacle detection.
- **CRC8 in CAN application layer** — Documented in CAN protocol specification but not enforced in code. CAN hardware has its own CRC, but the application-level CRC adds protection against message corruption between software layers.
- **I2C bus recovery** — No mechanism to recover if the I2C bus locks up (SDA stuck low). The TCA9548A or INA226 could enter an error state requiring SCL bit-banging to recover.
- **Battery voltage monitoring** — `Voltage_GetBus()` is implemented but never called from any safety or monitoring path. No low-voltage protection exists.
- **DMA for ADC/I2C** — ADC uses blocking polling, I2C uses blocking transmit with 50ms timeout. Both block the main loop during conversion.
- **Window watchdog (WWDG)** — Only IWDG is used. WWDG would provide faster fault detection and ensure the main loop runs at the expected rate (not just "at all").
- **Persistent error logging** — No non-volatile fault history. Errors are lost on reset.

---

## 🔴 CRITICAL ISSUES

All items below have been **fixed in this PR**:

- **`last_can_rx_time` was not `volatile`** — Written by ISR (`HAL_FDCAN_RxFifo0Callback` → `Safety_UpdateCANRxTime()`), read by main loop (`Safety_CheckCANTimeout()`). Without `volatile`, the compiler could cache the value in a register across iterations, causing a **false CAN timeout** and spurious transition to SAFE state even while the ESP32 is alive. **Fixed: added `volatile` qualifier.**

- **`Safety_CheckSensors()` did not transition to SAFE state** — All three plausibility check branches (temperature, current, speed) called `Safety_SetError()` but NOT `Safety_SetState(SYS_STATE_SAFE)`. Compare with `Safety_CheckCurrent()` and `Safety_CheckTemperature()` which correctly do both. This meant a sensor plausibility failure was recorded as an error but **had no safety effect** — commands continued to be accepted, actuators remained active. **Fixed: added `Safety_SetState(SYS_STATE_SAFE)` to all three branches.**

- **Fault handlers did not safe actuators** — `HardFault_Handler`, `MemManage_Handler`, `BusFault_Handler`, and `UsageFault_Handler` entered infinite loops without disabling motor enables or relays. A hard fault would leave the motors running at their last commanded power for up to ~500ms until the IWDG fired. On a vehicle, 500ms of uncontrolled motion is dangerous. **Fixed: all fault handlers now drive GPIOC outputs LOW (enables + relays) via direct register write before entering the infinite loop.**

- **`Error_Handler()` did not safe actuators** — Same issue as above. If any peripheral init function (GPIO, CAN, I2C, TIM, ADC) failed, `Error_Handler()` disabled interrupts and hung forever — but relays and motor enables retained their current state. If the failure occurred after `MX_GPIO_Init()` but before `MX_IWDG_Init()`, the watchdog was not yet running and the system would hang indefinitely with hardware in an unknown state. **Fixed: `Error_Handler()` now drives GPIOC outputs LOW before hanging.**

- **`Steering_ControlLoop()` first-call PID spike** — `last_time` was initialized to 0. On first call (e.g., at tick 500ms after boot), `dt = 500ms`, producing a potentially large PID integral and derivative term. If the encoder had any drift from the zero point set in `Steering_Init()`, this could cause a violent initial steering movement. **Fixed: first call now seeds `last_time` and returns without computing, ensuring a clean dt on the next iteration.**

---

## 🟡 IMPROVEMENTS

- **`Relay_PowerUp()` blocks for 70ms** — Uses `HAL_Delay()` which blocks the main loop. During this time, safety checks, CAN processing, and IWDG refresh are suspended. While 70ms < 500ms IWDG timeout, it's not ideal. Consider a non-blocking state-machine approach for relay sequencing.

- **OneWire timing uses busy-wait NOP loop** — `OW_DelayUs()` approximates microsecond timing with a NOP loop calibrated for 170MHz. This is fragile if clock speed changes, compiler optimization varies, or cache behavior differs. A DWT cycle counter or dedicated timer would be more precise.

- **`EXTI15_10_IRQHandler` always calls `Wheel_RR_IRQHandler()`** — The EXTI15_10 shared handler doesn't check which specific line (10–15) triggered. If any other pin on GPIOB15:10 triggers (e.g., noise on PB10–PB14), the RR wheel counter would increment falsely. Should check `__HAL_GPIO_EXTI_GET_IT(PIN_WHEEL_RR)` before calling the handler.

- **`TIM1_UP_TIM16_IRQHandler` name vs NVIC** — The MSP configures `TIM1_UP_IRQn` but the handler is named `TIM1_UP_TIM16_IRQHandler`. On STM32G4, TIM1_UP and TIM16 share the same vector, so the naming is correct — but it's worth noting TIM16 interrupts would also be handled here.

- **ADC calibration not performed** — `HAL_ADCEx_Calibration_Start()` is not called before first ADC conversion. The STM32G4 ADC benefits from factory calibration load at init for best accuracy.

- **I2C timeout handling in sensor reads** — `HAL_I2C_Master_Transmit` and `HAL_I2C_Mem_Read` use a 50ms timeout. If a sensor or the TCA9548A bus hangs, these calls block for 50ms each × 6 sensors = up to 300ms of blocking per `Current_ReadAll()` cycle. This could approach the IWDG timeout.

---

## Summary

| Category | Count |
|----------|-------|
| ✅ Complete | 30+ features |
| ⚠️ Partial | 6 items |
| ❌ Missing | 9 items (all documented, well-scoped) |
| 🔴 Critical (fixed) | 5 bugs |
| 🟡 Improvements | 6 suggestions |

**Overall assessment:** The core architecture is sound, the safety authority pattern is correctly implemented, and the system is deterministic. The 5 critical bugs found were real safety risks that have been fixed with minimal, targeted changes. Remaining work (partial and missing items) is well-scoped and does not require architectural changes.
