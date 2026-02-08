# Hardware and Sensor Map — STM32G474RE Vehicle Controller

**Revision:** 1.0
**Date:** 2026-02-08
**Scope:** Complete hardware inventory, sensor index mapping, and CAN payload definitions for the STM32 firmware.
**Source of truth:** `Core/Inc/main.h` pin definitions and `Core/Src/*.c` implementations.

> **This document is verification-only.** No firmware behavior was changed.
> All tables were extracted directly from the source code.

---

## Table of Contents

1. [Hardware Component Inventory](#1-hardware-component-inventory)
   - [1.1 Motors](#11-motors-5-total)
   - [1.2 Sensors](#12-sensors)
   - [1.3 Relays](#13-relays-3-total)
   - [1.4 Communication](#14-communication)
   - [1.5 Watchdog](#15-watchdog)
2. [Sensor Index Mapping](#2-sensor-index-mapping)
   - [2.1 INA226 Current Sensors](#21-ina226-current-sensor-index-mapping)
   - [2.2 DS18B20 Temperature Sensors](#22-ds18b20-temperature-sensor-index-mapping)
   - [2.3 Wheel Speed Sensors](#23-wheel-speed-sensor-index-mapping)
3. [CAN Payload Mapping](#3-can-payload-mapping)
   - [3.1 STM32 → ESP32 Messages](#31-stm32--esp32-messages)
   - [3.2 ESP32 → STM32 Messages](#32-esp32--stm32-messages)
4. [Service Mode Module ID Reference](#4-service-mode-module-id-reference)
5. [Cross-Check vs Base Firmware](#5-cross-check-vs-base-firmware)
6. [Documentation Discrepancies](#6-documentation-discrepancies-found-in-existing-docs)

---

## 1. Hardware Component Inventory

### 1.1 Motors (5 total)

| Motor | Position | PWM Pin | Timer / Channel | DIR Pin | EN Pin | Interface | File(s) | Service Mode |
|-------|----------|---------|-----------------|---------|--------|-----------|---------|--------------|
| Traction FL | Front-Left | PA8 | TIM1_CH1 | PC0 | PC5 | PWM 20 kHz + GPIO | `motor_control.c`, `main.c` | — (always on) |
| Traction FR | Front-Right | PA9 | TIM1_CH2 | PC1 | PC6 | PWM 20 kHz + GPIO | `motor_control.c`, `main.c` | — (always on) |
| Traction RL | Rear-Left | PA10 | TIM1_CH3 | PC2 | PC7 | PWM 20 kHz + GPIO | `motor_control.c`, `main.c` | — (always on) |
| Traction RR | Rear-Right | PA11 | TIM1_CH4 | PC3 | PC13 | PWM 20 kHz + GPIO | `motor_control.c`, `main.c` | — (always on) |
| Steering | Center rack | PC8 | TIM8_CH3 | PC4 | PC9 | PWM 20 kHz + GPIO | `motor_control.c`, `main.c` | — (always on) |

**Notes:**
- TIM1 period: ARR = 8499 → 170 MHz / 8500 = **20 kHz**.
- TIM8 period: identical (ARR = 8499).
- PWM duty range: 0–8499 counts.
- Motor drivers: BTS7960 H-bridge (5 units). Each requires PWM + DIR + EN.
- All direction and enable pins are on GPIOC (push-pull output, low speed).

### 1.2 Sensors

#### 1.2.1 Wheel Speed Sensors (4 units — LJ12A3 inductive)

| Sensor | Wheel | Pin | Port | EXTI Line | Pulses/Rev | File(s) | Service Module ID |
|--------|-------|-----|------|-----------|------------|---------|-------------------|
| WHEEL_FL | Front-Left | PA0 | GPIOA | EXTI0 | 6 | `sensor_manager.c` | `MODULE_WHEEL_SPEED_FL` (15) |
| WHEEL_FR | Front-Right | PA1 | GPIOA | EXTI1 | 6 | `sensor_manager.c` | `MODULE_WHEEL_SPEED_FR` (16) |
| WHEEL_RL | Rear-Left | PA2 | GPIOA | EXTI2 | 6 | `sensor_manager.c` | `MODULE_WHEEL_SPEED_RL` (17) |
| WHEEL_RR | Rear-Right | PB15 | GPIOB | EXTI15 | 6 | `sensor_manager.c` | `MODULE_WHEEL_SPEED_RR` (18) |

- **Classification:** NON-CRITICAL (all four).
- **Interface:** GPIO input with internal pull-up, rising-edge EXTI interrupt.
- **Wheel circumference:** 1100 mm (`WHEEL_CIRCUM_MM` in `vehicle_physics.h`).
- **Speed formula:** `speed_kmh = (delta_pulses / 6) × 1.1 m × (1000/dt_ms) × 3.6`

#### 1.2.2 Steering Encoder (1 unit — E6B2-CWZ6C)

| Signal | Pin | Port | Peripheral | Notes |
|--------|-----|------|------------|-------|
| ENC_A (Phase A) | PA15 | GPIOA | TIM2_CH1 | Quadrature input |
| ENC_B (Phase B) | PB3 | GPIOB | TIM2_CH2 | Quadrature input |
| ENC_Z (Index) | PB4 | GPIOB | (NOT USED) | No EXTI4 init in firmware |

- **Resolution:** 1200 PPR × 4 (quadrature) = **4800 counts/revolution**.
- **Angular resolution:** 360° / 4800 = **0.075° per count**.
- **TIM2 config:** Encoder mode TI12, ARR = 65535, no prescaler.
- **Z-index pulse is intentionally NOT used** (see `motor_control.c` comment block).
- **Classification:** NON-CRITICAL (`MODULE_STEER_ENCODER`, ID 20).
- **File(s):** `motor_control.c` (encoder read, health check), `main.c` (TIM2 init).

#### 1.2.3 Steering Center Inductive Sensor (1 unit — LJ12A3-type)

| Signal | Pin | Port | EXTI | Notes |
|--------|-----|------|------|-------|
| STEER_CENTER | PB5 | GPIOB | EXTI5 (via EXTI9_5) | Rising-edge, internal pull-up |

- **Purpose:** Detects a physical screw at the mechanical center of the steering rack.
- **Used by:** `steering_centering.c` — automatic centering sweep at startup.
- **Classification:** NON-CRITICAL (`MODULE_STEER_CENTER`, ID 19).
- **File(s):** `sensor_manager.c` (ISR + flag), `steering_centering.c` (consumer).

#### 1.2.4 INA226 Current / Voltage Sensors (6 units via TCA9548A)

| Index | TCA9548A Channel | Subsystem | Service Module ID | Reported via CAN |
|-------|------------------|-----------|-------------------|------------------|
| 0 | CH0 | **Motor FL** | `MODULE_CURRENT_SENSOR_0` (9) | STATUS_CURRENT bytes 0–1 |
| 1 | CH1 | **Motor FR** | `MODULE_CURRENT_SENSOR_1` (10) | STATUS_CURRENT bytes 2–3 |
| 2 | CH2 | **Motor RL** | `MODULE_CURRENT_SENSOR_2` (11) | STATUS_CURRENT bytes 4–5 |
| 3 | CH3 | **Motor RR** | `MODULE_CURRENT_SENSOR_3` (12) | STATUS_CURRENT bytes 6–7 |
| 4 | CH4 | **Battery** | `MODULE_CURRENT_SENSOR_4` (13) | NOT in STATUS_CURRENT |
| 5 | CH5 | **Steering motor** | `MODULE_CURRENT_SENSOR_5` (14) | NOT in STATUS_CURRENT |

- **Bus:** I2C1 @ 400 kHz (PB6 = SCL, PB7 = SDA).
- **Multiplexer:** TCA9548A at I2C address **0x70**.
- **INA226 address:** All at **0x40** (unique per TCA9548A channel).
- **Shunt resistor:** 1 mΩ (`INA226_SHUNT_MOHM` in `main.h`).
- **Shunt LSB:** 2.5 µV. **Bus LSB:** 1.25 mV.
- **Classification:** All six are NON-CRITICAL.
- **File(s):** `sensor_manager.c` (read logic), `safety_system.c` (overcurrent check).

> **⚠ Important:** `CAN_SendStatusCurrent()` in `main.c` only sends indices 0–3 (four traction motors).
> Battery (index 4) and steering (index 5) currents are **read and checked by safety** but are
> **NOT transmitted to the ESP32** via any CAN message. The ESP32 HMI cannot currently display
> battery or steering motor current.

#### 1.2.5 DS18B20 Temperature Sensors (5 units on OneWire bus)

| Index | Intended Location | Service Module ID | CAN Byte in STATUS_TEMP |
|-------|-------------------|-------------------|-------------------------|
| 0 | **Motor FL** | `MODULE_TEMP_SENSOR_0` (4) | Byte 0 |
| 1 | **Motor FR** | `MODULE_TEMP_SENSOR_1` (5) | Byte 1 |
| 2 | **Motor RL** | `MODULE_TEMP_SENSOR_2` (6) | Byte 2 |
| 3 | **Motor RR** | `MODULE_TEMP_SENSOR_3` (7) | Byte 3 |
| 4 | **Ambient** | `MODULE_TEMP_SENSOR_4` (8) | Byte 4 |

- **Bus:** OneWire bit-bang on **PB0** (GPIOB, open-drain with external 4.7 kΩ pull-up).
- **Discovery:** Search ROM algorithm at `Sensor_Init()` — sensors are enumerated by their 64-bit ROM address.
- **Classification:** All five are NON-CRITICAL.
- **File(s):** `sensor_manager.c` (OneWire driver, search, read), `safety_system.c` (overtemp check).

> **⚠ Important — index ordering depends on ROM address discovery order.**
> DS18B20 sensors are discovered via Search ROM and stored in the order the algorithm
> finds them. This order depends on the 64-bit ROM codes burned into each sensor at
> the factory. If sensors are physically swapped or replaced, the index ↔ location mapping
> may change. There is no physical-location verification in firmware.
>
> **Recommendation:** When deploying sensors, record the ROM address of each sensor and
> verify the mapping by reading temperatures with a known heat source on one motor.

#### 1.2.6 Pedal (1 unit — Hall-effect analog)

| Signal | Pin | Peripheral | Channel | Resolution |
|--------|-----|------------|---------|------------|
| PEDAL | PA3 | ADC1 | IN4 | 12-bit (0–4095) |

- **Conversion:** `pedal_pct = raw × 100 / 4095` (linear 0–100 %).
- **Sampling:** Single conversion triggered every 50 ms from main loop.
- **File(s):** `sensor_manager.c` (ADC read), `motor_control.c` (EMA filter + ramp limiter).
- **Not a service module** — no dedicated service mode entry.

### 1.3 Relays (3 total)

| Relay | Pin | Port | Function | Power-up Order | Service Module ID |
|-------|-----|------|----------|----------------|-------------------|
| RELAY_MAIN | PC10 | GPIOC | Main power | 1st (50 ms settle) | `MODULE_RELAY_MAIN` (3) — CRITICAL |
| RELAY_TRAC | PC11 | GPIOC | Traction motor power | 2nd (20 ms settle) | — |
| RELAY_DIR | PC12 | GPIOC | Steering motor power | 3rd | — |

- **Interface:** GPIO push-pull output, active HIGH = relay energized.
- **Default state:** LOW (de-energized, fail-safe open).
- **Power-down order:** DIR → TRAC → MAIN (reverse of power-up).
- **File(s):** `safety_system.c` (`Relay_PowerUp`, `Relay_PowerDown`), `main.c` (GPIO init).

### 1.4 Communication

| Bus | Peripheral | Pins | Speed | Protocol | File(s) |
|-----|------------|------|-------|----------|---------|
| CAN | FDCAN1 | PB8 (RX), PB9 (TX) | 500 kbps | CAN 2.0A, 11-bit IDs | `can_handler.c`, `main.c` |
| I2C | I2C1 | PB6 (SCL), PB7 (SDA) | 400 kHz | I2C Fast-mode | `sensor_manager.c`, `main.c` |
| OneWire | GPIO bit-bang | PB0 | ~16 kbps | Dallas 1-Wire | `sensor_manager.c` |

**CAN bit timing (verified from `MX_FDCAN1_Init`):**
- Clock source: 170 MHz APB1 (FDCAN_CLOCK_DIV1).
- Prescaler: 17 → Time quantum = 100 ns.
- Segments: 1 + 14 + 5 = 20 TQ per bit → 170 MHz / (17 × 20) = **500,000 bps**.

### 1.5 Watchdog

| Peripheral | Prescaler | Reload | Timeout | File(s) |
|------------|-----------|--------|---------|---------|
| IWDG | /32 | 4095 | ~500 ms | `main.c` (init + refresh in main loop) |

- **Classification:** CRITICAL (`MODULE_WATCHDOG`, ID 2).
- Fed every main-loop iteration via `HAL_IWDG_Refresh()`.

---

## 2. Sensor Index Mapping

This section provides the **definitive mapping** between array indices used in firmware and their
physical meaning. This is what the ESP32 HMI needs to correctly label each value.

### 2.1 INA226 Current Sensor Index Mapping

**Source:** `service_mode.h` comments (lines 61–66), `main.c` (lines 139–143), `sensor_manager.c`.

| `Current_GetAmps(index)` | Physical Location | TCA9548A Channel | Service Module |
|---------------------------|-------------------|------------------|----------------|
| `index = 0` | **Traction motor FL** | CH0 | `MODULE_CURRENT_SENSOR_0` (9) |
| `index = 1` | **Traction motor FR** | CH1 | `MODULE_CURRENT_SENSOR_1` (10) |
| `index = 2` | **Traction motor RL** | CH2 | `MODULE_CURRENT_SENSOR_2` (11) |
| `index = 3` | **Traction motor RR** | CH3 | `MODULE_CURRENT_SENSOR_3` (12) |
| `index = 4` | **Battery (main supply)** | CH4 | `MODULE_CURRENT_SENSOR_4` (13) |
| `index = 5` | **Steering motor** | CH5 | `MODULE_CURRENT_SENSOR_5` (14) |

**CAN transmission:** Only indices 0–3 are sent in `STATUS_CURRENT` (0x201).
Indices 4–5 are read by the safety system but NOT transmitted to ESP32.

### 2.2 DS18B20 Temperature Sensor Index Mapping

**Source:** `service_mode.h` comments (lines 56–60), `main.c` (lines 156–161), `motor_control.c` (line 289).

| `Temperature_Get(index)` | Intended Location | CAN STATUS_TEMP Byte | Service Module |
|---------------------------|-------------------|----------------------|----------------|
| `index = 0` | **Motor FL** | Byte 0 | `MODULE_TEMP_SENSOR_0` (4) |
| `index = 1` | **Motor FR** | Byte 1 | `MODULE_TEMP_SENSOR_1` (5) |
| `index = 2` | **Motor RL** | Byte 2 | `MODULE_TEMP_SENSOR_2` (6) |
| `index = 3` | **Motor RR** | Byte 3 | `MODULE_TEMP_SENSOR_3` (7) |
| `index = 4` | **Ambient** | Byte 4 | `MODULE_TEMP_SENSOR_4` (8) |

**⚠ Caveat:** Indices are assigned based on OneWire Search ROM discovery order, which depends on
factory-burned 64-bit ROM codes. Physical location is NOT verified by firmware. See section 1.2.5.

### 2.3 Wheel Speed Sensor Index Mapping

**Source:** `sensor_manager.c` (lines 28–31), `motor_control.c` (lines 283–286), `motor_control.h` (lines 13–16).

| Internal Index | Wheel | EXTI Handler | Getter Function | Service Module |
|----------------|-------|-------------|-----------------|----------------|
| `0` | **Front-Left** | `Wheel_FL_IRQHandler` → `wheel_pulse[0]++` | `Wheel_GetSpeed_FL()` | `MODULE_WHEEL_SPEED_FL` (15) |
| `1` | **Front-Right** | `Wheel_FR_IRQHandler` → `wheel_pulse[1]++` | `Wheel_GetSpeed_FR()` | `MODULE_WHEEL_SPEED_FR` (16) |
| `2` | **Rear-Left** | `Wheel_RL_IRQHandler` → `wheel_pulse[2]++` | `Wheel_GetSpeed_RL()` | `MODULE_WHEEL_SPEED_RL` (17) |
| `3` | **Rear-Right** | `Wheel_RR_IRQHandler` → `wheel_pulse[3]++` | `Wheel_GetSpeed_RR()` | `MODULE_WHEEL_SPEED_RR` (18) |

**Motor index constants** (from `motor_control.h`):
```c
#define MOTOR_FL  0  // Front Left
#define MOTOR_FR  1  // Front Right
#define MOTOR_RL  2  // Rear Left
#define MOTOR_RR  3  // Rear Right
```

These same indices are used for `traction_state.wheels[]`, `Current_GetAmps()` (0–3), and
`Temperature_Get()` (0–3) when accessing per-wheel data.

---

## 3. CAN Payload Mapping

All multi-byte integer values use **little-endian** byte order.
CAN bus runs at 500 kbps, CAN 2.0A (classic, 11-bit standard IDs).

### 3.1 STM32 → ESP32 Messages

#### HEARTBEAT_STM32 (0x001) — 4 bytes, every 100 ms

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | `alive_counter` | uint8 | Cyclic counter 0–255 (intentional rollover) |
| 1 | `system_state` | uint8 | See system state table below |
| 2 | `fault_flags` | uint8 | Bitmask of active faults (see below) |
| 3 | `error_code` | uint8 | `Safety_Error_t` value — specific fault ID for HMI |

**Source:** `CAN_SendHeartbeat()` in `can_handler.c`

**System states (byte 1):**

| Value | State | Meaning | Commands accepted? |
|-------|-------|---------|--------------------|
| 0 | `SYS_STATE_BOOT` | Power-on, peripherals initializing | No |
| 1 | `SYS_STATE_STANDBY` | Ready, waiting for ESP32 heartbeat | No |
| 2 | `SYS_STATE_ACTIVE` | Normal operation | Yes |
| 3 | `SYS_STATE_DEGRADED` | Limp / reduced power (40% power, 50% speed) | Yes (limited) |
| 4 | `SYS_STATE_SAFE` | Severe fault — actuators inhibited | No |
| 5 | `SYS_STATE_ERROR` | Unrecoverable — relays de-energized, reset required | No |

**Fault flags bitmask (byte 2):**

| Bit | Mask | Flag | Meaning |
|-----|------|------|---------|
| 0 | 0x01 | `FAULT_CAN_TIMEOUT` | ESP32 heartbeat not received within 250 ms |
| 1 | 0x02 | `FAULT_TEMP_OVERLOAD` | Temperature exceeds threshold |
| 2 | 0x04 | `FAULT_CURRENT_OVERLOAD` | Current exceeds 25 A |
| 3 | 0x08 | `FAULT_ENCODER_ERROR` | Steering encoder fault detected |
| 4 | 0x10 | `FAULT_WHEEL_SENSOR` | At least one wheel speed sensor faulted |
| 5 | 0x20 | `FAULT_ABS_ACTIVE` | ABS currently intervening |
| 6 | 0x40 | `FAULT_TCS_ACTIVE` | TCS currently intervening |
| 7 | 0x80 | `FAULT_CENTERING` | Steering centering failed |

**Source:** `safety_system.h` defines, `Safety_GetFaultFlags()` in `safety_system.c`

#### STATUS_SPEED (0x200) — 8 bytes, every 100 ms

| Bytes | Field | Type | Unit | Description |
|-------|-------|------|------|-------------|
| 0–1 | `speed_FL` | uint16 LE | 0.1 km/h | Front-Left wheel speed |
| 2–3 | `speed_FR` | uint16 LE | 0.1 km/h | Front-Right wheel speed |
| 4–5 | `speed_RL` | uint16 LE | 0.1 km/h | Rear-Left wheel speed |
| 6–7 | `speed_RR` | uint16 LE | 0.1 km/h | Rear-Right wheel speed |

**Encoding:** `(uint16_t)(Wheel_GetSpeed_XX() * 10)` — km/h multiplied by 10.
**Decoding:** `speed_kmh = (float)raw_value / 10.0f`

**Source:** `CAN_SendStatusSpeed()` in `can_handler.c`, called from `main.c`

#### STATUS_CURRENT (0x201) — 8 bytes, every 100 ms

| Bytes | Field | Type | Unit | Description |
|-------|-------|------|------|-------------|
| 0–1 | `current_FL` | uint16 LE | 0.01 A | Front-Left motor current (INA226 index 0) |
| 2–3 | `current_FR` | uint16 LE | 0.01 A | Front-Right motor current (INA226 index 1) |
| 4–5 | `current_RL` | uint16 LE | 0.01 A | Rear-Left motor current (INA226 index 2) |
| 6–7 | `current_RR` | uint16 LE | 0.01 A | Rear-Right motor current (INA226 index 3) |

**Encoding:** `(uint16_t)(Current_GetAmps(n) * 100)` — amperes multiplied by 100.
**Decoding:** `current_A = (float)raw_value / 100.0f`

> **Note:** Battery (index 4) and steering (index 5) currents are NOT included.

**Source:** `CAN_SendStatusCurrent()` in `can_handler.c`, called from `main.c`

#### STATUS_TEMP (0x202) — 5 bytes, every 1000 ms

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0 | `temp_motor_FL` | int8 | °C | DS18B20 index 0 — Motor FL temperature |
| 1 | `temp_motor_FR` | int8 | °C | DS18B20 index 1 — Motor FR temperature |
| 2 | `temp_motor_RL` | int8 | °C | DS18B20 index 2 — Motor RL temperature |
| 3 | `temp_motor_RR` | int8 | °C | DS18B20 index 3 — Motor RR temperature |
| 4 | `temp_ambient` | int8 | °C | DS18B20 index 4 — Ambient temperature |

**Encoding:** `(int8_t)Temperature_Get(n)` — truncated to integer °C.
**Range:** −128 to +127 °C (int8 limits; DS18B20 range is −55 to +125 °C).

**Source:** `CAN_SendStatusTemp()` in `can_handler.c`, called from `main.c`

#### STATUS_SAFETY (0x203) — 3 bytes, every 100 ms

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | `abs_active` | uint8 | 1 = ABS currently active, 0 = inactive |
| 1 | `tcs_active` | uint8 | 1 = TCS currently active, 0 = inactive |
| 2 | `error_code` | uint8 | `Safety_Error_t` value (0 = none, see table below) |

**Safety error codes:**

| Code | Name | Trigger |
|------|------|---------|
| 0 | `SAFETY_ERROR_NONE` | Normal operation |
| 1 | `SAFETY_ERROR_OVERCURRENT` | Any INA226 reads > 25 A |
| 2 | `SAFETY_ERROR_OVERTEMP` | Any DS18B20 reads > 80 °C (warning) or > 90 °C (critical) |
| 3 | `SAFETY_ERROR_CAN_TIMEOUT` | No ESP32 heartbeat for > 250 ms |
| 4 | `SAFETY_ERROR_SENSOR_FAULT` | Sensor plausibility check failed |
| 5 | `SAFETY_ERROR_MOTOR_STALL` | Reserved (not implemented) |
| 6 | `SAFETY_ERROR_EMERGENCY_STOP` | Emergency stop triggered |
| 7 | `SAFETY_ERROR_WATCHDOG` | IWDG timeout |
| 8 | `SAFETY_ERROR_CENTERING` | Steering centering failed |

**Source:** `CAN_SendStatusSafety()` in `can_handler.c`, `Safety_Error_t` in `safety_system.h`

#### STATUS_STEERING (0x204) — 3 bytes, every 100 ms

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0 | `angle_LSB` | uint8 | — | Low byte of signed 16-bit angle |
| 1 | `angle_MSB` | uint8 | — | High byte of signed 16-bit angle |
| 2 | `calibrated` | uint8 | — | 1 = encoder calibrated, 0 = not calibrated |

**Encoding:** `(int16_t)(Steering_GetCurrentAngle() * 10)` — degrees × 10.
**Decoding:** `angle_deg = (float)(int16_t)(byte[0] | (byte[1] << 8)) / 10.0f`
**Sign convention:** positive = left turn.

**Source:** `CAN_SendStatusSteering()` in `can_handler.c`, called from `main.c`

#### DIAG_ERROR (0x300) — 2 bytes, on-demand

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | `error_code` | uint8 | Error type (see safety error codes above) |
| 1 | `subsystem` | uint8 | 0 = Global, 1 = Motor, 2 = Sensor, 3 = CAN |

**Source:** `CAN_SendError()` in `can_handler.c`

#### SERVICE_FAULTS (0x301) — 4 bytes, every 1000 ms

| Bytes | Field | Type | Description |
|-------|-------|------|-------------|
| 0–3 | `fault_mask` | uint32 LE | Bit N = 1 means module N has a fault (WARNING or ERROR) |

**Source:** `ServiceMode_GetFaultMask()`, transmitted by `CAN_SendServiceStatus()`

#### SERVICE_ENABLED (0x302) — 4 bytes, every 1000 ms

| Bytes | Field | Type | Description |
|-------|-------|------|-------------|
| 0–3 | `enabled_mask` | uint32 LE | Bit N = 1 means module N is enabled |

**Source:** `ServiceMode_GetEnabledMask()`, transmitted by `CAN_SendServiceStatus()`

#### SERVICE_DISABLED (0x303) — 4 bytes, every 1000 ms

| Bytes | Field | Type | Description |
|-------|-------|------|-------------|
| 0–3 | `disabled_mask` | uint32 LE | Bit N = 1 means module N is manually disabled |

**Source:** `ServiceMode_GetDisabledMask()`, transmitted by `CAN_SendServiceStatus()`

### 3.2 ESP32 → STM32 Messages

#### HEARTBEAT_ESP32 (0x011) — any DLC, every 100 ms

Payload is NOT parsed by STM32. Only frame arrival resets the heartbeat watchdog.

#### CMD_THROTTLE (0x100) — 1+ bytes, every 50 ms

| Byte | Field | Type | Range | Description |
|------|-------|------|-------|-------------|
| 0 | `throttle_pct` | uint8 | 0–100 | Requested throttle percentage |

**STM32 validation:** Clamped to 0–100 %. Zeroed if ABS active. Halved if TCS active.
Multiplied by degraded-mode factor (0.4) if in DEGRADED state. Rejected if not ACTIVE/DEGRADED.

#### CMD_STEERING (0x101) — 2+ bytes, every 50 ms

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | `angle_LSB` | uint8 | Low byte of signed 16-bit requested angle |
| 1 | `angle_MSB` | uint8 | High byte of signed 16-bit requested angle |

**Decoding:** `requested_deg = (float)(int16_t)(byte[0] | (byte[1] << 8)) / 10.0f`
**STM32 validation:** Clamped to ±54° (`MAX_STEER_DEG`). Rate-limited to 200°/s. Rejected if not ACTIVE/DEGRADED.

#### CMD_MODE (0x102) — 1+ bytes, on-demand

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | `mode_flags` | uint8 | Bit 0: enable 4×4 (1 = 4WD, 0 = 4×2). Bit 1: tank turn. |

**STM32 validation:** Rejected if not ACTIVE/DEGRADED or average wheel speed > 1 km/h.

#### SERVICE_CMD (0x110) — 1–2 bytes, on-demand

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | `command` | uint8 | 0x00 = disable module, 0x01 = enable module, 0xFF = factory restore |
| 1 | `module_id` | uint8 | Module ID (0–24), required for disable/enable only |

**Safety constraint:** Critical modules (IDs 0–3) reject disable commands silently.

**Source:** `CAN_ProcessMessages()` case `CAN_ID_SERVICE_CMD` in `can_handler.c`

---

## 4. Service Mode Module ID Reference

Complete module registry with classification, mapped hardware, and CAN bitmask position.

| ID | Name | Classification | Hardware / Subsystem | Bit in 0x301/0x302/0x303 |
|----|------|----------------|----------------------|--------------------------|
| 0 | `MODULE_CAN_TIMEOUT` | **CRITICAL** | FDCAN1 heartbeat watchdog | bit 0 |
| 1 | `MODULE_EMERGENCY_STOP` | **CRITICAL** | Hardware emergency stop | bit 1 |
| 2 | `MODULE_WATCHDOG` | **CRITICAL** | IWDG (500 ms) | bit 2 |
| 3 | `MODULE_RELAY_MAIN` | **CRITICAL** | Main power relay (PC10) | bit 3 |
| 4 | `MODULE_TEMP_SENSOR_0` | NON-CRITICAL | DS18B20 index 0 → Motor FL | bit 4 |
| 5 | `MODULE_TEMP_SENSOR_1` | NON-CRITICAL | DS18B20 index 1 → Motor FR | bit 5 |
| 6 | `MODULE_TEMP_SENSOR_2` | NON-CRITICAL | DS18B20 index 2 → Motor RL | bit 6 |
| 7 | `MODULE_TEMP_SENSOR_3` | NON-CRITICAL | DS18B20 index 3 → Motor RR | bit 7 |
| 8 | `MODULE_TEMP_SENSOR_4` | NON-CRITICAL | DS18B20 index 4 → Ambient | bit 8 |
| 9 | `MODULE_CURRENT_SENSOR_0` | NON-CRITICAL | INA226 CH0 → Motor FL | bit 9 |
| 10 | `MODULE_CURRENT_SENSOR_1` | NON-CRITICAL | INA226 CH1 → Motor FR | bit 10 |
| 11 | `MODULE_CURRENT_SENSOR_2` | NON-CRITICAL | INA226 CH2 → Motor RL | bit 11 |
| 12 | `MODULE_CURRENT_SENSOR_3` | NON-CRITICAL | INA226 CH3 → Motor RR | bit 12 |
| 13 | `MODULE_CURRENT_SENSOR_4` | NON-CRITICAL | INA226 CH4 → Battery | bit 13 |
| 14 | `MODULE_CURRENT_SENSOR_5` | NON-CRITICAL | INA226 CH5 → Steering motor | bit 14 |
| 15 | `MODULE_WHEEL_SPEED_FL` | NON-CRITICAL | EXTI0 (PA0) → Wheel FL | bit 15 |
| 16 | `MODULE_WHEEL_SPEED_FR` | NON-CRITICAL | EXTI1 (PA1) → Wheel FR | bit 16 |
| 17 | `MODULE_WHEEL_SPEED_RL` | NON-CRITICAL | EXTI2 (PA2) → Wheel RL | bit 17 |
| 18 | `MODULE_WHEEL_SPEED_RR` | NON-CRITICAL | EXTI15 (PB15) → Wheel RR | bit 18 |
| 19 | `MODULE_STEER_CENTER` | NON-CRITICAL | LJ12A3 inductive on PB5 (EXTI5) | bit 19 |
| 20 | `MODULE_STEER_ENCODER` | NON-CRITICAL | E6B2-CWZ6C on PA15/PB3 (TIM2) | bit 20 |
| 21 | `MODULE_ABS` | NON-CRITICAL | Anti-lock braking (software) | bit 21 |
| 22 | `MODULE_TCS` | NON-CRITICAL | Traction control (software) | bit 22 |
| 23 | `MODULE_ACKERMANN` | NON-CRITICAL | Ackermann steering geometry (software) | bit 23 |
| 24 | `MODULE_OBSTACLE_DETECT` | NON-CRITICAL | Obstacle detection — **ESP32-side only** | bit 24 |

**Source:** `ModuleID_t` enum in `service_mode.h`, classification table in `service_mode.c`

---

## 5. Cross-Check vs Base Firmware

Reference: [FULL-FIRMWARE-Coche-Marcos](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos) at commit `c52beec`

### 5.1 Components Present in Both STM32 and Base Firmware

| Component | Base Firmware File(s) | STM32 File(s) | Notes |
|-----------|----------------------|---------------|-------|
| 4× Traction motors | `src/control/traction.cpp` | `motor_control.c` | Ported: 4WD, 4×2, tank-turn modes |
| 1× Steering motor | `src/control/steering_motor.cpp` | `motor_control.c` | Ported: PID control, deadband |
| Ackermann geometry | `src/control/steering_model.cpp` | `ackermann.c` | Ported: pure calculation module |
| 6× INA226 sensors | `src/sensors/current.cpp` | `sensor_manager.c` | Ported: via TCA9548A |
| 5× DS18B20 sensors | `src/sensors/temperature.cpp` | `sensor_manager.c` | Ported: OneWire with Search ROM |
| 4× Wheel speed sensors | `src/sensors/wheels.cpp` | `sensor_manager.c` | Ported: EXTI pulse counting |
| 1× Steering encoder | `src/input/steering.cpp` | `motor_control.c` | Ported: TIM2 quadrature mode |
| 1× Pedal input | `src/input/pedal.cpp` | `sensor_manager.c`, `motor_control.c` | Ported: ADC + EMA filter (α=0.15) |
| 3× Relays | `src/control/relays.cpp` | `safety_system.c` | Ported: sequenced power-up/down |
| ABS system | `src/safety/abs_system.cpp` | `safety_system.c` | Ported: slip-based threshold (15%) |
| TCS system | `src/control/tcs_system.cpp` | `safety_system.c` | Ported: slip-based threshold (15%) |
| Limp / degraded mode | `src/system/limp_mode.cpp` | `safety_system.c`, `service_mode.c` | Ported: 40% power / 50% speed limits |
| Sensor enable flags | `src/sensors/car_sensors.cpp` | `service_mode.c` | Ported: per-module enable/disable |

### 5.2 Components in Base Firmware Moved to ESP32 by Design

| Component | Base Firmware File(s) | Reason NOT in STM32 |
|-----------|----------------------|---------------------|
| TFT display HMI | `src/hud/` | Display is on ESP32-S3 (SPI bus) |
| Audio / DFPlayer | `src/audio/` | Audio output is on ESP32-S3 |
| WS2812B LEDs | `src/lighting/` | LED strip is on ESP32-S3 |
| Menu system | `src/menu/` | Touch UI runs on ESP32-S3 |
| Logging (Serial) | `src/logging/` | STM32 uses CAN for all diagnostics |
| Obstacle detection (ultrasonic) | `src/sensors/obstacle_detection.cpp`, `src/safety/obstacle_safety.cpp` | Ultrasonic sensors on ESP32 GPIO; STM32 has `MODULE_OBSTACLE_DETECT` placeholder |
| Adaptive cruise | `src/control/adaptive_cruise.cpp` | Higher-level feature, ESP32 domain |
| Regen AI braking | `src/safety/regen_ai.cpp` | Higher-level feature, ESP32 domain |
| Button inputs | `src/input/buttons.cpp` | Physical buttons on ESP32 board |

### 5.3 Components in Base Firmware NOT Present in STM32

| Component | Base Firmware File(s) | Status |
|-----------|----------------------|--------|
| **Shifter (F/N/R)** | `src/input/shifter.cpp` | **Not ported.** No GPIO pins (PB12/PB13/PB14) initialized in STM32 `MX_GPIO_Init()`. Drive direction is controlled via `CMD_MODE` (0x102) bit flags from ESP32. The physical shifter hardware is read by the ESP32 and translated to CAN commands; it is not connected to the STM32. |
| NVS persistence | `src/system/limp_mode.cpp` (`reset()`) | Not ported. STM32 has no EEPROM. Service mode flags reset on every power cycle. |
| Power management | `src/system/power_mgmt.cpp` | Partially ported — relay sequencing exists in `safety_system.c` but battery SOC monitoring is not present. |

### 5.4 Components in STM32 NOT in Base Firmware

| Component | STM32 File(s) | Notes |
|-----------|---------------|-------|
| Steering centering (inductive sensor) | `steering_centering.c` | New: automatic center-find at startup using LJ12A3 sensor on PB5. Base firmware used manual centering or assumed known center position. |
| Service mode (per-module disable/enable) | `service_mode.c` | New: extends base firmware's `cfg.*Enabled` flags to a full module system with CAN-reported bitmasks. |
| IWDG watchdog | `main.c` | New: 500 ms independent watchdog. Base firmware relied on FreeRTOS task watchdog. |

---

## 6. Documentation Discrepancies Found in Existing Docs

The following discrepancies were identified between existing documentation files and the
actual firmware source code (`Core/Inc/main.h` and `Core/Src/*.c`).

**Source of truth is always the code in `main.h` and the `.c` implementation files.**

### 6.1 Pin Assignment Discrepancies

| Signal | **Actual (main.h)** | PINOUT.md | PINOUT_DEFINITIVO.md | HARDWARE.md | HARDWARE_SPEC.md |
|--------|---------------------|-----------|----------------------|-------------|-------------------|
| Wheel FL | **PA0 / EXTI0** | PB0 / EXTI0 ❌ | PB5 / EXTI5 ❌ | — | PB5 ❌ |
| Wheel FR | **PA1 / EXTI1** | PB1 / EXTI1 ❌ | PB10 / EXTI10 ❌ | — | PB10 ❌ |
| Wheel RL | **PA2 / EXTI2** | PB2 / EXTI2 ❌ | PB11 / EXTI11 ❌ | — | PB11 ❌ |
| Wheel RR | **PB15 / EXTI15** | PB10 / EXTI10 ❌ | PB12 / EXTI12 ❌ | — | PB12 ❌ |
| OneWire | **PB0** | PB5 ❌ | PB0 ✅ | PB5 ❌ | PB0 ✅ |
| Pedal ADC | **PA3 / ADC1_IN4** | PA0 / ADC1_IN1 ❌ | PA3 / ADC1_IN4 ✅ | PA0 / ADC1_IN1 ❌ | PA3 ✅ |
| EN_RR | **PC13** | PC7 ❌ | PD2 ❌ | — | PD2 ❌ |
| CAN RX | **PB8 (AF9)** | PB8 ✅ | PA11 ❌ | — | PB8 ✅ |
| CAN TX | **PB9 (AF9)** | PB9 ✅ | PA12 ❌ | — | PB9 ✅ |
| DIR pins | **PC0–PC4** | Mixed ❌ | PC0–PC4 ✅ | — | PC0–PC4 ✅ |
| EN pins | **PC5,6,7,13,9** | Mixed ❌ | PC5,6,7,PD2,9 ❌(RR) | — | PC5,6,7,PD2,9 ❌(RR) |
| Relay MAIN | **PC10** | PC11 ❌ | PC10 ✅ | — | PC10 ✅ |
| Relay TRAC | **PC11** | PC12 ❌ | PC11 ✅ | — | PC11 ✅ |
| Relay DIR | **PC12** | PD2 ❌ | PC12 ✅ | — | PC12 ✅ |

### 6.2 CAN Protocol Discrepancies

| Item | **Actual (code)** | CAN_CONTRACT_FINAL.md | CAN_PROTOCOL.md (Spanish) |
|------|--------------------|-----------------------|---------------------------|
| System state 3 | **DEGRADED** | SAFE ❌ | — |
| System state 4 | **SAFE** | ERROR ❌ | — |
| System state 5 | **ERROR** | (missing) ❌ | — |
| Heartbeat byte 3 | **error_code** | "reserved = 0x00" ❌ | CRC8 ❌ |
| Fault flag bit 7 | **FAULT_CENTERING** | "(unused) Reserved" ❌ | "RESERVED" ❌ |
| STATUS_SPEED unit | **0.1 km/h** | 0.1 km/h ✅ | mm/s ❌ |
| STATUS_CURRENT format | **4× uint16 LE @ 0.01A** | 4× uint16 LE @ 0.01A ✅ | 6× uint8 @ 0.1A ❌ |
| STATUS_CURRENT count | **4 motors only (FL/FR/RL/RR)** | 4 ✅ | 6 (includes STEER+BATT) ❌ |
| Service CAN messages | **0x301, 0x302, 0x303, 0x110** | Not documented ❌ | Not documented ❌ |
| RX filter for 0x110 | **Filter 2 in code** | Not documented ❌ | Not documented ❌ |
| CMD_STEERING format | **int16 LE, degrees×10** | int16 LE, degrees×10 ✅ | int8, -100 to +100 % ❌ |
| CMD_MODE byte 0 | **Bit 0=4×4, Bit 1=tank** | Bit 0=4×4, Bit 1=tank ✅ | 0=REV, 1=NEU, 2=FWD ❌ |
| MAX_STEER_DEG | **54°** | ±45° ❌ | ±45° ❌ |
| CAN prescaler | **17** | 17 ✅ | 20 ❌ |

### 6.3 Shifter (F/N/R) — Not Present in STM32 Firmware

The following documents mention a physical shifter on PB12/PB13/PB14 that does **not exist**
in the STM32 firmware code:

- `docs/HARDWARE.md` — lists Shifter F/N/R with PB12/PB13/PB14
- `docs/HARDWARE_SPECIFICATION.md` — lists shifter sensors
- `docs/PINOUT.md` — lists PB12/PB13/PB14 as shifter pins
- `docs/PINOUT_DEFINITIVO.md` — lists PB12/PB13/PB14 as shifter pins
- `docs/CAN_PROTOCOL.md` — describes CMD_MODE as F/N/R shift

None of these pins are initialized in `MX_GPIO_Init()` in `main.c`. The shifter is NOT in
the STM32 firmware. Drive mode is controlled via CAN `CMD_MODE` (0x102) with 4×4 / tank-turn
bit flags, not a forward/neutral/reverse selector.

### 6.4 DS18B20 Sensor 5 Location Discrepancy

| Document | DS18B20 index 4 (5th sensor) |
|----------|------------------------------|
| `CAN_PROTOCOL.md` | `temp_AMB` — Ambiente ✅ |
| `HARDWARE.md` | Ambiente ✅ |
| `HARDWARE_SPECIFICATION.md` | Volante/Dirección ❌ |
| `PINOUT.md` | Ambiente ✅ |
| `PINOUT_DEFINITIVO.md` | Volante/Dirección ❌ |

The code in `motor_control.c` line 289 uses `Temperature_Get(i)` for `i` in 0–3 to populate
`traction_state.wheels[i].tempC`, leaving index 4 as the non-motor sensor. The `service_mode.h`
comment and the CAN protocol treat index 4 as **ambient temperature**. This is the intended design.

---

## Summary

This document provides the definitive hardware and sensor mapping for the STM32 firmware.
All tables are derived from source code inspection.

**For ESP32 HMI developers:**
- Use the **Sensor Index Mapping** (section 2) to correctly label each sensor value on the display.
- Use the **CAN Payload Mapping** (section 3) to decode all CAN frames.
- Use the **Service Mode Module ID Reference** (section 4) to map bitmask bits to module names.
- Be aware that DS18B20 index order depends on ROM discovery order (section 2.2 caveat).
- Battery and steering motor currents are NOT available via CAN (section 2.1 note).
