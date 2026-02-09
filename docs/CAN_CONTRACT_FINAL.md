# CAN Bus Contract — FINAL

**Revision:** 1.0
**Status:** FROZEN — Locked to release `v1.0-stm32-safety-baseline`
**Date:** 2025-06-15
**Scope:** CAN communication between STM32G474RE (safety authority) and ESP32-S3 (HMI)

Any change to this contract requires a new numbered revision and a corresponding firmware release.

---

## 1. Roles and Authority

| Role | Device | Responsibility |
|------|--------|----------------|
| Safety authority | STM32G474RE | Validates, clamps, rate-limits, or rejects every command. Controls actuators, relays, and power. Monitors sensors. Enforces fail-safe. |
| HMI (intent sender) | ESP32-S3 | Sends operator intent only. Has no direct control over actuators. Must not assume any command will be executed as sent. |

The STM32 decides what is allowed. The ESP32 requests; the STM32 disposes.

---

## 2. CAN Bus Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| Standard | CAN 2.0A (classic CAN) | `FDCAN_FRAME_CLASSIC`, `FDCAN_CLASSIC_CAN` in `can_handler.c` |
| Identifier type | Standard 11-bit | `FDCAN_STANDARD_ID` in `can_handler.c` |
| Bitrate | 500 kbps | `MX_FDCAN1_Init` in `main.c`: 170 MHz / (17 × (1 + 14 + 5)) = 500,000 bps |
| Prescaler | 17 | `NominalPrescaler = 17` |
| Time Seg 1 | 14 TQ | `NominalTimeSeg1 = 14` |
| Time Seg 2 | 5 TQ | `NominalTimeSeg2 = 5` |
| Sync Jump Width | 1 TQ | `NominalSyncJumpWidth = 1` |
| Total quanta per bit | 20 (1 + 14 + 5) | — |
| FDCAN clock source | APB1 = 170 MHz (no divider) | `FDCAN_CLOCK_DIV1` in `main.c` |
| Auto retransmission | Enabled | `AutoRetransmission = ENABLE` |
| FD bit-rate switch | Off | `FDCAN_BRS_OFF` in `can_handler.c` |
| Max payload | 8 bytes | Classic CAN frame limit |
| Topology | Point-to-point (ESP32-S3 ↔ STM32G474RE) | — |
| Pins | PB8 (FDCAN1_RX), PB9 (FDCAN1_TX) — AF9 | `main.h` |
| Error handling | Hardware auto-retransmission; protocol exception disabled (`ProtocolException = DISABLE`) | `main.c` |

### RX Filter Policy

The STM32 accepts only white-listed CAN IDs. All other messages are rejected at the hardware filter level and never reach the application.

| Filter | Type | Accepted IDs | Destination |
|--------|------|-------------|-------------|
| 0 | Dual (exact match) | 0x011 (ESP32 heartbeat) | RXFIFO0 |
| 1 | Range | 0x100 – 0x102 (ESP32 commands) | RXFIFO0 |
| Global | Reject | Everything else | Discarded |

Remote frames are rejected. Extended-ID frames are rejected.

Source: `CAN_ConfigureFilters()` in `Core/Src/can_handler.c`

---

## 3. Message List

### 3.1 ESP32 → STM32 (Commands / Heartbeat)

| CAN ID | Name | DLC | Rate | Description | Source file |
|--------|------|-----|------|-------------|-------------|
| 0x011 | HEARTBEAT_ESP32 | — | 100 ms | ESP32 alive signal. Payload is not parsed by STM32; only message arrival is checked. | `can_handler.c` |
| 0x100 | CMD_THROTTLE | 1 | 50 ms | Throttle request (percent) | `can_handler.c` |
| 0x101 | CMD_STEERING | 2 | 50 ms | Steering angle request (raw units) | `can_handler.c` |
| 0x102 | CMD_MODE | 1 | On-demand | Drive mode request | `can_handler.c` |

### 3.2 STM32 → ESP32 (Status / Heartbeat)

| CAN ID | Name | DLC | Rate | Description | Source file |
|--------|------|-----|------|-------------|-------------|
| 0x001 | HEARTBEAT_STM32 | 4 | 100 ms | System alive, state, and fault flags | `can_handler.c` |
| 0x200 | STATUS_SPEED | 8 | 100 ms | Four wheel speeds | `can_handler.c`, `main.c` |
| 0x201 | STATUS_CURRENT | 8 | 100 ms | Four motor currents | `can_handler.c`, `main.c` |
| 0x202 | STATUS_TEMP | 5 | 1000 ms | Five temperature sensors | `can_handler.c`, `main.c` |
| 0x203 | STATUS_SAFETY | 3 | 100 ms | ABS/TCS active flags and error code | `can_handler.c`, `main.c` |
| 0x204 | STATUS_STEERING | 3 | 100 ms | Actual steering angle and calibration flag | `can_handler.c`, `main.c` |
| 0x205 | STATUS_TRACTION | 4 | 100 ms | Per-wheel traction scale (ABS/TCS) | `can_handler.c`, `main.c` |
| 0x206 | STATUS_TEMP_MAP | 5 | 1000 ms | Explicit temperature sensor mapping (FL/FR/RL/RR/AMB) | `can_handler.c`, `main.c` |

### 3.3 Bidirectional (Diagnostic)

| CAN ID | Name | DLC | Rate | Description | Source file |
|--------|------|-----|------|-------------|-------------|
| 0x300 | DIAG_ERROR | 2 | On-demand | Error code and subsystem identifier | `can_handler.c` |

---

## 4. Payload Definitions

### 4.1 HEARTBEAT_STM32 (0x001) — STM32 → ESP32

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | alive_counter | uint8 | Incrementing counter, 0–255, intentional rollover |
| 1 | system_state | uint8 | Current system state (see section 6) |
| 2 | fault_flags | uint8 | Bitmask of active faults (see section 6) |
| 3 | reserved | uint8 | Always 0x00 in current firmware |

Source: `CAN_SendHeartbeat()` in `can_handler.c`

### 4.2 HEARTBEAT_ESP32 (0x011) — ESP32 → STM32

The STM32 does not parse the payload of this message. Only the arrival of a frame with ID 0x011 is used to reset the heartbeat watchdog. The ESP32 may define its own payload structure, but the STM32 ignores it.

Source: `CAN_ProcessMessages()` case `CAN_ID_HEARTBEAT_ESP32` in `can_handler.c`

### 4.3 CMD_THROTTLE (0x100) — ESP32 → STM32

| Byte | Field | Type | Unit | Range | Description |
|------|-------|------|------|-------|-------------|
| 0 | throttle_pct | uint8 | % | 0–100 | Requested throttle percentage |

The STM32 clamps this value to 0–100, rejects it entirely if not in ACTIVE state, forces zero if ABS is active, and halves it if TCS is active.

Source: `CAN_ProcessMessages()` and `Safety_ValidateThrottle()` in `safety_system.c`

### 4.4 CMD_STEERING (0x101) — ESP32 → STM32

| Byte | Field | Type | Unit | Range | Description |
|------|-------|------|------|-------|-------------|
| 0 | angle_LSB | uint8 | — | — | Low byte of signed 16-bit value |
| 1 | angle_MSB | uint8 | — | — | High byte of signed 16-bit value |

Decoding: `int16_t angle_raw = rx_payload[0] | (rx_payload[1] << 8)`
Conversion: `requested_deg = (float)angle_raw / 10.0`

The STM32 clamps the result to ±45.0°, rate-limits to 200°/s maximum, and rejects the command if not in ACTIVE state.

Source: `CAN_ProcessMessages()` and `Safety_ValidateSteering()` in `safety_system.c`

### 4.5 CMD_MODE (0x102) — ESP32 → STM32

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | mode_flags | uint8 | Bit 0: enable 4×4 (1 = 4×4, 0 = 4×2). Bit 1: tank turn (1 = enabled). Bits 2–7: reserved. |

The STM32 rejects the mode change unless in ACTIVE state and average wheel speed is below 1.0 km/h.

Source: `CAN_ProcessMessages()` and `Safety_ValidateModeChange()` in `safety_system.c`

### 4.6 STATUS_SPEED (0x200) — STM32 → ESP32

| Bytes | Field | Type | Unit | Description |
|-------|-------|------|------|-------------|
| 0–1 | speed_FL | uint16 LE | 0.1 km/h | Front-left wheel speed |
| 2–3 | speed_FR | uint16 LE | 0.1 km/h | Front-right wheel speed |
| 4–5 | speed_RL | uint16 LE | 0.1 km/h | Rear-left wheel speed |
| 6–7 | speed_RR | uint16 LE | 0.1 km/h | Rear-right wheel speed |

Encoding at source: `(uint16_t)(Wheel_GetSpeed_XX() * 10)` — value is km/h multiplied by 10.

Source: `CAN_SendStatusSpeed()` in `can_handler.c`, called from `main.c`

### 4.7 STATUS_CURRENT (0x201) — STM32 → ESP32

| Bytes | Field | Type | Unit | Description |
|-------|-------|------|------|-------------|
| 0–1 | current_FL | uint16 LE | 0.01 A | Front-left motor current |
| 2–3 | current_FR | uint16 LE | 0.01 A | Front-right motor current |
| 4–5 | current_RL | uint16 LE | 0.01 A | Rear-left motor current |
| 6–7 | current_RR | uint16 LE | 0.01 A | Rear-right motor current |

Encoding at source: `(uint16_t)(Current_GetAmps(n) * 100)` — value is amperes multiplied by 100.

Source: `CAN_SendStatusCurrent()` in `can_handler.c`, called from `main.c`

### 4.8 STATUS_TEMP (0x202) — STM32 → ESP32

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0 | temp_0 | int8 | °C | Temperature sensor 0 |
| 1 | temp_1 | int8 | °C | Temperature sensor 1 |
| 2 | temp_2 | int8 | °C | Temperature sensor 2 |
| 3 | temp_3 | int8 | °C | Temperature sensor 3 |
| 4 | temp_4 | int8 | °C | Temperature sensor 4 |

Values are truncated to integer degrees Celsius. Five DS18B20 sensors.

Source: `CAN_SendStatusTemp()` in `can_handler.c`, called from `main.c`

### 4.9 STATUS_SAFETY (0x203) — STM32 → ESP32

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | abs_active | uint8 | 1 = ABS currently active, 0 = inactive |
| 1 | tcs_active | uint8 | 1 = TCS currently active, 0 = inactive |
| 2 | error_code | uint8 | Current safety error code (see section 6) |

Source: `CAN_SendStatusSafety()` in `can_handler.c`, called from `main.c`

### 4.10 STATUS_STEERING (0x204) — STM32 → ESP32

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0 | angle_LSB | uint8 | — | Low byte of signed 16-bit actual angle |
| 1 | angle_MSB | uint8 | — | High byte of signed 16-bit actual angle |
| 2 | calibrated | uint8 | — | 1 = encoder calibrated, 0 = not calibrated |

Decoding: `int16_t angle_raw = byte[0] | (byte[1] << 8)`
Conversion: `actual_deg = (float)angle_raw / 10.0`

Encoding at source: `(int16_t)(Steering_GetCurrentAngle() * 10)`

Source: `CAN_SendStatusSteering()` in `can_handler.c`, called from `main.c`

### 4.11 STATUS_TRACTION (0x205) — STM32 → ESP32

| Byte | Field | Type | Unit | Range | Description |
|------|-------|------|------|-------|-------------|
| 0 | traction_FL | uint8 | % | 0–100 | Front-left wheel traction scale |
| 1 | traction_FR | uint8 | % | 0–100 | Front-right wheel traction scale |
| 2 | traction_RL | uint8 | % | 0–100 | Rear-left wheel traction scale |
| 3 | traction_RR | uint8 | % | 0–100 | Rear-right wheel traction scale |

Encoding: `(uint8_t)(safety_status.wheel_scale[i] * 100.0f)`

- 100 = full power available (no ABS/TCS intervention on this wheel)
- 0 = wheel fully inhibited (ABS has cut this wheel)
- Intermediate values = TCS is progressively limiting this wheel

The values are the same `wheel_scale[4]` array used by `Traction_Update()` to modulate per-wheel PWM. No recalculation is performed; the ESP32 receives exactly what the STM32 applies.

Source: `CAN_SendStatusTraction()` in `can_handler.c`, called from `main.c`

### 4.12 STATUS_TEMP_MAP (0x206) — STM32 → ESP32

| Byte | Field | Type | Unit | Range | Description |
|------|-------|------|------|-------|-------------|
| 0 | temp_motor_FL | int8 | °C | −128 to +127 | Motor FL temperature (DS18B20 sensor index 0) |
| 1 | temp_motor_FR | int8 | °C | −128 to +127 | Motor FR temperature (DS18B20 sensor index 1) |
| 2 | temp_motor_RL | int8 | °C | −128 to +127 | Motor RL temperature (DS18B20 sensor index 2) |
| 3 | temp_motor_RR | int8 | °C | −128 to +127 | Motor RR temperature (DS18B20 sensor index 3) |
| 4 | temp_ambient | int8 | °C | −128 to +127 | Ambient temperature (DS18B20 sensor index 4) |

Sensor index mapping:

| Byte | Sensor index | Physical location |
|------|-------------|-------------------|
| 0 | `Temperature_Get(0)` | Motor front-left |
| 1 | `Temperature_Get(1)` | Motor front-right |
| 2 | `Temperature_Get(2)` | Motor rear-left |
| 3 | `Temperature_Get(3)` | Motor rear-right |
| 4 | `Temperature_Get(4)` | Ambient |

Values are the same DS18B20 readings used by `Safety_CheckTemperature()`. No filtering or recalculation is performed. If a sensor is disabled in Service Mode, the last read value is still reported (the safety system handles threshold checks independently).

The existing STATUS_TEMP (0x202) message is unchanged and continues to transmit at the same rate. This message provides an explicit byte-to-sensor mapping for unambiguous HMI display.

Source: `CAN_SendStatusTempMap()` in `can_handler.c`, called from `main.c`

### 4.13 DIAG_ERROR (0x300) — Both Directions

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | error_code | uint8 | Error type (see safety error codes) |
| 1 | subsystem | uint8 | 0 = Global, 1 = Motor, 2 = Sensor, 3 = CAN |

Source: `CAN_SendError()` in `can_handler.c`

---

## 5. Requested vs. Actual Signals

| Signal | Requested (ESP32 → STM32) | Actual (STM32 → ESP32) |
|--------|---------------------------|------------------------|
| Throttle | 0x100 byte 0: requested % (0–100) | No direct actual throttle message. The STM32 applies the validated value internally. |
| Steering angle | 0x101 bytes 0–1: requested angle (°/10) | 0x204 bytes 0–1: actual measured angle (°/10) |
| Drive mode | 0x102 byte 0: requested mode flags | No direct actual mode message. Mode change is applied or silently rejected. |
| Wheel speed | Not applicable | 0x200: actual measured speeds |
| Motor current | Not applicable | 0x201: actual measured currents |
| Temperature | Not applicable | 0x202: actual measured temperatures |
| ABS/TCS state | Not applicable | 0x203: actual safety flags |

The ESP32 must never assume that a requested value has been applied. It must read the actual values from the status messages to confirm system state.

---

## 6. Heartbeat Definition

### STM32 Heartbeat (0x001)

| Property | Value |
|----------|-------|
| Transmission rate | Every 100 ms (10 Hz) |
| Payload length | 4 bytes |
| Byte 0 | alive_counter — cyclic 0–255, intentional rollover |
| Byte 1 | system_state — see table below |
| Byte 2 | fault_flags — see bitmask table below |
| Byte 3 | reserved — 0x00 |

**System states (byte 1):**

| Value | State | Meaning |
|-------|-------|---------|
| 0 | BOOT | Power-on, peripherals initializing. No commands accepted. |
| 1 | STANDBY | Ready, waiting for ESP32 heartbeat. No commands accepted. |
| 2 | ACTIVE | Normal operation. Commands are accepted and validated. |
| 3 | SAFE | Fault detected. Actuators inhibited. Traction stopped, steering centered. |
| 4 | ERROR | Unrecoverable fault. Relays de-energized. Manual reset required. |

Source: `SystemState_t` in `safety_system.h`

**Fault flags bitmask (byte 2):**

| Bit | Mask | Name | Meaning |
|-----|------|------|---------|
| 0 | 0x01 | FAULT_CAN_TIMEOUT | ESP32 heartbeat not received within 250 ms |
| 1 | 0x02 | FAULT_TEMP_OVERLOAD | Motor temperature exceeds 90 °C |
| 2 | 0x04 | FAULT_CURRENT_OVERLOAD | Motor current exceeds 25 A |
| 3 | 0x08 | FAULT_ENCODER_ERROR | Sensor fault (includes encoder) |
| 4 | 0x10 | FAULT_WHEEL_SENSOR | Sensor fault (includes wheel speed) |
| 5 | 0x20 | FAULT_ABS_ACTIVE | ABS is currently intervening |
| 6 | 0x40 | FAULT_TCS_ACTIVE | TCS is currently intervening |
| 7 | 0x80 | (unused) | Reserved |

Note: Bits 3 and 4 are both set when `SAFETY_ERROR_SENSOR_FAULT` is the active error. They are not independently assignable in the current implementation.

Source: `Safety_GetFaultFlags()` in `safety_system.c`, fault flag defines in `safety_system.h`

### ESP32 Heartbeat (0x011)

| Property | Value |
|----------|-------|
| Expected rate | Every 100 ms (10 Hz) |
| Timeout | 250 ms — if no message with ID 0x011 is received within this window, the STM32 triggers SAFE state |
| Payload parsing | None. The STM32 does not read the payload. Only frame arrival matters. |

### Timeout Behavior

1. The STM32 checks for ESP32 heartbeat timeout every 10 ms in the main loop via `Safety_CheckCANTimeout()`.
2. If `HAL_GetTick() - last_can_rx_time > 250 ms`:
   - `SAFETY_ERROR_CAN_TIMEOUT` is set.
   - System transitions to SAFE state.
   - `Safety_FailSafe()` is called: traction motors stopped, steering centered.
3. If heartbeat is restored while in SAFE state with CAN timeout error:
   - Error is cleared.
   - System transitions back to ACTIVE.
4. If heartbeat is received while in STANDBY with no faults:
   - System transitions to ACTIVE.

Source: `Safety_CheckCANTimeout()` in `safety_system.c`

---

## 7. Fault and State Signaling

### Safety Error Codes

| Code | Name | Trigger |
|------|------|---------|
| 0 | SAFETY_ERROR_NONE | Normal operation |
| 1 | SAFETY_ERROR_OVERCURRENT | Any motor current > 25 A |
| 2 | SAFETY_ERROR_OVERTEMP | Any motor temperature > 90 °C |
| 3 | SAFETY_ERROR_CAN_TIMEOUT | No ESP32 heartbeat for > 250 ms |
| 4 | SAFETY_ERROR_SENSOR_FAULT | Sensor plausibility check failed (temperature outside −40 to 125 °C, current negative or > 50 A, speed negative or > 60 km/h) |
| 5 | SAFETY_ERROR_MOTOR_STALL | Reserved (not implemented in current firmware) |
| 6 | SAFETY_ERROR_EMERGENCY_STOP | Emergency stop triggered |
| 7 | SAFETY_ERROR_WATCHDOG | Independent watchdog timeout (IWDG, 500 ms period) |

Source: `Safety_Error_t` in `safety_system.h`, threshold defines in `safety_system.c`

### SAFE State Actions

When the system enters SAFE state, `Safety_FailSafe()` executes:

1. `Traction_EmergencyStop()` — all traction motor outputs are cut immediately.
2. `Steering_SetAngle(0.0f)` — steering is driven to center position.
3. Relays remain energized (dynamic motor braking is still available).

### ERROR State Actions

When the system enters ERROR state, `Safety_PowerDown()` executes:

1. `Traction_EmergencyStop()` — motors stopped.
2. `Relay_PowerDown()` — all relays de-energized (Direction → Traction → Main, in that order).
3. Manual hardware reset is required to exit ERROR state.

### What the ESP32 Must Do

- Monitor `system_state` (byte 1 of 0x001) at all times.
- Display fault information from `fault_flags` (byte 2 of 0x001) and `error_code` (byte 2 of 0x203) to the operator.
- Continue sending heartbeat at 100 ms intervals regardless of system state.
- Cease sending actuator commands when `system_state` is not ACTIVE (2). The STM32 will reject them, but sending unnecessary traffic is discouraged.

### What the ESP32 Must Not Do

- Must not attempt to override or work around a SAFE or ERROR state.
- Must not interpret the absence of a fault flag as permission to exceed physical limits.
- Must not rely on the STM32 being in ACTIVE state without verifying it in every heartbeat.

---

## 8. Invalid and Forbidden Behavior

### Messages That Are Ignored

- Any CAN ID not in the set {0x011, 0x100, 0x101, 0x102} is hardware-filtered and never reaches the STM32 application.
- A `CMD_THROTTLE` (0x100) with DLC < 1 is silently dropped.
- A `CMD_STEERING` (0x101) with DLC < 2 is silently dropped.
- A `CMD_MODE` (0x102) with DLC < 1 is silently dropped.
- All commands received while `system_state != ACTIVE` are rejected. Throttle returns 0; steering holds the current position; mode changes return false.

### Conditions That Force SAFE State

| Condition | Threshold | Source |
|-----------|-----------|--------|
| ESP32 heartbeat lost | > 250 ms | `Safety_CheckCANTimeout()` |
| Motor overcurrent | > 25 A on any channel | `Safety_CheckCurrent()` |
| Motor overtemperature | > 90 °C on any sensor | `Safety_CheckTemperature()` |
| Temperature out of range | < -40 °C or > 125 °C | `Safety_CheckSensors()` |
| Current out of range | < 0 A or > 50 A | `Safety_CheckSensors()` |
| Wheel speed out of range | < 0 or > 60 km/h | `Safety_CheckSensors()` |

### Conditions That Force ERROR State (Unrecoverable)

| Condition | Effect |
|-----------|--------|
| Emergency stop triggered | `Safety_EmergencyStop()` — motors stopped, relays de-energized, state = ERROR |
| IWDG watchdog timeout | Hardware reset (500 ms period, fed in main loop) |
| `Error_Handler()` | Interrupts disabled, all GPIOC outputs driven low, infinite loop |

### What the ESP32 Must Not Assume

- A sent command does not imply it was executed. Always verify via status messages.
- The throttle value received by the STM32 may be clamped (0–100%), zeroed (ABS active), or halved (TCS active).
- The steering angle may be clamped (±45°) or rate-limited (200°/s max).
- A mode change may be silently rejected if speed > 1 km/h.
- The STM32 may transition to SAFE at any time without prior warning.
- ABS and TCS interventions happen without ESP32 consent and override the requested throttle.

---

## 9. Command Validation Summary

| Command | Validation | Limits | Source |
|---------|------------|--------|--------|
| Throttle (0x100) | Clamped to 0–100%. Zeroed if ABS active. Halved if TCS active. Rejected if not ACTIVE. | 0.0–100.0 % | `Safety_ValidateThrottle()` |
| Steering (0x101) | Clamped to ±45°. Rate-limited to 200°/s. Returns current angle if not ACTIVE. | ±45.0°, 200°/s max rate | `Safety_ValidateSteering()` |
| Mode (0x102) | Rejected if not ACTIVE or if average wheel speed > 1 km/h. | Speed < 1.0 km/h | `Safety_ValidateModeChange()` |

---

## 10. Versioning and Stability

This document describes the CAN protocol as implemented in the firmware tagged `v1.0-stm32-safety-baseline`.

| Property | Value |
|----------|-------|
| Contract revision | 1.0 |
| Firmware tag | `v1.0-stm32-safety-baseline` |
| Contract status | FROZEN |
| Change policy | Any modification to CAN IDs, payloads, timing, or behavior requires a new contract revision number and a new firmware release tag. |
| Backward compatibility | Not guaranteed across revisions. Each revision is self-contained. |

The source files that define this contract are:

| File | Content |
|------|---------|
| `Core/Inc/can_handler.h` | CAN ID definitions, timeout constant, function prototypes |
| `Core/Src/can_handler.c` | TX/RX implementation, RX filters, message processing |
| `Core/Inc/safety_system.h` | State machine, fault flags, error codes, validation prototypes |
| `Core/Src/safety_system.c` | Safety logic, thresholds, command validation, fail-safe actions |
| `Core/Src/main.c` | Main loop timing, status message encoding and transmission |
| `Core/Inc/main.h` | Hardware pin definitions, sensor counts |
