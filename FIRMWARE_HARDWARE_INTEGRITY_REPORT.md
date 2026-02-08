# FIRMWARE & HARDWARE INTEGRITY REPORT

**Target Repository:** STM32-Control-Coche-Marcos (current branch)
**Reference Base:** [FULL-FIRMWARE-Coche-Marcos](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos)
**MCU:** STM32G474RE (Cortex-M4F, 170 MHz)
**Date:** 2026-02-08

---

## SECTION 1 — SYSTEM OVERVIEW

The STM32G474RE firmware acts as the **low-level motor control and safety authority** for a 4-wheel-drive electric vehicle. It controls traction motors (4×), steering motor (1×), relay power sequencing, and sensor acquisition. It communicates with an ESP32-S3 via CAN bus (FDCAN1 @ 500 kbps).

**Role of STM32 vs ESP32:**
- **STM32G474RE** (this repo): Motor PWM generation, PID steering control, safety state machine, sensor reading (wheel speed, current, temperature, pedal), relay control. It is the **safety authority** — all ESP32 commands pass through validation gates before reaching actuators.
- **ESP32-S3** (base repo): HMI (TFT display, touch), audio (DFPlayer Mini), lighting (WS2812B LEDs), obstacle detection (TOFSense-M S LiDAR), gear shifter reading (MCP23017), ignition key detection, adaptive cruise control. It sends commands to STM32 via CAN.

**Standalone operation:**
The STM32 firmware **cannot operate the vehicle standalone**. Proof:
- File: `safety_system.c`, function `Safety_CheckCANTimeout()`, lines 324–339: the system transitions from STANDBY→ACTIVE only when ESP32 heartbeat is received. Without CAN heartbeat, the system remains in STANDBY and `Safety_IsCommandAllowed()` (line 97–100) returns `false`, preventing any actuator commands.
- File: `main.c`, line 106–111: throttle demand is set to 0.0 when `Safety_IsCommandAllowed()` returns `false`.
- CAN timeout of 250 ms triggers transition to SAFE state (`safety_system.c`, line 324–326).

The base firmware (FULL-FIRMWARE-Coche-Marcos) is a **monolithic ESP32-S3** firmware that runs ALL vehicle functions on a single MCU (PlatformIO/Arduino framework). The STM32 repo is a **split-architecture port** where motor control and safety functions were extracted to a dedicated STM32 MCU.

---

## SECTION 2 — POWER & RELAY CONTROL

### Ignition Key Detection

**NOT IMPLEMENTED** on the STM32.

In the base firmware, ignition detection is handled by the ESP32-S3:
- `include/pins.h`: `PIN_KEY_ON = GPIO 40`, `PIN_KEY_OFF = GPIO 41`
- `src/system/power_mgmt.cpp`: Power management logic on ESP32 side.

The STM32 firmware has no GPIO defined for ignition key and no code referencing it. The STM32 relies on the ESP32 to detect ignition and initiate the CAN heartbeat that triggers STANDBY→ACTIVE transition.

### Relays Controlled by STM32

| Name | GPIO Pin | Defined In | Voltage Domain | Purpose |
|------|----------|------------|----------------|---------|
| `PIN_RELAY_MAIN` | PC10 | `main.h`, line 39 | System (12V/24V bus) | Main power relay — first on, last off |
| `PIN_RELAY_TRAC` | PC11 | `main.h`, line 40 | 24V | Traction motor power bus |
| `PIN_RELAY_DIR` | PC12 | `main.h`, line 41 | 12V | Steering motor power bus |

The base firmware defines **4 relays** (`src/control/relays.cpp`): MAIN, TRAC, DIR, and SPARE (plus an optional MEDIA relay via `#ifdef PIN_RELAY_MEDIA`). The STM32 firmware implements only **3 relays**. The SPARE relay is **NOT IMPLEMENTED**.

### Startup Sequence

File: `safety_system.c`, function `Relay_PowerUp()`, lines 118–127:
```
1. PIN_RELAY_MAIN → HIGH
2. HAL_Delay(50 ms)                    ← RELAY_MAIN_SETTLE_MS
3. PIN_RELAY_TRAC → HIGH
4. HAL_Delay(20 ms)                    ← RELAY_TRACTION_SETTLE_MS
5. PIN_RELAY_DIR  → HIGH
```

This is a **blocking** sequence using `HAL_Delay()`. The base firmware uses a **non-blocking** state-machine approach (`SEQ_EN_ENABLE_MAIN` → `SEQ_EN_ENABLE_TRAC` → `SEQ_EN_ENABLE_DIR` in `relays.cpp`). This is a behavioral difference: the STM32 blocks the main loop for ~70 ms during power-up.

### Shutdown Sequence

File: `safety_system.c`, function `Relay_PowerDown()`, lines 129–135:
```
1. PIN_RELAY_DIR  → LOW
2. PIN_RELAY_TRAC → LOW
3. PIN_RELAY_MAIN → LOW
```

Reverse order, no delays. Matches base firmware disable logic.

### Delayed Shutdown for Audio Goodbye

**NO.** Audio is handled by the ESP32-S3 (DFPlayer Mini on UART1). The STM32 has no audio hardware and no delay logic for audio playback before power-down.

---

## SECTION 3 — TRACTION SYSTEM (24V)

### Motors Controlled

4 traction motors via TIM1 PWM at 20 kHz (`main.c`, `MX_TIM1_Init()`, lines 282–303):

| Motor | Timer Channel | PWM Pin | Direction Pin | Enable Pin |
|-------|--------------|---------|---------------|------------|
| FL (Front Left) | TIM1_CH1 | PA8 | PC0 | PC5 |
| FR (Front Right) | TIM1_CH2 | PA9 | PC1 | PC6 |
| RL (Rear Left) | TIM1_CH3 | PA10 | PC2 | PC7 |
| RR (Rear Right) | TIM1_CH4 | PA11 | PC3 | PC13 |

Assigned in `motor_control.c`, `Motor_Init()`, lines 123–149.

### Pedal Input Path

```
ADC1 (PA3, ADC_CHANNEL_4, 12-bit)
  → Pedal_Update() [sensor_manager.c:75–83] — raw ADC → 0–100%
  → Safety_ValidateThrottle() [safety_system.c:141–155] — clamp + ABS/TCS override
  → Traction_SetDemand() [motor_control.c:189–231]:
      A) EMA noise filter (α = 0.15, from base pedal.cpp)
      B) Ramp rate limiter (50 %/s up, 100 %/s down)
      → Final clamp ±100%
  → Traction_Update() [motor_control.c:243–289] — compute PWM from demand
```

### Gear Logic (P / N / D1 / D2 / R)

**NOT IMPLEMENTED** on the STM32.

In the base firmware, the gear shifter is read by the ESP32 via MCP23017 (`src/input/shifter.cpp`), using GPIOB0–B4 for P, R, N, D1, D2 positions. The base firmware gear logic includes:
- Speed-based reverse lockout (max 3 km/h for R, `shifter.cpp` line 128)
- Audio feedback per gear

The STM32 CAN protocol (`can_handler.h`, lines 23–25) defines three command IDs from ESP32:
- `CAN_ID_CMD_THROTTLE` (0x100): 1-byte unsigned throttle (0–255)
- `CAN_ID_CMD_STEERING` (0x101): 2-byte signed angle
- `CAN_ID_CMD_MODE` (0x102): 1-byte mode flags (bit 0 = 4x4, bit 1 = tank turn)

**There is no CAN command for gear selection or reverse.** The throttle command is received as a `uint8_t` (`can_handler.c`, line 270: `float requested_pct = (float)rx_payload[0]`), which cannot encode a negative (reverse) value. The `Safety_ValidateThrottle()` function clamps to 0–100% (`safety_system.c`, lines 147–148), and the `Traction_SetDemand()` function's ramp limiter can handle ±100%, but the only path that can produce a negative demand is the internal `Traction_SetDemand()` API — never from CAN. Reverse direction via CAN is therefore **inaccessible**.

The mode command handles:
- **4x2 / 4x4**: `can_handler.c`, line 288–289 → `Traction_SetMode4x4()` (`motor_control.c`, line 233)
- **Tank turn**: `can_handler.c`, line 289 → `Traction_SetAxisRotation()` (`motor_control.c`, line 238)

Speed limits per gear (D1/D2 differentiation) are **NOT IMPLEMENTED**.

### ABS / TCS

**ABS** — File: `safety_system.c`, `ABS_Update()`, lines 216–248:
- Detection: Compares each wheel speed to the 4-wheel average; slip = ((avg − wheel) × 100) / avg.
- Threshold: 15% (`ABS_SLIP_THRESHOLD`, line 20). Matches base `abs_system.cpp: slipThreshold = 15.0f`.
- Minimum speed: 10 km/h (line 225). Matches base `abs_system.cpp: minSpeedKmh = 10.0f`.
- Intervention: Cuts throttle to 0 (line 243: `Traction_SetDemand(0)`).
- Scope: **Per-wheel detection** (bitmask `abs_wheel_mask`), **global throttle cut**.

**TCS** — File: `safety_system.c`, `TCS_Update()`, lines 255–287:
- Detection: Compares each wheel speed to the 4-wheel average; slip = ((wheel − avg) × 100) / avg.
- Threshold: 15% (`TCS_SLIP_THRESHOLD`, line 21). Matches base `tcs_system.cpp: slipThreshold = 15.0f`.
- Minimum speed: 3 km/h (line 264). Matches base `tcs_system.cpp: minSpeedKmh = 3.0f`.
- Intervention: Halves current pedal demand (line 282: `Traction_SetDemand(Pedal_GetPercent() / 2.0f)`).
- Scope: **Per-wheel detection** (bitmask `tcs_wheel_mask`), **global throttle reduction**.

### Emergency Stop Behavior

File: `motor_control.c`, `Traction_EmergencyStop()`, lines 291–309:
- Disables all 5 motors (4 traction + steering).
- Sets all PWM to 0.
- Resets pedal filter/ramp state to ensure immediate zero demand.
- Resets `demandPct` to 0.

Called from: `Safety_FailSafe()` (line 427) and `Safety_EmergencyStop()` (line 417).

---

## SECTION 4 — STEERING SYSTEM (12V)

### Steering Motor Control Path

Single motor via TIM8_CH3 at 20 kHz:

| Parameter | Value | Defined In |
|-----------|-------|------------|
| Timer | TIM8 | `main.c`, `MX_TIM8_Init()`, lines 330–349 |
| Channel | CH3 | `motor_control.c`, line 141 |
| PWM Pin | PC8 | `main.h`, line 22 |
| Direction Pin | PC4 | `main.h`, line 29 |
| Enable Pin | PC9 | `main.h`, line 36 |

Control path:
```
CAN steering command (0x101) or local pedal → Safety_ValidateSteering()
  → Steering_SetAngle() [motor_control.c:320–342]
      → Ackermann_ComputeWheelAngles() [ackermann.c:20–60]
      → setpoint = angle × ENCODER_CPR / 360
  → Steering_ControlLoop() [motor_control.c:344–399] (every 10 ms)
      → PID_Compute() [motor_control.c:595–606]
      → Motor_SetPWM / Motor_SetDirection / Motor_Enable
```

### Encoder Type and Handling

- Model: E6B2-CWZ6C, 1200 PPR
- Mode: Quadrature (A/B channels), TIM2 encoder mode
- Counts per revolution: 4800 (1200 × 4) (`main.h`, line 15)
- Resolution: 0.075° per count
- Pins: PA15 (TIM2_CH1 = Channel A), PB3 (TIM2_CH2 = Channel B)
- MSP Init: `stm32g4xx_hal_msp.c`, `HAL_TIM_Encoder_MspInit()`, lines 144–169

**Z-index (PB4):** Defined in `main.h` line 52 (`PIN_ENC_Z = GPIO_PIN_4`) but **intentionally NOT USED**. Explicitly documented in `motor_control.c`, lines 79–88: no EXTI4 hardware initialization exists, and steering uses relative positioning zeroed at `Steering_Init()`.

### Calibration Model Used

**Relative zero calibration.** File: `motor_control.c`, `Steering_Init()`, lines 170–183:
- Counter is zeroed at current physical position: `__HAL_TIM_SET_COUNTER(&htim2, 0)` (line 176).
- `steering_calibrated` is set to 1 (line 177).
- No search for absolute reference (Z pulse not used).
- This means calibration accuracy depends on the steering being physically centered when the MCU boots.

### Fault Detection Mechanisms

File: `motor_control.c`, `Encoder_CheckHealth()`, lines 430–478. Three fault classes:

1. **Out-of-range** (lines 444–447): Counter exceeds ±4933 counts (±370° of steering wheel travel = 350° max + 20° margin, defined by `ENC_MAX_COUNTS` at line 93).
2. **Implausible jump** (lines 452–459): Delta between consecutive reads exceeds 100 counts per cycle (`ENC_MAX_JUMP`, line 96). At 200°/s steering rate this would be ~27 counts; 100 counts corresponds to ~750°/s.
3. **Frozen value** (lines 465–475): Encoder count unchanged for 200 ms (`ENC_FROZEN_TIMEOUT_MS`, line 100) while PID output exceeds 10% (`ENC_MOTOR_ACTIVE_PCT`, line 103).

Faults are **latched** (line 436: `if (enc_fault) return;`). Only a full system reset clears the latch.

On fault, `Safety_CheckEncoder()` (`safety_system.c`, lines 398–410) raises `SAFETY_ERROR_SENSOR_FAULT` and transitions to SAFE state, which calls `Steering_Neutralize()` (`motor_control.c`, lines 492–499) to cut PWM and disable the H-bridge.

### Ackermann Geometry

**Where computed:** `ackermann.c`, `Ackermann_ComputeWheelAngles()`, lines 20–60.

**Where applied:** Called from `Steering_SetAngle()` in `motor_control.c`, line 338. The computed per-wheel angles (`steer_fl_deg`, `steer_fr_deg`) are stored but the single steering motor PID targets the road angle (the mechanical linkage translates to Ackermann geometry).

**Constants used** (from `vehicle_physics.h`, lines 12–21):

| Constant | Value | Description |
|----------|-------|-------------|
| `WHEELBASE_M` | 0.95 m | Front-to-rear axle distance |
| `TRACK_WIDTH_M` | 0.70 m | Left-to-right wheel distance |
| `MAX_STEER_DEG` | 54.0° | Maximum road-wheel angle |
| `STEERING_WHEEL_MAX_DEG` | 350.0° | Steering wheel mechanical travel |
| `WHEEL_CIRCUM_MM` | 1100.0 mm | Wheel circumference |

**Equations:** Standard Ackermann bicycle-model extension:
- `R = L / tan(|road_angle|)` (centreline turn radius)
- `inner = atan(L / (R − T/2))`
- `outer = atan(L / (R + T/2))`

Both outputs clamped to `±MAX_STEER_DEG`. Left turn: FL = inner, FR = outer. Right turn: FL = outer, FR = inner.

Also independently implemented in `motor_control.c`, `Ackermann_Compute()` (lines 505–530), which computes inner/outer angles and returns them in an `AckermannResult_t` struct. This function is **IMPLEMENTED BUT UNUSED** — the main code path calls `Ackermann_ComputeWheelAngles()` from `ackermann.c` instead.

---

## SECTION 5 — SAFETY & STATES

### Full System State Machine

```
BOOT (0) → STANDBY (1) → ACTIVE (2) ⇄ SAFE (3) → ERROR (4)
```

Defined in `safety_system.h`, lines 43–49.

### Conditions

| State | Entry Condition | File / Function / Lines |
|-------|----------------|------------------------|
| BOOT | Power-on reset | `safety_system.c`, `Safety_Init()`, line 211: `system_state = SYS_STATE_BOOT` |
| STANDBY | Peripheral init complete | `main.c`, line 67: `Safety_SetState(SYS_STATE_STANDBY)` (after all `MX_*_Init()` and module inits) |
| ACTIVE | ESP32 heartbeat received AND no active faults | `safety_system.c`, `Safety_CheckCANTimeout()`, lines 328–332: requires `system_state == SYS_STATE_STANDBY` AND `safety_error == SAFETY_ERROR_NONE`. Also `Safety_SetState(SYS_STATE_ACTIVE)` (lines 69–76) requires `safety_error == SAFETY_ERROR_NONE`, calls `Relay_PowerUp()`. |
| SAFE | Fault detected (CAN timeout, overcurrent, overtemp, sensor fault, encoder fault) | `safety_system.c`, `Safety_SetState(SYS_STATE_SAFE)` (lines 79–84): calls `Safety_FailSafe()` which invokes `Traction_EmergencyStop()` and steers to center (or neutralizes if encoder faulted). Recovery: SAFE→ACTIVE if fault clears (lines 334–338). |
| ERROR | Unrecoverable fault (emergency stop) | `safety_system.c`, `Safety_EmergencyStop()` (lines 414–423): sets `SYS_STATE_ERROR`, calls `Relay_PowerDown()`. Also `Safety_SetState(SYS_STATE_ERROR)` (lines 87–90): calls `Safety_PowerDown()`. |

### CAN Timeout Behavior

File: `safety_system.c`, `Safety_CheckCANTimeout()`, lines 320–340.
- Timeout threshold: 250 ms (`CAN_TIMEOUT_MS`, line 24).
- If `(HAL_GetTick() - last_can_rx_time) > 250 ms`: sets `SAFETY_ERROR_CAN_TIMEOUT`, transitions to SAFE.
- If heartbeat restored while in SAFE with CAN_TIMEOUT error: clears error and transitions to ACTIVE (lines 334–338).
- `last_can_rx_time` is updated by `Safety_UpdateCANRxTime()` (line 345), called from both `CAN_ProcessMessages()` (on ESP32 heartbeat, `can_handler.c`, line 265) and `HAL_FDCAN_RxFifo0Callback()` (`stm32g4xx_it.c`, line 127).

### What Prevents Motion Without ESP32

1. System starts in BOOT, transitions to STANDBY (`main.c`, line 67).
2. STANDBY→ACTIVE requires ESP32 CAN heartbeat (`safety_system.c`, lines 328–332).
3. `Safety_IsCommandAllowed()` returns `true` only in ACTIVE state (`safety_system.c`, lines 97–100).
4. `main.c`, lines 106–111: throttle demand is forced to 0.0 when commands are not allowed.
5. CAN timeout (250 ms without heartbeat) forces SAFE state, which calls `Traction_EmergencyStop()`.

---

## SECTION 6 — SENSORS

### Sensors Handled by STM32

| Sensor | Type | Quantity | Interface | Pins | File / Lines |
|--------|------|----------|-----------|------|-------------|
| Wheel speed | LJ12A3 inductive | 4 | EXTI interrupt | PA0 (FL), PA1 (FR), PA2 (RL), PB15 (RR) | `sensor_manager.c`, lines 22–63; `main.c` GPIO init lines 231–237; `stm32g4xx_it.c`, lines 79–101 |
| Current | INA226 (via TCA9548A) | 6 | I2C1 (PB6/PB7) | Channels 0–5 | `sensor_manager.c`, lines 92–152; `main.h` line 71: `NUM_INA226 = 6` |
| Temperature | DS18B20 | 5 | OneWire bit-bang (PB0) | PB0 | `sensor_manager.c`, lines 154–438; `main.h` line 72: `NUM_DS18B20 = 5` |
| Pedal | Hall-effect (A1324LUA-T) | 1 | ADC1_IN4 | PA3 | `sensor_manager.c`, lines 70–86; `main.c`, `MX_ADC1_Init()`, lines 351–371 |
| Steering encoder | E6B2-CWZ6C | 1 | TIM2 quadrature | PA15 (A), PB3 (B) | `main.c`, `MX_TIM2_Init()`, lines 306–328 |

**Obstacle sensors: NO.** No obstacle detection code exists in the STM32 firmware. In the base firmware, obstacle detection is handled entirely by the ESP32 via TOFSense-M S LiDAR sensor (`src/sensors/obstacle_detection.cpp`, `src/safety/obstacle_safety.cpp`).

**Unused sensor definitions:**
- `PIN_ENC_Z` (PB4, `main.h` line 52): Defined but **intentionally not used** (no EXTI4 init, no Z-index handling code). Documented in `motor_control.c`, lines 79–88.
- `Wheel_GetRPM_FL()` (`sensor_manager.c`, line 64 / `sensor_manager.h`, line 25): Implemented for FL only; RPM values for FR/RL/RR are computed internally but no getter functions exist for them. **IMPLEMENTED BUT UNUSED** (never called from main loop or any other module).

**Sensor count differences vs base firmware:**
- Base firmware: 4× DS18B20, 6× INA226 (channels 0–3 motors, 4 battery, 5 steering).
- STM32 firmware: 5× DS18B20 (`NUM_DS18B20 = 5`), 6× INA226 (`NUM_INA226 = 6`).
- The 5th DS18B20 may correspond to ambient or controller temperature — no explicit mapping documentation exists in code.

---

## SECTION 7 — CAN & COMMUNICATION

### All CAN IDs Used

**TX (STM32 → ESP32):**

| CAN ID | Name | Period | DLC | Payload | File / Lines |
|--------|------|--------|-----|---------|-------------|
| 0x001 | `CAN_ID_HEARTBEAT_STM32` | 100 ms | 4 | [0] alive_counter, [1] system_state, [2] fault_flags, [3] reserved | `can_handler.c`, lines 131–150 |
| 0x200 | `CAN_ID_STATUS_SPEED` | 100 ms | 8 | 4× uint16_t LE: FL, FR, RL, RR (×10 km/h) | `can_handler.c`, lines 152–166 |
| 0x201 | `CAN_ID_STATUS_CURRENT` | 100 ms | 8 | 4× uint16_t LE: FL, FR, RL, RR (×100 A) | `can_handler.c`, lines 168–182 |
| 0x202 | `CAN_ID_STATUS_TEMP` | 1000 ms | 5 | 5× int8_t: t1–t5 (°C) | `can_handler.c`, lines 184–195 |
| 0x203 | `CAN_ID_STATUS_SAFETY` | 100 ms | 3 | [0] ABS active, [1] TCS active, [2] error_code | `can_handler.c`, lines 197–205 |
| 0x204 | `CAN_ID_STATUS_STEERING` | 100 ms | 3 | [0–1] int16_t LE angle (×10°), [2] calibrated | `can_handler.c`, lines 207–216 |
| 0x300 | `CAN_ID_DIAG_ERROR` | On-demand | 2 | [0] error_code, [1] subsystem | `can_handler.c`, lines 218–225 |

**RX (ESP32 → STM32):**

| CAN ID | Name | DLC | Payload | File / Lines |
|--------|------|-----|---------|-------------|
| 0x011 | `CAN_ID_HEARTBEAT_ESP32` | any | Heartbeat signal | `can_handler.c`, lines 263–266 |
| 0x100 | `CAN_ID_CMD_THROTTLE` | ≥1 | [0] uint8_t throttle % | `can_handler.c`, lines 268–273 |
| 0x101 | `CAN_ID_CMD_STEERING` | ≥2 | [0–1] int16_t LE angle (×10°) | `can_handler.c`, lines 275–283 |
| 0x102 | `CAN_ID_CMD_MODE` | ≥1 | [0] bit0=4x4, bit1=tank_turn | `can_handler.c`, lines 285–295 |

**RX Filters:** Configured in `CAN_ConfigureFilters()` (`can_handler.c`, lines 75–102):
- Filter 0: Accept 0x011 (ESP32 heartbeat)
- Filter 1: Range 0x100–0x102 (ESP32 commands)
- Global: Reject all non-matching standard and extended IDs.

### What Happens If CAN Is Silent

- 250 ms timeout → `SAFETY_ERROR_CAN_TIMEOUT` → SAFE state → `Traction_EmergencyStop()` + steer to center (`safety_system.c`, lines 324–326, 425–439).
- If CAN resumes, automatic recovery: SAFE→ACTIVE (`safety_system.c`, lines 334–338).

### Whether CAN Is REQUIRED for Movement

**YES.** The system cannot reach ACTIVE state without ESP32 CAN heartbeat. All actuator commands are gated by `Safety_IsCommandAllowed()` which requires ACTIVE state. Local pedal input is also gated (`main.c`, lines 106–111).

---

## SECTION 8 — HARDWARE VS FIRMWARE MATCH

| Dashboard Element | Firmware Support | Details |
|-------------------|-----------------|---------|
| **Gear selector (P/D2/D1/N/R)** | **NOT IMPLEMENTED** on STM32 | Shifter is read by ESP32 via MCP23017 GPIOB0–B4 (base: `src/input/shifter.cpp`). No CAN command for gear selection exists. No reverse direction path in CAN protocol. |
| **2WD / 4WD switch** | **IMPLEMENTED** | Via `CAN_ID_CMD_MODE` (0x102) bit 0. `can_handler.c`, line 288 → `Traction_SetMode4x4()`. Mode change validated at low speed (`Safety_ValidateModeChange()`, `safety_system.c`, lines 181–192, ≤ 1 km/h). |
| **Ignition key** | **NOT IMPLEMENTED** on STM32 | In base firmware: `PIN_KEY_ON = GPIO 40`, `PIN_KEY_OFF = GPIO 41` on ESP32. STM32 has no ignition key GPIO or detection logic. |
| **Pedal** | **IMPLEMENTED** | ADC1 on PA3 (`sensor_manager.c`, `Pedal_Update()`). EMA filter + ramp limiter in `motor_control.c`, `Traction_SetDemand()`. |
| **Steering wheel** | **IMPLEMENTED** | Encoder on TIM2 (PA15/PB3), motor on TIM8_CH3 (PC8). PID control in `motor_control.c`, `Steering_ControlLoop()`. Ackermann geometry in `ackermann.c`. |

### Elements Visible in Hardware with NO STM32 Firmware Support

1. **Gear selector positions (P/R/N/D1/D2)**: Physical shifter exists (5-position, connected via MCP23017 on ESP32). STM32 has no gear awareness — it receives only throttle percentage. **Reverse gear cannot be activated** through the CAN protocol.
2. **Ignition key/switch**: Physical key switch exists (connected to ESP32 GPIO 40/41). STM32 relies entirely on CAN heartbeat for activation.
3. **Obstacle sensors (TOFSense-M S LiDAR)**: Physical sensor connected to ESP32 UART0. STM32 has no obstacle awareness.
4. **Lighting (WS2812B LED strips)**: Front (28 LEDs) and rear (16 LEDs) on ESP32 GPIOs. STM32 has no lighting control.
5. **Audio system (DFPlayer Mini)**: Connected to ESP32 UART1. STM32 has no audio support.
6. **TFT Display + Touch**: Connected to ESP32 SPI. STM32 has no display interface.
7. **SPARE/Media relay**: Base firmware has `PIN_RELAY_SPARE` (GPIO 46/18 on ESP32). STM32 only controls 3 relays (MAIN, TRAC, DIR).
8. **Lights button (BTN_LIGHTS)**: Physical button on ESP32 GPIO 0. STM32 has no button input.

---

## SECTION 9 — FINAL VERDICT

### Is the STM32 firmware internally coherent?

**YES.** Within its defined scope as a motor control and safety authority module, the firmware is internally coherent:
- All declared peripherals are initialized and used.
- The state machine transitions are logically consistent.
- Safety checks (overcurrent, overtemp, CAN timeout, encoder fault, sensor plausibility) all route through the same state machine.
- All CAN TX messages are populated with live sensor data.
- All CAN RX commands are validated through the safety gate before reaching actuators.
- The watchdog (IWDG, ~500 ms) is refreshed in the main loop.
- Fault handlers (HardFault, MemManage, BusFault, UsageFault) all safe the hardware by driving GPIOC outputs LOW.

### Is anything missing that existed in the base firmware?

| Missing Feature | Base Firmware Location | Impact |
|----------------|----------------------|--------|
| Gear logic (P/N/D1/D2/R) | `src/input/shifter.cpp` | **Critical**: No reverse command path in CAN protocol. Vehicle cannot reverse via STM32. |
| Speed limits per gear (D1/D2) | `src/control/traction.cpp` | D1/D2 differentiation not implemented; throttle is applied uniformly. |
| Reverse lockout at speed | `src/input/shifter.cpp`, line 128 | On ESP32 side only; STM32 has no reverse concept. |
| Ignition key detection | `include/pins.h`: GPIO 40/41 | STM32 relies on ESP32 CAN for activation. |
| Obstacle detection & safety | `src/sensors/obstacle_detection.cpp`, `src/safety/obstacle_safety.cpp` | NOT IMPLEMENTED on STM32. Entirely on ESP32. |
| Adaptive cruise control | `src/control/adaptive_cruise.cpp` | NOT IMPLEMENTED on STM32. |
| Limp mode | `src/system/limp_mode.cpp` | NOT IMPLEMENTED on STM32. |
| Non-blocking relay sequencing | `src/control/relays.cpp` (state machine) | STM32 uses blocking `HAL_Delay()`. |
| SPARE/Media relay | `include/pins.h`: `PIN_RELAY_SPARE` | NOT IMPLEMENTED. STM32 has 3 relays, not 4. |
| Regenerative braking AI | `src/safety/regen_ai.cpp` | NOT IMPLEMENTED on STM32. |
| Ackermann traction differentiation | `src/control/traction.cpp` (speed factor per wheel in curves) | Base firmware adjusts per-wheel PWM based on steering angle; STM32 applies uniform PWM to all driven wheels. |
| Audio / Lighting / HMI | Various ESP32 modules | Expected to remain on ESP32 in split architecture. |

### Is anything present that should NOT be there?

**NO.** All implemented features in the STM32 firmware correspond to functions that existed in the base firmware. No extraneous or contradictory functionality was identified. The `Ackermann_Compute()` function in `motor_control.c` (lines 505–530) duplicates logic in `ackermann.c` but does not create incorrect behavior — it is simply unused.

### Is the system SAFE as-is?

**CONDITIONAL YES**, with the following caveats:

1. **Safe as a motor controller**: The STM32 enforces CAN heartbeat requirements, validates all commands, implements ABS/TCS, monitors current/temperature/encoder health, and has a robust fail-safe chain (SAFE state → emergency stop → relay power-down). The watchdog prevents firmware hangs. Fault handlers safe the hardware on hard faults.

2. **Reverse gap is a safety concern**: The absence of a reverse direction CAN command means the vehicle cannot reverse through normal operation. If the ESP32 were to somehow inject a negative throttle (which the current CAN protocol cannot encode as `uint8_t`), the STM32 would accept it through `Traction_SetDemand()`. This is an **architectural gap**, not a safety vulnerability, because no path exists to trigger unintended reverse motion.

3. **Blocking relay sequence**: The `Relay_PowerUp()` function blocks the main loop for ~70 ms via `HAL_Delay()`. During this time, the watchdog is not refreshed (IWDG timeout is ~500 ms, so this is within margin). Safety checks are also paused. This is a **minor concern** for production use.

4. **Relative encoder calibration**: Steering calibration assumes the wheel is centered at boot. If it is not, all subsequent angle commands will be offset. This is a **known limitation** documented in the code.

5. **Single error code limitation**: `safety_error` stores only one error at a time (`Safety_Error_t` is a scalar, not a bitmask). If multiple faults occur simultaneously, only the last one written is retained. The fault flags bitmask in the heartbeat (`Safety_GetFaultFlags()`, `safety_system.c`, lines 102–112) partially compensates but maps multiple error types to the same bits.

---

*End of report. All claims reference specific files, functions, and line ranges from the target repository (STM32-Control-Coche-Marcos) and the authoritative base (FULL-FIRMWARE-Coche-Marcos).*
