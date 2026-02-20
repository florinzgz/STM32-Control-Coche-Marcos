# Fail-Operational Architecture — Migration & Validation Audit

**Document type:** Engineering Audit  
**Revision:** 1.0  
**Date:** 2026-02-20  
**Scope:** Zero-regression migration plan for evolving STM32 firmware toward full fail-operational architecture  
**Original firmware:** [`florinzgz/FULL-FIRMWARE-Coche-Marcos`](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos) v2.17.1  
**Current firmware:** [`florinzgz/STM32-Control-Coche-Marcos`](https://github.com/florinzgz/STM32-Control-Coche-Marcos) (STM32G474RE)  
**Source references:** All behavioral claims in this document are derived exclusively from code in `Core/Src/` and `Core/Inc/`

---

## PHASE 1 — Firmware Understanding

### 1.1 Current STM32 Firmware — Complete Behavioral Description

#### 1.1.1 Boot Sequence

1. `Boot_ReadResetCause()` reads `RCC->CSR` and classifies reset origin: POWERON, SOFTWARE, IWDG, WWDG, BROWNOUT, PIN. Flags are cleared immediately.
2. `MX_FDCAN1_Init()` and `MX_I2C1_Init()` are non-fatal: if they fail, `fdcan_init_ok`/`i2c_init_ok` are set `false` and execution continues. No `Error_Handler()` called.
3. `IWDG` is initialised at ~500 ms timeout (PRESCALER_32, RELOAD=4095, 32 kHz LSI).
4. `Safety_Init()` sets `system_state = SYS_STATE_BOOT`, all wheel_scale = 1.0, obstacle_scale = 1.0, last_can_rx_time = HAL_GetTick().
5. `SteeringCentering_Init()` starts centering in CENTERING_IDLE; actual sweep begins on the first `SteeringCentering_Step()` call.
6. `Safety_SetState(SYS_STATE_STANDBY)` is called after all module init: `BOOT → STANDBY` transition is unconditional.
7. During STANDBY, every 10 ms loop iteration calls:
   - `BootValidation_Run()` — evaluates 6 checks: temperature plausible, current plausible, encoder healthy, battery ≥ 20.0 V, no safety error (except CAN), CAN-busoff (always passes).
   - `SteeringCentering_Step()` — runs automatic centering sweep.
   - `Safety_CheckCANTimeout()` — if ESP32 heartbeat has been received and all 3 conditions are met (CAN alive, `safety_error == NONE`, `Steering_IsCalibrated()`, `BootValidation_IsPassed()`), transitions `STANDBY → ACTIVE`.
   - If `STANDBY` and CAN timeout > 250 ms AND steering calibrated AND boot validated → `STANDBY → LIMP_HOME`.

#### 1.1.2 Safety State Machine

States (from `safety_system.h`):

| State | Value | Motion allowed | CAN commands | Description |
|-------|-------|---------------|--------------|-------------|
| BOOT | 0 | No | No | HAL init in progress |
| STANDBY | 1 | No | No | Waiting for centering + boot validation + ESP32 |
| ACTIVE | 2 | Yes | Yes (full) | Normal operation |
| DEGRADED | 3 | Yes | Yes (power limited) | Non-critical fault, limp capability |
| SAFE | 4 | No | No | Hardware danger; relays off, traction stopped |
| ERROR | 5 | No | No | Unrecoverable; permanent shutdown |
| LIMP_HOME | 6 | Yes (local pedal only) | No | CAN-loss degraded; walking speed |

**Transition guards** (from `Safety_SetState()`):

- `BOOT → STANDBY`: unconditional (called once in `main.c` after all init).
- `STANDBY/SAFE/DEGRADED/LIMP_HOME → ACTIVE`: requires `safety_error == SAFETY_ERROR_NONE`.
- `ACTIVE/STANDBY → DEGRADED`: relays stay on, no `Safety_FailSafe()`.
- `ACTIVE/STANDBY/DEGRADED/LIMP_HOME → SAFE`: calls `Safety_FailSafe()` (stops traction, centers steering if encoder healthy, relay stays on — only `Relay_PowerDown` on ERROR/EmergencyStop).
- `any → ERROR`: calls `Safety_PowerDown()` which calls `Relay_PowerDown()`.
- `STANDBY/ACTIVE/DEGRADED → LIMP_HOME`: relays stay on, `Relay_PowerUp()` called.

**SAFE does NOT cut relays** — only `ERROR` and `EmergencyStop` do. In SAFE, the relays remain energised but traction is commanded to 0 via `Traction_EmergencyStop()`.

#### 1.1.3 Fault Triggers and Their State Transitions

| Fault | Trigger condition | Immediate state | Recovery |
|-------|-----------------|-----------------|---------|
| Overcurrent (single) | Any INA226 channel > 25.0 A, first event | DEGRADED (L1 or L3) | Automatic after 1 s clean current |
| Overcurrent (≥3 consecutive) | consecutive_errors ≥ 3 | SAFE | No auto-recovery; requires ESP32 heartbeat + no error |
| Overtemperature warning | Any DS18B20 > 80°C | DEGRADED (L2) | Auto after all < 75°C (5°C hysteresis) |
| Overtemperature critical | Any DS18B20 > 90°C | SAFE | No auto-recovery |
| Battery warning | Voltage < 20.0 V | DEGRADED (L2) | Auto after > 20.5 V |
| Battery critical | Voltage < 18.0 V | SAFE | No auto-recovery; operator must reset |
| Battery sensor fail | 0.0 V reading from INA226 | SAFE | No auto-recovery |
| CAN timeout | No heartbeat from ESP32 for > 250 ms | LIMP_HOME | Auto when heartbeat resumes |
| Sensor fault | Temperature/current/speed out of range | DEGRADED (L1 or L3) | Auto when sensors return to range |
| Pedal implausible | ADC vs ADS1115 diverge | Throttle → 0 + DEGRADED | Auto when channels agree |
| Encoder fault | Range exceeded / jump / frozen | DEGRADED (L1), steering neutralised | No auto-recovery on encoder |
| Steering centering fail | Timeout (10 s) or range (6000 counts) | DEGRADED (L1) | No auto-recovery |
| Emergency stop | External call | ERROR | Permanent; IWDG reset needed |
| CAN bus-off | `HAL_FDCAN_GetProtocolStatus()` bus-off bit | DEGRADED + recovery attempts (max 10) | Auto retry every 500 ms |
| Demand anomaly | Step > 15%/10ms or frozen pedal + speed delta | DEGRADED (L1) | Auto after demand normalises |
| Watchdog | IWDG fires (> 500 ms loop stall) | (hard reset, detected as IWDG reset cause) | Boot from reset |

#### 1.1.4 Traction Control Pipeline (per 10 ms cycle)

Order of operations in `Traction_Update()`:

1. Read `demandPct` from `traction_state` (set by `Traction_SetDemand()`).
2. Apply per-motor overtemperature cutoff (130°C threshold, 115°C recovery).
3. Apply `Safety_GetTractionCapFactor()` (1.0 in ACTIVE; 0.7–0.4 in DEGRADED levels; 0.2 in LIMP_HOME).
4. Compute Ackermann differential torque (±15% max correction based on steering angle).
5. Apply `safety_status.obstacle_scale` (0.0–1.0 from `Obstacle_Update()`).
6. Apply `safety_status.wheel_scale[i]` (ABS/TCS per-wheel, min of ABS and TCS).
7. Gear power scaling (D1 = 60%, D2 = 100%, R = 60%).
8. Smooth-driving state machine (BRAKE/COAST/DRIVE phases, deadzone compensation, jerk limiter, dynamic braking).
9. Write PWM to TIM1 CH1-4 (center-aligned, 20 kHz, period = 4249 counts).

#### 1.1.5 ABS

- Active when vehicle speed ≥ 10 km/h.
- Per-wheel slip = `(avg_speed - wheel_speed) / avg_speed × 100`.
- Trigger: slip > 15%.
- Pulse modulation: 80 ms period, 60% ON. ON phase: wheel_scale = 0.70. OFF phase: wheel_scale = 1.0.
- All-4-wheels-locked fallback: `Traction_SetDemand(0)`.
- Traced to `abs_system.cpp`: `slipThreshold=15`, `pressureReduction=0.30`, same pulse parameters.

#### 1.1.6 TCS

- Active when vehicle speed ≥ 3 km/h.
- Per-wheel slip = `(wheel_speed - avg_speed) / avg_speed × 100`.
- Trigger: slip > 15%.
- Initial cut: 40%. Progressive: +5% per cycle. Max: 80%.
- Recovery: 25%/s when slip clears.
- ABS and TCS interact: `wheel_scale[i] = min(ABS_scale, TCS_scale)`.
- Traced to `tcs_system.cpp`: `aggressiveReduction=40`, `smoothReduction=5`, `recoveryRatePerSec=25`.

#### 1.1.7 Steering Control

- PID controller on encoder counts. Hardware quadrature (TIM2, 32-bit, 4800 CPR).
- P-only controller: kp = 0.09 (encoder-count space), ki = 0, kd = 0.
- Rate limit: 200°/s maximum steering rate.
- Deadband: 0.5° (≈ 6.67 counts).
- EPS torque-assist: angular velocity computed from angle derivative, EMA-filtered. High-speed fade: 100% at 20 km/h → 50% at 30 km/h.
- Steering commands rejected if system not ACTIVE or DEGRADED.
- Steering calibrated flag gated: no PID until centering complete.

#### 1.1.8 Obstacle Safety Module

- CAN 0x208 messages from ESP32 are **advisory only**.
- STM32 runs a local state machine: NO_SENSOR → NORMAL → CONFIRMING → ACTIVE → CLEARING → SENSOR_FAULT.
- Plausibility: max approach rate = 8 m/s. Stuck sensor detection: if vehicle > 1 km/h and distance unchanged for 1 s → SENSOR_FAULT.
- Speed-dependent stopping distance: d = v²/(2×3m/s²) + 200mm margin.
- Distance tiers: < 200mm → scale=0.0; 200–500mm → scale=0.3; 500–1000mm → scale=0.7; > 1000mm → scale=1.0.
- Temporal confirmation: 200 ms to confirm obstacle, 1000 ms to confirm clearance.
- CAN timeout (500 ms): if obstacle was active, hold last scale; if no obstacle, scale → 1.0 (LIMP_HOME speed cap provides safety net).
- Reverse escape: `Obstacle_IsForwardBlocked()` — when forward blocked, reverse direction still has full scale.

#### 1.1.9 CAN Protocol

| ID | Direction | Rate | Payload |
|----|-----------|------|---------|
| 0x001 | STM32→ESP32 | 100ms | alive_counter, system_state, fault_flags, error_code |
| 0x011 | ESP32→STM32 | 100ms | alive_counter |
| 0x100 | ESP32→STM32 | 50ms | throttle_pct (uint8) |
| 0x101 | ESP32→STM32 | 50ms | angle (int16 LE), reserved |
| 0x102 | ESP32→STM32 | on-demand | mode_flags, gear (0-4), reserved×6 |
| 0x103 | STM32→ESP32 | on-demand | cmd_id_low, ack_result, system_state |
| 0x110 | ESP32→STM32 | on-demand | module_id, action (enable/disable/reset) |
| 0x200 | STM32→ESP32 | 100ms | FL/FR/RL/RR speed (uint16 LE, 0.1 km/h) |
| 0x201 | STM32→ESP32 | 100ms | FL/FR/RL/RR current (uint16 LE, 0.01A) |
| 0x202 | STM32→ESP32 | 1000ms | 5× temperature (int8, °C) |
| 0x203 | STM32→ESP32 | 100ms | abs_active, tcs_active, error_code |
| 0x204 | STM32→ESP32 | 100ms | angle (int16 LE, 0.1°), calibrated |
| 0x205 | STM32→ESP32 | 100ms | FL/FR/RL/RR traction scale (uint8, 0–100%) |
| 0x206 | STM32→ESP32 | 1000ms | FL/FR/RL/RR/AMB temperature (int8, °C) |
| 0x207 | STM32→ESP32 | 100ms | battery current + voltage (uint16 LE, 0.01A/V) |
| 0x208 | ESP32→STM32 | 66ms | distance_mm (uint16 LE), zone, sensor_healthy, counter |
| 0x209 | ESP32→STM32 | 100ms | (filter configured, decode body empty) |
| 0x300 | Both | on-demand | error_code, subsystem (also used for encoder diagnostic) |
| 0x301–0x303 | STM32→ESP32 | 1000ms | service fault/enabled/disabled bitmasks (uint32) |

**CAN timeout**: `last_can_rx_time` is updated in the FDCAN RxFifo0 ISR callback via `Safety_UpdateCANRxTime()`. Timeout = 250 ms.

**Bus-off recovery**: Non-blocking, 500 ms retry interval, max 10 retries. If retries exhausted → DEGRADED (not ERROR, which would cut relays).

#### 1.1.10 Boot Validation

6 checks run every 10 ms during STANDBY:

1. `BOOT_CHECK_TEMP_PLAUSIBLE`: All enabled DS18B20 sensors in [-40, +125]°C. All-zeros reading → fail.
2. `BOOT_CHECK_CURRENT_PLAUSIBLE`: All enabled INA226 channels in [-1, +50] A.
3. `BOOT_CHECK_ENCODER_HEALTHY`: `!Encoder_HasFault()`.
4. `BOOT_CHECK_BATTERY_OK`: Battery INA226 ≥ 20.0 V.
5. `BOOT_CHECK_NO_SAFETY_ERROR`: No error, OR error is SAFETY_ERROR_CAN_TIMEOUT or SAFETY_ERROR_CAN_BUSOFF (CAN absence is NOT a boot blocker).
6. `BOOT_CHECK_CAN_NOT_BUSOFF`: Always returns `true` (CAN bus-off no longer blocks boot).

Validation passes when ALL 6 checks pass simultaneously. If validation fails, the system waits in STANDBY (does not SAFE out).

---

### 1.2 Original Firmware (FULL-FIRMWARE-Coche-Marcos) — Extracted Behavioral Rules

*Source: `docs/ORIGINAL_REPO_COMPARATIVE_AUDIT.md` (present in repository), original repository README, and code structure analysis.*

#### 1.2.1 Architecture

- Monolithic ESP32-S3 (FreeRTOS, 5 tasks, 2 cores).
- Motor control via PCA9685 I2C PWM expander (not direct timer PWM).
- Gear shifting via MCP23017 I2C GPIO expander.
- Relays via MCP23017 (4 relays: main, traction, direction, spare).
- Steering encoder read by GPIO interrupt on ESP32 (not hardware quadrature).
- No dedicated safety MCU; safety logic mixed into Manager classes.

#### 1.2.2 Safety State Machine (Original)

- Distributed across `SafetyManager.cpp`, `limp_mode.cpp`, `relays.cpp`.
- States: NORMAL, DEGRADED (two levels), LIMP, EMERGENCY.
- No formal LIMP_HOME for CAN loss (communication was internal, not CAN).
- ABS slip threshold: 15.0%.
- TCS slip threshold: 15.0%.
- Consecutive errors threshold: 3 (`relays.cpp: consecutiveErrors >= 3`).
- Temperature warning: 80°C → DEGRADED. Critical: 90°C → EMERGENCY.
- Overcurrent: 25 A → fault escalation.
- Watchdog: ESP32 Task WDT — software-managed, resets on task stall.

#### 1.2.3 Degraded Mode (Original — `limp_mode.cpp`)

- Two-level model:
  - DEGRADED: 70% power, 80% speed.
  - LIMP: 40% power, 50% speed.
- STATE_HYSTERESIS_MS = 500 ms before recovery from LIMP → NORMAL.
- Steering not centered → LimpState::LIMP.
- Recovery: only if fault clears AND operator confirms via HMI.

#### 1.2.4 Boot Sequence (Original — `boot_guard.cpp`, `system.cpp`)

- NVS counter tracks boot attempts. After 3 bootloops, enters "safe boot" mode (limited features).
- `handleCriticalError()`: 3 retries → `ESP.restart()`. If hardware fault persists → permanent bootloop.
- Manager init chain: if any Manager fails → `handleCriticalError()` → restart.
- No power-on sensor validation gate before enabling traction.

#### 1.2.5 ABS (Original — `abs_system.cpp`)

- Per-wheel slip detection, threshold = 15%.
- `pressureReduction = 0.30` → scale = 0.70 during active phase.
- Pulse modulation implied by the reference comments in current firmware.
- No minimum speed threshold documented in audit.

#### 1.2.6 TCS (Original — `tcs_system.cpp`)

- `aggressiveReduction = 40.0f`, `smoothReduction = 5.0f`, `recoveryRatePerSec = 25.0f`, max reduction = 80%.
- Minimum speed: 3.0 km/h.

#### 1.2.7 Traction (Original — `traction.cpp`, `regen_ai.cpp`)

- EMA filter coefficient: 0.15.
- Maximum acceleration rate: 50%/s.
- Dynamic braking: proportional to throttle release rate; AI-modulated regen (`regen_ai.cpp`).
- No gear-based power scaling documented.
- No demand anomaly step-rate validation.
- No NaN/Inf float sanitization in traction pipeline.

#### 1.2.8 Steering (Original — `steering_motor.cpp`, `steering_model.cpp`)

- Control via PCA9685 (I2C, not direct PWM).
- kp = 1.2 (degree space). P-only.
- Encoder read by GPIO interrupt on ESP32.
- Deadband: 0.5°.
- No hardware quadrature — susceptible to missed pulses under EMI.
- No EPS torque-assist (assist came from PCA9685 output only).
- No high-speed steering fade.

#### 1.2.9 Obstacle Detection (Original — `obstacle_detection.cpp`, `obstacle_safety.cpp`)

- Ultrasonic sensor on ESP32 (direct GPIO, same MCU as motors).
- Three-tier distance check: CLEAR, WARNING, EMERGENCY.
- No stuck-sensor detection documented.
- No speed-dependent stopping distance.
- No temporal hysteresis documented.
- Sensor fault → conservative mode.

#### 1.2.10 Sensor Management (Original)

- INA226: direct I2C from ESP32 (no multiplexer documented).
- DS18B20: OneWire from ESP32.
- Pedal: single ADC channel (no dual-channel plausibility).
- Wheel speed: GPIO interrupts on ESP32.
- No ROM search / CRC validation documented for DS18B20.

---

## PHASE 2 — Behavioral Comparison

### Legend: MATCH / IMPROVED / MISSING / RISKY

**MATCH**: Current behavior is functionally equivalent to original.  
**IMPROVED**: Current behavior is better than original (more robust, more precise, or adds protection).  
**MISSING**: Original had this behavior; current does not.  
**RISKY**: Behavioral difference that could affect vehicle safety or limp capability.

---

### 2.1 Traction Subsystem

| Aspect | ORIGINAL behavior | CURRENT behavior | Status |
|--------|------------------|-----------------|--------|
| PWM generation | PCA9685 via I2C (0–4095, 12-bit, latency ~1 ms) | TIM1 CH1-4 direct, center-aligned, 20 kHz, 4249 steps | IMPROVED |
| Gear system | MCP23017 physical shifter, no enforcement | P/R/N/D1/D2 enforced: speed gate for mode change, park hold active brake | IMPROVED |
| D1 vs D2 power | Not documented | D1 = 60%, D2 = 100% power cap | NEW — see risk R4 |
| Pedal EMA filter | Alpha = 0.15, 20 Hz update rate | Alpha = 0.15, 20 Hz update rate | MATCH |
| Ramp rate | 50%/s up | 50%/s up, 100%/s down | IMPROVED |
| Demand anomaly detection | Not present | Step > 15%/10ms or frozen pedal → DEGRADED | IMPROVED |
| NaN/Inf sanitization | Not present | `sanitize_float()` on all PWM paths | IMPROVED |
| Dynamic braking | AI regen (`regen_ai.cpp`) | Proportional dynamic braking, no AI | MISSING — see risk R5 |
| Deadzone/creep | Not documented | 8% motor deadzone, 3%/1% hysteresis | NEW |
| 4×4/4×2 mode | ESP32-side, no speed gate | STM32-side, requires speed < 1 km/h | IMPROVED |
| Tank turn mode | ESP32-side | STM32-side | IMPROVED |
| Adaptive cruise control | `adaptive_cruise.cpp` | Not implemented | MISSING |

### 2.2 Steering Subsystem

| Aspect | ORIGINAL behavior | CURRENT behavior | Status |
|--------|------------------|-----------------|--------|
| Motor interface | PCA9685 I2C | TIM8 CH3 direct PWM, 20 kHz | IMPROVED |
| Encoder | GPIO interrupt on ESP32, 1200 PPR | TIM2 hardware quadrature, 32-bit, 4800 CPR | IMPROVED |
| PID gains | kp = 1.2 (degree space), P-only | kp = 0.09 (count space ≡ same), ki=0, kd=0, P-only | MATCH |
| Deadband | 0.5° | 0.5° (≈6.67 counts) | MATCH |
| Centering | Manual encoder calibration menu | Automatic sweep + inductive sensor, fault-tolerant | IMPROVED |
| Rate limiting | Not documented | 200°/s max | IMPROVED |
| EPS torque assist | Not present | Angular velocity feedback, high-speed fade (20→30 km/h) | NEW |
| Steering fault handling | Not documented | Encoder fault → neutralise + DEGRADED | IMPROVED |
| Steering in SAFE | Not documented | Center command + neutralise if encoder faulted | IMPROVED |

### 2.3 Safety State Machine

| Aspect | ORIGINAL behavior | CURRENT behavior | Status |
|--------|------------------|-----------------|--------|
| CAN loss behavior | Not applicable (monolithic) | CAN loss → LIMP_HOME (relays on, local pedal) | NEW / IMPROVED |
| Communication loss = SAFE | N/A | Explicitly prevented: CAN timeout → LIMP_HOME, not SAFE | IMPROVED |
| Consecutive error escalation | After 3 errors → EMERGENCY | After 3 errors → SAFE | MATCH |
| DEGRADED levels | 2 levels (DEGRADED / LIMP) | 3 levels (L1/L2/L3) with per-level power/steering/traction | IMPROVED |
| DEGRADED power limits | 70% / 40% | L1=70%, L2=50%, L3=40% | MATCH (worst case preserved) |
| Recovery debounce | STATE_HYSTERESIS_MS = 500 ms | RECOVERY_HOLD_MS = 500 ms | MATCH |
| Relay sequencing | MCP23017 relay control | GPIO direct: Main→50ms→Traction→20ms→Direction (non-blocking) | IMPROVED |
| SAFE triggers relay cutoff | Not documented | SAFE does NOT cut relays. Only ERROR and EmergencyStop call Relay_PowerDown() | RISKY — see risk R1 |
| Watchdog | Software Task WDT | IWDG hardware, 500 ms, LSI-independent | IMPROVED |
| Emergency stop → ERROR | → ESP.restart() | → SYS_STATE_ERROR (permanent, IWDG reset needed) | MATCH |

### 2.4 Boot Subsystem

| Aspect | ORIGINAL behavior | CURRENT behavior | Status |
|--------|------------------|-----------------|--------|
| Boot guard | NVS bootloop counter, safe boot after 3 | Reset cause classification (IWDG/BROWNOUT/etc.) | DIFFERENT |
| Pre-drive validation | No sensor gate | 6-check gate: temp, current, encoder, battery, no-error, CAN | IMPROVED |
| Centering gate | No gate — motors available immediately | No motion until centering complete + boot validated | IMPROVED |
| Init failure handling | Any manager fail → restart (bootloop) | FDCAN/I2C fail → non-fatal, system continues | IMPROVED |
| NVS persistence | Bootloop count, config persist | Not present | MISSING — see risk R2 |
| Centering required for ACTIVE | No | Yes — STANDBY until `Steering_IsCalibrated()` | RISKY if sensor fails — see risk R3 |

### 2.5 Sensors Subsystem

| Aspect | ORIGINAL behavior | CURRENT behavior | Status |
|--------|------------------|-----------------|--------|
| INA226 topology | Direct I2C from ESP32 | TCA9548A 8-ch mux, 6 INA226 channels (4 motors + battery + spare) | IMPROVED |
| DS18B20 topology | OneWire from ESP32 | OneWire bit-bang from STM32 PB0, ROM search, CRC-8 | IMPROVED |
| Wheel speed sensors | GPIO interrupt on ESP32 | EXTI on STM32 with 1 ms software debounce | IMPROVED |
| Pedal dual-channel | Single ADC | ADC1 primary + ADS1115 I2C plausibility, cross-validated | IMPROVED |
| Pedal failure action | Not documented | Force throttle to 0 + DEGRADED | IMPROVED |
| DS18B20 hot-plug | Not documented | ROM search only at init (no hot-plug support) | MISSING (non-critical) |
| Current sensor plausibility | Not documented | < -1 A or > 50 A → fault + DEGRADED | IMPROVED |
| Speed plausibility | Not documented | > 25 km/h → fault + DEGRADED | IMPROVED |

### 2.6 Obstacle Detection

| Aspect | ORIGINAL behavior | CURRENT behavior | Status |
|--------|------------------|-----------------|--------|
| Sensor authority | ESP32 is sole authority (direct GPIO) | ESP32 sends CAN 0x208; STM32 is safety authority | IMPROVED |
| Stuck sensor detection | Not documented | Vehicle moves > 1 km/h, distance unchanged 1 s → SENSOR_FAULT | IMPROVED |
| Speed-dependent threshold | Not documented | Stopping distance = v²/(2×3m/s²) + 200 mm | IMPROVED |
| Temporal hysteresis | Not documented | 200 ms to confirm, 1000 ms to clear | IMPROVED |
| CAN timeout fallback | N/A (direct sensor) | Obstacle was active: hold last scale; no obstacle: scale=1.0 | IMPROVED |
| Reverse escape | Not documented | `Obstacle_IsForwardBlocked()` — only forward direction blocked | IMPROVED |
| ESP32 sensor driver | Local driver in ESP32 firmware | No driver in current ESP32 firmware (CAN parse ready, sensor not wired) | MISSING — see risk R6 |

### 2.7 CAN Communication

| Aspect | ORIGINAL behavior | CURRENT behavior | Status |
|--------|------------------|-----------------|--------|
| Protocol | None (monolithic) | 20+ CAN IDs, bidirectional heartbeat, hardware filters | NEW |
| Bus-off handling | None | Non-blocking, 10 retries at 500 ms → DEGRADED | NEW |
| Command ACK | None | 0x103 ACK with result code | NEW |
| CAN loss = immobilization | N/A | Explicitly prevented: LIMP_HOME keeps vehicle moving | IMPROVED |
| 0x209 decode body | N/A | Filter configured, decode body empty | MISSING (non-critical) |
| Command rate | N/A | Throttle 50 ms, Steering 50 ms, Mode on-demand | NEW |

### 2.8 Fault Handling

| Aspect | ORIGINAL behavior | CURRENT behavior | Status |
|--------|------------------|-----------------|--------|
| Overcurrent per motor | Not documented | Per-motor 130°C cutoff (independent from system SAFE) | IMPROVED |
| Per-motor emergency cutoff | Not present | Yes: wheel_scale[i] = 0 at 130°C, recovery at 115°C | IMPROVED |
| Fault logging | `logger.cpp` serial | Serial via `ServiceMode_SetFault()` per-module | DIFFERENT |
| Service mode | None | 24 modules, enable/disable, fault/warning/error classification | NEW |
| Config persistence | NVS via `config_storage.cpp` | RAM-only; lost on reset | MISSING — see risk R2 |
| Calibration persistence | `eeprom_persistence.cpp` | Not present; re-centering required every boot | MISSING — see risk R2 |

---

## PHASE 3 — Risk Analysis

Each risk is identified with code-level justification.

---

### R1 — SAFE State Does Not Cut Relays

**Risk category:** False SAFE state / removed protection  
**Severity:** HIGH

**Code facts:**
- `Safety_FailSafe()` calls `Traction_EmergencyStop()` (sets all PWM to 0, disables enable pins) and commands steering to center. It does NOT call `Relay_PowerDown()`.
- `Relay_PowerDown()` is called only from `Safety_EmergencyStop()` and `Safety_PowerDown()` (which is called only from `ERROR` transition).
- In `Safety_SetState(SYS_STATE_SAFE)`: `Safety_FailSafe()` is called, but relays remain energised.

**Original behavior:** Not clearly documented, but the implied safe state in a safety-critical system is power removal.

**Risk:** If the traction driver (BTS7960) develops a fault where EN=LOW still allows current to flow (e.g., a latch-up condition), relays remaining on means motor current could persist even in SAFE state. The current firmware relies entirely on the H-bridge EN pin to cut torque in SAFE.

**Why it exists:** SAFE is designed to allow recovery (SAFE → ACTIVE when fault clears + heartbeat). If relays are cut, recovery requires power cycling. The design trade-off is intentional for the "drive home" philosophy, but creates the risk above.

**Mitigation in current code:** `Traction_EmergencyStop()` sets all EN pins LOW and PWM to 0. This is a GPIO-level action, independent of I2C.

**Recommendation:** Document this as a known architectural decision. If relay cutoff in SAFE is required, add it to the migration plan.

---

### R2 — No Non-Volatile Calibration Storage

**Risk category:** Limp-home capability degradation  
**Severity:** MEDIUM

**Code facts:**
- `steering_calibrated` flag is a RAM variable in `motor_control.c`.
- `boot_status.validated` is a RAM variable in `boot_validation.c`.
- Service mode enable/disable bitmask is RAM-only (`service_mode.c`).
- There is no flash write or NVS operation anywhere in `Core/Src/`.

**Original behavior:** `eeprom_persistence.cpp` and `config_storage.cpp` persisted calibration and config to NVS. `boot_guard.cpp` persisted bootloop count.

**Risk:** Every power cycle requires steering centering to complete before ACTIVE is reachable. If the inductive center sensor fails or the rack is mechanically stuck during centering, the system enters DEGRADED (centering fault) and never reaches ACTIVE. With persistent calibration, the last-known center position could be used as a fallback.

**Why it exists:** STM32G474RE has internal flash (512 KB). Flash write is not trivially available through HAL without careful sector management to avoid wear and accidental code erasure.

**Impact on limp-home:** Does not affect LIMP_HOME (which bypasses centering requirement). Only affects STANDBY → ACTIVE transition.

---

### R3 — Centering Required for ACTIVE; Not Required for LIMP_HOME

**Risk category:** Risky behavioral difference  
**Severity:** MEDIUM

**Code facts:**
In `Safety_CheckCANTimeout()`:
```c
if (system_state == SYS_STATE_STANDBY &&
    Steering_IsCalibrated() &&
    BootValidation_IsPassed()) {
    Safety_SetState(SYS_STATE_ACTIVE);
}
```
But for LIMP_HOME:
```c
if (system_state == SYS_STATE_STANDBY &&
    Steering_IsCalibrated() &&  // ← ALSO gated
    BootValidation_IsPassed()) {
    Safety_SetState(SYS_STATE_LIMP_HOME);
}
```

**Risk:** If the inductive center sensor fails (broken cable, misaligned screw), centering never completes, `Steering_IsCalibrated()` stays false, and the vehicle can never transition out of STANDBY — even to LIMP_HOME. The vehicle is immobilized even though the mechanical hardware is functional.

**Original behavior:** The original had a fallback calibration menu. The current design has no fallback if the inductive sensor fails.

**Impact on fail-operational:** Centering fault enters DEGRADED (L1), but STANDBY never enters DEGRADED — it either goes ACTIVE/LIMP_HOME or stays in STANDBY forever.

**Note:** The centering fault enters DEGRADED via `Centering_Abort()`, but the STANDBY→LIMP_HOME path requires `Steering_IsCalibrated()` which is only set by `Centering_Complete()`. DEGRADED and STANDBY are separate states — the system is currently in STANDBY and would need a manual transition out.

---

### R4 — D1 = 60% Power Cap Not in Original

**Risk category:** Missing original behavior / behavioral difference  
**Severity:** LOW-MEDIUM

**Code facts:**
- `GEAR_POWER_FORWARD_PCT = 0.60f` applied in `Traction_Update()`.
- D2 = 100% (`GEAR_POWER_FORWARD_D2_PCT = 1.00f`).
- In D1, maximum effective power is `Safety_GetPowerLimitFactor() × 60%`.
- In ACTIVE with D1: effective max = 1.0 × 0.60 = 60%.
- In ACTIVE with D2: effective max = 1.0 × 1.00 = 100%.

**Risk:** This is NOT a regression from the original (the original had no documented gear power cap). However, the D2/D1 distinction is new behavior that the original did not have. A driver expecting full power in the default gear (D1) will experience unexpected power limitation.

**Mitigation:** Default gear at boot is D1. Driver must explicitly command D2 via ESP32 CAN 0x102 to get full power.

---

### R5 — Dynamic Braking AI Removed (regen_ai.cpp Not Ported)

**Risk category:** Missing original behavior  
**Severity:** LOW

**Code facts:**
- Original had `regen_ai.cpp` with AI-modulated regenerative braking.
- Current has proportional dynamic braking: `brake_pct = |throttle_rate| × DYNBRAKE_FACTOR`, clamped to 60%.
- Dynamic braking is active only above 3 km/h, disabled in SAFE/ERROR/emergency stop states, reduced in DEGRADED.

**Risk:** Current dynamic braking is less sophisticated than the original's AI regen. Stopping distances may differ slightly from the original. No energy recovery to battery (H-bridge dissipates as heat). The H-bridge BTS7960 active-brake approach is thermally aggressive under repeated hard stops.

**Mitigation:** Current implementation is deterministic and verifiable; AI regen adds complexity with potential corner-case failures.

---

### R6 — Obstacle Sensor ESP32 Driver Not Implemented

**Risk category:** Missing original behavior  
**Severity:** MEDIUM

**Code facts:**
- `CAN_ID_OBSTACLE_DISTANCE = 0x208` filter is configured in `CAN_ConfigureFilters()`.
- `Obstacle_ProcessCAN()` in `safety_system.c` accepts the decoded frame.
- ESP32 `can_rx.cpp` decodes 0x208 into `vehicle::ObstacleData`.
- There is no driver code in the ESP32 firmware that reads a physical ultrasonic or ToF sensor and populates `ObstacleData`.

**Risk:** The obstacle safety module on STM32 starts in `OBS_STATE_NO_SENSOR`. Without CAN 0x208 frames, the obstacle scale remains 1.0 (full power) indefinitely. If a physical obstacle is in front of the vehicle, the STM32 has no data to act on.

**Impact on fail-operational:** CAN loss → scale = 1.0. No sensor → scale = 1.0. Both are safe from a motion perspective (no false stops). However, the vehicle does NOT slow down when approaching an obstacle.

**Note:** The LIMP_HOME speed cap (5 km/h) provides an indirect safety net.

---

### R7 — ESP32 Availability Required for ACTIVE State

**Risk category:** ESP32 dependency  
**Severity:** LOW (by design, mitigated by LIMP_HOME)

**Code facts:**
- `STANDBY → ACTIVE` requires `last_can_rx_time` to be within 250 ms.
- If ESP32 is permanently offline, system stays in STANDBY or transitions to LIMP_HOME.
- LIMP_HOME provides local pedal traction at 5 km/h walking speed.

**Risk:** The vehicle requires ESP32 to enter full ACTIVE mode. Without ESP32, full power (up to D2 100%) is never available; LIMP_HOME caps at 20% torque (5 km/h).

**Architectural intent:** This is explicitly designed. The ESP32 is the "operator interface" and its heartbeat is the authorization signal for full operation.

**Risk that remains:** If ESP32 crashes but appears alive (e.g., sends heartbeats but no throttle commands), the system transitions to ACTIVE but traction demand stays at 0 (no CAN throttle commands). The vehicle would appear "dead" with all systems healthy.

---

### R8 — `consecutive_errors` Not Saved Across Resets

**Risk category:** Removed protection  
**Severity:** LOW

**Code facts:**
- `consecutive_errors` is a RAM variable reset to 0 in `Safety_Init()`.
- Pattern: overcurrent → DEGRADED → IWDG reset → boot → `consecutive_errors = 0` → overcurrent again → DEGRADED (not SAFE).

**Risk:** A persistent hardware overcurrent condition that causes repeated IWDG resets may never escalate to SAFE. The consecutive-error counter only accumulates within a single power cycle.

**Original behavior:** `relays.cpp` `consecutiveErrors` was also RAM-only in the original, so this is a MATCH for this specific risk.

---

### R9 — Pedal Dual-Channel Plausibility Requires ADS1115

**Risk category:** ESP32 dependency / sensor dependency  
**Severity:** LOW-MEDIUM

**Code facts:**
- `Pedal_IsPlausible()` cross-validates ADC1 primary vs ADS1115 I2C secondary channel.
- If `i2c_init_ok = false` (I2C peripheral failed at boot), ADS1115 is unreachable.
- The current implementation falls back to `Pedal_IsPlausible()` returning... (requires reading sensor_manager.c pedal section).
- If ADS1115 is unavailable, the plausibility check behavior depends on the implementation.

**Mitigation needed:** Verify that `Pedal_IsPlausible()` returns `true` when ADS1115 is not available (degraded-graceful behavior) rather than blocking motion entirely.

---

## PHASE 4 — Migration Plan

### Governing rules (from problem statement)

1. Each step is atomic — single-file change or single behavioral change.
2. After each step, the vehicle must be drivable.
3. No step requires simultaneous multi-module modification.
4. No regressions allowed.

---

### Step 1 — Verify and document relay-in-SAFE behavior

**What files are touched:** `docs/` (new documentation only). No code change.  
**What behavior changes:** None. This step is analysis and decision only.  
**Physical test:** Enter SAFE state (trigger overcurrent via 25 A sustained load). Verify relays remain energised but traction PWM = 0. Test with multimeter on relay coil terminal and oscilloscope on motor PWM pin.  
**Expected CAN:** 0x001 byte 1 = 0x04 (SAFE). 0x201 current = 0 A. 0x205 traction scale = 0x00.  
**Rollback:** N/A — documentation only.  
**Decision to make:** Decide whether SAFE must cut relays (power removal) or whether EN-pin inhibition is sufficient. Document the decision. Only then proceed to Step 2.

---

### Step 2 — Add STANDBY → LIMP_HOME path without centering requirement

**Context:** Risk R3. If the inductive center sensor fails, the vehicle is immobilized in STANDBY.  
**What files are touched:** `Core/Src/safety_system.c` — `Safety_CheckCANTimeout()` only.  
**What behavior changes:** When CAN timeout > 250 ms AND STANDBY AND boot validated (regardless of `Steering_IsCalibrated()`), transition to LIMP_HOME. Steering remains uncalibrated; `Steering_ControlLoop()` rejects setpoints until calibrated.  
**Physical test:**  
1. Disconnect inductive center sensor cable.  
2. Power on. Verify centering enters FAULT (CENTERING_FAULT state).  
3. Disconnect ESP32 CAN.  
4. Wait > 250 ms. Verify CAN 0x001 byte 1 = 0x06 (LIMP_HOME).  
5. Press pedal. Verify vehicle moves at walking speed (< 5 km/h per CAN 0x200).  
**Expected CAN:** 0x001 byte 1 = 0x06. 0x200 speed ≤ 5 km/h equivalent raw value.  
**Rollback:** Revert the one condition change in `Safety_CheckCANTimeout()`. Single `git revert` on that commit.

---

### Step 3 — Add STANDBY → ACTIVE path without centering requirement (service bypass)

**Context:** Risk R3 for normal operation (not just LIMP_HOME).  
**What files are touched:** `Core/Src/safety_system.c`, `Core/Inc/service_mode.h` (add `MODULE_STEER_CENTERING` service bypass flag).  
**What behavior changes:** If `MODULE_STEER_CENTERING` is disabled in service mode, the centering check is bypassed for the STANDBY → ACTIVE gate. Normal default behavior (centering required) is unchanged.  
**Physical test:**  
1. Send CAN 0x110 to disable `MODULE_STEER_CENTERING`.  
2. Disconnect center sensor.  
3. Verify system transitions STANDBY → ACTIVE despite centering fault.  
4. Verify steering PID is active (commands accepted, but position relative to unknown zero).  
5. Re-enable centering module. Verify centering gate is restored.  
**Expected CAN:** 0x001 byte 1 = 0x02 (ACTIVE). 0x204 calibrated byte = 0x00 (uncalibrated, but system active).  
**Rollback:** Revert `safety_system.c` change. Service mode module addition is additive-only (no change to existing module IDs).

---

### Step 4 — Pedal plausibility fail-operational migration

**Context:** Risk R9 — pedal sensor failure must not immobilize vehicle.  
**What files are touched:**  
- `Core/Src/sensor_manager.c` — add `pedal_channels_contradict` flag, `Pedal_IsContradictory()` getter.  
- `Core/Inc/sensor_manager.h` — declare `Pedal_IsContradictory()`.  
- `Core/Src/safety_system.c` — `Safety_CheckSensors()` pedal fault → LIMP_HOME (fail-operational).  
- `Core/Src/boot_validation.c` — allow `SAFETY_ERROR_SENSOR_FAULT` through boot gate.  
- `Core/Src/main.c` — LIMP_HOME: enforce zero demand for contradictory pedals.  

**What behavior changes:** Pedal plausibility failure no longer immobilizes the vehicle.  Two failure modes are distinguished:  
1. **Contradictory** (both ADC and ADS1115 read OK but diverge > 5 % for > 200 ms): demand forced to zero in ALL states — no uncontrolled acceleration possible.  
2. **Unavailable** (ADS1115 I2C lost, primary ADC still functional): LIMP_HOME allows limited throttle from primary ADC with 20 % torque cap, 5 km/h speed limit, and 10 %/s ramp rate.  

In both cases ACTIVE → LIMP_HOME (not SAFE), and recovery to ACTIVE requires full pedal plausibility restored (`Pedal_IsPlausible() == true`).  

#### Before / After State Transition Table

| Scenario | State before | BEFORE (old behavior) | AFTER (new behavior) |
|----------|-------------|----------------------|---------------------|
| ADS1115 I2C fails, stale > 500 ms | ACTIVE | DEGRADED + demand=0 every 10 ms (immobilized) | LIMP_HOME — primary ADC at 20 % torque cap |
| ADS1115 I2C fails, stale > 500 ms | STANDBY | DEGRADED + demand=0 (cannot reach ACTIVE) | LIMP_HOME if boot validation passed |
| ADC vs ADS1115 diverge > 5 % for > 200 ms | ACTIVE | DEGRADED + demand=0 every 10 ms | LIMP_HOME + demand=0 (contradictory → zero torque) |
| ADC vs ADS1115 diverge > 5 % for > 200 ms | LIMP_HOME | demand=0 every 10 ms (immobilized) | demand=0 (contradictory → zero torque, same effect but explicit) |
| ADS1115 unavailable | LIMP_HOME | demand=0 every 10 ms (immobilized) | Primary ADC accepted with 20 % torque clamp |
| Pedal plausibility restored | LIMP_HOME | Stayed in DEGRADED, demand=0 cleared | SENSOR_FAULT cleared → ACTIVE via CAN timeout recovery |
| Pedal plausibility restored | DEGRADED | SENSOR_FAULT cleared → ACTIVE | Unchanged (non-pedal sensor faults still → DEGRADED) |
| Boot validation with SENSOR_FAULT | STANDBY | Boot validation fails (blocks LIMP_HOME) | Boot validation passes (SENSOR_FAULT allowed) |

#### Unintended Torque Impossibility Analysis

1. **Contradictory channels (both active, values disagree):** `Pedal_IsContradictory()` returns `true` → `Traction_SetDemand(0.0f)` in EVERY code path:  
   - `Safety_CheckSensors()` (10 ms): forces demand=0.  
   - `main.c` LIMP_HOME (50 ms): checks `Pedal_IsContradictory()` → demand=0.  
   Result: demand is forced to zero at both 10 ms and 50 ms rates.  No torque possible.  

2. **ADS1115 unavailable (I2C failure):** Primary ADC still physically connected.  
   - In ACTIVE: `Safety_CheckSensors()` forces demand=0 AND transitions to LIMP_HOME.  One-shot zero before LIMP_HOME takes effect.  
   - In LIMP_HOME: `main.c` reads primary ADC, applies `× 0.20` hard clamp + `max(20%)` ceiling.  The traction pipeline then applies `Safety_GetTractionCapFactor()` = 0.20 (another 20 % clamp) and the 5 km/h speed cap.  Even if ADC reads 100 %, effective demand = 20 % × 20 % = 4 % of full torque — walking speed only.  

3. **ACTIVE entry guard:** `Safety_SetState(SYS_STATE_ACTIVE)` requires `safety_error == SAFETY_ERROR_NONE`.  Pedal fault sets `SAFETY_ERROR_SENSOR_FAULT`.  ACTIVE is impossible until `Safety_CheckSensors()` clears the error (which only happens when `Pedal_IsPlausible() == true` AND all other sensors pass).  

4. **Boot validation gate:** `SAFETY_ERROR_SENSOR_FAULT` is now allowed through `check_no_safety_error()`, but ACTIVE still requires `error == NONE` in `Safety_SetState()`.  Allowing SENSOR_FAULT in boot validation only enables STANDBY → LIMP_HOME — never STANDBY → ACTIVE.  

5. **CAN protocol:** No changes to CAN message IDs, formats, or rates.  LIMP_HOME is already reported as state=6 in heartbeat byte 1.  

#### Physical Test Procedure

1. **ADS1115 unavailable test:**  
   a. Short SDA to GND via 1 kΩ resistor after init (simulates I2C bus failure).  
   b. Wait > 500 ms (stale timeout).  
   c. Verify CAN heartbeat: byte 1 = 0x06 (LIMP_HOME), byte 2 has sensor fault flag.  
   d. Press pedal.  Verify vehicle moves at walking speed (≤ 5 km/h).  
   e. Release pedal.  Verify vehicle stops (demand returns to 0).  
   f. Remove SDA short.  Wait > 500 ms.  Verify `Pedal_IsPlausible()` returns true.  
   g. If CAN heartbeat present: verify transition to ACTIVE (byte 1 = 0x02).  

2. **Contradictory channels test:**  
   a. Apply fixed 2.5 V to ADS1115 A0 pin (simulates stuck at 50 %).  
   b. Move pedal to 0 % (released).  Wait > 200 ms (divergence timeout).  
   c. Verify CAN heartbeat: byte 1 = 0x06 (LIMP_HOME).  
   d. Press pedal.  Verify vehicle does NOT move (demand = 0).  
   e. Remove fixed voltage, reconnect pedal to ADS1115.  
   f. Verify channels agree → transition to ACTIVE.  

3. **STANDBY pedal fault test:**  
   a. Disconnect ADS1115 I2C before boot.  
   b. Power on.  Wait for boot validation to pass (> 500 ms).  
   c. Do NOT send ESP32 CAN heartbeat (force CAN timeout).  
   d. Verify system reaches LIMP_HOME (not stuck in STANDBY).  
   e. Verify pedal controls traction at walking speed.  

4. **Recovery test:**  
   a. From LIMP_HOME (pedal fault), reconnect ADS1115.  
   b. Verify plausibility restored within 500 ms.  
   c. Send ESP32 CAN heartbeat.  
   d. Verify transition to ACTIVE within RECOVERY_HOLD_MS (500 ms debounce).  

#### Regression Checklist

- [ ] ABS unchanged: 10 km/h min, 15 % slip, 80 ms pulse — no code touched  
- [ ] TCS unchanged: 3 km/h min, 15 % slip, progressive reduction — no code touched  
- [ ] Ackermann differential unchanged: ±15 % correction — no code touched  
- [ ] Thermal protection unchanged: 80°C warning→DEGRADED, 90°C→SAFE — no code touched  
- [ ] Overcurrent unchanged: single > 25 A→DEGRADED, ≥3 consecutive→SAFE — no code touched  
- [ ] Battery UV unchanged: < 20 V→DEGRADED, < 18 V→SAFE — no code touched  
- [ ] CAN timeout → LIMP_HOME transition: unchanged (already fail-operational)  
- [ ] Steering centering fault: unchanged (DEGRADED, not affected by pedal changes)  
- [ ] Encoder fault: unchanged (DEGRADED, not affected by pedal changes)  
- [ ] Obstacle safety: unchanged (CAN advisory, local state machine, no code touched)  
- [ ] Relay sequencing: unchanged (SAFE keeps relays ON, only ERROR cuts power)  
- [ ] ACTIVE entry guard: `safety_error == SAFETY_ERROR_NONE` requirement preserved  
- [ ] Emergency stop: unchanged (→ ERROR → power down)  
- [ ] Watchdog: unchanged (IWDG 500 ms)  
- [ ] CAN message format: unchanged (no contract changes)  
- [ ] Smooth-driving state machine: unchanged (brake/coast/drive phases)  
- [ ] Speed cap in LIMP_HOME: 5 km/h unchanged  
- [ ] Torque limit in LIMP_HOME: 20 % unchanged  
- [ ] Ramp rate in LIMP_HOME: 10 %/s unchanged  

**Rollback:** Revert changes to `sensor_manager.c`, `sensor_manager.h`, `safety_system.c`, `boot_validation.c`, `main.c`.  No CAN contract, flash layout, or HAL configuration changes — clean revert.

---

### Step 5 — Implement obstacle sensor driver in ESP32

**Context:** Risk R6 — obstacle safety module has no sensor data.  
**What files are touched:** `esp32/src/` — new file `obstacle_sensor_driver.cpp/.h`. No STM32 changes.  
**What behavior changes:** ESP32 reads physical ultrasonic or VL53L8CX sensor and populates CAN 0x208 at 66 ms rate. STM32 obstacle module transitions from OBS_STATE_NO_SENSOR to NORMAL/CONFIRMING/ACTIVE based on distance data.  
**Physical test:**  
1. Place obstacle at 30 cm from front sensor.  
2. Verify CAN 0x208 received by STM32 (verify via CAN analyzer).  
3. Verify vehicle forward motion slows (obstacle_scale < 1.0 applied).  
4. Remove obstacle. Verify clearance after 1000 ms (CLEARING → NORMAL).  
5. Disconnect ESP32. Verify obstacle scale returns to 1.0 after 500 ms CAN timeout.  
**Expected CAN:** 0x208 with distance_mm = ~300, zone = 0x02 (WARNING), counter incrementing. 0x205 traction scale reflects obstacle_scale.  
**Rollback:** Remove the new ESP32 driver file. STM32 receives no 0x208 frames → returns to NO_SENSOR state with obstacle_scale = 1.0.

---

### Step 6 — Implement flash persistence for steering calibration

**Context:** Risk R2 — calibration lost on every reset.  
**What files are touched:** `Core/Src/` — new file `flash_storage.c/.h`. `motor_control.c` — write calibration on `Steering_SetCalibrated()`. Read on `Steering_Init()`.  
**What behavior changes:** If flash record present and valid (CRC check), `Steering_IsCalibrated()` returns `true` immediately at boot without waiting for centering to complete. Centering still runs as a verification step but does not gate ACTIVE anymore.  
**Physical test:**  
1. Allow centering to complete. Record calibration to flash.  
2. Power cycle. Verify system reaches ACTIVE without waiting for centering (boot time reduced).  
3. Move steering rack manually to non-center before power-on. Verify system still runs centering to revalidate.  
4. Corrupt flash sector. Verify fallback to centering-required behavior.  
**Expected CAN:** 0x001 byte 1 = 0x02 (ACTIVE) within ~2 s of boot (no centering wait). 0x204 calibrated = 0x01.  
**Rollback:** Remove `flash_storage.c`. Revert `motor_control.c` call. Calibration returns to RAM-only.

---

### Step 7 — Implement relay cutoff in SAFE if decided in Step 1

**Context:** Risk R1 — decision-dependent step.  
**What files are touched:** `Core/Src/safety_system.c` — `Safety_FailSafe()` only.  
**What behavior changes:** `Safety_FailSafe()` calls `Relay_PowerDown()` in addition to `Traction_EmergencyStop()`. Recovery from SAFE requires power cycle (cannot auto-recover without relay power-up sequence).  
**Trade-off to resolve before executing:** Auto-recovery from SAFE requires relay power-up delay (~70 ms). With relays off, CAN heartbeat restoration alone cannot bring the system back to ACTIVE — a power cycle is required. Verify this is acceptable for the limp-home use case.  
**Physical test:**  
1. Trigger SAFE state (overcurrent).  
2. Verify relays de-energise (measure coil voltage drops to 0).  
3. Restore CAN heartbeat. Verify system stays in SAFE (not auto-recover).  
4. Power cycle. Verify clean boot to STANDBY.  
**Expected CAN:** 0x001 byte 1 = 0x04. No status messages after relay cutoff (bus still active from STM32). After power cycle: 0x001 byte 1 = 0x01 (STANDBY).  
**Rollback:** Revert `Safety_FailSafe()`. SAFE returns to traction-stopped but relays-on behavior.

---

## PHASE 5 — Real-World Validation Checklist

### Test T1 — Startup Validation

**Procedure:**
1. Power on system with ESP32 connected and sending heartbeat.
2. Wait for steering centering to complete (observe motor movement).
3. Wait for CAN heartbeat from ESP32 to arrive.
4. Observe CAN 0x001 sequence.

**Expected vehicle behavior:**
- TFT shows boot screen with "CAN: WAITING..." then "CAN: LINKED" when STM32 heartbeat arrives.
- 0x001 byte 1 progresses: 0x00 (BOOT) → 0x01 (STANDBY) → 0x02 (ACTIVE).
- Steering motor sweeps left, then right (or finds center immediately), then stops.
- After ACTIVE: pedal becomes responsive. Wheel speed 0. Traction available.

**Pass criteria:** CAN 0x001 byte 1 = 0x02 within 15 s of power-on under nominal conditions.

---

### Test T2 — Pedal Response

**Procedure:**
1. System in ACTIVE (D1 gear). Apply pedal from 0% to 100% progressively.
2. Monitor CAN 0x200 (speed), 0x201 (current), 0x205 (traction scale).

**Expected vehicle behavior:**
- Vehicle accelerates smoothly (EMA filter + 50%/s ramp).
- Speed increases proportionally.
- Motor currents rise proportionally, no spikes.
- In D1 gear: maximum traction limited to 60% even at 100% pedal.
- ABS/TCS do NOT activate on dry flat surface.

**Pass criteria:** No CAN error flags in 0x001 byte 2. Smooth acceleration without DEGRADED transition.

---

### Test T3 — Steering Response

**Procedure:**
1. System in ACTIVE. Command steering via ESP32 CAN 0x101 from -30° to +30°.
2. Monitor CAN 0x204 (actual angle).

**Expected vehicle behavior:**
- Steering tracks commanded angle with <1° steady-state error.
- Rate-limited: no faster than 200°/s.
- No oscillation at center (deadband = 0.5°).

**Pass criteria:** 0x204 angle matches commanded within 1°. 0x001 byte 2 has no FAULT_ENCODER_ERROR.

---

### Test T4 — Reverse Escape

**Procedure:**
1. System in ACTIVE. Place obstacle at 25 cm front (sensor connected per Step 5).
2. Command forward throttle. Observe vehicle slow.
3. Command GEAR_REVERSE (ESP32 CAN 0x102 byte 1 = 0x01).
4. Apply throttle in reverse.

**Expected vehicle behavior:**
- Forward motion limited by obstacle_scale (approaching 0.3 as distance drops below 500 mm).
- Reverse direction: obstacle_scale does not apply (forward blocked flag does not inhibit reverse).
- Vehicle reverses freely at commanded speed.

**Pass criteria:** CAN 0x205 shows per-wheel traction scale < 100% when obstacle present. Reverse motion achieves normal speed.

---

### Test T5 — CAN Loss Recovery (LIMP_HOME)

**Procedure:**
1. System in ACTIVE. Disconnect ESP32 CAN wire.
2. Wait 300 ms.
3. Apply local pedal input.
4. Reconnect ESP32 CAN.
5. Wait for heartbeat.

**Expected vehicle behavior:**
- After 250 ms: 0x001 byte 1 = 0x06 (LIMP_HOME). ESP32 screen shows mode transition.
- Local pedal drives traction at ≤ 5 km/h (20% torque cap).
- Steering remains operational.
- After reconnect: 0x001 byte 1 = 0x02 (ACTIVE). Full power restored.

**Pass criteria:** Vehicle moves in LIMP_HOME without CAN. No immobilization. Automatic recovery to ACTIVE after heartbeat.

---

### Test T6 — Sensor Loss (Single DS18B20 Failure)

**Procedure:**
1. System in ACTIVE. Disconnect one DS18B20 temperature sensor.
2. Wait for next temperature reading cycle (1000 ms).

**Expected vehicle behavior:**
- 0x001 byte 1 = 0x03 (DEGRADED). 0x001 byte 2 has fault flag set.
- Power limited per DEGRADED level.
- Vehicle remains drivable at reduced power.
- Reconnect sensor: after 500 ms debounce, 0x001 byte 1 returns to 0x02 (ACTIVE).

**Pass criteria:** System enters DEGRADED (not SAFE or ERROR). Vehicle does not stop.

---

### Test T7 — Battery Low (Undervoltage)

**Procedure:**
1. Simulate battery discharge to 19 V (adjust battery or inject voltage via INA226 test point).

**Expected vehicle behavior:**
- 0x001 byte 1 = 0x03 (DEGRADED). Power limited to 40% (L2 for battery fault).
- Vehicle remains drivable at reduced speed.
- At 17.5 V (below critical 18.0 V): 0x001 byte 1 = 0x04 (SAFE). Vehicle stops. NO auto-recovery.

**Pass criteria:** DEGRADED at 19 V. SAFE at 17.5 V. No motor movement in SAFE. No auto-recovery from SAFE without operator power cycle.

---

### Test T8 — Single Wheel Traction (ABS/TCS)

**Procedure:**
1. System in ACTIVE, D2 gear (full power). Place one wheel on low-friction surface.
2. Accelerate aggressively.
3. Monitor CAN 0x205 (per-wheel traction scale) and 0x203 (ABS/TCS active flags).

**Expected vehicle behavior:**
- TCS activates on spinning wheel (0x203 byte 1 = 0x01).
- CAN 0x205: affected wheel traction scale drops (< 100%). Other wheels unaffected.
- No sudden jerk — progressive TCS reduction (40% initial, +5% per cycle).
- After slippage: recovery at 25%/s rate.

**Pass criteria:** CAN 0x205 shows per-wheel differentiation. 0x203 shows TCS active. No DEGRADED or SAFE transition from TCS activation.

---

### Test T9 — LIMP_HOME Activation from ACTIVE (CAN Loss under Load)

**Procedure:**
1. System in ACTIVE at 80% throttle demand.
2. Abruptly disconnect ESP32 CAN during motion.

**Expected vehicle behavior:**
- Within 250 ms: 0x001 byte 1 = 0x06 (LIMP_HOME).
- Vehicle does NOT suddenly stop — traction continues but capped at 20% torque.
- Speed naturally drops to ≤ 5 km/h due to torque cap.
- No unexpected direction change or ABS activation from the transition.

**Pass criteria:** Smooth transition to LIMP_HOME. No abrupt deceleration jerk. Vehicle continues moving.

---

### Test T10 — Obstacle Emergency Reaction (Requires Step 5 complete)

**Procedure:**
1. System in ACTIVE, D1 gear. Vehicle moving forward at ~3 km/h.
2. Place obstacle at 15 cm from front sensor while vehicle is moving.

**Expected vehicle behavior:**
- Distance drops below OBSTACLE_EMERGENCY_MM (200 mm) after temporal confirmation (200 ms).
- obstacle_scale → 0.0. Vehicle stops.
- CAN 0x205 all wheels → 0%.
- Obstacle cleared (moved away): after 1000 ms confirmation, obstacle_scale returns to 1.0.
- Vehicle resumes motion when pedal pressed.

**Pass criteria:** Vehicle stops within 200 ms of obstacle distance dropping below 200 mm. Clean restart after obstacle removed.

---

### Test T11 — Steering Centering Fault + Degraded Boot

**Procedure:**
1. Disconnect inductive center sensor before power-on.
2. Power on system.
3. After Step 2 is implemented: verify vehicle still reaches LIMP_HOME.

**Expected vehicle behavior:**
- Centering attempts sweep, stalls at end-of-travel, enters FAULT within 10 s.
- 0x001 byte 1 = 0x01 (STANDBY). Fault flag = FAULT_CENTERING.
- After Step 2: if CAN timeout also detected → LIMP_HOME. Vehicle moves at walking speed.
- Steering commands are rejected (not calibrated).

**Pass criteria (after Step 2):** Vehicle is not permanently immobilized by centering sensor failure. LIMP_HOME active.

---

### Test T12 — Watchdog Recovery

**Procedure:**
1. Simulate a blocking operation > 500 ms (disconnect HAL_IWDG_Refresh by modifying test build or observing actual IWDG reset in field).
2. Power cycle.

**Expected vehicle behavior:**
- System resets due to IWDG.
- `reset_cause` = `RESET_CAUSE_IWDG` reported in serial output at next boot.
- System boots clean to STANDBY, no error latched.
- Steering re-centers. Boot validation runs. System reaches ACTIVE after centering.

**Pass criteria:** Clean boot after IWDG reset. No permanent fault latched from watchdog event.

---

## Summary Matrix

| Risk | Severity | Current mitigation | Migration step |
|------|----------|--------------------|---------------|
| R1 — SAFE doesn't cut relays | HIGH | EN-pin inhibition | Step 1 (decision), Step 7 (optional implementation) |
| R2 — No NVM calibration | MEDIUM | Centering runs every boot | Step 6 |
| R3 — Centering gates LIMP_HOME | MEDIUM | None | Step 2 (LIMP_HOME), Step 3 (service bypass) |
| R4 — D1 = 60% new cap | LOW-MEDIUM | D2 available for full power | Documentation in QUICK_START |
| R5 — No AI regen braking | LOW | Proportional dynbrake exists | Deferred — Phase 5 feature |
| R6 — No obstacle sensor | MEDIUM | LIMP_HOME speed cap indirect safety | Step 5 |
| R7 — ESP32 needed for ACTIVE | LOW | LIMP_HOME by design | Accepted architectural decision |
| R8 — consecutive_errors not persisted | LOW | MATCH with original | Accepted |
| R9 — ADS1115 fallback unknown | LOW-MEDIUM | Requires code verification | Step 4 |

**Zero-regression requirement:** Steps 1–4 address existing bugs/risks with no functional regression. Steps 5–7 add new capabilities. Each step is independently reversible via single `git revert`.
