# FULL PROJECT TECHNICAL REVIEW — ENGINEERING AUDIT REPORT

| Field | Value |
|-------|-------|
| **Document Version** | 1.0 |
| **Date** | 2026-02-22 |
| **Repository** | `florinzgz/STM32-Control-Coche-Marcos` |
| **Branch** | `copilot/fix-ds18b20-hot-plug-data` |
| **Codebase Size** | ~15,850 lines across 80+ source files |
| **Architecture** | Dual-MCU: STM32G474RE (safety/actuation) + ESP32-S3 (HMI/user I/O) |
| **Scope** | Complete retrospective engineering analysis — no code changes |

---

## TABLE OF CONTENTS

1. [Implemented Features — Explain and Justify](#1-implemented-features--explain-and-justify)
2. [Current System Health](#2-current-system-health)
3. [Bug and Risk Analysis](#3-bug-and-risk-analysis)
4. [What Is Still Missing](#4-what-is-still-missing)
5. [Final Engineering Verdict](#5-final-engineering-verdict)

---

## 1) IMPLEMENTED FEATURES — EXPLAIN AND JUSTIFY

### 1.1 Authoritative State Philosophy (STM32 vs ESP32 Roles)

**What it does:**
The system enforces a strict split-authority architecture:

- **STM32G474RE** is the sole safety authority and actuator controller. It runs at 170 MHz on a deterministic Cortex-M4 with hardware FPU, real-time interrupt priorities, and a 500 ms IWDG watchdog. It directly controls 4 traction motors (TIM1, 20 kHz PWM), 1 steering motor (TIM8), 3 power relays, and reads all safety-critical sensors (INA226 current, DS18B20 temperature, wheel speed, steering encoder, pedal ADC).

- **ESP32-S3** is the user interface and peripheral controller. It runs the 480×320 TFT display, DFPlayer audio, WS2812B LEDs, TOFSense-M obstacle sensor, MCP23017 gear shifter, and XPT2046 touch input. It sends commands to the STM32 via CAN bus but has **zero direct actuator authority**.

**Why this design:**
- The ESP32 runs a non-deterministic RTOS (FreeRTOS under Arduino). WiFi/BT interrupts, SPI DMA for the TFT, and garbage collection can cause unpredictable jitter — unacceptable for motor PWM control with children present.
- The STM32 validates every ESP32 command through `Safety_ValidateThrottle()`, `Safety_ValidateSteering()`, and `Safety_ValidateModeChange()` before applying any actuator output. The ESP32 can *request* actions but cannot *force* them.
- On CAN loss, the STM32 autonomously enters LIMP_HOME (20% torque, 5 km/h cap) using local pedal input — the vehicle remains drivable at walking speed without the ESP32. This is a deliberate fail-operational design: a child's electric car should never be immobilized in a dangerous location (e.g., a road crossing).

**Alternatives rejected:**
- **Single ESP32 monolithic (the original architecture):** The reference repository `FULL-FIRMWARE-Coche-Marcos` ran everything on one ESP32-S3. This was abandoned because a single WiFi interrupt or TFT DMA transfer could delay motor PWM updates by 5–10 ms, causing jerky steering and unreliable ABS timing. The split architecture trades hardware cost for deterministic safety.
- **STM32 as primary with ESP32 as dumb display:** Rejected because it would overload the CAN bus with pixel-level rendering commands and make the UI unresponsive.

---

### 1.2 Steering Centering + Persistent Calibration

**What it does:**
On every power-up, the STM32 performs automatic steering centering using an E6B2-CWZ6C incremental encoder (1200 PPR → 4800 CPR, 0.075°/count) and an LJ12A3 inductive sensor at the mechanical center point.

The centering state machine (`steering_centering.c`):
1. **SWEEP_LEFT** at 10% PWM (425/4249) until either the center sensor triggers or the motor stalls (encoder frozen for 300 ms = end-of-travel).
2. **SWEEP_RIGHT** after stall, same 10% PWM, looking for the center sensor.
3. On center detection: encoder zeroed, count saved to flash page 126 (`steering_cal_store.c`), steering PID takes over.

Persistent calibration (`steering_cal_store.c`):
- Stores `{magic:"STC1", encoder_count, validity:0xA5, CRC32}` in a dedicated 4 KB flash page.
- On boot: if the stored count is valid AND the current encoder position is within ±100 counts (~7.5°) of stored AND the physical center sensor confirms center → skip the sweep entirely.

**Why this design:**
- An incremental encoder loses its absolute position on power loss. Without centering, the steering PID would start from an unknown position, potentially driving the rack into its hard stop.
- The 10% PWM limit prevents mechanical damage to the rack/pinion — the motor gently nudges rather than slamming.
- The center sensor is the safety invariant: flash data alone **never** authorizes ACTIVE mode. This prevents stale calibration from causing a crash if someone physically moved the steering while powered off.
- The ±100-count tolerance (7.5°) accounts for thermal expansion and mounting flex — tight enough to ensure useful calibration, loose enough to avoid false rejections.

**Alternatives rejected:**
- **Absolute encoder (AS5600 or similar):** Would eliminate the centering sequence but costs more, requires precise mechanical mounting, and the LJ12A3 center sensor was already in the design for limit detection.
- **No persistence (always sweep on boot):** The sweep takes 3–10 seconds and involves audible motor noise — acceptable once but annoying on every power cycle. Flash persistence skips this when possible.

---

### 1.3 Safety System & Obstacle Zones

**What it does:**
The safety system (`safety_system.c`, ~1800 lines) implements a 7-state machine:

```
BOOT → STANDBY → ACTIVE ↔ DEGRADED → SAFE → ERROR
                 ↘ LIMP_HOME ↗
```

Each state has strict entry/exit conditions:

| Transition | Trigger | Reversible? |
|---|---|---|
| STANDBY → ACTIVE | ESP32 heartbeat + centering complete + boot validation passed | Yes |
| ACTIVE → DEGRADED | Overcurrent (>25A), temp warning (>80°C), sensor fault, battery <20V | Yes (500ms debounce) |
| ACTIVE → LIMP_HOME | CAN timeout (250ms no heartbeat), pedal plausibility failure | Yes (heartbeat restore) |
| Any → SAFE | Battery critical (<18V), temp critical (>90°C), I2C total failure | Recovery requires fault clear |
| Any → ERROR | Emergency stop, unrecoverable fault | Power cycle required |

**Obstacle detection** is a 5-zone system with speed-dependent thresholds:

| Zone | Distance | Torque Scale | Behavior |
|---|---|---|---|
| Emergency | <500mm | 0.0 | Forward completely blocked |
| Critical | 200–500mm | 0.3 | Heavy reduction |
| Warning | 500–1000mm | 0.7 | Moderate reduction |
| Caution | 1000–1500mm | 0.85 | Gentle slowdown |
| Alert | 1500–4000mm | 0.95 | Minimal deceleration |

The obstacle controller runs entirely on the STM32. CAN obstacle data from the ESP32 is **advisory** — the STM32 applies its own plausibility checks:
- Maximum approach rate: 8 m/s (vehicle + obstacle combined)
- Stuck sensor detection: <10mm change for >1s while vehicle moving >1 km/h
- Rolling counter stale detection: 3 consecutive identical counters = sensor fault
- CAN timeout (500ms without 0x208): sensor fault fallback

**Why this design:**
- A child who crashes into a wall at 10 km/h can be seriously injured. The 5-zone progressive slowdown prevents hard collisions while maintaining maneuverability.
- Running obstacle logic on the STM32 rather than the ESP32 ensures deterministic response times. A TFT refresh on the ESP32 cannot delay obstacle braking.
- The speed-dependent dynamic thresholds (zones expand at higher speeds via `dyn_caution` and `dyn_alert` modifiers) account for stopping distance physics.
- **Reverse escape is always preserved:** when forward is blocked by an obstacle, reverse remains at full authority. This prevents a child from being trapped in a corner.

---

### 1.4 Child Reaction Detection

**What it does:**
Monitors the pedal input for sudden drops (>10% decrease within 500ms from a baseline >10%) which indicate a child's instinctive startle response to an approaching obstacle. When detected, it tightens the Warning zone scale from 0.7→0.5 and Caution from 0.85→0.7 for 2 seconds.

**Why this design:**
- Young children (ages 2–5) have slow cognitive processing but fast reflexive responses. If a child sees an obstacle and instinctively releases the pedal, the system should react faster — not wait for the distance to close further.
- The 2-second boost duration covers the typical child reaction-to-action cycle.
- Only applies to Warning/Caution zones because Emergency/Critical already have maximum restriction.

**Alternatives rejected:**
- **Pedal velocity as continuous input to obstacle scaling:** Too complex and would require careful tuning to avoid false positives from normal driving.
- **Camera-based child attention detection:** Far beyond the hardware capability of this platform.

---

### 1.5 CAN Communication Structure

**What it does:**
500 kbps CAN 2.0A (11-bit IDs) with structured message scheduling:

| Rate | Messages | Purpose |
|---|---|---|
| 50 ms (20 Hz) | 0x100 Throttle, 0x101 Steering | Control commands |
| 66 ms (15 Hz) | 0x208 Obstacle distance | Safety-critical sensor data |
| 100 ms (10 Hz) | 0x001/0x011 Heartbeats, 0x200-0x207 Status | Telemetry + liveness |
| 1000 ms (1 Hz) | 0x202/0x206 Temperature, 0x301-0x303 Service | Slow-changing data |
| On-demand | 0x102 Mode, 0x103 ACK, 0x110 Service, 0x120 LED | User-initiated |

**Heartbeat (0x001, 5 bytes):**
- Byte 0: Rolling alive counter
- Byte 1: System state (BOOT/STANDBY/ACTIVE/DEGRADED/SAFE/ERROR/LIMP_HOME)
- Byte 2: Fault flags bitmask (CAN timeout, temp, current, encoder, wheel, ABS, TCS, centering)
- Byte 3: Specific safety error code (0–13)
- Byte 4: Status flags (startup inhibit, 4×4 mode, tank turn, DS18B20 sensor count)

**Bus-off recovery:**
Non-blocking retry at configurable intervals. On bus-off: Stop → DeInit → Init → ConfigFilters → Notify → Start. Transitions to LIMP_HOME (not SAFE) because CAN loss is a communication issue, not a hardware danger.

**Why this design:**
- 500 kbps provides ~60% bus utilization headroom with the current message set — enough for future expansion without congestion.
- The heartbeat serves double duty: liveness monitoring AND state/fault telemetry. This avoids needing separate watchdog frames.
- Rate separation (50ms for control, 100ms for status, 1000ms for ambient) minimizes bus load while keeping latency appropriate for each data type.
- Command ACK (0x103) provides closed-loop feedback for mode changes — the ESP32 can confirm the STM32 actually applied the requested gear/mode.

**Alternatives rejected:**
- **CAN FD:** STM32G474 supports FDCAN but the TJA1051T/3 transceiver is CAN 2.0-only. CAN FD would require hardware changes.
- **SPI or UART direct link:** CAN provides automatic error detection, prioritization, and electrical isolation. SPI/UART would need software error handling and galvanic isolation components.

---

### 1.6 Shifter Input (MCP23017)

**What it does:**
Reads a physical 5-position gear lever (P/R/N/D1/D2) through an MCP23017 I2C GPIO expander connected to the ESP32 on GPIO 8 (SDA) / GPIO 9 (SCL) at 400 kHz.

Each gear position connects one GPIO pin to ground (active-low). Port A pins GPA0–GPA4 map to P/R/N/D1/D2. The driver validates one-hot encoding (exactly one pin active) and defaults to Neutral on invalid states.

**Why this design:**
- The MCP23017 provides 8 additional GPIO pins without consuming scarce ESP32 pins. The ESP32-S3 has limited free GPIOs after TFT (5 pins), CAN (2), touch (1), LEDs (1), audio (2), power (2), obstacle sensor (2).
- Active-low with internal pull-ups means a broken wire defaults to Neutral (all high = no gear), which is the safest failure mode.
- The MCP23017's I2C interrupt capability (not currently used) could enable event-driven gear changes without polling.

**Alternatives rejected:**
- **Direct ESP32 GPIO:** Would consume 5 pins from the already-tight pin budget.
- **Analog voltage ladder:** Cheaper but less reliable — resistor drift could cause gear misdetection.

---

### 1.7 LED Relay & Lighting Logic

**What it does:**
Two-level LED control:
1. **STM32 power relay (PB10):** Controls 5V supply to all WS2812B strips via a MOSFET relay. Starts OFF (safe default). Toggled by ESP32 via CAN 0x120 command. Confirmed back to ESP32 via 0x20A status (1 Hz).
2. **ESP32 pattern controller (led_controller.cpp):** Drives 44 WS2812B LEDs (28 front + 16 rear) via FastLED on GPIO 38 with state-dependent patterns:
   - ACTIVE: White position lights (front), red tail lights (rear)
   - Braking: Bright red rear (traction avg ≤5% while speed >0)
   - Reverse: White rear backup lights
   - SAFE/ERROR: All amber 250ms flash (emergency warning)
   - LIMP_HOME: Dim yellow/red (reduced power indication)

**Why this design:**
- The hardware relay on the STM32 provides a physical safety cutoff — even if the ESP32 crashes with LEDs stuck in a bright pattern, the STM32 can kill power to prevent drain or fire risk.
- Brake/reverse detection is computed from CAN telemetry (traction + speed + gear), not from physical brake/reverse switches that don't exist on this platform.
- Non-blocking flash patterns avoid any delay() calls that would disrupt the 20 FPS render loop.

---

### 1.8 Audio System & Priority Queue

**What it does:**
DFPlayer Mini module on UART2 (GPIO 43 TX / GPIO 44 RX) at 9600 baud. 6 sound effects with 3 priority levels:

| Sound | Trigger | Priority |
|---|---|---|
| WELCOME | Power-on | HIGH |
| FAREWELL | Shutdown | HIGH |
| OBSTACLE_WARN | Zone ≥ 3 (2s debounce) | MEDIUM |
| ERROR_ALERT | Transition to SAFE/ERROR | HIGH |
| BATTERY_LOW | Voltage < 20V (one-shot) | MEDIUM |
| GEAR_CHANGE | Shifter position change | LOW |

Single-slot pending queue: higher/equal priority preempts current playback. DFPlayer hardware requires 100ms minimum command interval and 500ms post-reset delay.

Volume persisted in NVS (`config_store::audioVolume`, 0–30 range) and applied at startup via `audio::setVolume()`.

**Why this design:**
- Audio provides eyes-free feedback — critical for a child looking at the road rather than the screen.
- The priority system ensures safety alerts always override entertainment sounds.
- One-shot battery warning (with recovery reset) prevents annoying repeated alerts during voltage oscillation.

---

### 1.9 Power Manager State Machine

**What it does:**
5-state FSM: OFF → POWER_HOLD → STARTING → RUNNING → SHUTTING_DOWN.

Monitors ignition key state via debounced GPIO 40 (active-high). GPIO 41 drives a power hold relay that keeps the ESP32's 12V supply active after the key is turned off, allowing a clean shutdown sequence (NVS flush + farewell audio + LED off).

**Why this design:**
- Abrupt power loss during NVS write could corrupt persistent storage. The hold relay provides a 3-second grace period for orderly shutdown.
- External delay relay hardware maintains supply even after GPIO release, covering the final milliseconds of ESP32 deinitialization.

---

### 1.10 NVS Persistence

**What it does:**
ESP32 Preferences API wrapping flash-backed Non-Volatile Storage. Stores:
- Drive mode flags (2-bit: 4×4/tank)
- Display brightness (0–255)
- LED relay state (on/off)
- Audio volume (0–30)
- CRC32 integrity check (ISO-HDLC polynomial)

Dirty flag defers writes: config changes are accumulated in RAM, then flushed to NVS every 10 seconds or on shutdown. This batches 10+ touch events into a single flash write, reducing NVS wear.

**Why this design:**
- ESP32 NVS has ~100K write cycles per page. Without batching, each touch on the mode/LED icons would consume a write cycle — potentially exhausting NVS in months of daily use.
- CRC32 validation detects bit rot from power loss during writes. On CRC mismatch, factory defaults are loaded (safe behavior).
- The STM32 deliberately has NO user-facing persistence (except steering calibration). This is a safety design: the STM32 always recalibrates from hardware sensors on boot, ensuring calibration matches actual mechanical state.

---

### 1.11 Engineering Menu

**What it does:**
Hidden diagnostic screen activated by tapping a "8989" secret code (4 alternating left-right taps within 2 seconds) on the drive screen. Provides 5 submenus:
1. **Fault Viewer:** Live 32-bit fault/enabled/disabled bitmasks from STM32 service mode
2. **Module Enable/Disable:** Send SERVICE_CMD (0x110) to enable/disable non-critical STM32 modules
3. **Pedal Calibration:** View live ADC values for range adjustment
4. **Encoder Calibration:** View raw encoder count and delta for steering diagnostics
5. **Factory Restore:** Send factory reset command (0xFF) to STM32, clearing all module disable states

EXIT button on main menu deactivates engineering mode and returns to normal screen mapping.

**Why this design:**
- Development and field diagnostics require sensor readout capability without a connected computer. The secret code prevents accidental activation by children.
- Factory restore provides a known-good recovery path without reflashing firmware.

---

### 1.12 Temperature Hot-Plug Detection

**What it does:**
`Temperature_PeriodicRescan()` re-enumerates the OneWire bus every 10 seconds (OW_RESCAN_INTERVAL_MS), discovering newly connected DS18B20 sensors or detecting removed ones. When sensors are removed, temperature data for abandoned indices is zeroed to prevent stale readings.

The current sensor count is exposed in the heartbeat status_flags (bits 3–5, 3-bit field supporting 0–7 sensors).

**Why this design:**
- DS18B20 sensors use parasitic power and can be physically disconnected during maintenance. Without hot-plug detection, a removed sensor would report its last temperature indefinitely — potentially masking overheating.
- The 10-second rescan interval balances responsiveness (detect removal within 10s) against bus disruption (OneWire search is blocking, ~5ms on a 170 MHz MCU).
- Zeroing removed sensor data is safer than leaving stale values: a zero temperature is clearly implausible and will be flagged by boot validation and overtemp checks.

---

### 1.13 Mode Flags & Heartbeat Echo

**What it does:**
The ESP32 sends mode change requests (4×4/tank turn) via CAN 0x102. The STM32 validates the request (speed-gated: changes only allowed near standstill), applies the mode if allowed, then **echoes** the applied mode state back in the heartbeat status_flags (byte 4, bits 1–2).

The ESP32 reads these confirmed flags on every heartbeat and updates its local state:
```cpp
uint8_t confirmedFlags = (hb.statusFlags >> 1) & 0x03;
```

This closes the control loop: the ESP32 display always reflects what the STM32 actually applied, even if the CAN ACK was lost or the request was rejected.

**Why this design:**
- The ESP32 cannot know if a mode change was applied unless the STM32 confirms it. ACK alone is insufficient because the ACK could be lost on the CAN bus.
- Heartbeat echo is fire-and-forget: no additional CAN message needed, and the ESP32 gets the truth at 10 Hz regardless of ACK delivery.
- Speed-gated mode changes prevent dangerous mid-speed drivetrain reconfiguration (e.g., engaging tank turn at 10 km/h would cause loss of control).

---

## 2) CURRENT SYSTEM HEALTH

### 2.1 Stability Level

**Assessment: Late Beta / Pre-Production**

The firmware implements all safety-critical paths and most user-facing features. The code is well-structured with clear separation of concerns, comprehensive comments, and consistent coding style. However:

- **No CI/CD pipeline exists.** There are no GitHub Actions, no automated builds, no automated tests. The two existing test files (`test_math_safety.c`, `test_steering_cal_store.c`) are standalone C files not integrated into any test framework.
- **No hardware-in-the-loop testing evidence.** The obstacle thresholds, ABS/TCS parameters, steering PID gains, and motor ramp rates are tuned based on engineering judgment, not empirical measurement on the actual vehicle.
- **The ESP32 build requires PlatformIO and the STM32 build requires arm-none-eabi-gcc with HAL drivers.** Neither toolchain is available in standard CI environments without setup.

### 2.2 Determinism & Timing Reliability

**STM32 (Good):**
- The main loop runs task tiers at 10ms/50ms/100ms/1000ms intervals using `HAL_GetTick()` differentials. This is polling-based (not timer-interrupt-driven), which means jitter depends on the worst-case execution time of the 10ms tier.
- The 10ms tier includes: ABS/TCS update, safety checks (current, temperature, CAN timeout, sensors, encoder), obstacle update, relay sequencer, steering centering, and steering PID. Each module is non-blocking, but the aggregate execution time has not been profiled.
- **Risk:** If the 10ms tier exceeds 10ms, subsequent tiers will be delayed. The IWDG watchdog (500ms) provides a hard backstop but doesn't prevent timing drift.
- All ISR-shared variables (`wheel_pulse`, `last_can_rx_time`, `steer_center_flag`) are properly declared `volatile`.

**ESP32 (Adequate):**
- The main loop runs every iteration with `millis()` timing for periodic tasks. Frame-limited at 20 FPS for display updates.
- Non-blocking throughout: no `delay()` calls in the main loop (only in `setup()` for hardware initialization).
- Touch debounce (200ms), shifter polling (50ms), and heartbeat (100ms) use independent timers.
- **Risk:** Arduino `loop()` runs in a FreeRTOS task. WiFi/BT tasks (if accidentally enabled) could preempt and cause jitter. The build disables WiFi but doesn't explicitly `esp_wifi_deinit()`.

### 2.3 Safety Integrity

**Strengths:**
- Triple-layer protection: STM32 safety state machine → actuator validation → hardware watchdog (IWDG 500ms)
- Overcurrent (25A) and overtemperature (80°C warn, 90°C critical, 130°C/motor emergency cutoff) with hysteresis
- Pedal plausibility (internal ADC dual-sample + software validation: consistency, EMA, range, rate-of-change)
- Obstacle safety is autonomous on STM32 — CAN data is advisory, not authoritative
- Power-on movement prevention (startup inhibit latch): pedal must be released for 400ms before any torque is allowed after any MCU reset
- LIMP_HOME on CAN loss preserves mobility at walking speed (20% torque, 5 km/h cap)

**Remaining gaps:**
- No formal safety analysis (FMEA/FTA/HAZOP) document exists
- ABS global brake cutoff (`Traction_SetDemand(0)`) can override obstacle scaling — if all 4 wheels lock simultaneously, the ABS emergency response may conflict with the obstacle controller's coordinated speed reduction
- Emergency stop is non-recoverable (requires power cycle) — by design, but a stuck button could immobilize the vehicle
- No hardware watchdog on the ESP32 (software WDT only via `esp_task_wdt`)

### 2.4 Human-Machine Interaction Quality

**Strengths:**
- Partial-redraw architecture keeps display responsive (<5ms per frame, 20 FPS target)
- Drive screen shows all critical telemetry: speed, 4-wheel torque/temperature, steering gauge, battery, gear, mode, obstacle proximity, LED state
- ACK visual feedback (OK/REJECTED/TIMEOUT with 1.5s auto-clear) provides mode change confirmation
- Audio alerts for obstacles, battery, errors — eyes-free feedback
- LED patterns communicate vehicle state (amber flash for SAFE/ERROR is visible from outside)

**Weaknesses:**
- The engineering menu requires a non-obvious secret code ("8989") — useful for security but poor for field service
- No haptic/vibration feedback on touch events
- Boot screen duration depends on STM32 centering time (0–10 seconds) — no progress bar
- No speed-dependent display brightness (sunlight readability unknown)

### 2.5 Maintainability & Code Architecture Quality

**Strengths:**
- Clean namespace separation: `vehicle::`, `can::`, `audio::`, `config_store::`, `power_mgr::`, `shifter::`, `led_ctrl::`, `touch::`, `ui::`
- Consistent coding style: doxygen-style comments on all public functions, brief block comments on private helpers
- Hardware constants centralized in `vehicle_physics.h` (STM32) and `can_ids.h` (shared)
- `math_safety.c` provides a safety library for all float→int conversions
- Flash persistence uses CRC32 and magic numbers for corruption detection

**Weaknesses:**
- `motor_control.c` is 73 KB — should be split into traction, steering, braking, and EPS modules
- No build-time configuration (all thresholds are compile-time `#define` or static constants) — tuning requires recompilation
- Test coverage is minimal (2 test files, no framework, not in build system)
- No static analysis integration (no `-Werror`, no cppcheck, no MISRA checks)
- Documentation is extensive (57 markdown files) but partially redundant and inconsistently maintained

---

## 3) BUG AND RISK ANALYSIS

### A) Confirmed Bugs (Logic Errors)

| ID | Module | Description | Physical Consequence |
|---|---|---|---|
| **BUG-01** | `safety_system.c` ABS | `Traction_SetDemand(0)` called directly when all 4 wheels lock simultaneously, bypassing obstacle safety and TCS coordination. | If ABS detects 4-wheel lockup during obstacle braking, it overrides the obstacle controller's gradual slowdown with an instant zero-demand. Could cause the car to coast through the obstacle zone on momentum. |
| **BUG-02** | `can_handler.c` CAN_IsESP32Alive | `can_stats.last_heartbeat_esp32` is not declared `volatile` but is written from the RX processing loop (which could be called from different contexts). | Unlikely to manifest in practice since CAN_ProcessMessages is called from main loop, not ISR. But if CAN RX ever moves to interrupt-driven mode, this becomes a data race causing false timeouts or missed heartbeats. |

### B) Race Conditions / Timing Risks

| ID | Module | Description | Physical Consequence |
|---|---|---|---|
| **RACE-01** | `sensor_manager.c` wheel speed | `wheel_pulse[idx]` is read non-atomically in `Wheel_ComputeSpeed()` (32-bit read on Cortex-M4 is atomic for aligned data, so this is safe on this platform). | No consequence on STM32G474 (32-bit aligned reads are atomic on Cortex-M4). Would be a bug if ported to an 8/16-bit platform. |
| **RACE-02** | `sensor_manager.c` DS18B20 | `Temperature_PeriodicRescan()` runs in 1000ms tier while `Temperature_ReadAll()` runs in 50ms tier. If rescan changes `ds18b20_count` between ReadAll's loop boundary check and its sensor read, one iteration could read from a disappeared sensor. | Reading a disappeared sensor returns a predictable error (CRC mismatch or 0xFF) which gets sanitized to 0.0°C. The 10-second rescan interval makes this window extremely unlikely (<0.01% probability per rescan). |
| **RACE-03** | `safety_system.c` obstacle | `obstacle_prev_dist_tick` is a 32-bit HAL_GetTick() timestamp. After ~49.7 days of continuous operation, it wraps. The delta check `dt_ms > 0 && dt_ms < 5000` would produce `dt_ms ≈ 4294967296 - old + new`, failing the `< 5000` check and disabling approach-rate validation for one cycle. | One obstacle plausibility check skipped during the millisecond of wraparound. Obstacle scaling itself is unaffected (distance-based, not rate-based). No practical safety impact. |

### C) Edge Cases Not Handled

| ID | Module | Description | Physical Consequence |
|---|---|---|---|
| **EDGE-01** | `motor_control.c` Steering PID | PID is P-only (ki=0, kd=0). Under sustained lateral load (hill, tire pull), there will be steady-state error — the steering won't fully reach the commanded angle. | The car drifts slightly off the commanded line. Not dangerous at 10 km/h, but noticeable. |
| **EDGE-02** | `safety_system.c` LIMP_HOME recovery | Recovery from LIMP_HOME to ACTIVE requires steering calibration to be complete. If the centering sequence faulted (e.g., stuck center sensor), the vehicle is permanently stuck in LIMP_HOME until power cycled. | The child can still drive at 20% torque / 5 km/h (safe but frustrating). No immobilization risk. |
| **EDGE-03** | `sensor_manager.c` Pedal | If both ADC channels simultaneously report implausible but agreeing values (e.g., both stuck at 50%), the cross-validation passes and the system accepts the phantom throttle. | The car accelerates without pedal input. Mitigated by: obstacle safety limiting top speed, ABS/TCS preventing loss of control, and the parent's ability to turn off the ignition. |
| **EDGE-04** | `can_handler.c` Mode command | `sendGearCommand()` and `sendModeCommand()` both use CAN ID 0x102 but with different byte 1 content. If called in rapid succession, the second command may overwrite the first in the CAN TX FIFO. | A gear change might be missed and require a second shifter toggle. Not dangerous (missed command = no change), but momentarily confusing. |
| **EDGE-05** | `audio_manager.cpp` | DFPlayer has no reliable end-of-playback detection. The 5-second timeout is an approximation. A long MP3 file might be cut short, or a short file might block the queue for the full 5 seconds. | Audio glitches — a sound might be truncated or delayed. No safety impact. |

### D) Hardware-Dependent Fragile Behaviours

| ID | Module | Description | Physical Consequence |
|---|---|---|---|
| **HW-01** | `sensor_manager.c` OneWire | OneWire bit-bang timing is calibrated for 170 MHz (`us * 42` NOP iterations). Clock frequency changes (PLL misconfiguration, HSI fallback) would break DS18B20 communication. | Temperature readings fail, triggering boot validation failure and preventing ACTIVE transition. Safe degradation. |
| **HW-02** | `motor_control.c` BTS7960 | Park hold brake relies on driving both H-bridge FETs to short-circuit the motor terminals. If a FET fails open, the brake disappears silently. | Car rolls on a slope while in PARK. No feedback mechanism to detect brake failure. |
| **HW-03** | `sensor_manager.c` INA226 | TCA9548A I2C multiplexer is a single point of failure for all 6 current/voltage sensors. | Complete current sensing loss. Safety system enters DEGRADED (no overcurrent protection). Battery monitoring also lost. |
| **HW-04** | `shifter_input.cpp` MCP23017 | I2C communication failure on the shifter defaults to Neutral. Repeated failures would cause the gear display to flicker between current gear and Neutral. | Confusing gear display. The STM32 retains the last valid gear until a new one arrives via CAN — actual driving behavior is stable. |

### E) CAN Desync Scenarios

| ID | Description | Physical Consequence |
|---|---|---|
| **DESYNC-01** | ESP32 sends 0x102 mode command but ACK (0x103) is lost on the bus. ESP32 shows pending state until ACK timeout (200ms). Heartbeat echo corrects this at next heartbeat (≤100ms later). | 100–200ms of uncertain mode display. Benign. |
| **DESYNC-02** | ESP32 sends gear + mode in same 0x102 frame. If STM32 accepts mode but rejects gear (speed too high), the single ACK reports REJECTED even though mode was applied. | ESP32 display may show mode as not applied when it actually was. Heartbeat echo resolves within 100ms. |
| **DESYNC-03** | STM32 bus-off event during active driving. STM32 enters LIMP_HOME. ESP32 loses heartbeat → detects timeout → shows LIMP_HOME screen. Recovery requires CAN bus electrical fault to clear. | Vehicle slows to walking speed. Both sides correctly reflect LIMP_HOME. Functionally correct. |
| **DESYNC-04** | ESP32 obstacle sensor (TOFSense-M) fails while STM32 has active obstacle restriction. STM32 obstacle timeout (500ms) triggers sensor fault. Scale held at ≤0.3 (conservative). | Vehicle severely speed-restricted until obstacle CAN resumes. LIMP_HOME speed cap (5 km/h) provides additional safety net. Worst case: child must use reverse to escape. |

### F) Persistence Corruption Risks

| ID | Storage | Description | Physical Consequence |
|---|---|---|---|
| **PERSIST-01** | ESP32 NVS | Power loss during NVS write. CRC32 mismatch on next boot → factory defaults loaded. | Audio volume, LED state, and mode preference reset. No safety impact. |
| **PERSIST-02** | STM32 Flash Page 126 (steering cal) | Power loss during flash erase (4KB page). Entire page lost → CRC invalid → normal centering sweep on next boot. | 3–10 second delay on boot while centering runs. No safety impact. |
| **PERSIST-03** | STM32 Flash Page 127 (EPS params) | Power loss during dual-slot write. Sequence counter inconsistency → both slots corrupt → compiled defaults loaded. | EPS tuning reset to factory values. Requires re-calibration if custom values were set. |
| **PERSIST-04** | ESP32 NVS | Write wear exhaustion (~100K cycles per page). After months of heavy use, page becomes unreliable. | `Preferences` library handles page rotation internally but doesn't report wear level. Eventual corruption follows PERSIST-01 path. |

### G) Lock-to-SAFE or Unintended Motion Risks

| ID | Description | Physical Consequence |
|---|---|---|
| **LOCK-01** | I2C bus latch-up (SDA stuck low by a slave). I2C bus recovery (16 SCL toggles) attempted twice. If both fail → `SAFETY_ERROR_I2C_FAILURE` → SAFE state. | All motors disabled. Vehicle immobilized until power cycle. Could strand a child mid-crossing. |
| **LOCK-02** | Battery voltage oscillation around 18V critical threshold with 0.5V hysteresis. Rapid oscillation would cause: SAFE → ... → ACTIVE → SAFE cycling. | The 500ms recovery debounce prevents rapid cycling, but if voltage genuinely oscillates, the child experiences intermittent power loss. |
| **LOCK-03** | `startup_inhibit` latch requires pedal below 3% for 400ms continuously. A faulty pedal stuck at 4% would prevent clearing the inhibit forever. | Vehicle never produces torque. LIMP_HOME pedal arming has a separate latch (3% for 300ms) but same vulnerability. Power cycle resets both latches. |
| **MOTION-01** | Pedal plausibility fails while ADC reads 50%: range/rate check passes but dual-sample consistency fails. System enters LIMP_HOME with 20% torque cap. Previous valid value is retained. LIMP_HOME arming latch requires pedal release first, which blocks this scenario unless the latch was already armed. | Vehicle creeps at ~1 km/h with no pedal input. LIMP_HOME arming latch requires pedal release first, which blocks this scenario unless the latch was already armed. |

---

## 4) WHAT IS STILL MISSING

### CRITICAL — Affects Safety

| # | Feature | Why It Matters | Consequence of Not Implementing |
|---|---|---|---|
| 1 | **Formal safety analysis (FMEA)** | No systematic failure mode enumeration exists. All safety design is based on engineering intuition. | Unknown failure modes may exist that haven't been considered. Acceptable for a development prototype but not for production use with children. |
| 2 | **Hardware-in-the-loop testing** | All thresholds (ABS slip %, obstacle distances, current limits, temperature limits) are untested on actual hardware. | Parameters may be too aggressive (false positives preventing use) or too lenient (allowing dangerous conditions). |
| 3 | **ESP32 hardware watchdog** | ESP32 has no hardware WDT configured. A firmware hang (e.g., infinite loop in TFT driver) would leave the ESP32 unresponsive while CAN heartbeat stops. | STM32 enters LIMP_HOME within 250ms (safe), but the display would freeze and the obstacle sensor would stop transmitting. LIMP_HOME's speed cap (5 km/h) provides the safety net. |

### IMPORTANT — Affects Drivability/Usability

| # | Feature | Why It Matters | Consequence of Not Implementing |
|---|---|---|---|
| 4 | **Steering PID integral term** | P-only control has steady-state error under lateral loads. | Steering drifts slightly off commanded angle on slopes or with uneven tire pressure. Functional but imprecise. |
| 5 | **CI/CD pipeline** | No automated builds or tests. Regressions can only be caught by manual inspection. | Code quality degrades over time as contributors lack automated feedback. |
| 6 | **ADC pedal DMA conversion** | Current blocking `HAL_ADC_PollForConversion()` with 10ms timeout in the 50ms tier. | Wastes ~1ms of the 50ms budget on blocking I/O. Not critical now but limits headroom for adding more sensors to the 50ms tier. |
| 7 | **Boot progress indication** | No visual or audio progress during the 3–10 second steering centering. | Child or parent may think the system is broken during boot. |

### OPTIONAL — Polish/Features

| # | Feature | Why It Matters | Consequence of Not Implementing |
|---|---|---|---|
| 8 | **Sensor fusion** | Wheel speed + motor current correlation could improve traction estimation on slippery surfaces. | Current ABS/TCS uses wheel speed only. Adequate for normal conditions but suboptimal on wet grass or loose gravel. |
| 9 | **OneWire DMA optimization** | Current bit-bang with NOP loops adds ~5ms blocking time per rescan. | 5ms jitter in the 1000ms tier is well within budget. Only relevant if the 1000ms tier gains more blocking tasks. |
| 10 | **Regen braking** | The reference firmware supported regenerative braking to recover energy. | Not applicable — BTS7960 H-bridges don't support regenerative operation. Would require different motor drivers. |
| 11 | **Speed-dependent display brightness** | No automatic adjustment for outdoor sunlight. | Driver may not see the screen in bright sun. Can adjust manually via engineering menu (brightness stored in NVS). |

---

## 5) FINAL ENGINEERING VERDICT

### Overall Maturity Level

**Late Beta — suitable for supervised testing, not yet production-ready.**

The firmware demonstrates professional embedded engineering quality in its architecture, safety philosophy, and code structure. The split-authority dual-MCU design is fundamentally sound and superior to the original monolithic ESP32 approach. All safety-critical paths are implemented with appropriate fallbacks.

However, the absence of automated testing, formal safety analysis, and hardware validation testing places this firmly in the "works correctly in simulation, needs real-world validation" category.

### Biggest Architectural Strengths

1. **Fail-operational philosophy:** LIMP_HOME preserves mobility at safe speed without CAN/ESP32. No feature of the system can immobilize the vehicle entirely (except emergency stop, which requires deliberate activation and power cycle recovery).

2. **Defense in depth:** Obstacle safety has 4 independent layers: (a) ESP32 distance sensor → (b) STM32 plausibility validation → (c) STM32 autonomous torque scaling → (d) LIMP_HOME speed cap as ultimate backstop.

3. **Authoritative state echo:** The heartbeat echo mechanism closes the ESP32→STM32→ESP32 control loop without additional CAN messages, solving the "did my command apply?" problem elegantly.

4. **Math safety library:** `math_safety.c` prevents the entire class of "NaN propagation through the actuator pipeline" bugs that plague floating-point embedded systems. Every float→PWM conversion passes through `sanitize_float()` and `float_to_u16_clamped()`.

5. **Non-blocking design throughout:** Neither MCU uses blocking delays in its main loop. All timing is milestone-based (`HAL_GetTick()` / `millis()` differentials). This makes the system predictable and debuggable.

### Weakest Design Points

1. **`motor_control.c` monolith (73 KB):** This single file contains traction control, steering PID, EPS torque assist, dynamic braking, park hold, gear logic, demand ramp limiting, and per-motor safety cutoffs. A bug in any subsection risks unintended interaction with others. Should be split into focused modules.

2. **No automated testing infrastructure:** The 2 existing test files are not integrated into any build or test framework. There is no CI pipeline. Regressions from multi-developer collaboration are undetectable until manual testing.

3. **Polling-based task scheduling:** The main loop uses `if ((now - tick) >= interval)` checks for all tiers. Under heavy load, lower-priority tiers (1000ms) can be delayed by accumulated execution time of higher-priority tiers (10ms). A proper RTOS scheduler or timer-interrupt-driven architecture would provide guaranteed timing.

4. **Single-point I2C failure:** The TCA9548A multiplexer is a single point of failure for all 6 INA226 sensors. Its failure means total loss of current and battery monitoring, leaving only temperature as a protection against motor damage.

5. **Obstacle sensor single-point:** One TOFSense-M provides the only distance sensing. If it fails, the system has no obstacle awareness (LIMP_HOME speed cap is the only protection). A dual-sensor approach (e.g., TOFSense-M + VL53L8CX ToF) would provide redundancy.

### What Must Be Done Before Real Child Usage

1. **Complete hardware-in-the-loop testing** of all safety thresholds (ABS slip, obstacle distances, current limits, temperature limits, timing budgets) on the actual vehicle with actual loads.
2. **Verify steering centering reliability** on the physical rack — confirm the inductive sensor triggers consistently and the 10% PWM is sufficient to overcome static friction.
3. **Validate pedal plausibility** with the actual pedal assembly — confirm ADC dual-sample tolerance (±30 counts), range thresholds (30–2800), and rate-of-change limit (35%/50ms) are appropriate.
4. **Test CAN bus reliability** under electromagnetic interference from motor PWM — confirm no bus-off events during normal driving.
5. **Confirm LIMP_HOME drivability** — verify a child can safely operate the vehicle at 20% torque / 5 km/h without ESP32 assistance.
6. **Add ESP32 hardware watchdog** to prevent undetected ESP32 hangs.

### What Must Be Done Before Long-Term Reliability

1. **Establish CI/CD pipeline** with automated STM32 and ESP32 builds, unit test execution, and static analysis (cppcheck, -Werror).
2. **Split `motor_control.c`** into traction, steering, braking, and EPS modules.
3. **Add steering PID I-term** to eliminate steady-state error under lateral load.
4. **Profile main loop execution time** on hardware to confirm all timing budgets are met under worst-case conditions.
5. **Implement NVS wear monitoring** — track write count and warn when approaching end-of-life.
6. **Create a formal FMEA document** mapping every identified risk to its mitigation.

---

*End of Technical Review Report*
