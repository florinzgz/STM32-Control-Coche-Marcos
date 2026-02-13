# Obstacle System Implementation Strategy — Comparative Design

**Date:** 2026-02-13  
**Mode:** ANALYSIS + DESIGN PROPOSAL ONLY — No code modifications  
**Repositories Compared:**
- **Current (Split):** `florinzgz/STM32-Control-Coche-Marcos` — STM32G474RE (safety authority) + ESP32-S3 (HMI + sensors) via CAN 500 kbps
- **Reference (Monolithic):** `florinzgz/FULL-FIRMWARE-Coche-Marcos` — ESP32-S3 single-board, v2.17.1 PHASE 14

**Source Files Analyzed:**
- `obstacle_detection.cpp` (`src/sensors/`)
- `obstacle_safety.cpp` (`src/safety/`)
- `adaptive_cruise.cpp` (`src/control/`)
- `obstacle_config.h` (`include/`)
- `obstacle_safety.h` (`include/`)
- `pins.h` (`include/`)
- `traction.cpp` (`src/control/`)
- `traction.h` (`include/`)

---

## PART 1 — ORIGINAL DESIGN SUMMARY

### 1) Sensor Connection (GPIO, UART, Baudrate)

The TOFSense-M S LiDAR sensor connects to the ESP32-S3 via **UART0** (native UART):

| Parameter | Value | Source |
|-----------|-------|--------|
| RX Pin | GPIO 44 (`PIN_TOFSENSE_RX`) | `pins.h` |
| TX Pin | GPIO -1 (`PIN_TOFSENSE_TX`) — not physically connected | `pins.h` |
| Baudrate | 921,600 bps | `obstacle_config.h`: `UART_BAUDRATE = 921600` |
| UART Peripheral | UART0 | `obstacle_config.h`: `UART_NUM = 0` |
| Frame Format | 8N1 (`SERIAL_8N1`) | `obstacle_detection.cpp`: `TOFSerial->begin()` |

The sensor is TX-only (unidirectional). The code passes `PIN_TOFSENSE_TX` (which is defined as `-1`) to the `TOFSerial->begin()` call. When the Arduino framework receives `-1` as the TX pin parameter, it disables the TX function entirely, resulting in RX-only operation. GPIO 43 is listed in `pins.h` comments as the physical TX pin for reference, but it is not actually configured or used by the UART peripheral. The `HardwareSerial` object is allocated lazily via `new (std::nothrow)` in `init()` (v2.17.4 bootloop fix) rather than as a global constructor.

### 2) Where Parsing Occurred

All parsing occurs in **`obstacle_detection.cpp`** within the `ObstacleDetection` namespace:

- **Frame accumulation:** `update()` — byte-by-byte UART read with 4-byte header synchronization (`0x57 0x01 0xFF 0x00`), limited to `MAX_BYTES_PER_UPDATE = 800 bytes` per cycle to prevent infinite loops on corrupted data.
- **Header validation:** `validateFrameHeader()` — matches the 4-byte constant header sequence.
- **Pixel distance extraction:** `parsePixelDistance()` — reads 3-byte little-endian signed value, sign-extends from 24-bit to 32-bit, divides by 256 to get millimeters.
- **Full frame parsing:** `parseFrame()` — validates header and checksum (8-bit sum of bytes 0–394), iterates all 64 pixels (8×8 matrix), updates `ObstacleSensor` state with `minDistance` and `proximityLevel`.
- **Checksum calculation:** `calculateChecksum()` — simple 8-bit sum of all preceding bytes.

The 400-byte frame buffer (`frameBuffer[FRAME_LENGTH]`) is statically allocated. Buffer overflow protection exists at `bufferIndex > FRAME_LENGTH`.

### 3) 5-Zone Logic Implementation

The 5-zone system is implemented in **`obstacle_safety.cpp`** within the `ObstacleSafety::update()` function:

| Zone | Distance Range | Threshold Constant | `speedReductionFactor` | Behavior |
|------|---------------|---------------------|----------------------|----------|
| Zone 5 (Emergency) | < 200 mm | `ZONE_5_THRESHOLD = 200` | **0.0** (full stop) | `emergencyBrakeApplied = true`, `collisionImminent = true`, `AUDIO_EMERGENCIA` |
| Zone 4 (Critical) | 200–500 mm | `ZONE_4_THRESHOLD = 500` | **0.10–0.40** (linear) | `collisionImminent = true`, `AUDIO_EMERGENCIA` |
| Zone 3 (Warning) | 500–1000 mm | `ZONE_3_THRESHOLD = 1000` | **0.40–0.70** or **0.70** (child) | Child reaction modifies response |
| Zone 2 (Caution) | 1000–1500 mm | `ZONE_2_THRESHOLD = 1500` | **0.85–1.00** or **1.00** (child) | Gentle brake if no child reaction |
| Zone 1 (Alert) | 1500–4000 mm | `ZONE_1_THRESHOLD = 4000` | **1.00** | Audio-only warning |

Zone determination uses a simple `if/else if` chain comparing `minDist` against threshold constants. The result is stored in `state.obstacleZone` (uint8_t, 0–5).

### 4) speedReductionFactor Calculation

The factor is calculated per zone using **linear interpolation** within each zone's range:

- **Zone 5:** `factor = 0.0` (constant)
- **Zone 4:** `factor = 0.1 + (dist - 200) × 0.3 / (500 - 200)`, clamped to `[0.1, 0.4]`
- **Zone 3 (no child reaction):** `factor = 0.4 + (dist - 500) × 0.3 / (1000 - 500)`, clamped to `[0.4, 0.7]`
- **Zone 3 (child reacted):** `factor = 0.7` (fixed soft assist)
- **Zone 2 (no child reaction):** `factor = 0.85 + (dist - 1000) × 0.15 / (1500 - 1000)`, clamped to `[0.85, 1.0]`
- **Zone 2 (child reacted):** `factor = 1.0` (full speed, ACC may manage)
- **Zone 1 / No obstacle:** `factor = 1.0`

Child reaction detection triggers when pedal reduction > 10% within 500ms (`CHILD_REACTION_THRESHOLD = 10.0f`, `CHILD_REACTION_WINDOW_MS = 500`).

### 5) Interaction with traction.cpp

In `traction.cpp` → `Traction::update()`:

1. `ObstacleSafety::getState(safetyState)` retrieves the current safety state.
2. **Emergency stop (<200mm):** If `safetyState.closestObstacleDistanceMm < 200`, all PCA9685 PWM channels are set to 0 (hardware cutoff) and the function returns immediately — no further PWM computation occurs.
3. **Normal operation:** `obstacleFactor = safetyState.speedReductionFactor` is obtained.
4. `accFactor = AdaptiveCruise::getSpeedAdjustment()` is obtained separately.
5. `combinedFactor = obstacleFactor * accFactor` — multiplicative combination.
6. Per-wheel demand is calculated as: `w[i].demandPct = front × ackermann_factor × combinedFactor`
7. The `combinedFactor` is validated for NaN/Inf and clamped to `[0.0, 1.0]`.

The obstacle factor is applied **uniformly** to all 4 wheels. In the reference ESP32, ABS/TCS operates through separate modules (`ABSSystem::update()`, `TCSSystem::update()`) called via `SafetyManager::update()`. These modules independently modulate individual wheel PWM outputs by directly adjusting the PCA9685 channels — they do NOT feed into the traction demand pipeline shown above via a `wheel_scale[]` array. The obstacle factor and ACC factor are the only external multipliers applied within the `Traction::update()` demand calculation itself.

### 6) SAFE State vs. Torque Scaling

The reference firmware uses **only torque scaling**, NOT a SAFE state transition:

- `speedReductionFactor` (0.0–1.0) is recalculated every 50ms (20 Hz).
- `emergencyBrakeApplied` is a **logical flag**, not a hardware state machine transition.
- There is no `SystemState::SAFE` or equivalent triggered by obstacle events.
- The obstacle system operates as a **continuous torque modifier**, not a discrete state change.
- The only "hard stop" is in `traction.cpp` where `closestObstacleDistanceMm < 200` triggers direct PWM-to-zero hardware cutoff.

### 7) Follow-Person / ACC

Implemented in **`adaptive_cruise.cpp`** within the `AdaptiveCruise` namespace:

- **Type:** PID-based distance-maintaining system (NOT person identification/tracking).
- **PID gains:** `Kp = 0.3`, `Ki = 0.05`, `Kd = 0.15` (tuned in `init()`).
- **Target distance:** 500 mm (50 cm), configurable via `ACCConfig`.
- **Emergency brake:** < 300 mm — `speedAdjustment = 0.0`, PID state reset.
- **Target lost timeout:** 2000 ms — after losing target, returns to standby (`speedAdjustment = 1.0`).
- **Anti-windup:** Integral clamped to ±500.
- **Child pedal safety:** `speedAdjustment = min(adjustment, pedalFactor)` — never exceeds child's pedal demand.
- **Update rate:** 10 Hz (100 ms interval).
- **ACC/Obstacle coordination:** ACC has priority in zones 2–3 (50–150 cm). Obstacle safety overrides in zones 4–5 (<50 cm). Set via `state.accHasPriority` flag.
- **Steering:** Not involved — ACC only modifies `speedAdjustment`, no lateral control.

### 8) Emergency Stop Trigger and Clearing

**Triggered by:**
1. Zone 5 distance (`< 200 mm`): `obstacle_safety.cpp` sets `emergencyBrakeApplied = true`, `speedReductionFactor = 0.0`.
2. Sensor failure (`sensorsHealthy == 0`): Fail-safe emergency brake — assumes danger when sensors are lost.
3. ACC emergency (< 300 mm): `adaptive_cruise.cpp` sets `speedAdjustment = 0.0`.
4. Manual call: `ObstacleSafety::triggerEmergencyStop()`.

**Cleared by:**
1. **Zone-based (auto-recovery):** When distance increases above 200 mm, the next `update()` cycle recalculates the factor to a non-zero value. No explicit reset needed — the state is continuously recalculated.
2. **Sensor recovery (auto-recovery):** When a sensor becomes healthy again, the fail-safe brake is released on the next `update()` cycle.
3. **Manual reset:** `ObstacleSafety::resetEmergencyStop()` clears the `emergencyBrakeApplied` flag.
4. **ACC reset:** `AdaptiveCruise::reset()` clears PID state and returns `speedAdjustment = 1.0`.

Emergency stop **auto-recovers** in the reference — no manual reset required for distance-based or sensor-failure events.

### 9) Relay Involvement

**Relays are NEVER touched by the obstacle system in the reference.**

- `obstacle_safety.cpp` does not include `relays.h` or call any relay functions.
- `traction.cpp` does not open relays during obstacle emergency — it only sets PCA9685 PWM to 0.
- Relay control is managed entirely by `relays.cpp` for power management (main, traction, direction, auxiliary).
- The obstacle system operates exclusively at the **torque/PWM level**, not at the **power relay level**.

### 10) Exact Torque Formula Path

In the reference monolithic ESP32 system, the complete path from pedal to motor is:

```
pedal_percent = Pedal::get().percent                    [pedal.cpp]
    ↓
Traction::setDemand(pedal_percent)                      [traction.cpp]
    ↓
s.demandPct = clamp(pedal_percent, 0, 100)              [traction.cpp]
    ↓
                                                        [traction.cpp::update()]
base = s.demandPct
    ↓
front = base × 0.5  (4x4) or base (4x2)                [50/50 split]
rear  = base × 0.5  (4x4) or 0    (4x2)
    ↓
factorFL, factorFR = Ackermann(steering_angle)          [1.0 - angle^1.2 × 0.3]
    ↓
obstacleFactor = ObstacleSafety::speedReductionFactor   [obstacle_safety.cpp]
accFactor = AdaptiveCruise::getSpeedAdjustment()        [adaptive_cruise.cpp]
combinedFactor = obstacleFactor × accFactor
    ↓
w[FL].demandPct = front × factorFL × combinedFactor
w[FR].demandPct = front × factorFR × combinedFactor
w[RL].demandPct = rear × combinedFactor
w[RR].demandPct = rear × combinedFactor
    ↓
w[i].outPWM = clamp(w[i].demandPct, 0, 100) × 255 / 100   [0-255 range]
    ↓
pwmTicks = round(outPWM × 4095 / 255)                  [PCA9685 12-bit]
    ↓
applyHardwareControl(i, pwmTicks, reverse)              [PCA9685 + MCP23017]
```

**Note:** ABS/TCS in the reference ESP32 operates via `ABSSystem::update()` and `TCSSystem::update()` in a separate `SafetyManager`. These modules independently modulate per-wheel PWM outputs by directly adjusting PCA9685 channels outside the traction demand pipeline. They do NOT feed into the demand calculation above via a `wheel_scale[]` array. In contrast, the STM32 split architecture integrates ABS/TCS via the `wheel_scale[]` array within the traction formula (see Proposed Split section below). The obstacle factor and ACC factor are the only external multipliers in the ESP32's `Traction::update()` demand calculation.

---

## PART 2 — SPLIT ARCHITECTURE CONSTRAINTS

### 1) Keep full obstacle stack on ESP32?

**Yes, predominantly.** The TOFSense-M S sensor physically connects to the ESP32 via UART0 (GPIO 44). The 921,600 baud UART driver, 400-byte frame parser, 8×8 matrix processing, checksum validation, and sensor health monitoring should remain on the ESP32. This avoids:
- Routing a high-speed UART signal across the CAN bus (impossible — CAN has 8-byte payload).
- Duplicating sensor hardware connections.
- Adding UART wiring from the sensor to the STM32 (additional wiring, EMI exposure, new failure point).

### 2) Port full stack to STM32?

**No.** Full porting would require:
- Physical rewiring of the TOFSense-M S UART to a STM32 USART peripheral (new wiring harness).
- Rewriting the Arduino `HardwareSerial` driver to STM32 HAL UART (DMA or interrupt-based).
- Moving all 5-zone logic, child reaction detection, ACC PID controller to bare-metal C.
- Loss of the ESP32 HMI's ability to display obstacle data locally (would need CAN round-trip for display).
- The STM32G474RE has free USART peripherals (1-4 + LPUART1) and could technically support 921,600 baud, but the physical sensor connection to the ESP32 makes this impractical without hardware changes.

### 3) Split responsibilities?

**Yes. This is the recommended approach.** Split as follows:

| Responsibility | Location | Rationale |
|---------------|----------|-----------|
| UART driver + frame parser | ESP32 | Sensor is physically connected to ESP32 UART0 |
| 8×8 matrix processing | ESP32 | Compute-intensive, ESP32 has capacity |
| 5-zone determination | ESP32 | Logic depends on raw distance data, minimal latency |
| `speedReductionFactor` calculation | ESP32 | Depends on child reaction detection (needs local pedal data) |
| ACC / follow-person PID | ESP32 | Safety-critical loop needs direct sensor access, CAN latency unacceptable for PID |
| Audio/visual alerts | ESP32 | HMI is on ESP32 |
| **Distance + zone CAN transmission** | **ESP32 → STM32** | **CAN messages carry processed data** |
| **Safety backstop limiter** | **STM32** | **Validates ESP32-reported distance, applies basic speed limiting** |
| **Emergency stop enforcement** | **STM32** | **Safety authority decides final motor state** |
| **CAN timeout fail-safe** | **STM32** | **If ESP32 stops reporting, STM32 assumes danger** |

### 4) Where should distance parsing live?

**ESP32.** The raw UART parsing (400-byte frames at 921,600 baud, 15 Hz) must stay on the ESP32 because:
- The sensor is physically wired to ESP32 GPIO 44.
- Parsing 400-byte frames at 15 Hz requires dedicated UART buffering — not feasible over CAN (8 bytes/frame).
- The ESP32 sends **processed results** (minimum distance in mm, zone level) via CAN to the STM32.

### 5) Where should 5-zone logic live?

**Primary: ESP32. Secondary: Simplified backstop on STM32.**

- **ESP32:** Full 5-zone logic with linear interpolation, child reaction detection, ACC coordination.
- **STM32:** Simplified 3-tier backstop that receives minimum distance via CAN and applies:
  - Distance < 200 mm → `obstacle_scale = 0.0` (emergency stop, transition to SAFE state)
  - Distance 200–500 mm → `obstacle_scale = 0.3` (heavy braking)
  - Distance 500–1000 mm → `obstacle_scale = 0.7` (moderate reduction)
  - Distance > 1000 mm → `obstacle_scale = 1.0` (no reduction)

This dual-layer approach ensures the STM32 can independently protect the vehicle even if the ESP32's 5-zone logic is corrupted.

### 6) Where should emergency decision live?

**STM32 (safety authority).** The STM32 makes the final emergency stop decision because:
- It controls the relays and motor PWM directly.
- It already has a safety state machine (ACTIVE → DEGRADED → SAFE → ERROR).
- It can enforce the stop even if the ESP32 crashes.
- The STM32 should trigger SAFE state if:
  - CAN-reported distance < 200 mm (Zone 5 equivalent).
  - CAN obstacle messages timeout (no message for >500 ms — ESP32 may have crashed).
  - ESP32 heartbeat lost (existing 250 ms timeout already triggers SAFE).

### 7) Should obstacle event trigger SAFE state or only torque scaling?

**Both, depending on severity:**

| Condition | Response |
|-----------|----------|
| Distance > 500 mm | Torque scaling only (multiply `wheel_scale[]` by `obstacle_scale`) |
| Distance 200–500 mm | Torque scaling (0.3 factor) + transition to DEGRADED if persistent (>2 seconds) |
| Distance < 200 mm | **SAFE state** — relay shutdown, full motor inhibit |
| CAN obstacle timeout | **SAFE state** — assume danger, cannot verify clearance |
| Sensor health = 0 (reported via CAN) | **SAFE state** — fail-safe, assume danger |

This is a **key architectural difference** from the reference monolithic design, which only uses torque scaling. In the split architecture, the STM32 should use its state machine for critical events because:
- The STM32 cannot verify sensor data independently — it must treat ESP32 data loss as a fault.
- The SAFE state ensures relay shutdown, which is a more robust stop mechanism than PWM-to-zero alone.

### 8) Should relays be opened?

**Yes, for Zone 5 / emergency conditions.** Unlike the reference (which never touches relays), the split architecture should open the traction relay (`PIN_RELAY_TRAC`) when:
- Distance < 200 mm and sustained for > 500 ms.
- CAN obstacle messages timeout.
- ESP32 heartbeat lost.

**Rationale:** In the monolithic system, a single ESP32 crash kills both the sensor and the motor controller simultaneously — natural fail-safe. In the split system, the STM32 motors could continue running even if the ESP32 (which monitors obstacles) crashes. Opening relays provides a physical safety interlock that PWM-to-zero cannot guarantee (e.g., if a PCA9685 I²C bus failure locks PWM high).

**Exception:** Direction relay (`PIN_RELAY_DIR`) should remain energized during emergency stop to allow steering control for clearing the obstacle after recovery.

### 9) Should emergency auto-recover or require reset?

**Tiered approach:**

| Trigger | Recovery |
|---------|----------|
| Distance-based (Zone 5, <200 mm) | **Auto-recover** when distance > 500 mm for > 1 second (hysteresis prevents oscillation) |
| CAN obstacle timeout | **Auto-recover** when obstacle messages resume for > 3 consecutive frames |
| Sensor health failure | **Auto-recover** when ESP32 reports sensor healthy via CAN |
| ESP32 heartbeat timeout | **Require ESP32 reboot** — existing behavior, SAFE state is non-recoverable until CAN heartbeat resumes |
| Manual emergency stop | **Require manual reset** via ESP32 HMI or service mode |

The hysteresis for distance-based recovery (clear at 500 mm, trigger at 200 mm) prevents oscillation when an obstacle is right at the 200 mm boundary.

### 10) CAN Messages Structure (IDs only)

| CAN ID | Direction | Content | Rate | DLC |
|--------|-----------|---------|------|-----|
| `0x208` | ESP32 → STM32 | Obstacle distance (min distance mm, zone level 0–5, sensor health bitmask) | 66 ms (15 Hz) | 8 bytes |
| `0x209` | ESP32 → STM32 | Obstacle safety state (speedReductionFactor ×100, emergencyBrakeApplied, collisionImminent, obstacleZone, childReactionDetected) | 100 ms | 8 bytes |
| `0x20A` | ESP32 → STM32 | ACC status (ACCState, currentDistance, speedAdjustment ×100, vehicleDetected) | 100 ms | 8 bytes |
| `0x103` | ESP32 → STM32 | Obstacle config command (threshold overrides — optional, for future use) | On-demand | 8 bytes |

**Bus load impact:** 3 new periodic messages at 66–100 ms intervals add approximately 3–5% to the current ~20% bus utilization at 500 kbps. Well within CAN capacity.

---

## PART 3 — MATHEMATICAL TORQUE PATH

### Original Monolithic (ESP32) — FinalPWM

```
FinalPWM[i] = demandPctToPwm(
    clamp(
        axle_split × ackermann[i] × obstacleFactor × accFactor,
        0, 100
    )
)

Where:
  pedal_pct       = Pedal::get().percent                    (0–100%)
  axle_split      = pedal_pct × 0.5 (4x4) or pedal_pct (4x2 front) or 0 (4x2 rear)
  ackermann[FL]   = 1.0 - (|angle|/60)^1.2 × 0.3          (if turning left)
  ackermann[FR]   = 1.0 - (|angle|/60)^1.2 × 0.3          (if turning right)
  ackermann[other]= 1.0
  obstacleFactor  = ObstacleSafety::speedReductionFactor    (0.0–1.0)
  accFactor       = AdaptiveCruise::getSpeedAdjustment()    (0.0–1.0)

  demandPctToPwm(pct) = clamp(pct, 0, 100) × 255 / 100    → [0–255]
  pwmTicks = round(outPWM × 4095/255)                      → [0–4095 PCA9685]

Full expansion:
  FinalPWM[i] = round(
    clamp(pedal_pct × axle_ratio × ackermann[i] × obstacleFactor × accFactor, 0, 100)
    × 255/100 × 4095/255
  )

Simplified:
  FinalPWM[i] = round(
    clamp(pedal_pct × axle_ratio × ackermann[i] × obstacleFactor × accFactor, 0, 100)
    × 40.95
  )
```

**Note:** ABS/TCS in the reference ESP32 operates independently via `SafetyManager::update()` but does NOT multiply into this pipeline via `wheel_scale[]`. The reference ESP32 ABS/TCS applies its own PWM modulation separately.

### Proposed Split Architecture (STM32 + ESP32) — FinalPWM

```
FinalPWM[i] = |effective_demand| × (PWM_PERIOD / 100) × gear_scale × wheel_scale[i]

Where:
  raw_pedal       = ADC_ReadPedal()                         (STM32 ADC)
  filtered_pedal  = EMA(raw_pedal, α=0.15)                  (smoothed)
  ramped_pedal    = ramp_limit(filtered, 50%/s↑, 100%/s↓)   (rate-limited)
  gear_scale      = 0.6 (D1), 1.0 (D2), 0.6 (REVERSE)     (gear_position)
  
  --- Dynamic Braking (if throttle dropping) ---
  dynamic_brake   = (prev_demand - ramped_pedal) × 0.5      (max 60%)
  effective_demand = ramped_pedal - dynamic_brake            (can go negative)
  
  --- Safety Scaling (per-wheel) ---
  wheel_scale[i]  = ABS_scale[i] × TCS_scale[i] × obstacle_scale
  
  Where:
    ABS_scale[i]     = 0.0 (wheel locking) or 1.0 (normal)
    TCS_scale[i]     = 1.0 → 0.6 → 0.55 → ... (progressive, 25%/s recovery)
    obstacle_scale   = f(CAN_distance)  ← NEW, from ESP32 via CAN 0x208
  
  --- Final PWM ---
  FinalPWM[i] = |effective_demand| × (PWM_PERIOD / 100) × gear_scale × wheel_scale[i]

  If effective_demand < 0: apply reverse polarity (dynamic braking)
  If gear == PARK: hold brake at 30% PWM
  If gear == NEUTRAL: PWM = 0

Proposed obstacle_scale values (STM32 backstop):
  CAN_distance < 200 mm  → obstacle_scale = 0.0 + SAFE state
  CAN_distance 200-500   → obstacle_scale = 0.3
  CAN_distance 500-1000  → obstacle_scale = 0.7
  CAN_distance > 1000    → obstacle_scale = 1.0
  CAN timeout            → obstacle_scale = 0.0 + SAFE state
```

### Key Differences in Torque Pipeline

| Aspect | Monolithic (ESP32) | Split (STM32+ESP32) |
|--------|-------------------|---------------------|
| Pedal filtering | Software EMA | Hardware ADC + software EMA + ramp limiter |
| Gear scaling | Not in formula (handled elsewhere) | Explicit `gear_scale` multiplier |
| Dynamic braking | Not in obstacle path | Integrated into `effective_demand` |
| Obstacle factor | Applied as global multiplier in traction | Applied via per-wheel `wheel_scale[]` |
| ABS/TCS | Separate modules, independently modulate per-wheel PWM via PCA9685 (not in traction demand formula) | Integrated into `wheel_scale[]` in traction formula (multiplicative with obstacle_scale) |
| ACC factor | Multiplied with obstacle factor | Not present on STM32 (ESP32-only) |
| Safety override | PWM-to-zero in traction.cpp | SAFE state → relay open + PWM = 0 |
| PWM resolution | 12-bit PCA9685 (0–4095) | Timer PWM (configurable period) |

---

## PART 4 — SAFETY COMPARISON

### Original Monolithic Design — Failure Modes

| Failure Mode | Behavior | Severity |
|-------------|----------|----------|
| **Sensor UART timeout** | `sensor.healthy = false` → `sensorsHealthy == 0` → `speedReductionFactor = 0.0` (fail-safe emergency brake) | **HANDLED** — auto-recovery when data resumes |
| **Sensor data corruption** | Checksum validation fails → `errorCount++` → after 10 consecutive errors, `sensor.healthy = false` → emergency brake | **HANDLED** — checksum + error threshold |
| **ESP32 crash** | **TOTAL FAILURE** — both sensor processing AND motor control die simultaneously. Natural fail-safe: motors stop because no PWM updates. However, PCA9685 may hold last PWM value if I²C bus is not reset | **PARTIALLY HANDLED** — watchdog should reset ESP32, but PCA9685 latch risk exists |
| **Pedal stuck at max** | Child reaction detection only works if pedal is actively reduced. A stuck-at-max pedal gives `childReactionDetected = false`, meaning zones 2–3 apply full braking factor. Zone 5 still triggers full stop regardless of pedal | **HANDLED** — zone 5 overrides pedal |
| **I²C bus failure (PCA9685)** | PCA9685 holds last PWM value. If bus fails during full-power operation, motors continue at full power. Watchdog timeout would eventually reset, but delay could be dangerous | **PARTIALLY HANDLED** — relays not opened by obstacle system |
| **Obstacle spoofing** | No integrity check on distance data beyond checksum. Injecting UART data at GPIO 44 could report false distances | **NOT HANDLED** — single-factor authentication (checksum only) |
| **Single point of failure** | ESP32 is SPOF for: sensor driver, obstacle logic, traction control, HMI, audio alerts | **HIGH RISK** — no redundancy |

### Proposed Split Design — Failure Modes

| Failure Mode | Behavior | Severity |
|-------------|----------|----------|
| **CAN timeout (no obstacle messages)** | STM32 detects no `0x208` messages for > 500 ms → sets `obstacle_scale = 0.0` → transitions to SAFE state → opens traction relay | **HANDLED** — independent timeout, hardware interlock |
| **ESP32 crash** | STM32 heartbeat timeout (250 ms) → SAFE state → relay open → motors physically disconnected. STM32 continues operating independently with relays open | **HANDLED** — STM32 survives ESP32 crash, hardware safety enforced |
| **STM32 crash** | All PWM outputs go to default (0) on reset. Relays de-energize (fail-safe relay design = normally open). Motors stop. ESP32 detects STM32 heartbeat loss, displays error screen | **HANDLED** — hardware fail-safe (relays are normally-open, de-energize = disconnect) |
| **Sensor data corruption** | ESP32 checksum validation catches corrupted frames → reports unhealthy sensor via CAN `0x208`. STM32 receives unhealthy status → applies `obstacle_scale = 0.0`. Double validation: ESP32 (checksum) + STM32 (CAN health byte) | **HANDLED** — dual-layer validation |
| **Distance spoofing (UART level)** | Same as monolithic — UART injection at ESP32 GPIO 44. However, STM32 has independent CAN timeout that would catch a frozen ESP32 | **PARTIALLY HANDLED** — UART spoofing still possible, but CAN timeout provides secondary protection |
| **Distance spoofing (CAN level)** | An attacker injecting CAN messages with false distances at `0x208` could disable safety braking. Mitigation: CAN message authentication (CMAC), sequence counters, or alive counters in obstacle CAN frames | **RISK EXISTS** — CAN bus is unauthenticated in current design. Mitigation recommended |
| **Sensor + CAN simultaneous failure** | Both ESP32 UART and CAN bus fail. STM32 CAN timeout triggers SAFE state within 500 ms. Vehicle stops | **HANDLED** — CAN timeout is the universal backstop |
| **ESP32 sends stale data** | ESP32 obstacle module crashes but CAN driver continues sending cached values. Distance appears valid but is frozen. Mitigation: include frame counter in `0x208` — STM32 detects frozen counter | **RISK EXISTS** — recommended mitigation: monotonic frame counter in CAN message |

### Safety Architecture Verdict

**The split architecture is safer.** Key advantages:

1. **No single point of failure.** The monolithic ESP32 is a SPOF — crash kills both obstacle sensing AND motor control. In the split design, STM32 crash and ESP32 crash are independent failure domains with independent fail-safes.

2. **Hardware-level safety interlock.** The STM32 can open relays (physical motor disconnection) when the ESP32 fails. The monolithic design only sets PWM to zero, which can fail if I²C bus is stuck.

3. **Defense in depth.** Three independent safety layers:
   - Layer 1: ESP32 obstacle system (5-zone logic, checksum validation)
   - Layer 2: STM32 obstacle backstop (simplified distance check, CAN timeout)
   - Layer 3: STM32 safety state machine (SAFE state, relay control, watchdog)

4. **Deterministic response.** STM32 real-time loop (10 ms) provides deterministic obstacle response. ESP32 FreeRTOS scheduling can introduce jitter.

5. **CAN timeout universality.** Every failure mode that results in ESP32 communication loss (crash, sensor failure, software hang) is caught by the STM32's CAN timeout — a single mechanism covers all ESP32-side failures.

**Key weakness of split design:** CAN bus itself becomes a critical path. CAN bus failure (short circuit, connector issue) would blind the STM32 to obstacle data. However, CAN failure also triggers SAFE state via heartbeat timeout, so the vehicle would stop.

---

## PART 5 — FINAL RECOMMENDATION

### Option Analysis

| Option | Description | Pros | Cons |
|--------|------------|------|------|
| **A) Full port to STM32** | Move sensor UART driver, 5-zone logic, ACC to STM32 | Single safety authority, no CAN dependency for obstacle | Requires physical rewiring of sensor, high porting effort, loses ESP32 HMI display of obstacle data |
| **B) Full stack stays on ESP32** | All obstacle logic on ESP32, STM32 has no obstacle awareness | Zero STM32 changes, simplest option | STM32 has no safety backstop, ESP32 crash = unprotected motors, violates safety authority principle |
| **C) Basic limiter on STM32 + full logic on ESP32** | ESP32 runs full obstacle stack, sends distance+zone via CAN. STM32 applies simplified backstop limiter via `wheel_scale[]` | Preserves safety authority, no hardware changes, defense in depth, compatible with current CAN contract | STM32 depends on CAN for obstacle data (mitigated by timeout), slight bus load increase |
| **D) Dual sensor** | Connect sensor to both ESP32 (UART) and STM32 (UART splitter) | True independence, no CAN dependency | Hardware modification, UART bus contention risk, complex |

### Recommendation: **Option C — Basic limiter on STM32 + full logic on ESP32**

**Technical justification:**

1. **No hardware changes required.** The TOFSense-M S sensor stays connected to ESP32 UART0. No rewiring.

2. **Minimal STM32 firmware changes.** The STM32 needs:
   - New CAN RX handler for IDs `0x208` and `0x209` (~50 lines of C).
   - New `obstacle_scale` variable in `safety_system.c` (~20 lines).
   - Multiplication of `obstacle_scale` into existing `wheel_scale[]` in `motor_control.c` (~4 lines per wheel, or 1 line if applied globally).
   - New timeout check in safety state machine for obstacle CAN messages (~15 lines).
   - Total: ~100 lines of new C code.

3. **Preserves safety-first philosophy.** The STM32 (safety authority) validates and enforces. The ESP32 (HMI + sensors) processes and suggests. This matches the existing architecture where ESP32 sends throttle/steering commands and STM32 validates them via `Safety_ValidateThrottle()`.

4. **Compatible with existing CAN contract.** CAN IDs `0x208`, `0x209`, `0x20A` are in the unallocated `0x200` status range. DLC of 8 bytes per message is standard. Bus load increase of 3–5% is negligible.

5. **Defense in depth.** Even if the ESP32's 5-zone logic is incorrect, the STM32's simplified backstop (3-tier check on raw distance) provides an independent safety net. The STM32 applies `obstacle_scale` through the existing `wheel_scale[]` mechanism, which is already validated with ABS/TCS.

6. **ACC remains ESP32-side.** The PID-based follow-person system stays on ESP32 where it has direct sensor access. STM32 does NOT implement ACC — it only receives and applies the combined `speedReductionFactor` as a safety multiplier.

7. **Fail-safe by default.** CAN timeout (no obstacle data for > 500 ms) triggers SAFE state on STM32 — vehicle stops. This is more conservative than the reference (which auto-recovers) but appropriate for a split architecture where communication loss is a real failure mode.

---

## Implementation Strategy Verdict

**Option C: Basic CAN-based obstacle limiter on STM32 + full obstacle detection/safety stack on ESP32.**

The ESP32 runs the complete obstacle system (UART driver, 8×8 matrix parser, 5-zone logic, child reaction detection, ACC PID) and transmits processed distance + zone data to the STM32 via CAN (IDs `0x208`/`0x209`). The STM32 applies a simplified 3-tier obstacle backstop through the existing `wheel_scale[]` pipeline and transitions to SAFE state for emergency distances or CAN timeouts. This preserves safety authority separation, requires no hardware changes, adds ~100 lines of STM32 firmware, and provides defense-in-depth protection that the monolithic architecture cannot offer.
