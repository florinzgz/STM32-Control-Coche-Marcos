# Obstacle Detection & Safety System — Reference Alignment Audit Report

**Date:** 2026-02-13  
**Branch:** `audit/obstacle-system-reference-alignment`  
**Mode:** STRICT AUDIT ONLY — NO CODE MODIFICATIONS  
**Auditor:** Automated Comparative Analysis  

**Repositories Compared:**
- **Current:** `florinzgz/STM32-Control-Coche-Marcos` (STM32G474RE control firmware)
- **Reference:** `florinzgz/FULL-FIRMWARE-Coche-Marcos` (ESP32-S3 full firmware, v2.17.1 PHASE 14)

---

## PART 1 — REFERENCE REPOSITORY ANALYSIS

### 1.1 Files Implementing Obstacle Detection

| File | Path | Size | Purpose |
|------|------|------|---------|
| `obstacle_detection.cpp` | `src/sensors/obstacle_detection.cpp` | 19,677 B | TOFSense-M S UART driver, frame parser, sensor state management |
| `obstacle_safety.cpp` | `src/safety/obstacle_safety.cpp` | 11,673 B | 5-zone collision avoidance, parking assist, emergency braking |
| `adaptive_cruise.cpp` | `src/control/adaptive_cruise.cpp` | 5,529 B | PID-based ACC with person-following mode |
| `obstacle_detection.h` | `include/obstacle_detection.h` | ~6,000 B | Public API: structs (ObstacleSensor, ObstacleZone, ObstacleStatus), enums (ObstacleLevel, SensorID) |
| `obstacle_config.h` | `include/obstacle_config.h` | ~3,500 B | Hardware constants: UART config, frame format, distance thresholds, error codes |
| `obstacle_safety.h` | `include/obstacle_safety.h` | ~2,500 B | Safety structs (SafetyConfig, SafetyState), zone control API |
| `adaptive_cruise.h` | `include/adaptive_cruise.h` | ~2,000 B | ACC structs (ACCConfig, ACCStatus), PID controller API |
| `pins.h` | `include/pins.h` | ~12,000 B | Pin assignments including `PIN_TOFSENSE_RX=44`, `PIN_TOFSENSE_TX=-1` |
| `menu_obstacle_config.cpp` | `src/menu/menu_obstacle_config.cpp` | 13,412 B | TFT touchscreen config menu: threshold sliders, sensor toggle, EEPROM persistence |
| `SafetyManager.h` | `src/managers/SafetyManager.h` | 909 B | Integration manager: calls `ObstacleSafety::init()` and `ObstacleSafety::update()` |
| `SafetyManagerEnhanced.cpp` | `src/managers/SafetyManagerEnhanced.cpp` | 2,364 B | Heartbeat failsafe: emergency stop if control loop stalls >200ms |

### 1.2 Sensor Type Used in Reference

| Parameter | Value |
|-----------|-------|
| **Sensor Model** | TOFSense-M S LiDAR (by Nooploop) |
| **Type** | 8×8 Matrix Time-of-Flight LiDAR |
| **Communication Interface** | UART (HardwareSerial) |
| **UART Peripheral** | UART0 (ESP32-S3 native UART) |
| **Baud Rate** | 921,600 bps (configured in `obstacle_config.h`, line `UART_BAUDRATE`) |
| **Frame Format** | NLink_TOFSense_M_Frame0 — 400 bytes total |
| **Update Rate** | ~15 Hz |
| **Range** | 4 meters maximum |
| **Field of View** | 65° |
| **Matrix Resolution** | 8×8 = 64 distance points per frame |

#### Frame Format (400 bytes — `obstacle_config.h`)

| Byte Position | Length | Field | Description |
|---------------|--------|-------|-------------|
| `[0-3]` | 4 B | Header | `0x57 0x01 0xFF 0x00` (constant) |
| `[4]` | 1 B | ID | Sensor ID (`0x01`) |
| `[5-6]` | 2 B | Length | `0x0190` = 400 (little-endian) |
| `[7-10]` | 4 B | System Time | Milliseconds (little-endian) |
| `[11-394]` | 384 B | Matrix Data | 64 pixels × 6 bytes each |
| `[395]` | 1 B | Checksum | Sum of bytes `[0..394]` mod 256 |
| `[396-399]` | 4 B | Reserved | Unused |

#### Per-Pixel Data (6 bytes each — `obstacle_detection.cpp`)

| Byte | Field | Format |
|------|-------|--------|
| `[0-2]` | Distance | 3 bytes, little-endian signed, value/256 = mm |
| `[3]` | Signal Strength | 0-255 (clamped to 0-100 for confidence) |
| `[4]` | Status | Pixel measurement status |
| `[5]` | Reserved | Unused |

#### CRC / Checksum Usage

- **Algorithm:** Simple 8-bit sum of all preceding bytes (bytes 0 through 394)
- **Location:** `obstacle_detection.cpp` → `calculateChecksum()` function
- **Validation:** `parseFrame()` compares calculated vs received checksum at position 395
- **On failure:** Logs warning, increments `errorCount`, calls `System::logError(ERROR_CODE_CHECKSUM=810)`

#### Parsing Logic Location

- **Frame accumulation:** `obstacle_detection.cpp` → `update()` function — byte-by-byte UART read with header synchronization
- **Header validation:** `validateFrameHeader()` — matches 4-byte header sequence
- **Pixel parsing:** `parsePixelDistance()` — extracts 3-byte LE signed distance, sign-extends, converts to mm
- **Full frame parsing:** `parseFrame()` — validates header, checksum, iterates 64 pixels, updates sensor state
- **Buffer overflow protection:** `bufferIndex > FRAME_LENGTH` check with error logging
- **Read iteration limit:** `MAX_BYTES_PER_UPDATE = FRAME_LENGTH * 2 = 800 bytes` per update cycle

### 1.3 Hardware Connection in Reference

| Parameter | Value | Source |
|-----------|-------|--------|
| **MCU** | ESP32-S3 N16R8 (DevKitC-1, 44 pins) | `pins.h` |
| **UART Peripheral** | UART0 (native) | `obstacle_config.h`: `UART_NUM = 0` |
| **RX Pin** | GPIO 44 (`PIN_TOFSENSE_RX`) | `pins.h` line: `#define PIN_TOFSENSE_RX 44` |
| **TX Pin** | GPIO -1 (`PIN_TOFSENSE_TX`) — not connected | `pins.h` line: `#define PIN_TOFSENSE_TX -1` |
| **GPIO Enable (XSHUT)** | **None** — eliminated in v2.12.0 migration from VL53L5X | `pins.h` comments |
| **Interrupt Usage** | **None** — polling-based UART read in `update()` | `obstacle_detection.cpp` |
| **Power Control** | **None** — sensor is always powered when main relay active | No power gating logic found |

**Note on TX pin:** `pins.h` comments state the sensor is TX-only (unidirectional), so `PIN_TOFSENSE_TX = -1`. However, `obstacle_detection.cpp` `init()` passes both `PIN_TOFSENSE_RX` and `PIN_TOFSENSE_TX` to `TOFSerial->begin()`, configuring bidirectional UART for potential configuration commands. In practice with TX=-1, Arduino framework treats this as RX-only.

**Bootloop Fix (v2.17.4):** `HardwareSerial` is allocated via `new (std::nothrow)` in `init()` rather than as a global constructor, preventing FreeRTOS stack canary watchpoint triggers during early boot.

### 1.4 Safety Behavior Implemented in Reference

#### 1.4.1 Distance Zones (`obstacle_safety.cpp`)

The reference implements a **5-zone detection system** (v2.12.0):

| Zone | Range (mm) | Threshold Constant | Speed Reduction Factor | Audio Alert | Behavior |
|------|-----------|---------------------|----------------------|-------------|----------|
| **Zone 5** — Emergency | < 200 mm | `ZONE_5_THRESHOLD = 200` | **0.0** (full stop) | `AUDIO_EMERGENCIA` every 1s | `emergencyBrakeApplied = true`, `collisionImminent = true` |
| **Zone 4** — Critical | 200–500 mm | `ZONE_4_THRESHOLD = 500` | **0.10–0.40** (linear) | `AUDIO_EMERGENCIA` every 1s | `collisionImminent = true`, linear interpolation |
| **Zone 3** — Warning | 500–1000 mm | `ZONE_3_THRESHOLD = 1000` | **0.40–0.70** or **0.70** (child reaction) | `AUDIO_ERROR_GENERAL` every 1s | Child reaction detection modifies response |
| **Zone 2** — Caution | 1000–1500 mm | `ZONE_2_THRESHOLD = 1500` | **0.85–1.00** or **1.00** (child reaction) | None | Gentle brake if no child reaction |
| **Zone 1** — Alert | 1500–4000 mm | `ZONE_1_THRESHOLD = 4000` | **1.00** (no reduction) | `AUDIO_ERROR_GENERAL` every 2s | Audio-only warning |
| **No obstacle** | > 4000 mm | — | **1.00** | None | Normal operation |

#### 1.4.2 Speed Reduction Factor Calculation

- **Zone 4:** `factor = 0.1 + (dist - 200) × 0.3 / (500 - 200)`, clamped to `[0.1, 0.4]`
- **Zone 3 (no child reaction):** `factor = 0.4 + (dist - 500) × 0.3 / (1000 - 500)`, clamped to `[0.4, 0.7]`
- **Zone 3 (child reacted):** `factor = 0.7` (soft assist)
- **Zone 2 (no child reaction):** `factor = 0.85 + (dist - 1000) × 0.15 / (1500 - 1000)`, clamped to `[0.85, 1.0]`
- **Zone 2 (child reacted):** `factor = 1.0` (full speed, ACC may manage)

#### 1.4.3 Emergency Stop Behavior

- **Triggered when:** `zone == 5` (distance < 200mm) OR sensor health failure (`sensorsHealthy == 0`)
- **Actions:** `state.emergencyBrakeApplied = true`, `state.collisionImminent = true`, `state.speedReductionFactor = 0.0`
- **Fail-safe philosophy:** If ALL sensors unhealthy, apply emergency brake — assume danger and stop
- **Manual trigger:** `triggerEmergencyStop()` function available
- **Recovery:** `resetEmergencyStop()` clears `emergencyBrakeApplied` flag

#### 1.4.4 Child Reaction Detection (v2.12.0)

| Parameter | Value |
|-----------|-------|
| Threshold | `CHILD_REACTION_THRESHOLD = 10.0f` (10% pedal reduction) |
| Window | `CHILD_REACTION_WINDOW_MS = 500` (500ms) |
| Logic | If child reduces pedal by >10% within 500ms → `childReactionDetected = true` |
| Effect | Zone 3: soft assist (70% instead of 40-70%), Zone 2: full speed (1.0 instead of 0.85-1.0) |

#### 1.4.5 Interaction with ABS/TCS

- **SafetyManager.h** integrates obstacle safety alongside ABS and TCS:
  ```cpp
  inline void update() {
    ABSSystem::update();
    TCSSystem::update();
    ObstacleSafety::update();
  }
  ```
- Obstacle safety produces `speedReductionFactor` which is applied as a **torque multiplier** by the traction system
- ABS/TCS operate **independently** on per-wheel `wheel_scale[]` basis
- Both systems can reduce power simultaneously — the final output is the product of all scaling factors
- No direct cross-module dependency between obstacle zones and ABS/TCS thresholds

#### 1.4.6 Interaction with State Machine

- The reference (ESP32) does **NOT** transition to a separate "SAFE" state in a state machine for obstacle events
- Instead, it **only scales torque** via `speedReductionFactor` (0.0 to 1.0)
- The `emergencyBrakeApplied` flag is a **logical flag**, not a hardware state transition
- There is no relay shutdown triggered by obstacle detection in the reference

#### 1.4.7 Relay Shutdown

- Obstacle detection does **NOT** trigger relay shutdown in the reference
- Relay control is managed separately in `src/control/relays.cpp`
- The obstacle system only controls motor power through the torque scaling factor

#### 1.4.8 ACC Coordination

- **ACC has priority** in zones 2–3 (50–150 cm): `state.accHasPriority = true`
- **Obstacle safety overrides** in zones 4–5 (< 50 cm)
- ACC uses **PID controller** for person-following:
  - `Kp = 0.3`, `Ki = 0.05`, `Kd = 0.15`
  - Target distance: 500mm (50 cm)
  - Emergency brake distance: 300mm (30 cm)
  - Anti-windup on integral: ±500
  - Target lost timeout: 2000ms → return to standby

### 1.5 Follow-Person Logic Analysis

| Question | Answer |
|----------|--------|
| **Is it truly implemented?** | Yes — `adaptive_cruise.cpp` implements PID-based person following |
| **Is it distance + tracking based?** | **Distance only** — single-axis distance tracking via TOFSense-M S front sensor; no lateral tracking, no camera, no person identification |
| **Is it AI or rule-based?** | **Rule-based** — PID controller with fixed gains, not ML/AI. The `regen_ai.cpp` file handles regenerative braking intelligence, not person detection |
| **Is steering involved?** | **No** — ACC only adjusts `speedAdjustment` (0.0–1.0) multiplier on throttle demand. No steering angle modification for tracking |
| **Which modules interact?** | `obstacle_detection.cpp` → provides `getMinDistance(SENSOR_FRONT)`, `adaptive_cruise.cpp` → consumes distance + `Pedal::get()`, produces `speedAdjustment`, `obstacle_safety.cpp` → coordinates zone priority with ACC, `traction.cpp` → applies `speedAdjustment` to motor PWM |
| **Is it a "follow-person" system?** | It is a **distance-maintaining system** — it slows/stops when an object gets closer than target distance and speeds up when it moves away. It does NOT identify, track, or follow a specific person |

---

## PART 2 — CURRENT STM32 FIRMWARE ANALYSIS

### 2.1 Obstacle Detection Code Present?

**NO.** No obstacle detection code exists in the STM32 firmware.

- No files named `obstacle_*` in `Core/Src/` or `Core/Inc/`
- No references to "TOFSense", "LiDAR", "obstacle", or "distance sensor" in any `.c` or `.h` file
- No UART-based sensor driver code
- The service mode module list (`service_mode.c`) references obstacle detection as "ESP32-side, STM32 tolerates absence" — confirming it was always designed to run on the ESP32

### 2.2 UART Configuration Status

The `.ioc` file defines clock frequencies for USART peripherals but **none are initialized or configured**:

| Peripheral | Clock (MHz) | Status |
|------------|-------------|--------|
| USART1 | 170 | **NOT INITIALIZED** — clock defined only, no pin mapping or mode configuration |
| USART2 | 170 | **NOT INITIALIZED** — clock defined only |
| USART3 | 170 | **NOT INITIALIZED** — clock defined only |
| UART4 | 170 | **NOT INITIALIZED** — clock defined only |
| LPUART1 | 170 | **NOT INITIALIZED** — clock defined only |

**No `MX_USARTx_UART_Init()` functions exist in `main.c`**. All UART peripherals are available for future use.

### 2.3 Are USART1/2/3 Free?

**YES.** All USART peripherals (USART1, USART2, USART3, UART4, LPUART1) are free and unconfigured. The STM32G474RE provides:

- USART1: Up to 10.625 Mbit/s (APB2)
- USART2: Up to 5.3125 Mbit/s (APB1)
- USART3: Up to 5.3125 Mbit/s (APB1)
- UART4: Up to 5.3125 Mbit/s (APB1)
- LPUART1: Low-power UART

### 2.4 Placeholder for Obstacle Logic?

**YES — in service_mode.c only.**

`Core/Src/service_mode.c` includes obstacle detection in its module enumeration as Module ID 24:
- Classification: **NON-CRITICAL**
- Description: "obstacle detection (ESP32-side, STM32 tolerates absence)"
- This is a **monitoring slot**, not implementation code — it tracks whether the ESP32 reports obstacle sensor health via CAN

### 2.5 Service Mode Obstacle Module Slot

The service mode architecture supports 25 modules. Module 24 is reserved for obstacle detection:

| Module ID | Name | Classification | Status |
|-----------|------|---------------|--------|
| 24 | Obstacle Detection | NON-CRITICAL | ESP32-side, STM32 tolerates absence |

The STM32 does not enforce obstacle detection — it only monitors the ESP32's reported status.

### 2.6 Free CAN IDs for Obstacle Telemetry

**Current CAN ID allocation:**

| Range | IDs Used | Purpose |
|-------|---------|---------|
| `0x001-0x011` | 2 | Heartbeat (STM32 + ESP32) |
| `0x100-0x110` | 4 | Commands (Throttle, Steering, Mode, Service) |
| `0x200-0x207` | 8 | Status (Speed, Current, Temp, Safety, Steering, Traction, TempMap, Battery) |
| `0x300-0x303` | 4 | Diagnostics (Errors, Service bitmasks) |

**Available ranges for obstacle telemetry:**

| Suggested ID | Purpose |
|-------------|---------|
| `0x208` | Obstacle distance front (min distance, zone level) |
| `0x209` | Obstacle sensor health/status |
| `0x20A` | Obstacle safety state (speedReductionFactor, emergencyBrake) |
| `0x103` | Obstacle configuration command (thresholds) |

There is ample room in the `0x200` status range and `0x100` command range for obstacle-related CAN messages.

---

## PART 3 — ARCHITECTURE COMPATIBILITY CHECK

### 3.1 Can the Reference System Be Ported As-Is?

**NO.** The reference system cannot be ported directly because:

1. **Different MCU architecture:** Reference runs on ESP32-S3 (Arduino framework, C++), STM32 runs bare-metal C with HAL
2. **Different UART API:** Reference uses `HardwareSerial` class (Arduino), STM32 requires `HAL_UART_Receive_IT()` or DMA
3. **Different safety integration:** Reference uses `ObstacleSafety::getSpeedReductionFactor()` applied in ESP32-side traction; STM32 would need to apply this through `wheel_scale[]` or a new scaling path
4. **ESP32-specific dependencies:** `Alerts::play()`, `Logger::info()`, `Pedal::get()`, `Watchdog::feed()`, `millis()` — all ESP32-specific
5. **No FreeRTOS on STM32 (current):** Reference uses `new (std::nothrow)` for lazy allocation; STM32 uses static allocation

### 3.2 Porting Requirements

| Requirement | Complexity | Details |
|-------------|-----------|---------|
| **New UART init** | LOW | Add `MX_USARTx_UART_Init()` in CubeMX, configure 921,600 baud, 8N1 |
| **UART driver** | MEDIUM | Implement DMA or interrupt-based 400-byte frame reception, replace `HardwareSerial` with HAL calls |
| **Frame parser** | LOW | `parseFrame()`, `parsePixelDistance()`, `calculateChecksum()` are pure C-portable — minimal adaptation needed |
| **CAN contract expansion** | LOW | Add 2-3 new CAN IDs (obstacle distance, status, safety state) |
| **Safety state machine extension** | MEDIUM | Option A: New `OBSTACLE_ALERT` state, or Option B: Apply `speedReductionFactor` to existing `wheel_scale[]` without new state |
| **`wheel_scale[]` integration** | LOW | Multiply existing `wheel_scale[i]` by `obstacleSpeedFactor` in `motor_control.c` — single line change per wheel |
| **New error codes** | LOW | Add `ERROR_OBSTACLE_TIMEOUT`, `ERROR_OBSTACLE_CHECKSUM`, `ERROR_OBSTACLE_INVALID` to `safety_system.h` |
| **Child reaction detection** | LOW | Port pedal monitoring logic — already have `ADC_ReadPedal()` in STM32 |
| **ACC/Follow-person** | HIGH | PID controller, target tracking, multi-module coordination — complex and safety-critical |
| **Menu/config UI** | N/A | Runs on ESP32 HMI, not on STM32 — would need CAN-based config commands |

### 3.3 Conflict Analysis

| Existing System | Conflict Risk | Analysis |
|-----------------|--------------|----------|
| **Dynamic braking** | **NONE** | Obstacle `speedReductionFactor` reduces throttle demand; dynamic braking responds to throttle reduction naturally |
| **ABS** | **NONE** | ABS operates on per-wheel slip ratio; obstacle scaling operates on overall demand — independent layers |
| **TCS** | **NONE** | Same as ABS — operates on per-wheel scale independently |
| **Gear logic** | **LOW** | Obstacle system only scales forward/reverse power; gear selection remains independent. However, emergency stop in reverse gear needs consideration |
| **Park hold** | **NONE** | Park hold engages when vehicle is stopped; obstacle emergency stop would complement this behavior |
| **Undervoltage protection** | **NONE** | Battery UV operates at electrical level; obstacle operates at control level. UV can override obstacle speed if battery critical |
| **I2C recovery logic** | **NONE** | TOFSense uses UART, not I2C. Completely separate bus. Legacy VL53L5X I2C code was eliminated in reference v2.12.0 |
| **FDCAN bus** | **LOW** | Adding obstacle CAN messages increases bus load by ~3-5%. At 500 kbps with current ~20% utilization, this is well within capacity |
| **Main loop timing** | **LOW** | Obstacle update at 15 Hz (66ms) fits within existing 100 Hz (10ms) main loop. UART parsing should use DMA to avoid blocking |

### 3.4 Follow-Person Safety in STM32 Split Architecture

**Assessment: NOT RECOMMENDED for STM32 split architecture.**

| Factor | Risk |
|--------|------|
| **Latency** | CAN bus round-trip (ESP32→STM32→Motor) adds 5-10ms latency to control loop. PID controller designed for direct motor access would need re-tuning |
| **Sensor location** | TOFSense sensor connects to ESP32 (UART0). Distance data would cross CAN bus to STM32, adding jitter |
| **Safety authority** | STM32 is the safety authority. Receiving "follow this target" commands from ESP32 would require STM32 to trust ESP32's obstacle assessment — violating the safety separation principle |
| **Failure modes** | CAN timeout during follow mode would trigger emergency stop. This creates a cyclic dependency: follow-person needs fast CAN, but CAN failure triggers stop |
| **Child safety** | A follow-person system on a children's vehicle raises significant safety concerns. Distance-only following without person identification could follow the wrong target |

**Verdict:** Follow-person/ACC should remain ESP32-side if implemented. STM32 should only implement **basic obstacle limiter** (speed reduction based on CAN-reported distance) as a safety backstop.

---

## PART 4 — FINAL OUTPUT STRUCTURE

### 4.1 Exact File Paths (Reference Repository)

```
FULL-FIRMWARE-Coche-Marcos/
├── include/
│   ├── obstacle_detection.h      ← Public API (structs, enums, function declarations)
│   ├── obstacle_config.h         ← Hardware constants, frame format, thresholds
│   ├── obstacle_safety.h         ← Safety structs, zone control API
│   ├── adaptive_cruise.h         ← ACC structs, PID API
│   └── pins.h                    ← Pin assignments (GPIO 44 RX, GPIO 43/-1 TX)
├── src/
│   ├── sensors/
│   │   └── obstacle_detection.cpp ← UART driver, frame parser, sensor state
│   ├── safety/
│   │   └── obstacle_safety.cpp    ← 5-zone collision avoidance, emergency brake
│   ├── control/
│   │   └── adaptive_cruise.cpp    ← PID-based ACC / person-following
│   ├── menu/
│   │   └── menu_obstacle_config.cpp ← TFT config UI, EEPROM persistence
│   └── managers/
│       ├── SafetyManager.h        ← Integration: ABS + TCS + ObstacleSafety
│       └── SafetyManagerEnhanced.cpp ← Heartbeat failsafe (200ms timeout)
```

### 4.2 Exact Pin Assignments (Reference Repository)

| Pin | GPIO | Direction | Function |
|-----|------|-----------|----------|
| `PIN_TOFSENSE_RX` | GPIO 44 | Input | Sensor TX → ESP32 RX (receives LiDAR data) |
| `PIN_TOFSENSE_TX` | GPIO -1 | Not connected | ESP32 TX → Sensor RX (bidirectional config, unused) |

**Historical (removed in v2.12.0):**
- VL53L5X XSHUT pins on GPIO 18, 19, 45, 46 — eliminated with TOFSense migration
- PCA9548A multiplexer @ I2C 0x71 — eliminated (was for VL53L5X array)

### 4.3 Exact UART Configuration

| Parameter | Value | Source |
|-----------|-------|--------|
| Peripheral | UART0 | `obstacle_config.h`: `UART_NUM = 0` |
| Baud Rate | 921,600 bps | `obstacle_config.h`: `UART_BAUDRATE = 921600` |
| Data Bits | 8 | `SERIAL_8N1` in `obstacle_detection.cpp` |
| Parity | None | `SERIAL_8N1` |
| Stop Bits | 1 | `SERIAL_8N1` |
| Flow Control | None | Not configured |
| RX Pin | GPIO 44 | `pins.h`: `PIN_TOFSENSE_RX = 44` |
| TX Pin | GPIO -1 (disabled) | `pins.h`: `PIN_TOFSENSE_TX = -1` |
| Buffer | 400 bytes static | `frameBuffer[FRAME_LENGTH]` |
| Read Limit | 800 bytes/update | `MAX_BYTES_PER_UPDATE = FRAME_LENGTH * 2` |

### 4.4 Exact Distance Thresholds

| Threshold | Value (mm) | Value (cm) | Constant Name | Source |
|-----------|-----------|-----------|---------------|--------|
| CRITICAL | 200 | 20 | `DISTANCE_CRITICAL` / `ZONE_5_THRESHOLD` | `obstacle_config.h` / `obstacle_safety.cpp` |
| WARNING | 500 | 50 | `DISTANCE_WARNING` / `ZONE_4_THRESHOLD` | `obstacle_config.h` / `obstacle_safety.cpp` |
| CAUTION | 1000 | 100 | `DISTANCE_CAUTION` / `ZONE_3_THRESHOLD` | `obstacle_config.h` / `obstacle_safety.cpp` |
| ZONE 2 | 1500 | 150 | `ZONE_2_THRESHOLD` | `obstacle_safety.cpp` |
| ZONE 1 / MAX | 4000 | 400 | `ZONE_1_THRESHOLD` / `DISTANCE_MAX` | `obstacle_safety.cpp` / `obstacle_config.h` |
| INVALID | 65535 | — | `DISTANCE_INVALID = UINT16_MAX` | `obstacle_config.h` |

### 4.5 Exact Scaling Factors

| Zone | Factor Range | Formula |
|------|-------------|---------|
| Zone 5 (Emergency) | 0.0 | Fixed: full stop |
| Zone 4 (Critical) | 0.10 – 0.40 | `0.1 + (dist - 200) × 0.3 / 300` |
| Zone 3 (Warning, no child) | 0.40 – 0.70 | `0.4 + (dist - 500) × 0.3 / 500` |
| Zone 3 (Warning, child reacted) | 0.70 | Fixed: soft assist |
| Zone 2 (Caution, no child) | 0.85 – 1.00 | `0.85 + (dist - 1000) × 0.15 / 500` |
| Zone 2 (Caution, child reacted) | 1.00 | Fixed: full speed |
| Zone 1 (Alert) | 1.00 | Fixed: audio only |
| No obstacle | 1.00 | Fixed: normal |

### 4.6 State Transitions

```
                    ┌──────────────────────────────────────────┐
                    │         Obstacle Safety States           │
                    │                                          │
    Sensors OK      │  Zone 0 ──► speedFactor = 1.0            │
    ───────────────►│  Zone 1 ──► speedFactor = 1.0 + audio    │
                    │  Zone 2 ──► speedFactor = 0.85-1.0       │
                    │  Zone 3 ──► speedFactor = 0.4-0.7        │
                    │  Zone 4 ──► speedFactor = 0.1-0.4        │
                    │  Zone 5 ──► speedFactor = 0.0 (E-STOP)   │
                    └──────────────────────────────────────────┘

    Sensors FAIL    │  sensorsHealthy == 0                      │
    ───────────────►│  speedFactor = 0.0 (FAIL-SAFE E-STOP)    │
                    └──────────────────────────────────────────┘
```

**Note:** These are NOT hardware state machine transitions. The reference firmware uses a **continuous torque scaling model**, not a discrete state machine. The `speedReductionFactor` is recalculated every 50ms (20 Hz) and applied as a multiplier to motor demand.

### 4.7 Power Limiting Behavior

1. `ObstacleSafety::update()` calculates `speedReductionFactor` (0.0–1.0)
2. `ObstacleSafety::getSpeedReductionFactor()` returns the current factor
3. Traction system multiplies throttle demand by this factor
4. The factor is applied **uniformly** to all 4 wheels (not per-wheel)
5. ABS/TCS `wheel_scale[]` applies **additionally** on top (multiplicative)
6. Final motor PWM = `basePWM × obstacleSpeedFactor × wheel_scale[i]`

### 4.8 Emergency Behavior

| Trigger | Response | Recovery |
|---------|----------|----------|
| Distance < 200mm (Zone 5) | `speedReductionFactor = 0.0`, `emergencyBrakeApplied = true`, `AUDIO_EMERGENCIA` | Automatic when distance > 200mm |
| All sensors unhealthy | `speedReductionFactor = 0.0`, `emergencyBrakeApplied = true`, `AUDIO_EMERGENCIA` (fail-safe) | Automatic when any sensor recovers |
| ACC distance < 300mm | ACC `speedAdjustment = 0.0`, PID reset | Automatic when target moves away |
| Heartbeat timeout > 200ms | Motor demand set to 0 (separate system) | Automatic when heartbeat resumes |
| Manual emergency trigger | `triggerEmergencyStop()` → full stop | Manual `resetEmergencyStop()` required |

---

## FINAL ASSESSMENT

### IMPLEMENTATION STATUS (Current STM32)

**NONE**

No obstacle detection or safety code exists in the STM32 firmware. The service_mode.c has a monitoring slot (Module 24) that tracks ESP32-reported obstacle status, but no sensor driver, frame parser, distance evaluation, or safety response logic exists on the STM32 side.

### PORTING COMPLEXITY

**MEDIUM**

- The frame parser and checksum logic are pure arithmetic — easily portable to C
- UART driver requires HAL adaptation (DMA recommended for 921,600 baud at 400 bytes/frame)
- Safety integration requires careful placement in the existing `wheel_scale[]` pipeline
- CAN message definitions needed for ESP32↔STM32 obstacle data exchange
- The 5-zone logic is straightforward but requires thorough testing with the state machine

### RISK LEVEL

**MEDIUM**

- **Hardware risk:** LOW — UART peripherals are free, no pin conflicts on STM32G474RE
- **Software risk:** MEDIUM — Incorrect `speedReductionFactor` application could cause unexpected vehicle behavior
- **Safety risk:** MEDIUM — Emergency stop logic must be carefully validated with existing ABS/TCS/relay systems
- **Integration risk:** LOW — CAN bus has capacity, timing margins exist in main loop
- **Architecture risk:** HIGH (for follow-person only) — CAN latency and safety separation concerns

### RECOMMENDATION

**IMPLEMENT BASIC LIMITER ONLY**

Rationale:
1. The reference obstacle system is designed for ESP32 single-board architecture where sensor, processing, and motor control are on the same MCU
2. In the split architecture (ESP32 HMI + STM32 control), the sensor physically connects to the ESP32 (UART0)
3. The STM32 should implement a **basic obstacle speed limiter** that:
   - Receives obstacle distance via CAN from ESP32 (`0x208`)
   - Applies `speedReductionFactor` to `wheel_scale[]` as a safety backstop
   - Triggers `SAFE` state transition on emergency distance (< 200mm)
   - Does NOT implement full 5-zone logic (leave that on ESP32)
   - Does NOT implement ACC/follow-person (safety-critical, needs direct sensor access)
4. The ESP32 should run the full obstacle detection stack (sensor driver, 5-zone logic, ACC) and send processed distance + zone data to STM32 via CAN
5. This preserves the safety separation principle: STM32 validates and enforces, ESP32 processes and decides

---

*End of Audit Report*
