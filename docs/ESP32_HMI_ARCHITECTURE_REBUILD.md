# ESP32-S3 HMI Architecture Rebuild — Full Vehicle Feature Set

**Version:** 2.0  
**Date:** 2026-02-14  
**Status:** Design Phase — No code implementation  
**Scope:** Full-feature architectural redesign for ESP32-S3 HMI (LED + Audio + Obstacle + HMI)  
**Constraint:** STM32 firmware and CAN contract remain unchanged  
**Principle:** Full vehicle functionality with zero bootloop risk

---

## Table of Contents

1. [Phase 1 — Structural Analysis of Original Repo](#phase-1--structural-analysis)
2. [Phase 2 — Clean Architecture Proposal](#phase-2--clean-architecture-proposal)
3. [Phase 3 — LED Subsystem Architecture](#phase-3--led-subsystem-architecture)
4. [Phase 4 — Audio Subsystem Architecture](#phase-4--audio-subsystem-architecture)
5. [Phase 5 — Obstacle Sensor Display](#phase-5--obstacle-sensor-display)
6. [Phase 6 — Hidden Diagnostics Menu](#phase-6--hidden-diagnostics-menu)
7. [Phase 7 — Deliverables](#phase-7--deliverables)
   - [7.1 Clean File Structure](#71-clean-file-structure)
   - [7.2 Initialization Order Diagram](#72-initialization-order-diagram)
   - [7.3 Memory Model](#73-memory-model)
   - [7.4 Watchdog Safety Analysis](#74-watchdog-safety-analysis)
   - [7.5 Render Pipeline Diagram](#75-render-pipeline-diagram)
   - [7.6 Risk Comparison Table](#76-risk-comparison-table)
8. [Phase 8 — Bootloop Prevention Proof](#phase-8--bootloop-prevention-proof)
9. [Phase 9 — Integration Plan](#phase-9--integration-plan)

---

## Phase 1 — Structural Analysis

### 1.1 Original Repository Structure (FULL-FIRMWARE-Coche-Marcos)

The original repository contains approximately **90+ header files** and **15+ source directories** organized as:

```
src/
├── audio/          — DFPlayer MP3 audio feedback
├── control/        — Traction, steering, ABS/TCS control loops
├── core/           — System initialization, boot guard, storage
├── hud/            — HUD layers, compositor, render engine, sprites
├── input/          — Touch, buttons, encoder, pedal input
├── lighting/       — LED controller, FastLED animations
├── logging/        — Serial logger, telemetry logging
├── managers/       — 6 global manager singletons (Control, HUD, Mode, Power, Safety, Sensor, Telemetry)
├── menu/           — Hidden menu, LED control, sensor config, power config, INA226 monitor
├── safety/         — Obstacle detection, regen AI, limp mode
├── sensors/        — Temperature, current (INA226), wheel speed, pedal
├── system/         — RTOS tasks, shared data, watchdog, I2C recovery
├── test/           — Boot sequence test, memory stress test, hardware tests
├── utils/          — Math utilities, filters, queue
├── main.cpp        — 14 KB entry point, ~280 lines
├── i2c.cpp         — I2C bus management
└── test_display.cpp — Display test routines
```

**Dependencies:** 9 external libraries including TFT_eSPI, FastLED, Adafruit MCP23017, DFPlayer, INA226, DallasTemperature, OneWire.

### 1.2 Instability Patterns Identified

The original repository exhibits **7 categories** of patterns that cause early-boot instability:

#### Pattern 1: Global/Static Object Constructors

| Evidence | Risk |
|----------|------|
| `HUDManager` — global singleton with TFT_eSPI member | Constructor runs before `setup()`, may access uninitialized hardware |
| `SafetyManager` — global singleton | Static initialization order undefined across translation units |
| `ControlManager`, `PowerManager`, `SensorManager`, `TelemetryManager`, `ModeManager` — 6 global singletons | Cross-dependency between singletons during construction |
| `SharedData` — global mutex/queue objects | FreeRTOS primitives created before scheduler starts |

**Root cause:** C++ static initialization order fiasco. When Manager A depends on Manager B, and both are global objects, the construction order is undefined across compilation units.

#### Pattern 2: PSRAM Access Before Initialization

| Evidence | Risk |
|----------|------|
| `BOARD_HAS_PSRAM` build flag forces PSRAM dependency | If PSRAM bus fails to initialize, all allocations fail silently |
| `psramInit()` called in `setup()` but TFT_eSPI may allocate before that | TFT_eSPI uses `ps_malloc()` for sprite buffers if PSRAM is detected |
| `board_build.arduino.memory_type = qio_opi` — Octal PSRAM mode | OPI PSRAM requires correct eFuse configuration; misconfigured boards bootloop |
| 8 MB PSRAM dependency for sprite buffers | If PSRAM init fails or is slow, all sprite allocations corrupt memory |

**Root cause:** The N16R8 board uses Octal-SPI PSRAM which requires specific eFuse settings. If the flash/PSRAM mode is misconfigured (QIO vs OPI), the ESP32-S3 bootloops before reaching `setup()`.

#### Pattern 3: Large Sprite Allocations

| Evidence | Risk |
|----------|------|
| `TFT_eSprite` objects used for HUD compositing | Each 320×480×16bpp sprite = 307,200 bytes |
| Shadow render buffer for dirty-rect optimization | Doubles memory usage for partial redraw |
| Multiple HUD layers composited via sprites | N layers × 300 KB = potential multi-MB allocation |
| Sprite creation in `HUDManager::init()` | Single allocation failure cascades to boot failure |

**Root cause:** Sprite-based compositing requires contiguous memory blocks that fragment the heap. On ESP32-S3 with 512 KB internal SRAM, even one full-screen sprite exhausts available memory without PSRAM.

#### Pattern 4: Deep Call Chains in Rendering

| Evidence | Risk |
|----------|------|
| `HUDManager → HUDCompositor → HUDLayer → RenderEngine → SafeDraw → TFT_eSPI` | 6+ levels of function call nesting |
| Each layer has its own `update()` and `draw()` methods | Stack depth multiplied by number of active layers |
| `RenderEvent` queue processed recursively | Potential stack overflow on complex frames |
| Obstacle display overlays on top of HUD layers | Additional compositing depth |

**Root cause:** Deep call chains consume stack space. ESP32-S3 default task stack is 8 KB. With 6+ call levels and local sprite references, stack overflow can occur silently, corrupting adjacent memory.

#### Pattern 5: Blocking Delays

| Evidence | Risk |
|----------|------|
| `while(!Serial && millis() < 2000)` in `setup()` | 2-second potential block at boot |
| `delay(100)` for UART stabilization | Watchdog starvation if accumulated |
| TFT reset sequence: `LOW → delay → HIGH → delay → delay` | 3 sequential delays for display reset |
| Logo display: `while(millis() - logoStart < 1500)` | 1.5-second blocking loop |
| `handleCriticalError()` retry with 5-second delay per attempt | 15 seconds total if 3 retries |

**Root cause:** Blocking delays in `setup()` prevent the FreeRTOS idle task from running, causing watchdog timeouts. The ESP32-S3 internal watchdog fires at 5 seconds by default.

#### Pattern 6: Multi-Core Task Startup Order

| Evidence | Risk |
|----------|------|
| Safety/Control/Power tasks pinned to Core 0 | Core 0 also runs WiFi/BT stack and system tasks |
| HUD/Telemetry tasks pinned to Core 1 | Core 1 tasks may start before all hardware is initialized |
| `SharedData::init()` creates FreeRTOS mutexes | Mutex used before all tasks are created |
| Task priorities: Safety=5, Control=4, Power=3, HUD=2, Telemetry=1 | High-priority tasks on Core 0 can starve idle task |
| No explicit task synchronization barrier | Tasks may race during boot |

**Root cause:** FreeRTOS tasks on ESP32-S3 start executing immediately when created. If a task on Core 1 begins rendering before Core 0 has finished initializing shared data, undefined behavior results.

#### Pattern 7: Bootloop Recovery Mechanisms (Meta-Instability)

| Evidence | Risk |
|----------|------|
| `BootGuard` with boot counter in NVS | Adds NVS flash operations to boot path |
| Reset marker system with multiple marker types | NVS write during boot can fail, causing a different bootloop |
| Safe mode that skips "non-critical" managers | Partial initialization creates inconsistent state |
| `ESP.restart()` in error handler | Restart during failed init can corrupt NVS boot counter |
| 30+ bootloop fix documents in repository | Evidence of unresolved systemic instability |

**Root cause:** The bootloop detection mechanism itself adds complexity to the boot path. NVS operations can fail or be slow, and the safe mode creates a partially initialized system that may not be stable either.

### 1.3 Functional Concepts Extracted

These are the **features** that must be preserved, independent of implementation:

#### Feature Set

| Feature | Description | CAN Dependency |
|---------|-------------|----------------|
| **Boot Screen** | Splash display during system startup | None (local) |
| **Standby Screen** | "System Ready" display with basic telemetry | RX: 0x001 heartbeat |
| **Drive Screen** | Full operational dashboard | RX: 0x200–0x207 (all telemetry) |
| **Safe Mode Screen** | Read-only telemetry, controls disabled | RX: 0x001 (state=4) |
| **Error Screen** | Frozen telemetry, fault display | RX: 0x001 (state=5), 0x300 |
| **Degraded Mode Overlay** | Amber overlay on drive screen | RX: 0x001 (state=3) |
| **Heartbeat Loss Overlay** | Red overlay when STM32 unresponsive | Timeout on 0x001 (>250 ms) |
| **Hidden Diagnostics Menu** | Engineering access to internal data | RX: all; TX: 0x110 service |
| **CAN Heartbeat TX** | Periodic alive signal to STM32 | TX: 0x011 every 100 ms |
| **Command Transmission** | Throttle, steering, mode commands | TX: 0x100, 0x101, 0x102 |
| **ACK Tracking** | Non-blocking command acknowledgment | RX: 0x103 |
| **LED Control (Front)** | 28× WS2812B — Knight Rider sweep, throttle-reactive colors | None (local GPIO47) |
| **LED Control (Rear)** | 16× WS2812B — Position lights, brake, turn signals | None (local GPIO43) |
| **Audio Feedback** | DFPlayer Mini — 68 tracks, event-driven playback | None (local UART1 GPIO19/20) |
| **Obstacle Display** | Real-time distance, zone, health from TOFSense LiDAR | TX: 0x208, 0x209 (ESP32→STM32) |
| **Obstacle Audio Warning** | Proximity beeps triggered by obstacle distance | Linked to obstacle data |

#### CAN Messages (Frozen Contract v1.3)

**RX (STM32 → ESP32):**

| ID | Name | Rate | Data |
|----|------|------|------|
| 0x001 | HEARTBEAT_STM32 | 100 ms | alive_counter, system_state, fault_flags, error_code |
| 0x200 | STATUS_SPEED | 50 ms | 4× wheel speeds (uint16 LE, ×0.1 km/h) |
| 0x201 | STATUS_CURRENT | 100 ms | 4× motor currents (uint16 LE, ×0.01 A) |
| 0x202 | STATUS_TEMP | 1000 ms | 5× temperatures (int8, °C) |
| 0x203 | STATUS_SAFETY | 100 ms | ABS/TCS flags, error_code |
| 0x204 | STATUS_STEERING | 50 ms | angle (int16 LE, ×0.1°), calibrated flag |
| 0x205 | STATUS_TRACTION | 100 ms | 4× traction % (uint8, 0–100) |
| 0x206 | STATUS_TEMP_MAP | 1000 ms | FL/FR/RL/RR/AMB temps (int8, °C) |
| 0x207 | STATUS_BATTERY | 500 ms | current (int16 LE, ×0.01 A), voltage (uint16 LE, ×0.01 V) |
| 0x103 | ACK_RESULT | on-demand | command_id, result_code |
| 0x300 | DIAG_ERROR | on-demand | error_code, subsystem |

**TX (ESP32 → STM32):**

| ID | Name | Rate | Data |
|----|------|------|------|
| 0x011 | HEARTBEAT_ESP32 | 100 ms | alive_counter |
| 0x100 | CMD_THROTTLE | 50 ms | throttle % (uint8, 0–100) — only in ACTIVE/DEGRADED |
| 0x101 | CMD_STEERING | 50 ms | angle (int16 LE, ×0.1°) — only in ACTIVE/DEGRADED |
| 0x102 | CMD_MODE | on-demand | mode + gear — only when speed < 1 km/h |
| 0x110 | SERVICE_CMD | on-demand | subsystem, action — only from diagnostics menu |
| 0x208 | OBSTACLE_DISTANCE | 66 ms | distance (uint16 LE, mm), zone, sensor_health, counter |
| 0x209 | OBSTACLE_SAFETY | 100 ms | speedReductionFactor, emergencyBrake, collisionImminent, zone |

#### User Interactions

| Interaction | Trigger | Constraint |
|-------------|---------|------------|
| Screen auto-transition | CAN heartbeat `system_state` changes | Driven entirely by STM32 |
| Heartbeat loss overlay | No 0x001 for >250 ms | Overlay, not screen change |
| Fault overlay display | `fault_flags` in 0x001 byte 2 | Persistent until cleared |
| Hidden menu entry | Multi-step gesture in STANDBY or ERROR | Zero wheel speed, no CAN timeout |
| Hidden menu navigation | Touch/button input | Non-blocking, frame-limited |
| Throttle command | Pedal input or touch | Only in ACTIVE/DEGRADED |
| Steering command | Steering input or touch | Only in ACTIVE/DEGRADED |
| Mode/gear change | Touch menu selection | Only when speed < 1 km/h |
| LED mode change | Automatic from system state/throttle | No user interaction needed |
| Audio playback | Event-driven (gear change, fault, etc.) | Non-blocking, queue-based |
| Obstacle warning | Proximity distance thresholds | Audio + visual overlay |

---

## Phase 2 — Clean Architecture Proposal

### 2.1 Design Constraints

All decisions below are driven by these hard constraints:

| Constraint | Rationale |
|------------|-----------|
| Zero global objects with constructors | Prevents static init order fiasco and pre-`setup()` crashes |
| Zero PSRAM dependency | Eliminates OPI/QIO eFuse misconfiguration bootloops |
| No `TFT_eSprite` usage | Removes 300 KB+ per sprite memory pressure |
| No dynamic allocation in `loop()` | Prevents heap fragmentation over time |
| No recursion | Guarantees bounded stack usage |
| Frame-limited rendering (20 FPS max) | Prevents watchdog starvation from rendering |
| Deterministic update/draw separation | Ensures data consistency per frame |
| Single-core execution | Eliminates all race conditions and mutex overhead |
| All initialization deferred to `setup()` | Hardware is ready before any object uses it |

### 2.2 Core Architecture: POD + Function Pointers

Instead of C++ classes with constructors, the architecture uses:

- **Plain Old Data (POD) structs** for all state (zero-initialized by default)
- **Free functions** grouped by module (no methods, no `this` pointer)
- **Function pointer tables** for screen dispatch (replaces virtual methods)
- **Static arrays** with compile-time sizes (no `new`, no `malloc` in loop)

```
Architecture Pattern:
                    ┌─────────────────────────┐
                    │       main.cpp           │
                    │  setup() → init chain    │
                    │  loop()  → tick()        │
                    └─────────┬───────────────┘
                              │
                    ┌─────────▼───────────────┐
                    │      app_tick()          │
                    │  1. can_poll()           │
                    │  2. heartbeat_tick()     │
                    │  3. obstacle_tick()      │
                    │  4. led_update()         │
                    │  5. audio_update()       │
                    │  6. screen_update()      │
                    │  7. screen_draw()  [FPS] │
                    │  8. ack_tick()           │
                    └─────────────────────────┘
```

### 2.3 Module Decomposition

Each module is a pair of files (`module.h` + `module.c`/`.cpp`) containing:
- A POD state struct (defined in `.h`, instantiated once in `.cpp`)
- `module_init()` — called once from `setup()`
- `module_update()` — called every loop iteration (fast, non-blocking)
- `module_draw()` — called at 20 FPS (only for UI modules)

| Module | Responsibility | State Size (est.) |
|--------|---------------|-------------------|
| `can_bus` | TWAI init, frame TX/RX, queue polling | 64 bytes |
| `can_rx` | Decode incoming frames → vehicle data | 32 bytes |
| `can_tx` | Encode outgoing frames (heartbeat, commands, obstacle) | 48 bytes |
| `vehicle_data` | Central telemetry store (all CAN RX data) | 128 bytes |
| `heartbeat` | 100 ms heartbeat TX, 250 ms loss detection | 16 bytes |
| `ack_tracker` | Non-blocking ACK with 200 ms timeout | 16 bytes |
| `screen_mgr` | State machine, screen dispatch, transition | 32 bytes |
| `screen_boot` | Boot screen draw functions | 8 bytes |
| `screen_standby` | Standby screen draw functions | 8 bytes |
| `screen_drive` | Drive dashboard draw + dirty flags | 64 bytes |
| `screen_safe` | Safe mode screen draw | 8 bytes |
| `screen_error` | Error screen draw | 8 bytes |
| `overlay` | Heartbeat loss / fault / degraded / obstacle overlays | 32 bytes |
| `diag_menu` | Hidden diagnostics menu (Phase 6) | 128 bytes |
| `ui_car` | Top-view car renderer | 32 bytes |
| `ui_gauge` | Speed/current/temp gauge drawing | 16 bytes |
| `ui_bar` | Pedal bar, battery bar drawing | 8 bytes |
| `ui_icons` | Mode icons (4×4, 4×2, 360°) | 8 bytes |
| `ui_obstacle` | Obstacle distance bar / zone indicator | 16 bytes |
| `led_ctrl` | Non-blocking WS2812B controller (front + rear) | 224 bytes (see Phase 3) |
| `audio_ctrl` | DFPlayer non-blocking driver + priority queue | 48 bytes (see Phase 4) |
| `obstacle` | TOFSense UART driver + 8×8 matrix processing | 464 bytes (see Phase 5) |
| `obstacle_can` | Obstacle CAN TX (0x208/0x209 encoding) | 16 bytes |
| `frame_limit` | 20 FPS frame timing | 8 bytes |
| `tft_direct` | Thin wrapper over TFT_eSPI (no sprites) | 4 bytes (pointer) |

**Total estimated state: ~1,444 bytes** (vs. original: multi-MB with sprites)

> **Note:** The LED array buffers (`CRGB[28]` + `CRGB[16]`) account for 132 bytes
> within `led_ctrl`. The obstacle UART frame buffer (400 bytes) is the largest
> single allocation. All are static, pre-allocated at compile time.

### 2.4 State Machine

```
         ┌──────────┐
    ┌───>│  BOOT(0) │───┐
    │    └──────────┘   │ first heartbeat RX
    │                   ▼
    │    ┌────────────────┐
    │    │  STANDBY(1)    │<──────────────────────┐
    │    └───────┬────────┘                       │
    │            │ system_state==ACTIVE            │
    │            ▼                                │
    │    ┌────────────────┐                       │
    │    │  ACTIVE(2)     │──┐                    │
    │    └────────────────┘  │ state==DEGRADED    │
    │            │           ▼                    │
    │            │   ┌────────────────┐           │
    │            │   │ DEGRADED(3)    │           │
    │            │   └───────┬────────┘           │
    │            │           │                    │
    │            ▼           ▼                    │
    │    ┌────────────────────────┐               │
    │    │      SAFE(4)          │───────────────>│
    │    └───────────────────────┘  state==STANDBY│
    │            │                                │
    │            │ state==ERROR                   │
    │            ▼                                │
    │    ┌────────────────────────┐               │
    │    │     ERROR(5)          │───────────────>│
    │    └───────────────────────┘  state==STANDBY
    │
    └── Heartbeat loss overlay (any state, independent)
```

All transitions are driven **exclusively** by the `system_state` field in CAN message 0x001. The ESP32 never self-transitions except for the initial BOOT → STANDBY on first heartbeat reception.

### 2.5 Rendering Strategy: Direct TFT Drawing

**No sprites. No framebuffer. No PSRAM.**

The rendering pipeline uses TFT_eSPI's direct drawing API:

- `tft.fillRect()` — clear regions before redraw
- `tft.drawString()` — text rendering with built-in fonts
- `tft.drawLine()`, `tft.drawRect()`, `tft.fillCircle()` — geometric primitives
- `tft.setTextColor()`, `tft.setTextDatum()` — styling

**Partial redraw strategy:**
- Each screen maintains dirty flags for each UI region
- On `update()`, compare new data with cached data; set dirty flag if changed
- On `draw()`, only redraw regions with dirty flags set
- Full screen clear only on screen transitions (not every frame)

**Color palette (RGB565):**

| Name | Value | Usage |
|------|-------|-------|
| BG_DARK | 0x2104 | Screen background |
| TEXT_WHITE | 0xFFFF | Primary text |
| TEXT_GRAY | 0x8410 | Secondary text |
| TORQUE_GREEN | 0x07E0 | Normal torque |
| TORQUE_YELLOW | 0xFFE0 | Medium torque |
| TORQUE_RED | 0xF800 | High torque / error |
| SAFE_AMBER | 0xFD20 | Degraded mode overlay |
| ERROR_RED | 0xF800 | Error overlay |
| ACTIVE_GREEN | 0x07E0 | Active gear indicator |
| INACTIVE_GRAY | 0x4208 | Inactive gear indicator |

---

## Phase 3 — LED Subsystem Architecture

### 3.1 Hardware Configuration

| Parameter | Value |
|-----------|-------|
| **Front strip** | 28× WS2812B on GPIO 47 |
| **Rear strip** | 16× WS2812B on GPIO 43 |
| **LED type** | WS2812B (GRB order, 800 kHz) |
| **Power** | 12 V LED strip, level-shifted from ESP32 3.3 V |
| **Library** | FastLED (pre-allocated static arrays) |

**Rear LED segment map:**

| Segment | Indices | Count | Function |
|---------|---------|-------|----------|
| Left turn | 0–2 | 3 | Turn signal (amber blink 500 ms) |
| Center | 3–12 | 10 | Position lights (red 20%) / Brake (red 100%) |
| Right turn | 13–15 | 3 | Turn signal (amber blink 500 ms) |

### 3.2 Front LED Modes

| Mode | Trigger | Animation | Blocking? |
|------|---------|-----------|-----------|
| `FRONT_OFF` | System off / SAFE / ERROR | All LEDs off | No |
| `FRONT_KITT_IDLE` | STANDBY state | Red sweep left↔right (Knight Rider) | No — step-based |
| `FRONT_ACCEL_LOW` | Throttle 1–25% | Red→orange gradient fill | No |
| `FRONT_ACCEL_MED` | Throttle 25–50% | Orange→yellow gradient fill | No |
| `FRONT_ACCEL_HIGH` | Throttle 50–75% | Yellow→green gradient fill | No |
| `FRONT_ACCEL_MAX` | Throttle 75–100% | Green→blue→rainbow fill | No |
| `FRONT_REVERSE` | Gear = R | White scanner sweep | No — step-based |
| `FRONT_ABS_ALERT` | ABS fault flag | Red/white alternating flash | No |
| `FRONT_TCS_ALERT` | TCS fault flag | Amber flash | No |

### 3.3 Rear LED Modes

| Mode | Trigger | Animation | Blocking? |
|------|---------|-----------|-----------|
| `REAR_OFF` | System off | All LEDs off | No |
| `REAR_POSITION` | Lights on, no brake | Center red at 20% brightness | No |
| `REAR_BRAKE` | Brake active | Center red at 100% brightness | No |
| `REAR_BRAKE_EMERGENCY` | Emergency brake | Center red flashing 100 ms toggle | No |
| `REAR_REVERSE` | Gear = R | Center white + side amber | No |
| `REAR_REGEN_ACTIVE` | Regen braking | Blue pulse on center | No — step-based |

### 3.4 Non-Blocking Animation Strategy

**Problem:** Traditional LED animation loops (`for (i=0; i<N; i++) { show(); delay(); }`) block `loop()` for hundreds of milliseconds, starving the watchdog and CAN processing.

**Solution:** Step-based state machine animations. Each animation stores its current position in the POD state struct and advances one step per `led_update()` call.

```
led_update() execution model:

1. Read current front_mode + rear_mode + turn_signal from vehicle state
2. If mode changed → reset animation step counter to 0
3. Compute next LED color values based on:
   - Animation step counter
   - millis() for time-based effects (blink, fade)
   - No loops over frames — single step only
4. Write colors to static CRGB arrays (no allocation)
5. Call FastLED.show() — hardware SPI transfer (~1.1 ms for 44 LEDs)
6. Increment step counter (wrap at animation cycle length)
```

**Knight Rider (KITT) example — fully non-blocking:**
- State: `{ uint8_t position; bool direction; uint32_t last_step_ms; }`
- Every 40 ms: advance position by 1, reverse at ends
- Apply CRGB values with exponential fade trail
- Single `FastLED.show()` call
- Total execution time: **~1.5 ms** (compute + SPI transfer)

### 3.5 Memory Impact

| Item | Size | Notes |
|------|------|-------|
| `CRGB front_leds[28]` | 84 bytes | 28 × 3 bytes (R, G, B) |
| `CRGB rear_leds[16]` | 48 bytes | 16 × 3 bytes (R, G, B) |
| FastLED controller state | ~64 bytes | Internal driver state (static) |
| Animation state (front) | 12 bytes | position, direction, last_step_ms, mode |
| Animation state (rear) | 12 bytes | position, blink_state, last_step_ms, mode |
| Turn signal state | 4 bytes | signal_type, last_blink_ms |
| **Total LED memory** | **~224 bytes** | **All static, zero heap allocation** |

### 3.6 Timing Impact

| Operation | Time | Frequency |
|-----------|------|-----------|
| Compute next colors (front) | ~0.1 ms | Every tick |
| Compute next colors (rear) | ~0.1 ms | Every tick |
| `FastLED.show()` (44 LEDs) | ~1.1 ms | Every tick (rate-limited internally) |
| **Total per tick** | **~1.3 ms** | **Every loop iteration** |

### 3.7 Watchdog Safety

| Metric | Value |
|--------|-------|
| Worst-case `led_update()` time | 1.5 ms |
| WDT timeout | 5,000 ms |
| Margin | **3,333×** |
| Can `FastLED.show()` block? | No — bit-bang transfer is interrupt-driven, completes in ~1.1 ms for 44 LEDs |
| Can animation loops block? | **No** — step-based, single step per call |

### 3.8 Integration Without Instability

The original LED controller (`LEDController` namespace) used global initialization and could potentially block during animations. The new design eliminates these risks:

| Original Risk | New Design Solution |
|---------------|-------------------|
| Global `CRGB` arrays with FastLED constructors | Static POD arrays, `FastLED.addLeds()` called only in `led_init()` during `setup()` |
| Animation `for` loops with `delay()` | Step-based state machine, one step per `led_update()` |
| `FastLED.show()` called from multiple contexts | Single call site in `led_update()`, never from ISR |
| Dynamic brightness changes | Brightness stored in POD config, applied before `show()` |
| Mode transitions during animation | Mode change resets step counter atomically |

### 3.9 Menu-Configurable LED Limits

The hidden diagnostics menu (Phase 6) includes LED configuration:

| Setting | Range | Default | Storage |
|---------|-------|---------|---------|
| Global brightness | 0–255 | 128 | POD struct (runtime only) |
| Front LED enable | on/off | on | POD struct |
| Rear LED enable | on/off | on | POD struct |
| KITT speed | 20–100 ms/step | 40 ms | POD struct |
| Turn signal rate | 200–1000 ms | 500 ms | POD struct |

---

## Phase 4 — Audio Subsystem Architecture

### 4.1 Hardware Configuration

| Parameter | Value |
|-----------|-------|
| **Module** | DFPlayer Mini MP3 |
| **Interface** | UART1 @ 9600 baud |
| **TX pin** | GPIO 19 (ESP32 → DFPlayer RX) |
| **RX pin** | GPIO 20 (DFPlayer TX → ESP32) |
| **Tracks** | 68 MP3 files on SD card |
| **Library** | DFRobotDFPlayerMini (pointer-based lazy init) |

### 4.2 Non-Blocking Playback Design

**Problem:** The DFPlayer library's `begin()` method blocks for up to 3 seconds waiting for a serial response. The `play()` method itself is fast (~5 ms UART command), but `available()` polling can block if the serial buffer is not ready.

**Solution:** Deferred initialization with timeout protection, and a priority queue for track scheduling.

```
Audio pipeline:

  Event occurs (gear change, fault, obstacle)
       │
       ▼
  audio_queue_push(track_id, priority)
       │ Fixed-size ring buffer (8 slots)
       │ If full → drop lowest priority
       ▼
  audio_update() [called every tick]
       │
       ├─ If not initialized → attempt init (non-blocking, with timeout)
       ├─ If DFPlayer busy → skip (check via available())
       ├─ If queue not empty → pop highest priority → dfPlayer->play(track)
       └─ Check for DFPlayer error messages (non-blocking read)
```

### 4.3 Initialization Strategy

The original repo's bootloop was partly caused by `DFRobotDFPlayerMini` and `HardwareSerial(1)` being global objects whose constructors ran before `setup()`. The fix:

| Step | Timing | Action |
|------|--------|--------|
| 1 | `audio_init()` in `setup()` | `Serial1.begin(9600, SERIAL_8N1, GPIO20, GPIO19)` |
| 2 | Same | `dfPlayer.begin(Serial1)` with 1-second timeout |
| 3 | Same | If timeout → mark `audio_ok = false`, log, continue |
| 4 | `audio_update()` in `loop()` | If `!audio_ok` → retry once per 5 seconds (non-blocking) |

**Critical:** `HardwareSerial` and `DFRobotDFPlayerMini` are instantiated as **local variables** inside `audio_init()` and stored via **pointers in a POD struct**, never as global objects. This guarantees zero pre-`setup()` constructor execution.

### 4.4 Priority Queue

| Priority | Level | Example Tracks |
|----------|-------|---------------|
| CRITICAL (3) | Immediate, interrupts current | Emergency (31), Obstacle (54) |
| HIGH (2) | Next after current finishes | ABS (39), TCS (41), Overcurrent (53) |
| NORMAL (1) | FIFO order | Gear changes (20–24), Mode changes (37–38) |
| LOW (0) | Play only if queue empty | Informational (56–60), Beep (68) |

**Queue implementation:** Fixed-size ring buffer of 8 `{track_id, priority}` entries. No dynamic allocation. On overflow, lowest-priority entry is dropped.

### 4.5 Audio Triggers (Event → Track Mapping)

| Event | Track | Priority |
|-------|-------|----------|
| System startup | AUDIO_INICIO (1) | NORMAL |
| System shutdown | AUDIO_APAGADO (2) | NORMAL |
| Gear → D1 | AUDIO_MARCHA_D1 (20) | NORMAL |
| Gear → D2 | AUDIO_MARCHA_D2 (21) | NORMAL |
| Gear → R | AUDIO_MARCHA_R (22) | NORMAL |
| Gear → N | AUDIO_MARCHA_N (23) | NORMAL |
| Gear → P | AUDIO_MARCHA_P (24) | NORMAL |
| Mode → 4×4 | AUDIO_TRACCION_4X4 (37) | NORMAL |
| Mode → 4×2 | AUDIO_TRACCION_4X2 (38) | NORMAL |
| ABS active | AUDIO_ABS_ACTIVADO (39) | HIGH |
| TCS active | AUDIO_TCS_ACTIVADO (41) | HIGH |
| Obstacle detected | AUDIO_OBSTACULO (54) | CRITICAL |
| Emergency stop | AUDIO_EMERGENCIA (31) | CRITICAL |
| Overcurrent | AUDIO_SOBRECORRIENTE (53) | HIGH |
| High temperature | AUDIO_TEMP_ALTA (10) | HIGH |
| Battery low | AUDIO_BATERIA_BAJA (12) | HIGH |
| Battery critical | AUDIO_BATERIA_CRITICA (13) | CRITICAL |
| Hidden menu open | AUDIO_MENU_OCULTO (25) | LOW |
| Config saved | AUDIO_CONFIG_GUARDADA (64) | LOW |
| Beep confirmation | AUDIO_BEEP (68) | LOW |

### 4.6 Memory Impact

| Item | Size | Notes |
|------|------|-------|
| `DFRobotDFPlayerMini*` pointer | 4 bytes | Allocated once in `audio_init()` |
| `HardwareSerial*` pointer | 4 bytes | UART1, allocated once in `audio_init()` |
| Audio queue (`Item[8]`) | 24 bytes | 8 × {uint16_t track, uint8_t prio} |
| Queue indices | 12 bytes | head, tail, count |
| Audio state flags | 4 bytes | initialized, playing, retry_timer |
| **Total audio memory** | **~48 bytes** (POD) + **~80 bytes** (heap, one-time DFPlayer object) |

> **Note:** The `DFRobotDFPlayerMini` object (~80 bytes) is heap-allocated once during `audio_init()` and never freed. This is acceptable because it is a permanent allocation with zero churn.

### 4.7 Timing Impact

| Operation | Time | Frequency |
|-----------|------|-----------|
| Queue check + pop | ~0.01 ms | Every tick |
| `dfPlayer->play(track)` | ~5 ms (UART TX) | When track queued |
| `dfPlayer->available()` check | ~0.1 ms | Every tick |
| **Typical tick (no playback)** | **~0.1 ms** | |
| **Tick with playback** | **~5 ms** | Occasional |

### 4.8 Watchdog Safety

| Metric | Value |
|--------|-------|
| Worst-case `audio_update()` time | 5 ms (UART command TX) |
| WDT timeout | 5,000 ms |
| Margin | **1,000×** |
| Can `begin()` block? | Yes — but only in `setup()`, with 1 s timeout, before WDT is critical |
| Can `play()` block? | No — 9600 baud UART TX of 10-byte command = ~10 ms max |
| Can queue processing block? | **No** — fixed-size array, O(1) push/pop |

---

## Phase 5 — Obstacle Sensor Display

### 5.1 Obstacle Data Source

The obstacle detection stack runs on the ESP32-S3. The TOFSense-M S LiDAR sensor is connected via UART0 (GPIO 44 RX, native). The ESP32 processes the 8×8 distance matrix, computes zones, and:

1. **Displays** obstacle data on the HMI (distance, zone, health)
2. **Transmits** processed data to STM32 via CAN (0x208/0x209) for safety backstop

### 5.2 Obstacle Module Design

| Component | Responsibility |
|-----------|---------------|
| `obstacle` module | UART0 driver, TOFSense frame parser, 8×8 matrix processing, zone computation |
| `obstacle_can` module | Encodes processed data into CAN frames 0x208/0x209 |
| `ui_obstacle` widget | Draws distance bar and zone indicator on drive screen |
| `overlay` (obstacle) | Shows proximity warning overlay when distance < CAUTION threshold |

### 5.3 Obstacle State (POD)

```
struct ObstacleState {
    // UART frame buffer
    uint8_t  frame_buf[400];       // TOFSense-M S frame (400 bytes)
    uint16_t frame_pos;            // Current parse position
    
    // Processed data
    uint16_t min_distance_mm;      // Minimum distance across 8×8 matrix
    uint8_t  zone;                 // Proximity zone (0=safe..3=critical)
    uint8_t  confidence;           // Measurement confidence (0–100)
    bool     sensor_healthy;       // Sensor health flag
    uint8_t  rolling_counter;      // CAN TX rolling counter
    
    // Timing
    uint32_t last_update_ms;       // Last successful measurement
    uint32_t last_can_tx_ms;       // Last CAN 0x208 transmission
    uint8_t  error_count;          // Consecutive parse errors
    
    // Display cache (for dirty-flag redraw)
    uint16_t displayed_distance;   // Last drawn distance value
    uint8_t  displayed_zone;       // Last drawn zone
};
// Total: ~464 bytes (frame_buf dominates)
```

### 5.4 Proximity Zones

| Zone | Distance | Color | Audio | Safety Action (STM32) |
|------|----------|-------|-------|----------------------|
| SAFE (0) | > 1000 mm | Green | None | No speed limit |
| CAUTION (1) | 500–1000 mm | Yellow | None | 70% power limit |
| WARNING (2) | 200–500 mm | Orange | AUDIO_OBSTACULO (54) | 30% power limit |
| CRITICAL (3) | < 200 mm | Red (flashing) | AUDIO_EMERGENCIA (31) | SAFE state (emergency stop) |

### 5.5 Display Integration

The obstacle distance indicator is rendered on the Drive screen as a dedicated region:

```
Drive Screen Layout (updated):

    ┌──────────────────────────────────┐ 0
    │  [4×4] [4×2] [360]    [BAT 12V] │ 40    ← Mode icons + battery
    ├──────────────────────────────────┤
    │                                  │
    │         ┌──────────┐             │
    │  FL     │          │     FR      │
    │  [T%]   │   CAR    │   [T%]     │
    │  [T°C]  │  BODY    │   [T°C]    │ 240   ← Car top-view + wheels
    │  RL     │          │     RR      │
    │  [T%]   │          │   [T%]     │
    │  [T°C]  └──────────┘   [T°C]    │
    │                                  │
    ├──────────────────────────────────┤
    │  SPEED: 0.0 km/h    OBS: 1.2m ● │ 320   ← Speed + obstacle distance
    ├──────────────────────────────────┤
    │                                  │
    │  ┌───────────────────────────┐   │
    │  │ PEDAL BAR (0-100%)       │   │ 380   ← Pedal/throttle bar
    │  └───────────────────────────┘   │
    │                                  │
    │  [P] [R] [N] [D1] [D2]          │ 440   ← Gear selector
    │                                  │
    │  Steering: 0.0° ← → ►           │ 480   ← Steering angle
    └──────────────────────────────────┘
```

- The obstacle indicator shows: distance value (m), color-coded dot (zone color), health icon
- When zone = CRITICAL, a full-width red overlay bar appears (same overlay system as heartbeat loss)
- Obstacle display uses dirty-flag redraw: only redraws when `min_distance_mm` or `zone` changes

### 5.6 Obstacle Overlay Integration

The overlay system (used for heartbeat loss, degraded mode, and fault flags) now also handles obstacle proximity warnings:

| Overlay | Trigger | Display | Priority |
|---------|---------|---------|----------|
| Heartbeat loss | No 0x001 for >250 ms | Red bar: "STM32 HEARTBEAT LOST" | Highest |
| Obstacle CRITICAL | zone == 3, distance < 200 mm | Red bar: "OBSTACLE ⚠ 0.15m" | High |
| Fault flags | fault_flags ≠ 0 | Amber bar with fault description | Medium |
| Degraded mode | system_state == 3 | Amber bar: "DEGRADED MODE" | Low |

Overlays are drawn in priority order. Only the highest-priority active overlay is displayed to avoid visual clutter.

### 5.7 Audio Warning Integration

Obstacle proximity triggers audio warnings through the audio queue (Phase 4):

| Zone Transition | Audio Action | Rate Limit |
|----------------|-------------|------------|
| Any → CAUTION | No audio | — |
| Any → WARNING | `audio_queue_push(AUDIO_OBSTACULO, HIGH)` | Once per 3 seconds |
| Any → CRITICAL | `audio_queue_push(AUDIO_EMERGENCIA, CRITICAL)` | Once per 2 seconds |
| CRITICAL → SAFE | `audio_queue_push(AUDIO_BEEP, LOW)` | Once |

Rate limiting prevents continuous audio spam when the obstacle distance fluctuates near a threshold boundary.

### 5.8 Timing Impact

| Operation | Time | Frequency |
|-----------|------|-----------|
| UART0 read (non-blocking) | ~0.3 ms | Every tick |
| Frame parse (400 bytes) | ~0.2 ms | Every 66 ms (15 Hz) |
| 8×8 matrix → min distance | ~0.05 ms | Every 66 ms |
| CAN TX (0x208 + 0x209) | ~0.1 ms | Every 66/100 ms |
| **Typical tick** | **~0.3 ms** | |
| **Frame parse tick** | **~0.65 ms** | Every 66 ms |

### 5.9 Watchdog Safety

| Metric | Value |
|--------|-------|
| Worst-case `obstacle_tick()` time | 0.65 ms |
| WDT timeout | 5,000 ms |
| Margin | **7,692×** |
| Can UART read block? | **No** — `Serial.available()` checked first, reads only available bytes |
| Can frame parse block? | **No** — processes only buffered data, no waiting |

---

## Phase 6 — Hidden Diagnostics Menu

### 6.1 Entry Conditions

The diagnostics menu is accessible **only** when ALL of the following are true:

| Condition | Check |
|-----------|-------|
| System state is STANDBY (1) or ERROR (5) | `vehicle_data.system_state == 1 OR 5` |
| All wheel speeds are zero | `vehicle_data.wheel_speed[0..3] == 0` |
| No CAN timeout active | `heartbeat.stm32_alive == true` |
| Multi-step gesture completed | 3 taps on specific screen region within 2 seconds |

### 6.2 Menu Structure

```
Hidden Diagnostics Menu
├── [1] Module Status
│   ├── CAN Bus: OK / ERROR
│   ├── Display: OK / ERROR
│   ├── Heartbeat TX: OK / TIMEOUT
│   ├── Heartbeat RX: OK / LOST
│   ├── ACK Tracker: IDLE / PENDING / TIMEOUT
│   ├── LED Controller: OK / ERROR (front + rear init status)
│   ├── Audio (DFPlayer): OK / ERROR / NOT CONNECTED
│   └── Obstacle Sensor: OK / UNHEALTHY / TIMEOUT
│
├── [2] Sensor Status
│   ├── Wheel Speed FL/FR/RL/RR: value km/h
│   ├── Motor Current FL/FR/RL/RR: value A
│   ├── Temperature FL/FR/RL/RR/AMB: value °C
│   ├── Steering Angle: value° (calibrated: Y/N)
│   ├── Battery: voltage V / current A
│   ├── Pedal Position: value %
│   └── Obstacle Distance: value mm (zone, health, confidence)
│
├── [3] CAN Statistics
│   ├── RX Frames Total: count
│   ├── TX Frames Total: count
│   ├── RX Errors: count
│   ├── TX Errors: count
│   ├── Last RX ID: 0xNNN
│   ├── Last TX ID: 0xNNN
│   ├── Heartbeat RX interval: ms (avg/min/max)
│   ├── Obstacle TX count: count (0x208 + 0x209)
│   └── Bus Status: ACTIVE / BUS-OFF / ERROR-PASSIVE
│
├── [4] Fault Display
│   ├── Active Faults: bitmask + descriptions
│   │   ├── Bit 0: CAN Timeout
│   │   ├── Bit 1: Overtemperature
│   │   ├── Bit 2: Overcurrent
│   │   ├── Bit 3: Encoder Fault
│   │   ├── Bit 4: Wheel Sensor Fault
│   │   ├── Bit 5: ABS Active
│   │   ├── Bit 6: TCS Active
│   │   └── Bit 7: Centering Fault
│   ├── Last Error Code: value
│   ├── Last Error Subsystem: name
│   └── Error History: last 8 entries (ring buffer)
│
├── [5] Mode Switching
│   ├── Current Mode: 4×4 / 4×2 / Tank Turn
│   ├── Current Gear: P / R / N / D1 / D2
│   ├── [Send Mode Change] — only if speed < 1 km/h
│   └── [Send Service Command] — 0x110 frames
│
├── [6] LED Configuration
│   ├── Front LEDs: ON / OFF (toggle)
│   ├── Rear LEDs: ON / OFF (toggle)
│   ├── Global Brightness: 0–255 (slider)
│   ├── KITT Speed: 20–100 ms/step (slider)
│   ├── Turn Signal Rate: 200–1000 ms (slider)
│   ├── Current Front Mode: name
│   └── Current Rear Mode: name
│
├── [7] Audio Configuration
│   ├── Audio Enabled: ON / OFF (toggle)
│   ├── DFPlayer Status: OK / ERROR
│   ├── Queue Depth: count / 8
│   ├── Last Track Played: number
│   ├── [Test Beep] — plays AUDIO_BEEP (68)
│   └── Obstacle Audio Alerts: ON / OFF (toggle)
│
├── [8] Obstacle Configuration
│   ├── Sensor Status: HEALTHY / UNHEALTHY / TIMEOUT
│   ├── Current Distance: value mm
│   ├── Current Zone: SAFE / CAUTION / WARNING / CRITICAL
│   ├── Confidence: value %
│   ├── Error Count: value
│   ├── Visual Alerts: ON / OFF (toggle)
│   ├── Audio Alerts: ON / OFF (toggle)
│   └── [Reset Errors] — clears error counter
│
└── [9] System Info
    ├── Firmware Version: string
    ├── Uptime: HH:MM:SS
    ├── Free Heap: value bytes
    ├── Frame Time: avg/min/max ms
    ├── FPS: current
    ├── LED Memory: value bytes
    ├── Obstacle Buffer: value bytes
    └── CAN Bus Speed: 500 kbps
```

### 6.3 Implementation Constraints

| Constraint | Implementation |
|------------|---------------|
| No blocking loops | Menu renders within normal `draw()` cycle at 20 FPS |
| No long delays | Navigation is immediate; no animations or transitions |
| No dynamic allocation | Menu state uses a fixed POD struct (~96 bytes) |
| No recursion | Menu pages are flat; selected via index, not tree traversal |
| Frame-limited | Menu draws are subject to same 20 FPS limiter |
| Exit on state change | Menu auto-closes if `system_state` changes to ACTIVE/DEGRADED/SAFE |
| No parameter modification | Menu is read-only except for mode switching and service commands |
| Ring buffer for errors | Fixed 8-entry array with wrap-around index |

### 6.4 Menu State Machine

```
CLOSED ──(gesture)──> PAGE_SELECT ──(tap 1)──> MODULE_STATUS
                           │                        │
                           ├──(tap 2)──> SENSOR_STATUS
                           │                        │
                           ├──(tap 3)──> CAN_STATS  │
                           │                   ┌────┘
                           ├──(tap 4)──> FAULTS│
                           │                   │
                           ├──(tap 5)──> MODE_SWITCH
                           │                   │
                           ├──(tap 6)──> LED_CONFIG
                           │                   │
                           ├──(tap 7)──> AUDIO_CONFIG
                           │                   │
                           ├──(tap 8)──> OBSTACLE_CONFIG
                           │                   │
                           └──(tap 9)──> SYS_INFO
                                               │
         All pages ──(back gesture)──> PAGE_SELECT
         All pages ──(state change)──> CLOSED
```

---

## Phase 7 — Deliverables

### 7.1 Clean File Structure

```
esp32/
├── platformio.ini              # Build config (NO PSRAM flags, NO BOARD_HAS_PSRAM)
├── include/
│   └── can_ids.h               # CAN protocol definitions (FROZEN — no changes)
└── src/
    ├── main.cpp                 # setup() + loop() — NO global objects
    ├── app.h                    # app_init(), app_tick() declarations
    ├── app.cpp                  # Initialization chain + main tick
    │
    ├── can/
    │   ├── can_bus.h            # TWAI driver init, raw frame TX/RX
    │   ├── can_bus.cpp
    │   ├── can_rx.h             # Frame decoder → vehicle_data
    │   ├── can_rx.cpp
    │   ├── can_tx.h             # Heartbeat, command, service TX
    │   └── can_tx.cpp
    │
    ├── data/
    │   ├── vehicle_data.h       # POD struct: all telemetry fields
    │   └── vehicle_data.cpp     # Single global instance (POD, zero-init)
    │
    ├── hmi/
    │   ├── screen_mgr.h         # Screen state machine + dispatch table
    │   ├── screen_mgr.cpp
    │   ├── screen_boot.h        # Boot screen update/draw
    │   ├── screen_boot.cpp
    │   ├── screen_standby.h
    │   ├── screen_standby.cpp
    │   ├── screen_drive.h       # Main dashboard (dirty-flag partial redraw)
    │   ├── screen_drive.cpp
    │   ├── screen_safe.h
    │   ├── screen_safe.cpp
    │   ├── screen_error.h
    │   └── screen_error.cpp
    │
    ├── hmi/overlay/
    │   ├── overlay.h            # Heartbeat loss, fault, degraded, obstacle overlays
    │   └── overlay.cpp
    │
    ├── hmi/diag/
    │   ├── diag_menu.h          # Hidden diagnostics menu (9 pages)
    │   └── diag_menu.cpp
    │
    ├── hmi/widgets/
    │   ├── ui_car.h             # Top-view car renderer
    │   ├── ui_car.cpp
    │   ├── ui_gauge.h           # Speed, current, temp gauges
    │   ├── ui_gauge.cpp
    │   ├── ui_bar.h             # Pedal bar, battery bar
    │   ├── ui_bar.cpp
    │   ├── ui_icons.h           # Mode icons (4×4, 4×2, 360°)
    │   ├── ui_icons.cpp
    │   ├── ui_obstacle.h        # Obstacle distance bar + zone indicator
    │   └── ui_obstacle.cpp
    │
    ├── led/
    │   ├── led_ctrl.h           # Non-blocking LED controller (front + rear)
    │   └── led_ctrl.cpp         # Step-based animations, FastLED.show()
    │
    ├── audio/
    │   ├── audio_ctrl.h         # DFPlayer non-blocking driver + priority queue
    │   └── audio_ctrl.cpp       # Queue management, event→track mapping
    │
    ├── obstacle/
    │   ├── obstacle.h           # TOFSense UART driver + 8×8 matrix processing
    │   ├── obstacle.cpp
    │   ├── obstacle_can.h       # CAN TX encoding (0x208/0x209)
    │   └── obstacle_can.cpp
    │
    ├── sys/
    │   ├── heartbeat.h          # Heartbeat TX + loss detection
    │   ├── heartbeat.cpp
    │   ├── ack_tracker.h        # Non-blocking ACK with timeout
    │   ├── ack_tracker.cpp
    │   ├── frame_limit.h        # 20 FPS frame timing (header-only)
    │   └── tft_direct.h         # TFT_eSPI wrapper (header-only, no sprites)
    │
    └── util/
        └── ui_common.h          # Color constants, layout constants, snprintf helpers
```

**File count:** 44 files (vs. original: 90+ headers + sources)  
**Directories:** 10 logical groups (vs. original: 15 directories)  
**Dependencies:** TFT_eSPI, ESP32-TWAI-CAN, FastLED, DFRobotDFPlayerMini (4 libraries vs. original 9)

### 7.2 Initialization Order Diagram

All initialization happens in `setup()` → `app_init()`, in strict sequential order:

```
setup()
  │
  ├─ 1. Serial.begin(115200)          [UART0 ready — also used by TOFSense RX]
  │     └─ No delay, no while(!Serial)
  │
  ├─ 2. tft_direct_init()             [Display hardware]
  │     ├─ pinMode(TFT_RST, OUTPUT)
  │     ├─ Reset pulse: LOW → 10ms → HIGH → 50ms
  │     ├─ tft.init()
  │     ├─ tft.setRotation(0)         [Portrait 320×480]
  │     ├─ tft.fillScreen(BG_DARK)
  │     └─ tft.setTextFont(2)
  │
  ├─ 3. can_bus_init()                [CAN hardware]
  │     ├─ TWAI driver install (500 kbps, GPIO4 TX, GPIO5 RX)
  │     ├─ TWAI driver start
  │     └─ Verify bus status
  │
  ├─ 4. vehicle_data_init()           [Data store]
  │     └─ memset(&vd, 0, sizeof(vd)) [Zero all fields]
  │
  ├─ 5. heartbeat_init()              [Timing]
  │     └─ Record millis() as t0
  │
  ├─ 6. ack_tracker_init()            [ACK state]
  │     └─ state = IDLE
  │
  ├─ 7. led_init()                    [LED hardware — FastLED]
  │     ├─ FastLED.addLeds<WS2812B, 47>(front_leds, 28)
  │     ├─ FastLED.addLeds<WS2812B, 43>(rear_leds, 16)
  │     ├─ FastLED.setBrightness(128)
  │     ├─ FastLED.clear()
  │     └─ FastLED.show()             [All LEDs off]
  │
  ├─ 8. audio_init()                  [Audio hardware — DFPlayer]
  │     ├─ Serial1 = new HardwareSerial(1)
  │     ├─ Serial1->begin(9600, SERIAL_8N1, GPIO20, GPIO19)
  │     ├─ dfPlayer = new DFRobotDFPlayerMini()
  │     ├─ dfPlayer->begin(*Serial1) with 1s timeout
  │     ├─ If timeout → audio_ok = false (non-fatal, retry in loop)
  │     └─ Initialize audio queue (head=tail=count=0)
  │
  ├─ 9. obstacle_init()               [Obstacle sensor — TOFSense UART0]
  │     ├─ Configure UART0 for TOFSense (921600 baud, GPIO44 RX)
  │     ├─ Zero obstacle state struct
  │     └─ sensor_healthy = false (until first valid frame)
  │
  ├─ 10. screen_mgr_init()            [UI state machine]
  │     ├─ current_screen = BOOT
  │     ├─ Register screen function pointers
  │     └─ Call screen_boot_enter()
  │
  ├─ 11. diag_menu_init()             [Hidden menu]
  │     └─ state = CLOSED, all counters = 0
  │
  └─ 12. frame_limit_init()           [FPS limiter]
        └─ last_frame_ms = millis()

loop()
  └─ app_tick()
       ├─ can_bus_poll()              [Always: read all pending frames]
       ├─ can_rx_process()            [Always: decode frames → vehicle_data]
       ├─ heartbeat_tick()            [Always: TX heartbeat, check RX timeout]
       ├─ ack_tracker_tick()          [Always: check ACK timeout]
       ├─ obstacle_tick()             [Always: read UART, parse, CAN TX]
       ├─ led_update()               [Always: compute colors, FastLED.show()]
       ├─ audio_update()             [Always: check queue, play if ready]
       ├─ screen_mgr_update()         [Always: check state transitions]
       ├─ overlay_update()            [Always: check overlay conditions]
       ├─ diag_menu_update()          [Always: check gesture, update menu data]
       │
       └─ if (frame_limit_ready())    [20 FPS gate]
            ├─ screen_mgr_draw()      [Draw current screen]
            ├─ overlay_draw()         [Draw active overlays on top]
            └─ diag_menu_draw()       [Draw menu if open (replaces screen)]
```

**Key properties:**
- Total init time: <1.5 s (DFPlayer begin() is the slowest at ~1 s; non-fatal if it fails)
- Init order: UART → Display → CAN → Data → Peripherals (LED, Audio, Obstacle) → UI (dependency-safe)
- No step depends on a later step
- No step can fail silently (each step logs to Serial on failure)
- LED and audio failures are non-fatal: system continues without them
- Obstacle sensor failure is non-fatal: no obstacle data sent to STM32 (fail-safe: STM32 applies its own timeout)

### 7.3 Memory Model

#### Stack Usage

| Context | Stack Size | Usage |
|---------|-----------|-------|
| Arduino `loop()` task | 8,192 bytes (default) | ~1,800 bytes peak (draw + LED compute + obstacle parse) |
| Interrupt handlers | 2,048 bytes | TWAI RX interrupt: ~256 bytes |
| **Headroom** | **~6,300 bytes** | **77% margin** |

**Stack budget per tick (worst case):**
- `snprintf` format buffer: 64 bytes × 4 calls = 256 bytes
- Local variables (coords, colors): ~128 bytes
- Function call frames (max 4 levels deep): ~256 bytes
- LED color computation temporaries: ~64 bytes
- Obstacle frame parsing locals: ~32 bytes
- **Total peak: ~736 bytes per tick**

#### Heap Usage

| Category | Size | Lifetime |
|----------|------|----------|
| TFT_eSPI object (internal) | ~2,048 bytes | Allocated in `tft.init()`, permanent |
| TWAI driver buffers | ~1,024 bytes | Allocated in `twai_driver_install()`, permanent |
| Arduino framework overhead | ~8,192 bytes | Permanent |
| DFRobotDFPlayerMini object | ~80 bytes | Allocated once in `audio_init()`, permanent |
| HardwareSerial(1) object | ~256 bytes | Allocated once in `audio_init()`, permanent |
| **Application state (all POD structs)** | **~1,444 bytes** | **Static, zero-initialized** |
| **Total heap usage** | **~13,044 bytes** | **All permanent, no churn** |

#### Static Memory Breakdown (Application POD State)

| Module | Size | Dominant Item |
|--------|------|---------------|
| CAN (bus + rx + tx) | 144 bytes | TX/RX frame buffers |
| Vehicle data | 128 bytes | Telemetry fields |
| Heartbeat + ACK | 32 bytes | Timestamps |
| Screen manager + screens | 128 bytes | Dirty flags, cached values |
| Overlay | 32 bytes | Active overlay state |
| Diagnostics menu | 128 bytes | 9 pages of cached display data |
| UI widgets (car, gauge, bar, icons, obstacle) | 80 bytes | Cached draw values |
| **LED controller** | **224 bytes** | **CRGB[28] + CRGB[16] + animation state** |
| **Audio controller** | **48 bytes** | **Queue[8] + state flags** |
| **Obstacle** | **464 bytes** | **UART frame buffer (400 bytes)** |
| Obstacle CAN TX | 16 bytes | Rolling counter, timestamps |
| Frame limiter + TFT | 12 bytes | Timestamp + pointer |
| **Total** | **~1,444 bytes** | |

#### Available Memory

| Resource | Total | Used | Free | Margin |
|----------|-------|------|------|--------|
| Internal SRAM | 512 KB | ~75 KB (FW + stack + heap) | ~437 KB | 85% |
| PSRAM | 8 MB | **0 bytes** (not used) | 8 MB | 100% |
| Flash | 16 MB | ~2.0 MB (firmware + libs + FastLED + DFPlayer) | ~14 MB | 87% |

**Key difference from original:**
- Original: ~4 MB PSRAM used for sprites + ~200 KB internal SRAM → PSRAM failure = bootloop
- New: 0 bytes PSRAM, ~75 KB internal SRAM → PSRAM irrelevant, boot guaranteed
- New includes LED (224 B), Audio (48 B + 336 B heap), Obstacle (464 B) — all fit comfortably in internal SRAM

#### Heap Fragmentation Analysis

| Risk Factor | Original | New Design |
|-------------|----------|------------|
| Dynamic allocation in `loop()` | Yes (String, sprites) | **Zero** |
| `new`/`delete` patterns | Yes (HUD layers) | **Two permanent `new` calls in `audio_init()` only** |
| PSRAM allocation failure | Fatal | **N/A** |
| Heap growth over time | Yes (fragmentation) | **Zero** (no allocation after init) |
| Time-to-exhaustion | Hours to days | **Never** (no allocation after init) |
| FastLED heap usage | Dynamic in original | **Zero** — static CRGB arrays, no heap |
| DFPlayer heap usage | Global constructor | **Deferred** — pointer-based, allocated in `setup()` |

### 7.4 Watchdog Safety Analysis

#### ESP32-S3 Watchdog Configuration

| Watchdog | Timeout | Feed Method | Risk in Original | Risk in New |
|----------|---------|-------------|-------------------|-------------|
| Task WDT (Core 0) | 5 s | `yield()` or `delay()` | HIGH — FreeRTOS tasks can starve idle task | **NONE** — single-core, `loop()` returns every tick |
| Task WDT (Core 1) | 5 s | `yield()` or `delay()` | HIGH — HUD rendering can exceed 5 s | **NONE** — not used (single-core) |
| Interrupt WDT | 300 ms | Automatic (ISR return) | LOW — ISRs are short | **NONE** — same pattern |
| IWDG (STM32 side) | 500 ms | `IWDG_Feed()` in main loop | N/A (STM32 firmware) | N/A |

#### Worst-Case Frame Time Analysis (Full Vehicle Feature Set)

| Operation | Time (worst case) | Notes |
|-----------|-------------------|-------|
| `can_bus_poll()` + `can_rx_process()` | 0.5 ms | Max 10 frames per poll |
| `heartbeat_tick()` | 0.05 ms | Simple timestamp comparison |
| `ack_tracker_tick()` | 0.05 ms | Simple timeout check |
| `obstacle_tick()` (UART read + parse) | 0.65 ms | 400-byte frame parse at 15 Hz |
| `led_update()` (compute + FastLED.show) | 1.5 ms | 44 LEDs, bit-bang SPI |
| `audio_update()` (queue + UART TX) | 5.0 ms | 9600 baud command (worst case) |
| `screen_mgr_update()` | 0.1 ms | State comparison + dirty flags |
| `overlay_update()` | 0.05 ms | Heartbeat + obstacle timeout check |
| `diag_menu_update()` | 0.1 ms | Gesture detection |
| `screen_mgr_draw()` (full redraw) | 3.0 ms | Direct TFT, 320×480, partial |
| `overlay_draw()` | 0.5 ms | Single rect + text |
| `diag_menu_draw()` | 2.0 ms | Text-heavy, no graphics |
| **Total worst-case tick** | **~13.5 ms** | **370× margin vs 5 s WDT** |

> **Note:** The audio UART TX (5 ms) occurs only when a track is being played,
> not every tick. Typical tick without audio playback is ~8.5 ms.

#### Watchdog Safety Verdict

| Scenario | Time Budget | Actual Time | Margin |
|----------|-------------|-------------|--------|
| Normal frame (partial redraw, no audio) | 50 ms (20 FPS) | ~6.5 ms | 7.7× |
| Full screen transition + audio + LED | 50 ms (20 FPS) | ~13.5 ms | 3.7× |
| CAN burst (30 frames) + obstacle parse | 50 ms (20 FPS) | ~10 ms | 5× |
| Task WDT timeout | 5,000 ms | ~13.5 ms | 370× |
| **Verdict** | | | **SAFE** |

### 7.5 Render Pipeline Diagram

```
                    ┌─────────────────────────────────────┐
                    │            app_tick()                │
                    │         (called every loop)          │
                    └─────────────┬───────────────────────┘
                                  │
                    ┌─────────────▼───────────────────────┐
                    │         UPDATE PHASE                 │
                    │      (runs every tick, ~3 ms)        │
                    │                                      │
                    │  ┌─────────────────────────────┐     │
                    │  │ can_rx_process()             │     │
                    │  │  └─ Decode frames            │     │
                    │  │     └─ Write to vehicle_data │     │
                    │  └─────────────────────────────┘     │
                    │                                      │
                    │  ┌─────────────────────────────┐     │
                    │  │ obstacle_tick()              │     │
                    │  │  ├─ Read UART0 (non-block)  │     │
                    │  │  ├─ Parse TOFSense frame    │     │
                    │  │  ├─ Compute zones           │     │
                    │  │  └─ CAN TX 0x208/0x209      │     │
                    │  └─────────────────────────────┘     │
                    │                                      │
                    │  ┌─────────────────────────────┐     │
                    │  │ led_update()                 │     │
                    │  │  ├─ Compute front colors     │     │
                    │  │  ├─ Compute rear colors      │     │
                    │  │  └─ FastLED.show()           │     │
                    │  └─────────────────────────────┘     │
                    │                                      │
                    │  ┌─────────────────────────────┐     │
                    │  │ audio_update()               │     │
                    │  │  ├─ Check DFPlayer status    │     │
                    │  │  └─ Pop queue → play()       │     │
                    │  └─────────────────────────────┘     │
                    │                                      │
                    │  ┌─────────────────────────────┐     │
                    │  │ screen_mgr_update()          │     │
                    │  │  ├─ Check system_state       │     │
                    │  │  ├─ Detect transitions       │     │
                    │  │  └─ Set dirty flags          │     │
                    │  └─────────────────────────────┘     │
                    │                                      │
                    │  ┌─────────────────────────────┐     │
                    │  │ overlay_update()             │     │
                    │  │  ├─ Heartbeat timeout?       │     │
                    │  │  ├─ Obstacle CRITICAL?       │     │
                    │  │  ├─ Fault flags changed?     │     │
                    │  │  └─ Degraded state?          │     │
                    │  └─────────────────────────────┘     │
                    └─────────────┬───────────────────────┘
                                  │
                    ┌─────────────▼───────────────────────┐
                    │       FRAME GATE (20 FPS)            │
                    │   if (millis() - last_frame < 50)    │
                    │       return;  // skip draw          │
                    └─────────────┬───────────────────────┘
                                  │ (50 ms elapsed)
                    ┌─────────────▼───────────────────────┐
                    │          DRAW PHASE                   │
                    │      (runs at 20 FPS, ~3 ms)         │
                    │                                      │
                    │  ┌─────────────────────────────┐     │
                    │  │ Is diag_menu open?           │     │
                    │  │  YES → diag_menu_draw()      │     │
                    │  │  NO  → continue              │     │
                    │  └──────────┬──────────────────┘     │
                    │             │ (menu closed)          │
                    │  ┌──────────▼──────────────────┐     │
                    │  │ screen_draw() dispatch       │     │
                    │  │  ├─ BOOT:    screen_boot_draw│     │
                    │  │  ├─ STANDBY: screen_standby_ │     │
                    │  │  ├─ ACTIVE:  screen_drive_   │     │
                    │  │  ├─ DEGRADED:screen_drive_   │     │
                    │  │  ├─ SAFE:    screen_safe_    │     │
                    │  │  └─ ERROR:   screen_error_   │     │
                    │  └──────────┬──────────────────┘     │
                    │             │                        │
                    │  ┌──────────▼──────────────────┐     │
                    │  │ overlay_draw()               │     │
                    │  │  ├─ Heartbeat loss: red bar  │     │
                    │  │  ├─ Obstacle CRITICAL: red   │     │
                    │  │  ├─ Fault flags: amber bar   │     │
                    │  │  └─ Degraded: amber banner   │     │
                    │  └─────────────────────────────┘     │
                    │                                      │
                    │  last_frame = millis();               │
                    └──────────────────────────────────────┘

    Direct TFT Drawing (no sprites, no framebuffer):
    ┌────────────────────────────────────────────────┐
    │              TFT_eSPI Hardware                  │
    │                                                │
    │  tft.fillRect()    ← Clear dirty region        │
    │  tft.setTextColor()← Set color                 │
    │  tft.drawString()  ← Render text               │
    │  tft.drawLine()    ← Geometric elements        │
    │  tft.fillCircle()  ← Indicators                │
    │  tft.drawRect()    ← Borders, frames           │
    │                                                │
    │  SPI Bus: 40 MHz, 320×480 pixels               │
    │  Partial redraw: only dirty regions             │
    │  No double-buffering, no sprites               │
    └────────────────────────────────────────────────┘

    LED Output (parallel to TFT, separate GPIO):
    ┌────────────────────────────────────────────────┐
    │              FastLED Hardware                   │
    │                                                │
    │  GPIO 47 → 28× WS2812B (front)                │
    │  GPIO 43 → 16× WS2812B (rear)                 │
    │                                                │
    │  Bit-bang transfer: ~1.1 ms for 44 LEDs        │
    │  Step-based animations, no blocking loops      │
    └────────────────────────────────────────────────┘

    Audio Output (UART, independent):
    ┌────────────────────────────────────────────────┐
    │            DFPlayer Mini (UART1)               │
    │                                                │
    │  GPIO 19 TX → DFPlayer RX (9600 baud)          │
    │  GPIO 20 RX ← DFPlayer TX (status)             │
    │                                                │
    │  Queue-based: push(track, priority)            │
    │  Pop one per tick if DFPlayer ready             │
    │  Non-blocking: ~5 ms per play command           │
    └────────────────────────────────────────────────┘
```

#### Drive Screen Layout (320×480 Portrait) — Full Feature Set

```
    ┌──────────────────────────────────┐ 0
    │  [4×4] [4×2] [360]    [BAT 12V] │ 40    ← Mode icons + battery
    ├──────────────────────────────────┤
    │                                  │
    │         ┌──────────┐             │
    │  FL     │          │     FR      │
    │  [T%]   │   CAR    │   [T%]     │
    │  [T°C]  │  BODY    │   [T°C]    │ 240   ← Car top-view + wheels
    │  RL     │          │     RR      │
    │  [T%]   │          │   [T%]     │
    │  [T°C]  └──────────┘   [T°C]    │
    │                                  │
    ├──────────────────────────────────┤
    │  SPEED: 0.0 km/h    OBS: 1.2m ● │ 320   ← Speed + obstacle distance
    ├──────────────────────────────────┤
    │                                  │
    │  ┌───────────────────────────┐   │
    │  │ PEDAL BAR (0-100%)       │   │ 380   ← Pedal/throttle bar
    │  └───────────────────────────┘   │
    │                                  │
    │  [P] [R] [N] [D1] [D2]          │ 440   ← Gear selector
    │                                  │
    │  Steering: 0.0° ← → ►           │ 480   ← Steering angle
    └──────────────────────────────────┘

    Overlay layers (drawn on top when active):
    ┌──────────────────────────────────┐
    │ ████ STM32 HEARTBEAT LOST ████  │ ← Red bar (highest priority)
    ├──────────────────────────────────┤
    │ ████ OBSTACLE ⚠ 0.15m █████████ │ ← Red bar (high priority)
    ├──────────────────────────────────┤
    │ ████ DEGRADED MODE █████████████ │ ← Amber bar (low priority)
    └──────────────────────────────────┘
```

**Screen elements confirmed present (matching original FULL-FIRMWARE layout):**
- ✅ Top row mode icons (4×4 / 4×2 / 360°)
- ✅ Battery indicator (voltage + percentage)
- ✅ Central top-view car diagram
- ✅ 4 wheels with torque % and temperature
- ✅ Steering indicator (angle + direction arrow)
- ✅ Gear selector (P R N D1 D2)
- ✅ Pedal bar (0–100%)
- ✅ Obstacle distance indicator (value + color-coded zone dot)
- ✅ Degraded mode overlay (amber)
- ✅ SAFE mode overlay
- ✅ Heartbeat loss overlay (red)
- ✅ Speed display

### 7.6 Risk Comparison Table

| Risk Category | Original (FULL-FIRMWARE) | New Design (Full Feature) | Improvement |
|---------------|--------------------------|---------------------------|-------------|
| **Boot reliability** | CRITICAL — Bootloop before `setup()` completes | SAFE — No pre-`setup()` code, no PSRAM dependency | Eliminates root cause |
| **PSRAM dependency** | FATAL — OPI PSRAM misconfiguration causes bootloop | NONE — Zero PSRAM usage, internal SRAM only | 100% risk removal |
| **Memory usage** | ~4 MB PSRAM + ~200 KB SRAM | ~75 KB SRAM total, 0 PSRAM | 98% reduction |
| **Heap fragmentation** | HIGH — Dynamic allocation in `loop()`, String usage | ZERO — All allocations are static/permanent (except 2 init-time `new`) | Eliminates risk |
| **Sprite exhaustion** | HIGH — 300 KB+ per sprite, multiple sprites | ZERO — No sprites used | Eliminates risk |
| **Static init order** | CRITICAL — 7+ global singletons with cross-dependencies | SAFE — Zero global objects with constructors | Eliminates risk |
| **Watchdog starvation** | HIGH — 15+ seconds of blocking delays possible | SAFE — Max 13.5 ms per tick, 370× margin | 370× improvement |
| **Stack overflow** | MEDIUM — 6+ call levels in render chain | LOW — Max 4 call levels, ~736 bytes peak | 3× stack margin improvement |
| **Multi-core races** | HIGH — 5 FreeRTOS tasks across 2 cores with shared data | NONE — Single-core, single-task execution | Eliminates risk |
| **Render pipeline depth** | 6 levels (Manager→Compositor→Layer→Engine→SafeDraw→TFT) | 3 levels (ScreenMgr→ScreenDraw→TFT) | 50% reduction |
| **Code complexity** | 90+ files, 15 directories, 9 dependencies | 44 files, 10 directories, 4 dependencies | 51% file reduction |
| **Boot time** | 3-5 seconds (logo, delays, PSRAM init, FreeRTOS task creation) | <1.5 s (DFPlayer init is slowest; non-fatal if it fails) | 2-3× faster |
| **Recovery from failure** | Complex (BootGuard, NVS counters, safe mode, restart loops) | Simple (fail → log → continue without feature) | Dramatic simplification |
| **Bootloop fix attempts** | 30+ documents, none successful | N/A — Architecture prevents bootloops by design | Architectural prevention |
| **Hidden menu** | Present but depends on full HUD stack | Independent module, works in any state, includes LED/audio/obstacle config | Decoupled + extended |
| **LED subsystem** | Present — global FastLED, animation loops | Present — static arrays, step-based non-blocking animations | Stability preserved |
| **Audio subsystem** | Present — global DFPlayer, blocking serial | Present — pointer-based lazy init, priority queue | Stability preserved |
| **Obstacle display** | Present — depends on HUD compositor | Present — direct TFT overlay, CAN-fed data | Stability preserved |
| **CAN contract** | Same (v1.3) | Same (v1.3) — FROZEN, no changes | Compatible |
| **Display quality** | Full sprite compositing with shadows | Direct drawing with partial redraw | Minor visual reduction, major stability gain |
| **FPS capability** | 30 FPS (with PSRAM sprites) | 20 FPS (direct drawing, sufficient) | Adequate for HMI |
| **Feature parity** | Full vehicle features | **Full vehicle features** (LED + audio + obstacle + HMI) | **No feature reduction** |

#### Risk Summary

| Metric | Original | New | Delta |
|--------|----------|-----|-------|
| Critical risks | 4 | 0 | −4 |
| High risks | 3 | 0 | −3 |
| Medium risks | 1 | 1 (stack, mitigated) | 0 |
| Low risks | 0 | 0 | 0 |
| Features removed | 0 | **0** | **No reduction** |
| **Total risk score** | **HIGH/CRITICAL** | **LOW** | **Major reduction** |

---

## Phase 8 — Bootloop Prevention Proof

This section demonstrates that **no bootloop condition can occur** in the new architecture, even with full vehicle features (LED + Audio + Obstacle).

### 8.1 Bootloop Root Cause Elimination

| Original Bootloop Cause | How It Is Eliminated |
|--------------------------|---------------------|
| **Global object constructors** (7+ singletons) | Zero global objects with constructors. All state is POD (zero-initialized). `DFRobotDFPlayerMini` and `HardwareSerial` are allocated via pointers in `audio_init()` during `setup()`, not at static init time. |
| **PSRAM OPI eFuse misconfiguration** | `BOARD_HAS_PSRAM` build flag is removed. No `psramInit()` call. No `ps_malloc()`. PSRAM bus is completely ignored. |
| **TFT_eSprite allocation before PSRAM ready** | No sprites used. TFT_eSPI used for direct drawing only. No memory allocation in TFT path. |
| **FreeRTOS task race conditions** | No FreeRTOS tasks created. Single-core Arduino `loop()` only. No mutexes, no shared data. |
| **Blocking delays in setup()** | No `while(!Serial)`. TFT reset is 60 ms total. DFPlayer `begin()` has 1 s timeout (non-fatal). No logo delay. |
| **BootGuard NVS operations** | No NVS operations. No boot counter. No reset markers. No safe mode. |
| **Multi-core task startup order** | Not applicable — single core only. |

### 8.2 Subsystem Failure Modes (Non-Fatal)

| Subsystem | Failure Mode | System Behavior |
|-----------|-------------|-----------------|
| **LED (FastLED)** | `FastLED.addLeds()` fails | LEDs stay off; `led_ok = false`; log error; system continues |
| **Audio (DFPlayer)** | `dfPlayer->begin()` times out | No audio; `audio_ok = false`; retry every 5 s; system continues |
| **Obstacle (TOFSense)** | No UART data within 500 ms | `sensor_healthy = false`; no CAN TX 0x208/0x209; STM32 applies its own timeout; system continues |
| **TFT Display** | `tft.init()` fails | Black screen; log error; CAN/LED/audio still functional |
| **CAN Bus** | TWAI driver install fails | No telemetry; heartbeat not sent; STM32 enters SAFE state via its own timeout |

**Key property:** No single subsystem failure causes a bootloop or system halt. Each failure is logged and the remaining subsystems continue operating.

### 8.3 Pre-setup() Execution Analysis

The only code that executes before `setup()` is:
1. ESP32-S3 ROM bootloader (hardware, cannot be changed)
2. Arduino framework initialization (stack, heap, scheduler — no PSRAM dependency)
3. Global variable zero-initialization (all POD structs, no constructors)

**There are zero C++ constructors that run before `setup()`.** This is the single most important architectural guarantee.

### 8.4 Memory Safety Proof

| Property | Guarantee |
|----------|-----------|
| No heap allocation after `setup()` | All allocations happen in init functions; `loop()` uses only stack and static data |
| No PSRAM access | `BOARD_HAS_PSRAM` flag removed; PSRAM bus never initialized |
| Bounded stack usage | Max 4 call levels, ~736 bytes peak, vs. 8,192 byte stack = 90% margin |
| No recursion | All algorithms are iterative (LED animations, menu navigation, frame parsing) |
| No `String` class | All text formatting uses `snprintf()` with stack-allocated `char[]` buffers |

---

## Phase 9 — Integration Plan

### 9.1 Phased Implementation Schedule

Each phase produces a **bootable, testable firmware** with progressively more features. Phases are ordered by dependency and risk.

| Phase | Week | Modules | Test Criteria |
|-------|------|---------|---------------|
| **A: Core CAN** | 1 | `main.cpp`, `app`, `can_bus`, `can_rx`, `can_tx`, `vehicle_data`, `heartbeat`, `ack_tracker` | CAN heartbeat TX/RX working; serial log shows decoded telemetry |
| **B: Basic Display** | 2 | `tft_direct`, `screen_mgr`, `screen_boot`, `screen_standby`, `frame_limit` | TFT shows boot screen → standby on first heartbeat; 20 FPS confirmed |
| **C: Drive Dashboard** | 3 | `screen_drive`, `ui_car`, `ui_gauge`, `ui_bar`, `ui_icons` | Full drive screen with live CAN data; dirty-flag partial redraw verified |
| **D: Fault Handling** | 4 | `overlay`, `screen_safe`, `screen_error` | Heartbeat loss overlay; degraded overlay; SAFE/ERROR screens |
| **E: LED Subsystem** | 5 | `led_ctrl` | Front KITT animation in STANDBY; throttle-reactive colors in ACTIVE; rear position/brake/turn |
| **F: Audio Subsystem** | 5 | `audio_ctrl` | DFPlayer init; queue push/pop; gear change plays correct track; obstacle beep |
| **G: Obstacle System** | 6 | `obstacle`, `obstacle_can`, `ui_obstacle` | UART frame parse; CAN 0x208/0x209 TX; distance displayed on drive screen; overlay on CRITICAL |
| **H: Diagnostics Menu** | 7 | `diag_menu` | 9-page menu navigable; LED/audio/obstacle config pages functional |
| **I: Integration Test** | 8 | All modules | Full system test with STM32 hardware; all screens, overlays, LED modes, audio triggers, obstacle display verified |

### 9.2 Safety Gates Between Phases

Each phase must pass these criteria before proceeding:

| Gate | Criteria |
|------|----------|
| **Boot stability** | System boots 100 times without failure (power cycle test) |
| **Watchdog margin** | Worst-case tick time < 20 ms (measured with `micros()`) |
| **Memory stability** | Free heap does not decrease over 1 hour of operation |
| **CAN integrity** | Heartbeat TX maintains 100 ms ± 5 ms interval |
| **No regression** | All previous phase test criteria still pass |

### 9.3 Rollback Strategy

If any phase introduces instability:

1. Disable the new module by commenting out its `_init()` and `_update()` calls in `app.cpp`
2. Previous-phase firmware remains functional
3. Debug the issue in isolation
4. Re-integrate after fix is verified

This is possible because every module is a pair of free functions (`module_init()` + `module_update()`) with no cross-dependencies except through `vehicle_data` (read-only for most modules).

---

## Appendix A — Design Decisions Log

| Decision | Rationale | Alternative Rejected |
|----------|-----------|---------------------|
| POD structs instead of classes | Zero constructors, zero static init risk | C++ classes (original pattern, caused bootloop) |
| No PSRAM | Eliminates OPI eFuse bootloop entirely | PSRAM with fallback (adds complexity) |
| Direct TFT drawing | No memory overhead, bounded draw time | Sprites (300 KB+ each, PSRAM required) |
| Single-core | No race conditions, no mutexes, no task starvation | FreeRTOS dual-core (original pattern, caused issues) |
| 20 FPS limit | Sufficient for dashboard HMI, 50 ms frame budget | 30 FPS (original, tighter timing budget) |
| Flat menu pages | No recursion, bounded navigation | Tree-structured menu (unbounded depth) |
| Ring buffer for errors | Fixed memory, no allocation | Dynamic list (heap fragmentation) |
| No BootGuard/NVS | Removes boot-path complexity | NVS boot counter (adds failure mode) |
| No logo delay | Eliminates 1.5 s blocking wait | Logo display (blocking delay, WDT risk) |
| No safe mode | Simple architecture doesn't need partial init | Safe mode (creates inconsistent state) |
| Static CRGB arrays | Zero heap allocation for LEDs | Dynamic FastLED allocation (fragmentation risk) |
| Step-based LED animations | Non-blocking, bounded execution time | Animation loops with delay() (blocks loop) |
| Pointer-based DFPlayer | Defers construction to setup() | Global DFPlayer object (pre-setup constructor) |
| Priority audio queue | Bounded memory, O(1) operations | Dynamic list (unbounded growth) |
| UART0 for obstacle | Native pin, no conflict with UART1 (audio) | I2C sensor (requires PSRAM-conflicting pins) |
| Obstacle CAN TX from ESP32 | Preserves existing sensor wiring | Move sensor to STM32 (requires hardware change) |
| Non-fatal subsystem failures | System continues without LED/audio/obstacle | Fatal failures (single subsystem crash = bootloop) |

## Appendix B — Migration Path

This architecture is designed to be **incrementally implementable** (see Phase 9 for detailed schedule):

1. **Week 1:** `main.cpp` + `can_bus` + `vehicle_data` + `heartbeat` — CAN communication working
2. **Week 2:** `screen_mgr` + `screen_boot` + `screen_standby` + `tft_direct` — Basic display
3. **Week 3:** `screen_drive` + `ui_car` + `ui_gauge` + `ui_bar` + `ui_icons` — Full dashboard
4. **Week 4:** `overlay` + `screen_safe` + `screen_error` — Fault handling
5. **Week 5:** `led_ctrl` + `audio_ctrl` — LED animations + DFPlayer audio
6. **Week 6:** `obstacle` + `obstacle_can` + `ui_obstacle` — Obstacle detection + display
7. **Week 7:** `diag_menu` + `can_tx` (commands) + `ack_tracker` — Full diagnostics + commands
8. **Week 8:** Integration testing with STM32 hardware

Each week produces a **bootable, testable firmware** with progressively more features.

## Appendix C — Feature Parity Confirmation

All ESP32-side vehicle features from the original FULL-FIRMWARE are preserved:

| Feature | Original | New Design | Status |
|---------|----------|------------|--------|
| HMI screens (Boot, Standby, Drive, Safe, Error) | ✅ | ✅ | Preserved |
| Degraded mode overlay | ✅ | ✅ | Preserved |
| Heartbeat loss overlay | ✅ | ✅ | Preserved |
| Front LEDs (28× WS2812B, Knight Rider, throttle-reactive) | ✅ | ✅ | Preserved |
| Rear LEDs (16× WS2812B, position, brake, turn signals) | ✅ | ✅ | Preserved |
| Audio feedback (DFPlayer, 68 tracks, event-driven) | ✅ | ✅ | Preserved |
| Obstacle display (TOFSense LiDAR, distance, zone, health) | ✅ | ✅ | Preserved |
| Obstacle CAN TX (0x208/0x209 to STM32) | ✅ | ✅ | Preserved |
| Obstacle audio warnings | ✅ | ✅ | Preserved |
| Hidden diagnostics menu | ✅ | ✅ Extended (9 pages vs. 6) | Enhanced |
| CAN heartbeat TX/RX | ✅ | ✅ | Preserved |
| Command transmission (throttle, steering, mode) | ✅ | ✅ | Preserved |
| ACK tracking | ✅ | ✅ | Preserved |
| Service commands (0x110) | ✅ | ✅ | Preserved |
| Top-view car diagram | ✅ | ✅ | Preserved |
| Wheel torque + temperature display | ✅ | ✅ | Preserved |
| Gear selector (P R N D1 D2) | ✅ | ✅ | Preserved |
| Mode icons (4×4, 4×2, 360°) | ✅ | ✅ | Preserved |
| Battery indicator | ✅ | ✅ | Preserved |
| Pedal bar | ✅ | ✅ | Preserved |
| Speed display | ✅ | ✅ | Preserved |
| Steering indicator | ✅ | ✅ | Preserved |

**Features intentionally NOT included** (already migrated to STM32 or not applicable):

- Motor control (PWM, PID, traction) — STM32 responsibility
- Sensor reading (INA226, DS18B20, wheel speed, encoder, pedal ADC) — STM32 responsibility
- Safety system (ABS, TCS, state machine) — STM32 responsibility
- I2C peripherals (PCA9685, MCP23017, TCA9548A) — STM32 responsibility
- WiFi/Bluetooth — not implemented in original either
- OTA updates — not implemented in original either
