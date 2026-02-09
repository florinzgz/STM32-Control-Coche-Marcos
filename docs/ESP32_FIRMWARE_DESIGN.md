# ESP32-S3 HMI Firmware — Design Document

**Revision:** 1.1  
**Status:** Pending validation  
**Date:** 2026-02-09  
**Scope:** Architecture and technology decisions for the ESP32-S3 HMI firmware  
**References:** `docs/CAN_CONTRACT_FINAL.md` rev 1.0, `docs/HMI_STATE_MODEL.md` rev 1.0, `docs/SERVICE_MODE.md`

---

## 1. Language and Framework Decision

### Language: C++

C++ (standard C++17) is the chosen language for the ESP32-S3 HMI firmware.

### Framework: Arduino on PlatformIO

| Choice | Value |
|--------|-------|
| Language | **C++ (C++17)** |
| Framework | **Arduino** |
| Build system | **PlatformIO** |
| ESP-IDF | ⛔ **NOT used** — neither directly nor mixed with Arduino |

### Technical Justification

| Requirement | Why C++ on Arduino is adequate |
|-------------|--------------------------------|
| **TFT display / GUI** | Arduino TFT libraries (TFT_eSPI, LVGL Arduino port) are C++ APIs. Classes encapsulate screens, widgets, and rendering state cleanly. |
| **Menu system / screens** | A state-machine with polymorphic screen classes (`class Screen`) is natural in C++. Each screen overrides `draw()` and `handleInput()`. |
| **CAN message parsing** | `struct`s with typed fields, `enum class` for message IDs and states, and constructors for byte-level decoding are safer and more readable than raw C casts. |
| **Service mode / diagnostics** | Service module viewer benefits from containers (`std::array`, fixed-size buffers) and RAII for resource management. |
| **Fault overlays** | Overlay logic maps naturally to composition of UI objects. C++ allows overlays to be independent objects drawn on top of the active screen. |
| **Code safety** | `enum class` prevents implicit int conversions on CAN IDs and system states. `constexpr` replaces `#define` for compile-time constants. |
| **Arduino ecosystem** | All Arduino libraries are C++ compatible. The Arduino core for ESP32-S3 is itself C++. No API mismatch. |

### Explicit Confirmation

- ✅ All code will use **Arduino APIs only** (`Serial`, `SPI`, `Wire`, GPIO via `digitalRead`/`digitalWrite`, `millis()`).
- ✅ CAN communication will use an **Arduino-compatible CAN library** (e.g., `arduino-CAN` or `ESP32-TWAI-CAN` that wraps the peripheral through an Arduino-style API).
- ⛔ **No direct ESP-IDF calls** (`esp_*`, `twai_*`, `driver/*`, `freertos/*` used directly).
- ⛔ **No `menuconfig`** or `sdkconfig` manual edits.
- ⛔ **No mixed Arduino + ESP-IDF component builds**.

---

## 2. Repository Compatibility

### Is it a problem that the repo is marked as "C"?

**No.** GitHub language detection is based on file extensions and byte count. Adding `.cpp` and `.h` files in the `esp32/` directory will cause GitHub to detect both C and C++. This is purely cosmetic and has zero impact on compilation or project structure.

### Organization Strategy

The STM32 firmware remains untouched at the repository root. The ESP32 firmware lives in its own `esp32/` subdirectory with an independent PlatformIO build. The two projects share no build system, no headers, and no source files.

```
STM32-Control-Coche-Marcos/          ← Repository root
│
├── Core/                            ← STM32 firmware (C, unchanged)
│   ├── Inc/
│   └── Src/
├── Drivers/                         ← STM32 HAL (gitignored)
├── Makefile                         ← STM32 build
├── STM32G474RETX_FLASH.ld           ← STM32 linker script
├── *.ioc                            ← STM32CubeMX config
│
├── esp32/                           ← ESP32-S3 HMI firmware (C++, new)
│   ├── platformio.ini               ← PlatformIO build config
│   ├── src/                         ← C++ source files
│   │   └── main.cpp                 ← (future) Arduino entry point
│   └── include/                     ← C++ headers
│       └── can_ids.h                ← (future) CAN ID mirror constants
│
├── docs/                            ← Shared documentation
│   ├── CAN_CONTRACT_FINAL.md        ← Authoritative CAN contract
│   ├── HMI_STATE_MODEL.md           ← HMI behavior spec
│   ├── ESP32_FIRMWARE_DESIGN.md     ← This document
│   └── ...
│
└── README.md
```

### Isolation Guarantees

| Concern | Mitigation |
|---------|------------|
| STM32 build breaks | ESP32 code is in `esp32/`. The root `Makefile` does not recurse into `esp32/`. |
| C/C++ header conflicts | ESP32 code never includes anything from `Core/Inc/`. CAN IDs are mirrored as `constexpr` in `esp32/include/can_ids.h`, traced to `CAN_CONTRACT_FINAL.md`. |
| Shared CAN definitions | The ESP32 mirrors (not imports) the CAN IDs. The source of truth is `docs/CAN_CONTRACT_FINAL.md` rev 1.0. If the contract changes, both sides update independently under the new revision. |

---

## 3. Concepts Derived from Base Firmware

The original base firmware ([FULL-FIRMWARE-Coche-Marcos](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos)) is treated as a **read-only reference**. No code has been copied from it. The following concepts have been extracted and will be **cleanly reimplemented** in the ESP32 HMI:

### 3.1 What Was Extracted (Concept Only)

| Concept | Base Firmware Source | How It Applies to ESP32 HMI |
|---------|----------------------|-----------------------------|
| **Menu hierarchy** | Multi-screen UI with drive, settings, diagnostics pages | Screen-per-state architecture: Boot, Standby, Drive, Safe, Error (mapped 1:1 to STM32 `system_state`) |
| **Hidden / engineering menu** | Hidden diagnostic screens accessible under specific conditions | Engineering menu accessible only in STANDBY or ERROR, with multi-step entry gesture, vehicle stopped, no CAN timeout (see `HMI_STATE_MODEL.md` §6) |
| **Fault visualization philosophy** | Visual icons and banners for ABS, TCS, temperature, current faults | Fault overlays driven by `fault_flags` (byte 2 of 0x001) and `error_code` (byte 2 of 0x203) — displayed as colored banners and icons on top of the active screen |
| **Icon semantics** | ABS indicator, TCS indicator, temperature warning, current warning | ABS/TCS as steady informational icons (bits 5–6 of `fault_flags`); temperature/current as amber warning icons (bits 1–2); encoder/wheel as fault markers (bits 3–4) |
| **Startup / shutdown behavior** | Boot splash → ready → active progression | HMI starts at Boot screen, transitions to Standby on first heartbeat, then follows `system_state` exclusively |
| **"Degraded but driveable" concept** | `limp_mode.cpp`: NORMAL → DEGRADED → LIMP → CRITICAL states | ESP32 displays degraded status when non-critical modules are faulted/disabled. Power is limited by STM32 (40% power, 50% speed). The ESP32 only shows the state — it never computes it. |
| **Service mode / module viewer** | `car_sensors.cpp`: per-subsystem enable flags; `temperature.cpp`: `sensorOk[]` | Service viewer screen shows per-module status from CAN IDs 0x301–0x303 and allows enable/disable via 0x110. All safety decisions remain on STM32. |

### 3.2 What Was NOT Taken

| Base Feature | Reason Excluded |
|-------------|-----------------|
| Safety computation logic (ABS, TCS, current/temperature limits) | ESP32 is HMI only — STM32 is the sole safety authority |
| Sensor reading code | ESP32 has no vehicle sensors — all data arrives via CAN |
| Motor control / PWM logic | Actuator control is exclusively STM32 domain |
| FreeRTOS task architecture | ESP32 Arduino firmware uses cooperative `loop()` — no RTOS tasks |
| NVS/persistent storage of settings | Not required for HMI; settings reset on power cycle |
| Direct CAN driver code (TWAI) | ESP32 uses Arduino-compatible CAN library abstraction only |

### 3.3 Explicit Confirmations

- ✅ **No code was copied** from the base firmware — all behavior is reimplemented from concept descriptions
- ✅ **STM32 firmware is the sole safety authority** — the ESP32 never computes, overrides, or bypasses safety decisions
- ✅ **All vehicle data comes from CAN messages** — the ESP32 assumes no sensors that are not explicitly sent over CAN
- ✅ **The base repo is known to have a reboot loop** — this ESP32 firmware is independent and not affected

---

## 4. ESP32 Firmware Architecture (High Level)

### 4.1 Module Overview

| Module | Directory | Responsibility |
|--------|-----------|----------------|
| **CAN Interface** | `src/can_interface.*` | Send/receive CAN frames. Manage heartbeat TX (0x011 @ 100 ms). Detect STM32 heartbeat loss (> 250 ms without 0x001). Parse incoming status frames. |
| **Vehicle Data** | `src/vehicle_data.*` | Central data store. Holds the latest decoded values from all status CAN messages. Thread-safe read access for the UI. Tracks data staleness. |
| **Display** | `src/display.*` | TFT hardware initialization and drawing primitives. Wraps the TFT library (e.g., TFT_eSPI). Provides `drawText()`, `drawGauge()`, `fillRect()`, etc. |
| **Screens** | `src/screens/*` | One class per HMI screen: `BootScreen`, `StandbyScreen`, `DriveScreen`, `SafeScreen`, `ErrorScreen`. Each implements `draw()` and `handleInput()`. |
| **Screen Manager** | `src/screen_manager.*` | State machine that maps `system_state` (byte 1 of 0x001) to the active screen. Enforces transition rules from `HMI_STATE_MODEL.md` §1. |
| **Fault Overlay** | `src/fault_overlay.*` | Renders fault banners and indicators on top of the active screen. Driven by `fault_flags` (byte 2 of 0x001) and `error_code` (byte 2 of 0x203). |
| **Service Viewer** | `src/service_viewer.*` | Displays module enable/disable/fault status from CAN IDs 0x301–0x303. Sends 0x110 (SERVICE_CMD) to toggle non-critical modules. Only accessible under engineering menu conditions (§6 of HMI_STATE_MODEL.md). |
| **Input** | `src/input.*` | Reads touch panel, physical buttons, or encoder knob. Debounces inputs. Passes events to the active screen. |
| **Engineering Menu** | `src/engineering_menu.*` | Hidden diagnostic menu. Shows CAN stats, raw hex, firmware info. Entry conditions enforced per HMI_STATE_MODEL.md §6. Read-only — never sends actuator commands. |

### 4.2 CAN Messages Consumed (STM32 → ESP32)

These are the messages the ESP32 will **receive and parse**. IDs 0x001–0x300 are defined in `CAN_CONTRACT_FINAL.md` rev 1.0. IDs 0x301–0x303 are defined in `docs/SERVICE_MODE.md` and implemented in `Core/Inc/can_handler.h`.

| CAN ID | Name | Data Used By HMI | Rate |
|--------|------|-------------------|------|
| 0x001 | HEARTBEAT_STM32 | `alive_counter` (byte 0), `system_state` (byte 1), `fault_flags` (byte 2) | 100 ms |
| 0x200 | STATUS_SPEED | 4 × uint16 LE wheel speeds (×0.1 km/h) | 100 ms |
| 0x201 | STATUS_CURRENT | 4 × uint16 LE motor currents (×0.01 A) | 100 ms |
| 0x202 | STATUS_TEMP | 5 × int8 temperatures (°C) | 1000 ms |
| 0x203 | STATUS_SAFETY | `abs_active` (byte 0), `tcs_active` (byte 1), `error_code` (byte 2) | 100 ms |
| 0x204 | STATUS_STEERING | int16 LE actual angle (×0.1°), `calibrated` (byte 2) | 100 ms |
| 0x205 | STATUS_TRACTION | 4 × uint8 per-wheel traction scale (%) | 100 ms |
| 0x206 | STATUS_TEMP_MAP | 5 × int8 mapped temps: FL, FR, RL, RR, Ambient (°C) | 1000 ms |
| 0x300 | DIAG_ERROR | `error_code` (byte 0), `subsystem` (byte 1) | On-demand |
| 0x301 | SERVICE_FAULTS | 32-bit fault bitmask | 1000 ms |
| 0x302 | SERVICE_ENABLED | 32-bit enabled bitmask | 1000 ms |
| 0x303 | SERVICE_DISABLED | 32-bit disabled bitmask | 1000 ms |

### 4.3 CAN Messages Sent (ESP32 → STM32)

These are the messages the ESP32 will **transmit**. IDs 0x011–0x102 are defined in `CAN_CONTRACT_FINAL.md` rev 1.0. ID 0x110 is defined in `docs/SERVICE_MODE.md` and accepted by STM32 RX filter 2 (`Core/Src/can_handler.c`).

| CAN ID | Name | Payload | Condition | Rate |
|--------|------|---------|-----------|------|
| 0x011 | HEARTBEAT_ESP32 | (payload ignored by STM32) | **Always** — every system state, including heartbeat-loss | 100 ms |
| 0x100 | CMD_THROTTLE | uint8 throttle % (0–100) | **Only when `system_state == ACTIVE (2)`** | 50 ms |
| 0x101 | CMD_STEERING | int16 LE angle (×10, in 0.1° units) | **Only when `system_state == ACTIVE (2)`** | 50 ms |
| 0x102 | CMD_MODE | uint8 mode_flags (bit 0: 4×4, bit 1: tank turn) | **Only when `system_state == ACTIVE (2)`** | On-demand |
| 0x110 | SERVICE_CMD | Module ID + action (enable/disable) | **Only from engineering menu** | On-demand |

### 4.4 Data Flow Diagram

```
┌──────────────────────────────────────────────┐
│                  CAN BUS (500 kbps)          │
└──────────┬──────────────────────┬────────────┘
           │ RX                  │ TX
           ▼                     │
   ┌───────────────┐    ┌───────┴───────┐
   │ CAN Interface │    │ CAN Interface │
   │  (RX parser)  │    │  (TX sender)  │
   └───────┬───────┘    └───────────────┘
           │                     ▲
           ▼                     │
   ┌───────────────┐    ┌───────┴───────┐
   │ Vehicle Data  │    │ Screen/Input  │
   │  (data store) │    │  (commands)   │
   └───────┬───────┘    └───────────────┘
           │
     ┌─────┴─────┐
     ▼           ▼
┌─────────┐ ┌──────────┐
│ Screen  │ │  Fault   │
│ Manager │ │ Overlay  │
└────┬────┘ └────┬─────┘
     │           │
     ▼           ▼
   ┌───────────────┐
   │    Display    │
   │    (TFT)      │
   └───────────────┘
```

---

## 5. Strict Rules — Compliance Matrix

| Rule | Status | How Enforced |
|------|--------|--------------|
| ❌ No ESP-IDF | ✅ Compliant | `platformio.ini` sets `framework = arduino`. No `esp-idf` component. No `#include "driver/*.h"`. |
| ❌ No C/C++ mixing without encapsulation | ✅ Compliant | ESP32 project is pure C++. STM32 project is pure C. No cross-includes. |
| ❌ No safety logic in ESP32 | ✅ Compliant | ESP32 never computes ABS, TCS, temperature thresholds, or current limits. It only displays values received via CAN. |
| ❌ No assumed sensors | ✅ Compliant | Every value displayed comes from a defined CAN message. No GPIO reads for vehicle sensors. |
| ❌ No redefined CAN IDs | ✅ Compliant | All CAN IDs mirror `CAN_CONTRACT_FINAL.md` rev 1.0 exactly. `constexpr` values in `can_ids.h` are traced to the contract. |
| ✅ Arduino + PlatformIO | ✅ Compliant | `platformio.ini` specifies `framework = arduino`, `platform = espressif32`, `board = esp32-s3-devkitc-1`. |
| ✅ C++ moderno y limpio | ✅ Compliant | C++17, `enum class`, `constexpr`, RAII, no raw `new`/`delete`. |
| ✅ ESP32 solo interpreta y muestra | ✅ Compliant | The ESP32 is a read-only display with intent-sending capability. It never overrides STM32 decisions. |
| ✅ Todo viene del CAN | ✅ Compliant | Vehicle Data store is populated exclusively from CAN RX. No local sensor reads. |

---

## 6. Summary

| Item | Decision |
|------|----------|
| **Language** | C++ (C++17) |
| **Framework** | Arduino on PlatformIO |
| **ESP-IDF** | Not used in any form |
| **Project location** | `esp32/` subdirectory, independent from STM32 root |
| **STM32 impact** | None — no files modified, no build changes |
| **CAN contract** | `CAN_CONTRACT_FINAL.md` rev 1.0 (FROZEN) + `SERVICE_MODE.md` (service IDs) |
| **HMI behavior** | `HMI_STATE_MODEL.md` rev 1.0 |
| **Safety authority** | STM32 only — ESP32 is HMI / intent sender |
| **Base repo concepts** | Menu hierarchy, hidden menus, fault icons, startup sequence, degraded mode — all reimplemented from scratch (no code copied) |

---

## Next Steps (Pending Validation)

After this design is approved:

1. Implement `can_ids.h` — mirror CAN ID constants from the contract
2. Implement `can_interface` — Arduino CAN library integration, heartbeat TX/RX
3. Implement `vehicle_data` — central data store for all telemetry
4. Implement `screen_manager` — state machine driven by `system_state`
5. Implement individual screens — one per system state
6. Implement `fault_overlay` — overlay rendering from `fault_flags` and `error_code`
7. Implement `input` — touch/button input handling
8. Implement `service_viewer` and `engineering_menu`

No code will be written until this design document is validated.
