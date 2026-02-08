# Service Mode / Module Disable System

## Overview

The Service Mode system implements the SERVICE / LIMP philosophy from the base firmware
([FULL-FIRMWARE-Coche-Marcos](https://github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos)),
adapted for the STM32 safety controller.

**Core philosophy**: Non-critical sensor faults do NOT block the vehicle.
The vehicle remains drivable with reduced performance. The operator is informed
of the exact fault. Faulty modules can be temporarily disabled. The vehicle can
be driven home and repaired later.

## Base Firmware Traceability

| STM32 Feature | Base Firmware Source | Concept |
|---|---|---|
| Module enable/disable | `car_sensors.cpp`: `cfg.tempSensorsEnabled`, `cfg.currentSensorsEnabled`, `cfg.wheelSensorsEnabled` | Per-subsystem enable flags |
| Per-sensor fault tracking | `temperature.cpp`: `sensorOk[]` array | Individual sensor OK/fault state |
| DEGRADED state (40% power, 50% speed) | `limp_mode.cpp`: `LimpState::LIMP`, `POWER_LIMP=0.4`, `SPEED_LIMP=0.5` | Drive-home mode |
| Consecutive error escalation | `relays.cpp`: `consecutiveErrors >= 3` | Only escalate to SAFE after repeated failures |
| State hysteresis | `limp_mode.cpp`: `STATE_HYSTERESIS_MS = 500` | Prevent rapid state oscillation |
| Factory restore | `limp_mode.cpp`: `reset()` | Clear all manual overrides |

## Architecture

### Files

| File | Purpose |
|---|---|
| `Core/Inc/service_mode.h` | Module IDs, classification, fault types, public API |
| `Core/Src/service_mode.c` | Module registry, enable/disable logic, fault tracking, bitmask generation |
| `Core/Src/safety_system.c` | Integration: consults service_mode for disabled modules |
| `Core/Src/can_handler.c` | CAN TX/RX for service status and module control commands |
| `Core/Src/main.c` | Initialisation and periodic service status reporting |

### Module Classification

**CRITICAL** (never disableable — always force SAFE/ERROR on fault):

| Module ID | Name | Reason |
|---|---|---|
| 0 | CAN Timeout | Loss of ESP32 link = no operator control |
| 1 | Emergency Stop | Hardware safety interlock |
| 2 | Watchdog | System health monitor |
| 3 | Main Relay | Power delivery integrity |

**NON-CRITICAL** (can be disabled by user — enters DEGRADED, not SAFE):

| Module ID | Name | Base Firmware Reference |
|---|---|---|
| 4–8 | Temperature Sensors 0–4 | `cfg.tempSensorsEnabled` |
| 9–14 | Current Sensors 0–5 | `cfg.currentSensorsEnabled` |
| 15–18 | Wheel Speed Sensors FL/FR/RL/RR | `cfg.wheelSensorsEnabled` |
| 19 | Steering Center Sensor | Optional centering assist |
| 20 | Steering Encoder | Steering PID feedback |
| 21 | ABS | Anti-lock braking |
| 22 | TCS | Traction control |
| 23 | Ackermann Correction | Steering geometry |
| 24 | Obstacle Detection | ESP32-side, STM32 tolerates absence |

### Fault States

| State | Meaning |
|---|---|
| `MODULE_FAULT_NONE` | No fault — sensor operating normally |
| `MODULE_FAULT_WARNING` | Value out of expected range but still readable |
| `MODULE_FAULT_ERROR` | Sensor not responding or invalid data |
| `MODULE_FAULT_DISABLED` | Manually disabled by user |

### Behavior Matrix

| Module Class | Fault Present | Enabled | System Effect |
|---|---|---|---|
| CRITICAL | Yes | Always | → SAFE or ERROR |
| NON-CRITICAL | Yes | Yes | → DEGRADED (40% power, 50% speed) |
| NON-CRITICAL | Yes | No (disabled) | Fault reported, no system impact |
| NON-CRITICAL | No | Yes | Normal operation |
| NON-CRITICAL | No | No (disabled) | Module skipped, no impact |

## CAN Protocol Extension

### New Message IDs (STM32 → ESP32)

| ID | Name | Interval | Bytes | Format |
|---|---|---|---|---|
| `0x301` | Service Fault Mask | 1000 ms | 4 | uint32 LE — bit N = module N has fault |
| `0x302` | Service Enabled Mask | 1000 ms | 4 | uint32 LE — bit N = module N is enabled |
| `0x303` | Service Disabled Mask | 1000 ms | 4 | uint32 LE — bit N = module N is disabled |

### New Message ID (ESP32 → STM32)

| ID | Name | Trigger | Bytes | Format |
|---|---|---|---|---|
| `0x110` | Service Command | On-demand | 2 | Byte 0: command, Byte 1: module_id |

**Commands:**

| Byte 0 | Action | Byte 1 |
|---|---|---|
| `0x00` | Disable module | Module ID (0–24) |
| `0x01` | Enable module | Module ID (0–24) |
| `0xFF` | Factory restore | Ignored |

**Safety constraint**: Critical modules (ID 0–3) reject disable commands silently.

### Existing Heartbeat Enhancement

The existing heartbeat (`0x001`) already carries:
- Byte 1: `system_state` (now includes DEGRADED = 3)
- Byte 2: `fault_flags` (8-bit bitmask, category-level)
- Byte 3: `error_code` (specific fault ID)

The new service messages (`0x301–0x303`) provide **per-module granularity**.

## ESP32 Integration Guide

The ESP32 firmware should:

1. **Parse** `0x301` (fault mask) to identify which specific modules are faulted
2. **Parse** `0x302` (enabled mask) and `0x303` (disabled mask) to show module states
3. **Display** a service menu showing each module's name, status, and fault
4. **Send** `0x110` commands when the user disables/enables a module
5. **Send** `0x110` with `0xFF` for factory restore

The STM32 handles all safety logic. The ESP32 only provides the UI.

## Safety Constraints

1. **Emergency stop ALWAYS overrides** — no service mode bypass
2. **CAN timeout ALWAYS forces SAFE** — cannot be disabled
3. **Disabled modules are explicitly acknowledged** — fault is still reported via CAN
4. **No automatic re-enable** — user must explicitly re-enable or factory restore
5. **Critical modules cannot be disabled** — `ServiceMode_DisableModule()` returns false

## What Was NOT Ported from Base Firmware (and Why)

| Base Feature | Reason Not Ported |
|---|---|
| `LimpState::CRITICAL` (15% power) | STM32 uses SAFE state (actuators off) for critical faults — simpler and safer for bare-metal controller |
| `LimpState::DEGRADED` vs `LIMP` split | STM32 collapses into single DEGRADED at LIMP limits (40%/50%) — conservative approach for safety-critical controller |
| NVS/Storage persistence of enable flags | STM32 has no EEPROM — flags reset on power cycle (safer default) |
| Logger/Serial debug output | STM32 uses CAN for all diagnostics (no UART to operator) |
| FreeRTOS mutex protection | STM32 is single-threaded bare-metal — no concurrency issues |
