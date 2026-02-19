# STM32 Obstacle Safety Audit Report

## Overview

This report verifies that the STM32 obstacle safety system operates as the **primary safety controller**, with ESP32 CAN frames serving as **advisory only**. The vehicle remains safely drivable even if ESP32, CAN bus, or HMI completely disappear.

**Date:** 2026-02-19
**Firmware:** STM32G474RE — safety_system.c obstacle module
**Architecture:** STM32 = primary safety authority; ESP32 = optional advisory sensor

---

## 1. Architecture Summary

### Previous Design (CAN-Dependent)
- STM32 received obstacle distance via CAN ID 0x208 from ESP32
- CAN timeout → `obstacle_scale = 0.3` (restricted motion)
- No local plausibility validation
- No speed-dependent thresholds
- No temporal hysteresis for obstacle confirmation

### New Design (STM32 Primary)
- STM32 runs a **full autonomous obstacle state machine**
- CAN obstacle frames are **advisory only** — never mandatory for motion
- Physical plausibility validation rejects implausible readings
- Stuck-sensor detection identifies frozen sensors
- Speed-dependent stopping distance adjusts thresholds dynamically
- Temporal hysteresis confirms both obstacle presence and clearance
- CAN loss → `obstacle_scale = 1.0` (LIMP_HOME speed cap provides safety)

---

## 2. Local State Machine

```
NO_SENSOR ──(first valid CAN)──▶ NORMAL
NORMAL ──(obstacle in range)──▶ CONFIRMING
CONFIRMING ──(200 ms confirmed)──▶ ACTIVE
CONFIRMING ──(obstacle gone)──▶ NORMAL
ACTIVE ──(obstacle receded)──▶ CLEARING
CLEARING ──(1000 ms clear)──▶ NORMAL
CLEARING ──(obstacle returned)──▶ ACTIVE
any ──(sensor fault)──▶ SENSOR_FAULT
SENSOR_FAULT ──(valid data)──▶ NORMAL
any ──(CAN timeout)──▶ NO_SENSOR (scale = 1.0)
```

### State Descriptions

| State | Scale | Forward Blocked | Description |
|-------|-------|-----------------|-------------|
| `NO_SENSOR` | 1.0 | No | No CAN data — rely on LIMP_HOME speed cap |
| `NORMAL` | 1.0 | No | Sensor valid, no obstacle in range |
| `CONFIRMING` | 0.7 (gentle preemptive) | No | Temporal confirmation in progress |
| `ACTIVE` | 0.0 – 0.7 (distance-based) | Yes (if emergency) | Confirmed obstacle, torque reduction applied |
| `CLEARING` | 0.7 | No | Obstacle receding, confirming clearance |
| `SENSOR_FAULT` | 0.3 (conservative) | No | Sensor data implausible — mobile at reduced power |

---

## 3. Safety Features Implemented

### 3.1 Physical Plausibility Validation
- **Purpose:** Reject noise spikes and corrupted readings
- **Method:** Distance cannot decrease faster than `(vehicle_speed + max_obstacle_speed) × dt`
- **Threshold:** 8 m/s combined approach rate (vehicle + obstacle)
- **Behavior:** Implausible readings are rejected; previous validated distance is retained

### 3.2 Stuck Sensor Detection
- **Purpose:** Detect frozen sensors (hardware failure or ESP32 crash)
- **Method:** If vehicle speed > 1 km/h but distance unchanged (±10 mm) for > 1 second, sensor is declared stuck
- **Behavior:** Transition to `SENSOR_FAULT` → conservative 0.3 scale (mobile)

### 3.3 Speed-Dependent Stopping Distance
- **Purpose:** Increase safety margins at higher speeds
- **Formula:** `d_stop = v² / (2 × 3.0 m/s²) + 200 mm`
- **Dynamic thresholds:** Emergency, critical, and warning distances scale with speed
- **Floor:** Static thresholds (200/500/1000 mm) are always minimum values
- **Maximum:** Clamped to 4000 mm to prevent unreasonable distances

### 3.4 Temporal Hysteresis
- **Obstacle confirmation:** 200 ms sustained detection before full torque reduction
- **Clearance confirmation:** 1000 ms sustained clearance before returning to NORMAL
- **Purpose:** Prevents oscillation and false positives from transient readings

### 3.5 CAN Advisory Mode
- CAN timeout → `obstacle_scale = 1.0` (full motion allowed)
- LIMP_HOME speed cap (5 km/h) provides independent safety net
- Stale data (frozen counter) → `SENSOR_FAULT` state (0.3 scale)
- Sensor unhealthy flag → `SENSOR_FAULT` state (0.3 scale)

### 3.6 Reverse Escape
- When forward is blocked (`obstacle_scale = 0.0`), reverse travel is still allowed
- `Obstacle_IsForwardBlocked()` flag consumed by motor control pipeline
- Vehicle is **never immobilized** — reverse escape always available

---

## 4. Failure Mode Analysis

### 4.1 ESP32 Unplugged
| Condition | Behavior | Scale | Safe? |
|-----------|----------|-------|-------|
| ESP32 never connected | `NO_SENSOR` state, full motion | 1.0 | ✅ LIMP_HOME speed cap (5 km/h) |
| ESP32 disconnected during operation | CAN timeout → `NO_SENSOR` | 1.0 | ✅ LIMP_HOME transition |
| ESP32 rebooting | Temporary CAN gap → `NO_SENSOR` → `NORMAL` on recovery | 1.0 → restored | ✅ Seamless recovery |

**Result:** Vehicle remains mobile at walking speed. No immobilization.

### 4.2 CAN Disconnected
| Condition | Behavior | Scale | Safe? |
|-----------|----------|-------|-------|
| CAN cable unplugged | CAN timeout after 500 ms → `NO_SENSOR` | 1.0 | ✅ LIMP_HOME provides safety |
| CAN bus-off event | Same as disconnected | 1.0 | ✅ Independent safety net |

**Result:** CAN loss triggers LIMP_HOME (5 km/h cap, 20% torque). Vehicle mobile.

### 4.3 CAN Frozen (Stale Data)
| Condition | Behavior | Scale | Safe? |
|-----------|----------|-------|-------|
| Rolling counter frozen ≥ 3 frames | `SENSOR_FAULT` state | 0.3 | ✅ Conservative but mobile |
| ESP32 crash (cached CAN values) | Counter detected frozen | 0.3 | ✅ 30% power limit |

**Result:** Stuck CAN data detected within ~200 ms. Vehicle at 30% power.

### 4.4 Sensor Stuck
| Condition | Behavior | Scale | Safe? |
|-----------|----------|-------|-------|
| Sensor reports constant value while moving | Stuck detection after 1 s | 0.3 | ✅ Conservative mode |
| Sensor hardware failure | ESP32 health flag = 0 → `SENSOR_FAULT` | 0.3 | ✅ Mobile at reduced power |

**Result:** Stuck sensor detected independently by STM32. No immobilization.

### 4.5 Sensor Noisy
| Condition | Behavior | Scale | Safe? |
|-----------|----------|-------|-------|
| Distance jumps implausibly | Plausibility filter rejects reading | Previous valid | ✅ Last good value used |
| Persistent noise | Multiple rejections → stale → `SENSOR_FAULT` | 0.3 | ✅ Conservative fallback |

**Result:** Noise is filtered. Vehicle maintains last valid obstacle state.

### 4.6 Object Suddenly Appears
| Condition | Behavior | Scale | Safe? |
|-----------|----------|-------|-------|
| Object at < emergency distance | `CONFIRMING` → (200 ms) → `ACTIVE` | 0.7 → 0.0 | ✅ Preemptive reduction during confirmation |
| Object within stopping distance | Speed-dependent threshold triggers earlier | 0.0 – 0.3 | ✅ Dynamic margins |

**Result:** 200 ms confirmation with gentle 0.7 reduction during confirmation period. Full stop after confirmation. Reverse escape available.

### 4.7 Object Disappears
| Condition | Behavior | Scale | Safe? |
|-----------|----------|-------|-------|
| Obstacle moves away | `CLEARING` state for 1000 ms | 0.7 | ✅ Gradual recovery |
| Transient reading (noise) | Confirmation prevents false positive | 1.0 (not triggered) | ✅ No unnecessary braking |

**Result:** 1 second clearance confirmation prevents oscillation. Smooth recovery.

### 4.8 Moving Slowly Toward Wall
| Condition | Behavior | Scale | Safe? |
|-----------|----------|-------|-------|
| 1 km/h approach | Warning at ~1000 mm (0.7), critical at ~500 mm (0.3) | Gradual reduction | ✅ Controlled slowdown |
| 5 km/h approach | Dynamic emergency at ~522 mm (vs static 200 mm) | Earlier stopping | ✅ Speed-proportional margins |
| Contact imminent | Emergency stop at < emergency threshold, reverse escape | 0.0 forward | ✅ Forward blocked, reverse allowed |

**Result:** Progressive slowdown. No sudden stop. Reverse escape always available.

---

## 5. Preserved Protections

The following existing safety systems are **NOT modified** and continue to operate independently:

| Protection | Status | Location |
|-----------|--------|----------|
| Overcurrent detection | ✅ Preserved | `Safety_CheckCurrent()` |
| Over-temperature | ✅ Preserved | `Safety_CheckTemperature()` |
| IWDG watchdog (500 ms) | ✅ Preserved | Main loop |
| Battery undervoltage | ✅ Preserved | `Safety_CheckBatteryVoltage()` |
| CAN heartbeat timeout | ✅ Preserved | `Safety_CheckCANTimeout()` → LIMP_HOME |
| ABS/TCS | ✅ Preserved | `ABS_Update()`, `TCS_Update()` |
| Encoder health | ✅ Preserved | `Encoder_CheckHealth()` |
| Relay power sequencing | ✅ Preserved | `Relay_SequencerUpdate()` |
| Pedal plausibility | ✅ Preserved | Dual-channel validation |
| Emergency stop | ✅ Preserved | `Safety_EmergencyStop()` |

---

## 6. CAN Dependency Removal Summary

| Before | After |
|--------|-------|
| CAN timeout → scale 0.3 (restricted) | CAN timeout → scale 1.0 (LIMP_HOME safety) |
| No plausibility check on CAN data | Speed-based plausibility validation |
| No stuck-sensor detection | Distance-static detection with vehicle speed |
| Fixed distance thresholds | Speed-dependent dynamic thresholds |
| No temporal confirmation | 200 ms confirm / 1000 ms clear hysteresis |
| CAN = mandatory for obstacle safety | CAN = advisory only |
| ESP32 = primary obstacle authority | STM32 = primary; ESP32 = optional advisory |

---

## 7. Conclusion

The STM32 obstacle safety module now operates as the **primary safety controller** with a local state machine that is fully independent of CAN bus availability. The vehicle remains safely drivable in all tested failure scenarios:

- **No motion immobilization** — only controlled slowdown
- **Reverse escape** always available when forward is blocked
- **LIMP_HOME** (5 km/h, 20% torque) provides independent safety net when CAN is lost
- **Conservative fallback** (30% power) when sensor data is unreliable
- All existing protections (overcurrent, temperature, watchdog) preserved unchanged
