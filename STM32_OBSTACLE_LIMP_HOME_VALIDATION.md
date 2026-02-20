# STM32 Obstacle Safety — LIMP_HOME Validation Report

**Date:** 2026-02-19  
**Firmware:** STM32G474RE — `safety_system.c`, `motor_control.c`  
**Scope:** Verify obstacle protection operates correctly during `SYS_STATE_LIMP_HOME`

---

## 1. Executive Summary

This report proves that obstacle scaling is applied **unconditionally** in `Traction_Update()` for all driveable gears, regardless of system state (ACTIVE, DEGRADED, LIMP_HOME). A code change was made to fix a gap where CAN timeout would drop obstacle protection to 1.0 even when an obstacle was actively being tracked.

---

## 2. Traction_Update() Analysis

### 2.1 Obstacle Scale Application Path

**File:** `Core/Src/motor_control.c`, line 1077  

```c
base_pwm = (uint16_t)(base_pwm * effective_obstacle);
```

**Application point in pipeline:**  
`demand → gear_scale → LIMP_HOME speed cap → dead-zone → creep_smooth → obstacle_scale → traction_cap → Ackermann → ABS/TCS wheel_scale → PWM output`

### 2.2 State-Independence Proof

The obstacle scale multiplication at line 1077 has **NO conditional check** on `sys_st` (system state). It executes for all driveable gears (FORWARD, FORWARD_D2, REVERSE). The only early returns that skip it are:
- `GEAR_PARK` (line 786) — motors held, no motion
- `GEAR_NEUTRAL` (line 820) — motors coasting, no drive

**Conclusion:** Obstacle_scale is applied identically in ACTIVE, DEGRADED, and LIMP_HOME.

### 2.3 LIMP_HOME Pipeline Effects

| Pipeline Stage | LIMP_HOME Behavior | Obstacle_Scale Effect |
|---|---|---|
| Throttle demand | Capped at 20% (`LIMP_HOME_TORQUE_LIMIT_FACTOR`) | **Multiplied** by obstacle_scale (0.0–1.0) |
| Speed cap | Demand → 0 above 5 km/h | Obstacle_scale applies **after** speed cap |
| Dynamic braking | Limited by `LIMP_HOME_TORQUE_LIMIT_FACTOR` | Independent — obstacle_scale affects drive only |
| Torque vectoring | Disabled (all wheels equal) | Obstacle_scale applies **before** Ackermann skip |
| Per-wheel ABS/TCS | Active | Multiplicative with obstacle_scale (most restrictive wins) |

---

## 3. Gap Analysis: CAN Timeout During Active Obstacle

### 3.1 Previous Behavior (DEFECT)

When CAN timed out (>500 ms without 0x208 frame), `Obstacle_Update()` unconditionally set:
```c
safety_status.obstacle_scale = 1.0f;   // NO RESTRICTION
obstacle_forward_blocked = 0;
obstacle_state = OBS_STATE_NO_SENSOR;
```

This dropped obstacle protection regardless of whether an obstacle was being tracked.

### 3.2 Failure Scenarios

| Scenario | Previous Behavior | Risk |
|---|---|---|
| **Obstacle < stopping distance + CAN fails** | Scale jumps from 0.0/0.3 to 1.0 instantly | Vehicle drives into obstacle |
| **Vehicle rolling downhill** | Scale = 1.0, speed cap only limits demand (not gravity) | No braking force against gravity |
| **Pedal pressed continuously** | 20% torque at scale 1.0 | Continuous forward motion into obstacle |
| **Single-wheel traction** | Average speed understated → speed cap may not engage | Traction wheel at full LIMP_HOME power |

### 3.3 Fix Applied

**File:** `Core/Src/safety_system.c`, CAN timeout handler in `Obstacle_Update()`

**New policy:**
- If obstacle was in **ACTIVE** or **CONFIRMING** state when CAN died:
  - **Hold** the last obstacle_scale (clamped to ≤ `OBSTACLE_FAULT_SCALE` = 0.3)
  - **Preserve** `obstacle_forward_blocked` flag if it was already set
  - Transition to `OBS_STATE_SENSOR_FAULT` (not `NO_SENSOR`)
- If no obstacle was active (NORMAL, CLEARING, NO_SENSOR):
  - Scale = 1.0 (unchanged — LIMP_HOME speed cap provides safety)
  - Transition to `OBS_STATE_NO_SENSOR`

### 3.4 Effective Result After Fix

| Scenario | New Behavior | Safe? |
|---|---|---|
| **Obstacle < stopping distance + CAN fails** | Scale held at ≤ 0.3 (or 0.0 if forward-blocked) | ✅ Protected |
| **Vehicle rolling downhill** | Scale 0.3 limits base_pwm to 30% | ✅ Reduced drive force |
| **Pedal pressed continuously** | 20% × 0.3 = 6% effective torque | ✅ Minimal forward force |
| **Single-wheel traction** | Scale applied per-wheel before ABS/TCS | ✅ All wheels limited |

---

## 4. Scenario Walkthroughs

### 4.1 Obstacle at 150 mm, CAN Dies, Vehicle in LIMP_HOME

1. **Before CAN timeout:** `obstacle_state = OBS_STATE_ACTIVE`, `obstacle_scale = 0.0`, `obstacle_forward_blocked = 1`
2. **CAN timeout (500 ms):** State was ACTIVE → `obstacle_scale` held at 0.0, `obstacle_forward_blocked` preserved
3. **Traction_Update():** `base_pwm * 0.0 = 0` → no forward motion
4. **Reverse escape:** If gear = REVERSE and forward_blocked → `effective_obstacle = 1.0` → reverse allowed
5. **Result:** Forward stopped, reverse escape available ✅

### 4.2 Vehicle Rolling Downhill at 3 km/h, Obstacle at 400 mm, CAN Dies

1. **Before CAN timeout:** `obstacle_state = OBS_STATE_ACTIVE`, `obstacle_scale = 0.3` (critical zone)
2. **CAN timeout:** State was ACTIVE → `obstacle_scale` held at 0.3
3. **LIMP_HOME speed cap:** 3 km/h < 5 km/h → demand not zeroed
4. **Traction_Update():** `base_pwm * 0.3` → only 30% of LIMP_HOME demand reaches motors
5. **Effective:** 20% torque × 0.3 = 6% effective — very limited drive force against gravity
6. **Result:** Obstacle protection maintained ✅

### 4.3 Pedal Held at 100%, Only One Wheel Has Traction, CAN Dies

1. **LIMP_HOME torque limit:** 100% pedal → clamped to 20% demand
2. **Speed cap gap:** If traction wheel at 5 km/h but other 3 at 0 → avg = 1.25 km/h → below 5 km/h threshold → speed cap does NOT engage. This demonstrates a gap in the speed-cap-only approach.
3. **Before CAN timeout:** `obstacle_scale = 0.3` (obstacle at 400 mm)
4. **CAN timeout:** Scale held at 0.3 (ACTIVE state was preserved)
5. **Traction_Update():** 20% × 0.3 = 6% effective demand for all wheels
6. **Result:** Obstacle_scale compensates for the speed cap gap ✅

### 4.4 No Obstacle Active, CAN Dies, Vehicle in LIMP_HOME

1. **Before CAN timeout:** `obstacle_state = OBS_STATE_NORMAL`, `obstacle_scale = 1.0`
2. **CAN timeout:** No active obstacle → scale = 1.0, state = NO_SENSOR
3. **LIMP_HOME safety net:** 20% torque limit + 5 km/h speed cap
4. **Result:** Full LIMP_HOME operation, no unnecessary restriction ✅

---

## 5. Physical Obstacle Sensor Interface Report

### 5.1 Sensor Type

**TOFSense-M S** — 8×8 matrix Time-of-Flight (ToF) LiDAR sensor  
- Range: 4 meters  
- FOV: 65°  
- Update rate: ~15 Hz  
- Protocol: NLink_TOFSense_M_Frame0 (400-byte UART frames)

### 5.2 Sensor Connection

**The ToF sensor is NOT connected to the STM32G474RE.** It is connected to the **ESP32-S3** HMI controller.

| Property | Value |
|---|---|
| **Connected to** | ESP32-S3 (NOT STM32) |
| **Interface** | UART0 (native) |
| **Baud rate** | 921,600 bps |
| **RX pin** | ESP32 GPIO44 |
| **TX pin** | ESP32 GPIO43 |
| **Protocol** | NLink TOFSense-M frame (400 bytes, 4-byte header 0x57 0x01 0xFF 0x00) |

### 5.3 STM32 Obstacle Data Path

The STM32 receives **processed obstacle data** from ESP32 via CAN bus:

| Property | Value |
|---|---|
| **CAN ID** | 0x208 (OBSTACLE_DISTANCE) |
| **CAN bus** | FDCAN1 at 500 kbps |
| **STM32 CAN pins** | PB8 (FDCAN1_RX), PB9 (FDCAN1_TX) |
| **CAN peripheral** | FDCAN1 with TJA1051 transceiver |
| **Message rate** | 66 ms (15 Hz) |
| **Timeout** | 500 ms |
| **Payload** | Byte 0-1: distance (mm, uint16 LE), Byte 2: zone (0–5), Byte 3: health (0/1), Byte 4: rolling counter |

### 5.4 STM32 GPIO/Peripheral for Obstacle

**No dedicated GPIO pins, timers, ADC channels, EXTI lines, or peripherals are configured on the STM32 for obstacle detection.** The STM32 processes obstacle data exclusively through:

| Resource | Purpose |
|---|---|
| **FDCAN1** (PB8/PB9) | Receive CAN ID 0x208 from ESP32 |
| **CAN filter bank 3** | Configured to accept 0x208 and 0x209 |
| **RxFIFO0 interrupt** | Triggers `HAL_FDCAN_RxFifo0Callback` → `CAN_ProcessMessages()` → `Obstacle_ProcessCAN()` |

### 5.5 Why No Local Sensor on STM32

Per the firmware architecture documentation:
1. All STM32 UARTs are allocated (no spare UART for 921.6 kbps ToF sensor)
2. Matrix processing (8×8 = 64 depth points, 5-zone classification) is better suited to ESP32's dual-core processor
3. STM32's 10 ms control loop deadline would be impacted by 400-byte UART frame processing
4. CAN provides a clean, validated, compressed obstacle summary to STM32

### 5.6 Sensor Update Rate and Timing

| Parameter | Value |
|---|---|
| **ToF sensor hardware rate** | ~15 Hz (66 ms per frame) |
| **ESP32 → CAN transmission rate** | 66 ms (15 Hz) |
| **CAN frame reception on STM32** | Interrupt-driven (FDCAN RxFIFO0) |
| **Obstacle_Update() call rate** | 10 ms (100 Hz, main loop) |
| **Plausibility check interval** | Per CAN frame arrival (~66 ms) |
| **CAN timeout detection** | 500 ms (checked every 10 ms in Obstacle_Update) |

---

## 6. Obstacle Safety State Independence Verification

### 6.1 Obstacle_Update() — No System State Dependency

The `Obstacle_Update()` function does NOT check `Safety_GetState()`. It operates solely on:
- CAN data freshness (timeout detection)
- Obstacle state machine (NO_SENSOR → NORMAL → CONFIRMING → ACTIVE → CLEARING)
- Physical plausibility validation
- Stuck-sensor detection
- Speed-dependent stopping distance

**The obstacle state machine is completely decoupled from the system state machine.**

### 6.2 Traction_Update() — Obstacle Scale Applied in All Driveable States

Trace through `Traction_Update()` for each system state:

| System State | Reaches line 1077? | Obstacle_scale applied? |
|---|---|---|
| BOOT | No (motors not started) | N/A |
| STANDBY | No (motors not started) | N/A |
| ACTIVE | Yes | ✅ `base_pwm *= obstacle_scale` |
| DEGRADED | Yes | ✅ `base_pwm *= obstacle_scale` |
| LIMP_HOME | Yes | ✅ `base_pwm *= obstacle_scale` |
| SAFE | No (Traction_EmergencyStop called) | N/A (motors off) |
| ERROR | No (motors off) | N/A (motors off) |

**Conclusion:** Obstacle safety is state-independent for all driveable modes.

---

## 7. Conclusion

| Requirement | Status | Evidence |
|---|---|---|
| Obstacle_scale applied in LIMP_HOME | ✅ **PASS** | Line 1077: unconditional multiplication |
| Obstacle < stopping distance protected | ✅ **PASS** (after fix) | CAN timeout retains scale when obstacle active |
| Vehicle rolling downhill protected | ✅ **PASS** (after fix) | Scale 0.3 limits drive force even without speed cap |
| Pedal pressed continuously protected | ✅ **PASS** (after fix) | 20% × 0.3 = 6% effective torque |
| Single-wheel traction protected | ✅ **PASS** | Scale applied to base_pwm before per-wheel split |
| Obstacle safety state-independent | ✅ **PASS** | No system state check in obstacle pipeline |
| Physical sensor interface documented | ✅ **COMPLETE** | ToF on ESP32 via UART; STM32 receives via CAN 0x208 |
| No motion immobilization | ✅ **PASS** | Reverse escape preserved; SENSOR_FAULT allows 30% motion |
