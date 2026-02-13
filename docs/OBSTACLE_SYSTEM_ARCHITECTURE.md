# Obstacle System Architecture — Split Design

**Date:** 2026-02-13
**Status:** IMPLEMENTED
**Scope:** Obstacle safety integration for STM32G474RE + ESP32-S3 split architecture

---

## 1. Architecture Decision

### Selected: Option C — Basic CAN-based obstacle limiter on STM32 + full obstacle detection stack on ESP32

The obstacle system is split between the ESP32-S3 (sensor + logic) and STM32G474RE (safety backstop):

- **ESP32-S3**: Runs the full obstacle detection stack (UART driver, TOFSense-M S sensor parsing, 8×8 matrix processing, 5-zone logic, child reaction detection, ACC PID). Transmits processed distance and zone data to the STM32 via CAN (IDs 0x208/0x209).
- **STM32G474RE**: Receives CAN obstacle data and applies a simplified 3-tier backstop limiter through the existing torque pipeline. Enforces SAFE state for emergency distances, CAN timeouts, sensor failures, and stale data.

### Rationale

1. **No hardware changes required.** Sensor stays on ESP32 UART0.
2. **Preserves safety authority.** STM32 makes final motor control decisions.
3. **Defence in depth.** Three independent safety layers:
   - Layer 1: ESP32 obstacle system (5-zone logic, checksum validation)
   - Layer 2: STM32 obstacle backstop (simplified distance check, CAN timeout)
   - Layer 3: STM32 safety state machine (SAFE state, relay control, watchdog)
4. **Minimal firmware changes.** ~200 lines of new C code on STM32.
5. **Backward compatible.** STM32 operates normally without obstacle messages until the first 0x208 is received.

### Alternatives Rejected

| Option | Reason for Rejection |
|--------|---------------------|
| A) Full port to STM32 | Requires physical rewiring of sensor, high effort, loses ESP32 HMI display |
| B) Full stack on ESP32 only | STM32 has no safety backstop, violates safety authority principle |
| D) Dual sensor | Hardware modification required, UART bus contention risk |

---

## 2. CAN Message Definitions

### 2.1 OBSTACLE_DISTANCE (0x208) — ESP32 → STM32

| Byte | Field | Type | Unit | Range | Description |
|------|-------|------|------|-------|-------------|
| 0 | distance_LSB | uint8 | mm | — | Low byte of minimum obstacle distance |
| 1 | distance_MSB | uint8 | mm | — | High byte of minimum obstacle distance |
| 2 | zone | uint8 | — | 0–5 | Obstacle proximity zone |
| 3 | sensor_health | uint8 | — | 0–1 | 0 = unhealthy, 1 = healthy |
| 4 | counter | uint8 | — | 0–255 | Rolling counter (must increment) |

- **Rate:** 66 ms (15 Hz, matching TOFSense-M S sensor update rate)
- **DLC:** 5 bytes minimum

### 2.2 OBSTACLE_SAFETY (0x209) — ESP32 → STM32

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | speedReductionFactor | uint8 | ×100 (0–100). Informational only. |
| 1 | emergencyBrakeApplied | uint8 | 1 = ESP32 emergency brake active |
| 2 | collisionImminent | uint8 | 1 = ESP32 collision risk detected |
| 3 | obstacleZone | uint8 | ESP32 zone (0–5) |
| 4 | childReactionDetected | uint8 | 1 = child reaction detected |
| 5–7 | reserved | uint8 | Always 0x00 |

- **Rate:** 100 ms (10 Hz)
- **DLC:** 8 bytes
- **Note:** Informational only. STM32 does NOT use these values for safety decisions. Reserved for future coordination.

---

## 3. Safety State Interactions

### State Machine Integration

The obstacle system integrates into the existing STM32 safety state machine:

```
BOOT → STANDBY → ACTIVE ⇄ DEGRADED → SAFE → ERROR

Obstacle triggers:
  Distance < 200 mm         → ACTIVE/DEGRADED → SAFE
  CAN timeout (> 500 ms)    → ACTIVE/DEGRADED → SAFE
  Sensor unhealthy           → ACTIVE/DEGRADED → SAFE
  Stale data (≥ 3 frames)   → ACTIVE/DEGRADED → SAFE

Recovery:
  Distance > 500 mm for > 1 s + healthy sensor → SAFE → ACTIVE
```

### Interaction with Existing Safety Checks

- **CAN heartbeat timeout (250 ms):** Independent. ESP32 heartbeat loss triggers SAFE state regardless of obstacle data.
- **ABS/TCS:** Independent per-wheel modulation. Obstacle_scale is applied before wheel_scale[], so effects are multiplicative.
- **Overcurrent/Overtemperature:** Independent. These faults take priority (set their own error codes).
- **Battery undervoltage:** Independent. Battery faults take priority.
- **Service Mode:** Obstacle detection can be disabled via `MODULE_OBSTACLE_DETECT` (module ID 24). When disabled, obstacle_scale = 1.0 (no reduction).

---

## 4. Failure Mode Analysis

| Failure Mode | Detection | Response | Recovery |
|-------------|-----------|----------|----------|
| CAN timeout (no 0x208 for > 500 ms) | `Obstacle_Update()` timestamp check | obstacle_scale = 0.0, SAFE state | Auto-recover when 0x208 resumes with incrementing counter |
| ESP32 crash | Heartbeat timeout (250 ms) → SAFE | Independent of obstacle system | ESP32 must reboot and resume heartbeat |
| Sensor UART failure | ESP32 reports `sensor_health = 0` in 0x208 | obstacle_scale = 0.0, SAFE state | Auto-recover when ESP32 reports healthy sensor |
| Stale data (frozen counter) | Counter not incrementing for ≥ 3 frames | obstacle_scale = 0.0, SAFE state | Auto-recover when counter resumes incrementing |
| CAN spoofing (false distance) | Not directly detectable | Rolling counter provides sequence integrity | CAN bus is point-to-point; physical access required |
| Distance near boundary (200 mm) | Hysteresis: trigger at 200 mm, recover at 500 mm | Prevents oscillation | 1 s debounce before recovery |
| ESP32 sends cached data | Counter frozen detection | SAFE state after 3 stale frames | Counter must resume incrementing |

### CAN Spoofing Mitigation

The current design does not implement cryptographic CAN message authentication. Mitigations:

1. **Point-to-point topology:** Only ESP32 and STM32 are on the CAN bus. Physical access to the bus connector is required for spoofing.
2. **Rolling counter:** Provides basic sequence integrity. An attacker would need to match the expected counter value.
3. **Hardware RX filters:** STM32 only accepts white-listed CAN IDs at the hardware level.
4. **Future enhancement:** CMAC authentication could be added if the threat model requires it.

---

## 5. Timeout Strategy

| Timeout | Value | Trigger | Response |
|---------|-------|---------|----------|
| ESP32 heartbeat | 250 ms | No 0x011 received | SAFE state (existing behavior) |
| Obstacle CAN data | 500 ms | No 0x208 received (after first reception) | obstacle_scale = 0.0, SAFE state |
| Stale data | 3 consecutive frames | Counter not changing | obstacle_scale = 0.0, SAFE state |

### Why 500 ms for obstacle timeout?

- Obstacle messages are sent at 66 ms (15 Hz). A 500 ms timeout allows 7+ missed frames before declaring failure.
- This is longer than the heartbeat timeout (250 ms) because obstacle data is less critical than overall ESP32 liveness.
- The heartbeat timeout (250 ms) provides a faster backstop — if the ESP32 crashes entirely, the heartbeat timeout triggers first.

### First-message grace period

Before the first 0x208 message is received, the STM32 sets `obstacle_scale = 1.0` (no reduction). This allows normal operation during:
- ESP32 boot sequence (obstacle module may not be initialized yet)
- Operation without obstacle sensor hardware installed
- CAN contract revision 1.0 ESP32 firmware (does not send 0x208)

---

## 6. Recovery Strategy

### Distance-Based Recovery

| Condition | Threshold | Debounce |
|-----------|-----------|----------|
| Trigger emergency | distance < 200 mm | Immediate |
| Recovery from emergency | distance > 500 mm | 1 second sustained |

The 200 mm → 500 mm hysteresis band (300 mm gap) prevents oscillation when an obstacle is near the trigger boundary. The 1-second debounce ensures the obstacle has truly cleared before resuming motor operation.

### CAN Timeout Recovery

When obstacle CAN messages resume after a timeout:
1. `obstacle_data_valid` is set to 1
2. `obstacle_last_rx_tick` is updated
3. Rolling counter stale detection is reset
4. If distance > 500 mm and sensor healthy, recovery debounce starts
5. After 1 second of sustained clearance, obstacle_scale returns to 1.0

### Service Mode Override

The obstacle detection module (`MODULE_OBSTACLE_DETECT`, ID 24) can be disabled via CAN service command (0x110). When disabled:
- `obstacle_scale` is forced to 1.0
- No CAN timeout detection is performed
- The operator assumes responsibility for obstacle awareness

---

## 7. Torque Pipeline Formula

### Final PWM calculation with obstacle integration:

```
FinalPWM[i] = base_pwm × obstacle_scale × wheel_scale[i]

Where:
  raw_pedal       = ADC_ReadPedal()                         (STM32 ADC)
  filtered_pedal  = EMA(raw_pedal, α=0.15)                  (noise filter)
  ramped_pedal    = ramp_limit(filtered, 50%/s↑, 100%/s↓)   (rate limit)
  gear_scale      = 0.6 (D1), 1.0 (D2), 0.6 (R)            (gear power)

  --- Dynamic Braking (if throttle dropping) ---
  dynamic_brake   = |throttle_rate| × 0.5                    (max 60%)
  effective_demand = ramped_pedal - dynamic_brake             (can go negative)

  --- Base PWM ---
  base_pwm        = |effective_demand| × (PWM_PERIOD / 100) × gear_scale

  --- Obstacle Scale (uniform, all wheels) ---
  obstacle_scale  = f(CAN_distance):
    distance < 200 mm   → 0.0 + SAFE state
    distance 200-500    → 0.3
    distance 500-1000   → 0.7
    distance > 1000     → 1.0
    CAN timeout         → 0.0 + SAFE state
    sensor unhealthy    → 0.0 + SAFE state

  --- Per-Wheel Safety Scale ---
  wheel_scale[i]  = min(ABS_scale[i], TCS_scale[i])
    ABS_scale[i]  = 0.0 (locking) or 1.0 (normal)
    TCS_scale[i]  = 1.0 → progressive reduction (max 0.2)

  --- Final ---
  FinalPWM[i]     = base_pwm × obstacle_scale × wheel_scale[i]
```

### Integration point in code

In `Traction_Update()` (motor_control.c), `obstacle_scale` is applied to `base_pwm` after gear scaling and before per-wheel `wheel_scale[]` multiplication:

```c
uint16_t base_pwm = (uint16_t)(fabs(effective_demand) * PWM_PERIOD / 100.0f);
base_pwm = (uint16_t)(base_pwm * safety_status.obstacle_scale);  // ← NEW
// Then per-wheel: pwm = base_pwm * safety_status.wheel_scale[i]
```

---

## 8. Split Responsibility Diagram

```
┌─────────────────────────────────────────────────┐
│                   ESP32-S3                       │
│                                                  │
│  ┌──────────────┐   ┌──────────────────────────┐│
│  │ TOFSense-M S │   │ Obstacle Detection       ││
│  │ LiDAR Sensor ├──►│ • UART parser (921.6kbps)││
│  │ (UART0)      │   │ • 8×8 matrix processing  ││
│  └──────────────┘   │ • Checksum validation    ││
│                     │ • Min distance extraction ││
│                     └───────┬──────────────────┘│
│                             │                    │
│  ┌──────────────────────────▼──────────────────┐│
│  │ Obstacle Safety (5-zone logic)               ││
│  │ • Zone 1-5 determination                     ││
│  │ • speedReductionFactor calculation            ││
│  │ • Child reaction detection                    ││
│  │ • ACC coordination                            ││
│  │ • Audio/visual alerts                         ││
│  └───────┬──────────────────────────────────────┘│
│          │                                       │
│  ┌───────▼──────────────────────────────────────┐│
│  │ CAN TX (0x208, 0x209)                         ││
│  │ • distance_mm + zone + health + counter       ││
│  │ • speedReductionFactor (informational)        ││
│  └───────┬──────────────────────────────────────┘│
└──────────┼───────────────────────────────────────┘
           │ CAN Bus 500 kbps
           │
┌──────────▼───────────────────────────────────────┐
│                 STM32G474RE                        │
│                                                    │
│  ┌─────────────────────────────────────────────┐  │
│  │ CAN RX (filter bank 3: 0x208–0x209)          │  │
│  │ • Parse distance, zone, health, counter       │  │
│  │ • Update obstacle state variables             │  │
│  └───────┬─────────────────────────────────────┘  │
│          │                                         │
│  ┌───────▼─────────────────────────────────────┐  │
│  │ Obstacle_Update() — 10 ms cycle              │  │
│  │ • CAN timeout detection (500 ms)             │  │
│  │ • Stale-data detection (counter frozen ≥ 3)  │  │
│  │ • Sensor health check                        │  │
│  │ • 3-tier distance → obstacle_scale mapping   │  │
│  │ • Emergency recovery with hysteresis         │  │
│  │ • SAFE state transition for emergencies      │  │
│  └───────┬─────────────────────────────────────┘  │
│          │                                         │
│  ┌───────▼─────────────────────────────────────┐  │
│  │ Traction_Update() — 10 ms cycle              │  │
│  │ • base_pwm × obstacle_scale × wheel_scale[i]│  │
│  │ • ABS/TCS modulation (independent)           │  │
│  │ • Dynamic braking (independent)              │  │
│  └─────────────────────────────────────────────┘  │
│                                                    │
│  ┌─────────────────────────────────────────────┐  │
│  │ Safety State Machine                         │  │
│  │ • SAFE state enforced for obstacle emergency │  │
│  │ • Relay control (independent safety layer)   │  │
│  │ • Watchdog (IWDG 500 ms)                     │  │
│  └─────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────┘
```

### What stays on ESP32 (NOT implemented on STM32)

- TOFSense-M S UART driver and frame parser
- 8×8 pixel matrix processing
- Full 5-zone logic with linear interpolation
- Child reaction detection
- Adaptive Cruise Control (ACC) PID controller
- Audio alerts (DFPlayer Mini)
- Obstacle display overlay on TFT
- speedReductionFactor computation

### What is implemented on STM32

- CAN RX handler for 0x208 (obstacle distance)
- CAN RX handler for 0x209 (informational, not parsed)
- CAN timeout detection (500 ms)
- Stale-data detection (rolling counter)
- 3-tier distance-to-scale backstop limiter
- obstacle_scale integration into torque pipeline
- SAFE state transition for emergency conditions
- Recovery with hysteresis and debounce
- Service mode enable/disable for obstacle module

---

## 9. Files Modified

| File | Changes |
|------|---------|
| `Core/Inc/safety_system.h` | Added `SAFETY_ERROR_OBSTACLE` (code 12), `obstacle_scale` field in `SafetyStatus_t`, `Obstacle_Update()` / `Obstacle_ProcessCAN()` / `Obstacle_GetScale()` prototypes |
| `Core/Src/safety_system.c` | Added obstacle state variables, `Obstacle_ProcessCAN()`, `Obstacle_Update()`, `Obstacle_GetScale()` functions, obstacle initialization in `Safety_Init()` |
| `Core/Inc/can_handler.h` | Added `CAN_ID_OBSTACLE_DISTANCE` (0x208), `CAN_ID_OBSTACLE_SAFETY` (0x209), `CAN_TIMEOUT_OBSTACLE_MS` (500) |
| `Core/Src/can_handler.c` | Added CAN RX filter bank 3 (0x208–0x209), added obstacle message handlers in `CAN_ProcessMessages()` |
| `Core/Src/motor_control.c` | Applied `obstacle_scale` to `base_pwm` in `Traction_Update()` |
| `Core/Src/main.c` | Added `Obstacle_Update()` call in 10 ms tier |
| `esp32/include/can_ids.h` | Added `OBSTACLE_DISTANCE` (0x208), `OBSTACLE_SAFETY` (0x209), `SafetyError::OBSTACLE` (12), timing constants |
| `docs/CAN_CONTRACT_FINAL.md` | Updated to revision 1.1: added obstacle CAN IDs, payload definitions, timeout behavior, validation rules, safety error code 12 |
| `docs/FIRMWARE_MATURITY_ROADMAP.md` | Updated obstacle system status, safety gap analysis, risk classification |
| `docs/OBSTACLE_SYSTEM_ARCHITECTURE.md` | This document — full architecture documentation |

---

*Document generated: 2026-02-13*
*This document describes the obstacle system as actually implemented in code.*
