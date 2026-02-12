# Regenerative Braking Validation — Audit Report

**Date**: 2026-02-12  
**Branch**: `feature/regenerative-braking-audit`  
**Scope**: STM32G474RE firmware (all `.c`/`.h` source files in `Core/Src` and `Core/Inc`)  
**Reference**: `github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos` (ESP32-S3 base firmware, read-only concept reference)  
**Type**: AUDIT ONLY — no code modifications

---

## 1) EXISTENCE CHECK

### Result: Regenerative braking is NOT implemented in current firmware.

A line-by-line review of every source file confirms there is **no regenerative braking logic** anywhere in the STM32 firmware.

**Files reviewed:**

| File | Regen references found |
|------|----------------------|
| `motor_control.c` (638 lines) | NONE |
| `safety_system.c` (~760 lines) | NONE |
| `sensor_manager.c` (511 lines) | NONE |
| `can_handler.c` (~466 lines) | NONE |
| `main.c` (~429 lines) | NONE |
| `ackermann.c` | NONE |
| `service_mode.c` | NONE |
| `steering_centering.c` | NONE |
| All `.h` headers | NONE |

**Specific absence evidence:**

- **No negative PWM logic**: `Traction_SetDemand()` (`motor_control.c:194-236`) processes throttle input through an EMA filter and ramp limiter. Although it accepts negative values (for reverse), there is no code path that uses negative torque for energy recovery. The demand simply maps to H-bridge direction + PWM magnitude.

- **No H-bridge inversion for braking**: `Motor_SetDirection()` (`motor_control.c:615-619`) sets a GPIO pin for forward/reverse. There is no mode where the direction pin is used to oppose wheel rotation for regenerative purposes.

- **No active braking mode of BTS7960**: The comment at `motor_control.c:288-290` mentions H-bridge active braking: _"ABS sets scale=0.0 but the motor must stay enabled so the H-bridge can actively brake (coast mode would lose control)."_ This refers to **dynamic friction braking** (shorting motor windings), NOT regenerative energy recovery. No back-EMF is captured or routed to the battery.

- **No motor back-EMF handling**: There is no code that monitors motor terminal voltage, detects back-EMF conditions, or manages current flow direction during deceleration.

- **No regen-related data structures**: Neither `WheelState_t` (`motor_control.h`) nor `TractionState_t` contain any fields for regen power, energy recovery, or bidirectional current tracking.

- **No regen CAN messages**: The CAN protocol (`can_handler.c`) defines no message IDs for regen status, regen power, or energy recovery feedback.

**Explicit statement**: Regenerative braking is not implemented in current firmware.

---

## 2) ACTIVATION CONDITIONS

### Result: NOT APPLICABLE — no regen implementation exists.

Since regenerative braking is not implemented, none of the following conditions are present:

| Condition | Status |
|-----------|--------|
| Regen activates when throttle < previous demand | ❌ NOT IMPLEMENTED |
| Deadband near zero throttle | ❌ NOT IMPLEMENTED (throttle deadband exists for motor control but not for regen) |
| Regen is speed-dependent | ❌ NOT IMPLEMENTED |
| Regen disabled below minimum wheel speed | ❌ NOT IMPLEMENTED |
| Per-wheel or global regen operation | ❌ NOT IMPLEMENTED |

**Note on existing pedal behavior**: `Traction_SetDemand()` (`motor_control.c:194-236`) includes an EMA filter (alpha=0.15) and ramp rate limiter (50 %/s up, 100 %/s down). When the driver releases the pedal, the ramp-down simply reduces motor torque toward zero — there is no regenerative braking phase during deceleration.

---

## 3) SAFETY VALIDATION

### Result: NOT APPLICABLE — no regen exists to validate safety around.

However, for completeness, the following safety states and conditions are confirmed to exist in the firmware. Any future regen implementation MUST disable regen in all of these:

| Safety condition | Code location | Disables motor output? | Would need to disable regen? |
|-----------------|---------------|----------------------|------------------------------|
| `SYS_STATE_SAFE` | `safety_system.c:115-121` | ✅ YES — calls `Safety_FailSafe()` | ✅ YES |
| `SYS_STATE_ERROR` | `safety_system.c:124-127` | ✅ YES — calls `Safety_PowerDown()` | ✅ YES |
| Emergency stop | `safety_system.c:731-740` | ✅ YES — `Traction_EmergencyStop()` + relay off | ✅ YES |
| CAN timeout | `safety_system.c:578-583` | ✅ YES — transitions to SAFE | ✅ YES |
| Encoder fault | `safety_system.c:704-727` | ✅ Steering only — transitions to DEGRADED | ✅ YES |
| Overtemperature (>90°C critical) | `safety_system.c:541-544` | ✅ YES — transitions to SAFE | ✅ YES |
| Overtemperature (>80°C warning) | `safety_system.c:547-551` | Partial — transitions to DEGRADED (40% power limit) | ✅ YES |
| Overcurrent (>25A) | `safety_system.c:478-502` | Partial — DEGRADED (first), SAFE (after 3 consecutive) | ✅ YES |
| DEGRADED mode | `safety_system.c:106-113` | Reduced (40% power, 50% speed via `Safety_GetPowerLimitFactor()`) | ⚠️ Must limit or disable regen |

---

## 4) BATTERY PROTECTION

### Result: NO battery protection for regenerative braking exists.

**CRITICAL FINDING**: The firmware has no battery-side protections that would be necessary for safe regenerative braking.

| Protection | Status | Detail |
|-----------|--------|--------|
| Battery voltage monitoring before regen | ❌ NOT IMPLEMENTED | `Voltage_GetBus()` (`sensor_manager.c:190-193`) is implemented and reads INA226 bus voltage, but it is **never called** from any module. |
| Upper voltage cutoff | ❌ NOT IMPLEMENTED | No maximum battery voltage check exists anywhere. |
| INA226 battery channel for regen current limiting | ❌ NOT IMPLEMENTED | INA226 channel 4 (`INA226_CHANNEL_BATTERY`, `sensor_manager.c:173`) is read with correct shunt resistance (0.5 mΩ for 100A sensor), but the value is only stored — never used for any protective logic. |
| Maximum regen current limit | ❌ NOT IMPLEMENTED | No regen current concept exists. |
| Regen disabled when battery near full | ❌ NOT IMPLEMENTED | No battery SOC estimation or full-charge detection exists. |

**Specific evidence:**

1. `Voltage_GetBus()` is defined at `sensor_manager.c:190-193` and reads the INA226 bus voltage register. The data is stored in `voltage_bus[]` and updated every 50 ms via `Current_ReadAll()`. However, **no function in the entire firmware calls `Voltage_GetBus()`**.

2. `Current_GetAmps(4)` (battery channel) is readable but only used for overcurrent checks in `Safety_CheckCurrent()` (`safety_system.c:478-519`), which triggers at >25A. There is no bidirectional current monitoring.

3. There is no battery SOC (State of Charge) estimation — no voltage-to-SOC mapping, no coulomb counting, no battery management of any kind.

**Explicit statement**: None of these battery protections exist.

---

## 5) INTERACTION WITH ABS/TCS

### Result: NOT APPLICABLE — no regen exists to interact with ABS/TCS.

For future implementation reference, the current ABS/TCS architecture is documented:

| ABS/TCS feature | Code location | Regen interaction concern |
|----------------|---------------|--------------------------|
| ABS per-wheel scale (`wheel_scale[]`) | `safety_system.c:338-351` | Regen must NOT apply braking torque to ABS-active wheels. `wheel_scale[i] = 0.0` means the wheel is locking — regen would worsen lockup. |
| TCS per-wheel reduction | `safety_system.c:422-454` | Regen during TCS would conflict — TCS reduces torque for spinning wheels; regen adds braking torque. These interact differently per wheel. |
| Global ABS fallback (all 4 wheels) | `safety_system.c:360-362` | When `mask == 0x0F` (all wheels locking), `Traction_SetDemand(0)` is called. Any regen must also be disabled in this condition. |
| Traction_Update wheel modulation | `motor_control.c:258,270-277` | PWM is modulated by `wheel_scale[i]`. Regen torque would need the same per-wheel modulation. |

**Key risk**: The current ABS implementation sets `wheel_scale = 0.0` for locking wheels (`motor_control.c:288-290` comment confirms motor stays enabled for H-bridge active brake). Regenerative braking would add opposing torque to wheels, which could trigger or worsen wheel lockup. Any regen implementation must check `ABS_IsActive()` and disable regen on affected wheels.

---

## 6) COMPARISON WITH BASE FIRMWARE

### Reference: `FULL-FIRMWARE-Coche-Marcos` (`src/safety/regen_ai.cpp`, `include/regen_ai.h`)

The base firmware (ESP32-S3) implements an AI-based regenerative braking system. Conceptual comparison:

| Feature | Base firmware (`regen_ai.cpp`) | STM32 firmware |
|---------|-------------------------------|----------------|
| **Regen implemented** | ✅ YES — full `RegenAI` namespace | ❌ NO |
| **Speed-based activation** | ✅ YES — minimum 5 km/h; lookup table by speed bins (0-10, 10-30, 30-50, 50-70, 70+ km/h) | ❌ N/A |
| **Deceleration-based activation** | ✅ YES — only when `acceleration < -0.1 m/s²`; lookup table by deceleration bins | ❌ N/A |
| **Voltage protected** | ✅ YES — battery SOC estimated from voltage; regen disabled when SOC > 95% (`batterySOCMax`) | ❌ N/A (`Voltage_GetBus()` exists but is never called) |
| **Temperature protected** | ✅ YES — regen disabled when battery temp > 45°C (`tempMax`); gradual reduction above 35°C | ❌ N/A |
| **SOC lower limit** | ✅ YES — regen disabled below 20% SOC (`batterySOCMin`) | ❌ N/A |
| **Max regen power** | ✅ YES — capped at 80% (`maxRegenPower`) with aggressiveness factor (0.7) | ❌ N/A |
| **Limp/degraded mode limits** | Conceptually present (regen disabled when system not in normal mode) | ❌ N/A (DEGRADED mode exists but no regen to limit) |
| **Energy recovery tracking** | ✅ YES — cumulative Wh tracking (`energyRecovered`) | ❌ N/A |
| **Per-wheel application** | Not directly — base firmware calculates global regen power | ❌ N/A |
| **AI/lookup model** | ✅ YES — 5×4 lookup table (speed × deceleration) with SOC and temperature correction factors | ❌ N/A |

**Key conceptual differences**:

1. The base firmware regen is an **ESP32-S3 software feature** that runs in a different MCU context. It uses Arduino-style `millis()`, `constrain()`, and sensor abstractions (`Sensors::getWheelSpeed()`, `Sensors::getVoltage()`, `Sensors::getTemperature()`).

2. The STM32 firmware has the **hardware interfaces** that would be needed for regen (INA226 current/voltage sensors, wheel speed sensors, temperature sensors) but lacks all the regen software logic.

3. The base firmware regen model assumes a higher-voltage battery system (36-42V range for SOC calculation), while the STM32 hardware operates at 24V traction / 12V steering.

---

## 7) FINAL VERDICT

| Criterion | Assessment |
|-----------|------------|
| **Implementation status** | **NONE** — Regenerative braking is completely absent from the STM32 firmware. No code, no data structures, no CAN messages, no activation logic. |
| **Safety level** | **N/A (SAFE by absence)** — Since regen is not implemented, there is no regen-related safety risk. The vehicle decelerates by reducing motor torque to zero (coast) or by H-bridge dynamic braking (shorting windings). No energy is fed back to the battery. |
| **Risk level** | **LOW** (current state) — The absence of regen means there is no risk of battery overvoltage, overcurrent during charging, or unintended braking torque. However, if regen were to be added without the protections documented in this audit, risk would be **HIGH**. |
| **Recommended next action** | See below |

### Recommended Next Actions

If regenerative braking is to be implemented in the STM32 firmware:

1. **Battery voltage monitoring must be activated first**: `Voltage_GetBus()` exists but is never called. Add voltage checks in the safety system with upper cutoff (e.g., 28.8V for a 24V lead-acid system).

2. **Battery current limiting must be added**: Use INA226 channel 4 (battery, 0.5 mΩ shunt) to limit regen current. Define maximum regen current based on battery chemistry and wiring capacity.

3. **ABS/TCS interaction must be designed**: Regen must be disabled on any wheel where `wheel_scale[i] < 1.0` (ABS/TCS active). Regen must be globally disabled when `ABS_IsActive()` returns true.

4. **State machine integration**: Regen must be disabled in `SYS_STATE_SAFE`, `SYS_STATE_ERROR`, `SYS_STATE_BOOT`, `SYS_STATE_STANDBY`, and limited in `SYS_STATE_DEGRADED`.

5. **Speed threshold**: Regen must be disabled below a minimum speed (e.g., 5 km/h per base firmware) to prevent stalling at low speeds.

6. **CAN reporting**: Add a CAN message for regen status (power, energy recovered, active/inactive) for HMI display.

7. **Hardware verification required**: Confirm that the BTS7960 H-bridge drivers used on this vehicle support regenerative current flow (motor → battery) and that the power path can handle bidirectional current.

---

## Summary Table

```
┌──────────────────────────────┬──────────────────────────────────────┐
│ Audit Item                   │ Result                               │
├──────────────────────────────┼──────────────────────────────────────┤
│ 1. Existence                 │ NOT IMPLEMENTED                      │
│ 2. Activation conditions     │ N/A — no regen exists                │
│ 3. Safety validation         │ N/A — safety framework exists,       │
│                              │ regen not integrated                 │
│ 4. Battery protection        │ NO protections exist for regen       │
│    - Voltage monitoring      │ Code exists, never called            │
│    - Current limiting        │ Battery INA226 read, not used        │
│    - SOC tracking            │ Not implemented                      │
│ 5. ABS/TCS interaction       │ N/A — no regen to interact           │
│ 6. Base firmware comparison  │ Base has full AI regen; STM32 = NONE │
│ 7. Final verdict             │ NONE / SAFE(absent) / LOW risk       │
└──────────────────────────────┴──────────────────────────────────────┘
```

---

*Audit generated by line-by-line analysis of the STM32 firmware source code. All file references, function names, and line numbers are verifiable in the repository. Base firmware reference: `github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos` (read-only, no code copied).*
