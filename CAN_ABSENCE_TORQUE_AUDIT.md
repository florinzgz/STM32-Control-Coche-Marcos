# CAN Absence Torque Audit — STM32G474RE Firmware

**Date:** 2026-02-19  
**Scope:** Determine if the STM32G474RE control firmware allows motor torque generation WITHOUT any CAN bus connected (ESP32 absent).  
**Method:** Static analysis of source code only — no modifications.

---

## 1. State Machine Initial State

**File:** `Core/Src/safety_system.c`, line 70 & 499  
```c
static SystemState_t system_state = SYS_STATE_BOOT;
```

At `Safety_Init()` (line 465–499), `system_state` is explicitly reset to `SYS_STATE_BOOT`.

Immediately after all module initialisations complete in `main()` (line 121):
```c
Safety_SetState(SYS_STATE_STANDBY);
```

The `Safety_SetState()` function (line 164–226) permits `BOOT → STANDBY` (line 172–174):
```c
case SYS_STATE_STANDBY:
    if (system_state == SYS_STATE_BOOT)
        system_state = SYS_STATE_STANDBY;
    break;
```

**Result:** After power-up, the system enters **`SYS_STATE_STANDBY`** and remains there until transition conditions are satisfied.

---

## 2. Every Transition Condition to ACTIVE/DRIVE

There is exactly **one code path** that transitions from `STANDBY → ACTIVE`. It resides in `Safety_CheckCANTimeout()` (`Core/Src/safety_system.c`, lines 852–893):

```c
void Safety_CheckCANTimeout(void)
{
    if ((HAL_GetTick() - last_can_rx_time) > CAN_TIMEOUT_MS) {
        // TIMEOUT PATH — sets SAFE state
        ServiceMode_SetFault(MODULE_CAN_TIMEOUT, MODULE_FAULT_ERROR);
        Safety_SetError(SAFETY_ERROR_CAN_TIMEOUT);
        Safety_SetState(SYS_STATE_SAFE);
    } else {
        // HEARTBEAT ALIVE PATH — only path to ACTIVE
        ServiceMode_ClearFault(MODULE_CAN_TIMEOUT);
        if (system_state == SYS_STATE_STANDBY &&
            safety_error == SAFETY_ERROR_NONE &&
            Steering_IsCalibrated() &&
            BootValidation_IsPassed()) {
            Safety_SetState(SYS_STATE_ACTIVE);      // ← ONLY STANDBY→ACTIVE gate
        }
        ...
    }
}
```

### Transition conditions (all must be true simultaneously):

| # | Condition | Code Reference | CAN Dependency |
|---|-----------|---------------|----------------|
| 1 | `(HAL_GetTick() - last_can_rx_time) <= CAN_TIMEOUT_MS` | safety_system.c:854 | **YES — requires CAN heartbeat within 250 ms** |
| 2 | `system_state == SYS_STATE_STANDBY` | safety_system.c:863 | No |
| 3 | `safety_error == SAFETY_ERROR_NONE` | safety_system.c:864 | No |
| 4 | `Steering_IsCalibrated() == true` | safety_system.c:865 | No |
| 5 | `BootValidation_IsPassed() == true` | safety_system.c:866 | No |

### Additional ACTIVE transitions (recovery paths):

| Transition | Condition | Code Reference | CAN Dependency |
|-----------|-----------|---------------|----------------|
| `SAFE → ACTIVE` | CAN alive AND `safety_error == SAFETY_ERROR_CAN_TIMEOUT` | safety_system.c:870–874 | **YES** |
| `DEGRADED → ACTIVE` | CAN alive AND `safety_error == SAFETY_ERROR_NONE` AND debounce (500 ms) | safety_system.c:880–888 | **YES** |

**All transitions to ACTIVE are gated inside the `else` branch of the CAN timeout check (line 858), meaning CAN heartbeat reception within 250 ms is a mandatory prerequisite.**

---

## 3. Dependency Classification

### 3.1 CAN Heartbeat Dependency

**`last_can_rx_time`** is initialised to `HAL_GetTick()` at `Safety_Init()` (line 483):
```c
last_can_rx_time = HAL_GetTick();
```

It is **only ever updated** by `Safety_UpdateCANRxTime()` (line 896–898):
```c
void Safety_UpdateCANRxTime(void)
{
    last_can_rx_time = HAL_GetTick();
}
```

Which is **only called** from `CAN_ProcessMessages()` when a `CAN_ID_HEARTBEAT_ESP32` (0x011) frame is received (`Core/Src/can_handler.c`, lines 479–482):
```c
case CAN_ID_HEARTBEAT_ESP32:
    can_stats.last_heartbeat_esp32 = HAL_GetTick();
    Safety_UpdateCANRxTime();
    break;
```

**Conclusion:** Without ESP32 CAN frames, `last_can_rx_time` is never refreshed after init. After the initial 250 ms grace period (from the init-time seed), the CAN timeout will fire permanently.

### 3.2 CAN Command Dependency

Throttle commands are received via `CAN_ID_CMD_THROTTLE` (0x100) in `CAN_ProcessMessages()` (line 484–489). Without CAN, no throttle command is received from ESP32.

However, the main loop also feeds pedal demand directly (main.c lines 190–199):
```c
if (Safety_IsCommandAllowed()) {
    ...
    float validated = Safety_ValidateThrottle(Pedal_GetPercent());
    Traction_SetDemand(validated);
} else {
    Traction_SetDemand(0.0f);    // ← This path is taken when not ACTIVE/DEGRADED
}
```

`Safety_IsCommandAllowed()` returns `true` only in `ACTIVE` or `DEGRADED` (line 228–232):
```c
bool Safety_IsCommandAllowed(void)
{
    return (system_state == SYS_STATE_ACTIVE ||
            system_state == SYS_STATE_DEGRADED);
}
```

**Since the system never reaches ACTIVE without CAN, the `else` branch executes, forcing `Traction_SetDemand(0.0f)` — zero torque.**

### 3.3 HMI Acknowledgement Dependency

There is **no** explicit HMI acknowledgement requirement for the `STANDBY → ACTIVE` transition. The ESP32 heartbeat (0x011) serves as the implicit "HMI present" signal. No CAN command frame or special handshake is required — only the periodic heartbeat.

---

## 4. Missing CAN Traffic Effects

### 4.1 Timeout Fault

**Timeline without CAN after power-up:**

1. `Safety_Init()` sets `last_can_rx_time = HAL_GetTick()` (boot time seed).
2. System transitions `BOOT → STANDBY` at main.c line 121.
3. `Safety_CheckCANTimeout()` runs every 10 ms (main.c line 141).
4. After **250 ms** (`CAN_TIMEOUT_MS = 250`, defined in safety_system.c line 25):
   - `(HAL_GetTick() - last_can_rx_time) > 250` becomes **true**.
   - `Safety_SetError(SAFETY_ERROR_CAN_TIMEOUT)` fires (line 856).
   - `Safety_SetState(SYS_STATE_SAFE)` fires (line 857).

The system transitions **STANDBY → SAFE** within 250 ms of boot if no ESP32 heartbeat arrives.

### 4.2 SAFE State Effects

When entering `SYS_STATE_SAFE` (safety_system.c lines 205–213):
```c
case SYS_STATE_SAFE:
    ...
    system_state = SYS_STATE_SAFE;
    Safety_FailSafe();
```

`Safety_FailSafe()` (line 1112–1127):
```c
void Safety_FailSafe(void)
{
    Traction_EmergencyStop();    // All motor PWMs → 0, all enables → OFF
    ...
}
```

### 4.3 Torque Inhibition (Multiple Independent Barriers)

Even if the system were somehow in SAFE/STANDBY, torque is inhibited at multiple levels:

| Barrier | Code Path | Effect |
|---------|-----------|--------|
| **State gate** | `Safety_IsCommandAllowed()` returns `false` in SAFE/STANDBY | `Traction_SetDemand(0.0f)` (main.c:199) |
| **Throttle validation** | `Safety_ValidateThrottle()` → returns `0.0f` if not ACTIVE/DEGRADED | Zero throttle (safety_system.c:405) |
| **Power limit factor** | `Safety_GetPowerLimitFactor()` → returns `0.0f` for states other than ACTIVE/DEGRADED | Multiplier = 0 (safety_system.c:255) |
| **FailSafe call** | `Safety_FailSafe()` → `Traction_EmergencyStop()` | PWM = 0, motor enables = OFF |
| **Relay sequencing** | `Relay_PowerUp()` is only called inside `Safety_SetState(SYS_STATE_ACTIVE)` (line 187) | Power relays never energised without ACTIVE |

### 4.4 Degraded Mode

Degraded mode is **not reachable** without first having been in ACTIVE or STANDBY with CAN alive. The `DEGRADED` state is entered from `ACTIVE` (non-critical fault) or `STANDBY` (safety_system.c lines 196–203):
```c
case SYS_STATE_DEGRADED:
    if (system_state == SYS_STATE_ACTIVE ||
        system_state == SYS_STATE_STANDBY) {
        system_state = SYS_STATE_DEGRADED;
    }
    break;
```

However, even if `STANDBY → DEGRADED` occurs due to a sensor fault, the system would still be moved to `SAFE` within 250 ms by the CAN timeout check (which fires regardless of current state, line 854–857).

### 4.5 Recovery Impossible

Recovery from `SAFE → ACTIVE` (line 870–874) requires:
```c
if (system_state == SYS_STATE_SAFE &&
    safety_error == SAFETY_ERROR_CAN_TIMEOUT) {
    Safety_ClearError(SAFETY_ERROR_CAN_TIMEOUT);
    Safety_SetState(SYS_STATE_ACTIVE);
}
```

This code is inside the `else` branch (CAN alive). Without CAN, this branch is never entered. The system **cannot recover from SAFE without a live ESP32 heartbeat**.

---

## 5. Conclusion

### **C) Vehicle will never produce torque**

Without any CAN bus connected (ESP32 absent), the STM32G474RE firmware will **never** produce motor torque. The complete execution path is:

1. **Power-on:** `SYS_STATE_BOOT` (safety_system.c:499)
2. **Init complete:** `SYS_STATE_STANDBY` (main.c:121)
3. **≤ 250 ms later:** `Safety_CheckCANTimeout()` fires → `SAFETY_ERROR_CAN_TIMEOUT` → `SYS_STATE_SAFE` (safety_system.c:854–857)
4. **SAFE triggers:** `Safety_FailSafe()` → `Traction_EmergencyStop()` → all PWM = 0, all motor enables OFF (safety_system.c:1112–1114)
5. **Perpetual SAFE:** No recovery possible because recovery requires CAN heartbeat reception (safety_system.c:870–874, inside `else` branch of timeout check)
6. **Main loop torque gate:** `Safety_IsCommandAllowed()` returns `false` → `Traction_SetDemand(0.0f)` every 50 ms (main.c:199)
7. **Power relays never energised:** `Relay_PowerUp()` only called during `Safety_SetState(SYS_STATE_ACTIVE)` transition (safety_system.c:187), which never occurs

The firmware implements **five independent torque-inhibition barriers** that all prevent motor output when CAN is absent. The system is safe-by-design: CAN heartbeat is the mandatory "ESP32 alive" signal that gates all actuator authority.
