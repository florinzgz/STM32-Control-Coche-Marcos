# LIMP_HOME Without Steering Centering — Implementation Report

## Migration Step 2: STANDBY → LIMP_HOME path without centering requirement

**Reference:** `docs/FAIL_OPERATIONAL_MIGRATION_AUDIT.md` — Step 2 (Risk R3)  
**Goal:** Convert from "fail-safe immobilizing" to "fail-operational degraded mobility"  
**Scope:** Steering calibration must NOT block LIMP_HOME entry

---

## Files Modified

| File | Function | Change |
|------|----------|--------|
| `Core/Src/safety_system.c` | `Safety_CheckCANTimeout()` | Removed `Steering_IsCalibrated()` from STANDBY→LIMP_HOME guard |
| `Core/Src/safety_system.c` | `Safety_CheckCANTimeout()` | Added `Steering_IsCalibrated()` to LIMP_HOME→ACTIVE recovery guard |

No other files were modified. Boot validation, steering centering, motor control, CAN protocol, traction math, and steering PID are untouched.

---

## Guards Changed

### Guard 1: STANDBY → LIMP_HOME (CAN timeout branch)

**Before:**
```c
if (system_state == SYS_STATE_STANDBY &&
    Steering_IsCalibrated() &&        // ← BLOCKED LIMP_HOME
    BootValidation_IsPassed()) {
    Safety_SetState(SYS_STATE_LIMP_HOME);
}
```

**After:**
```c
if (system_state == SYS_STATE_STANDBY &&
    BootValidation_IsPassed()) {      // ← calibration NOT required
    Safety_SetState(SYS_STATE_LIMP_HOME);
}
```

**Rationale:** LIMP_HOME operates at walking speed (5 km/h, 20 % torque) with no assisted steering guarantee. Steering calibration is irrelevant — the `Steering_ControlLoop()` already rejects setpoints when uncalibrated. Removing this guard allows the vehicle to move under its own power even if the inductive center sensor fails, the encoder is missing, or centering times out.

### Guard 2: LIMP_HOME → ACTIVE (CAN restored branch)

**Before:**
```c
if (system_state == SYS_STATE_LIMP_HOME) {
    Safety_ClearError(SAFETY_ERROR_CAN_TIMEOUT);
    Safety_SetState(SYS_STATE_ACTIVE);  // ← no calibration check
}
```

**After:**
```c
if (system_state == SYS_STATE_LIMP_HOME &&
    Steering_IsCalibrated()) {          // ← ACTIVE requires calibration
    Safety_ClearError(SAFETY_ERROR_CAN_TIMEOUT);
    Safety_SetState(SYS_STATE_ACTIVE);
}
```

**Rationale:** ACTIVE state requires full steering control. If centering never completed, the vehicle must remain in LIMP_HOME even when CAN is restored. This preserves the original safety invariant: ACTIVE always has calibrated steering.

---

## State Transition Table — Before / After

### STANDBY transitions

| Condition | Before | After |
|-----------|--------|-------|
| CAN alive + calibrated + boot validated + no error | → ACTIVE | → ACTIVE (unchanged) |
| CAN alive + NOT calibrated | stays STANDBY | stays STANDBY (unchanged) |
| CAN timeout + calibrated + boot validated | → LIMP_HOME | → LIMP_HOME (unchanged) |
| CAN timeout + NOT calibrated + boot validated | **stays STANDBY (BLOCKED)** | **→ LIMP_HOME** |
| CAN timeout + NOT calibrated + boot NOT validated | stays STANDBY | stays STANDBY |

### LIMP_HOME transitions

| Condition | Before | After |
|-----------|--------|-------|
| CAN restored + calibrated | → ACTIVE | → ACTIVE (unchanged) |
| CAN restored + NOT calibrated | **→ ACTIVE (unsafe)** | **stays LIMP_HOME** |
| Hardware danger (overcurrent, etc.) | → SAFE | → SAFE (unchanged) |

### Other transitions (unchanged)

| Transition | Guard | Status |
|------------|-------|--------|
| BOOT → STANDBY | Peripheral init complete | Unchanged |
| STANDBY → ACTIVE | CAN alive + no error + calibrated + boot validated | Unchanged |
| ACTIVE → DEGRADED | Non-critical fault | Unchanged |
| ACTIVE → LIMP_HOME | CAN timeout | Unchanged |
| DEGRADED → LIMP_HOME | CAN timeout | Unchanged |
| DEGRADED → SAFE | Consecutive errors ≥ 3 | Unchanged |
| SAFE → ACTIVE | Fault cleared + heartbeat | Unchanged |
| any → ERROR | Watchdog / emergency stop | Unchanged |

---

## Why ACTIVE Safety Remains Intact

1. **STANDBY → ACTIVE** guard is unchanged: still requires `Steering_IsCalibrated() && safety_error == SAFETY_ERROR_NONE && BootValidation_IsPassed()`.

2. **LIMP_HOME → ACTIVE** recovery now explicitly requires `Steering_IsCalibrated()`, which was previously implicit (LIMP_HOME could only be reached with calibration).

3. **`Safety_SetState(SYS_STATE_ACTIVE)`** internal guard requires `safety_error == SAFETY_ERROR_NONE`. If `SAFETY_ERROR_CENTERING` is latched (centering aborted), ACTIVE is blocked regardless.

4. **DEGRADED → ACTIVE** recovery requires `safety_error == SAFETY_ERROR_NONE` and a 500 ms debounce period. Centering fault blocks this path.

5. No path bypasses the calibration requirement for ACTIVE. The only relaxation is for LIMP_HOME, which already operates with:
   - 20 % torque limit (`LIMP_HOME_TORQUE_LIMIT_FACTOR`)
   - 5 km/h speed cap (`LIMP_HOME_SPEED_LIMIT_KMH`)
   - 10 %/s ramp rate (`LIMP_HOME_RAMP_RATE_PCT_PER_S`)
   - No torque vectoring (Ackermann disabled)
   - No assisted steering guarantee

---

## Oscillation Prevention (Requirement 4)

No infinite oscillation between STANDBY and SAFE is possible:

- **STANDBY → LIMP_HOME** is one-way when CAN is timed out. There is no LIMP_HOME → STANDBY path.
- If centering aborts while in LIMP_HOME, `Centering_Abort()` attempts `Safety_SetState(SYS_STATE_DEGRADED)`, but LIMP_HOME → DEGRADED is not an allowed transition in `Safety_SetState()`. The system stays in LIMP_HOME with `SAFETY_ERROR_CENTERING` latched. The fault is reported but does not cause state oscillation.
- CAN restored with uncalibrated steering: system stays in LIMP_HOME (new guard prevents ACTIVE entry).

---

## Fault Reporting Preserved (Requirement 5)

The following fault reporting mechanisms are untouched:

| Fault indicator | Source | Status |
|----------------|--------|--------|
| `SAFETY_ERROR_CENTERING` | `Centering_Abort()` in `steering_centering.c` | Still latched on centering fault |
| `FAULT_CENTERING` bit | `Safety_GetFaultFlags()` in CAN heartbeat byte 2 | Still reported |
| `ServiceMode_SetFault(MODULE_STEER_ENCODER, ...)` | Boot validation and `Safety_CheckEncoder()` | Still logged |
| `BOOT_CHECK_ENCODER_HEALTHY` | `BootValidation_Run()` | Still evaluated and reported |
| CAN STATUS_STEERING (0x204) | `CAN_SendStatusSteering()` with `Steering_IsCalibrated()` | Still transmitted |

---

## Verification Test Cases

### Test 1: Boot with disconnected steering encoder

**Procedure:** Disconnect encoder cable. Power on.  
**Expected:** Centering detects `Encoder_HasFault()` on first tick → `Centering_Abort()` → DEGRADED. CAN timeout → DEGRADED → LIMP_HOME (via existing ACTIVE/DEGRADED→LIMP_HOME path). Vehicle accepts pedal input at walking speed.  
**ACTIVE safety:** ACTIVE blocked by `SAFETY_ERROR_CENTERING` and `Steering_IsCalibrated() == false`.

### Test 2: Centering timeout (center sensor disconnected)

**Procedure:** Disconnect inductive center sensor. Power on. Wait > 250 ms (CAN timeout).  
**Expected:** Boot validation passes (encoder healthy). `Steering_IsCalibrated() == false`. With new guard: STANDBY → LIMP_HOME (CAN timeout + boot validated). Vehicle accepts pedal input at walking speed.  
**ACTIVE safety:** ACTIVE requires calibration — unchanged.

### Test 3: Steering sensor unplugged while driving (ACTIVE → LIMP_HOME → recovery)

**Procedure:** Start normally (ACTIVE). Unplug center sensor (does not affect ACTIVE operation — centering only runs at boot). Disconnect CAN cable.  
**Expected:** CAN timeout → ACTIVE → LIMP_HOME. If encoder also fails, `Safety_CheckEncoder()` enters DEGRADED, then LIMP_HOME.  
**Recovery:** Reconnect CAN. If steering was calibrated at boot, LIMP_HOME → ACTIVE. If not, stays LIMP_HOME.

### Test 4: Normal calibrated startup (regression)

**Procedure:** Normal boot with all hardware connected. ESP32 sends heartbeat.  
**Expected:** Centering completes → `Steering_IsCalibrated() == true`. Boot validation passes. CAN alive → STANDBY → ACTIVE. **Identical to current firmware.** LIMP_HOME guard change is irrelevant because CAN is alive (CAN timeout branch not taken).

---

## Summary

Two guards changed in `Safety_CheckCANTimeout()`:
1. STANDBY → LIMP_HOME: removed `Steering_IsCalibrated()` requirement
2. LIMP_HOME → ACTIVE: added `Steering_IsCalibrated()` requirement

The system transitions from "fail-safe immobilizing" to "fail-operational degraded mobility." A steering failure during boot leads to STANDBY → LIMP_HOME, and the vehicle accepts pedal input with degraded limits. ACTIVE safety is preserved — calibration is still mandatory for full operation.
