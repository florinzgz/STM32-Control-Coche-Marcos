# Frama-C EVA + RTE Deep Logical Analysis Report

**Tool:** Frama-C 25.0-beta (Manganese)  
**Plugins:** EVA (value analysis, precision 3) + RTE (runtime error checking)  
**Date:** 2026-03-18  
**Scope:** All 14 STM32/Core C source files (69 public entry points analyzed)

---

## 1. Executive Summary

| Category | Count |
|----------|-------|
| Files analyzed (STM32/Core) | 14 |
| Entry points exercised | 69 |
| Total EVA alarms generated | 369 |
| **Confirmed real bugs — fixed** | **7** |
| Confirmed false positives | ~340 |
| Low-risk informational | ~22 |
| ESP32 files (C++ — incompatible) | 29 |

### Alarm Breakdown by Type

| Alarm Type | Count | Real Bugs | False Positives |
|------------|-------|-----------|-----------------|
| Non-finite float value | ~83 | 7 (missing sanitize_float) | ~76 |
| Unsigned overflow | ~55 | 4 (counter wrap) | ~51 |
| Signed overflow | ~32 | 0 | ~32 |
| Out-of-bounds access | ~30 | 0 | ~30 |
| \_Bool trap representation | ~120 | 0 | ~120 |
| Uninitialized access | ~4 | 0 | ~4 |
| Other | ~45 | 0 | ~45 |

---

## 2. Confirmed Real Bugs (Fixed)

### BUG-1: NaN Bypasses Speed Gate in Mode Change Validation
**Severity:** Error lógico crítico  
**File:** `Core/Src/safety_system.c` line 634–636  
**Function:** `Safety_ValidateModeChange()`

**Cause:**  
The average wheel speed was computed from four `Wheel_GetSpeed_*()` calls
but was not passed through `sanitize_float()` before the comparison. If any
wheel-speed float becomes NaN (e.g., due to a sensor fault injecting 0/0
through a corrupted ISR counter), the expression `NaN > 1.0f` evaluates to
**false** in IEEE 754, causing the speed gate to be bypassed.

**Impact on vehicle:**  
Mode changes (4×4, tank turn) could be accepted at any speed — including
highway speed — potentially causing catastrophic loss of control when
switching to tank-turn mode at speed.

**Reproduction:**  
Inject NaN into any `wheel_speed_kmh[]` slot (e.g., by corrupting the pulse
counter so `delta = 0` and `dt = 0` produces 0.0f/0.0f = NaN). Then send
CAN ID 0x102 (mode change command) while the vehicle is moving.

**Fix applied:**
```c
avg_speed = sanitize_float(avg_speed, MODE_CHANGE_MAX_SPEED_KMH + 1.0f);
```
The safe default (above the threshold) ensures NaN → mode change rejected.

---

### BUG-2: NaN Propagation in Obstacle Speed Calculation
**Severity:** Error lógico medio  
**File:** `Core/Src/safety_system.c` line 1497–1500  
**Function:** `Obstacle_GetVehicleSpeed()`

**Cause:**  
Wheel speed sum computed without `sanitize_float()`. NaN propagates through
the `avg < 0.0f` check (which evaluates false for NaN) and returns NaN to
callers. This affects dynamic obstacle thresholds which use speed to adjust
warning/caution/emergency distances.

**Impact on vehicle:**  
NaN speed causes all speed-dependent obstacle threshold adjustments to
produce NaN distances, potentially disabling the obstacle detection
speed-scaling entirely.

**Fix applied:**
```c
avg = sanitize_float(avg, 0.0f);
```

---

### BUG-3: NaN in Frozen Pedal Detection (3 locations)
**Severity:** Error lógico medio  
**File:** `Core/Src/motor_control.c` lines 539, 579, 583  
**Function:** `Traction_SetDemand()`

**Cause:**  
`frozen_pedal_speed` and `current_speed` computed from unsanitized
`Wheel_GetSpeed_*()` sums. If NaN, `fabsf(NaN - NaN)` returns NaN,
and `NaN > FROZEN_PEDAL_SPEED_DELTA_KMH` is false — the frozen-pedal
anomaly detector silently stops working.

**Impact on vehicle:**  
A stuck pedal sensor combined with a wheel speed NaN would go undetected.
The safety system would not raise `SAFETY_ERROR_SENSOR_FAULT`.

**Fix applied:**  
Added `sanitize_float(..., 0.0f)` to all three speed computations.

---

### BUG-4: NaN in Dynamic Braking Speed Check
**Severity:** Error lógico medio  
**File:** `Core/Src/motor_control.c` line 901–905  
**Function:** `Traction_Update()` — dynamic braking minimum speed gate

**Cause:**  
`avg_speed` not sanitized. NaN causes `NaN < DYNBRAKE_MIN_SPEED_KMH` →
false, so dynamic braking remains enabled below minimum speed.

**Impact on vehicle:**  
Dynamic braking could activate at standstill (below minimum speed threshold),
causing unexpected wheel lock or motor stress.

**Fix applied:**
```c
float avg_speed = sanitize_float((...) / 4.0f, 0.0f);
```

---

### BUG-5: NaN in LIMP_HOME Speed Limiter
**Severity:** Error lógico medio  
**File:** `Core/Src/motor_control.c` line 980–981  
**Function:** `Traction_Update()` — LIMP_HOME speed limit

**Cause:**  
`avg_spd` not sanitized. NaN bypasses `NaN > LIMP_HOME_SPEED_LIMIT_KMH` →
false, allowing unlimited speed in LIMP_HOME mode.

**Impact on vehicle:**  
Vehicle in LIMP_HOME (degraded safety) could accelerate beyond the intended
speed cap, defeating the purpose of LIMP_HOME mode.

**Fix applied:**
```c
float avg_spd = sanitize_float((...) / 4.0f, 0.0f);
```

---

### BUG-6: Counter Overflow to Zero (3 counters in safety_system.c)
**Severity:** Riesgo bajo  
**File:** `Core/Src/safety_system.c` lines 442, 812, 931  
**Counters:** `degraded_telemetry_count`, `abs_activation_count`, `tcs_activation_count`

**Cause:**  
Bare `counter++` on `uint32_t` values. After 4,294,967,295 increments the
counter wraps to 0, producing a misleading telemetry reset. The rest of the
codebase (15 call sites in `can_handler.c`) consistently uses `sat_inc_u32()`
for all CAN statistics counters.

**Impact on vehicle:**  
Telemetry dashboard would show 0 ABS/TCS activations and 0 degraded-mode
entries after ~4 billion events. Not safety-critical, but violates the
codebase convention and produces incorrect diagnostic data.

**Fix applied:**  
Added local `sat_inc_u32()` helper (matching `can_handler.c` pattern) and
replaced all three bare increments.

---

### BUG-7: Counter Overflow in Error Log
**Severity:** Riesgo bajo  
**File:** `Core/Src/error_log.c` line 198  
**Counter:** `log_header.total_events`

**Cause:**  
Same pattern as BUG-6. `total_events++` wraps at UINT32_MAX.

**Impact on vehicle:**  
Error log total event count would wrap to 0 after ~4 billion logged events.
Purely diagnostic — no safety impact.

**Fix applied:**
```c
if (log_header.total_events < UINT32_MAX) {
    log_header.total_events++;
}
```

---

## 3. Confirmed False Positives (Not Bugs)

### FP-1: `HAL_GetTick()` Subtraction Unsigned Overflow (~40 alarms)
**Lines:** safety_system.c:575,606,779,812,888,931,989,1075,1133,1181,1728,1823,1836,1891,1938; motor_control.c:543,580,619,869; sensor_manager.c:85,89; can_handler.c:285,1099,1160,1177; steering_centering.c:136,201,231; etc.

**Reason false positive:** In embedded firmware, `HAL_GetTick()` returns a 32-bit counter that wraps at `UINT32_MAX`. The expression `now - last_tick` using **unsigned** arithmetic correctly computes elapsed time even across the wrap boundary. This is standard and intentional embedded practice. Frama-C's EVA warns because it tracks unsigned overflow strictly, but this is well-defined behavior in C.

### FP-2: `_Bool` Trap Representation (~120 alarms)
**Lines:** Throughout all files.

**Reason false positive:** When using `-lib-entry`, Frama-C assumes all global variables (including `bool` fields in structs) may contain any bit pattern. Since `_Bool` in C11 has only two valid values (0 and 1), any unspecified `_Bool` read triggers a trap-representation alarm. In the actual firmware, all `bool` variables are initialized to valid values.

### FP-3: Uninitialized Access After `HAL_FDCAN_GetRxMessage()` (4 alarms)
**Lines:** can_handler.c:697,701,708

**Reason false positive:** Frama-C's `-lib-entry` mode has no specification for `HAL_FDCAN_GetRxMessage()` and cannot verify it initializes its output parameters. In reality, the HAL function fully initializes `rx_hdr` and `rx_payload` on `HAL_OK`.

### FP-4: Out-of-Bounds Pointer Access in Library Entry (~30 alarms)
**Lines:** ackermann.c:26-27,54-58; encoder_reader.c:40,45,53; service_mode.c:106,114,135,174; etc.

**Reason false positive:** Frama-C's `-lib-entry` doesn't know that function pointer parameters are valid. In the actual firmware, callers always pass valid pointers.

### FP-5: INA226 Shunt Division by Constant (~4 alarms)
**Lines:** sensor_manager.c:418,419,423

**Reason false positive:** `shunt_mohm` is always either `INA226_SHUNT_MOHM_BATTERY` (0.5) or `INA226_SHUNT_MOHM_MOTOR` (1.0) — never zero. Frama-C cannot track that the ternary operator eliminates zero.

### FP-6: ABS Slip Division by Zero (1 alarm)
**Line:** safety_system.c:772

**Reason false positive:** `avg` is already checked to be >= 10.0f on line 757 before the division. Frama-C's per-function analysis doesn't track this constraint through the `return` on line 763.

### FP-7: EPS Parameter Division by Zero (2 alarms)
**Lines:** motor_control.c:1548,1549

**Reason false positive:** `p->assist_vs_speed` and `p->return_vs_speed` are validated in `EPS_Params_Set()` to reject values <= 0.0f (lines 139-141 of eps_params.c). Defaults are 18.0f and 35.0f respectively.

### FP-8: Encoder Int32 Subtraction Overflow (4 alarms)
**Lines:** motor_control.c:1689-1690; steering_centering.c:146-147

**Reason false positive:** Encoder count is bounded by `ENC_MAX_COUNTS` ≈ 987 (from mechanical steering limits). Maximum possible `|delta|` is ~1974, well within `int32_t` range.

### FP-9: CAN Steering Angle Shift Overflow (3 alarms)
**Line:** can_handler.c:786

**Reason false positive:** `rx_payload[1]` is `uint8_t` (0-255). After integer promotion to `int`, `int << 8` gives 0-65280, within `int` range on all platforms with `sizeof(int) >= 2`.

---

## 4. ESP32/src Analysis

All 29 ESP32 source files are C++ (`.cpp`). Frama-C only supports C analysis.

**Verdict:** ESP32 files are **not compatible** with Frama-C.

The ESP32 code has been separately validated with:
- CppCheck 2.13.0 (0 errors, 0 warnings post-fixes)
- Clang-tidy 18.1.3 (0 real bugs)
- 438 host-compiled unit tests (all passing)

---

## 5. Files Analyzed

| File | Size | Entry Points | Alarms | Real Bugs |
|------|------|-------------|--------|-----------|
| safety_system.c | 84 KB | 15 | 97 | 5 |
| motor_control.c | 80 KB | 8 | 116 | 5 |
| can_handler.c | 51 KB | 9 | 47 | 0 |
| sensor_manager.c | 29 KB | 9 | 49 | 0 |
| main.c | 38 KB | 1 | 0 | 0 |
| error_log.c | 8.4 KB | 5 | 6 | 1 |
| steering_centering.c | 9.3 KB | 3 | 12 | 0 |
| service_mode.c | 9.2 KB | 5 | 6 | 0 |
| boot_validation.c | 7.7 KB | 1 | 10 | 0 |
| steering_cal_store.c | 6.8 KB | 3 | 6 | 0 |
| eps_params.c | 8.1 KB | 3 | 1 | 0 |
| math_safety.c | 2.7 KB | 4 | 1 | 0 |
| ackermann.c | 2.2 KB | 1 | 10 | 0 |
| encoder_reader.c | 2.3 KB | 4 | 8 | 0 |

---

## 6. Methodology

### Analysis Environment
- **Frama-C version:** 25.0-beta (Manganese)
- **EVA precision:** 3 (auto-configures slevel=35, ilevel=24, plevel=70)
- **EVA domains:** cvalue, equality, gauges, symbolic-locations
- **Machine model:** x86_32 (closest to ARM Cortex-M4 32-bit)
- **C standard:** C11

### Approach
1. Created minimal STM32 HAL type stubs (GPIO, TIM, ADC, FDCAN, I2C, IWDG, FLASH)
2. Analyzed each source file independently using `-lib-entry` mode
3. Exercised all 69 public API functions as separate entry points
4. Classified each alarm by inspecting source code context
5. Applied conservative fixes only for confirmed real bugs

### Limitations
- `-lib-entry` mode over-approximates: all globals are assumed to have
  arbitrary initial values, causing many false positives on `_Bool` fields
  and pointer validity
- HAL functions have no specifications, so Frama-C assumes they may not
  initialize output parameters
- Cross-module data flow (e.g., `EPS_Params_Set` validation → `Steering_ControlLoop`
  usage) is not tracked in per-file analysis
- Floating-point analysis uses abstract intervals, not bit-exact IEEE 754

---

## 7. Reproduction Commands

```bash
# Install Frama-C
sudo apt-get install -y frama-c-base

# Verify version
frama-c --version   # 25.0-beta (Manganese)

# Analyze a single file (example: safety_system.c, entry Safety_CheckCurrent)
frama-c \
  -c11 \
  -cpp-extra-args="-I<STUBS_DIR> -ICore/Inc -DSTM32G474xx -DUSE_HAL_DRIVER \
                   -nostdinc -I/usr/share/frama-c/libc" \
  -machdep x86_32 \
  -lib-entry \
  -main Safety_CheckCurrent \
  -rte \
  -eva \
  -eva-precision 3 \
  -eva-warn-key alarm=active \
  -warn-signed-overflow \
  -warn-unsigned-overflow \
  -no-unicode \
  Core/Src/safety_system.c
```

Note: `<STUBS_DIR>` must contain `stm32g4xx_hal.h` and related HAL type
stubs (GPIO_TypeDef, FDCAN_HandleTypeDef, etc.) since the actual STM32 HAL
driver sources are not included in the repository.
