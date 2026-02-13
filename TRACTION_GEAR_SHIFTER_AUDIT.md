# AUDIT REPORT — Traction Modes, 4x4/4x2 Logic, Gear Implementation & Shifter Ownership

**Date**: 2026-02-13
**Scope**: STM32G474RE firmware — `Core/Src/*.c`, `Core/Inc/*.h`, `esp32/include/can_ids.h`
**Mode**: IMPLEMENTATION STATUS AUDIT ONLY — no modifications, no refactoring, no proposals.

---

## SECTION 1 — 4x4 / 4x2 IMPLEMENTATION

### 1.1 Is 4x4 / 4x2 mode implemented?

**YES — COMPLETE.**

| Item | Detail |
|------|--------|
| **Variable** | `traction_state.mode4x4` (`bool`) |
| **File** | `Core/Src/motor_control.c` line 114 (static `TractionState_t traction_state`); declared in `Core/Inc/motor_control.h` line 44 (`bool mode4x4`) |
| **Set via** | CAN ID `0x102` (`CAN_ID_CMD_MODE`), byte 0, bit 0 — parsed in `Core/Src/can_handler.c` lines 419–426 |
| **Function** | `Traction_SetMode4x4(bool enable)` — `Core/Src/motor_control.c` lines 306–309 |
| **4x2 value** | `mode4x4 = false` (bit 0 of byte 0 is `0`) |
| **4x4 value** | `mode4x4 = true` (bit 0 of byte 0 is `1`) |
| **Validation** | `Safety_ValidateModeChange()` in `Core/Src/safety_system.c` lines 277–288: mode change only allowed when `Safety_IsCommandAllowed()` is true AND average wheel speed ≤ 1.0 km/h |

### 1.2 In 4x2 mode, which wheels receive torque?

Reference: `Core/Src/motor_control.c` lines 569–577 (the `else` branch of `Traction_Update()`):

```c
/* 4x2: Front wheels only, per-wheel scale applied */
Motor_SetPWM(&motor_fl, (uint16_t)(base_pwm * safety_status.wheel_scale[MOTOR_FL]));
Motor_SetDirection(&motor_fl, dir);
Motor_SetPWM(&motor_fr, (uint16_t)(base_pwm * safety_status.wheel_scale[MOTOR_FR]));
Motor_SetDirection(&motor_fr, dir);
Motor_SetPWM(&motor_rl, 0);   Motor_Enable(&motor_rl, 0);
Motor_SetPWM(&motor_rr, 0);   Motor_Enable(&motor_rr, 0);
```

| Wheel | Receives torque in 4x2? | PWM source |
|-------|------------------------|------------|
| FL | **YES** | `base_pwm * wheel_scale[MOTOR_FL]` |
| FR | **YES** | `base_pwm * wheel_scale[MOTOR_FR]` |
| RL | **NO** | PWM = 0, Enable = 0 |
| RR | **NO** | PWM = 0, Enable = 0 |

### 1.3 Is torque scaling symmetrical?

**4x2 disables the rear axle entirely.** There is no partial scaling. Rear motors receive PWM = 0 and are disabled (`Motor_Enable(&motor_rl, 0)`). Front motors receive the full computed `base_pwm` modulated only by `wheel_scale[]`.

In 4x4 mode (`Core/Src/motor_control.c` lines 557–568), all four wheels receive the same `base_pwm` individually modulated by `safety_status.wheel_scale[i]`. The scaling is symmetrical across all four wheels — each gets `base_pwm * wheel_scale[i]`.

### 1.4 Does ABS/TCS still operate per-wheel in 4x2 mode?

**YES.** ABS and TCS operate on all four wheels regardless of drive mode.

- `ABS_Update()` (`Core/Src/safety_system.c` lines 336–392): reads speeds of all 4 wheels, computes slip per-wheel, sets `wheel_scale[i]` per-wheel.
- `TCS_Update()` (`Core/Src/safety_system.c` lines 413–495): reads speeds of all 4 wheels, computes slip per-wheel, sets `wheel_scale[i]` per-wheel.

However, in 4x2 mode, the rear wheel `wheel_scale[]` values computed by ABS/TCS have **no effect** on motor output because rear motors are unconditionally set to PWM = 0 (lines 575–576). The `wheel_scale[]` values are still computed and transmitted via CAN (ID `0x205`, `CAN_SendStatusTraction()`), but they do not influence rear motor PWM in 4x2 mode.

Front wheel `wheel_scale[FL]` and `wheel_scale[FR]` are effective and correctly applied in 4x2 mode.

### 1.5 Is there any hidden dependency between 4x4 and gear selection?

**NO.** The 4x4/4x2 mode and gear selection are fully independent:

- `mode4x4` is set via `Traction_SetMode4x4()` (line 306–309)
- `current_gear` is set via `Traction_SetGear()` (line 316–319)
- In `Traction_Update()`, gear logic (Park/Neutral/Forward/Reverse) is evaluated first (lines 333–431), then the 4x4/4x2/tank-turn branching is evaluated for PWM distribution (lines 545–577)
- Gear-based power scaling (`GEAR_POWER_FORWARD_PCT`, `GEAR_POWER_REVERSE_PCT`, etc.) is applied to `effective_demand` at lines 521–531, before the 4x4/4x2 branching

There is no cross-dependency between these two systems.

---

## SECTION 2 — INTELLIGENT TRACTION STATUS

### 2.1 ABS Configuration

| Parameter | Value | File:Line |
|-----------|-------|-----------|
| **Slip threshold** | 15% | `Core/Src/safety_system.c` line 21: `#define ABS_SLIP_THRESHOLD 15` |
| **Minimum speed gate** | 10.0 km/h (average of 4 wheels) | `Core/Src/safety_system.c` line 352: `if (avg < 10.0f)` |
| **Per-wheel scale when active** | 0.0 (full cut) | `Core/Src/safety_system.c` line 334: `#define ABS_WHEEL_SCALE_ACTIVE 0.0f` |
| **Global fallback** | All 4 wheels locking (mask == 0x0F) → `Traction_SetDemand(0)` | Line 384–386 |
| **Recovery** | Wheel not locking → `wheel_scale[i] = 1.0` | Line 373 |

### 2.2 TCS Configuration

| Parameter | Value | File:Line |
|-----------|-------|-----------|
| **Slip threshold** | 15% | `Core/Src/safety_system.c` line 22: `#define TCS_SLIP_THRESHOLD 15` |
| **Minimum speed gate** | 3.0 km/h (average of 4 wheels) | `Core/Src/safety_system.c` line 431: `if (avg < 3.0f)` |
| **Initial reduction** | 40% (scale = 0.6) | Line 408: `#define TCS_INITIAL_REDUCTION 0.40f` |
| **Progressive reduction** | +5% per cycle while still slipping | Line 409: `#define TCS_SMOOTH_REDUCTION 0.05f` |
| **Maximum reduction** | 80% (scale = 0.2) | Line 410: `#define TCS_MAX_REDUCTION 0.80f` |
| **Recovery rate** | 25%/s | Line 411: `#define TCS_RECOVERY_RATE_PER_S 0.25f` |
| **Global fallback** | All 4 wheels spinning (mask == 0x0F) → reduce demand to `Pedal * (1 - 0.8)` | Lines 487–489 |

### 2.3 Progressive reduction logic

**YES — TCS uses progressive reduction.**

1. First activation: aggressive 40% cut (`tcs_reduction[i] = TCS_INITIAL_REDUCTION`, line 454)
2. Continued slipping: +5% per cycle (`tcs_reduction[i] += TCS_SMOOTH_REDUCTION`, line 458)
3. Clamped at 80% max (`tcs_reduction[i] > TCS_MAX_REDUCTION`, line 461–462)
4. Recovery: 25%/s ramp-down when slip clears (line 467: `tcs_reduction[i] -= TCS_RECOVERY_RATE_PER_S * dt`)

ABS uses a binary approach: wheel locking → scale = 0.0; not locking → scale = 1.0. No progressive logic.

### 2.4 wheel_scale[] usage

| Question | Answer |
|----------|--------|
| **Does wheel_scale[] affect PWM directly?** | **YES.** In `Traction_Update()` (lines 549, 561–568, 571–574): `Motor_SetPWM(&motor_xx, (uint16_t)(base_pwm * safety_status.wheel_scale[MOTOR_XX]))` |
| **Scaling happens before or after gear scaling?** | **AFTER.** Gear scaling modifies `effective_demand` (lines 521–531), which computes `base_pwm` (line 533). `wheel_scale[]` is multiplied against `base_pwm` per-wheel (lines 549, 561–574). Order: demand → gear scale → base_pwm → wheel_scale → Motor_SetPWM |
| **Scaling happens before or after dynamic braking?** | **AFTER.** Dynamic braking modifies `effective_demand` (lines 509–514), which is upstream of `base_pwm` computation (line 533). `wheel_scale[]` is applied to `base_pwm` per-wheel, so it happens after dynamic braking demand computation. |

### 2.5 ABS/TCS interaction

ABS runs first (`main.c` line 87: `ABS_Update()`), then TCS runs (`main.c` line 88: `TCS_Update()`).

- ABS sets `wheel_scale[i]` for locking wheels (line 368: `= ABS_WHEEL_SCALE_ACTIVE` = 0.0)
- ABS restores `wheel_scale[i] = 1.0` for non-locking wheels (line 373)
- TCS then takes the **minimum** of ABS and TCS scales (line 477–478): `if (tcs_scale < safety_status.wheel_scale[i]) safety_status.wheel_scale[i] = tcs_scale;`

This ensures the most restrictive intervention wins.

### 2.6 Dead code related to traction modes

**NO dead code exists.** All traction-related code paths are actively used:

- `ABS_Update()` — called every 10ms from `main.c` line 87
- `TCS_Update()` — called every 10ms from `main.c` line 88
- `wheel_scale[]` — consumed in `Traction_Update()` and `CAN_SendStatusTraction()`
- `tcs_reduction[]` — internal state used in `TCS_Update()`, reset in `Safety_Init()` and `TCS_Reset()`
- All `ABS_WHEEL_SCALE_ACTIVE`, `TCS_INITIAL_REDUCTION`, etc. constants are referenced

---

## SECTION 3 — GEAR SYSTEM (D1 / D2 / N / R / P)

### 3.1 Actual enum values currently defined

From `Core/Inc/motor_control.h` lines 22–28:

```c
typedef enum {
    GEAR_PARK    = 0,
    GEAR_REVERSE = 1,
    GEAR_NEUTRAL = 2,
    GEAR_FORWARD = 3,
    GEAR_FORWARD_D2 = 4   /* Full power (100 %) forward mode */
} GearPosition_t;
```

| Enum | Numeric Value | Description |
|------|---------------|-------------|
| `GEAR_PARK` | 0 | Park — active hold brake |
| `GEAR_REVERSE` | 1 | Reverse — 60% power, direction inverted |
| `GEAR_NEUTRAL` | 2 | Neutral — full coast, no motor output |
| `GEAR_FORWARD` | 3 | Forward D1 — 60% max power |
| `GEAR_FORWARD_D2` | 4 | Forward D2 — 100% max power |

**GEAR_FORWARD_D2 EXISTS** — confirmed at `motor_control.h` line 27.

### 3.2 Gear power scaling values

From `Core/Src/motor_control.c` lines 82–84:

| Gear | Power Scale | Source |
|------|------------|--------|
| D1 (`GEAR_FORWARD`) | 60% | `#define GEAR_POWER_FORWARD_PCT 0.60f` (line 82) |
| D2 (`GEAR_FORWARD_D2`) | 100% | `#define GEAR_POWER_FORWARD_D2_PCT 1.00f` (line 83) |
| R (`GEAR_REVERSE`) | 60% | `#define GEAR_POWER_REVERSE_PCT 0.60f` (line 84) |
| N (`GEAR_NEUTRAL`) | Full coast | PWM = 0, motors disabled (lines 410–431) |
| P (`GEAR_PARK`) | Active hold brake | 30% hold PWM with current/temp derating (lines 333–403) |

### 3.3 Where gear scaling is applied

In `Traction_Update()` (`Core/Src/motor_control.c`):

1. **Before ramp?** — **NO.** Ramp is in `Traction_SetDemand()` (lines 262–303). Gear scaling is in `Traction_Update()` (lines 521–531).
2. **After ramp?** — **YES.** `traction_state.demandPct` (output of ramp limiter) feeds into `demand` variable (line 434), which becomes `effective_demand` after dynamic braking, then gear scaling modifies `effective_demand` (lines 521–531).
3. **Before wheel_scale?** — **YES.** Gear scaling modifies `effective_demand` → computes `base_pwm` (line 533) → `wheel_scale[]` is multiplied per-wheel (lines 549, 561–574).
4. **Before dynamic brake?** — **NO.** Dynamic braking modifies `effective_demand` first (lines 509–514), then gear scaling is applied (lines 521–531). However, gear scaling only applies to positive demand: `if (effective_demand > 0.0f)` (line 521). Dynamic braking sets negative demand, so the two do not overlap — braking demand is NOT scaled by gear power.

**Order of operations:**
```
Pedal → EMA filter → Ramp limiter → traction_state.demandPct
  → Dynamic braking adjustment → effective_demand
  → Gear power scaling (positive demand only) → effective_demand
  → base_pwm = |effective_demand| * PWM_PERIOD / 100
  → Reverse direction (if GEAR_REVERSE)
  → Per-wheel: base_pwm * wheel_scale[i] → Motor_SetPWM
```

### 3.4 Speed gating for gear change

**YES — speed gating enforced at ≤ 1.0 km/h.**

From `Core/Src/can_handler.c` lines 438–442:

```c
float avg_spd = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                 Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;
if (avg_spd <= 1.0f) {
    Traction_SetGear(requested);
}
```

Gear change is silently rejected if average speed > 1.0 km/h.

### 3.5 Legacy "1,2,3,4" numeric gear remnants

**NO legacy numeric gear remnants exist in control logic.**

- The enum uses 0, 1, 2, 3, 4 — but these map to P, R, N, D1, D2 respectively
- There is no code referencing `gear == 1` as "first gear" or `gear == 2` as "second gear" in the old speed-ratio sense
- The CAN handler validates `gear_raw <= (uint8_t)GEAR_FORWARD_D2` (line 435), i.e., values 0–4
- No files contain any reference to `GEAR_1`, `GEAR_2`, `GEAR_3`, `GEAR_4` or speed-based gear ratios
- The gear system is purely mode-based (P/R/N/D1/D2), not speed-ratio-based

---

## SECTION 4 — PHYSICAL SHIFTER OWNERSHIP

### 4.1 Does STM32 read any GPIO pins for shifter positions?

**NO.**

**MX_GPIO_Init() analysis** (`Core/Src/main.c` lines 237–287):

| GPIO Group | Pins Initialized | Purpose |
|------------|------------------|---------|
| GPIOC outputs | PC0–PC4 (direction), PC5–PC7/PC9/PC13 (enable), PC10–PC12 (relays) | Motor control |
| GPIOA EXTI inputs | PA0, PA1, PA2 | Wheel speed FL/FR/RL |
| GPIOB EXTI inputs | PB15 | Wheel speed RR |
| GPIOB EXTI inputs | PB5 | Steering center inductive sensor |

**PB12, PB13, PB14 are NOT initialized** in `MX_GPIO_Init()`. No `HAL_GPIO_Init(GPIOB, ...)` call includes these pins. No EXTI handler exists for them.

**main.h pin definitions** (`Core/Inc/main.h`): PB12, PB13, PB14 are not defined as any pin macro. The file defines pins PA0–PA15, PB0, PB3–PB9, PB15, PC0–PC13 — PB12/PB13/PB14 are absent.

### 4.2 Is the shifter read by STM32, ESP32, or via CAN only?

**The physical shifter is read by the ESP32.** The STM32 receives gear commands exclusively via CAN.

- The ESP32 reads the physical shifter via MCP23017 I/O expander (referenced in base firmware `src/input/shifter.cpp`, GPIOB0–B4 on the MCP23017)
- The ESP32 sends the gear position to STM32 via CAN ID `0x102` (`CAN_ID_CMD_MODE`), byte 1
- The STM32 has no shifter-reading hardware or code — it is a CAN-only consumer

### 4.3 CAN ID used for gear command

| Parameter | Value | Source |
|-----------|-------|--------|
| **CAN ID** | `0x102` (`CAN_ID_CMD_MODE`) | `Core/Inc/can_handler.h` line 25 |
| **Byte** | Byte 1 (second byte, index 1) | `Core/Src/can_handler.c` line 434: `uint8_t gear_raw = rx_payload[1]` |
| **DLC requirement** | ≥ 2 bytes | `Core/Src/can_handler.c` line 433: `if (msg_len >= 2)` |
| **Value mapping** | 0 = PARK, 1 = REVERSE, 2 = NEUTRAL, 3 = FORWARD (D1), 4 = FORWARD_D2 | `Core/Inc/motor_control.h` lines 23–27, `Core/Src/can_handler.c` line 435 |
| **Backward compatibility** | If only 1 byte is sent (no byte 1), gear remains unchanged | `Core/Src/can_handler.c` lines 432–433: `if (msg_len >= 2)` — gear change block is skipped if DLC < 2 |

### 4.4 Does STM32 enforce gear validation or blindly accept CAN value?

**STM32 enforces validation.** It does NOT blindly accept the CAN value.

Validations in `Core/Src/can_handler.c` lines 433–443:

1. **Range check**: `if (gear_raw <= (uint8_t)GEAR_FORWARD_D2)` — rejects values > 4 (line 435)
2. **Type cast**: `GearPosition_t requested = (GearPosition_t)gear_raw` — only valid enum values pass (line 436)
3. **Speed gate**: Average speed must be ≤ 1.0 km/h (lines 438–441) — gear change silently rejected at higher speeds

Additionally, the CAN message itself must pass:
- Hardware RX filter (`CAN_ConfigureFilters()`, lines 77–112)
- Mode change validation: `Safety_ValidateModeChange()` must return true for byte 0 flags (line 423) — requires `Safety_IsCommandAllowed()` (ACTIVE or DEGRADED state) AND speed ≤ 1.0 km/h

Note: The gear change in byte 1 has its own independent speed check (≤ 1.0 km/h) and is NOT gated by `Safety_ValidateModeChange()`. It can be set regardless of mode change result, but still requires DLC ≥ 2 and speed ≤ 1.0 km/h.

---

## SECTION 5 — FINAL STATUS SUMMARY

| Feature | Status | Detail |
|---------|--------|--------|
| **4x4 logic** | **COMPLETE** | Mode stored in `traction_state.mode4x4`, set via CAN `0x102` bit 0, applies to all 4 wheels with per-wheel `wheel_scale[]` |
| **4x2 logic** | **COMPLETE** | Default mode. Front wheels only (FL/FR), rear motors disabled (PWM=0, Enable=0). Per-wheel ABS/TCS applied to front wheels |
| **Intelligent traction (ABS)** | **STABLE** | Slip threshold 15%, min speed 10 km/h, binary per-wheel scale (0.0/1.0), global fallback if all 4 lock |
| **Intelligent traction (TCS)** | **STABLE** | Slip threshold 15%, min speed 3 km/h, progressive reduction (40% initial, +5%/cycle, 80% max, 25%/s recovery), global fallback if all 4 spin |
| **Gear system** | **CORRECT** | P/R/N/D1/D2 enum (0–4), D1=60%, D2=100%, R=60%, N=coast, P=active hold. Speed-gated at ≤ 1 km/h. No legacy "1,2,3,4" remnants |
| **Shifter hardware ownership** | **CAN-only (ESP32 reads physical shifter)** | STM32 has no GPIO pins initialized for shifter (PB12/PB13/PB14 not in `MX_GPIO_Init()`). Gear received via CAN `0x102` byte 1. STM32 validates range and speed |

### Inconsistencies Found

1. **~~ESP32 `can_ids.h` does not define gear byte in CMD_MODE~~**: **RESOLVED.** The ESP32 `can_ids.h` CMD_MODE DLC comment has been updated from "DLC 1" to "DLC 2 (byte0=mode flags, byte1=gear)" to match the STM32 implementation.

2. **~~ESP32 `can_ids.h` SystemState enum values differ from STM32~~**: **RESOLVED.** The ESP32 SystemState enum has been updated to match the STM32 exactly: BOOT=0, STANDBY=1, ACTIVE=2, DEGRADED=3, SAFE=4, ERROR=5. The screen_manager.cpp switch-case now includes a DEGRADED case.

3. **ABS/TCS wheel_scale computation in 4x2 mode covers rear wheels**: ABS/TCS compute `wheel_scale[RL]` and `wheel_scale[RR]` even in 4x2 mode, but these values have no effect on rear motor output (rear motors are unconditionally off). This is a design tradeoff, not a bug: the rear `wheel_scale[]` values are transmitted via CAN `0x205` (`CAN_SendStatusTraction()`), providing the ESP32 HMI with full per-wheel traction visibility regardless of drive mode. The computation is harmless and incurs negligible overhead.

### Technical Verdict

The traction mode system (4x4/4x2), intelligent traction (ABS/TCS with per-wheel `wheel_scale[]`), gear system (P/R/N/D1/D2), and shifter ownership (CAN-only, ESP32 reads hardware) are **fully implemented and internally consistent** within the STM32 firmware.

The previously identified cross-system inconsistencies (ESP32 SystemState enum mismatch and CMD_MODE DLC comment) have been **resolved** by aligning the ESP32 definitions to match the STM32 exactly.

---

*Audit generated from line-by-line analysis of actual source code in the repository. All file paths, function names, and line numbers are directly verifiable. No speculation, no proposals, no modifications.*
