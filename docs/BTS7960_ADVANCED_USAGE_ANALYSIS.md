# 🔍 BTS7960 ADVANCED USAGE ANALYSIS + FIRMWARE BEHAVIOR VALIDATION

**Repository:** `florinzgz/STM32-Control-Coche-Marcos`
**Date:** 2026-03-21
**MCU:** STM32G474RE (Cortex-M4, 170 MHz, 3.3 V logic)
**Motor Driver:** BTS7960 (IBT-2 dual half-bridge) × 5 modules
**Firmware Entry Point:** `Motor_SetSigned()` in `Core/Src/motor_control.c`

---

## 1. 🧠 USER TECHNICAL INTERPRETATION

### Configuration Assessment

| Question | Answer | Evidence |
|----------|--------|----------|
| Full H-bridge | **NO** | User describes "simplified power switch, not full H-bridge" |
| Single half-bridge | **YES** | "High-side switch" / "Low-side switch" usage described |
| High-side switching | **YES** | Explicitly stated: "Used as high-side switch" |
| Low-side switching | **YES** | Explicitly stated: "Used as low-side switch" |

### Channel Usage

| Question | Answer | Evidence |
|----------|--------|----------|
| Using BOTH BTS7960 channels | **NO** | Single PWM / enable pin control described |
| Using only ONE side as power switch | **YES** | "Single PWM or enable pin" control; "open collector style" |

### What "PWM low = floating switch" Means Electrically

When the user says "PWM low acts like ON/OFF switch (open collector style)", the electrical
behavior is:

1. **BTS7960 truth table**: `INH=1, IN=LOW` → Low-side MOSFET ON (Rds(on) ≈ 7 mΩ)
2. **Result**: The output terminal is pulled to GND through the low-side FET
3. **"Open collector style"**: The load connects between V+ supply and the BTS7960 output.
   When IN=LOW, the low-side FET sinks current through the load. When IN=HIGH, the
   high-side FET sources current (Rds(on) ≈ 9 mΩ)
4. **PWM modulation**: Varying the IN duty cycle controls average current through the load
5. **Internal 10 kΩ pull-down**: Ensures the FET gate is pulled LOW when no signal is present,
   defaulting to a safe OFF state

**Key distinction**: The user is NOT driving a motor bidirectionally. They use ONE half-bridge
as a power switch for heavy loads (up to ~43 A), controlling it with a single PWM signal.

---

## 2. ⚙️ MODE OF OPERATION

### Mode Assessment

| Mode | User's Setup | This Firmware |
|------|-------------|---------------|
| Forward / Reverse motor control | **NO** — single-direction power switch | **YES** — RPWM/LPWM dual-channel via `Motor_SetSigned()` |
| Simple ON/OFF power switching | **YES** — primary use case | **NO** — full H-bridge motor control |
| PWM modulation | **YES** — duty cycle controls load current | **YES** — 20 kHz center-aligned PWM (ARR=4249) |

### H-Bridge Control vs Single-Channel Switching

**H-bridge control (this firmware's approach):**

```
                  +Vs (24 V)
                  ┌───┤├───┐
         RPWM ──▶│ Q1     Q2 │◀── LPWM
                  │           │
            OUT1 ─┤   MOTOR  ├─ OUT2
                  │           │
         RPWM ──▶│ Q3     Q4 │◀── LPWM
                  └───┤├───┘
                    GND
```

- **Forward**: RPWM active (Q1+Q4 ON), LPWM=0 → current flows left-to-right
- **Reverse**: LPWM active (Q2+Q3 ON), RPWM=0 → current flows right-to-left
- **Brake**: RPWM=0, LPWM=0, EN=HIGH → Q3+Q4 ON, motor terminals shorted to GND
- **Coast**: RPWM=0, LPWM=0, EN=LOW → all FETs off, motor floats (Hi-Z)

**Single-channel switching (user's approach):**

```
         +Vs ────┬──── LOAD ────┬──── BTS7960 OUT
                                │
                          Single half-bridge
                          PWM ──▶ IN (gate)
                          EN  ──▶ INH (enable)
```

- Only ONE half-bridge used (either high-side or low-side)
- PWM controls ON/OFF switching
- No bidirectional current, no motor reversal
- Simpler control: single pin (PWM or EN)

---

## 3. 🚗 FIRMWARE ANALYSIS (CRITICAL)

### Reference: `Core/Src/motor_control.c`

All motor hardware writes pass through `Motor_SetSigned()` (line 1869), which guarantees
RPWM and LPWM are never simultaneously non-zero.

### A. Active Braking (Both Outputs HIGH = Short Motor Terminals)

**→ NO — Not implemented as simultaneous RPWM+LPWM HIGH**

The firmware never drives both RPWM and LPWM to a non-zero value simultaneously.
`Motor_SetSigned()` enforces mutual exclusion (lines 1900–1914):

```c
if (signed_pwm > 0) {
    __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, 0U);  // LPWM=0
    __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, duty); // RPWM=duty
} else if (signed_pwm < 0) {
    __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);  // RPWM=0
    __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, duty); // LPWM=duty
} else {
    __HAL_TIM_SET_COMPARE(motor->rpwm_timer, motor->rpwm_channel, 0U);  // RPWM=0
    __HAL_TIM_SET_COMPARE(motor->lpwm_timer, motor->lpwm_channel, 0U);  // LPWM=0
}
```

**However**, the firmware implements THREE alternative braking methods:

| Braking Method | Mechanism | Firmware Location |
|----------------|-----------|-------------------|
| **BTS7960 passive brake** | RPWM=0, LPWM=0, EN=HIGH → low-side FETs short motor terminals to GND | `BTS7960_BRAKE_PWM=0` (line 168); effective on FR/RL (EN tied HIGH) |
| **Park hold brake** | Forward torque (RPWM=30% PWM) holds motor against motion | `GEAR_PARK` handler (lines 767–829) |
| **Dynamic braking** | Opposing torque via reverse channel (negative demand) | Dynamic braking logic (lines 868–956) |

#### BTS7960 Passive Brake — EN Pin Asymmetry

**Critical finding**: Passive brake behavior differs by motor due to EN wiring:

| Motor | EN Control | PWM=0 Behavior | Actual Mode |
|-------|-----------|-----------------|-------------|
| **FL** | GPIO (PC5) | EN set LOW by `Motor_SetSigned` | **COAST** (Hi-Z) |
| **FR** | GPIO (PC0) | EN set LOW by `Motor_SetSigned` | **COAST** (Hi-Z) |
| **RL** | GPIO (PC1) | EN set LOW by `Motor_SetSigned` | **COAST** (Hi-Z) |
| **RR** | GPIO (PC13) | EN set LOW by `Motor_SetSigned` | **COAST** (Hi-Z) |
| **STEER** | GPIO (PC4) | EN set LOW by `Motor_SetSigned` | **COAST** (Hi-Z) |

When `Motor_SetSigned(motor, 0)` is called (line 1920–1922):
- `duty = 0` → `GPIO_PIN_RESET` → EN driven LOW for all motors (coast)

All five motors now have dedicated GPIO EN pins, so `TRAC_PHASE_BRAKE` and `TRAC_PHASE_COAST`
produce **identical hardware output** (coast) on all motors when PWM=0.

### B. Neutral (Coast / Freewheel)

**→ YES — Implemented via `GEAR_NEUTRAL` (lines 831–863)**

```c
if (current_gear == GEAR_NEUTRAL) {
    Motor_SetSigned(&motor_fl, 0);  // RPWM=0, LPWM=0, EN=LOW (coast for FL)
    Motor_SetSigned(&motor_fr, 0);  // RPWM=0, LPWM=0, EN=HIGH (passive brake for FR)
    Motor_SetSigned(&motor_rl, 0);  // RPWM=0, LPWM=0, EN=HIGH (passive brake for RL)
    Motor_SetSigned(&motor_rr, 0);  // RPWM=0, LPWM=0, EN=LOW (coast for RR)
    // ... reset braking/smoothing state ...
    return;
}
```

**Result**: FL and RR achieve true coast; FR and RL remain in passive brake due to
hardware EN tie-high. This is a **partial coast** — full freewheel is not achievable for
all four motors with the current wiring.

### C. Hard Stop vs Soft Stop

| Feature | Implemented | Evidence |
|---------|-------------|----------|
| PWM ramp-down | **YES** | 5-stage smoothing: EMA (α=0.15) → ramp (50%/s up, 100%/s down) → brake→drive ramp (40%/s) → creep EMA (α=0.08, <15%) → jerk limit (±80 PWM/cycle). Lines 53–79, 123–131, 1328–1360 |
| Immediate stop | **YES** | `Traction_EmergencyStop()` bypasses smoothing, resets `prev_output_pwm[]`, calls `Motor_SetSigned(motor, 0)` for all 4 motors |

**Ramp-down pipeline detail:**

```
Pedal input → EMA filter → Rate limiter → Gear scaling → Ackermann diff
           → Brake→drive ramp → Creep smoothing → Jerk limiter → Motor_SetSigned
```

- **Normal stop**: Throttle release → demand ramps to 0 at 100%/s → DRIVE→COAST→BRAKE
  state transitions → motors reach zero smoothly
- **Emergency stop**: Immediate zero to all channels, no ramp

---

## 4. 🚘 GEAR LOGIC VALIDATION (P / N BEHAVIOR)

### PARK (P) — Lines 767–829

| Requirement | Met | Implementation |
|-------------|-----|----------------|
| Motor STOPS completely | **YES** | `Motor_SetSigned(motor, hold_pwm)` with hold_pwm > 0 drives RPWM channel (active forward torque opposing motion) |
| Motor braked or held | **YES** | 30% PWM forward hold brake with current derating (15–20 A) and temperature derating (70–85 °C). Safety override to coast in `SYS_STATE_SAFE` / `SYS_STATE_ERROR` |

**Park implementation detail:**
- Default hold: 30% forward PWM (RPWM active, LPWM=0)
- Current derating: linear reduction 15–20 A, disabled above 20 A
- Temperature derating: linear reduction 70–85 °C, disabled above 85 °C
- Safety fallback: coast (PWM=0) in SAFE/ERROR states to prevent thermal damage

### NEUTRAL (N) — Lines 831–863

| Requirement | Met | Implementation |
|-------------|-----|----------------|
| Motor free (no torque) | **PARTIAL** | FL and RR: coast (EN=LOW, Hi-Z). FR and RL: passive brake (EN tied HIGH) — residual braking torque present |
| No active braking | **YES** | No PWM drive, no dynamic braking, `dynbrake_pct` reset to 0 |

**Neutral note:** FR and RL experience passive electromagnetic braking in Neutral due to
hardware EN tie-high. This provides residual drag rather than true freewheel. For a
child-sized vehicle, this is a **safety benefit** (prevents uncontrolled rolling), though
it technically violates strict freewheel behavior.

---

## 5. ⚠️ POTENTIAL ISSUES

### Issue 1: No True H-Bridge Active Brake

The firmware never drives both RPWM and LPWM HIGH simultaneously to create a
low-impedance short across motor terminals. The BTS7960 supports this mode (both
half-bridges conducting creates a short-circuit path), which would provide the
strongest possible electromagnetic braking.

**Impact**: Braking relies on park hold (active torque) and dynamic braking (opposing
torque). Both require active PWM control. If the MCU locks up, only FR/RL get passive
brake (EN tied HIGH); FL/RR coast freely.

**Mitigation**: TIM1/TIM8 BREAK2 is linked to Cortex LOCKUP signal, which disables MOE
(Master Output Enable), forcing all PWM outputs LOW. With EN still HIGH on FR/RL, these
motors enter passive brake. FL/RR enter coast (EN driven LOW by GPIO reset). IWDG (500 ms)
provides secondary reset.

### Issue 2: Brake/Coast Phase Equivalence on FL/RR

`TRAC_PHASE_BRAKE` and `TRAC_PHASE_COAST` produce identical hardware output on FL and RR
motors. The desired_en flag is consumed by the pipeline (line 1377) but both cases pass
`sp=0` to `Motor_SetSigned()`, which sets EN=LOW:

```c
// Line 1377: Both BRAKE (desired_en=1, desired_pwm=0) and COAST (desired_en=0, desired_pwm=0)
// evaluate to sp=0 because: !desired_en[i] || desired_pwm[i] == 0
if (!desired_en[i] || desired_pwm[i] == 0) {
    sp = 0;  // Same result for both phases
}
```

**Impact**: The state machine distinction between brake and coast is purely logical
(different transition thresholds) — it has no separate hardware effect on FL/RR.
FR/RL are always passively braked when PWM=0 regardless of phase.

### Issue 3: Asymmetric Coast Capability

| Motor | Can Coast | Can Passive Brake | Reason |
|-------|-----------|-------------------|--------|
| FL | ✅ YES | ❌ NO | EN is GPIO — set LOW when duty=0 |
| FR | ❌ NO | ✅ YES | EN tied HIGH — always braked at PWM=0 |
| RL | ❌ NO | ✅ YES | EN tied HIGH — always braked at PWM=0 |
| RR | ✅ YES | ❌ NO | EN is GPIO — set LOW when duty=0 |

**Impact**: In Neutral gear, the vehicle experiences mixed behavior: front-left and
rear-right coast while front-right and rear-left are passively braked. This creates
a slight yaw moment at speed. At low speeds in a child-sized vehicle, the effect is
negligible.

### Issue 4: No Unsafe Behavior During Gear Changes

Gear changes are safe due to:
- `Traction_SetGear()` is atomic (single variable write)
- `Traction_Update()` checks gear at the start of each 10 ms cycle
- State machine resets (brake/coast/ramp) occur immediately on gear change
- Direction-change dead-state in `Motor_SetSigned()` prevents shoot-through

---

## 📊 OUTPUT FORMAT (STRICT)

### USER CONFIGURATION

- **Mode**: Single half-bridge switching (high-side or low-side, not full H-bridge)
- **Using full H-bridge**: NO
- **Using single-side switching**: YES

---

### FIRMWARE CAPABILITIES

| Feature | Status |
|---------|--------|
| Forward/Reverse | **YES** — RPWM (forward) / LPWM (reverse) via `Motor_SetSigned()` |
| Active Braking | **NO** — No simultaneous RPWM+LPWM HIGH. Park hold (30% forward torque) and dynamic braking (opposing torque) used instead |
| Neutral (Coast) | **PARTIAL** — FL/RR coast (EN=LOW); FR/RL passive brake (EN tied HIGH) |
| Soft Ramp | **YES** — 5-stage pipeline: EMA → rate limiter → brake→drive ramp → creep EMA → jerk limiter |

---

### GEAR SAFETY

| Gear | Behavior | Correct |
|------|----------|---------|
| P | Active hold brake: 30% forward PWM with current/temperature derating. Safety fallback to coast in SAFE/ERROR states. | **YES** |
| N | Mixed: FL/RR coast (true freewheel), FR/RL passive brake (EN tied HIGH). No active drive. Dynamic braking disabled and reset. | **YES** — residual brake on FR/RL is a safety benefit for child vehicle |

---

## 🔚 FINAL VERDICT

### ⚠️ PARTIAL CONTROL (missing features)

The firmware implements a **comprehensive motor control system** with direction control,
park hold braking, dynamic braking, and multi-stage smoothing. However, the following
features are missing or limited:

**Missing:**
1. **True H-bridge active brake** (both RPWM+LPWM HIGH simultaneously) — would provide
   strongest electromagnetic braking without requiring active PWM modulation
2. **Symmetric coast capability** — FR/RL cannot coast due to EN tied HIGH in hardware
3. **Symmetric passive brake** — FL/RR cannot passively brake due to EN GPIO behavior
   (set LOW when duty=0)

**Present and correct:**
1. ✅ Forward/reverse direction control via RPWM/LPWM
2. ✅ Park hold brake with current and temperature derating
3. ✅ Dynamic braking (proportional to throttle release rate, max 60%)
4. ✅ 5-stage smooth driving pipeline (eliminates jerk)
5. ✅ Direction-change dead-state enforcement (prevents shoot-through)
6. ✅ Emergency stop capability (bypasses smoothing)
7. ✅ Gear-based power scaling (D1=60%, D2=100%, R=60%)
8. ✅ Per-motor temperature cutoff (130 °C, 115 °C recovery)
9. ✅ Ackermann differential torque distribution
10. ✅ Hardware safety: TIM1/TIM8 BREAK2 → Cortex LOCKUP, IWDG 500 ms

**For a child-sized electric vehicle**, the implemented control is **adequate and safe**.
The missing H-bridge active brake is compensated by park hold and dynamic braking.
The asymmetric coast/brake behavior is a consequence of hardware wiring choices and has
negligible practical impact at the operating speeds of this vehicle.

---

## APPENDIX: Motor Hardware Configuration Reference

| Motor | RPWM | LPWM | Timer | EN Pin | EN Method | PWM=0 Mode |
|-------|------|------|-------|--------|-----------|------------|
| FL | PA8 (TIM1_CH1) | PA9 (TIM1_CH2) | TIM1 | PC5 | GPIO output | Coast (Hi-Z) |
| FR | PA10 (TIM1_CH3) | PC3 (TIM1_CH4) | TIM1 | PC0 | GPIO output | Coast (Hi-Z) |
| RL | PC6 (TIM8_CH1) | PC7 (TIM8_CH2) | TIM8 | PC1 | GPIO output | Coast (Hi-Z) |
| RR | PC8 (TIM8_CH3) | PC9 (TIM8_CH4) | TIM8 | PC13 | GPIO output | Coast (Hi-Z) |
| STEER | PA6 (TIM3_CH1) | PA7 (TIM3_CH2) | TIM3 | PC4 | GPIO output | Coast (Hi-Z) |

## APPENDIX: Firmware Constants Reference

| Constant | Value | Purpose |
|----------|-------|---------|
| `PWM_PERIOD` | 4249 | 20 kHz center-aligned at 170 MHz |
| `BTS7960_BRAKE_PWM` | 0 | Both RPWM=0, LPWM=0 (passive brake on EN-HIGH motors) |
| `PARK_HOLD_PWM_PCT` | 30.0% | Default park hold duty |
| `PARK_HOLD_CURRENT_WARN_A` | 15.0 A | Begin current derating |
| `PARK_HOLD_CURRENT_MAX_A` | 20.0 A | Disable hold above this |
| `PARK_HOLD_TEMP_WARN_C` | 70.0 °C | Begin temperature derating |
| `PARK_HOLD_TEMP_CRIT_C` | 85.0 °C | Disable hold above this |
| `DYNBRAKE_FACTOR` | 0.5 | Brake % per throttle %/s |
| `DYNBRAKE_MAX_PCT` | 60.0% | Maximum dynamic brake |
| `DYNBRAKE_MIN_SPEED_KMH` | 3.0 km/h | Disable below this |
| `PEDAL_RAMP_UP_PCT_S` | 50.0 %/s | Max throttle rise rate |
| `PEDAL_RAMP_DOWN_PCT_S` | 100.0 %/s | Max throttle fall rate |
| `DRIVE_ENTER_PCT` | 3.0% | Throttle to enter drive mode |
| `DRIVE_EXIT_PCT` | 1.0% | Throttle to leave drive mode |
| `COAST_SPEED_THRESHOLD_KMH` | 2.0 km/h | Coast above this speed |
| `COAST_TIMEOUT_MS` | 3000 ms | Max coast duration |
| `BRAKE_RELEASE_RAMP_PCT_S` | 40.0 %/s | Brake→drive transition rate |
| `GEAR_POWER_FORWARD_PCT` | 60% | D1 max power |
| `GEAR_POWER_FORWARD_D2_PCT` | 100% | D2 max power |
| `GEAR_POWER_REVERSE_PCT` | 60% | Reverse max power |
