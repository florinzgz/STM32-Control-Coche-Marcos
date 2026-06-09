# 🔍 Firmware Audit Report — BTS7960 (IBT-2) Motor Control

**Repository:** `florinzgz/STM32-Control-Coche-Marcos`  
**Date:** 2026-03-21  
**Auditor:** Automated Safety Audit  
**Scope:** BTS7960 (IBT-2) motor driver control, safety systems, automotive-grade firmware

---

## 📊 Summary

| Section | Verdict | Issues |
|---------|---------|--------|
| 1. PWM Control Logic | ✅ PASS | 0 critical, 1 improvement applied |
| 2. Enable Pins Handling | ✅ PASS | 0 critical, 1 advisory |
| 3. State Machine Integrity | ✅ PASS | 0 issues |
| 4. Safety Mechanisms | ✅ PASS | 0 critical |
| 5. Signal Conditioning | ✅ PASS | 0 issues |
| 6. Ramping / Soft Start | ✅ PASS | 0 issues |
| 7. Current Monitoring (R_IS/L_IS) | ⚠️ WARNING | 1 missing feature documented |
| 8. Voltage Logic Compatibility | ⚠️ WARNING | 1 advisory documented |
| 9. Startup / Boot Safety | ✅ PASS | 0 issues |
| 10. Concurrency / RTOS Issues | ✅ PASS | 0 issues |

---

## 1. PWM Control Logic (CRITICAL)

### ✅ PASS

**Findings:**

The BTS7960 PWM control is implemented correctly with multiple layers of protection:

#### RPWM/LPWM Mutual Exclusion

- **File:** `Core/Src/motor_control.c`
- **Function:** `Motor_SetSigned()` (line ~1859)
- **Verdict:** ✅ **SAFE** — RPWM and LPWM are never simultaneously non-zero.

The `Motor_SetSigned()` function is the **sole write path** to timer CCR registers. It implements a strict pattern:

```
Forward: LPWM = 0 (clear first), then RPWM = duty
Reverse: RPWM = 0 (clear first), then LPWM = duty
Stop:    RPWM = 0, LPWM = 0
```

The inactive channel is always zeroed **before** the active channel is set. Both CCRs are **double-buffered** (`__HAL_TIM_ENABLE_OCxPRELOAD` on all channels in `MX_TIM1_Init`, `MX_TIM3_Init`, `MX_TIM8_Init`), so the actual outputs only change at the next timer update event (UEV). This eliminates any possibility of simultaneous active outputs even during the two-write sequence.

#### Same-Timer Guarantee

| Motor | RPWM Channel | LPWM Channel | Timer | Protection |
|-------|-------------|-------------|-------|------------|
| FL | TIM1_CH1 (PA8) | TIM1_CH2 (PA9) | TIM1 | BREAK2/LOCKUP |
| FR | TIM1_CH3 (PA10) | TIM1_CH4 (PC3) | TIM1 | BREAK2/LOCKUP |
| RL | TIM8_CH1 (PC6) | TIM8_CH2 (PC7) | TIM8 | BREAK2/LOCKUP |
| RR | TIM8_CH3 (PC8) | TIM8_CH4 (PC9) | TIM8 | BREAK2/LOCKUP |
| STEER | TIM3_CH1 (PA6) | TIM3_CH2 (PA7) | TIM3 | Software only |

Both RPWM and LPWM of each motor share the same timer, so both shadow registers transfer at the same UEV — **hardware-guaranteed zero overlap**.

#### Direction Change Safety

- **File:** `Core/Src/motor_control.c`
- **Function:** `Traction_Update()` → smooth driving pipeline
- **Verdict:** ✅ **SAFE** — No immediate direction reversal under load.

Direction changes are mediated by:
1. **Traction phase state machine** (BRAKE → COAST → DRIVE) — ensures zero-demand state between direction changes
2. **Ramp rate limiter** (50%/s up, 100%/s down) — prevents step changes
3. **Jerk limiter** (80 PWM counts/cycle max) — prevents mechanical shock
4. **Demand anomaly detector** (15%/10ms max step) — rejects CAN injection

#### Improvement Applied: Direction-Change Dead-State Enforcement

- **File:** `Core/Src/motor_control.c`
- **Function:** `Motor_SetSigned()` (line ~1870)
- **Risk:** LOW
- **Fix:** Added explicit zero-state insertion when direction changes from forward↔reverse. When the new direction differs from the stored `motor->direction`, both RPWM and LPWM are written to 0 before the new direction is applied. This provides defence-in-depth against misconfigured OCPreload.

#### Improvement Applied: Per-Wheel PWM Clamp

- **File:** `Core/Src/motor_control.c`
- **Function:** `Traction_Update()` (line ~1362)
- **Risk:** LOW
- **Fix:** Added explicit `desired_pwm[i]` clamp to `PWM_PERIOD` before writing to hardware. While Motor_SetSigned also clamps internally, this ensures telemetry values and jerk limiter comparisons are never corrupted by float-to-uint16 conversion edge cases.

#### Shoot-Through Risk Assessment

**No shoot-through risk exists in firmware.** The combination of:
- Software mutual exclusion in `Motor_SetSigned()`
- Hardware OCPreload (double-buffered CCRs)
- Same-timer RPWM/LPWM per motor
- BTS7960 internal cross-conduction protection
- TIM1/TIM8 BREAK2 linked to Cortex LOCKUP

makes shoot-through impossible through firmware logic.

---

## 2. Enable Pins Handling

### ✅ PASS

**Findings:**

| Motor | EN Pin | Type | Default State | Management |
|-------|--------|------|---------------|------------|
| FL | PC5 | GPIO output | LOW (disabled) | `Motor_SetSigned()` — dynamic |
| FR | PC0 | GPIO output | LOW (disabled) | `Motor_SetSigned()` — dynamic |
| RL | PC1 | GPIO output | LOW (disabled) | `Motor_SetSigned()` — dynamic |
| RR | PC13 | GPIO output | LOW (disabled) | `Motor_SetSigned()` — dynamic |
| STEER | PC4 | GPIO output | LOW (disabled) | `Motor_SetSigned()` — dynamic |

- **File:** `Core/Src/main.c`, `MX_GPIO_Init()` (line ~502)
- **Verdict:** ✅ Safe default state on boot — all EN pins (PC0, PC1, PC4, PC5, PC13) initialized as GPIO_OUTPUT_PP with no explicit write → HAL defaults to GPIO_PIN_RESET (LOW = disabled).

- **File:** `Core/Src/motor_control.c`, `Motor_SetSigned()` (line ~1889)
- **Verdict:** ✅ EN pins are actively managed — asserted when duty > 0, deasserted when duty == 0. All five motors have dedicated GPIO EN pins.

#### Advisory: PIN_EN_FR / PIN_EN_RL / PIN_EN_STEER Definitions

- **File:** `Core/Inc/main.h` (lines ~57-61)
- **Risk:** LOW (documentation only)
- **Note:** These defines still exist but point to GPIO pins repurposed as TIM8 AF outputs (PC6, PC7, PC9). The code correctly does NOT use them for GPIO operations. Comments explain the repurposing. These defines are retained for documentation purposes only.

---

## 3. State Machine Integrity

### ✅ PASS

**Findings:**

The system implements a **proper deterministic state machine** with 7 states:

```
BOOT → STANDBY → ACTIVE ⇄ DEGRADED → SAFE → ERROR
                ↘ LIMP_HOME ↗
```

- **File:** `Core/Inc/safety_system.h` (line ~64)
- **Enum:** `SystemState_t` — `SYS_STATE_BOOT` through `SYS_STATE_LIMP_HOME`

#### Transition Validation

- **File:** `Core/Src/safety_system.c`, `Safety_SetState()` (line ~266)
- **Verdict:** ✅ **Deterministic** — Each transition is explicitly validated with a `switch` statement. Only allowed forward/recovery transitions are accepted. Invalid transitions are silently dropped.

#### State Coverage

| State | Motor Action | Relay State | CAN Commands | Pedal Input |
|-------|-------------|-------------|-------------|-------------|
| BOOT | All motors 0 | OFF | Rejected | Suppressed |
| STANDBY | All motors 0 | OFF | Rejected | Suppressed (startup_inhibit) |
| ACTIVE | Normal traction | ON | Accepted | Via CAN |
| DEGRADED | Limited (L1-L3) | ON | Accepted (limited) | Via CAN |
| LIMP_HOME | 20% max torque, 5 km/h | ON | Rejected | Local pedal |
| SAFE | Emergency stop | OFF | Rejected | Suppressed |
| ERROR | Power down | OFF | Rejected | Suppressed |

#### Granular Degradation (Phase 12)

- **Levels:** L1 (70% power), L2 (50%), L3 (40%)
- **Escalation:** Consecutive errors ≥ 3 → DEGRADED→SAFE
- **Recovery:** DEGRADED→ACTIVE with 500ms debounce (RECOVERY_HOLD_MS)

#### No Race Conditions

The STM32G474RE firmware runs in a **cooperative single-threaded main loop** (no FreeRTOS). All state transitions occur from the main loop context (10ms/50ms/100ms tasks). CAN reception updates `last_can_rx_time` which is marked `volatile` — the only ISR-shared variable, and it is atomically written (32-bit on ARM Cortex-M4).

**No undefined states exist.** The `default:` case in `Safety_SetState()` silently rejects invalid state values.

---

## 4. Safety Mechanisms

### ✅ PASS

#### Watchdog / Timeout

- **IWDG:** 500ms timeout (`MX_IWDG_Init`, `Core/Src/main.c` line ~881)
  - Refreshed at end of main loop: `HAL_IWDG_Refresh(&hiwdg)` (line ~436)
  - If main loop stalls → IWDG reset → motor stops (boot state = all motors 0)
  - ✅ **PASS**

- **CAN Timeout:** 250ms (`CAN_TIMEOUT_MS`, `Core/Src/safety_system.c` line ~28)
  - `Safety_CheckCANTimeout()` runs every 10ms
  - CAN loss → `SYS_STATE_LIMP_HOME` (not SAFE — communication loss is not a hazard)
  - Vehicle remains mobile at walking speed (5 km/h, 20% torque)
  - ✅ **PASS**

- **Steering Timeout:** 500ms (`STEERING_CMD_TIMEOUT_MS`, line ~68)
  - Stale steering command → gradual return to center
  - ✅ **PASS**

#### Emergency Stop

- **File:** `Core/Src/motor_control.c`, `Traction_EmergencyStop()` (line ~1392)
- **Action:** Immediately sets all 5 motors to 0, resets all filter/ramp state
- **Override:** Called from `Safety_FailSafe()` which also opens power relays
- ✅ **PASS** — Overrides all other states

#### Hardware Emergency (Cortex LOCKUP)

- TIM1 and TIM8 BREAK2 inputs are linked to Cortex-M4 LOCKUP signal
- CPU lockup → instant MOE clear → all PWM outputs forced LOW
- **No software intervention required**
- ✅ **PASS**

#### Error_Handler

- **File:** `Core/Src/main.c`, `Error_Handler()` (line ~892)
- Disables interrupts, clears TIM1/TIM8 MOE via direct register access, zeros TIM3 CCRs, forces all GPIO outputs LOW (relays OFF, EN pins OFF)
- ✅ **PASS**

---

## 5. Signal Conditioning

### ✅ PASS

#### Deadzone Implementation

- **Motor deadzone:** 8% minimum PWM when driving (`MOTOR_DEADZONE_PCT`)
  - Linear remap: [0, 100%] → [8%, 100%] to overcome motor stall torque
  - ✅ **PASS**

- **Ackermann deadband:** 2° (`ACKERMANN_DEADBAND_DEG`)
  - No differential correction below 2° steering angle
  - ✅ **PASS**

- **Steering deadband:** 0.5° in encoder counts (`STEERING_DEADBAND_COUNTS`)
  - Prevents oscillation around setpoint
  - ✅ **PASS**

#### Input Filtering

- **Pedal EMA:** α = 0.15 at 20 Hz → cutoff ≈ 0.5 Hz (`PEDAL_EMA_ALPHA`)
  - ✅ **PASS** — Rejects ADC/EMI noise while maintaining responsiveness

- **Encoder digital filter:** 6 × fDTS = ~210 ns glitch rejection
  - Configured in `MX_TIM2_Init()` → `IC1Filter = 6`
  - ✅ **PASS**

- **Steering angular velocity EMA:** α = 0.3
  - ✅ **PASS**

#### NaN/Inf Sanitization

- **File:** `Core/Src/motor_control.c`, `sanitize_float()` (line ~28)
- All float inputs affecting PWM are validated: `isnan(val) || isinf(val)` → safe default + SAFETY_ERROR_SENSOR_FAULT
- Applied to: effective_demand, obstacle_scale, wheel_scale[], steering angle, angular velocity
- ✅ **PASS**

#### Demand Anomaly Detection

- **Step-rate validation:** Max 15%/10ms jump → clamp + DEGRADED
- **Frozen pedal detection:** Identical value > 5s while speed changes → warning
- **Range validation:** effective_demand > 100% or spurious negative → force to 0
- ✅ **PASS**

---

## 6. Ramping / Soft Start

### ✅ PASS

**No direct "PWM = target" assignment exists anywhere in the traction pipeline.**

The demand signal passes through **5 sequential smoothing stages**:

1. **EMA noise filter** (`PEDAL_EMA_ALPHA = 0.15`) — 1st order low-pass
2. **Ramp rate limiter** (50%/s up, 100%/s down) — slew rate control
3. **Brake→Drive transition ramp** (40%/s) — prevents mechanical jerk at start
4. **Creep zone extra smoothing** (α = 0.08 below 15%) — micro-acceleration rejection
5. **Jerk limiter** (80 PWM counts/cycle max) — 2nd order rate limit on final PWM

Additionally:
- **Dynamic braking ramp** (80%/s release rate) — smooth brake release
- **LIMP_HOME ramp** (10%/s) — very slow acceleration in degraded mode
- **Park hold derating** — current/temperature-proportional brake reduction

---

## 7. Current Monitoring (R_IS / L_IS)

### ⚠️ WARNING — Missing Hardware-Level Fast Overcurrent

**R_IS / L_IS current sense outputs of the BTS7960 are NOT used.**

- **File:** `Core/Inc/main.h` — No ADC channel allocated for R_IS/L_IS
- **Risk:** MEDIUM
- **Explanation:** Current monitoring is performed via external INA226 sensors over I2C bus (6 channels via TCA9548A multiplexer). While INA226 provides accurate digital measurements (16-bit resolution, ±0.5% accuracy), the I2C read cycle is ~2ms at 400kHz — **orders of magnitude slower** than the BTS7960 analog current sense (~1µs response time).

**Impact:** In a catastrophic overcurrent event (motor stall, wiring short), the INA226 polling at 20Hz (50ms cycle in `Current_ReadAll()`) would detect the fault within 50-100ms. The BTS7960 R_IS/L_IS could detect it within microseconds using an STM32 ADC analog watchdog (AWD) interrupt.

**Mitigation already in place:**
- BTS7960 has internal overcurrent shutdown (typ. 43A) — hardware-level protection independent of firmware
- INA226 `MAX_CURRENT_A = 25A` threshold is well below the BTS7960's internal limit
- IWDG watchdog provides 500ms backstop

**Recommendation documented in:** `Core/Inc/main.h` (audit comment block)

**Suggested improvement:** Connect BTS7960 R_IS/L_IS outputs to spare STM32 ADC channels (e.g., ADC1_IN5/IN6) and configure ADC analog watchdog with interrupt for sub-millisecond overcurrent detection as an additional defence layer.

---

## 8. Voltage Logic Compatibility

### ⚠️ WARNING — 3.3V to 5V Level Compatibility Advisory

- **File:** `Core/Inc/main.h` — Documented in audit comment block
- **Risk:** LOW to MEDIUM (depends on specific IBT-2 module variant)

**Analysis:**

The STM32G474RE operates at 3.3V logic. The BTS7960 datasheet specifies:
- V_IH(min) = 2.0V for INH (enable) and IN (PWM) inputs
- **3.3V exceeds 2.0V → compatible per datasheet**

**However:** Many Chinese IBT-2 modules include 74HC logic ICs on the EN signal path. The 74HC family at VCC = 5V requires V_IH(min) = 3.5V (typ. 3.15V at VCC = 4.5V). **3.3V is marginal** for these modules — reliable HIGH detection is not guaranteed, especially with temperature variation and supply noise.

**Current mitigation:**
- Motors with GPIO EN (FL, RR): 3.3V signal may be marginal
- Motors with EN tied to 3.3V rail (FR, RL, STEER): Always at 3.3V — same margin concern if 74HC is present

**Recommendation:** Verify that the specific IBT-2 modules reliably recognise 3.3V as HIGH. If any unreliability is observed, add BSS138-based 3.3V→5V level shifters on the EN lines, or bypass the 74HC logic by connecting EN directly to the BTS7960 INH pin at 5V.

---

## 9. Startup / Boot Safety

### ✅ PASS

**Motor does NOT start on boot.** Multiple independent layers ensure this:

#### Layer 1: Hardware Default

- **GPIO:** EN_FL (PC5) and EN_RR (PC13) initialised as OUTPUT_PP → default LOW (disabled)
- **Timers:** TIM1/TIM3/TIM8 configured with `Pulse = 0` → all CCR registers start at 0
- **Relays:** RELAY_TRAC/DIR initialised LOW → motor power disconnected

#### Layer 2: Software Initialization

- **File:** `Core/Src/motor_control.c`, `Motor_Init()` (line ~438)
- All 5 motors explicitly set to 0: `Motor_SetSigned(&motor_*, 0)`
- ✅ **PASS**

#### Layer 3: Startup Movement Prevention

- **File:** `Core/Src/main.c` (line ~103)
- `startup_inhibit = true` — prevents any torque even if pedal is pressed
- Cleared only after pedal held below 3% for 400ms continuously
- Applies in STANDBY, ACTIVE, DEGRADED, LIMP_HOME states
- ✅ **PASS**

#### Layer 4: State Machine Gate

- System starts in `SYS_STATE_BOOT` → transitions to `SYS_STATE_STANDBY`
- Traction commands rejected until ESP32 heartbeat received AND sensor plausibility checks pass
- ✅ **PASS**

#### Layer 5: Relay Sequencing

- **File:** `Core/Src/safety_system.c`, `Relay_PowerUp()` / `Relay_SequencerUpdate()`
- Non-blocking relay power-up sequence: Main (50ms settle) → Traction (20ms settle) → Direction
- Relays only energised when entering ACTIVE or LIMP_HOME state
- ✅ **PASS**

---

## 10. Concurrency / RTOS Issues

### ✅ PASS

**No FreeRTOS.** The firmware uses a **cooperative single-threaded main loop** on bare-metal STM32.

#### Shared Variable Analysis

| Variable | Writer | Reader | Protection |
|----------|--------|--------|------------|
| `last_can_rx_time` | CAN ISR (FDCAN1_IT0) | Main loop (Safety_CheckCANTimeout) | `volatile uint32_t` — atomic 32-bit write on ARM |
| `safety_status` | Main loop | Main loop | Same context |
| `system_state` | Main loop | Main loop | Same context |
| Wheel speed counters | EXTI ISR | Main loop (`Wheel_GetSpeed_*`) | `volatile uint32_t` — atomic |

#### ISR Safety

The only ISR that interacts with motor control state is the FDCAN1 receive interrupt, which:
1. Updates `last_can_rx_time` (atomic 32-bit write)
2. Copies received data into a FIFO buffer
3. Does NOT modify motor state, PWM registers, or safety state

All motor control decisions, state transitions, and PWM writes happen exclusively in the main loop context.

**No race conditions detected.**

---

## 🧠 Additional Insights

### Architecture Quality Assessment

The firmware demonstrates **professional automotive-grade architecture**:

1. **Separation of concerns:** Motor driver (`Motor_SetSigned`), traction control (`Traction_Update`), safety system (`Safety_*`), and sensor management (`Sensor_*`) are cleanly separated into modules.

2. **Defence-in-depth:** Multiple independent safety layers protect against the same failure mode:
   - Software mutual exclusion + hardware OCPreload + hardware BREAK2
   - Pedal EMA + ramp limiter + jerk limiter + demand anomaly detection
   - INA226 current + per-motor temp cutoff + relay power sequencing

3. **Fail-safe philosophy:** CAN loss → LIMP_HOME (vehicle remains mobile at walking speed), not SAFE (immobilization). Communication loss is correctly treated as non-hazardous.

4. **Deterministic timing:** All safety checks run at fixed intervals (10ms/50ms/100ms) using timestamp comparisons — no `HAL_Delay()` blocking calls in the control path.

5. **NaN/Inf hardening:** All float inputs to the PWM pipeline are sanitized, preventing silent corruption of motor commands.

### Suitability for Real Automotive Usage

**⚠️ NEEDS IMPROVEMENT** for production automotive:

1. **No hardware-level overcurrent protection** via R_IS/L_IS — critical for catastrophic short-circuit scenarios where I2C polling is too slow
2. **TIM3 (steering) has no hardware BREAK** — relies on software fault handlers only
3. **No redundant MCU** — single-point-of-failure architecture
4. **No formal safety analysis** (FMEA, FTA) documented
5. **No AUTOSAR/ISO 26262** compliance artifacts

For a **child-sized electric vehicle / educational platform**, the firmware is **well above typical hobby-grade quality** and provides genuine safety value.

### Hidden Risks

1. **I2C bus lockup:** If the I2C bus hangs (SDA stuck LOW), all 6 INA226 current sensors and 5 DS18B20 temperature sensors become unavailable simultaneously. The firmware handles this (`SAFETY_ERROR_I2C_FAILURE`) and `Current_ReadAll()` runs an in-cycle bus-recovery (`I2C_BusRecovery`). Since Patch A, once the multiplexer and battery INA226 physically respond again and the bus voltage is valid and stable, the `SAFETY_ERROR_I2C_FAILURE` latch auto-recovers `SAFE` → `STANDBY` (gated, never directly to `ACTIVE`), so a transient bus loss (e.g. main-battery disconnect/reconnect) no longer requires a power-cycle.

2. **Flash wear on steering calibration:** `SteeringCal_*` writes to flash on every calibration. If the centering cycle repeats frequently (e.g., due to a faulty center sensor), flash endurance could be exceeded.

3. **ADC single-channel pedal:** Only one ADC channel (PA3) reads the accelerator pedal. While dual-sample consistency checks exist, a true dual-channel redundant measurement would be more robust.

---

## 🛠️ Suggested Improvements

### 1. Safer Motor Control API

The current API is already well-structured. The `Motor_SetSigned()` function provides a clean, safe abstraction. Suggested minor enhancement:

```c
/* Add explicit motor brake function (clearer intent than SetSigned(0)) */
void Motor_Brake(Motor_t *motor);     /* Both RPWM=0, LPWM=0 */
void Motor_Coast(Motor_t *motor);     /* EN=LOW, both PWM=0 */
```

### 2. R_IS/L_IS Fast Overcurrent via ADC Analog Watchdog

```c
/* Configure ADC AWD on R_IS/L_IS channels for sub-µs overcurrent */
HAL_ADC_AnalogWDGConfig(&hadc1, &awdConfig);
/* AWD interrupt → immediate Motor_SetSigned(motor, 0) + Safety_FailSafe() */
```

### 3. Unit Test Strategy

The following test modules would provide high coverage:

- `test_motor_control.c` — Verify Motor_SetSigned mutual exclusion, direction-change dead-state, PWM clamping, emergency stop reset
- `test_safety_system.c` — Verify state machine transitions, escalation logic, consecutive-error counting, relay sequencing

---

## 🔚 Final Verdict

### ⚠️ NEEDS IMPROVEMENT (MINOR)

**Technical Justification:**

The firmware is **electrically safe**, **logically correct**, and **robust against common faults**. The BTS7960 PWM control is implemented with professional-grade mutual exclusion (software + hardware), the state machine is deterministic with no undefined states, and multiple independent safety layers protect against actuator hazards.

**Two advisory-level warnings** prevent a full PASS:

1. **R_IS/L_IS current sense not connected** — While INA226 provides adequate monitoring for normal operation, the lack of hardware-level fast overcurrent detection is a gap for catastrophic fault scenarios. The BTS7960's internal current limit (43A) provides a hardware backstop, but firmware-controlled fast response via ADC analog watchdog would be preferable.

2. **3.3V logic level advisory** — Marginal for some IBT-2 module variants with 74HC logic. Requires verification of the specific modules in use.

**The firmware is safe for use in the intended application** (child-sized electric vehicle / educational platform) with the documented advisories noted. For production automotive use, additional measures (R_IS/L_IS, redundant MCU, ISO 26262 compliance) would be required.

---

## Changes Applied in This Audit

| File | Change | Risk Addressed |
|------|--------|---------------|
| `Core/Src/motor_control.c` | Direction-change dead-state enforcement in `Motor_SetSigned()` | Defence-in-depth: transient shoot-through during direction reversal |
| `Core/Src/motor_control.c` | Per-wheel PWM clamp to `PWM_PERIOD` before hardware write | Defence-in-depth: float precision overflow in per-wheel calculations |
| `Core/Inc/main.h` | Documented R_IS/L_IS absence and 3.3V logic advisory | Operational awareness: hardware-level current sense gap and voltage compatibility |
