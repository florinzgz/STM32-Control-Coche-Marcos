# Low-Speed Control Strategy — Deterministic Creep Behavior

> **Scope**: Concrete control architecture for smooth low-speed operation.
> No hardware changes. No driver changes. Firmware-only design.
>
> **Target behavior**: Walking-pace creep like an automatic-transmission vehicle.
>
> **Source files referenced**: `Core/Src/motor_control.c`, `Core/Inc/motor_control.h`,
> `Core/Src/main.c`, `Core/Inc/main.h`, `Core/Inc/vehicle_physics.h`,
> `Core/Inc/safety_system.h`, `Core/Inc/sensor_manager.h`.
>
> **Related analysis**: `docs/LOW_THROTTLE_BEHAVIOR_ANALYSIS.md`,
> `docs/BTS7960_MOTOR_DRIVER_AUDIT.md`, `docs/TIMER_PWM_ANALYSIS.md`.

---

## System Context and Constraints

### Hardware (immutable)

| Resource | Detail |
|----------|--------|
| MCU | STM32G474RE @ 170 MHz |
| Motors | 4× brushed DC, 24 V bus |
| Drivers | BTS7960 / IBT-2 (DIR + single PWM + EN per motor) |
| PWM timer | TIM1 CH1–4, center-aligned, ARR = 4249, 20 kHz |
| Wheel speed | 4× Hall sensors, 6 pulses/rev (EXTI), min detectable ≈ 0.5 km/h |
| Current sense | INA226 via I2C/TCA9548A, 20 Hz sample rate |
| Bus voltage | INA226 on TCA9548A channel 4 (battery bus, `INA226_CHANNEL_BATTERY`), 20 Hz |
| Temperature | DS18B20 OneWire, ~1 Hz per sensor |
| Control loop | 100 Hz (10 ms period, `Traction_Update()`) |
| Pedal update | 20 Hz (50 ms period, `Pedal_Update()` → `Traction_SetDemand()`) |

### BTS7960 Quirk (Chinese Module)

- **EN=HIGH, PWM=0** → motor **floats** (not braked) — this is a defect.
- **EN=HIGH, PWM=100%** → motor terminals **shorted** → electromagnetic brake.
- **EN=LOW** → H-bridge disabled → coast mode.

All braking strategies in this document respect this quirk.

### Identified Problems (from `LOW_THROTTLE_BEHAVIOR_ANALYSIS.md`)

1. **Binary discontinuity** at `TRACTION_ZERO_DEMAND_PCT = 0.5 %`:
   PWM jumps 4249→~25 (100% brake → 0.6% drive) in one 10 ms cycle.
2. **Motor dead zone**: 0–6 % duty produces current but no rotation.
3. **Stick-slip**: Motor stiction + dead zone → lurch/stop/lurch cycle.
4. **No coasting region**: Throttle=0 → immediate full electromagnetic brake.
5. **Voltage-mode control**: No torque/current feedback in the traction path.

---

## 1. Low-Speed State Machine

### 1.1 State Definitions

```
┌───────────┐     ┌───────────┐     ┌───────────┐     ┌───────────┐
│ HOLD_STOP │────►│   COAST   │────►│   CREEP   │────►│   DRIVE   │
│           │◄────│           │◄────│           │◄────│           │
└─────┬─────┘     └─────┬─────┘     └─────┬─────┘     └─────┬─────┘
      │                 │                 │                 │
      │    ┌────────────┴─────────────────┴─────────────────┘
      │    │
      │    ▼
      │  ┌───────────┐
      ├─►│   BRAKE   │
      │  └─────┬─────┘
      │        │
      ▼        ▼
┌───────────────────┐
│    EMERGENCY      │
└───────────────────┘
```

### 1.2 Per-State Specification

#### HOLD_STOP — Vehicle Stationary, No Pedal Input

**Purpose**: Hold vehicle stationary on flat ground or mild slope.
Equivalent to automatic transmission's brake-pedal-held state.

| Parameter | Value |
|-----------|-------|
| **PWM** | `BTS7960_BRAKE_PWM` (4249 = 100 % duty) |
| **EN** | HIGH (all driven motors) |
| **DIR** | Last direction (irrelevant at 100 % duty — both FETs conduct) |
| **Electrical effect** | Motor terminals shorted → electromagnetic hold |

**Entry conditions** (any one):
- Vehicle speed < 0.5 km/h AND pedal < `HOLD_ENTER_PEDAL_PCT` (1.0 %) for > `HOLD_SETTLE_MS` (200 ms)
- Transition from BRAKE when speed reaches zero
- System startup (initial state for D/R gears)
- Gear change to FORWARD/REVERSE from PARK or NEUTRAL

**Exit conditions**:
| Target state | Condition |
|-------------|-----------|
| COAST | Pedal > `COAST_ENTER_PEDAL_PCT` (1.0 %) AND pedal < `CREEP_ENTER_PEDAL_PCT` (3.0 %) — held for > `HOLD_EXIT_DEBOUNCE_MS` (50 ms) |
| CREEP | Pedal > `CREEP_ENTER_PEDAL_PCT` (3.0 %) — held for > `HOLD_EXIT_DEBOUNCE_MS` (50 ms) |
| EMERGENCY | Safety system triggers `Traction_EmergencyStop()` |

**Time constants**:
- `HOLD_SETTLE_MS = 200`: Debounce entry to prevent rapid state flapping.
- `HOLD_EXIT_DEBOUNCE_MS = 50`: Prevent noise-triggered departure from hold.

**Hysteresis**: Once in HOLD_STOP, pedal must exceed 1.0 % (not 0.5 %)
to exit. This creates a 0.5 % dead zone that absorbs ADC noise and
prevents the brake/drive oscillation identified in the low-throttle analysis.

---

#### COAST — Brake Released, No Drive Torque

**Purpose**: Transitional state between brake hold and driving.
The vehicle is free to roll under external forces (gravity, momentum)
but no motor torque is applied. Simulates the moment when an automatic
transmission's torque converter begins slipping before the vehicle creeps.

| Parameter | Value |
|-----------|-------|
| **PWM** | Ramping down from `BTS7960_BRAKE_PWM` toward 0 (see Section 3) |
| **EN** | Transitions from HIGH → LOW during ramp (coast when PWM=0) |
| **DIR** | Forward (set for upcoming drive direction) |
| **Electrical effect** | Gradual brake release → motor float → no holding torque |

**Entry conditions**:
- From HOLD_STOP: pedal > `COAST_ENTER_PEDAL_PCT` (1.0 %)
- From CREEP/DRIVE: pedal < `COAST_REENTER_PEDAL_PCT` (1.5 %) AND speed > 0.5 km/h

**Exit conditions**:
| Target state | Condition |
|-------------|-----------|
| CREEP | Pedal > `CREEP_ENTER_PEDAL_PCT` (3.0 %) — immediate |
| HOLD_STOP | Speed < 0.5 km/h AND pedal < 1.0 % for > `HOLD_SETTLE_MS` (200 ms) |
| BRAKE | Pedal = 0 AND speed > 3.0 km/h (dynamic braking active) |
| EMERGENCY | Safety system trigger |

**Time constants**:
- Brake release ramp: `BRAKE_RELEASE_RAMP_MS = 80` (see Section 3.1).
- Maximum dwell time: none — COAST can persist as long as pedal is in
  [1.0 %, 3.0 %) range and vehicle is moving.

**Hysteresis**:
- Entry from HOLD_STOP requires pedal > 1.0 %.
- Entry from CREEP/DRIVE requires pedal < 1.5 % (not 1.0 %).
  The 0.5 % gap prevents flapping between COAST and CREEP.

---

#### CREEP — Automatic Transmission Creep Torque

**Purpose**: Apply minimum effective motor voltage sufficient to
overcome static friction and move the vehicle at walking pace (2–5 km/h)
without any pedal input beyond the threshold. This is the core of the
"automatic transmission feel" — foot off brake → vehicle rolls forward slowly.

| Parameter | Value |
|-----------|-------|
| **PWM** | `CREEP_PWM_MIN` to `CREEP_PWM_MAX` (see Section 2) |
| **EN** | HIGH |
| **DIR** | Set by current gear (forward/reverse) |
| **Electrical effect** | Motor receives minimum-effective voltage → slow rotation |

The creep duty is computed from the pedal-to-torque mapping (Section 2).
At the CREEP entry threshold (3.0 %), the mapped PWM output starts at
`CREEP_PWM_MIN` — the empirically-determined minimum duty cycle for the
motor to reliably begin rotating under worst-case load (uphill + full
vehicle weight).

**Entry conditions**:
- From COAST: pedal ≥ `CREEP_ENTER_PEDAL_PCT` (3.0 %)
- From HOLD_STOP: pedal ≥ `CREEP_ENTER_PEDAL_PCT` (3.0 %)
- From DRIVE: speed < `CREEP_SPEED_MAX_KMH` (5.0 km/h) AND pedal < `DRIVE_ENTER_PEDAL_PCT` (8.0 %)

**Exit conditions**:
| Target state | Condition |
|-------------|-----------|
| DRIVE | Pedal > `DRIVE_ENTER_PEDAL_PCT` (8.0 %) OR speed > `CREEP_SPEED_MAX_KMH` (5.0 km/h) |
| COAST | Pedal < `COAST_REENTER_PEDAL_PCT` (1.5 %) |
| HOLD_STOP | Speed < 0.5 km/h AND pedal < 1.0 % for > 200 ms |
| BRAKE | Pedal = 0 AND speed > 3.0 km/h |
| EMERGENCY | Safety system trigger |

**Time constants**:
- PWM ramp-up from 0 to `CREEP_PWM_MIN`: `CREEP_RAMP_UP_MS = 150`.
  Slow enough to prevent lurch, fast enough to feel responsive.
- Target creep speed regulation: soft speed cap at 5 km/h by reducing
  duty when speed exceeds target (see Section 4).

**Hysteresis**:
- Entry at 3.0 % pedal, exit to COAST at 1.5 % pedal (1.5 % band).
- Entry at 3.0 % pedal, exit to DRIVE at 8.0 % (5.0 % band).

---

#### DRIVE — Normal Traction

**Purpose**: Full voltage-mode traction control. This is the current
firmware behavior above the dead zone, with all existing pipeline stages:
demand → EMA → ramp → gear scaling → ABS/TCS/Ackermann → PWM.

| Parameter | Value |
|-----------|-------|
| **PWM** | `mapped_pwm` from pedal-to-torque curve (Section 2) |
| **EN** | HIGH |
| **DIR** | Set by current gear |
| **Electrical effect** | Motor driven at commanded duty |

In DRIVE state, the pedal-to-torque mapping (Section 2) ensures a smooth
transition from the CREEP region. The deadband shaping guarantees that
the first usable PWM output is already above the motor's start voltage.

**Entry conditions**:
- From CREEP: pedal > `DRIVE_ENTER_PEDAL_PCT` (8.0 %) OR speed > 5.0 km/h
- From BRAKE: pedal > `DRIVE_ENTER_PEDAL_PCT` (8.0 %) AND speed > 0.5 km/h

**Exit conditions**:
| Target state | Condition |
|-------------|-----------|
| CREEP | Speed < 5.0 km/h AND pedal < 8.0 % |
| COAST | Pedal < 1.5 % AND speed > 0.5 km/h |
| BRAKE | Pedal = 0 AND speed > 3.0 km/h |
| HOLD_STOP | Speed < 0.5 km/h AND pedal < 1.0 % for > 200 ms |
| EMERGENCY | Safety system trigger |

**Time constants**:
- All existing pipeline ramps apply (50 %/s up, 100 %/s down).
- No additional ramps beyond the existing `PEDAL_RAMP_UP/DOWN_PCT_S`.

---

#### BRAKE — Dynamic Deceleration

**Purpose**: Active deceleration via opposing motor torque. This is the
existing dynamic braking system, triggered when the driver releases the
throttle at speed.

| Parameter | Value |
|-----------|-------|
| **PWM** | Proportional to brake demand: `dynbrake_pct × PWM_PERIOD / 100` |
| **EN** | HIGH |
| **DIR** | Reversed from travel direction (opposing torque) |
| **Electrical effect** | Motor driven against rotation → decelerating torque |

The dynamic braking logic is unchanged from the current firmware
(`DYNBRAKE_FACTOR = 0.5`, `DYNBRAKE_MAX_PCT = 60 %`).

**Entry conditions**:
- From DRIVE/CREEP/COAST: pedal = 0 AND speed > `DYNBRAKE_MIN_SPEED_KMH` (3.0 km/h) AND demand_rate < 0

**Exit conditions**:
| Target state | Condition |
|-------------|-----------|
| COAST | Speed > 0.5 km/h AND dynbrake_pct < 0.5 % |
| HOLD_STOP | Speed < 0.5 km/h for > 200 ms |
| CREEP | Pedal > 3.0 % (driver reapplies throttle during braking) |
| DRIVE | Pedal > 8.0 % |
| EMERGENCY | Safety system trigger |

**Time constants**:
- Existing `DYNBRAKE_RAMP_DOWN_PCT_S = 80 %/s` for brake release.
- Transition to HOLD_STOP uses the standard 200 ms settle timer.

---

#### EMERGENCY — Full De-energization

**Purpose**: All H-bridges completely disabled. No braking, no driving.
Coast mode for safety (fault may involve power stage itself).

| Parameter | Value |
|-----------|-------|
| **PWM** | 0 |
| **EN** | LOW (all 5 motors) |
| **DIR** | Don't care |
| **Electrical effect** | H-bridges off — motors float freely |

This state is unchanged from the current `Traction_EmergencyStop()`.

**Entry conditions**:
- From ANY state: `Safety_EmergencyStop()` triggered

**Exit conditions**:
- None within the low-speed controller. Recovery requires system-level
  fault clearance and restart sequence.

---

### 1.3 State Transition Summary Table

| From \ To | HOLD_STOP | COAST | CREEP | DRIVE | BRAKE | EMERGENCY |
|-----------|-----------|-------|-------|-------|-------|-----------|
| **HOLD_STOP** | — | pedal∈[1,3) | pedal≥3% | — | — | fault |
| **COAST** | spd<0.5 & pdl<1% | — | pedal≥3% | — | pdl=0 & spd>3 | fault |
| **CREEP** | spd<0.5 & pdl<1% | pedal<1.5% | — | pdl>8% ∨ spd>5 | pdl=0 & spd>3 | fault |
| **DRIVE** | spd<0.5 & pdl<1% | pedal<1.5% | spd<5 & pdl<8% | — | pdl=0 & spd>3 | fault |
| **BRAKE** | spd<0.5 | dynbrk<0.5% | pedal>3% | pedal>8% | — | fault |
| **EMERGENCY** | — | — | — | — | — | — |

### 1.4 Threshold Constants

| Constant | Value | Unit | Purpose |
|----------|-------|------|---------|
| `HOLD_ENTER_PEDAL_PCT` | 1.0 | % | Pedal below this → candidate for HOLD_STOP |
| `HOLD_SETTLE_MS` | 200 | ms | Debounce time before entering HOLD_STOP |
| `HOLD_EXIT_DEBOUNCE_MS` | 50 | ms | Debounce time before leaving HOLD_STOP |
| `COAST_ENTER_PEDAL_PCT` | 1.0 | % | Pedal above this → release brake, enter COAST |
| `COAST_REENTER_PEDAL_PCT` | 1.5 | % | Pedal below this from CREEP/DRIVE → COAST |
| `CREEP_ENTER_PEDAL_PCT` | 3.0 | % | Pedal above this → engage minimum drive |
| `CREEP_SPEED_MAX_KMH` | 5.0 | km/h | Speed above this → transition to DRIVE |
| `DRIVE_ENTER_PEDAL_PCT` | 8.0 | % | Pedal above this → full voltage-mode drive |
| `DYNBRAKE_MIN_SPEED_KMH` | 3.0 | km/h | Existing constant — no dynamic braking below |
| `BRAKE_RELEASE_RAMP_MS` | 80 | ms | Time to ramp PWM from brake (4249) to 0 |
| `CREEP_RAMP_UP_MS` | 150 | ms | Time to ramp PWM from 0 to `CREEP_PWM_MIN` |
| `CREEP_PWM_MIN` | 340 | counts | ~8 % duty = ~1.9 V at nominal 24 V bus (motor start threshold). Voltage-proportional: actual V = duty × V_bus_measured. See Section 6 for calibration. |
| `CREEP_PWM_MAX` | 510 | counts | ~12 % duty = ~2.9 V at nominal 24 V bus → walking speed (~5 km/h). Same voltage-proportional note applies. |
| `STALL_DETECT_MS` | 300 | ms | Time at CREEP with no speed → stall detected |
| `SPEED_ZERO_THRESHOLD_KMH` | 0.5 | km/h | Below this = "stopped" (sensor resolution limit) |
| `DYNBRAKE_ACTIVE_THRESHOLD` | 0.5 | % | Existing constant (`motor_control.c` line 78) — dynbrake_pct above this = active braking |

---

## 2. Pedal-to-Torque Mapping

### 2.1 Problem: Dead Zone

The current firmware maps pedal demand directly to PWM duty:
```
base_pwm = |effective_demand| × PWM_PERIOD / 100
         = pedal% × 4249 / 100
```

At 1 % pedal: `base_pwm = 42` → 1 % duty → 0.24 V applied to motor.
Typical 24 V brushed motors need 1.5–2.0 V to begin rotating under load.
This means pedal 0–6 % is a **dead zone** where current flows but the
motor does not move.

### 2.2 Solution: Deadband-Compensated Mapping Curve

The mapping is split into four regions:

```
Pedal %:  0    1.0   3.0        8.0            100
          │     │     │          │               │
Region:   │DEAD │COAST│  CREEP   │    DRIVE      │
          │ZONE │     │          │               │
          ▼     ▼     ▼          ▼               ▼
PWM:      0     0   CREEP_MIN  DRIVE_MIN       PWM_PERIOD
          (brake)    (340)      (340)           (4249)
```

#### Region A: Dead Zone (pedal 0–1.0 %)

- **Output**: 0 PWM (state machine handles brake/coast).
- **Rationale**: Absorbs ADC noise, prevents unintended drive engagement.

#### Region B: Coast Zone (pedal 1.0–3.0 %)

- **Output**: 0 PWM (brake released, motor in coast or brake-release-ramp).
- **Rationale**: Provides a neutral zone between brake and drive.
  The driver must deliberately press past 3 % to start moving.

#### Region C: Creep Zone (pedal 3.0–8.0 %)

- **Output**: Linear map from `CREEP_PWM_MIN` (340) to `CREEP_PWM_MAX` (510).
- **Formula**:
  ```
  mapped_pwm = CREEP_PWM_MIN + (pedal - 3.0) / (8.0 - 3.0)
               × (CREEP_PWM_MAX - CREEP_PWM_MIN)
             = 340 + (pedal - 3.0) / 5.0 × 170
  ```
- **Voltage range**: 1.9 V to 2.9 V (above motor start threshold).
- **Rationale**: Every position in the creep zone produces useful rotation.
  No dead pedal feel. The minimum is set at 8 % duty (~340 counts)
  which reliably starts the motor under worst-case load.

#### Region D: Drive Zone (pedal 8.0–100 %)

- **Output**: Linear map from `CREEP_PWM_MAX` (510) to `PWM_PERIOD` (4249).
  Ensures continuity at the creep/drive boundary.
- **Formula**:
  ```
  mapped_pwm = 510 + (pedal - 8.0) / (100.0 - 8.0) × (4249 - 510)
             = 510 + (pedal - 8.0) / 92.0 × 3739
  ```
- **Rationale**: Full voltage-mode range with a continuous join to
  the creep zone. No step discontinuity.

### 2.3 Pedal Mapping Function (C pseudocode)

```c
uint16_t pedal_to_pwm(float pedal_pct)
{
    /* Input invariant: pedal_pct is expected in [0, 100] after upstream
     * EMA filter, ramp limiter, and sanitize_float() processing.
     * Clamp defensively in case of float rounding or pipeline edge cases. */
    if (pedal_pct < 0.0f) pedal_pct = 0.0f;
    if (pedal_pct > 100.0f) pedal_pct = 100.0f;

    if (pedal_pct < COAST_ENTER_PEDAL_PCT)      /* < 1.0 % */
        return 0;
    if (pedal_pct < CREEP_ENTER_PEDAL_PCT)      /* 1.0 – 3.0 % */
        return 0;  /* COAST: brake released, no drive */
    if (pedal_pct < DRIVE_ENTER_PEDAL_PCT) {    /* 3.0 – 8.0 % */
        /* CREEP zone: linear map [3, 8] → [CREEP_PWM_MIN, CREEP_PWM_MAX] */
        float frac = (pedal_pct - CREEP_ENTER_PEDAL_PCT)
                   / (DRIVE_ENTER_PEDAL_PCT - CREEP_ENTER_PEDAL_PCT);
        return (uint16_t)(CREEP_PWM_MIN + frac * (CREEP_PWM_MAX - CREEP_PWM_MIN));
    }
    /* DRIVE zone: linear map [8, 100] → [CREEP_PWM_MAX, PWM_PERIOD] */
    float frac = (pedal_pct - DRIVE_ENTER_PEDAL_PCT)
               / (100.0f - DRIVE_ENTER_PEDAL_PCT);
    uint16_t pwm = (uint16_t)(CREEP_PWM_MAX + frac * (PWM_PERIOD - CREEP_PWM_MAX));
    if (pwm > PWM_PERIOD) pwm = PWM_PERIOD;
    return pwm;
}
```

### 2.4 Slope Handling

On an incline, the motor needs more than `CREEP_PWM_MIN` to begin moving.
The creep controller detects this via the stick-slip prevention system
(Section 4): if the motor is commanded at `CREEP_PWM_MIN` but speed
remains zero for > `STALL_DETECT_MS` (300 ms), the creep PWM is
progressively increased up to `CREEP_PWM_MAX`:

```
if (state == CREEP && speed < SPEED_ZERO_THRESHOLD_KMH && stall_timer > STALL_DETECT_MS):
    creep_boost = min(creep_boost + CREEP_BOOST_STEP, CREEP_BOOST_MAX)
    actual_pwm = mapped_pwm + creep_boost
    actual_pwm = min(actual_pwm, CREEP_PWM_MAX)
else if (speed > 1.0 km/h):
    creep_boost = 0  /* Reset boost when moving */
```

| Constant | Value | Purpose |
|----------|-------|---------|
| `STALL_DETECT_MS` | 300 | Time at creep with no speed → stall |
| `CREEP_BOOST_STEP` | 17 | ~0.4 % duty increment per 10 ms cycle |
| `CREEP_BOOST_MAX` | 170 | Maximum additional duty (~4 %, giving up to 12 %) |

This auto-boost adapts the creep torque to the incline without requiring
an accelerometer or IMU. If the boost reaches `CREEP_PWM_MAX` and the
vehicle still doesn't move, the motor is likely stalled on a steep slope
— the system remains in CREEP (not HOLD_STOP) so the driver can apply
more pedal to enter DRIVE.

### 2.5 Creep Torque Specification

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Creep speed (flat) | 2–5 km/h | Walking pace, controllable |
| Creep duty (flat) | 8–12 % | 1.9–2.9 V, above motor start threshold |
| Creep duty (max incline) | 12 % | With stall boost, enough for moderate slope |
| Creep current limit | < 15 A per motor | Below INA226 overcurrent threshold (20 A) |
| Creep thermal budget | Indefinite at < 15 A | DS18B20 < 70 °C operating range |

---

## 3. Brake-to-Drive and Drive-to-Brake Transition Shaping

### 3.1 Brake Release Ramp (HOLD_STOP → COAST)

**Problem solved**: The current firmware transitions from PWM=4249 (full
brake) to PWM=0 or PWM=25 (tiny drive) in a single 10 ms cycle. This
produces a mechanical "clunk" as the electromagnetic brake releases
instantaneously.

**Solution**: When the state machine transitions HOLD_STOP → COAST,
the brake PWM is ramped down over `BRAKE_RELEASE_RAMP_MS`:

```
Brake Release Profile (80 ms, 8 control cycles):

Time (ms):   0    10    20    30    40    50    60    70    80
PWM:       4249  3718  3187  2656  2125  1594  1063   531    0
EN:        HIGH  HIGH  HIGH  HIGH  HIGH  HIGH  HIGH  HIGH  LOW

Ramp rate: (4249 - 0) / 8 cycles = 531 counts/cycle
         = 12.5 % duty per 10 ms
         = 1250 % duty/s
```

During the ramp, the motor experiences gradually decreasing brake torque.
The vehicle begins rolling gently as the brake weakens, rather than
lurching when the brake snaps off.

**Critical detail**: EN transitions to LOW only when PWM reaches 0.
If EN went LOW while PWM is mid-ramp, the BTS7960 would float the motor
(loss of braking) instead of providing proportional brake.

### 3.2 Coast-to-Creep Ramp (COAST → CREEP)

**Problem solved**: Without a ramp, the motor jumps from 0 PWM to
`CREEP_PWM_MIN` (340) in one cycle — a 8 % duty step = 1.9 V sudden
application. On a stationary vehicle, this produces a lurch.

**Solution**: When entering CREEP from COAST, the target PWM ramps
up over `CREEP_RAMP_UP_MS`:

```
Creep Engage Profile (150 ms, 15 control cycles):

Time (ms):   0    10    20   ...   140   150
PWM:         0    23    45   ...   317   340 (CREEP_PWM_MIN)
EN:         LOW  HIGH  HIGH  ...  HIGH  HIGH
DIR:         ←── Set to gear direction at t=0 ──►

Ramp rate: 340 / 15 = 22.7 counts/cycle
         = 0.53 % duty per 10 ms
         = 53 % duty/s
```

EN switches to HIGH at the first non-zero PWM cycle (t=10 ms), and DIR
is set before the first pulse.

The slow ramp gives the motor time to overcome static friction
progressively. The vehicle creeps forward smoothly rather than jerking.

### 3.3 Creep-to-Drive Transition

**No explicit ramp needed**: The pedal-to-torque mapping (Section 2) is
continuous at the CREEP/DRIVE boundary (both map to 510 counts at 8 %).
The existing `PEDAL_RAMP_UP_PCT_S = 50 %/s` ramp limiter ensures the
demand moves through the boundary smoothly. The state transition is
instantaneous, but the PWM output changes continuously.

### 3.4 Drive-to-Brake Transition (DRIVE/CREEP → BRAKE)

The existing dynamic braking system handles this transition with its own
ramp (`DYNBRAKE_RAMP_DOWN_PCT_S = 80 %/s` for brake release). No changes
needed — the dynamic braking already provides smooth deceleration.

### 3.5 Any-State-to-HOLD_STOP Transition

When the vehicle comes to a stop (speed < 0.5 km/h, pedal < 1.0 %),
the transition to HOLD_STOP applies the brake ramp in reverse:

```
Hold Engage Profile (50 ms, 5 control cycles):

Time (ms):   0    10    20    30    40    50
PWM:         0   850  1700  2550  3400  4249 (BTS7960_BRAKE_PWM)
EN:         HIGH HIGH  HIGH  HIGH  HIGH  HIGH

Ramp rate: 4249 / 5 = 850 counts/cycle
         = 20 % duty per 10 ms
         = 2000 % duty/s
```

The hold engage ramp is faster than the brake release ramp (50 ms vs.
80 ms) because the vehicle is already stationary — there's no motion
to absorb. The faster ramp prevents the vehicle from rolling backward
on a slope during the transition.

### 3.6 Transition Timing Summary

| Transition | Ramp Direction | Duration | Rate | Safety Impact |
|------------|---------------|----------|------|---------------|
| HOLD_STOP → COAST (brake release) | PWM 4249→0 | 80 ms | 531 cts/cycle | Low — vehicle already stationary |
| COAST → CREEP (drive engage) | PWM 0→340 | 150 ms | 23 cts/cycle | Low — gradual torque application |
| CREEP → DRIVE | Continuous (mapping) | N/A | Existing ramp | None — smooth curve |
| DRIVE → BRAKE | Existing dynbrake | Existing | 80 %/s | None — unchanged |
| Any → HOLD_STOP | PWM 0→4249 | 50 ms | 850 cts/cycle | Low — vehicle stopping |
| Any → EMERGENCY | Instant cut | 0 ms | Immediate | None — EN=LOW, PWM=0 |

---

## 4. Stick-Slip Prevention at Very Low Speed

### 4.1 Root Cause

Stick-slip occurs in the CREEP and low-DRIVE region when:
1. Applied voltage is near the motor's start threshold.
2. Motor overcomes static friction → begins rotating.
3. Back-EMF develops → net current drops → torque drops below kinetic friction.
4. Motor stops → back-EMF disappears → current rises → cycle repeats.

The oscillation frequency depends on motor inertia, winding time constant,
and drivetrain compliance. For this vehicle (heavy, high-inertia wheels,
compliant tires), the stick-slip frequency is typically 2–10 Hz.

### 4.2 Detection Using Existing Sensors

The firmware has three sensor channels available at the 100 Hz control rate:

| Sensor | Sample Rate | Resolution at Low Speed | Latency |
|--------|------------|------------------------|---------|
| Wheel speed (EXTI) | Continuous (interrupt) | 6 pulses/rev → min ~0.5 km/h | < 1 ms (hardware interrupt) |
| Bus voltage (INA226 ch4) | 20 Hz | 12-bit, ~1 mV | ~6 ms (I2C) |
| Motor current (INA226 ch0-3) | 20 Hz | 12-bit, ~1 mA | ~6 ms (I2C) |

**Primary detection**: Wheel speed.
At very low speed, the wheel speed sensor produces intermittent pulses
(6 pulses per revolution, wheel circumference 1.1 m). At 2 km/h:
```
wheel_speed = 2 km/h = 0.556 m/s
rev/s = 0.556 / 1.1 = 0.505 rev/s
pulses/s = 0.505 × 6 = 3.03 pulses/s
pulse_period = 330 ms
```

At 0.5 km/h (detection threshold):
```
pulse_period = 1320 ms → ~0.76 Hz pulse rate
```

This means at very low speed, the firmware may see **no speed pulses for
over 1 second** during normal creep. The speed sensor cannot detect
stick-slip oscillations at 2–10 Hz — it's too slow.

**Secondary detection: Current estimation.**

Since the INA226 current sensors are also too slow (20 Hz) for direct
stick-slip detection, the firmware can use a **model-based current
estimate** computed at 100 Hz (Section 4.3).

### 4.3 Model-Based Stick-Slip Prevention

Instead of detecting stick-slip after it occurs, the strategy is to
**prevent it from starting** by ensuring the motor always receives
enough voltage to maintain continuous rotation once it begins moving.

**Algorithm**: "Start-and-Hold" with speed confirmation.

```
State: CREEP, target_pwm from pedal mapping

1. STARTING phase (speed < SPEED_ZERO_THRESHOLD_KMH = 0.5 km/h):
   - Apply target_pwm (which is ≥ CREEP_PWM_MIN = 340 = 8% duty)
   - Start stall_timer
   - If stall_timer > STALL_DETECT_MS (300 ms) and still no speed:
     → Increment creep_boost (slope compensation, Section 2.4)
   - Monitor INA226 current at 20 Hz as safety check:
     If current > 15 A → motor is stalled against obstacle → hold PWM,
     do not increase further

2. RUNNING phase (speed ≥ SPEED_ZERO_THRESHOLD_KMH):
   - Motor is rotating — back-EMF is developing
   - Reduce PWM slightly to find equilibrium:
     adjusted_pwm = target_pwm × (1.0 - BACK_EMF_COMPENSATION)
     where BACK_EMF_COMPENSATION = speed_kmh / CREEP_SPEED_MAX_KMH × 0.15
   - This compensates for back-EMF: as speed rises, the motor needs
     less voltage to maintain the same current/torque
   - Reset creep_boost and stall_timer

3. SPEED REGULATION (speed > CREEP_SPEED_MAX_KMH = 5.0 km/h):
   - Vehicle is above creep target speed
   - Reduce PWM: adjusted_pwm = target_pwm × (CREEP_SPEED_MAX_KMH / speed_kmh)
   - This provides a soft speed limiter that prevents the creep from
     accelerating indefinitely on a downhill slope
   - If pedal increases above DRIVE_ENTER_PEDAL_PCT → transition to DRIVE
```

**Why this prevents stick-slip**:

The key insight is that `CREEP_PWM_MIN = 340` (8 %) is set **above** the
motor's start voltage (typically 6–7 % for these motors at 24 V). By
starting at this level:

1. The motor reliably begins rotating on the first attempt.
2. Once rotating, back-EMF develops and the motor enters a stable
   operating point where torque ≈ kinetic friction.
3. The back-EMF compensation (step 2) gently reduces duty as speed rises,
   preventing the motor from accelerating past the target creep speed.
4. If an external disturbance (bump, slope) reduces speed, the compensation
   reduces less, allowing more current to flow and more torque to develop.
   This provides inherent disturbance rejection.

The motor never enters the dead zone (0–6 %) during CREEP — the mapping
jumps over it entirely (Section 2.2).

### 4.4 Current Estimation for Stall Detection

A software current estimate at 100 Hz supplements the 20 Hz INA226 reading:

```c
float estimate_motor_current(float duty_frac, float speed_kmh, float v_bus)
{
    /* Motor parameters — placeholder values, must be measured during
     * commissioning (see Section 6: Calibration Requirements).
     * These should be stored as configurable constants, not magic numbers. */
    const float R_winding = 0.35f;   /* Ω — measured at ambient 20°C */
    const float Ke = 0.033f;         /* V·s/rad — back-EMF constant */

    /* Convert speed to angular velocity */
    float wheel_radius = WHEEL_CIRCUMF_M / (2.0f * M_PI);
    float omega = speed_kmh / 3.6f / wheel_radius;  /* rad/s */

    /* V_applied - V_back_emf */
    float v_net = (duty_frac * v_bus) - (Ke * omega);
    if (v_net < 0.0f) v_net = 0.0f;  /* Regenerating — not modeled here */

    return v_net / R_winding;
}
```

This estimate is used for:
1. **Stall protection in CREEP**: If `I_estimated > 15 A` at low speed
   for > 500 ms, the motor is stalled against an obstacle or mechanical
   limit. The creep PWM is capped (not increased further).
2. **Thermal protection supplement**: Between 20 Hz INA226 reads, the
   estimate provides a 100 Hz current approximation for the per-motor
   thermal model.

**Accuracy**: The estimate is approximate (±20–30 %) because R_winding
varies with temperature (+0.39 %/°C copper) and Ke varies with magnet
temperature. The DS18B20 sensor can correct R_winding at 1 Hz:
```
R_compensated = R_winding_20C × (1.0 + 0.00393 × (T_motor - 20.0))
```

The estimate is sufficient for stall detection (only needs to detect
"current is unusually high") but not accurate enough for closed-loop
torque control.

---

## 5. Pseudocode: 100 Hz Low-Speed Control Loop

### 5.1 Integration with Existing Firmware

The low-speed controller replaces the zero-demand brake logic and the
direct `pedal% → base_pwm` mapping in `Traction_Update()`. All other
pipeline stages remain unchanged:

```
Existing pipeline (preserved):
  Traction_SetDemand() → EMA → ramp limiter → demandPct
  ↓
  Traction_Update()
    ├── [NEW] Low-speed state machine + pedal-to-torque mapping
    ├── gear power scaling          (unchanged)
    ├── obstacle_scale              (unchanged)
    ├── traction_cap (DEGRADED)     (unchanged)
    ├── ackermann_diff[i]           (unchanged)
    ├── wheel_scale[i] (ABS/TCS)   (unchanged)
    └── Motor_SetPWM()             (unchanged)
```

### 5.2 State Variables

```c
/* Low-speed control state */
typedef enum {
    LSCS_HOLD_STOP = 0,
    LSCS_COAST,
    LSCS_CREEP,
    LSCS_DRIVE,
    LSCS_BRAKE,
    LSCS_EMERGENCY
} LowSpeedState_t;

static LowSpeedState_t  ls_state        = LSCS_HOLD_STOP;
static uint32_t          ls_state_tick   = 0;     /* Tick when state was entered */
static float             ls_ramp_pwm     = 0.0f;  /* Current ramp output (float for smooth ramp) */
static float             ls_creep_boost  = 0.0f;  /* Slope compensation boost */
static uint32_t          ls_stall_tick   = 0;      /* Start of stall detection timer */
static float             ls_v_bus        = 24.0f;  /* Cached bus voltage (updated at 20 Hz) */
```

### 5.3 Main Control Loop (100 Hz, called from `Traction_Update`)

```c
void LowSpeed_Update(float demand_pct, float avg_speed_kmh, float dt)
{
    uint32_t now = HAL_GetTick();
    float pedal = demand_pct;  /* After EMA + ramp from Traction_SetDemand */

    /* ---- Emergency check (highest priority) ---- */
    SystemState_t sys = Safety_GetState();
    if (sys == SYS_STATE_SAFE || sys == SYS_STATE_ERROR) {
        ls_state = LSCS_EMERGENCY;
        /* Traction_EmergencyStop() handles the actual motor shutdown */
        return;
    }

    /* ---- Update cached bus voltage (from 20 Hz INA226 read) ----
     * sanitize_float() is the existing NaN/Inf guard defined in
     * motor_control.c (line 27); returns safe_default on invalid input. */
    ls_v_bus = sanitize_float(Voltage_GetBus(INA226_CHANNEL_BATTERY), 24.0f);
    if (ls_v_bus < 12.0f) ls_v_bus = 24.0f;  /* Implausible — use nominal */

    /* ---- State machine ---- */
    switch (ls_state) {

    case LSCS_HOLD_STOP:
        /* Apply active brake (BTS7960: PWM=100%, EN=HIGH) */
        ls_ramp_pwm = BTS7960_BRAKE_PWM;
        set_all_motors_brake();  /* PWM=4249, EN=HIGH for driven wheels */

        /* Check exit conditions */
        if (pedal >= CREEP_ENTER_PEDAL_PCT) {
            /* Direct to CREEP (skip COAST if pedal is firm) */
            if ((now - ls_state_tick) > HOLD_EXIT_DEBOUNCE_MS) {
                ls_state = LSCS_COAST;  /* Always go through COAST for ramp */
                ls_state_tick = now;
                ls_ramp_pwm = BTS7960_BRAKE_PWM;  /* Start ramp from brake */
            }
        } else if (pedal >= COAST_ENTER_PEDAL_PCT) {
            if ((now - ls_state_tick) > HOLD_EXIT_DEBOUNCE_MS) {
                ls_state = LSCS_COAST;
                ls_state_tick = now;
                ls_ramp_pwm = BTS7960_BRAKE_PWM;  /* Start ramp from brake */
            }
        }
        break;

    case LSCS_COAST:
        /* Ramp brake PWM down toward zero */
        {
            float ramp_rate = (float)BTS7960_BRAKE_PWM / (BRAKE_RELEASE_RAMP_MS / 10.0f);
            /* rate = 4249 / 8 = 531 counts per 10 ms cycle */

            if (ls_ramp_pwm > 0.0f) {
                ls_ramp_pwm -= ramp_rate;
                if (ls_ramp_pwm < 0.0f) ls_ramp_pwm = 0.0f;
                set_all_motors_pwm((uint16_t)ls_ramp_pwm, true);  /* EN=HIGH during ramp */
            } else {
                set_all_motors_coast();  /* PWM=0, EN=LOW */
            }
        }

        /* Check exit conditions */
        if (pedal >= CREEP_ENTER_PEDAL_PCT && ls_ramp_pwm <= 0.0f) {
            /* Brake fully released — can enter CREEP */
            ls_state = LSCS_CREEP;
            ls_state_tick = now;
            ls_ramp_pwm = 0.0f;    /* Start creep ramp from zero */
            ls_creep_boost = 0.0f;
            ls_stall_tick = now;
        } else if (pedal < COAST_ENTER_PEDAL_PCT && avg_speed_kmh < SPEED_ZERO_THRESHOLD_KMH) {
            if ((now - ls_state_tick) > HOLD_SETTLE_MS) {
                ls_state = LSCS_HOLD_STOP;
                ls_state_tick = now;
            }
        } else if (pedal < 0.1f && avg_speed_kmh > DYNBRAKE_MIN_SPEED_KMH) {
            ls_state = LSCS_BRAKE;
            ls_state_tick = now;
        }
        break;

    case LSCS_CREEP:
        {
            /* Compute target PWM from pedal mapping */
            uint16_t target_pwm = pedal_to_pwm(pedal);

            /* Apply stall boost (slope compensation) */
            if (avg_speed_kmh < SPEED_ZERO_THRESHOLD_KMH) {
                if ((now - ls_stall_tick) > STALL_DETECT_MS) {
                    /* Motor stalled — increase duty */
                    ls_creep_boost += CREEP_BOOST_STEP;
                    if (ls_creep_boost > CREEP_BOOST_MAX)
                        ls_creep_boost = CREEP_BOOST_MAX;
                    ls_stall_tick = now;  /* Reset timer for next boost step */
                }

                /* Safety: check estimated current.
                 * Uses PARK_HOLD_CURRENT_WARN_A (15 A) as the creep stall
                 * current limit — same physical threshold (motor thermal
                 * budget) applies whether the motor is holding in park or
                 * stalled during creep.  A dedicated CREEP_CURRENT_LIMIT_A
                 * constant could be defined if different limits are needed. */
                float i_est = estimate_motor_current(
                    (float)(target_pwm + ls_creep_boost) / PWM_PERIOD,
                    avg_speed_kmh, ls_v_bus);
                if (i_est > PARK_HOLD_CURRENT_WARN_A) {
                    /* Near current limit — do not boost further */
                    ls_creep_boost -= CREEP_BOOST_STEP;
                    if (ls_creep_boost < 0.0f) ls_creep_boost = 0.0f;
                }
            } else {
                /* Vehicle is moving — reset stall state */
                ls_creep_boost = 0.0f;
                ls_stall_tick = now;
            }

            uint16_t final_pwm = target_pwm + (uint16_t)ls_creep_boost;
            if (final_pwm > CREEP_PWM_MAX) final_pwm = CREEP_PWM_MAX;

            /* Ramp up from current ramp position to target */
            float creep_ramp_rate = (float)CREEP_PWM_MIN / (CREEP_RAMP_UP_MS / 10.0f);
            /* rate = 340 / 15 = 22.7 counts per 10 ms cycle */

            if (ls_ramp_pwm < (float)final_pwm) {
                ls_ramp_pwm += creep_ramp_rate;
                if (ls_ramp_pwm > (float)final_pwm)
                    ls_ramp_pwm = (float)final_pwm;
            } else {
                ls_ramp_pwm = (float)final_pwm;
            }

            /* Back-EMF compensation: reduce duty slightly as speed increases */
            float bemf_comp = 1.0f;
            if (avg_speed_kmh > 0.5f && avg_speed_kmh < CREEP_SPEED_MAX_KMH) {
                bemf_comp = 1.0f - (avg_speed_kmh / CREEP_SPEED_MAX_KMH * 0.15f);
            } else if (avg_speed_kmh >= CREEP_SPEED_MAX_KMH) {
                /* Soft speed limit: reduce duty proportionally */
                bemf_comp = CREEP_SPEED_MAX_KMH / avg_speed_kmh;
                if (bemf_comp < 0.5f) bemf_comp = 0.5f;
            }

            uint16_t output_pwm = (uint16_t)(ls_ramp_pwm * bemf_comp);
            set_all_motors_drive(output_pwm);  /* PWM=output, EN=HIGH, DIR=gear */
        }

        /* Check exit conditions */
        if (pedal >= DRIVE_ENTER_PEDAL_PCT || avg_speed_kmh > CREEP_SPEED_MAX_KMH) {
            ls_state = LSCS_DRIVE;
            ls_state_tick = now;
        } else if (pedal < COAST_REENTER_PEDAL_PCT) {
            ls_state = LSCS_COAST;
            ls_state_tick = now;
            /* ls_ramp_pwm retains current value — ramp down from here */
        } else if (pedal < HOLD_ENTER_PEDAL_PCT && avg_speed_kmh < SPEED_ZERO_THRESHOLD_KMH) {
            if ((now - ls_state_tick) > HOLD_SETTLE_MS) {
                ls_state = LSCS_HOLD_STOP;
                ls_state_tick = now;
            }
        }
        break;

    case LSCS_DRIVE:
        {
            /* Standard voltage-mode mapping with deadband compensation */
            uint16_t target_pwm = pedal_to_pwm(pedal);

            /* Apply existing pipeline: gear scale, obstacle, traction cap,
             * ackermann, wheel_scale — these are handled by the caller
             * (Traction_Update) using the returned base_pwm.
             *
             * The low-speed controller outputs base_pwm; the rest of the
             * pipeline applies multipliers on top of it.                    */
            ls_ramp_pwm = (float)target_pwm;  /* Track for potential ramp-down */
        }

        /* Check exit conditions */
        if (avg_speed_kmh < CREEP_SPEED_MAX_KMH && pedal < DRIVE_ENTER_PEDAL_PCT) {
            ls_state = LSCS_CREEP;
            ls_state_tick = now;
            ls_creep_boost = 0.0f;
            ls_stall_tick = now;
        } else if (pedal < COAST_REENTER_PEDAL_PCT && avg_speed_kmh > SPEED_ZERO_THRESHOLD_KMH) {
            ls_state = LSCS_COAST;
            ls_state_tick = now;
        } else if (pedal < 0.1f && avg_speed_kmh > DYNBRAKE_MIN_SPEED_KMH) {
            ls_state = LSCS_BRAKE;
            ls_state_tick = now;
        } else if (pedal < HOLD_ENTER_PEDAL_PCT && avg_speed_kmh < SPEED_ZERO_THRESHOLD_KMH) {
            if ((now - ls_state_tick) > HOLD_SETTLE_MS) {
                ls_state = LSCS_HOLD_STOP;
                ls_state_tick = now;
            }
        }
        break;

    case LSCS_BRAKE:
        /* Dynamic braking — handled by existing dynbrake logic in Traction_Update.
         * The low-speed controller only manages the state transitions.
         *
         * The existing dynbrake code sets effective_demand = -dynbrake_pct.
         * We pass through to the existing pipeline.                         */

        /* Check exit conditions */
        if (avg_speed_kmh < SPEED_ZERO_THRESHOLD_KMH) {
            if ((now - ls_state_tick) > HOLD_SETTLE_MS) {
                ls_state = LSCS_HOLD_STOP;
                ls_state_tick = now;
            }
        } else if (pedal >= CREEP_ENTER_PEDAL_PCT) {
            ls_state = LSCS_CREEP;
            ls_state_tick = now;
            ls_ramp_pwm = 0.0f;
            ls_creep_boost = 0.0f;
            ls_stall_tick = now;
        } else if (pedal >= DRIVE_ENTER_PEDAL_PCT) {
            ls_state = LSCS_DRIVE;
            ls_state_tick = now;
        } else if (dynbrake_pct < DYNBRAKE_ACTIVE_THRESHOLD && pedal < COAST_ENTER_PEDAL_PCT) {
            ls_state = LSCS_COAST;
            ls_state_tick = now;
        }
        break;

    case LSCS_EMERGENCY:
        /* No action — Traction_EmergencyStop() handles shutdown.
         * Stay in EMERGENCY until system recovery (handled externally). */
        ls_ramp_pwm = 0.0f;
        break;
    }
}
```

### 5.4 Helper Functions

```c
static void set_all_motors_brake(void)
{
    /* Apply BTS7960 active brake to all driven motors.
     * PWM=100% (both FETs on) shorts motor terminals. */
    bool rear_active = traction_state.mode4x4 || traction_state.axisRotation;
    Motor_SetPWM(&motor_fl, BTS7960_BRAKE_PWM);  Motor_Enable(&motor_fl, 1);
    Motor_SetPWM(&motor_fr, BTS7960_BRAKE_PWM);  Motor_Enable(&motor_fr, 1);
    if (rear_active) {
        Motor_SetPWM(&motor_rl, BTS7960_BRAKE_PWM);  Motor_Enable(&motor_rl, 1);
        Motor_SetPWM(&motor_rr, BTS7960_BRAKE_PWM);  Motor_Enable(&motor_rr, 1);
    }
}

static void set_all_motors_coast(void)
{
    /* Disable H-bridges — motors float freely. */
    bool rear_active = traction_state.mode4x4 || traction_state.axisRotation;
    Motor_SetPWM(&motor_fl, 0);  Motor_Enable(&motor_fl, 0);
    Motor_SetPWM(&motor_fr, 0);  Motor_Enable(&motor_fr, 0);
    if (rear_active) {
        Motor_SetPWM(&motor_rl, 0);  Motor_Enable(&motor_rl, 0);
        Motor_SetPWM(&motor_rr, 0);  Motor_Enable(&motor_rr, 0);
    }
}

static void set_all_motors_pwm(uint16_t pwm, bool en)
{
    /* Set uniform PWM and enable state to all driven motors. */
    bool rear_active = traction_state.mode4x4 || traction_state.axisRotation;
    Motor_SetPWM(&motor_fl, pwm);  Motor_Enable(&motor_fl, en ? 1 : 0);
    Motor_SetPWM(&motor_fr, pwm);  Motor_Enable(&motor_fr, en ? 1 : 0);
    if (rear_active) {
        Motor_SetPWM(&motor_rl, pwm);  Motor_Enable(&motor_rl, en ? 1 : 0);
        Motor_SetPWM(&motor_rr, pwm);  Motor_Enable(&motor_rr, en ? 1 : 0);
    }
}

static void set_all_motors_drive(uint16_t pwm)
{
    /* Set drive PWM with direction from current gear.
     * EN=HIGH, DIR set by gear.
     * current_gear is the existing module-static variable in motor_control.c
     * (line 242), set by Traction_SetGear() from the CAN gear command. */
    int8_t dir = (current_gear == GEAR_REVERSE) ? -1 : 1;
    bool rear_active = traction_state.mode4x4 || traction_state.axisRotation;
    Motor_SetPWM(&motor_fl, pwm);  Motor_SetDirection(&motor_fl, dir);  Motor_Enable(&motor_fl, 1);
    Motor_SetPWM(&motor_fr, pwm);  Motor_SetDirection(&motor_fr, dir);  Motor_Enable(&motor_fr, 1);
    if (rear_active) {
        Motor_SetPWM(&motor_rl, pwm);  Motor_SetDirection(&motor_rl, dir);  Motor_Enable(&motor_rl, 1);
        Motor_SetPWM(&motor_rr, pwm);  Motor_SetDirection(&motor_rr, dir);  Motor_Enable(&motor_rr, 1);
    }
}
```

### 5.5 Integration Point in Traction_Update()

The low-speed controller replaces the `base_pwm` computation and the
zero-demand brake logic. The integration point is:

```c
void Traction_Update(void)
{
    /* ... gear P/N handling unchanged ... */

    float demand = traction_state.demandPct;
    float avg_speed = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                       Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;

    /* ---- [NEW] Low-speed state machine ---- */
    LowSpeed_Update(demand, avg_speed, 0.01f);

    /* In HOLD_STOP, COAST, and CREEP states, the low-speed controller
     * has already written PWM/EN/DIR to the motors directly.
     * Skip the existing base_pwm → pipeline → Motor_SetPWM chain.      */
    if (ls_state == LSCS_HOLD_STOP || ls_state == LSCS_COAST ||
        ls_state == LSCS_CREEP     || ls_state == LSCS_EMERGENCY) {
        /* Update state telemetry only */
        update_traction_state_telemetry();
        return;
    }

    /* ---- DRIVE and BRAKE: use existing pipeline ---- */
    /* ... dynamic braking logic (unchanged) ... */
    /* ... gear scaling (unchanged) ... */

    /* [MODIFIED] Replace direct demand→PWM conversion with deadband-compensated mapping */
    uint16_t base_pwm = pedal_to_pwm(fabsf(effective_demand));

    /* ... obstacle_scale, traction_cap, ackermann, wheel_scale ... (unchanged) */
    /* ... Motor_SetPWM per wheel ... (unchanged) */

    /* [REMOVED] The old zero-demand brake block (if fabs(effective_demand) <= 0.5%)
     * is no longer needed — the state machine handles all brake/coast/creep transitions. */
}
```

---

## 6. Calibration Requirements

The following parameters require empirical measurement on the actual vehicle
hardware during commissioning:

| Parameter | How to Measure | Expected Range |
|-----------|---------------|----------------|
| `CREEP_PWM_MIN` | Ramp PWM from 0 until motor reliably starts with full vehicle weight on flat ground. Repeat 10× with warm and cold motors. Use the highest value. | 250–400 counts (6–10 %) |
| `CREEP_PWM_MAX` | Record PWM at which vehicle speed reaches 5 km/h on flat ground (no pedal, creep only). | 425–600 counts (10–14 %) |
| `R_winding` | Measure DC resistance between motor terminals at 20°C. Average of all 4 motors. | 0.2–0.5 Ω |
| `Ke` | Spin motor at known RPM (e.g. jack up wheel, drive at known duty), measure back-EMF with oscilloscope or derive from no-load speed/voltage relationship. | 0.02–0.05 V·s/rad |
| `STALL_DETECT_MS` | Observe time from PWM application to first wheel speed pulse on flat ground. Add 100 ms margin. | 200–500 ms |

---

## 7. Safety Analysis

### 7.1 Failure Modes

| Failure | State Machine Response | Existing Protection |
|---------|----------------------|---------------------|
| Pedal stuck at CREEP level | Vehicle creeps at walking speed — bounded by `CREEP_PWM_MAX` | Frozen pedal detection (`FROZEN_PEDAL_TIMEOUT_MS = 5 s`) → DEGRADED |
| Pedal stuck at DRIVE level | Normal DRIVE behavior — existing ramp limits apply | Same frozen pedal detection |
| Wheel speed sensor fails | Creep boost may over-boost (thinks vehicle is stalled) | `CREEP_PWM_MAX` caps output at 12 %; overcurrent protection at 20 A |
| INA226 fails | Current estimation becomes sole source — less accurate but functional | Temperature protection via DS18B20 as backup |
| State machine stuck in COAST | Vehicle coasts freely (no brake, no drive) | Driver can press pedal to enter CREEP/DRIVE, or come to rest → HOLD_STOP |
| BTS7960 driver fault | Existing emergency stop, relay shutdown | `Error_Handler()` direct register writes |

### 7.2 Worst-Case Creep Torque

At `CREEP_PWM_MAX = 510` (12 % duty), with 24 V bus:
```
V_applied = 0.12 × 24 = 2.88 V
I_stall = 2.88 / 0.35 = 8.2 A   (per motor)
P_dissipated = 8.2² × 0.35 = 23.5 W   (per motor, worst case stall)
```

At 8.2 A per motor × 4 motors = 32.8 A total — well below the battery
100 A sensor range and the 20 A per-motor overcurrent threshold. The
DS18B20 thermal protection will trigger at 90°C if a motor is stalled
for an extended period.

### 7.3 Emergency Override

In ALL states except EMERGENCY, the `Safety_EmergencyStop()` function
can be triggered by any safety system fault. This immediately:
1. Sets `ls_state = LSCS_EMERGENCY`
2. Executes the existing `Traction_EmergencyStop()` (EN=LOW, PWM=0 for all motors)
3. Relay shutdown sequence (if fault is severe)

The state machine does not interfere with or delay the emergency stop path.
