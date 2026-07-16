# EPS local isolation policy — mechanical-only fallback

The vehicle steering is **mechanical**. The steering motor only provides
**assistance** (EPS). Therefore any *isolable* fault of the assist path must
disconnect the motor and leave the driver with pure mechanical steering —
**without** degrading the global vehicle state, reducing traction, changing
gear, opening traction relays, or blocking the pedal.

This document describes the local EPS authority introduced in
`Core/Src/steering_eps.{c,h}` and how it is wired into the rest of the firmware.

## 0. Physical topology of the steering branch (owner-confirmed)

```
12 V BATTERY
    |
    +---- INA226 CH5 + SHUNT            (measurement point, BEFORE the relay)
              |
              +---- STEERING-MOTOR RELAY, PC12   (isolation contact command)
                        |
                        +---- BTS7960 / STEERING MOTOR
```

Key consequences of this wiring, which the firmware and HMI **must** respect:

- **INA226 CH5 + shunt sit BEFORE the relay.** CH5 therefore measures the 12 V
  supply available to the steering branch *before* the relay, and the current
  drawn from the battery into that branch. It is **pre-relay** voltage — never
  call it "voltage after the relay".
- **PC12 commands the steering-motor isolation relay** in the final leg that
  connects the motor. De-energising PC12 (`OFF`) opens that contact and
  electrically isolates the BTS7960/motor. It does **not** remove the INA226/
  shunt supply (those are upstream of the relay).
- **CH5 cannot confirm the relay contact.** There is no post-relay voltage or
  contact-position feedback. CH5 alone can never prove `RELAY CLOSED`,
  `RELAY OPEN CONFIRMED`, "motor energised" or "motor isolated". PC12 is a
  *command*; the true contact state stays **UNKNOWN**.
- With the relay **OFF** it is entirely **normal** to still read ~12 V on CH5
  (pre-relay bus present) with zero or calibrated-residual current. This is the
  expected isolation condition and must **not** be interpreted as a welded
  relay, closed contact, powered motor or isolation failure.
- `RELAY COMMANDED ON + current near zero` does **not** by itself prove
  `RELAY OPEN` (it can equally be PWM zero, unloaded wheel, stalled motor,
  sub-resolution current, driver coast, INA offset or a stale sample). Without
  independent post-relay feedback the firmware distinguishes only
  `STEER_RELAY_CMD_ON` / `STEER_RELAY_CMD_OFF` / `STEER_RELAY_ACTUAL_UNKNOWN`,
  and at most `RELAY_OPEN_SUSPECTED` — never a confirmed verdict, and never a
  traction block or confirmed DTC from CH5 alone.

### Naming note (compatible alias)

The legacy identifiers `PIN_RELAY_STEER_PWR` / `Steering_SteerPowerOff()` are
**retained** for source/CAN stability, but they denote the **steering-motor
isolation relay command** described above (semantically `STEER_MOTOR_RELAY` /
`Steering_MotorRelayOff()`), **not** the INA226 supply. `PIN_RELAY_STEER_PWR`
is aliased to `PIN_RELAY_STEER_MOTOR` in `project_config.h`; both resolve to
PC12.

## 1. State model

`EpsState_t` (single source of truth, `steering_eps.c`):

| State                      | Meaning                                                        |
|----------------------------|----------------------------------------------------------------|
| `EPS_STATE_STARTING`       | Power-up self checks in progress.                              |
| `EPS_STATE_CALIBRATING`    | Controlled centering / calibration sweep in progress.         |
| `EPS_STATE_ACTIVE`         | Motor assisting correctly (same direction as the driver).     |
| `EPS_STATE_MECHANICAL_ONLY`| Motor fully disconnected — **latched**. Steering is mechanical.|
| `EPS_STATE_ELECTRICAL_HAZARD` | Real, non-isolable electrical danger — may justify SAFE/ERROR.|

`MECHANICAL_ONLY` and `ELECTRICAL_HAZARD` are **latched** for the whole power
cycle: `Steering_EpsSetHealthyState()` refuses to clear them, and the owner is
forced to `NONE`. There is **no automatic reconnection** — recovery requires a
new power-up whose start-up checks pass.

## 2. Fault causes

`EpsFaultReason_t` records *why* the assist was disabled. It is latched on the
**first** call so the root cause survives the idempotent repeat calls:

`NONE, CENTER_NOT_FOUND, PB5, ENCODER_AB, ENCODER_Z, POSITION_MISMATCH,
CALIBRATION_INVALID, PARAMETERS_INVALID, CH5_MISSING, CH5_STALE, CH5_CONFIG,
OVERCURRENT, DIRECTION_POLARITY, OSCILLATION, DRIVER, POWER_CONFIRMATION,
OWNER_CONFLICT, UNKNOWN`.

## 3. Isolation function — exact shutdown order

`Steering_DisableAssistFault(reason)` is **idempotent** and performs, in order:

1. Latch `reason` (first call only).
2. `Steering_Neutralize()` — cancels any centering/EPS order and sets
   **PA6 CCR = 0, PA7 CCR = 0, PC4 = LOW** (BTS7960 coast).
3. `Steering_SteerPowerOff()` — **PC12 = OFF** via an atomic `GPIOC->BSRR`
   write. It does **not** touch the traction relay (PC11) nor the relay
   sequencer state, so `Safety_IsPowerReady()` stays valid for traction.
4. `owner = STEER_OWNER_NONE`.
5. `state = EPS_STATE_MECHANICAL_ONLY` (unless already `ELECTRICAL_HAZARD`).

It explicitly does **not** call `Safety_SetState(DEGRADED/LIMP_HOME/SAFE/ERROR)`,
does not change traction demand, gear, or traction relays. Repeated calls do not
re-energize PC12/PC4, generate pulses, toggle states, or restart centering — no
flapping.

### 3.1 Single relay-command authority in the EFFECTIVE sequencer

The firmware that actually compiles uses the wrappers
`safety_system_patched.c` / `steering_centering_patched.c` (see the `Makefile`
source list and the `.cproject` exclusions). Every path in that effective build
that could energise PC12 is gated by **one** authority,
`Steering_MotorRelayAllowed()`:

```c
Steering_IsCalibrated() &&
Steering_EpsIsAvailable() &&
!Steering_IsMechanicalOnly() &&
Steering_GetEpsState() != EPS_STATE_ELECTRICAL_HAZARD
```

When it is false, **PC12 = OFF**. This is applied at `RELAY_SEQ_TRACTION_ON`
completion, in `RELAY_SEQ_COMPLETE` on every tick, and the engineering relay
override plus the centering IDLE re-entry force PC12 OFF while the EPS is
latched. The previous patched sequencer used `Steering_IsCalibrated()` **alone**
and therefore re-closed PC12 after `Steering_DisableAssistFault()` — the defect
corrected here and covered by `Core/Src/test_relay_seq_eps.c`. After the EPS
latches `MECHANICAL_ONLY`, **no** function re-energises PC12 for the rest of the
power cycle. The traction relay (PC11) and `Safety_IsPowerReady()` are never
touched by this gate.

## 4. Isolable fault → mechanical-only

Any of the following isolable faults route through `Steering_DisableAssistFault`
and leave `PA6=0, PA7=0, PC4=LOW, PC12=OFF, owner=NONE, EPS=MECHANICAL_ONLY`
while the global state stays `ACTIVE` and traction keeps running:

PB5 not found / stuck / incoherent, centering timeout, encoder A/B no pulses or
incoherent or reversed, encoder Z absent/incoherent, PB5≠Z, calibration
missing/corrupt/bad-CRC, EPS parameters out of range / NaN / Inf / invalid
gains, impossible position or wheel speed, sustained oscillation, assist in the
wrong direction, CENTERING↔EPS owner conflict, INA226 CH5 missing/stale/config
invalid/unreadable, steering power confirmation absent, recoverable BTS7960 fault.

## 5. EPS fault vs electrical hazard

Loss of assistance is **not** a general danger. Only a **non-isolable**
electrical hazard (PC12 commanded OFF but power persists, BTS7960 short, motor
still drawing current with PA6=PA7=0 and PC4 LOW, critical over-current that does
not disappear when isolated, welded steering relay with active driver, smoke,
critical temperature, shared-supply collapse, bus-affecting short) may escalate
to `SAFE`/`ERROR`/global power cut, via `Steering_DeclareElectricalHazard()`.

**Always isolate first and verify** before escalating. A zero/stale/absent CH5
reading alone is an isolable fault, never a hazard.

### 5.1 Active detector supervisor (`steering_supervisor.{c,h}`)

Sections 4–5 describe *how* the assist is isolated; the **supervisor** is the
module that actually *connects the real detectors to that policy* every control
cycle. It runs at 100 Hz from `main.c` (after the steering motor writer, before
`Traction_Update()`), is initialised in `Steering_Init()`, and drives the
idempotent `steering_eps.c` API — it never calls `Safety_SetState` for an
isolable fault and never touches traction, gear, relays or the pedal.

The decision logic lives in pure, host-tested helpers; the thin production glue
that reads the real globals/HAL is `steering_supervisor_io.c`
(`SteeringSupervisor_Service()`).

| Detector (real source) | Condition | EPS action |
|------------------------|-----------|------------|
| `Sensor_GetChannel5Diag()->fault_reason` (`Ina226_ClassifyChannel`) | `MISSING` / `MUX_SELECT_FAIL` / `READ_FAIL` | `DisableAssistFault(CH5_MISSING)` |
| " | `STALE` | `DisableAssistFault(CH5_STALE)` |
| " | `CONFIG_LOST` / `WRONG_ID` | `DisableAssistFault(CH5_CONFIG)` |
| " | `POLARITY_REVERSED` | `DisableAssistFault(DIRECTION_POLARITY)` |
| INA226 CH5 signed current | `|I|` > `STEERING_OC_LIMIT_MA` (calibrated) | overcurrent FSM (below) |
| `EPS_Params_IsFlashCorrupt()` | slot present but bad CRC/structure | `DisableAssistFault(PARAMETERS_INVALID)` |
| `SteeringCal_IsFlashCorrupt()` / centering finished unrecovered | corrupt cal, or homing done without a valid centre | `DisableAssistFault(CALIBRATION_INVALID)` |
| `SteeringZ_GetStatus()` with `STEERING_Z_REQUIRED=1` | centre known but Z `NOT_SEEN` / `OUT_OF_WINDOW` / `MECH_OFFSET` | `DisableAssistFault(ENCODER_Z)` |

CH5 isolable faults are **debounced** (`STEERING_CH5_FAULT_DEBOUNCE`
consecutive cycles) so a single transient I2C hiccup never isolates the assist.

**Fresh vs corrupt store (audit item 4/5).** `EPS_Params_IsFlashCorrupt()` and
`SteeringCal_IsFlashCorrupt()` return true only when a slot is *present* (its
magic word matches, proving the page was written) **but** fails CRC/validity.
An erased/fresh device (magic never matches) returns false and silently accepts
compiled defaults; a genuinely corrupt existing device isolates the assist with
`PARAMETERS_INVALID` / `CALIBRATION_INVALID` instead of running on defaults.

**Encoder-Z policy (audit item 3).** `STEERING_Z_REQUIRED` (default `0`) selects
the policy: **optional** → an absent/incoherent Z is diagnostic only; **mandatory
(`1`)** → once PB5 confirms the centre, an absent/incoherent Z isolates the
assist with `EPS_FAULT_ENCODER_Z`. Both configurations are covered by
`test_steering_supervisor.c`.

**Overcurrent state machine (audit items 2, 5 & 6).** CH5 current drives a
**non-blocking** FSM with two physically distinct thresholds:

- `STEERING_ACTIVE_OVERCURRENT_MA` — the working-motor ceiling (mirrors the
  documented 25 A per-motor limit). Crossing it on a **valid** sample starts
  isolation.
- `STEERING_ISOLATION_CURRENT_MAX_MA` — the maximum residual current that may
  still flow through CH5 **after** the relay is commanded open and still be
  considered harmless (budgets INA226 offset, pre-relay electronics, shunt
  tolerance and noise). It is only consulted once
  `STEERING_ISOLATION_THRESHOLD_CALIBRATED == 1`.

States and transitions (`OcState_t`):

1. `OC_STATE_NORMAL` → over the **active** ceiling on a valid sample →
   **isolate** (`DisableAssistFault(OVERCURRENT)`: PA6/PA7 = 0, PC4 LOW,
   PC12 OFF, owner NONE, MECHANICAL_ONLY), capture the current CH5
   `sample_sequence` + isolation tick, and enter `OC_STATE_CONFIRM_WAIT`.
2. `OC_STATE_CONFIRM_WAIT` → wait (without blocking) for a genuinely **new**
   valid CH5 sample (`sample_sequence != captured`):
   - current below the residual ceiling → `OC_STATE_MECHANICAL_CONFIRMED`,
     `MECHANICAL_ONLY` retained, **no** hazard (isolable);
   - current still above the residual ceiling **and** the threshold is
     calibrated → `OC_STATE_HAZARD` (`Steering_DeclareElectricalHazard`);
   - current still above the residual ceiling but the threshold is **not**
     calibrated → `OC_STATE_ISOLATED_UNCONFIRMED` (stay isolated, **never**
     auto-SAFE; report `THRESHOLD UNCALIBRATED`).
3. `OC_STATE_CONFIRM_WAIT` timeout (`STEERING_OC_CONFIRM_MS` elapses **without**
   a fresh valid sample) → `OC_STATE_ISOLATED_UNCONFIRMED`. **Missing
   information is NOT proof of a persistent hazard**: the motor stays isolated,
   the diagnostic reports `ISOLATION UNCONFIRMED`, and the FSM keeps waiting for
   a future valid sample — it **never** escalates to HAZARD/SAFE on a timeout.
4. `OC_STATE_ISOLATED_UNCONFIRMED` → a later fresh valid sample is evaluated as
   in step 2 (below residual → `MECHANICAL_CONFIRMED`; above + calibrated →
   `HAZARD`).

Only a sample that is **new + valid + taken after isolation + still dangerous
+ with a calibrated residual threshold** can reach `OC_STATE_HAZARD`. In that
case the glue sees `SteeringSupervisor_WantsSafe()` and calls
`Safety_SetError(OVERCURRENT)` + `Safety_SetState(SAFE)` — the single point
where a *proven, non-isolable* steering hazard reaches the global safety state.

**Real acquisition identity (audit item 3).** The supervisor runs at 100 Hz but
INA226 CH5 is acquired at 20 Hz. `Ina226ChannelDiag::sample_sequence` increments
**exactly once** per new valid CH5 acquisition (never derived from the reader's
tick, never on an invalid read), so the 100 Hz service sees the **same** value
across the ~5 cycles that re-read one sample and only observes a change when a
genuinely new sample lands. The FSM compares `sample_sequence` for **inequality**
(the free-running `uint32_t` wrap after 2³² samples — >6 years at 20 Hz — is
intentional and safe). The production glue transports this real sequence
(`steering_supervisor_io.c`), so the confirm step can never re-process one
sample as if it were fresh.

**Residual-threshold calibration procedure.** `STEERING_ISOLATION_CURRENT_MAX_MA`
has **no physical measurement** yet, so `STEERING_ISOLATION_THRESHOLD_CALIBRATED`
ships at `0`. While uncalibrated, a post-isolation residual current can **never**
escalate to SAFE — the assist stays `MECHANICAL_ONLY` and the HMI shows
`THRESHOLD UNCALIBRATED`. To calibrate on the real vehicle:
1. Command PC12 OFF with the ignition/battery live and the steering motor at
   rest.
2. Log CH5 signed current for ≥60 s across the operating temperature range,
   capturing the worst-case residual (offset + pre-relay electronics + noise).
3. Set `STEERING_ISOLATION_CURRENT_MAX_MA` to that worst case plus an
   engineering margin.
4. Set `STEERING_ISOLATION_THRESHOLD_CALIBRATED` to `1`. Only then may the FSM
   declare `ELECTRICAL_HAZARD` from a persistent post-isolation current.

Fault injection tests exercise the **real** detectors and the **real production
glue**: `test_steering_supervisor.c` builds `Ina226ChannelDiag` snapshots,
classifies them with the genuine `Ina226_ClassifyChannel()`, and lets the
supervisor drive the real `steering_eps.c` (never calling
`Steering_DisableAssistFault()` directly). `test_steering_supervisor_io.c`
additionally drives `SteeringSupervisor_Service()` through a controllable
`Sensor_GetChannel5Diag()` seam, proving the real `sample_sequence` transport,
the 100 Hz/20 Hz freshness contract, CH5-always-powered, and that a missing
post-isolation sample stays `ISOLATION UNCONFIRMED` (never a false hazard).

## 6. Single motor owner

`SteeringMotorOwner_t`: `STEER_OWNER_CENTERING (0)`, `STEER_OWNER_EPS (1)`,
`STEER_OWNER_NONE (2)`. Centering, EPS, diagnostics, engineering menu, test and
recovery may **never** drive the motor simultaneously. On any conflict the EPS is
isolated (`EPS_FAULT_OWNER_CONFLICT`), owner becomes `NONE`, and traction is
preserved.

## 7. Never brake the wheel

When the command is neutral the outputs are PA6=0/PA7=0 with PC4 leaving the
BTS7960 in **COAST** — no residual PWM, no BRAKE to hold position. In
`EPS_STATE_MECHANICAL_ONLY` PC12 stays OFF, PC4 stays LOW, PA6/PA7 stay zero, and
no centering/return/damping/motor-test runs (`Steering_ControlLoop()` returns
early after neutralizing).

## 8. Reactivation policy

The fault is latched for the power cycle. A new power-up may re-enable EPS only if
calibration is valid, PB5/encoder are coherent, parameters are valid, CH5 is valid
(when required), owner is NONE, PA6/PA7 are zero, PC4 is LOW, there is no electrical
hazard, and the start-up checks pass. The fault history is **not** erased.

## 9. Telemetry / HMI

The existing `0x316` steering-centering diagnostic frame already carries the owner
nibble plus PWM/EN/relay/system-state, so telemetry is reused. Owner value `2`
(NONE) renders on the HMI as **"EPS OFF (MECANICA)"**. An EPS notice is a *local*
assist fault and must **not** trigger the global LIMP_HOME/DEGRADED screens.

## 10. Wiring summary

| Signal | Pin  | Role                                                     |
|--------|------|----------------------------------------------------------|
| RPWM   | PA6  | TIM3_CH1 steering PWM                                     |
| LPWM   | PA7  | TIM3_CH2 steering PWM                                     |
| EN     | PC4  | EN_STEER (LOW = coast)                                    |
| RELAY  | PC12 | Steering-**motor** isolation relay **command** (OFF=open); pre-relay INA226/shunt supply stays present. Legacy name `PIN_RELAY_STEER_PWR`. |
| CENTER | PB5  | Physical center reference                                |
| CH5    | —    | TCA9548A ch5 → INA226, **before** PC12: pre-relay 12 V bus voltage + steering-branch current. Does NOT confirm the relay contact. |

Traction relay (PC11) and traction power-ready are independent and untouched by
EPS isolation.
