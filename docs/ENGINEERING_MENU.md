# Engineering Menu (Hidden)

## Access

The engineering menu is a hidden screen in the ESP32-S3 HMI. To open it:

1. **Long-press (hold ~3 s) on any free area of the touch screen.** This opens
   the PIN entry screen. The gesture works from any normal screen — including
   **Safe Mode** — and is guarded against accidental activation: lifting the
   finger before 3 s, or dragging it more than ~40 px, cancels/restarts the
   timer (no false positives).
2. Enter the secret code **"8989"** on the PIN keypad.
3. On the correct PIN the Engineering menu opens.

> **Compatibility:** long-pressing the battery icon still works, because the icon
> region is simply a subset of the full-screen gesture. No separate handling is
> required.

It provides diagnostic and calibration tools for use during development and
maintenance.

**File**: `esp32/src/screens/engineering_screen.cpp/h`

## Main Menu Options

| # | Label | SubMenu | Description |
|---|-------|---------|-------------|
| 1 | FAULT VIEWER | `FAULT_VIEWER` | View fault/enabled/disabled bitmasks (32-bit hex) |
| 2 | MODULE ENABLE/DISABLE | `MODULE_CONTROL` | Toggle individual non-critical modules with ACK feedback |
| 3 | PEDAL CALIBRATION | `PEDAL_CAL` | Persistent pedal-endpoint capture + save (flash page 124). See [`CALIBRATION.md`](CALIBRATION.md) |
| 4 | ENCODER CALIBRATION | `ENCODER_CAL` | Live steering encoder verification with visual gauge |
| 5 | INA226 SENSOR MAPPING | `SENSOR_MAP_INA` | Map INA226 I2C channels to vehicle positions (FL/FR/RL/RR/Battery/Steering) |
| 6 | TEMP SENSOR MAPPING | `SENSOR_MAP_TEMP` | Map DS18B20 sensors to vehicle positions (FL/FR/RL/RR/Ambient) |
| 7 | FACTORY DEFAULTS | `FACTORY_DEFAULTS` | Individual factory-default reset options (see below), includes full factory restore |
| 8 | DTC ERROR LOG | `DTC_LOG_VIEWER` | Persistent NVS-backed DTC fault log viewer |
| 9 | MAINTENANCE | `MAINTENANCE` | Maintenance counter status / acknowledge / reset |
| 10 | RELAY CONTROL (DEBUG) | `RELAY_CONTROL` | Manual relay override for engineering diagnostics |
| 11 | INA226 LIVE DIAG | `INA226_LIVE_DIAG` | Live INA226 diagnostics: CH0-3 amps, CH4 battery amps/volts, CH4 INA OK/expected, masks, mux, fail/recovery, 0x309 stale/no-data |
| 12 | DEBOUNCE DEBUG | `DEBOUNCE_DIAG` | DWT-debounce EMI filtered counters viewer |
| 13 | TOUCH CALIBRATION | _wizard_ | Launch the persistent touch-calibration wizard. See [`TOUCH_CALIBRATION_SYSTEM.md`](TOUCH_CALIBRATION_SYSTEM.md). |
| 14 | RESET TOUCH CAL | _action_ | Erase persisted touch calibration in NVS and re-arm the first-boot wizard for the next reboot. |
| 15 | MCP23017 LIVE (SHIFTER) | `MCP23017_LIVE` | ESP32-local MCP23017 shifter I2C live diagnostic (cached, no bus access from render path) |
| 16 | GEAR LIMITS | `GEAR_LIMITS` | View/adjust per-gear traction **power limits** and **accel response** profile (D2/D1/R %, two paged groups via PAGE). Edits staged locally, applied + persisted on the STM32 only on SAVE. See [`#gear-limits-screen`](#gear-limits-screen) |

## Presentation — Professional Tile Layout (FASE 2)

The main menu is rendered as a **professional, workshop-grade tile grid**
(inspired by automotive diagnostic tools) instead of a dense list of thin
rows. **The 15 functions, their dispatch and all submenu logic are
unchanged** — only the presentation/touch geometry was redesigned. A tile tap
maps back to the exact same item index and runs the exact same code path as
the old list row. (A 16th tile, **GEAR LIMITS**, was later appended on PAGE 2
without altering items 1–15.)

- **Two pages** of large touch tiles:
  - **PAGE 1** (3×3, items 1–9): Fault Viewer, Module Enable/Disable, Pedal
    Calibration, Encoder Calibration, INA226 Mapping, Temp Mapping, Factory
    Defaults, DTC Error Log, Maintenance.
  - **PAGE 2** (3×3, items 10–16): Relay Control, INA226 Live Diag,
    Debounce/CAN Diag, Touch Calibration, Reset Touch Cal, MCP23017 Live,
    Gear Limits.
- **Tile size**: 148 × 72 px with a 10 px separation — well above the 44 px
  minimum touch target, usable with gloves. No overlaps; gaps intentionally
  separate tiles.
- **Captions**: rendered at text size 2 (two short lines) for legibility from
  the driver's seat; critical captions are never size 1.
- **Icons**: each tile carries a category icon (warning triangle, ON/OFF
  switch, pedal, steering wheel, ammeter, thermometer, reset, clipboard,
  wrench, relay, CAN network, target, IC chip). Icons are drawn procedurally
  with TFT_eSPI primitives — **no PROGMEM bitmap arrays, no heap, zero RAM**.
- **Colour coding**: Diagnostic = cyan, Calibration = green, Configuration =
  blue, Maintenance = yellow, Destructive (Factory Defaults / Reset Touch Cal /
  Relay Control) = amber/red. Captions stay white so legibility never depends
  on the accent hue.
- **Navigation bar** (always visible on both pages): **PAGE 1**, **PAGE 2**,
  **EXIT**. Submenus keep their existing **BACK** button.
- The compact **RELAY STATUS** read-out moved to the header's top-right corner
  and is shown on both pages.

> **Safety unchanged (PR #385):** Factory Defaults and Module Enable/Disable
> keep their double-tap confirmation, Relay Control stays STANDBY-only, and
> mapping validation is intact. The redesign only changes which tile opens a
> submenu, not what happens inside it.

## Submenu Details

### 1. Fault Viewer

Displays three 32-bit bitmasks from CAN messages 0x301–0x303:
- **Fault Bits** (red/green): Modules with active faults
- **Enabled Bits** (green): Currently enabled modules
- **Disabled Bits** (amber): Manually disabled modules

Values update in real-time via CAN telemetry.

### 2. Module Enable/Disable

Paginated list of all 25 modules (3 pages, 9 modules per page):
- **CRITICAL** modules (0–3): Cannot be toggled (shown as "CRITICAL" in amber)
- **NON-CRITICAL** modules (4–24): Tap to toggle enable/disable
- Status indicators: ENABLED (green dot), DISABLED (gray), FAULT (red), FAULT+DIS (orange)
- ACK feedback bar shows result after toggle (OK/REJECTED/BLOCKED)

**Double-tap confirmation (UI guard):**
- A toggle is only sent after the operator taps the **same** non-critical module **twice**.
- The first tap arms the row (status shows `CONFIRM? TAP`, amber) and does **not** send any CAN command.
- The second tap on the same module sends the ENABLE/DISABLE command.
- The pending confirmation is cancelled by: tapping a different row, the PAGE button, BACK, leaving the screen, or a 5 s timeout.
- Critical modules (0–3) never arm a confirmation and never send a command.

**Safety gates** (enforced server-side on STM32):
- System state must be ACTIVE or DEGRADED
- Safety-relevant modules (ABS, TCS, wheel speed, obstacle) cannot be disabled while vehicle speed > 0

### 3. Pedal Calibration

Persistent capture of the accelerator-pedal raw ADC endpoints (released → MIN, fully-pressed → MAX), saved to STM32 flash page 124 (`0x0807C000`).  See [`CALIBRATION.md`](CALIBRATION.md) for the full procedure.

**Layout (live, fed by 0x308 on-demand burst at 10 Hz):**

| Left column (live) | Right column (slot state) |
|--------------------|----------------------------|
| Raw ADC            | Stored MIN (or "default")  |
| Pedal %            | Stored MAX (or "default")  |
| Stable yes/no      | Pending MIN (or `--`)      |
| Plausible          | Pending MAX (or `--`)      |
| Safety gate OK/BLOCKED | Validation OK / OUT OF RANGE / incomplete |

**Buttons:**

- **CAPTURE MIN** — sample 8 ADC readings @ 50 ms (400 ms total) and store the mean as *pending MIN* if dispersion ≤ 8 counts (else `ACK_REJECTED`).
- **CAPTURE MAX** — identical workflow for *pending MAX*.
- **SAVE** — validates pending pair and persists to flash. Greyed-out until validation **and** safety gates are OK. Hard validators: `min ≥ 50`, `max ≤ 2600`, `max > min`, `max − min ≥ 800`.
- **RESET DEFAULTS** — re-persists the compile-time defaults (150 / 2413) and applies them immediately.
- **BACK** — return to engineering main; the 0x308 burst naturally stops 1 s later.

**Safety gates** (enforced server-side on STM32, all sub-opcodes except QUERY):

- `Safety_GetState() == SYS_STATE_STANDBY`
- `Startup_IsInhibited() == true`
- `Pedal_GetPercent() < 3.0f` (for MIN/MAX/SAVE — the operator deliberately drives the pedal during CAPTURE_MAX, but the gate is evaluated *before* the 400 ms sample window)
- `Pedal_IsPlausible() == true`
- all four wheel speeds `< 0.3 km/h`

Any gate failure → `ACK_BLOCKED_BY_SAFETY` (CMD_ACK 0x103, DLC 3 preserved).

**Fallback:** if flash page 124 has invalid CRC/magic or stores an out-of-range pair, the STM32 silently falls back to the compile-time defaults (150 / 2413) — boot is never blocked and the pedal is never unusable.

### 4. Encoder Calibration

Live steering encoder verification:
- Steering angle (±XX.X degrees)
- Calibration status (CALIBRATED / NOT CALIBRATED)
- Visual angle gauge bar (centered, range ±45°)
- Instructions for turning steering to verify encoder response

**STEERING Z CENTER section** (PB5 + encoder-Z dual reference diagnostic):
- PB5 state (LJ12A3 inductive sensor — the **primary** center/safety reference)
- Z pulse count, Z last position, Z↔center offset
- Z calibrated (YES/NO), Z slip (YES/NO), active tolerance
- Combined status: **OK / Z NOT SEEN / Z OUT OF WINDOW / MECHANICAL OFFSET /
  NOT CALIBRATED**
- Actions: **QUERY Z**, **CALIBRATE Z OFFSET**, **CLEAR Z CALIBRATION** (CLEAR
  requires a double confirmation)

The encoder Z (index) pulse is a **secondary, precision-only** reference — it
can never center on its own and a Z seen without PB5 is **not** a center. All
actions are validated on the STM32 (the ESP32 is never the authority);
CALIBRATE is only accepted while PB5 reads center, in BOOT/STANDBY. Telemetry
arrives on CAN `0x30E` (see [`STEERING_Z_CENTER.md`](STEERING_Z_CENTER.md) and
`CAN_CONTRACT_FINAL.md` §4.19).

### 5. INA226 Sensor Mapping

Table editor for mapping INA226 TCA9548A channels to vehicle positions:
- 6 channels (Ch0–Ch5) → 6 positions (FL Motor, FR Motor, RL Motor, RR Motor, Battery, Steering)
- Tap a row to cycle through position assignments
- SAVE button to persist to config store
- BACK to discard changes

> **Config-store guard:** `config_store::setIna226Map()` bounds-checks every
> entry (valid = `0..5`, or `0xFF` unset). A single out-of-range byte rejects the
> whole update — the existing mapping and NVS blob are left untouched (no NVS
> format change, no config wipe).

### 6. Temp Sensor Mapping

Table editor for mapping DS18B20 sensors to vehicle positions:
- 5 sensors (Sens0–Sens4) → 5 positions (FL Wheel, FR Wheel, RL Wheel, RR Wheel, Ambient)
- Same tap-to-cycle and SAVE/BACK behavior as INA226 mapping

> **Config-store guard:** `config_store::setTempSensorMap()` bounds-checks every
> entry (valid = `0..4`, or `0xFF` unset) and rejects the whole update on any
> out-of-range byte, leaving the existing mapping intact.

### 7. Factory Defaults

Individual reset options for specific calibration categories:

| Option | CAN Action Code | Modules Affected |
|--------|----------------|-----------------|
| RESET STEERING PID | `0xF0` | Steer Center (19), Steer Encoder (20) |
| RESET WHEEL SENSORS | `0xF1` | Wheel Speed FL/FR/RL/RR (15–18) |
| RESET INA226 / SHUNTS | `0xF2` | Current Sensor 0–5 (9–14) |
| RESET TRACTION MOTOR FORCE | `0xF3` | ABS (21), TCS (22) |
| RESET STEERING MOTOR FORCE | `0xF4` | Steer Center (19), Steer Encoder (20), Ackermann (23) |
| RESET ALL (FACTORY RESTORE) | `0xFF` | All 25 modules |

Each option sends a SERVICE_CMD (0x110) via CAN to the STM32, which re-enables the
affected modules and clears their faults.

**Double-tap confirmation (UI guard):**
- The first tap on an option arms it (the row turns amber and shows `CONFIRMAR? PULSA OTRA VEZ`) and does **not** send any CAN command.
- A second tap on the **same** option sends the reset command.
- Any other touch, BACK, leaving the screen, or a 5 s timeout cancels the pending confirmation.

**Warning text**: "Vehicle must be stationary. Reboot may be required."

### 11. INA226 Live Diag

Read-only diagnostic page driven by existing CAN telemetry only:
- `0x201` → CH0 FL, CH1 FR, CH2 RL, CH3 RR currents (0.01 A)
- `0x207` → CH4 BT current + voltage (0.01 A / 0.01 V)
- `0x309` → MUX state, INA OK mask, expected mask, fail/recovery counters, stale/no-data status

Displayed values:
- `CH0..CH3: xx.x A`
- `CH4 BT: xx.x A   xx.x V` (highlighted)
- `CH5 ST: --.- A (n/d)` (not on CAN yet)
- `BT volts`, `BT amps`, `BT INA = OK/FAIL`, `BT expected = YES/NO` (or `N/D` when `0x309` has no data)
- `INA OK` / `EXPECTED` masks (hex), MUX state, fail/recovery counters, `0x309` staleness

Interpretation hint:
- rest current ~0 A → expected sensor behavior
- rest current ~100 A with volts OK → suspect shunt/sense path or calibration/wiring issue

### 10. Relay Control (Debug)

Manual relay override for engineering diagnostics (TRACTION PC11, STEER PWR PC12).

**STANDBY gate (UI guard, FASE 2 §2):**
- Relay override is **STANDBY-only** by safety design on the STM32.
- Before changing any local visual state, the HMI verifies the cached system
  state (`sysStateRaw_`) is `STANDBY`. If it is not, the tap is refused: the
  local UI is **not** changed and an `ONLY IN STANDBY` notice is flashed instead,
  so the buttons never misrepresent the real relay state.
- **Auto-off** is preserved on every exit path: the override is disabled (CAN
  `RELAY_OVERRIDE` enable=0, mask=0) when the operator presses BACK, when the
  engineering screen is left, and automatically when the STM32 leaves STANDBY
  (detected from the heartbeat `systemState`).

## Navigation

- **BACK** button (bottom-left): Returns to previous submenu or exits engineering mode
- **EXIT** button (main menu): Returns to normal driving screens
- **SAVE** button (sensor mapping only, bottom-right): Persists mapping to config store
- **PAGE >>** button (module control only): Cycles through module pages

## CAN Protocol

All factory defaults and module control commands use CAN ID `0x110` (SERVICE_CMD):
- DLC: 2 bytes
- Byte 0: Action code (0x00=disable, 0x01=enable, 0xF0-F4=category reset, 0xFF=factory restore)
- Byte 1: Module ID (for disable/enable) or 0x00 (for factory reset commands)

ACK responses received on CAN ID `0x103` (CMD_ACK):
- Byte 0: Command ID low byte (0x10 for SERVICE_CMD)
- Byte 1: Result (0=OK, 1=REJECTED, 2=INVALID, 3=BLOCKED_BY_SAFETY)

## Gear Limits Screen

The **GEAR LIMITS** sub-screen (`GEAR_LIMITS`, item 16) lets a technician view
and adjust two per-gear profiles the **STM32** applies, both expressed as a
percentage and edited on the same screen across two pages (toggled by the
top-right **PAGE** button):

1. **POWER LIMIT %** — the maximum traction power per shifter position, applied
   in `Traction_Update()`.
2. **ACCEL RESPONSE %** — the acceleration aggressiveness per shifter position,
   applied in `Traction_SetDemand()` after the EMA pedal filter and **before**
   the global ramp limiter. The factor scales a strictly positive demand target
   and is clamped ≤ 100 %, so it can only **soften** (never amplify) the pedal
   response. It does not touch dynamic braking, ABS, TCS, SAFE or LIMP_HOME.

| Gear | POWER default | POWER range | RESPONSE default | RESPONSE range |
|------|---------------|-------------|------------------|----------------|
| **D2** | 100 % | 30–100 % | 100 % | 50–100 % |
| **D1** | 60 %  | 20–100 % | 70 %  | 30–100 % |
| **R**  | 60 %  | 10–60 %  | 40 %  | 20–80 %  |

> **R power discrepancy:** the original change request quoted R = 30 %, but the
> shipped firmware applies `GEAR_POWER_REVERSE_PCT = 0.60` (60 %). To avoid
> silently changing traction behaviour, the power default is kept at the firmware
> value (60 %); the allowed range (10–60 %) lets an operator set 30 % from this
> screen if desired. The RESPONSE defaults (100/70/40) are new and make reverse
> markedly more progressive without changing any power limit.

**Controls**
- **PAGE** (top-right) toggles between the POWER LIMIT and ACCEL RESPONSE groups.
  Switching pages does **not** discard pending edits — both groups are committed
  together on SAVE.
- Each gear row has a **−5 %** and **+5 %** stepper (clamped to the gear's
  range for the currently shown page) that edits a *local pending* value only.
- **SAVE** transmits all six staged values (`SET_D2/D1/R` + `SET_D2/D1/R_RESPONSE`)
  followed by `SAVE`; the STM32 validates and persists both groups atomically.
- **RESTORE DEF.** restores factory defaults for **both** power and response; it
  is destructive so it requires a **double-tap confirmation** (the button turns
  red and reads `CONFIRM RESTORE`; a 5 s timeout disarms it).
- **BACK** discards all pending edits and returns to the main menu.

**Display**
- **ACTIVE** column = the value the STM32 is applying right now (from the 0x30D
  telemetry burst — POWER and RESPONSE frames).
- **EDIT** column = the pending value (highlighted amber when it differs from
  ACTIVE).
- Status banner: `SAVING…`, `SAVED`, `REJECTED (need STANDBY)`, `INVALID`, or
  `TIMEOUT`.

**Safety / behaviour**
- Nothing changes until **SAVE** is pressed; `SET_*` only stages a pending value
  on the STM32, it does not apply it.
- `SET_*`, `SAVE` and `RESTORE DEFAULTS` are **STANDBY-only** on the STM32; a tap
  outside STANDBY is answered with `BLOCKED_BY_SAFETY` (banner shows REJECTED).
- The STM32 re-validates every value against its own ranges (`INVALID` on
  failure) and only persists to flash (page 122, magic+CRC32, mirroring the
  pedal-calibration store) when validation passes. Values survive reboot.
- The flash slot is v2 (`"GLM2"`, power + response). A pre-existing v1
  (power-only, `"GLM1"`) slot is migrated safely on boot: the persisted power
  limits are kept, the response defaults (100/70/40) are applied, and the slot
  is rewritten as v2 only on the next SAVE.
- `QUERY` is read-only (no safety gate) and just triggers the telemetry burst.

See the [CAN contract](CAN_CONTRACT_FINAL.md) for the `SERVICE_ACTION_GEAR_LIMITS`
(`0xF7`) sub-opcodes and the `0x30D DIAG_GEAR_LIMITS` frame layout (including the
byte0 bit4 POWER/RESPONSE frame-kind discriminator).
