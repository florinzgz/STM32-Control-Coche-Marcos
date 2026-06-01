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
| 11 | DEBOUNCE DEBUG | `DEBOUNCE_DIAG` | DWT-debounce EMI filtered counters viewer |
| 12 | TOUCH CALIBRATION | _wizard_ | Launch the persistent touch-calibration wizard. See [`TOUCH_CALIBRATION_SYSTEM.md`](TOUCH_CALIBRATION_SYSTEM.md). |
| 13 | RESET TOUCH CAL | _action_ | Erase persisted touch calibration in NVS and re-arm the first-boot wizard for the next reboot. |

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

**Safety gates**:
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

### 5. INA226 Sensor Mapping

Table editor for mapping INA226 TCA9548A channels to vehicle positions:
- 6 channels (Ch0–Ch5) → 6 positions (FL Motor, FR Motor, RL Motor, RR Motor, Battery, Steering)
- Tap a row to cycle through position assignments
- SAVE button to persist to config store
- BACK to discard changes

### 6. Temp Sensor Mapping

Table editor for mapping DS18B20 sensors to vehicle positions:
- 5 sensors (Sens0–Sens4) → 5 positions (FL Wheel, FR Wheel, RL Wheel, RR Wheel, Ambient)
- Same tap-to-cycle and SAVE/BACK behavior as INA226 mapping

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

**Warning text**: "Vehicle must be stationary. Reboot may be required."

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
