# Engineering Menu (Hidden)

## Access

The engineering menu is a hidden screen in the ESP32-S3 HMI, accessed by entering the
secret code **"8989"** on the touch screen. It provides diagnostic and calibration tools
for use during development and maintenance.

**File**: `esp32/src/screens/engineering_screen.cpp/h`

## Main Menu Options

| # | Label | SubMenu | Description |
|---|-------|---------|-------------|
| 1 | FAULT VIEWER | `FAULT_VIEWER` | View fault/enabled/disabled bitmasks (32-bit hex) |
| 2 | MODULE ENABLE/DISABLE | `MODULE_CONTROL` | Toggle individual non-critical modules with ACK feedback |
| 3 | PEDAL CALIBRATION | `PEDAL_CAL` | Live verification of pedal→motor response |
| 4 | ENCODER CALIBRATION | `ENCODER_CAL` | Live steering encoder verification with visual gauge |
| 5 | INA226 SENSOR MAPPING | `SENSOR_MAP_INA` | Map INA226 I2C channels to vehicle positions (FL/FR/RL/RR/Battery/Steering) |
| 6 | TEMP SENSOR MAPPING | `SENSOR_MAP_TEMP` | Map DS18B20 sensors to vehicle positions (FL/FR/RL/RR/Ambient) |
| 7 | FACTORY DEFAULTS | `FACTORY_DEFAULTS` | Individual factory-default reset options (see below) |
| 8 | FACTORY RESTORE (ALL) | — | Send full factory restore (0xFF) to STM32 |

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

Live telemetry display for verifying pedal response:
- Per-wheel: Speed (km/h), Current (A, color-coded), Traction scale (%)
- Battery: Voltage (V), Current (A)
- Safety: ABS/TCS active indicators
- Current color coding: Green <15A, Yellow 15–25A, Red >25A (BTS7960 limits)

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

### 8. Factory Restore (ALL)

Direct button on the main engineering menu that sends SERVICE_CMD 0xFF to the STM32.
Re-enables ALL modules and clears all manual disables.

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
