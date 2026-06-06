// =============================================================================
// ESP32-S3 HMI — Centralized UI Configuration
//
// All magic numbers, thresholds, text padding widths, and overlay layout
// constants are defined here. No screen or widget should contain hardcoded
// visual parameters — they must reference constants from this header.
//
// Naming convention:
//   PAD_*   — text padding widths (pixels) for setTextPadding()
//   THR_*   — threshold values for hysteresis / jitter filtering
//   OVL_*   — overlay layout constants
//   BATT_*  — battery indicator constants
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef UI_CONFIG_H
#define UI_CONFIG_H

#include <cstdint>

namespace ui {
namespace cfg {

// =========================================================================
// Text Padding Widths (pixels)
//
// Each constant is sized for the worst-case string at the given font size.
// Derived from TFT_eSPI default font metrics:
//   Size 1: ~6 px/char,  Size 2: ~12 px/char,  Size 3: ~18 px/char
// =========================================================================

// DriveScreen — speed display (size 3, max "999.9")
inline constexpr int16_t PAD_SPEED           = 120;

// DriveScreen — RPM display (size 1, max "400 rpm")
inline constexpr int16_t PAD_RPM             = 60;

// DriveScreen — CAN link state (size 1, "OK"/"--")
inline constexpr int16_t PAD_CAN_STATE       = 16;

// DriveScreen — ACK indicator (size 1, max "REJECTED")
inline constexpr int16_t PAD_ACK             = 80;

// DriveScreen — wheel labels (size 2, max "100%"; size 1, max "-99°C")
inline constexpr int16_t PAD_WHEEL_LABEL     = 46;

// DriveScreen — pedal percentage (size 2, max "100%")
inline constexpr int16_t PAD_PEDAL_TEXT      = 70;

// Obstacle sensor — distance text (size 1, max "12.00 m")
inline constexpr int16_t PAD_OBSTACLE_DIST   = 80;

// ErrorScreen — fault hex code (size 2, max "0xFF")
inline constexpr int16_t PAD_ERROR_HEX       = 80;

// ErrorScreen — full-width text fields (size 2)
inline constexpr int16_t PAD_ERROR_FULL      = 460;

// SafeScreen — fault/error text (size 1, full-width)
inline constexpr int16_t PAD_SAFE_FULL       = 460;

// SafeScreen — per-wheel speed/current values (size 1, max "999.9 km/h")
inline constexpr int16_t PAD_SAFE_WHEEL_VAL  = 78;

// SafeScreen — temperature values (size 1, max "-99°C")
inline constexpr int16_t PAD_SAFE_TEMP       = 52;

// SafeScreen — steering angle (size 1, max "-999.9°")
inline constexpr int16_t PAD_SAFE_STEERING   = 140;

// StandbyScreen — temperature values (size 1, max "-99 C")
inline constexpr int16_t PAD_STANDBY_TEMP    = 80;

// StandbyScreen — fault flags (size 1, centered)
inline constexpr int16_t PAD_STANDBY_FAULTS  = 240;

// BootScreen — CAN status text (size 1, max "CAN: WAITING...")
inline constexpr int16_t PAD_BOOT_CAN_STATUS = 160;

// Battery indicator — percentage text (size 1, max "100%")
// Based on battery inner width (75 total - 10 outline - 4 margin ≈ 61) minus padding
inline constexpr int16_t PAD_BATTERY_TEXT    = 55;

// =========================================================================
// CAN Jitter Filtering Thresholds (DriveScreen wheels)
//
// Prevent redraw from minor CAN noise. Only redraw when the value change
// exceeds the threshold. Applied in update() phase only.
// =========================================================================

// Traction: redraw only if absolute Δ > this value (%)
inline constexpr uint8_t THR_TRACTION_DELTA  = 2;

// Temperature: redraw only if absolute Δ ≥ this value (°C)
inline constexpr int8_t  THR_TEMP_DELTA      = 1;

// =========================================================================
// Battery Indicator Hysteresis
//
// Text on the battery bar switches between dark-on-color and light-on-bg.
// Hysteresis prevents rapid flipping near the 50% fill boundary.
// Thresholds are expressed as percentage of bar inner width (0–100).
// =========================================================================

// Switch TO dark text when fill >= this %
inline constexpr uint8_t BATT_HYSTERESIS_HIGH = 55;

// Switch FROM dark text when fill < this %
inline constexpr uint8_t BATT_HYSTERESIS_LOW  = 45;

// Battery voltage range (0.01V units) for 24V pack
inline constexpr uint16_t BATT_VOLTAGE_MIN_RAW = 1800;  // 18.00 V = 0%
inline constexpr uint16_t BATT_VOLTAGE_MAX_RAW = 2520;  // 25.20 V = 100%

// =========================================================================
// ACK Visual Feedback
// =========================================================================

// Duration the ACK indicator is shown (ms)
inline constexpr unsigned long ACK_DISPLAY_DURATION_MS = 1500;

// ACK indicator position in top bar
inline constexpr int16_t ACK_X = 200;
inline constexpr int16_t ACK_Y = 2;
inline constexpr int16_t ACK_W = 80;
inline constexpr int16_t ACK_H = 16;

// =========================================================================
// Overlay Layout — Degraded/Limp Banner (DriveScreen)
// =========================================================================

inline constexpr int16_t OVL_DEGRADED_X = 0;
inline constexpr int16_t OVL_DEGRADED_Y = 40;
inline constexpr int16_t OVL_DEGRADED_W = 480;
inline constexpr int16_t OVL_DEGRADED_H = 18;

// =========================================================================
// Overlay Layout — Fault Indicators (DriveScreen top bar bottom margin)
// =========================================================================

inline constexpr int16_t OVL_FAULT_Y = 28;
inline constexpr int16_t OVL_FAULT_H = 10;

// Gap between fault label strings (pixels)
inline constexpr int16_t OVL_FAULT_LABEL_GAP = 4;

// =========================================================================
// Temperature Color Thresholds (SafeScreen)
// =========================================================================

inline constexpr int8_t TEMP_COLOR_WARNING  = 60;  // °C → amber
inline constexpr int8_t TEMP_COLOR_CRITICAL = 80;  // °C → red

// =========================================================================
// Pedal Bar Color Thresholds
// =========================================================================

inline constexpr uint8_t PEDAL_COLOR_LOW    = 40;  // ≤ this → green
inline constexpr uint8_t PEDAL_COLOR_MID    = 70;  // ≤ this → yellow, else red

// =========================================================================
// Battery Level Color Thresholds
// =========================================================================

inline constexpr uint8_t BATT_COLOR_LOW     = 20;  // ≤ this → red
inline constexpr uint8_t BATT_COLOR_MID     = 50;  // ≤ this → yellow, else green

// =========================================================================
// RPM Display (DriveScreen — alongside speed)
//
// Wheel RPM derived from average speed and wheel circumference (1.1 m).
// Formula: RPM = raw_0_1kmh * 100 / 66  (integer-only, no float)
// Capped at RPM_DISPLAY_MAX for the display (matches vehicle design max).
// =========================================================================

// Maximum RPM shown on display (clamp)
inline constexpr uint16_t RPM_DISPLAY_MAX    = 400;

// X position for RPM sub-label (TC_DATUM, at SPEED_Y + 26)
inline constexpr int16_t RPM_LABEL_X         = 295;

// =========================================================================
// Obstacle Sensor Bar Max Range (cm)
// =========================================================================

inline constexpr uint16_t OBSTACLE_BAR_MAX_CM = 600;

// =========================================================================
// DriveScreen Tile Layout Dimensions
//
// Defines the geometry of each tile region on the drive screen.
// These constants are referenced in DriveScreen::onEnter() setRect() calls.
// All values are in pixels relative to the 480×320 screen.
// =========================================================================

// Top bar sub-regions (Y: 0–40)
inline constexpr int16_t DTILE_MODE_ICONS_X  = 0;
inline constexpr int16_t DTILE_MODE_ICONS_W  = 180;
inline constexpr int16_t DTILE_LED_TOGGLE_X  = 180;
inline constexpr int16_t DTILE_LED_TOGGLE_W  = 120;
inline constexpr int16_t DTILE_BATTERY_W     = 75;

// Car area split (Y: CAR_AREA_Y)
inline constexpr int16_t DTILE_WHEELS_W      = 370;
inline constexpr int16_t DTILE_STEERING_X    = 370;
inline constexpr int16_t DTILE_STEERING_W    = 110;

// =========================================================================
// ErrorScreen Tile Layout Dimensions
// =========================================================================

inline constexpr int16_t ETILE_BANNER_H      = 75;
inline constexpr int16_t ETILE_CONTENT_X     = 10;
inline constexpr int16_t ETILE_CONTENT_W     = 460;
inline constexpr int16_t ETILE_FAULTS_Y      = 80;
inline constexpr int16_t ETILE_FAULTS_H      = 83;
inline constexpr int16_t ETILE_SAFETY_Y      = 170;
inline constexpr int16_t ETILE_SAFETY_H      = 40;
inline constexpr int16_t ETILE_DIAG_Y        = 220;
inline constexpr int16_t ETILE_DIAG_H        = 40;
inline constexpr int16_t ETILE_ELAPSED_Y     = 270;
inline constexpr int16_t ETILE_ELAPSED_H     = 30;

// =========================================================================
// StandbyScreen Tile Layout Dimensions
// =========================================================================

inline constexpr int16_t YTILE_TEMPS_X       = 140;
inline constexpr int16_t YTILE_TEMPS_Y       = 185;
inline constexpr int16_t YTILE_TEMPS_W       = 200;
inline constexpr int16_t YTILE_TEMPS_H       = 110;
inline constexpr int16_t YTILE_FAULTS_Y      = 290;
inline constexpr int16_t YTILE_FAULTS_H      = 30;

// =========================================================================
// SafeScreen Layout Dimensions
//
// Vertical stacking layout for SAFE MODE screen:
//   Banner (0–40), Info (46), Faults (62–86), Error (90–114),
//   Separator (118), Speeds (126–152), Currents (154–180),
//   Temps (182–208), Steering (214+)
// =========================================================================

inline constexpr int16_t STILE_BANNER_H        = 40;
inline constexpr int16_t STILE_INFO_Y           = 46;
inline constexpr int16_t STILE_FAULT_LABEL_Y    = 62;
inline constexpr int16_t STILE_FAULT_VALUE_Y    = 74;
inline constexpr int16_t STILE_ERR_LABEL_Y      = 90;
inline constexpr int16_t STILE_ERR_VALUE_Y      = 102;
inline constexpr int16_t STILE_TELEM_SEP_Y      = 118;
inline constexpr int16_t STILE_SPEED_LABEL_Y    = 126;
inline constexpr int16_t STILE_SPEED_VAL_Y      = 138;
inline constexpr int16_t STILE_CURR_LABEL_Y     = 154;
inline constexpr int16_t STILE_CURR_VAL_Y       = 166;
inline constexpr int16_t STILE_TEMP_LABEL_Y     = 182;
inline constexpr int16_t STILE_TEMP_VAL_Y       = 194;
inline constexpr int16_t STILE_STEER_Y          = 214;
inline constexpr int16_t STILE_COL_LABEL_X      = 10;
inline constexpr int16_t STILE_COL_VAL_START    = 80;
inline constexpr int16_t STILE_COL_VAL_SPACE    = 100;

// Extended SafeScreen rows — added below existing telemetry (Y: 230+)
inline constexpr int16_t STILE_STEER_VIS_Y      = 230;  // Steering visual indicator row
inline constexpr int16_t STILE_OBSTACLE_Y        = 246;  // Obstacle sensor bar row
inline constexpr int16_t STILE_LED_STATUS_Y      = 266;  // LED system status row
inline constexpr int16_t STILE_GEAR_BAR_Y        = 288;  // Gear bar (bottom, matching DriveScreen style)
inline constexpr int16_t STILE_RELAY_X           = 430;  // Relay indicator (right side of gear bar)

// Obstacle bar in SafeScreen (simplified, compact)
inline constexpr int16_t STILE_OBS_BAR_X         = 80;
inline constexpr int16_t STILE_OBS_BAR_W         = 200;
inline constexpr int16_t STILE_OBS_BAR_H         = 10;

// Text padding for new SafeScreen tiles
inline constexpr int16_t PAD_SAFE_STEER_VIS      = 200;
inline constexpr int16_t PAD_SAFE_OBSTACLE_TEXT   = 120;
inline constexpr int16_t PAD_SAFE_LED_STATUS      = 300;

// I2C bus diagnostic block (SafeScreen, top-right column) — report-only.
// Lets the operator tell a missing TCA9548A mux apart from a dead INA226.
// Placed to the right of the FAULT/ERROR labels (which are narrowed so they
// no longer clear the full screen width).
inline constexpr int16_t STILE_I2C_X             = 244;  // Left edge of the I2C diag column
inline constexpr int16_t STILE_I2C_TITLE_Y       = STILE_FAULT_LABEL_Y;  // 62
inline constexpr int16_t STILE_I2C_MUX_Y         = STILE_FAULT_VALUE_Y;  // 74
inline constexpr int16_t STILE_I2C_INA_Y         = STILE_ERR_LABEL_Y;    // 90
inline constexpr int16_t STILE_I2C_CNT_Y         = STILE_ERR_VALUE_Y;    // 102
inline constexpr int16_t STILE_I2C_CH_SPACING    = 39;   // Horizontal step between channel cells
inline constexpr int16_t PAD_SAFE_DIAG_NARROW    = 150;  // Narrowed FAULT/ERROR value padding
inline constexpr int16_t PAD_SAFE_I2C_MUX        = 230;  // Clear width for the MUX status line
inline constexpr int16_t PAD_SAFE_I2C_CH         = 36;   // Clear width per channel cell
inline constexpr int16_t PAD_SAFE_I2C_CNT        = 230;  // Clear width for the counters line

// -------------------------------------------------------------------------
// Main battery indicator (Safe Mode) — middle diagnostic column between the
// FAULT/ERROR column (left, x=10) and the I2C BUS DIAG column (right, x=244).
// Lives in the x=165..241 gap so it never overlaps the I2C tile.  Voltage is
// sourced from CAN 0x207 (STATUS_BATTERY); INA-BAT health from CAN 0x309.
// -------------------------------------------------------------------------
inline constexpr int16_t STILE_BAT_X       = 165;
inline constexpr int16_t STILE_BAT_TITLE_Y = STILE_FAULT_LABEL_Y;  // 62
inline constexpr int16_t STILE_BAT_VOLT_Y  = STILE_FAULT_VALUE_Y;  // 74
inline constexpr int16_t STILE_BAT_STAT_Y  = STILE_ERR_LABEL_Y;    // 90
inline constexpr int16_t STILE_BAT_SAFE_Y  = STILE_ERR_VALUE_Y;    // 102
inline constexpr int16_t PAD_SAFE_BAT      = 76;  // Clear width (165..241), left of I2C col

// Main-battery freshness window: 0x207 arrives every 100 ms; treat >2 s old as
// stale (same threshold used for the I2C 0x309 diagnostic).
inline constexpr unsigned long BAT_DIAG_STALE_MS = 2000;

// Battery voltage thresholds (0.01 V units) mirroring the STM32 firmware limits
// reported via SafetyError codes (BATTERY_UV_WARN/CRIT, BATTERY_OV_WARN/CRIT).
inline constexpr uint16_t BATT_UV_WARN_RAW = 2000;  // < 20.0 V → LOW  (amber)
inline constexpr uint16_t BATT_UV_CRIT_RAW = 1800;  // < 18.0 V → CRITICAL (red)
inline constexpr uint16_t BATT_OV_WARN_RAW = 3000;  // > 30.0 V → HIGH (amber)
inline constexpr uint16_t BATT_OV_CRIT_RAW = 3500;  // > 35.0 V → CRITICAL (red)

// INA226 channel bit (in CAN 0x309 inaOkMask) for the always-powered main
// battery shunt (FL,FR,RL,RR,BAT,STEER = bits 0..5).
inline constexpr uint8_t INA_BAT_BIT = 4;

// =========================================================================
// BootScreen Diagnostic Layout Dimensions
//
// Diagnostic panel below obstacle indicator on boot screen.
// Rendered as a single tile (BTILE_DIAGNOSTICS) with multi-line text.
// =========================================================================

inline constexpr int16_t BTILE_DIAG_SEP_Y       = 264;  // SCREEN_H/2 + 104
inline constexpr int16_t BTILE_DIAG_LINE_H       = 10;
inline constexpr int16_t BTILE_DIAG_MARGIN_X     = 10;
inline constexpr unsigned long BTILE_DIAG_RX_RECENT_MS = 2000;
inline constexpr unsigned long BTILE_DIAG_FREEZE_MS    = 1000;
inline constexpr unsigned long BTILE_DIAG_UPDATE_MS    = 125;  // 8 Hz

// Boot splash premium emblem geometry (Mercedes-style, vector only)
inline constexpr int16_t BOOT_EMBLEM_CENTER_Y    = 145;
inline constexpr int16_t BOOT_EMBLEM_OFFSET_X    = 152;
inline constexpr int16_t BOOT_EMBLEM_RADIUS      = 38;

// =========================================================================
// Hash Failsafe — Periodic Forced Redraw Interval (frames)
//
// Critical tiles (SPEED, FAULT, WARNING) are force-redrawn every N frames
// as a safety net against hash collisions or missed invalidation.
// At the nominal 20 FPS target frame rate, 100 frames ≈ 5 seconds.
// Under load the actual interval may be longer (frame rate < 20 FPS).
// =========================================================================

inline constexpr uint16_t HASH_FAILSAFE_INTERVAL = 100;

// =========================================================================
// PinScreen Layout Dimensions
//
// Numeric keypad for engineering access. Layout on 480×320 landscape:
//   3×4 grid of 90×50 buttons, 4 PIN dots (28×28), title/subtitle above.
// =========================================================================

// Keypad button dimensions
inline constexpr int16_t PSCR_BTN_W      = 90;
inline constexpr int16_t PSCR_BTN_H      = 50;
inline constexpr int16_t PSCR_BTN_GAP    = 15;   // horizontal gap between columns
inline constexpr int16_t PSCR_BTN_VGAP   = 10;   // vertical gap between rows

// PIN dot display
inline constexpr int16_t PSCR_DOT_W      = 28;
inline constexpr int16_t PSCR_DOT_H      = 28;
inline constexpr int16_t PSCR_DOT_GAP    = 12;
inline constexpr int16_t PSCR_DOT_Y      = 38;

// Wrong-code message timeout (ms)
inline constexpr uint32_t PSCR_WRONG_CODE_DISPLAY_MS = 1000;

// =========================================================================
// Tank Turn Confirmation Bar Layout
//
// Horizontal bar drawn at the center of the screen when user taps 360° icon.
// Contains question text + SÍ / NO buttons.
// =========================================================================

inline constexpr int16_t TANK_CONFIRM_BAR_X  = 60;
inline constexpr int16_t TANK_CONFIRM_BAR_Y  = 130;
inline constexpr int16_t TANK_CONFIRM_BAR_W  = 360;
inline constexpr int16_t TANK_CONFIRM_BAR_H  = 60;

// SÍ button (left side of bar)
inline constexpr int16_t TANK_CONFIRM_YES_X  = 80;
inline constexpr int16_t TANK_CONFIRM_YES_Y  = 155;
inline constexpr int16_t TANK_CONFIRM_YES_W  = 120;
inline constexpr int16_t TANK_CONFIRM_YES_H  = 28;

// NO button (right side of bar)
inline constexpr int16_t TANK_CONFIRM_NO_X   = 280;
inline constexpr int16_t TANK_CONFIRM_NO_Y   = 155;
inline constexpr int16_t TANK_CONFIRM_NO_W   = 120;
inline constexpr int16_t TANK_CONFIRM_NO_H   = 28;

} // namespace cfg
} // namespace ui

#endif // UI_CONFIG_H
