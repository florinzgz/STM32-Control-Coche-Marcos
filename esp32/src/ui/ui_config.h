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

// =========================================================================
// Hash Failsafe — Periodic Forced Redraw Interval (frames)
//
// Critical tiles (SPEED, FAULT, WARNING) are force-redrawn every N frames
// as a safety net against hash collisions or missed invalidation.
// At 20 FPS, 100 frames = 5 seconds.
// =========================================================================

inline constexpr uint16_t HASH_FAILSAFE_INTERVAL = 100;

} // namespace cfg
} // namespace ui

#endif // UI_CONFIG_H
