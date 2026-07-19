// =============================================================================
// ESP32-S3 HMI — Centralized UI Configuration
//
// All magic numbers, thresholds, text padding widths, and overlay layout
// constants are defined here. No screen or widget should contain hardcoded
// visual parameters — they must reference constants from this header.
// =============================================================================

#ifndef UI_CONFIG_H
#define UI_CONFIG_H

#include <cstdint>

namespace ui {
namespace cfg {

// =========================================================================
// Text Padding Widths (pixels)
// =========================================================================
inline constexpr int16_t PAD_SPEED           = 120;
inline constexpr int16_t PAD_RPM             = 60;
inline constexpr int16_t PAD_CAN_STATE       = 16;
inline constexpr int16_t PAD_ACK             = 80;
inline constexpr int16_t PAD_WHEEL_LABEL     = 46;
inline constexpr int16_t PAD_PEDAL_TEXT      = 70;
inline constexpr int16_t PAD_OBSTACLE_DIST   = 80;
inline constexpr int16_t PAD_ERROR_HEX       = 80;
inline constexpr int16_t PAD_ERROR_FULL      = 460;
inline constexpr int16_t PAD_SAFE_FULL       = 460;
inline constexpr int16_t PAD_SAFE_WHEEL_VAL  = 78;
inline constexpr int16_t PAD_SAFE_TEMP       = 52;
inline constexpr int16_t PAD_SAFE_STEERING   = 140;
inline constexpr int16_t PAD_STANDBY_TEMP    = 80;
inline constexpr int16_t PAD_STANDBY_FAULTS  = 240;
inline constexpr int16_t PAD_BOOT_CAN_STATUS = 160;
inline constexpr int16_t PAD_BATTERY_TEXT    = 55;

// =========================================================================
// CAN Jitter Filtering Thresholds
// =========================================================================
inline constexpr uint8_t THR_TRACTION_DELTA = 2;
inline constexpr int8_t  THR_TEMP_DELTA     = 1;

// =========================================================================
// Battery Indicator Hysteresis
// =========================================================================
inline constexpr uint8_t BATT_HYSTERESIS_HIGH = 55;
inline constexpr uint8_t BATT_HYSTERESIS_LOW  = 45;

// Installed traction-pack visual range (0.01 V units).
// These values affect only the percentage icon; STM32 protection thresholds
// are configured independently through BATTERY LIMITS.
inline constexpr uint16_t BATT_VOLTAGE_MIN_RAW = 1600;  // 16.00 V = 0 %
inline constexpr uint16_t BATT_VOLTAGE_MAX_RAW = 2850;  // 28.50 V = 100 %

// =========================================================================
// ACK Visual Feedback
// =========================================================================
inline constexpr unsigned long ACK_DISPLAY_DURATION_MS = 1500;
inline constexpr int16_t ACK_X = 200;
inline constexpr int16_t ACK_Y = 2;
inline constexpr int16_t ACK_W = 80;
inline constexpr int16_t ACK_H = 16;

// =========================================================================
// Overlay Layout — Degraded/Limp Banner
// =========================================================================
inline constexpr int16_t OVL_DEGRADED_X = 0;
inline constexpr int16_t OVL_DEGRADED_Y = 40;
inline constexpr int16_t OVL_DEGRADED_W = 480;
inline constexpr int16_t OVL_DEGRADED_H = 18;

// =========================================================================
// Overlay Layout — Fault Indicators
// =========================================================================
inline constexpr int16_t OVL_FAULT_Y = 60;
inline constexpr int16_t OVL_FAULT_H = 10;
inline constexpr int16_t OVL_FAULT_LABEL_GAP = 4;

// =========================================================================
// Temperature Color Thresholds
// =========================================================================
inline constexpr int8_t TEMP_COLOR_WARNING  = 60;
inline constexpr int8_t TEMP_COLOR_CRITICAL = 80;

// =========================================================================
// Pedal Bar Color Thresholds
// =========================================================================
inline constexpr uint8_t PEDAL_COLOR_LOW = 40;
inline constexpr uint8_t PEDAL_COLOR_MID = 70;

// =========================================================================
// Battery Level Color Thresholds
// =========================================================================
inline constexpr uint8_t BATT_COLOR_LOW = 20;
inline constexpr uint8_t BATT_COLOR_MID = 50;

// =========================================================================
// RPM Display
// =========================================================================
inline constexpr uint16_t RPM_DISPLAY_MAX = 400;
inline constexpr int16_t RPM_LABEL_X = 295;

// =========================================================================
// Obstacle Sensor Bar Max Range
// =========================================================================
inline constexpr uint16_t OBSTACLE_BAR_MAX_CM = 600;

// =========================================================================
// DriveScreen Tile Layout Dimensions
// =========================================================================
inline constexpr int16_t DTILE_MODE_ICONS_X = 0;
inline constexpr int16_t DTILE_MODE_ICONS_W = 180;
inline constexpr int16_t DTILE_LED_TOGGLE_X = 180;
inline constexpr int16_t DTILE_LED_TOGGLE_W = 120;
inline constexpr int16_t DTILE_BATTERY_W = 75;
inline constexpr int16_t DTILE_WHEELS_W = 370;
inline constexpr int16_t DTILE_STEERING_X = 370;
inline constexpr int16_t DTILE_STEERING_W = 110;

} // namespace cfg
} // namespace ui

#endif // UI_CONFIG_H
