// =============================================================================
// ESP32-S3 HMI — UI Common Definitions
//
// Layout constants, color palette, and static helpers for all UI elements.
// All values are compile-time constants. No dynamic allocation.
//
// Target display: 320×480 TFT (portrait orientation)
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef UI_COMMON_H
#define UI_COMMON_H

#include <cstdint>

namespace ui {

// -------------------------------------------------------------------------
// Screen dimensions (ST7796 in portrait mode)
// -------------------------------------------------------------------------
inline constexpr int16_t SCREEN_W = 320;
inline constexpr int16_t SCREEN_H = 480;

// -------------------------------------------------------------------------
// Color palette (RGB565)
// -------------------------------------------------------------------------
inline constexpr uint16_t COL_BG          = 0x2104;  // Dark gray background
inline constexpr uint16_t COL_WHITE       = 0xFFFF;
inline constexpr uint16_t COL_BLACK       = 0x0000;
inline constexpr uint16_t COL_GREEN       = 0x07E0;
inline constexpr uint16_t COL_YELLOW      = 0xFFE0;
inline constexpr uint16_t COL_RED         = 0xF800;
inline constexpr uint16_t COL_CYAN        = 0x07FF;
inline constexpr uint16_t COL_GRAY        = 0x8410;
inline constexpr uint16_t COL_DARK_GRAY   = 0x4208;
inline constexpr uint16_t COL_ORANGE      = 0xFD20;
inline constexpr uint16_t COL_AMBER       = 0xFBE0;

// -------------------------------------------------------------------------
// Layout zones (Y coordinates)
// -------------------------------------------------------------------------

// Top bar: mode icons (4x4, 4x2, 360°) + battery  (0–50px)
inline constexpr int16_t TOP_BAR_Y      = 0;
inline constexpr int16_t TOP_BAR_H      = 50;

// Obstacle sensor zone (frontal)                   (52–90px)
inline constexpr int16_t SENSOR_Y       = 52;
inline constexpr int16_t SENSOR_H       = 38;

// Car rendering area (wheels + body)                (90–270px)
inline constexpr int16_t CAR_AREA_Y     = 90;
inline constexpr int16_t CAR_AREA_H     = 180;

// Speed display (large, centered)                   (280–330px)
inline constexpr int16_t SPEED_Y        = 285;
inline constexpr int16_t SPEED_H        = 40;

// Pedal bar                                         (330–400px)
inline constexpr int16_t PEDAL_Y        = 335;
inline constexpr int16_t PEDAL_H        = 58;

// Gear display                                      (400–450px)
inline constexpr int16_t GEAR_Y         = 405;
inline constexpr int16_t GEAR_H         = 30;

// Steering direction display                        (450–480px)
inline constexpr int16_t STEER_TEXT_Y   = 450;
inline constexpr int16_t STEER_TEXT_H   = 28;

// -------------------------------------------------------------------------
// Car body geometry (centered on screen, within CAR_AREA)
// -------------------------------------------------------------------------
inline constexpr int16_t CAR_BODY_X     = 110;
inline constexpr int16_t CAR_BODY_Y     = 130;
inline constexpr int16_t CAR_BODY_W     = 100;
inline constexpr int16_t CAR_BODY_H     = 110;

// Wheel positions relative to screen
inline constexpr int16_t WHEEL_W        = 44;
inline constexpr int16_t WHEEL_H        = 50;

// Front left
inline constexpr int16_t WHL_FL_X       = 46;
inline constexpr int16_t WHL_FL_Y       = 100;
// Front right
inline constexpr int16_t WHL_FR_X       = 230;
inline constexpr int16_t WHL_FR_Y       = 100;
// Rear left
inline constexpr int16_t WHL_RL_X       = 46;
inline constexpr int16_t WHL_RL_Y       = 210;
// Rear right
inline constexpr int16_t WHL_RR_X       = 230;
inline constexpr int16_t WHL_RR_Y       = 210;

// Steering indicator (circular gauge, right side of car area)
inline constexpr int16_t STEER_CX       = 290;
inline constexpr int16_t STEER_CY       = 180;
inline constexpr int16_t STEER_RADIUS   = 22;

// -------------------------------------------------------------------------
// Obstacle sensor (frontal) layout
// -------------------------------------------------------------------------
inline constexpr int16_t SENSOR_BAR_X   = 80;
inline constexpr int16_t SENSOR_BAR_W   = 160;
inline constexpr int16_t SENSOR_BAR_H   = 12;

// -------------------------------------------------------------------------
// Battery indicator position (top-right, within top bar)
// -------------------------------------------------------------------------
inline constexpr int16_t BAT_X          = 250;
inline constexpr int16_t BAT_Y          = 10;
inline constexpr int16_t BAT_W          = 65;
inline constexpr int16_t BAT_H          = 28;

// -------------------------------------------------------------------------
// Mode icons position (top-left area, within top bar 0–50px)
// -------------------------------------------------------------------------
inline constexpr int16_t ICON_Y         = 10;
inline constexpr int16_t ICON_W         = 50;
inline constexpr int16_t ICON_H         = 28;
inline constexpr int16_t ICON_SPACING   = 8;
inline constexpr int16_t ICON_4X4_X     = 10;
inline constexpr int16_t ICON_4X2_X     = 68;
inline constexpr int16_t ICON_360_X     = 126;

// -------------------------------------------------------------------------
// Gear display layout (400–450px zone, evenly spaced)
// -------------------------------------------------------------------------
inline constexpr int16_t GEAR_LABEL_W   = 48;
inline constexpr int16_t GEAR_LABEL_H   = 24;
inline constexpr int16_t GEAR_START_X   = 16;
inline constexpr int16_t GEAR_SPACING   = 60;

// -------------------------------------------------------------------------
// Pedal bar layout (330–400px zone)
// -------------------------------------------------------------------------
inline constexpr int16_t PEDAL_BAR_X    = 10;
inline constexpr int16_t PEDAL_BAR_W    = 230;
inline constexpr int16_t PEDAL_BAR_H    = 26;
inline constexpr int16_t PEDAL_TEXT_X   = 248;
inline constexpr int16_t PEDAL_ARROW_X  = 295;

// -------------------------------------------------------------------------
// Format buffer sizes (all on stack, no heap)
// -------------------------------------------------------------------------
inline constexpr int FMT_BUF_SMALL  = 8;    // "100%"
inline constexpr int FMT_BUF_MED    = 16;   // "25.5 km/h"
inline constexpr int FMT_BUF_LARGE  = 32;   // Longer labels

// -------------------------------------------------------------------------
// Helper: torque percentage to color
// -------------------------------------------------------------------------
inline uint16_t torqueColor(uint8_t pct) {
    if (pct <= 50)  return COL_GREEN;
    if (pct <= 80)  return COL_YELLOW;
    return COL_RED;
}

// -------------------------------------------------------------------------
// Helper: obstacle distance to color (proximity indicator)
// -------------------------------------------------------------------------
inline uint16_t proximityColor(uint16_t distanceCm) {
    if (distanceCm == 0)   return COL_GRAY;    // no reading
    if (distanceCm > 150)  return COL_GREEN;   // > 1.5 m: safe
    if (distanceCm > 50)   return COL_YELLOW;  // 0.5–1.5 m: caution
    return COL_RED;                             // < 0.5 m: danger
}

} // namespace ui

#endif // UI_COMMON_H
