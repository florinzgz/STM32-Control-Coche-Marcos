// =============================================================================
// ESP32-S3 HMI — UI Common Definitions
//
// Layout constants, color palette, and static helpers for all UI elements.
// All values are compile-time constants. No dynamic allocation.
//
// Target display: 480×320 TFT (landscape orientation, rotation 1)
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef UI_COMMON_H
#define UI_COMMON_H

#include <cstdint>

namespace ui {

// -------------------------------------------------------------------------
// Screen dimensions (ST7796 in landscape mode — rotation 1)
// -------------------------------------------------------------------------
inline constexpr int16_t SCREEN_W = 480;
inline constexpr int16_t SCREEN_H = 320;

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
// Layout zones (Y coordinates, 480×320 landscape)
// -------------------------------------------------------------------------

// Top bar: mode icons (4x4, 4x2, 360°) + battery  (Y: 0–40)
inline constexpr int16_t TOP_BAR_Y      = 0;
inline constexpr int16_t TOP_BAR_H      = 40;

// Obstacle sensor zone (frontal)                   (Y: 40–85)
inline constexpr int16_t SENSOR_Y       = 40;
inline constexpr int16_t SENSOR_H       = 45;

// Car rendering area (wheels + body)                (Y: 85–230)
inline constexpr int16_t CAR_AREA_Y     = 85;
inline constexpr int16_t CAR_AREA_H     = 145;

// Speed display (large, centered)                   (Y: 230–270)
inline constexpr int16_t SPEED_Y        = 232;
inline constexpr int16_t SPEED_H        = 38;

// Pedal bar                                         (Y: 270–300)
inline constexpr int16_t PEDAL_Y        = 272;
inline constexpr int16_t PEDAL_H        = 28;

// Gear display                                      (Y: 300–320)
inline constexpr int16_t GEAR_Y         = 300;
inline constexpr int16_t GEAR_H         = 20;

// -------------------------------------------------------------------------
// Car body geometry (centered horizontally, within CAR_AREA)
// -------------------------------------------------------------------------
inline constexpr int16_t CAR_BODY_X     = 190;
inline constexpr int16_t CAR_BODY_Y     = 105;
inline constexpr int16_t CAR_BODY_W     = 100;
inline constexpr int16_t CAR_BODY_H     = 110;

// Wheel positions relative to screen
inline constexpr int16_t WHEEL_W        = 44;
inline constexpr int16_t WHEEL_H        = 44;

// Front left
inline constexpr int16_t WHL_FL_X       = 130;
inline constexpr int16_t WHL_FL_Y       = 95;
// Front right
inline constexpr int16_t WHL_FR_X       = 306;
inline constexpr int16_t WHL_FR_Y       = 95;
// Rear left
inline constexpr int16_t WHL_RL_X       = 130;
inline constexpr int16_t WHL_RL_Y       = 180;
// Rear right
inline constexpr int16_t WHL_RR_X       = 306;
inline constexpr int16_t WHL_RR_Y       = 180;

// Steering indicator (circular gauge, right side of car area)
inline constexpr int16_t STEER_CX       = 410;
inline constexpr int16_t STEER_CY       = 160;
inline constexpr int16_t STEER_RADIUS   = 24;

// -------------------------------------------------------------------------
// Obstacle sensor (frontal) layout — centered in 480px width
// -------------------------------------------------------------------------
inline constexpr int16_t SENSOR_BAR_X   = 140;
inline constexpr int16_t SENSOR_BAR_W   = 200;
inline constexpr int16_t SENSOR_BAR_H   = 12;

// -------------------------------------------------------------------------
// Battery indicator position (top-right, within top bar)
// -------------------------------------------------------------------------
inline constexpr int16_t BAT_X          = 405;
inline constexpr int16_t BAT_Y          = 6;
inline constexpr int16_t BAT_W          = 65;
inline constexpr int16_t BAT_H          = 28;

// -------------------------------------------------------------------------
// Mode icons position (top-left area, within top bar 0–40px)
// -------------------------------------------------------------------------
inline constexpr int16_t ICON_Y         = 6;
inline constexpr int16_t ICON_W         = 50;
inline constexpr int16_t ICON_H         = 28;
inline constexpr int16_t ICON_SPACING   = 8;
inline constexpr int16_t ICON_4X4_X     = 10;
inline constexpr int16_t ICON_4X2_X     = 68;
inline constexpr int16_t ICON_360_X     = 126;

// -------------------------------------------------------------------------
// Gear display layout (Y: 300–320, evenly spaced across 480px)
// -------------------------------------------------------------------------
inline constexpr int16_t GEAR_LABEL_W   = 52;
inline constexpr int16_t GEAR_LABEL_H   = 18;
inline constexpr int16_t GEAR_START_X   = 50;
inline constexpr int16_t GEAR_SPACING   = 80;

// -------------------------------------------------------------------------
// Pedal bar layout (Y: 270–300)
// -------------------------------------------------------------------------
inline constexpr int16_t PEDAL_BAR_X    = 10;
inline constexpr int16_t PEDAL_BAR_W    = 380;
inline constexpr int16_t PEDAL_BAR_H    = 16;
inline constexpr int16_t PEDAL_TEXT_X   = 400;

// -------------------------------------------------------------------------
// Format buffer sizes (all on stack, no heap)
// -------------------------------------------------------------------------
inline constexpr int FMT_BUF_SMALL  = 16;   // "100%"
inline constexpr int FMT_BUF_MED    = 24;   // "25.5 km/h"
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
