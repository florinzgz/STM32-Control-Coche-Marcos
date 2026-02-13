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

// Top bar: mode icons + battery
inline constexpr int16_t TOP_BAR_Y      = 0;
inline constexpr int16_t TOP_BAR_H      = 38;

// Speed display
inline constexpr int16_t SPEED_Y        = 40;
inline constexpr int16_t SPEED_H        = 32;

// Car rendering area
inline constexpr int16_t CAR_AREA_Y     = 78;
inline constexpr int16_t CAR_AREA_H     = 295;

// Gear display
inline constexpr int16_t GEAR_Y         = 378;
inline constexpr int16_t GEAR_H         = 30;

// Pedal bar
inline constexpr int16_t PEDAL_Y        = 415;
inline constexpr int16_t PEDAL_H        = 58;

// -------------------------------------------------------------------------
// Car body geometry (centered on screen)
// -------------------------------------------------------------------------
inline constexpr int16_t CAR_BODY_X     = 100;
inline constexpr int16_t CAR_BODY_Y     = 148;
inline constexpr int16_t CAR_BODY_W     = 120;
inline constexpr int16_t CAR_BODY_H     = 160;

// Wheel positions relative to screen
inline constexpr int16_t WHEEL_W        = 44;
inline constexpr int16_t WHEEL_H        = 60;

// Front left
inline constexpr int16_t WHL_FL_X       = 46;
inline constexpr int16_t WHL_FL_Y       = 110;
// Front right
inline constexpr int16_t WHL_FR_X       = 230;
inline constexpr int16_t WHL_FR_Y       = 110;
// Rear left
inline constexpr int16_t WHL_RL_X       = 46;
inline constexpr int16_t WHL_RL_Y       = 280;
// Rear right
inline constexpr int16_t WHL_RR_X       = 230;
inline constexpr int16_t WHL_RR_Y       = 280;

// Steering indicator center
inline constexpr int16_t STEER_CX       = 160;
inline constexpr int16_t STEER_CY       = 228;
inline constexpr int16_t STEER_RADIUS   = 20;

// -------------------------------------------------------------------------
// Battery indicator position (top-right)
// -------------------------------------------------------------------------
inline constexpr int16_t BAT_X          = 250;
inline constexpr int16_t BAT_Y          = 5;
inline constexpr int16_t BAT_W          = 65;
inline constexpr int16_t BAT_H          = 28;

// -------------------------------------------------------------------------
// Mode icons position (top-left area)
// -------------------------------------------------------------------------
inline constexpr int16_t ICON_Y         = 5;
inline constexpr int16_t ICON_W         = 50;
inline constexpr int16_t ICON_H         = 28;
inline constexpr int16_t ICON_SPACING   = 8;
inline constexpr int16_t ICON_4X4_X     = 10;
inline constexpr int16_t ICON_4X2_X     = 68;
inline constexpr int16_t ICON_360_X     = 126;

// -------------------------------------------------------------------------
// Gear display layout
// -------------------------------------------------------------------------
inline constexpr int16_t GEAR_LABEL_W   = 44;
inline constexpr int16_t GEAR_LABEL_H   = 24;
inline constexpr int16_t GEAR_START_X   = 28;
inline constexpr int16_t GEAR_SPACING   = 56;

// -------------------------------------------------------------------------
// Pedal bar layout
// -------------------------------------------------------------------------
inline constexpr int16_t PEDAL_BAR_X    = 10;
inline constexpr int16_t PEDAL_BAR_W    = 230;
inline constexpr int16_t PEDAL_BAR_H    = 30;
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

} // namespace ui

#endif // UI_COMMON_H
