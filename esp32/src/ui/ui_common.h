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
inline constexpr uint16_t COL_HEADLIGHT   = 0xFE60;  // Warm orange-yellow for headlight housing
inline constexpr uint16_t COL_BLUE        = 0x041F;  // Cold (low temperature) blue

// Premium top-down vehicle shading palette (RGB565).
// Layered greys fake the cylindrical volume of a tyre; the body bands fake
// a metallic gradient.  Pure presentation — no extra RAM, no sprites.
inline constexpr uint16_t COL_TYRE_DARK   = 0x18C3;  // Tyre core / contact shadow
inline constexpr uint16_t COL_TYRE_MID    = 0x39E7;  // Tyre sidewall mid tone
inline constexpr uint16_t COL_TYRE_LIGHT  = 0x52AA;  // Tyre highlight band
inline constexpr uint16_t COL_RIM         = 0x9CD3;  // Alloy rim
inline constexpr uint16_t COL_BODY_DARK   = 0x10A4;  // Vehicle body lower band (shadow)
inline constexpr uint16_t COL_BODY_MID    = 0x2945;  // Vehicle body mid band
inline constexpr uint16_t COL_BODY_LIGHT  = 0x4A8B;  // Vehicle body upper band (sheen)
inline constexpr uint16_t COL_BODY_EDGE   = 0x6B6D;  // Body top reflective edge

// Premium OEM cluster accent palette (RGB565) — used by the analog dials,
// the gear selector and the AMG accent bar.  Pure presentation, no extra RAM.
inline constexpr uint16_t COL_DIAL_FACE   = 0x18E3;  // Dial interior (slightly darker than BG)
inline constexpr uint16_t COL_DIAL_TICK   = 0x6B4D;  // Inactive tick / bezel marks
inline constexpr uint16_t COL_DIAL_RING   = 0x4A69;  // Bezel ring
inline constexpr uint16_t COL_NEEDLE      = 0xFD20;  // Needle (warm amber/orange)
inline constexpr uint16_t COL_NEEDLE_RPM  = 0x07FF;  // RPM needle (cyan)
inline constexpr uint16_t COL_AMG_RED     = 0xC000;  // AMG accent red (deep)
inline constexpr uint16_t COL_AMG_SILVER  = 0xC618;  // AMG silver lettering
inline constexpr uint16_t COL_GEAR_OFF    = 0x10A2;  // Inactive gear cell fill
inline constexpr uint16_t COL_GEAR_EDGE   = 0x4A69;  // Inactive gear cell border
inline constexpr uint16_t COL_ACCENT      = 0x05BF;  // Cool cyan accent (separators)

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
// Premium OEM instrument cluster — bottom zone (Y: 232–320)
//
// Layout (keeps the central vehicle/wheels/steering arc above, Y 85–230):
//   * Analog SPEED dial with needle   — bottom-left  corner
//   * Analog RPM dial with needle     — bottom-right corner
//   * Gear selector P/R/N/D1/D2       — centred between the dials
//   * Throttle micro-bar              — centred, below the gears
//   * AMG accent bar                  — centred, bottom
// All geometry uses only TFT_eSPI primitives (no sprites, no heap).
// -------------------------------------------------------------------------

// Analog dials (270° sweep, gap at the bottom; needle pivots at centre).
inline constexpr int16_t DIAL_R          = 40;   // outer radius
inline constexpr int16_t SPEED_DIAL_CX   = 58;
inline constexpr int16_t SPEED_DIAL_CY   = 278;
inline constexpr int16_t RPM_DIAL_CX     = 422;
inline constexpr int16_t RPM_DIAL_CY     = 278;

// Sweep angles, degrees, 0 = up, clockwise positive.  225°→495°(=135°) is a
// 270° arc with a 90° gap centred at the bottom (180°).
inline constexpr int16_t DIAL_START_DEG  = 225;
inline constexpr int16_t DIAL_SWEEP_DEG  = 270;

// Display scaling for the dials.
inline constexpr uint16_t SPEED_DIAL_MAX = 400;  // 40.0 km/h full-scale (raw 0.1 km/h → /10)

// Centre cluster (between the two dials).
inline constexpr int16_t CLUSTER_X       = 104;
inline constexpr int16_t CLUSTER_W       = 272;  // 104..376

// Gear selector — 5 compact pills + relay status letters on the right.
inline constexpr int16_t DGEAR_Y         = 240;
inline constexpr int16_t DGEAR_H         = 26;
inline constexpr int16_t DGEAR_CELL_W    = 40;
inline constexpr int16_t DGEAR_CELL_GAP  = 5;
inline constexpr int16_t DGEAR_START_X   = 110;  // first cell left edge

// Throttle micro-bar (below the gears).
inline constexpr int16_t DTHR_X          = 110;
inline constexpr int16_t DTHR_Y          = 274;
inline constexpr int16_t DTHR_W          = 220;
inline constexpr int16_t DTHR_H          = 8;

// AMG accent bar (bottom of the centre cluster).
inline constexpr int16_t AMG_X           = 104;
inline constexpr int16_t AMG_Y           = 292;
inline constexpr int16_t AMG_W           = 272;
inline constexpr int16_t AMG_H           = 24;

// CAN link status indicator (top-bar gap between LED toggles and battery).
inline constexpr int16_t CAN_IND_X       = 312;
inline constexpr int16_t CAN_IND_Y       = 8;
inline constexpr int16_t CAN_IND_W       = 86;
inline constexpr int16_t CAN_IND_H       = 24;

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
// Helper: motor / sensor temperature to a 5-level "smart" color.
//   < 25 °C  → BLUE   (cold)
//   25–49    → GREEN  (normal)
//   50–64    → YELLOW (warm)
//   65–79    → ORANGE (hot)
//   ≥ 80     → RED    (critical)
// Applied to both motor and ambient temperatures so the whole dash speaks
// the same color language.
// -------------------------------------------------------------------------
inline uint16_t tempColorFull(int8_t tempC) {
    if (tempC < 25)  return COL_BLUE;
    if (tempC < 50)  return COL_GREEN;
    if (tempC < 65)  return COL_YELLOW;
    if (tempC < 80)  return COL_ORANGE;
    return COL_RED;
}

// -------------------------------------------------------------------------
// Helper: obstacle distance to color (proximity indicator) — 5 zones + gray
// Aligned 1:1 with STM32 safety zones (safety_system.c / obstacle_sensor.cpp):
//   < 50 cm  (< 500 mm)   → RED    — EMERGENCY (scale 0.0, forward blocked)
//   50–100   (500–1000)   → ORANGE — CRITICAL  (scale 0.3)
//   100–150  (1000–1500)  → YELLOW — WARNING   (scale 0.7)
//   150–200  (1500–2000)  → CYAN   — CAUTION   (scale 0.85)
//   ≥ 200    (≥ 2000)     → GREEN  — ALERT / normal
// -------------------------------------------------------------------------
inline uint16_t proximityColor(uint16_t distanceCm) {
    if (distanceCm == 0)   return COL_GRAY;    // no reading
    if (distanceCm < 50)   return COL_RED;     // < 0.5 m: emergency
    if (distanceCm < 100)  return COL_ORANGE;  // 0.5–1.0 m: critical
    if (distanceCm < 150)  return COL_YELLOW;  // 1.0–1.5 m: warning
    if (distanceCm < 200)  return COL_CYAN;    // 1.5–2.0 m: caution
    return COL_GREEN;                          // ≥ 2.0 m: alert / safe
}

} // namespace ui

#endif // UI_COMMON_H
