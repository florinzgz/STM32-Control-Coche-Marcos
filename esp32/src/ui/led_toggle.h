// =============================================================================
// ESP32-S3 HMI — Light Control Buttons
//
// Two independent touch-responsive toggle buttons for LED relay strips:
//   Front lights: headlight / low-beam icon (left button)
//   Rear  lights: brake/tail light icon    (right button)
//
// Each button sends CAN CMD_LED (0x120) byte0=front, byte1=rear on toggle.
// State reflects STM32 STATUS_LIGHTS (0x20A) byte0/byte1.
//
// Reference: FIRMWARE_MIGRATION_AUDIT.md Step 6
// =============================================================================

#ifndef LED_TOGGLE_H
#define LED_TOGGLE_H

#include <TFT_eSPI.h>
#include "ui_common.h"

namespace ui {

class LedToggle {
public:
    /// Draw both light buttons in their default (OFF) state.
    static void drawStatic(TFT_eSPI& tft);

    /// Update button highlights based on relay states.
    /// Only redraws when state changes.
    static void draw(TFT_eSPI& tft,
                     bool curFront, bool prevFront,
                     bool curRear,  bool prevRear);

    /// Force a full repaint of both buttons at the given state, regardless of
    /// any previous value.  Used to restore the buttons after an overlay (e.g.
    /// the ACK pill) painted over them, where the underlying relay state has
    /// not changed and the differential draw() would otherwise skip the redraw.
    static void redraw(TFT_eSPI& tft, bool front, bool rear);

    /// Check if a touch point hits the front light button.
    static bool hitTestFront(int16_t touchX, int16_t touchY);

    /// Check if a touch point hits the rear light button.
    static bool hitTestRear(int16_t touchX, int16_t touchY);

private:
    static void drawFrontButton(TFT_eSPI& tft, bool active);
    static void drawRearButton(TFT_eSPI& tft, bool active);
};

// ---- Front light button (low-beam icon) — just right of mode icons ----
inline constexpr int16_t LED_FRONT_X  = 185;
inline constexpr int16_t LED_FRONT_Y  = ICON_Y;
inline constexpr int16_t LED_FRONT_W  = 46;
inline constexpr int16_t LED_FRONT_H  = ICON_H;

// ---- Rear light button (brake-light icon) — next to front button ----
inline constexpr int16_t LED_REAR_X   = 237;
inline constexpr int16_t LED_REAR_Y   = ICON_Y;
inline constexpr int16_t LED_REAR_W   = 46;
inline constexpr int16_t LED_REAR_H   = ICON_H;

// Keep legacy alias so existing hitTest callers compile (maps to front)
inline constexpr int16_t LED_ICON_X  = LED_FRONT_X;
inline constexpr int16_t LED_ICON_Y  = LED_FRONT_Y;
inline constexpr int16_t LED_ICON_W  = LED_FRONT_W;
inline constexpr int16_t LED_ICON_H  = LED_FRONT_H;

} // namespace ui

#endif // LED_TOGGLE_H
