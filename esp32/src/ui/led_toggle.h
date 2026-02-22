// =============================================================================
// ESP32-S3 HMI — LED Toggle Button
//
// Touch-responsive toggle for WS2812B LED strip (power relay on STM32).
// Drawn in the top bar area, between mode icons and battery indicator.
// Sends CAN CMD_LED (0x120) on toggle, reflects STM32 STATUS_LIGHTS (0x20A).
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
    /// Draw the toggle button (call once on screen enter)
    static void drawStatic(TFT_eSPI& tft);

    /// Update highlight based on LED relay state.
    /// Only redraws when state changes.
    static void draw(TFT_eSPI& tft, bool currentOn, bool previousOn);

    /// Check if a touch point hits the LED toggle.
    /// Returns true if the toggle was tapped.
    static bool hitTest(int16_t touchX, int16_t touchY);

private:
    static void drawButton(TFT_eSPI& tft, bool active);
};

// LED toggle position — right of mode icons, left of battery
inline constexpr int16_t LED_ICON_X  = 200;
inline constexpr int16_t LED_ICON_Y  = ICON_Y;
inline constexpr int16_t LED_ICON_W  = 50;
inline constexpr int16_t LED_ICON_H  = ICON_H;

} // namespace ui

#endif // LED_TOGGLE_H
