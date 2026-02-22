// =============================================================================
// ESP32-S3 HMI — LED Toggle Button Implementation
// =============================================================================

#include "led_toggle.h"

namespace ui {

// -------------------------------------------------------------------------
// Draw the toggle in default (OFF) state
// -------------------------------------------------------------------------
void LedToggle::drawStatic(TFT_eSPI& tft) {
    drawButton(tft, false);
}

// -------------------------------------------------------------------------
// Update highlight when state changes
// -------------------------------------------------------------------------
void LedToggle::draw(TFT_eSPI& tft, bool currentOn, bool previousOn) {
    if (currentOn != previousOn) {
        drawButton(tft, currentOn);
    }
}

// -------------------------------------------------------------------------
// Hit test for touch input
// -------------------------------------------------------------------------
bool LedToggle::hitTest(int16_t touchX, int16_t touchY) {
    return (touchX >= LED_ICON_X &&
            touchX <= LED_ICON_X + LED_ICON_W &&
            touchY >= LED_ICON_Y &&
            touchY <= LED_ICON_Y + LED_ICON_H);
}

// -------------------------------------------------------------------------
// Draw the button
// -------------------------------------------------------------------------
void LedToggle::drawButton(TFT_eSPI& tft, bool active) {
    uint16_t bgCol   = active ? COL_YELLOW : COL_BG;
    uint16_t txtCol  = active ? COL_BLACK  : COL_GRAY;
    uint16_t bordCol = active ? COL_WHITE  : COL_GRAY;

    tft.fillRect(LED_ICON_X, LED_ICON_Y, LED_ICON_W, LED_ICON_H, bgCol);
    tft.drawRect(LED_ICON_X, LED_ICON_Y, LED_ICON_W, LED_ICON_H, bordCol);

    tft.setTextColor(txtCol, bgCol);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("LED", LED_ICON_X + LED_ICON_W / 2,
                          LED_ICON_Y + LED_ICON_H / 2);
    tft.setTextDatum(TL_DATUM);
}

} // namespace ui
