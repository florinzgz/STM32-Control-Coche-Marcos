// =============================================================================
// ESP32-S3 HMI — Mode Icons Implementation
// =============================================================================

#include "mode_icons.h"

namespace ui {

// -------------------------------------------------------------------------
// Draw all icons in default (inactive) state
// -------------------------------------------------------------------------
void ModeIcons::drawStatic(TFT_eSPI& tft) {
    drawIcon(tft, ICON_4X4_X, "4x4", false);
    drawIcon(tft, ICON_4X2_X, "4x2", false);
    drawIcon(tft, ICON_360_X, "360", false);
}

// -------------------------------------------------------------------------
// Update icon highlights
// -------------------------------------------------------------------------
void ModeIcons::draw(TFT_eSPI& tft,
                     const ModeState& current,
                     const ModeState& previous) {
    // 4x4 icon
    if (current.is4x4 != previous.is4x4) {
        drawIcon(tft, ICON_4X4_X, "4x4", current.is4x4);
    }

    // 4x2 icon (inverse of 4x4)
    bool cur4x2 = !current.is4x4;
    bool prev4x2 = !previous.is4x4;
    if (cur4x2 != prev4x2) {
        drawIcon(tft, ICON_4X2_X, "4x2", cur4x2);
    }

    // 360° icon
    if (current.isTankTurn != previous.isTankTurn) {
        drawIcon(tft, ICON_360_X, "360", current.isTankTurn);
    }
}

// -------------------------------------------------------------------------
// Hit test for touch input
// -------------------------------------------------------------------------
uint8_t ModeIcons::hitTest(int16_t touchX, int16_t touchY) {
    if (touchY < ICON_Y || touchY > ICON_Y + ICON_H) return 0;

    if (touchX >= ICON_4X4_X && touchX <= ICON_4X4_X + ICON_W) return 1;
    if (touchX >= ICON_4X2_X && touchX <= ICON_4X2_X + ICON_W) return 2;
    if (touchX >= ICON_360_X && touchX <= ICON_360_X + ICON_W) return 3;

    return 0;
}

// -------------------------------------------------------------------------
// Draw a single icon
// -------------------------------------------------------------------------
void ModeIcons::drawIcon(TFT_eSPI& tft, int16_t x, const char* label,
                         bool active) {
    uint16_t bgCol   = active ? COL_CYAN : COL_BG;
    uint16_t txtCol  = active ? COL_BLACK : COL_GRAY;
    uint16_t bordCol = active ? COL_WHITE : COL_GRAY;

    tft.fillRect(x, ICON_Y, ICON_W, ICON_H, bgCol);
    tft.drawRect(x, ICON_Y, ICON_W, ICON_H, bordCol);

    tft.setTextColor(txtCol, bgCol);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(label, x + ICON_W / 2, ICON_Y + ICON_H / 2);
    tft.setTextDatum(TL_DATUM);
}

} // namespace ui
