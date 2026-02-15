// =============================================================================
// ESP32-S3 HMI — Pedal Bar Implementation
// =============================================================================

#include "pedal_bar.h"
#include <cstdio>

namespace ui {

// -------------------------------------------------------------------------
// Static outline
// -------------------------------------------------------------------------
void PedalBar::drawStatic(TFT_eSPI& tft) {
    // Label
    tft.setTextColor(COL_WHITE, COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("PEDAL", PEDAL_BAR_X, PEDAL_Y);

    // Bar outline
    int16_t barY = PEDAL_Y + 12;
    tft.drawRect(PEDAL_BAR_X, barY, PEDAL_BAR_W, PEDAL_BAR_H, COL_WHITE);
}

// -------------------------------------------------------------------------
// Update pedal bar fill and text
// -------------------------------------------------------------------------
void PedalBar::draw(TFT_eSPI& tft, uint8_t pedalPct, uint8_t prevPct) {
    if (pedalPct == prevPct) return;

    int16_t barY = PEDAL_Y + 12;

    // Clamp to 100
    uint8_t pct = (pedalPct > 100) ? 100 : pedalPct;

    // Calculate fill width
    int16_t fillW = static_cast<int16_t>(
        (static_cast<int32_t>(pct) * (PEDAL_BAR_W - 4)) / 100);

    // Choose color based on percentage
    uint16_t fillCol;
    if (pct <= 40) {
        fillCol = COL_GREEN;
    } else if (pct <= 70) {
        fillCol = COL_YELLOW;
    } else {
        fillCol = COL_RED;
    }

    // Clear entire bar interior
    tft.fillRect(PEDAL_BAR_X + 2, barY + 2,
                 PEDAL_BAR_W - 4, PEDAL_BAR_H - 4, COL_BG);

    // Draw filled portion
    if (fillW > 0) {
        tft.fillRect(PEDAL_BAR_X + 2, barY + 2,
                     fillW, PEDAL_BAR_H - 4, fillCol);
    }

    // Percentage text (right of bar)
    char buf[FMT_BUF_SMALL];
    snprintf(buf, sizeof(buf), "%3u%%", pct);
    tft.setTextColor(COL_WHITE, COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(ML_DATUM);
    // Clear old text area
    tft.fillRect(PEDAL_TEXT_X, barY, 70, PEDAL_BAR_H, COL_BG);
    tft.drawString(buf, PEDAL_TEXT_X, barY + PEDAL_BAR_H / 2);

    tft.setTextDatum(TL_DATUM);  // Reset
    tft.setTextSize(1);
}

} // namespace ui
