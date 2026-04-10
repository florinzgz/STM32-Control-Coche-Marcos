// =============================================================================
// ESP32-S3 HMI — Pedal Bar Implementation
// =============================================================================

#include "pedal_bar.h"
#include "render_trace.h"
#include "ui_config.h"
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
    RTRACE_TEXT(PEDAL_BAR_X, PEDAL_Y, "PEDAL", COL_WHITE, COL_BG, 1, TL_DATUM);

    // Bar outline
    int16_t barY = PEDAL_Y + 12;
    tft.drawRect(PEDAL_BAR_X, barY, PEDAL_BAR_W, PEDAL_BAR_H, COL_WHITE);
    RTRACE_DRAW_RECT(PEDAL_BAR_X, barY, PEDAL_BAR_W, PEDAL_BAR_H, COL_WHITE);
}

// -------------------------------------------------------------------------
// Update pedal bar fill and text
//
// Anti-flicker differential update: instead of clearing the entire bar
// interior and redrawing, we only update the changed portion.
// If the bar grew, fill the new portion. If it shrank, clear the old tail.
// This eliminates the visible flash from fillRect→fillRect sequences.
// -------------------------------------------------------------------------
void PedalBar::draw(TFT_eSPI& tft, uint8_t pedalPct, uint8_t prevPct) {
    if (pedalPct == prevPct) return;

    int16_t barY = PEDAL_Y + 12;

    // Clamp to 100
    uint8_t pct     = (pedalPct > 100) ? 100 : pedalPct;
    uint8_t prevCl  = (prevPct > 100)  ? 100 : prevPct;

    // Calculate fill widths
    int16_t innerW  = PEDAL_BAR_W - 4;
    int16_t fillW   = static_cast<int16_t>((static_cast<int32_t>(pct)    * innerW) / 100);
    int16_t prevFW  = static_cast<int16_t>((static_cast<int32_t>(prevCl) * innerW) / 100);

    // Choose color based on new percentage
    uint16_t fillCol;
    if (pct <= cfg::PEDAL_COLOR_LOW) {
        fillCol = COL_GREEN;
    } else if (pct <= cfg::PEDAL_COLOR_MID) {
        fillCol = COL_YELLOW;
    } else {
        fillCol = COL_RED;
    }

    // Determine previous color to check if a color boundary was crossed
    uint16_t prevCol;
    if (prevCl <= cfg::PEDAL_COLOR_LOW) {
        prevCol = COL_GREEN;
    } else if (prevCl <= cfg::PEDAL_COLOR_MID) {
        prevCol = COL_YELLOW;
    } else {
        prevCol = COL_RED;
    }

    if (fillCol != prevCol) {
        // Color threshold crossed — must redraw entire bar to change color
        // Clear the old fill area and draw new one in one pass
        if (prevFW > fillW) {
            // Bar shrank: clear the tail first
            tft.fillRect(PEDAL_BAR_X + 2 + fillW, barY + 2,
                         prevFW - fillW, PEDAL_BAR_H - 4, COL_BG);
        }
        // Redraw entire filled portion in new color
        if (fillW > 0) {
            tft.fillRect(PEDAL_BAR_X + 2, barY + 2,
                         fillW, PEDAL_BAR_H - 4, fillCol);
            RTRACE_FILL_RECT(PEDAL_BAR_X + 2, barY + 2,
                             fillW, PEDAL_BAR_H - 4, fillCol);
        }
    } else if (fillW > prevFW) {
        // Bar grew — fill only the new portion (differential)
        if (prevFW < 0) prevFW = 0;
        tft.fillRect(PEDAL_BAR_X + 2 + prevFW, barY + 2,
                     fillW - prevFW, PEDAL_BAR_H - 4, fillCol);
        RTRACE_FILL_RECT(PEDAL_BAR_X + 2 + prevFW, barY + 2,
                         fillW - prevFW, PEDAL_BAR_H - 4, fillCol);
    } else if (fillW < prevFW) {
        // Bar shrank — clear only the removed portion
        tft.fillRect(PEDAL_BAR_X + 2 + fillW, barY + 2,
                     prevFW - fillW, PEDAL_BAR_H - 4, COL_BG);
        RTRACE_FILL_RECT(PEDAL_BAR_X + 2 + fillW, barY + 2,
                         prevFW - fillW, PEDAL_BAR_H - 4, COL_BG);
    }

    // Percentage text (right of bar) — anti-flicker: padding instead of fillRect
    char buf[FMT_BUF_SMALL];
    snprintf(buf, sizeof(buf), "%3u%%", pct);
    tft.setTextColor(COL_WHITE, COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(ML_DATUM);
    tft.setTextPadding(cfg::PAD_PEDAL_TEXT);
    tft.drawString(buf, PEDAL_TEXT_X, barY + PEDAL_BAR_H / 2);
    RTRACE_TEXT(PEDAL_TEXT_X, barY + PEDAL_BAR_H / 2, buf,
                COL_WHITE, COL_BG, 2, ML_DATUM);
    tft.setTextPadding(0);

    tft.setTextDatum(TL_DATUM);  // Reset
    tft.setTextSize(1);
}

} // namespace ui
