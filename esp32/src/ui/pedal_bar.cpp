// =============================================================================
// ESP32-S3 HMI — Throttle Micro-Bar Implementation
//
// Slim, label-free throttle indicator centred in the bottom cluster, below the
// gear pills.  A small ">>" glyph hints at "accelerate"; the bar itself fills
// green -> yellow -> red with the live throttle demand.  No numeric text keeps
// the premium cluster clean (favours graphics over digits).
// =============================================================================

#include "pedal_bar.h"
#include "render_trace.h"
#include "ui_config.h"

namespace ui {

// -------------------------------------------------------------------------
// Static frame: throttle glyph + bar outline.
// -------------------------------------------------------------------------
void PedalBar::drawStatic(TFT_eSPI& tft) {
    // Small ">" accelerate glyph to the left of the bar (iconographic).
    int16_t gy = DTHR_Y + DTHR_H / 2;
    tft.fillTriangle(DTHR_X - 12, gy - 4, DTHR_X - 12, gy + 4,
                     DTHR_X - 6, gy, COL_GRAY);
    RTRACE_LINE(DTHR_X - 12, gy - 4, DTHR_X - 6, gy, COL_GRAY);

    // Bar trough.
    tft.fillRoundRect(DTHR_X, DTHR_Y, DTHR_W, DTHR_H, 3, COL_GEAR_OFF);
    RTRACE_FILL_RECT(DTHR_X, DTHR_Y, DTHR_W, DTHR_H, COL_GEAR_OFF);
    tft.drawRoundRect(DTHR_X, DTHR_Y, DTHR_W, DTHR_H, 3, COL_GEAR_EDGE);
    RTRACE_DRAW_RECT(DTHR_X, DTHR_Y, DTHR_W, DTHR_H, COL_GEAR_EDGE);
}

// -------------------------------------------------------------------------
// Update fill — differential redraw (only the changed portion).
// -------------------------------------------------------------------------
void PedalBar::draw(TFT_eSPI& tft, uint8_t pedalPct, uint8_t prevPct) {
    if (pedalPct == prevPct) return;

    uint8_t pct    = (pedalPct > 100) ? 100 : pedalPct;
    uint8_t prevCl = (prevPct  > 100) ? 100 : prevPct;

    int16_t innerW = DTHR_W - 4;
    int16_t fillW  = static_cast<int16_t>((static_cast<int32_t>(pct)    * innerW) / 100);
    int16_t prevFW = static_cast<int16_t>((static_cast<int32_t>(prevCl) * innerW) / 100);

    uint16_t fillCol;
    if (pct <= cfg::PEDAL_COLOR_LOW)      fillCol = COL_GREEN;
    else if (pct <= cfg::PEDAL_COLOR_MID) fillCol = COL_YELLOW;
    else                                  fillCol = COL_RED;

    uint16_t prevCol;
    if (prevCl <= cfg::PEDAL_COLOR_LOW)      prevCol = COL_GREEN;
    else if (prevCl <= cfg::PEDAL_COLOR_MID) prevCol = COL_YELLOW;
    else                                     prevCol = COL_RED;

    int16_t y = DTHR_Y + 2;
    int16_t h = DTHR_H - 4;

    if (fillCol != prevCol) {
        if (prevFW > fillW) {
            tft.fillRect(DTHR_X + 2 + fillW, y, prevFW - fillW, h, COL_GEAR_OFF);
        }
        if (fillW > 0) {
            tft.fillRect(DTHR_X + 2, y, fillW, h, fillCol);
            RTRACE_FILL_RECT(DTHR_X + 2, y, fillW, h, fillCol);
        }
    } else if (fillW > prevFW) {
        if (prevFW < 0) prevFW = 0;
        tft.fillRect(DTHR_X + 2 + prevFW, y, fillW - prevFW, h, fillCol);
        RTRACE_FILL_RECT(DTHR_X + 2 + prevFW, y, fillW - prevFW, h, fillCol);
    } else if (fillW < prevFW) {
        tft.fillRect(DTHR_X + 2 + fillW, y, prevFW - fillW, h, COL_GEAR_OFF);
        RTRACE_FILL_RECT(DTHR_X + 2 + fillW, y, prevFW - fillW, h, COL_GEAR_OFF);
    }
}

} // namespace ui
