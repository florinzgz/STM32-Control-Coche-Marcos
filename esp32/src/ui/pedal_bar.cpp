// =============================================================================
// ESP32-S3 HMI — Throttle Bar Implementation (FASE 3.5)
//
// Premium throttle read-out replacing the old decorative "AMG / 4MATIC" strip.
// A labelled bar ("THROTTLE  NN%") that fills with a smooth
// green -> yellow -> orange -> red gradient proportional to the live throttle
// demand.  Presentation only — the value is the existing traction-average
// percentage already computed by the drive screen.
// =============================================================================

#include "pedal_bar.h"
#include "render_trace.h"
#include "ui_config.h"
#include <cstdio>

namespace ui {

// -------------------------------------------------------------------------
// Static frame: "THROTTLE" caption + bar trough.  The live percentage and
// the gradient fill are painted by draw().
// -------------------------------------------------------------------------
void PedalBar::drawStatic(TFT_eSPI& tft) {
    // Caption above the bar (left aligned).
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
    tft.setTextColor(COL_GRAY, COL_BG);
    tft.drawString("THROTTLE", DTHR_X, DTHR_LABEL_Y);
    RTRACE_TEXT(DTHR_X, DTHR_LABEL_Y, "THROTTLE", COL_GRAY, COL_BG, 1, TL_DATUM);

    // Bar trough.
    tft.fillRoundRect(DTHR_X, DTHR_Y, DTHR_W, DTHR_H, 3, COL_GEAR_OFF);
    RTRACE_FILL_RECT(DTHR_X, DTHR_Y, DTHR_W, DTHR_H, COL_GEAR_OFF);
    tft.drawRoundRect(DTHR_X, DTHR_Y, DTHR_W, DTHR_H, 3, COL_GEAR_EDGE);
    RTRACE_DRAW_RECT(DTHR_X, DTHR_Y, DTHR_W, DTHR_H, COL_GEAR_EDGE);
}

// -------------------------------------------------------------------------
// Update fill + percentage.  The gradient depends on each column's position,
// so the filled span is rebuilt as a single clean pass (no overlapping
// partial fills → no flicker).  Only repaints when the value changes.
// -------------------------------------------------------------------------
void PedalBar::draw(TFT_eSPI& tft, uint8_t pedalPct, uint8_t prevPct,
                    bool stale, bool prevStale) {
    if (pedalPct == prevPct && stale == prevStale) return;

    uint8_t pct = (pedalPct > 100) ? 100 : pedalPct;
    if (stale) pct = 0U;

    int16_t x0     = DTHR_X + 2;
    int16_t y      = DTHR_Y + 2;
    int16_t h      = DTHR_H - 4;
    int16_t innerW = DTHR_W - 4;
    int16_t fillW  = static_cast<int16_t>(
        (static_cast<int32_t>(pct) * innerW) / 100);

    // Clear the trough interior, then paint the gradient-filled span.  Each
    // column is shaded by its own position so the bar reads green at the
    // bottom of travel and red near full throttle.
    tft.fillRect(x0, y, innerW, h, COL_GEAR_OFF);
    RTRACE_FILL_RECT(x0, y, innerW, h, COL_GEAR_OFF);
    for (int16_t i = 0; i < fillW; ++i) {
        uint8_t colPct = static_cast<uint8_t>(
            (static_cast<int32_t>(i) * 100) / (innerW > 0 ? innerW : 1));
        tft.drawFastVLine(x0 + i, y, h, throttleGradColor(colPct));
    }
    if (fillW > 0) {
        RTRACE_FILL_RECT(x0, y, fillW, h, throttleGradColor(pct));
    }

    // Live percentage (right-aligned, on the caption row).
    char buf[FMT_BUF_SMALL];
    if (stale) snprintf(buf, sizeof(buf), "--");
    else       snprintf(buf, sizeof(buf), "%u%%", pct);
    uint16_t valCol = stale ? COL_GRAY : throttleGradColor(pct);
    tft.setTextDatum(TR_DATUM);
    tft.setTextSize(2);
    tft.setTextColor(valCol, COL_BG);
    tft.setTextPadding(64);
    tft.drawString(buf, DTHR_X + DTHR_W, DTHR_LABEL_Y - 4);
    RTRACE_TEXT(DTHR_X + DTHR_W, DTHR_LABEL_Y - 4, buf, valCol, COL_BG, 2, TR_DATUM);
    tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}

} // namespace ui
