// =============================================================================
// ESP32-S3 HMI — Battery Indicator Implementation
// =============================================================================

#include "battery_indicator.h"
#include "render_trace.h"
#include <cstdio>

namespace ui {

// 24V lead-acid/LiFePO4 pack: 18.0V = 0%, 25.2V = 100%
static constexpr uint16_t BATT_MIN_RAW = 1800;   // 18.00 V in 0.01V units
static constexpr uint16_t BATT_MAX_RAW = 2520;   // 25.20 V in 0.01V units

// -------------------------------------------------------------------------
// Static outline
// -------------------------------------------------------------------------
void BatteryIndicator::drawStatic(TFT_eSPI& tft) {
    // Battery icon outline
    tft.drawRect(BAT_X, BAT_Y, BAT_W - 6, BAT_H, COL_WHITE);
    RTRACE_DRAW_RECT(BAT_X, BAT_Y, BAT_W - 6, BAT_H, COL_WHITE);
    // Battery terminal nub
    tft.fillRect(BAT_X + BAT_W - 6, BAT_Y + 8, 6, 12, COL_WHITE);
    RTRACE_FILL_RECT(BAT_X + BAT_W - 6, BAT_Y + 8, 6, 12, COL_WHITE);
}

// -------------------------------------------------------------------------
// Update percentage display
//
// Anti-flicker differential update: instead of clearing the entire
// battery interior and redrawing, only the changed portion is updated.
// -------------------------------------------------------------------------
void BatteryIndicator::draw(TFT_eSPI& tft,
                            uint16_t voltageRaw,
                            uint16_t prevVoltageRaw) {
    uint8_t pct     = voltageToPercent(voltageRaw);
    uint8_t prevPct = voltageToPercent(prevVoltageRaw);

    if (pct == prevPct) return;

    // Choose color based on level
    uint16_t col;
    if (pct > 50) {
        col = COL_GREEN;
    } else if (pct > 20) {
        col = COL_YELLOW;
    } else {
        col = COL_RED;
    }

    // Previous color for threshold detection
    uint16_t prevCol;
    if (prevPct > 50) {
        prevCol = COL_GREEN;
    } else if (prevPct > 20) {
        prevCol = COL_YELLOW;
    } else {
        prevCol = COL_RED;
    }

    // Fill interior — differential update
    int16_t innerW = BAT_W - 10;
    int16_t barInner = innerW - 4;
    int16_t fillW = static_cast<int16_t>(
        (static_cast<int32_t>(pct)     * barInner) / 100);
    int16_t prevFW = static_cast<int16_t>(
        (static_cast<int32_t>(prevPct) * barInner) / 100);

    if (col != prevCol) {
        // Color threshold crossed — redraw entire bar
        if (prevFW > fillW) {
            tft.fillRect(BAT_X + 2 + fillW, BAT_Y + 2,
                         prevFW - fillW, BAT_H - 4, COL_BG);
        }
        if (fillW > 0) {
            tft.fillRect(BAT_X + 2, BAT_Y + 2, fillW, BAT_H - 4, col);
            RTRACE_FILL_RECT(BAT_X + 2, BAT_Y + 2, fillW, BAT_H - 4, col);
        }
    } else if (fillW > prevFW) {
        // Bar grew — fill only the new portion
        if (prevFW < 0) prevFW = 0;
        tft.fillRect(BAT_X + 2 + prevFW, BAT_Y + 2,
                     fillW - prevFW, BAT_H - 4, col);
        RTRACE_FILL_RECT(BAT_X + 2 + prevFW, BAT_Y + 2,
                         fillW - prevFW, BAT_H - 4, col);
    } else if (fillW < prevFW) {
        // Bar shrank — clear only the removed portion
        tft.fillRect(BAT_X + 2 + fillW, BAT_Y + 2,
                     prevFW - fillW, BAT_H - 4, COL_BG);
        RTRACE_FILL_RECT(BAT_X + 2 + fillW, BAT_Y + 2,
                         prevFW - fillW, BAT_H - 4, COL_BG);
    }

    // Percentage text centered in battery — use black text for contrast
    char buf[FMT_BUF_SMALL];
    snprintf(buf, sizeof(buf), "%u%%", pct);
    // Use background color matching the dominant fill for text contrast
    uint16_t txtBg = (fillW > (barInner / 2)) ? col : COL_BG;
    uint16_t txtFg = (fillW > (barInner / 2)) ? COL_BLACK : col;
    tft.setTextColor(txtFg, txtBg);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.setTextPadding(innerW - 4);
    tft.drawString(buf, BAT_X + (BAT_W - 6) / 2, BAT_Y + BAT_H / 2);
    RTRACE_TEXT(BAT_X + (BAT_W - 6) / 2, BAT_Y + BAT_H / 2, buf,
                txtFg, txtBg, 1, MC_DATUM);
    tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// Voltage to percentage conversion
// -------------------------------------------------------------------------
uint8_t BatteryIndicator::voltageToPercent(uint16_t voltageRaw) {
    if (voltageRaw <= BATT_MIN_RAW) return 0;
    if (voltageRaw >= BATT_MAX_RAW) return 100;

    uint32_t range = BATT_MAX_RAW - BATT_MIN_RAW;
    uint32_t delta = voltageRaw - BATT_MIN_RAW;
    return static_cast<uint8_t>((delta * 100) / range);
}

} // namespace ui
