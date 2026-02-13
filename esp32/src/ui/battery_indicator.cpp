// =============================================================================
// ESP32-S3 HMI — Battery Indicator Implementation
// =============================================================================

#include "battery_indicator.h"
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
    // Battery terminal nub
    tft.fillRect(BAT_X + BAT_W - 6, BAT_Y + 8, 6, 12, COL_WHITE);
}

// -------------------------------------------------------------------------
// Update percentage display
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

    // Fill interior
    int16_t innerW = BAT_W - 10;
    int16_t fillW = static_cast<int16_t>(
        (static_cast<int32_t>(pct) * (innerW - 4)) / 100);

    tft.fillRect(BAT_X + 2, BAT_Y + 2, innerW - 4, BAT_H - 4, COL_BG);
    if (fillW > 0) {
        tft.fillRect(BAT_X + 2, BAT_Y + 2, fillW, BAT_H - 4, col);
    }

    // Percentage text centered in battery
    char buf[FMT_BUF_SMALL];
    snprintf(buf, sizeof(buf), "%u%%", pct);
    tft.setTextColor(COL_WHITE, col);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(buf, BAT_X + (BAT_W - 6) / 2, BAT_Y + BAT_H / 2);
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
