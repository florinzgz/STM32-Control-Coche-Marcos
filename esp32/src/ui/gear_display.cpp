// =============================================================================
// ESP32-S3 HMI — Gear Display Implementation
// =============================================================================

#include "gear_display.h"

namespace ui {

constexpr const char* GearDisplay::GEAR_LABELS[NUM_GEARS];

// -------------------------------------------------------------------------
// Draw all gear labels in inactive state
// -------------------------------------------------------------------------
void GearDisplay::drawStatic(TFT_eSPI& tft) {
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);

    for (uint8_t i = 0; i < NUM_GEARS; ++i) {
        int16_t x = GEAR_START_X + i * GEAR_SPACING + GEAR_LABEL_W / 2;
        int16_t y = GEAR_Y + GEAR_LABEL_H / 2;

        tft.fillRect(GEAR_START_X + i * GEAR_SPACING, GEAR_Y,
                     GEAR_LABEL_W, GEAR_LABEL_H, COL_BG);
        tft.drawRect(GEAR_START_X + i * GEAR_SPACING, GEAR_Y,
                     GEAR_LABEL_W, GEAR_LABEL_H, COL_GRAY);
        tft.setTextColor(COL_GRAY, COL_BG);
        tft.drawString(GEAR_LABELS[i], x, y);
    }

    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}

// -------------------------------------------------------------------------
// Update highlighted gear
// -------------------------------------------------------------------------
void GearDisplay::draw(TFT_eSPI& tft, Gear current, Gear previous) {
    if (current == previous) return;

    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);

    // Unhighlight previous gear
    uint8_t prevIdx = static_cast<uint8_t>(previous);
    if (prevIdx < NUM_GEARS) {
        int16_t px = GEAR_START_X + prevIdx * GEAR_SPACING;
        int16_t cx = px + GEAR_LABEL_W / 2;
        int16_t cy = GEAR_Y + GEAR_LABEL_H / 2;

        tft.fillRect(px, GEAR_Y, GEAR_LABEL_W, GEAR_LABEL_H, COL_BG);
        tft.drawRect(px, GEAR_Y, GEAR_LABEL_W, GEAR_LABEL_H, COL_GRAY);
        tft.setTextColor(COL_GRAY, COL_BG);
        tft.drawString(GEAR_LABELS[prevIdx], cx, cy);
    }

    // Highlight current gear
    uint8_t curIdx = static_cast<uint8_t>(current);
    if (curIdx < NUM_GEARS) {
        int16_t px = GEAR_START_X + curIdx * GEAR_SPACING;
        int16_t cx = px + GEAR_LABEL_W / 2;
        int16_t cy = GEAR_Y + GEAR_LABEL_H / 2;

        tft.fillRect(px, GEAR_Y, GEAR_LABEL_W, GEAR_LABEL_H, COL_GREEN);
        tft.drawRect(px, GEAR_Y, GEAR_LABEL_W, GEAR_LABEL_H, COL_WHITE);
        tft.setTextColor(COL_BLACK, COL_GREEN);
        tft.drawString(GEAR_LABELS[curIdx], cx, cy);
    }

    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}

} // namespace ui
