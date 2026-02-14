// =============================================================================
// ESP32-S3 HMI — Obstacle Sensor Display Implementation
// =============================================================================

#include "obstacle_sensor.h"
#include <cstdio>

namespace ui {

// -------------------------------------------------------------------------
// Static label
// -------------------------------------------------------------------------
void ObstacleSensor::drawStatic(TFT_eSPI& tft) {
    tft.setTextColor(COL_WHITE, COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("SENSOR FRONTAL", SCREEN_W / 2, SENSOR_Y);
    tft.setTextDatum(TL_DATUM);

    // Proximity bar outline
    int16_t barY = SENSOR_Y + 24;
    tft.drawRect(SENSOR_BAR_X, barY, SENSOR_BAR_W, SENSOR_BAR_H, COL_GRAY);
}

// -------------------------------------------------------------------------
// Update distance value and proximity bar
// -------------------------------------------------------------------------
void ObstacleSensor::draw(TFT_eSPI& tft, uint16_t distanceCm,
                          uint16_t prevDistanceCm) {
    if (distanceCm == prevDistanceCm) return;

    // Distance text: show in meters with 2 decimal places
    char buf[FMT_BUF_MED];
    if (distanceCm == 0) {
        snprintf(buf, sizeof(buf), "---");
    } else {
        uint16_t meters = distanceCm / 100;
        uint16_t cents  = distanceCm % 100;
        snprintf(buf, sizeof(buf), "%u.%02u m", meters, cents);
    }

    // Clear text area and redraw
    int16_t textY = SENSOR_Y + 10;
    tft.fillRect(SCREEN_W / 2 - 40, textY, 80, 12, COL_BG);
    tft.setTextColor(COL_WHITE, COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(TC_DATUM);
    tft.drawString(buf, SCREEN_W / 2, textY);
    tft.setTextDatum(TL_DATUM);

    // Proximity bar fill
    int16_t barY = SENSOR_Y + 24;

    // Clear bar interior
    tft.fillRect(SENSOR_BAR_X + 2, barY + 2,
                 SENSOR_BAR_W - 4, SENSOR_BAR_H - 4, COL_BG);

    // Only draw fill when sensor has a valid reading
    if (distanceCm > 0) {
        uint16_t col = proximityColor(distanceCm);

        // Map distance to bar width (400 cm = empty, 0 cm = full)
        uint16_t clampDist = (distanceCm > 400) ? 400 : distanceCm;
        int16_t fillW = static_cast<int16_t>(
            (static_cast<int32_t>(400 - clampDist) * (SENSOR_BAR_W - 4)) / 400);

        if (fillW > 0) {
            tft.fillRect(SENSOR_BAR_X + 2, barY + 2,
                         fillW, SENSOR_BAR_H - 4, col);
        }
    }
}

} // namespace ui
