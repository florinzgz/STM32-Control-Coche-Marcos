// =============================================================================
// ESP32-S3 HMI — Obstacle Sensor Display Implementation
// =============================================================================

#include "obstacle_sensor.h"
#include "render_trace.h"
#include "ui_config.h"
#include <cstdio>

namespace ui {

// -------------------------------------------------------------------------
// Static label — centered horizontally on 480px-wide screen
// -------------------------------------------------------------------------
void ObstacleSensor::drawStatic(TFT_eSPI& tft) {
    tft.setTextColor(COL_WHITE, COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("SENSOR FRONTAL", SCREEN_W / 2, SENSOR_Y + 2);
    RTRACE_TEXT(SCREEN_W / 2, SENSOR_Y + 2, "SENSOR FRONTAL",
                COL_WHITE, COL_BG, 1, TC_DATUM);
    tft.setTextDatum(TL_DATUM);

    // Proximity bar outline — centered horizontally
    int16_t barY = SENSOR_Y + 28;
    tft.drawRect(SENSOR_BAR_X, barY, SENSOR_BAR_W, SENSOR_BAR_H, COL_GRAY);
    RTRACE_DRAW_RECT(SENSOR_BAR_X, barY, SENSOR_BAR_W, SENSOR_BAR_H, COL_GRAY);
}

// -------------------------------------------------------------------------
// Update distance value and proximity bar
//
// Anti-flicker differential update: only the changed portion of the
// proximity bar is redrawn. When bar grows, fill the new portion.
// When it shrinks, clear the old tail. Zero SPI overhead on static values.
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

    // Clear text area and redraw — centered horizontally (anti-flicker: padding)
    int16_t textY = SENSOR_Y + 14;
    tft.setTextColor(COL_WHITE, COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(TC_DATUM);
    tft.setTextPadding(cfg::PAD_OBSTACLE_DIST);
    tft.drawString(buf, SCREEN_W / 2, textY);
    RTRACE_TEXT(SCREEN_W / 2, textY, buf, COL_WHITE, COL_BG, 1, TC_DATUM);
    tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);

    // Proximity bar fill — differential update
    int16_t barY = SENSOR_Y + 28;
    static constexpr uint16_t BAR_MAX_CM = cfg::OBSTACLE_BAR_MAX_CM;
    int16_t barInner = SENSOR_BAR_W - 4;

    // Compute fill widths for current and previous values
    auto computeFill = [&](uint16_t dist) -> int16_t {
        if (dist == 0) return 0;
        uint16_t clamped = (dist > BAR_MAX_CM) ? BAR_MAX_CM : dist;
        return static_cast<int16_t>(
            (static_cast<int32_t>(BAR_MAX_CM - clamped) * barInner) / BAR_MAX_CM);
    };

    int16_t fillW = computeFill(distanceCm);
    int16_t prevFW = computeFill(prevDistanceCm);

    uint16_t fillCol = (distanceCm > 0) ? proximityColor(distanceCm) : COL_BG;
    uint16_t prevCol = (prevDistanceCm > 0) ? proximityColor(prevDistanceCm) : COL_BG;

    if (fillCol != prevCol || (prevDistanceCm == 0) != (distanceCm == 0)) {
        // Color changed or transitioning from/to no-reading: full bar redraw
        tft.fillRect(SENSOR_BAR_X + 2, barY + 2,
                     barInner, SENSOR_BAR_H - 4, COL_BG);
        if (fillW > 0 && distanceCm > 0) {
            tft.fillRect(SENSOR_BAR_X + 2, barY + 2,
                         fillW, SENSOR_BAR_H - 4, fillCol);
            RTRACE_FILL_RECT(SENSOR_BAR_X + 2, barY + 2,
                             fillW, SENSOR_BAR_H - 4, fillCol);
        }
    } else if (fillW > prevFW) {
        // Bar grew (object closer) — fill the new portion
        if (prevFW < 0) prevFW = 0;
        tft.fillRect(SENSOR_BAR_X + 2 + prevFW, barY + 2,
                     fillW - prevFW, SENSOR_BAR_H - 4, fillCol);
        RTRACE_FILL_RECT(SENSOR_BAR_X + 2 + prevFW, barY + 2,
                         fillW - prevFW, SENSOR_BAR_H - 4, fillCol);
    } else if (fillW < prevFW) {
        // Bar shrank (object further) — clear the removed portion
        tft.fillRect(SENSOR_BAR_X + 2 + fillW, barY + 2,
                     prevFW - fillW, SENSOR_BAR_H - 4, COL_BG);
        RTRACE_FILL_RECT(SENSOR_BAR_X + 2 + fillW, barY + 2,
                         prevFW - fillW, SENSOR_BAR_H - 4, COL_BG);
    }
}

} // namespace ui
