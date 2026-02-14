// =============================================================================
// ESP32-S3 HMI — Steering Text Display Implementation
// =============================================================================

#include "steering_display.h"
#include <cstdio>
#include <cstdlib>

namespace ui {

// -------------------------------------------------------------------------
// Static label
// -------------------------------------------------------------------------
void SteeringDisplay::drawStatic(TFT_eSPI& tft) {
    tft.setTextColor(COL_GRAY, COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("Steering:", 10, STEER_TEXT_Y);
}

// -------------------------------------------------------------------------
// Update steering angle text and direction indicator
// -------------------------------------------------------------------------
void SteeringDisplay::draw(TFT_eSPI& tft, int16_t angleRaw,
                           int16_t prevAngleRaw) {
    if (angleRaw == prevAngleRaw) return;

    // Convert raw (0.1° units) to display
    int16_t intPart  = angleRaw / 10;
    int16_t fracPart = (angleRaw < 0) ? (-(angleRaw % 10)) : (angleRaw % 10);

    char buf[FMT_BUF_MED];
    if (angleRaw < 0 && intPart == 0) {
        // Handle -0.X case: intPart is 0 but value is negative
        snprintf(buf, sizeof(buf), "-%d.%u", intPart, static_cast<unsigned>(fracPart));
    } else {
        snprintf(buf, sizeof(buf), "%d.%u", intPart, static_cast<unsigned>(fracPart));
    }

    // Clear value area
    tft.fillRect(80, STEER_TEXT_Y, 120, 14, COL_BG);

    tft.setTextColor(COL_WHITE, COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(buf, 80, STEER_TEXT_Y);

    // Degree symbol and direction
    int16_t textEnd = 80 + static_cast<int16_t>(strlen(buf)) * 6;
    tft.drawString("deg", textEnd + 2, STEER_TEXT_Y);

    // Direction arrow indicator (right side)
    tft.fillRect(220, STEER_TEXT_Y, 90, 14, COL_BG);
    tft.setTextDatum(TL_DATUM);
    if (angleRaw > 5) {
        // Turning right
        tft.setTextColor(COL_CYAN, COL_BG);
        tft.drawString(">> RIGHT", 220, STEER_TEXT_Y);
    } else if (angleRaw < -5) {
        // Turning left
        tft.setTextColor(COL_CYAN, COL_BG);
        tft.drawString("LEFT <<", 220, STEER_TEXT_Y);
    } else {
        // Center
        tft.setTextColor(COL_GREEN, COL_BG);
        tft.drawString("CENTER", 220, STEER_TEXT_Y);
    }
}

} // namespace ui
