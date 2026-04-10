// =============================================================================
// ESP32-S3 HMI — Obstacle Sensor Boot Indicator (implementation)
//
// Renders sensor status text below the CAN link status on the boot screen.
// Uses the same style conventions as other boot screen elements.
//
// Reference: docs/FAIL_OPERATIONAL_MIGRATION_AUDIT.md — Step 5
// =============================================================================

#include "obstacle_indicator.h"
#include "../ui/ui_common.h"
#include "../ui/render_trace.h"

namespace hmi {

void ObstacleIndicator::draw(TFT_eSPI& tft,
                             obstacle_sensor::SensorStatus status,
                             obstacle_sensor::SensorStatus prevStatus,
                             uint16_t distance_mm,
                             uint16_t prevDistance_mm) {
    if (status == prevStatus && distance_mm == prevDistance_mm) return;

    // Position below CAN status line (boot screen layout)
    int16_t statusY = ui::SCREEN_H / 2 + 90;

    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.setTextPadding(280);   // anti-flicker: padding instead of fillRect

    // Buffer for status + distance text (e.g. "SENSOR: INVALID  65535 mm\0" = 29 chars)
    static constexpr int INDICATOR_BUF_SIZE = 40;
    char buf[INDICATOR_BUF_SIZE];

    switch (status) {
        case obstacle_sensor::SensorStatus::WAITING:
            tft.setTextColor(ui::COL_YELLOW, ui::COL_BG);
            tft.drawString("SENSOR: WAITING", ui::SCREEN_W / 2, statusY);
            RTRACE_TEXT(ui::SCREEN_W / 2, statusY, "SENSOR: WAITING",
                        ui::COL_YELLOW, ui::COL_BG, 1, MC_DATUM);
            break;
        case obstacle_sensor::SensorStatus::INVALID:
            snprintf(buf, sizeof(buf), "SENSOR: INVALID  %u mm", (unsigned)distance_mm);
            tft.setTextColor(ui::COL_RED, ui::COL_BG);
            tft.drawString(buf, ui::SCREEN_W / 2, statusY);
            RTRACE_TEXT(ui::SCREEN_W / 2, statusY, buf,
                        ui::COL_RED, ui::COL_BG, 1, MC_DATUM);
            break;
        case obstacle_sensor::SensorStatus::VALID:
            snprintf(buf, sizeof(buf), "SENSOR: VALID  %u mm", (unsigned)distance_mm);
            tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
            tft.drawString(buf, ui::SCREEN_W / 2, statusY);
            RTRACE_TEXT(ui::SCREEN_W / 2, statusY, buf,
                        ui::COL_GREEN, ui::COL_BG, 1, MC_DATUM);
            break;
    }

    tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);
}

}  // namespace hmi
