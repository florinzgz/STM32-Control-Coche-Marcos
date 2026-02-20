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

namespace hmi {

void ObstacleIndicator::draw(TFT_eSPI& tft,
                             obstacle_sensor::SensorStatus status,
                             obstacle_sensor::SensorStatus prevStatus) {
    if (status == prevStatus) return;

    // Position below CAN status line (boot screen layout)
    int16_t statusY = ui::SCREEN_H / 2 + 90;
    tft.fillRect(0, statusY - 10, ui::SCREEN_W, 20, ui::COL_BG);

    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);

    switch (status) {
        case obstacle_sensor::SensorStatus::WAITING:
            tft.setTextColor(ui::COL_YELLOW, ui::COL_BG);
            tft.drawString("SENSOR: WAITING", ui::SCREEN_W / 2, statusY);
            break;
        case obstacle_sensor::SensorStatus::INVALID:
            tft.setTextColor(ui::COL_RED, ui::COL_BG);
            tft.drawString("SENSOR: INVALID", ui::SCREEN_W / 2, statusY);
            break;
        case obstacle_sensor::SensorStatus::VALID:
            tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
            tft.drawString("SENSOR: VALID", ui::SCREEN_W / 2, statusY);
            break;
    }

    tft.setTextDatum(TL_DATUM);
}

} // namespace hmi
