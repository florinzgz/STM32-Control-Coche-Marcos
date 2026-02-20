// =============================================================================
// ESP32-S3 HMI — Obstacle Sensor Boot Indicator
//
// Displays obstacle sensor status on the boot screen:
//   WAITING  — sensor warmup in progress (yellow)
//   INVALID  — sensor fault, stuck, or out-of-range (red)
//   VALID    — healthy reading within physical range (green)
//
// Reference: docs/FAIL_OPERATIONAL_MIGRATION_AUDIT.md — Step 5
// =============================================================================

#ifndef OBSTACLE_INDICATOR_H
#define OBSTACLE_INDICATOR_H

#include <TFT_eSPI.h>
#include "../sensors/obstacle_sensor.h"

namespace hmi {

class ObstacleIndicator {
public:
    /// Draw the obstacle sensor status line on the boot screen.
    /// Only redraws when status changes.
    static void draw(TFT_eSPI& tft,
                     obstacle_sensor::SensorStatus status,
                     obstacle_sensor::SensorStatus prevStatus);
};

} // namespace hmi

#endif // OBSTACLE_INDICATOR_H
