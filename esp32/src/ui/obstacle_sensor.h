// =============================================================================
// ESP32-S3 HMI — Obstacle Sensor Display
//
// Draws frontal obstacle distance indicator above the car body.
// Shows distance in meters and a color-coded proximity bar.
// Green (>3m), cyan (1.5–3m), yellow (0.8–1.5m), orange (0.3–0.8m), red (<0.3m).
// Tuned for TF-Mini Plus sensor (range 10 cm – 12 m).
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef OBSTACLE_SENSOR_H
#define OBSTACLE_SENSOR_H

#include <TFT_eSPI.h>
#include "ui_common.h"

namespace ui {

class ObstacleSensor {
public:
    /// Draw static label (call once on screen enter)
    static void drawStatic(TFT_eSPI& tft);

    /// Update distance display. Only redraws if value changed.
    static void draw(TFT_eSPI& tft, uint16_t distanceCm,
                     uint16_t prevDistanceCm);
};

} // namespace ui

#endif // OBSTACLE_SENSOR_H
