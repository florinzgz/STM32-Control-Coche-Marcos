// =============================================================================
// ESP32-S3 HMI — Steering Text Display
//
// Shows steering angle in degrees with left/right direction indicator.
// Drawn at the bottom of the screen (450–480px).
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef STEERING_DISPLAY_H
#define STEERING_DISPLAY_H

#include <TFT_eSPI.h>
#include "ui_common.h"

namespace ui {

class SteeringDisplay {
public:
    /// Draw static label (call once on screen enter)
    static void drawStatic(TFT_eSPI& tft);

    /// Update steering text. Only redraws if value changed.
    static void draw(TFT_eSPI& tft, int16_t angleRaw,
                     int16_t prevAngleRaw);
};

} // namespace ui

#endif // STEERING_DISPLAY_H
