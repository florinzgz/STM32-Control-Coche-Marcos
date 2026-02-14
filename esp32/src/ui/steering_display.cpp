// =============================================================================
// ESP32-S3 HMI — Steering Text Display Implementation
//
// In landscape layout (480×320), the steering angle is conveyed
// by the circular gauge in the car area. This module is retained
// for compatibility but drawStatic/draw are no-ops.
// =============================================================================

#include "steering_display.h"
#include <cstdio>
#include <cstdlib>

namespace ui {

// -------------------------------------------------------------------------
// Static label — no-op in landscape layout
// -------------------------------------------------------------------------
void SteeringDisplay::drawStatic(TFT_eSPI& /* tft */) {
    // Steering angle is shown via the circular gauge in the car area.
    // No separate text zone in 480×320 landscape layout.
}

// -------------------------------------------------------------------------
// Update — no-op in landscape layout
// -------------------------------------------------------------------------
void SteeringDisplay::draw(TFT_eSPI& /* tft */, int16_t /* angleRaw */,
                           int16_t /* prevAngleRaw */) {
    // Steering angle is shown via the circular gauge in the car area.
    // No separate text zone in 480×320 landscape layout.
}

} // namespace ui
