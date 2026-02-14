// =============================================================================
// ESP32-S3 HMI — Pedal Bar Widget
//
// Full-width pedal position bar (0–100%).
// Green → yellow → red gradient based on percentage.
// Includes arrow indicator and large percentage text.
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef PEDAL_BAR_H
#define PEDAL_BAR_H

#include <TFT_eSPI.h>
#include "ui_common.h"

namespace ui {

class PedalBar {
public:
    /// Draw static outline (call once on screen enter)
    static void drawStatic(TFT_eSPI& tft);

    /// Update pedal fill and text. Only redraws if value changed.
    static void draw(TFT_eSPI& tft, uint8_t pedalPct, uint8_t prevPct);
};

} // namespace ui

#endif // PEDAL_BAR_H
