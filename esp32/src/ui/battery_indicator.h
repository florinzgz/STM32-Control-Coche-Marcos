// =============================================================================
// ESP32-S3 HMI — Battery Indicator
//
// Displays battery percentage in the top-right corner.
// Derives percentage from voltage: 24V nominal (18V=0%, 25.2V=100%).
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef BATTERY_INDICATOR_H
#define BATTERY_INDICATOR_H

#include <TFT_eSPI.h>
#include "ui_common.h"

namespace ui {

class BatteryIndicator {
public:
    /// Draw static outline (call once on screen enter)
    static void drawStatic(TFT_eSPI& tft);

    /// Update battery percentage display. Only redraws if value changed.
    /// When `stale` is true the pack reading is older than the CAN-loss
    /// timeout (or never received): the gauge shows "--" instead of a
    /// misleading frozen percentage.  `prevStale` is the previous frame's
    /// stale flag, used to force a redraw across stale⇆fresh transitions.
    static void draw(TFT_eSPI& tft, uint16_t voltageRaw, uint16_t prevVoltageRaw,
                     bool stale = false, bool prevStale = false);

    /// Convert raw voltage (0.01V units) to percentage (0–100)
    static uint8_t voltageToPercent(uint16_t voltageRaw);
};

} // namespace ui

#endif // BATTERY_INDICATOR_H
