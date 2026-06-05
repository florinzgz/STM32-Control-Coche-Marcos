// =============================================================================
// ESP32-S3 HMI — Analog Dial Gauge Widget
//
// Premium OEM-style circular gauge with a sweeping needle, used for the SPEED
// and RPM dials on the DriveScreen.  The gauge draws a 270° arc (gap at the
// bottom) with major ticks, a colour progress arc, a needle and a centred
// digital read-out.
//
// Layered draw model (mirrors the rest of the HMI):
//   drawStatic()  — face disc + bezel ring + major ticks. Drawn once.
//   draw()        — clears the interior disc, repaints the progress arc,
//                   needle, hub, digital value and unit.  Called only when the
//                   value changed (tile is dirty).
//
// Radii are laid out so the dynamic interior (cleared every update) never
// touches the static bezel/ticks:
//   bezel ring        : r, r-1
//   static major ticks: r-5 … r-2
//   dynamic clear disc : r-6
//   dynamic progress   : r-10 … r-7
//   dynamic needle     : length r-12
//
// Only TFT_eSPI primitives are used (drawLine, drawCircle, fillCircle,
// fillTriangle, drawString).  No sprites, no heap, no String.
// =============================================================================

#ifndef DIAL_GAUGE_H
#define DIAL_GAUGE_H

#include <TFT_eSPI.h>
#include <cstdint>
#include "ui_common.h"

namespace ui {

class DialGauge {
public:
    /// Draw the static face, bezel ring and major ticks (call once on enter).
    static void drawStatic(TFT_eSPI& tft, int16_t cx, int16_t cy, int16_t r);

    /// Repaint the dynamic interior: progress arc, needle, hub, value + unit.
    /// @param value       current value (0..maxValue, clamped)
    /// @param maxValue    full-scale value (must be > 0)
    /// @param needleColor needle + progress accent colour
    /// @param zoned       true → progress arc coloured green/amber/red by
    ///                    position (RPM style); false → solid needleColor
    /// @param valueText   pre-formatted digital read-out (caller owns buffer)
    /// @param unit        short unit string drawn under the value ("km/h"/"rpm")
    static void draw(TFT_eSPI& tft, int16_t cx, int16_t cy, int16_t r,
                     uint16_t value, uint16_t maxValue,
                     uint16_t needleColor, bool zoned,
                     const char* valueText, const char* unit);
};

} // namespace ui

#endif // DIAL_GAUGE_H
