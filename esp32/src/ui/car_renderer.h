// =============================================================================
// ESP32-S3 HMI — Car Renderer
//
// Draws top-view vehicle outline with 4 wheels.
// Each wheel displays torque percentage, motor temperature, and
// color-coded fill based on torque level.
// Car body is clean — no numbers or text overlaid on the body.
// Steering angle shown as circular gauge on the right side of car area.
//
// All drawing uses direct TFT calls — no sprites, no heap allocation.
// Layout: 480×320 landscape orientation.
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef CAR_RENDERER_H
#define CAR_RENDERER_H

#include <TFT_eSPI.h>
#include "ui_common.h"
#include "vehicle_data.h"

namespace ui {

class CarRenderer {
public:
    /// Draw static car body outline (call once on screen enter)
    static void drawStatic(TFT_eSPI& tft);

    /// Update wheel displays with current data.
    /// Only redraws elements that have changed.
    /// tractionValid / tempValid: when false the corresponding telemetry is
    /// stale or never received — the label shows "--" / "N/A" instead of a
    /// misleading default (e.g. 100% torque or 0 °C) and the prev* caches are
    /// invalidated by the caller so the placeholder is drawn.
    static void drawWheels(TFT_eSPI& tft,
                           const vehicle::TractionData& traction,
                           const vehicle::TempMapData& tempMap,
                           const uint8_t prevTraction[4],
                           const int8_t prevTemp[4],
                           bool tractionValid = true,
                           bool tempValid = true);

    /// Draw steering angle indicator.
    /// Only redraws if angle changed.
    static void drawSteering(TFT_eSPI& tft,
                             int16_t angleRaw,
                             int16_t prevAngleRaw);

private:
    /// Draw a single wheel tyre graphic at given position
    static void drawWheel(TFT_eSPI& tft,
                          int16_t x, int16_t y,
                          uint8_t torquePct, int8_t tempC,
                          bool tractionValid = true);

    /// Draw torque/temp label text next to a wheel
    static void drawWheelLabel(TFT_eSPI& tft,
                               int16_t wx, int16_t wy,
                               uint8_t torquePct, int8_t tempC,
                               bool rightSide,
                               bool tractionValid = true,
                               bool tempValid = true);

    /// Clear and redraw steering rotation line
    static void drawSteerLine(TFT_eSPI& tft,
                              int16_t angleRaw, uint16_t color);
};

} // namespace ui

#endif // CAR_RENDERER_H
