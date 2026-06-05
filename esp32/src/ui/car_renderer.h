// =============================================================================
// ESP32-S3 HMI — Car Renderer
//
// Draws a premium top-down vehicle for the drive dashboard.  The intent is an
// OEM "instrument cluster" look, NOT a CAD/debug schematic:
//   * a ghost vehicle body (rounded, gradient-shaded) sits behind the wheels
//   * four shaded, semi-realistic tyres are the visual hero (wide 4x4 track)
//   * a stylised steering wheel + a green→orange steering arc sit on the body
//   * the mechanical driveline (axles / driveshaft) is a faint, de-emphasised
//     accent INSIDE the body — never the dominant element
// Per-wheel torque/temperature/current data is drawn beside each tyre so the
// tyre graphic itself stays clean.
//
// All drawing uses direct TFT primitives — no sprites, no heap allocation.
// Body, steering wheel and driveline are static (drawn once on screen enter);
// only the tyres and steering arc are repainted per frame.
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
    /// Draw the static vehicle (body + steering wheel + faint driveline +
    /// steering arc track).  Call once on screen enter.
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

    /// Draw steering angle indicator (green→orange arc + numeric angle).
    /// Only redraws if angle changed.
    static void drawSteering(TFT_eSPI& tft,
                             int16_t angleRaw,
                             int16_t prevAngleRaw);

private:
    /// Draw the static ghost vehicle body (gradient-shaded rounded silhouette).
    static void drawVehicleBody(TFT_eSPI& tft);

    /// Draw the static stylised steering wheel (rim + 3 spokes + hub).
    static void drawSteeringWheel(TFT_eSPI& tft);

    /// Draw the faint, de-emphasised mechanical driveline (axles + shaft).
    static void drawDriveline(TFT_eSPI& tft);

    /// Draw a single shaded tyre graphic at given position
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

    /// Draw the steering arc + numeric angle for a given raw angle / color.
    static void drawSteerArc(TFT_eSPI& tft,
                             int16_t angleRaw, uint16_t color);
};

} // namespace ui

#endif // CAR_RENDERER_H
