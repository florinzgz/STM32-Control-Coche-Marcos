// =============================================================================
// ESP32-S3 HMI — Car Renderer (FASE 3.5 — OEM premium top-down cockpit)
//
// Premium top-down vehicle for the drive dashboard.  Inspiration: 70% Hummer
// EV (robust, technological), 30% Mercedes AMG (3-spoke wheel).  The goal is a
// finished, premium-vehicle reading at a glance — NOT a CAD/wireframe/debug
// schematic.
//
//   * Solid, gradient-shaded SUV body with volume, depth and soft shadow.  The
//     mechanical driveline (axles/driveshaft) and the old wireframe accents are
//     gone — the body reads as a vehicle, not a diagram.
//   * The four tyres are the protagonists: shaded "capsules" at the corners,
//     each with % torque, °C and a relative 4-segment load bar.  The
//     hardest-working wheel is highlighted so it stands out immediately.
//   * ONE single steering system: a large Mercedes-style 3-spoke wheel at the
//     driver position inside the cockpit pod.  It rotates with the visually
//     reconstructed steering-wheel angle (road-wheel x 6.48).  Both magnitudes
//     are shown but never mixed: Steering Wheel (~+-350) and Wheel Angle (+-54).
//
// All drawing uses direct TFT primitives — no sprites, no heap allocation.
// The body shoulders + cockpit pod ring are static (drawn once on enter); only
// the tyre capsules and the rotating steering wheel + read-out repaint.
// Layout: 480x320 landscape orientation.
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
    /// Draw the static vehicle (premium body + cockpit pod ring).
    /// Call once on screen enter.
    static void drawStatic(TFT_eSPI& tft);

    /// Update the four wheel capsules with current data.  Always redraws all
    /// four when called (the relative "hardest-working" highlight depends on
    /// all four values together).  The owning tile only fires when telemetry
    /// crossed its change threshold, so this stays cheap.
    /// tractionValid / tempValid: when false the corresponding telemetry is
    /// stale or never received — the capsule shows "--" / "N/A" instead of a
    /// misleading default (e.g. 100% torque or 0 C).
    /// tractionActive: false when the powertrain is not engaged (no torque
    /// commanded — e.g. the car is being pushed by hand).  The 0x205 scale is
    /// then "available power" (typically 100%) rather than applied torque, so
    /// the capsule dims the value to grey to avoid a misleading red "100%".
    static void drawWheels(TFT_eSPI& tft,
                           const vehicle::TractionData& traction,
                           const vehicle::TempMapData& tempMap,
                           bool tractionValid = true,
                           bool tempValid = true,
                           bool tractionActive = true);

    /// Draw the single rotating steering wheel + dual angle read-out.
    /// Only repaints if the angle changed.
    static void drawSteering(TFT_eSPI& tft,
                             int16_t angleRaw,
                             int16_t prevAngleRaw);

private:
    /// Static premium body silhouette (gradient shoulders, soft shadow, hint
    /// of headlights — no wireframe/driveline).
    static void drawVehicleBody(TFT_eSPI& tft);

    /// Static cockpit pod ring (the recessed well the steering wheel sits in).
    static void drawCockpitPodStatic(TFT_eSPI& tft);

    /// Draw a single wheel capsule (tyre + % + C + relative load bar).
    static void drawWheelCapsule(TFT_eSPI& tft,
                                 int16_t x, int16_t y,
                                 const char* label,
                                 uint8_t torquePct, int8_t tempC,
                                 bool leftSide,
                                 bool isMaxLoad,
                                 bool tractionValid,
                                 bool tempValid,
                                 bool tractionActive = true);

    /// Draw the rotating 3-spoke wheel + read-out for a given raw angle.
    static void drawSteeringWheelDynamic(TFT_eSPI& tft, int16_t angleRaw);
};

} // namespace ui

#endif // CAR_RENDERER_H
