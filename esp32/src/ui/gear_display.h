// =============================================================================
// ESP32-S3 HMI — Gear Display Widget
//
// Shows gear positions: P, R, N, D1, D2
// Highlights current gear only. No invented labels.
// Matches physical shifter exactly.
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef GEAR_DISPLAY_H
#define GEAR_DISPLAY_H

#include <TFT_eSPI.h>
#include "ui_common.h"

namespace ui {

/// Gear positions matching physical shifter
enum class Gear : uint8_t {
    P  = 0,
    R  = 1,
    N  = 2,
    D1 = 3,
    D2 = 4
};

inline constexpr uint8_t NUM_GEARS = 5;

class GearDisplay {
public:
    /// Draw all gear labels (call once on screen enter)
    static void drawStatic(TFT_eSPI& tft);

    /// Highlight the STM32-applied gear and show the physical lever request.
    /// When applied telemetry is stale no pill is presented as confirmed.
    static void draw(TFT_eSPI& tft,
                     Gear applied, Gear previousApplied,
                     bool appliedValid, bool previousAppliedValid,
                     Gear requested, Gear previousRequested);

private:
    static constexpr const char* GEAR_LABELS[NUM_GEARS] = {
        "P", "R", "N", "D1", "D2"
    };
};

} // namespace ui

#endif // GEAR_DISPLAY_H
