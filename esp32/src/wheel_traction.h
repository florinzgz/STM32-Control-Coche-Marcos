// =============================================================================
// ESP32-S3 HMI — Per-wheel traction share helper (pure, host-testable)
//
// Converts the four per-wheel motor currents (0x201, in 0.01 A units) into a
// per-wheel "traction percentage" so the operator can compare how hard each
// wheel is pulling.  This is a RELATIVE metric: the wheel drawing the most
// current is scaled to 100 %, the others are expressed as a percentage of that
// maximum.  During a turn this makes it obvious whether one side is pulling
// harder than the other — a quick visual check of the Ackermann / traction
// calibration.
//
// Design notes:
//   * Pure function, no Arduino / FastLED / TFT dependencies, so it can be
//     unit-tested on the host (see test_wheel_traction.cpp).
//   * Normalising to the maximum (not the sum) keeps a balanced 4-wheel drive
//     reading 100/100/100/100, and a fully-unloaded wheel reading 0 %.
//   * If every wheel reads 0 (vehicle stopped / no torque), all shares are 0 %
//     — never a divide-by-zero.
// =============================================================================

#ifndef WHEEL_TRACTION_H
#define WHEEL_TRACTION_H

#include <cstdint>

namespace traction {

// Number of driven wheels (FL, FR, RL, RR).
inline constexpr uint8_t NUM_TRACTION_WHEELS = 4;

// Compute the relative traction share for each wheel.
//
//   rawCurrent0p01A : four per-wheel currents in 0.01 A units (as received on
//                     CAN 0x201: FL, FR, RL, RR).
//   outPct          : four output percentages 0..100, normalised so the wheel
//                     with the highest current reads 100 %.  All zero when the
//                     maximum current is zero.
inline void computeShare(const uint16_t rawCurrent0p01A[NUM_TRACTION_WHEELS],
                         uint8_t outPct[NUM_TRACTION_WHEELS]) {
    uint16_t maxRaw = 0;
    for (uint8_t i = 0; i < NUM_TRACTION_WHEELS; ++i) {
        if (rawCurrent0p01A[i] > maxRaw) maxRaw = rawCurrent0p01A[i];
    }

    if (maxRaw == 0) {
        for (uint8_t i = 0; i < NUM_TRACTION_WHEELS; ++i) outPct[i] = 0;
        return;
    }

    for (uint8_t i = 0; i < NUM_TRACTION_WHEELS; ++i) {
        // Round to nearest percent.  (raw * 100 + max/2) / max.
        const uint32_t scaled =
            ((uint32_t)rawCurrent0p01A[i] * 100U + (maxRaw / 2U)) / maxRaw;
        outPct[i] = (scaled > 100U) ? 100U : (uint8_t)scaled;
    }
}

}  // namespace traction

#endif  // WHEEL_TRACTION_H
