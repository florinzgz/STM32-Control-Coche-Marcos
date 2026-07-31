// =============================================================================
// ESP32-S3 HMI — 0x31A DIAG_TRACTION_LIMITS decoder/presentation (pure)
//
// DLC 4, 1000 ms: obstacle scale, traction cap, brake-release ramp, and
// obstacle FSM state.  Diagnostic-only and host-testable; no Arduino/TFT.
// =============================================================================
#ifndef TRACTION_LIMIT_DIAG_VIEW_H
#define TRACTION_LIMIT_DIAG_VIEW_H

#include <cstddef>
#include <cstdint>

namespace traction_limit_diag_view {

inline constexpr uint8_t FRAME_DLC = 4U;
inline constexpr uint32_t FRAME_PERIOD_MS = 1000U;
inline constexpr uint32_t STALE_TIMEOUT_MS = 3000U;

enum class Freshness : uint8_t { NEVER_RECEIVED = 0, VALID = 1, STALE = 2 };

struct TractionLimitView {
    uint8_t obstacleScalePct = 0;
    uint8_t tractionCapPct = 0;
    uint8_t brakeReleasePct = 0;
    uint8_t obstacleState = 0;
};

inline bool decode(const uint8_t* data, size_t dlc, TractionLimitView& out) {
    if (data == nullptr || dlc < FRAME_DLC) return false;
    out.obstacleScalePct = data[0];
    out.tractionCapPct = data[1];
    out.brakeReleasePct = data[2];
    out.obstacleState = data[3];
    return true;
}

inline uint32_t ageMs(uint32_t nowMs, uint32_t stampMs) {
    return nowMs - stampMs;
}

inline Freshness freshness(bool valid, uint32_t nowMs, uint32_t stampMs) {
    if (!valid) return Freshness::NEVER_RECEIVED;
    return ageMs(nowMs, stampMs) <= STALE_TIMEOUT_MS ? Freshness::VALID
                                                     : Freshness::STALE;
}

inline const char* obstacleStateText(uint8_t state) {
    switch (state) {
        case 0: return "NO-SENS";
        case 1: return "NORMAL";
        case 2: return "CONFIRM";
        case 3: return "ACTIVE";
        case 4: return "CLEAR";
        case 5: return "SENSOR";
        default: return "UNKNOWN";
    }
}

} // namespace traction_limit_diag_view
#endif // TRACTION_LIMIT_DIAG_VIEW_H
