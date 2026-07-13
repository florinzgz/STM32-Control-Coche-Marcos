// -------------------------------------------------------------------------
// pedal_cal_session_view.h — pure, host-testable presentation helpers for the
// guided pedal-calibration session (audit P5).  Mirrors the STM32-side
// PedalCalSession_StateText() / PedalCalSession_ReasonText() so the ESP32 HMI
// can render the 0x319 DIAG_PEDAL_CAL_SESSION frame without any Arduino/TFT
// dependency.  Unit-tested by test_pedal_cal_session_view.cpp.
// -------------------------------------------------------------------------
#ifndef PEDAL_CAL_SESSION_VIEW_H
#define PEDAL_CAL_SESSION_VIEW_H

#include <cstdint>

namespace pedalcal {

// Freshness of the last 0x319 frame.  A session frame older than this is
// considered stale (STM32 stopped emitting / CAN link down).
inline constexpr unsigned long kSessionStaleMs = 1500UL;  // >10× the 100 ms cadence

enum class Freshness : uint8_t { NEVER, FRESH, STALE };

inline Freshness sessionFreshness(unsigned long lastTs, unsigned long nowMs) {
    if (lastTs == 0UL) return Freshness::NEVER;
    return (nowMs - lastTs) <= kSessionStaleMs ? Freshness::FRESH : Freshness::STALE;
}

// Human label for a PedalCalState value (byte 0 of 0x319).
inline const char* sessionStateText(uint8_t state) {
    switch (state) {
        case 0:  return "IDLE";
        case 1:  return "ENTERING";
        case 2:  return "RELEASE PEDAL";
        case 3:  return "CAPTURING MIN";
        case 4:  return "PRESS FULLY";
        case 5:  return "CAPTURING MAX";
        case 6:  return "RELEASE TO SAVE";
        case 7:  return "READY TO SAVE";
        case 8:  return "SAVING";
        case 9:  return "COMPLETED";
        case 10: return "ABORTED";
        default: return "UNKNOWN";
    }
}

// Human text for the highest-priority reason bit set in the mask (byte 2-3).
// Priority order mirrors the STM32 PedalCalSession_ReasonText() exactly so the
// same bit yields the semantically-equivalent top message.  Returns "OK" when
// the mask is zero.
inline const char* sessionReasonText(uint16_t reason) {
    if (reason == 0x0000u)         return "OK";
    if (reason & 0x0100u)          return "EMERGENCY STOP";     // ABORT_EMERGENCY
    if (reason & 0x0040u)          return "SAFE MODE";          // ABORT_SAFE
    if (reason & 0x0080u)          return "CRITICAL ERROR";     // ABORT_ERROR
    if (reason & 0x0200u)          return "VEHICLE MOVING";     // ABORT_MOVEMENT
    if (reason & 0x0400u)          return "CAN LOSS";           // ABORT_CAN_LOSS
    if (reason & 0x0800u)          return "TIMEOUT";            // ABORT_TIMEOUT
    if (reason & 0x0001u)          return "NOT STANDBY";        // BLOCK_NOT_STANDBY
    if (reason & 0x0002u)          return "SHIFT TO P/N";       // BLOCK_GEAR
    if (reason & 0x0004u)          return "WHEELS MOVING";      // BLOCK_WHEELS_MOVING
    if (reason & 0x0020u)          return "TRACTION LIVE";      // BLOCK_TRACTION_LIVE
    if (reason & 0x0010u)          return "CRITICAL ERROR";     // BLOCK_CRITICAL_ERROR
    if (reason & 0x0008u)          return "PEDAL UNRELIABLE";   // BLOCK_PEDAL_IMPLAUSIBLE
    if (reason & 0x1000u)          return "MIN >= MAX";         // FAIL_MIN_GE_MAX
    if (reason & 0x2000u)          return "RANGE TOO SMALL";    // FAIL_RANGE_SMALL
    if (reason & 0x4000u)          return "SAMPLES UNSTABLE";   // FAIL_UNSTABLE
    if (reason & 0x8000u)          return "SAVE FAILED";        // FAIL_READBACK
    return "BLOCKED";
}

// Convenience: is the session currently active (running, not terminal/idle)?
inline bool sessionActive(uint8_t state) {
    return state != 0 /*IDLE*/ && state != 9 /*COMPLETED*/ && state != 10 /*ABORTED*/;
}

}  // namespace pedalcal

#endif  // PEDAL_CAL_SESSION_VIEW_H
