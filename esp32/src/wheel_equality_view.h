// =============================================================================
// ESP32-S3 HMI — 0x31D DIAG_WHEEL_EQUALITY decoder/presentation (pure)
//
// Wheel-equality / BTS7960 health self-test results (Hito 2, PR #445).  Pure,
// host-testable decode/text conversions consumed by the Engineering-screen
// "SERVICE AUTOTEST" submenu (wheel-equality results page).  Deliberately
// free of Arduino / TFT / TWAI dependencies so the exact same code is
// exercised by the host round-trip test (test_wheel_equality_view.cpp) and
// by the firmware — same pattern as traction_limit_diag_view.h /
// service_diag_view.h.
//
// Wire layout — see Core/Inc/wheel_equality_frame.h (the single authoritative
// source) for the byte-for-byte contract; FrameView mirrors
// WheelEqualityFrame_t exactly.  ONE CAN frame carries ONE field_id for ONE
// wheel (4 field kinds x 4 wheels = 16 frames per results burst); the
// per-wheel accumulation across field_ids happens in vehicle_data.h
// (WheelEqualityData), not here — this header only ever decodes a single
// received frame.
//
// No dynamic allocation; text producers return static string literals only.
// The WHEQ_* wire values are the single definitions in can_ids.h.
// =============================================================================

#ifndef WHEEL_EQUALITY_VIEW_H
#define WHEEL_EQUALITY_VIEW_H

#include <cstddef>
#include <cstdint>

#include "can_ids.h"

namespace wheel_equality_view {

// -------------------------------------------------------------------------
// Frame shape + cadence (docs/CAN_CONTRACT_FINAL.md).
// -------------------------------------------------------------------------
inline constexpr uint8_t  FRAME_DLC = 8U;   // 0x31D — fixed, no short form accepted
// 0x31D is an ON-DEMAND burst (16 frames, once when Fase 1 completes and
// again if Fase 2 also runs), never periodic — there is no natural
// "refresh rate" to report.  Kept at 0 (not periodic) so callers that
// generically log/display FRAME_PERIOD_MS for every diagnostic view have a
// well-defined, documented value instead of a fabricated cadence.
inline constexpr uint32_t FRAME_PERIOD_MS = 0U;
// Freshness horizon for a wheel's LAST received results (any field_id).
// 3000 ms exactly: <=3000 is VALID, >3000 (i.e. >=3001) is STALE — matches
// the mandatory host-test boundary in test_wheel_equality_view.cpp.
inline constexpr uint32_t STALE_TIMEOUT_MS = 3000U;

// -------------------------------------------------------------------------
// Decoded view of one 0x31D sub-frame (one wheel, one field_id) — mirrors
// WheelEqualityFrame_t (Core/Inc/wheel_equality_frame.h) field-for-field.
// Only the fields belonging to the decoded fieldId are meaningful; the
// others are zero/false (decode() always resets the whole struct first).
// -------------------------------------------------------------------------
struct FrameView {
    uint8_t wheel          = 0;   // 0=FL,1=FR,2=RL,3=RR
    uint8_t fieldId        = 0;   // WHEQ_FIELD_*
    bool    phase2Included = false;

    // FIELD_SPEED
    uint16_t pulsesPerSec25      = 0;
    uint16_t pulsesPerSec50      = 0;
    uint16_t normalizedSpeedX1000 = 0;   // /1000.0 for the real value
    uint8_t  deviationPct        = 0;    // 0..100, magnitude only

    // FIELD_CURRENT
    uint16_t currentMa25        = 0;
    uint16_t currentMa50        = 0;     // the HMI's single "corriente" column
    uint16_t slopeMaPerPctX10   = 0;      // /10.0 for mA per PWM-percent
    uint8_t  probableCause      = 0;      // WHEQ_CAUSE_*

    // FIELD_HEALTH
    uint16_t asymmetryPctX10    = 0;      // /10.0 for the real percent
    uint16_t deltaTempCX10      = 0;      // /10.0 for degrees C; 0 if !tempPresent
    uint8_t  halfBridgeVerdict  = 0;      // WHEQ_HALFBRIDGE_*
    bool     tempPresent        = false;

    // FIELD_VERDICT
    uint8_t  wheelVerdict       = 0;      // WHEQ_WHEEL_VERDICT_*
    uint8_t  driverVerdict      = 0;      // WHEQ_DRIVER_*
    bool     phase2Ran          = false;
    uint8_t  driverReasonMask   = 0;      // WHEQ_DRIVER_REASON_* bitmask
};

// Decode ONE received 0x31D payload.  Rejects outright (returns false,
// leaves *out untouched) unless dlc is EXACTLY FRAME_DLC and the embedded
// field_id is 0..3 — mirrors WheelEqualityFrame_Unpack()'s "no
// forward-compatible short form" contract exactly (strict equality, not
// merely dlc >= FRAME_DLC as the simpler single-shot diagnostic views use).
inline bool decode(const uint8_t* data, size_t dlc, FrameView& out) {
    if (data == nullptr || dlc != FRAME_DLC) return false;
    const uint8_t fieldId = static_cast<uint8_t>((data[0] >> 2) & 0x03U);
    if (fieldId > 3U) return false;

    out = FrameView{};   // reset every field so a caller never sees stale data
    out.wheel          = static_cast<uint8_t>(data[0] & 0x03U);
    out.fieldId        = fieldId;
    out.phase2Included = (data[0] & (1U << 4)) != 0U;

    switch (fieldId) {
    case can::WHEQ_FIELD_SPEED:
        out.pulsesPerSec25       = static_cast<uint16_t>(data[1] | (static_cast<uint16_t>(data[2]) << 8));
        out.pulsesPerSec50       = static_cast<uint16_t>(data[3] | (static_cast<uint16_t>(data[4]) << 8));
        out.normalizedSpeedX1000 = static_cast<uint16_t>(data[5] | (static_cast<uint16_t>(data[6]) << 8));
        out.deviationPct         = data[7];
        break;
    case can::WHEQ_FIELD_CURRENT:
        out.currentMa25      = static_cast<uint16_t>(data[1] | (static_cast<uint16_t>(data[2]) << 8));
        out.currentMa50      = static_cast<uint16_t>(data[3] | (static_cast<uint16_t>(data[4]) << 8));
        out.slopeMaPerPctX10 = static_cast<uint16_t>(data[5] | (static_cast<uint16_t>(data[6]) << 8));
        out.probableCause    = data[7];
        break;
    case can::WHEQ_FIELD_HEALTH:
        out.asymmetryPctX10   = static_cast<uint16_t>(data[1] | (static_cast<uint16_t>(data[2]) << 8));
        out.deltaTempCX10     = static_cast<uint16_t>(data[3] | (static_cast<uint16_t>(data[4]) << 8));
        out.halfBridgeVerdict = data[5];
        out.tempPresent       = (data[6] != 0U);
        break;
    case can::WHEQ_FIELD_VERDICT:
        out.wheelVerdict     = data[1];
        out.driverVerdict    = data[2];
        out.phase2Ran        = (data[3] != 0U);
        out.driverReasonMask = data[4];
        break;
    default:
        return false;
    }
    return true;
}

// -------------------------------------------------------------------------
// Freshness (millis()-wrap safe, matches traction_limit_diag_view.h /
// service_diag_view.h conventions).  ageMs<=3000 is VALID, >3000 is STALE.
// -------------------------------------------------------------------------
enum class Freshness : uint8_t { NEVER_RECEIVED = 0, VALID = 1, STALE = 2 };

inline uint32_t ageMs(uint32_t nowMs, uint32_t stampMs) {
    return nowMs - stampMs;
}

inline Freshness freshness(bool valid, uint32_t nowMs, uint32_t stampMs) {
    if (!valid) return Freshness::NEVER_RECEIVED;
    return ageMs(nowMs, stampMs) <= STALE_TIMEOUT_MS ? Freshness::VALID
                                                      : Freshness::STALE;
}

// -------------------------------------------------------------------------
// Text helpers — legible Spanish labels for the HMI table, never a bare
// numeric code (T5-style requirement).  Full/verbose wording mirrors
// Core/Src/wheel_equality_test.c WheelEqTest_*Text() verbatim; the *_Short
// variants below are compact abbreviations sized to fit the 4-row results
// table's narrow columns while staying unambiguous.
// -------------------------------------------------------------------------
inline const char* wheelVerdictText(uint8_t v) {
    switch (v) {
        case can::WHEQ_WHEEL_VERDICT_PENDING:               return "PENDIENTE";
        case can::WHEQ_WHEEL_VERDICT_PASS:                   return "PASS";
        case can::WHEQ_WHEEL_VERDICT_WARN:                   return "WARN";
        case can::WHEQ_WHEEL_VERDICT_FAIL:                   return "FAIL";
        case can::WHEQ_WHEEL_VERDICT_FAIL_ACKERMANN_OFFSET:  return "FAIL_ACKERMANN_OFFSET";
        default:                                             return "?";
    }
}

inline const char* wheelVerdictShortText(uint8_t v) {
    switch (v) {
        case can::WHEQ_WHEEL_VERDICT_PENDING:               return "PEND";
        case can::WHEQ_WHEEL_VERDICT_PASS:                   return "PASS";
        case can::WHEQ_WHEEL_VERDICT_WARN:                   return "WARN";
        case can::WHEQ_WHEEL_VERDICT_FAIL:                   return "FAIL";
        case can::WHEQ_WHEEL_VERDICT_FAIL_ACKERMANN_OFFSET:  return "F.ACKER";
        default:                                             return "?";
    }
}

inline const char* driverVerdictText(uint8_t v) {
    switch (v) {
        case can::WHEQ_DRIVER_PENDING:     return "PENDIENTE";
        case can::WHEQ_DRIVER_SANO:        return "SANO";
        case can::WHEQ_DRIVER_SOSPECHOSO:  return "SOSPECHOSO";
        case can::WHEQ_DRIVER_DEGRADADO:   return "DEGRADADO";
        default:                           return "?";
    }
}

inline const char* driverVerdictShortText(uint8_t v) {
    switch (v) {
        case can::WHEQ_DRIVER_PENDING:     return "PEND";
        case can::WHEQ_DRIVER_SANO:        return "SANO";
        case can::WHEQ_DRIVER_SOSPECHOSO:  return "SOSPECH";
        case can::WHEQ_DRIVER_DEGRADADO:   return "DEGRAD";
        default:                           return "?";
    }
}

// Full (verbose) probable-cause wording — mirrors WheelEqTest_CauseText()
// verbatim so an operator reading a firmware log and the HMI see the same
// words for the same wire byte.
inline const char* causeText(uint8_t c) {
    switch (c) {
        case can::WHEQ_CAUSE_NONE:          return "-";
        case can::WHEQ_CAUSE_MECHANICAL:    return "RESISTENCIA MECANICA (rodamiento/freno/reductora/rozamiento)";
        case can::WHEQ_CAUSE_ELECTRICAL:    return "FALTA DE PAR ELECTRICO (escobillas/conexion/driver/caida tension)";
        case can::WHEQ_CAUSE_SENSOR:        return "SENSOR (no la rueda)";
        case can::WHEQ_CAUSE_OTHERS_BRAKED: return "REVISAR SI LAS OTRAS TRES ESTAN FRENADAS";
        default:                            return "?";
    }
}

// Compact cause label for the narrow results-table column.
inline const char* causeShortText(uint8_t c) {
    switch (c) {
        case can::WHEQ_CAUSE_NONE:          return "-";
        case can::WHEQ_CAUSE_MECHANICAL:    return "MECANICA";
        case can::WHEQ_CAUSE_ELECTRICAL:    return "ELECTRICA";
        case can::WHEQ_CAUSE_SENSOR:        return "SENSOR";
        case can::WHEQ_CAUSE_OTHERS_BRAKED: return "OTRAS FREN";
        default:                            return "?";
    }
}

inline const char* halfBridgeVerdictText(uint8_t v) {
    switch (v) {
        case can::WHEQ_HALFBRIDGE_PASS: return "PASS";
        case can::WHEQ_HALFBRIDGE_WARN: return "WARN_HALFBRIDGE";
        case can::WHEQ_HALFBRIDGE_FAIL: return "FAIL_HALFBRIDGE";
        default:                        return "?";
    }
}

// True for any wheel verdict that represents an actionable defect (WARN,
// FAIL or FAIL_ACKERMANN_OFFSET) — lets the HMI colour/sort a row without a
// second, possibly-drifting definition of "this wheel needs attention".
inline bool isWheelVerdictFault(uint8_t v) {
    return v == can::WHEQ_WHEEL_VERDICT_WARN ||
           v == can::WHEQ_WHEEL_VERDICT_FAIL ||
           v == can::WHEQ_WHEEL_VERDICT_FAIL_ACKERMANN_OFFSET;
}

} // namespace wheel_equality_view

#endif // WHEEL_EQUALITY_VIEW_H
