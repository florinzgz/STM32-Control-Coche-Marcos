// =============================================================================
// ESP32-S3 HMI — 0x317 DIAG_RELAY_HEALTH presentation helpers (pure)
//
// Pure, host-testable conversions used by the Engineering-screen relay/current
// health page.  Free of Arduino / TFT / TWAI dependencies so the same code is
// exercised by the host unit test (esp32/src/test_relay_health_view.cpp) and
// by the firmware.
//
//   * raw 8-byte 0x317 frame -> decoded RelayHealthView struct
//   * diag reason code       -> human MOTIVO text + recommended SOLUCIÓN text
//   * reason                 -> evidence grade (CONFIRMADO/PROBABLE/INCONCLUSO)
//   * frame age              -> VALID / STALE / NEVER RECEIVED (wrap-safe)
//
// The wire layout / reason / flag codes are defined once in
// esp32/include/can_ids.h, mirroring Core/Inc/relay_health_frame.h.
// =============================================================================

#ifndef RELAY_HEALTH_VIEW_H
#define RELAY_HEALTH_VIEW_H

#include <cstdint>

#include "can_ids.h"

namespace relay_health_view {

// 0x317 is transmitted at 1 Hz.  STALE after three nominal periods (3 s).
inline constexpr uint32_t FRAME_PERIOD_MS  = 1000U;
inline constexpr uint32_t STALE_TIMEOUT_MS = 3U * FRAME_PERIOD_MS;

struct RelayHealthView {
    uint8_t  reason;              // RELAY_DIAG_* (byte 0)
    bool     relayCommanded;
    bool     seqComplete;
    bool     powerReady;
    bool     anyWheelMoving;
    bool     currentValid;
    bool     currentStale;
    bool     inaMissing;
    bool     polarityReversed;
    uint16_t currentSumCa;        // sum |CH0..3| in centi-amps (bytes 2-3)
    uint8_t  throttlePct;         // byte 4
    uint8_t  finalPwmPct;         // byte 5
    uint16_t sampleAgeMs;         // bytes 6-7
};

// Decode an 8-byte 0x317 payload.  `len` must be >= 8; a short frame leaves
// `out` zero-initialised and returns false.
inline bool decode(const uint8_t* data, uint8_t len, RelayHealthView& out) {
    out = RelayHealthView{};
    if (data == nullptr || len < 8) {
        return false;
    }
    out.reason = data[0];

    const uint8_t f = data[1];
    out.relayCommanded   = (f & can::RELAY_DIAG_FLAG_RELAY_CMD)     != 0;
    out.seqComplete      = (f & can::RELAY_DIAG_FLAG_SEQ_COMPLETE)  != 0;
    out.powerReady       = (f & can::RELAY_DIAG_FLAG_POWER_READY)   != 0;
    out.anyWheelMoving   = (f & can::RELAY_DIAG_FLAG_WHEEL_MOVING)  != 0;
    out.currentValid     = (f & can::RELAY_DIAG_FLAG_CURRENT_VALID) != 0;
    out.currentStale     = (f & can::RELAY_DIAG_FLAG_CURRENT_STALE) != 0;
    out.inaMissing       = (f & can::RELAY_DIAG_FLAG_INA_MISSING)   != 0;
    out.polarityReversed = (f & can::RELAY_DIAG_FLAG_POLARITY_REV)  != 0;

    out.currentSumCa = static_cast<uint16_t>(
        static_cast<uint16_t>(data[2]) | (static_cast<uint16_t>(data[3]) << 8));
    out.throttlePct  = data[4];
    out.finalPwmPct  = data[5];
    out.sampleAgeMs  = static_cast<uint16_t>(
        static_cast<uint16_t>(data[6]) | (static_cast<uint16_t>(data[7]) << 8));
    return true;
}

// Sum current as amps (for display).  centi-A / 100.
inline float currentAmps(const RelayHealthView& v) {
    return static_cast<float>(v.currentSumCa) / 100.0f;
}

// -------------------------------------------------------------------------
// Freshness (millis()-wrap-safe).
// -------------------------------------------------------------------------
enum class Freshness : uint8_t {
    NEVER_RECEIVED = 0,
    VALID          = 1,
    STALE          = 2
};

inline uint32_t ageMs(uint32_t nowMs, uint32_t stampMs) {
    return nowMs - stampMs;
}

inline Freshness freshness(bool valid, uint32_t nowMs, uint32_t stampMs,
                           uint32_t timeoutMs = STALE_TIMEOUT_MS) {
    if (!valid) {
        return Freshness::NEVER_RECEIVED;
    }
    return (ageMs(nowMs, stampMs) <= timeoutMs) ? Freshness::VALID
                                                : Freshness::STALE;
}

inline const char* freshnessText(Freshness fresh) {
    switch (fresh) {
        case Freshness::VALID:          return "VALID";
        case Freshness::STALE:          return "STALE";
        case Freshness::NEVER_RECEIVED: return "NEVER RECEIVED";
    }
    return "NEVER RECEIVED";
}

// -------------------------------------------------------------------------
// Diagnostic reason -> MOTIVO (short human cause).
// -------------------------------------------------------------------------
inline const char* reasonText(uint8_t reason) {
    switch (reason) {
        case can::RELAY_DIAG_OK:                    return "OK";
        case can::RELAY_DIAG_OPEN_CONFIRMED:        return "RELE ABIERTO CONFIRMADO";
        case can::RELAY_DIAG_OPEN_SUSPECTED:        return "RELE ABIERTO SOSPECHA";
        case can::RELAY_DIAG_CURRENT_SENSE_INVALID: return "SENSOR CORRIENTE INVALIDO";
        case can::RELAY_DIAG_SHUNT_OPEN:            return "SHUNT ABIERTO";
        case can::RELAY_DIAG_SHUNT_BYPASSED:        return "SHUNT PUENTEADO";
        case can::RELAY_DIAG_POLARITY_REVERSED:     return "POLARIDAD INVERTIDA";
        case can::RELAY_DIAG_DATA_STALE:            return "CORRIENTE OBSOLETA";
        case can::RELAY_DIAG_INA_MISSING:           return "INA AUSENTE";
        case can::RELAY_DIAG_SCALE_INVALID:         return "ESCALA INVALIDA";
        case can::RELAY_DIAG_INCONCLUSIVE:
        default:                                    return "INCONCLUSO";
    }
}

// Evidence grade word for the HMI.
inline const char* confidenceText(uint8_t reason) {
    switch (reason) {
        case can::RELAY_DIAG_OPEN_CONFIRMED:
            return "CONFIRMADO";
        case can::RELAY_DIAG_INCONCLUSIVE:
            return "INCONCLUSO";
        default:
            return "PROBABLE";
    }
}

// -------------------------------------------------------------------------
// Diagnostic reason -> SOLUCIÓN (recommended physical check / next step).
// -------------------------------------------------------------------------
inline const char* solutionText(uint8_t reason) {
    switch (reason) {
        case can::RELAY_DIAG_OK:
            return "NINGUNA";
        case can::RELAY_DIAG_OPEN_CONFIRMED:
        case can::RELAY_DIAG_OPEN_SUSPECTED:
            return "REVISAR RELE TRAC/24V/CONTACTO";
        case can::RELAY_DIAG_CURRENT_SENSE_INVALID:
            return "REVISAR SHUNT/R002/CABLEADO INA";
        case can::RELAY_DIAG_SHUNT_OPEN:
            return "REVISAR SHUNT R002";
        case can::RELAY_DIAG_SHUNT_BYPASSED:
            return "REVISAR RUTA POTENCIA/SHUNT";
        case can::RELAY_DIAG_POLARITY_REVERSED:
            return "INVERTIR IN+/IN- INA";
        case can::RELAY_DIAG_DATA_STALE:
            return "REVISAR I2C/MUX INA";
        case can::RELAY_DIAG_INA_MISSING:
            return "REVISAR INA226 EN BUS";
        case can::RELAY_DIAG_SCALE_INVALID:
            return "RECALIBRAR ESCALA INA";
        case can::RELAY_DIAG_INCONCLUSIVE:
        default:
            return "PISAR PEDAL CON RELE ON";
    }
}

// True when the reason denotes an active problem worth surfacing.
inline bool isFault(uint8_t reason) {
    return (reason != can::RELAY_DIAG_OK) &&
           (reason != can::RELAY_DIAG_INCONCLUSIVE);
}

} // namespace relay_health_view

#endif // RELAY_HEALTH_VIEW_H
