// =============================================================================
// ESP32-S3 HMI — 0x318 DIAG_INA_CH5 presentation helpers (pure)
//
// Pure, host-testable conversions used by the Engineering-screen INA226 LIVE
// DIAG page for the steering INA226 (CH5).  Free of Arduino / TFT / TWAI
// dependencies so the same code is exercised by the host unit test
// (esp32/src/test_ina226_ch5_view.cpp) and by the firmware.
//
//   * raw 8-byte 0x318 frame -> decoded Ina226Ch5View struct (SIGNED shunt)
//   * reason code            -> human status word + short cause + SOLUCIÓN
//   * frame age              -> VALID / STALE / NEVER RECEIVED (wrap-safe)
//
// A steering current is derived SIGNED from the raw shunt register and is
// NEVER clamped to zero, so a reversed installation shows a negative current.
// The mere ABSENCE of this frame is "n/d" (no CAN contract); a decoded frame
// with reason MISSING means the chip genuinely did not ACK — two different
// problems that must never be conflated.
//
// The wire layout / reason / flag codes are defined once in
// esp32/include/can_ids.h, mirroring Core/Inc/ina226_ch5_frame.h.
// =============================================================================

#ifndef INA226_CH5_VIEW_H
#define INA226_CH5_VIEW_H

#include <cstdint>

#include "can_ids.h"

namespace ina226_ch5_view {

// 0x318 is transmitted at 1 Hz.  STALE after three nominal periods (3 s).
inline constexpr uint32_t FRAME_PERIOD_MS  = 1000U;
inline constexpr uint32_t STALE_TIMEOUT_MS = 3U * FRAME_PERIOD_MS;

// Derivation constants (steering INA226 uses the 50A/75mV = 1.5 mΩ shunt);
// mirror INA226_CH5_SHUNT_* in Core/Inc/ina226_ch5_frame.h.
inline constexpr float SHUNT_LSB_UV = 2.5f;
inline constexpr float SHUNT_MOHM   = 1.5f;

struct Ina226Ch5View {
    uint8_t  reason;          // INA_CH5_* (byte 0)
    bool     muxSelectOk;
    bool     i2cAck;
    bool     identityOk;
    bool     configOk;
    bool     shuntReadOk;
    bool     busReadOk;
    bool     channelPowered;
    bool     stale;
    int16_t  rawShunt;        // signed shunt register (bytes 2-3)
    int32_t  shuntMicroVolts; // derived signed µV (never zeroed)
    int32_t  currentMilliAmps;// derived signed mA (never zeroed)
    uint16_t busMilliVolts;   // bytes 4-5
    uint16_t sampleAgeMs;     // bytes 6-7 (0xFFFF = never sampled)
};

// Round-to-nearest for signed floats (matches firmware, no libm).
inline int32_t iRound(float v) {
    return static_cast<int32_t>(v < 0.0f ? (v - 0.5f) : (v + 0.5f));
}

// Decode an 8-byte 0x318 payload.  `len` must be >= 8; a short frame leaves
// `out` zero-initialised and returns false.
inline bool decode(const uint8_t* data, uint8_t len, Ina226Ch5View& out) {
    out = Ina226Ch5View{};
    if (data == nullptr || len < 8) {
        return false;
    }
    out.reason = data[0];

    const uint8_t f = data[1];
    out.muxSelectOk    = (f & can::INA_CH5_FLAG_MUX_OK)      != 0;
    out.i2cAck         = (f & can::INA_CH5_FLAG_I2C_ACK)     != 0;
    out.identityOk     = (f & can::INA_CH5_FLAG_IDENTITY_OK) != 0;
    out.configOk       = (f & can::INA_CH5_FLAG_CONFIG_OK)   != 0;
    out.shuntReadOk    = (f & can::INA_CH5_FLAG_SHUNT_OK)    != 0;
    out.busReadOk      = (f & can::INA_CH5_FLAG_BUS_OK)      != 0;
    out.channelPowered = (f & can::INA_CH5_FLAG_POWERED)     != 0;
    out.stale          = (f & can::INA_CH5_FLAG_STALE)       != 0;

    out.rawShunt = static_cast<int16_t>(
        static_cast<uint16_t>(data[2]) | (static_cast<uint16_t>(data[3]) << 8));
    out.shuntMicroVolts   = iRound(static_cast<float>(out.rawShunt) * SHUNT_LSB_UV);
    out.currentMilliAmps  = iRound(static_cast<float>(out.shuntMicroVolts) / SHUNT_MOHM);
    out.busMilliVolts = static_cast<uint16_t>(
        static_cast<uint16_t>(data[4]) | (static_cast<uint16_t>(data[5]) << 8));
    out.sampleAgeMs = static_cast<uint16_t>(
        static_cast<uint16_t>(data[6]) | (static_cast<uint16_t>(data[7]) << 8));
    return true;
}

// Steering current in amps (signed, never zeroed).
inline float currentAmps(const Ina226Ch5View& v) {
    return static_cast<float>(v.currentMilliAmps) / 1000.0f;
}

// Bus voltage in volts.
inline float busVolts(const Ina226Ch5View& v) {
    return static_cast<float>(v.busMilliVolts) / 1000.0f;
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
// Reason -> one-line HMI status word (the audit's required CH5 states).
// -------------------------------------------------------------------------
inline const char* statusText(uint8_t reason) {
    switch (reason) {
        case can::INA_CH5_OK:                return "CH5 OK";
        case can::INA_CH5_PRESENT_NO_SHUNT:  return "CH5 PRESENT NO SHUNT";
        case can::INA_CH5_POLARITY_REVERSED: return "CH5 POLARITY REVERSED";
        case can::INA_CH5_STALE:             return "CH5 STALE";
        case can::INA_CH5_MUX_SELECT_FAIL:   return "CH5 MUX SELECT FAIL";
        case can::INA_CH5_MISSING:           return "CH5 MISSING";
        case can::INA_CH5_WRONG_ID:          return "CH5 WRONG ADDRESS";
        case can::INA_CH5_CONFIG_LOST:       return "CH5 CONFIG FAIL";
        case can::INA_CH5_READ_FAIL:         return "CH5 READ FAIL";
        case can::INA_CH5_UNKNOWN:
        default:                             return "CH5 UNKNOWN";
    }
}

// The "n/d" state used ONLY when no 0x318 frame has ever been received (no CAN
// contract / transport gap) — deliberately distinct from CH5 MISSING.
inline const char* notAvailableText() {
    return "CH5 n/d (SIN CONTRATO CAN)";
}

// -------------------------------------------------------------------------
// Reason -> SOLUCIÓN (recommended physical check / next step).
// -------------------------------------------------------------------------
inline const char* solutionText(uint8_t reason) {
    switch (reason) {
        case can::INA_CH5_OK:
            return "NINGUNA";
        case can::INA_CH5_PRESENT_NO_SHUNT:
            return "REVISAR SHUNT R002 DIRECCION";
        case can::INA_CH5_POLARITY_REVERSED:
            return "INVERTIR IN+/IN- INA CH5";
        case can::INA_CH5_STALE:
            return "REVISAR I2C/MUX CANAL 5";
        case can::INA_CH5_MUX_SELECT_FAIL:
            return "REVISAR TCA9548A CANAL 5";
        case can::INA_CH5_MISSING:
            return "REVISAR INA CH5 / RELE DIRECCION";
        case can::INA_CH5_WRONG_ID:
            return "REVISAR DIRECCION A0/A1/A2";
        case can::INA_CH5_CONFIG_LOST:
            return "REVISAR ALIMENTACION/RUIDO I2C";
        case can::INA_CH5_READ_FAIL:
            return "REVISAR I2C/MUX CANAL 5";
        case can::INA_CH5_UNKNOWN:
        default:
            return "REVISAR TELEMETRIA CH5";
    }
}

// True when the reason denotes an active problem worth surfacing.
inline bool isFault(uint8_t reason) {
    return (reason != can::INA_CH5_OK);
}

} // namespace ina226_ch5_view

#endif // INA226_CH5_VIEW_H
