// =============================================================================
// ESP32-S3 HMI — 0x31B DIAG_SERVICE_SESSION / 0x31C DIAG_TEST_RESULT
// decoder + presentation helpers (pure)
//
// Pure, host-testable decode/text conversions for the SERVICE_DIAG self-test
// session (Bloque A, PR #445 Hito 1) consumed by the Engineering-screen
// "SERVICE AUTOTEST" submenu.  Deliberately free of Arduino / TFT / TWAI
// dependencies so the exact same code is exercised by the host cross-parity
// test (esp32/src/test_service_diag_view.cpp, which packs raw bytes with
// Core/Inc/service_diag_frame.h and decodes them here) and by the firmware —
// same pattern as traction_limit_diag_view.h / relay_health_view.h.
//
// Wire layout — see Core/Inc/service_diag_frame.h (the single authoritative
// source) for the byte-for-byte contract; the struct fields below mirror
// ServiceDiagSessionFrame_t / ServiceDiagTestResultFrame_t exactly.
//
// No dynamic allocation; text producers return static string literals only.
// The SVCDIAG_* wire values are the single definitions in can_ids.h.
// =============================================================================

#ifndef SERVICE_DIAG_VIEW_H
#define SERVICE_DIAG_VIEW_H

#include <cstddef>
#include <cstdint>

#include "can_ids.h"

namespace service_diag_view {

// -------------------------------------------------------------------------
// Frame shape + cadence (docs/CAN_CONTRACT_FINAL.md).
// -------------------------------------------------------------------------
inline constexpr uint8_t  SESSION_FRAME_DLC     = 8U;   // 0x31B
inline constexpr uint8_t  TEST_RESULT_FRAME_DLC = 8U;   // 0x31C
inline constexpr uint32_t SESSION_FRAME_PERIOD_MS = 100U;  // 10 Hz while active
// 0x31B is SILENT whenever no session is active — that is a normal IDLE
// condition, not a transport fault.  STALE_TIMEOUT_MS is therefore only
// meaningful while the caller's own cached state believes a session is
// live; it tolerates one dropped frame plus scheduling jitter (3x period).
inline constexpr uint32_t SESSION_STALE_TIMEOUT_MS = 3U * SESSION_FRAME_PERIOD_MS;  // 300 ms

// -------------------------------------------------------------------------
// Decoded 0x31B DIAG_SERVICE_SESSION view.
// -------------------------------------------------------------------------
struct SessionView {
    uint8_t state          = can::SVCDIAG_STATE_IDLE;
    uint8_t stepIndex       = 0;
    uint8_t activeChannel   = can::SVCDIAG_CH_NONE;
    uint8_t progressPct     = 0;
    uint8_t reason          = can::SVCDIAG_REASON_NONE;
    uint8_t originStateRaw  = 0;
    uint8_t elapsedSec      = 0;
    uint8_t activePwmPct    = 0;
};

// -------------------------------------------------------------------------
// Decoded 0x31C DIAG_TEST_RESULT view (one closed step).
// -------------------------------------------------------------------------
struct TestResultView {
    uint8_t  channel       = can::SVCDIAG_CH_NONE;
    uint8_t  pwmStepPct    = 0;
    uint16_t currentMa     = 0;
    uint16_t pulsesPerSec  = 0;
    uint8_t  verdict       = can::SVCDIAG_STEP_NONE;
    uint8_t  stepIndex     = 0;
};

inline bool decodeSession(const uint8_t* data, size_t dlc, SessionView& out) {
    if (data == nullptr || dlc < SESSION_FRAME_DLC) return false;
    out.state         = data[0];
    out.stepIndex      = data[1];
    out.activeChannel  = data[2];
    out.progressPct    = data[3];
    out.reason         = data[4];
    out.originStateRaw = data[5];
    out.elapsedSec     = data[6];
    out.activePwmPct   = data[7];
    return true;
}

inline bool decodeTestResult(const uint8_t* data, size_t dlc, TestResultView& out) {
    if (data == nullptr || dlc < TEST_RESULT_FRAME_DLC) return false;
    out.channel      = data[0];
    out.pwmStepPct   = data[1];
    out.currentMa    = (uint16_t)(data[2] | ((uint16_t)data[3] << 8));
    out.pulsesPerSec = (uint16_t)(data[4] | ((uint16_t)data[5] << 8));
    out.verdict      = data[6];
    out.stepIndex    = data[7];
    return true;
}

// -------------------------------------------------------------------------
// Freshness (millis()-wrap safe, matches motion_inhibit_view.h /
// traction_limit_diag_view.h conventions).
// -------------------------------------------------------------------------
enum class Freshness : uint8_t { NEVER_RECEIVED = 0, VALID = 1, STALE = 2 };

inline uint32_t ageMs(uint32_t nowMs, uint32_t stampMs) {
    return nowMs - stampMs;
}

inline Freshness freshness(bool valid, uint32_t nowMs, uint32_t stampMs,
                            uint32_t timeoutMs = SESSION_STALE_TIMEOUT_MS) {
    if (!valid) return Freshness::NEVER_RECEIVED;
    return (ageMs(nowMs, stampMs) <= timeoutMs) ? Freshness::VALID : Freshness::STALE;
}

// -------------------------------------------------------------------------
// Text helpers — mirror the exact Spanish wording of
// Core/Src/service_diag_session.c ServiceDiagSession_StateText() /
// ServiceDiagSession_ReasonText(), so the operator reads the same words a
// firmware log would use for the same wire byte.  A legible rejection
// message is a T5 requirement, hence Spanish (the vehicle's HMI language)
// rather than the terse English abbreviations used by the passive
// engineering diagnostic views (motion_inhibit_view.h etc.).
// -------------------------------------------------------------------------
inline const char* stateText(uint8_t state) {
    switch (state) {
        case can::SVCDIAG_STATE_IDLE:     return "INACTIVO";
        case can::SVCDIAG_STATE_ENTERING: return "ENTRANDO";
        case can::SVCDIAG_STATE_ARMED:    return "LISTO";
        case can::SVCDIAG_STATE_STEPPING: return "EJECUTANDO";
        case can::SVCDIAG_STATE_DEADTIME: return "PAUSA";
        case can::SVCDIAG_STATE_ABORTED:  return "ABORTADO";
        default:                          return "?";
    }
}

inline const char* reasonText(uint8_t reason) {
    switch (reason) {
        case can::SVCDIAG_REASON_NONE:            return "OK";
        case can::SVCDIAG_REASON_OPERATOR:         return "CANCELADO POR OPERADOR";
        case can::SVCDIAG_REASON_BOOT_STATE:       return "SISTEMA ARRANCANDO";
        case can::SVCDIAG_REASON_GEAR:             return "PON P O N";
        case can::SVCDIAG_REASON_WHEELS_MOVING:    return "RUEDAS EN MOVIMIENTO";
        case can::SVCDIAG_REASON_NOT_CONFIRMED:    return "FALTA CONFIRMACION";
        case can::SVCDIAG_REASON_BATTERY_LOW:      return "BATERIA BAJO CORTE";
        case can::SVCDIAG_REASON_BATTERY_WARN:     return "BATERIA BAJO AVISO";
        case can::SVCDIAG_REASON_ALREADY_ACTIVE:   return "SESION YA ACTIVA";
        case can::SVCDIAG_REASON_STATE_CHANGED:    return "ESTADO DEL VEHICULO CAMBIO";
        case can::SVCDIAG_REASON_CAN_LOSS:         return "SIN CAN";
        case can::SVCDIAG_REASON_GROUNDED_WHEEL:   return "COCHE APOYADO";
        case can::SVCDIAG_REASON_OVERCURRENT:      return "SOBRECORRIENTE DE PRUEBA";
        case can::SVCDIAG_REASON_SESSION_TIMEOUT:  return "TIEMPO DE SESION AGOTADO";
        case can::SVCDIAG_REASON_STEP_TIMEOUT:     return "TIEMPO DE PASO AGOTADO";
        case can::SVCDIAG_REASON_WATCHDOG:         return "WATCHDOG";
        default:                                   return "?";
    }
}

inline const char* channelText(uint8_t channel) {
    switch (channel) {
        case can::SVCDIAG_CH_FL:       return "FL";
        case can::SVCDIAG_CH_FR:       return "FR";
        case can::SVCDIAG_CH_RL:       return "RL";
        case can::SVCDIAG_CH_RR:       return "RR";
        case can::SVCDIAG_CH_STEERING: return "DIR";
        case can::SVCDIAG_CH_NONE:     return "--";
        default:                       return "?";
    }
}

inline const char* verdictText(uint8_t verdict) {
    switch (verdict) {
        case can::SVCDIAG_STEP_NONE:            return "--";
        case can::SVCDIAG_STEP_RUNNING:         return "EN CURSO";
        case can::SVCDIAG_STEP_PASS:            return "OK";
        case can::SVCDIAG_STEP_FAIL_OVERCURRENT: return "SOBRECORRIENTE";
        default:                                return "?";
    }
}

// True while the session is neither IDLE nor ABORTED — mirrors
// ServiceDiagSession_Active() on the STM32 side, so the HMI never needs a
// second, possibly-drifting definition of "is a session running".
inline bool isActive(uint8_t state) {
    return state != can::SVCDIAG_STATE_IDLE && state != can::SVCDIAG_STATE_ABORTED;
}

} // namespace service_diag_view

#endif // SERVICE_DIAG_VIEW_H
