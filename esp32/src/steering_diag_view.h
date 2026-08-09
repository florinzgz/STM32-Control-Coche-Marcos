// =============================================================================
// ESP32-S3 HMI — 0x316 DIAG_STEERING_CENTERING presentation helpers (pure)
//
// Pure, host-testable conversions used by the Engineering-screen
// "DIRECCIÓN NO SE MUEVE" page.  Deliberately free of Arduino / TFT / TWAI
// dependencies so the exact same code is exercised by the host unit test
// (esp32/src/test_steering_diag_view.cpp) and by the firmware.
//
//   * raw 8-byte 0x316 frame -> decoded SteeringDiagView struct
//   * diag reason code       -> human MOTIVO text + recommended ACCIÓN text
//   * FSM / owner / state     -> human text
//   * frame age               -> VALID / STALE / NEVER RECEIVED (wrap-safe)
//
// The wire layout and reason/flag codes are the single definitions in
// esp32/include/can_ids.h, mirroring Core/Inc/steering_centering_frame.h.
// No dynamic allocation; no side effects.
// =============================================================================

#ifndef STEERING_DIAG_VIEW_H
#define STEERING_DIAG_VIEW_H

#include <cstdint>

#include "can_ids.h"

namespace steering_diag_view {

// 0x316 is transmitted at 1 Hz (1000 ms nominal period).  A frame is STALE
// once three nominal periods have elapsed without a fresh timestamp (3 s),
// tolerating a couple of dropped frames while still flagging a silent link.
inline constexpr uint32_t FRAME_PERIOD_MS  = 1000U;
inline constexpr uint32_t STALE_TIMEOUT_MS = 3U * FRAME_PERIOD_MS;  // 3000 ms

// -------------------------------------------------------------------------
// Decoded view of one 0x316 frame.
// -------------------------------------------------------------------------
struct SteeringDiagView {
    uint8_t  reason;                // STEER_DIAG_* (byte 0)
    uint8_t  fsmState;              // CenteringState_t (byte 1 low nibble)
    uint8_t  motorOwner;            // 0 CENTERING, 1 EPS (byte 1 high nibble)
    uint8_t  systemState;           // STEER_DIAG_SS_* (byte 3 low nibble)

    bool     pb5Raw;
    bool     pb5Debounced;
    bool     pb5AlreadyActive;
    bool     relayPc12;             // PC12 steering-rail relay commanded
    bool     powerReady;
    bool     enPc4;                 // PC4 EN_STEER commanded
    bool     encoderFault;
    bool     restoredFromFlash;
    bool     moduleDisabled;
    bool     faultLatched;
    bool     pwmRequested;          // pwm_requested > 0
    bool     currentGuardArmed;     // CH5 end-stop guard armed (Bloque 2)

    uint16_t pwmReal;               // max CCR PA6/PA7 (bytes 4-5)
    int16_t  encoderDelta;          // signed delta from sweep origin (bytes 6-7)
};

// Decode an 8-byte 0x316 payload into a SteeringDiagView.  `len` must be >= 8;
// a short frame leaves `out` zero-initialised and returns false.
inline bool decode(const uint8_t* data, uint8_t len, SteeringDiagView& out) {
    out = SteeringDiagView{};
    if (data == nullptr || len < 8) {
        return false;
    }
    out.reason      = data[0];
    out.fsmState    = static_cast<uint8_t>(data[1] & 0x0F);
    out.motorOwner  = static_cast<uint8_t>((data[1] >> 4) & 0x0F);

    const uint8_t f = data[2];
    out.pb5Raw            = (f & can::STEER_DIAG_FLAG_PB5_RAW)        != 0;
    out.pb5Debounced      = (f & can::STEER_DIAG_FLAG_PB5_DEBOUNCED)  != 0;
    out.pb5AlreadyActive  = (f & can::STEER_DIAG_FLAG_PB5_ACTIVE)     != 0;
    out.relayPc12         = (f & can::STEER_DIAG_FLAG_RELAY_PC12)     != 0;
    out.powerReady        = (f & can::STEER_DIAG_FLAG_POWER_READY)    != 0;
    out.enPc4             = (f & can::STEER_DIAG_FLAG_EN_PC4)         != 0;
    out.encoderFault      = (f & can::STEER_DIAG_FLAG_ENCODER_FAULT)  != 0;
    out.restoredFromFlash = (f & can::STEER_DIAG_FLAG_RESTORED_FLASH) != 0;

    const uint8_t s = data[3];
    out.systemState    = static_cast<uint8_t>(s & can::STEER_DIAG_STATE_MASK);
    out.moduleDisabled = (s & can::STEER_DIAG_STATUS_MODULE_DISABLED) != 0;
    out.faultLatched   = (s & can::STEER_DIAG_STATUS_FAULT_LATCHED)   != 0;
    out.pwmRequested   = (s & can::STEER_DIAG_STATUS_PWM_REQUESTED)   != 0;
    out.currentGuardArmed = (s & can::STEER_DIAG_STATUS_CURRENT_GUARD_ARMED) != 0;

    out.pwmReal = static_cast<uint16_t>(
        static_cast<uint16_t>(data[4]) |
        (static_cast<uint16_t>(data[5]) << 8));
    out.encoderDelta = static_cast<int16_t>(
        static_cast<uint16_t>(data[6]) |
        (static_cast<uint16_t>(data[7]) << 8));
    return true;
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
// FSM state text (CenteringState_t: IDLE=0, SWEEP_LEFT=1, SWEEP_RIGHT=2,
// DONE=3, FAULT=4, WAIT_RAIL=5).
// -------------------------------------------------------------------------
inline const char* fsmText(uint8_t s) {
    switch (s) {
        case 0: return "IDLE";
        case 1: return "SWEEP LEFT";
        case 2: return "SWEEP RIGHT";
        case 3: return "DONE";
        case 4: return "FAULT";
        case 5: return "WAIT RAIL";
        default: return "UNKNOWN";
    }
}

// Motor owner (SteeringMotorOwner_t: CENTERING=0, EPS=1, NONE=2).
// NONE is reported once the assist has been isolated (EPS OFF): the motor is
// unowned and steering is purely mechanical.
inline const char* ownerText(uint8_t o) {
    switch (o) {
        case 0: return "CENTERING";
        case 1: return "EPS";
        case 2: return "EPS OFF (MECANICA)";
        default: return "UNKNOWN";
    }
}

// System state (STEER_DIAG_SS_*).
inline const char* systemStateText(uint8_t s) {
    switch (s) {
        case 0: return "BOOT";
        case 1: return "STANDBY";
        case 2: return "ACTIVE";
        case 3: return "DEGRADED";
        case 4: return "SAFE";
        case 5: return "ERROR";
        case 6: return "LIMP HOME";
        default: return "UNKNOWN";
    }
}

// -------------------------------------------------------------------------
// Diagnostic reason -> MOTIVO (short human cause).
// -------------------------------------------------------------------------
inline const char* reasonText(uint8_t reason) {
    switch (reason) {
        case can::STEER_DIAG_OK:                   return "OK";
        case can::STEER_DIAG_RESTORED_FROM_FLASH:  return "RESTAURADO FLASH";
        case can::STEER_DIAG_WAITING_POWER:        return "ESPERANDO POWER";
        case can::STEER_DIAG_CENTER_SENSOR_ACTIVE: return "PB5 YA ACTIVO";
        case can::STEER_DIAG_SWEEP_LEFT:           return "BARRIDO IZQUIERDA";
        case can::STEER_DIAG_SWEEP_RIGHT:          return "BARRIDO DERECHA";
        case can::STEER_DIAG_NO_ENCODER_MOVEMENT:  return "SIN MOV. ENCODER";
        case can::STEER_DIAG_ENCODER_FAULT:        return "FALLO ENCODER";
        case can::STEER_DIAG_RANGE_EXCEEDED:       return "RANGO EXCEDIDO";
        case can::STEER_DIAG_TOTAL_TIMEOUT:        return "TIMEOUT TOTAL";
        case can::STEER_DIAG_LOST_HOMING_STATE:    return "SALIO DE STANDBY";
        case can::STEER_DIAG_RELAY_NOT_READY:      return "RELE PC12 OFF";
        case can::STEER_DIAG_MODULE_DISABLED:      return "MODULO DESHAB.";
        case can::STEER_DIAG_ABORTED_SAFE:         return "ABORTADO (SAFE)";
        case can::STEER_DIAG_ABORTED_ERROR:        return "ABORTADO (ERROR)";
        case can::STEER_DIAG_UNKNOWN:
        default:                                   return "DESCONOCIDO";
    }
}

// -------------------------------------------------------------------------
// Diagnostic reason -> ACCIÓN (recommended physical check / next step).
// Phrased honestly: the firmware never invents the cause, and the ACCIÓN
// points the technician at the physical thing to verify.
// -------------------------------------------------------------------------
inline const char* actionText(uint8_t reason) {
    switch (reason) {
        case can::STEER_DIAG_OK:
        case can::STEER_DIAG_RESTORED_FROM_FLASH:
            return "NINGUNA";
        case can::STEER_DIAG_WAITING_POWER:
            return "ESPERAR RELE/POWER";
        case can::STEER_DIAG_CENTER_SENSOR_ACTIVE:
            return "REVISAR PB5/TORNILLO";
        case can::STEER_DIAG_SWEEP_LEFT:
        case can::STEER_DIAG_SWEEP_RIGHT:
            return "EN CURSO";
        case can::STEER_DIAG_NO_ENCODER_MOVEMENT:
            return "REVISAR BTS7960/EN/12V";
        case can::STEER_DIAG_ENCODER_FAULT:
            return "REVISAR ENCODER A/B";
        case can::STEER_DIAG_RANGE_EXCEEDED:
            return "REVISAR MECANICA/RACK";
        case can::STEER_DIAG_TOTAL_TIMEOUT:
            return "REVISAR MOTOR/PB5";
        case can::STEER_DIAG_LOST_HOMING_STATE:
            return "MANTENER EN STANDBY";
        case can::STEER_DIAG_RELAY_NOT_READY:
            return "REVISAR RELE PC12";
        case can::STEER_DIAG_MODULE_DISABLED:
            return "HABILITAR EN SERVICE";
        case can::STEER_DIAG_ABORTED_SAFE:
        case can::STEER_DIAG_ABORTED_ERROR:
            return "RESOLVER FALLO SISTEMA";
        case can::STEER_DIAG_UNKNOWN:
        default:
            return "DIAGNOSTICO MANUAL";
    }
}

// True when the reason indicates the sweep is stuck / not moving as expected
// (used to title the page "DIRECCIÓN NO SE MUEVE" only when warranted).
inline bool isStuck(uint8_t reason) {
    switch (reason) {
        case can::STEER_DIAG_NO_ENCODER_MOVEMENT:
        case can::STEER_DIAG_ENCODER_FAULT:
        case can::STEER_DIAG_RANGE_EXCEEDED:
        case can::STEER_DIAG_TOTAL_TIMEOUT:
        case can::STEER_DIAG_LOST_HOMING_STATE:
        case can::STEER_DIAG_RELAY_NOT_READY:
        case can::STEER_DIAG_CENTER_SENSOR_ACTIVE:
        case can::STEER_DIAG_ABORTED_SAFE:
        case can::STEER_DIAG_ABORTED_ERROR:
            return true;
        default:
            return false;
    }
}

} // namespace steering_diag_view

#endif // STEERING_DIAG_VIEW_H
