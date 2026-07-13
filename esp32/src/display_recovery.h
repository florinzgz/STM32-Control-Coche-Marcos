// display_recovery.h — Core-0 TFT recovery choreography + "PANTALLA RECUPERADA"
// banner (audit Problem 2, requirements §4 and §6).
//
// The display::Supervisor (display_supervisor.h) owns the PURE detection and
// retry policy.  This header adds the two remaining PURE, host-testable pieces
// of the recovery vertical slice:
//
//   1. RecoverySequence — the exact ORDERED list of hardware steps that the
//      recovery MUST perform, and only ever from Core 0 (never re-initialise
//      the TFT from Core 1).  The firmware executes the steps in this order,
//      calling Supervisor::advanceRecovery() between the coarse FSM stages; the
//      list here is the single source of truth and is asserted by the host
//      test so the choreography cannot silently drift.
//
//   2. formatRecoveryBanner() — renders the post-recovery banner with the exact
//      fields the audit requires (causa / duración / intentos / render
//      continuó / reboot ESP32 / heap / recuperaciones totales / acción
//      recomendada) into a caller-provided buffer using snprintf, so the buffer
//      bound is explicit and testable (audit P6 "buffers snprintf").
//
// No hardware access, no Arduino, no delays — fully host-testable
// (test_display_recovery.cpp).

#ifndef DISPLAY_RECOVERY_H
#define DISPLAY_RECOVERY_H

#include <cstdint>
#include <cstdio>

#include "display_supervisor.h"

namespace display {

// ---- Ordered recovery steps (audit §4) -----------------------------------
// Executed strictly in this order and exclusively on Core 0.
enum class RecoveryStep : uint8_t {
    STOP_RENDER_TOUCH = 0,  // detener render/touch
    BLOCK_TFT_ACCESS,       // bloquear acceso TFT (exclusive ownership)
    TFT_CS_HIGH,            // TFT_CS HIGH
    TOUCH_CS_HIGH,          // TOUCH_CS HIGH
    CLOSE_SPI,              // cerrar SPI
    PULSE_GPIO38,           // pulso controlado en GPIO38 (panel reset line)
    TFT_INIT,               // tft.init()
    SET_ROTATION_1,         // setRotation(1)
    RESTORE_TOUCH_CAL,      // restaurar calibración touch
    RESTORE_BACKLIGHT,      // restaurar backlight
    INVALIDATE_CACHES,      // invalidar tiles, hashes y cachés
    FORCE_FULL_REDRAW,      // forzar redraw completo
    VERIFY,                 // verificar
    RESUME,                 // reanudar render/touch
};

inline const char* recoveryStepText(RecoveryStep s) {
    switch (s) {
    case RecoveryStep::STOP_RENDER_TOUCH: return "detener render/touch";
    case RecoveryStep::BLOCK_TFT_ACCESS:  return "bloquear acceso TFT";
    case RecoveryStep::TFT_CS_HIGH:       return "TFT_CS HIGH";
    case RecoveryStep::TOUCH_CS_HIGH:     return "TOUCH_CS HIGH";
    case RecoveryStep::CLOSE_SPI:         return "cerrar SPI";
    case RecoveryStep::PULSE_GPIO38:      return "pulso GPIO38";
    case RecoveryStep::TFT_INIT:          return "tft.init()";
    case RecoveryStep::SET_ROTATION_1:    return "setRotation(1)";
    case RecoveryStep::RESTORE_TOUCH_CAL: return "restaurar cal. touch";
    case RecoveryStep::RESTORE_BACKLIGHT: return "restaurar backlight";
    case RecoveryStep::INVALIDATE_CACHES: return "invalidar cachés";
    case RecoveryStep::FORCE_FULL_REDRAW: return "redraw completo";
    case RecoveryStep::VERIFY:            return "verificar";
    case RecoveryStep::RESUME:            return "reanudar";
    default:                              return "?";
    }
}

// ---- Differentiated recovery trigger (audit §4 "PROBLEMA DE DETECCIÓN") ---
// The supervisor must NEVER label a plain render timeout as a confirmed white
// screen.  These five categories are transported honestly so the log/banner
// distinguishes an automatically-observable cause from an inferred one.
enum class RecoveryTrigger : uint8_t {
    NONE = 0,
    RENDER_STALLED,               // Core-1 saw the render heartbeat stop
                                  // (reliable) — does NOT by itself prove the
                                  // panel went white.
    TFT_CONTROLLER_STATUS_LOST,   // status/ID readback INVALID (reliable only
                                  // when the panel supports readback).
    TFT_RESET_SIGNAL_DETECTED,    // GPIO38 / TFT_RST latched low unexpectedly
                                  // (reliable only with the hardware latch).
    RECOVERY_MANUAL_REQUEST,      // explicit bench/manual request (the panel
                                  // went white but is not auto-observable).
    WHITE_SCREEN_NOT_OBSERVABLE,  // no automatic signal available with the
                                  // current hardware — white cannot be proven.
};

inline const char* recoveryTriggerText(RecoveryTrigger t) {
    switch (t) {
    case RecoveryTrigger::NONE:                       return "SIN SOLICITUD";
    case RecoveryTrigger::RENDER_STALLED:             return "RENDER DETENIDO";
    case RecoveryTrigger::TFT_CONTROLLER_STATUS_LOST: return "ESTADO TFT PERDIDO";
    case RecoveryTrigger::TFT_RESET_SIGNAL_DETECTED:  return "SENAL RESET TFT";
    case RecoveryTrigger::RECOVERY_MANUAL_REQUEST:    return "SOLICITUD MANUAL";
    case RecoveryTrigger::WHITE_SCREEN_NOT_OBSERVABLE:return "BLANCO NO OBSERVABLE";
    default:                                          return "?";
    }
}

// True only when the "white screen" condition can be observed automatically —
// i.e. the panel supports status readback OR a hardware reset-signal latch
// exists.  When both are false, a stale render is the ONLY evidence and white
// must NOT be asserted automatically (audit §4.4).
inline bool whiteScreenObservable(bool readback_supported,
                                  bool reset_latch_available) {
    return readback_supported || reset_latch_available;
}

// Pure classification of the recovery trigger from the raw observations, in the
// strict priority order the audit mandates.  A stale render with no reliable
// readback and no reset latch is reported as RENDER_STALLED (an honest,
// automatically-observable fact) — never as a confirmed white screen.
inline RecoveryTrigger classifyRecoveryTrigger(bool manual_request,
                                               bool reset_signal_latched,
                                               StatusRead status,
                                               bool render_stale) {
    if (manual_request)                    return RecoveryTrigger::RECOVERY_MANUAL_REQUEST;
    if (reset_signal_latched)              return RecoveryTrigger::TFT_RESET_SIGNAL_DETECTED;
    if (status == StatusRead::INVALID)     return RecoveryTrigger::TFT_CONTROLLER_STATUS_LOST;
    if (render_stale)                      return RecoveryTrigger::RENDER_STALLED;
    return RecoveryTrigger::WHITE_SCREEN_NOT_OBSERVABLE;
}

// Map a trigger onto the banner Fault cause (kept consistent with the honest
// "no auto white" policy: a manual request or a stale render maps to
// TFT_RESET_PROBABLE / RENDER_TIMEOUT, never a fabricated confirmed white).
inline Fault recoveryTriggerToFault(RecoveryTrigger t) {
    switch (t) {
    case RecoveryTrigger::RENDER_STALLED:             return Fault::RENDER_TIMEOUT;
    case RecoveryTrigger::TFT_CONTROLLER_STATUS_LOST: return Fault::TFT_STATUS_LOST;
    case RecoveryTrigger::TFT_RESET_SIGNAL_DETECTED:  return Fault::TFT_RESET_PROBABLE;
    case RecoveryTrigger::RECOVERY_MANUAL_REQUEST:    return Fault::TFT_RESET_PROBABLE;
    case RecoveryTrigger::WHITE_SCREEN_NOT_OBSERVABLE:return Fault::READBACK_UNSUPPORTED;
    case RecoveryTrigger::NONE:
    default:                                          return Fault::NONE;
    }
}

// ---- Cross-core recovery request mailbox (audit §4) ----------------------
// Core 1 (loop/supervisor) is the only producer; Core 0 (render task) is the
// only consumer.  This is the PURE handshake state; the firmware wraps
// request()/take()/done() in a portMUX critical section so the two cores never
// race.  Core 1 NEVER touches TFT_eSPI — it only posts a request here.
class RecoveryRequestMailbox {
public:
    // Core 1 posts a request.  Rejected (returns false) when one is already
    // pending or a recovery is in flight, so requests never stack up.
    bool request(RecoveryTrigger t) {
        if (t == RecoveryTrigger::NONE) return false;
        if (pending_ != RecoveryTrigger::NONE || busy_) return false;
        pending_ = t;
        return true;
    }

    // Core 0 takes the pending request (clears it and marks a recovery busy).
    bool take(RecoveryTrigger& out) {
        if (pending_ == RecoveryTrigger::NONE) return false;
        out      = pending_;
        pending_ = RecoveryTrigger::NONE;
        busy_    = true;
        return true;
    }

    // Core 0 reports the hardware recovery finished (verified or not).  The
    // result is latched for Core 1 to finalise the supervisor FSM.
    void done(bool verified_ok, uint32_t duration_ms) {
        busy_          = false;
        result_ready_  = true;
        verified_ok_   = verified_ok;
        duration_ms_   = duration_ms;
    }

    // Core 1 consumes the result exactly once.
    bool takeResult(bool& verified_ok, uint32_t& duration_ms) {
        if (!result_ready_) return false;
        result_ready_ = false;
        verified_ok   = verified_ok_;
        duration_ms   = duration_ms_;
        return true;
    }

    bool pending() const { return pending_ != RecoveryTrigger::NONE; }
    bool busy()    const { return busy_; }

private:
    RecoveryTrigger pending_      = RecoveryTrigger::NONE;
    bool            busy_         = false;
    bool            result_ready_ = false;
    bool            verified_ok_  = false;
    uint32_t        duration_ms_  = 0;
};

// Number of steps in the ordered recovery sequence.
inline constexpr uint8_t kRecoveryStepCount = 14;

// The canonical ordered sequence.  Returns a pointer to a static array of
// kRecoveryStepCount entries.
inline const RecoveryStep* recoverySequence() {
    static const RecoveryStep seq[kRecoveryStepCount] = {
        RecoveryStep::STOP_RENDER_TOUCH,
        RecoveryStep::BLOCK_TFT_ACCESS,
        RecoveryStep::TFT_CS_HIGH,
        RecoveryStep::TOUCH_CS_HIGH,
        RecoveryStep::CLOSE_SPI,
        RecoveryStep::PULSE_GPIO38,
        RecoveryStep::TFT_INIT,
        RecoveryStep::SET_ROTATION_1,
        RecoveryStep::RESTORE_TOUCH_CAL,
        RecoveryStep::RESTORE_BACKLIGHT,
        RecoveryStep::INVALIDATE_CACHES,
        RecoveryStep::FORCE_FULL_REDRAW,
        RecoveryStep::VERIFY,
        RecoveryStep::RESUME,
    };
    return seq;
}

// ---- Post-recovery banner (audit §6) -------------------------------------
struct RecoveryBannerInfo {
    Fault    cause;               // causa
    uint32_t duration_ms;         // duración
    uint8_t  attempts;            // intentos
    bool     render_continued;    // render continuó (Core 1 never stalled)
    bool     esp32_rebooted;      // reboot ESP32 (whole-chip reset detected)
    uint32_t free_heap;           // heap
    uint32_t total_recoveries;    // recuperaciones totales
};

// Recommended action derived from the cause — an honest next step, never an
// unfounded "white screen confirmed" claim.
inline const char* recoveryActionText(const RecoveryBannerInfo& b) {
    switch (b.cause) {
    case Fault::LOW_MEMORY:
        return "revisar fugas de heap / fragmentacion";
    case Fault::ESP32_RESET:
        return "revisar alimentacion (brownout) y WDT";
    case Fault::TFT_STATUS_LOST:
        return "revisar cableado SPI/CS del panel";
    case Fault::SPI_TIMEOUT:
        return "revisar bus SPI y frecuencia";
    case Fault::READBACK_UNSUPPORTED:
        return "sin readback: confirmar visualmente";
    case Fault::RENDER_TIMEOUT:
    case Fault::TFT_RESET_PROBABLE:
    default:
        return "vigilar reincidencia; revisar GPIO38/reset";
    }
}

// Format the "PANTALLA RECUPERADA" banner into @p out (size @p n).  Returns the
// number of characters that WOULD have been written (snprintf semantics), so a
// caller can detect truncation with (ret >= n).
inline int formatRecoveryBanner(char* out, size_t n, const RecoveryBannerInfo& b) {
    if (out == nullptr || n == 0) return 0;
    return snprintf(
        out, n,
        "PANTALLA RECUPERADA\n"
        "causa: %s\n"
        "duracion: %lu ms\n"
        "intentos: %u\n"
        "render continuo: %s\n"
        "reboot ESP32: %s\n"
        "heap: %lu\n"
        "recuperaciones totales: %lu\n"
        "accion recomendada: %s",
        faultText(b.cause),
        (unsigned long)b.duration_ms,
        (unsigned)b.attempts,
        b.render_continued ? "si" : "no",
        b.esp32_rebooted   ? "si" : "no",
        (unsigned long)b.free_heap,
        (unsigned long)b.total_recoveries,
        recoveryActionText(b));
}

}  // namespace display

#endif  // DISPLAY_RECOVERY_H
