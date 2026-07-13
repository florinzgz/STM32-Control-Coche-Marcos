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

// ---- Tristate recovery verification result (audit P2.3) ------------------
// The current hardware has NO validated TFT readback (readTftStatus never
// yields INVALID and g_tftResetSignalLatched has no real producer), so a
// recovery can rarely be positively confirmed.  The result is therefore a
// tristate that is transported honestly to the banner/log — we never claim a
// white screen was fixed when we cannot observe it.
enum class RecoveryResult : uint8_t {
    VERIFIED = 0,                     // real readback confirmed the panel
    UNVERIFIED_READBACK_UNSUPPORTED,  // best-effort: no validated readback
    FAILED,                           // readback available and implausible
};

inline const char* recoveryResultText(RecoveryResult r) {
    switch (r) {
    case RecoveryResult::VERIFIED:                        return "VERIFICADO";
    case RecoveryResult::UNVERIFIED_READBACK_UNSUPPORTED: return "NO VERIFICADO (sin readback)";
    case RecoveryResult::FAILED:                          return "FALLIDO";
    default:                                              return "?";
    }
}

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
    // pending or a recovery is in flight, so requests never stack up.  `now` is
    // latched so Core 1 can later detect a render task that never consumes the
    // request (audit P2.1 — the "Render realmente bloqueado" case).
    bool request(RecoveryTrigger t, uint32_t now = 0) {
        if (t == RecoveryTrigger::NONE) return false;
        if (pending_ != RecoveryTrigger::NONE || busy_) return false;
        pending_  = t;
        mark_ms_  = now;
        return true;
    }

    // Core 0 takes the pending request (clears it and marks a recovery busy).
    bool take(RecoveryTrigger& out, uint32_t now = 0) {
        if (pending_ == RecoveryTrigger::NONE) return false;
        out      = pending_;
        pending_ = RecoveryTrigger::NONE;
        busy_    = true;
        mark_ms_ = now;
        return true;
    }

    // Core 0 reports the hardware recovery finished, carrying the tristate
    // verification result (audit P2.3).  The result is latched for Core 1 to
    // finalise the supervisor FSM and assemble the banner.
    void done(RecoveryResult result, uint32_t duration_ms) {
        busy_          = false;
        result_ready_  = true;
        result_        = result;
        duration_ms_   = duration_ms;
    }

    // Core 1 consumes the result exactly once.
    bool takeResult(RecoveryResult& result, uint32_t& duration_ms) {
        if (!result_ready_) return false;
        result_ready_ = false;
        result        = result_;
        duration_ms   = duration_ms_;
        return true;
    }

    bool pending() const { return pending_ != RecoveryTrigger::NONE; }
    bool busy()    const { return busy_; }

    // Audit P2.1 — "Render realmente bloqueado".  Core 1 asks whether the render
    // task has failed to make progress on an outstanding recovery within a
    // bounded timeout.  Returns true when a request is still pending (render
    // never take()d it) OR a recovery has been busy (render take()d it but never
    // done()d it) for longer than `timeout_ms`.  Core 1 must treat this as the
    // last-resort trigger to persist the cause and reboot the ESP32 — it must
    // NEVER touch the TFT/SPI itself while render may be blocked.
    bool renderStalled(uint32_t now, uint32_t timeout_ms) const {
        if (pending_ == RecoveryTrigger::NONE && !busy_) return false;
        return (uint32_t)(now - mark_ms_) >= timeout_ms;
    }

private:
    RecoveryTrigger pending_      = RecoveryTrigger::NONE;
    bool            busy_         = false;
    bool            result_ready_ = false;
    RecoveryResult  result_       = RecoveryResult::FAILED;
    uint32_t        duration_ms_  = 0;
    uint32_t        mark_ms_      = 0;   // when pending/busy was last (re)armed
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

// ---- Pure recovery runner (audit §4 order + §6 retries/final failure) -----
// Executes the canonical recoverySequence() through an injected set of hardware
// callbacks so the exact ORDER of hardware calls, the retry policy and the
// final-failure outcome are all host-testable with mocks (test_display_recovery
// .cpp), while the firmware (main.cpp) runs the very same logic with the real
// TFT/GPIO ops.  Non-capturing function pointers (+ void* ctx) keep it
// header-only and Arduino-friendly.  No hardware access, no delays here.
struct RecoveryOps {
    void (*stopRenderTouch)(void*);
    void (*blockBus)(void*);
    void (*tftCsHigh)(void*);
    void (*touchCsHigh)(void*);
    void (*closeSpi)(void*);
    void (*pulseReset)(void*);
    void (*tftInit)(void*);
    void (*setRotation)(void*);
    void (*restoreTouchCal)(void*);
    void (*restoreBacklight)(void*);
    void (*invalidateCaches)(void*);
    void (*forceFullRedraw)(void*);
    // Tristate verify (audit P2.3): VERIFIED (real readback confirmed the
    // panel), UNVERIFIED_READBACK_UNSUPPORTED (best-effort — the hardware has
    // no validated readback so the white screen is NOT observable and we must
    // not claim success), or FAILED (readback available and implausible).
    RecoveryResult (*verify)(void*);
    void (*resumeRenderTouch)(void*);
    void (*onStep)(void*, RecoveryStep);  // optional recorder; may be nullptr
};

struct RecoveryOutcome {
    RecoveryResult result = RecoveryResult::FAILED;  // tristate verify result
    bool    verified  = false;  // convenience: result == VERIFIED
    uint8_t attempts  = 0;      // reset→verify cycles actually performed
    bool    exhausted = false;  // true only when every attempt FAILED verify()
};

namespace detail {
inline void recStep(const RecoveryOps& ops, void* ctx,
                    void (*fn)(void*), RecoveryStep s) {
    if (ops.onStep) ops.onStep(ctx, s);
    if (fn)         fn(ctx);
}
}  // namespace detail

// Run the full choreography.  The one-time teardown (stop render/touch, claim
// the bus, deselect both CS, close SPI) runs once; the reset→init→redraw→verify
// cycle repeats up to @p maxAttempts.  A FAILED verify (implausible readback)
// retries; VERIFIED or UNVERIFIED_READBACK_UNSUPPORTED is terminal (retrying a
// panel with no readback cannot change the outcome).  render/touch is ALWAYS
// resumed at the end, even after a final failure (degraded but alive).
inline RecoveryOutcome runRecovery(const RecoveryOps& ops, void* ctx,
                                   uint8_t maxAttempts) {
    if (maxAttempts == 0) maxAttempts = 1;
    RecoveryOutcome out;

    detail::recStep(ops, ctx, ops.stopRenderTouch, RecoveryStep::STOP_RENDER_TOUCH);
    detail::recStep(ops, ctx, ops.blockBus,        RecoveryStep::BLOCK_TFT_ACCESS);
    detail::recStep(ops, ctx, ops.tftCsHigh,       RecoveryStep::TFT_CS_HIGH);
    detail::recStep(ops, ctx, ops.touchCsHigh,     RecoveryStep::TOUCH_CS_HIGH);
    detail::recStep(ops, ctx, ops.closeSpi,        RecoveryStep::CLOSE_SPI);

    RecoveryResult res = RecoveryResult::FAILED;
    for (uint8_t a = 0; a < maxAttempts; ++a) {
        out.attempts = static_cast<uint8_t>(a + 1);
        detail::recStep(ops, ctx, ops.pulseReset,       RecoveryStep::PULSE_GPIO38);
        detail::recStep(ops, ctx, ops.tftInit,          RecoveryStep::TFT_INIT);
        detail::recStep(ops, ctx, ops.setRotation,      RecoveryStep::SET_ROTATION_1);
        detail::recStep(ops, ctx, ops.restoreTouchCal,  RecoveryStep::RESTORE_TOUCH_CAL);
        detail::recStep(ops, ctx, ops.restoreBacklight, RecoveryStep::RESTORE_BACKLIGHT);
        detail::recStep(ops, ctx, ops.invalidateCaches, RecoveryStep::INVALIDATE_CACHES);
        detail::recStep(ops, ctx, ops.forceFullRedraw,  RecoveryStep::FORCE_FULL_REDRAW);
        if (ops.onStep) ops.onStep(ctx, RecoveryStep::VERIFY);
        res = ops.verify ? ops.verify(ctx)
                         : RecoveryResult::UNVERIFIED_READBACK_UNSUPPORTED;
        if (res != RecoveryResult::FAILED) break;
    }
    out.result    = res;
    out.verified  = (res == RecoveryResult::VERIFIED);
    out.exhausted = (res == RecoveryResult::FAILED);

    detail::recStep(ops, ctx, ops.resumeRenderTouch, RecoveryStep::RESUME);
    return out;
}

// ---- Post-recovery banner (audit §6) -------------------------------------
struct RecoveryBannerInfo {
    Fault    cause;               // causa
    RecoveryTrigger trigger;      // trigger (qué disparó la recuperación)
    uint32_t duration_ms;         // duración
    uint8_t  attempts;            // intentos
    RecoveryResult verify_result; // resultado de verificación (triestado, P2.3)
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
        "trigger: %s\n"
        "duracion: %lu ms\n"
        "intentos: %u\n"
        "verificacion: %s\n"
        "render continuo: %s\n"
        "reboot ESP32: %s\n"
        "heap: %lu\n"
        "recuperaciones totales: %lu\n"
        "accion recomendada: %s",
        faultText(b.cause),
        recoveryTriggerText(b.trigger),
        (unsigned long)b.duration_ms,
        (unsigned)b.attempts,
        recoveryResultText(b.verify_result),
        b.render_continued ? "si" : "no",
        b.esp32_rebooted   ? "si" : "no",
        (unsigned long)b.free_heap,
        (unsigned long)b.total_recoveries,
        recoveryActionText(b));
}

// ---- Persisted render-blocked reboot banner (audit P2 final) -------------
// When the render task is genuinely blocked, Core 1 persists the cause to NVS
// and performs a CONTROLLED ESP32 restart (persistRenderBlockedAndReboot).
// The NEXT boot must surface that fact ON THE HMI exactly once — not only on
// the serial log — so the operator sees why the display restarted.  The
// firmware persists:
//   rb_cause   : the RecoveryTrigger that led to the reboot
//   rb_uptime  : millis() at the moment of the failure
//   rb_count   : historical counter of render-blocked reboots (never cleared)
//   rb_pending : true while a persisted reboot still needs to be shown
// After the banner is published the firmware clears ONLY rb_pending, keeping
// rb_count as the historical counter, so later normal boots do NOT repeat it.
struct RebootRecoveryInfo {
    RecoveryTrigger trigger;    // rb_cause  — what triggered the render block
    uint32_t        uptime_ms;  // rb_uptime — moment of the failure
    uint32_t        count;      // rb_count  — historical reincidences
};

// The persisted reboot banner must be shown exactly once: only when the pending
// flag is set AND at least one reboot has been recorded.  A normal boot
// (rb_pending == false) shows nothing; a boot right after the fault
// (rb_pending == true) shows it once and then the firmware clears rb_pending.
inline bool shouldShowRebootBanner(bool rb_pending, uint32_t rb_count) {
    return rb_pending && (rb_count > 0U);
}

// Format the "PANTALLA RECUPERADA TRAS REINICIO" banner into @p out (size @p n)
// using snprintf, so the buffer bound is explicit and testable (audit P6).
// Returns the number of characters that WOULD have been written (snprintf
// semantics) so a caller can detect truncation with (ret >= n).
inline int formatRebootRecoveryBanner(char* out, size_t n,
                                      const RebootRecoveryInfo& b) {
    if (out == nullptr || n == 0) return 0;
    return snprintf(
        out, n,
        "PANTALLA RECUPERADA TRAS REINICIO\n"
        "\n"
        "Causa: RENDER BLOQUEADO\n"
        "Trigger: %s\n"
        "Reinicio: CONTROLADO\n"
        "Momento del fallo: %lu ms\n"
        "Reincidencias: %lu\n"
        "Verificacion: REINICIO COMPLETO\n"
        "Accion:\n"
        "revisar TFT_RST, alimentacion, SPI y tarea Render",
        recoveryTriggerText(b.trigger),
        (unsigned long)b.uptime_ms,
        (unsigned long)b.count);
}

}  // namespace display

#endif  // DISPLAY_RECOVERY_H
