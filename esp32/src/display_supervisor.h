// display_supervisor.h — Robust TFT (ST7796) display supervisor / recovery FSM.
//
// Problem 2 of the audit: the ST7796 TFT turns completely white while the
// WS2812B LEDs keep animating, the distance sensor keeps firing, and CAN /
// the main loop stay alive.  Render + touch live on the Core-0 Render task;
// CAN, LEDs, sensors and safety live on Core 1.  So a white screen must NOT
// be blindly classified as a full ESP32 reboot.
//
// This module is the PURE decision core of the supervisor:
//   - layered detection (render heartbeat staleness with multi-sample
//     debounce so a single slow frame never trips it; TFT status-register
//     readback validity; low-memory; explicit ESP32 reset reason),
//   - an explicit recovery FSM (OK → SUSPECT → CONFIRMED_LOST →
//     RECOVERY_REQUESTED → RESETTING → REINITIALIZING → REDRAWING →
//     RECOVERED / RECOVERY_FAILED),
//   - bounded retries with backoff and persistent counters (no infinite
//     recovery loops),
//   - a captured fault reason for the "PANTALLA RECUPERADA" banner.
//
// It performs NO hardware access and NO delays, so the whole policy is
// host-testable (test_display_supervisor.cpp).  The caller owns the actual
// TFT/SPI operations (exclusive bus ownership) and drives the recovery steps
// by calling advanceRecovery()/completeRecovery() after each hardware stage.

#ifndef DISPLAY_SUPERVISOR_H
#define DISPLAY_SUPERVISOR_H

#include <cstdint>

namespace display {

// ---- Recovery FSM states (audit §2.3) ----
enum class State : uint8_t {
    OK = 0,
    SUSPECT,
    CONFIRMED_LOST,
    RECOVERY_REQUESTED,
    RESETTING,
    REINITIALIZING,
    REDRAWING,
    RECOVERED,
    RECOVERY_FAILED,
};

// ---- Captured fault reason for the banner (audit §2.4) ----
enum class Fault : uint8_t {
    NONE = 0,
    RENDER_TIMEOUT,      // Render task heartbeat stalled
    SPI_TIMEOUT,         // Bus transaction stuck
    TFT_STATUS_LOST,     // Status/ID registers no longer sane
    TFT_RESET_PROBABLE,  // Inferred controller reset (readback unavailable)
    LOW_MEMORY,          // Heap exhaustion suspected
    ESP32_RESET,         // Whole-chip reset (brownout/watchdog/panic)
    READBACK_UNSUPPORTED,// Panel cannot be read back — "white" is NOT confirmable
};

// Human-readable label for a captured fault (banner "causa:" field).  Kept
// header-only and allocation-free so it is usable from the render/Core-1
// path and from host tests alike.
inline const char* faultText(Fault f) {
    switch (f) {
    case Fault::NONE:                 return "SIN FALLO";
    case Fault::RENDER_TIMEOUT:       return "RENDER TIMEOUT";
    case Fault::SPI_TIMEOUT:          return "SPI TIMEOUT";
    case Fault::TFT_STATUS_LOST:      return "TFT STATUS PERDIDO";
    case Fault::TFT_RESET_PROBABLE:   return "TFT RESET PROBABLE";
    case Fault::LOW_MEMORY:           return "MEMORIA BAJA";
    case Fault::ESP32_RESET:          return "RESET ESP32";
    case Fault::READBACK_UNSUPPORTED: return "READBACK NO FIABLE";
    default:                          return "?";
    }
}

// Human-readable label for the recovery FSM state (diagnostics / HMI).
inline const char* stateText(State st) {
    switch (st) {
    case State::OK:                 return "OK";
    case State::SUSPECT:            return "SOSPECHA";
    case State::CONFIRMED_LOST:     return "PERDIDA CONFIRMADA";
    case State::RECOVERY_REQUESTED: return "RECUPERACION SOLICITADA";
    case State::RESETTING:          return "RESETEANDO";
    case State::REINITIALIZING:     return "REINICIALIZANDO";
    case State::REDRAWING:          return "REDIBUJANDO";
    case State::RECOVERED:          return "RECUPERADA";
    case State::RECOVERY_FAILED:    return "RECUPERACION FALLIDA";
    default:                        return "?";
    }
}

// ---- Result of the periodic status-register readback (audit §2.2 layer B) ----
enum class StatusRead : uint8_t {
    UNSUPPORTED = 0,     // Panel/TFT_eSPI cannot read back reliably
    VALID,               // Registers sane this sample
    INVALID,             // Registers implausible (0xFFFF/0x0000/bad ID)
};

struct Config {
    // A frame is "stale" if no frame completed for this long.  At 20 FPS a
    // frame is ~50 ms, so 500 ms tolerates transient slow frames.
    uint32_t render_stale_ms      = 500;
    // Consecutive bad samples to move OK→SUSPECT (reject a single slow frame).
    uint8_t  suspect_samples      = 2;
    // Consecutive bad samples (total) to CONFIRM the display is lost.
    uint8_t  confirm_samples      = 4;
    // Consecutive INVALID status reads required before trusting them.
    uint8_t  status_invalid_samples = 3;
    // Heap floor (bytes) below which LOW_MEMORY is suspected.
    uint32_t min_heap_floor       = 8192;
    // Bounded retries.
    uint8_t  max_attempts         = 3;
    // Backoff between attempts (grows linearly with attempt count).
    uint32_t backoff_base_ms      = 1000;
};

class Supervisor {
public:
    explicit Supervisor(const Config& cfg = Config()) : cfg_(cfg) {}

    // Periodic evaluation.  Call at the supervisor cadence (e.g. 10 Hz) from
    // Core 1 — never from inside the Render task.
    //   now_ms             : monotonic time
    //   render_complete_ms : timestamp of the last FULLY completed frame
    //                        (advanced only after screenManager.update(),
    //                        TFT ops, touch read and bus release)
    //   status             : latest status-register readback result
    //   free_heap          : current free heap bytes
    State update(uint32_t now_ms, uint32_t render_complete_ms,
                 StatusRead status, uint32_t free_heap);

    // Force an immediate CONFIRMED_LOST with an explicit reason (e.g. an
    // ESP32 reset detected at boot must be surfaced after reboot).
    void forceFault(uint32_t now_ms, Fault reason);

    // Caller drives the hardware recovery steps and reports completion:
    //   RECOVERY_REQUESTED -> RESETTING -> REINITIALIZING -> REDRAWING
    void advanceRecovery(uint32_t now_ms);

    // Caller finished the redraw and verified (status/readback when possible).
    //   verified_ok == true  -> RECOVERED (counters/backoff reset intent)
    //   verified_ok == false -> next attempt or RECOVERY_FAILED at the cap
    void completeRecovery(uint32_t now_ms, bool verified_ok);

    // ---- Telemetry accessors (audit §2.1) ----
    State    state()                const { return state_; }
    Fault    lastFault()            const { return last_fault_; }
    uint8_t  attempts()             const { return attempts_; }
    uint32_t recoveryCount()        const { return recovery_count_; }
    uint32_t recoveryFailCount()    const { return recovery_fail_count_; }
    // Duration between the confirmed loss and the successful recovery.
    uint32_t lastFaultDurationMs()  const { return last_fault_duration_ms_; }
    bool     recovering()           const {
        return state_ == State::RECOVERY_REQUESTED ||
               state_ == State::RESETTING ||
               state_ == State::REINITIALIZING ||
               state_ == State::REDRAWING;
    }

private:
    Fault classify(bool render_stale, StatusRead status, bool heap_low) const;
    void  enterConfirmed(uint32_t now_ms, Fault reason);
    void  requestOrFail(uint32_t now_ms);

    Config   cfg_;
    State    state_               = State::OK;
    Fault    last_fault_          = Fault::NONE;

    uint8_t  bad_samples_         = 0;
    uint8_t  status_invalid_run_  = 0;
    uint8_t  attempts_            = 0;

    uint32_t recovery_count_      = 0;
    uint32_t recovery_fail_count_ = 0;

    uint32_t confirmed_at_ms_     = 0;
    uint32_t next_attempt_ms_     = 0;
    uint32_t last_fault_duration_ms_ = 0;
    bool     forced_fault_        = false;
    Fault    forced_reason_       = Fault::NONE;
};

// ---- Implementation (header-only, like stm32_liveness.h) ----

inline Fault Supervisor::classify(bool render_stale, StatusRead status,
                                  bool heap_low) const {
    // Priority: explicit invalid status (confirmed by debounce) is the
    // strongest evidence the controller lost its state; then low memory;
    // then a stalled render heartbeat; otherwise, with no reliable readback,
    // infer a probable controller reset rather than asserting "white".
    if (status == StatusRead::INVALID) {
        return Fault::TFT_STATUS_LOST;
    }
    if (heap_low) {
        return Fault::LOW_MEMORY;
    }
    if (render_stale) {
        return (status == StatusRead::UNSUPPORTED)
                   ? Fault::TFT_RESET_PROBABLE
                   : Fault::RENDER_TIMEOUT;
    }
    return Fault::TFT_RESET_PROBABLE;
}

inline void Supervisor::enterConfirmed(uint32_t now_ms, Fault reason) {
    state_           = State::CONFIRMED_LOST;
    last_fault_      = reason;
    confirmed_at_ms_ = now_ms;
    requestOrFail(now_ms);
}

inline void Supervisor::requestOrFail(uint32_t now_ms) {
    if (attempts_ >= cfg_.max_attempts) {
        state_ = State::RECOVERY_FAILED;
        return;
    }
    // Honour backoff between attempts.
    if (next_attempt_ms_ != 0 &&
        (int32_t)(now_ms - next_attempt_ms_) < 0) {
        // Still backing off — remain CONFIRMED_LOST until the window opens.
        state_ = State::CONFIRMED_LOST;
        return;
    }
    state_ = State::RECOVERY_REQUESTED;
}

inline State Supervisor::update(uint32_t now_ms, uint32_t render_complete_ms,
                                StatusRead status, uint32_t free_heap) {
    // Do not re-evaluate detection while a recovery is in flight; the caller
    // is exclusively driving the FSM through advance/complete.
    if (recovering()) {
        return state_;
    }

    const bool render_stale =
        (int32_t)(now_ms - render_complete_ms) >= (int32_t)cfg_.render_stale_ms;
    const bool heap_low = (free_heap < cfg_.min_heap_floor);

    // Debounced status-invalid run.
    if (status == StatusRead::INVALID) {
        if (status_invalid_run_ < 0xFF) status_invalid_run_++;
    } else {
        status_invalid_run_ = 0;
    }
    const bool status_bad =
        (status_invalid_run_ >= cfg_.status_invalid_samples);

    const bool bad = render_stale || status_bad || heap_low;

    switch (state_) {
    case State::OK:
        if (bad) {
            if (bad_samples_ < 0xFF) bad_samples_++;
            if (bad_samples_ >= cfg_.suspect_samples) {
                state_ = State::SUSPECT;
            }
        } else {
            bad_samples_ = 0;
        }
        break;

    case State::SUSPECT:
        if (bad) {
            if (bad_samples_ < 0xFF) bad_samples_++;
            if (bad_samples_ >= cfg_.confirm_samples) {
                enterConfirmed(now_ms, classify(render_stale, status, heap_low));
            }
        } else {
            // Recovered on its own (was just a slow patch) — no false alarm.
            bad_samples_ = 0;
            state_ = State::OK;
        }
        break;

    case State::CONFIRMED_LOST:
        // Waiting for the backoff window to open a new attempt.
        requestOrFail(now_ms);
        break;

    case State::RECOVERED:
        // One cycle of "recovered"; if still healthy, return to OK.
        if (!bad) {
            bad_samples_ = 0;
            state_ = State::OK;
        } else {
            // Regressed immediately — treat as a fresh confirmed loss.
            enterConfirmed(now_ms, classify(render_stale, status, heap_low));
        }
        break;

    case State::RECOVERY_FAILED:
        // Terminal until the caller resets; stay put (no infinite loop).
        break;

    default:
        break;
    }

    (void)forced_fault_;
    return state_;
}

inline void Supervisor::forceFault(uint32_t now_ms, Fault reason) {
    forced_fault_  = true;
    forced_reason_ = reason;
    enterConfirmed(now_ms, reason);
}

inline void Supervisor::advanceRecovery(uint32_t now_ms) {
    (void)now_ms;
    switch (state_) {
    case State::RECOVERY_REQUESTED: state_ = State::RESETTING;      break;
    case State::RESETTING:          state_ = State::REINITIALIZING; break;
    case State::REINITIALIZING:     state_ = State::REDRAWING;      break;
    default: break;  // ignore out-of-sequence calls
    }
}

inline void Supervisor::completeRecovery(uint32_t now_ms, bool verified_ok) {
    if (state_ != State::REDRAWING) {
        return;  // completion only valid after redraw stage
    }
    if (verified_ok) {
        recovery_count_++;
        last_fault_duration_ms_ = now_ms - confirmed_at_ms_;
        attempts_       = 0;
        bad_samples_    = 0;
        status_invalid_run_ = 0;
        next_attempt_ms_ = 0;
        state_          = State::RECOVERED;
    } else {
        recovery_fail_count_++;
        attempts_++;
        // Linear backoff before the next attempt.
        next_attempt_ms_ = now_ms + (uint32_t)attempts_ * cfg_.backoff_base_ms;
        if (attempts_ >= cfg_.max_attempts) {
            state_ = State::RECOVERY_FAILED;
        } else {
            state_ = State::CONFIRMED_LOST;
        }
    }
}

}  // namespace display

#endif  // DISPLAY_SUPERVISOR_H
