// =============================================================================
// ESP32-S3 HMI — TWAI (CAN) BUS_OFF recovery policy (pure, host-testable)
//
// The physical fault captured on the vehicle was an ESP32 TWAI oscillation:
//     BUS_OFF → RECOVERING → STOPPED → RUNNING → BUS_OFF → …
//
// The naive recovery (reset the attempt counter as soon as the controller
// reports RUNNING, or as soon as STOPPED→RUNNING completes) treats a bus that
// merely *reports* RUNNING as healthy — even though the very next frame drives
// it straight back to BUS_OFF.  It also gave up permanently after a fixed
// number of attempts, so a transient physical disconnection left the node dead.
//
// This module mirrors the STM32 FDCAN "stable-heartbeat" policy
// (Core/Src/busoff_recovery.c):
//   - RUNNING alone does NOT confirm recovery and does NOT reset counters;
//   - recovery is confirmed only after ADVANCING STM32 heartbeats are sustained
//     continuously for a stable window (a single heartbeat gap restarts it);
//   - the lifetime BUS_OFF event count is preserved across recoveries;
//   - retries are FAST and bounded first, then fall back to a SLOW unbounded
//     backoff so the node ALWAYS recovers after a physical reconnection.
//
// Pure logic: no ESP-IDF / Arduino dependency, unit-tested on the host by
// esp32/src/test_twai_recovery.cpp.  main.cpp maps twai_status_info_t.state to
// TwaiState, feeds the STM32 heartbeat-alive flag, and performs the returned
// TWAI action (twai_initiate_recovery / twai_start).
// =============================================================================

#ifndef TWAI_RECOVERY_H
#define TWAI_RECOVERY_H

#include <cstdint>

namespace twai_recovery {

/// Controller state, mirrors the ESP-IDF twai_state_t values we care about.
enum class TwaiState : uint8_t {
    RUNNING    = 0,  ///< Controller running (may still be error-passive).
    BUS_OFF    = 1,  ///< TEC saturated — bus-off, needs recovery.
    RECOVERING = 2,  ///< Recovery in progress (128×11 recessive bits).
    STOPPED    = 3,  ///< Recovery completed — driver stopped, needs start.
};

/// Action the owner must perform after update().
enum class TwaiAction : uint8_t {
    NONE              = 0,  ///< Do nothing this tick.
    INITIATE_RECOVERY = 1,  ///< Call twai_initiate_recovery() (from BUS_OFF).
    START             = 2,  ///< Call twai_start() (from STOPPED).
    CONFIRMED         = 3,  ///< Recovery just confirmed by sustained heartbeats.
};

/// Default recovery pacing — chosen to match the STM32 side where sensible.
inline constexpr uint32_t DEFAULT_FAST_INTERVAL_MS = 500;    ///< bounded fast retries
inline constexpr uint32_t DEFAULT_SLOW_INTERVAL_MS = 5000;   ///< unbounded slow backoff
inline constexpr uint8_t  DEFAULT_MAX_FAST         = 10;     ///< fast attempts before backoff
inline constexpr uint32_t DEFAULT_STABLE_WINDOW_MS = 1000;   ///< sustained-heartbeat window

/**
 * Stable-heartbeat TWAI BUS_OFF recovery state machine.
 *
 * Feed it the observed controller state and whether the STM32 heartbeat is
 * currently alive (advancing aliveCounter + not absent); it returns the single
 * action to perform this tick.  All time comparisons use unsigned modular
 * arithmetic and are therefore wrap-safe across the millis() 32-bit rollover.
 */
class TwaiBusOffRecovery {
public:
    TwaiBusOffRecovery(uint32_t fastIntervalMs = DEFAULT_FAST_INTERVAL_MS,
                       uint32_t slowIntervalMs = DEFAULT_SLOW_INTERVAL_MS,
                       uint8_t  maxFast        = DEFAULT_MAX_FAST,
                       uint32_t stableWindowMs = DEFAULT_STABLE_WINDOW_MS)
        : fastIntervalMs_(fastIntervalMs),
          slowIntervalMs_(slowIntervalMs),
          maxFast_(maxFast),
          stableWindowMs_(stableWindowMs) {}

    /**
     * Advance the recovery FSM one tick.
     *
     * @param nowMs          Current millis().
     * @param observed       Controller state read from twai_get_status_info().
     * @param heartbeatAlive True when the STM32 heartbeat is alive (advancing
     *                       counter, not timed-out).  Only a sustained-alive
     *                       streak confirms recovery.
     * @return The action the owner must perform (see TwaiAction).
     */
    TwaiAction update(uint32_t nowMs, TwaiState observed, bool heartbeatAlive) {
        // Count lifetime BUS_OFF *events* on the rising edge into BUS_OFF only
        // (staying in BUS_OFF for many ticks is a single event).
        const bool nowBusOff = (observed == TwaiState::BUS_OFF);
        if (nowBusOff && !prevBusOff_) {
            ++lifetimeBusOffCount_;
        }
        prevBusOff_ = nowBusOff;

        switch (observed) {
            case TwaiState::BUS_OFF:
                // A bus-off episode is active; drop any probation streak.
                episodeActive_ = true;
                probation_     = false;
                streak_        = false;
                return paceRecovery(nowMs);

            case TwaiState::RECOVERING:
                // Controller is clearing the fault — just wait.
                probation_ = false;
                streak_    = false;
                return TwaiAction::NONE;

            case TwaiState::STOPPED:
                // Recovery finished; restart the driver.  Do NOT reset the
                // attempt counter here — recovery is not yet proven healthy.
                probation_ = false;
                streak_    = false;
                return episodeActive_ ? TwaiAction::START : TwaiAction::NONE;

            case TwaiState::RUNNING:
            default:
                if (!episodeActive_) {
                    // Nominal healthy running — no recovery in flight.
                    return TwaiAction::NONE;
                }
                // RUNNING after a bus-off episode is only PROBATION: the bus is
                // not trusted until sustained advancing heartbeats prove it.
                probation_ = true;
                if (heartbeatAlive) {
                    if (!streak_) {
                        streak_        = true;
                        streakStartMs_ = nowMs;
                    }
                    if ((uint32_t)(nowMs - streakStartMs_) >= stableWindowMs_) {
                        // Confirmed — reset per-episode state, keep lifetime.
                        episodeActive_ = false;
                        probation_     = false;
                        streak_        = false;
                        attempts_      = 0;
                        ++confirmedRecoveries_;
                        return TwaiAction::CONFIRMED;
                    }
                } else {
                    // Heartbeat gap — the bus is not proven healthy; restart
                    // the window from scratch when heartbeats return.
                    streak_ = false;
                }
                return TwaiAction::NONE;
        }
    }

    // ---- Diagnostics --------------------------------------------------------
    uint32_t lifetimeBusOffCount() const { return lifetimeBusOffCount_; }
    uint32_t confirmedRecoveries() const { return confirmedRecoveries_; }
    uint8_t  attempts()            const { return attempts_; }
    bool     episodeActive()       const { return episodeActive_; }
    bool     inProbation()         const { return probation_; }

private:
    // Pace twai_initiate_recovery(): first maxFast_ attempts at the fast
    // interval, then unbounded attempts at the slow interval (so the node
    // always recovers after a physical reconnection).
    TwaiAction paceRecovery(uint32_t nowMs) {
        if (attempts_ == 0) {
            attempts_      = 1;
            lastAttemptMs_ = nowMs;
            return TwaiAction::INITIATE_RECOVERY;
        }
        const uint32_t interval =
            (attempts_ < maxFast_) ? fastIntervalMs_ : slowIntervalMs_;
        if ((uint32_t)(nowMs - lastAttemptMs_) >= interval) {
            if (attempts_ < 0xFFu) ++attempts_;
            lastAttemptMs_ = nowMs;
            return TwaiAction::INITIATE_RECOVERY;
        }
        return TwaiAction::NONE;
    }

    const uint32_t fastIntervalMs_;
    const uint32_t slowIntervalMs_;
    const uint8_t  maxFast_;
    const uint32_t stableWindowMs_;

    uint32_t lifetimeBusOffCount_ = 0;
    uint32_t confirmedRecoveries_ = 0;
    uint32_t lastAttemptMs_       = 0;
    uint32_t streakStartMs_       = 0;
    uint8_t  attempts_            = 0;
    bool     episodeActive_       = false;
    bool     probation_           = false;
    bool     streak_              = false;
    bool     prevBusOff_          = false;
};

}  // namespace twai_recovery

#endif  // TWAI_RECOVERY_H
