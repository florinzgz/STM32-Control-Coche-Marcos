#ifndef CAN_HEARTBEAT_GUARD_POLICY_H
#define CAN_HEARTBEAT_GUARD_POLICY_H

#include <cstdint>

namespace can_heartbeat {

/*
 * Pure scheduling policy for the dedicated ESP32 -> STM32 safety heartbeat.
 *
 * The producer is periodic and independent from Arduino loop().  Unsigned
 * subtraction is used everywhere, so millis() rollover is handled naturally.
 */
struct GuardConfig {
    uint32_t periodMs        = 100U;
    uint32_t retryIntervalMs = 10U;
    uint32_t warningGapMs    = 150U;
};

struct GuardStats {
    uint32_t attempts              = 0U;
    uint32_t successes             = 0U;
    uint32_t failures              = 0U;
    uint32_t retries               = 0U;
    uint32_t queueFullObservations = 0U;
    uint32_t txDropNotifications   = 0U;
    uint32_t skippedNotRunning     = 0U;
    uint32_t warningGapEvents      = 0U;
    uint32_t lastSuccessMs         = 0U;
    uint32_t lastFailureMs         = 0U;
    uint32_t maxSuccessGapMs       = 0U;
    uint32_t maxObservedLoopGapMs  = 0U;
};

class GuardPolicy {
public:
    explicit GuardPolicy(const GuardConfig& cfg = GuardConfig{}) : cfg_(cfg) {}

    void reset(uint32_t nowMs) {
        anchorMs_ = nowMs;
        lastAttemptMs_ = nowMs;
        lastSuccessMs_ = nowMs;
        lastLoopKickMs_ = nowMs;
        retryPending_ = false;
        stats_ = GuardStats{};
        stats_.lastSuccessMs = nowMs;
    }

    void notifyLoopAlive(uint32_t nowMs) {
        const uint32_t gap = nowMs - lastLoopKickMs_;
        if (gap > stats_.maxObservedLoopGapMs) {
            stats_.maxObservedLoopGapMs = gap;
        }
        lastLoopKickMs_ = nowMs;
    }

    void notifyTxDrop() { ++stats_.txDropNotifications; }
    void noteQueueFull() { ++stats_.queueFullObservations; }
    void noteSkippedNotRunning() { ++stats_.skippedNotRunning; }

    bool shouldAttempt(uint32_t nowMs) const {
        if (retryPending_) {
            return (nowMs - lastAttemptMs_) >= cfg_.retryIntervalMs;
        }
        return (nowMs - anchorMs_) >= cfg_.periodMs;
    }

    void noteAttempt(uint32_t nowMs, bool success) {
        lastAttemptMs_ = nowMs;
        ++stats_.attempts;

        if (success) {
            const uint32_t gap = nowMs - lastSuccessMs_;
            ++stats_.successes;
            stats_.lastSuccessMs = nowMs;
            if (gap > stats_.maxSuccessGapMs) {
                stats_.maxSuccessGapMs = gap;
            }
            if (gap > cfg_.warningGapMs) {
                ++stats_.warningGapEvents;
            }
            lastSuccessMs_ = nowMs;
            retryPending_ = false;

            /* Absolute cadence: advance the schedule by whole periods instead
             * of setting anchor=now.  This prevents long-term drift while also
             * collapsing missed periods after a temporary controller outage. */
            do {
                anchorMs_ += cfg_.periodMs;
            } while ((nowMs - anchorMs_) >= cfg_.periodMs);
        } else {
            ++stats_.failures;
            stats_.lastFailureMs = nowMs;
            if (!retryPending_) {
                ++stats_.retries;
            }
            retryPending_ = true;
        }
    }

    const GuardStats& stats() const { return stats_; }
    uint32_t currentSuccessGapMs(uint32_t nowMs) const {
        return nowMs - lastSuccessMs_;
    }
    bool retryPending() const { return retryPending_; }

private:
    GuardConfig cfg_{};
    GuardStats stats_{};
    uint32_t anchorMs_ = 0U;
    uint32_t lastAttemptMs_ = 0U;
    uint32_t lastSuccessMs_ = 0U;
    uint32_t lastLoopKickMs_ = 0U;
    bool retryPending_ = false;
};

}  // namespace can_heartbeat

#endif  // CAN_HEARTBEAT_GUARD_POLICY_H
