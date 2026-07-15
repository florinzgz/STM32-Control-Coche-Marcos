#ifndef CAN_HEARTBEAT_GUARD_POLICY_H
#define CAN_HEARTBEAT_GUARD_POLICY_H

#include <cstdint>

namespace can_heartbeat {

/*
 * Pure scheduling policy for the ESP32 -> STM32 safety-heartbeat backup.
 *
 * The legacy/main-loop producer remains the normal single producer.  This
 * policy only permits a backup 0x011 when the main loop has stopped kicking
 * for long enough, or after a TX-drop/congestion event.  That avoids two
 * independent counters transmitting continuously while still protecting the
 * STM32 250 ms watchdog from NVS/LED/audio/main-loop stalls.
 *
 * All time arithmetic uses unsigned subtraction, so millis() rollover is
 * handled naturally.
 */
struct GuardConfig {
    uint32_t loopStallMs       = 120U;
    uint32_t retryIntervalMs   = 10U;
    uint32_t backupIntervalMs  = 80U;
    uint8_t  queueHighWater    = 4U;
    uint8_t  queueRecovered    = 2U;
};

struct GuardStats {
    uint32_t loopStallEvents       = 0U;
    uint32_t congestionEvents      = 0U;
    uint32_t backupAttempts        = 0U;
    uint32_t backupSuccess         = 0U;
    uint32_t backupFailures        = 0U;
    uint32_t retries               = 0U;
    uint32_t skippedNotRunning     = 0U;
    uint32_t maxObservedLoopGapMs  = 0U;
    uint32_t lastBackupSuccessMs   = 0U;
    uint32_t lastBackupFailureMs   = 0U;
};

class GuardPolicy {
public:
    explicit GuardPolicy(const GuardConfig& cfg = GuardConfig{}) : cfg_(cfg) {}

    void reset(uint32_t nowMs) {
        lastLoopKickMs_ = nowMs;
        lastAttemptMs_ = nowMs;
        queueDepth_ = 0U;
        congestionLatched_ = false;
        stallLatched_ = false;
        retryPending_ = false;
        stats_ = GuardStats{};
    }

    void notifyLoopAlive(uint32_t nowMs) {
        const uint32_t gap = nowMs - lastLoopKickMs_;
        if (gap > stats_.maxObservedLoopGapMs) {
            stats_.maxObservedLoopGapMs = gap;
        }
        lastLoopKickMs_ = nowMs;
        stallLatched_ = false;
    }

    void observeQueue(uint8_t depth) {
        queueDepth_ = depth;
        if (depth >= cfg_.queueHighWater && !congestionLatched_) {
            congestionLatched_ = true;
            ++stats_.congestionEvents;
        }
    }

    void notifyTxDrop() {
        if (!congestionLatched_) {
            congestionLatched_ = true;
            ++stats_.congestionEvents;
        }
    }

    void noteSkippedNotRunning() { ++stats_.skippedNotRunning; }

    bool shouldAttempt(uint32_t nowMs) {
        const uint32_t loopGap = nowMs - lastLoopKickMs_;
        if (loopGap > stats_.maxObservedLoopGapMs) {
            stats_.maxObservedLoopGapMs = loopGap;
        }

        const bool stalled = loopGap >= cfg_.loopStallMs;
        if (stalled && !stallLatched_) {
            stallLatched_ = true;
            ++stats_.loopStallEvents;
        }

        const bool queueRecovered = congestionLatched_ &&
                                    queueDepth_ <= cfg_.queueRecovered;
        if (!stalled && !queueRecovered && !retryPending_) {
            return false;
        }

        const uint32_t minGap = retryPending_ ? cfg_.retryIntervalMs
                                               : cfg_.backupIntervalMs;
        return (nowMs - lastAttemptMs_) >= minGap;
    }

    void noteAttempt(uint32_t nowMs, bool success) {
        lastAttemptMs_ = nowMs;
        ++stats_.backupAttempts;

        if (success) {
            ++stats_.backupSuccess;
            stats_.lastBackupSuccessMs = nowMs;
            congestionLatched_ = false;
            retryPending_ = false;
        } else {
            ++stats_.backupFailures;
            stats_.lastBackupFailureMs = nowMs;
            if (!retryPending_) {
                ++stats_.retries;
            }
            retryPending_ = true;
        }
    }

    const GuardStats& stats() const { return stats_; }
    uint32_t loopAge(uint32_t nowMs) const { return nowMs - lastLoopKickMs_; }
    bool congestionLatched() const { return congestionLatched_; }
    bool retryPending() const { return retryPending_; }

private:
    GuardConfig cfg_{};
    GuardStats stats_{};
    uint32_t lastLoopKickMs_ = 0U;
    uint32_t lastAttemptMs_ = 0U;
    uint8_t queueDepth_ = 0U;
    bool congestionLatched_ = false;
    bool stallLatched_ = false;
    bool retryPending_ = false;
};

}  // namespace can_heartbeat

#endif  // CAN_HEARTBEAT_GUARD_POLICY_H
