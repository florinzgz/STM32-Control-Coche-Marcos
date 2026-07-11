/**
 ****************************************************************************
 * @file    mode_sync.h
 * @brief   Pure, host-testable drive-mode (4x4/2WD, tank-turn) command
 *          synchroniser with heartbeat gating, ACK and bounded retry.
 *
 *          The debounced physical 4x4 selector (traction_switch.h) is the
 *          SOURCE OF TRUTH for the drive mode.  Previously main.cpp sent a
 *          single CMD_MODE frame on a debounced change and immediately
 *          cleared the "changed" flag — if that frame was dropped (TX queue
 *          full) or never ACKed, the selection was silently lost and the
 *          STM32 stayed in the old mode with no retry.
 *
 *          This FSM decouples "what the operator selected" (desired_) from
 *          "what the STM32 has confirmed" (confirmed_) and:
 *            - refuses to transmit until the STM32 heartbeat is confirmed,
 *            - (re)transmits until an ACK arrives or a bounded retry budget
 *              is exhausted (then latches FAILED, without spamming the bus),
 *            - re-arms automatically when the operator selects a new mode or
 *              when the heartbeat drops and returns.
 *
 *          It is free of any Arduino / ESP-IDF dependency so the logic is
 *          linked directly into host unit tests (no re-implementation).
 ****************************************************************************
 */
#ifndef MODE_SYNC_H
#define MODE_SYNC_H

#include <cstdint>

class ModeSync {
public:
    /** Action the owner must perform after update(). */
    enum class Action : uint8_t {
        NONE = 0,   ///< Do nothing this tick.
        SEND = 1,   ///< Transmit CMD_MODE with sendMode() now.
    };

    /** Result of the most recent ACK observed for our command. */
    enum class AckResult : uint8_t {
        OK       = 0,   ///< STM32 accepted and applied the mode.
        REJECTED = 1,   ///< STM32 rejected/blocked (retry will not help).
    };

    /**
     * @param ackTimeoutMs  Time to wait for an ACK before retransmitting.
     * @param maxRetries    Maximum RE-transmissions after the first send
     *                      (total attempts = maxRetries + 1) before latching
     *                      FAILED.
     */
    ModeSync(uint32_t ackTimeoutMs, uint8_t maxRetries)
        : ackTimeoutMs_(ackTimeoutMs), maxRetries_(maxRetries) {}

    /**
     * Set the operator's desired mode (from the debounced selector, a tank
     * toggle, or the remote).  This is the source of truth.  A change re-arms
     * the FSM (clears FAILED) so it will be (re)synchronised to the STM32.
     */
    void setDesired(uint8_t modeFlags) {
        if (modeFlags != desired_) {
            desired_ = modeFlags;
            failed_  = false;   // new intent — allow a fresh sync attempt
        }
    }

    /**
     * Advance the FSM one tick.
     *
     * @param nowMs                Current millis().
     * @param heartbeatConfirmed   True once the STM32 heartbeat is alive.
     * @return SEND when the owner must transmit sendMode(); NONE otherwise.
     *
     * @note  When SEND is returned the FSM has already recorded the attempt
     *        (timestamp + retry accounting); the owner only performs the CAN
     *        write and calls sendMode() for the payload.
     */
    Action update(uint32_t nowMs, bool heartbeatConfirmed) {
        // Never transmit into a dead link.  If the heartbeat is not (yet)
        // confirmed, drop any in-flight attempt and re-arm so the mode is
        // resynchronised once the STM32 is back.
        if (!heartbeatConfirmed) {
            pending_ = false;
            failed_  = false;
            return Action::NONE;
        }

        // Nothing to do if the STM32 already confirmed the desired mode.
        if (desired_ == confirmed_) {
            pending_ = false;
            return Action::NONE;
        }

        if (!pending_) {
            // Start a fresh synchronisation attempt (unless we've given up).
            if (failed_) return Action::NONE;
            beginAttempt(nowMs, /*firstAttempt=*/true);
            return Action::SEND;
        }

        // An attempt is in flight — wait for ACK or retransmit on timeout.
        if ((uint32_t)(nowMs - sentMs_) < ackTimeoutMs_) {
            return Action::NONE;   // still within ACK window
        }

        if (retries_ >= maxRetries_) {
            // Exhausted the retry budget — latch FAILED, stop transmitting.
            pending_ = false;
            failed_  = true;
            return Action::NONE;
        }

        beginAttempt(nowMs, /*firstAttempt=*/false);
        return Action::SEND;
    }

    /**
     * Feed back an ACK observed for our CMD_MODE.
     * @param result  OK confirms the mode; REJECTED latches FAILED (a retry
     *                would not help, e.g. blocked by safety / speed gate).
     */
    void onAck(AckResult result) {
        if (!pending_) return;
        pending_ = false;
        if (result == AckResult::OK) {
            confirmed_ = pendingMode_;
            failed_    = false;
        } else {
            failed_ = true;   // do not spam the bus on a hard rejection
        }
    }

    /** Payload to transmit when update() returns SEND. */
    uint8_t sendMode()  const { return pendingMode_; }

    uint8_t desired()   const { return desired_; }
    uint8_t confirmed() const { return confirmed_; }
    bool    inSync()    const { return desired_ == confirmed_; }
    bool    pending()   const { return pending_; }
    bool    failed()    const { return failed_; }
    uint8_t retries()   const { return retries_; }

private:
    void beginAttempt(uint32_t nowMs, bool firstAttempt) {
        pending_     = true;
        pendingMode_ = desired_;
        sentMs_      = nowMs;
        retries_     = firstAttempt ? 0 : (uint8_t)(retries_ + 1);
    }

    const uint32_t ackTimeoutMs_;
    const uint8_t  maxRetries_;

    uint8_t  desired_     = 0;   ///< Operator-selected mode (source of truth).
    uint8_t  confirmed_   = 0;   ///< Last mode ACKed by the STM32.
    uint8_t  pendingMode_ = 0;   ///< Mode value of the in-flight attempt.
    uint32_t sentMs_      = 0;   ///< Timestamp of the current attempt.
    uint8_t  retries_     = 0;   ///< Retransmissions used in this episode.
    bool     pending_     = false;
    bool     failed_      = false;
};

#endif  // MODE_SYNC_H
