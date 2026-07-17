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
 *          Remote-confirmation validity (confirmedValid_): confirmed_ is only
 *          trusted while the link is proven.  Heartbeat loss and an explicit
 *          STM32 restart (startup_inhibit rising edge, invalidateConfirmed())
 *          both INVALIDATE the remote confirmation, so an already-confirmed
 *          mode is RE-TRANSMITTED after the STM32 restarts — the bug where a
 *          confirmed 4x4 was never resent because desired_ == confirmed_ still
 *          held numerically.  The STM32 heartbeat mode echo is also accepted as
 *          confirmation (onHeartbeatModeEcho()), so the mode still confirms
 *          even if the CMD_ACK for our CMD_MODE was lost.  The physical
 *          selector remains the desired source of truth throughout; requested
 *          (desired_) and confirmed (confirmed_) stay separate.
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

    /**
     * Result of the most recent ACK observed for our command.  Mirrors the
     * four CAN-level CMD_ACK codes (can::AckResult) so the owner maps them
     * one-to-one.  Only INVALID is a HARD failure that latches FAILED — a
     * REJECTED (speed/condition gate) and a BLOCKED_BY_SAFETY (STM32 not yet
     * ACTIVE, e.g. during boot/STANDBY) are TEMPORARY: the request is kept
     * pending and retried at a bounded, non-flooding cadence until the STM32
     * accepts it.  Previously every non-OK ACK collapsed to a hard FAILED,
     * which is why a boot in STANDBY (ACK_BLOCKED_BY_SAFETY) permanently lost
     * the selected mode until the operator physically toggled the selector.
     */
    enum class AckResult : uint8_t {
        OK       = 0,   ///< STM32 accepted and applied the mode.
        REJECTED = 1,   ///< Temporary refusal (speed/condition gate) — retry.
        INVALID  = 2,   ///< Hard payload/format error — latch FAILED (no retry).
        BLOCKED  = 3,   ///< Temporary safety block (e.g. STANDBY) — retry.
    };

    /**
     * @param ackTimeoutMs      Time to wait for an ACK before retransmitting a
     *                          lost frame.
     * @param maxRetries        Maximum RE-transmissions after the first send
     *                          (total attempts = maxRetries + 1) on the
     *                          NO-RESPONSE path before latching FAILED.  A
     *                          TEMPORARY ACK (REJECTED / BLOCKED) does NOT
     *                          consume this budget.
     * @param blockedBackoffMs  Cooldown before re-sending after a TEMPORARY
     *                          ACK, so the request stays pending indefinitely
     *                          (until the STM32 becomes ACTIVE) without
     *                          flooding the bus.
     */
    ModeSync(uint32_t ackTimeoutMs, uint8_t maxRetries,
             uint32_t blockedBackoffMs = 1000)
        : ackTimeoutMs_(ackTimeoutMs), maxRetries_(maxRetries),
          blockedBackoffMs_(blockedBackoffMs) {}

    /**
     * Wrap-safe test of whether an ACK (or echo) timestamped @p ackTs arrived
     * at or after we transmitted at @p sendTs.  Uses signed modular arithmetic
     * so it stays correct across the millis() 32-bit rollover — a plain
     * `ackTs >= sendTs` comparison silently mis-fires when the timer wraps
     * between the send and the ACK.
     */
    static bool ackIsAtOrAfterSend(uint32_t ackTs, uint32_t sendTs) {
        return (int32_t)(ackTs - sendTs) >= 0;
    }

    /**
     * Set the operator's desired mode (from the debounced selector, a tank
     * toggle, or the remote).  This is the source of truth.  A change re-arms
     * the FSM (clears FAILED) so it will be (re)synchronised to the STM32.
     */
    void setDesired(uint8_t modeFlags) {
        if (modeFlags != desired_) {
            desired_ = modeFlags;
            failed_  = false;   // new intent — allow a fresh sync attempt
            blocked_ = false;   // and retry immediately, not after the cooldown
            episodeArmed_ = false;  // §4: a new intent starts a fresh age timer
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
        // Never transmit into a dead link.  A heartbeat loss also INVALIDATES
        // the remote confirmation: we can no longer assume the STM32 still has
        // our mode (it may have restarted), so it must be resynchronised once
        // the heartbeat returns.
        if (!heartbeatConfirmed) {
            pending_        = false;
            failed_         = false;
            blocked_        = false;
            blockedArmed_   = false;
            confirmedValid_ = false;
            return Action::NONE;
        }

        // Nothing to do only if the STM32 confirmation is still VALID and it
        // matches the desired mode.  An invalidated confirmation (restart /
        // link loss) forces a resend even when desired_ == confirmed_.
        if (confirmedValid_ && desired_ == confirmed_) {
            pending_      = false;
            blocked_      = false;
            blockedArmed_ = false;
            episodeArmed_ = false;   // §4: in sync — next miss restarts the timer
            return Action::NONE;
        }

        // An attempt is in flight — wait for ACK or retransmit on timeout.
        // This NO-RESPONSE (lost frame) path is bounded by maxRetries_.
        if (pending_) {
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

        // Not pending and not in sync.
        if (failed_) return Action::NONE;

        // A TEMPORARY ACK (REJECTED / BLOCKED_BY_SAFETY) keeps the request
        // pending: wait a bounded cooldown, then retransmit.  This never
        // latches FAILED, so a mode selected while the STM32 is still in
        // BOOT/STANDBY is applied automatically once it becomes ACTIVE —
        // without the operator having to move the physical selector.  The
        // cooldown paces the retries so the bus is never flooded.
        if (blocked_) {
            if (!blockedArmed_) {          // lazily start the cooldown timer
                blockedAtMs_  = nowMs;
                blockedArmed_ = true;
                return Action::NONE;
            }
            if ((uint32_t)(nowMs - blockedAtMs_) < blockedBackoffMs_) {
                return Action::NONE;       // still cooling down
            }
            blocked_      = false;
            blockedArmed_ = false;
            beginAttempt(nowMs, /*firstAttempt=*/true);
            return Action::SEND;
        }

        // Start a fresh synchronisation attempt.
        beginAttempt(nowMs, /*firstAttempt=*/true);
        return Action::SEND;
    }

    /**
     * Feed back an ACK observed for our CMD_MODE.
     * @param result  OK       → confirms the mode.
     *                INVALID  → HARD failure: latch FAILED (retry cannot help).
     *                REJECTED → TEMPORARY refusal (speed/condition gate): keep
     *                           pending and retry after the cooldown.
     *                BLOCKED  → TEMPORARY safety block (e.g. STM32 in STANDBY
     *                           during boot): keep pending and retry after the
     *                           cooldown.  This is the key fix for the boot
     *                           4x2/4x4 desync — an ACK_BLOCKED_BY_SAFETY no
     *                           longer permanently gives up.
     */
    void onAck(AckResult result) {
        if (!pending_) return;
        pending_ = false;
        lastAck_    = result;   // §4 diagnostics — record every observed ACK
        hasLastAck_ = true;
        switch (result) {
            case AckResult::OK:
                confirmed_      = pendingMode_;
                confirmedValid_ = true;
                failed_         = false;
                blocked_        = false;
                blockedArmed_   = false;
                break;
            case AckResult::INVALID:
                failed_       = true;    // hard error — do not spam the bus
                blocked_      = false;
                blockedArmed_ = false;
                break;
            case AckResult::REJECTED:
            case AckResult::BLOCKED:
            default:
                // Temporary — keep the request pending and retry after a
                // bounded cooldown (armed in update()); never latch FAILED.
                // §4: a permanent temporary-block never latches FAILED (the
                // boot fix depends on that), but it IS made visible through the
                // block counter / reason / request age below, so an operator
                // can tell a stuck REJECTED apart from a healthy in-sync mode.
                blocked_      = true;
                blockedArmed_ = false;
                failed_       = false;
                blockReason_  = result;
                if (blockCount_ != 0xFFFFu) ++blockCount_;
                break;
        }
    }

    /**
     * Accept the STM32 heartbeat's echoed drive mode as the authoritative
     * remote state.  This confirms our mode even when the CMD_ACK was lost:
     * once the STM32 applies and echoes 4x4 we treat it as in-sync.  It also
     * naturally detects a silent restart — if the STM32 reverts to 4x2 the
     * echo makes confirmed_ != desired_ again so the selector mode is resent.
     *
     * @param echoModeFlags  STM32-applied mode in CMD_MODE flag format
     *                       (bit0 = 4x4, bit1 = tank turn).
     */
    void onHeartbeatModeEcho(uint8_t echoModeFlags) {
        confirmed_      = echoModeFlags;
        confirmedValid_ = true;
        if (pending_ && echoModeFlags == pendingMode_) {
            // The STM32 clearly applied our command — the ACK was just lost.
            pending_ = false;
            failed_  = false;
        }
        if (echoModeFlags == desired_) {
            // The STM32 is now in the requested mode — cancel any pending
            // temporary-block cooldown.
            blocked_      = false;
            blockedArmed_ = false;
        }
    }

    /**
     * Explicitly invalidate the remote confirmation.  Call on a detected STM32
     * restart (heartbeat startup_inhibit rising edge): the physical selector
     * is still the desired truth, but whatever the STM32 last confirmed is no
     * longer valid, so the mode is resynchronised on the next update().
     */
    void invalidateConfirmed() {
        confirmedValid_ = false;
        pending_        = false;
        failed_         = false;
        blocked_        = false;
        blockedArmed_   = false;
    }

    /** Alias for invalidateConfirmed() used on a heartbeat/link-lost edge. */
    void onLinkLost() { invalidateConfirmed(); }

    /** Payload to transmit when update() returns SEND. */
    uint8_t sendMode()  const { return pendingMode_; }

    uint8_t desired()        const { return desired_; }
    uint8_t confirmed()      const { return confirmed_; }
    bool    confirmedValid() const { return confirmedValid_; }
    /** In sync only when a VALID confirmation matches the desired mode. */
    bool    inSync()         const { return confirmedValid_ && desired_ == confirmed_; }
    bool    pending()        const { return pending_; }
    bool    failed()         const { return failed_; }
    /** True while a temporary ACK (REJECTED/BLOCKED) is pending its retry. */
    bool    blocked()        const { return blocked_; }
    uint8_t retries()        const { return retries_; }

    // ---- §4 diagnostics: a temporary block stays pending forever (by design,
    // so a mode selected before ACTIVE is applied automatically), but it must
    // NEVER be silent.  These expose enough to tell a healthy pending sync from
    // a stuck one without flooding the bus (the retry cadence is already paced
    // by blockedBackoffMs_).
    /** Number of temporary (REJECTED/BLOCKED) ACKs seen this episode-run. */
    uint16_t  blockCount()   const { return blockCount_; }
    /** Whether any ACK has been observed yet (lastAck() valid only if true). */
    bool      hasLastAck()   const { return hasLastAck_; }
    /** The most recently observed ACK result. */
    AckResult lastAck()      const { return lastAck_; }
    /** Cause of the current temporary block (REJECTED vs BLOCKED). */
    AckResult blockReason()  const { return blockReason_; }
    /** How long (ms) the current mode has been unconfirmed (0 when in sync). */
    uint32_t  requestAgeMs(uint32_t nowMs) const {
        if (!episodeArmed_) return 0;
        return (uint32_t)(nowMs - episodeStartMs_);
    }

private:
    void beginAttempt(uint32_t nowMs, bool firstAttempt) {
        pending_     = true;
        pendingMode_ = desired_;
        sentMs_      = nowMs;
        retries_     = firstAttempt ? 0 : (uint8_t)(retries_ + 1);
        if (firstAttempt && !episodeArmed_) {
            // §4: timestamp the start of the current synchronisation episode so
            // requestAgeMs() can report how long a mode has been unconfirmed
            // (a long-pending temporary block is thus visible in diagnostics).
            episodeStartMs_ = nowMs;
            episodeArmed_   = true;
        }
    }

    const uint32_t ackTimeoutMs_;
    const uint8_t  maxRetries_;
    const uint32_t blockedBackoffMs_;

    uint8_t  desired_     = 0;   ///< Operator-selected mode (source of truth).
    uint8_t  confirmed_   = 0;   ///< Last mode confirmed by the STM32 (ACK/echo).
    uint8_t  pendingMode_ = 0;   ///< Mode value of the in-flight attempt.
    uint32_t sentMs_      = 0;   ///< Timestamp of the current attempt.
    uint32_t blockedAtMs_ = 0;   ///< Start of the temporary-block cooldown.
    uint8_t  retries_     = 0;   ///< Retransmissions used in this episode.
    bool     pending_     = false;
    bool     failed_      = false;
    bool     blocked_     = false;  ///< A temporary ACK is awaiting its retry.
    bool     blockedArmed_ = false; ///< Cooldown timer has been started.
    // confirmed_ is only trusted while confirmedValid_ is true.  It starts
    // INVALID (false): a cold boot must NOT assume the STM32 already holds 4x2
    // — the physical selector position is transmitted and the request stays
    // pending until the STM32 really confirms it (via ACK or heartbeat echo).
    // This transmits the real selector state on the first live tick even when
    // it is 4x2, so the STM32 and HMI are synchronised from the physical
    // position rather than an assumed default.
    bool     confirmedValid_ = false;

    // ---- §4 diagnostics state (does not affect the FSM decisions) ----
    uint16_t  blockCount_     = 0;      ///< Temporary blocks observed this run.
    bool      hasLastAck_     = false;  ///< An ACK has been observed.
    AckResult lastAck_        = AckResult::OK;   ///< Last observed ACK result.
    AckResult blockReason_    = AckResult::OK;   ///< Cause of the current block.
    uint32_t  episodeStartMs_ = 0;      ///< Start of the current sync episode.
    bool      episodeArmed_   = false;  ///< episodeStartMs_ has been captured.
};

#endif  // MODE_SYNC_H
