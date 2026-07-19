/**
 ****************************************************************************
 * @file    cmd_mode_transaction.h
 * @brief   Single owner of all CAN 0x102 (CMD_MODE) transactions.
 *
 *          Both sendGearCommand() and sendModeCommand() (ESP32 main.cpp) use
 *          CAN ID 0x102 and produce an ACK with cmdIdLow = 0x02.  Without
 *          serialisation, ModeSync can consume a gear ACK and vice-versa,
 *          leaving one transaction permanently unconfirmed.
 *
 *          This class serialises all 0x102 traffic:
 *            - Only one transaction (MODE_SYNC or GEAR_CHANGE) is in-flight
 *              at any time.
 *            - A gear change requested while a mode sync is pending is saved
 *              in a one-position queue (last-wins) and sent automatically
 *              once the mode is confirmed.
 *            - ACKs are dispatched only to the correct owner through
 *              AckDispatch so ModeSync never sees a gear ACK and vice-versa.
 *            - A per-transaction timeout (default: ACK_TIMEOUT_MS) frees the
 *              slot on a lost frame, allowing the FSM owners to retry.
 *
 *          This file is header-only and HAL-free; it compiles on host GCC
 *          unchanged for use in test_cmd_mode_transaction.cpp.
 *
 *          Compile the test (from repo root):
 *            g++ -std=c++17 -Wall -Wextra -Iesp32/src \
 *                esp32/src/test_cmd_mode_transaction.cpp \
 *                -o /tmp/test_cmd_mode_transaction
 ****************************************************************************
 */

#ifndef CMD_MODE_TRANSACTION_H
#define CMD_MODE_TRANSACTION_H

#include <cstdint>

/**
 * Serialises all CAN 0x102 transactions so mode-sync and gear-change ACKs
 * are never mis-routed.
 */
class CmdModeTransaction {
public:
    /// The type of the transaction currently in-flight.
    enum class TxType : uint8_t { NONE = 0, MODE_SYNC, GEAR_CHANGE };

    /**
     * Returned by onAck() to tell the caller which owner should process the
     * ACK and whether a queued gear should now be sent.
     */
    struct AckDispatch {
        bool    toModeSync  = false; ///< ACK belongs to the ModeSync FSM.
        bool    toGear      = false; ///< ACK belongs to the gear sender.
        uint8_t ackResult   = 0;     ///< Raw ACK code (0=OK,1=REJ,2=INV,3=BLK).
        bool    releaseGear = false; ///< True: send queuedGear now (slot free).
        uint8_t queuedGear  = 0;     ///< Gear to send when releaseGear is true.
    };

    /**
     * @param ackTimeoutMs  How long to wait for an ACK before auto-freeing
     *                      the slot so the owner can retry.
     */
    explicit CmdModeTransaction(uint32_t ackTimeoutMs = 300) noexcept
        : ackTimeoutMs_(ackTimeoutMs) {}

    // -------------------------------------------------------------------------
    // Mode-sync interface
    // -------------------------------------------------------------------------

    /**
     * Called by the ModeSync FSM before transmitting a CMD_MODE (mode) frame.
     *
     * Returns true  if the slot is free: the caller MUST transmit the frame.
     * Returns false if a GEAR_CHANGE is in-flight: do NOT transmit yet; the
     *               FSM may retry after its own ackTimeoutMs expires.
     *
     * When pending_ is already MODE_SYNC (a retry), the slot is re-armed with
     * the new timestamp and modeFlags without consuming any gear queue.
     */
    bool tryBeginMode(uint8_t modeFlags, uint32_t nowMs) noexcept {
        if (pending_ == TxType::GEAR_CHANGE) {
            return false;  // gear in-flight; defer mode
        }
        pending_          = TxType::MODE_SYNC;
        pendingModeFlags_ = modeFlags;
        sentMs_           = nowMs;
        return true;
    }

    // -------------------------------------------------------------------------
    // Gear-change interface
    // -------------------------------------------------------------------------

    /**
     * Called by the gear sender when a gear change is desired.
     *
     * Returns true  if the slot is free: the caller MUST transmit the frame.
     * Returns false if a MODE_SYNC is in-flight: the gear is queued (one
     *               position, last-wins) and will be released automatically
     *               via AckDispatch.releaseGear once the mode is confirmed.
     *
     * If no transaction is pending (NONE), the slot is taken immediately.
     * If a previous GEAR_CHANGE for a DIFFERENT gear is still in-flight
     * (e.g. ACK lost), the slot is retaken with the new gear.
     */
    bool tryBeginGear(uint8_t gear, uint32_t nowMs) noexcept {
        if (pending_ == TxType::MODE_SYNC) {
            // Mode is in-flight; queue the gear (last-wins).
            gearQueued_ = true;
            queuedGear_ = gear;
            return false;
        }
        // Free slot or retaking a stale gear slot.
        pending_     = TxType::GEAR_CHANGE;
        pendingGear_ = gear;
        sentMs_      = nowMs;
        gearQueued_  = false;  // we are sending this gear now
        return true;
    }

    // -------------------------------------------------------------------------
    // ACK routing
    // -------------------------------------------------------------------------

    /**
     * Process an incoming CAN ACK whose cmdIdLow == 0x02 (CMD_MODE / 0x102).
     *
     * Routes the ACK to the correct owner:
     *   - If MODE_SYNC was in-flight → AckDispatch.toModeSync = true.
     *   - If GEAR_CHANGE was in-flight → AckDispatch.toGear = true.
     *   - Spurious ACK (pending_ == NONE) → both false.
     *
     * After an OK mode ACK, if a gear was queued, the slot is released for
     * the gear: AckDispatch.releaseGear = true, queuedGear carries the gear.
     * The caller must then call tryBeginGear(dispatch.queuedGear, nowMs) and
     * transmit the frame.
     *
     * For a HARD mode failure (ackResult == ACK_INVALID = 2), the gear queue
     * is discarded (the operator should manually retry).
     */
    AckDispatch onAck(uint8_t ackResult) noexcept {
        AckDispatch d{};
        if (pending_ == TxType::NONE) return d;  // spurious ACK

        d.ackResult   = ackResult;
        d.toModeSync  = (pending_ == TxType::MODE_SYNC);
        d.toGear      = (pending_ == TxType::GEAR_CHANGE);

        pending_ = TxType::NONE;  // free the slot

        if (d.toModeSync) {
            if (ackResult == ACK_OK && gearQueued_) {
                // Mode confirmed — release the queued gear.
                d.releaseGear = true;
                d.queuedGear  = queuedGear_;
                gearQueued_   = false;
            } else if (ackResult == ACK_INVALID) {
                // Hard failure — discard queue; operator must retry.
                gearQueued_ = false;
            }
            // REJECTED (1) / BLOCKED (3): slot freed but gear queue preserved.
            // ModeSync will retry; once it succeeds the gear will be sent.
        }

        return d;
    }

    // -------------------------------------------------------------------------
    // Timeout / tick
    // -------------------------------------------------------------------------

    /**
     * Called every loop tick.  If the in-flight transaction has not received
     * an ACK within ackTimeoutMs_, the slot is freed so the owner can retry.
     *
     * Returns the type of the transaction that expired (NONE if nothing
     * expired this tick).  The caller does NOT need to do anything special;
     * the FSM owners will re-arm naturally on their next update() call.
     *
     * Note: a mode timeout with a queued gear keeps the gear queued.  Once
     * ModeSync retries and eventually gets an OK, the gear will be sent.
     */
    TxType tick(uint32_t nowMs) noexcept {
        if (pending_ == TxType::NONE) return TxType::NONE;
        if ((uint32_t)(nowMs - sentMs_) < ackTimeoutMs_) return TxType::NONE;

        TxType expired = pending_;
        pending_       = TxType::NONE;
        return expired;
    }

    // -------------------------------------------------------------------------
    // Reset
    // -------------------------------------------------------------------------

    /**
     * Reset all state.  Call on STM32 restart / link loss so stale pending
     * transactions do not block subsequent ones.  The gear queue is cleared:
     * the gear sender will naturally re-request if the physical gear changed.
     */
    void reset() noexcept {
        pending_    = TxType::NONE;
        gearQueued_ = false;
        queuedGear_ = 0;
        sentMs_     = 0;
    }

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    TxType  pendingType()    const noexcept { return pending_; }
    bool    hasQueuedGear()  const noexcept { return gearQueued_; }
    uint8_t queuedGear()     const noexcept { return queuedGear_; }
    uint8_t pendingMode()    const noexcept { return pendingModeFlags_; }
    uint8_t pendingGear()    const noexcept { return pendingGear_; }

    // -------------------------------------------------------------------------
    // ACK result constants (mirror STM32 CAN ACK codes)
    // -------------------------------------------------------------------------
    static constexpr uint8_t ACK_OK      = 0;
    static constexpr uint8_t ACK_REJECTED= 1;
    static constexpr uint8_t ACK_INVALID = 2;
    static constexpr uint8_t ACK_BLOCKED = 3;

private:
    uint32_t ackTimeoutMs_;
    TxType   pending_          = TxType::NONE;
    uint8_t  pendingModeFlags_ = 0;
    uint8_t  pendingGear_      = 0;
    uint32_t sentMs_           = 0;
    bool     gearQueued_       = false;
    uint8_t  queuedGear_       = 0;
};

#endif  // CMD_MODE_TRANSACTION_H
