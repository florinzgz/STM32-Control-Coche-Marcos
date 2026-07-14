// ack_tracker.h — Bounded, independent command-ACK tracker (audit §Entrega 3-D).
//
// The ESP32 sends several distinct commands to the STM32 that each expect a
// CMD_ACK (0x103) reply carrying the low byte of the acknowledged command ID:
//   - CMD_MODE           (0x102 → 0x02)  gear / drive-mode changes
//   - CMD_LED            (0x120 → 0x20)  decorative relay on/off
//   - SERVICE_CMD        (0x110 → 0x10)  service / diagnostic actions
//   - CMD_SENSOR_MAP_*   (0x112 → 0x12)  temperature sensor role mapping
//
// The previous implementation used a SINGLE global pending slot
// (ackPending / ackExpectedCmd / ackSentMs).  Sending a second command before
// the first was acknowledged silently overwrote the first — its ACK or timeout
// was then lost.  This module replaces that with a fixed-size table so several
// different commands can be tracked simultaneously without one erasing another.
//
// Design constraints (audit):
//   - fixed table, NO dynamic memory,
//   - independent per-entry timeout,
//   - out-of-order ACK matching,
//   - unexpected ACK (no matching pending entry) is ignored,
//   - bounded capacity (begin() reports failure when the table is full),
//   - millis() rollover safe (all time math is unsigned subtraction).
//
// It performs NO CAN or hardware access, so the whole policy is host-testable
// (test_ack_tracker.cpp).  The CAN frame format is unchanged.

#ifndef ACK_TRACKER_H
#define ACK_TRACKER_H

#include <cstdint>

namespace ack {

// Result of consuming an incoming CMD_ACK.
enum class MatchResult : uint8_t {
    UNEXPECTED = 0,  // no pending entry matched — ACK ignored
    MATCHED,         // a pending entry was acknowledged and cleared
};

// One entry can expire (timeout) independently; the caller polls drainTimeouts.
struct TimeoutInfo {
    uint8_t cmdIdLow = 0;
    bool    valid    = false;  // false when no timeout was pending
};

// Fixed-capacity ACK tracker.  Capacity covers the distinct acknowledgeable
// command classes with headroom; there is never a reason to have more than a
// handful of commands in flight at once.
template <uint8_t CAPACITY = 8>
class Tracker {
public:
    explicit Tracker(uint32_t timeoutMs) : timeoutMs_(timeoutMs) {}

    // Register that a command expecting an ACK was just sent at time nowMs.
    // If the same command class is already pending, its deadline is refreshed
    // in place (the latest send wins, no duplicate slot).  Returns false only
    // when the table is full and the command is not already pending.
    bool begin(uint8_t cmdIdLow, uint32_t nowMs) {
        for (uint8_t i = 0; i < CAPACITY; i++) {
            if (slots_[i].active && slots_[i].cmdIdLow == cmdIdLow) {
                slots_[i].sentMs = nowMs;  // refresh, keep single slot
                return true;
            }
        }
        for (uint8_t i = 0; i < CAPACITY; i++) {
            if (!slots_[i].active) {
                slots_[i].active   = true;
                slots_[i].cmdIdLow = cmdIdLow;
                slots_[i].sentMs   = nowMs;
                count_++;
                return true;
            }
        }
        return false;  // table full — caller decides how to surface this
    }

    // Consume an incoming ACK for cmdIdLow.  Out-of-order acknowledgements are
    // handled because every pending slot is scanned.  An ACK that matches no
    // pending entry is reported UNEXPECTED and otherwise ignored.
    MatchResult onAck(uint8_t cmdIdLow) {
        for (uint8_t i = 0; i < CAPACITY; i++) {
            if (slots_[i].active && slots_[i].cmdIdLow == cmdIdLow) {
                slots_[i].active = false;
                count_--;
                return MatchResult::MATCHED;
            }
        }
        return MatchResult::UNEXPECTED;
    }

    // Expire any entry whose deadline has passed and return the first one that
    // timed out.  Call repeatedly until valid == false to drain all timeouts in
    // a single cycle.  Uses unsigned subtraction so millis() rollover is safe.
    TimeoutInfo drainTimeout(uint32_t nowMs) {
        for (uint8_t i = 0; i < CAPACITY; i++) {
            if (slots_[i].active &&
                (uint32_t)(nowMs - slots_[i].sentMs) >= timeoutMs_) {
                slots_[i].active = false;
                count_--;
                return TimeoutInfo{ slots_[i].cmdIdLow, true };
            }
        }
        return TimeoutInfo{ 0, false };
    }

    bool    isPending(uint8_t cmdIdLow) const {
        for (uint8_t i = 0; i < CAPACITY; i++) {
            if (slots_[i].active && slots_[i].cmdIdLow == cmdIdLow) return true;
        }
        return false;
    }

    uint8_t pendingCount() const { return count_; }
    uint8_t capacity()     const { return CAPACITY; }

private:
    struct Slot {
        uint8_t  cmdIdLow = 0;
        uint32_t sentMs   = 0;
        bool     active   = false;
    };
    Slot     slots_[CAPACITY] = {};
    uint8_t  count_    = 0;
    uint32_t timeoutMs_;
};

}  // namespace ack

#endif  // ACK_TRACKER_H
