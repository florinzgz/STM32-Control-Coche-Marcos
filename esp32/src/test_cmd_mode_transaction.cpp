// =============================================================================
// test_cmd_mode_transaction.cpp
//
// Host unit tests for CmdModeTransaction: the single owner of all CAN 0x102
// transactions (mode-sync and gear-change).
//
// Verified properties:
//   – Mode pending + simultaneous gear change → gear queued, not sent.
//   – Gear ACK does NOT confirm the mode.
//   – Mode ACK does NOT confirm the gear.
//   – Lost ACK (timeout) → slot freed, retry possible.
//   – Last requested gear is sent after mode confirms.
//   – STM32 restart (reset()) clears all state.
//   – Gear queue preserved across mode REJECTED/BLOCKED; cleared on INVALID.
//
// Compile (from repo root):
//   g++ -std=c++17 -Wall -Wextra -Iesp32/src \
//       esp32/src/test_cmd_mode_transaction.cpp \
//       -o /tmp/test_cmd_mode_transaction
//   /tmp/test_cmd_mode_transaction
// =============================================================================

#include "cmd_mode_transaction.h"
#include "mode_sync.h"       // for AckResult enum in some tests

#include <cstdio>
#include <cstdint>

// ---- Minimal test harness ---------------------------------------------------
static int g_run = 0, g_fail = 0;

#define ASSERT(cond) do {                                                   \
    g_run++;                                                                \
    if (!(cond)) {                                                          \
        std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);         \
        g_fail++;                                                           \
    }                                                                       \
} while (0)

#define ASSERT_EQ(a, b) do {                                                \
    g_run++;                                                                \
    long _va = (long)(a); long _vb = (long)(b);                            \
    if (_va != _vb) {                                                       \
        std::printf("FAIL %s:%d  %s == %s (got %ld vs %ld)\n",             \
                    __FILE__, __LINE__, #a, #b, _va, _vb);                 \
    }                                                                       \
} while (0)

using TxType = CmdModeTransaction::TxType;
using Dispatch = CmdModeTransaction::AckDispatch;
static constexpr uint32_t TX_TIMEOUT = 300;

// ---- §3 Tests ---------------------------------------------------------------

// Basic: free slot → tryBeginMode returns true.
static void test_mode_begins_when_slot_free() {
    CmdModeTransaction t{TX_TIMEOUT};
    ASSERT(t.pendingType() == TxType::NONE);
    ASSERT(t.tryBeginMode(0x01, 0));
    ASSERT(t.pendingType() == TxType::MODE_SYNC);
}

// Basic: free slot → tryBeginGear returns true.
static void test_gear_begins_when_slot_free() {
    CmdModeTransaction t{TX_TIMEOUT};
    ASSERT(t.tryBeginGear(3, 0));
    ASSERT(t.pendingType() == TxType::GEAR_CHANGE);
}

// Mode pending + gear change → gear queued, send deferred.
static void test_mode_pending_gear_is_queued() {
    CmdModeTransaction t{TX_TIMEOUT};
    ASSERT(t.tryBeginMode(0x01, 0));          // mode in-flight
    ASSERT(!t.tryBeginGear(3, 10));           // must NOT send now
    ASSERT(t.pendingType() == TxType::MODE_SYNC);  // mode still owns slot
    ASSERT(t.hasQueuedGear());
    ASSERT_EQ(t.queuedGear(), 3);
}

// Gear ACK does NOT confirm the mode (toModeSync is false when gear in-flight).
static void test_gear_ack_does_not_confirm_mode() {
    CmdModeTransaction t{TX_TIMEOUT};
    // Only gear is in-flight; mode is not pending.
    ASSERT(t.tryBeginGear(3, 0));
    Dispatch d = t.onAck(CmdModeTransaction::ACK_OK);
    ASSERT(!d.toModeSync);  // gear ACK must NOT go to ModeSync
    ASSERT(d.toGear);
    ASSERT(t.pendingType() == TxType::NONE);
}

// Mode ACK does NOT confirm the gear.
static void test_mode_ack_does_not_confirm_gear() {
    CmdModeTransaction t{TX_TIMEOUT};
    // Only mode is in-flight; no gear transaction.
    ASSERT(t.tryBeginMode(0x01, 0));
    Dispatch d = t.onAck(CmdModeTransaction::ACK_OK);
    ASSERT(d.toModeSync);   // ACK goes to ModeSync
    ASSERT(!d.toGear);      // NOT to the gear sender
    ASSERT(t.pendingType() == TxType::NONE);
}

// Lost ACK (timeout) → slot freed, mode can retry.
static void test_mode_ack_lost_slot_freed_on_timeout() {
    CmdModeTransaction t{TX_TIMEOUT};
    ASSERT(t.tryBeginMode(0x01, 0));
    // No ACK arrives; before timeout slot is still occupied.
    ASSERT(t.tick(TX_TIMEOUT - 1) == TxType::NONE);
    ASSERT(t.pendingType() == TxType::MODE_SYNC);
    // At or after timeout → slot freed.
    ASSERT(t.tick(TX_TIMEOUT) == TxType::MODE_SYNC);
    ASSERT(t.pendingType() == TxType::NONE);
    // Now mode can retry.
    ASSERT(t.tryBeginMode(0x01, TX_TIMEOUT + 10));
}

// Lost gear ACK → timeout frees slot, gear sender can retry.
static void test_gear_ack_lost_slot_freed_on_timeout() {
    CmdModeTransaction t{TX_TIMEOUT};
    ASSERT(t.tryBeginGear(3, 0));
    ASSERT(t.tick(TX_TIMEOUT - 1) == TxType::NONE);
    ASSERT(t.tick(TX_TIMEOUT) == TxType::GEAR_CHANGE);
    ASSERT(t.pendingType() == TxType::NONE);
    ASSERT(t.tryBeginGear(3, TX_TIMEOUT + 1));  // retry allowed
}

// Last requested gear is sent after mode OK confirms.
static void test_last_gear_sent_after_mode_confirms() {
    CmdModeTransaction t{TX_TIMEOUT};
    ASSERT(t.tryBeginMode(0x01, 0));          // mode in-flight
    ASSERT(!t.tryBeginGear(2, 5));            // gear queued
    ASSERT(!t.tryBeginGear(3, 10));           // updated to gear 3 (last-wins)
    ASSERT(t.hasQueuedGear());
    ASSERT_EQ(t.queuedGear(), 3);             // last gear wins

    // Mode is confirmed OK.
    Dispatch d = t.onAck(CmdModeTransaction::ACK_OK);
    ASSERT(d.toModeSync);
    ASSERT(d.releaseGear);                    // gear 3 should be sent now
    ASSERT_EQ(d.queuedGear, 3);
    ASSERT(!t.hasQueuedGear());
    ASSERT(t.pendingType() == TxType::NONE);  // slot free for gear

    // Caller sends the gear: tryBeginGear() must succeed.
    ASSERT(t.tryBeginGear(d.queuedGear, 20));
    ASSERT(t.pendingType() == TxType::GEAR_CHANGE);
}

// Mode REJECTED: slot freed, gear queue preserved (mode will retry).
static void test_mode_rejected_gear_queue_preserved() {
    CmdModeTransaction t{TX_TIMEOUT};
    ASSERT(t.tryBeginMode(0x01, 0));
    ASSERT(!t.tryBeginGear(2, 5));            // gear queued
    Dispatch d = t.onAck(CmdModeTransaction::ACK_REJECTED);
    ASSERT(d.toModeSync);
    ASSERT(!d.releaseGear);                   // not released yet (mode pending)
    ASSERT(t.hasQueuedGear());                // queue preserved
    ASSERT(t.pendingType() == TxType::NONE);  // slot free for ModeSync retry
}

// Mode BLOCKED: gear queue preserved (same as REJECTED).
static void test_mode_blocked_gear_queue_preserved() {
    CmdModeTransaction t{TX_TIMEOUT};
    ASSERT(t.tryBeginMode(0x01, 0));
    ASSERT(!t.tryBeginGear(2, 5));
    Dispatch d = t.onAck(CmdModeTransaction::ACK_BLOCKED);
    ASSERT(d.toModeSync);
    ASSERT(!d.releaseGear);
    ASSERT(t.hasQueuedGear());
    ASSERT(t.pendingType() == TxType::NONE);
}

// Mode INVALID (hard failure): gear queue discarded.
static void test_mode_invalid_gear_queue_discarded() {
    CmdModeTransaction t{TX_TIMEOUT};
    ASSERT(t.tryBeginMode(0x01, 0));
    ASSERT(!t.tryBeginGear(2, 5));            // gear queued
    Dispatch d = t.onAck(CmdModeTransaction::ACK_INVALID);
    ASSERT(d.toModeSync);
    ASSERT(!d.releaseGear);
    ASSERT(!t.hasQueuedGear());               // discarded on hard failure
}

// Spurious ACK (no pending transaction) → ignored (no-op).
static void test_spurious_ack_ignored() {
    CmdModeTransaction t{TX_TIMEOUT};
    Dispatch d = t.onAck(CmdModeTransaction::ACK_OK);
    ASSERT(!d.toModeSync);
    ASSERT(!d.toGear);
    ASSERT(!d.releaseGear);
    ASSERT(t.pendingType() == TxType::NONE);
}

// STM32 restart during mode transaction → reset() clears everything.
static void test_stm32_restart_during_mode_clears_state() {
    CmdModeTransaction t{TX_TIMEOUT};
    ASSERT(t.tryBeginMode(0x01, 0));
    ASSERT(!t.tryBeginGear(3, 5));  // gear queued
    ASSERT(t.hasQueuedGear());

    t.reset();
    ASSERT(t.pendingType() == TxType::NONE);
    ASSERT(!t.hasQueuedGear());
    // After reset both mode and gear can start fresh.
    ASSERT(t.tryBeginMode(0x01, 100));
    ASSERT(t.pendingType() == TxType::MODE_SYNC);
}

// STM32 restart during gear transaction → reset clears.
static void test_stm32_restart_during_gear_clears_state() {
    CmdModeTransaction t{TX_TIMEOUT};
    ASSERT(t.tryBeginGear(3, 0));
    t.reset();
    ASSERT(t.pendingType() == TxType::NONE);
    ASSERT(t.tryBeginGear(3, 100));
}

// Gear cannot start when mode is in-flight, but once mode ACK clears and the
// released gear succeeds, a subsequent second gear sends as expected.
static void test_gear_after_released_gear_ack() {
    CmdModeTransaction t{TX_TIMEOUT};
    ASSERT(t.tryBeginMode(0x01, 0));
    ASSERT(!t.tryBeginGear(3, 5));
    Dispatch d = t.onAck(CmdModeTransaction::ACK_OK);
    ASSERT(d.releaseGear);
    // "Send" the released gear.
    ASSERT(t.tryBeginGear(d.queuedGear, 10));
    // Gear OK → slot free.
    d = t.onAck(CmdModeTransaction::ACK_OK);
    ASSERT(d.toGear);
    ASSERT(!d.toModeSync);
    ASSERT(t.pendingType() == TxType::NONE);
}

// Mode cannot start when gear is in-flight.
static void test_mode_blocked_while_gear_in_flight() {
    CmdModeTransaction t{TX_TIMEOUT};
    ASSERT(t.tryBeginGear(3, 0));
    ASSERT(!t.tryBeginMode(0x01, 5));          // must NOT start
    ASSERT(t.pendingType() == TxType::GEAR_CHANGE);  // gear still owns slot
}

// Heartbeat-path: mode confirmed without an explicit ACK (ModeSync handles
// this via onHeartbeatModeEcho).  The transaction owner is not involved; it
// stays NONE after a mode-free idle tick.
static void test_no_pending_stays_idle() {
    CmdModeTransaction t{TX_TIMEOUT};
    ASSERT(t.tick(1000) == TxType::NONE);
    ASSERT(t.pendingType() == TxType::NONE);
}

int main() {
    test_mode_begins_when_slot_free();
    test_gear_begins_when_slot_free();
    test_mode_pending_gear_is_queued();
    test_gear_ack_does_not_confirm_mode();
    test_mode_ack_does_not_confirm_gear();
    test_mode_ack_lost_slot_freed_on_timeout();
    test_gear_ack_lost_slot_freed_on_timeout();
    test_last_gear_sent_after_mode_confirms();
    test_mode_rejected_gear_queue_preserved();
    test_mode_blocked_gear_queue_preserved();
    test_mode_invalid_gear_queue_discarded();
    test_spurious_ack_ignored();
    test_stm32_restart_during_mode_clears_state();
    test_stm32_restart_during_gear_clears_state();
    test_gear_after_released_gear_ack();
    test_mode_blocked_while_gear_in_flight();
    test_no_pending_stays_idle();

    std::printf("\n--- cmd_mode_transaction tests: %d run, %d failed ---\n",
                g_run, g_fail);
    return g_fail ? 1 : 0;
}
