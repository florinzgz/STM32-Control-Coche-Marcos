// =============================================================================
// Host unit tests for the drive-mode ACK/retry state machine (mode_sync.h)
// and its integration with the debounced 4x4 selector (traction_switch.h).
//
// The debounced physical selector is the SOURCE OF TRUTH; ModeSync gates the
// CMD_MODE transmission on the STM32 heartbeat and (re)transmits with a
// bounded retry budget until an ACK arrives.
//
// Compile (from repo root):
//   g++ -std=c++17 -Iesp32/src -Iesp32/test_stubs
//       esp32/src/traction_switch.cpp esp32/src/test_mode_sync.cpp
//       -o /tmp/test_mode_sync && /tmp/test_mode_sync
// =============================================================================

#include "Arduino.h"
#include "mode_sync.h"
#include "traction_switch.h"

#include <cstdio>

// ---- Minimal test harness ---------------------------------------------------
static int g_tests_run    = 0;
static int g_tests_failed = 0;

#define ASSERT(cond) do {                                                   \
    g_tests_run++;                                                          \
    if (!(cond)) {                                                          \
        std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);         \
        g_tests_failed++;                                                   \
    }                                                                       \
} while (0)

#define ASSERT_EQ(a, b) do {                                                \
    g_tests_run++;                                                          \
    auto _va = (a); auto _vb = (b);                                        \
    if (!(_va == _vb)) {                                                    \
        std::printf("FAIL %s:%d  %s == %s (got %ld vs %ld)\n",             \
                    __FILE__, __LINE__, #a, #b, (long)_va, (long)_vb);      \
        g_tests_failed++;                                                   \
    }                                                                       \
} while (0)

static constexpr uint32_t ACK_TIMEOUT_MS = 200;   // mirrors can::ACK_TIMEOUT_MS
static constexpr uint8_t  MAX_RETRIES    = 3;

using Action = ModeSync::Action;
using AckRes = ModeSync::AckResult;

// ---- ModeSync unit tests ----------------------------------------------------

// No transmission is attempted until the STM32 heartbeat is confirmed.
static void test_no_send_without_heartbeat() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x01);                       // operator selects 4x4
    ASSERT(ms.update(0,   /*hb=*/false) == Action::NONE);
    ASSERT(ms.update(100, /*hb=*/false) == Action::NONE);
    ASSERT(!ms.pending());
    // Heartbeat comes up → first attempt is sent.
    ASSERT(ms.update(200, /*hb=*/true) == Action::SEND);
    ASSERT_EQ(ms.sendMode(), 0x01);
    ASSERT(ms.pending());
}

// A confirmed ACK stops retransmission and marks the mode in-sync.
static void test_ack_confirms_and_stops() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x01);
    ASSERT(ms.update(0, true) == Action::SEND);
    ms.onAck(AckRes::OK);
    ASSERT(ms.inSync());
    ASSERT(!ms.pending());
    ASSERT_EQ(ms.confirmed(), 0x01);
    // No further sends once in sync.
    ASSERT(ms.update(1000, true) == Action::NONE);
}

// Within the ACK window no retransmission happens; after timeout it retries,
// bounded to MAX_RETRIES, then latches FAILED (no bus spamming).
static void test_bounded_retry_then_fail() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x01);
    uint32_t t = 0;
    ASSERT(ms.update(t, true) == Action::SEND);      // attempt 1 (retries=0)
    ASSERT_EQ(ms.retries(), 0);

    // Still within window → no resend.
    ASSERT(ms.update(t + ACK_TIMEOUT_MS - 1, true) == Action::NONE);

    // Timeout → retransmit, up to MAX_RETRIES times.
    for (uint8_t r = 1; r <= MAX_RETRIES; ++r) {
        t += ACK_TIMEOUT_MS;
        ASSERT(ms.update(t, true) == Action::SEND);
        ASSERT_EQ(ms.retries(), r);
    }
    // One more timeout with the budget exhausted → FAILED, no send.
    t += ACK_TIMEOUT_MS;
    ASSERT(ms.update(t, true) == Action::NONE);
    ASSERT(ms.failed());
    ASSERT(!ms.pending());
    ASSERT(!ms.inSync());
}

// A late ACK arriving during the retry sequence still confirms the mode.
static void test_late_ack_confirms() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x01);
    ASSERT(ms.update(0, true) == Action::SEND);
    ASSERT(ms.update(ACK_TIMEOUT_MS, true) == Action::SEND);  // retry 1
    ms.onAck(AckRes::OK);
    ASSERT(ms.inSync());
    ASSERT(ms.update(10 * ACK_TIMEOUT_MS, true) == Action::NONE);
}

// A HARD INVALID ACK (malformed payload) latches FAILED without retrying —
// a retry cannot fix a bad payload.
static void test_invalid_latches_failed() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x01);
    ASSERT(ms.update(0, true) == Action::SEND);
    ms.onAck(AckRes::INVALID);
    ASSERT(ms.failed());
    ASSERT(!ms.blocked());
    ASSERT(!ms.pending());
    ASSERT(ms.update(ACK_TIMEOUT_MS, true) == Action::NONE);   // no retry
    // A NEW selection (distinct from the confirmed mode) re-arms the FSM.
    ms.setDesired(0x03);
    ASSERT(!ms.failed());
    ASSERT(ms.update(2 * ACK_TIMEOUT_MS, true) == Action::SEND);
    ASSERT_EQ(ms.sendMode(), 0x03);
}

// A BLOCKED_BY_SAFETY ACK (STM32 in BOOT/STANDBY at power-on) is TEMPORARY:
// it must NOT latch FAILED.  The request stays pending and is retransmitted
// after the cooldown, again and again, until the STM32 becomes ACTIVE and
// finally accepts it — with NO physical selector toggle.  This is the exact
// reproduced boot 4x2/4x4 desync.
static void test_blocked_retries_until_active() {
    const uint32_t BACKOFF = 1000;
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES, BACKOFF);
    ms.setDesired(0x01);                              // selector at 4x4

    // Boot: STM32 in STANDBY → first CMD_MODE is BLOCKED_BY_SAFETY.
    ASSERT(ms.update(0, true) == Action::SEND);
    ms.onAck(AckRes::BLOCKED);
    ASSERT(!ms.failed());                             // NOT a hard failure
    ASSERT(ms.blocked());
    ASSERT(!ms.inSync());

    // Within the cooldown → no bus traffic (no flooding).
    ASSERT(ms.update(1, true) == Action::NONE);       // arms the cooldown timer
    ASSERT(ms.update(BACKOFF - 1, true) == Action::NONE);

    // Cooldown elapsed → retransmit; still BLOCKED (vehicle still STANDBY).
    ASSERT(ms.update(BACKOFF + 1, true) == Action::SEND);
    ASSERT_EQ(ms.sendMode(), 0x01);
    ms.onAck(AckRes::BLOCKED);
    ASSERT(!ms.failed());

    // Many cycles later the vehicle is finally ACTIVE → the retransmit is
    // accepted with OK and the mode applies, no selector toggle required.
    uint32_t t = 2 * BACKOFF + 2;
    ASSERT(ms.update(t, true) == Action::NONE);       // re-arm cooldown
    ASSERT(ms.update(t + BACKOFF + 1, true) == Action::SEND);
    ms.onAck(AckRes::OK);
    ASSERT(ms.inSync());
    ASSERT_EQ(ms.confirmed(), 0x01);
    ASSERT(!ms.failed());
    ASSERT(!ms.blocked());
}

// A REJECTED ACK (temporary speed/condition gate) behaves like BLOCKED: kept
// pending and retried after the cooldown, never latching FAILED.
static void test_rejected_is_temporary() {
    const uint32_t BACKOFF = 1000;
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES, BACKOFF);
    ms.setDesired(0x01);
    ASSERT(ms.update(0, true) == Action::SEND);
    ms.onAck(AckRes::REJECTED);
    ASSERT(!ms.failed());
    ASSERT(ms.blocked());
    ASSERT(ms.update(1, true) == Action::NONE);
    ASSERT(ms.update(BACKOFF + 2, true) == Action::SEND);   // retried
    ASSERT_EQ(ms.sendMode(), 0x01);
}

// Selecting a new mode after giving up re-arms the state machine.
static void test_new_desired_rearms_after_fail() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x01);
    uint32_t t = 0;
    ms.update(t, true);
    for (uint8_t r = 1; r <= MAX_RETRIES; ++r) { t += ACK_TIMEOUT_MS; ms.update(t, true); }
    t += ACK_TIMEOUT_MS; ms.update(t, true);
    ASSERT(ms.failed());
    // Operator flips the selector to a mode distinct from the confirmed one.
    ms.setDesired(0x03);
    ASSERT(!ms.failed());
    ASSERT(ms.update(t + 1, true) == Action::SEND);
    ASSERT_EQ(ms.sendMode(), 0x03);
}

// Losing the heartbeat mid-attempt drops the in-flight send and re-arms so the
// mode is resynchronised once the STM32 returns.
static void test_heartbeat_loss_rearms() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x01);
    ASSERT(ms.update(0, true) == Action::SEND);
    ASSERT(ms.pending());
    // Heartbeat drops → pending cleared, nothing transmitted.
    ASSERT(ms.update(50, false) == Action::NONE);
    ASSERT(!ms.pending());
    // Heartbeat returns → fresh attempt (still not in sync).
    ASSERT(ms.update(100, true) == Action::SEND);
    ASSERT_EQ(ms.sendMode(), 0x01);
}

// ---- Remote-confirmation validity (STM32 restart / lost ACK) ----------------

// Cold boot with the selector already at 4x4: desired 4x4 differs from the
// power-on default (4x2) confirmation, so CMD_MODE(4x4) is transmitted.
static void test_cold_boot_selector_4x4() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x01);                         // selector reads 4x4 at boot
    ASSERT(ms.update(0, /*hb=*/true) == Action::SEND);
    ASSERT_EQ(ms.sendMode(), 0x01);
    ASSERT(!ms.inSync());
}

// The CMD_ACK for our CMD_MODE(4x4) is lost, but the STM32 heartbeat echoes
// 4x4 — that echo alone must confirm the mode and stop retransmission.
static void test_ack_lost_heartbeat_echo_confirms() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x01);
    ASSERT(ms.update(0, true) == Action::SEND);
    ASSERT(ms.pending());
    // No onAck() — the ACK never arrives.  The STM32 applies and echoes 4x4.
    ms.onHeartbeatModeEcho(0x01);
    ASSERT(ms.inSync());
    ASSERT(!ms.pending());
    ASSERT_EQ(ms.confirmed(), 0x01);
    ASSERT(ms.update(10 * ACK_TIMEOUT_MS, true) == Action::NONE);   // no resend
}

// 4x4 is confirmed; the STM32 restarts (startup_inhibit rising edge triggers
// invalidateConfirmed()).  Even though desired_ == confirmed_ numerically, the
// selector mode must be re-sent because the confirmation is no longer valid.
static void test_confirmed_then_stm32_restart_resends() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x01);
    ASSERT(ms.update(0, true) == Action::SEND);
    ms.onAck(AckRes::OK);
    ASSERT(ms.inSync());

    // STM32 restarts — link is still alive but its confirmation is stale.
    ms.invalidateConfirmed();
    ASSERT(!ms.inSync());
    ASSERT(!ms.confirmedValid());
    ASSERT(ms.update(1000, true) == Action::SEND);           // CMD_MODE resent
    ASSERT_EQ(ms.sendMode(), 0x01);
    // The restarted STM32 applies and echoes 4x4 → back in sync.
    ms.onHeartbeatModeEcho(0x01);
    ASSERT(ms.inSync());
}

// The ESP32 itself restarts while the physical selector still reads 4x4: a
// fresh ModeSync must (re)transmit the selector mode on the first live tick.
static void test_esp32_restart_selector_4x4() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);    // fresh boot state
    ms.setDesired(0x01);                         // selector still at 4x4
    ASSERT(ms.update(0, true) == Action::SEND);
    ASSERT_EQ(ms.sendMode(), 0x01);
    ms.onHeartbeatModeEcho(0x01);
    ASSERT(ms.inSync());
}

// Losing the heartbeat during a pending attempt drops the in-flight send,
// invalidates any confirmation, and re-arms for the link's return.
static void test_heartbeat_loss_during_pending() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x01);
    ASSERT(ms.update(0, true) == Action::SEND);
    ASSERT(ms.pending());
    ASSERT(ms.update(50, /*hb=*/false) == Action::NONE);     // link drops
    ASSERT(!ms.pending());
    ASSERT(!ms.confirmedValid());
    ASSERT(ms.update(100, true) == Action::SEND);            // resync on return
    ASSERT_EQ(ms.sendMode(), 0x01);
}

// A heartbeat loss AFTER a successful ACK invalidates the confirmation, so the
// selector mode is resent when the link returns (the STM32 may have restarted
// during the outage).  A matching echo then re-confirms it.
static void test_heartbeat_loss_after_ack() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x01);
    ASSERT(ms.update(0, true) == Action::SEND);
    ms.onAck(AckRes::OK);
    ASSERT(ms.inSync());

    ASSERT(ms.update(500, /*hb=*/false) == Action::NONE);    // heartbeat lost
    ASSERT(!ms.confirmedValid());
    ASSERT(!ms.inSync());
    ASSERT(ms.update(1000, true) == Action::SEND);           // resent on return
    ASSERT_EQ(ms.sendMode(), 0x01);
    ms.onHeartbeatModeEcho(0x01);
    ASSERT(ms.inSync());
}

// Wrap-safe ACK/echo timestamp comparison across the 32-bit millis() rollover.
static void test_wrap_safe_ack_timestamp() {
    // Normal ordering.
    ASSERT(ModeSync::ackIsAtOrAfterSend(1000, 1000));        // equal
    ASSERT(ModeSync::ackIsAtOrAfterSend(1050, 1000));        // after
    ASSERT(!ModeSync::ackIsAtOrAfterSend(999, 1000));        // before

    // Send just before the wrap, ACK just after: still "after".
    const uint32_t nearMax = 0xFFFFFFF0u;
    ASSERT(ModeSync::ackIsAtOrAfterSend(nearMax + 20, nearMax));   // wrapped +20
    // Send just after the wrap, a pre-wrap timestamp is genuinely "before".
    ASSERT(!ModeSync::ackIsAtOrAfterSend(nearMax, nearMax + 20));
}

// ---- Integration: debounced selector → ModeSync -----------------------------

static constexpr int SWITCH_PIN = 15;

static void selector_reset(int pinState) {
    g_test_millis = 0;
    for (int i = 0; i < 64; ++i) { g_gpio_state[i] = 0; g_gpio_write_count[i] = 0; }
    g_gpio_state[SWITCH_PIN] = pinState;
    traction_sw::init();
}

static void selector_tick(unsigned long ms, int pinState, float speedKmh = 0.0f) {
    g_test_millis = ms;
    g_gpio_state[SWITCH_PIN] = pinState;
    traction_sw::update(speedKmh);
}

// A debounced 2WD→4WD transition drives one CMD_MODE send; ACK brings it into
// sync.  The selector remains the source of truth throughout.
static void test_debounce_drives_sync() {
    selector_reset(HIGH);                       // start 2WD
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    traction_sw::clearChanged();                // ignore the init-time change

    // Physically move the switch to 4WD (LOW) and let it debounce.
    for (unsigned long t = 10; t <= 100; t += 10) selector_tick(t, LOW);
    ASSERT(traction_sw::hasChanged());
    ASSERT(traction_sw::is4WD());

    // Feed the debounced selection into the sync FSM.
    ms.setDesired(traction_sw::getModeFlag());
    traction_sw::clearChanged();
    ASSERT_EQ(ms.desired(), 0x01);

    ASSERT(ms.update(110, /*hb=*/true) == Action::SEND);
    ASSERT_EQ(ms.sendMode(), 0x01);
    ms.onAck(AckRes::OK);
    ASSERT(ms.inSync());
    ASSERT_EQ(ms.confirmed(), 0x01);
}

// At cold boot the physical selector position (2WD here) is transmitted once
// and confirmed.  A contact glitch that does not survive debounce must NOT
// produce any FURTHER send.
static void test_glitch_produces_no_send() {
    selector_reset(HIGH);                        // 2WD
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    traction_sw::clearChanged();

    // Cold-boot sync of the real 2WD position (confirmedValid_ starts false).
    ms.setDesired(traction_sw::getModeFlag());   // 2WD (0x00)
    ASSERT(ms.update(5, true) == Action::SEND);
    ASSERT_EQ(ms.sendMode(), 0x00);
    ms.onAck(AckRes::OK);
    ASSERT(ms.inSync());

    // Brief glitch to LOW then back to HIGH — shorter than the debounce.
    selector_tick(10, LOW);
    selector_tick(20, HIGH);
    selector_tick(30, HIGH);
    ASSERT(!traction_sw::hasChanged());          // debounce rejected the glitch

    // Nothing new to sync; no further transmission with a live heartbeat.
    ms.setDesired(traction_sw::getModeFlag());   // still 2WD (0x00)
    ASSERT(ms.update(40, true) == Action::NONE);
    ASSERT(ms.inSync());
}

// Cold boot with the selector at 2WD: confirmedValid_ starts FALSE, so the
// real 2WD position is transmitted (not assumed) and stays pending until the
// STM32 confirms it — no silent 4x2-default assumption.
static void test_cold_boot_selector_2wd_transmits() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x00);                          // selector reads 2WD at boot
    ASSERT(!ms.confirmedValid());
    ASSERT(!ms.inSync());
    ASSERT(ms.update(0, true) == Action::SEND);   // 2WD is actively synced
    ASSERT_EQ(ms.sendMode(), 0x00);
    ms.onAck(AckRes::OK);
    ASSERT(ms.inSync());
}

int main() {
    test_no_send_without_heartbeat();
    test_ack_confirms_and_stops();
    test_bounded_retry_then_fail();
    test_late_ack_confirms();
    test_invalid_latches_failed();
    test_blocked_retries_until_active();
    test_rejected_is_temporary();
    test_new_desired_rearms_after_fail();
    test_heartbeat_loss_rearms();
    test_cold_boot_selector_4x4();
    test_cold_boot_selector_2wd_transmits();
    test_ack_lost_heartbeat_echo_confirms();
    test_confirmed_then_stm32_restart_resends();
    test_esp32_restart_selector_4x4();
    test_heartbeat_loss_during_pending();
    test_heartbeat_loss_after_ack();
    test_wrap_safe_ack_timestamp();
    test_debounce_drives_sync();
    test_glitch_produces_no_send();

    std::printf("\n--- mode_sync tests: %d run, %d failed ---\n",
                g_tests_run, g_tests_failed);
    return g_tests_failed ? 1 : 0;
}
