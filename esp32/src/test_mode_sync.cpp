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

// A hard rejection latches FAILED without retrying (retry cannot help).
static void test_reject_latches_failed() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x01);
    ASSERT(ms.update(0, true) == Action::SEND);
    ms.onAck(AckRes::REJECTED);
    ASSERT(ms.failed());
    ASSERT(!ms.pending());
    ASSERT(ms.update(ACK_TIMEOUT_MS, true) == Action::NONE);   // no retry
    // A NEW selection (distinct from the confirmed mode) re-arms the FSM.
    ms.setDesired(0x03);
    ASSERT(!ms.failed());
    ASSERT(ms.update(2 * ACK_TIMEOUT_MS, true) == Action::SEND);
    ASSERT_EQ(ms.sendMode(), 0x03);
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

// A contact glitch that does not survive debounce must NOT produce any send.
static void test_glitch_produces_no_send() {
    selector_reset(HIGH);                        // 2WD
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    traction_sw::clearChanged();

    // Brief glitch to LOW then back to HIGH — shorter than the debounce.
    selector_tick(10, LOW);
    selector_tick(20, HIGH);
    selector_tick(30, HIGH);
    ASSERT(!traction_sw::hasChanged());          // debounce rejected the glitch

    // Nothing new to sync; no transmission even with a live heartbeat.
    ms.setDesired(traction_sw::getModeFlag());   // still 2WD (0x00)
    ASSERT(ms.update(40, true) == Action::NONE);
    ASSERT(ms.inSync());
}

int main() {
    test_no_send_without_heartbeat();
    test_ack_confirms_and_stops();
    test_bounded_retry_then_fail();
    test_late_ack_confirms();
    test_reject_latches_failed();
    test_new_desired_rearms_after_fail();
    test_heartbeat_loss_rearms();
    test_debounce_drives_sync();
    test_glitch_produces_no_send();

    std::printf("\n--- mode_sync tests: %d run, %d failed ---\n",
                g_tests_run, g_tests_failed);
    return g_tests_failed ? 1 : 0;
}
