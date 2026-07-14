// test_ack_tracker.cpp — host tests for the bounded command-ACK tracker.
//
// Covers the audit's Entrega-3-D / §PRUEBAS ACK matrix (40-45):
//   40. two independent pending commands are tracked without erasing each other
//   41. out-of-order ACK matches the correct pending entry
//   42. independent per-entry timeout
//   43. unexpected ACK (no pending entry) is ignored
//   44. table full is reported
//   45. millis() rollover is handled
//
// Compile (from repository root):
//   g++ -std=c++17 -Wall -Wextra -Werror -Iesp32/src
//       esp32/src/test_ack_tracker.cpp -o /tmp/test_ack_tracker
//   /tmp/test_ack_tracker

#include "ack_tracker.h"

#include <cstdio>

using namespace ack;

static int tests_run = 0, tests_failed = 0;
#define CHECK(expr) do {                                             \
    tests_run++;                                                     \
    if (!(expr)) {                                                   \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);       \
        tests_failed++;                                              \
    }                                                                \
} while (0)

static constexpr uint32_t TIMEOUT_MS = 200;

// Command low-byte identifiers (mirror can_ids.h low bytes).
static constexpr uint8_t CMD_MODE_LOW    = 0x02;
static constexpr uint8_t CMD_SERVICE_LOW = 0x10;
static constexpr uint8_t CMD_MAP_LOW     = 0x12;
static constexpr uint8_t CMD_LED_LOW     = 0x20;

// 40. Two independent pending commands are tracked without erasing each other.
static void test_two_independent_pending() {
    Tracker<8> t(TIMEOUT_MS);
    CHECK(t.begin(CMD_MODE_LOW, 1000));
    CHECK(t.begin(CMD_LED_LOW, 1005));
    CHECK(t.pendingCount() == 2);
    CHECK(t.isPending(CMD_MODE_LOW));
    CHECK(t.isPending(CMD_LED_LOW));

    // Acknowledging one leaves the other untouched.
    CHECK(t.onAck(CMD_MODE_LOW) == MatchResult::MATCHED);
    CHECK(!t.isPending(CMD_MODE_LOW));
    CHECK(t.isPending(CMD_LED_LOW));
    CHECK(t.pendingCount() == 1);

    CHECK(t.onAck(CMD_LED_LOW) == MatchResult::MATCHED);
    CHECK(t.pendingCount() == 0);
}

// 41. Out-of-order ACK matches the correct pending entry.
static void test_out_of_order_ack() {
    Tracker<8> t(TIMEOUT_MS);
    CHECK(t.begin(CMD_MODE_LOW, 100));
    CHECK(t.begin(CMD_SERVICE_LOW, 110));
    CHECK(t.begin(CMD_MAP_LOW, 120));
    // ACK arrives for the last one first.
    CHECK(t.onAck(CMD_MAP_LOW) == MatchResult::MATCHED);
    CHECK(t.onAck(CMD_MODE_LOW) == MatchResult::MATCHED);
    CHECK(t.isPending(CMD_SERVICE_LOW));
    CHECK(t.pendingCount() == 1);
}

// 42. Independent per-entry timeout: only the stale entry expires.
static void test_independent_timeout() {
    Tracker<8> t(TIMEOUT_MS);
    CHECK(t.begin(CMD_MODE_LOW, 1000));
    CHECK(t.begin(CMD_LED_LOW, 1150));  // sent 150 ms later

    // At t=1201 the first (deadline 1200) has expired, the second (1350) not.
    TimeoutInfo to = t.drainTimeout(1201);
    CHECK(to.valid);
    CHECK(to.cmdIdLow == CMD_MODE_LOW);
    CHECK(t.isPending(CMD_LED_LOW));
    // No more timeouts pending yet.
    CHECK(!t.drainTimeout(1201).valid);

    // Later, the second expires too.
    TimeoutInfo to2 = t.drainTimeout(1400);
    CHECK(to2.valid);
    CHECK(to2.cmdIdLow == CMD_LED_LOW);
    CHECK(t.pendingCount() == 0);
}

// A matched ACK before the deadline means no timeout is ever produced.
static void test_ack_before_timeout() {
    Tracker<8> t(TIMEOUT_MS);
    CHECK(t.begin(CMD_MODE_LOW, 500));
    CHECK(t.onAck(CMD_MODE_LOW) == MatchResult::MATCHED);
    CHECK(!t.drainTimeout(5000).valid);  // nothing pending, no timeout
}

// 43. Unexpected ACK (no matching pending entry) is ignored.
static void test_unexpected_ack() {
    Tracker<8> t(TIMEOUT_MS);
    CHECK(t.onAck(CMD_LED_LOW) == MatchResult::UNEXPECTED);
    CHECK(t.begin(CMD_MODE_LOW, 10));
    // ACK for a different command is unexpected and must not clear CMD_MODE.
    CHECK(t.onAck(CMD_LED_LOW) == MatchResult::UNEXPECTED);
    CHECK(t.isPending(CMD_MODE_LOW));
    CHECK(t.pendingCount() == 1);
    // A duplicate ACK after the real one is also unexpected.
    CHECK(t.onAck(CMD_MODE_LOW) == MatchResult::MATCHED);
    CHECK(t.onAck(CMD_MODE_LOW) == MatchResult::UNEXPECTED);
}

// 44. Table full is reported; begin() on an already-pending cmd still succeeds.
static void test_table_full() {
    Tracker<4> t(TIMEOUT_MS);
    CHECK(t.begin(1, 0));
    CHECK(t.begin(2, 0));
    CHECK(t.begin(3, 0));
    CHECK(t.begin(4, 0));
    CHECK(t.pendingCount() == 4);
    // Table full — a new distinct command is rejected.
    CHECK(!t.begin(5, 0));
    CHECK(t.pendingCount() == 4);
    // Re-registering an already-pending command refreshes in place (no reject).
    CHECK(t.begin(2, 50));
    CHECK(t.pendingCount() == 4);
    // Freeing a slot allows a new command again.
    CHECK(t.onAck(1) == MatchResult::MATCHED);
    CHECK(t.begin(5, 0));
    CHECK(t.pendingCount() == 4);
}

// Refreshing an already-pending command extends its deadline (latest send wins).
static void test_refresh_extends_deadline() {
    Tracker<8> t(TIMEOUT_MS);
    CHECK(t.begin(CMD_MODE_LOW, 1000));
    CHECK(t.begin(CMD_MODE_LOW, 1100));  // resend, refresh deadline to 1300
    CHECK(t.pendingCount() == 1);
    CHECK(!t.drainTimeout(1250).valid);  // old deadline 1200 must NOT fire
    CHECK(t.drainTimeout(1300).valid);   // new deadline 1300 fires
}

// 45. millis() rollover is handled (unsigned subtraction).
static void test_millis_rollover() {
    Tracker<8> t(TIMEOUT_MS);
    const uint32_t nearMax = 0xFFFFFF80u;  // 128 before wrap
    CHECK(t.begin(CMD_MODE_LOW, nearMax));
    // 100 ms later time has wrapped past zero; not yet timed out.
    uint32_t after100 = nearMax + 100u;    // wraps around
    CHECK(!t.drainTimeout(after100).valid);
    CHECK(t.isPending(CMD_MODE_LOW));
    // 200 ms after send: timed out despite the wrap.
    uint32_t after200 = nearMax + 200u;
    TimeoutInfo to = t.drainTimeout(after200);
    CHECK(to.valid);
    CHECK(to.cmdIdLow == CMD_MODE_LOW);
}

int main() {
    test_two_independent_pending();
    test_out_of_order_ack();
    test_independent_timeout();
    test_ack_before_timeout();
    test_unexpected_ack();
    test_table_full();
    test_refresh_extends_deadline();
    test_millis_rollover();

    printf("ack_tracker: %d checks, %d failed\n", tests_run, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
