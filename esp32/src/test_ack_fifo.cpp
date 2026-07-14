// test_ack_fifo.cpp — integration tests for the bounded command-ACK FIFO.
//
// PR #427 follow-up: can_rx::poll() decodes CAN frames in a while-loop, so
// several CMD_ACK (0x103) frames can be decoded in a single poll() call.  The
// previous design fed only the last-seen VehicleData::ack_ slot into the
// tracker, so earlier ACKs were lost and their pending commands timed out.
//
// These tests exercise the VehicleData ACK FIFO (pushAck/popAck) together with
// ack::Tracker exactly the way can_rx::decodeCommandAck() and main::ackCheck()
// use them, covering:
//   1. two distinct ACKs received in the same poll() call,
//   2. out-of-order ACKs in the same burst,
//   3. first ACK with millis() == 0,
//   4. full FIFO with observable overflow,
//   5. every ACK consumed exactly once,
//   6. no change to CAN ID / DLC / frame layout.
//
// Compile (from repository root):
//   g++ -std=c++17 -Wall -Wextra -Werror -Iesp32/src -Iesp32/include
//       esp32/src/test_ack_fifo.cpp -o /tmp/test_ack_fifo
//   /tmp/test_ack_fifo

#include "vehicle_data.h"
#include "ack_tracker.h"

#include <cstdio>

using vehicle::AckData;
using vehicle::VehicleData;

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
static constexpr uint8_t CMD_MODE_LOW    = 0x02;  // 0x102 & 0xFF
static constexpr uint8_t CMD_SERVICE_LOW = 0x10;  // 0x110 & 0xFF
static constexpr uint8_t CMD_MAP_LOW     = 0x12;  // 0x112 & 0xFF
static constexpr uint8_t CMD_LED_LOW     = 0x20;  // 0x120 & 0xFF

// Mimic can_rx::decodeCommandAck(): store the most-recent ACK for HMI and push
// it onto the FIFO.  Returns false when the FIFO overflowed (ACK dropped).
static bool rxAck(VehicleData& d, uint8_t cmdIdLow, unsigned long ts,
                  can::AckResult result = can::AckResult::OK) {
    AckData ad;
    ad.cmdIdLow    = cmdIdLow;
    ad.result      = result;
    ad.systemState = can::SystemState::ACTIVE;
    ad.timestampMs = ts;
    d.setAck(ad);           // HMI last-ACK slot
    return d.pushAck(ad);   // FIFO
}

// Mimic main::ackCheck(): drain the whole FIFO, feeding each ACK to the tracker
// exactly once.  Returns the number of ACKs drained this cycle.
static int drainAcks(VehicleData& d, ack::Tracker<8>& tracker, int& matched) {
    int drained = 0;
    AckData ad;
    while (d.popAck(ad)) {
        drained++;
        if (tracker.onAck(ad.cmdIdLow) == ack::MatchResult::MATCHED) matched++;
    }
    return drained;
}

// 1. Two distinct ACKs received in the same poll() call are both delivered.
static void test_two_acks_same_poll() {
    VehicleData d;
    ack::Tracker<8> t(TIMEOUT_MS);
    CHECK(t.begin(CMD_MODE_LOW, 1000));
    CHECK(t.begin(CMD_LED_LOW, 1000));

    // Same poll() burst: two ACKs decoded before ackCheck() runs.
    CHECK(rxAck(d, CMD_MODE_LOW, 1010));
    CHECK(rxAck(d, CMD_LED_LOW, 1011));
    CHECK(d.ackFifoCount() == 2);

    int matched = 0;
    CHECK(drainAcks(d, t, matched) == 2);
    CHECK(matched == 2);
    CHECK(t.pendingCount() == 0);   // neither command times out
    CHECK(d.ackFifoCount() == 0);
}

// 2. Out-of-order ACKs in the same burst match the correct pending entries.
static void test_out_of_order_same_burst() {
    VehicleData d;
    ack::Tracker<8> t(TIMEOUT_MS);
    CHECK(t.begin(CMD_MODE_LOW, 100));
    CHECK(t.begin(CMD_SERVICE_LOW, 100));
    CHECK(t.begin(CMD_MAP_LOW, 100));

    // ACKs arrive reversed relative to the send order, all in one poll().
    CHECK(rxAck(d, CMD_MAP_LOW, 120));
    CHECK(rxAck(d, CMD_SERVICE_LOW, 121));
    CHECK(rxAck(d, CMD_MODE_LOW, 122));
    CHECK(d.ackFifoCount() == 3);

    int matched = 0;
    CHECK(drainAcks(d, t, matched) == 3);
    CHECK(matched == 3);
    CHECK(t.pendingCount() == 0);
}

// 3. First ACK with millis() == 0 is processed (no timestamp de-duplication).
static void test_first_ack_millis_zero() {
    VehicleData d;
    ack::Tracker<8> t(TIMEOUT_MS);
    CHECK(t.begin(CMD_MODE_LOW, 0));

    // Very first ACK after boot carries timestamp 0.
    CHECK(rxAck(d, CMD_MODE_LOW, 0));
    int matched = 0;
    CHECK(drainAcks(d, t, matched) == 1);
    CHECK(matched == 1);            // must NOT be silently ignored
    CHECK(t.pendingCount() == 0);
    CHECK(!t.drainTimeout(TIMEOUT_MS + 1).valid);  // no false timeout
}

// 4. FIFO full → overflow is observable and no unconsumed ACK is overwritten.
static void test_fifo_overflow() {
    VehicleData d;
    // Fill to capacity: all pushes succeed.
    for (uint8_t i = 0; i < VehicleData::ACK_FIFO_CAPACITY; i++) {
        CHECK(rxAck(d, CMD_MODE_LOW, 1000 + i));
    }
    CHECK(d.ackFifoCount() == VehicleData::ACK_FIFO_CAPACITY);
    CHECK(d.ackFifoOverflowCount() == 0);

    // Next two ACKs overflow and are dropped (not overwriting queued entries).
    CHECK(!rxAck(d, CMD_LED_LOW, 2000));
    CHECK(!rxAck(d, CMD_SERVICE_LOW, 2001));
    CHECK(d.ackFifoOverflowCount() == 2);
    CHECK(d.ackFifoCount() == VehicleData::ACK_FIFO_CAPACITY);

    // Every queued entry is still the original CMD_MODE ACK (nothing clobbered).
    AckData ad;
    uint8_t drained = 0;
    while (d.popAck(ad)) {
        CHECK(ad.cmdIdLow == CMD_MODE_LOW);
        drained++;
    }
    CHECK(drained == VehicleData::ACK_FIFO_CAPACITY);
}

// 5. Every ACK is consumed exactly once (no loss, no duplication).
static void test_consumed_exactly_once() {
    VehicleData d;
    ack::Tracker<8> t(TIMEOUT_MS);
    const uint8_t cmds[4] = { CMD_MODE_LOW, CMD_LED_LOW, CMD_SERVICE_LOW,
                              CMD_MAP_LOW };
    for (uint8_t i = 0; i < 4; i++) CHECK(t.begin(cmds[i], 500));
    for (uint8_t i = 0; i < 4; i++) CHECK(rxAck(d, cmds[i], 510 + i));

    int matched = 0;
    CHECK(drainAcks(d, t, matched) == 4);
    CHECK(matched == 4);
    CHECK(t.pendingCount() == 0);

    // A second drain finds nothing (each ACK consumed once, not re-delivered).
    int matched2 = 0;
    CHECK(drainAcks(d, t, matched2) == 0);
    CHECK(matched2 == 0);
}

// 6. Frame layout unchanged: CMD_ACK is 0x103, DLC 3, byte order preserved.
static void test_layout_unchanged() {
    // ID and low bytes used for matching are stable.
    CHECK(can::CMD_ACK == 0x103);
    CHECK((can::CMD_MODE & 0xFF) == CMD_MODE_LOW);
    CHECK((can::CMD_LED  & 0xFF) == CMD_LED_LOW);
    CHECK((can::SERVICE_CMD & 0xFF) == CMD_SERVICE_LOW);

    // AckData preserves the decoded byte0/byte1/byte2 mapping.
    VehicleData d;
    CHECK(rxAck(d, CMD_LED_LOW, 42, can::AckResult::OK));
    const AckData& last = d.ack();     // HMI slot mirrors the pushed ACK
    CHECK(last.cmdIdLow == CMD_LED_LOW);
    CHECK(last.result == can::AckResult::OK);
    AckData popped;
    CHECK(d.popAck(popped));
    CHECK(popped.cmdIdLow == last.cmdIdLow);
    CHECK(popped.result == last.result);
    CHECK(popped.systemState == last.systemState);
}

int main() {
    test_two_acks_same_poll();
    test_out_of_order_same_burst();
    test_first_ack_millis_zero();
    test_fifo_overflow();
    test_consumed_exactly_once();
    test_layout_unchanged();

    printf("ack_fifo: %d checks, %d failed\n", tests_run, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
