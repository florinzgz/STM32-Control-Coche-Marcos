// Host test for the single ESP32 -> STM32 0x011 heartbeat producer policy.
//
// Verifies the pure HeartbeatProducer that backs the dedicated FreeRTOS task:
//   * unconditional 100 ms cadence, independent of any loop() kick;
//   * a single rolling counter that advances ONLY on an accepted transmit;
//   * one bounded 10 ms retry on rejection re-sending the same counter;
//   * no accepted-to-accepted gap above 250 ms under congestion + recovery;
//   * correct uint8_t wrap.
//
// The .cpp task glue (FreeRTOS/TWAI) is not host-compilable, but every timing
// and counter decision it makes lives in this HAL-free policy.

#include <cstdint>
#include <cstdio>
#include <vector>

#include "can/can_heartbeat_guard_policy.h"

static int checks = 0;
static int failures = 0;

#define CHECK(expr) do { ++checks; if (!(expr)) { ++failures; \
    std::printf("FAIL line %d: %s\n", __LINE__, #expr); } } while (0)

using can_heartbeat::HeartbeatProducer;
using can_heartbeat::HeartbeatDiag;

// -------------------------------------------------------------------------
// Simulate the dedicated producer task over a timeline.  `accept(slot, retry)`
// decides whether the (simulated) twai_transmit call succeeds.  Mirrors the
// task: one primary attempt per 100 ms slot anchored on absolute time, plus a
// single bounded 10 ms retry when the primary attempt is rejected.
// -------------------------------------------------------------------------
struct SimOutcome {
    uint32_t maxGap = 0;
    uint32_t accepted = 0;
    uint32_t rejected = 0;
    std::vector<uint8_t> seq;   // counter values as they went on the wire
};

template <typename AcceptFn>
static SimOutcome simulate(HeartbeatProducer& p, int slots, AcceptFn accept) {
    SimOutcome out;
    uint32_t wake = 0;
    for (int slot = 0; slot < slots; ++slot) {
        wake += 100;                 // vTaskDelayUntil(&wake, 100 ms)
        uint32_t now = wake;

        // Every slot MUST attempt a transmit — unconditional cadence.
        uint8_t c = p.counter();
        bool acc = accept(slot, false);
        p.onResult(now, acc);
        if (acc) { out.seq.push_back(c); ++out.accepted; }
        else     { ++out.rejected; }

        if (!acc) {
            // Bounded retry 10 ms later; re-sends the same counter value.
            uint32_t rnow = now + 10;
            uint8_t rc = p.counter();
            CHECK(rc == c);          // counter unchanged after a rejection
            bool acc2 = accept(slot, true);
            p.onResult(rnow, acc2);
            if (acc2) { out.seq.push_back(rc); ++out.accepted; }
            else      { ++out.rejected; }
        }
    }
    out.maxGap = p.maxGap();
    return out;
}

int main() {
    // ---- 1. Unconditional 100 ms cadence, no dependence on any loop kick ----
    {
        HeartbeatProducer p;
        p.reset(0);
        // Before any accept, due() fires exactly on the 100 ms boundary.
        CHECK(!p.due(99));
        CHECK(p.due(100));
        // A healthy run: every slot accepts; gaps are all 100 ms.
        SimOutcome o = simulate(p, 50, [](int, bool){ return true; });
        CHECK(o.accepted == 50);
        CHECK(o.rejected == 0);
        CHECK(o.maxGap == 100);            // never above the 250 ms watchdog
        CHECK(o.maxGap < 250);
    }

    // ---- 2. "Loop blocked" / heavy TFT-audio-LED-NVS work ----
    // The producer is independent of loop(); there is no kick to starve.  A
    // fully blocked application loop still yields a heartbeat every slot.
    {
        HeartbeatProducer p;
        p.reset(0);
        SimOutcome o = simulate(p, 200, [](int, bool){ return true; });
        CHECK(o.accepted == 200);
        CHECK(o.maxGap == 100);
        CHECK(o.maxGap < 250);
    }

    // ---- 3. Congested queue and recovery: one fully-missed slot ----
    // Slot 5 rejects both the primary attempt and its 10 ms retry (queue full),
    // then the bus drains.  The worst accepted-to-accepted gap stays <= 200 ms.
    {
        HeartbeatProducer p;
        p.reset(0);
        SimOutcome o = simulate(p, 30, [](int slot, bool){
            return slot != 5;               // slot 5 fully congested
        });
        CHECK(o.accepted == 29);            // one slot dropped entirely
        CHECK(o.rejected == 2);             // primary + retry both rejected
        CHECK(o.maxGap <= 200);
        CHECK(o.maxGap < 250);              // never trips the STM32 watchdog
        HeartbeatDiag d = p.snapshot(3000);
        CHECK(d.txRejectedCount == 2);
        CHECK(d.retryCount == 1);           // one bounded retry episode
    }

    // ---- 4. Congestion recovered by the bounded 10 ms retry ----
    // Primary attempts are rejected under pressure but the 10 ms retry always
    // succeeds, so no frame is ever lost and gaps stay at 100 ms.
    {
        HeartbeatProducer p;
        p.reset(0);
        SimOutcome o = simulate(p, 40, [](int slot, bool retry){
            // Slots 10..19: primary rejected, retry accepted.
            if (slot >= 10 && slot < 20 && !retry) return false;
            return true;
        });
        CHECK(o.accepted == 40);            // retry rescued every congested slot
        CHECK(o.maxGap < 250);
        HeartbeatDiag d = p.snapshot(5000);
        CHECK(d.retryCount == 10);
        CHECK(d.txRejectedCount == 10);
    }

    // ---- 5. Single producer, single counter: advances ONLY on accept ----
    {
        HeartbeatProducer p;
        p.reset(0);
        CHECK(p.counter() == 0);
        p.onResult(100, false);             // rejected
        CHECK(p.counter() == 0);            // no advance
        p.onResult(110, false);             // rejected again
        CHECK(p.counter() == 0);
        p.onResult(120, true);              // accepted
        CHECK(p.counter() == 1);            // advanced by exactly one
        p.onResult(220, true);
        CHECK(p.counter() == 2);
        // Accepted sequence is strictly consecutive (no skipped values).
        SimOutcome o = simulate(p, 10, [](int, bool){ return true; });
        for (size_t i = 1; i < o.seq.size(); ++i) {
            CHECK(static_cast<uint8_t>(o.seq[i-1] + 1) == o.seq[i]);
        }
    }

    // ---- 6. uint8_t wrap 255 -> 0 ----
    {
        HeartbeatProducer p;
        p.reset(0);
        SimOutcome o = simulate(p, 600, [](int, bool){ return true; });
        CHECK(o.accepted == 600);
        // Verify the wire sequence wrapped and stayed consecutive mod 256.
        CHECK(o.seq.size() == 600);
        CHECK(o.seq[0] == 0);
        CHECK(o.seq[255] == 255);
        CHECK(o.seq[256] == 0);             // wrap
        CHECK(o.seq[511] == 255);
        CHECK(o.seq[512] == 0);             // second wrap
        for (size_t i = 1; i < o.seq.size(); ++i) {
            CHECK(static_cast<uint8_t>(o.seq[i-1] + 1) == o.seq[i]);
        }
    }

    // ---- 7. Diagnostics snapshot exposes all required fields ----
    {
        HeartbeatProducer p;
        p.reset(1000);
        p.setResetReason(3);                // e.g. ESP_RST_SW
        p.onResult(1100, true);
        p.observe(/*queueDepth=*/4, /*twaiState=*/1,
                  /*busOff=*/false, /*errorPassive=*/true,
                  /*hwTxFailed=*/7);
        HeartbeatDiag d = p.snapshot(1250);
        CHECK(d.lastAcceptedMs == 1100);
        CHECK(d.currentGapMs == 150);       // 1250 - 1100
        CHECK(d.maxGapMs == 0);             // only one accept so far
        CHECK(d.txAcceptedCount == 1);
        CHECK(d.queueDepth == 4);
        CHECK(d.twaiState == 1);
        CHECK(d.busOff == false);
        CHECK(d.errorPassive == true);
        CHECK(d.hwTxFailedCount == 7);
        CHECK(d.resetReason == 3);
        CHECK(d.counter == 1);
    }

    // ---- 8. millis() rollover handled by unsigned subtraction ----
    {
        HeartbeatProducer p;
        p.reset(0xFFFFFFF0U);
        CHECK(!p.due(0xFFFFFFF0U + 99U));
        CHECK(p.due(0xFFFFFFF0U + 100U));   // wraps past 0
        const uint32_t acceptedAt = 0xFFFFFFF0U + 100U;   // wraps to 0x54
        p.onResult(acceptedAt, true);
        const uint32_t later = acceptedAt + 50U;
        CHECK(p.currentGap(later) == 50U);  // correct across the wrap
    }

    std::printf("can_heartbeat_guard: %d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
