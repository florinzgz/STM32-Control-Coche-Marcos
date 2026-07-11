/**
 ****************************************************************************
 * @file    test_stm32_liveness.cpp
 * @brief   Host-compilable unit tests for the STM32 heartbeat liveness
 *          tracker (Stm32Liveness).  Links the REAL production header
 *          (stm32_liveness.h) — the logic is NOT re-implemented here.
 *
 *          Compile and run on host (from esp32/src/):
 *            g++ -std=c++17 -I. test_stm32_liveness.cpp \
 *                -o /tmp/test_stm32_liveness && /tmp/test_stm32_liveness
 *
 *          Requirements covered (problem statement §2):
 *            1. loop @1 ms, heartbeat @100 ms: NEVER falsely dead.
 *            2. Five new frames with increasing counter: alive.
 *            3. Five new frames with identical counter: frozen.
 *            4. No heartbeat during the timeout: dead.
 *            5. Reappearance with advancing counter: recovery.
 *            6. Wrap 255 -> 0 treated as advancing (alive).
 *            7. Frozen edge fires once (rate-limit friendly).
 ****************************************************************************
 */

#include <cstdio>
#include <cstdint>
#include "stm32_liveness.h"

static int s_tests_run    = 0;
static int s_tests_failed = 0;

#define ASSERT(cond) do {                                                    \
    s_tests_run++;                                                           \
    if (!(cond)) {                                                           \
        printf("FAIL  %s:%d  Assertion: %s\n", __FILE__, __LINE__, #cond);  \
        s_tests_failed++;                                                    \
    }                                                                        \
} while (0)

// Matches the production constants: 5 frames, 500 ms absence window.
static constexpr uint8_t  FREEZE   = 5;
static constexpr uint32_t ABSENCE  = 500;
static constexpr uint32_t HB_MS    = 100;

// --- Test 1: loop @1 ms, heartbeat @100 ms, increasing counter: never dead --
static void test_fast_loop_never_false_dead() {
    Stm32Liveness lv(FREEZE, ABSENCE);
    uint32_t frameTs   = 0;
    uint8_t  counter   = 0;
    uint32_t nextHbAt  = HB_MS;   // first heartbeat at t = 100 ms
    bool everDeadAfterSeen = false;

    // Simulate 5 seconds of 1 ms loop iterations.
    for (uint32_t t = 1; t <= 5000; ++t) {
        if (t >= nextHbAt) {
            // New heartbeat frame: timestamp = arrival time, counter advances.
            frameTs  = t;
            counter  = (uint8_t)(counter + 1);
            nextHbAt = t + HB_MS;
        }
        lv.update(frameTs, counter, t);
        // Once the first frame has been seen it must stay alive forever.
        if (lv.hasSeen() && !lv.isAlive()) {
            everDeadAfterSeen = true;
        }
    }
    ASSERT(!everDeadAfterSeen);
    ASSERT(lv.isAlive());
}

// --- Test 2: five new frames, increasing counter: alive -----------------
static void test_five_increasing_alive() {
    Stm32Liveness lv(FREEZE, ABSENCE);
    uint32_t t = 10;
    for (uint8_t i = 1; i <= 5; ++i) {
        lv.update(t, i, t);
        t += HB_MS;
    }
    ASSERT(lv.isAlive());
    ASSERT(lv.sameCount() == 0);
}

// --- Test 3: five identical new frames: frozen --------------------------
static void test_identical_frozen() {
    Stm32Liveness lv(FREEZE, ABSENCE);
    uint32_t t = 10;
    // Baseline frame.
    lv.update(t, 42, t);
    ASSERT(lv.isAlive());
    // Five more NEW frames (new timestamps) all carrying the same counter.
    for (int i = 0; i < 5; ++i) {
        t += 50;  // < absence timeout, so frames keep "arriving"
        lv.update(t, 42, t);
    }
    ASSERT(!lv.isAlive());
}

// --- Test 4: absence during timeout: dead -------------------------------
static void test_absence_dead() {
    Stm32Liveness lv(FREEZE, ABSENCE);
    uint32_t frameTs = 100;
    lv.update(frameTs, 7, 100);
    ASSERT(lv.isAlive());
    // No new frame: frameTs frozen, now advances past the absence timeout.
    lv.update(frameTs, 7, 100 + ABSENCE + 1);
    ASSERT(!lv.isAlive());
}

// --- Test 5: recovery after freeze/absence ------------------------------
static void test_recovery() {
    Stm32Liveness lv(FREEZE, ABSENCE);
    uint32_t t = 10;
    lv.update(t, 5, t);
    // Go absent -> dead.
    lv.update(t, 5, t + ABSENCE + 5);
    ASSERT(!lv.isAlive());
    // New frame with advancing counter -> alive again.
    uint32_t t2 = t + ABSENCE + 100;
    lv.update(t2, 6, t2);
    ASSERT(lv.isAlive());
}

// --- Test 6: wrap 255 -> 0 is advancing (alive) -------------------------
static void test_wrap() {
    Stm32Liveness lv(FREEZE, ABSENCE);
    uint32_t t = 10;
    lv.update(t, 255, t);
    ASSERT(lv.isAlive());
    t += HB_MS;
    lv.update(t, 0, t);   // wrap
    ASSERT(lv.isAlive());
    ASSERT(lv.sameCount() == 0);
}

// --- Test 7: frozen edge fires exactly once -----------------------------
static void test_frozen_edge_once() {
    Stm32Liveness lv(FREEZE, ABSENCE);
    uint32_t t = 10;
    lv.update(t, 3, t);
    for (int i = 0; i < 5; ++i) {
        t += 50;
        lv.update(t, 3, t);
    }
    ASSERT(!lv.isAlive());
    int edges = 0;
    if (lv.consumeFrozenEdge()) edges++;
    // Further identical frames must NOT re-raise the edge.
    t += 50;
    lv.update(t, 3, t);
    if (lv.consumeFrozenEdge()) edges++;
    ASSERT(edges == 1);
}

int main() {
    test_fast_loop_never_false_dead();
    test_five_increasing_alive();
    test_identical_frozen();
    test_absence_dead();
    test_recovery();
    test_wrap();
    test_frozen_edge_once();

    printf("\n%d tests run, %d failed\n", s_tests_run, s_tests_failed);
    return s_tests_failed == 0 ? 0 : 1;
}
