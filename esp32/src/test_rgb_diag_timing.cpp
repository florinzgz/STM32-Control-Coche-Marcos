// =============================================================================
// Host test — RGB_DIAG colour-order diagnostic TIMING (integration)
//
// The Engineering LED-mode TEST runs a decorative mode for a bounded window
// then auto-restores the previous mode.  RGB_DIAG walks a 10-phase per-strip
// RED → GREEN → BLUE → WHITE → OFF sequence (front 0-4, rear 5-9) at 2.5 s per
// phase = 25 s total.  A 10 s auto-restore window can only reach phases 0-3 —
// the rear strip and both OFF phases are never diagnosed.
//
// Testing only the colour renderer (test_decor_modes.cpp) is INSUFFICIENT: it
// proves each phase renders the right colour but not that each phase is
// REACHABLE before auto-restore.  This test replays the exact phase-advance
// timing against the per-mode auto-restore window and asserts every phase of
// the sequence is reached for RGB_DIAG while the old fixed 10 s window is not.
// =============================================================================

#include <cstdint>
#include <cstdio>
#include "led_controller.h"

static int g_run = 0;
static int g_fail = 0;

#define CHECK(cond, msg) do {                                   \
    ++g_run;                                                    \
    if (!(cond)) { ++g_fail; printf("FAIL: %s\n", (msg)); }     \
    else         { printf("PASS: %s\n", (msg)); }               \
} while (0)

// Replay the RGB_DIAG phase advance for @p windowMs and return a bitmask of the
// distinct phases (bit N set == phase N was active at some point) that are
// reachable before auto-restore fires.  Mirrors the render-loop advance in
// led_controller.cpp: phase advances every DECOR_RGB_DIAG_STEP_MS, wrapping at
// DECOR_RGB_DIAG_PHASES; the test remains active while elapsed < windowMs.
static uint16_t phasesReached(uint32_t windowMs) {
    uint16_t seen = 0;
    uint8_t  phase = 0;
    uint32_t lastAdvance = 0;
    // Sample every 50 ms — much finer than the 2500 ms phase step.
    for (uint32_t now = 0; now < windowMs; now += 50) {
        seen |= static_cast<uint16_t>(1u << phase);
        if ((now - lastAdvance) >= led_ctrl::DECOR_RGB_DIAG_STEP_MS) {
            lastAdvance = now;
            phase = static_cast<uint8_t>((phase + 1) % led_ctrl::DECOR_RGB_DIAG_PHASES);
        }
    }
    return seen;
}

int main() {
    // --- Contract: per-mode auto-restore windows -----------------------------
    CHECK(led_ctrl::decorTestDurationMs(led_ctrl::DecorMode::NORMAL)
              == led_ctrl::DECOR_TEST_DEFAULT_MS,
          "NORMAL uses the default 10 s test window");
    CHECK(led_ctrl::decorTestDurationMs(led_ctrl::DecorMode::POLICE_US)
              == led_ctrl::DECOR_TEST_DEFAULT_MS,
          "non-RGB_DIAG modes use the default 10 s test window");
    CHECK(led_ctrl::decorTestDurationMs(led_ctrl::DecorMode::RGB_DIAG)
              == led_ctrl::DECOR_TEST_RGB_DIAG_MS,
          "RGB_DIAG uses the extended test window");

    // --- Sequence length sanity ---------------------------------------------
    CHECK(led_ctrl::DECOR_RGB_DIAG_SEQUENCE_MS == 25000u,
          "full RGB_DIAG sequence is 10 x 2500 ms = 25 s");
    CHECK(led_ctrl::DECOR_TEST_RGB_DIAG_MS >= led_ctrl::DECOR_RGB_DIAG_SEQUENCE_MS,
          "RGB_DIAG window covers the complete 25 s sequence");
    CHECK(led_ctrl::DECOR_TEST_RGB_DIAG_MS >= 27000u
              && led_ctrl::DECOR_TEST_RGB_DIAG_MS <= 30000u,
          "RGB_DIAG window is in the requested 27-30 s band");

    // --- Regression: the OLD fixed 10 s window is INSUFFICIENT ---------------
    // Phases 4-9 (front OFF, all rear phases incl. rear OFF) must be missing.
    const uint16_t allPhases = 0x03FFu;  // bits 0..9
    uint16_t old10s = phasesReached(led_ctrl::DECOR_TEST_DEFAULT_MS);
    CHECK((old10s & (1u << 4)) == 0, "10 s window CANNOT reach phase 4 (front OFF)");
    CHECK((old10s & (1u << 5)) == 0, "10 s window CANNOT reach phase 5 (rear RED)");
    CHECK((old10s & (1u << 9)) == 0, "10 s window CANNOT reach phase 9 (rear OFF)");
    CHECK(old10s != allPhases, "10 s window leaves part of the sequence undiagnosed");

    // --- Fix: the RGB_DIAG window reaches EVERY phase before auto-restore -----
    uint16_t full = phasesReached(led_ctrl::decorTestDurationMs(led_ctrl::DecorMode::RGB_DIAG));
    CHECK((full & (1u << 4)) != 0, "RGB_DIAG window reaches phase 4 (front OFF)");
    CHECK((full & (1u << 5)) != 0, "RGB_DIAG window reaches phase 5 (rear RED)");
    CHECK((full & (1u << 8)) != 0, "RGB_DIAG window reaches phase 8 (rear WHITE)");
    CHECK((full & (1u << 9)) != 0, "RGB_DIAG window reaches phase 9 (rear OFF)");
    CHECK(full == allPhases, "RGB_DIAG window reaches ALL 10 phases before auto-restore");

    printf("--- rgb_diag_timing tests: %d run, %d failed ---\n", g_run, g_fail);
    return g_fail ? 1 : 0;
}
