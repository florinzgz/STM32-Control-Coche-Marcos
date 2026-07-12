// =============================================================================
// ESP32 HMI — host unit test for the pure per-wheel traction share helper
// (esp32/src/wheel_traction.h).
//
// Covers traction::computeShare() used by the Engineering "INA226 LIVE DIAG"
// page to show a relative per-wheel traction percentage from the 0x201 motor
// currents:
//   * all-zero currents -> all 0 % (no divide-by-zero)
//   * balanced currents -> all 100 %
//   * one dominant wheel -> 100 % vs proportional others
//   * rounding to nearest percent
//
// Build & run (from repository root):
//   g++ -std=c++17 -Wall -Wextra -Iesp32/src -Iesp32/include
//       esp32/src/test_wheel_traction.cpp -o /tmp/t && /tmp/t
//
// NOTE: has its own main(); excluded from the firmware build via
// build_src_filter in esp32/platformio.ini.
// =============================================================================

#include <cstdint>
#include <cstdio>

#include "wheel_traction.h"

static int tests_run = 0, tests_failed = 0;
#define CHECK(cond, msg) do {                                     \
        tests_run++;                                              \
        if (!(cond)) { tests_failed++;                            \
            std::printf("  FAIL: %s\n", msg); }                   \
        else { std::printf("  ok:   %s\n", msg); }                \
    } while (0)

int main() {
    std::printf("=== test_wheel_traction ===\n");

    // ---- All zero currents -> all 0 %, no divide-by-zero -------------------
    {
        const uint16_t raw[4] = {0, 0, 0, 0};
        uint8_t pct[4] = {9, 9, 9, 9};
        traction::computeShare(raw, pct);
        CHECK(pct[0] == 0 && pct[1] == 0 && pct[2] == 0 && pct[3] == 0,
              "all-zero currents -> all 0%");
    }

    // ---- Balanced currents -> all 100 % -----------------------------------
    {
        const uint16_t raw[4] = {500, 500, 500, 500};  // 5.00 A each
        uint8_t pct[4] = {0};
        traction::computeShare(raw, pct);
        CHECK(pct[0] == 100 && pct[1] == 100 && pct[2] == 100 && pct[3] == 100,
              "balanced currents -> all 100%");
    }

    // ---- One dominant wheel, others proportional --------------------------
    {
        // FL 10 A (max), FR 5 A, RL 2.5 A, RR 0 A.
        const uint16_t raw[4] = {1000, 500, 250, 0};
        uint8_t pct[4] = {0};
        traction::computeShare(raw, pct);
        CHECK(pct[0] == 100, "dominant wheel -> 100%");
        CHECK(pct[1] == 50,  "half current -> 50%");
        CHECK(pct[2] == 25,  "quarter current -> 25%");
        CHECK(pct[3] == 0,   "zero current -> 0%");
    }

    // ---- Rounding to nearest percent --------------------------------------
    {
        // 1/3 of max should round to 33%.
        const uint16_t raw[4] = {300, 100, 0, 0};
        uint8_t pct[4] = {0};
        traction::computeShare(raw, pct);
        CHECK(pct[0] == 100, "max -> 100%");
        CHECK(pct[1] == 33,  "100/300 rounds to 33%");
    }

    // ---- Output never exceeds 100 -----------------------------------------
    {
        const uint16_t raw[4] = {65535, 65535, 1, 0};
        uint8_t pct[4] = {0};
        traction::computeShare(raw, pct);
        CHECK(pct[0] == 100 && pct[1] == 100, "large equal maxima clamp to 100%");
        CHECK(pct[2] <= 100 && pct[3] == 0,   "outputs bounded 0..100");
    }

    std::printf("\n%d checks, %d failed\n", tests_run, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
