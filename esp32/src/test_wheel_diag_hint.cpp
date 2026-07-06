// =============================================================================
// ESP32 HMI — host unit test for the Pedal-Calibration wheel-diag hint line.
//
// Exercises the REAL helper esp32/src/ui/wheel_diag_hint.h (buildWheelBlockText
// + wheelReasonShort) on host g++ — no Arduino/TFT needed.
//
// Build & run (from repository root):
//   g++ -std=c++17 -DHOST_TEST -Iesp32/include -Iesp32/src/ui
//       esp32/src/test_wheel_diag_hint.cpp
//       -o /tmp/test_wheel_diag_hint && /tmp/test_wheel_diag_hint
//
// NOTE: has its own main(); excluded from the firmware build via
// build_src_filter in esp32/platformio.ini.
// =============================================================================

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "wheel_diag_hint.h"   // real helper under test

using ui::WheelHintKind;
using ui::buildWheelBlockText;
using ui::wheelReasonShort;

// ---- Tiny test framework ----
static int tests_run = 0, tests_failed = 0;
#define CHECK(cond, msg) do {                                     \
        tests_run++;                                              \
        if (!(cond)) { tests_failed++;                            \
            std::printf("  FAIL: %s\n", msg); }                   \
        else { std::printf("  ok:   %s\n", msg); }                \
    } while (0)

static bool contains(const char* hay, const char* needle) {
    return std::strstr(hay, needle) != nullptr;
}

int main() {
    std::printf("=== test_wheel_diag_hint ===\n");

    // 1. Active fault: FR + MISMATCH  → "WD: FR MIS", red.
    {
        uint8_t reason[5] = {0, can::WHEEL_DIAG_REASON_MISMATCH, 0, 0, 0};
        char b[48];
        WheelHintKind k = buildWheelBlockText(reason, 0x02, true, 0, false, b, sizeof(b));
        CHECK(contains(b, "FR MIS"), "faultMask FR MISMATCH -> contains 'FR MIS'");
        CHECK(k == WheelHintKind::FAULT, "faultMask FR -> FAULT (red)");
        CHECK(ui::wheelHintColor(k) == ui::COL_RED, "FAULT colour is red");
    }

    // 2. Active fault: RL + NO_PULSE  → "WD: RL NP".
    {
        uint8_t reason[5] = {0, 0, can::WHEEL_DIAG_REASON_NO_PULSE, 0, 0};
        char b[48];
        WheelHintKind k = buildWheelBlockText(reason, 0x04, true, 0, false, b, sizeof(b));
        CHECK(contains(b, "RL NP"), "faultMask RL NO_PULSE -> contains 'RL NP'");
        CHECK(k == WheelHintKind::FAULT, "faultMask RL -> FAULT");
    }

    // 2b. Two active faults: FR MISMATCH + RL NO_PULSE.
    {
        uint8_t reason[5] = {0, can::WHEEL_DIAG_REASON_MISMATCH,
                             can::WHEEL_DIAG_REASON_NO_PULSE, 0, 0};
        char b[48];
        WheelHintKind k = buildWheelBlockText(reason, 0x06, true, 0, false, b, sizeof(b));
        CHECK(contains(b, "FR MIS") && contains(b, "RL NP"),
              "two faults -> contains both 'FR MIS' and 'RL NP'");
        CHECK(k == WheelHintKind::FAULT, "two faults -> FAULT");
    }

    // 3. No fault + MANUAL_MOVEMENT  → "WD: MAN", cyan (never red).
    {
        uint8_t reason[5] = {can::WHEEL_DIAG_REASON_MANUAL_MOVEMENT, 0, 0, 0, 0};
        char b[48];
        WheelHintKind k = buildWheelBlockText(reason, 0x00, true, 0, false, b, sizeof(b));
        CHECK(contains(b, "MAN"), "manual movement -> contains 'MAN'");
        CHECK(k == WheelHintKind::INFO, "manual movement -> INFO (not FAULT)");
        CHECK(ui::wheelHintColor(k) == ui::COL_CYAN, "INFO colour is cyan");
    }

    // 4. No fault + all OK/DISABLED  → "WD: OK", green.
    {
        uint8_t reason[5] = {can::WHEEL_DIAG_REASON_OK, can::WHEEL_DIAG_REASON_DISABLED_STATE,
                             can::WHEEL_DIAG_REASON_OK, can::WHEEL_DIAG_REASON_DISABLED_STATE,
                             can::WHEEL_DIAG_REASON_OK};
        char b[48];
        WheelHintKind k = buildWheelBlockText(reason, 0x00, true, 0, false, b, sizeof(b));
        CHECK(std::strcmp(b, "WD: OK") == 0, "all OK/DIS -> 'WD: OK'");
        CHECK(k == WheelHintKind::OK, "all OK/DIS -> OK (green)");
        CHECK(ui::wheelHintColor(k) == ui::COL_GREEN, "OK colour is green");
    }

    // 5. valid == false  → "WD: NO DATA", gray.
    {
        uint8_t reason[5] = {0, 0, 0, 0, 0};
        char b[48];
        WheelHintKind k = buildWheelBlockText(reason, 0x00, false, 0, false, b, sizeof(b));
        CHECK(std::strcmp(b, "WD: NO DATA") == 0, "invalid -> 'WD: NO DATA'");
        CHECK(k == WheelHintKind::NODATA, "invalid -> NODATA");
    }

    // 6. Stale (age > 2 s)  → "WD: STALE".  Stale wins over an active fault.
    {
        uint8_t reason[5] = {0, can::WHEEL_DIAG_REASON_MISMATCH, 0, 0, 0};
        char b[48];
        WheelHintKind k = buildWheelBlockText(reason, 0x02, true, 2001, false, b, sizeof(b));
        CHECK(std::strcmp(b, "WD: STALE") == 0, "age>2s -> 'WD: STALE'");
        CHECK(k == WheelHintKind::STALE, "age>2s -> STALE");
    }

    // 7. DTC latched but faultMask == 0  → "WD: CLR DTC", amber.
    {
        uint8_t reason[5] = {can::WHEEL_DIAG_REASON_OK, can::WHEEL_DIAG_REASON_OK,
                             can::WHEEL_DIAG_REASON_OK, can::WHEEL_DIAG_REASON_OK, 0};
        char b[48];
        WheelHintKind k = buildWheelBlockText(reason, 0x00, true, 0, true, b, sizeof(b));
        CHECK(contains(b, "CLR DTC"), "latched DTC + no live fault -> 'CLR DTC'");
        CHECK(k == WheelHintKind::WARN, "CLR DTC -> WARN (amber)");
        CHECK(ui::wheelHintColor(k) == ui::COL_AMBER, "WARN colour is amber");
    }

    // 8. No buffer overflow: all four wheels faulting into a tiny buffer.
    {
        uint8_t reason[5] = {can::WHEEL_DIAG_REASON_STUCK_HIGH, can::WHEEL_DIAG_REASON_STUCK_HIGH,
                             can::WHEEL_DIAG_REASON_STUCK_HIGH, can::WHEEL_DIAG_REASON_STUCK_HIGH,
                             can::WHEEL_DIAG_REASON_IMPOSSIBLE_RATE};
        char b[10];
        std::memset(b, 0x7F, sizeof(b));
        WheelHintKind k = buildWheelBlockText(reason, 0x1F, true, 0, false, b, sizeof(b));
        // Must stay NUL-terminated within bounds and never write past b[9].
        bool terminated = false;
        for (size_t i = 0; i < sizeof(b); ++i) { if (b[i] == '\0') { terminated = true; break; } }
        CHECK(terminated, "tiny buffer stays NUL-terminated (no overflow)");
        CHECK(k == WheelHintKind::FAULT, "tiny buffer still classified FAULT");
    }

    // 9. Fault priority: faultMask wins even if a MANUAL reason is also present.
    {
        uint8_t reason[5] = {can::WHEEL_DIAG_REASON_MANUAL_MOVEMENT,
                             can::WHEEL_DIAG_REASON_STUCK_LOW, 0, 0, 0};
        char b[48];
        WheelHintKind k = buildWheelBlockText(reason, 0x02, true, 0, false, b, sizeof(b));
        CHECK(contains(b, "FR S-LO"), "faultMask beats manual -> 'FR S-LO'");
        CHECK(k == WheelHintKind::FAULT, "faultMask beats manual -> FAULT");
    }

    // 9b. Small buffer that fits the prefix + one full segment but truncates the
    //     second: output must be a clean, NUL-terminated prefix (no overflow,
    //     no garbage past the NUL) — exercises the snprintf pointer/remain path.
    {
        uint8_t reason[5] = {can::WHEEL_DIAG_REASON_MISMATCH,
                             can::WHEEL_DIAG_REASON_NO_PULSE, 0, 0, 0};
        char b[12];                 // "WD: FL MIS" (10) + NUL fits; " FR NP" won't
        std::memset(b, 0x7F, sizeof(b));
        WheelHintKind k = buildWheelBlockText(reason, 0x03, true, 0, false, b, sizeof(b));
        bool terminated = false;
        size_t nulAt = sizeof(b);
        for (size_t i = 0; i < sizeof(b); ++i) { if (b[i] == '\0') { terminated = true; nulAt = i; break; } }
        CHECK(terminated, "small buffer stays NUL-terminated");
        CHECK(nulAt < sizeof(b), "NUL is within bounds");
        CHECK(contains(b, "WD: FL MIS"), "small buffer keeps clean prefix 'WD: FL MIS'");
        CHECK(k == WheelHintKind::FAULT, "small buffer still classified FAULT");
    }

    // 10. Short-reason mnemonic table matches the spec.
    {
        CHECK(std::strcmp(wheelReasonShort(can::WHEEL_DIAG_REASON_OK), "OK") == 0, "short OK");
        CHECK(std::strcmp(wheelReasonShort(can::WHEEL_DIAG_REASON_NO_PULSE), "NP") == 0, "short NP");
        CHECK(std::strcmp(wheelReasonShort(can::WHEEL_DIAG_REASON_STUCK_HIGH), "S-HI") == 0, "short S-HI");
        CHECK(std::strcmp(wheelReasonShort(can::WHEEL_DIAG_REASON_STUCK_LOW), "S-LO") == 0, "short S-LO");
        CHECK(std::strcmp(wheelReasonShort(can::WHEEL_DIAG_REASON_MISMATCH), "MIS") == 0, "short MIS");
        CHECK(std::strcmp(wheelReasonShort(can::WHEEL_DIAG_REASON_IMPOSSIBLE_RATE), "RATE") == 0, "short RATE");
        CHECK(std::strcmp(wheelReasonShort(can::WHEEL_DIAG_REASON_MANUAL_MOVEMENT), "MAN") == 0, "short MAN");
        CHECK(std::strcmp(wheelReasonShort(can::WHEEL_DIAG_REASON_DISABLED_STATE), "DIS") == 0, "short DIS");
        CHECK(std::strcmp(wheelReasonShort(can::WHEEL_DIAG_REASON_UNKNOWN), "UNK") == 0, "short UNK");
        CHECK(std::strcmp(wheelReasonShort(200), "UNK") == 0, "out-of-range -> UNK");
    }

    std::printf("=== %d run, %d failed ===\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
