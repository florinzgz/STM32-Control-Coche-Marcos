// =============================================================================
// ESP32 HMI — host unit test for the pure 0x315 presentation helpers
// (esp32/src/motion_inhibit_view.h).
//
// Covers the conversions used by the Engineering "MOTION INHIBIT DIAG" page:
//   * reason bitmask -> label list (single / multiple / zero motives)
//   * relay phase -> text (IDLE / IN_PROGRESS / COMPLETE / reserved -> UNKNOWN)
//   * age -> VALID / STALE / NEVER RECEIVED, including a millis() wrap
//   * bounded buffer rendering never overflows
//
// Build & run (from repository root):
//   g++ -std=c++17 -Wall -Wextra -Iesp32/src -Iesp32/include
//       esp32/src/test_motion_inhibit_view.cpp -o /tmp/t && /tmp/t
//
// NOTE: has its own main(); excluded from the firmware build via
// build_src_filter in esp32/platformio.ini.
// =============================================================================

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "motion_inhibit_view.h"

namespace miv = motion_inhibit_view;

static int tests_run = 0, tests_failed = 0;
#define CHECK(cond, msg) do {                                     \
        tests_run++;                                              \
        if (!(cond)) { tests_failed++;                            \
            std::printf("  FAIL: %s\n", msg); }                   \
        else { std::printf("  ok:   %s\n", msg); }                \
    } while (0)

int main() {
    std::printf("=== test_motion_inhibit_view ===\n");

    // ---- Freshness: NEVER RECEIVED ----------------------------------------
    {
        // valid == false must be NEVER_RECEIVED no matter the timestamps.
        miv::Freshness f = miv::freshness(false, 10000UL, 0UL);
        CHECK(f == miv::Freshness::NEVER_RECEIVED, "never: valid=false -> NEVER");
        CHECK(std::strcmp(miv::freshnessText(f), "NEVER RECEIVED") == 0,
              "never: text");
        // Even with now == stamp, invalid stays NEVER.
        CHECK(miv::freshness(false, 5000UL, 5000UL) == miv::Freshness::NEVER_RECEIVED,
              "never: invalid ignores age");
    }

    // ---- Freshness: valid recent frame ------------------------------------
    {
        // 50 ms old at 10 Hz -> well within the 300 ms window.
        miv::Freshness f = miv::freshness(true, 10050UL, 10000UL);
        CHECK(f == miv::Freshness::VALID, "valid: 50ms old -> VALID");
        CHECK(std::strcmp(miv::freshnessText(f), "VALID") == 0, "valid: text");
        // Exactly on the boundary (== timeout) is still VALID.
        CHECK(miv::freshness(true, 10300UL, 10000UL) == miv::Freshness::VALID,
              "valid: exactly at timeout boundary is VALID");
        CHECK(miv::ageMs(10050UL, 10000UL) == 50UL, "valid: ageMs == 50");
    }

    // ---- Freshness: transition to STALE -----------------------------------
    {
        // 301 ms old -> just past the 300 ms window.
        miv::Freshness f = miv::freshness(true, 10301UL, 10000UL);
        CHECK(f == miv::Freshness::STALE, "stale: 301ms old -> STALE");
        CHECK(std::strcmp(miv::freshnessText(f), "STALE") == 0, "stale: text");
        // Far older is also STALE.
        CHECK(miv::freshness(true, 20000UL, 10000UL) == miv::Freshness::STALE,
              "stale: 10s old -> STALE");
    }

    // ---- millis() wrap ----------------------------------------------------
    {
        // Stamp just before the 32-bit wrap, now just after it.
        const uint32_t stamp = 0xFFFFFFF0UL;              // -16
        const uint32_t now40 = (uint32_t)(stamp + 40U);   // wraps to 0x00000018
        CHECK(now40 < stamp, "wrap: now numerically less than stamp");
        CHECK(miv::ageMs(now40, stamp) == 40UL, "wrap: ageMs still 40ms");
        CHECK(miv::freshness(true, now40, stamp) == miv::Freshness::VALID,
              "wrap: 40ms across wrap -> VALID");
        const uint32_t now400 = (uint32_t)(stamp + 400U); // 400 ms across the wrap
        CHECK(miv::ageMs(now400, stamp) == 400UL, "wrap: ageMs 400ms across wrap");
        CHECK(miv::freshness(true, now400, stamp) == miv::Freshness::STALE,
              "wrap: 400ms across wrap -> STALE");
    }

    // ---- Reason mask: single motive ---------------------------------------
    {
        const char* labels[miv::REASON_MAX];
        uint8_t n = miv::reasonLabels(can::MOTION_INHIBIT_POWER_NOT_READY,
                                      labels, miv::REASON_MAX);
        CHECK(n == 1, "single: one label");
        CHECK(std::strcmp(labels[0], "POWER NOT READY") == 0, "single: correct label");

        char buf[64];
        uint8_t cnt = miv::reasonListToBuffer(can::MOTION_INHIBIT_OBSTACLE_BLOCK,
                                              buf, sizeof(buf));
        CHECK(cnt == 1, "single: buffer count 1");
        CHECK(std::strcmp(buf, "OBSTACLE BLOCK") == 0, "single: buffer text");
    }

    // ---- Reason mask: multiple motives (table order preserved) ------------
    {
        uint16_t mask = can::MOTION_INHIBIT_GEAR_PARK        // 0x0008 (idx 3)
                      | can::MOTION_INHIBIT_PWM_ZERO         // 0x0100 (idx 8)
                      | can::MOTION_INHIBIT_SERVICE_DISABLED;// 0x8000 (idx 15)
        const char* labels[miv::REASON_MAX];
        uint8_t n = miv::reasonLabels(mask, labels, miv::REASON_MAX);
        CHECK(n == 3, "multi: three labels");
        CHECK(std::strcmp(labels[0], "GEAR PARK") == 0, "multi: order[0]");
        CHECK(std::strcmp(labels[1], "PWM ZERO") == 0, "multi: order[1]");
        CHECK(std::strcmp(labels[2], "SERVICE DISABLED") == 0, "multi: order[2]");

        char buf[80];
        uint8_t cnt = miv::reasonListToBuffer(mask, buf, sizeof(buf));
        CHECK(cnt == 3, "multi: buffer count 3");
        CHECK(std::strcmp(buf, "GEAR PARK, PWM ZERO, SERVICE DISABLED") == 0,
              "multi: joined buffer");
    }

    // ---- Reason mask: 0x0000 ----------------------------------------------
    {
        const char* labels[miv::REASON_MAX];
        uint8_t n = miv::reasonLabels(can::MOTION_INHIBIT_NONE, labels, miv::REASON_MAX);
        CHECK(n == 0, "zero: no labels");

        char buf[16];
        uint8_t cnt = miv::reasonListToBuffer(0x0000, buf, sizeof(buf));
        CHECK(cnt == 0, "zero: buffer count 0");
        CHECK(std::strcmp(buf, "NONE") == 0, "zero: buffer says NONE");
    }

    // ---- Reason mask: all bits set ----------------------------------------
    {
        const char* labels[miv::REASON_MAX];
        uint8_t n = miv::reasonLabels(0xFFFF, labels, miv::REASON_MAX);
        CHECK(n == miv::REASON_MAX, "all: 16 labels");
    }

    // ---- Relay phase text --------------------------------------------------
    {
        CHECK(std::strcmp(miv::relayPhaseText(can::MOTION_INHIBIT_RELAY_SEQ_IDLE),
                          "IDLE") == 0, "relay: IDLE");
        CHECK(std::strcmp(miv::relayPhaseText(can::MOTION_INHIBIT_RELAY_SEQ_IN_PROGRESS),
                          "IN PROGRESS") == 0, "relay: IN PROGRESS");
        CHECK(std::strcmp(miv::relayPhaseText(can::MOTION_INHIBIT_RELAY_SEQ_COMPLETE),
                          "COMPLETE") == 0, "relay: COMPLETE");
        // byte7 bits2-3 can carry value 3 (reserved) -> UNKNOWN.
        CHECK(std::strcmp(miv::relayPhaseText(3), "UNKNOWN") == 0, "relay: reserved 3 -> UNKNOWN");
    }

    // ---- System-state text + motion-permission derivation -----------------
    {
        CHECK(std::strcmp(miv::systemStateText(0), "BOOT") == 0, "state: BOOT");
        CHECK(std::strcmp(miv::systemStateText(1), "STANDBY") == 0, "state: STANDBY");
        CHECK(std::strcmp(miv::systemStateText(2), "ACTIVE") == 0, "state: ACTIVE");
        CHECK(std::strcmp(miv::systemStateText(3), "DEGRADED") == 0, "state: DEGRADED");
        CHECK(std::strcmp(miv::systemStateText(4), "SAFE") == 0, "state: SAFE");
        CHECK(std::strcmp(miv::systemStateText(5), "ERROR") == 0, "state: ERROR");
        CHECK(std::strcmp(miv::systemStateText(6), "LIMP HOME") == 0, "state: LIMP HOME");
        CHECK(std::strcmp(miv::systemStateText(99), "UNKNOWN") == 0, "state: unknown -> UNKNOWN");

        // BOOT/STANDBY/SAFE/ERROR/unknown are no-motion states.
        CHECK(!miv::stateAllowsMotion(0), "motion: BOOT no-motion");
        CHECK(!miv::stateAllowsMotion(1), "motion: STANDBY no-motion");
        CHECK(!miv::stateAllowsMotion(4), "motion: SAFE no-motion");
        CHECK(!miv::stateAllowsMotion(5), "motion: ERROR no-motion");
        CHECK(!miv::stateAllowsMotion(99), "motion: unknown no-motion");
        CHECK(miv::stateAllowsMotion(2), "motion: ACTIVE permits");
        CHECK(miv::stateAllowsMotion(3), "motion: DEGRADED permits");
        CHECK(miv::stateAllowsMotion(6), "motion: LIMP_HOME permits");
    }

    // ---- Gear text ---------------------------------------------------------
    {
        CHECK(std::strcmp(miv::gearText(0), "PARK") == 0, "gear: PARK");
        CHECK(std::strcmp(miv::gearText(4), "FORWARD D2") == 0, "gear: D2");
        CHECK(std::strcmp(miv::gearText(200), "UNKNOWN") == 0, "gear: unknown");
    }

    // ---- Buffer never overflows on a tiny buffer --------------------------
    {
        // Every bit set into a deliberately short buffer must stay bounded and
        // NUL-terminated (bounded snprintf, no dynamic allocation).
        char small[8];
        std::memset(small, 0x7F, sizeof(small));      // poison
        uint8_t cnt = miv::reasonListToBuffer(0xFFFF, small, sizeof(small));
        CHECK(cnt == miv::REASON_MAX, "overflow: reports all 16 active");
        CHECK(small[sizeof(small) - 1] == '\0', "overflow: last byte NUL-terminated");
        CHECK(std::strlen(small) <= sizeof(small) - 1, "overflow: length within bounds");

        // A one-byte buffer can only hold the terminator.
        char one[1];
        one[0] = 0x7F;
        miv::reasonListToBuffer(0xFFFF, one, sizeof(one));
        CHECK(one[0] == '\0', "overflow: 1-byte buffer -> empty NUL");

        // Zero-length / null are handled without writing.
        CHECK(miv::reasonListToBuffer(0xFFFF, nullptr, 0) == 0, "overflow: null buffer safe");
    }

    std::printf("=== %d run, %d failed ===\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
