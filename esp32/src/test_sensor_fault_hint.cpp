// =============================================================================
// ESP32 HMI — host unit test for the SENSOR_FAULT (code 4) cause hint.
//
// Exercises the REAL helper esp32/src/ui/sensor_fault_hint.h
// (buildSensorFaultHint) on host g++ — no Arduino/TFT needed.
//
// Build & run (from repository root):
//   g++ -std=c++17 -DHOST_TEST -Iesp32/src/ui
//       esp32/src/test_sensor_fault_hint.cpp
//       -o /tmp/test_sensor_fault_hint && /tmp/test_sensor_fault_hint
//
// NOTE: has its own main(); excluded from the firmware build via
// build_src_filter in esp32/platformio.ini.
// =============================================================================

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "sensor_fault_hint.h"   // real helper under test

using ui::SensorFaultCause;
using ui::buildSensorFaultHint;

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
    std::printf("=== test_sensor_fault_hint ===\n");

    // 1. Not SENSOR_FAULT (e.g. OVERCURRENT=1): NONE, buffer untouched (empty).
    {
        char b[48];
        SensorFaultCause c = buildSensorFaultHint(
            1, true, false, true, true, 0, 0, true, b, sizeof(b));
        CHECK(c == SensorFaultCause::NONE, "errorCode != 4 -> NONE");
        CHECK(b[0] == '\0', "errorCode != 4 -> buffer empty");
    }

    // 2. Fresh pedal contradictory -> highest priority, even with a live
    //    INA226 bit also set.
    {
        char b[48];
        uint32_t faultMask = 1UL << 9;  // CURRENT_SENSOR_0 (FL)
        SensorFaultCause c = buildSensorFaultHint(
            4, /*extended*/true, /*plausible*/false, /*contradictory*/true, /*fresh*/true,
            faultMask, 0, true, b, sizeof(b));
        CHECK(c == SensorFaultCause::PEDAL, "fresh pedal contradictory -> PEDAL");
        CHECK(contains(b, "CONTRADICTORY"), "pedal contradictory -> contains 'CONTRADICTORY'");
    }

    // 3. Fresh pedal implausible (not contradictory).
    {
        char b[48];
        SensorFaultCause c = buildSensorFaultHint(
            4, true, false, false, true, 0, 0, true, b, sizeof(b));
        CHECK(c == SensorFaultCause::PEDAL, "fresh pedal implausible -> PEDAL");
        CHECK(contains(b, "IMPLAUSIBLE"), "pedal implausible -> contains 'IMPLAUSIBLE'");
    }

    // 4. Pedal plausible==false but NOT extended (legacy single-byte 0x20B
    //    frame; the fault bits are not actually present) -> ignored, falls
    //    through to service mask.
    {
        char b[48];
        uint32_t faultMask = 1UL << 15;  // WHEEL_SPEED_FL
        SensorFaultCause c = buildSensorFaultHint(
            4, /*extended*/false, /*plausible*/false, /*contradictory*/false, /*fresh*/true,
            faultMask, 0, true, b, sizeof(b));
        CHECK(c == SensorFaultCause::WHEEL, "pedal not extended -> ignored, falls through");
    }

    // 5. Pedal plausible==false but STALE (>2s) -> ignored, falls through.
    {
        char b[48];
        uint32_t faultMask = 1UL << 9;  // CURRENT_SENSOR_0 (FL)
        SensorFaultCause c = buildSensorFaultHint(
            4, true, false, true, /*fresh*/false,
            faultMask, 0, true, b, sizeof(b));
        CHECK(c == SensorFaultCause::CURRENT, "stale pedal fault -> ignored, falls through");
    }

    // 6. Pedal plausible==true (no pedal fault) -> falls through to service mask.
    // INA226 CH0 (FL) live fault.
    {
        char b[48];
        uint32_t faultMask = 1UL << 9;  // CURRENT_SENSOR_0 (FL)
        SensorFaultCause c = buildSensorFaultHint(
            4, true, true, false, true, faultMask, 0, true, b, sizeof(b));
        CHECK(c == SensorFaultCause::CURRENT, "INA226 CH0 live -> CURRENT");
        CHECK(contains(b, "INA226 CH0") && contains(b, "FL"), "-> contains 'INA226 CH0' and 'FL'");
    }

    // 6b. INA226 CH5 (steering) live fault -> correct axis label.
    {
        char b[48];
        uint32_t faultMask = 1UL << 14;  // CURRENT_SENSOR_5 (steer)
        SensorFaultCause c = buildSensorFaultHint(
            4, true, true, false, true, faultMask, 0, true, b, sizeof(b));
        CHECK(c == SensorFaultCause::CURRENT, "INA226 CH5 live -> CURRENT");
        CHECK(contains(b, "INA226 CH5") && contains(b, "Steering"),
              "-> contains 'INA226 CH5' and 'Steering'");
    }

    // 7. INA226 bit set but ALSO in the disabled mask -> module was
    //    manually disabled, must NOT be reported as the live cause.
    //    No other bit set -> NONE.
    {
        char b[48];
        uint32_t faultMask    = 1UL << 9;   // CURRENT_SENSOR_0 latched (stale, pre-disable)
        uint32_t disabledMask = 1UL << 9;   // ... but now manually disabled
        SensorFaultCause c = buildSensorFaultHint(
            4, true, true, false, true, faultMask, disabledMask, true, b, sizeof(b));
        CHECK(c == SensorFaultCause::NONE, "disabled module bit -> skipped -> NONE");
        CHECK(b[0] == '\0', "disabled-only fault -> buffer empty");
    }

    // 7b. Disabled CH0 masked out, but CH1 (FR) still genuinely live -> FR reported.
    {
        char b[48];
        uint32_t faultMask    = (1UL << 9) | (1UL << 10);  // CH0 + CH1
        uint32_t disabledMask = 1UL << 9;                   // CH0 disabled
        SensorFaultCause c = buildSensorFaultHint(
            4, true, true, false, true, faultMask, disabledMask, true, b, sizeof(b));
        CHECK(c == SensorFaultCause::CURRENT, "CH0 disabled, CH1 live -> CURRENT");
        CHECK(contains(b, "INA226 CH1") && contains(b, "FR"), "-> contains 'INA226 CH1' and 'FR'");
    }

    // 8. Temp sensor 2 fault (no current-sensor bit set).
    {
        char b[48];
        uint32_t faultMask = 1UL << 6;  // TEMP_SENSOR_2 (bit 4+2)
        SensorFaultCause c = buildSensorFaultHint(
            4, true, true, false, true, faultMask, 0, true, b, sizeof(b));
        CHECK(c == SensorFaultCause::TEMP, "temp sensor 2 -> TEMP");
        CHECK(contains(b, "Temp Sensor 2"), "-> contains 'Temp Sensor 2'");
    }

    // 9. Wheel speed FR fault fallback (no pedal/current/temp fault).
    {
        char b[48];
        uint32_t faultMask = 1UL << 16;  // WHEEL_SPEED_FR (bit 15+1)
        SensorFaultCause c = buildSensorFaultHint(
            4, true, true, false, true, faultMask, 0, true, b, sizeof(b));
        CHECK(c == SensorFaultCause::WHEEL, "wheel speed FR -> WHEEL");
        CHECK(contains(b, "Wheel FR"), "-> contains 'Wheel FR'");
    }

    // 10. Priority: current-sensor bit wins over a simultaneously-set temp bit.
    {
        char b[48];
        uint32_t faultMask = (1UL << 9) | (1UL << 4);  // CURRENT_SENSOR_0 + TEMP_SENSOR_0
        SensorFaultCause c = buildSensorFaultHint(
            4, true, true, false, true, faultMask, 0, true, b, sizeof(b));
        CHECK(c == SensorFaultCause::CURRENT, "current+temp both set -> CURRENT wins");
    }

    // 11. Stale service mask (>2s) with no pedal fault -> NONE, buffer empty.
    {
        char b[48];
        uint32_t faultMask = 1UL << 9;
        SensorFaultCause c = buildSensorFaultHint(
            4, true, true, false, true, faultMask, 0, /*serviceFresh*/false, b, sizeof(b));
        CHECK(c == SensorFaultCause::NONE, "stale service mask -> NONE");
        CHECK(b[0] == '\0', "stale service mask -> buffer empty");
    }

    // 12. Nothing at all set -> NONE, buffer empty (plain generic text kept).
    {
        char b[48];
        SensorFaultCause c = buildSensorFaultHint(
            4, true, true, false, true, 0, 0, true, b, sizeof(b));
        CHECK(c == SensorFaultCause::NONE, "no fault bits -> NONE");
        CHECK(b[0] == '\0', "no fault bits -> buffer empty");
    }

    // 13. No buffer overflow: tiny buffer stays NUL-terminated.
    {
        char b[6];
        std::memset(b, 0x7F, sizeof(b));
        uint32_t faultMask = 1UL << 13;  // CURRENT_SENSOR_4 (battery) — long label
        SensorFaultCause c = buildSensorFaultHint(
            4, true, true, false, true, faultMask, 0, true, b, sizeof(b));
        bool terminated = false;
        for (size_t i = 0; i < sizeof(b); ++i) { if (b[i] == '\0') { terminated = true; break; } }
        CHECK(terminated, "tiny buffer stays NUL-terminated (no overflow)");
        CHECK(c == SensorFaultCause::CURRENT, "tiny buffer still classified CURRENT");
    }

    // 14. Null/zero-size buffer guard.
    {
        SensorFaultCause c1 = buildSensorFaultHint(4, true, false, true, true, 0, 0, true, nullptr, 10);
        CHECK(c1 == SensorFaultCause::NONE, "null buffer -> NONE (no crash)");
        char b[4];
        SensorFaultCause c2 = buildSensorFaultHint(4, true, false, true, true, 0, 0, true, b, 0);
        CHECK(c2 == SensorFaultCause::NONE, "zero-size buffer -> NONE (no crash)");
    }

    std::printf("=== %d run, %d failed ===\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
