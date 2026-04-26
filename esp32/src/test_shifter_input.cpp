/**
 ****************************************************************************
 * @file    test_shifter_input.cpp
 * @brief   Host-compilable unit tests for shifter_input I2C error handling.
 *
 *          Tests the I2C error detection, backoff, and reconnection logic
 *          without real hardware.  Arduino and Wire API calls are replaced
 *          by lightweight stubs in ../test_stubs/.
 *
 *          Compile and run on host (from esp32/src/):
 *            g++ -std=c++17 -I. -I../test_stubs \
 *                shifter_input.cpp test_shifter_input.cpp \
 *                -o /tmp/test_shifter_input && /tmp/test_shifter_input
 *
 *          Requirements covered:
 *            1. readReg() checks endTransmission() before requestFrom()
 *            2. Consecutive I2C errors trigger backoff (1 s poll interval)
 *            3. Gear defaults to NEUTRAL on I2C failure
 *            4. Successful read after errors restores normal operation
 *            5. isConnected() tracks MCP23017 presence
 *            6. init() detects missing device at startup
 *            7. Normal gear reading still works (Park via GPA0 active-low)
 ****************************************************************************
 */

#include <Arduino.h>
#include <Wire.h>
#include <cstdio>
#include "shifter_input.h"

/* ---- Test harness ------------------------------------------------------- */

static int s_tests_run    = 0;
static int s_tests_failed = 0;

#define ASSERT(cond) do {                                                    \
    s_tests_run++;                                                           \
    if (!(cond)) {                                                           \
        printf("FAIL  %s:%d  Assertion: %s\n", __FILE__, __LINE__, #cond);  \
        s_tests_failed++;                                                    \
    }                                                                        \
} while (0)

#define ASSERT_EQ(a, b) do {                                                 \
    s_tests_run++;                                                           \
    if ((a) != (b)) {                                                        \
        printf("FAIL  %s:%d  %s == %ld (expected %ld)\n",                   \
               __FILE__, __LINE__, #a,                                       \
               (long)(a), (long)(b));                                        \
        s_tests_failed++;                                                    \
    }                                                                        \
} while (0)

/* Helper: reset all stubs and init the shifter */
static void reset_and_init(bool i2c_ok = true) {
    g_test_millis         = 0;
    g_wire_end_result     = i2c_ok ? 0 : 2;  // 0=ok, 2=NACK
    g_wire_read_value     = 0xFF;
    g_wire_request_result = i2c_ok ? 1 : 0;
    shifter::init();
}

/* Helper: advance time and update */
static void tick(unsigned long ms) {
    g_test_millis = ms;
    shifter::update();
}

/* ====================================================================== */
/* TEST 1 — init() with I2C success                                       */
/* ====================================================================== */
static void test_init_success() {
    printf("  test_init_success...\n");
    reset_and_init(true);

    ASSERT(shifter::isConnected());
    ASSERT_EQ((int)shifter::getGear(), (int)shifter::Gear::NEUTRAL);
    ASSERT_EQ((int)shifter::getGearRaw(), 2);
}

/* ====================================================================== */
/* TEST 2 — init() with I2C failure (device not present)                  */
/* ====================================================================== */
static void test_init_failure() {
    printf("  test_init_failure...\n");
    reset_and_init(false);

    ASSERT(!shifter::isConnected());
    ASSERT_EQ((int)shifter::getGear(), (int)shifter::Gear::NEUTRAL);
}

/* ====================================================================== */
/* TEST 3 — Normal gear read: Park (GPA0 active-low = 0xFE)              */
/* ====================================================================== */
static void test_read_park() {
    printf("  test_read_park...\n");
    reset_and_init(true);

    // GPA0 active-low: bit 0 = 0 → Park.  Port value = 0b11111110 = 0xFE
    g_wire_read_value = 0xFE;
    tick(100);

    ASSERT_EQ((int)shifter::getGear(), (int)shifter::Gear::PARK);
    ASSERT(shifter::isConnected());
}

/* ====================================================================== */
/* TEST 4 — Normal gear read: Reverse (GPA3 active-low = 0xF7)           */
/* Blue+white wire → GPA3.                                                */
/* ====================================================================== */
static void test_read_reverse() {
    printf("  test_read_reverse...\n");
    reset_and_init(true);

    // GPA3 active-low: bit 3 = 0 → Reverse.  Port value = 0b11110111 = 0xF7
    g_wire_read_value = 0xF7;
    tick(100);

    ASSERT_EQ((int)shifter::getGear(), (int)shifter::Gear::REVERSE);
}

/* ====================================================================== */
/* TEST 4b — Neutral is implicit (all contacts open = 0xFF in low nibble) */
/* Lever in N position: no contact closes, all GPA0-3 remain HIGH.       */
/* ====================================================================== */
static void test_read_neutral_implicit() {
    printf("  test_read_neutral_implicit...\n");
    reset_and_init(true);

    // All pins HIGH (0xFF) — lever in N, no contact active
    g_wire_read_value = 0xFF;
    tick(100);

    ASSERT_EQ((int)shifter::getGear(), (int)shifter::Gear::NEUTRAL);
    ASSERT(shifter::isConnected());
}

/* ====================================================================== */
/* TEST 5 — I2C error triggers backoff after ERROR_THRESHOLD failures     */
/* ====================================================================== */
static void test_error_backoff() {
    printf("  test_error_backoff...\n");
    reset_and_init(true);

    // Simulate I2C failure
    g_wire_end_result     = 2;  // NACK
    g_wire_request_result = 0;

    // Consecutive errors with pollMs=50: at 50, 100, 150, 200, 250
    tick(50);   // error 1
    ASSERT(shifter::isConnected());  // not yet in backoff

    tick(100);  // error 2
    tick(150);  // error 3
    tick(200);  // error 4
    ASSERT(shifter::isConnected());  // still not in backoff (threshold=5)

    tick(250);  // error 5 — threshold reached
    ASSERT(!shifter::isConnected());
    ASSERT_EQ((int)shifter::getGear(), (int)shifter::Gear::NEUTRAL);

    // After backoff: next poll should not happen until 1250ms (250 + 1000)
    g_wire_end_result     = 0;
    g_wire_read_value     = 0xFE;  // Park
    g_wire_request_result = 1;

    tick(500);  // too soon — still in backoff interval
    ASSERT(!shifter::isConnected());  // should NOT have polled yet

    tick(1250);  // 250 + 1000 = 1250, backoff expired
    ASSERT(shifter::isConnected());  // recovered
    ASSERT_EQ((int)shifter::getGear(), (int)shifter::Gear::PARK);
}

/* ====================================================================== */
/* TEST 6 — Recovery after I2C reconnection                               */
/* ====================================================================== */
static void test_recovery() {
    printf("  test_recovery...\n");
    reset_and_init(true);

    // Fail 5 times to enter backoff
    g_wire_end_result     = 2;
    g_wire_request_result = 0;
    tick(50);
    tick(100);
    tick(150);
    tick(200);
    tick(250);
    ASSERT(!shifter::isConnected());

    // Device comes back
    g_wire_end_result     = 0;
    g_wire_read_value     = 0xFB;  // Forward/D1 (GPA2 active-low)
    g_wire_request_result = 1;

    tick(1250);  // backoff expired
    ASSERT(shifter::isConnected());
    ASSERT_EQ((int)shifter::getGear(), (int)shifter::Gear::FORWARD);
}

/* ====================================================================== */
/* TEST 7 — Gear defaults to NEUTRAL during I2C error                     */
/* ====================================================================== */
static void test_neutral_on_error() {
    printf("  test_neutral_on_error...\n");
    reset_and_init(true);

    // First set a valid gear
    g_wire_read_value = 0xFE;  // Park
    tick(50);
    ASSERT_EQ((int)shifter::getGear(), (int)shifter::Gear::PARK);

    // Now I2C fails
    g_wire_end_result     = 2;
    g_wire_request_result = 0;
    tick(100);

    // Gear should revert to NEUTRAL
    ASSERT_EQ((int)shifter::getGear(), (int)shifter::Gear::NEUTRAL);
}

/* ====================================================================== */
/* TEST 8 — isConnected() reflects init state correctly                   */
/* ====================================================================== */
static void test_connected_reflects_init() {
    printf("  test_connected_reflects_init...\n");
    reset_and_init(true);
    ASSERT(shifter::isConnected());

    reset_and_init(false);
    ASSERT(!shifter::isConnected());
}

/* ====================================================================== */
/* TEST 9 — Init failure goes straight to backoff polling                  */
/* ====================================================================== */
static void test_init_failure_backoff() {
    printf("  test_init_failure_backoff...\n");
    reset_and_init(false);

    ASSERT(!shifter::isConnected());

    // Device stays absent — update should use backoff interval (1000ms)
    g_wire_end_result     = 2;
    g_wire_request_result = 0;
    tick(500);  // too soon for backoff poll
    ASSERT(!shifter::isConnected());

    // Device appears after backoff — lever in N (no contact active)
    g_wire_end_result     = 0;
    g_wire_read_value     = 0xFF;  // Neutral (no contact closed — all pins HIGH)
    g_wire_request_result = 1;
    tick(1000);  // backoff expired
    ASSERT(shifter::isConnected());
    ASSERT_EQ((int)shifter::getGear(), (int)shifter::Gear::NEUTRAL);
}

/* ====================================================================== */
/* Main                                                                    */
/* ====================================================================== */

int main(void) {
    printf("Running shifter_input unit tests...\n\n");

    test_init_success();
    test_init_failure();
    test_read_park();
    test_read_reverse();
    test_read_neutral_implicit();
    test_error_backoff();
    test_recovery();
    test_neutral_on_error();
    test_connected_reflects_init();
    test_init_failure_backoff();

    printf("\n--- shifter_input tests: %d run, %d failed ---\n",
           s_tests_run, s_tests_failed);

    return s_tests_failed ? 1 : 0;
}
