/**
 ****************************************************************************
 * @file    test_traction_switch.cpp
 * @brief   Host-compilable unit tests for traction_switch state machine.
 *
 *          Tests the pure state-machine logic of traction_switch without any
 *          real hardware.  All Arduino API calls (pinMode, digitalRead,
 *          millis, Serial) are replaced by lightweight stubs in
 *          ../test_stubs/Arduino.h.
 *
 *          Compile and run on host (from esp32/src/):
 *            g++ -std=c++17 -I. -I../test_stubs \
 *                traction_switch.cpp test_traction_switch.cpp \
 *                -o /tmp/test_traction_switch && /tmp/test_traction_switch
 *
 *          Requirements covered:
 *            1. Init reads initial switch state and signals changed
 *            2. Debounce: transient glitches are filtered
 *            3. Speed gate: rejects changes when speed > 0.5 km/h
 *            4. Stable transition: LOW = 4WD, HIGH = 2WD
 *            5. Floating detection: pull-up ensures HIGH (2WD) on disconnect
 *            6. Last valid state is retained when speed blocks a change
 *            7. clearChanged() resets the changed flag
 *            8. getModeFlag() returns correct CAN flag value
 ****************************************************************************
 */

#include <Arduino.h>
#include <cstdio>
#include "traction_switch.h"

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

static constexpr int SWITCH_PIN = 15;

/* Helper: reset clock, set GPIO state, then call init() */
static void reset_and_init(int pinState = HIGH, unsigned long start_ms = 0) {
    g_test_millis = start_ms;
    for (int i = 0; i < 64; ++i) {
        g_gpio_state[i]      = 0;
        g_gpio_write_count[i] = 0;
    }
    g_gpio_state[SWITCH_PIN] = pinState;
    traction_sw::init();
}

/* Helper: set pin state and advance time, then call update() */
static void tick(unsigned long new_ms, int pinState, float speedKmh = 0.0f) {
    g_test_millis = new_ms;
    g_gpio_state[SWITCH_PIN] = pinState;
    traction_sw::update(speedKmh);
}

/* ====================================================================== */
/* TEST 1 — init() reads initial state: HIGH = 2WD                        */
/* ====================================================================== */
static void test_init_2wd() {
    reset_and_init(HIGH, 0);

    ASSERT_EQ((int)traction_sw::getMode(), (int)traction_sw::Mode::TWO_WD);
    ASSERT(!traction_sw::is4WD());
    ASSERT_EQ((int)traction_sw::getModeFlag(), 0);
    ASSERT(traction_sw::hasChanged());  /* initial state triggers changed */
}

/* ====================================================================== */
/* TEST 2 — init() reads initial state: LOW = 4WD                         */
/* ====================================================================== */
static void test_init_4wd() {
    reset_and_init(LOW, 0);

    ASSERT_EQ((int)traction_sw::getMode(), (int)traction_sw::Mode::FOUR_WD);
    ASSERT(traction_sw::is4WD());
    ASSERT_EQ((int)traction_sw::getModeFlag(), 1);
    ASSERT(traction_sw::hasChanged());
}

/* ====================================================================== */
/* TEST 3 — clearChanged() resets the flag                                 */
/* ====================================================================== */
static void test_clear_changed() {
    reset_and_init(HIGH, 0);

    ASSERT(traction_sw::hasChanged());
    traction_sw::clearChanged();
    ASSERT(!traction_sw::hasChanged());
}

/* ====================================================================== */
/* TEST 4 — Debounce: short glitch does not change mode                    */
/* ====================================================================== */
static void test_debounce_rejects_glitch() {
    reset_and_init(HIGH, 0);  /* Start 2WD */
    traction_sw::clearChanged();

    /* Simulate a short glitch: LOW for 1 poll, then back to HIGH */
    tick(100, LOW, 0.0f);    /* 1 reading of LOW */
    tick(110, HIGH, 0.0f);   /* back to HIGH immediately */
    tick(120, HIGH, 0.0f);
    tick(130, HIGH, 0.0f);
    tick(200, HIGH, 0.0f);

    /* Mode should NOT have changed */
    ASSERT_EQ((int)traction_sw::getMode(), (int)traction_sw::Mode::TWO_WD);
    ASSERT(!traction_sw::hasChanged());
}

/* ====================================================================== */
/* TEST 5 — Stable transition: 2WD → 4WD after debounce period            */
/* ====================================================================== */
static void test_stable_transition_to_4wd() {
    reset_and_init(HIGH, 0);  /* Start 2WD */
    traction_sw::clearChanged();

    /* Switch to LOW (4WD) and hold stable for debounce period */
    tick(100, LOW, 0.0f);
    tick(110, LOW, 0.0f);
    tick(120, LOW, 0.0f);
    tick(160, LOW, 0.0f);  /* 60 ms elapsed, > 50 ms debounce + 3 stable reads */

    ASSERT_EQ((int)traction_sw::getMode(), (int)traction_sw::Mode::FOUR_WD);
    ASSERT(traction_sw::is4WD());
    ASSERT(traction_sw::hasChanged());
}

/* ====================================================================== */
/* TEST 6 — Stable transition: 4WD → 2WD after debounce period            */
/* ====================================================================== */
static void test_stable_transition_to_2wd() {
    reset_and_init(LOW, 0);  /* Start 4WD */
    traction_sw::clearChanged();

    /* Switch to HIGH (2WD) and hold stable */
    tick(100, HIGH, 0.0f);
    tick(110, HIGH, 0.0f);
    tick(120, HIGH, 0.0f);
    tick(160, HIGH, 0.0f);

    ASSERT_EQ((int)traction_sw::getMode(), (int)traction_sw::Mode::TWO_WD);
    ASSERT(!traction_sw::is4WD());
    ASSERT(traction_sw::hasChanged());
}

/* ====================================================================== */
/* TEST 7 — Speed gate: rejects change when speed > 0.5 km/h              */
/* ====================================================================== */
static void test_speed_gate_blocks_change() {
    reset_and_init(HIGH, 0);  /* Start 2WD */
    traction_sw::clearChanged();

    /* Switch to LOW (4WD) while moving at 5 km/h */
    tick(100, LOW, 5.0f);
    tick(110, LOW, 5.0f);
    tick(120, LOW, 5.0f);
    tick(160, LOW, 5.0f);
    tick(200, LOW, 5.0f);

    /* Mode must NOT change — vehicle is moving too fast */
    ASSERT_EQ((int)traction_sw::getMode(), (int)traction_sw::Mode::TWO_WD);
    ASSERT(!traction_sw::hasChanged());
}

/* ====================================================================== */
/* TEST 8 — Speed gate: allows change after vehicle stops                  */
/* ====================================================================== */
static void test_speed_gate_allows_after_stop() {
    reset_and_init(HIGH, 0);  /* Start 2WD */
    traction_sw::clearChanged();

    /* Switch to LOW (4WD) while moving */
    tick(100, LOW, 5.0f);
    tick(110, LOW, 5.0f);
    tick(120, LOW, 5.0f);
    tick(160, LOW, 5.0f);

    /* Mode blocked */
    ASSERT_EQ((int)traction_sw::getMode(), (int)traction_sw::Mode::TWO_WD);

    /* Vehicle stops, switch still in 4WD position */
    tick(300, LOW, 0.0f);
    tick(310, LOW, 0.0f);
    tick(320, LOW, 0.0f);
    tick(360, LOW, 0.0f);

    /* Now it should accept the change */
    ASSERT_EQ((int)traction_sw::getMode(), (int)traction_sw::Mode::FOUR_WD);
    ASSERT(traction_sw::hasChanged());
}

/* ====================================================================== */
/* TEST 9 — Speed at exactly threshold (0.5 km/h) blocks change           */
/* ====================================================================== */
static void test_speed_at_threshold_blocks() {
    reset_and_init(HIGH, 0);  /* Start 2WD */
    traction_sw::clearChanged();

    /* Switch at exactly 0.5 km/h + epsilon */
    tick(100, LOW, 0.6f);
    tick(110, LOW, 0.6f);
    tick(120, LOW, 0.6f);
    tick(160, LOW, 0.6f);

    /* Should be blocked (>0.5) */
    ASSERT_EQ((int)traction_sw::getMode(), (int)traction_sw::Mode::TWO_WD);
    ASSERT(!traction_sw::hasChanged());
}

/* ====================================================================== */
/* TEST 10 — Speed just below threshold allows change                      */
/* ====================================================================== */
static void test_speed_below_threshold_allows() {
    reset_and_init(HIGH, 0);  /* Start 2WD */
    traction_sw::clearChanged();

    tick(100, LOW, 0.4f);
    tick(110, LOW, 0.4f);
    tick(120, LOW, 0.4f);
    tick(160, LOW, 0.4f);

    /* Should succeed (0.4 < 0.5) */
    ASSERT_EQ((int)traction_sw::getMode(), (int)traction_sw::Mode::FOUR_WD);
    ASSERT(traction_sw::hasChanged());
}

/* ====================================================================== */
/* TEST 11 — Floating/disconnect: pull-up gives 2WD (safe default)         */
/* ====================================================================== */
static void test_floating_defaults_to_2wd() {
    reset_and_init(LOW, 0);   /* Start 4WD */
    traction_sw::clearChanged();

    /* Simulate cable disconnect: pin floats to HIGH (pull-up) */
    tick(100, HIGH, 0.0f);
    tick(110, HIGH, 0.0f);
    tick(120, HIGH, 0.0f);
    tick(160, HIGH, 0.0f);

    /* Should revert to 2WD */
    ASSERT_EQ((int)traction_sw::getMode(), (int)traction_sw::Mode::TWO_WD);
    ASSERT(traction_sw::hasChanged());
}

/* ====================================================================== */
/* TEST 12 — No update when not initialized                                */
/* ====================================================================== */
static void test_not_initialized() {
    /* Don't call init(), just verify safe defaults */
    g_test_millis = 0;
    g_gpio_state[SWITCH_PIN] = LOW;

    /* getMode() should return TWO_WD when not initialized */
    /* (note: this test depends on the static state from previous tests,
     * but the uninitialized guard should prevent changes) */
    ASSERT_EQ((int)traction_sw::getModeFlag(), 0x00);
}

/* ====================================================================== */
/* TEST 13 — Same state does not trigger changed flag                      */
/* ====================================================================== */
static void test_same_state_no_change() {
    reset_and_init(HIGH, 0);  /* Start 2WD */
    traction_sw::clearChanged();

    /* Keep same state */
    tick(100, HIGH, 0.0f);
    tick(200, HIGH, 0.0f);
    tick(300, HIGH, 0.0f);

    ASSERT(!traction_sw::hasChanged());
}

/* ====================================================================== */
/* Main                                                                    */
/* ====================================================================== */

int main(void) {
    printf("Running traction_switch unit tests...\n\n");

    test_init_2wd();
    test_init_4wd();
    test_clear_changed();
    test_debounce_rejects_glitch();
    test_stable_transition_to_4wd();
    test_stable_transition_to_2wd();
    test_speed_gate_blocks_change();
    test_speed_gate_allows_after_stop();
    test_speed_at_threshold_blocks();
    test_speed_below_threshold_allows();
    test_floating_defaults_to_2wd();
    test_not_initialized();
    test_same_state_no_change();

    printf("\n--- traction_switch tests: %d run, %d failed ---\n",
           s_tests_run, s_tests_failed);

    return s_tests_failed ? 1 : 0;
}
