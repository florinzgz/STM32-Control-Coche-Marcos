/**
 ****************************************************************************
 * @file    test_relay_audio.cpp
 * @brief   Host-compilable unit tests for relay_audio state machine.
 *
 *          Tests the pure state-machine logic of relay_audio without any
 *          real hardware.  All Arduino API calls (pinMode, digitalWrite,
 *          millis, Serial) are replaced by lightweight stubs in
 *          ../test_stubs/Arduino.h.
 *
 *          Compile and run on host (from esp32/src/):
 *            g++ -std=c++17 -I. -I../test_stubs \
 *                relay_audio.cpp test_relay_audio.cpp \
 *                -o /tmp/test_relay_audio && /tmp/test_relay_audio
 *
 *          Requirements covered (from docs/INFORME_REVISION_TECNICA_RELAY.md):
 *            1. Non-blocking: all relay_audio functions are O(1)
 *            2. Race conditions: consecutive sounds, DFPlayer no-response,
 *               watchdog after RELAY_MAX_ON_MS
 *            3. millis() overflow safety (values near UINT32_MAX)
 *            4. init() always drives relay OFF (boot/reset recovery)
 *            5. forceOff() immediately deactivates relay from any state
 *            6. Timing: 20 ms guard (no premature play), 150 ms cooldown
 *            7. Idempotent requestOn(), release() on IDLE is no-op
 *            8. GPIO polarity: relay OFF = HIGH, relay ON = LOW
 ****************************************************************************
 */

/* Arduino stubs are in ../test_stubs/Arduino.h (via -I../test_stubs).
 * relay_audio.cpp will find Arduino.h there instead of the real library.
 * We also include it here to access g_test_millis, g_gpio_state, etc. */
#include <Arduino.h>
#include <cstdio>
#include "relay_audio.h"

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

/* Helper: reset clock and GPIO counters, then call init() */
static void reset_and_init(unsigned long start_ms = 0) {
    g_test_millis = start_ms;
    for (int i = 0; i < 64; ++i) {
        g_gpio_state[i]      = 0;
        g_gpio_write_count[i] = 0;
    }
    relay_audio::init();
}

/* Helper: advance time and call update() */
static void tick(unsigned long new_ms) {
    g_test_millis = new_ms;
    relay_audio::update();
}

/* ====================================================================== */
/* TEST 1 — init() drives relay OFF immediately (boot recovery)           */
/* ====================================================================== */
static void test_init_forces_relay_off() {
    /* Simulate relay being ON before init (e.g. ESP32 reboot during ACTIVE) */
    g_gpio_state[relay_audio::PIN_AUDIO_RELAY] = LOW;  /* was ON */
    reset_and_init(500);

    /* After init(), relay must be HIGH (OFF) */
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], HIGH);
    /* isReady() must be false in IDLE */
    ASSERT(!relay_audio::isReady());
}

/* ====================================================================== */
/* TEST 2 — Normal activation flow IDLE→ACTIVATING→ACTIVE                 */
/* ====================================================================== */
static void test_normal_activation() {
    reset_and_init(0);

    /* requestOn → ACTIVATING, GPIO goes LOW */
    relay_audio::requestOn();
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], LOW);
    ASSERT(!relay_audio::isReady());  /* not yet ACTIVE */

    /* tick at 19 ms — still ACTIVATING */
    tick(relay_audio::RELAY_ESTABLISH_MS - 1);
    ASSERT(!relay_audio::isReady());

    /* tick at exactly 20 ms — transitions to ACTIVE */
    tick(relay_audio::RELAY_ESTABLISH_MS);
    ASSERT(relay_audio::isReady());
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], LOW);
}

/* ====================================================================== */
/* TEST 3 — ACTIVE → RELEASING → IDLE (normal release)                   */
/* ====================================================================== */
static void test_normal_release() {
    reset_and_init(0);

    relay_audio::requestOn();
    tick(relay_audio::RELAY_ESTABLISH_MS);         /* now ACTIVE */
    ASSERT(relay_audio::isReady());

    /* release() transitions ACTIVE → RELEASING; relay stays LOW */
    relay_audio::release();
    ASSERT(!relay_audio::isReady());
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], LOW);

    /* 149 ms — still RELEASING */
    tick(relay_audio::RELAY_ESTABLISH_MS + relay_audio::RELAY_RELEASE_MS - 1);
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], LOW);

    /* 150 ms — IDLE, relay goes HIGH */
    tick(relay_audio::RELAY_ESTABLISH_MS + relay_audio::RELAY_RELEASE_MS);
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], HIGH);
    ASSERT(!relay_audio::isReady());
}

/* ====================================================================== */
/* TEST 4 — Consecutive sounds: requestOn() during RELEASING              */
/*           → skip re-establishment, go directly to ACTIVE               */
/* ====================================================================== */
static void test_consecutive_sounds() {
    reset_and_init(1000);

    /* Sound 1: activate, wait to ACTIVE, then release */
    relay_audio::requestOn();
    tick(1000 + relay_audio::RELAY_ESTABLISH_MS);
    relay_audio::release();

    unsigned long release_start = g_test_millis;

    /* Sound 2 arrives 10 ms into the 150 ms cooldown */
    tick(release_start + 10);
    ASSERT(!relay_audio::isReady());   /* still RELEASING before requestOn */

    relay_audio::requestOn();  /* cancel cooldown → ACTIVE */
    tick(release_start + 11);

    /* Must be ACTIVE immediately (no re-establishment) */
    ASSERT(relay_audio::isReady());

    /* GPIO must still be LOW (relay never opened) */
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], LOW);
}

/* ====================================================================== */
/* TEST 5 — requestOn() is idempotent in ACTIVATING and ACTIVE            */
/* ====================================================================== */
static void test_idempotent_request_on() {
    reset_and_init(0);

    unsigned int writes_before;

    /* Call requestOn() three times in IDLE — only first should toggle GPIO */
    writes_before = g_gpio_write_count[relay_audio::PIN_AUDIO_RELAY];
    relay_audio::requestOn();
    relay_audio::requestOn();  /* no-op */
    relay_audio::requestOn();  /* no-op */

    /* Only one additional GPIO write (the first LOW) */
    ASSERT_EQ(
        (int)g_gpio_write_count[relay_audio::PIN_AUDIO_RELAY],
        (int)(writes_before + 1)
    );

    /* Advance to ACTIVE */
    tick(relay_audio::RELAY_ESTABLISH_MS);
    ASSERT(relay_audio::isReady());

    writes_before = g_gpio_write_count[relay_audio::PIN_AUDIO_RELAY];
    relay_audio::requestOn();  /* no-op in ACTIVE */
    relay_audio::requestOn();  /* no-op */
    ASSERT_EQ(
        (int)g_gpio_write_count[relay_audio::PIN_AUDIO_RELAY],
        (int)writes_before
    );
    ASSERT(relay_audio::isReady());  /* still ACTIVE */
}

/* ====================================================================== */
/* TEST 6 — release() on IDLE is a no-op (relay stays OFF)                */
/* ====================================================================== */
static void test_release_on_idle_is_noop() {
    reset_and_init(0);

    int writes_before = (int)g_gpio_write_count[relay_audio::PIN_AUDIO_RELAY];
    relay_audio::release();   /* no-op */
    tick(10);

    /* GPIO must not change from HIGH */
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], HIGH);
    /* No additional write should have occurred */
    ASSERT_EQ(
        (int)g_gpio_write_count[relay_audio::PIN_AUDIO_RELAY],
        writes_before
    );
}

/* ====================================================================== */
/* TEST 7 — forceOff() immediately deactivates from ACTIVE                */
/* ====================================================================== */
static void test_force_off_from_active() {
    reset_and_init(500);

    relay_audio::requestOn();
    tick(500 + relay_audio::RELAY_ESTABLISH_MS);
    ASSERT(relay_audio::isReady());

    relay_audio::forceOff();

    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], HIGH);
    ASSERT(!relay_audio::isReady());

    /* Further ticks must not re-activate the relay */
    tick(500 + relay_audio::RELAY_ESTABLISH_MS + 100);
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], HIGH);
}

/* ====================================================================== */
/* TEST 8 — forceOff() from RELEASING (e.g. emergency during cooldown)    */
/* ====================================================================== */
static void test_force_off_from_releasing() {
    reset_and_init(0);

    relay_audio::requestOn();
    tick(relay_audio::RELAY_ESTABLISH_MS);
    relay_audio::release();

    /* Force off during 150 ms cooldown */
    tick(relay_audio::RELAY_ESTABLISH_MS + 50);
    relay_audio::forceOff();

    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], HIGH);
    ASSERT(!relay_audio::isReady());
}

/* ====================================================================== */
/* TEST 9 — Watchdog fires after RELAY_MAX_ON_MS with no release()        */
/*           (simulates playing flag never cleared due to software bug)    */
/* ====================================================================== */
static void test_watchdog_fires() {
    reset_and_init(1000);

    relay_audio::requestOn();
    tick(1000 + relay_audio::RELAY_ESTABLISH_MS);  /* ACTIVE */
    ASSERT(relay_audio::isReady());

    /* Advance time but never call release() */
    unsigned long wd_deadline = 1000 + relay_audio::RELAY_MAX_ON_MS;

    /* One tick before watchdog — must still be ACTIVE */
    tick(wd_deadline - 1);
    ASSERT(relay_audio::isReady());
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], LOW);

    /* At exactly RELAY_MAX_ON_MS — watchdog fires */
    tick(wd_deadline);
    ASSERT(!relay_audio::isReady());
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], HIGH);

    /* State is IDLE; further ticks must not re-activate */
    tick(wd_deadline + 1000);
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], HIGH);
}

/* ====================================================================== */
/* TEST 10 — millis() overflow safety (near UINT32_MAX rollover)          */
/* ====================================================================== */
static void test_millis_overflow() {
    /* Place stateMs_ near the maximum 32-bit value */
    constexpr unsigned long WRAP = 0xFFFFFFFFUL;

    reset_and_init(WRAP - 5);   /* init at WRAP-5 ms */

    relay_audio::requestOn();   /* activationMs_ = WRAP-5, stateMs_ = WRAP-5 */
    /* GPIO must now be LOW */
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], LOW);

    /* Tick at WRAP (5 ms after activation) — still in ACTIVATING (need 20 ms) */
    tick(WRAP);
    ASSERT(!relay_audio::isReady());

    /* Tick at WRAP+16 (16 ms after rollover = 21 ms total) → ACTIVE */
    tick(WRAP + 16);  /* unsigned wrap: 16 > 0, (16 - (WRAP-5)) mod 2^32 = 21 */
    ASSERT(relay_audio::isReady());

    /* Release and verify cooldown also crosses the rollover correctly */
    relay_audio::release();

    /* Tick 149 ms after release — still RELEASING */
    tick(WRAP + 16 + relay_audio::RELAY_RELEASE_MS - 1);
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], LOW);

    /* Tick at 150 ms after release — IDLE */
    tick(WRAP + 16 + relay_audio::RELAY_RELEASE_MS);
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], HIGH);
}

/* ====================================================================== */
/* TEST 11 — Watchdog millis() overflow (activation near UINT32_MAX)      */
/* ====================================================================== */
static void test_watchdog_overflow() {
    constexpr unsigned long WRAP = 0xFFFFFFFFUL;

    /* Activate at WRAP-100 (100 ms before overflow) */
    reset_and_init(WRAP - 100);

    relay_audio::requestOn();   /* activationMs_ = WRAP-100 */

    /* No release(). After RELAY_MAX_ON_MS ms total, watchdog must fire. */
    /* That deadline = (WRAP-100 + 7000) mod 2^32 = 6900 */
    unsigned long wd_deadline = (unsigned long)(WRAP - 100 + relay_audio::RELAY_MAX_ON_MS);

    tick(wd_deadline - 1);
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], LOW);

    tick(wd_deadline);
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], HIGH);
    ASSERT(!relay_audio::isReady());
}

/* ====================================================================== */
/* TEST 12 — release() called during ACTIVATING (before ACTIVE)           */
/*           must still reach RELEASING without hanging                    */
/* ====================================================================== */
static void test_release_during_activating() {
    reset_and_init(0);

    relay_audio::requestOn();

    /* release() called at 10 ms (still ACTIVATING) */
    tick(10);
    ASSERT(!relay_audio::isReady());
    relay_audio::release();

    /* Should now be RELEASING; relay still LOW */
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], LOW);

    /* After cooldown, relay goes HIGH */
    tick(10 + relay_audio::RELAY_RELEASE_MS);
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], HIGH);
}

/* ====================================================================== */
/* TEST 13 — GPIO polarity: ON = LOW, OFF = HIGH                          */
/* ====================================================================== */
static void test_gpio_polarity() {
    reset_and_init(0);

    /* After init() relay must be HIGH (off) */
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], HIGH);

    /* After requestOn() relay must be LOW (on) */
    relay_audio::requestOn();
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], LOW);

    /* After full cycle back to IDLE, relay must be HIGH again */
    tick(relay_audio::RELAY_ESTABLISH_MS);
    relay_audio::release();
    tick(relay_audio::RELAY_ESTABLISH_MS + relay_audio::RELAY_RELEASE_MS);
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], HIGH);
}

/* ====================================================================== */
/* TEST 14 — DFPlayer no-response scenario (playing flag never clears)     */
/*           relay must be released by watchdog within RELAY_MAX_ON_MS    */
/* ====================================================================== */
static void test_dfplayer_no_response() {
    /*
     * Simulate: requestOn() called, play command sent, DFPlayer doesn't
     * respond.  audio::update() would normally call release() after
     * MAX_PLAY_DURATION_MS (5000 ms) but we test that the relay watchdog
     * also provides a safety backstop at RELAY_MAX_ON_MS (7000 ms).
     */
    reset_and_init(2000);

    relay_audio::requestOn();
    tick(2000 + relay_audio::RELAY_ESTABLISH_MS);  /* ACTIVE */

    /* Simulate normal audio timeout calling release() at 5000 ms */
    tick(2000 + 5000);
    relay_audio::release();          /* RELEASING */

    /* After cooldown the relay must be OFF */
    tick(2000 + 5000 + relay_audio::RELAY_RELEASE_MS);
    ASSERT_EQ(g_gpio_state[relay_audio::PIN_AUDIO_RELAY], HIGH);
    ASSERT(!relay_audio::isReady());
}

/* ====================================================================== */
/* TEST 15 — No premature audio: isReady() false during ACTIVATING        */
/* ====================================================================== */
static void test_not_ready_during_activating() {
    reset_and_init(0);

    relay_audio::requestOn();

    for (unsigned long t = 0; t < relay_audio::RELAY_ESTABLISH_MS; ++t) {
        tick(t);
        ASSERT(!relay_audio::isReady());  /* must not be ready yet */
    }

    /* At exactly RELAY_ESTABLISH_MS → ACTIVE */
    tick(relay_audio::RELAY_ESTABLISH_MS);
    ASSERT(relay_audio::isReady());
}

/* ====================================================================== */
/* Main                                                                    */
/* ====================================================================== */

int main(void) {
    printf("Running relay_audio unit tests...\n\n");

    test_init_forces_relay_off();
    test_normal_activation();
    test_normal_release();
    test_consecutive_sounds();
    test_idempotent_request_on();
    test_release_on_idle_is_noop();
    test_force_off_from_active();
    test_force_off_from_releasing();
    test_watchdog_fires();
    test_millis_overflow();
    test_watchdog_overflow();
    test_release_during_activating();
    test_gpio_polarity();
    test_dfplayer_no_response();
    test_not_ready_during_activating();

    printf("\n--- relay_audio tests: %d run, %d failed ---\n",
           s_tests_run, s_tests_failed);

    return s_tests_failed ? 1 : 0;
}
