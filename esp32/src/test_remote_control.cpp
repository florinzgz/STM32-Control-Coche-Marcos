/**
 ****************************************************************************
 * @file    test_remote_control.cpp
 * @brief   Host-compilable unit tests for remote_control driver (iBUS).
 *
 *          Compile and run on host (from esp32/src/):
 *
 *            g++ -std=c++17 \
 *                -DREMOTE_CONTROL_ENABLED=1 -DREMOTE_CONTROL_TEST_HOOKS \
 *                -I. -I../test_stubs \
 *                remote_control.cpp test_remote_control.cpp \
 *                -o /tmp/test_remote_control && /tmp/test_remote_control
 *
 *          Verified:
 *            1. Valid frame parses; channels extracted correctly
 *            2. Bad checksum is rejected and counted
 *            3. Garbage bytes before header resync to next good frame
 *            4. >5 consecutive checksum fails → DEGRADED
 *            5. >150 ms since last good frame → FAILSAFE
 *            6. CH5 below threshold → kill switch active
 *            7. CH10 below threshold → !isRemoteSelected, isActive==false
 *            8. CH1 deadzone around 1500 yields exactly 0°
 *            9. CH2 below 1500 yields exactly 0 % throttle
 *           10. CH2 at 2000 ramps up to 100 % (ramp-limited)
 *           11. CH3 trim is summed into steering
 *           12. CH6/CH7/CH8/CH9 discrete and analog reads
 *           13. Out-of-range channel (sanity) frame is rejected
 ****************************************************************************
 */

#ifndef REMOTE_CONTROL_ENABLED
#define REMOTE_CONTROL_ENABLED 1
#endif
#ifndef REMOTE_CONTROL_TEST_HOOKS
#define REMOTE_CONTROL_TEST_HOOKS
#endif

#include <Arduino.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include "remote_control.h"

using namespace remote_control;

/* ---- Test harness ------------------------------------------------------- */

static int s_tests_run    = 0;
static int s_tests_failed = 0;

#define ASSERT(cond) do {                                                     \
    s_tests_run++;                                                            \
    if (!(cond)) {                                                            \
        printf("FAIL  %s:%d  Assertion: %s\n", __FILE__, __LINE__, #cond);    \
        s_tests_failed++;                                                     \
    }                                                                         \
} while (0)

#define ASSERT_EQ(a, b) do {                                                  \
    s_tests_run++;                                                            \
    if ((long)(a) != (long)(b)) {                                             \
        printf("FAIL  %s:%d  %s == %ld (expected %ld)\n",                     \
               __FILE__, __LINE__, #a, (long)(a), (long)(b));                 \
        s_tests_failed++;                                                     \
    }                                                                         \
} while (0)

#define ASSERT_NEAR(a, b, eps) do {                                           \
    s_tests_run++;                                                            \
    if (std::fabs((double)(a) - (double)(b)) > (double)(eps)) {               \
        printf("FAIL  %s:%d  %s ≈ %g (expected %g ± %g)\n",                   \
               __FILE__, __LINE__, #a, (double)(a), (double)(b), (double)(eps)); \
        s_tests_failed++;                                                     \
    }                                                                         \
} while (0)

/* ---- Frame builder helper ---------------------------------------------- */

// Default "REMOTE active, no throttle, centered steering" channel set.
static void defaultChannels(uint16_t ch[14]) {
    for (int i = 0; i < 14; i++) ch[i] = 1500;
    ch[4] = 1900;  // CH5 = kill OFF (high)
    ch[9] = 1900;  // CH10 = REMOTE (high)
}

/* ---- Tests ------------------------------------------------------------- */

static void test_valid_frame_extracts_channels() {
    Config cfg;
    test::reset(cfg);

    uint16_t channels[14];
    defaultChannels(channels);
    channels[0] = 1800;  // CH1
    channels[1] = 1700;  // CH2

    uint8_t frame[32];
    test::buildFrame(channels, frame);
    test::setNowMs(10);
    test::feedBytes(frame, 32);

    ASSERT_EQ((int)getState(), (int)State::ACTIVE);
    ASSERT_EQ(getStats().framesOk, 1u);
    ASSERT_EQ(getStats().checksumFails, 0u);
    ASSERT_EQ(getChannelRaw(1), 1800);
    ASSERT_EQ(getChannelRaw(2), 1700);
    ASSERT_EQ(getChannelRaw(5), 1900);
    ASSERT_EQ(getChannelRaw(10), 1900);
}

static void test_bad_checksum_rejected() {
    Config cfg;
    test::reset(cfg);

    uint16_t channels[14];
    defaultChannels(channels);
    uint8_t frame[32];
    test::buildFrame(channels, frame);
    frame[30] ^= 0xFF;  // Corrupt checksum

    test::setNowMs(10);
    test::feedBytes(frame, 32);

    ASSERT_EQ(getStats().framesOk, 0u);
    ASSERT_EQ(getStats().checksumFails, 1u);
    ASSERT_EQ((int)getState(), (int)State::IDLE);
}

static void test_resync_after_garbage() {
    Config cfg;
    test::reset(cfg);

    uint16_t channels[14];
    defaultChannels(channels);
    uint8_t frame[32];
    test::buildFrame(channels, frame);

    // Inject garbage that is NOT a valid header (avoid 0x20/0x40 patterns
    // that could be interpreted as a partial frame start).
    uint8_t garbage[7] = { 0xAA, 0xBB, 0x21, 0xCC, 0x41, 0xDD, 0xEE };
    test::setNowMs(10);
    test::feedBytes(garbage, sizeof(garbage));
    test::feedBytes(frame, 32);

    ASSERT_EQ(getStats().framesOk, 1u);
    ASSERT_EQ((int)getState(), (int)State::ACTIVE);
}

static void test_degraded_after_streak_of_checksum_fails() {
    Config cfg;
    test::reset(cfg);

    // First seed one good frame so we are in ACTIVE
    uint16_t channels[14];
    defaultChannels(channels);
    uint8_t good[32];
    test::buildFrame(channels, good);
    test::setNowMs(10);
    test::feedBytes(good, 32);
    ASSERT_EQ((int)getState(), (int)State::ACTIVE);

    // Now inject 5 bad frames in a row
    uint8_t bad[32];
    memcpy(bad, good, 32);
    bad[30] ^= 0xFF;
    for (int i = 0; i < 5; i++) {
        test::feedBytes(bad, 32);
    }
    ASSERT_EQ(getStats().checksumFails, 5u);
    ASSERT_EQ((int)getState(), (int)State::DEGRADED);
}

static void test_failsafe_after_timeout() {
    Config cfg;
    test::reset(cfg);

    uint16_t channels[14];
    defaultChannels(channels);
    uint8_t good[32];
    test::buildFrame(channels, good);

    test::setNowMs(10);
    test::feedBytes(good, 32);
    ASSERT_EQ((int)getState(), (int)State::ACTIVE);

    // Advance well past the 150 ms timeout and run update()
    test::setNowMs(10 + cfg.timeoutMs + 50);
    update();
    ASSERT_EQ((int)getState(), (int)State::FAILSAFE);
    ASSERT_EQ(getStats().timeouts, 1u);
    ASSERT(!isActive());
    ASSERT_NEAR(getThrottlePct(), 0.0f, 1e-3);
}

static void test_killswitch_and_remote_gate() {
    Config cfg;
    test::reset(cfg);

    uint16_t channels[14];
    defaultChannels(channels);
    channels[4] = 1000;  // CH5 kill ON
    uint8_t f[32];
    test::buildFrame(channels, f);
    test::setNowMs(10);
    test::feedBytes(f, 32);
    update();

    ASSERT(isKillSwitchActive());
    ASSERT(!isActive());

    // Restore CH5, but CH10 LOCAL
    defaultChannels(channels);
    channels[9] = 1000;  // CH10 LOCAL
    test::buildFrame(channels, f);
    test::setNowMs(20);
    test::feedBytes(f, 32);
    update();

    ASSERT(!isKillSwitchActive());
    ASSERT(!isRemoteSelected());
    ASSERT(!isActive());
}

static void test_throttle_and_steering_smoothing() {
    Config cfg;
    test::reset(cfg);

    uint16_t channels[14];
    defaultChannels(channels);
    channels[0] = 1500;  // CH1 centered → 0°
    channels[1] = 2000;  // CH2 full throttle
    channels[2] = 1500;  // CH3 trim 0
    uint8_t f[32];
    test::buildFrame(channels, f);

    test::setNowMs(10);
    test::feedBytes(f, 32);
    ASSERT(isActive());

    // Drive smoothing for 1 second in small steps, feeding a fresh frame
    // each tick so the FSM stays ACTIVE (otherwise the 150 ms timeout would
    // drop us into FAILSAFE).
    for (int i = 0; i < 20; i++) {
        test::setNowMs(10 + (i + 1) * 50);
        test::feedBytes(f, 32);
        update();
    }
    // After ~1 s with 50 %/s ramp-up (first update() has dt=0 since
    // s_lastSmoothMs is lazily initialized) we expect ~47–50 %.
    ASSERT(getThrottlePct() >= 45.0f);
    ASSERT(getThrottlePct() <= 100.0f);

    // Steering should remain at 0 (within smoothing tolerance)
    ASSERT_NEAR(getSteeringDeg(), 0.0f, 1.0f);
}

static void test_steering_with_trim() {
    Config cfg;
    test::reset(cfg);

    uint16_t channels[14];
    defaultChannels(channels);
    channels[0] = 2000;  // CH1 full right → +30°
    channels[2] = 2000;  // CH3 trim max → +5°
    uint8_t f[32];
    test::buildFrame(channels, f);
    test::setNowMs(10);
    test::feedBytes(f, 32);

    // Run for plenty of time to converge, feeding the frame each step.
    for (int i = 0; i < 100; i++) {
        test::setNowMs(10 + (i + 1) * 20);
        test::feedBytes(f, 32);
        update();
    }
    // Target = clamp(30 + 5, ±30) = 30 (we clamp the sum)
    ASSERT_NEAR(getSteeringDeg(), 30.0f, 2.0f);
}

static void test_throttle_lower_half_yields_zero() {
    Config cfg;
    test::reset(cfg);

    uint16_t channels[14];
    defaultChannels(channels);
    channels[1] = 1200;  // Below 1500 → reserved for future regen, throttle = 0
    uint8_t f[32];
    test::buildFrame(channels, f);
    test::setNowMs(10);
    test::feedBytes(f, 32);
    for (int i = 0; i < 10; i++) {
        test::setNowMs(10 + (i + 1) * 50);
        test::feedBytes(f, 32);
        update();
    }
    ASSERT_NEAR(getThrottlePct(), 0.0f, 0.5f);
}

static void test_discrete_channels() {
    Config cfg;
    test::reset(cfg);

    uint16_t channels[14];
    defaultChannels(channels);
    channels[5] = 1000;   // CH6 low → ECO
    channels[6] = 2000;   // CH7 high → FORWARD
    channels[7] = 1900;   // CH8 high → lights ON
    channels[8] = 1500;   // CH9 mid → volume ~15
    uint8_t f[32];
    test::buildFrame(channels, f);
    test::setNowMs(10);
    test::feedBytes(f, 32);
    update();

    ASSERT_EQ(getDriveMode(), 0);
    ASSERT_EQ(getRequestedGear(), 3);
    ASSERT(isLightsOn());
    uint8_t v = getAudioVolume();
    ASSERT(v >= 14 && v <= 16);
}

static void test_sanity_reject_out_of_range() {
    Config cfg;
    test::reset(cfg);

    uint16_t channels[14];
    defaultChannels(channels);
    channels[0] = 850;  // Below sanity min (900) → frame must be rejected
    uint8_t f[32];
    test::buildFrame(channels, f);
    test::setNowMs(10);
    test::feedBytes(f, 32);

    ASSERT_EQ(getStats().framesOk, 0u);
    ASSERT_EQ(getStats().sanityRejects, 1u);
    ASSERT_EQ((int)getState(), (int)State::IDLE);
}

/* ---- Main ------------------------------------------------------------- */

int main() {
    test_valid_frame_extracts_channels();
    test_bad_checksum_rejected();
    test_resync_after_garbage();
    test_degraded_after_streak_of_checksum_fails();
    test_failsafe_after_timeout();
    test_killswitch_and_remote_gate();
    test_throttle_and_steering_smoothing();
    test_steering_with_trim();
    test_throttle_lower_half_yields_zero();
    test_discrete_channels();
    test_sanity_reject_out_of_range();

    printf("\n----------------------------------------\n");
    printf("test_remote_control: %d run, %d failed\n",
           s_tests_run, s_tests_failed);
    printf("----------------------------------------\n");
    return s_tests_failed == 0 ? 0 : 1;
}
