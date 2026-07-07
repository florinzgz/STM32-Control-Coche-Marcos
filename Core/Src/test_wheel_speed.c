/**
  ****************************************************************************
  * @file    test_wheel_speed.c
  * @brief   Unit tests for wheel speed sensing: debounce constants,
  *          speed calculation, flood detection thresholds, and stale
  *          detection logic.
  *
  *          Compile with host GCC (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -lm \
  *                Core/Src/test_wheel_speed.c -o /tmp/test_wheel_speed
  ****************************************************************************
  */

#include "main.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

/* ---- Stub HAL handles required by main.h externs ---- */
#ifdef HOST_TEST
ADC_HandleTypeDef  hadc1;
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef  hi2c1;
TIM_HandleTypeDef  htim1, htim2, htim3, htim8;
IWDG_HandleTypeDef hiwdg;
#endif

/* ---- Replicate firmware constants for validation ---- */
#define WHEEL_CIRCUM_MM           1100.0f
#define WHEEL_CIRCUMF_M_TEST     (WHEEL_CIRCUM_MM / 1000.0f)

/* ---- Test framework ---- */

static int tests_run    = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(cond, msg) do {                                      \
    tests_run++;                                                         \
    if (!(cond)) {                                                       \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, (msg));          \
        tests_failed++;                                                  \
    }                                                                    \
} while (0)

#define ASSERT_NEAR(got, expected, tol, msg) do {                        \
    float _g = (got), _e = (expected), _t = (tol);                      \
    tests_run++;                                                         \
    if (fabsf(_g - _e) > _t) {                                          \
        printf("FAIL %s:%d  %s: got %.6f, expected %.6f (±%.6f)\n",     \
               __FILE__, __LINE__, (msg), (double)_g, (double)_e,       \
               (double)_t);                                              \
        tests_failed++;                                                  \
    }                                                                    \
} while (0)

#define ASSERT_EQ_U32(got, expected, msg) do {                           \
    uint32_t _g = (got), _e = (expected);                                \
    tests_run++;                                                         \
    if (_g != _e) {                                                      \
        printf("FAIL %s:%d  %s: got %u, expected %u\n",                 \
               __FILE__, __LINE__, (msg), (unsigned)_g, (unsigned)_e);   \
        tests_failed++;                                                  \
    }                                                                    \
} while (0)

/* =====================================================================
 *  1. Debounce constant validation
 * ===================================================================== */

static void test_debounce_constant(void)
{
    /* WHEEL_MIN_PULSE_INTERVAL_MS must be at least 1 (HAL_GetTick resolution) */
    ASSERT_TRUE(WHEEL_MIN_PULSE_INTERVAL_MS >= 1U,
                "Debounce interval must be >= 1 ms");

    /* Must be small enough to not attenuate valid pulses.
     * At max speed (25 km/h), pulse period ~26 ms.  Debounce must be << 26 ms. */
    float max_speed_ms = 25.0f / 3.6f;   /* m/s */
    float max_freq_hz  = max_speed_ms / WHEEL_CIRCUMF_M_TEST * (float)WHEEL_PULSES_REV;
    float min_period_ms = 1000.0f / max_freq_hz;

    ASSERT_TRUE((float)WHEEL_MIN_PULSE_INTERVAL_MS < min_period_ms / 2.0f,
                "Debounce interval must be < half the minimum valid pulse period");
}

/* =====================================================================
 *  2. Flood detection threshold validation
 * ===================================================================== */

static void test_flood_threshold(void)
{
    /* WHEEL_MAX_FREQ_HZ must exceed the physical maximum pulse rate */
    float max_speed_ms = 25.0f / 3.6f;
    float max_freq_hz  = max_speed_ms / WHEEL_CIRCUMF_M_TEST * (float)WHEEL_PULSES_REV;

    ASSERT_TRUE((float)WHEEL_MAX_FREQ_HZ > max_freq_hz,
                "Flood threshold must exceed physical max pulse rate");

    /* Should provide at least 2× margin above physical max */
    ASSERT_TRUE((float)WHEEL_MAX_FREQ_HZ >= max_freq_hz * 2.0f,
                "Flood threshold should have >= 2x margin above max pulse rate");

    /* Must be reasonable — not so high it provides no protection */
    ASSERT_TRUE(WHEEL_MAX_FREQ_HZ <= 1000U,
                "Flood threshold must be <= 1 kHz (sanity limit)");
}

/* =====================================================================
 *  3. Stale timeout validation
 * ===================================================================== */

static void test_stale_timeout(void)
{
    /* Stale timeout must be long enough to not trigger at low speeds.
     * At 1 km/h, pulse period ~660 ms.  Timeout should accommodate this. */
    float low_speed_ms = 1.0f / 3.6f;
    float low_freq_hz  = low_speed_ms / WHEEL_CIRCUMF_M_TEST * (float)WHEEL_PULSES_REV;
    float low_period_ms = 1000.0f / low_freq_hz;

    ASSERT_TRUE(WHEEL_STALE_TIMEOUT_MS >= 200U,
                "Stale timeout must be >= 200 ms");
    ASSERT_TRUE(WHEEL_STALE_TIMEOUT_MS <= 2000U,
                "Stale timeout must be <= 2 s (responsiveness)");

    /* Compute the speed boundary where stale detection activates.
     * Below this speed, the inter-pulse period exceeds the timeout
     * and the wheel is flagged stale (speed forced to 0).             */
    float stale_freq = 1000.0f / (float)WHEEL_STALE_TIMEOUT_MS;
    float stale_rps  = stale_freq / (float)WHEEL_PULSES_REV;
    float stale_speed_kmh = stale_rps * WHEEL_CIRCUMF_M_TEST * 3.6f;
    printf("  INFO: Stale triggers below %.1f km/h (timeout=%u ms, period=%.0f ms at 1 km/h)\n",
           (double)stale_speed_kmh, (unsigned)WHEEL_STALE_TIMEOUT_MS, (double)low_period_ms);

    /* The stale boundary must be below 2 km/h — at any meaningful
     * driving speed (>2 km/h) the sensor must NOT be falsely stale. */
    ASSERT_TRUE(stale_speed_kmh < 2.0f,
                "Stale boundary must be below 2 km/h (no false stale while moving)");

    /* The stale boundary must be above 0.5 km/h — a truly stopped wheel
     * must eventually be detected as stale (not stuck at last speed).  */
    ASSERT_TRUE(stale_speed_kmh > 0.5f,
                "Stale boundary must be above 0.5 km/h (detect stopped wheels)");
}

/* =====================================================================
 *  4. Speed calculation validation (pure math)
 * ===================================================================== */

static void test_speed_calculation(void)
{
    /* Simulate: 6 pulses in 100 ms → 1 revolution → 1.1 m in 0.1 s = 11 m/s = 39.6 km/h */
    uint32_t delta = 6;
    uint32_t dt_ms = 100;
    float revolutions = (float)delta / (float)WHEEL_PULSES_REV;
    float dist_m      = revolutions * WHEEL_CIRCUMF_M_TEST;
    float speed_ms    = dist_m * 1000.0f / (float)dt_ms;
    float speed_kmh   = speed_ms * 3.6f;

    ASSERT_NEAR(speed_kmh, 39.6f, 0.1f,
                "6 pulses in 100 ms = 39.6 km/h");

    /* 1 pulse in 50 ms → 0.167 rev → 0.183 m in 0.05 s = 3.67 m/s = 13.2 km/h */
    delta = 1; dt_ms = 50;
    revolutions = (float)delta / (float)WHEEL_PULSES_REV;
    dist_m = revolutions * WHEEL_CIRCUMF_M_TEST;
    speed_ms = dist_m * 1000.0f / (float)dt_ms;
    speed_kmh = speed_ms * 3.6f;
    ASSERT_NEAR(speed_kmh, 13.2f, 0.1f,
                "1 pulse in 50 ms = 13.2 km/h");

    /* 0 pulses → 0 km/h */
    delta = 0; dt_ms = 100;
    revolutions = (float)delta / (float)WHEEL_PULSES_REV;
    dist_m = revolutions * WHEEL_CIRCUMF_M_TEST;
    speed_ms = dist_m * 1000.0f / (float)dt_ms;
    speed_kmh = speed_ms * 3.6f;
    ASSERT_NEAR(speed_kmh, 0.0f, 0.001f,
                "0 pulses = 0 km/h");
}

/* =====================================================================
 *  5. RPM calculation validation
 * ===================================================================== */

static void test_rpm_calculation(void)
{
    /* 6 pulses in 100 ms → 1 rev in 0.1 s → 600 RPM */
    uint32_t delta = 6;
    uint32_t dt_ms = 100;
    float revolutions = (float)delta / (float)WHEEL_PULSES_REV;
    float rpm = revolutions * 60000.0f / (float)dt_ms;
    ASSERT_NEAR(rpm, 600.0f, 0.1f, "6 pulses in 100 ms = 600 RPM");

    /* 3 pulses in 200 ms → 0.5 rev in 0.2 s → 150 RPM */
    delta = 3; dt_ms = 200;
    revolutions = (float)delta / (float)WHEEL_PULSES_REV;
    rpm = revolutions * 60000.0f / (float)dt_ms;
    ASSERT_NEAR(rpm, 150.0f, 0.1f, "3 pulses in 200 ms = 150 RPM");
}

/* =====================================================================
 *  6. Constant consistency checks
 * ===================================================================== */

static void test_constant_consistency(void)
{
    /* NUM_WHEELS must be 4 */
    ASSERT_EQ_U32(NUM_WHEELS, 4, "NUM_WHEELS must be 4");

    /* WHEEL_PULSES_REV must be 6 (6 bolts) */
    ASSERT_EQ_U32(WHEEL_PULSES_REV, 6, "WHEEL_PULSES_REV must be 6");

    /* WHEEL_CIRCUMF_M must be 1.1 m */
    ASSERT_NEAR(WHEEL_CIRCUMF_M_TEST, 1.1f, 0.01f,
                "Wheel circumference must be 1.1 m");

    /* Debounce interval must be strictly less than flood window */
    ASSERT_TRUE(WHEEL_MIN_PULSE_INTERVAL_MS < 1000U / WHEEL_MAX_FREQ_HZ,
                "Debounce interval must not suppress flood-limit-rate pulses");
}

/* =====================================================================
 *  7. ISR load estimation
 * ===================================================================== */

static void test_isr_load_estimate(void)
{
    /* Max interrupt rate per wheel: WHEEL_MAX_FREQ_HZ */
    /* Total max rate: 4 × WHEEL_MAX_FREQ_HZ */
    uint32_t total_max_hz = 4 * WHEEL_MAX_FREQ_HZ;

    /* ISR body: ~20 cycles at 170 MHz.  At max rate: */
    float cycles_per_isr   = 20.0f;
    float mhz              = 170.0f;
    float cpu_pct = (float)total_max_hz * cycles_per_isr / (mhz * 1e6f) * 100.0f;

    printf("  INFO: Max interrupt rate = %u Hz total (4 × %u Hz per wheel)\n",
           (unsigned)total_max_hz, (unsigned)WHEEL_MAX_FREQ_HZ);
    printf("  INFO: Estimated CPU load = %.4f%% at 170 MHz\n", (double)cpu_pct);

    /* CPU overhead must be negligible (< 1%) */
    ASSERT_TRUE(cpu_pct < 1.0f,
                "Wheel ISR CPU load must be < 1% at max flood rate");
}

/* =====================================================================
 *  8. Flood detection math validation
 * ===================================================================== */

static void test_flood_math(void)
{
    /* In a 1-second window, at most WHEEL_MAX_FREQ_HZ pulses are accepted.
     * Verify that this converts to a sane maximum speed. */
    float max_accepted_freq = (float)WHEEL_MAX_FREQ_HZ;
    float max_rps = max_accepted_freq / (float)WHEEL_PULSES_REV;
    float max_speed_kmh = max_rps * WHEEL_CIRCUMF_M_TEST * 3.6f;

    printf("  INFO: Max accepted speed (flood ceiling) = %.1f km/h\n",
           (double)max_speed_kmh);

    /* Must be above physical plausibility (25 km/h) */
    ASSERT_TRUE(max_speed_kmh > 25.0f,
                "Flood ceiling speed must exceed plausibility limit");

    /* Must be below absurd levels (< 500 km/h) */
    ASSERT_TRUE(max_speed_kmh < 500.0f,
                "Flood ceiling speed must be within sanity bounds");
}

/* =====================================================================
 *  9. Counter overflow delta safety
 * ===================================================================== */

static void test_counter_overflow(void)
{
    /* uint32_t subtraction is modular: wrapping yields correct delta.
     * Simulate counter near UINT32_MAX that wraps around.              */
    uint32_t prev = UINT32_MAX - 5;
    uint32_t curr = 3;  /* Wrapped past 0 */
    uint32_t delta = curr - prev;  /* UINT32_MAX-5 → MAX → 0 → 3 = 9 steps */

    ASSERT_EQ_U32(delta, 9, "Overflow delta: (3) - (MAX-5) == 9");

    /* Edge case: no wrap */
    prev = 100; curr = 106;
    delta = curr - prev;
    ASSERT_EQ_U32(delta, 6, "Normal delta: 106 - 100 == 6");

    /* Edge case: same value (no pulses) */
    prev = 42; curr = 42;
    delta = curr - prev;
    ASSERT_EQ_U32(delta, 0, "Zero delta: 42 - 42 == 0");

    /* Large wrap distance */
    prev = UINT32_MAX;
    curr = 0;
    delta = curr - prev;
    ASSERT_EQ_U32(delta, 1, "Wrap by 1: 0 - MAX == 1");
}

/* =====================================================================
 *  10. Speed clamp ceiling validation
 * ===================================================================== */

static void test_speed_clamp(void)
{
    /* WHEEL_SPEED_CLAMP_KMH must be defined and positive */
    ASSERT_TRUE(WHEEL_SPEED_CLAMP_KMH > 0.0f,
                "Speed clamp must be positive");

    /* Must be above the physical max speed (25 km/h safety threshold) */
    ASSERT_TRUE(WHEEL_SPEED_CLAMP_KMH > 25.0f,
                "Speed clamp must exceed safety threshold (25 km/h)");

    /* Must be below absurd values — catches wild computation errors */
    ASSERT_TRUE(WHEEL_SPEED_CLAMP_KMH <= 100.0f,
                "Speed clamp must be <= 100 km/h (sanity)");

    /* RPM ceiling consistent with speed ceiling */
    float rpm_ceil = WHEEL_SPEED_CLAMP_KMH / 3.6f / WHEEL_CIRCUMF_M_TEST * 60.0f;
    printf("  INFO: Speed clamp = %.1f km/h, RPM ceiling = %.1f\n",
           (double)WHEEL_SPEED_CLAMP_KMH, (double)rpm_ceil);
    ASSERT_TRUE(rpm_ceil > 0.0f && rpm_ceil < 10000.0f,
                "RPM ceiling must be reasonable");
}

/* =====================================================================
 *  11. Period-based low-speed precision
 * ===================================================================== */

static void test_period_based_speed(void)
{
    /* Period measurement: single pulse with known inter-pulse time.
     * 1 pulse, period = 200 ms between two consecutive pulses
     * → 1/6 rev in 200 ms → 0.183 m in 0.2 s → 0.917 m/s → 3.3 km/h  */
    uint32_t period_ms = 200;
    float dist_per_pulse = WHEEL_CIRCUMF_M_TEST / (float)WHEEL_PULSES_REV;
    float speed_ms       = dist_per_pulse * 1000.0f / (float)period_ms;
    float speed_kmh      = speed_ms * 3.6f;

    ASSERT_NEAR(speed_kmh, 3.3f, 0.1f,
                "Period 200 ms → 3.3 km/h (period-based)");

    /* Very slow: period = 500 ms (near stale boundary)
     * → 0.183 m in 0.5 s → 0.367 m/s → 1.32 km/h                     */
    period_ms = 500;
    speed_ms  = dist_per_pulse * 1000.0f / (float)period_ms;
    speed_kmh = speed_ms * 3.6f;

    ASSERT_NEAR(speed_kmh, 1.32f, 0.05f,
                "Period 500 ms → 1.32 km/h (near stale boundary)");

    /* Count-based comparison at same rate: 1 pulse in 10 ms window
     * gives 1/6 rev in 10 ms → 0.183 m in 0.01 s → 18.3 m/s → 66 km/h
     * (wildly overestimated!).  Period-based gives correct 3.3 km/h.
     * This demonstrates the quantisation error at low speed.            */
    float count_based_kmh = dist_per_pulse * 1000.0f / 10.0f * 3.6f;
    ASSERT_TRUE(count_based_kmh > 60.0f,
                "Count-based at 1 pulse/10ms greatly overestimates speed");
    ASSERT_TRUE(speed_kmh < 5.0f,
                "Period-based gives correct low speed");
}

/* =====================================================================
 *  12. Decay estimate (delta == 0)
 * ===================================================================== */

static void test_decay_estimate(void)
{
    /* When no new pulse arrives, upper-bound speed decays with time.
     * dist_per_pulse / since_last gives decreasing upper bound.         */
    float dist_per_pulse = WHEEL_CIRCUMF_M_TEST / (float)WHEEL_PULSES_REV;

    /* 50 ms since last pulse → upper bound = 0.183 / 0.05 * 3.6 = 13.2 km/h */
    uint32_t since_last = 50;
    float ub = dist_per_pulse * 1000.0f / (float)since_last * 3.6f;
    ASSERT_NEAR(ub, 13.2f, 0.1f, "Upper bound at 50 ms since last pulse");

    /* 200 ms since last → upper bound = 3.3 km/h */
    since_last = 200;
    ub = dist_per_pulse * 1000.0f / (float)since_last * 3.6f;
    ASSERT_NEAR(ub, 3.3f, 0.1f, "Upper bound at 200 ms since last pulse");

    /* 499 ms (just before stale) → upper bound = 1.32 km/h */
    since_last = 499;
    ub = dist_per_pulse * 1000.0f / (float)since_last * 3.6f;
    ASSERT_TRUE(ub > 1.0f && ub < 2.0f,
                "Upper bound at 499 ms (near stale) is near 1.3 km/h");
}

/* =====================================================================
 *  11. First pulse from standstill must NOT fabricate speed
 *
 *  Regression for the "push the car a few cm slowly → 15 km/h" phantom.
 *  Replicates the fixed delta==1 branch of Wheel_ComputeSpeed(): a single
 *  isolated pulse (prev_pt == 0 after boot, or a prev pulse older than the
 *  stale window because the wheel was stopped) must fall back to the
 *  time-since-pulse upper bound capped by the previous reading — never to
 *  the ~10 ms compute-cycle dt, which produced the phantom.
 * ===================================================================== */

/* Pure model of the corrected delta==1 speed branch. */
static float model_delta1_speed(uint32_t now, uint32_t dt,
                                uint32_t last_pt, uint32_t prev_pt,
                                float prev_speed_kmh)
{
    const float dist_per_pulse = WHEEL_CIRCUMF_M_TEST / (float)WHEEL_PULSES_REV;
    (void)dt;  /* Corrected code must NOT use dt on the isolated-pulse path. */
    uint32_t period_ms = last_pt - prev_pt;
    float speed_kmh;
    if (prev_pt != 0U && period_ms > 0U && period_ms < WHEEL_STALE_TIMEOUT_MS) {
        speed_kmh = dist_per_pulse * 1000.0f / (float)period_ms * 3.6f;
    } else {
        uint32_t since_last = now - last_pt;
        if (since_last > 0U) {
            speed_kmh = dist_per_pulse * 1000.0f / (float)since_last * 3.6f;
            if (speed_kmh > prev_speed_kmh) speed_kmh = prev_speed_kmh;
        } else {
            speed_kmh = prev_speed_kmh;
        }
    }
    return speed_kmh;
}

static void test_first_pulse_from_standstill(void)
{
    /* Boot case: prev_pt == 0, first ever pulse just arrived.
     * dt (compute interval) = 10 ms would have given ~66 km/h phantom.
     * Corrected: capped by previous reading (0) → 0 km/h.               */
    float s = model_delta1_speed(/*now*/5005, /*dt*/10,
                                 /*last_pt*/5000, /*prev_pt*/0,
                                 /*prev_speed*/0.0f);
    ASSERT_NEAR(s, 0.0f, 0.001f,
                "First pulse from boot (prev_pt=0) must be 0 km/h, not a phantom");

    /* Push-after-stop case: prev pulse was 3 s ago (wheel had stopped),
     * new pulse just arrived.  period_ms = 3000 >= stale window → isolated
     * pulse path → capped by previous reading (0) → 0 km/h.             */
    s = model_delta1_speed(/*now*/8007, /*dt*/7,
                           /*last_pt*/8000, /*prev_pt*/5000,
                           /*prev_speed*/0.0f);
    ASSERT_NEAR(s, 0.0f, 0.001f,
                "First pulse after a stop (stale prev) must be 0 km/h, not a phantom");

    /* Sanity: the OLD (buggy) fallback would have produced a large value. */
    float dist_per_pulse = WHEEL_CIRCUMF_M_TEST / (float)WHEEL_PULSES_REV;
    float buggy = dist_per_pulse * 1000.0f / 10.0f * 3.6f;  /* dt = 10 ms */
    ASSERT_TRUE(buggy > 60.0f,
                "Old dt-based fallback would have fabricated >60 km/h (clamped to 40)");

    /* Genuine low-speed motion: a real 200 ms interval between two
     * consecutive pulses still yields the correct 3.3 km/h.            */
    s = model_delta1_speed(/*now*/8205, /*dt*/5,
                           /*last_pt*/8200, /*prev_pt*/8000,
                           /*prev_speed*/3.3f);
    ASSERT_NEAR(s, 3.3f, 0.1f,
                "Valid 200 ms inter-pulse interval → 3.3 km/h (period-based)");
}

/* =====================================================================
 *  main
 * ===================================================================== */

int main(void)
{
    printf("=== Wheel Speed Sensing Tests ===\n\n");

    test_debounce_constant();
    test_flood_threshold();
    test_stale_timeout();
    test_speed_calculation();
    test_rpm_calculation();
    test_constant_consistency();
    test_isr_load_estimate();
    test_flood_math();
    test_counter_overflow();
    test_speed_clamp();
    test_period_based_speed();
    test_decay_estimate();
    test_first_pulse_from_standstill();

    printf("\n--- wheel_speed tests: %d run, %d failed ---\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
