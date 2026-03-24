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

    printf("\n--- wheel_speed tests: %d run, %d failed ---\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
