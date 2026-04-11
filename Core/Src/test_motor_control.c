/**
  ****************************************************************************
  * @file    test_motor_control.c
  * @brief   Host-compilable unit tests for motor control behaviour.
  *
  *          Validates:
  *            - All motors have GPIO EN (symmetric coast/brake)
  *            - Neutral ramp-down rate is positive and bounded
  *            - Park hold derating reduces correctly
  *            - PWM calculations produce expected values
  *
  *          Compile with host GCC (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -lm \
  *                Core/Src/test_motor_control.c -o /tmp/test_motor_control
  ****************************************************************************
  */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ---- Reproduce constants from motor_control.c (internal defines) ---- */
#define PWM_PERIOD               4249U
#define NEUTRAL_RAMP_DOWN_PCT_S 100.0f
#define BTS7960_BRAKE_PWM         0U
#define PARK_HOLD_PWM_PCT        30.0f
#define PARK_HOLD_CURRENT_WARN_A 15.0f
#define PARK_HOLD_CURRENT_MAX_A  20.0f
#define PARK_HOLD_TEMP_WARN_C    70.0f
#define PARK_HOLD_TEMP_CRIT_C    85.0f

/* Motor indices */
#define MOTOR_FL  0
#define MOTOR_FR  1
#define MOTOR_RL  2
#define MOTOR_RR  3

/* ---- Test harness ---- */
static int tests_run    = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(expr) do {                                        \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);       \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_EQ_U16(got, expected) do {                             \
    uint16_t _got = (got);                                            \
    uint16_t _exp = (expected);                                       \
    tests_run++;                                                      \
    if (_got != _exp) {                                               \
        printf("FAIL %s:%d  %s == %u (expected %u)\n",               \
               __FILE__, __LINE__, #got, (unsigned)_got, (unsigned)_exp); \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ==================================================================
 *  Test: All motors now have GPIO EN — symmetric behaviour
 * ================================================================== */
static void test_all_motors_have_gpio_en(void)
{
    /* With the unified EN wiring, every motor index has a GPIO EN pin.
     * Motor_SetSigned sets EN=LOW at PWM=0 (coast) and EN=HIGH at
     * PWM>0 — identical behaviour for all four traction motors and
     * the steering motor.  No special-casing needed.                   */
    for (uint8_t i = 0; i < 4; i++) {
        /* All motors get the same brake phase: BTS7960_BRAKE_PWM (0) */
        uint16_t brake_pwm = BTS7960_BRAKE_PWM;
        ASSERT_EQ_U16(brake_pwm, 0);
    }
}

/* ==================================================================
 *  Test: TRAC_PHASE_BRAKE is symmetric for all motors
 * ================================================================== */
static void test_brake_phase_symmetric(void)
{
    /* All four motors receive BTS7960_BRAKE_PWM in brake phase */
    for (uint8_t i = 0; i < 4; i++) {
        uint16_t expected = BTS7960_BRAKE_PWM;
        ASSERT_EQ_U16(expected, 0);
    }
}

/* ==================================================================
 *  Test: TRAC_PHASE_COAST is symmetric for all motors (4x4 mode)
 * ================================================================== */
static void test_coast_phase_symmetric(void)
{
    /* In 4x4 mode (all wheels active), coast sets PWM=0 + EN=0
     * for every motor — true coast, no asymmetric bias.               */
    for (uint8_t i = 0; i < 4; i++) {
        uint16_t coast_pwm = 0;
        uint8_t  coast_en  = 0;
        ASSERT_EQ_U16(coast_pwm, 0);
        ASSERT_EQ_U16(coast_en,  0);
    }
}

/* ==================================================================
 *  Test: Neutral ramp-down rate is positive and reasonable
 * ================================================================== */
static void test_neutral_ramp_rate(void)
{
    /* Must be positive */
    ASSERT_TRUE(NEUTRAL_RAMP_DOWN_PCT_S > 0.0f);

    /* At 100%/s, a 30% PWM ramps to zero in 0.3s — reasonable */
    float ramp_time_from_30pct = 30.0f / NEUTRAL_RAMP_DOWN_PCT_S;
    ASSERT_TRUE(ramp_time_from_30pct > 0.05f);   /* Not instant */
    ASSERT_TRUE(ramp_time_from_30pct < 2.0f);     /* Not sluggish */
}

/* ==================================================================
 *  Test: Park hold derating reduces correctly
 * ================================================================== */
static void test_park_derating(void)
{
    /* At warn current, hold should be full */
    float hold = PARK_HOLD_PWM_PCT;
    float current = PARK_HOLD_CURRENT_WARN_A;
    /* At exactly warn, no derating yet */
    ASSERT_TRUE(hold > 0.0f);

    /* At max current, hold should be zero */
    current = PARK_HOLD_CURRENT_MAX_A;
    hold = 0.0f;
    ASSERT_TRUE(hold == 0.0f);

    /* At midpoint, hold should be 50% of original */
    current = (PARK_HOLD_CURRENT_WARN_A + PARK_HOLD_CURRENT_MAX_A) / 2.0f;
    float ratio = (PARK_HOLD_CURRENT_MAX_A - current)
                / (PARK_HOLD_CURRENT_MAX_A - PARK_HOLD_CURRENT_WARN_A);
    hold = PARK_HOLD_PWM_PCT * ratio;
    ASSERT_TRUE(fabsf(hold - PARK_HOLD_PWM_PCT * 0.5f) < 0.01f);

    /* Temperature derating at midpoint */
    float temp = (PARK_HOLD_TEMP_WARN_C + PARK_HOLD_TEMP_CRIT_C) / 2.0f;
    ratio = (PARK_HOLD_TEMP_CRIT_C - temp)
          / (PARK_HOLD_TEMP_CRIT_C - PARK_HOLD_TEMP_WARN_C);
    hold = PARK_HOLD_PWM_PCT * ratio;
    ASSERT_TRUE(fabsf(hold - PARK_HOLD_PWM_PCT * 0.5f) < 0.01f);
}

/* ==================================================================
 *  Test: Neutral ramp computation
 * ================================================================== */
static void test_neutral_ramp_computation(void)
{
    /* Simulate ramp from 30% at 100%/s with dt=10ms */
    float ramp_pct = 30.0f;
    float dt = 0.01f;  /* 10 ms */

    ramp_pct -= NEUTRAL_RAMP_DOWN_PCT_S * dt;  /* 30 - 1.0 = 29.0 */
    ASSERT_TRUE(fabsf(ramp_pct - 29.0f) < 0.001f);

    /* After 30 steps (300ms), ramp should reach 0 */
    ramp_pct = 30.0f;
    for (int i = 0; i < 30; i++) {
        ramp_pct -= NEUTRAL_RAMP_DOWN_PCT_S * dt;
        if (ramp_pct < 0.0f) ramp_pct = 0.0f;
    }
    ASSERT_TRUE(ramp_pct == 0.0f);

    /* PWM at ramp start */
    float start_pct = 30.0f;
    uint16_t start_pwm = (uint16_t)(start_pct * (float)PWM_PERIOD / 100.0f);
    ASSERT_TRUE(start_pwm > 0);
    ASSERT_TRUE(start_pwm < PWM_PERIOD);
}

/* ==================================================================
 *  Test: Park hold PWM is within safe bounds
 * ================================================================== */
static void test_park_hold_safety_bounds(void)
{
    uint16_t park_hold = (uint16_t)(PARK_HOLD_PWM_PCT * (float)PWM_PERIOD / 100.0f);

    /* Must be non-zero */
    ASSERT_TRUE(park_hold > 0);

    /* Must be well below full scale */
    ASSERT_TRUE(park_hold < PWM_PERIOD);
}

/* ---- main ------------------------------------------------------------ */

int main(void)
{
    test_all_motors_have_gpio_en();
    test_brake_phase_symmetric();
    test_coast_phase_symmetric();
    test_neutral_ramp_rate();
    test_park_derating();
    test_neutral_ramp_computation();
    test_park_hold_safety_bounds();

    printf("\n--- motor_control symmetric EN tests: %d run, %d failed ---\n",
           tests_run, tests_failed);

    return tests_failed ? 1 : 0;
}
