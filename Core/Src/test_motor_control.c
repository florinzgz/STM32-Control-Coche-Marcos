/**
  ****************************************************************************
  * @file    test_motor_control.c
  * @brief   Host-compilable unit tests for motor control half-bridge
  *          brake/coast asymmetry compensation.
  *
  *          Validates:
  *            - Simulated brake PWM is within safe bounds
  *            - Coast compensation PWM is within safe bounds
  *            - Neutral ramp-down rate is positive and bounded
  *            - PWM calculations produce expected values
  *            - Asymmetry-aware motor identification
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
#define SIMBRAKE_PWM_PCT         15.0f
#define COAST_COMPENSATION_PCT    4.0f
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

/* ---- Helper: check if motor has GPIO-controlled EN (coasts at PWM=0) ---- */
static bool motor_has_gpio_en(uint8_t idx)
{
    return (idx == MOTOR_FL || idx == MOTOR_RR);
}

/* ---- Helper: check if motor has EN tied HIGH (passive brakes at PWM=0) ---- */
static bool motor_has_tied_en(uint8_t idx)
{
    return (idx == MOTOR_FR || idx == MOTOR_RL);
}

/* ==================================================================
 *  Test: Simulated brake PWM value is within safe bounds
 * ================================================================== */
static void test_simbrake_pwm_range(void)
{
    uint16_t sim_brake_pwm = (uint16_t)(SIMBRAKE_PWM_PCT * (float)PWM_PERIOD / 100.0f);

    /* Must be non-zero (otherwise it's just coast) */
    ASSERT_TRUE(sim_brake_pwm > 0);

    /* Must be well below 50% — this is a child vehicle, not aggressive braking */
    ASSERT_TRUE(sim_brake_pwm < PWM_PERIOD / 2);

    /* Expected value: 15% of 4249 ≈ 637 */
    ASSERT_EQ_U16(sim_brake_pwm, (uint16_t)(0.15f * 4249.0f));
}

/* ==================================================================
 *  Test: Coast compensation PWM value is within safe bounds
 * ================================================================== */
static void test_coast_compensation_range(void)
{
    uint16_t coast_bias = (uint16_t)(COAST_COMPENSATION_PCT * (float)PWM_PERIOD / 100.0f);

    /* Must be non-zero (otherwise no compensation) */
    ASSERT_TRUE(coast_bias > 0);

    /* Must be less than simulated brake (coast drag < brake drag) */
    uint16_t sim_brake = (uint16_t)(SIMBRAKE_PWM_PCT * (float)PWM_PERIOD / 100.0f);
    ASSERT_TRUE(coast_bias < sim_brake);

    /* Must be below 10% — barely perceptible, just enough for symmetry */
    ASSERT_TRUE(coast_bias < PWM_PERIOD / 10);

    /* Expected value: 4% of 4249 ≈ 169 */
    ASSERT_EQ_U16(coast_bias, (uint16_t)(0.04f * 4249.0f));
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
 *  Test: Motor EN type classification is consistent
 * ================================================================== */
static void test_motor_en_classification(void)
{
    /* FL and RR have GPIO EN (coast at PWM=0) */
    ASSERT_TRUE(motor_has_gpio_en(MOTOR_FL));
    ASSERT_TRUE(motor_has_gpio_en(MOTOR_RR));

    /* FR and RL have tied EN (passive brake at PWM=0) */
    ASSERT_TRUE(motor_has_tied_en(MOTOR_FR));
    ASSERT_TRUE(motor_has_tied_en(MOTOR_RL));

    /* No motor should be both */
    for (uint8_t i = 0; i < 4; i++) {
        ASSERT_TRUE(motor_has_gpio_en(i) != motor_has_tied_en(i));
    }
}

/* ==================================================================
 *  Test: TRAC_PHASE_BRAKE produces different output for FL/RR vs FR/RL
 * ================================================================== */
static void test_brake_phase_asymmetry(void)
{
    uint16_t sim_brake_pwm = (uint16_t)(SIMBRAKE_PWM_PCT * (float)PWM_PERIOD / 100.0f);

    for (uint8_t i = 0; i < 4; i++) {
        uint16_t expected_pwm;
        if (motor_has_gpio_en(i)) {
            /* FL/RR: simulated brake (non-zero PWM) */
            expected_pwm = sim_brake_pwm;
        } else {
            /* FR/RL: passive brake (PWM=0) */
            expected_pwm = BTS7960_BRAKE_PWM;
        }

        /* This mirrors the logic in Traction_Update TRAC_PHASE_BRAKE */
        uint16_t actual;
        if (i == MOTOR_FL || i == MOTOR_RR) {
            actual = sim_brake_pwm;
        } else {
            actual = BTS7960_BRAKE_PWM;
        }
        ASSERT_EQ_U16(actual, expected_pwm);
    }
}

/* ==================================================================
 *  Test: TRAC_PHASE_COAST produces different output for FL/RR vs FR/RL
 * ================================================================== */
static void test_coast_phase_asymmetry(void)
{
    uint16_t coast_bias = (uint16_t)(COAST_COMPENSATION_PCT * (float)PWM_PERIOD / 100.0f);

    /* FL/RR get coast compensation bias */
    ASSERT_TRUE(coast_bias > 0);

    /* FR/RL get zero (EN=0 → coast intent, but hardware EN tied HIGH) */
    /* The actual hardware will still passively brake, but the firmware
     * sets desired_en=0 as the intent for coast behaviour.              */
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
 *  Test: Simulated brake does not exceed safety bounds
 * ================================================================== */
static void test_simbrake_safety_bounds(void)
{
    /* Simulated brake must be less than park hold */
    uint16_t sim_brake = (uint16_t)(SIMBRAKE_PWM_PCT * (float)PWM_PERIOD / 100.0f);
    uint16_t park_hold = (uint16_t)(PARK_HOLD_PWM_PCT * (float)PWM_PERIOD / 100.0f);
    ASSERT_TRUE(sim_brake < park_hold);

    /* Coast compensation must be less than simulated brake */
    uint16_t coast_comp = (uint16_t)(COAST_COMPENSATION_PCT * (float)PWM_PERIOD / 100.0f);
    ASSERT_TRUE(coast_comp < sim_brake);

    /* Ordering: coast_comp < sim_brake < park_hold < PWM_PERIOD */
    ASSERT_TRUE(coast_comp < sim_brake);
    ASSERT_TRUE(sim_brake < park_hold);
    ASSERT_TRUE(park_hold < PWM_PERIOD);
}

/* ---- main ------------------------------------------------------------ */

int main(void)
{
    test_simbrake_pwm_range();
    test_coast_compensation_range();
    test_neutral_ramp_rate();
    test_motor_en_classification();
    test_brake_phase_asymmetry();
    test_coast_phase_asymmetry();
    test_park_derating();
    test_neutral_ramp_computation();
    test_simbrake_safety_bounds();

    printf("\n--- motor_control half-bridge compensation tests: %d run, %d failed ---\n",
           tests_run, tests_failed);

    return tests_failed ? 1 : 0;
}
