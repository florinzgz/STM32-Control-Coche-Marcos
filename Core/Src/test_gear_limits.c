/**
  ****************************************************************************
  * @file    test_gear_limits.c
  * @brief   Host-compilable unit tests for gear power-limit validation.
  *
  *          Validates the range/boundary logic of GearLimitsStore_Validate()
  *          and the compile-time defaults without requiring flash or HAL.
  *
  *          Compile with host GCC:
  *            gcc -std=c11 -DHOST_TEST -Ianalysis_artifacts/stubs \
  *                -ICore/Inc -O2 -lm Core/Src/test_gear_limits.c \
  *                -o test_gear_limits
  ****************************************************************************
  */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Pull in the single source of truth for ranges + defaults.  The header
 * is HAL-free (pure macros + _Static_assert), so it compiles standalone. */
#include "gear_limits_store.h"

/* ---- Test harness ---- */
static int tests_run    = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(expr) do {                                        \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_FALSE(expr) do {                                       \
    tests_run++;                                                      \
    if ((expr)) {                                                     \
        printf("FAIL %s:%d  !(%s)\n", __FILE__, __LINE__, #expr);     \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* Re-implement GearLimitsStore_Validate() with the SAME logic as
 * gear_limits_store.c, referencing the canonical range macros from the
 * header so any future range change is reflected here automatically.   */
static bool test_validate(uint8_t d2, uint8_t d1, uint8_t r)
{
    if (d2 < GEAR_LIMIT_D2_MIN_PCT || d2 > GEAR_LIMIT_D2_MAX_PCT) return false;
    if (d1 < GEAR_LIMIT_D1_MIN_PCT || d1 > GEAR_LIMIT_D1_MAX_PCT) return false;
    if (r  < GEAR_LIMIT_R_MIN_PCT  || r  > GEAR_LIMIT_R_MAX_PCT)  return false;
    return true;
}

int main(void)
{
    /* Defaults must always validate (RESTORE DEFAULTS must never reject). */
    ASSERT_TRUE(test_validate(GEAR_LIMIT_D2_DEFAULT_PCT,
                              GEAR_LIMIT_D1_DEFAULT_PCT,
                              GEAR_LIMIT_R_DEFAULT_PCT));

    /* Defaults match the historic compile-time behaviour (100/60/60). */
    ASSERT_TRUE(GEAR_LIMIT_D2_DEFAULT_PCT == 100U);
    ASSERT_TRUE(GEAR_LIMIT_D1_DEFAULT_PCT == 60U);
    ASSERT_TRUE(GEAR_LIMIT_R_DEFAULT_PCT  == 60U);

    /* Boundary acceptance — exact min/max of each gear are valid. */
    ASSERT_TRUE(test_validate(GEAR_LIMIT_D2_MIN_PCT,
                              GEAR_LIMIT_D1_MIN_PCT,
                              GEAR_LIMIT_R_MIN_PCT));
    ASSERT_TRUE(test_validate(GEAR_LIMIT_D2_MAX_PCT,
                              GEAR_LIMIT_D1_MAX_PCT,
                              GEAR_LIMIT_R_MAX_PCT));

    /* Below-min rejection for each gear (guard the off-by-one). */
    ASSERT_FALSE(test_validate(GEAR_LIMIT_D2_MIN_PCT - 1U,
                               GEAR_LIMIT_D1_DEFAULT_PCT,
                               GEAR_LIMIT_R_DEFAULT_PCT));
    ASSERT_FALSE(test_validate(GEAR_LIMIT_D2_DEFAULT_PCT,
                               GEAR_LIMIT_D1_MIN_PCT - 1U,
                               GEAR_LIMIT_R_DEFAULT_PCT));
    ASSERT_FALSE(test_validate(GEAR_LIMIT_D2_DEFAULT_PCT,
                               GEAR_LIMIT_D1_DEFAULT_PCT,
                               GEAR_LIMIT_R_MIN_PCT - 1U));

    /* Above-max rejection: R must reject values that are valid for D1/D2.
     * R max is 60 %, so 100 % (a legal D1/D2) must be rejected for R —
     * this is the "no reverse too fast" safety guard.                   */
    ASSERT_FALSE(test_validate(GEAR_LIMIT_D2_DEFAULT_PCT,
                               GEAR_LIMIT_D1_DEFAULT_PCT,
                               GEAR_LIMIT_R_MAX_PCT + 1U));
    ASSERT_FALSE(test_validate(100U, 100U, 100U));   /* R=100 illegal */
    ASSERT_FALSE(test_validate(101U, 60U, 60U));     /* D2>100 illegal */
    ASSERT_FALSE(test_validate(0U, 0U, 0U));         /* all zero illegal */

    /* A realistic re-tune (D2 80, D1 40, R 30) must validate. */
    ASSERT_TRUE(test_validate(80U, 40U, 30U));

    printf("test_gear_limits: %d run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}
