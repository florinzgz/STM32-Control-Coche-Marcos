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
  *
  *          This file defines main() and is intended ONLY for host-side unit
  *          testing.  It is excluded from the STM32 firmware build via the
  *          HOST_TEST guard so that its main() does not collide with the
  *          firmware's main() in Core/Src/main.c at link time.
  ****************************************************************************
  */

#ifdef HOST_TEST

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

/* Re-implement GearLimitsStore_ValidateResponse() with the SAME logic as
 * gear_limits_store.c, referencing the canonical response range macros.   */
static bool test_validate_response(uint8_t d2, uint8_t d1, uint8_t r)
{
    if (d2 < GEAR_RESPONSE_D2_MIN_PCT || d2 > GEAR_RESPONSE_D2_MAX_PCT) return false;
    if (d1 < GEAR_RESPONSE_D1_MIN_PCT || d1 > GEAR_RESPONSE_D1_MAX_PCT) return false;
    if (r  < GEAR_RESPONSE_R_MIN_PCT  || r  > GEAR_RESPONSE_R_MAX_PCT)  return false;
    return true;
}

int main(void)
{
    /* Defaults must always validate (RESTORE DEFAULTS must never reject). */
    ASSERT_TRUE(test_validate(GEAR_LIMIT_D2_DEFAULT_PCT,
                              GEAR_LIMIT_D1_DEFAULT_PCT,
                              GEAR_LIMIT_R_DEFAULT_PCT));

    /* Defaults match the requested defaults (100/60/80). */
    ASSERT_TRUE(GEAR_LIMIT_D2_DEFAULT_PCT == 100U);
    ASSERT_TRUE(GEAR_LIMIT_D1_DEFAULT_PCT == 60U);
    ASSERT_TRUE(GEAR_LIMIT_R_DEFAULT_PCT  == 80U);

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
     * R max is 80 %, so 100 % (a legal D1/D2) must be rejected for R —
     * this is the "no reverse too fast" safety guard.                   */
    ASSERT_FALSE(test_validate(GEAR_LIMIT_D2_DEFAULT_PCT,
                               GEAR_LIMIT_D1_DEFAULT_PCT,
                               GEAR_LIMIT_R_MAX_PCT + 1U));
    ASSERT_FALSE(test_validate(100U, 100U, 100U));   /* R=100 illegal */
    ASSERT_FALSE(test_validate(101U, 60U, 80U));     /* D2>100 illegal */
    ASSERT_FALSE(test_validate(0U, 0U, 0U));         /* all zero illegal */

    /* Requested reverse power and a lower retune both validate. */
    ASSERT_TRUE(test_validate(100U, 60U, 80U));
    ASSERT_TRUE(test_validate(80U, 40U, 30U));

    /* ============================================================
     * Accel RESPONSE profile (v2) validation
     * ============================================================ */

    /* Response defaults must always validate. */
    ASSERT_TRUE(test_validate_response(GEAR_RESPONSE_D2_DEFAULT_PCT,
                                       GEAR_RESPONSE_D1_DEFAULT_PCT,
                                       GEAR_RESPONSE_R_DEFAULT_PCT));

    /* Response defaults match the task spec (D2 100 / D1 70 / R 40). */
    ASSERT_TRUE(GEAR_RESPONSE_D2_DEFAULT_PCT == 100U);
    ASSERT_TRUE(GEAR_RESPONSE_D1_DEFAULT_PCT == 70U);
    ASSERT_TRUE(GEAR_RESPONSE_R_DEFAULT_PCT  == 40U);

    /* Response ranges match the task spec. */
    ASSERT_TRUE(GEAR_RESPONSE_D2_MIN_PCT == 50U && GEAR_RESPONSE_D2_MAX_PCT == 100U);
    ASSERT_TRUE(GEAR_RESPONSE_D1_MIN_PCT == 30U && GEAR_RESPONSE_D1_MAX_PCT == 100U);
    ASSERT_TRUE(GEAR_RESPONSE_R_MIN_PCT  == 20U && GEAR_RESPONSE_R_MAX_PCT  == 80U);

    /* The "soften only" invariant: no response factor may exceed 100 %. */
    ASSERT_TRUE(GEAR_RESPONSE_D2_MAX_PCT <= 100U);
    ASSERT_TRUE(GEAR_RESPONSE_D1_MAX_PCT <= 100U);
    ASSERT_TRUE(GEAR_RESPONSE_R_MAX_PCT  <= 100U);

    /* Boundary acceptance — exact min/max of each gear are valid. */
    ASSERT_TRUE(test_validate_response(GEAR_RESPONSE_D2_MIN_PCT,
                                       GEAR_RESPONSE_D1_MIN_PCT,
                                       GEAR_RESPONSE_R_MIN_PCT));
    ASSERT_TRUE(test_validate_response(GEAR_RESPONSE_D2_MAX_PCT,
                                       GEAR_RESPONSE_D1_MAX_PCT,
                                       GEAR_RESPONSE_R_MAX_PCT));

    /* Below-min rejection for each gear. */
    ASSERT_FALSE(test_validate_response(GEAR_RESPONSE_D2_MIN_PCT - 1U,
                                        GEAR_RESPONSE_D1_DEFAULT_PCT,
                                        GEAR_RESPONSE_R_DEFAULT_PCT));
    ASSERT_FALSE(test_validate_response(GEAR_RESPONSE_D2_DEFAULT_PCT,
                                        GEAR_RESPONSE_D1_MIN_PCT - 1U,
                                        GEAR_RESPONSE_R_DEFAULT_PCT));
    ASSERT_FALSE(test_validate_response(GEAR_RESPONSE_D2_DEFAULT_PCT,
                                        GEAR_RESPONSE_D1_DEFAULT_PCT,
                                        GEAR_RESPONSE_R_MIN_PCT - 1U));

    /* Above-max rejection: R caps at 80 %, so 100 (legal for D1/D2) is
     * rejected for R — the "reverse must stay progressive" guard.       */
    ASSERT_FALSE(test_validate_response(GEAR_RESPONSE_D2_DEFAULT_PCT,
                                        GEAR_RESPONSE_D1_DEFAULT_PCT,
                                        GEAR_RESPONSE_R_MAX_PCT + 1U));
    ASSERT_FALSE(test_validate_response(100U, 100U, 100U));  /* R=100 illegal */
    ASSERT_FALSE(test_validate_response(101U, 70U, 40U));    /* D2>100 illegal */
    ASSERT_FALSE(test_validate_response(0U, 0U, 0U));        /* all zero illegal */

    /* A legacy (power-only) slot reads response bytes as 0, which must be
     * detected as out-of-range so the migration path applies defaults.   */
    ASSERT_FALSE(test_validate_response(0U, 0U, 0U));

    /* A realistic response re-tune (D2 90, D1 60, R 50) must validate. */
    ASSERT_TRUE(test_validate_response(90U, 60U, 50U));

    printf("test_gear_limits: %d run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}

#endif /* HOST_TEST */
