/**
  ****************************************************************************
  * @file    test_gear_limits.c
  * @brief   Host-compilable unit tests for gear power-limit validation.
  *
  *          Validates the range/boundary logic of GearLimitsStore_Validate()
  *          and the compile-time defaults without requiring flash or HAL.
  *
  *          Compile with host GCC (include production source):
  *            gcc -std=c11 -DHOST_TEST -Ianalysis_artifacts/stubs \
  *                -ICore/Inc -O2 -lm \
  *                Core/Src/test_gear_limits.c Core/Src/gear_limits_store.c \
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

/* Safety_GetState() is called by GearLimitsStore_Save(); provide a stub
 * that always returns STANDBY so the write-context guard passes in tests. */
#ifndef SAFETY_SYSTEM_H
typedef enum { SYS_STATE_STANDBY = 1 } SystemState_t;
typedef enum { SAFETY_ERROR_NONE = 0 } Safety_Error_t;
SystemState_t Safety_GetState(void) { return SYS_STATE_STANDBY; }
Safety_Error_t Safety_GetError(void) { return SAFETY_ERROR_NONE; }
#endif

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

int main(void)
{
    /* Defaults must always validate (RESTORE DEFAULTS must never reject). */
    ASSERT_TRUE(GearLimitsStore_Validate(GEAR_LIMIT_D2_DEFAULT_PCT,
                                         GEAR_LIMIT_D1_DEFAULT_PCT,
                                         GEAR_LIMIT_R_DEFAULT_PCT));

    /* Defaults match the required safe values (D2=100, D1=60, R=60). */
    ASSERT_TRUE(GEAR_LIMIT_D2_DEFAULT_PCT == 100U);
    ASSERT_TRUE(GEAR_LIMIT_D1_DEFAULT_PCT == 60U);
    ASSERT_TRUE(GEAR_LIMIT_R_DEFAULT_PCT  == 60U);

    /* Maximum reverse must not exceed 60 % (safety cap). */
    ASSERT_TRUE(GEAR_LIMIT_R_MAX_PCT <= 60U);

    /* Boundary acceptance — exact min/max of each gear are valid. */
    ASSERT_TRUE(GearLimitsStore_Validate(GEAR_LIMIT_D2_MIN_PCT,
                                         GEAR_LIMIT_D1_MIN_PCT,
                                         GEAR_LIMIT_R_MIN_PCT));
    ASSERT_TRUE(GearLimitsStore_Validate(GEAR_LIMIT_D2_MAX_PCT,
                                         GEAR_LIMIT_D1_MAX_PCT,
                                         GEAR_LIMIT_R_MAX_PCT));

    /* Below-min rejection for each gear (guard the off-by-one). */
    ASSERT_FALSE(GearLimitsStore_Validate(GEAR_LIMIT_D2_MIN_PCT - 1U,
                                          GEAR_LIMIT_D1_DEFAULT_PCT,
                                          GEAR_LIMIT_R_DEFAULT_PCT));
    ASSERT_FALSE(GearLimitsStore_Validate(GEAR_LIMIT_D2_DEFAULT_PCT,
                                          GEAR_LIMIT_D1_MIN_PCT - 1U,
                                          GEAR_LIMIT_R_DEFAULT_PCT));
    ASSERT_FALSE(GearLimitsStore_Validate(GEAR_LIMIT_D2_DEFAULT_PCT,
                                          GEAR_LIMIT_D1_DEFAULT_PCT,
                                          GEAR_LIMIT_R_MIN_PCT - 1U));

    /* Above-max rejection: R must reject values above 60 %.
     * D1/D2 allow up to 100 %, but R is capped at 60 % for safety. */
    ASSERT_FALSE(GearLimitsStore_Validate(GEAR_LIMIT_D2_DEFAULT_PCT,
                                          GEAR_LIMIT_D1_DEFAULT_PCT,
                                          GEAR_LIMIT_R_MAX_PCT + 1U));
    ASSERT_FALSE(GearLimitsStore_Validate(100U, 100U, 100U));  /* R=100 illegal */
    ASSERT_FALSE(GearLimitsStore_Validate(101U, 60U, 60U));    /* D2>100 illegal */
    ASSERT_FALSE(GearLimitsStore_Validate(0U, 0U, 0U));        /* all zero illegal */

    /* Valid non-default combinations. */
    ASSERT_TRUE(GearLimitsStore_Validate(100U, 60U, 60U));
    ASSERT_TRUE(GearLimitsStore_Validate(80U, 40U, 30U));

    /* ============================================================
     * Accel RESPONSE profile (v2) validation
     * ============================================================ */

    /* Response defaults must always validate. */
    ASSERT_TRUE(GearLimitsStore_ValidateResponse(GEAR_RESPONSE_D2_DEFAULT_PCT,
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
    ASSERT_TRUE(GearLimitsStore_ValidateResponse(GEAR_RESPONSE_D2_MIN_PCT,
                                                  GEAR_RESPONSE_D1_MIN_PCT,
                                                  GEAR_RESPONSE_R_MIN_PCT));
    ASSERT_TRUE(GearLimitsStore_ValidateResponse(GEAR_RESPONSE_D2_MAX_PCT,
                                                  GEAR_RESPONSE_D1_MAX_PCT,
                                                  GEAR_RESPONSE_R_MAX_PCT));

    /* Below-min rejection for each gear. */
    ASSERT_FALSE(GearLimitsStore_ValidateResponse(GEAR_RESPONSE_D2_MIN_PCT - 1U,
                                                   GEAR_RESPONSE_D1_DEFAULT_PCT,
                                                   GEAR_RESPONSE_R_DEFAULT_PCT));
    ASSERT_FALSE(GearLimitsStore_ValidateResponse(GEAR_RESPONSE_D2_DEFAULT_PCT,
                                                   GEAR_RESPONSE_D1_MIN_PCT - 1U,
                                                   GEAR_RESPONSE_R_DEFAULT_PCT));
    ASSERT_FALSE(GearLimitsStore_ValidateResponse(GEAR_RESPONSE_D2_DEFAULT_PCT,
                                                   GEAR_RESPONSE_D1_DEFAULT_PCT,
                                                   GEAR_RESPONSE_R_MIN_PCT - 1U));

    /* Above-max rejection: R caps at 80 %, so 100 (legal for D1/D2) is
     * rejected for R — the "reverse must stay progressive" guard.       */
    ASSERT_FALSE(GearLimitsStore_ValidateResponse(GEAR_RESPONSE_D2_DEFAULT_PCT,
                                                   GEAR_RESPONSE_D1_DEFAULT_PCT,
                                                   GEAR_RESPONSE_R_MAX_PCT + 1U));
    ASSERT_FALSE(GearLimitsStore_ValidateResponse(100U, 100U, 100U));  /* R=100 illegal */
    ASSERT_FALSE(GearLimitsStore_ValidateResponse(101U, 70U, 40U));    /* D2>100 illegal */
    ASSERT_FALSE(GearLimitsStore_ValidateResponse(0U, 0U, 0U));        /* all zero illegal */

    /* A legacy (power-only) slot reads response bytes as 0, which must be
     * detected as out-of-range so the migration path applies defaults.   */
    ASSERT_FALSE(GearLimitsStore_ValidateResponse(0U, 0U, 0U));

    /* A realistic response re-tune (D2 90, D1 60, R 50) must validate. */
    ASSERT_TRUE(GearLimitsStore_ValidateResponse(90U, 60U, 50U));

    printf("test_gear_limits: %d run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}

#endif /* HOST_TEST */
