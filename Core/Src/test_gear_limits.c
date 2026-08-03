/**
  ****************************************************************************
  * @file    test_gear_limits.c
  * @brief   Host-compilable unit tests for gear power-limit validation.
  *
  *          Validates the range/boundary logic of GearLimitsStore_Validate()
  *          and GearLimitsStore_ValidateResponse() by calling the REAL
  *          production functions from gear_limits_store.c.
  *
  *          Compile with host GCC:
  *            gcc -std=c11 -DHOST_TEST -Ianalysis_artifacts/stubs \
  *                -ICore/Inc -O2 -lm Core/Src/test_gear_limits.c \
  *                Core/Src/gear_limits_store.c -o test_gear_limits
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

/* Pull in the single source of truth for ranges + defaults. */
#include "gear_limits_store.h"
#include "safety_system.h"

/* ---- Minimal safety-system stub -----------------------------------
 * GearLimitsStore_Save() gates on Safety_GetState() == SYS_STATE_STANDBY.
 * The test only calls GearLimitsStore_Validate() and
 * GearLimitsStore_ValidateResponse(), which do NOT consult system state.
 * This stub satisfies the linker without pulling in safety_system.c.    */
SystemState_t Safety_GetState(void) { return SYS_STATE_STANDBY; }

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

    /* Defaults match the historic compile-time behaviour (100/60/60). */
    ASSERT_TRUE(GEAR_LIMIT_D2_DEFAULT_PCT == 100U);
    ASSERT_TRUE(GEAR_LIMIT_D1_DEFAULT_PCT == 60U);
    ASSERT_TRUE(GEAR_LIMIT_R_DEFAULT_PCT  == 60U);

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

    /* Above-max rejection: R must reject values that are valid for D1/D2.
     * R max is 60 %, so 100 % (a legal D1/D2) must be rejected for R —
     * this is the "no reverse too fast" safety guard.                   */
    ASSERT_FALSE(GearLimitsStore_Validate(GEAR_LIMIT_D2_DEFAULT_PCT,
                                          GEAR_LIMIT_D1_DEFAULT_PCT,
                                          GEAR_LIMIT_R_MAX_PCT + 1U));
    ASSERT_FALSE(GearLimitsStore_Validate(100U, 100U, 100U));  /* R=100 illegal */
    ASSERT_FALSE(GearLimitsStore_Validate(101U, 60U, 60U));    /* D2>100 illegal */
    ASSERT_FALSE(GearLimitsStore_Validate(0U, 0U, 0U));        /* all zero illegal */

    /* A realistic re-tune (D2 80, D1 40, R 30) must validate. */
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
    ASSERT_FALSE(GearLimitsStore_ValidateResponse(100U, 100U, 100U)); /* R=100 illegal */
    ASSERT_FALSE(GearLimitsStore_ValidateResponse(101U, 70U, 40U));   /* D2>100 illegal */
    ASSERT_FALSE(GearLimitsStore_ValidateResponse(0U, 0U, 0U));       /* all zero illegal */

    /* A legacy (power-only) slot reads response bytes as 0, which must be
     * detected as out-of-range so the migration path applies defaults.   */
    ASSERT_FALSE(GearLimitsStore_ValidateResponse(0U, 0U, 0U));

    /* A realistic response re-tune (D2 90, D1 60, R 50) must validate. */
    ASSERT_TRUE(GearLimitsStore_ValidateResponse(90U, 60U, 50U));

    printf("test_gear_limits: %d run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}

#endif /* HOST_TEST */
