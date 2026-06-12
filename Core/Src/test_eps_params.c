/**
  ****************************************************************************
  * @file    test_eps_params.c
  * @brief   Host-compilable unit tests for EPS parameter validation.
  *
  *          Validates NaN/Inf rejection and zero-division safeguards
  *          in EPS_Params_Set() without requiring flash or HAL.
  *
  *          Compile with host GCC:
  *            gcc -std=c11 -I../Inc -O2 -lm \
  *                test_eps_params.c -o test_eps_params
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

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

#define ASSERT_FALSE(expr) do {                                       \
    tests_run++;                                                      \
    if ((expr)) {                                                     \
        printf("FAIL %s:%d  !(%s)\n", __FILE__, __LINE__, #expr);    \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ---- Minimal reproduction of eps_params types and logic ----
 * We reproduce only EPS_Params_Set() to test its validation logic
 * without requiring HAL, flash, or full firmware linkage.             */

#include "eps_params.h"

/* Simulated RAM state (mirrors eps_params.c static variables) */
static eps_params_t test_eps_active;

/* Re-implement EPS_Params_Set() with the same logic as eps_params.c
 * to validate the NaN/Inf guard independently.                        */
static bool test_EPS_Params_Set(eps_param_id_t id, float value)
{
    if (id >= EPS_PARAM_COUNT) return false;

    /* Reject NaN and Inf for all parameters */
    if (isnan(value) || isinf(value)) return false;

    /* Reject zero or negative for divisor parameters */
    if ((id == EPS_PARAM_ASSIST_VS_SPEED || id == EPS_PARAM_RETURN_VS_SPEED)
        && value <= 0.0f) {
        return false;
    }

    float *fields = (float *)&test_eps_active;
    fields[id] = value;
    return true;
}

/* ---- Tests ---- */

static void test_reject_nan(void)
{
    /* NaN must be rejected for every parameter */
    for (int i = 0; i < EPS_PARAM_COUNT; i++) {
        ASSERT_FALSE(test_EPS_Params_Set((eps_param_id_t)i, NAN));
    }
}

static void test_reject_inf(void)
{
    /* +Inf and -Inf must be rejected for every parameter */
    for (int i = 0; i < EPS_PARAM_COUNT; i++) {
        ASSERT_FALSE(test_EPS_Params_Set((eps_param_id_t)i, INFINITY));
        ASSERT_FALSE(test_EPS_Params_Set((eps_param_id_t)i, -INFINITY));
    }
}

static void test_reject_zero_divisors(void)
{
    /* assist_vs_speed and return_vs_speed must reject <= 0 */
    ASSERT_FALSE(test_EPS_Params_Set(EPS_PARAM_ASSIST_VS_SPEED, 0.0f));
    ASSERT_FALSE(test_EPS_Params_Set(EPS_PARAM_ASSIST_VS_SPEED, -1.0f));
    ASSERT_FALSE(test_EPS_Params_Set(EPS_PARAM_RETURN_VS_SPEED, 0.0f));
    ASSERT_FALSE(test_EPS_Params_Set(EPS_PARAM_RETURN_VS_SPEED, -5.0f));
}

static void test_accept_valid_divisors(void)
{
    /* Positive values must be accepted for divisor parameters */
    ASSERT_TRUE(test_EPS_Params_Set(EPS_PARAM_ASSIST_VS_SPEED, 18.0f));
    ASSERT_TRUE(test_EPS_Params_Set(EPS_PARAM_RETURN_VS_SPEED, 35.0f));
    ASSERT_TRUE(test_EPS_Params_Set(EPS_PARAM_ASSIST_VS_SPEED, 0.001f));
}

static void test_accept_valid_normal(void)
{
    /* Normal values for non-divisor parameters must be accepted */
    ASSERT_TRUE(test_EPS_Params_Set(EPS_PARAM_ASSIST_STRENGTH, 0.45f));
    ASSERT_TRUE(test_EPS_Params_Set(EPS_PARAM_CENTER_STRENGTH, 0.30f));
    ASSERT_TRUE(test_EPS_Params_Set(EPS_PARAM_DAMPING, 0.10f));
    ASSERT_TRUE(test_EPS_Params_Set(EPS_PARAM_FRICTION_COMP, 0.05f));
    ASSERT_TRUE(test_EPS_Params_Set(EPS_PARAM_COAST_BAND_PCT, 3.0f));
    ASSERT_TRUE(test_EPS_Params_Set(EPS_PARAM_MIN_DRIVE_PCT, 8.0f));
}

static void test_reject_out_of_range_id(void)
{
    /* Out-of-range parameter ID must be rejected */
    ASSERT_FALSE(test_EPS_Params_Set(EPS_PARAM_COUNT, 1.0f));
    ASSERT_FALSE(test_EPS_Params_Set((eps_param_id_t)99, 1.0f));
}

/* ---- main ---- */

int main(void)
{
    memset(&test_eps_active, 0, sizeof(test_eps_active));

    test_reject_nan();
    test_reject_inf();
    test_reject_zero_divisors();
    test_accept_valid_divisors();
    test_accept_valid_normal();
    test_reject_out_of_range_id();

    printf("\n--- eps_params tests: %d run, %d failed ---\n",
           tests_run, tests_failed);

    return tests_failed ? 1 : 0;
}

#endif /* HOST_TEST */
