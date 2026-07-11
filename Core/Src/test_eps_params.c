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

/* ---- Drive the REAL EPS_Params_Set() ----
 * The CI compiles this test together with the real Core/Src/eps_params.c
 * (see .github/workflows/firmware-validation.yml), so the tests exercise
 * the authoritative server-side validation logic directly rather than a
 * re-implementation.  This guarantees the test and firmware can never
 * silently diverge.                                                     */

#include "eps_params.h"

/* ---- Tests ---- */

static void test_reject_nan(void)
{
    /* NaN must be rejected for every parameter */
    for (int i = 0; i < EPS_PARAM_COUNT; i++) {
        ASSERT_FALSE(EPS_Params_Set((eps_param_id_t)i, NAN));
    }
}

static void test_reject_inf(void)
{
    /* +Inf and -Inf must be rejected for every parameter */
    for (int i = 0; i < EPS_PARAM_COUNT; i++) {
        ASSERT_FALSE(EPS_Params_Set((eps_param_id_t)i, INFINITY));
        ASSERT_FALSE(EPS_Params_Set((eps_param_id_t)i, -INFINITY));
    }
}

static void test_reject_zero_divisors(void)
{
    /* assist_vs_speed and return_vs_speed must reject <= 0 */
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_ASSIST_VS_SPEED, 0.0f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_ASSIST_VS_SPEED, -1.0f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_RETURN_VS_SPEED, 0.0f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_RETURN_VS_SPEED, -5.0f));
}

static void test_accept_valid_divisors(void)
{
    /* Positive values must be accepted for divisor parameters */
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_ASSIST_VS_SPEED, 18.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_RETURN_VS_SPEED, 35.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_ASSIST_VS_SPEED, 0.001f));
}

static void test_accept_valid_normal(void)
{
    /* Normal values for non-divisor parameters must be accepted */
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_ASSIST_STRENGTH, 0.45f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_CENTER_STRENGTH, 0.30f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_DAMPING, 0.10f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_FRICTION_COMP, 0.05f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_COAST_BAND_PCT, 3.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_MIN_DRIVE_PCT, 8.0f));
}

static void test_reject_out_of_range_id(void)
{
    /* Out-of-range parameter ID must be rejected */
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_COUNT, 1.0f));
    ASSERT_FALSE(EPS_Params_Set((eps_param_id_t)99, 1.0f));
}

static void test_reject_zero_deadband(void)
{
    /* deadband_deg <= 0 must be rejected (would disable deadband) */
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_DEADBAND_DEG, 0.0f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_DEADBAND_DEG, -1.0f));
    /* Positive values must be accepted */
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_DEADBAND_DEG, 1.8f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_DEADBAND_DEG, 0.1f));
}

static void test_reject_invalid_max_pwm(void)
{
    /* max_pwm_pct must be > 0 and <= 100 */
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_MAX_PWM_PCT, 0.0f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_MAX_PWM_PCT, -5.0f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_MAX_PWM_PCT, 100.1f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_MAX_PWM_PCT, 200.0f));
    /* Valid boundary values must be accepted */
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_MAX_PWM_PCT, 60.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_MAX_PWM_PCT, 100.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_MAX_PWM_PCT, 1.0f));
}

static void test_reject_zero_slew(void)
{
    /* slew_rate_pct <= 0 must be rejected (would freeze slew) */
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_SLEW_RATE_PCT, 0.0f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_SLEW_RATE_PCT, -0.1f));
    /* Positive slew accepted */
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_SLEW_RATE_PCT, 5.883f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_SLEW_RATE_PCT, 0.1f));
}

static void test_accept_valid_mechanical(void)
{
    /* All four new mechanical-limit parameters accept their default values */
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_DEADBAND_DEG,     1.8f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_MAX_PWM_PCT,      60.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_SLEW_RATE_PCT,    5.883f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_CENTER_OFFSET_DEG, 0.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_CENTER_OFFSET_DEG, -5.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_CENTER_OFFSET_DEG,  5.0f));
}

/* ---- Fase 5: server-side upper-bound enforcement ----
 * The HMI is not trusted to clamp its own inputs; the STM32 must reject
 * out-of-range values for EVERY parameter, not just the divisor/mechanical
 * ones that were previously guarded.                                     */

static void test_reject_gain_over_limit(void)
{
    /* Gains are bounded to [0, 1]; anything above 1.0 must be rejected. */
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_ASSIST_STRENGTH, 1.01f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_ASSIST_STRENGTH, 1000.0f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_CENTER_STRENGTH, 5.0f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_DAMPING,         2.0f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_FRICTION_COMP,   1.5f));
    /* Negative gains are also out of range. */
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_ASSIST_STRENGTH, -0.01f));
    /* Boundary values 0.0 and 1.0 must be accepted. */
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_ASSIST_STRENGTH, 0.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_ASSIST_STRENGTH, 1.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_DAMPING,         1.0f));
}

static void test_reject_pct_over_limit(void)
{
    /* Percent-band parameters are bounded to [0, 100]. */
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_COAST_BAND_PCT, 100.1f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_COAST_BAND_PCT, 1000.0f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_COAST_BAND_PCT, -1.0f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_MIN_DRIVE_PCT,  100.1f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_MIN_DRIVE_PCT,  -5.0f));
    /* Boundaries accepted. */
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_COAST_BAND_PCT, 0.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_COAST_BAND_PCT, 100.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_MIN_DRIVE_PCT,  100.0f));
}

static void test_reject_divisor_over_limit(void)
{
    /* Divisors are bounded above at 200 to reject absurd HMI inputs. */
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_ASSIST_VS_SPEED, 200.1f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_ASSIST_VS_SPEED, 1e6f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_RETURN_VS_SPEED, 500.0f));
    /* Upper boundary accepted. */
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_ASSIST_VS_SPEED, 200.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_RETURN_VS_SPEED, 200.0f));
}

static void test_reject_deadband_over_limit(void)
{
    /* deadband_deg is bounded to (0, 30]. */
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_DEADBAND_DEG, 30.1f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_DEADBAND_DEG, 1000.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_DEADBAND_DEG, 30.0f));
}

static void test_reject_slew_over_limit(void)
{
    /* slew_rate_pct is bounded to (0, 100]. */
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_SLEW_RATE_PCT, 100.1f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_SLEW_RATE_PCT, 5000.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_SLEW_RATE_PCT, 100.0f));
}

static void test_center_offset_range(void)
{
    /* center_offset_deg is bounded to [-10, +10]; previously unbounded. */
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_CENTER_OFFSET_DEG, 10.1f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_CENTER_OFFSET_DEG, -10.1f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_CENTER_OFFSET_DEG, 9999.0f));
    ASSERT_FALSE(EPS_Params_Set(EPS_PARAM_CENTER_OFFSET_DEG, -9999.0f));
    /* Boundaries accepted. */
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_CENTER_OFFSET_DEG,  10.0f));
    ASSERT_TRUE(EPS_Params_Set(EPS_PARAM_CENTER_OFFSET_DEG, -10.0f));
}

/* ---- main ---- */

/* ---- Item A: SET_PARAM safety gate ---- */

static eps_setparam_gate_t safe_gate(void)
{
    /* A gate snapshot that satisfies all five conditions.  The reference
     * enum values are supplied by the snapshot itself (pure predicate),
     * so the test needs no firmware enums. */
    eps_setparam_gate_t g;
    g.state               = 1;   /* == state_standby below (STANDBY)      */
    g.state_standby       = 1;
    g.gear                = 0;   /* == gear_park below (PARK)             */
    g.gear_park           = 0;
    g.gear_neutral        = 2;
    g.max_wheel_speed_kmh = 0.0f;
    g.demand_pct          = 0.0f;
    g.dangerous_fault     = false;
    return g;
}

static void test_setparam_gate_allows_safe_state(void)
{
    eps_setparam_gate_t g = safe_gate();
    ASSERT_TRUE(EPS_Params_SetAllowed(&g));   /* PARK, stationary, no fault */
    g.gear = g.gear_neutral;                  /* NEUTRAL is also allowed    */
    ASSERT_TRUE(EPS_Params_SetAllowed(&g));
}

static void test_setparam_gate_requires_standby(void)
{
    eps_setparam_gate_t g = safe_gate();
    g.state = 0; ASSERT_FALSE(EPS_Params_SetAllowed(&g)); /* BOOT     */
    g.state = 2; ASSERT_FALSE(EPS_Params_SetAllowed(&g)); /* ACTIVE   */
    g.state = 3; ASSERT_FALSE(EPS_Params_SetAllowed(&g)); /* DEGRADED */
    g.state = 4; ASSERT_FALSE(EPS_Params_SetAllowed(&g)); /* SAFE     */
    g.state = 6; ASSERT_FALSE(EPS_Params_SetAllowed(&g)); /* LIMP     */
}

static void test_setparam_gate_requires_park_or_neutral(void)
{
    eps_setparam_gate_t g = safe_gate();
    g.gear = 1; ASSERT_FALSE(EPS_Params_SetAllowed(&g)); /* REVERSE */
    g.gear = 3; ASSERT_FALSE(EPS_Params_SetAllowed(&g)); /* DRIVE   */
}

static void test_setparam_gate_requires_stationary(void)
{
    eps_setparam_gate_t g = safe_gate();
    g.max_wheel_speed_kmh = EPS_SETPARAM_MAX_WHEEL_SPEED_KMH + 0.5f;
    ASSERT_FALSE(EPS_Params_SetAllowed(&g));
    g.max_wheel_speed_kmh = -(EPS_SETPARAM_MAX_WHEEL_SPEED_KMH + 0.5f);
    ASSERT_FALSE(EPS_Params_SetAllowed(&g));   /* magnitude, sign ignored */
    g.max_wheel_speed_kmh = EPS_SETPARAM_MAX_WHEEL_SPEED_KMH;  /* boundary */
    ASSERT_TRUE(EPS_Params_SetAllowed(&g));
    g.max_wheel_speed_kmh = NAN;               /* unknown motion -> unsafe */
    ASSERT_FALSE(EPS_Params_SetAllowed(&g));
}

static void test_setparam_gate_requires_zero_demand(void)
{
    eps_setparam_gate_t g = safe_gate();
    g.demand_pct = EPS_SETPARAM_MAX_DEMAND_PCT;      /* at threshold -> no */
    ASSERT_FALSE(EPS_Params_SetAllowed(&g));
    g.demand_pct = -40.0f;                           /* reverse demand    */
    ASSERT_FALSE(EPS_Params_SetAllowed(&g));
    g.demand_pct = NAN;
    ASSERT_FALSE(EPS_Params_SetAllowed(&g));
    g.demand_pct = 2.0f;                             /* below threshold OK */
    ASSERT_TRUE(EPS_Params_SetAllowed(&g));
}

static void test_setparam_gate_requires_no_fault(void)
{
    eps_setparam_gate_t g = safe_gate();
    g.dangerous_fault = true;
    ASSERT_FALSE(EPS_Params_SetAllowed(&g));
}

static void test_setparam_gate_null(void)
{
    ASSERT_FALSE(EPS_Params_SetAllowed(NULL));
}

int main(void)
{
    test_reject_nan();
    test_reject_inf();
    test_reject_zero_divisors();
    test_accept_valid_divisors();
    test_accept_valid_normal();
    test_reject_out_of_range_id();
    test_reject_zero_deadband();
    test_reject_invalid_max_pwm();
    test_reject_zero_slew();
    test_accept_valid_mechanical();
    test_reject_gain_over_limit();
    test_reject_pct_over_limit();
    test_reject_divisor_over_limit();
    test_reject_deadband_over_limit();
    test_reject_slew_over_limit();
    test_center_offset_range();

    test_setparam_gate_allows_safe_state();
    test_setparam_gate_requires_standby();
    test_setparam_gate_requires_park_or_neutral();
    test_setparam_gate_requires_stationary();
    test_setparam_gate_requires_zero_demand();
    test_setparam_gate_requires_no_fault();
    test_setparam_gate_null();

    printf("\n--- eps_params tests: %d run, %d failed ---\n",
           tests_run, tests_failed);

    return tests_failed ? 1 : 0;
}

#endif /* HOST_TEST */
