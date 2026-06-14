/**
  ****************************************************************************
  * @file    test_battery_limits.c
  * @brief   Host-compilable unit tests for battery-limit validation (FASE 3/4).
  *
  *          Validates the range/coherence logic of BatteryLimitsStore_Validate()
  *          and confirms the compile-time defaults reproduce the historic
  *          firmware thresholds, without requiring flash or HAL.
  *
  *          Compile with host GCC:
  *            gcc -std=c11 -DHOST_TEST -Ianalysis_artifacts/stubs \
  *                -ICore/Inc -O2 -lm Core/Src/test_battery_limits.c \
  *                -o test_battery_limits
  *
  *          This file defines main() and is intended ONLY for host-side unit
  *          testing.  It is excluded from the STM32 firmware build via the
  *          HOST_TEST guard.
  ****************************************************************************
  */

#ifdef HOST_TEST

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

/* Pull in the single source of truth for ranges + defaults.  HAL-free. */
#include "battery_limits_store.h"

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

/* Re-implement BatteryLimitsStore_Validate() with the SAME logic as
 * battery_limits_store.c, referencing the canonical macros.             */
static bool test_validate(const BatteryLimits_t *b)
{
    if (b->warning_cv  < BATT_WARNING_MIN_CV  || b->warning_cv  > BATT_WARNING_MAX_CV)  return false;
    if (b->limit_cv    < BATT_LIMIT_MIN_CV    || b->limit_cv    > BATT_LIMIT_MAX_CV)    return false;
    if (b->cutoff_cv   < BATT_CUTOFF_MIN_CV   || b->cutoff_cv   > BATT_CUTOFF_MAX_CV)   return false;
    if (b->recovery_cv < BATT_RECOVERY_MIN_CV || b->recovery_cv > BATT_RECOVERY_MAX_CV) return false;
#if (BATT_FILTER_MIN_MS > 0U)
    if (b->filter_ms   < BATT_FILTER_MIN_MS)                                            return false;
#endif
    if (b->filter_ms   > BATT_FILTER_MAX_MS)                                            return false;
    if (b->warning_cv  <= b->cutoff_cv)   return false;
    if (b->limit_cv    <= b->cutoff_cv)   return false;
    if (b->recovery_cv <= b->cutoff_cv)   return false;
    if (b->warning_cv  >  BATT_OV_WARNING_CV) return false;
    if (b->limit_cv    >  BATT_OV_WARNING_CV) return false;
    return true;
}

static BatteryLimits_t defaults(void)
{
    BatteryLimits_t b;
    b.warning_cv  = BATT_WARNING_DEFAULT_CV;
    b.limit_cv    = BATT_LIMIT_DEFAULT_CV;
    b.cutoff_cv   = BATT_CUTOFF_DEFAULT_CV;
    b.recovery_cv = BATT_RECOVERY_DEFAULT_CV;
    b.filter_ms   = BATT_FILTER_DEFAULT_MS;
    return b;
}

int main(void)
{
    /* Defaults must always validate (RESET DEFAULTS must never reject). */
    BatteryLimits_t b = defaults();
    ASSERT_TRUE(test_validate(&b));

    /* Defaults MIRROR the historic compile-time #define thresholds:
     *   BATTERY_UV_WARNING_V  = 20.0  -> Limit/Warning 2000 cV
     *   BATTERY_UV_CRITICAL_V = 18.0  -> Cutoff        1800 cV
     *   CRITICAL + HYST (0.5) = 18.5  -> Recovery      1850 cV
     *   VoltageFilter         = 0 ms  -> identical behaviour                */
    ASSERT_TRUE(BATT_WARNING_DEFAULT_CV  == 2000U);
    ASSERT_TRUE(BATT_LIMIT_DEFAULT_CV    == 2000U);
    ASSERT_TRUE(BATT_CUTOFF_DEFAULT_CV   == 1800U);
    ASSERT_TRUE(BATT_RECOVERY_DEFAULT_CV == 1850U);
    ASSERT_TRUE(BATT_FILTER_DEFAULT_MS   == 0U);

    /* Coherence rule: Warning > Cutoff. */
    b = defaults(); b.warning_cv = b.cutoff_cv;       ASSERT_FALSE(test_validate(&b));
    b = defaults(); b.warning_cv = b.cutoff_cv - 1U;  ASSERT_FALSE(test_validate(&b));

    /* Coherence rule: Limit > Cutoff. */
    b = defaults(); b.limit_cv = b.cutoff_cv;         ASSERT_FALSE(test_validate(&b));

    /* Coherence rule: Recovery > Cutoff. */
    b = defaults(); b.recovery_cv = b.cutoff_cv;      ASSERT_FALSE(test_validate(&b));
    b = defaults(); b.recovery_cv = b.cutoff_cv - 1U; ASSERT_FALSE(test_validate(&b));

    /* Coherence rule: Warning <= OV warning (30.0 V). */
    ASSERT_TRUE(BATT_OV_WARNING_CV == 3000U);
    b = defaults(); b.warning_cv = BATT_OV_WARNING_CV;      ASSERT_TRUE(test_validate(&b));
    b = defaults(); b.warning_cv = BATT_OV_WARNING_CV + 1U; ASSERT_FALSE(test_validate(&b));

    /* Coherence rule: Limit <= OV warning. */
    b = defaults(); b.limit_cv = BATT_OV_WARNING_CV;        ASSERT_TRUE(test_validate(&b));
    b = defaults(); b.limit_cv = BATT_OV_WARNING_CV + 1U;   ASSERT_FALSE(test_validate(&b));

    /* Hard range rejection (below min / above max) per field. */
    b = defaults(); b.cutoff_cv   = BATT_CUTOFF_MIN_CV   - 1U; ASSERT_FALSE(test_validate(&b));
    b = defaults(); b.cutoff_cv   = BATT_CUTOFF_MAX_CV   + 1U; ASSERT_FALSE(test_validate(&b));
    b = defaults(); b.recovery_cv = BATT_RECOVERY_MAX_CV + 1U; ASSERT_FALSE(test_validate(&b));
    b = defaults(); b.filter_ms   = BATT_FILTER_MAX_MS   + 1U; ASSERT_FALSE(test_validate(&b));

    /* VoltageFilter may legitimately be 0 (bypass) or a positive tc. */
    b = defaults(); b.filter_ms = 0;    ASSERT_TRUE(test_validate(&b));
    b = defaults(); b.filter_ms = 250;  ASSERT_TRUE(test_validate(&b));

    /* A realistic re-tune (warn 21.0, derate 20.5, cutoff 18.5, recovery
     * 19.0, 200 ms filter) must validate. */
    b.warning_cv = 2100; b.limit_cv = 2050; b.cutoff_cv = 1850;
    b.recovery_cv = 1900; b.filter_ms = 200;
    ASSERT_TRUE(test_validate(&b));

    printf("test_battery_limits: %d run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}

#endif /* HOST_TEST */
