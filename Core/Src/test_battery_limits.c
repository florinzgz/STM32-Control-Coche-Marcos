/**
  ****************************************************************************
  * @file    test_battery_limits.c
  * @brief   Host tests for battery-limit range and coherence validation.
  ****************************************************************************
  */

#ifdef HOST_TEST

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "battery_limits_store.h"

static int tests_run;
static int tests_failed;

#define ASSERT_TRUE(expr) do {                                        \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);       \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_FALSE(expr) ASSERT_TRUE(!(expr))

static BatteryLimits_t defaults(void)
{
    BatteryLimits_t b = {
        BATT_WARNING_DEFAULT_CV,
        BATT_LIMIT_DEFAULT_CV,
        BATT_CUTOFF_DEFAULT_CV,
        BATT_RECOVERY_DEFAULT_CV,
        BATT_FILTER_DEFAULT_MS
    };
    return b;
}

int main(void)
{
    BatteryLimits_t b = defaults();
    ASSERT_TRUE(BatteryLimits_ValidateValues(&b));

    /* Installed traction-pack defaults. */
    ASSERT_TRUE(BATT_WARNING_DEFAULT_CV  == 1800U);
    ASSERT_TRUE(BATT_LIMIT_DEFAULT_CV    == 1700U);
    ASSERT_TRUE(BATT_CUTOFF_DEFAULT_CV   == 1600U);
    ASSERT_TRUE(BATT_RECOVERY_DEFAULT_CV == 1700U);
    ASSERT_TRUE(BATT_FILTER_DEFAULT_MS   == 500U);

    b = defaults(); b.warning_cv = b.cutoff_cv;       ASSERT_FALSE(BatteryLimits_ValidateValues(&b));
    b = defaults(); b.warning_cv = b.cutoff_cv - 1U;  ASSERT_FALSE(BatteryLimits_ValidateValues(&b));
    b = defaults(); b.limit_cv = b.cutoff_cv;         ASSERT_FALSE(BatteryLimits_ValidateValues(&b));
    b = defaults(); b.recovery_cv = b.cutoff_cv;      ASSERT_FALSE(BatteryLimits_ValidateValues(&b));
    b = defaults(); b.recovery_cv = b.cutoff_cv - 1U; ASSERT_FALSE(BatteryLimits_ValidateValues(&b));

    ASSERT_TRUE(BATT_OV_WARNING_CV == 3000U);
    b = defaults(); b.warning_cv = BATT_OV_WARNING_CV;      ASSERT_TRUE(BatteryLimits_ValidateValues(&b));
    b = defaults(); b.warning_cv = BATT_OV_WARNING_CV + 1U; ASSERT_FALSE(BatteryLimits_ValidateValues(&b));
    b = defaults(); b.limit_cv = BATT_OV_WARNING_CV;        ASSERT_TRUE(BatteryLimits_ValidateValues(&b));
    b = defaults(); b.limit_cv = BATT_OV_WARNING_CV + 1U;   ASSERT_FALSE(BatteryLimits_ValidateValues(&b));

    b = defaults(); b.cutoff_cv = BATT_CUTOFF_MIN_CV - 1U; ASSERT_FALSE(BatteryLimits_ValidateValues(&b));
    b = defaults(); b.cutoff_cv = BATT_CUTOFF_MAX_CV + 1U; ASSERT_FALSE(BatteryLimits_ValidateValues(&b));
    b = defaults(); b.recovery_cv = BATT_RECOVERY_MAX_CV + 1U; ASSERT_FALSE(BatteryLimits_ValidateValues(&b));
    b = defaults(); b.filter_ms = BATT_FILTER_MAX_MS + 1U; ASSERT_FALSE(BatteryLimits_ValidateValues(&b));

    b = defaults(); b.filter_ms = 0U;    ASSERT_TRUE(BatteryLimits_ValidateValues(&b));
    b = defaults(); b.filter_ms = 250U;  ASSERT_TRUE(BatteryLimits_ValidateValues(&b));

    /* Exact requested low-voltage profile. */
    b.warning_cv = 1800U;
    b.limit_cv = 1700U;
    b.cutoff_cv = 1600U;
    b.recovery_cv = 1700U;
    b.filter_ms = 500U;
    ASSERT_TRUE(BatteryLimits_ValidateValues(&b));

    printf("test_battery_limits: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}

#endif /* HOST_TEST */
