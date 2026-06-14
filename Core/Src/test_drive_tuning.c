/**
  ****************************************************************************
  * @file    test_drive_tuning.c
  * @brief   Host-compilable unit tests for drive-tuning validation (FASE 2/4).
  *
  *          Validates the range/coherence logic of DriveTuningStore_Validate()
  *          and confirms the compile-time defaults reproduce the historic
  *          firmware behaviour, without requiring flash or HAL.
  *
  *          Compile with host GCC:
  *            gcc -std=c11 -DHOST_TEST -Ianalysis_artifacts/stubs \
  *                -ICore/Inc -O2 -lm Core/Src/test_drive_tuning.c \
  *                -o test_drive_tuning
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

/* Pull in the single source of truth for ranges + defaults.  The header is
 * HAL-free (pure macros + struct + _Static_assert), so it compiles alone. */
#include "drive_tuning_store.h"

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

/* Re-implement DriveTuningStore_Validate() with the SAME logic as
 * drive_tuning_store.c, referencing the canonical range macros so any
 * future range change is reflected here automatically.                  */
static bool test_validate(const DriveTuning_t *t)
{
    if (t->accel_ramp   < DRIVE_ACCEL_RAMP_MIN   || t->accel_ramp   > DRIVE_ACCEL_RAMP_MAX)   return false;
    if (t->brake_ramp   < DRIVE_BRAKE_RAMP_MIN   || t->brake_ramp   > DRIVE_BRAKE_RAMP_MAX)   return false;
    if (t->reverse_ramp < DRIVE_REVERSE_RAMP_MIN || t->reverse_ramp > DRIVE_REVERSE_RAMP_MAX) return false;
    if (t->creep_enable > 1U)                                                                 return false;
#if (DRIVE_CREEP_POWER_MIN > 0U)
    if (t->creep_power  < DRIVE_CREEP_POWER_MIN)                                               return false;
#endif
    if (t->creep_power  > DRIVE_CREEP_POWER_MAX)                                               return false;
#if (DRIVE_CREEP_DELAY_MIN > 0U)
    if (t->creep_delay  < DRIVE_CREEP_DELAY_MIN)                                               return false;
#endif
    if (t->creep_delay  > DRIVE_CREEP_DELAY_MAX)                                               return false;
    return true;
}

static DriveTuning_t defaults(void)
{
    DriveTuning_t t;
    t.accel_ramp   = DRIVE_ACCEL_RAMP_DEFAULT;
    t.brake_ramp   = DRIVE_BRAKE_RAMP_DEFAULT;
    t.reverse_ramp = DRIVE_REVERSE_RAMP_DEFAULT;
    t.creep_enable = DRIVE_CREEP_ENABLE_DEFAULT;
    t.creep_power  = DRIVE_CREEP_POWER_DEFAULT;
    t.creep_delay  = DRIVE_CREEP_DELAY_DEFAULT;
    return t;
}

int main(void)
{
    /* Defaults must always validate (RESET DEFAULTS must never reject). */
    DriveTuning_t d = defaults();
    ASSERT_TRUE(test_validate(&d));

    /* Defaults MIRROR the historic compile-time firmware behaviour. */
    ASSERT_TRUE(DRIVE_ACCEL_RAMP_DEFAULT   == 50U);   /* PEDAL_RAMP_UP_PCT_S   */
    ASSERT_TRUE(DRIVE_BRAKE_RAMP_DEFAULT   == 100U);  /* PEDAL_RAMP_DOWN_PCT_S */
    ASSERT_TRUE(DRIVE_REVERSE_RAMP_DEFAULT == 50U);   /* == AccelRamp          */
    ASSERT_TRUE(DRIVE_CREEP_ENABLE_DEFAULT == 1U);    /* creep on              */
    ASSERT_TRUE(DRIVE_CREEP_POWER_DEFAULT  == 8U);    /* MOTOR_DEADZONE_PCT    */
    ASSERT_TRUE(DRIVE_CREEP_DELAY_DEFAULT  == 0U);    /* no delay              */

    /* Ramp rates must be strictly positive (FASE 4: AccelRamp/BrakeRamp/
     * ReverseRamp > 0).  Zero ramp must be rejected on every channel.       */
    d = defaults(); d.accel_ramp   = 0; ASSERT_FALSE(test_validate(&d));
    d = defaults(); d.brake_ramp   = 0; ASSERT_FALSE(test_validate(&d));
    d = defaults(); d.reverse_ramp = 0; ASSERT_FALSE(test_validate(&d));
    ASSERT_TRUE(DRIVE_ACCEL_RAMP_MIN   >= 1U);
    ASSERT_TRUE(DRIVE_BRAKE_RAMP_MIN   >= 1U);
    ASSERT_TRUE(DRIVE_REVERSE_RAMP_MIN >= 1U);

    /* Boundary acceptance — exact min/max are valid. */
    d = defaults();
    d.accel_ramp = DRIVE_ACCEL_RAMP_MIN; d.brake_ramp = DRIVE_BRAKE_RAMP_MIN;
    d.reverse_ramp = DRIVE_REVERSE_RAMP_MIN; d.creep_power = DRIVE_CREEP_POWER_MIN;
    d.creep_delay = DRIVE_CREEP_DELAY_MIN;
    ASSERT_TRUE(test_validate(&d));
    d.accel_ramp = DRIVE_ACCEL_RAMP_MAX; d.brake_ramp = DRIVE_BRAKE_RAMP_MAX;
    d.reverse_ramp = DRIVE_REVERSE_RAMP_MAX; d.creep_power = DRIVE_CREEP_POWER_MAX;
    d.creep_delay = DRIVE_CREEP_DELAY_MAX;
    ASSERT_TRUE(test_validate(&d));

    /* Above-max rejection on each channel. */
    d = defaults(); d.accel_ramp   = DRIVE_ACCEL_RAMP_MAX   + 1U; ASSERT_FALSE(test_validate(&d));
    d = defaults(); d.brake_ramp   = DRIVE_BRAKE_RAMP_MAX   + 1U; ASSERT_FALSE(test_validate(&d));
    d = defaults(); d.reverse_ramp = DRIVE_REVERSE_RAMP_MAX + 1U; ASSERT_FALSE(test_validate(&d));
    d = defaults(); d.creep_power  = DRIVE_CREEP_POWER_MAX  + 1U; ASSERT_FALSE(test_validate(&d));
    d = defaults(); d.creep_delay  = DRIVE_CREEP_DELAY_MAX  + 1U; ASSERT_FALSE(test_validate(&d));

    /* CreepPower may legitimately be 0 (creep floor disabled). */
    d = defaults(); d.creep_power = 0; ASSERT_TRUE(test_validate(&d));

    /* CreepEnable accepts only 0 or 1. */
    d = defaults(); d.creep_enable = 0; ASSERT_TRUE(test_validate(&d));
    d = defaults(); d.creep_enable = 1; ASSERT_TRUE(test_validate(&d));
    d = defaults(); d.creep_enable = 2; ASSERT_FALSE(test_validate(&d));

    /* A realistic re-tune (slower accel, snappier brake, gentle reverse,
     * stronger creep with a short delay) must validate. */
    d.accel_ramp = 30; d.brake_ramp = 120; d.reverse_ramp = 25;
    d.creep_enable = 1; d.creep_power = 12; d.creep_delay = 500;
    ASSERT_TRUE(test_validate(&d));

    printf("test_drive_tuning: %d run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}

#endif /* HOST_TEST */
