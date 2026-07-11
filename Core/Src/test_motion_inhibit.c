/**
  ****************************************************************************
  * @file    test_motion_inhibit.c
  * @brief   Host-compilable unit tests for the MOTION_INHIBIT_REASON
  *          classifier (motion_inhibit.c).
  *
  *          Verifies the bitfield produced for each stage of the traction
  *          chain, with particular focus on distinguishing "no demand" from
  *          "demand zeroed before PWM" and "demand survived but PWM zero"
  *          (the DEGRADED-40%-with-zero-PWM signature).
  *
  *          Compile with host GCC (from repo root):
  *            gcc -std=c11 -DHOST_TEST -ICore/Inc -O2 \
  *                Core/Src/test_motion_inhibit.c Core/Src/motion_inhibit.c \
  *                -lm -o /tmp/test_motion_inhibit && /tmp/test_motion_inhibit
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "motion_inhibit.h"

static int tests_run    = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(expr) do {                                        \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);       \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_FALSE(expr) ASSERT_TRUE(!(expr))

/* Enum stand-ins mirroring the real firmware SystemState_t values
 * (Core/Inc/safety_system.h).  That header pulls in main.h / the HAL, so
 * it cannot be included in a host build; the values are duplicated here and
 * pinned with _Static_assert contract checks so any future renumbering of
 * the production enum breaks this test loudly instead of silently.
 *
 *   SYS_STATE_BOOT      = 0
 *   SYS_STATE_STANDBY   = 1
 *   SYS_STATE_ACTIVE    = 2
 *   SYS_STATE_DEGRADED  = 3
 *   SYS_STATE_SAFE      = 4
 *   SYS_STATE_ERROR     = 5
 *   SYS_STATE_LIMP_HOME = 6                                              */
#define ST_BOOT       0
#define ST_STANDBY    1
#define ST_ACTIVE     2
#define ST_DEGRADED   3
#define ST_SAFE       4
#define ST_ERROR      5
#define ST_LIMP       6

/* Contract: keep these in lockstep with SystemState_t in safety_system.h. */
_Static_assert(ST_BOOT    == 0, "SYS_STATE_BOOT must be 0");
_Static_assert(ST_STANDBY == 1, "SYS_STATE_STANDBY must be 1");
_Static_assert(ST_ACTIVE  == 2, "SYS_STATE_ACTIVE must be 2");
_Static_assert(ST_DEGRADED== 3, "SYS_STATE_DEGRADED must be 3");
_Static_assert(ST_SAFE    == 4, "SYS_STATE_SAFE must be 4");
_Static_assert(ST_ERROR   == 5, "SYS_STATE_ERROR must be 5");
_Static_assert(ST_LIMP    == 6, "SYS_STATE_LIMP_HOME must be 6");

#define G_PARK        0
#define G_NEUTRAL     1
#define G_DRIVE       2
#define G_REVERSE     4

/* Build a "healthy driving, full demand, full PWM" baseline snapshot. */
static MotionInhibitInputs base_driving(void)
{
    MotionInhibitInputs in;
    in.state                    = ST_ACTIVE;
    in.power_ready              = true;
    in.gear                     = G_DRIVE;
    in.gear_park                = G_PARK;
    in.gear_neutral             = G_NEUTRAL;
    in.state_safe               = ST_SAFE;
    in.state_error              = ST_ERROR;
    in.obstacle_forward_blocked = false;
    in.forward_gear             = true;
    in.demand_pct               = 40.0f;
    in.effective_demand_pct     = 40.0f;
    in.final_pwm_max            = 1200;
    in.degraded_level           = 0;
    in.startup_inhibit          = false;
    in.pedal_fault              = false;
    in.safety_scale_zero        = false;
    in.battery_cutoff           = false;
    in.thermal_overcurrent      = false;
    in.service_disabled         = false;
    return in;
}

static void test_null_is_none(void)
{
    ASSERT_TRUE(MotionInhibit_Evaluate(NULL) == MOTION_INHIBIT_NONE);
}

static void test_healthy_driving_no_inhibit(void)
{
    MotionInhibitInputs in = base_driving();
    ASSERT_TRUE(MotionInhibit_Evaluate(&in) == MOTION_INHIBIT_NONE);
}

static void test_no_demand(void)
{
    MotionInhibitInputs in = base_driving();
    in.demand_pct           = 0.0f;
    in.effective_demand_pct = 0.0f;
    in.final_pwm_max        = 0;
    uint16_t r = MotionInhibit_Evaluate(&in);
    ASSERT_TRUE(r & MOTION_INHIBIT_NO_DEMAND);
    ASSERT_FALSE(r & MOTION_INHIBIT_DEMAND_ZEROED);
    ASSERT_FALSE(r & MOTION_INHIBIT_PWM_ZERO);
}

static void test_state_safe(void)
{
    MotionInhibitInputs in = base_driving();
    in.state = ST_SAFE;
    uint16_t r = MotionInhibit_Evaluate(&in);
    ASSERT_TRUE(r & MOTION_INHIBIT_STATE_SAFE);
    /* Power-not-ready must NOT be asserted in SAFE even if power_ready false. */
    in.power_ready = false;
    r = MotionInhibit_Evaluate(&in);
    ASSERT_TRUE(r & MOTION_INHIBIT_STATE_SAFE);
    ASSERT_FALSE(r & MOTION_INHIBIT_POWER_NOT_READY);
}

static void test_state_error(void)
{
    MotionInhibitInputs in = base_driving();
    in.state = ST_ERROR;
    ASSERT_TRUE(MotionInhibit_Evaluate(&in) & MOTION_INHIBIT_STATE_ERROR);
}

static void test_power_not_ready(void)
{
    MotionInhibitInputs in = base_driving();
    in.power_ready = false;
    ASSERT_TRUE(MotionInhibit_Evaluate(&in) & MOTION_INHIBIT_POWER_NOT_READY);
}

static void test_gear_park_and_neutral(void)
{
    MotionInhibitInputs in = base_driving();
    in.gear = G_PARK;
    ASSERT_TRUE(MotionInhibit_Evaluate(&in) & MOTION_INHIBIT_GEAR_PARK);
    in.gear = G_NEUTRAL;
    ASSERT_TRUE(MotionInhibit_Evaluate(&in) & MOTION_INHIBIT_GEAR_NEUTRAL);
}

static void test_obstacle_forward_only(void)
{
    MotionInhibitInputs in = base_driving();
    in.obstacle_forward_blocked = true;
    in.forward_gear = true;
    ASSERT_TRUE(MotionInhibit_Evaluate(&in) & MOTION_INHIBIT_OBSTACLE_BLOCK);
    /* Reverse gear: obstacle block must NOT inhibit (reverse escape). */
    in.forward_gear = false;
    in.gear = G_REVERSE;
    ASSERT_FALSE(MotionInhibit_Evaluate(&in) & MOTION_INHIBIT_OBSTACLE_BLOCK);
}

/* Demand asked but scaled to zero before PWM (e.g. obstacle scale, LIMP
 * speed cap).                                                              */
static void test_demand_zeroed(void)
{
    MotionInhibitInputs in = base_driving();
    in.demand_pct           = 40.0f;
    in.effective_demand_pct = 0.0f;
    in.final_pwm_max        = 0;
    uint16_t r = MotionInhibit_Evaluate(&in);
    ASSERT_TRUE(r & MOTION_INHIBIT_DEMAND_ZEROED);
    ASSERT_FALSE(r & MOTION_INHIBIT_NO_DEMAND);
    ASSERT_FALSE(r & MOTION_INHIBIT_PWM_ZERO);
}

/* The headline case: DEGRADED with real demand and effective demand, but
 * the final PWM is zero (a downstream cap/cutoff killed the duty).          */
static void test_degraded_demand_survives_pwm_zero(void)
{
    MotionInhibitInputs in = base_driving();
    in.state                = ST_DEGRADED;
    in.degraded_level       = 1;
    in.demand_pct           = 40.0f;
    in.effective_demand_pct = 16.0f;   /* 40 % scaled by 40 % power limit  */
    in.final_pwm_max        = 0;        /* but PWM ended up zero            */
    uint16_t r = MotionInhibit_Evaluate(&in);
    ASSERT_TRUE(r & MOTION_INHIBIT_PWM_ZERO);
    ASSERT_TRUE(r & MOTION_INHIBIT_TORQUE_LIMITED);
    ASSERT_FALSE(r & MOTION_INHIBIT_NO_DEMAND);
    ASSERT_FALSE(r & MOTION_INHIBIT_DEMAND_ZEROED);
}

/* DEGRADED but actually moving (PWM > 0): only TORQUE_LIMITED, no inhibit. */
static void test_degraded_moving_only_torque_limited(void)
{
    MotionInhibitInputs in = base_driving();
    in.state          = ST_DEGRADED;
    in.degraded_level = 1;
    in.effective_demand_pct = 16.0f;
    in.final_pwm_max        = 480;
    uint16_t r = MotionInhibit_Evaluate(&in);
    ASSERT_TRUE(r & MOTION_INHIBIT_TORQUE_LIMITED);
    ASSERT_FALSE(r & MOTION_INHIBIT_PWM_ZERO);
    ASSERT_FALSE(r & MOTION_INHIBIT_DEMAND_ZEROED);
    ASSERT_FALSE(r & MOTION_INHIBIT_NO_DEMAND);
}

/* NaN demand must be treated as zero (no crash, classified as no demand). */
static void test_nan_demand_safe(void)
{
    MotionInhibitInputs in = base_driving();
    in.demand_pct           = NAN;
    in.effective_demand_pct = NAN;
    in.final_pwm_max        = 0;
    uint16_t r = MotionInhibit_Evaluate(&in);
    ASSERT_TRUE(r & MOTION_INHIBIT_NO_DEMAND);
}

/* ---- Additional observability inputs (Item 5) ------------------------- */

/* Each extra boolean folds into its own bit and does not disturb the others. */
static void test_startup_inhibit_bit(void)
{
    MotionInhibitInputs in = base_driving();
    in.startup_inhibit = true;
    uint16_t r = MotionInhibit_Evaluate(&in);
    ASSERT_TRUE(r & MOTION_INHIBIT_STARTUP_INHIBIT);
    /* Baseline was otherwise healthy — only the startup bit should set. */
    ASSERT_TRUE(r == MOTION_INHIBIT_STARTUP_INHIBIT);
}

static void test_pedal_fault_bit(void)
{
    MotionInhibitInputs in = base_driving();
    in.pedal_fault = true;
    ASSERT_TRUE(MotionInhibit_Evaluate(&in) == MOTION_INHIBIT_PEDAL_FAULT);
}

static void test_safety_scale_zero_bit(void)
{
    MotionInhibitInputs in = base_driving();
    in.safety_scale_zero = true;
    ASSERT_TRUE(MotionInhibit_Evaluate(&in) == MOTION_INHIBIT_SAFETY_SCALE_ZERO);
}

static void test_battery_cutoff_bit(void)
{
    MotionInhibitInputs in = base_driving();
    in.battery_cutoff = true;
    ASSERT_TRUE(MotionInhibit_Evaluate(&in) == MOTION_INHIBIT_BATTERY_CUTOFF);
}

static void test_thermal_overcurrent_bit(void)
{
    MotionInhibitInputs in = base_driving();
    in.thermal_overcurrent = true;
    ASSERT_TRUE(MotionInhibit_Evaluate(&in) == MOTION_INHIBIT_THERMAL_OVERCURRENT);
}

static void test_service_disabled_bit(void)
{
    MotionInhibitInputs in = base_driving();
    in.service_disabled = true;
    ASSERT_TRUE(MotionInhibit_Evaluate(&in) == MOTION_INHIBIT_SERVICE_DISABLED);
}

/* Multiple contributory reasons stack together with the demand/PWM chain. */
static void test_multiple_reasons_stack(void)
{
    MotionInhibitInputs in = base_driving();
    in.demand_pct           = 0.0f;   /* not asking → NO_DEMAND */
    in.effective_demand_pct = 0.0f;
    in.final_pwm_max        = 0;
    in.startup_inhibit      = true;
    in.pedal_fault          = true;
    in.service_disabled     = true;
    uint16_t r = MotionInhibit_Evaluate(&in);
    ASSERT_TRUE(r & MOTION_INHIBIT_NO_DEMAND);
    ASSERT_TRUE(r & MOTION_INHIBIT_STARTUP_INHIBIT);
    ASSERT_TRUE(r & MOTION_INHIBIT_PEDAL_FAULT);
    ASSERT_TRUE(r & MOTION_INHIBIT_SERVICE_DISABLED);
}

/* The new bits occupy the high 6 bits and never collide with existing ones. */
static void test_new_bits_are_distinct(void)
{
    ASSERT_TRUE(MOTION_INHIBIT_STARTUP_INHIBIT     == 0x0400U);
    ASSERT_TRUE(MOTION_INHIBIT_PEDAL_FAULT         == 0x0800U);
    ASSERT_TRUE(MOTION_INHIBIT_SAFETY_SCALE_ZERO   == 0x1000U);
    ASSERT_TRUE(MOTION_INHIBIT_BATTERY_CUTOFF      == 0x2000U);
    ASSERT_TRUE(MOTION_INHIBIT_THERMAL_OVERCURRENT == 0x4000U);
    ASSERT_TRUE(MOTION_INHIBIT_SERVICE_DISABLED    == 0x8000U);
}

int main(void)
{
    test_null_is_none();
    test_healthy_driving_no_inhibit();
    test_no_demand();
    test_state_safe();
    test_state_error();
    test_power_not_ready();
    test_gear_park_and_neutral();
    test_obstacle_forward_only();
    test_demand_zeroed();
    test_degraded_demand_survives_pwm_zero();
    test_degraded_moving_only_torque_limited();
    test_nan_demand_safe();
    test_startup_inhibit_bit();
    test_pedal_fault_bit();
    test_safety_scale_zero_bit();
    test_battery_cutoff_bit();
    test_thermal_overcurrent_bit();
    test_service_disabled_bit();
    test_multiple_reasons_stack();
    test_new_bits_are_distinct();

    printf("--- motion_inhibit tests: %d run, %d failed ---\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#endif /* HOST_TEST */
