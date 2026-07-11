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

/* Enum stand-ins matching the real firmware values passed in the snapshot.
 * The classifier compares against the values carried in the struct, so the
 * test just has to be self-consistent.                                     */
#define ST_ACTIVE     3
#define ST_DEGRADED   4
#define ST_SAFE       5
#define ST_ERROR      2
#define ST_LIMP       6

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

    printf("--- motion_inhibit tests: %d run, %d failed ---\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#endif /* HOST_TEST */
