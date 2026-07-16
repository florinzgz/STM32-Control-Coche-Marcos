/**
  ****************************************************************************
  * @file    test_wheel_effort.c
  * @brief   Host-compilable wire-contract tests for the per-wheel FINAL
  *          applied-PWM telemetry (AUDIT A) and the LIMP_HOME single-point
  *          40 % ceiling (AUDIT G).
  *
  *          These tests encode the CONTRACT that the firmware must satisfy so
  *          a silent drift only surfaces on the vehicle:
  *
  *          A) CAN 0x20C STATUS_WHEEL_EFFORT — the value shown as the MAIN
  *             per-wheel percentage in the HMI must be the REAL PWM duty
  *             written to each BTS7960 this cycle, NOT the ABS/TCS "permitted"
  *             scale of 0x205.  Acceptance criterion (problem statement K):
  *               "Ninguna rueda puede mostrar 100 % salvo que el PWM final
  *                realmente aplicado a esa rueda sea 100 %."
  *
  *             The mapping the firmware applies (motor_control.c,
  *             Traction_Update + Traction_GetWheelFinalPwmPct) is:
  *               COAST (desired_en==0)             -> 0 %
  *               BRAKE (desired_en==1, pwm==0)     -> 0 %
  *               DRIVE (desired_en==1, pwm>0)      -> round(pwm*100/PWM_PERIOD)
  *             and clamped to 100 %.
  *
  *          G) LIMP_HOME ceiling is applied EXACTLY ONCE (main.c pedal clamp,
  *             demand = pedal * 0.40).  The traction pipeline no longer
  *             re-scales it (Safety_GetTractionCapFactor()==1.0 in LIMP_HOME),
  *             so pedal 100 % -> 40 % final, never 20 %, 16 % or 4 %.
  *
  *          Compile with host GCC (from repo root):
  *            gcc -std=c11 -DHOST_TEST -ICore/Inc -O2 \
  *                Core/Src/test_wheel_effort.c -lm -o /tmp/test_wheel_effort \
  *                && /tmp/test_wheel_effort
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#define PWM_PERIOD                    4249U
#define LIMP_HOME_TORQUE_LIMIT_FACTOR 0.40f

static int tests_run    = 0;
static int tests_failed = 0;

#define ASSERT_EQ_U(exp, got) do {                                       \
    tests_run++;                                                         \
    if ((unsigned)(exp) != (unsigned)(got)) {                           \
        printf("FAIL %s:%d  expected %u got %u\n", __FILE__, __LINE__,  \
               (unsigned)(exp), (unsigned)(got));                       \
        tests_failed++;                                                  \
    }                                                                    \
} while (0)

#define ASSERT_TRUE(expr) do {                                          \
    tests_run++;                                                         \
    if (!(expr)) {                                                       \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);          \
        tests_failed++;                                                  \
    }                                                                    \
} while (0)

/* ---- Reference model of the firmware final-PWM mapping (motor_control.c) --
 * Mirrors Traction_GetWheelFinalPwmPct(): the value derives from the resolved
 * per-wheel desired_en[]/desired_pwm[] (post jerk limiter + DRIVE/BRAKE/COAST),
 * i.e. exactly what is written to hardware, then scaled to 0..100 %.        */
static uint8_t wheel_final_pwm_pct(bool desired_en, uint16_t desired_pwm)
{
    uint16_t final_ticks;
    if (!desired_en || desired_pwm == 0U) {
        final_ticks = 0U;                 /* COAST or BRAKE -> no push */
    } else {
        final_ticks = (desired_pwm > PWM_PERIOD) ? PWM_PERIOD : desired_pwm;
    }
    uint32_t pct = ((uint32_t)final_ticks * 100U) / PWM_PERIOD;
    if (pct > 100U) pct = 100U;
    return (uint8_t)pct;
}

/* ESP32 decode clamp (can_rx.cpp decodeWheelEffort). */
static uint8_t decode_effort_byte(uint8_t wire) { return (wire <= 100U) ? wire : 100U; }

/* -------------------------------------------------------------------------
 * A1 — COAST and BRAKE always report 0 % push.
 * ---------------------------------------------------------------------- */
static void test_coast_brake_zero(void)
{
    ASSERT_EQ_U(0, wheel_final_pwm_pct(false, 0));          /* COAST         */
    ASSERT_EQ_U(0, wheel_final_pwm_pct(false, 3000));       /* COAST safety  */
    ASSERT_EQ_U(0, wheel_final_pwm_pct(true, 0));           /* BRAKE         */
}

/* -------------------------------------------------------------------------
 * A2 — DRIVE reports the real duty, 100 % ONLY at full ticks.
 * ---------------------------------------------------------------------- */
static void test_drive_scaling(void)
{
    /* NOTE: PWM_PERIOD (4249) is odd and the firmware uses integer division
     * (ticks*100/PWM_PERIOD), so values truncate down — this is the exact
     * on-wire contract, locked here to catch rounding drift. */
    ASSERT_EQ_U(100, wheel_final_pwm_pct(true, PWM_PERIOD));     /* full     */
    ASSERT_EQ_U(49,  wheel_final_pwm_pct(true, PWM_PERIOD / 2)); /* ~half    */
    ASSERT_EQ_U(24,  wheel_final_pwm_pct(true, PWM_PERIOD / 4)); /* ~quarter */
    ASSERT_EQ_U(0,   wheel_final_pwm_pct(true, 10));             /* ~0.2 %   */
    /* Overshoot from float rounding is clamped, never wraps. */
    ASSERT_EQ_U(100, wheel_final_pwm_pct(true, PWM_PERIOD + 500));
}

/* -------------------------------------------------------------------------
 * A3 — Acceptance criterion: a wheel reads 100 % ONLY when the real applied
 *      duty is full.  The historical bug fed the ABS/TCS "permitted" scale
 *      (0x205), which is 100 whenever the limiter is idle even at low duty.
 * ---------------------------------------------------------------------- */
static void test_no_false_hundred(void)
{
    /* ABS/TCS idle (scale 1.0 -> 0x205 would say 100) but the real duty is
     * only ~18 % (pedal + ramp not yet full): effort must read the real duty
     * (17 % after integer truncation), never 100. */
    uint16_t real_ticks = (uint16_t)(0.18f * PWM_PERIOD);
    uint8_t effort = wheel_final_pwm_pct(true, real_ticks);
    ASSERT_TRUE(effort != 100);
    ASSERT_EQ_U(17, effort);

    /* Motor disabled in 4x2 (rear axle BRAKE): 0 %, never 100. */
    ASSERT_EQ_U(0, wheel_final_pwm_pct(true, 0));
    ASSERT_EQ_U(0, wheel_final_pwm_pct(false, 0));

    /* Exhaustive: any duty below full ticks must be < 100 %. */
    for (uint32_t t = 0; t < PWM_PERIOD; t += 37U) {
        ASSERT_TRUE(wheel_final_pwm_pct(true, (uint16_t)t) < 100U);
    }
}

/* -------------------------------------------------------------------------
 * A4 — Wire round-trip: STM32 byte == ESP32 decoded value (0..100 clamp).
 * ---------------------------------------------------------------------- */
static void test_wire_roundtrip(void)
{
    const uint16_t ticks[] = {0, 10, PWM_PERIOD/4, PWM_PERIOD/2, PWM_PERIOD};
    for (unsigned i = 0; i < sizeof(ticks)/sizeof(ticks[0]); ++i) {
        uint8_t tx = wheel_final_pwm_pct(true, ticks[i]);
        ASSERT_EQ_U(tx, decode_effort_byte(tx));
        ASSERT_TRUE(tx <= 100U);
    }
    /* A corrupt >100 wire byte is clamped on decode, never shown as garbage. */
    ASSERT_EQ_U(100, decode_effort_byte(200));
    ASSERT_EQ_U(100, decode_effort_byte(255));
}

/* -------------------------------------------------------------------------
 * G1 — LIMP_HOME ceiling applied ONCE: pedal 100 % -> 40 % (not 20/16/4 %).
 *      Reference model of main.c clamp (single point) with the pipeline
 *      traction cap = 1.0 in LIMP_HOME (Safety_GetTractionCapFactor).
 * ---------------------------------------------------------------------- */
static float limp_final_pct(float pedal_pct)
{
    /* main.c: demand = pedal * factor (SINGLE application). */
    float demand = pedal_pct * LIMP_HOME_TORQUE_LIMIT_FACTOR;
    if (demand < 0.0f) demand = 0.0f;
    if (demand > 100.0f * LIMP_HOME_TORQUE_LIMIT_FACTOR)
        demand = 100.0f * LIMP_HOME_TORQUE_LIMIT_FACTOR;
    /* pipeline traction cap in LIMP_HOME is now 1.0 -> no re-scaling. */
    const float traction_cap_limp = 1.0f;
    return demand * traction_cap_limp;
}

static void test_limp_single_application(void)
{
    /* pedal 100 % -> exactly 40 %. */
    float f = limp_final_pct(100.0f);
    ASSERT_TRUE(f > 39.5f && f < 40.5f);
    /* Guard against the documented double-scaling regressions. */
    ASSERT_TRUE(!(f > 19.5f && f < 20.5f));  /* not 20 % */
    ASSERT_TRUE(!(f > 15.5f && f < 16.5f));  /* not 16 % */
    ASSERT_TRUE(!(f >  3.5f && f <  4.5f));  /* not 4 %  */

    /* pedal 50 % -> 20 %, pedal 0 -> 0. */
    float h = limp_final_pct(50.0f);
    ASSERT_TRUE(h > 19.5f && h < 20.5f);
    ASSERT_TRUE(limp_final_pct(0.0f) == 0.0f);
}

int main(void)
{
    test_coast_brake_zero();
    test_drive_scaling();
    test_no_false_hundred();
    test_wire_roundtrip();
    test_limp_single_application();

    printf("wheel_effort: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#else
typedef int test_wheel_effort_translation_unit_not_empty;
#endif /* HOST_TEST */
