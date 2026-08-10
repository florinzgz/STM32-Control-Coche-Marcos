/**
  ****************************************************************************
  * @file    test_wheel_equality_test.c
  * @brief   Host unit tests for the wheel-equality / BTS7960 health
  *          self-test pure decision core (wheel_equality_test.c) — Hito 2,
  *          PR #445.
  *
  *          Compile with host GCC (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -Wall -Wextra -Werror \
  *                Core/Src/test_wheel_equality_test.c \
  *                Core/Src/wheel_equality_test.c -lm \
  *                -o /tmp/test_wheel_equality_test
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "wheel_equality_test.h"

static int tests_run = 0, tests_failed = 0;

#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ---- Synthetic wheel model -----------------------------------------------
 * BASE_K is the "healthy" normalized speed (pulses/s per pwm% per volt): a
 * real motor's raw pulses/s at a given duty cycle scales linearly with both
 * PWM % and supply voltage, so pulses_ps = K * pwm_pct * battery_v exactly
 * reproduces that physical relationship and lets the synthetic tests control
 * K (and any per-wheel/per-direction perturbation of it) directly. -------- */
#define BASE_K              2.0f     /* healthy normalized speed            */
#define BASE_CURRENT_BASE   1.0f     /* A, healthy current offset           */
#define BASE_CURRENT_SLOPE  0.05f    /* A per PWM-percent, healthy slope    */

typedef void (*WheelSampler)(uint8_t wheel, uint8_t pwm_pct, uint8_t dir,
                              float battery_v, float *pulses_ps, float *current_a);

static void sample_healthy(uint8_t wheel, uint8_t pwm_pct, uint8_t dir,
                            float battery_v, float *pulses_ps, float *current_a)
{
    (void)wheel; (void)dir;
    *pulses_ps  = BASE_K * (float)pwm_pct * battery_v;
    *current_a  = BASE_CURRENT_BASE + BASE_CURRENT_SLOPE * (float)pwm_pct;
}

/* ---- Baseline conditions -------------------------------------------------*/
static WheelEqConds base_conds(uint32_t now_ms)
{
    WheelEqConds c;
    memset(&c, 0, sizeof(c));
    c.now_ms            = now_ms;
    c.svc_armed         = true;
    c.svc_active        = true;
    c.steering_centered = true;
    c.steering_angle_deg = 0.0f;
    for (uint8_t i = 0; i < WHEQ_NUM_WHEELS; i++) {
        c.ackermann_diff[i]     = 1.0f;
        c.wheel_temp_present[i] = false;
        c.wheel_temp_c[i]       = 20.0f;
    }
    c.abs_or_tcs_active = false;
    c.battery_v         = 24.0f;
    return c;
}

/* Drive Update() every 50 ms (realistic cadence, well under the 100 ms
 * watchdog), feeding whatever WheelEqTest_GetActuation() says is being
 * driven THIS cycle through `sampler` into the per-wheel conds fields, until
 * a terminal-for-this-call state is reached or max_ticks is exhausted
 * (safety bound so a test bug cannot hang the suite). */
static WheelEqState_t drive_until(WheelEqTest *t, WheelEqConds *c,
                                   WheelSampler sampler, uint32_t max_ticks)
{
    for (uint32_t i = 0; i < max_ticks; i++) {
        WheelEqActuation_t a = WheelEqTest_GetActuation(t);
        for (uint8_t w = 0; w < WHEQ_NUM_WHEELS; w++) {
            if (a.wheel_mask & (uint8_t)(1U << w)) {
                sampler(w, a.pwm_pct, (uint8_t)a.direction, c->battery_v,
                        &c->wheel_pulses_ps[w], &c->wheel_current_a[w]);
            } else {
                c->wheel_pulses_ps[w] = 0.0f;
                c->wheel_current_a[w] = 0.0f;
            }
        }
        c->now_ms += 50U;
        WheelEqState_t st = WheelEqTest_Update(t, c);
        if (st == WHEQ_STATE_PHASE1_DONE || st == WHEQ_STATE_PHASE2_DONE ||
            st == WHEQ_STATE_ABORTED) {
            return st;
        }
    }
    return WheelEqTest_State(t);
}

/* Enough ticks to run the full 16-step Fase 1 to completion:
 * 16 steps * ((2000/50)+1 measure ticks + 6 deadtime ticks) with margin. */
#define PHASE1_MAX_TICKS  1200U
/* Enough ticks for Fase 2's single 3000 ms plateau, with margin. */
#define PHASE2_MAX_TICKS  120U

/* ======================================================================
 * 1. Four equal wheels -> PASS with ~0 deviation.
 * ====================================================================== */
static void test_four_equal_wheels_pass(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(1000U);

    CHECK(WheelEqTest_Begin(&t, &c));
    CHECK(WheelEqTest_State(&t) == WHEQ_STATE_PHASE1_RUNNING);

    WheelEqState_t st = drive_until(&t, &c, sample_healthy, PHASE1_MAX_TICKS);
    CHECK(st == WHEQ_STATE_PHASE1_DONE);

    for (uint8_t w = 0; w < WHEQ_NUM_WHEELS; w++) {
        const WheelEqWheelResult *r = WheelEqTest_Result(&t, w);
        CHECK(r->wheel_verdict == WHEQ_WHEEL_VERDICT_PASS);
        CHECK(r->cause == WHEQ_CAUSE_NONE);
        CHECK(fabsf(r->deviation_pct) < 0.01f);
        CHECK(r->halfbridge_verdict == WHEQ_HALFBRIDGE_PASS);
        CHECK(r->driver_verdict == WHEQ_DRIVER_SANO);
        CHECK(r->driver_reason_mask == 0U);
        CHECK(fabsf(r->normalized_speed - BASE_K) < 1.0e-3f);
    }
    CHECK(WheelEqTest_Phase1AllPass(&t));
}

/* ======================================================================
 * 2. One wheel at 80% of median with HIGH current -> FAIL + MECHANICAL.
 * ====================================================================== */
static uint8_t g_bad_wheel;
static float   g_bad_speed_factor;      /* e.g. 0.8 -> 80% of healthy speed */
static float   g_bad_current_factor;    /* multiplies the healthy current  */

static void sample_bad_wheel(uint8_t wheel, uint8_t pwm_pct, uint8_t dir,
                              float battery_v, float *pulses_ps, float *current_a)
{
    float k = (wheel == g_bad_wheel) ? (BASE_K * g_bad_speed_factor) : BASE_K;
    float base_cur = BASE_CURRENT_BASE + BASE_CURRENT_SLOPE * (float)pwm_pct;
    (void)dir;
    *pulses_ps = k * (float)pwm_pct * battery_v;
    *current_a = (wheel == g_bad_wheel) ? (base_cur * g_bad_current_factor) : base_cur;
}

static void test_wheel_80pct_high_current_fail_mechanical(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(2000U);
    g_bad_wheel = WHEQ_WHEEL_RR;
    g_bad_speed_factor = 0.8f;     /* slower -> 20% below median -> FAIL   */
    g_bad_current_factor = 2.0f;   /* consumes clearly more -> MECHANICAL  */

    CHECK(WheelEqTest_Begin(&t, &c));
    WheelEqState_t st = drive_until(&t, &c, sample_bad_wheel, PHASE1_MAX_TICKS);
    CHECK(st == WHEQ_STATE_PHASE1_DONE);

    const WheelEqWheelResult *r = WheelEqTest_Result(&t, WHEQ_WHEEL_RR);
    CHECK(r->wheel_verdict == WHEQ_WHEEL_VERDICT_FAIL);
    CHECK(r->deviation_pct > WHEQ_EQUALITY_FAIL_PCT);
    CHECK(r->cause == WHEQ_CAUSE_MECHANICAL);
    for (uint8_t w = 0; w < WHEQ_NUM_WHEELS; w++) {
        if (w == g_bad_wheel) continue;
        CHECK(WheelEqTest_Result(&t, w)->wheel_verdict == WHEQ_WHEEL_VERDICT_PASS);
    }
    CHECK(!WheelEqTest_Phase1AllPass(&t));
}

/* ======================================================================
 * 3. One wheel at 80% of median with LOW current -> FAIL + ELECTRICAL.
 * ====================================================================== */
static void test_wheel_80pct_low_current_fail_electrical(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(3000U);
    g_bad_wheel = WHEQ_WHEEL_FL;
    g_bad_speed_factor = 0.8f;
    g_bad_current_factor = 0.5f;   /* consumes clearly less -> ELECTRICAL  */

    CHECK(WheelEqTest_Begin(&t, &c));
    WheelEqState_t st = drive_until(&t, &c, sample_bad_wheel, PHASE1_MAX_TICKS);
    CHECK(st == WHEQ_STATE_PHASE1_DONE);

    const WheelEqWheelResult *r = WheelEqTest_Result(&t, WHEQ_WHEEL_FL);
    CHECK(r->wheel_verdict == WHEQ_WHEEL_VERDICT_FAIL);
    CHECK(r->cause == WHEQ_CAUSE_ELECTRICAL);
}

/* ======================================================================
 * 4/5. Forward/reverse half-bridge asymmetry -> FAIL_HALFBRIDGE (>20%) and
 *      WARN_HALFBRIDGE (5-20%).  Perturbation is antisymmetric around the
 *      50% step only (+X forward, -X reverse) so the 4-sample AVERAGE that
 *      feeds the equality verdict is mathematically unchanged (proving the
 *      two checks are independent), while the half-bridge check (which
 *      compares forward vs reverse directly) sees exactly 2*X*100 percent.
 * ====================================================================== */
static uint8_t g_asym_wheel;
static float   g_asym_x;

static void sample_halfbridge_asym(uint8_t wheel, uint8_t pwm_pct, uint8_t dir,
                                    float battery_v, float *pulses_ps, float *current_a)
{
    float k = BASE_K;
    if (wheel == g_asym_wheel && pwm_pct == WHEQ_PWM_LEVEL_50_PCT) {
        k = BASE_K * ((dir == WHEQ_DIR_FORWARD) ? (1.0f + g_asym_x) : (1.0f - g_asym_x));
    }
    *pulses_ps = k * (float)pwm_pct * battery_v;
    /* Current kept symmetric so only the SPEED asymmetry drives the verdict
     * in this test (current-driven asymmetry is not separately required by
     * the mandatory test list, and mixing both would make the exact
     * threshold-crossing math ambiguous). */
    *current_a = BASE_CURRENT_BASE + BASE_CURRENT_SLOPE * (float)pwm_pct;
}

static void test_halfbridge_asymmetry_fail(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(4000U);
    g_asym_wheel = WHEQ_WHEEL_RL;
    g_asym_x = 0.15f;   /* -> asym_pct = 2*0.15*100 = 30% > 20% FAIL       */

    CHECK(WheelEqTest_Begin(&t, &c));
    WheelEqState_t st = drive_until(&t, &c, sample_halfbridge_asym, PHASE1_MAX_TICKS);
    CHECK(st == WHEQ_STATE_PHASE1_DONE);

    const WheelEqWheelResult *r = WheelEqTest_Result(&t, WHEQ_WHEEL_RL);
    CHECK(r->halfbridge_asym_pct > WHEQ_HALFBRIDGE_FAIL_PCT);
    CHECK(r->halfbridge_verdict == WHEQ_HALFBRIDGE_FAIL);
    CHECK(r->driver_verdict == WHEQ_DRIVER_DEGRADADO);
    CHECK((r->driver_reason_mask & WHEQ_DRIVER_REASON_HALFBRIDGE) != 0U);
    /* Overall equality verdict for this wheel must stay PASS: the
     * antisymmetric perturbation cancels out of the 4-sample average. */
    CHECK(r->wheel_verdict == WHEQ_WHEEL_VERDICT_PASS);
    CHECK(!WheelEqTest_Phase1AllPass(&t));  /* halfbridge FAIL still blocks Fase 2 */
}

static void test_halfbridge_asymmetry_warn(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(5000U);
    g_asym_wheel = WHEQ_WHEEL_FR;
    g_asym_x = 0.06f;   /* -> asym_pct = 12% : within (5,20] -> WARN       */

    CHECK(WheelEqTest_Begin(&t, &c));
    WheelEqState_t st = drive_until(&t, &c, sample_halfbridge_asym, PHASE1_MAX_TICKS);
    CHECK(st == WHEQ_STATE_PHASE1_DONE);

    const WheelEqWheelResult *r = WheelEqTest_Result(&t, WHEQ_WHEEL_FR);
    CHECK(r->halfbridge_asym_pct > WHEQ_HALFBRIDGE_WARN_PCT);
    CHECK(r->halfbridge_asym_pct <= WHEQ_HALFBRIDGE_FAIL_PCT);
    CHECK(r->halfbridge_verdict == WHEQ_HALFBRIDGE_WARN);
    CHECK(r->driver_verdict == WHEQ_DRIVER_SOSPECHOSO);
    CHECK((r->driver_reason_mask & WHEQ_DRIVER_REASON_HALFBRIDGE) != 0U);
    /* WARN (not FAIL) must still allow Fase 2 to be eligible. */
    CHECK(WheelEqTest_Phase1AllPass(&t));
}

/* ======================================================================
 * 6. Anomalous I/PWM slope on one channel -> driver SOSPECHOSO/DEGRADADO,
 *    with speed and half-bridge held healthy so the slope criterion is
 *    isolated as the sole contributor (reason mask proves it).
 * ====================================================================== */
static uint8_t g_slope_wheel;
static float   g_slope_current_25, g_slope_current_50;

static void sample_slope_anomaly(uint8_t wheel, uint8_t pwm_pct, uint8_t dir,
                                  float battery_v, float *pulses_ps, float *current_a)
{
    (void)dir;
    *pulses_ps = BASE_K * (float)pwm_pct * battery_v;  /* speed always healthy */
    if (wheel == g_slope_wheel) {
        *current_a = (pwm_pct == WHEQ_PWM_LEVEL_25_PCT) ? g_slope_current_25
                                                          : g_slope_current_50;
    } else {
        *current_a = BASE_CURRENT_BASE + BASE_CURRENT_SLOPE * (float)pwm_pct;
    }
}

static void test_slope_anomaly_driver_degraded(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(6000U);
    g_slope_wheel = WHEQ_WHEEL_FR;
    g_slope_current_25 = 2.0f;
    g_slope_current_50 = 10.0f;   /* slope = 8/25 = 0.32 A/pct vs healthy 0.05 */

    CHECK(WheelEqTest_Begin(&t, &c));
    WheelEqState_t st = drive_until(&t, &c, sample_slope_anomaly, PHASE1_MAX_TICKS);
    CHECK(st == WHEQ_STATE_PHASE1_DONE);

    const WheelEqWheelResult *r = WheelEqTest_Result(&t, WHEQ_WHEEL_FR);
    CHECK(r->wheel_verdict == WHEQ_WHEEL_VERDICT_PASS);       /* speed unaffected */
    CHECK(r->halfbridge_verdict == WHEQ_HALFBRIDGE_PASS);     /* symmetric fwd/rev */
    CHECK(r->driver_verdict == WHEQ_DRIVER_DEGRADADO);
    CHECK((r->driver_reason_mask & WHEQ_DRIVER_REASON_SLOPE) != 0U);
    CHECK((r->driver_reason_mask & WHEQ_DRIVER_REASON_HALFBRIDGE) == 0U);
}

static void test_slope_anomaly_driver_suspicious(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(6500U);
    g_slope_wheel = WHEQ_WHEEL_RR;
    /* healthy slope = 0.05 A/pct; pick a slope ~7% above the other three's
     * median (0.05) => WARN band (5,15], not FAIL. */
    g_slope_current_25 = 2.25f;
    g_slope_current_50 = 3.59f;   /* slope = 1.34/25 = 0.0536 -> +7.2% dev  */

    CHECK(WheelEqTest_Begin(&t, &c));
    WheelEqState_t st = drive_until(&t, &c, sample_slope_anomaly, PHASE1_MAX_TICKS);
    CHECK(st == WHEQ_STATE_PHASE1_DONE);

    const WheelEqWheelResult *r = WheelEqTest_Result(&t, WHEQ_WHEEL_RR);
    CHECK(r->driver_verdict == WHEQ_DRIVER_SOSPECHOSO);
    CHECK((r->driver_reason_mask & WHEQ_DRIVER_REASON_SLOPE) != 0U);
}

/* ======================================================================
 * 7. Steering outside the centre deadband -> the test does not start.
 * ====================================================================== */
static void test_steering_outside_deadband_blocks_start(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(7000U);
    c.steering_centered  = false;
    c.steering_angle_deg = 12.5f;

    CHECK(!WheelEqTest_Begin(&t, &c));
    CHECK(WheelEqTest_State(&t) == WHEQ_STATE_BLOCKED_DEADBAND);
    CHECK(fabsf(t.steering_angle_at_block_deg - 12.5f) < 1.0e-6f);
    CHECK(!WheelEqTest_Active(&t));

    /* Update() must be a no-op while blocked (nothing to drive). */
    WheelEqActuation_t a = WheelEqTest_GetActuation(&t);
    CHECK(a.wheel_mask == 0U);
}

/* ======================================================================
 * 8. Ackermann differential != 1.000 with the wheel straight ->
 *    FAIL_ACKERMANN_OFFSET, test blocked from starting.
 * ====================================================================== */
static void test_ackermann_offset_blocks_start(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(8000U);
    c.ackermann_diff[WHEQ_WHEEL_RL] = 1.02f;  /* offending wheel            */

    CHECK(!WheelEqTest_Begin(&t, &c));
    CHECK(WheelEqTest_State(&t) == WHEQ_STATE_BLOCKED_ACKERMANN);
    CHECK(!WheelEqTest_Active(&t));
    CHECK(WheelEqTest_Result(&t, WHEQ_WHEEL_RL)->wheel_verdict ==
          WHEQ_WHEEL_VERDICT_FAIL_ACKERMANN_OFFSET);
    for (uint8_t w = 0; w < WHEQ_NUM_WHEELS; w++) {
        if (w == WHEQ_WHEEL_RL) continue;
        CHECK(WheelEqTest_Result(&t, w)->wheel_verdict == WHEQ_WHEEL_VERDICT_PENDING);
    }
}

/* ======================================================================
 * 9. The measurement must NEVER be routed through
 *    TractionOutput_Resolve4x4()/TractionOutput_Resolve4x2Rear(): this test
 *    demonstrates WHY the explicit bypass is mandatory (equalising a
 *    single-active-wheel plan to the group's minimum silently zeroes the
 *    measured wheel out) and asserts the module's own contract never
 *    applies that reduction — GetActuation() always returns the raw,
 *    un-equalised PWM for the active wheel.
 * ====================================================================== */
static void test_bypasses_traction_output_equalization(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(9000U);
    CHECK(WheelEqTest_Begin(&t, &c));

    /* Step 0: wheel FL commanded at 25%, all others coasting. */
    WheelEqActuation_t a = WheelEqTest_GetActuation(&t);
    CHECK(a.wheel_mask == (1U << WHEQ_WHEEL_FL));
    CHECK(a.pwm_pct == WHEQ_PWM_LEVEL_25_PCT);

    /* Build the exact 4-wheel plan a naive (bypass-less) actuator would
     * hand to TractionOutput_Resolve4x4(): the masked wheel DRIVE at
     * a.pwm_pct, every other wheel DRIVE at 0 (the natural way to express
     * "not under test" if one forgot the bypass and reused the normal
     * traction pipeline, which always leaves wheels in DRIVE mode). */
    uint8_t  mode[TRACTION_OUTPUT_WHEEL_COUNT];
    int8_t   dir[TRACTION_OUTPUT_WHEEL_COUNT];
    uint16_t pwm[TRACTION_OUTPUT_WHEEL_COUNT];
    for (uint8_t w = 0; w < TRACTION_OUTPUT_WHEEL_COUNT; w++) {
        mode[w] = TRACTION_OUTPUT_MODE_DRIVE;
        dir[w]  = 1;
        pwm[w]  = (a.wheel_mask & (1U << w)) ? a.pwm_pct : 0U;
    }
    float wheel_scale[TRACTION_OUTPUT_WHEEL_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
    TractionOutputPlan resolved;
    TractionOutput_Resolve4x4(mode, dir, pwm, 0.0f, 2.0f, wheel_scale, &resolved);

    /* Proof of the failure mode the spec warns about: equalising to the
     * group minimum (0, since 3 wheels are commanded to 0%) would silently
     * erase the measurement -> ANULARÍAN LA MEDIDA. */
    CHECK(resolved.pwm[TRACTION_OUTPUT_FL] == 0U);
    CHECK(resolved.pwm[TRACTION_OUTPUT_FL] != a.pwm_pct);

    /* This module's own actuation contract must NOT be this value: it must
     * keep reporting the raw 25 %, proving wheel_equality_test.c never
     * pipes its actuation through the equalisation policy. */
    CHECK(a.pwm_pct == WHEQ_PWM_LEVEL_25_PCT);

    /* Structural guarantee (belt-and-braces): the pure decision core's own
     * translation unit must never reference the equalisation entry points
     * it is forbidden from calling. Grepping the actual shipped source (not
     * a copy) means this check rots the instant someone reintroduces the
     * call. */
    FILE *f = fopen("Core/Src/wheel_equality_test.c", "r");
    CHECK(f != NULL);
    if (f != NULL) {
        char buf[8192];
        size_t n = fread(buf, 1, sizeof(buf) - 1, f);
        buf[n] = '\0';
        fclose(f);
        CHECK(strstr(buf, "TractionOutput_Resolve4x4") == NULL);
        CHECK(strstr(buf, "TractionOutput_Resolve4x2Rear") == NULL);
    }
}

/* ======================================================================
 * 10. Voltage normalization: two runs with different (but each internally
 *     constant) battery voltages and the SAME real wheel behaviour (raw
 *     pulses/s scaling with both PWM% and battery_v, as a real DC motor
 *     does) must produce the SAME normalized_speed.
 * ====================================================================== */
static void test_voltage_normalization_equivalence(void)
{
    WheelEqTest t24, t20;
    WheelEqTest_Init(&t24);
    WheelEqTest_Init(&t20);

    WheelEqConds c24 = base_conds(10000U);
    c24.battery_v = 24.0f;
    CHECK(WheelEqTest_Begin(&t24, &c24));
    WheelEqState_t st24 = drive_until(&t24, &c24, sample_healthy, PHASE1_MAX_TICKS);
    CHECK(st24 == WHEQ_STATE_PHASE1_DONE);

    WheelEqConds c20 = base_conds(20000U);
    c20.battery_v = 20.0f;    /* battery sagged between test runs           */
    CHECK(WheelEqTest_Begin(&t20, &c20));
    WheelEqState_t st20 = drive_until(&t20, &c20, sample_healthy, PHASE1_MAX_TICKS);
    CHECK(st20 == WHEQ_STATE_PHASE1_DONE);

    for (uint8_t w = 0; w < WHEQ_NUM_WHEELS; w++) {
        const WheelEqWheelResult *r24 = WheelEqTest_Result(&t24, w);
        const WheelEqWheelResult *r20 = WheelEqTest_Result(&t20, w);
        /* Raw pulses/s DIFFER between the two runs (proportional to V)... */
        CHECK(r24->pulses_ps_50 > r20->pulses_ps_50 + 1.0f);
        /* ...but the NORMALIZED speed must match. */
        CHECK(fabsf(r24->normalized_speed - r20->normalized_speed) < 1.0e-3f);
        CHECK(r24->wheel_verdict == WHEQ_WHEEL_VERDICT_PASS);
        CHECK(r20->wheel_verdict == WHEQ_WHEEL_VERDICT_PASS);
    }
}

/* ======================================================================
 * 11. Fase 2 must not start if Fase 1 finished with any FAIL.
 * ====================================================================== */
static void test_phase2_blocked_if_phase1_has_fail(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(11000U);
    g_bad_wheel = WHEQ_WHEEL_RR;
    g_bad_speed_factor = 0.8f;
    g_bad_current_factor = 2.0f;

    CHECK(WheelEqTest_Begin(&t, &c));
    WheelEqState_t st = drive_until(&t, &c, sample_bad_wheel, PHASE1_MAX_TICKS);
    CHECK(st == WHEQ_STATE_PHASE1_DONE);
    CHECK(!WheelEqTest_Phase1AllPass(&t));

    bool started = WheelEqTest_BeginPhase2(&t, &c);
    CHECK(!started);
    CHECK(WheelEqTest_Reason(&t) == WHEQ_REASON_PHASE1_NOT_CLEAN);
    CHECK(WheelEqTest_State(&t) == WHEQ_STATE_PHASE1_DONE);  /* unchanged    */
    CHECK(!t.phase2_ran);
}

/* And the positive counterpart: Fase 2 DOES start (and complete) when
 * Fase 1 was fully clean, and only measures — it does not recompute the
 * formal Fase-1 verdicts. */
static void test_phase2_runs_when_phase1_clean(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(12000U);

    CHECK(WheelEqTest_Begin(&t, &c));
    WheelEqState_t st = drive_until(&t, &c, sample_healthy, PHASE1_MAX_TICKS);
    CHECK(st == WHEQ_STATE_PHASE1_DONE);
    CHECK(WheelEqTest_Phase1AllPass(&t));

    CHECK(WheelEqTest_BeginPhase2(&t, &c));
    CHECK(WheelEqTest_State(&t) == WHEQ_STATE_PHASE2_RUNNING);

    WheelEqActuation_t a = WheelEqTest_GetActuation(&t);
    CHECK(a.wheel_mask == 0x0FU);   /* all four wheels simultaneously       */
    CHECK(a.pwm_pct == WHEQ_PHASE2_PWM_CEILING_PCT);

    st = drive_until(&t, &c, sample_healthy, PHASE2_MAX_TICKS);
    CHECK(st == WHEQ_STATE_PHASE2_DONE);
    CHECK(t.phase2_ran);
    for (uint8_t w = 0; w < WHEQ_NUM_WHEELS; w++) {
        CHECK(t.phase2_pulses_ps[w] > 0.0f);
    }
}

/* ======================================================================
 * 12. Envelope abort: the underlying Hito 1 session going inactive mid-test
 *     must abort Hito 2 immediately (WHEQ_REASON_ENVELOPE_ABORT), and the
 *     actuation must drop to zero (coast) on the very next tick.
 * ====================================================================== */
static void test_envelope_abort_propagates(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(13000U);
    CHECK(WheelEqTest_Begin(&t, &c));

    /* Run a few healthy ticks, then yank the Hito 1 envelope. */
    for (int i = 0; i < 5; i++) {
        WheelEqActuation_t a = WheelEqTest_GetActuation(&t);
        sample_healthy(0, a.pwm_pct, (uint8_t)a.direction, c.battery_v,
                       &c.wheel_pulses_ps[0], &c.wheel_current_a[0]);
        c.now_ms += 50U;
        WheelEqTest_Update(&t, &c);
    }
    CHECK(WheelEqTest_State(&t) == WHEQ_STATE_PHASE1_RUNNING);

    c.svc_active = false;
    c.now_ms += 50U;
    WheelEqState_t st = WheelEqTest_Update(&t, &c);
    CHECK(st == WHEQ_STATE_ABORTED);
    CHECK(WheelEqTest_Reason(&t) == WHEQ_REASON_ENVELOPE_ABORT);

    WheelEqActuation_t a2 = WheelEqTest_GetActuation(&t);
    CHECK(a2.wheel_mask == 0U);
}

/* ======================================================================
 * 13. Watchdog: a stale gap since the last Update() forces an abort.
 * ====================================================================== */
static void test_watchdog_abort(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(14000U);
    CHECK(WheelEqTest_Begin(&t, &c));

    c.now_ms += (WHEQ_WATCHDOG_MAX_GAP_MS + 50U);  /* stale jump             */
    WheelEqState_t st = WheelEqTest_Update(&t, &c);
    CHECK(st == WHEQ_STATE_ABORTED);
    CHECK(WheelEqTest_Reason(&t) == WHEQ_REASON_WATCHDOG);
}

/* ======================================================================
 * 14. TCS/ABS interference: transient interference that clears within the
 *     retry bound must NOT abort (the step silently retries and completes);
 *     interference that persists past the bound must abort.
 * ====================================================================== */
static void sample_healthy_passthrough(uint8_t wheel, uint8_t pwm_pct, uint8_t dir,
                                        float battery_v, float *pulses_ps, float *current_a)
{
    sample_healthy(wheel, pwm_pct, dir, battery_v, pulses_ps, current_a);
}

static void test_tcs_abs_interference_transient_recovers(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(15000U);
    CHECK(WheelEqTest_Begin(&t, &c));

    /* Discard the startup window, then interfere for a couple of ticks
     * (well under WHEQ_STEP_MAX_RETRIES) before letting it clear. */
    for (int i = 0; i < 12; i++) {   /* 600 ms: past the 500 ms discard     */
        WheelEqActuation_t a = WheelEqTest_GetActuation(&t);
        sample_healthy_passthrough(0, a.pwm_pct, (uint8_t)a.direction, c.battery_v,
                                   &c.wheel_pulses_ps[0], &c.wheel_current_a[0]);
        c.abs_or_tcs_active = (i == 6 || i == 7);  /* 2 interfering ticks   */
        c.now_ms += 50U;
        WheelEqTest_Update(&t, &c);
        CHECK(WheelEqTest_State(&t) != WHEQ_STATE_ABORTED);
    }
    c.abs_or_tcs_active = false;

    WheelEqState_t st = drive_until(&t, &c, sample_healthy, PHASE1_MAX_TICKS);
    CHECK(st == WHEQ_STATE_PHASE1_DONE);
    CHECK(WheelEqTest_Result(&t, 0)->wheel_verdict == WHEQ_WHEEL_VERDICT_PASS);
}

static void test_tcs_abs_interference_persistent_aborts(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(16000U);
    CHECK(WheelEqTest_Begin(&t, &c));

    WheelEqState_t st = WHEQ_STATE_PHASE1_RUNNING;
    for (uint32_t i = 0; i < 200U; i++) {
        WheelEqActuation_t a = WheelEqTest_GetActuation(&t);
        sample_healthy_passthrough(0, a.pwm_pct, (uint8_t)a.direction, c.battery_v,
                                   &c.wheel_pulses_ps[0], &c.wheel_current_a[0]);
        c.abs_or_tcs_active = true;  /* never clears                       */
        c.now_ms += 50U;
        st = WheelEqTest_Update(&t, &c);
        if (st == WHEQ_STATE_ABORTED) break;
    }
    CHECK(st == WHEQ_STATE_ABORTED);
    CHECK(WheelEqTest_Reason(&t) == WHEQ_REASON_TCS_ABS_INTERFERENCE);
}

/* ======================================================================
 * 15. Miscellaneous guards: already-active Begin() rejected; Abort() is
 *     idempotent from IDLE; NULL-safety.
 * ====================================================================== */
static void test_begin_rejected_while_already_active(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(17000U);
    CHECK(WheelEqTest_Begin(&t, &c));
    CHECK(!WheelEqTest_Begin(&t, &c));
    CHECK(WheelEqTest_Reason(&t) == WHEQ_REASON_ALREADY_ACTIVE);
}

static void test_null_safety(void)
{
    WheelEqTest_Init(NULL);
    CHECK(!WheelEqTest_Begin(NULL, NULL));
    WheelEqConds c = base_conds(1U);
    WheelEqTest t;
    WheelEqTest_Init(&t);
    CHECK(!WheelEqTest_Begin(&t, NULL));
    CHECK(WheelEqTest_Update(NULL, &c) == WHEQ_STATE_IDLE);
    CHECK(WheelEqTest_Update(&t, NULL) == WHEQ_STATE_IDLE);
    CHECK(!WheelEqTest_BeginPhase2(NULL, &c));
    CHECK(!WheelEqTest_BeginPhase2(&t, NULL));
    WheelEqTest_Abort(NULL, WHEQ_REASON_OPERATOR);   /* must not crash      */
    WheelEqActuation_t a = WheelEqTest_GetActuation(NULL);
    CHECK(a.wheel_mask == 0U);
    CHECK(!WheelEqTest_Phase1AllPass(NULL));
}

/* ======================================================================
 * 16. WheelEqTest_ActiveWheelMask() — the exact accessor can_handler.c's
 *     svcdiag_build_conds() uses to exempt the wheel(s) Hito 2 is actuating
 *     from the Hito 1 grounded-wheel-pulse abort guard (P2, PR #445).
 *
 *     This must prove two things:
 *       a) The mask exempts ONLY the wheel(s) genuinely being driven THIS
 *          cycle (a single bit in Fase 1, all four only while Fase 2 is
 *          actually running) and is 0 at every other time (idle, between
 *          steps, done, aborted) — never a blanket/global permission.
 *       b) Re-deriving can_handler.c's grounded_wheel_pulse loop verbatim
 *          with this mask still flags movement on a wheel that is NOT
 *          exempt — i.e. a real fault on an unrelated wheel still aborts
 *          the session, exactly as test_grounded_wheel_pulse_aborts() in
 *          test_service_diag_session.c already proves for Hito 1 alone.
 * ====================================================================== */

/* Bit-for-bit copy of the exemption loop in can_handler.c's
 * svcdiag_build_conds() (kept in sync manually — can_handler.c itself is
 * never compiled in a host test, so this is the only way to exercise the
 * exact boolean it produces). `active` mirrors ServiceDiagSession_ActiveChannel()
 * (SVCDIAG_CH_NONE == 5 when nothing from Hito 1 is stepping). */
static bool grounded_wheel_pulse_replica(const float speeds[4], int active,
                                          uint8_t wheeleq_exempt_mask)
{
    bool grounded = false;
    for (uint8_t i = 0; i < 4U; i++) {
        if ((int)i == active) continue;
        if ((wheeleq_exempt_mask & (1U << i)) != 0U) continue;
        if (fabsf(speeds[i]) >= 0.5f) { grounded = true; break; }
    }
    return grounded;
}

static void test_active_wheel_mask_exemption(void)
{
    WheelEqTest t;
    WheelEqTest_Init(&t);
    WheelEqConds c = base_conds(18000U);

    /* IDLE: no exemption at all. */
    CHECK(WheelEqTest_ActiveWheelMask(&t) == 0U);

    /* Fase 1, step 0 (FL): exactly one bit, matching GetActuation(). */
    CHECK(WheelEqTest_Begin(&t, &c));
    CHECK(WheelEqTest_State(&t) == WHEQ_STATE_PHASE1_RUNNING);
    WheelEqActuation_t a = WheelEqTest_GetActuation(&t);
    CHECK(a.wheel_mask == 0x01U);   /* WHEQ_WHEEL_FL bit                    */
    CHECK(WheelEqTest_ActiveWheelMask(&t) == a.wheel_mask);

    /* --- (a) exemption is per-wheel, never global -------------------- */
    /* Only FL (bit0, exempt) moving, active channel NONE (Hito1 idle):
     * must NOT be flagged as a grounded-wheel-pulse. */
    float speeds_fl_only[4] = { 5.0f, 0.0f, 0.0f, 0.0f };
    CHECK(!grounded_wheel_pulse_replica(speeds_fl_only, (int)SVCDIAG_CH_NONE,
                                         WheelEqTest_ActiveWheelMask(&t)));

    /* --- (b) a NON-exempt wheel (RL, bit2) moving must still trip the
     * guard -- proves the fix does not silence real grounded-wheel
     * faults on wheels Hito 2 is not driving. */
    float speeds_rl_faulted[4] = { 5.0f, 0.0f, 3.0f, 0.0f };
    CHECK(grounded_wheel_pulse_replica(speeds_rl_faulted, (int)SVCDIAG_CH_NONE,
                                        WheelEqTest_ActiveWheelMask(&t)));

    /* Run Fase 1 to completion: PHASE1_DONE must exempt nobody (matches
     * GetActuation()'s own zero-mask "coast" contract at rest). */
    WheelEqState_t st = drive_until(&t, &c, sample_healthy, PHASE1_MAX_TICKS);
    CHECK(st == WHEQ_STATE_PHASE1_DONE);
    CHECK(WheelEqTest_ActiveWheelMask(&t) == 0U);

    /* Fase 2: all four wheels simultaneously, only while running. */
    CHECK(WheelEqTest_BeginPhase2(&t, &c));
    CHECK(WheelEqTest_ActiveWheelMask(&t) == 0x0FU);
    float speeds_all_moving[4] = { 5.0f, 5.0f, 5.0f, 5.0f };
    CHECK(!grounded_wheel_pulse_replica(speeds_all_moving, (int)SVCDIAG_CH_NONE,
                                         WheelEqTest_ActiveWheelMask(&t)));

    st = drive_until(&t, &c, sample_healthy, PHASE2_MAX_TICKS);
    CHECK(st == WHEQ_STATE_PHASE2_DONE);
    CHECK(WheelEqTest_ActiveWheelMask(&t) == 0U);   /* done: no exemption   */

    /* Aborted: no exemption either. */
    WheelEqTest_Abort(&t, WHEQ_REASON_OPERATOR);
    CHECK(WheelEqTest_ActiveWheelMask(&t) == 0U);

    /* NULL-safety: must not crash, must return 0 (no exemption). */
    CHECK(WheelEqTest_ActiveWheelMask(NULL) == 0U);
}

/* ---- Text accessors: every enum value must resolve to a non-"?" label. --*/
static void test_text_accessors_cover_every_enum_value(void)
{
    CHECK(strcmp(WheelEqTest_StateText((WheelEqState_t)999), "?") == 0);
    CHECK(strcmp(WheelEqTest_StateText(WHEQ_STATE_PHASE2_DONE), "?") != 0);
    CHECK(strcmp(WheelEqTest_ReasonText(WHEQ_REASON_PHASE1_NOT_CLEAN), "?") != 0);
    CHECK(strcmp(WheelEqTest_WheelVerdictText(WHEQ_WHEEL_VERDICT_FAIL_ACKERMANN_OFFSET), "?") != 0);
    CHECK(strcmp(WheelEqTest_CauseText(WHEQ_CAUSE_OTHERS_BRAKED), "?") != 0);
    CHECK(strcmp(WheelEqTest_HalfBridgeVerdictText(WHEQ_HALFBRIDGE_FAIL), "?") != 0);
    CHECK(strcmp(WheelEqTest_DriverVerdictText(WHEQ_DRIVER_DEGRADADO), "?") != 0);
}

int main(void)
{
    test_four_equal_wheels_pass();
    test_wheel_80pct_high_current_fail_mechanical();
    test_wheel_80pct_low_current_fail_electrical();
    test_halfbridge_asymmetry_fail();
    test_halfbridge_asymmetry_warn();
    test_slope_anomaly_driver_degraded();
    test_slope_anomaly_driver_suspicious();
    test_steering_outside_deadband_blocks_start();
    test_ackermann_offset_blocks_start();
    test_bypasses_traction_output_equalization();
    test_voltage_normalization_equivalence();
    test_phase2_blocked_if_phase1_has_fail();
    test_phase2_runs_when_phase1_clean();
    test_envelope_abort_propagates();
    test_watchdog_abort();
    test_tcs_abs_interference_transient_recovers();
    test_tcs_abs_interference_persistent_aborts();
    test_begin_rejected_while_already_active();
    test_null_safety();
    test_active_wheel_mask_exemption();
    test_text_accessors_cover_every_enum_value();

    printf("wheel_equality_test: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
#else
int main(void) { return 0; }
#endif
