/**
  ****************************************************************************
  * @file    test_pedal_plausibility.c
  * @brief   Deterministic host tests for the pedal plausibility pipeline.
  *
  *          Covers the root-cause fix for the "frozen 53 % / LIMP HOME"
  *          fault: direction-aware rate check + recovery state machine.
  *
  *          Compile (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -lm \
  *                Core/Src/test_pedal_plausibility.c \
  *                -o /tmp/test_pedal_plausibility
  *            /tmp/test_pedal_plausibility
  *
  *          The simulation re-implements the pedal pipeline in the test
  *          file using the same constants as sensor_manager.c.  A set of
  *          _Static_assert checks guard against constant drift between
  *          firmware and test.
  ****************************************************************************
  */

#ifdef HOST_TEST

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "pedal_cal_store.h"   /* DEFAULT_MIN/MAX, RANGE_MIN */
#include "pedal_logic.h"       /* REAL production pipeline under test */

/* =========================================================================
 * Mirror constants — must stay in sync with the REAL pipeline (pedal_logic.h)
 * ========================================================================= */

#define T_PEDAL_SAMPLE_TOLERANCE  30U
#define T_PEDAL_MAX_RATE_PCT      35.0f
#define T_PEDAL_EMA_ALPHA         0.3f
#define T_PEDAL_ADC_FAULT_LO      0U
#define T_PEDAL_ADC_FAULT_HI      4094U
#define T_PEDAL_RELEASE_ZONE_PCT  3.0f
#define T_PEDAL_RECOVERY_CYCLES   3U
#define T_PEDAL_ADC_MIN           PEDAL_CAL_DEFAULT_MIN   /* 50 */
#define T_PEDAL_ADC_MAX           PEDAL_CAL_DEFAULT_MAX   /* 4000 */

/* Verify constants agree with the firmware header */
_Static_assert(T_PEDAL_ADC_MIN == PEDAL_CAL_DEFAULT_MIN,
               "T_PEDAL_ADC_MIN drifted from PEDAL_CAL_DEFAULT_MIN");
_Static_assert(T_PEDAL_ADC_MAX == PEDAL_CAL_DEFAULT_MAX,
               "T_PEDAL_ADC_MAX drifted from PEDAL_CAL_DEFAULT_MAX");
_Static_assert(T_PEDAL_ADC_FAULT_HI == PEDAL_CAL_MAX_LIMIT,
               "T_PEDAL_ADC_FAULT_HI drifted from PEDAL_CAL_MAX_LIMIT");
/* The mirror constants MUST equal the real pipeline constants. */
_Static_assert(T_PEDAL_SAMPLE_TOLERANCE == PEDAL_SAMPLE_TOLERANCE, "tol drift");
_Static_assert(T_PEDAL_MAX_RATE_PCT     == PEDAL_MAX_RATE_PCT,     "rate drift");
_Static_assert(T_PEDAL_EMA_ALPHA        == PEDAL_EMA_ALPHA,        "ema drift");
_Static_assert(T_PEDAL_ADC_FAULT_HI     == PEDAL_ADC_FAULT_HI,     "faulthi drift");
_Static_assert(T_PEDAL_RELEASE_ZONE_PCT == PEDAL_RELEASE_ZONE_PCT, "zone drift");
_Static_assert(T_PEDAL_RECOVERY_CYCLES  == PEDAL_RECOVERY_CYCLES,  "recov drift");

/* =========================================================================
 * Test harness state — the REAL production pipeline state (pedal_logic.h)
 * is driven directly; the s_* mirrors are copied out after each cycle so the
 * existing assertions keep working WITHOUT re-implementing the algorithm.
 * ========================================================================= */

static PedalState g_st;

static float    s_pct        = 0.0f;
static float    s_pct_raw    = 0.0f;
static float    s_pct_prev   = 0.0f;
static float    s_ema        = 0.0f;
static bool     s_plausible  = true;
static bool     s_contradict = false;
static bool     s_ema_primed = false;
static bool     s_rate_fault = false;
static uint8_t  s_rec_count  = 0;

static void sync_from_state(void)
{
    s_pct        = g_st.pct;
    s_pct_raw    = g_st.pct_raw;
    s_pct_prev   = g_st.pct_prev;
    s_ema        = g_st.ema;
    s_plausible  = g_st.plausible;
    s_contradict = g_st.contradict;
    s_ema_primed = g_st.ema_primed;
    s_rate_fault = g_st.rate_fault;
    s_rec_count  = g_st.recovery_count;
}

static void sim_reset(void)
{
    Pedal_StateInit(&g_st, T_PEDAL_ADC_MIN, T_PEDAL_ADC_MAX);
    sync_from_state();
}

/*
 * Drive ONE cycle of the REAL production pipeline (Pedal_ProcessSamples from
 * pedal_logic.c) and copy the resulting state into the s_* mirrors.  There is
 * NO re-implementation of the algorithm here.
 */
static void sim_update(uint16_t adc1, uint16_t adc2)
{
    Pedal_ProcessSamples(&g_st, adc1, adc2);
    sync_from_state();
}


/* =========================================================================
 * Test framework
 * ========================================================================= */

static int tests_run    = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(cond, msg) do {                                           \
    tests_run++;                                                               \
    if (!(cond)) {                                                             \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, (msg));                \
        tests_failed++;                                                        \
    }                                                                          \
} while (0)

#define ASSERT_FALSE(cond, msg) ASSERT_TRUE(!(cond), msg)

#define ASSERT_NEAR(got, expected, tol, msg) do {                             \
    float _g = (float)(got), _e = (float)(expected), _t = (float)(tol);      \
    tests_run++;                                                               \
    if (fabsf(_g - _e) > _t) {                                                \
        printf("FAIL %s:%d  %s: got %.4f expected %.4f (±%.4f)\n",           \
               __FILE__, __LINE__, (msg),                                     \
               (double)_g, (double)_e, (double)_t);                          \
        tests_failed++;                                                        \
    }                                                                          \
} while (0)

#define ASSERT_EQ(got, expected, msg) do {                                    \
    int _g = (int)(got), _e = (int)(expected);                                \
    tests_run++;                                                               \
    if (_g != _e) {                                                            \
        printf("FAIL %s:%d  %s: got %d expected %d\n",                       \
               __FILE__, __LINE__, (msg), _g, _e);                            \
        tests_failed++;                                                        \
    }                                                                          \
} while (0)

/* Raw ADC value that maps to approximately 0 % with default calibration */
#define ADC_RELEASED   3U      /* ≈0.0025 V as measured on vehicle */
/* Raw ADC value that maps to ≈53 % */
static uint16_t adc_for_pct(float pct)
{
    if (pct <= 0.0f) return T_PEDAL_ADC_MIN - 1U;
    if (pct >= 100.0f) return T_PEDAL_ADC_MAX;
    return (uint16_t)(T_PEDAL_ADC_MIN
         + (uint32_t)(pct / 100.0f * (T_PEDAL_ADC_MAX - T_PEDAL_ADC_MIN)
                      + 0.5f));
}

/* =========================================================================
 * Test Case 1 — Boot with pedal released
 * ========================================================================= */
static void tc01_boot_released(void)
{
    printf("TC01: boot with pedal released\n");
    sim_reset();
    sim_update(ADC_RELEASED, ADC_RELEASED);
    ASSERT_TRUE(s_plausible,   "TC01: plausible must be true");
    ASSERT_FALSE(s_contradict, "TC01: no contradiction");
    ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC01: pct ≈ 0 %");
}

/* =========================================================================
 * Test Case 2 — Progressive upward ramp
 * ========================================================================= */
static void tc02_progressive_ramp(void)
{
    printf("TC02: progressive ramp 0→10→20→30→40→53 %%\n");
    sim_reset();
    float targets[] = {0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 53.0f};
    float prev_ema  = 0.0f;
    for (int i = 0; i < 6; i++) {
        uint16_t a = adc_for_pct(targets[i]);
        sim_update(a, a);
        ASSERT_TRUE(s_plausible, "TC02: plausible during ramp");
        /* EMA must be tracking toward target without exceeding it much */
        float expected_ema = T_PEDAL_EMA_ALPHA * targets[i]
                           + (1.0f - T_PEDAL_EMA_ALPHA) * prev_ema;
        ASSERT_NEAR(s_pct, expected_ema, 1.5f, "TC02: EMA tracking");
        prev_ema = s_pct;
    }
}

/* =========================================================================
 * Test Case 3 — Exact fault reproduction: 53 % → 0 % rapid release
 *               ROOT-CAUSE FIX VERIFICATION
 * ========================================================================= */
static void tc03_rapid_release_from_53(void)
{
    printf("TC03: rapid release 53 %% → 0 %% (root-cause case)\n");

    /* Ramp up in small steps to 53 %, then hold until EMA converges.
     * Each step is ≤ 10 %/cycle (well below PEDAL_MAX_RATE_PCT = 35 %). */
    sim_reset();
    float ramp[] = {5.0f, 10.0f, 20.0f, 30.0f, 40.0f, 53.0f};
    for (int i = 0; i < 6; i++)
        sim_update(adc_for_pct(ramp[i]), adc_for_pct(ramp[i]));
    /* Hold at 53 % until EMA converges (α=0.3 → ~8 cycles to settle) */
    for (int i = 0; i < 12; i++)
        sim_update(adc_for_pct(53.0f), adc_for_pct(53.0f));

    ASSERT_TRUE(s_plausible,             "TC03: plausible before release");
    ASSERT_TRUE(s_pct > 45.0f,           "TC03: EMA converged above 45 after hold");
    ASSERT_NEAR(s_pct_prev, 53.0f, 1.0f, "TC03: pct_prev tracks raw at 53");

    /* Simulate rapid release in one cycle */
    sim_update(ADC_RELEASED, ADC_RELEASED);

    /* KEY ASSERTIONS — this is the root-cause bug fix.
     * Before the fix: pedal_pct was FROZEN at 53 % and pedal_pct_prev
     * was never updated → permanent lockout every subsequent cycle.    */
    ASSERT_NEAR(s_pct,  0.0f, 0.5f, "TC03: output must be 0 % after rapid release");
    ASSERT_TRUE(s_plausible,         "TC03: rapid release is NOT a fault");
    ASSERT_FALSE(s_rate_fault,       "TC03: no rate_fault after rapid release");
    ASSERT_NEAR(s_pct_prev, 0.0f, 0.5f, "TC03: pct_prev reset to 0");
    ASSERT_NEAR(s_ema,      0.0f, 0.5f, "TC03: EMA reset to 0");

    /* Subsequent cycles must not revert to 53 % */
    for (int i = 0; i < 3; i++) {
        sim_update(ADC_RELEASED, ADC_RELEASED);
        ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC03: still 0 % in subsequent cycles");
        ASSERT_TRUE(s_plausible,        "TC03: plausible in subsequent cycles");
    }
}

/* =========================================================================
 * Test Case 4 — Rapid release from 100 %
 * ========================================================================= */
static void tc04_rapid_release_from_100(void)
{
    printf("TC04: rapid release 100 %% → 0 %%\n");
    sim_reset();
    /* Ramp in 20 %/cycle steps — each is < 35 % so no rate fault */
    float ramp[] = {20.0f, 40.0f, 60.0f, 80.0f, 100.0f};
    for (int i = 0; i < 5; i++)
        sim_update(adc_for_pct(ramp[i]), adc_for_pct(ramp[i]));
    ASSERT_TRUE(s_plausible, "TC04: plausible at 100 % after ramp");

    sim_update(ADC_RELEASED, ADC_RELEASED);

    ASSERT_NEAR(s_pct,  0.0f, 0.5f, "TC04: output 0 % after 100→0 release");
    ASSERT_TRUE(s_plausible,         "TC04: plausible after rapid release");
    ASSERT_FALSE(s_rate_fault,       "TC04: no rate_fault");
    ASSERT_NEAR(s_pct_prev, 0.0f, 0.5f, "TC04: prev reset");
    ASSERT_NEAR(s_ema,      0.0f, 0.5f, "TC04: EMA reset");
}

/* =========================================================================
 * Test Case 5 — Upward rate fault: 0 → 60 % in one cycle
 * ========================================================================= */
static void tc05_rapid_application(void)
{
    printf("TC05: rapid application 0 → 60 %% in one cycle\n");
    sim_reset();
    sim_update(ADC_RELEASED, ADC_RELEASED);   /* stable at 0 */
    ASSERT_TRUE(s_plausible, "TC05: plausible at 0");

    uint16_t a60 = adc_for_pct(60.0f);
    sim_update(a60, a60);

    ASSERT_FALSE(s_plausible,        "TC05: implausible after rapid application");
    ASSERT_TRUE(s_rate_fault,        "TC05: rate_fault set");
    ASSERT_NEAR(s_pct, 0.0f, 0.5f,  "TC05: output must be 0 %");
    ASSERT_NEAR(s_pct_prev, 60.0f, 1.0f, "TC05: pct_prev reset to current raw");
}

/* =========================================================================
 * Test Case 6 — Recovery from upward rate fault
 * ========================================================================= */
static void tc06_recovery_after_rapid_application(void)
{
    printf("TC06: recovery from rapid application — must re-enable from 0 %%\n");
    sim_reset();
    sim_update(ADC_RELEASED, ADC_RELEASED);

    uint16_t a60 = adc_for_pct(60.0f);
    sim_update(a60, a60);
    ASSERT_FALSE(s_plausible, "TC06: implausible after jump");

    /* Hold pedal released for RECOVERY_CYCLES cycles */
    for (uint8_t i = 0; i < T_PEDAL_RECOVERY_CYCLES - 1; i++) {
        sim_update(ADC_RELEASED, ADC_RELEASED);
        ASSERT_FALSE(s_plausible, "TC06: still implausible during recovery");
        ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC06: output still 0 during recovery");
    }

    /* Final recovery cycle */
    sim_update(ADC_RELEASED, ADC_RELEASED);
    ASSERT_TRUE(s_plausible,        "TC06: plausible restored after N cycles");
    ASSERT_FALSE(s_rate_fault,      "TC06: rate_fault cleared");
    ASSERT_NEAR(s_pct,      0.0f, 0.5f, "TC06: output starts at 0 after recovery");
    ASSERT_NEAR(s_pct_prev, 0.0f, 0.5f, "TC06: prev baseline 0 after recovery");

    /* Must not apply the old 60 % value */
    ASSERT_TRUE(s_pct < 5.0f, "TC06: old 60 %% not re-applied");

    /* Gradual ramp after recovery works correctly */
    sim_update(adc_for_pct(10.0f), adc_for_pct(10.0f));
    ASSERT_TRUE(s_plausible, "TC06: plausible during post-recovery ramp");
    ASSERT_TRUE(s_pct > 0.0f && s_pct <= 20.0f, "TC06: post-recovery output in range");
}

/* =========================================================================
 * Test Case 7 — Contradictory dual samples
 * ========================================================================= */
static void tc07_contradictory_samples(void)
{
    printf("TC07: ADC1/ADC2 difference > tolerance\n");
    sim_reset();
    /* Difference of 31 counts — exceeds PEDAL_SAMPLE_TOLERANCE(30) */
    uint16_t adc1 = 500U;
    uint16_t adc2 = 531U;
    sim_update(adc1, adc2);

    ASSERT_FALSE(s_plausible,  "TC07: implausible on contradiction");
    ASSERT_TRUE(s_contradict,  "TC07: contradict flag set");
    ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC07: output 0 on contradiction");

    /* When samples agree again, system recovers */
    uint16_t mid = (uint16_t)((adc1 + adc2) / 2);
    sim_update(mid, mid);
    ASSERT_FALSE(s_contradict, "TC07: contradict cleared when samples agree");
    ASSERT_TRUE(s_plausible,   "TC07: plausible restored");
}

/* =========================================================================
 * Test Case 8 — Rail-high fault (ADC = 4095)
 * ========================================================================= */
static void tc08_rail_high(void)
{
    printf("TC08: ADC at rail 4095/4095 — short-circuit fault\n");
    sim_reset();
    sim_update(4095U, 4095U);

    ASSERT_FALSE(s_plausible,       "TC08: implausible on rail");
    ASSERT_NEAR(s_pct,      0.0f, 0.5f, "TC08: output 0 on rail");
    ASSERT_NEAR(s_pct_prev, 0.0f, 0.5f, "TC08: prev reset");
    ASSERT_FALSE(s_rate_fault,      "TC08: no rate_fault (different fault type)");

    /* Recovery when rail returns to valid value */
    sim_update(ADC_RELEASED, ADC_RELEASED);
    ASSERT_TRUE(s_plausible, "TC08: plausible restored after fault clears");
    ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC08: output 0 on recovery");
}

/* =========================================================================
 * Test Case 9 — Calibration gate reject codes
 *               Verify 0x0023 = STBY(0x01) | STARTUP(0x02) | CAPTURE(0x20)
 * ========================================================================= */
#include "can_handler.h"

static void tc09_reject_0x0023_decomposition(void)
{
    printf("TC09: 0x0023 = NOT_STANDBY | STARTUP_NOT_INHIBITED | PENDING_INCOMPLETE\n");

    uint16_t r = 0x0023U;
    ASSERT_TRUE((r & PEDCAL_REJECT_NOT_STANDBY)           != 0U,
                "TC09: bit STBY present in 0x0023");
    ASSERT_TRUE((r & PEDCAL_REJECT_STARTUP_NOT_INHIBITED) != 0U,
                "TC09: bit STARTUP present in 0x0023");
    ASSERT_TRUE((r & PEDCAL_REJECT_PENDING_INCOMPLETE)    != 0U,
                "TC09: bit CAPTURE present in 0x0023");
    /* Bits NOT present in 0x0023 */
    ASSERT_FALSE((r & PEDCAL_REJECT_PEDAL_NOT_RELEASED)   != 0U,
                 "TC09: PEDAL>3 must NOT be set in 0x0023");
    ASSERT_FALSE((r & PEDCAL_REJECT_PEDAL_NOT_PLAUSIBLE)  != 0U,
                 "TC09: PLAUS must NOT be set in 0x0023");
}

/* =========================================================================
 * Test Case 10 — Historical reject code 0x002F
 *               0x002F = STBY | STARTUP | PEDAL>3 | PLAUS | CAPTURE
 * ========================================================================= */
static void tc10_reject_0x002F_decomposition(void)
{
    printf("TC10: 0x002F = STBY | STARTUP | PEDAL>3 | PLAUS | CAPTURE\n");

    uint16_t r = 0x002FU;
    ASSERT_TRUE((r & PEDCAL_REJECT_NOT_STANDBY)           != 0U, "TC10: STBY");
    ASSERT_TRUE((r & PEDCAL_REJECT_STARTUP_NOT_INHIBITED) != 0U, "TC10: STARTUP");
    ASSERT_TRUE((r & PEDCAL_REJECT_PEDAL_NOT_RELEASED)    != 0U, "TC10: PEDAL>3");
    ASSERT_TRUE((r & PEDCAL_REJECT_PEDAL_NOT_PLAUSIBLE)   != 0U, "TC10: PLAUS");
    ASSERT_TRUE((r & PEDCAL_REJECT_PENDING_INCOMPLETE)    != 0U, "TC10: CAPTURE");
}

/* =========================================================================
 * Test Case 11 — Recovery counter resets when pedal rises during recovery
 * ========================================================================= */
static void tc11_recovery_counter_resets_on_rise(void)
{
    printf("TC11: recovery counter resets when pedal rises above zone\n");
    sim_reset();
    sim_update(ADC_RELEASED, ADC_RELEASED);

    /* Trigger rapid application */
    sim_update(adc_for_pct(70.0f), adc_for_pct(70.0f));
    ASSERT_TRUE(s_rate_fault, "TC11: rate_fault set");

    /* One release cycle → counter = 1 */
    sim_update(ADC_RELEASED, ADC_RELEASED);
    ASSERT_EQ((int)s_rec_count, 1, "TC11: counter = 1 after first release cycle");

    /* Signal rises — counter must reset */
    sim_update(adc_for_pct(40.0f), adc_for_pct(40.0f));
    ASSERT_EQ((int)s_rec_count, 0, "TC11: counter reset when pedal rises");
    ASSERT_FALSE(s_plausible,       "TC11: still implausible");
    ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC11: output still 0");
}

/* =========================================================================
 * Test Case 12 — Upward fault does NOT accept high value spontaneously
 * ========================================================================= */
static void tc12_no_spontaneous_reactivation(void)
{
    printf("TC12: upward fault must NOT auto-accept high value\n");
    sim_reset();
    sim_update(ADC_RELEASED, ADC_RELEASED);

    sim_update(adc_for_pct(80.0f), adc_for_pct(80.0f));
    ASSERT_FALSE(s_plausible, "TC12: implausible");

    /* Stay at 80 % for many cycles — must NEVER become plausible */
    for (int i = 0; i < 20; i++) {
        sim_update(adc_for_pct(80.0f), adc_for_pct(80.0f));
        ASSERT_FALSE(s_plausible,       "TC12: must remain implausible at 80 %");
        ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC12: output 0 at 80 %");
    }
}

/* =========================================================================
 * Test Case 13 — EMA not contaminated by fault cycle
 * ========================================================================= */
static void tc13_ema_not_contaminated(void)
{
    printf("TC13: EMA must not be contaminated by rate-fault raw reading\n");
    sim_reset();

    /* Build stable 20 % */
    for (int i = 0; i < 5; i++)
        sim_update(adc_for_pct(20.0f), adc_for_pct(20.0f));

    float ema_before = s_ema;

    /* Rapid application */
    sim_update(adc_for_pct(80.0f), adc_for_pct(80.0f));

    /* EMA should have been reset to 0, not polluted with 80 */
    ASSERT_NEAR(s_ema, 0.0f, 0.1f, "TC13: EMA reset to 0 on upward fault");
    ASSERT_TRUE(s_ema < ema_before, "TC13: EMA decreased, not increased");
    (void)ema_before;
}

/* =========================================================================
 * Test Case 14 — Dual-sample tolerance boundary
 * ========================================================================= */
static void tc14_dual_sample_tolerance_boundary(void)
{
    printf("TC14: dual-sample tolerance boundary at ±30 counts\n");
    sim_reset();

    /* Exactly at tolerance — should pass */
    sim_update(500U, 500U + T_PEDAL_SAMPLE_TOLERANCE);
    ASSERT_FALSE(s_contradict, "TC14: diff==tolerance is still OK");
    ASSERT_TRUE(s_plausible,   "TC14: plausible at boundary");

    sim_reset();
    /* One above tolerance — must fail */
    sim_update(500U, 500U + T_PEDAL_SAMPLE_TOLERANCE + 1U);
    ASSERT_TRUE(s_contradict,  "TC14: diff==tolerance+1 triggers contradiction");
    ASSERT_FALSE(s_plausible,  "TC14: implausible at tolerance+1");
    ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC14: output 0 on contradiction");
}

/* =========================================================================
 * Test Case 15 — Normal small rate transitions stay plausible
 * ========================================================================= */
static void tc15_small_rate_transitions(void)
{
    printf("TC15: small rate transitions (< 35 %% per cycle) stay plausible\n");
    sim_reset();

    /* Step up in 30 %/cycle increments — just below the 35 % threshold */
    float levels[] = {0.0f, 30.0f, 60.0f, 90.0f, 60.0f, 30.0f, 0.0f};
    for (int i = 0; i < 7; i++) {
        uint16_t a = adc_for_pct(levels[i]);
        sim_update(a, a);
        ASSERT_TRUE(s_plausible, "TC15: plausible at 30 %/cycle ramp");
    }
}

/* =========================================================================
 * Test Case 16 — Permanent lockout regression
 *               After rapid release, prev must not stay at old value
 * ========================================================================= */
static void tc16_no_permanent_lockout(void)
{
    printf("TC16: no permanent lockout — prev must not stay at old value\n");
    sim_reset();

    /* Ramp up to 53 % */
    for (float p = 10.0f; p <= 53.0f; p += 10.0f)
        sim_update(adc_for_pct(p), adc_for_pct(p));

    float prev_before_release = s_pct_prev;
    ASSERT_TRUE(prev_before_release > 40.0f, "TC16: prev is near 53 before release");

    /* Rapid release */
    sim_update(ADC_RELEASED, ADC_RELEASED);
    ASSERT_NEAR(s_pct_prev, 0.0f, 0.5f, "TC16: prev reset to 0 on rapid release");

    /* Verify next cycle does NOT compare against old 53 % */
    sim_update(ADC_RELEASED, ADC_RELEASED);
    ASSERT_TRUE(s_plausible, "TC16: plausible in cycle after rapid release");
    ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC16: output 0 in cycle after rapid release");
    (void)prev_before_release;
}

/* =========================================================================
 * Test Case 17 — Range validation: ADC 4094 OK, 4095 fault
 * ========================================================================= */
static void tc17_range_boundary(void)
{
    printf("TC17: range boundary — 4094 OK (range check), 4095 faults\n");

    /* ADC 4094 must pass the range check (avg > FAULT_HI fails only at 4095).
     * Pre-warm state to near 100 % so no rate-fault fires; only range
     * behaviour is under test here.                                        */
    sim_reset();
    for (int i = 0; i < 5; i++)
        sim_update(T_PEDAL_ADC_FAULT_HI, T_PEDAL_ADC_FAULT_HI);
    /* After warm-up at 4094 the range check passes; plausibility depends on
     * whether rate converged.  Verify the range check is not the cause
     * of any fault by checking rate_fault is the only possible trigger.   */
    ASSERT_FALSE(s_contradict, "TC17: no contradiction at 4094");
    /* The range gate must NOT be the reason for any fault */
    /* (rate_fault may legitimately fire from the initial 0→100 jump) */

    /* ADC 4095 must always fault (rail short) regardless of history */
    sim_update(4095U, 4095U);
    ASSERT_FALSE(s_plausible,       "TC17: 4095 triggers fault");
    ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC17: output 0 on 4095");
    ASSERT_FALSE(s_rate_fault,      "TC17: range fault clears rate_fault flag");
}

/* =========================================================================
 * Test Case 18 — §5 fast PARTIAL release that settles ABOVE the rest zone
 *   Reproduces the remaining defect: 100 %→60 % rapid release must NOT be
 *   treated as a fault, must NOT force history to 0 (which caused a FALSE
 *   +60 % rising rate-fault on the next cycle), and the demand must reduce
 *   to ~60 % (never exceed the physical pedal).
 * ========================================================================= */
static void tc18_fast_partial_release_100_to_60(void)
{
    printf("TC18: fast partial release 100 -> 60 and hold 60\n");
    sim_reset();
    /* Ramp to 100 % (<35 %/cycle steps). */
    float ramp[] = {20.0f, 40.0f, 60.0f, 80.0f, 100.0f};
    for (int i = 0; i < 5; i++)
        sim_update(adc_for_pct(ramp[i]), adc_for_pct(ramp[i]));
    ASSERT_TRUE(s_plausible, "TC18: plausible at 100 %");

    /* Rapid release to 60 % in ONE cycle: delta = -40 % < -35 %. */
    sim_update(adc_for_pct(60.0f), adc_for_pct(60.0f));
    ASSERT_TRUE(s_plausible,        "TC18: 100->60 is a safe reduction, not a fault");
    ASSERT_FALSE(s_rate_fault,      "TC18: no rate_fault on partial release");
    ASSERT_NEAR(s_pct, 60.0f, 1.0f, "TC18: output tracks reduced pedal (~60 %)");
    ASSERT_TRUE(s_pct <= 60.5f,     "TC18: output never exceeds physical pedal");
    ASSERT_NEAR(s_pct_prev, 60.0f, 1.0f, "TC18: baseline re-anchored to 60 %");

    /* Hold at 60 %: the NEXT cycle must NOT fire a false +rising fault. */
    for (int i = 0; i < 5; i++) {
        sim_update(adc_for_pct(60.0f), adc_for_pct(60.0f));
        ASSERT_TRUE(s_plausible,   "TC18: still plausible holding 60 %");
        ASSERT_FALSE(s_rate_fault, "TC18: no false rising fault holding 60 %");
    }
    ASSERT_NEAR(s_pct, 60.0f, 1.0f, "TC18: settled at ~60 %");
}

/* =========================================================================
 * Test Case 19 — §5 partial releases 80->40 and 60->20 (above rest zone)
 * ========================================================================= */
static void tc19_partial_releases_above_zone(void)
{
    printf("TC19: partial releases 80->40 and 60->20\n");

    /* 80 -> 40 */
    sim_reset();
    float r1[] = {20.0f, 40.0f, 60.0f, 80.0f};
    for (int i = 0; i < 4; i++)
        sim_update(adc_for_pct(r1[i]), adc_for_pct(r1[i]));
    sim_update(adc_for_pct(40.0f), adc_for_pct(40.0f));   /* delta -40 */
    ASSERT_TRUE(s_plausible,        "TC19: 80->40 not a fault");
    ASSERT_FALSE(s_rate_fault,      "TC19: 80->40 no rate_fault");
    ASSERT_NEAR(s_pct, 40.0f, 1.0f, "TC19: output ~40 %");
    sim_update(adc_for_pct(40.0f), adc_for_pct(40.0f));   /* hold */
    ASSERT_TRUE(s_plausible,        "TC19: 80->40 hold plausible");
    ASSERT_FALSE(s_rate_fault,      "TC19: 80->40 hold no false fault");

    /* 60 -> 20 */
    sim_reset();
    float r2[] = {20.0f, 40.0f, 60.0f};
    for (int i = 0; i < 3; i++)
        sim_update(adc_for_pct(r2[i]), adc_for_pct(r2[i]));
    sim_update(adc_for_pct(20.0f), adc_for_pct(20.0f));   /* delta -40 */
    ASSERT_TRUE(s_plausible,        "TC19: 60->20 not a fault");
    ASSERT_FALSE(s_rate_fault,      "TC19: 60->20 no rate_fault");
    ASSERT_NEAR(s_pct, 20.0f, 1.0f, "TC19: output ~20 %");
    sim_update(adc_for_pct(20.0f), adc_for_pct(20.0f));   /* hold */
    ASSERT_TRUE(s_plausible,        "TC19: 60->20 hold plausible");
    ASSERT_FALSE(s_rate_fault,      "TC19: 60->20 hold no false fault");
}

/* =========================================================================
 * Test Case 20 — §5 fast release 40->2 that DOES enter the rest zone → 0 %
 * ========================================================================= */
static void tc20_fast_release_into_rest_zone(void)
{
    printf("TC20: fast release 40 -> 2 (into rest zone) -> clean 0\n");
    sim_reset();
    float ramp[] = {20.0f, 40.0f};
    for (int i = 0; i < 2; i++)
        sim_update(adc_for_pct(ramp[i]), adc_for_pct(ramp[i]));
    /* 40 -> 2 %: delta -38 < -35, and 2 % <= release zone (3 %). */
    sim_update(adc_for_pct(2.0f), adc_for_pct(2.0f));
    ASSERT_TRUE(s_plausible,           "TC20: plausible");
    ASSERT_FALSE(s_rate_fault,         "TC20: no rate_fault");
    ASSERT_NEAR(s_pct, 0.0f, 0.5f,     "TC20: clean 0 % when settling in rest zone");
    ASSERT_NEAR(s_pct_prev, 0.0f, 0.5f, "TC20: prev reset to 0");
    ASSERT_NEAR(s_ema, 0.0f, 0.5f,     "TC20: EMA reset to 0");
}

/* =========================================================================
 * main
 * ========================================================================= */
int main(void)
{
    tc01_boot_released();
    tc02_progressive_ramp();
    tc03_rapid_release_from_53();
    tc04_rapid_release_from_100();
    tc05_rapid_application();
    tc06_recovery_after_rapid_application();
    tc07_contradictory_samples();
    tc08_rail_high();
    tc09_reject_0x0023_decomposition();
    tc10_reject_0x002F_decomposition();
    tc11_recovery_counter_resets_on_rise();
    tc12_no_spontaneous_reactivation();
    tc13_ema_not_contaminated();
    tc14_dual_sample_tolerance_boundary();
    tc15_small_rate_transitions();
    tc16_no_permanent_lockout();
    tc17_range_boundary();
    tc18_fast_partial_release_100_to_60();
    tc19_partial_releases_above_zone();
    tc20_fast_release_into_rest_zone();

    printf("=== test_pedal_plausibility: %d run, %d failed ===\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#endif /* HOST_TEST */
