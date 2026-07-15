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
#define T_PEDAL_CONTRADICT_CYCLES 3U
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
_Static_assert(T_PEDAL_CONTRADICT_CYCLES == PEDAL_CONTRADICT_DEBOUNCE_CYCLES,
               "contradict debounce drift");

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
static bool     s_rate_limited = false;
static uint8_t  s_contradict_count = 0;

static void sync_from_state(void)
{
    s_pct        = g_st.pct;
    s_pct_raw    = g_st.pct_raw;
    s_pct_prev   = g_st.pct_prev;
    s_ema        = g_st.ema;
    s_plausible  = g_st.plausible;
    s_contradict = g_st.contradict;
    s_ema_primed = g_st.ema_primed;
    s_rate_limited = g_st.rate_limited;
    s_contradict_count = g_st.contradict_count;
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
    ASSERT_FALSE(s_rate_limited,     "TC03: release is not rate-limited");
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
    ASSERT_FALSE(s_rate_limited,     "TC04: release is not rate-limited");
    ASSERT_NEAR(s_ema,      0.0f, 0.5f, "TC04: EMA reset");
}

/* =========================================================================
 * Test Case 5 — Fast coherent application 0 → 60 % in one cycle
 *   NEW POLICY (§A): a fast but electrically-coherent stab is VALID intent,
 *   NOT a fault.  The demand must stay plausible and RISE (rate-limited),
 *   never collapse to 0 % and never raise SENSOR_FAULT.
 * ========================================================================= */
static void tc05_fast_application(void)
{
    printf("TC05: fast coherent application 0 → 60 %% (valid intent, ramp)\n");
    sim_reset();
    sim_update(ADC_RELEASED, ADC_RELEASED);   /* stable at 0 */
    ASSERT_TRUE(s_plausible, "TC05: plausible at 0");

    uint16_t a60 = adc_for_pct(60.0f);
    sim_update(a60, a60);

    ASSERT_TRUE(s_plausible,        "TC05: fast stab stays PLAUSIBLE (not a fault)");
    ASSERT_FALSE(s_contradict,      "TC05: no contradiction (coherent samples)");
    ASSERT_TRUE(s_pct > 0.0f,       "TC05: demand rises immediately (non-zero)");
    ASSERT_TRUE(s_pct <= 60.5f,     "TC05: demand never exceeds physical pedal");

    /* Hold 60 %: demand must keep climbing toward 60 %, still plausible. */
    float last = s_pct;
    for (int i = 0; i < 12; i++) {
        sim_update(a60, a60);
        ASSERT_TRUE(s_plausible, "TC05: plausible while ramping to 60 %");
        ASSERT_TRUE(s_pct >= last - 0.01f, "TC05: demand monotonically climbs");
        last = s_pct;
    }
    ASSERT_NEAR(s_pct, 60.0f, 1.5f, "TC05: demand converges to ~60 %");
}

/* =========================================================================
 * Test Case 6 — Fast application then immediate release
 *   The fast stab is accepted (ramp) and a subsequent release drops the
 *   demand at once with no lingering fault state.
 * ========================================================================= */
static void tc06_fast_application_then_release(void)
{
    printf("TC06: fast application then release — no lingering fault\n");
    sim_reset();
    sim_update(ADC_RELEASED, ADC_RELEASED);

    uint16_t a60 = adc_for_pct(60.0f);
    sim_update(a60, a60);
    ASSERT_TRUE(s_plausible, "TC06: plausible after fast stab");
    ASSERT_TRUE(s_pct > 0.0f, "TC06: demand non-zero after fast stab");

    /* Release immediately — demand must drop to 0 in a single cycle. */
    sim_update(ADC_RELEASED, ADC_RELEASED);
    ASSERT_TRUE(s_plausible,        "TC06: plausible after release");
    ASSERT_FALSE(s_rate_limited,    "TC06: release not rate-limited");
    ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC06: demand 0 % immediately after release");

    /* Re-apply — must respond again (no reset / no lockout required). */
    sim_update(adc_for_pct(10.0f), adc_for_pct(10.0f));
    ASSERT_TRUE(s_plausible, "TC06: plausible on re-apply");
    ASSERT_TRUE(s_pct > 0.0f, "TC06: responds again without restart");
}

/* =========================================================================
 * Test Case 7 — Transient vs persistent dual-sample contradiction
 *   §C: a single/short disagreement is transient noise — NOT an immediate
 *   fault (last demand held, stays plausible).
 *   §D: a PERSISTENT disagreement latches the contradiction fault → 0 torque.
 * ========================================================================= */
static void tc07_contradiction_debounce(void)
{
    printf("TC07: transient vs persistent ADC1/ADC2 contradiction\n");
    sim_reset();

    /* Build a stable ~30 % demand first. */
    for (int i = 0; i < 8; i++)
        sim_update(adc_for_pct(30.0f), adc_for_pct(30.0f));
    ASSERT_TRUE(s_plausible, "TC07: plausible at stable 30 %");
    float held = s_pct;

    /* Single transient contradiction (diff 31 > tol 30) — must NOT fault. */
    sim_update(500U, 531U);
    ASSERT_FALSE(s_contradict,  "TC07: single contradiction is NOT yet a fault");
    ASSERT_TRUE(s_plausible,    "TC07: stays plausible on transient noise");
    ASSERT_NEAR(s_pct, held, 0.5f, "TC07: last validated demand held");

    /* Samples agree again — counter resets, no fault ever declared. */
    sim_update(adc_for_pct(30.0f), adc_for_pct(30.0f));
    ASSERT_FALSE(s_contradict, "TC07: recovered, no fault after transient");
    ASSERT_TRUE(s_plausible,   "TC07: plausible after transient recovery");

    /* Now PERSISTENT contradiction for the full debounce window → fault. */
    for (uint8_t i = 0; i < T_PEDAL_CONTRADICT_CYCLES; i++)
        sim_update(500U, 560U);
    ASSERT_TRUE(s_contradict,  "TC07: persistent contradiction latches fault");
    ASSERT_FALSE(s_plausible,  "TC07: implausible on persistent contradiction");
    ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC07: zero torque on persistent contradiction");

    /* Recovery when samples agree again. */
    sim_update(adc_for_pct(30.0f), adc_for_pct(30.0f));
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
    ASSERT_FALSE(s_rate_limited,    "TC08: rail fault clears rate_limited flag");

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
 * Test Case 11 — Contradiction debounce counter resets when samples agree
 * ========================================================================= */
static void tc11_contradiction_counter_resets(void)
{
    printf("TC11: contradiction debounce counter resets when samples agree\n");
    sim_reset();
    sim_update(ADC_RELEASED, ADC_RELEASED);

    /* Two contradictory cycles (below the 3-cycle threshold) → no fault yet. */
    sim_update(500U, 560U);
    ASSERT_EQ((int)s_contradict_count, 1, "TC11: counter = 1 after 1st contradiction");
    ASSERT_FALSE(s_contradict, "TC11: not yet a fault after 1 cycle");
    sim_update(500U, 560U);
    ASSERT_EQ((int)s_contradict_count, 2, "TC11: counter = 2 after 2nd contradiction");
    ASSERT_FALSE(s_contradict, "TC11: not yet a fault after 2 cycles");

    /* Samples agree — counter must reset to 0. */
    sim_update(adc_for_pct(20.0f), adc_for_pct(20.0f));
    ASSERT_EQ((int)s_contradict_count, 0, "TC11: counter reset when samples agree");
    ASSERT_TRUE(s_plausible,       "TC11: plausible after recovery");
}

/* =========================================================================
 * Test Case 12 — Fast application is progressively ACCEPTED (not zeroed)
 *   Opposite of the old "must stay implausible" expectation: an 80 % stab is
 *   valid intent and the demand must climb toward 80 %, never collapse to 0.
 * ========================================================================= */
static void tc12_fast_application_accepted(void)
{
    printf("TC12: fast application is progressively accepted (ramp to 80 %%)\n");
    sim_reset();
    sim_update(ADC_RELEASED, ADC_RELEASED);

    sim_update(adc_for_pct(80.0f), adc_for_pct(80.0f));
    ASSERT_TRUE(s_plausible, "TC12: plausible after 0→80 stab");
    ASSERT_TRUE(s_pct > 0.0f, "TC12: demand non-zero after stab");

    /* Hold 80 % — demand climbs and stays plausible the whole time. */
    for (int i = 0; i < 15; i++) {
        sim_update(adc_for_pct(80.0f), adc_for_pct(80.0f));
        ASSERT_TRUE(s_plausible,   "TC12: remains plausible ramping to 80 %");
        ASSERT_TRUE(s_pct <= 80.5f, "TC12: never exceeds physical pedal");
    }
    ASSERT_NEAR(s_pct, 80.0f, 1.5f, "TC12: converged to ~80 %");
}

/* =========================================================================
 * Test Case 13 — EMA never exceeds the physical pedal on a fast stab
 * ========================================================================= */
static void tc13_ema_bounded_on_fast_stab(void)
{
    printf("TC13: EMA/demand never overshoots the physical pedal on a stab\n");
    sim_reset();

    /* Build stable 20 % */
    for (int i = 0; i < 8; i++)
        sim_update(adc_for_pct(20.0f), adc_for_pct(20.0f));

    /* Fast application to 80 % — the output must rise but never exceed 80 %. */
    for (int i = 0; i < 15; i++) {
        sim_update(adc_for_pct(80.0f), adc_for_pct(80.0f));
        ASSERT_TRUE(s_pct <= 80.5f, "TC13: demand bounded by physical pedal");
        ASSERT_TRUE(s_ema <= 80.5f, "TC13: EMA bounded by physical pedal");
    }
    ASSERT_TRUE(s_pct > 20.0f, "TC13: demand increased above the prior 20 %");
}

/* =========================================================================
 * Test Case 14 — Dual-sample tolerance boundary
 * ========================================================================= */
static void tc14_dual_sample_tolerance_boundary(void)
{
    printf("TC14: dual-sample tolerance boundary at ±30 counts\n");
    sim_reset();

    /* Exactly at tolerance — should pass (no contradiction). */
    sim_update(500U, 500U + T_PEDAL_SAMPLE_TOLERANCE);
    ASSERT_FALSE(s_contradict, "TC14: diff==tolerance is still OK");
    ASSERT_TRUE(s_plausible,   "TC14: plausible at boundary");

    sim_reset();
    /* One above tolerance for a SINGLE cycle — transient, not yet a fault. */
    sim_update(500U, 500U + T_PEDAL_SAMPLE_TOLERANCE + 1U);
    ASSERT_FALSE(s_contradict, "TC14: diff==tolerance+1 single cycle is transient");
    ASSERT_TRUE(s_plausible,   "TC14: still plausible on single transient");

    /* Persist it for the debounce window → contradiction fault. */
    for (uint8_t i = 0; i < T_PEDAL_CONTRADICT_CYCLES; i++)
        sim_update(500U, 500U + T_PEDAL_SAMPLE_TOLERANCE + 1U);
    ASSERT_TRUE(s_contradict,  "TC14: persistent diff>tol triggers contradiction");
    ASSERT_FALSE(s_plausible,  "TC14: implausible when persistent");
    ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC14: output 0 on persistent contradiction");
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
    /* After warm-up at 4094 the range check passes and the signal is a valid
     * (near-100 %) coherent reading, so it must be PLAUSIBLE (no fault).      */
    ASSERT_FALSE(s_contradict, "TC17: no contradiction at 4094");
    ASSERT_TRUE(s_plausible,   "TC17: 4094 is a valid in-range reading");

    /* ADC 4095 must always fault (rail short) regardless of history */
    sim_update(4095U, 4095U);
    ASSERT_FALSE(s_plausible,       "TC17: 4095 triggers fault");
    ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC17: output 0 on 4095");
    ASSERT_FALSE(s_rate_limited,    "TC17: rail fault clears rate_limited flag");
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
    ASSERT_FALSE(s_rate_limited,    "TC18: release is not rate-limited");
    ASSERT_NEAR(s_pct, 60.0f, 1.0f, "TC18: output tracks reduced pedal (~60 %)");
    ASSERT_TRUE(s_pct <= 60.5f,     "TC18: output never exceeds physical pedal");
    ASSERT_NEAR(s_pct_prev, 60.0f, 1.0f, "TC18: baseline re-anchored to 60 %");

    /* Hold at 60 %: the NEXT cycle must NOT fire a false +rising fault. */
    for (int i = 0; i < 5; i++) {
        sim_update(adc_for_pct(60.0f), adc_for_pct(60.0f));
        ASSERT_TRUE(s_plausible,   "TC18: still plausible holding 60 %");
        ASSERT_TRUE(s_plausible,   "TC18: no false rising fault holding 60 %");
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
    ASSERT_FALSE(s_rate_limited,    "TC19: 80->40 release not rate-limited");
    ASSERT_NEAR(s_pct, 40.0f, 1.0f, "TC19: output ~40 %");
    sim_update(adc_for_pct(40.0f), adc_for_pct(40.0f));   /* hold */
    ASSERT_TRUE(s_plausible,        "TC19: 80->40 hold plausible");
    ASSERT_TRUE(s_plausible,        "TC19: 80->40 hold no false fault");

    /* 60 -> 20 */
    sim_reset();
    float r2[] = {20.0f, 40.0f, 60.0f};
    for (int i = 0; i < 3; i++)
        sim_update(adc_for_pct(r2[i]), adc_for_pct(r2[i]));
    sim_update(adc_for_pct(20.0f), adc_for_pct(20.0f));   /* delta -40 */
    ASSERT_TRUE(s_plausible,        "TC19: 60->20 not a fault");
    ASSERT_FALSE(s_rate_limited,    "TC19: 60->20 release not rate-limited");
    ASSERT_NEAR(s_pct, 20.0f, 1.0f, "TC19: output ~20 %");
    sim_update(adc_for_pct(20.0f), adc_for_pct(20.0f));   /* hold */
    ASSERT_TRUE(s_plausible,        "TC19: 60->20 hold plausible");
    ASSERT_TRUE(s_plausible,        "TC19: 60->20 hold no false fault");
}

/* =========================================================================
 * Test Case 20 — Fast release 40->2 %: demand tracks the real (near-rest)
 *   pedal immediately.  2 % is below the powertrain-engage threshold so it is
 *   effectively released, but the output stays physically accurate (not a
 *   fabricated 0 and never a fault).
 * ========================================================================= */
static void tc20_fast_release_into_rest_zone(void)
{
    printf("TC20: fast release 40 -> 2 (near rest) tracks real pedal\n");
    sim_reset();
    float ramp[] = {20.0f, 40.0f};
    for (int i = 0; i < 2; i++)
        sim_update(adc_for_pct(ramp[i]), adc_for_pct(ramp[i]));
    /* 40 -> 2 %: a fast release; output snaps down to the real ~2 %. */
    sim_update(adc_for_pct(2.0f), adc_for_pct(2.0f));
    ASSERT_TRUE(s_plausible,           "TC20: plausible");
    ASSERT_FALSE(s_rate_limited,       "TC20: release is not rate-limited");
    ASSERT_NEAR(s_pct, 2.0f, 0.7f,     "TC20: demand tracks the real ~2 % pedal");
    ASSERT_TRUE(s_pct < T_PEDAL_RELEASE_ZONE_PCT,
                "TC20: below powertrain-engage threshold (effectively released)");
    ASSERT_NEAR(s_ema, 2.0f, 0.7f,     "TC20: EMA snapped down to ~2 %");
}

/* =========================================================================
 * Test Case 21 — Fast stab 0 → 100 % (full press) is valid, never a fault
 * ========================================================================= */
static void tc21_fast_stab_0_to_100(void)
{
    printf("TC21: fast stab 0 -> 100 %% is valid intent (ramp, no fault)\n");
    sim_reset();
    sim_update(ADC_RELEASED, ADC_RELEASED);

    uint16_t a100 = adc_for_pct(100.0f);
    sim_update(a100, a100);
    ASSERT_TRUE(s_plausible,  "TC21: plausible after 0->100 stab");
    ASSERT_FALSE(s_contradict,"TC21: no contradiction");
    ASSERT_TRUE(s_pct > 0.0f, "TC21: demand rises immediately (non-zero)");

    float last = s_pct;
    for (int i = 0; i < 20; i++) {
        sim_update(a100, a100);
        ASSERT_TRUE(s_plausible,  "TC21: plausible while ramping to 100 %");
        ASSERT_TRUE(s_pct >= last - 0.01f, "TC21: demand climbs monotonically");
        ASSERT_TRUE(s_pct <= 100.5f, "TC21: never exceeds 100 %");
        last = s_pct;
    }
    ASSERT_NEAR(s_pct, 100.0f, 2.0f, "TC21: converges to ~100 %");
}

/* =========================================================================
 * Test Case 22 — Stable hold at 20 / 50 / 100 % produces steady demand
 * ========================================================================= */
static void tc22_hold_stable_levels(void)
{
    printf("TC22: steady hold at 20 / 50 / 100 %%\n");
    const float levels[] = {20.0f, 50.0f, 100.0f};
    for (int l = 0; l < 3; l++) {
        sim_reset();
        uint16_t a = adc_for_pct(levels[l]);
        for (int i = 0; i < 25; i++)
            sim_update(a, a);
        ASSERT_TRUE(s_plausible, "TC22: plausible holding level");
        ASSERT_FALSE(s_rate_limited, "TC22: steady hold not rate-limited");
        ASSERT_NEAR(s_pct, levels[l], 1.5f, "TC22: demand settles at held level");
    }
}

/* =========================================================================
 * Test Case 23 — Single-sample noise must NOT change global state
 *   A one-cycle incoherent sample keeps the last demand and stays plausible;
 *   the very next coherent cycle resumes normally (no DTC, no lockout).
 * ========================================================================= */
static void tc23_single_sample_noise_no_state_change(void)
{
    printf("TC23: single-sample noise keeps last demand, no state change\n");
    sim_reset();
    for (int i = 0; i < 10; i++)
        sim_update(adc_for_pct(50.0f), adc_for_pct(50.0f));
    ASSERT_NEAR(s_pct, 50.0f, 1.5f, "TC23: settled at 50 %");
    float held = s_pct;

    /* One incoherent cycle */
    sim_update(adc_for_pct(50.0f), adc_for_pct(50.0f) + 200U);
    ASSERT_TRUE(s_plausible,   "TC23: still plausible on 1-sample noise");
    ASSERT_FALSE(s_contradict, "TC23: no contradiction fault on 1-sample noise");
    ASSERT_NEAR(s_pct, held, 0.5f, "TC23: last validated demand held");

    /* Next coherent cycle resumes normally */
    sim_update(adc_for_pct(50.0f), adc_for_pct(50.0f));
    ASSERT_TRUE(s_plausible,   "TC23: plausible resumes after noise");
    ASSERT_NEAR(s_pct, 50.0f, 1.5f, "TC23: demand back at 50 %");
}

/* =========================================================================
 * Test Case 24 — Valid saved calibration (50 / 4000) maps 0–100 % correctly
 * ========================================================================= */
static void tc24_calibration_endpoints(void)
{
    printf("TC24: calibration endpoints 50/4000 map 0 and 100 %%\n");
    Pedal_StateInit(&g_st, 50U, 4000U);
    sync_from_state();

    sim_update(50U, 50U);              /* at min → 0 % */
    ASSERT_TRUE(s_plausible,        "TC24: plausible at min");
    ASSERT_NEAR(s_pct, 0.0f, 0.5f,  "TC24: 0 % at adc_min");

    sim_reset();
    Pedal_StateInit(&g_st, 50U, 4000U);
    for (int i = 0; i < 20; i++)
        sim_update(4000U, 4000U);      /* at max → 100 % */
    ASSERT_TRUE(s_plausible,          "TC24: plausible at max");
    ASSERT_NEAR(s_pct, 100.0f, 1.0f,  "TC24: 100 % at adc_max");
}

/* =========================================================================
 * Test Case 25 — Corrupt/absent calibration → safe default endpoints usable
 *   The pipeline never divides by zero and produces a bounded 0–100 % output
 *   even when initialised with the compile-time safe defaults.
 * ========================================================================= */
static void tc25_corrupt_calibration_safe_defaults(void)
{
    printf("TC25: safe default calibration still yields bounded output\n");
    /* Simulate "corrupt → fall back to defaults" by initialising with the
     * compile-time defaults (what pedal_cal_store.c does on a bad CRC). */
    Pedal_StateInit(&g_st, T_PEDAL_ADC_MIN, T_PEDAL_ADC_MAX);
    sync_from_state();

    for (int i = 0; i < 20; i++)
        sim_update(adc_for_pct(50.0f), adc_for_pct(50.0f));
    ASSERT_TRUE(s_plausible,           "TC25: plausible with default cal");
    ASSERT_TRUE(s_pct >= 0.0f && s_pct <= 100.0f, "TC25: output bounded 0–100 %");
    ASSERT_NEAR(s_pct, 50.0f, 2.0f,    "TC25: ~50 % with default cal");
}

/* =========================================================================
 * Test Case 26 — 53 %→0 release: output must NOT freeze at 53 %/60 %
 * ========================================================================= */
static void tc26_release_53_to_0_not_frozen(void)
{
    printf("TC26: release 53 -> 0 output not frozen\n");
    sim_reset();
    for (int i = 0; i < 15; i++)
        sim_update(adc_for_pct(53.0f), adc_for_pct(53.0f));
    ASSERT_TRUE(s_pct > 45.0f, "TC26: converged above 45 % before release");

    sim_update(ADC_RELEASED, ADC_RELEASED);
    ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC26: not frozen — drops to 0 %");

    /* Many subsequent released cycles must stay at 0 (no revival to 53/60). */
    for (int i = 0; i < 10; i++) {
        sim_update(ADC_RELEASED, ADC_RELEASED);
        ASSERT_NEAR(s_pct, 0.0f, 0.5f, "TC26: stays 0 %, never revives");
        ASSERT_TRUE(s_plausible, "TC26: plausible while released");
    }
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
    tc05_fast_application();
    tc06_fast_application_then_release();
    tc07_contradiction_debounce();
    tc08_rail_high();
    tc09_reject_0x0023_decomposition();
    tc10_reject_0x002F_decomposition();
    tc11_contradiction_counter_resets();
    tc12_fast_application_accepted();
    tc13_ema_bounded_on_fast_stab();
    tc14_dual_sample_tolerance_boundary();
    tc15_small_rate_transitions();
    tc16_no_permanent_lockout();
    tc17_range_boundary();
    tc18_fast_partial_release_100_to_60();
    tc19_partial_releases_above_zone();
    tc20_fast_release_into_rest_zone();
    tc21_fast_stab_0_to_100();
    tc22_hold_stable_levels();
    tc23_single_sample_noise_no_state_change();
    tc24_calibration_endpoints();
    tc25_corrupt_calibration_safe_defaults();
    tc26_release_53_to_0_not_frozen();

    printf("=== test_pedal_plausibility: %d run, %d failed ===\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#endif /* HOST_TEST */
