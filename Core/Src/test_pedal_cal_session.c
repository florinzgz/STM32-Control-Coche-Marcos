/**
  ****************************************************************************
  * @file    test_pedal_cal_session.c
  * @brief   Host unit tests for the explicit pedal-calibration session FSM
  *          (pedal_cal_session.c) — audit Problem 5.
  *
  *          Covers the happy path MIN -> MAX -> SAVE and every safe abort:
  *          entry blocks, mid-session SAFE/ERROR/emergency/movement/CAN-loss,
  *          timeout, unstable capture, MIN>=MAX, range too small, readback
  *          mismatch, and — the core contradiction of the audit — that
  *          CAPTURE MAX is reachable with the pedal PRESSED (no <3 % rule).
  *
  *          Compile with host GCC (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -Wall -Wextra -Werror \
  *                Core/Src/test_pedal_cal_session.c \
  *                Core/Src/pedal_cal_session.c -lm \
  *                -o /tmp/test_pedal_cal_session
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <string.h>

#include "pedal_cal_session.h"

static int tests_run = 0, tests_failed = 0;

#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ---- Fake persistence backing store ------------------------------ */
static uint16_t g_store_min, g_store_max;
static bool     g_persist_ok  = true;
static bool     g_readback_ok = true;
static bool     g_readback_corrupt = false;  /* return a wrong value */
static int      g_apply_calls = 0;

static bool fake_validate(uint16_t mn, uint16_t mx)
{
    return (mx > mn) && ((uint32_t)(mx - mn) >= 800U) && (mx <= 4094U);
}
static bool fake_persist(uint16_t mn, uint16_t mx)
{
    if (!g_persist_ok) return false;
    g_store_min = mn; g_store_max = mx;
    return true;
}
static void fake_apply(uint16_t mn, uint16_t mx) { (void)mn; (void)mx; g_apply_calls++; }
static bool fake_readback(uint16_t *mn, uint16_t *mx)
{
    if (!g_readback_ok) return false;
    *mn = g_readback_corrupt ? (uint16_t)(g_store_min + 5U) : g_store_min;
    *mx = g_store_max;
    return true;
}

static PedalCalSessionHooks make_hooks(void)
{
    PedalCalSessionHooks h;
    h.validate = fake_validate;
    h.persist  = fake_persist;
    h.apply    = fake_apply;
    h.readback = fake_readback;
    return h;
}

/* Baseline "safe to calibrate" conditions. */
static PedalCalConds base_conds(uint32_t t)
{
    PedalCalConds c;
    memset(&c, 0, sizeof(c));
    c.now_ms               = t;
    c.in_standby           = true;
    c.gear_park_or_neutral = true;
    c.wheels_moving        = false;
    c.pedal_plausible      = true;
    c.pedal_released       = true;
    c.pedal_pressed_full   = false;
    c.pedal_raw            = 40;
    c.critical_error       = false;
    c.safe_state           = false;
    c.emergency            = false;
    c.can_loss             = false;
    c.traction_inhibited   = true;
    return c;
}

/* Drive the FSM for `n` ticks with a fixed condition snapshot (advancing
 * time by `dt` each tick and using `raw` as the ADC sample). */
static void run_ticks(PedalCalSession *s, PedalCalConds *c, int n,
                      uint16_t raw, uint32_t dt)
{
    for (int i = 0; i < n; i++) {
        c->pedal_raw = raw;
        PedalCalSession_Update(s, c);
        c->now_ms += dt;
    }
}

/* ================================================================== */
static void test_happy_path(void)
{
    g_persist_ok = true; g_readback_ok = true; g_readback_corrupt = false;
    g_apply_calls = 0;

    PedalCalSession s;
    PedalCalSessionHooks h = make_hooks();
    PedalCalSession_Init(&s, NULL, &h);
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_IDLE);

    PedalCalConds c = base_conds(1000);
    CHECK(PedalCalSession_Begin(&s, &c) == true);
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_ENTERING);

    /* ENTERING -> WAIT_RELEASED -> CAPTURING_MIN (pedal released). */
    PedalCalSession_Update(&s, &c); c.now_ms += 50;  /* ENTERING->WAIT_RELEASED */
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_WAIT_RELEASED);
    PedalCalSession_Update(&s, &c); c.now_ms += 50;  /* WAIT_RELEASED->CAPTURING_MIN */
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_CAPTURING_MIN);

    /* 8 stable released samples ~40 counts -> MIN locked. */
    run_ticks(&s, &c, 8, 42, 50);
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_WAIT_FULL_PRESS);
    CHECK(s.have_min && s.adc_min >= 40 && s.adc_min <= 44);

    /* Press the pedal -> CAPTURING_MAX (proves MAX works while PRESSED). */
    c.pedal_released     = false;
    c.pedal_pressed_full = true;
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_CAPTURING_MAX);
    run_ticks(&s, &c, 8, 4000, 50);
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_WAIT_RELEASE_FOR_SAVE);
    CHECK(s.have_max && s.adc_max >= 3996 && s.adc_max <= 4004);

    /* Release again -> READY_TO_SAVE. */
    c.pedal_released     = true;
    c.pedal_pressed_full = false;
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_READY_TO_SAVE);

    /* SAVE -> COMPLETED, persisted+applied+readback verified. */
    PedalCalSession_RequestSave(&s, &c);
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_COMPLETED);
    CHECK(g_apply_calls == 1);
    CHECK(g_store_min == s.adc_min && g_store_max == s.adc_max);
    CHECK(PedalCalSession_Reason(&s) == PEDAL_CAL_SESS_OK);
}

/* Capture MAX must be reachable with the pedal pressed — the exact
 * contradiction the audit fixes. */
static void test_capture_max_allows_pressed_pedal(void)
{
    PedalCalSession s;
    PedalCalSessionHooks h = make_hooks();
    PedalCalSession_Init(&s, NULL, &h);
    PedalCalConds c = base_conds(0);
    CHECK(PedalCalSession_Begin(&s, &c));
    PedalCalSession_Update(&s, &c); c.now_ms += 50;   /* WAIT_RELEASED */
    PedalCalSession_Update(&s, &c); c.now_ms += 50;   /* CAPTURING_MIN */
    run_ticks(&s, &c, 8, 30, 50);                     /* MIN */
    CHECK(s.state == PEDAL_CAL_WAIT_FULL_PRESS);

    /* Pedal fully pressed: pedal_released=false the whole time. */
    c.pedal_released     = false;
    c.pedal_pressed_full = true;
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    CHECK(s.state == PEDAL_CAL_CAPTURING_MAX);
    run_ticks(&s, &c, 8, 3900, 50);
    CHECK(s.state == PEDAL_CAL_WAIT_RELEASE_FOR_SAVE);
    CHECK(s.have_max);   /* captured while pressed — no <3 % rejection */
}

static void test_entry_blocks(void)
{
    PedalCalSession s;
    PedalCalSession_Init(&s, NULL, NULL);

    /* Not standby. */
    PedalCalConds c = base_conds(0); c.in_standby = false;
    CHECK(!PedalCalSession_Begin(&s, &c));
    CHECK(s.reason & PEDAL_CAL_BLOCK_NOT_STANDBY);
    CHECK(s.state == PEDAL_CAL_IDLE);

    /* Wrong gear. */
    c = base_conds(0); c.gear_park_or_neutral = false;
    CHECK(!PedalCalSession_Begin(&s, &c));
    CHECK(s.reason & PEDAL_CAL_BLOCK_GEAR);

    /* Wheels moving. */
    c = base_conds(0); c.wheels_moving = true;
    CHECK(!PedalCalSession_Begin(&s, &c));
    CHECK(s.reason & PEDAL_CAL_BLOCK_WHEELS_MOVING);

    /* Pedal implausible. */
    c = base_conds(0); c.pedal_plausible = false;
    CHECK(!PedalCalSession_Begin(&s, &c));
    CHECK(s.reason & PEDAL_CAL_BLOCK_PEDAL_IMPLAUSIBLE);

    /* Critical error. */
    c = base_conds(0); c.critical_error = true;
    CHECK(!PedalCalSession_Begin(&s, &c));
    CHECK(s.reason & PEDAL_CAL_BLOCK_CRITICAL_ERROR);

    /* Traction not inhibited. */
    c = base_conds(0); c.traction_inhibited = false;
    CHECK(!PedalCalSession_Begin(&s, &c));
    CHECK(s.reason & PEDAL_CAL_BLOCK_TRACTION_LIVE);
}

/* Every mid-session abort cause maps to its own bit and lands in ABORTED. */
static void test_runtime_aborts(void)
{
    struct { const char *name; void (*mut)(PedalCalConds*); uint16_t bit; } cases[] = {
        { "safe",      NULL, PEDAL_CAL_ABORT_SAFE },
        { "error",     NULL, PEDAL_CAL_ABORT_ERROR },
        { "emergency", NULL, PEDAL_CAL_ABORT_EMERGENCY },
        { "movement",  NULL, PEDAL_CAL_ABORT_MOVEMENT },
        { "can_loss",  NULL, PEDAL_CAL_ABORT_CAN_LOSS },
    };
    (void)cases;

    for (int k = 0; k < 5; k++) {
        PedalCalSession s;
        PedalCalSession_Init(&s, NULL, NULL);
        PedalCalConds c = base_conds(0);
        CHECK(PedalCalSession_Begin(&s, &c));
        PedalCalSession_Update(&s, &c);           /* WAIT_RELEASED */
        c.now_ms += 50;

        switch (k) {
        case 0: c.safe_state = true;     break;
        case 1: c.critical_error = true; break;
        case 2: c.emergency = true;      break;
        case 3: c.wheels_moving = true;  break;
        case 4: c.can_loss = true;       break;
        default: break;
        }
        PedalCalSession_Update(&s, &c);
        CHECK(s.state == PEDAL_CAL_ABORTED);
        CHECK(s.reason & cases[k].bit);
    }
}

static void test_timeout_abort(void)
{
    PedalCalSession s;
    PedalCalSession_Init(&s, NULL, NULL);
    PedalCalConds c = base_conds(0);
    /* Never release the pedal so WAIT_RELEASED lingers past the timeout. */
    c.pedal_released = false;
    /* Begin needs a released pedal? No — entry doesn't require release. */
    CHECK(PedalCalSession_Begin(&s, &c));
    PedalCalSession_Update(&s, &c);   /* ENTERING -> WAIT_RELEASED */
    c.now_ms += s.cfg.phase_timeout_ms + 1U;
    PedalCalSession_Update(&s, &c);
    CHECK(s.state == PEDAL_CAL_ABORTED);
    CHECK(s.reason & PEDAL_CAL_ABORT_TIMEOUT);
}

static void test_unstable_capture(void)
{
    PedalCalSession s;
    PedalCalSession_Init(&s, NULL, NULL);
    PedalCalConds c = base_conds(0);
    CHECK(PedalCalSession_Begin(&s, &c));
    PedalCalSession_Update(&s, &c); c.now_ms += 50;   /* WAIT_RELEASED */
    PedalCalSession_Update(&s, &c); c.now_ms += 50;   /* CAPTURING_MIN */
    CHECK(s.state == PEDAL_CAL_CAPTURING_MIN);
    /* Feed noisy samples (spread > tol) until the capture timeout fires. */
    for (int i = 0; i < 200; i++) {
        c.pedal_raw = (i & 1) ? 10 : 900;   /* huge spread, never stable */
        PedalCalSession_Update(&s, &c);
        c.now_ms += 50;
        if (s.state != PEDAL_CAL_CAPTURING_MIN) break;
    }
    CHECK(s.state == PEDAL_CAL_ABORTED);
    CHECK(s.reason & PEDAL_CAL_FAIL_UNSTABLE);
}

static void test_range_too_small(void)
{
    PedalCalSession s;
    PedalCalSessionHooks h = make_hooks();
    PedalCalSession_Init(&s, NULL, &h);
    PedalCalConds c = base_conds(0);
    CHECK(PedalCalSession_Begin(&s, &c));
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    run_ticks(&s, &c, 8, 100, 50);                    /* MIN=100 */
    CHECK(s.state == PEDAL_CAL_WAIT_FULL_PRESS);
    c.pedal_released = false; c.pedal_pressed_full = true;
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    run_ticks(&s, &c, 8, 500, 50);                    /* MAX=500 -> range 400<800 */
    CHECK(s.state == PEDAL_CAL_CAPTURING_MAX || s.state == PEDAL_CAL_WAIT_RELEASE_FOR_SAVE);
    c.pedal_released = true; c.pedal_pressed_full = false;
    PedalCalSession_Update(&s, &c);
    CHECK(s.state == PEDAL_CAL_ABORTED);
    CHECK(s.reason & PEDAL_CAL_FAIL_RANGE_SMALL);
}

static void test_readback_mismatch(void)
{
    g_persist_ok = true; g_readback_ok = true; g_readback_corrupt = true;
    PedalCalSession s;
    PedalCalSessionHooks h = make_hooks();
    PedalCalSession_Init(&s, NULL, &h);
    PedalCalConds c = base_conds(0);
    CHECK(PedalCalSession_Begin(&s, &c));
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    run_ticks(&s, &c, 8, 50, 50);
    c.pedal_released = false; c.pedal_pressed_full = true;
    PedalCalSession_Update(&s, &c); c.now_ms += 50;
    run_ticks(&s, &c, 8, 3800, 50);
    c.pedal_released = true; c.pedal_pressed_full = false;
    PedalCalSession_Update(&s, &c);
    CHECK(s.state == PEDAL_CAL_READY_TO_SAVE);
    PedalCalSession_RequestSave(&s, &c);
    CHECK(s.state == PEDAL_CAL_ABORTED);
    CHECK(s.reason & PEDAL_CAL_FAIL_READBACK);
    g_readback_corrupt = false;
}

static void test_reason_text(void)
{
    CHECK(strcmp(PedalCalSession_ReasonText(PEDAL_CAL_SESS_OK), "OK") == 0);
    /* Emergency dominates when several bits are set. */
    uint16_t m = PEDAL_CAL_ABORT_EMERGENCY | PEDAL_CAL_ABORT_MOVEMENT;
    CHECK(strcmp(PedalCalSession_ReasonText(m), "EMERGENCIA") == 0);
    CHECK(strcmp(PedalCalSession_StateText(PEDAL_CAL_CAPTURING_MAX), "CAPTURANDO MAX") == 0);
}

int main(void)
{
    test_happy_path();
    test_capture_max_allows_pressed_pedal();
    test_entry_blocks();
    test_runtime_aborts();
    test_timeout_abort();
    test_unstable_capture();
    test_range_too_small();
    test_readback_mismatch();
    test_reason_text();

    printf("pedal_cal_session: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
#else
int main(void) { return 0; }
#endif
