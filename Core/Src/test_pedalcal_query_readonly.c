/**
  ****************************************************************************
  * @file    test_pedalcal_query_readonly.c
  * @brief   Regression tests for the read-only pedal-cal telemetry seam.
  *
  *          audit fix — a telemetry QUERY (or any 0x319 "entry OK" evaluation)
  *          must NEVER enforce the traction movement lock, because that would
  *          force demand 0 / PWM 0 / traction enables LOW / relay OFF even
  *          outside a calibration session.  The real movement lock
  *          (Traction_CalibrationLock, mutating) may run ONLY from:
  *            - PEDAL_CAL_OP_CAPTURE_MIN, immediately before Begin;
  *            - CAN_PedalCalCaptureTick(), while a session is active.
  *          Every other path (entry-OK bit, session-status frame, QUERY) must
  *          use the pure read-only check Traction_IsCalibrationLockConfirmed().
  *
  *          can_handler.c / motor_control.c cannot be host-compiled (full HAL
  *          dependency), so — exactly like test_pedalcal_can_frame.c mirrors
  *          the 0x319 encoder — this test mirrors the can_handler decision
  *          logic against mock traction primitives and the REAL FSM
  *          (pedal_cal_session.c).  The mocks distinguish a MUTATING lock call
  *          from a pure READ, so a regression that re-introduces the mutating
  *          lock on a read-only path fails the build.
  *
  *          Compile with host GCC (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -Wall -Wextra \
  *                Core/Src/test_pedalcal_query_readonly.c \
  *                Core/Src/pedal_cal_session.c -lm \
  *                -o /tmp/test_pedalcal_query_readonly
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "pedal_cal_session.h"

static int tests_run = 0, tests_failed = 0;

#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ---- Mock traction outputs + primitives ---------------------------------
 * These model the physical traction state that Traction_CalibrationLock()
 * drives and that Traction_IsCalibrationLockConfirmed() only observes. */
typedef struct {
    float   demand;        /* effective traction demand   */
    uint8_t pwm;           /* resolved final PWM percent   */
    bool    en_low;        /* traction enables physically LOW */
    bool    relay_off;     /* traction relay de-energised  */
    int     lock_calls;    /* # of MUTATING CalibrationLock() calls */
    int     read_calls;    /* # of pure IsConfirmed() reads         */
} MockTraction;

static MockTraction g_tr;

static void mock_reset(bool locked)
{
    g_tr.demand     = locked ? 0.0f : 12.5f;
    g_tr.pwm        = locked ? 0U   : 40U;
    g_tr.en_low     = locked;
    g_tr.relay_off  = locked;
    g_tr.lock_calls = 0;
    g_tr.read_calls = 0;
}

/* Mirror of motor_control.c::Traction_CalibrationLock(): MUTATING.  Forces the
 * "cannot move" outputs, then verifies and returns the result. */
static bool mock_Traction_CalibrationLock(void)
{
    g_tr.lock_calls++;
    g_tr.demand = 0.0f;
    g_tr.pwm    = 0U;
    g_tr.en_low = true;          /* drives the four H-bridges to COAST */
    return g_tr.en_low && (g_tr.pwm == 0U);
}

/* Mirror of motor_control.c::Traction_IsCalibrationLockConfirmed(): READ-ONLY.
 * Only observes demand / PWM / enables / relay — never mutates. */
static bool mock_Traction_IsCalibrationLockConfirmed(void)
{
    g_tr.read_calls++;
    bool demand_zero = (g_tr.demand < 0.01f && g_tr.demand > -0.01f);
    return demand_zero && g_tr.en_low && (g_tr.pwm == 0U) && g_tr.relay_off;
}

/* Snapshot helper to assert "no side effects on outputs". */
typedef struct { float demand; uint8_t pwm; bool en_low; bool relay_off; } Outputs;
static Outputs mock_outputs(void)
{
    Outputs o = { g_tr.demand, g_tr.pwm, g_tr.en_low, g_tr.relay_off };
    return o;
}
static bool outputs_equal(Outputs a, Outputs b)
{
    return (a.demand == b.demand) && (a.pwm == b.pwm) &&
           (a.en_low == b.en_low) && (a.relay_off == b.relay_off);
}

/* ---- Live conditions builder (mirrors can_handler pedalcal_build_conds) --- */
static PedalCalConds sim_build_conds(uint32_t t)
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
    c.traction_locked      = false;   /* populated by the caller below */
    return c;
}

/* ---- Mirrors of the can_handler.c seams under test ----------------------- */

/* pedalcal_entry_ok(): STRICTLY read-only — uses the pure confirmation. */
static bool sim_entry_ok(uint32_t t)
{
    PedalCalConds c = sim_build_conds(t);
    c.traction_locked = mock_Traction_IsCalibrationLockConfirmed();
    return c.in_standby && c.gear_park_or_neutral && !c.wheels_moving &&
           c.pedal_plausible && !c.critical_error && c.traction_inhibited &&
           c.traction_locked;
}

/* pedalcal_send_session_status(): its only lock-related read is entry_ok. */
static bool sim_send_session_status(const PedalCalSession *s, uint32_t t)
{
    (void)s;
    /* flag bit 0x20 = entry guards OK now */
    return sim_entry_ok(t);
}

/* PEDAL_CAL_OP_QUERY: read-only telemetry burst + one session-status frame. */
static void sim_query(const PedalCalSession *s, uint32_t t)
{
    /* pedalcal_start_burst() only touches telemetry counters — no traction. */
    (void)sim_send_session_status(s, t);
}

/* PEDAL_CAL_OP_CAPTURE_MIN: enforce the REAL lock, then Begin. */
static bool sim_capture_min(PedalCalSession *s, uint32_t t)
{
    PedalCalConds c = sim_build_conds(t);
    bool en_pwm_locked = mock_Traction_CalibrationLock();
    bool relay_off     = g_tr.relay_off;
    c.traction_locked  = en_pwm_locked && relay_off;
    return PedalCalSession_Begin(s, &c);
}

/* CAN_PedalCalCaptureTick(): enforce the REAL lock every active tick. */
static PedalCalState sim_capture_tick(PedalCalSession *s, uint32_t t)
{
    if (!PedalCalSession_Active(s)) return PedalCalSession_State(s);
    bool en_pwm_locked = mock_Traction_CalibrationLock();
    bool relay_off     = g_tr.relay_off;
    PedalCalConds c = sim_build_conds(t);
    c.traction_locked = en_pwm_locked && relay_off;
    return PedalCalSession_Update(s, &c);
}

/* Drive a fresh session into an ACTIVE capturing state (lock held). */
static void begin_active(PedalCalSession *s, uint32_t *t)
{
    mock_reset(true);
    PedalCalSession_Init(s, NULL, NULL);
    CHECK(sim_capture_min(s, *t) == true);
    *t += 50;
    (void)sim_capture_tick(s, *t); *t += 50;   /* ENTERING -> WAIT_RELEASED */
    (void)sim_capture_tick(s, *t); *t += 50;   /* -> CAPTURING_MIN          */
    CHECK(PedalCalSession_Active(s));
}

/* ==========================================================================
 *  1. QUERY while a session is ACTIVE must not modify demand / PWM / EN / relay
 *     and must not run the mutating CalibrationLock.
 * ======================================================================== */
static void test_query_active_readonly(void)
{
    PedalCalSession s;
    uint32_t t = 1000;
    begin_active(&s, &t);

    /* Freeze the mutation counter and outputs, then run a QUERY. */
    int lock_before = g_tr.lock_calls;
    Outputs before  = mock_outputs();

    sim_query(&s, t);

    CHECK(g_tr.lock_calls == lock_before);          /* no CalibrationLock */
    CHECK(outputs_equal(mock_outputs(), before));   /* demand/PWM/EN/relay */
    CHECK(g_tr.read_calls > 0);                     /* only pure reads     */
    CHECK(PedalCalSession_Active(&s));              /* FSM state untouched */
}

/* ==========================================================================
 *  2. QUERY in STANDBY (no active session) must not run CalibrationLock.
 * ======================================================================== */
static void test_query_standby_no_lock(void)
{
    mock_reset(true);
    PedalCalSession s;
    PedalCalSession_Init(&s, NULL, NULL);
    CHECK(!PedalCalSession_Active(&s));

    Outputs before = mock_outputs();
    sim_query(&s, 2000);

    CHECK(g_tr.lock_calls == 0);                    /* never enforced      */
    CHECK(outputs_equal(mock_outputs(), before));
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_IDLE);
}

/* ==========================================================================
 *  3. pedalcal_send_session_status() has no side effects.
 * ======================================================================== */
static void test_session_status_no_side_effects(void)
{
    mock_reset(true);
    PedalCalSession s;
    PedalCalSession_Init(&s, NULL, NULL);

    Outputs before = mock_outputs();
    bool entry = sim_send_session_status(&s, 3000);

    CHECK(entry == true);                           /* lock confirmed -> OK */
    CHECK(g_tr.lock_calls == 0);                    /* no mutating lock     */
    CHECK(outputs_equal(mock_outputs(), before));

    /* And when the lock is NOT held, entry-OK reads false WITHOUT forcing it. */
    mock_reset(false);
    before = mock_outputs();
    entry = sim_send_session_status(&s, 3050);
    CHECK(entry == false);
    CHECK(g_tr.lock_calls == 0);
    CHECK(outputs_equal(mock_outputs(), before));   /* still not forced low */
}

/* ==========================================================================
 *  4. CAPTURE_MIN DOES run the mutating lock, before Begin.
 * ======================================================================== */
static void test_capture_min_runs_lock(void)
{
    mock_reset(false);                              /* start un-locked      */
    g_tr.relay_off = true;                          /* relay already OFF    */
    PedalCalSession s;
    PedalCalSession_Init(&s, NULL, NULL);

    CHECK(g_tr.lock_calls == 0);
    bool ok = sim_capture_min(&s, 1000);

    CHECK(ok == true);                              /* Begin accepted       */
    CHECK(g_tr.lock_calls == 1);                    /* lock enforced        */
    CHECK(g_tr.en_low == true);                     /* outputs forced safe  */
    CHECK(g_tr.pwm == 0U);
    CHECK(g_tr.demand == 0.0f);
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_ENTERING);
}

/* ==========================================================================
 *  5. Begin is REJECTED when the lock cannot be confirmed (relay stuck ON).
 * ======================================================================== */
static void test_begin_rejected_without_lock(void)
{
    mock_reset(false);
    g_tr.relay_off = false;   /* relay stuck energised -> lock never confirms */
    PedalCalSession s;
    PedalCalSession_Init(&s, NULL, NULL);

    bool ok = sim_capture_min(&s, 1000);

    CHECK(ok == false);                             /* Begin refused        */
    CHECK(g_tr.lock_calls == 1);                    /* it still tried        */
    CHECK(PedalCalSession_State(&s) == PEDAL_CAL_IDLE);
    CHECK((PedalCalSession_Reason(&s) &
           PEDAL_CAL_BLOCK_LOCK_NOT_CONFIRMED) != 0U);
}

/* ==========================================================================
 *  6. An active session re-applies the lock on EVERY tick.
 * ======================================================================== */
static void test_active_session_locks_each_tick(void)
{
    PedalCalSession s;
    uint32_t t = 1000;
    begin_active(&s, &t);                           /* already ran some ticks */

    int before = g_tr.lock_calls;
    const int N = 5;
    for (int i = 0; i < N; i++) {
        (void)sim_capture_tick(&s, t);
        t += 50;
    }
    CHECK(g_tr.lock_calls == before + N);           /* one lock per tick     */
    CHECK(PedalCalSession_Active(&s));
}

/* ==========================================================================
 *  7. Losing the lock mid-session still aborts with ABORT_LOCK_LOST.
 * ======================================================================== */
static void test_lock_loss_aborts(void)
{
    PedalCalSession s;
    uint32_t t = 1000;
    begin_active(&s, &t);

    /* Relay energises externally: even though CalibrationLock() still forces
     * EN LOW / PWM 0, the relay-off precondition fails, so traction_locked is
     * false and the session must abort. */
    g_tr.relay_off = false;
    PedalCalState st = sim_capture_tick(&s, t);

    CHECK(st == PEDAL_CAL_ABORTED);
    CHECK((PedalCalSession_Reason(&s) & PEDAL_CAL_ABORT_LOCK_LOST) != 0U);
}

int main(void)
{
    test_query_active_readonly();
    test_query_standby_no_lock();
    test_session_status_no_side_effects();
    test_capture_min_runs_lock();
    test_begin_rejected_without_lock();
    test_active_session_locks_each_tick();
    test_lock_loss_aborts();
    printf("pedalcal_query_readonly: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#else
int main(void) { return 0; }
#endif /* HOST_TEST */
