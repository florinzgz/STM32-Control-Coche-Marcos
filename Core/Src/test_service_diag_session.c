/**
  ****************************************************************************
  * @file    test_service_diag_session.c
  * @brief   Host unit tests for the SERVICE_DIAG session pure decision core
  *          (service_diag_session.c) — Bloque A, Hito 1, PR #445.
  *
  *          Compile with host GCC (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -Wall -Wextra -Werror \
  *                Core/Src/test_service_diag_session.c \
  *                Core/Src/service_diag_session.c -lm \
  *                -o /tmp/test_service_diag_session
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <string.h>

#include "service_diag_session.h"

static int tests_run = 0, tests_failed = 0;

#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* Advance the FSM clock from c->now_ms by total_ms, ticking Update() every
 * <= 50 ms (well under SVCDIAG_WATCHDOG_MAX_GAP_MS = 100 ms) instead of one
 * single jump. Root cause of the original 9 failures: several tests moved
 * c->now_ms by several hundred ms in ONE call while the channel was
 * STEPPING; the watchdog (correctly) saw that as a stale gap and aborted
 * with SVCDIAG_REASON_WATCHDOG instead of letting the plateau/timeout logic
 * run. Feeding the tick at a realistic (<=50 ms) cadence, exactly like the
 * real main loop is required to (T3.c, <=100 ms), avoids tripping it while
 * still exercising the same code path. Returns the state after the LAST
 * tick (total_ms == 0 returns the current state without ticking). */
static ServiceDiagState_t advance_ms(ServiceDiagSession *s, ServiceDiagConds *c,
                                      uint32_t total_ms)
{
    uint32_t target = c->now_ms + total_ms;
    ServiceDiagState_t st = ServiceDiagSession_State(s);
    while (c->now_ms != target) {
        uint32_t remaining = target - c->now_ms;
        uint32_t step = (remaining > 50U) ? 50U : remaining;
        c->now_ms += step;
        st = ServiceDiagSession_Update(s, c);
    }
    return st;
}

/* Raw SystemState_t values mirrored here (this module never includes
 * safety_system.h — it treats the state as an opaque byte). */
#define SYS_RAW_BOOT       0U
#define SYS_RAW_STANDBY    1U
#define SYS_RAW_ACTIVE     2U
#define SYS_RAW_DEGRADED   3U
#define SYS_RAW_SAFE       4U
#define SYS_RAW_ERROR      5U
#define SYS_RAW_LIMP_HOME  6U

/* Baseline "safe to enter" conditions: vehicle suspended, P/N, stationary,
 * confirmed, battery healthy. */
static ServiceDiagConds base_conds(uint32_t t, uint8_t sys_raw)
{
    ServiceDiagConds c;
    memset(&c, 0, sizeof(c));
    c.now_ms                   = t;
    c.boot_state                = (sys_raw == SYS_RAW_BOOT);
    c.gear_park_or_neutral      = true;
    c.wheels_stationary_1s      = true;
    c.confirm_token_ok          = true;
    c.battery_above_cutoff      = true;
    c.system_state_raw          = sys_raw;
    c.battery_above_warning     = true;
    c.can_rx_age_ms             = 0U;
    c.grounded_wheel_pulse      = false;
    c.test_overcurrent          = false;
    c.steering_pwm_ceiling_pct  = 45U;
    return c;
}

/* ---- Entry from every allowed state, and rejection from BOOT --------- */
static void test_entry_from_all_allowed_states(void)
{
    const uint8_t allowed[] = { SYS_RAW_STANDBY, SYS_RAW_ACTIVE, SYS_RAW_DEGRADED,
                                 SYS_RAW_SAFE, SYS_RAW_ERROR, SYS_RAW_LIMP_HOME };
    for (size_t i = 0; i < sizeof(allowed) / sizeof(allowed[0]); i++) {
        ServiceDiagSession s;
        ServiceDiagSession_Init(&s, NULL);
        ServiceDiagConds c = base_conds(1000U, allowed[i]);
        bool ok = ServiceDiagSession_Begin(&s, &c);
        CHECK(ok);
        CHECK(ServiceDiagSession_State(&s) == SVCDIAG_STATE_ENTERING);
        CHECK(ServiceDiagSession_OriginState(&s) == allowed[i]);
        CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_NONE);

        /* One Update() settles ENTERING -> ARMED without disturbing origin. */
        ServiceDiagState_t st = ServiceDiagSession_Update(&s, &c);
        CHECK(st == SVCDIAG_STATE_ARMED);
        CHECK(ServiceDiagSession_OriginState(&s) == allowed[i]);

        /* Abort restores nothing in the real machine (nothing was ever
         * mutated) — verify the module's own bookkeeping reflects a clean
         * terminal state and the recorded origin is unchanged (i.e. exact
         * "restoration" of what this module ever touched). */
        ServiceDiagSession_Abort(&s, SVCDIAG_REASON_OPERATOR);
        CHECK(ServiceDiagSession_State(&s) == SVCDIAG_STATE_ABORTED);
        CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_OPERATOR);
        CHECK(ServiceDiagSession_OriginState(&s) == allowed[i]);
        CHECK(!ServiceDiagSession_Active(&s));
        CHECK(ServiceDiagSession_ActiveChannel(&s) == SVCDIAG_CH_NONE);
        CHECK(ServiceDiagSession_ActivePwmPct(&s) == 0U);
    }
}

static void test_entry_blocked_from_boot(void)
{
    ServiceDiagSession s;
    ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(1000U, SYS_RAW_BOOT);
    bool ok = ServiceDiagSession_Begin(&s, &c);
    CHECK(!ok);
    CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_BOOT_STATE);
    CHECK(ServiceDiagSession_State(&s) == SVCDIAG_STATE_IDLE);
    CHECK(!ServiceDiagSession_Active(&s));
}

/* ---- Every other entry gate individually ------------------------------ */
static void test_entry_blocks_individually(void)
{
    struct { void (*mutate)(ServiceDiagConds *); ServiceDiagReason_t expect; } cases[8];
    (void)cases;

    { /* gear not P/N */
        ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
        ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
        c.gear_park_or_neutral = false;
        CHECK(!ServiceDiagSession_Begin(&s, &c));
        CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_GEAR);
    }
    { /* wheels moving */
        ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
        ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
        c.wheels_stationary_1s = false;
        CHECK(!ServiceDiagSession_Begin(&s, &c));
        CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_WHEELS_MOVING);
    }
    { /* confirm token missing */
        ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
        ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
        c.confirm_token_ok = false;
        CHECK(!ServiceDiagSession_Begin(&s, &c));
        CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_NOT_CONFIRMED);
    }
    { /* battery below cutoff */
        ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
        ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
        c.battery_above_cutoff = false;
        CHECK(!ServiceDiagSession_Begin(&s, &c));
        CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_BATTERY_LOW);
    }
    { /* already active: begin twice */
        ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
        ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
        CHECK(ServiceDiagSession_Begin(&s, &c));
        ServiceDiagState_t prior = ServiceDiagSession_State(&s);
        CHECK(!ServiceDiagSession_Begin(&s, &c));
        CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_ALREADY_ACTIVE);
        /* Running session is not disturbed by the rejected re-entry. */
        CHECK(ServiceDiagSession_State(&s) == prior);
    }
}

/* ---- battery gap: entry uses cutoff (looser), running uses warning
 * (stricter) — a battery between the two aborts on the very first tick. */
static void test_battery_entry_vs_running_gap(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    c.battery_above_cutoff  = true;   /* passes entry */
    c.battery_above_warning = false;  /* fails the stricter running check */
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ABORTED);
    CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_BATTERY_WARN);
}

/* ---- PWM ceiling: never exceeds the absolute 60% max ------------------ */
static void test_pwm_ceiling_default(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);

    /* Ask for 100% -- must clamp to the default ceiling (50%). */
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FL, SVCDIAG_DIR_FORWARD,
                                          100U, 1500U));
    CHECK(ServiceDiagSession_ActivePwmPct(&s) == SVCDIAG_PWM_DEFAULT_CEILING_PCT);
}

static void test_pwm_ceiling_corrupt_cfg_still_clamped(void)
{
    ServiceDiagSession s;
    ServiceDiagCfg cfg;
    cfg.wheel_pwm_ceiling_pct = 200U;   /* corrupt/bogus config value */
    ServiceDiagSession_Init(&s, &cfg);
    /* Init() must defensively clamp regardless of what was requested. */
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_RR, SVCDIAG_DIR_FORWARD,
                                          255U, 2000U));
    CHECK(ServiceDiagSession_ActivePwmPct(&s) <= SVCDIAG_PWM_ABS_MAX_PCT);
}

static void test_pwm_ceiling_steering_corrupt_conds_still_clamped(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    c.steering_pwm_ceiling_pct = 255U;  /* corrupt/huge runtime value */
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_STEERING, SVCDIAG_DIR_FORWARD,
                                          255U, 500U));
    CHECK(ServiceDiagSession_ActivePwmPct(&s) <= SVCDIAG_PWM_ABS_MAX_PCT);
}

static void test_pwm_within_ceiling_not_clamped(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FL, SVCDIAG_DIR_FORWARD,
                                          25U, 1500U));
    CHECK(ServiceDiagSession_ActivePwmPct(&s) == 25U);
}

/* ---- Never two actuators at once -------------------------------------- */
static void test_single_actuator_at_a_time(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FL, SVCDIAG_DIR_FORWARD,
                                          25U, 1500U));
    CHECK(ServiceDiagSession_ActiveChannel(&s) == SVCDIAG_CH_FL);

    /* A second request while STEPPING must be rejected outright. */
    bool second = ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FR, SVCDIAG_DIR_FORWARD,
                                                   25U, 1500U);
    CHECK(!second);
    CHECK(ServiceDiagSession_ActiveChannel(&s) == SVCDIAG_CH_FL);
}

/* ---- Dead-time between steps is mandatory ----------------------------- */
static void test_dead_time_enforced_between_steps(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FL, SVCDIAG_DIR_FORWARD,
                                          25U, 500U));

    /* Advance time past the plateau in <=50 ms slices (advance_ms) -- a
     * single 510 ms jump while STEPPING would itself be a stale gap and
     * (correctly) trip SVCDIAG_REASON_WATCHDOG instead of letting the
     * plateau complete normally. */
    CHECK(advance_ms(&s, &c, 500U + 10U) == SVCDIAG_STATE_DEADTIME);
    CHECK(ServiceDiagSession_ActivePwmPct(&s) == 0U);   /* coast, not braking */

    /* Immediately requesting the next step (dead-time not yet elapsed) must
     * be rejected -- state is not ARMED. */
    CHECK(!ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FR, SVCDIAG_DIR_FORWARD,
                                           25U, 500U));

    /* After the dead-time elapses, ARMED resumes and the next step works. */
    CHECK(advance_ms(&s, &c, SVCDIAG_DEAD_TIME_MS + 5U) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FR, SVCDIAG_DIR_FORWARD,
                                          25U, 500U));
    CHECK(ServiceDiagSession_ActiveChannel(&s) == SVCDIAG_CH_FR);
}

/* ---- Step completes PASS when plateau elapses without incident -------- */
static void test_step_pass_on_plateau_elapsed(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_RL, SVCDIAG_DIR_REVERSE,
                                          50U, 300U));
    CHECK(advance_ms(&s, &c, 300U) == SVCDIAG_STATE_DEADTIME);
    CHECK(ServiceDiagSession_StepVerdict(&s) == SVCDIAG_STEP_PASS);
}

/* ---- Grounded wheel pulse (car resting on wheels) aborts immediately -- */
static void test_grounded_wheel_pulse_aborts(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FL, SVCDIAG_DIR_FORWARD,
                                          25U, 1500U));

    c.now_ms = 50U;
    c.grounded_wheel_pulse = true;
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ABORTED);
    CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_GROUNDED_WHEEL);
    CHECK(ServiceDiagSession_ActivePwmPct(&s) == 0U);
}

/* ---- Test overcurrent aborts on a SINGLE sample, fails the step, and
 * NEVER escalates to the real safety machine (structurally true: this
 * module has zero references to Safety_SetError/Safety_SetState). -------- */
static void test_overcurrent_single_sample_fail_and_abort(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FR, SVCDIAG_DIR_FORWARD,
                                          50U, 1500U));

    c.now_ms = 20U;               /* well within the plateau */
    c.test_overcurrent = true;    /* single sample, first occurrence */
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ABORTED);
    CHECK(ServiceDiagSession_StepVerdict(&s) == SVCDIAG_STEP_FAIL_OVERCURRENT);
    CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_OVERCURRENT);
    CHECK(ServiceDiagSession_ActivePwmPct(&s) == 0U);
}

/* ---- Abort on leaving P/N mid-session --------------------------------- */
static void test_abort_on_gear_leaving_park_neutral(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);

    c.now_ms = 10U;
    c.gear_park_or_neutral = false;
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ABORTED);
    CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_GEAR);
}

/* ---- Abort on CAN heartbeat loss > 500 ms ------------------------------ */
static void test_abort_on_can_heartbeat_loss(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);

    c.now_ms = 10U;
    c.can_rx_age_ms = SVCDIAG_CAN_HEARTBEAT_ABORT_MS + 1U;
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ABORTED);
    CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_CAN_LOSS);
}

/* ---- Real vehicle state changing mid-session aborts (catch-all) ------- */
static void test_abort_on_state_changed(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_DEGRADED);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);

    c.now_ms = 10U;
    c.system_state_raw = SYS_RAW_SAFE;  /* escalated while testing */
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ABORTED);
    CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_STATE_CHANGED);
}

/* ---- Session-wide timeout ---------------------------------------------- */
static void test_session_timeout(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);

    c.now_ms = SVCDIAG_SESSION_TIMEOUT_MS + 1U;
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ABORTED);
    CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_SESSION_TIMEOUT);
}

/* ---- Per-step hard timeout is a real, reachable safety net (white-box:
 * force an artificially large plateau to prove the 4000 ms cap still
 * intervenes even if the plateau clamp were ever bypassed upstream). ----- */
static void test_step_hard_timeout_backstop(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FL, SVCDIAG_DIR_FORWARD,
                                          25U, SVCDIAG_STEP_PLATEAU_MAX_MS));
    /* Simulate a stuck plateau value (defense-in-depth backstop test). */
    s.active_plateau_ms = 65535U;  /* uint16_t max, still >> STEP_TIMEOUT_MS */

    CHECK(advance_ms(&s, &c, SVCDIAG_STEP_TIMEOUT_MS + 1U) == SVCDIAG_STATE_ABORTED);
    CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_STEP_TIMEOUT);
    CHECK(ServiceDiagSession_ActivePwmPct(&s) == 0U);
}

/* ---- Watchdog: a stale Update() gap while STEPPING forces PWM=0 and
 * aborts instead of silently resuming. ----------------------------------- */
static void test_watchdog_forces_pwm_zero(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    c.now_ms = 5U;
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FL, SVCDIAG_DIR_FORWARD,
                                          25U, SVCDIAG_STEP_PLATEAU_MAX_MS));
    CHECK(ServiceDiagSession_ActivePwmPct(&s) == 25U);

    /* Simulate the task not refreshing its tick for longer than the
     * watchdog gap, but still well inside the plateau window. */
    c.now_ms = 5U + SVCDIAG_WATCHDOG_MAX_GAP_MS + 50U;
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ABORTED);
    CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_WATCHDOG);
    CHECK(ServiceDiagSession_ActivePwmPct(&s) == 0U);
}

/* ---- The advance_ms() helper (added to fix the 9 original failures) must
 * NEVER be read as "the watchdog got relaxed for tests". This test does the
 * OPPOSITE of what advance_ms does: it feeds a SINGLE raw jump > 100 ms
 * (SVCDIAG_WATCHDOG_MAX_GAP_MS) directly to Update() while a channel is
 * STEPPING, bypassing the helper entirely -- exactly what a stalled/blocked
 * caller task would look like. It must still abort with the watchdog
 * reason and force the driven channel/PWM to 0, and must stay aborted
 * (never silently resume) on the following tick. Uses the boundary value
 * (gap == MAX_GAP_MS + 1, the smallest gap that must already be treated as
 * stale) and a different channel/direction than test_watchdog_forces_pwm_zero
 * for independent coverage. ------------------------------------------------ */
static void test_watchdog_not_defeated_by_single_raw_jump(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_STEERING, SVCDIAG_DIR_REVERSE,
                                          40U, SVCDIAG_STEP_PLATEAU_MAX_MS));
    CHECK(ServiceDiagSession_ActiveChannel(&s) == SVCDIAG_CH_STEERING);
    CHECK(ServiceDiagSession_ActivePwmPct(&s) == 40U);

    /* Single raw jump of exactly MAX_GAP_MS + 1 ms -- NOT via advance_ms --
     * well inside the 2000 ms plateau and the 4000 ms step timeout, so only
     * the watchdog (not the plateau/timeout logic) can explain the abort. */
    c.now_ms += SVCDIAG_WATCHDOG_MAX_GAP_MS + 1U;
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ABORTED);
    CHECK(ServiceDiagSession_Reason(&s) == SVCDIAG_REASON_WATCHDOG);
    CHECK(ServiceDiagSession_ActiveChannel(&s) == SVCDIAG_CH_NONE);   /* every PWM -> 0 */
    CHECK(ServiceDiagSession_ActivePwmPct(&s) == 0U);

    /* Must not silently resume on a later, normal-cadence tick. */
    c.now_ms += 10U;
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ABORTED);
    CHECK(ServiceDiagSession_ActivePwmPct(&s) == 0U);
}

/* ---- RequestStep input validation -------------------------------------- */
static void test_request_step_rejects_bad_inputs(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);

    /* Bogus channel (beyond STEERING, or NONE requested explicitly). */
    CHECK(!ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_NONE, SVCDIAG_DIR_FORWARD, 25U, 500U));
    CHECK(!ServiceDiagSession_RequestStep(&s, &c, (ServiceDiagChannel_t)99, SVCDIAG_DIR_FORWARD, 25U, 500U));
    /* Bogus direction. */
    CHECK(!ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FL, (ServiceDiagDirection_t)7, 25U, 500U));
    /* State still ARMED, nothing accepted. */
    CHECK(ServiceDiagSession_State(&s) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_ActiveChannel(&s) == SVCDIAG_CH_NONE);
}

static void test_request_step_rejected_while_idle_or_aborted(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    /* IDLE: never began. */
    CHECK(!ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FL, SVCDIAG_DIR_FORWARD, 25U, 500U));

    CHECK(ServiceDiagSession_Begin(&s, &c));
    ServiceDiagSession_Abort(&s, SVCDIAG_REASON_OPERATOR);
    CHECK(ServiceDiagSession_State(&s) == SVCDIAG_STATE_ABORTED);
    CHECK(!ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FL, SVCDIAG_DIR_FORWARD, 25U, 500U));
}

/* ---- Plateau clamping to [MIN,MAX] ------------------------------------- */
static void test_plateau_clamped_to_bounds(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FL, SVCDIAG_DIR_FORWARD, 25U, 1U));
    /* Requested 1 ms -> clamped up to the minimum. */
    c.now_ms = SVCDIAG_STEP_PLATEAU_MIN_MS - 1U;
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_STEPPING); /* not yet elapsed */
    c.now_ms = SVCDIAG_STEP_PLATEAU_MIN_MS;
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_DEADTIME);
}

static void test_plateau_clamped_to_max(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FL, SVCDIAG_DIR_FORWARD,
                                          25U, 65535U));
    /* Even though 65535 ms was requested, it must have been clamped to
     * SVCDIAG_STEP_PLATEAU_MAX_MS (2000), well inside the 4000 ms hard cap,
     * so it completes as PASS, not STEP_TIMEOUT. */
    CHECK(advance_ms(&s, &c, SVCDIAG_STEP_PLATEAU_MAX_MS) == SVCDIAG_STATE_DEADTIME);
    CHECK(ServiceDiagSession_StepVerdict(&s) == SVCDIAG_STEP_PASS);
}

/* ---- Progress / elapsed helpers ---------------------------------------- */
static void test_progress_and_elapsed(void)
{
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(ServiceDiagSession_ProgressPct(&s, 0U) == 0U);  /* IDLE */
    CHECK(ServiceDiagSession_Begin(&s, &c));
    CHECK(ServiceDiagSession_Update(&s, &c) == SVCDIAG_STATE_ARMED);
    CHECK(ServiceDiagSession_ProgressPct(&s, 0U) == 0U);  /* ARMED, no step */
    CHECK(ServiceDiagSession_RequestStep(&s, &c, SVCDIAG_CH_FL, SVCDIAG_DIR_FORWARD,
                                          25U, 1000U));
    CHECK(ServiceDiagSession_ProgressPct(&s, 0U) == 0U);
    CHECK(ServiceDiagSession_ProgressPct(&s, 500U) == 50U);
    CHECK(ServiceDiagSession_ProgressPct(&s, 1000U) == 100U);
    CHECK(ServiceDiagSession_ProgressPct(&s, 5000U) == 100U);  /* saturates */

    CHECK(ServiceDiagSession_ElapsedSec(&s, 0U) == 0U);
    CHECK(ServiceDiagSession_ElapsedSec(&s, 3000U) == 3U);
    CHECK(ServiceDiagSession_ElapsedSec(&s, 999999U) == 255U); /* saturates */
}

/* ---- HMI text sanity ---------------------------------------------------- */
static void test_state_and_reason_text(void)
{
    CHECK(strcmp(ServiceDiagSession_StateText(SVCDIAG_STATE_IDLE), "INACTIVO") == 0);
    CHECK(strcmp(ServiceDiagSession_StateText(SVCDIAG_STATE_ARMED), "LISTO") == 0);
    CHECK(strcmp(ServiceDiagSession_StateText(SVCDIAG_STATE_STEPPING), "EJECUTANDO") == 0);
    CHECK(strcmp(ServiceDiagSession_ReasonText(SVCDIAG_REASON_NONE), "OK") == 0);
    CHECK(strcmp(ServiceDiagSession_ReasonText(SVCDIAG_REASON_OVERCURRENT),
                 "SOBRECORRIENTE DE PRUEBA") == 0);
    CHECK(strcmp(ServiceDiagSession_ReasonText(SVCDIAG_REASON_WATCHDOG), "WATCHDOG") == 0);
}

/* ---- NULL safety -------------------------------------------------------- */
static void test_null_safety(void)
{
    ServiceDiagConds c = base_conds(0U, SYS_RAW_STANDBY);
    CHECK(!ServiceDiagSession_Begin(NULL, &c));
    ServiceDiagSession s; ServiceDiagSession_Init(&s, NULL);
    CHECK(!ServiceDiagSession_Begin(&s, NULL));
    CHECK(ServiceDiagSession_Update(NULL, &c) == SVCDIAG_STATE_IDLE);
    CHECK(ServiceDiagSession_Update(&s, NULL) == ServiceDiagSession_State(&s));
    CHECK(!ServiceDiagSession_RequestStep(NULL, &c, SVCDIAG_CH_FL, SVCDIAG_DIR_FORWARD, 25U, 500U));
    ServiceDiagSession_Abort(NULL, SVCDIAG_REASON_OPERATOR);  /* must not crash */
    CHECK(ServiceDiagSession_ProgressPct(NULL, 0U) == 0U);
    CHECK(ServiceDiagSession_ElapsedSec(NULL, 0U) == 0U);
}

int main(void)
{
    test_entry_from_all_allowed_states();
    test_entry_blocked_from_boot();
    test_entry_blocks_individually();
    test_battery_entry_vs_running_gap();
    test_pwm_ceiling_default();
    test_pwm_ceiling_corrupt_cfg_still_clamped();
    test_pwm_ceiling_steering_corrupt_conds_still_clamped();
    test_pwm_within_ceiling_not_clamped();
    test_single_actuator_at_a_time();
    test_dead_time_enforced_between_steps();
    test_step_pass_on_plateau_elapsed();
    test_grounded_wheel_pulse_aborts();
    test_overcurrent_single_sample_fail_and_abort();
    test_abort_on_gear_leaving_park_neutral();
    test_abort_on_can_heartbeat_loss();
    test_abort_on_state_changed();
    test_session_timeout();
    test_step_hard_timeout_backstop();
    test_watchdog_forces_pwm_zero();
    test_watchdog_not_defeated_by_single_raw_jump();
    test_request_step_rejects_bad_inputs();
    test_request_step_rejected_while_idle_or_aborted();
    test_plateau_clamped_to_bounds();
    test_plateau_clamped_to_max();
    test_progress_and_elapsed();
    test_state_and_reason_text();
    test_null_safety();

    printf("service_diag_session: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
#else
int main(void) { return 0; }
#endif
