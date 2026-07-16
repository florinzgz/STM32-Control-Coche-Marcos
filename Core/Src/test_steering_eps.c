/**
  ****************************************************************************
  * @file    test_steering_eps.c
  * @brief   Host unit tests for the EPS local isolation authority
  *          (steering_eps.c).  Links the REAL production module and stubs
  *          only its two hardware side effects (Steering_Neutralize and
  *          Steering_SteerPowerOff) so the tests assert the exact motor /
  *          rail / owner / state outcome demanded by the acceptance
  *          criteria:
  *
  *              PA6 == 0, PA7 == 0, PC4 == LOW, PC12 == OFF,
  *              owner == NONE, EPS == MECHANICAL_ONLY
  *
  *          while a simulated traction chain (system state, demand, relays,
  *          power-ready) stays completely unchanged.
  *
  *          Compile (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE -ICore/Inc -O2 \
  *                Core/Src/test_steering_eps.c Core/Src/steering_eps.c \
  *                -lm -o /tmp/test_steering_eps && /tmp/test_steering_eps
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "steering_eps.h"

static int tests_run = 0, tests_failed = 0;

#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ==================================================================
 *  Fake steering-motor "hardware" model.
 *
 *  The three motor lines the acceptance criteria call out plus the
 *  steering rail:
 *    PA6 = RPWM_STEER (TIM3_CH1), PA7 = LPWM_STEER (TIM3_CH2),
 *    PC4 = EN_STEER, PC12 = steering actuator rail.
 * ================================================================== */
typedef struct {
    uint16_t pa6;         /* CCR PA6 (0 = no drive)  */
    uint16_t pa7;         /* CCR PA7 (0 = no drive)  */
    bool     pc4_high;    /* EN_STEER level          */
    bool     pc12_on;     /* steering rail energised */
    int      neutralize_calls;
    int      poweroff_calls;
} SteerHw_t;

static SteerHw_t hw;

/* Production steering_eps.c delegates PA6=PA7=0 + PC4 LOW here. */
void Steering_PhysicalOff(void)
{
    hw.pa6 = 0U;
    hw.pa7 = 0U;
    hw.pc4_high = false;
    hw.neutralize_calls++;
}

/* Production steering_eps.c delegates PC12 OFF here. */
void Steering_SteerPowerOff(void)
{
    hw.pc12_on = false;
    hw.poweroff_calls++;
}

/* ==================================================================
 *  Simulated traction chain — MUST stay untouched by EPS isolation.
 *  steering_eps.c never references any of these; the test asserts they
 *  are unchanged to make the "no regression on traction" contract
 *  explicit and self-documenting.
 * ================================================================== */
typedef struct {
    int  system_state;      /* 2 == ACTIVE */
    float demand_pct;
    bool relays_on;
    bool power_ready;
    int  gear;
} Traction_t;

static Traction_t trac;

static void hw_energise_for_active(void)
{
    /* Simulate a healthy running EPS: motor driving, rail on, EN high. */
    hw.pa6 = 1200U;
    hw.pa7 = 0U;
    hw.pc4_high = true;
    hw.pc12_on = true;
    hw.neutralize_calls = 0;
    hw.poweroff_calls = 0;
}

static void reset_all(void)
{
    hw = (SteerHw_t){0};
    trac = (Traction_t){ .system_state = 2 /*ACTIVE*/, .demand_pct = 42.0f,
                         .relays_on = true, .power_ready = true, .gear = 1 };
    Steering_EpsInit();
}

/* Assert the full isolated-steering invariant. */
static void check_isolated(void)
{
    CHECK(hw.pa6 == 0U);
    CHECK(hw.pa7 == 0U);
    CHECK(hw.pc4_high == false);
    CHECK(hw.pc12_on == false);
    CHECK(Steering_EpsGetOwner() == STEER_OWNER_NONE);
    CHECK(Steering_GetEpsState() == EPS_STATE_MECHANICAL_ONLY);
    CHECK(Steering_IsMechanicalOnly() == true);
    CHECK(Steering_EpsIsAvailable() == false);
}

/* Assert the traction chain is exactly as before isolation. */
static void check_traction_untouched(void)
{
    CHECK(trac.system_state == 2 /*ACTIVE*/);
    CHECK(trac.demand_pct == 42.0f);
    CHECK(trac.relays_on == true);
    CHECK(trac.power_ready == true);
    CHECK(trac.gear == 1);
}

/* ==================================================================
 *  Tests
 * ================================================================== */

static void test_healthy_init(void)
{
    reset_all();
    CHECK(Steering_GetEpsState() == EPS_STATE_STARTING);
    CHECK(Steering_GetEpsFault() == EPS_FAULT_NONE);
    CHECK(Steering_EpsGetOwner() == STEER_OWNER_NONE);
    CHECK(Steering_IsMechanicalOnly() == false);
    CHECK(Steering_EpsIsAvailable() == true);

    /* Healthy progression STARTING → CALIBRATING → ACTIVE with a CONTROLLED
     * ownership hand-off (release to NONE before switching active writers). */
    Steering_EpsSetHealthyState(EPS_STATE_CALIBRATING);
    CHECK(Steering_GetEpsState() == EPS_STATE_CALIBRATING);
    Steering_EpsSetOwner(STEER_OWNER_CENTERING);
    CHECK(Steering_EpsGetOwner() == STEER_OWNER_CENTERING);
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);
    CHECK(Steering_GetEpsState() == EPS_STATE_ACTIVE);
    Steering_EpsSetOwner(STEER_OWNER_NONE);          /* explicit release      */
    Steering_EpsSetOwner(STEER_OWNER_EPS);           /* then acquire          */
    CHECK(Steering_EpsGetOwner() == STEER_OWNER_EPS);
    CHECK(Steering_GetEpsFault() == EPS_FAULT_NONE); /* no conflict raised    */
}

/* Table-driven: every isolable cause must produce the same safe outcome,
 * leave traction untouched, and latch the first cause. */
static void test_all_isolable_faults(void)
{
    static const EpsFaultReason_t reasons[] = {
        EPS_FAULT_CENTER_NOT_FOUND,   /* PB5 not found / centering timeout   */
        EPS_FAULT_PB5,                /* PB5 stuck / incoherent              */
        EPS_FAULT_ENCODER_AB,         /* encoder A / B defective             */
        EPS_FAULT_ENCODER_Z,          /* encoder Z incoherent                */
        EPS_FAULT_POSITION_MISMATCH,  /* PB5/Z mismatch / impossible pos.    */
        EPS_FAULT_CALIBRATION_INVALID,/* calibration missing / corrupt / CRC */
        EPS_FAULT_PARAMETERS_INVALID, /* NaN / Inf / out-of-range params     */
        EPS_FAULT_CH5_MISSING,        /* INA226 CH5 absent                   */
        EPS_FAULT_CH5_STALE,          /* INA226 CH5 stale                    */
        EPS_FAULT_CH5_CONFIG,         /* INA226 CH5 config invalid           */
        EPS_FAULT_DIRECTION_POLARITY, /* assist pushing the wrong way        */
        EPS_FAULT_OSCILLATION,        /* sustained oscillation               */
        EPS_FAULT_DRIVER,             /* recoverable BTS7960 driver fault    */
        EPS_FAULT_POWER_CONFIRMATION, /* steering rail confirmation absent   */
        EPS_FAULT_OWNER_CONFLICT,     /* CENTERING vs EPS owner conflict     */
    };
    const size_t n = sizeof(reasons) / sizeof(reasons[0]);
    for (size_t i = 0; i < n; ++i) {
        reset_all();
        hw_energise_for_active();
        Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);
        Steering_EpsSetOwner(STEER_OWNER_EPS);

        Steering_DisableAssistFault(reasons[i]);

        check_isolated();
        CHECK(Steering_GetEpsFault() == reasons[i]);   /* cause is reported  */
        check_traction_untouched();
    }
}

/* Idempotency: repeated calls neither generate pulses nor re-energise. */
static void test_idempotent_no_flapping(void)
{
    reset_all();
    hw_energise_for_active();
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);

    Steering_DisableAssistFault(EPS_FAULT_ENCODER_AB);
    check_isolated();

    /* Call many more times: PA6/PA7/PC4/PC12 stay zero/off, owner NONE,
     * state MECHANICAL_ONLY, and the first cause is preserved. */
    for (int i = 0; i < 10; ++i) {
        Steering_DisableAssistFault(EPS_FAULT_OVERCURRENT); /* different cause */
        check_isolated();
        CHECK(Steering_GetEpsFault() == EPS_FAULT_ENCODER_AB); /* first kept  */
        /* PC12 can never be re-energised by the isolation path. */
        CHECK(hw.pc12_on == false);
    }
    /* poweroff/neutralize were invoked each time (safe zero-writes), but the
     * observable state never toggled — no flapping. */
    CHECK(hw.poweroff_calls >= 11);
    CHECK(hw.neutralize_calls >= 11);
}

/* Latched fault: a healthy-state request cannot resurrect the assist. */
static void test_latched_no_auto_reconnect(void)
{
    reset_all();
    hw_energise_for_active();
    Steering_DisableAssistFault(EPS_FAULT_CH5_MISSING);
    check_isolated();

    /* Simulate a "recovery" attempt: something tries to mark ACTIVE and
     * re-take ownership.  Both must be ignored while latched. */
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);
    Steering_EpsSetOwner(STEER_OWNER_EPS);
    CHECK(Steering_GetEpsState() == EPS_STATE_MECHANICAL_ONLY);
    CHECK(Steering_EpsGetOwner() == STEER_OWNER_NONE);
    CHECK(Steering_IsMechanicalOnly() == true);

    /* Only a full re-init (new power cycle) clears the latch. */
    Steering_EpsInit();
    CHECK(Steering_GetEpsState() == EPS_STATE_STARTING);
    CHECK(Steering_GetEpsFault() == EPS_FAULT_NONE);
    CHECK(Steering_IsMechanicalOnly() == false);
}

/* Owner conflict resolves to isolation with NONE owner. */
static void test_owner_conflict(void)
{
    reset_all();
    hw_energise_for_active();
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);
    Steering_EpsSetOwner(STEER_OWNER_CENTERING);   /* conflicting writer */

    Steering_DisableAssistFault(EPS_FAULT_OWNER_CONFLICT);
    check_isolated();
    CHECK(Steering_GetEpsFault() == EPS_FAULT_OWNER_CONFLICT);
    check_traction_untouched();
}

/* CH5 overcurrent that DISAPPEARS once isolated → stays mechanical-only,
 * traction continues (no escalation). */
static void test_overcurrent_disappears(void)
{
    reset_all();
    hw_energise_for_active();
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);

    Steering_DisableAssistFault(EPS_FAULT_OVERCURRENT);
    check_isolated();
    /* Current gone after isolation → NOT an electrical hazard. */
    CHECK(Steering_GetEpsState() == EPS_STATE_MECHANICAL_ONLY);
    check_traction_untouched();
}

/* CH5 overcurrent that PERSISTS after isolation → electrical hazard.
 * The motor is still isolated (PA6=PA7=0, PC4 LOW, PC12 OFF), owner NONE,
 * but the local state escalates so safety can cut general power. */
static void test_overcurrent_persists_hazard(void)
{
    reset_all();
    hw_energise_for_active();
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);

    /* Step 1: attempt isolation. */
    Steering_DisableAssistFault(EPS_FAULT_OVERCURRENT);
    /* Step 2: current verified to persist → declare hazard. */
    Steering_DeclareElectricalHazard(EPS_FAULT_OVERCURRENT);

    CHECK(hw.pa6 == 0U);
    CHECK(hw.pa7 == 0U);
    CHECK(hw.pc4_high == false);
    CHECK(hw.pc12_on == false);
    CHECK(Steering_EpsGetOwner() == STEER_OWNER_NONE);
    CHECK(Steering_GetEpsState() == EPS_STATE_ELECTRICAL_HAZARD);
    CHECK(Steering_IsElectricalHazard() == true);
    CHECK(Steering_IsMechanicalOnly() == false);  /* precise: hazard != mech  */
    CHECK(Steering_IsAssistLatchedOff() == true); /* assist still off         */
    /* A later plain assist-fault call must NOT demote the hazard. */
    Steering_DisableAssistFault(EPS_FAULT_ENCODER_AB);
    CHECK(Steering_GetEpsState() == EPS_STATE_ELECTRICAL_HAZARD);
}

/* Item 5: a direct switch between two ACTIVE writers WITHOUT an intervening
 * release to NONE is a real conflict — Steering_EpsSetOwner() must NOT silently
 * overwrite; it isolates the assist and latches EPS_FAULT_OWNER_CONFLICT. */
static void test_setowner_conflict_detected(void)
{
    reset_all();
    hw_energise_for_active();
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);

    Steering_EpsSetOwner(STEER_OWNER_EPS);          /* EPS acquires (from NONE) */
    CHECK(Steering_EpsGetOwner() == STEER_OWNER_EPS);

    /* Centering tries to grab the motor directly while EPS still holds it. */
    Steering_EpsSetOwner(STEER_OWNER_CENTERING);

    /* Not silently overwritten: the assist is isolated, owner forced NONE. */
    check_isolated();
    CHECK(Steering_GetEpsFault() == EPS_FAULT_OWNER_CONFLICT);
    check_traction_untouched();

    /* A controlled transfer (release to NONE first) is NOT a conflict. */
    reset_all();
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);
    Steering_EpsSetOwner(STEER_OWNER_CENTERING);
    Steering_EpsSetOwner(STEER_OWNER_NONE);         /* explicit release        */
    Steering_EpsSetOwner(STEER_OWNER_EPS);          /* then acquire            */
    CHECK(Steering_EpsGetOwner() == STEER_OWNER_EPS);
    CHECK(Steering_GetEpsFault() == EPS_FAULT_NONE);
    CHECK(Steering_IsAssistLatchedOff() == false);
}

/* Fault injected while traction is actively driving: outcome is isolation
 * with the traction chain fully preserved (the headline acceptance test). */
static void test_fault_while_traction_running(void)
{
    reset_all();
    hw_energise_for_active();
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);
    Steering_EpsSetOwner(STEER_OWNER_EPS);

    /* Traction is driving hard in gear D1. */
    trac.demand_pct = 80.0f;
    CHECK(trac.system_state == 2);

    Steering_DisableAssistFault(EPS_FAULT_ENCODER_AB);

    check_isolated();
    /* Traction demand/relays/power-ready/gear/state unchanged. */
    CHECK(trac.system_state == 2 /*ACTIVE*/);
    CHECK(trac.demand_pct == 80.0f);
    CHECK(trac.relays_on == true);
    CHECK(trac.power_ready == true);
    CHECK(trac.gear == 1);
}

/* PA6 and PA7 are never simultaneously non-zero after isolation. */
static void test_never_both_channels(void)
{
    reset_all();
    hw.pa6 = 900U;
    hw.pa7 = 900U;   /* pathological pre-state */
    Steering_DisableAssistFault(EPS_FAULT_DIRECTION_POLARITY);
    CHECK(!(hw.pa6 != 0U && hw.pa7 != 0U));
    CHECK(hw.pa6 == 0U && hw.pa7 == 0U);
}

int main(void)
{
    test_healthy_init();
    test_all_isolable_faults();
    test_idempotent_no_flapping();
    test_latched_no_auto_reconnect();
    test_owner_conflict();
    test_setowner_conflict_detected();
    test_overcurrent_disappears();
    test_overcurrent_persists_hazard();
    test_fault_while_traction_running();
    test_never_both_channels();

    printf("\n==== test_steering_eps: %d run, %d failed ====\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#else
int main(void) { return 0; }
#endif
