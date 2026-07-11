/**
  ****************************************************************************
  * @file    test_relay_degraded.c
  * @brief   Host regression tests for the DEGRADED / LIMP_HOME "zero movement"
  *          hypothesis (leading, code-confirmed).
  *
  *          THIS IS AN INSTRUMENTATION / REGRESSION COMMIT ONLY.  It does NOT
  *          alter any safety policy.  It locks in — as executable
  *          documentation — the deterministic reason the vehicle produces zero
  *          traction PWM when it enters DEGRADED or LIMP_HOME directly from
  *          STANDBY (i.e. without ever passing through ACTIVE).
  *
  *          Leading hypothesis (verify on hardware with 0x315 MOTION_INHIBIT):
  *
  *            - Safety_SetSystemState(): the STANDBY→DEGRADED transition does
  *              NOT call Relay_PowerUp() (Core/Src/safety_system.c:610-617),
  *              so the relay sequencer stays RELAY_SEQ_IDLE and
  *              Safety_IsPowerReady() is false.
  *            - The STANDBY→LIMP_HOME transition DOES call Relay_PowerUp()
  *              (Core/Src/safety_system.c:642-669), which starts the sequencer
  *              at RELAY_SEQ_TRACTION_ON, but Relay_SequencerUpdate() only
  *              advances to RELAY_SEQ_COMPLETE while system_state == ACTIVE
  *              (Core/Src/safety_system.c:864-886).  Because LIMP_HOME is not
  *              ACTIVE, the very next 10 ms sequencer tick takes the
  *              "state != ACTIVE" branch and, finding the sequencer in
  *              RELAY_SEQ_TRACTION_ON, calls Relay_PowerDown() — powering the
  *              relay back down and returning to RELAY_SEQ_IDLE.
  *            - In BOTH cases Safety_IsPowerReady() stays false, so
  *              Traction_Update() takes its power-ready early-return
  *              (Core/Src/motor_control.c:1342-1349), emits final PWM = 0, and
  *              records MOTION_INHIBIT_POWER_NOT_READY (0x0004) into 0x315.
  *
  *          The relay sequencer here is a FAITHFUL host model of the real
  *          functions in safety_system.c (cited above); the emitted
  *          MOTION_INHIBIT reason mask is produced by the REAL, unmodified
  *          MotionInhibit_Evaluate() (Core/Src/motion_inhibit.c) so the
  *          POWER_NOT_READY classification is not re-implemented.
  *
  *          The actual policy correction (if the intended contract is that
  *          DEGRADED/LIMP_HOME from STANDBY must be drivable) MUST be a
  *          separate, minimal commit made only AFTER hardware validation.
  ****************************************************************************
  */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "motion_inhibit.h"

/* ---- System-state / gear enum values (mirror the firmware headers) ------- */
#define SYS_STATE_BOOT      0
#define SYS_STATE_STANDBY   1
#define SYS_STATE_ACTIVE    2
#define SYS_STATE_DEGRADED  3
#define SYS_STATE_SAFE      4
#define SYS_STATE_ERROR     5
#define SYS_STATE_LIMP_HOME 6

#define GEAR_PARK    0
#define GEAR_REVERSE 1
#define GEAR_NEUTRAL 2
#define GEAR_FORWARD 3

/* Matches RELAY_TRACTION_SETTLE_MS in Core/Src/safety_system.c:163. */
#define RELAY_TRACTION_SETTLE_MS 50U

/* ==========================================================================
 * Faithful host model of the relay power sequencer.
 * Mirrors Core/Src/safety_system.c:305-916 exactly (see per-function cites).
 * ========================================================================== */
typedef enum {
    RELAY_SEQ_IDLE = 0,      /* All relays off, no sequence in progress   */
    RELAY_SEQ_TRACTION_ON,   /* Traction relay energised, waiting settle  */
    RELAY_SEQ_COMPLETE       /* All relays on, sequence finished          */
} RelaySeqState_t;

static RelaySeqState_t relay_seq_state     = RELAY_SEQ_IDLE;
static uint32_t        relay_seq_timestamp = 0;
static int             system_state        = SYS_STATE_BOOT;
static uint32_t        g_tick              = 0;

/* GPIO write bookkeeping — proves the traction relay was energised then de-
 * energised in the LIMP_HOME case (equivalent to Relay_PowerDown()'s BSRR). */
static int trac_relay_level = 0;   /* 1 = energised (SET), 0 = de-energised */

/* Mirror of Relay_PowerUp() — safety_system.c:811-827. */
static void Relay_PowerUp(void)
{
    if (relay_seq_state != RELAY_SEQ_IDLE) {
        return;                              /* re-entry safe */
    }
    trac_relay_level    = 1;                 /* HAL_GPIO_WritePin(...SET) */
    relay_seq_state     = RELAY_SEQ_TRACTION_ON;
    relay_seq_timestamp = g_tick;            /* HAL_GetTick() */
}

/* Mirror of Relay_PowerDown() — safety_system.c:889-911. */
static void Relay_PowerDown(void)
{
    trac_relay_level = 0;                     /* atomic BSRR reset of relays */
    relay_seq_state  = RELAY_SEQ_IDLE;
}

/* Mirror of Relay_SequencerUpdate() — safety_system.c:829-887.
 * The critical gate: only advances while system_state == ACTIVE, and forces a
 * power-down if it finds a mid-sequence (TRACTION_ON) state outside ACTIVE. */
static void Relay_SequencerUpdate(void)
{
    if (system_state != SYS_STATE_ACTIVE) {
        if (relay_seq_state == RELAY_SEQ_TRACTION_ON) {
            Relay_PowerDown();               /* never leave a partial state */
        }
        return;
    }

    switch (relay_seq_state) {
        case RELAY_SEQ_TRACTION_ON:
            if ((g_tick - relay_seq_timestamp) >= RELAY_TRACTION_SETTLE_MS) {
                relay_seq_state = RELAY_SEQ_COMPLETE;
            }
            break;
        case RELAY_SEQ_IDLE:
        case RELAY_SEQ_COMPLETE:
        default:
            break;
    }
}

/* Mirror of Safety_IsPowerReady() — safety_system.c:913-916. */
static bool Safety_IsPowerReady(void)
{
    return (relay_seq_state == RELAY_SEQ_COMPLETE);
}

/* Faithful model of the relay-relevant part of Safety_SetSystemState() for the
 * STANDBY→{DEGRADED,LIMP_HOME} transitions — safety_system.c:610-669.
 * DEGRADED does NOT touch the relays; LIMP_HOME calls Relay_PowerUp(). */
static void SetSystemState(int new_state)
{
    switch (new_state) {
        case SYS_STATE_DEGRADED:
            if (system_state == SYS_STATE_ACTIVE ||
                system_state == SYS_STATE_STANDBY) {
                system_state = SYS_STATE_DEGRADED;
                /* NOTE: no Relay_PowerUp() here (safety_system.c:610-617). */
            }
            break;
        case SYS_STATE_LIMP_HOME:
            if (system_state == SYS_STATE_STANDBY  ||
                system_state == SYS_STATE_ACTIVE   ||
                system_state == SYS_STATE_DEGRADED ||
                system_state == SYS_STATE_SAFE) {
                system_state = SYS_STATE_LIMP_HOME;
                Relay_PowerUp();             /* safety_system.c:657 */
            }
            break;
        default:
            system_state = new_state;
            break;
    }
}

/* Reset the modelled world to a clean STANDBY, relays idle. */
static void world_reset(void)
{
    relay_seq_state     = RELAY_SEQ_IDLE;
    relay_seq_timestamp = 0;
    trac_relay_level    = 0;
    system_state        = SYS_STATE_STANDBY;
    g_tick              = 0;
}

/* Compute the MOTION_INHIBIT reason mask the way Traction_Update() would after
 * its power-ready early-return: it calls Traction_UpdateMotionInhibit(0.0f, 0)
 * (motor_control.c:1346), i.e. effective demand and final PWM are forced to
 * zero while the REAL operator demand is preserved (motor_control.c:1299-1301).
 * Returns the reason mask via *reason and the modelled final PWM via *pwm. */
static uint16_t inhibit_after_power_gate(float requested_demand_pct,
                                         uint16_t *final_pwm_out)
{
    const uint16_t final_pwm = 0U;   /* power gate forces PWM to zero */
    MotionInhibitInputs in = {0};
    in.state                    = (uint8_t)system_state;
    in.power_ready              = Safety_IsPowerReady();
    in.gear                     = GEAR_FORWARD;
    in.gear_park                = GEAR_PARK;
    in.gear_neutral             = GEAR_NEUTRAL;
    in.state_safe               = SYS_STATE_SAFE;
    in.state_error              = SYS_STATE_ERROR;
    in.obstacle_forward_blocked = false;
    in.forward_gear             = true;
    in.demand_pct               = requested_demand_pct; /* operator asked to move */
    in.effective_demand_pct     = 0.0f;                 /* zeroed by power gate    */
    in.final_pwm_max            = final_pwm;            /* zeroed by power gate    */
    in.degraded_level           = 0U;
    if (final_pwm_out) *final_pwm_out = final_pwm;
    return MotionInhibit_Evaluate(&in);
}

/* ---- Minimal harness ----------------------------------------------------- */
static int tests_run    = 0;
static int tests_failed = 0;

#define CHECK(cond, msg) do {                                     \
        tests_run++;                                              \
        if (!(cond)) { tests_failed++;                            \
            printf("  FAIL: %s\n", msg); }                        \
        else { printf("  ok:   %s\n", msg); }                     \
    } while (0)

/* -----------------------------------------------------------------------------
 * Scenario A — STANDBY → DEGRADED.
 * Relay sequence remains IDLE, power_ready = false, PWM = 0,
 * MOTION_INHIBIT_POWER_NOT_READY set.
 * --------------------------------------------------------------------------- */
static void test_standby_to_degraded_no_powerup(void)
{
    printf("Scenario A: STANDBY -> DEGRADED (no Relay_PowerUp)\n");
    world_reset();

    SetSystemState(SYS_STATE_DEGRADED);
    CHECK(system_state == SYS_STATE_DEGRADED, "entered DEGRADED");
    CHECK(relay_seq_state == RELAY_SEQ_IDLE,
          "relay sequencer stays IDLE (DEGRADED never calls Relay_PowerUp)");
    CHECK(trac_relay_level == 0, "traction relay never energised");

    /* Run the 10 ms sequencer many times — it must never advance. */
    for (int i = 0; i < 50; ++i) {
        g_tick += 10;
        Relay_SequencerUpdate();
    }
    CHECK(relay_seq_state == RELAY_SEQ_IDLE,
          "sequencer never advances outside ACTIVE");
    CHECK(!Safety_IsPowerReady(), "power_ready == false in DEGRADED");

    uint16_t final_pwm = 0xFFFF;
    uint16_t reason = inhibit_after_power_gate(80.0f, &final_pwm);
    CHECK((reason & MOTION_INHIBIT_POWER_NOT_READY) != 0,
          "0x315 reports MOTION_INHIBIT_POWER_NOT_READY (0x0004)");
    CHECK((reason & MOTION_INHIBIT_DEMAND_ZEROED) != 0,
          "demand requested but zeroed before PWM (MOTION_INHIBIT_DEMAND_ZEROED)");
    CHECK(final_pwm == 0U, "final PWM is zero");
}

/* -----------------------------------------------------------------------------
 * Scenario B — STANDBY → LIMP_HOME.
 * First relay step starts (TRACTION_ON), the next sequencer tick sees
 * state != ACTIVE, powers down; power_ready = false, PWM = 0.
 * --------------------------------------------------------------------------- */
static void test_standby_to_limp_home_powers_down(void)
{
    printf("Scenario B: STANDBY -> LIMP_HOME (PowerUp then sequencer power-down)\n");
    world_reset();

    SetSystemState(SYS_STATE_LIMP_HOME);
    CHECK(system_state == SYS_STATE_LIMP_HOME, "entered LIMP_HOME");
    CHECK(relay_seq_state == RELAY_SEQ_TRACTION_ON,
          "Relay_PowerUp started the first step (TRACTION_ON)");
    CHECK(trac_relay_level == 1, "traction relay momentarily energised");
    CHECK(!Safety_IsPowerReady(), "not power-ready mid-sequence");

    /* The very next 10 ms sequencer tick runs outside ACTIVE. */
    g_tick += 10;
    Relay_SequencerUpdate();
    CHECK(relay_seq_state == RELAY_SEQ_IDLE,
          "sequencer powered down (state != ACTIVE, mid-sequence)");
    CHECK(trac_relay_level == 0, "traction relay de-energised again");
    CHECK(!Safety_IsPowerReady(), "power_ready == false after power-down");

    /* Even waiting past the settle window can never complete the sequence. */
    for (int i = 0; i < 50; ++i) {
        g_tick += 10;
        Relay_SequencerUpdate();
    }
    CHECK(relay_seq_state == RELAY_SEQ_IDLE,
          "sequence can never complete outside ACTIVE");

    uint16_t final_pwm = 0xFFFF;
    uint16_t reason = inhibit_after_power_gate(80.0f, &final_pwm);
    CHECK((reason & MOTION_INHIBIT_POWER_NOT_READY) != 0,
          "0x315 reports MOTION_INHIBIT_POWER_NOT_READY (0x0004)");
    CHECK((reason & MOTION_INHIBIT_DEMAND_ZEROED) != 0,
          "demand requested but zeroed before PWM (MOTION_INHIBIT_DEMAND_ZEROED)");
    CHECK(final_pwm == 0U, "final PWM is zero");
}

/* -----------------------------------------------------------------------------
 * Control — a normal STANDBY→ACTIVE power-up DOES complete and become ready,
 * proving the model's sequencer is not trivially stuck (guards against a false
 * regression signal).
 * --------------------------------------------------------------------------- */
static void test_active_powerup_completes(void)
{
    printf("Control: STANDBY -> ACTIVE completes the relay sequence\n");
    world_reset();

    system_state = SYS_STATE_ACTIVE;
    Relay_PowerUp();
    CHECK(relay_seq_state == RELAY_SEQ_TRACTION_ON, "ACTIVE power-up started");

    /* Advance past the settle window while remaining ACTIVE. */
    g_tick += RELAY_TRACTION_SETTLE_MS;
    Relay_SequencerUpdate();
    CHECK(relay_seq_state == RELAY_SEQ_COMPLETE, "sequence completes in ACTIVE");
    CHECK(Safety_IsPowerReady(), "power_ready == true in ACTIVE");

    /* With power ready, the power-gate reason bit must be clear. */
    MotionInhibitInputs in = {0};
    in.state                = SYS_STATE_ACTIVE;
    in.power_ready          = true;
    in.gear                 = GEAR_FORWARD;
    in.gear_park            = GEAR_PARK;
    in.gear_neutral         = GEAR_NEUTRAL;
    in.state_safe           = SYS_STATE_SAFE;
    in.state_error          = SYS_STATE_ERROR;
    in.forward_gear         = true;
    in.demand_pct           = 80.0f;
    in.effective_demand_pct = 80.0f;
    in.final_pwm_max        = 1000U;
    uint16_t reason = MotionInhibit_Evaluate(&in);
    CHECK((reason & MOTION_INHIBIT_POWER_NOT_READY) == 0,
          "POWER_NOT_READY clear once the sequence completes in ACTIVE");
}

int main(void)
{
    printf("=== test_relay_degraded ===\n");
    test_standby_to_degraded_no_powerup();
    test_standby_to_limp_home_powers_down();
    test_active_powerup_completes();
    printf("\n--- relay_degraded tests: %d run, %d failed ---\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
