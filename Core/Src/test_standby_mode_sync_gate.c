/**
  ****************************************************************************
  * @file    test_standby_mode_sync_gate.c
  * @brief   Tests for the STANDBY logical-mode-sync gate.
  *
  *          Tests the PRODUCTIVE policy function StandbyModeSync_Evaluate()
  *          from standby_mode_sync_policy.c directly — NOT a hand-rolled
  *          replica.  Safety_IsStandbyModeSyncAllowed() in safety_system.c
  *          calls the same function, so the tests exercise exactly the code
  *          that runs in the firmware.
  *
  *          Compile with host GCC (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE -ICore/Inc -O2 -lm \
  *                Core/Src/standby_mode_sync_policy.c               \
  *                Core/Src/test_standby_mode_sync_gate.c             \
  *                -o /tmp/test_standby_mode_sync_gate
  ****************************************************************************
  */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Link the REAL productive policy — not a replica. */
#include "standby_mode_sync_policy.h"

/* ---- Convenience aliases ---- */
#define ST_BOOT      0u
#define ST_STANDBY   STANDBY_SYNC_STATE_STANDBY  /* 1 */
#define ST_ACTIVE    2u
#define ST_DEGRADED  3u
#define ST_SAFE      4u
#define ST_ERROR     5u
#define ST_LIMP_HOME 6u

/* ---- ACK codes (mirrors STM32 can_handler / ESP32 can::AckResult) ---- */
typedef enum { A_OK = 0, A_REJECTED = 1, A_INVALID = 2, A_BLOCKED = 3 } Ack_t;

/* ---- Safe-input builder ---- */
static StandbySyncInput_t make_input(uint8_t state,
                                     float pedal,
                                     float demand,
                                     float eff_demand,
                                     uint8_t pwm,
                                     float speed,
                                     bool hazard)
{
    StandbySyncInput_t in;
    in.state               = state;
    in.pedalPct            = pedal;
    in.demandPct           = demand;
    in.effectiveDemandPct  = eff_demand;
    in.finalPwmPct         = pwm;
    in.avgSpeedKmh         = speed;
    in.errorOrHazardActive = hazard;
    return in;
}

/* A fully-safe STANDBY input (all conditions met). */
static StandbySyncInput_t safe_standby(void) {
    return make_input(ST_STANDBY, 0.0f, 0.0f, 0.0f, 0, 0.0f, false);
}

/* ---- Model of the CMD_MODE handler's STANDBY branch ---- */
/* Returns the ACK the ESP32 would observe and updates applied_flags_io when
 * the gate opens.  Gear change is NEVER permitted in the STANDBY branch.    */
static Ack_t cmd_mode_handler_standby(const StandbySyncInput_t *in,
                                       uint8_t req_flags,
                                       uint8_t *applied_flags_io,
                                       bool    *gear_allowed_out)
{
    *gear_allowed_out = false;
    /* ACTIVE/DEGRADED: full path (not tested here). */
    if (in->state == ST_ACTIVE || in->state == ST_DEGRADED) {
        if (in->avgSpeedKmh > STANDBY_MODE_SYNC_MAX_SPEED_KMH) return A_REJECTED;
        *applied_flags_io = (uint8_t)(req_flags & 0x03u);
        *gear_allowed_out = true;
        return A_OK;
    }
    /* Non-ACTIVE gate: STANDBY policy decides. */
    if (StandbyModeSync_Evaluate(in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY) {
        *applied_flags_io = (uint8_t)(req_flags & 0x03u); /* logical only */
        return A_OK;                                       /* no gear/motion */
    }
    return A_BLOCKED;
}

/* ---- Test harness ---- */
static int g_run = 0, g_fail = 0;
#define CHECK(cond) do { g_run++; if (!(cond)) {                              \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while (0)

/* ---- §1 State gate ---- */

static void test_state_must_be_standby(void)
{
    StandbySyncInput_t in = safe_standby();
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY);

    uint8_t states[] = { ST_BOOT, ST_ACTIVE, ST_DEGRADED,
                         ST_SAFE, ST_ERROR, ST_LIMP_HOME };
    for (size_t i = 0; i < sizeof states / sizeof states[0]; ++i) {
        in.state = states[i];
        CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
    }
}

/* ---- §1 Pedal gate ---- */

static void test_pedal_blocks_gate(void)
{
    StandbySyncInput_t in = safe_standby();
    in.pedalPct = STANDBY_MODE_SYNC_MAX_PEDAL_PCT;       /* at limit → OK */
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY);
    in.pedalPct = STANDBY_MODE_SYNC_MAX_PEDAL_PCT + 0.1f; /* just over → BLOCKED */
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
    in.pedalPct = 100.0f;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
    in.pedalPct = NAN;   /* NaN → fail-safe BLOCKED */
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
}

/* ---- §1 Internal demand gate (NEW) ---- */

static void test_internal_demand_blocks_gate(void)
{
    StandbySyncInput_t in = safe_standby();
    /* At epsilon → still OK. */
    in.demandPct = STANDBY_MODE_SYNC_DEMAND_EPSILON;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY);
    /* Just above epsilon → BLOCKED. */
    in.demandPct = STANDBY_MODE_SYNC_DEMAND_EPSILON + 0.1f;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
    /* Large demand → BLOCKED. */
    in.demandPct = 50.0f;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
    /* NaN → fail-safe BLOCKED. */
    in.demandPct = NAN;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
}

/* ---- §1 Effective demand gate (NEW) ---- */

static void test_effective_demand_blocks_gate(void)
{
    StandbySyncInput_t in = safe_standby();
    /* At epsilon → still OK. */
    in.effectiveDemandPct = STANDBY_MODE_SYNC_DEMAND_EPSILON;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY);
    /* Just above epsilon → BLOCKED. */
    in.effectiveDemandPct = STANDBY_MODE_SYNC_DEMAND_EPSILON + 0.1f;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
    /* NaN → fail-safe BLOCKED. */
    in.effectiveDemandPct = NAN;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
}

/* ---- §1 PWM zero gate ---- */

static void test_pwm_zero_blocks_gate(void)
{
    StandbySyncInput_t in = safe_standby();
    in.finalPwmPct = 0;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY);
    in.finalPwmPct = 1;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
    in.finalPwmPct = 100;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
}

/* ---- §1 Speed gate ---- */

static void test_speed_blocks_gate(void)
{
    StandbySyncInput_t in = safe_standby();
    in.avgSpeedKmh = STANDBY_MODE_SYNC_MAX_SPEED_KMH;       /* at limit → OK */
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY);
    in.avgSpeedKmh = STANDBY_MODE_SYNC_MAX_SPEED_KMH + 0.1f; /* over → BLOCKED */
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
    in.avgSpeedKmh = NAN;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
}

/* ---- §1 Hazard gate ---- */

static void test_hazard_blocks_gate(void)
{
    StandbySyncInput_t in = safe_standby();
    in.errorOrHazardActive = false;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY);
    in.errorOrHazardActive = true;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
}

/* ---- NEW: PWM=0 but internal demand >0 must be BLOCKED ---- */

static void test_pwm_zero_but_internal_demand_nonzero_blocked(void)
{
    /* Scenario: inhibition logic has zeroed PWM but demandPct is still > 0.
     * A PWM of zero alone does not prove there is no stored demand. */
    StandbySyncInput_t in = safe_standby();
    in.finalPwmPct = 0;        /* PWM is zero */
    in.demandPct   = 10.0f;    /* but internal demand is non-zero */
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
}

/* ---- NEW: pedal=0 but creep/effective demand >0 must be BLOCKED ---- */

static void test_pedal_zero_but_effective_demand_nonzero_blocked(void)
{
    /* Scenario: pedal is released but creep or effective demand still flows. */
    StandbySyncInput_t in = safe_standby();
    in.pedalPct           = 0.0f;  /* pedal fully released */
    in.effectiveDemandPct = 2.0f;  /* but effective demand > epsilon */
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
}

/* ---- NEW: demand AND PWM truly zero → ACK_OK ---- */

static void test_demand_and_pwm_truly_zero_allows(void)
{
    /* All conditions genuinely met: pedal=0, demand=0, effDemand=0, PWM=0,
     * speed=0, no hazard — gate MUST open and return ALLOW_LOGICAL_ONLY. */
    StandbySyncInput_t in = safe_standby();
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY);
}

/* ---- Handler integration: STANDBY applies logical mode only ---- */

static void test_handler_standby_applies_logical_mode_only(void)
{
    StandbySyncInput_t in = safe_standby();
    uint8_t applied   = 0x00;   /* STM32 power-on default 4x2 */
    bool    gear_ok   = true;
    Ack_t a = cmd_mode_handler_standby(&in, 0x01, &applied, &gear_ok);
    CHECK(a == A_OK);
    CHECK(applied == 0x01);   /* 4x4 applied logically */
    CHECK(gear_ok == false);  /* gear/motion NEVER granted in STANDBY */
}

/* ---- Handler integration: unsafe STANDBY → BLOCKED, mode unchanged ---- */

static void test_handler_standby_unsafe_blocks(void)
{
    StandbySyncInput_t in = safe_standby();
    in.pedalPct = 50.0f;
    uint8_t applied = 0x00;
    bool    gear_ok = false;
    Ack_t a = cmd_mode_handler_standby(&in, 0x01, &applied, &gear_ok);
    CHECK(a == A_BLOCKED);
    CHECK(applied == 0x00);   /* unchanged */
    CHECK(gear_ok == false);  /* no gear change */
}

/* ---- Handler integration: tank-turn bit carried through ---- */

static void test_handler_standby_applies_tank_bit(void)
{
    StandbySyncInput_t in = safe_standby();
    uint8_t applied = 0x00;
    bool    gear_ok = false;
    Ack_t a = cmd_mode_handler_standby(&in, 0x03, &applied, &gear_ok);
    CHECK(a == A_OK);
    CHECK(applied == 0x03);   /* 4x4 + tank turn */
}

/* ---- Handler integration: SAFE state blocked ---- */

static void test_handler_safe_state_blocks(void)
{
    StandbySyncInput_t in = safe_standby();
    in.state = ST_SAFE;
    uint8_t applied = 0x00;
    bool    gear_ok = false;
    Ack_t a = cmd_mode_handler_standby(&in, 0x01, &applied, &gear_ok);
    CHECK(a == A_BLOCKED);
    CHECK(applied == 0x00);
}

/* ---- Handler integration: no relay, no PWM, no gear in STANDBY ---- */

static void test_handler_standby_no_motion(void)
{
    /* In STANDBY the handler applies ONLY the logical mode bits (0x03 mask).
     * Gear (byte 1 of 0x102) is ignored; no relay energised; no PWM. */
    StandbySyncInput_t in = safe_standby();
    uint8_t applied = 0x00;
    bool    gear_ok = false;

    /* Try 4x4 request with a gear byte — gear must stay blocked. */
    Ack_t a = cmd_mode_handler_standby(&in, 0x01, &applied, &gear_ok);
    CHECK(a == A_OK);
    CHECK((applied & 0x01u) != 0u);  /* 4x4 bit set */
    CHECK(gear_ok == false);         /* gear NEVER applied in STANDBY */
}

/* ---- Handler integration: boot window closed end-to-end ---- */

static void test_boot_window_closed(void)
{
    StandbySyncInput_t in = safe_standby();
    uint8_t applied = 0x00;   /* STM32 power-on default 4x2 */
    bool    gear_ok = false;

    /* ESP32 retransmits CMD_MODE(4x4) while in STANDBY. */
    Ack_t a = cmd_mode_handler_standby(&in, 0x01, &applied, &gear_ok);
    CHECK(a == A_OK);
    CHECK(applied == 0x01);   /* 4x4 now set */
    CHECK(gear_ok == false);

    /* Driver presses pedal → ACTIVE; mode is ALREADY 4x4. */
    in.state    = ST_ACTIVE;
    in.pedalPct = 80.0f;
    a = cmd_mode_handler_standby(&in, 0x01, &applied, &gear_ok);
    CHECK(a == A_OK);
    CHECK(applied == 0x01);   /* still 4x4 — no wrong-mode moment */
    CHECK(gear_ok == true);   /* gear IS allowed once ACTIVE */
}

int main(void)
{
    test_state_must_be_standby();
    test_pedal_blocks_gate();
    test_internal_demand_blocks_gate();
    test_effective_demand_blocks_gate();
    test_pwm_zero_blocks_gate();
    test_speed_blocks_gate();
    test_hazard_blocks_gate();
    test_pwm_zero_but_internal_demand_nonzero_blocked();
    test_pedal_zero_but_effective_demand_nonzero_blocked();
    test_demand_and_pwm_truly_zero_allows();
    test_handler_standby_applies_logical_mode_only();
    test_handler_standby_unsafe_blocks();
    test_handler_standby_applies_tank_bit();
    test_handler_safe_state_blocks();
    test_handler_standby_no_motion();
    test_boot_window_closed();

    printf("\n--- standby_mode_sync_gate tests: %d run, %d failed ---\n",
           g_run, g_fail);
    return g_fail ? 1 : 0;
}
