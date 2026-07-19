/**
  ****************************************************************************
  * @file    test_standby_mode_sync_gate.c
  * @brief   Host tests for the productive STANDBY logical-mode-sync policy.
  *
  * The workflow compiles every ordinary STM32 host test as one translation
  * unit.  Include the productive implementation directly so this test executes
  * the exact code used by Safety_IsStandbyModeSyncAllowed() without maintaining
  * a hand-written replica and without requiring special linker arguments in CI.
  *
  * Open work that is intentionally NOT hidden by these tests:
  *   - physical verification that a cold boot with the selector in 4x4 reaches
  *     APPLIED=4x4 before traction is accepted;
  *   - physical steering-homing diagnosis (PC12/PC4/PWM/encoder history),
  *     STEER DIAG zero-flicker rendering and the intermittent CAN timeout;
  *   - conversion of 4x2 to rear-wheel drive and the per-wheel virtual
  *     differential/TCS behaviour.  These require separate implementation and
  *     hardware validation; a green host test must never be used as proof.
  ****************************************************************************
  */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "standby_mode_sync_policy.h"

/*
 * CI compiles this test source by itself.  Pull in the production implementation
 * so the executable contains StandbyModeSync_Evaluate().  The firmware itself
 * continues to compile standby_mode_sync_policy.c normally via the Makefile.
 */
#include "standby_mode_sync_policy.c"

#define ST_BOOT      0u
#define ST_STANDBY   STANDBY_SYNC_STATE_STANDBY
#define ST_ACTIVE    2u
#define ST_DEGRADED  3u
#define ST_SAFE      4u
#define ST_ERROR     5u
#define ST_LIMP_HOME 6u

typedef enum {
    ACK_OK = 0,
    ACK_REJECTED = 1,
    ACK_INVALID = 2,
    ACK_BLOCKED = 3
} Ack_t;

static int g_run;
static int g_failed;

#define CHECK(expr) do {                                                       \
    ++g_run;                                                                    \
    if (!(expr)) {                                                              \
        ++g_failed;                                                             \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr);                \
    }                                                                           \
} while (0)

static StandbySyncInput_t make_input(uint8_t state,
                                     float pedal,
                                     float demand,
                                     float effective_demand,
                                     uint8_t pwm,
                                     float speed,
                                     bool hazard)
{
    StandbySyncInput_t in;
    in.state = state;
    in.pedalPct = pedal;
    in.demandPct = demand;
    in.effectiveDemandPct = effective_demand;
    in.finalPwmPct = pwm;
    in.avgSpeedKmh = speed;
    in.errorOrHazardActive = hazard;
    return in;
}

static StandbySyncInput_t safe_standby(void)
{
    return make_input(ST_STANDBY, 0.0f, 0.0f, 0.0f, 0u, 0.0f, false);
}

/* Minimal model of the can_handler.c branch: STANDBY may update only logical
 * mode flags; it never applies the gear or grants motion. */
static Ack_t apply_cmd_mode(const StandbySyncInput_t *in,
                            uint8_t requested_flags,
                            uint8_t *applied_flags,
                            bool *gear_applied)
{
    *gear_applied = false;

    if (in->state == ST_ACTIVE || in->state == ST_DEGRADED) {
        if (!isfinite(in->avgSpeedKmh) ||
            in->avgSpeedKmh > STANDBY_MODE_SYNC_MAX_SPEED_KMH) {
            return ACK_REJECTED;
        }
        *applied_flags = (uint8_t)(requested_flags & 0x03u);
        *gear_applied = true;
        return ACK_OK;
    }

    if (StandbyModeSync_Evaluate(in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY) {
        *applied_flags = (uint8_t)(requested_flags & 0x03u);
        return ACK_OK;
    }

    return ACK_BLOCKED;
}

static void test_null_input_is_fail_safe(void)
{
    CHECK(StandbyModeSync_Evaluate(NULL) == STANDBY_SYNC_BLOCKED);
}

static void test_exact_state_gate(void)
{
    StandbySyncInput_t in = safe_standby();
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY);

    const uint8_t blocked_states[] = {
        ST_BOOT, ST_ACTIVE, ST_DEGRADED, ST_SAFE, ST_ERROR, ST_LIMP_HOME
    };
    for (size_t i = 0; i < sizeof(blocked_states) / sizeof(blocked_states[0]); ++i) {
        in.state = blocked_states[i];
        CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
    }
}

static void test_pedal_gate(void)
{
    StandbySyncInput_t in = safe_standby();
    in.pedalPct = STANDBY_MODE_SYNC_MAX_PEDAL_PCT;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY);

    in.pedalPct = STANDBY_MODE_SYNC_MAX_PEDAL_PCT + 0.1f;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);

    in.pedalPct = NAN;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
    in.pedalPct = INFINITY;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
}

static void test_internal_and_effective_demand_gates(void)
{
    StandbySyncInput_t in = safe_standby();

    in.demandPct = STANDBY_MODE_SYNC_DEMAND_EPSILON;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY);
    in.demandPct = STANDBY_MODE_SYNC_DEMAND_EPSILON + 0.1f;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);

    in = safe_standby();
    in.effectiveDemandPct = STANDBY_MODE_SYNC_DEMAND_EPSILON;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY);
    in.effectiveDemandPct = STANDBY_MODE_SYNC_DEMAND_EPSILON + 0.1f;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);

    in = safe_standby();
    in.finalPwmPct = 0u;
    in.demandPct = 10.0f;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);

    in = safe_standby();
    in.pedalPct = 0.0f;
    in.effectiveDemandPct = 2.0f;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);

    in = safe_standby();
    in.demandPct = NAN;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
    in = safe_standby();
    in.effectiveDemandPct = INFINITY;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
}

static void test_pwm_speed_and_hazard_gates(void)
{
    StandbySyncInput_t in = safe_standby();
    in.finalPwmPct = 1u;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);

    in = safe_standby();
    in.avgSpeedKmh = STANDBY_MODE_SYNC_MAX_SPEED_KMH;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_ALLOW_LOGICAL_ONLY);
    in.avgSpeedKmh = STANDBY_MODE_SYNC_MAX_SPEED_KMH + 0.1f;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
    in.avgSpeedKmh = NAN;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);

    in = safe_standby();
    in.errorOrHazardActive = true;
    CHECK(StandbyModeSync_Evaluate(&in) == STANDBY_SYNC_BLOCKED);
}

static void test_standby_applies_logical_mode_only(void)
{
    StandbySyncInput_t in = safe_standby();
    uint8_t applied = 0u;
    bool gear_applied = true;

    CHECK(apply_cmd_mode(&in, 0x01u, &applied, &gear_applied) == ACK_OK);
    CHECK(applied == 0x01u);
    CHECK(!gear_applied);

    CHECK(apply_cmd_mode(&in, 0x03u, &applied, &gear_applied) == ACK_OK);
    CHECK(applied == 0x03u);
    CHECK(!gear_applied);
}

static void test_unsafe_standby_does_not_partially_apply(void)
{
    StandbySyncInput_t in = safe_standby();
    in.pedalPct = 50.0f;
    uint8_t applied = 0u;
    bool gear_applied = false;

    CHECK(apply_cmd_mode(&in, 0x01u, &applied, &gear_applied) == ACK_BLOCKED);
    CHECK(applied == 0u);
    CHECK(!gear_applied);
}

static void test_boot_window_is_closed_logically(void)
{
    StandbySyncInput_t in = safe_standby();
    uint8_t applied = 0u; /* STM32 cold default: 4x2. */
    bool gear_applied = false;

    CHECK(apply_cmd_mode(&in, 0x01u, &applied, &gear_applied) == ACK_OK);
    CHECK(applied == 0x01u);
    CHECK(!gear_applied);

    in.state = ST_ACTIVE;
    in.pedalPct = 80.0f;
    CHECK(apply_cmd_mode(&in, 0x01u, &applied, &gear_applied) == ACK_OK);
    CHECK(applied == 0x01u);
    CHECK(gear_applied);
}

int main(void)
{
    test_null_input_is_fail_safe();
    test_exact_state_gate();
    test_pedal_gate();
    test_internal_and_effective_demand_gates();
    test_pwm_speed_and_hazard_gates();
    test_standby_applies_logical_mode_only();
    test_unsafe_standby_does_not_partially_apply();
    test_boot_window_is_closed_logically();

    printf("\n--- standby_mode_sync_gate: %d run, %d failed ---\n",
           g_run, g_failed);
    return g_failed == 0 ? 0 : 1;
}
