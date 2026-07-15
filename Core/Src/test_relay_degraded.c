/**
 * @file test_relay_degraded.c
 * @brief Regression model for drive-capable relay sequencing.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "motion_inhibit.h"

#define SYS_STATE_BOOT      0
#define SYS_STATE_STANDBY   1
#define SYS_STATE_ACTIVE    2
#define SYS_STATE_DEGRADED  3
#define SYS_STATE_SAFE      4
#define SYS_STATE_ERROR     5
#define SYS_STATE_LIMP_HOME 6
#define GEAR_PARK           0
#define GEAR_NEUTRAL        2
#define GEAR_FORWARD        3
#define RELAY_TRACTION_SETTLE_MS 50U

typedef enum {
    RELAY_SEQ_IDLE = 0,
    RELAY_SEQ_TRACTION_ON,
    RELAY_SEQ_COMPLETE
} RelaySeqState_t;

static RelaySeqState_t seq;
static int state, trac_relay, steer_relay;
static bool steering_calibrated;
static uint32_t tick, seq_tick;
static int tests_run, tests_failed;

#define CHECK(c, m) do { tests_run++; if (!(c)) { tests_failed++; \
    printf("  FAIL: %s\n", m); } } while (0)

static bool drive_capable(void)
{
    return state == SYS_STATE_ACTIVE ||
           state == SYS_STATE_DEGRADED ||
           state == SYS_STATE_LIMP_HOME;
}

static void power_down(void)
{
    trac_relay = 0;
    steer_relay = 0;
    seq = RELAY_SEQ_IDLE;
}

static void power_up(void)
{
    if (seq != RELAY_SEQ_IDLE) return;
    trac_relay = 1;
    seq = RELAY_SEQ_TRACTION_ON;
    seq_tick = tick;
}

static void set_state(int new_state)
{
    state = new_state;
    if (drive_capable()) power_up();
}

static void sequencer_update(void)
{
    if (!drive_capable()) {
        if (seq == RELAY_SEQ_TRACTION_ON || seq == RELAY_SEQ_COMPLETE)
            power_down();
        return;
    }

    if (seq == RELAY_SEQ_IDLE) power_up();

    if (seq == RELAY_SEQ_TRACTION_ON &&
        tick - seq_tick >= RELAY_TRACTION_SETTLE_MS) {
        steer_relay = steering_calibrated ? 1 : 0;
        seq = RELAY_SEQ_COMPLETE;
    }

    if (seq == RELAY_SEQ_COMPLETE)
        steer_relay = steering_calibrated ? 1 : 0;
}

static bool power_ready(void)
{
    return seq == RELAY_SEQ_COMPLETE;
}

static void reset_world(void)
{
    seq = RELAY_SEQ_IDLE;
    state = SYS_STATE_STANDBY;
    trac_relay = 0;
    steer_relay = 0;
    steering_calibrated = false;
    tick = 0;
    seq_tick = 0;
}

static uint16_t motion_reason(float demand, float effective, uint16_t pwm)
{
    MotionInhibitInputs in = {0};
    in.state = (uint8_t)state;
    in.power_ready = power_ready();
    in.gear = GEAR_FORWARD;
    in.gear_park = GEAR_PARK;
    in.gear_neutral = GEAR_NEUTRAL;
    in.state_safe = SYS_STATE_SAFE;
    in.state_error = SYS_STATE_ERROR;
    in.forward_gear = true;
    in.demand_pct = demand;
    in.effective_demand_pct = effective;
    in.final_pwm_max = pwm;
    return MotionInhibit_Evaluate(&in);
}

static void complete(void)
{
    tick += RELAY_TRACTION_SETTLE_MS;
    sequencer_update();
}

static void test_degraded_from_standby_is_drive_ready(void)
{
    reset_world();
    set_state(SYS_STATE_DEGRADED);
    complete();
    CHECK(seq == RELAY_SEQ_COMPLETE, "DEGRADED sequence completes");
    CHECK(trac_relay == 1, "traction relay on");
    CHECK(steer_relay == 0, "uncalibrated steering rail stays off");
    CHECK(power_ready(), "DEGRADED traction power ready");
    uint16_t r = motion_reason(40.0f, 20.0f, 800U);
    CHECK((r & MOTION_INHIBIT_POWER_NOT_READY) == 0,
          "no POWER_NOT_READY");
    CHECK((r & MOTION_INHIBIT_DEMAND_ZEROED) == 0,
          "demand not zeroed");
}

static void test_limp_from_standby_is_drive_ready(void)
{
    reset_world();
    set_state(SYS_STATE_LIMP_HOME);
    complete();
    CHECK(seq == RELAY_SEQ_COMPLETE, "LIMP_HOME sequence completes");
    CHECK(trac_relay == 1, "LIMP traction relay on");
    CHECK(steer_relay == 0, "LIMP uncalibrated steering off/free");
    CHECK(power_ready(), "LIMP traction power ready");
}

static void test_active_calibrated_powers_both(void)
{
    reset_world();
    steering_calibrated = true;
    set_state(SYS_STATE_ACTIVE);
    complete();
    CHECK(power_ready(), "ACTIVE power ready");
    CHECK(trac_relay == 1 && steer_relay == 1,
          "calibrated ACTIVE powers both rails");
}

static void test_safe_mid_sequence_powers_down(void)
{
    reset_world();
    set_state(SYS_STATE_LIMP_HOME);
    state = SYS_STATE_SAFE;
    sequencer_update();
    CHECK(seq == RELAY_SEQ_IDLE, "SAFE cancels partial sequence");
    CHECK(trac_relay == 0 && steer_relay == 0, "SAFE rails off");
}

int main(void)
{
    test_degraded_from_standby_is_drive_ready();
    test_limp_from_standby_is_drive_ready();
    test_active_calibrated_powers_both();
    test_safe_mid_sequence_powers_down();
    printf("relay_degraded: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
