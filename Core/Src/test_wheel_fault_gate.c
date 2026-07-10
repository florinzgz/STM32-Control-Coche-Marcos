/**
  ****************************************************************************
  * @file    test_wheel_fault_gate.c
  * @brief   Unit tests for the wheel-speed "stale-while-others-moving"
  *          plausibility gate.  Validates that turning a wheel by hand
  *          (no traction commanded) never latches a WHEEL_SENSOR fault,
  *          that a genuine mismatch under load is only latched after a
  *          debounce, and that impossible/disabled channels behave.
  *
  *          This mirrors the decision table implemented in
  *          Safety_CheckSensors() (Core/Src/safety_system.c).  It is a
  *          self-contained logic replica (same style as test_wheel_speed.c)
  *          so it can run on host GCC without the full HAL.
  *
  *          Compile with host GCC (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -lm \
  *                Core/Src/test_wheel_fault_gate.c -o /tmp/test_wheel_fault_gate
  ****************************************************************************
  */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ---- Constants mirrored from safety_system.h / project_config.h ---- */
#define WHEEL_FAULT_DEBOUNCE_MS             1000U
#define WHEEL_INTERVENTION_MIN_DEMAND_PCT   3.0f
#define SENSOR_SPEED_MAX_KMH                25.0f

typedef enum {
    WHEEL_DIAG_OK              = 0,
    WHEEL_DIAG_NO_PULSE       = 1,
    WHEEL_DIAG_STUCK_HIGH     = 2,
    WHEEL_DIAG_STUCK_LOW      = 3,
    WHEEL_DIAG_MISMATCH       = 4,
    WHEEL_DIAG_IMPOSSIBLE_RATE = 5,
    WHEEL_DIAG_MANUAL_MOVEMENT = 6,
    WHEEL_DIAG_DISABLED_STATE  = 7
} WheelDiag_t;

/* Powertrain-engaged predicate (replica of Safety_PowertrainEngaged). */
static bool powertrain_engaged(float demand_pct, bool drive_state)
{
    float d = (demand_pct < 0.0f) ? -demand_pct : demand_pct;
    if (d < WHEEL_INTERVENTION_MIN_DEMAND_PCT) return false;
    return drive_state;
}

/* One evaluation cycle for a single wheel.  Returns whether the wheel
 * incremented fault_count and writes the resulting diagnostic + debounce
 * timer.  `now`/`since` model HAL_GetTick() persistence. */
static bool eval_wheel(bool enabled, float spd, bool stale, bool any_moving,
                       float demand_pct, bool drive_state, uint8_t gpio_level,
                       uint32_t now, uint32_t *since, WheelDiag_t *diag)
{
    if (!enabled) { *diag = WHEEL_DIAG_DISABLED_STATE; *since = 0; return false; }

    if (isnan(spd) || isinf(spd) || spd < 0.0f || spd > SENSOR_SPEED_MAX_KMH) {
        *diag = WHEEL_DIAG_IMPOSSIBLE_RATE; *since = 0; return true;
    }

    if (stale && spd == 0.0f) {
        if (!any_moving) { *diag = WHEEL_DIAG_OK; *since = 0; return false; }
        if (!powertrain_engaged(demand_pct, drive_state)) {
            *diag = WHEEL_DIAG_MANUAL_MOVEMENT; *since = 0; return false;
        }
        if (*since == 0) *since = now;
        if ((now - *since) >= WHEEL_FAULT_DEBOUNCE_MS) {
            if      (gpio_level == 1U) *diag = WHEEL_DIAG_STUCK_HIGH;
            else if (gpio_level == 0U) *diag = WHEEL_DIAG_STUCK_LOW;
            else                       *diag = WHEEL_DIAG_NO_PULSE;
            return true;
        }
        *diag = WHEEL_DIAG_MISMATCH;
        return false;
    }

    *diag = WHEEL_DIAG_OK; *since = 0; return false;
}

/* eval_wheel with 4x2 drag-wheel exemption (mirrors the Traction_GetState()
 * check added to Safety_CheckSensors in safety_system.c).                */
static bool eval_wheel_drag(uint8_t wheel_index, bool mode4x4, bool axisRotation,
                            bool enabled, float spd, bool stale,
                            bool any_moving, float demand_pct, bool drive_state,
                            uint8_t gpio_level, uint32_t now, uint32_t *since,
                            WheelDiag_t *diag)
{
    bool rear_drag = (wheel_index >= 2U) && !(mode4x4 || axisRotation);

    /* Drag-wheel exemption: a braked rear wheel in 4x2 may produce no
     * pulses while the front axle is moving under load.  This is normal
     * operation, not a sensor fault.                                    */
    if (rear_drag && stale && spd == 0.0f && any_moving) {
        *diag  = WHEEL_DIAG_OK;
        *since = 0;
        return false;
    }
    return eval_wheel(enabled, spd, stale, any_moving, demand_pct, drive_state,
                      gpio_level, now, since, diag);
}


static int tests_run = 0, tests_failed = 0;
#define CHECK(cond, msg) do {                                     \
        tests_run++;                                              \
        if (!(cond)) { tests_failed++;                            \
            printf("  FAIL: %s\n", msg); }                        \
        else { printf("  ok:   %s\n", msg); }                     \
    } while (0)

int main(void)
{
    printf("=== test_wheel_fault_gate ===\n");
    uint32_t since; WheelDiag_t diag; bool fault;

    /* 1. Manual movement (parked, no throttle): one wheel spun by hand,
     *    others stale.  Must NOT fault, diag = MANUAL_MOVEMENT.          */
    since = 0;
    fault = eval_wheel(true, 0.0f, true, true, 0.0f, false, 0, 5000, &since, &diag);
    CHECK(!fault, "manual movement (no traction) does not fault");
    CHECK(diag == WHEEL_DIAG_MANUAL_MOVEMENT, "manual movement diag reported");
    CHECK(since == 0, "manual movement does not arm debounce");

    /* 2. Manual movement while in ACTIVE but pedal released (demand 0).  */
    since = 0;
    fault = eval_wheel(true, 0.0f, true, true, 0.0f, true, 1, 5000, &since, &diag);
    CHECK(!fault, "ACTIVE + zero throttle manual spin does not fault");
    CHECK(diag == WHEEL_DIAG_MANUAL_MOVEMENT, "ACTIVE zero-throttle diag = manual");

    /* 3. Under traction, transient mismatch (< debounce): no fault yet.  */
    since = 0;
    fault = eval_wheel(true, 0.0f, true, true, 40.0f, true, 0, 1000, &since, &diag);
    CHECK(!fault, "under load, first cycle arms debounce (no latch)");
    CHECK(diag == WHEEL_DIAG_MISMATCH, "under load transient diag = mismatch");
    CHECK(since == 1000, "debounce timer armed at first detection");
    /* still within debounce window */
    fault = eval_wheel(true, 0.0f, true, true, 40.0f, true, 0, 1999, &since, &diag);
    CHECK(!fault, "under load, still within 1000ms debounce -> no fault");

    /* 4. Under traction, persistent mismatch (>= debounce): faults.      */
    fault = eval_wheel(true, 0.0f, true, true, 40.0f, true, 0, 2000, &since, &diag);
    CHECK(fault, "persistent mismatch under load latches fault");
    CHECK(diag == WHEEL_DIAG_STUCK_LOW, "silent + pin low -> STUCK_LOW");

    /* 5. Stuck-high classification. */
    since = 1000;
    fault = eval_wheel(true, 0.0f, true, true, 40.0f, true, 1, 2000, &since, &diag);
    CHECK(fault && diag == WHEEL_DIAG_STUCK_HIGH, "silent + pin high -> STUCK_HIGH");

    /* 6. Reverse demand also counts as under-power. */
    since = 1000;
    fault = eval_wheel(true, 0.0f, true, true, -40.0f, true, 2, 2000, &since, &diag);
    CHECK(fault && diag == WHEEL_DIAG_NO_PULSE, "reverse demand engages gate");

    /* 7. Vehicle fully stopped (all wheels stale): normal, no fault. */
    since = 0;
    fault = eval_wheel(true, 0.0f, true, false, 40.0f, true, 0, 5000, &since, &diag);
    CHECK(!fault && diag == WHEEL_DIAG_OK, "all wheels stopped is normal");

    /* 8. Impossible speed value faults regardless of state/traction. */
    since = 0;
    fault = eval_wheel(true, 99.0f, false, false, 0.0f, false, 0, 5000, &since, &diag);
    CHECK(fault && diag == WHEEL_DIAG_IMPOSSIBLE_RATE, "out-of-range speed faults");
    fault = eval_wheel(true, NAN, false, false, 0.0f, false, 0, 5000, &since, &diag);
    CHECK(fault && diag == WHEEL_DIAG_IMPOSSIBLE_RATE, "NaN speed faults");

    /* 9. Disabled channel never faults. */
    since = 0;
    fault = eval_wheel(false, 0.0f, true, true, 40.0f, true, 0, 5000, &since, &diag);
    CHECK(!fault && diag == WHEEL_DIAG_DISABLED_STATE, "disabled channel does not fault");

    /* 10. Healthy moving wheel clears any prior debounce arm. */
    since = 1500;
    fault = eval_wheel(true, 10.0f, false, true, 40.0f, true, 0, 5000, &since, &diag);
    CHECK(!fault && diag == WHEEL_DIAG_OK && since == 0,
          "wheel producing pulses clears debounce + diag OK");

    /* 11. 4x2 drag wheel (RL/RR) stale while front axle drives under load:
     *     must NOT fault — silence is expected behavior for a drag wheel. */
    since = 0;
    fault = eval_wheel_drag(2U, false, false, true, 0.0f, true, true,
                            40.0f, true, 0, 5000, &since, &diag);
    CHECK(!fault && diag == WHEEL_DIAG_OK,
          "4x2 drag wheel stale under load is normal (not a fault)");
    CHECK(since == 0, "4x2 drag wheel does not arm debounce timer");

    /* 12. Drag exemption does NOT apply when rear wheels ARE driven (4x4). */
    since = 0;
    fault = eval_wheel_drag(2U, true, false, true, 0.0f, true, true,
                            40.0f, true, 0, 1000, &since, &diag);
    CHECK(!fault && diag == WHEEL_DIAG_MISMATCH,
          "4x4 rear wheel stale under load arms debounce");
    since = 1000;
    fault = eval_wheel_drag(2U, true, false, true, 0.0f, true, true,
                            40.0f, true, 0, 2000, &since, &diag);
    CHECK(fault, "4x4 rear wheel stale under load latches fault after debounce");

    /* 13. Front wheels are never drag wheels in production (rear_drag is
     *     derived from wheel index >= 2), so the non-drag path must still
     *     arm debounce under load when the stale wheel is on the front axle. */
    since = 0;
    fault = eval_wheel_drag(0U, false, false, true, 0.0f, true, true,
                            40.0f, true, 0, 1000, &since, &diag);
    CHECK(!fault, "front wheel (no drag flag) stale under load arms debounce only");

    printf("=== %d run, %d failed ===\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
