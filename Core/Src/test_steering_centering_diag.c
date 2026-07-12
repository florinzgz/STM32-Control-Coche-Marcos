/**
  ****************************************************************************
  * @file    test_steering_centering_diag.c
  * @brief   Host unit tests for the steering-centering diagnostic classifier
  *          (SteeringCentering_ClassifyDiag).
  *
  *          Covers the audit's Problem-1 test matrix: ownership loss, homing
  *          start, flash restore, PB5 active/inactive, WAIT_RAIL→SWEEP,
  *          lost homing state, encoder frozen vs moving, DONE, FAULT and
  *          SAFE/ERROR aborts — proving each maps to a DISTINCT explicit
  *          reason instead of a single opaque "Error 8".
  *
  *          Compile with host GCC (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  *                Core/Src/test_steering_centering_diag.c \
  *                Core/Src/steering_centering_diag.c -lm \
  *                -o /tmp/test_steering_centering_diag
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <string.h>

#include "steering_centering_diag.h"

static int tests_run = 0, tests_failed = 0;

#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* Build a "healthy sweeping-left" baseline snapshot; individual tests
 * mutate only the fields relevant to the scenario under test.            */
static SteeringCenteringDiag base_sweeping_left(void)
{
    SteeringCenteringDiag d;
    memset(&d, 0, sizeof(d));
    d.fsm_state                   = CENTERING_SWEEP_LEFT;
    d.motor_owner                 = STEER_OWNER_CENTERING;
    d.system_state                = STEER_DIAG_SS_STANDBY;
    d.center_sensor_raw           = false;
    d.center_sensor_debounced     = false;
    d.center_sensor_already_active= false;
    d.encoder_count               = 1200;
    d.encoder_delta               = 300;
    d.relay_steer_commanded       = true;
    d.power_ready                 = true;
    d.enable_commanded            = true;
    d.pwm_requested               = 425;
    d.pwm_applied_ch1             = 425;
    d.pwm_applied_ch2             = 0;
    d.elapsed_ms                  = 1500;
    d.last_encoder_change_ms      = 10;   /* moving */
    d.encoder_fault               = false;
    d.restored_from_flash         = false;
    d.module_disabled             = false;
    d.fault_latched               = false;
    return d;
}

static SteerDiagReason_t classify(SteeringCenteringDiag d)
{
    return SteeringCentering_ClassifyDiag(&d);
}

int main(void)
{
    /* --- Nominal in-progress sweeps --- */
    {
        SteeringCenteringDiag d = base_sweeping_left();
        CHECK(classify(d) == STEER_DIAG_SWEEP_LEFT);
        d.fsm_state = CENTERING_SWEEP_RIGHT;
        CHECK(classify(d) == STEER_DIAG_SWEEP_RIGHT);
    }

    /* --- DONE: OK vs restored from flash --- */
    {
        SteeringCenteringDiag d = base_sweeping_left();
        d.fsm_state = CENTERING_DONE;
        CHECK(classify(d) == STEER_DIAG_OK);
        d.restored_from_flash = true;
        CHECK(classify(d) == STEER_DIAG_RESTORED_FROM_FLASH);
    }

    /* --- Homing start: IDLE / WAIT_RAIL are "waiting power" (not a fault) --- */
    {
        SteeringCenteringDiag d = base_sweeping_left();
        d.fsm_state = CENTERING_IDLE;
        d.pwm_requested = 0;
        CHECK(classify(d) == STEER_DIAG_WAITING_POWER);
        d.fsm_state = CENTERING_WAIT_RAIL;
        CHECK(classify(d) == STEER_DIAG_WAITING_POWER);
    }

    /* --- Relay not commanded while homing --- */
    {
        SteeringCenteringDiag d = base_sweeping_left();
        d.relay_steer_commanded = false;
        CHECK(classify(d) == STEER_DIAG_RELAY_NOT_READY);
    }

    /* --- Power not ready while relay commanded --- */
    {
        SteeringCenteringDiag d = base_sweeping_left();
        d.power_ready = false;
        CHECK(classify(d) == STEER_DIAG_WAITING_POWER);
    }

    /* --- PB5 already active at sweep start --- */
    {
        SteeringCenteringDiag d = base_sweeping_left();
        d.center_sensor_already_active = true;
        CHECK(classify(d) == STEER_DIAG_CENTER_SENSOR_ALREADY_ACTIVE);
    }

    /* --- Encoder moving vs frozen while sweeping --- */
    {
        SteeringCenteringDiag d = base_sweeping_left();
        d.last_encoder_change_ms = 10;   /* moving */
        CHECK(classify(d) == STEER_DIAG_SWEEP_LEFT);
        d.last_encoder_change_ms = STEER_DIAG_STALL_TIMEOUT_MS; /* frozen */
        CHECK(classify(d) == STEER_DIAG_NO_ENCODER_MOVEMENT);
        /* ...but only if PWM is actually being requested */
        d.pwm_requested = 0;
        CHECK(classify(d) == STEER_DIAG_SWEEP_LEFT);
    }

    /* --- Lost homing state: left STANDBY, or ownership taken by EPS --- */
    {
        SteeringCenteringDiag d = base_sweeping_left();
        d.system_state = STEER_DIAG_SS_LIMP_HOME;
        CHECK(classify(d) == STEER_DIAG_LOST_HOMING_STATE);
        d = base_sweeping_left();
        d.system_state = STEER_DIAG_SS_ACTIVE;
        CHECK(classify(d) == STEER_DIAG_LOST_HOMING_STATE);
        d = base_sweeping_left();
        d.motor_owner = STEER_OWNER_EPS;
        CHECK(classify(d) == STEER_DIAG_LOST_HOMING_STATE);
    }

    /* --- Range exceeded / total timeout while sweeping --- */
    {
        SteeringCenteringDiag d = base_sweeping_left();
        d.encoder_delta = STEER_DIAG_MAX_CENTERING_COUNTS + 1;
        CHECK(classify(d) == STEER_DIAG_RANGE_EXCEEDED);
        d = base_sweeping_left();
        d.encoder_delta = -(STEER_DIAG_MAX_CENTERING_COUNTS + 1); /* abs */
        CHECK(classify(d) == STEER_DIAG_RANGE_EXCEEDED);
        d = base_sweeping_left();
        d.elapsed_ms = STEER_DIAG_TOTAL_TIMEOUT_MS;
        CHECK(classify(d) == STEER_DIAG_TOTAL_TIMEOUT);
    }

    /* --- FAULT terminal state explains the underlying cause --- */
    {
        SteeringCenteringDiag d = base_sweeping_left();
        d.fsm_state = CENTERING_FAULT;
        d.last_encoder_change_ms = STEER_DIAG_STALL_TIMEOUT_MS;
        CHECK(classify(d) == STEER_DIAG_NO_ENCODER_MOVEMENT);
        d.elapsed_ms = STEER_DIAG_TOTAL_TIMEOUT_MS;
        CHECK(classify(d) == STEER_DIAG_TOTAL_TIMEOUT);
        d.encoder_delta = STEER_DIAG_MAX_CENTERING_COUNTS + 5;
        CHECK(classify(d) == STEER_DIAG_RANGE_EXCEEDED);
    }

    /* --- Encoder fault dominates every other signal --- */
    {
        SteeringCenteringDiag d = base_sweeping_left();
        d.encoder_fault = true;
        CHECK(classify(d) == STEER_DIAG_ENCODER_FAULT);
        d.system_state = STEER_DIAG_SS_SAFE; /* still encoder fault first */
        CHECK(classify(d) == STEER_DIAG_ENCODER_FAULT);
    }

    /* --- SAFE / ERROR aborts --- */
    {
        SteeringCenteringDiag d = base_sweeping_left();
        d.system_state = STEER_DIAG_SS_SAFE;
        CHECK(classify(d) == STEER_DIAG_ABORTED_SAFE);
        d.system_state = STEER_DIAG_SS_ERROR;
        CHECK(classify(d) == STEER_DIAG_ABORTED_ERROR);
    }

    /* --- Module disabled in Service Mode --- */
    {
        SteeringCenteringDiag d = base_sweeping_left();
        d.module_disabled = true;
        CHECK(classify(d) == STEER_DIAG_MODULE_DISABLED);
    }

    /* --- NULL guard + label safety --- */
    {
        CHECK(SteeringCentering_ClassifyDiag(NULL) == STEER_DIAG_UNKNOWN);
        for (int r = STEER_DIAG_OK; r <= STEER_DIAG_UNKNOWN; r++) {
            const char *s = SteeringCentering_DiagReasonStr((SteerDiagReason_t)r);
            CHECK(s != NULL && s[0] != '\0');
        }
        /* Out-of-range reason still returns a safe non-NULL string */
        CHECK(SteeringCentering_DiagReasonStr((SteerDiagReason_t)999) != NULL);
    }

    printf("steering_centering_diag: %d run, %d failed\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#endif /* HOST_TEST */
