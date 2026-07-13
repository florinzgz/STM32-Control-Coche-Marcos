/**
  ****************************************************************************
  * @file    test_steering_centering_frame.c
  * @brief   Host integration test for the steering-homing diagnostic
  *          telemetry path (Problem 1).
  *
  *          Unlike test_steering_centering_diag.c (which unit-tests the pure
  *          classifier in isolation), this test exercises the FULL telemetry
  *          transport that the audit demands:
  *
  *            1. Build REAL-looking SteeringCenteringDiag snapshots for the
  *               key homing scenarios.
  *            2. Run the real classifier (SteeringCentering_ClassifyDiag).
  *            3. Pack the snapshot into the 0x316 CAN frame with the SAME
  *               pure serialiser used by can_handler.c
  *               (SteerCentering_PackFrame).
  *            4. Unpack it with the SAME deserialiser the ESP32 uses
  *               (SteerCentering_UnpackFrame).
  *            5. Assert every field round-trips and the decoded reason still
  *               matches — i.e. the HMI would show the correct MOTIVO.
  *
  *          Compile with host GCC (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -ICore/Inc -O2 \
  *                Core/Src/test_steering_centering_frame.c \
  *                Core/Src/steering_centering_diag.c -lm \
  *                -o /tmp/test_steering_centering_frame
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <string.h>

#include "steering_centering_diag.h"
#include "steering_centering_frame.h"

static int tests_run = 0, tests_failed = 0;

#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* A nominal, actively-sweeping-left snapshot (rail up, power ready, encoder
 * moving) — the baseline the individual scenarios mutate.                  */
static SteeringCenteringDiag base_snapshot(void)
{
    SteeringCenteringDiag d;
    memset(&d, 0, sizeof(d));
    d.fsm_state             = CENTERING_SWEEP_LEFT;
    d.motor_owner           = STEER_OWNER_CENTERING;
    d.system_state          = STEER_DIAG_SS_STANDBY;
    d.center_sensor_raw     = false;
    d.center_sensor_debounced = false;
    d.center_sensor_already_active = false;
    d.encoder_count         = 1234;
    d.encoder_delta         = 200;
    d.relay_steer_commanded = true;
    d.power_ready           = true;
    d.enable_commanded      = true;
    d.pwm_requested         = 425;
    d.pwm_applied_ch1       = 425;
    d.pwm_applied_ch2       = 0;
    d.elapsed_ms            = 500;
    d.last_encoder_change_ms = 20;
    d.encoder_fault         = false;
    d.restored_from_flash   = false;
    d.module_disabled       = false;
    d.fault_latched         = false;
    return d;
}

/* Classify, pack, unpack, and assert the whole telemetry round-trips. */
static SteerCenteringFrame roundtrip(SteeringCenteringDiag *d)
{
    d->abort_reason = SteeringCentering_ClassifyDiag(d);

    uint8_t frame[8];
    SteerCentering_PackFrame(d, frame);

    SteerCenteringFrame out;
    memset(&out, 0, sizeof(out));
    SteerCentering_UnpackFrame(frame, &out);
    return out;
}

/* Assert the decoded frame faithfully reproduces the source snapshot. */
static void check_roundtrip(const SteeringCenteringDiag *d,
                            const SteerCenteringFrame *out)
{
    CHECK(out->reason       == (uint8_t)d->abort_reason);
    CHECK(out->fsm_state    == (uint8_t)d->fsm_state);
    CHECK(out->motor_owner  == (uint8_t)d->motor_owner);
    CHECK(out->system_state == (uint8_t)d->system_state);

    CHECK(out->center_raw            == d->center_sensor_raw);
    CHECK(out->center_debounced      == d->center_sensor_debounced);
    CHECK(out->center_already_active == d->center_sensor_already_active);
    CHECK(out->relay_commanded       == d->relay_steer_commanded);
    CHECK(out->power_ready           == d->power_ready);
    CHECK(out->enable_commanded      == d->enable_commanded);
    CHECK(out->encoder_fault         == d->encoder_fault);
    CHECK(out->restored_from_flash   == d->restored_from_flash);
    CHECK(out->module_disabled       == d->module_disabled);
    CHECK(out->fault_latched         == d->fault_latched);
    CHECK(out->pwm_requested         == (d->pwm_requested > 0U));

    uint16_t exp_pwm = (d->pwm_applied_ch1 > d->pwm_applied_ch2)
                           ? d->pwm_applied_ch1 : d->pwm_applied_ch2;
    CHECK(out->pwm_real == exp_pwm);
}

/* Scenario 1: nominal sweep left → SWEEP_LEFT survives the round-trip. */
static void test_nominal_sweep(void)
{
    SteeringCenteringDiag d = base_snapshot();
    SteerCenteringFrame out = roundtrip(&d);
    CHECK(d.abort_reason == STEER_DIAG_SWEEP_LEFT);
    check_roundtrip(&d, &out);
    CHECK(out.encoder_delta == 200);
    CHECK(out.pwm_real == 425);
}

/* Scenario 2: "DIRECCIÓN NO SE MUEVE" — PWM requested, encoder frozen. */
static void test_no_encoder_movement(void)
{
    SteeringCenteringDiag d = base_snapshot();
    d.last_encoder_change_ms = STEER_DIAG_STALL_TIMEOUT_MS + 10U;
    d.encoder_delta = 0;
    SteerCenteringFrame out = roundtrip(&d);
    CHECK(d.abort_reason == STEER_DIAG_NO_ENCODER_MOVEMENT);
    check_roundtrip(&d, &out);
    CHECK(out.reason == (uint8_t)STEER_DIAG_NO_ENCODER_MOVEMENT);
    CHECK(out.encoder_delta == 0);
    CHECK(out.pwm_requested == true);   /* PWM REQ line shows > 0 */
}

/* Scenario 3: relay not commanded → RELAY_NOT_READY, PC12 flag clears. */
static void test_relay_not_ready(void)
{
    SteeringCenteringDiag d = base_snapshot();
    d.relay_steer_commanded = false;
    d.power_ready = false;
    SteerCenteringFrame out = roundtrip(&d);
    CHECK(d.abort_reason == STEER_DIAG_RELAY_NOT_READY);
    check_roundtrip(&d, &out);
    CHECK(out.relay_commanded == false);
    CHECK(out.power_ready == false);
}

/* Scenario 4: PB5 already active before sweep began. */
static void test_center_already_active(void)
{
    SteeringCenteringDiag d = base_snapshot();
    d.center_sensor_already_active = true;
    d.center_sensor_raw = true;
    SteerCenteringFrame out = roundtrip(&d);
    CHECK(d.abort_reason == STEER_DIAG_CENTER_SENSOR_ALREADY_ACTIVE);
    check_roundtrip(&d, &out);
    CHECK(out.center_already_active == true);
    CHECK(out.center_raw == true);
}

/* Scenario 5: SAFE abort overrides everything; system_state survives. */
static void test_aborted_safe(void)
{
    SteeringCenteringDiag d = base_snapshot();
    d.system_state = STEER_DIAG_SS_SAFE;
    SteerCenteringFrame out = roundtrip(&d);
    CHECK(d.abort_reason == STEER_DIAG_ABORTED_SAFE);
    check_roundtrip(&d, &out);
    CHECK(out.system_state == STEER_DIAG_SS_SAFE);
}

/* Scenario 6: negative encoder delta (sweeping right) round-trips signed. */
static void test_negative_delta(void)
{
    SteeringCenteringDiag d = base_snapshot();
    d.fsm_state = CENTERING_SWEEP_RIGHT;
    d.encoder_delta = -1500;
    d.pwm_applied_ch1 = 0;
    d.pwm_applied_ch2 = 425;
    SteerCenteringFrame out = roundtrip(&d);
    CHECK(d.abort_reason == STEER_DIAG_SWEEP_RIGHT);
    check_roundtrip(&d, &out);
    CHECK(out.encoder_delta == -1500);
    CHECK(out.pwm_real == 425);
}

/* Scenario 7: encoder delta beyond int16 range is clamped, not wrapped. */
static void test_delta_clamp(void)
{
    SteeringCenteringDiag d = base_snapshot();
    d.encoder_delta = 100000;   /* > INT16_MAX */
    (void)roundtrip(&d);
    uint8_t frame[8];
    SteerCentering_PackFrame(&d, frame);
    SteerCenteringFrame out;
    SteerCentering_UnpackFrame(frame, &out);
    CHECK(out.encoder_delta == 32767);

    d.encoder_delta = -100000;  /* < INT16_MIN */
    SteerCentering_PackFrame(&d, frame);
    SteerCentering_UnpackFrame(frame, &out);
    CHECK(out.encoder_delta == -32768);
}

/* Scenario 8: DONE + restored-from-flash → RESTORED_FROM_FLASH, flag set. */
static void test_restored_from_flash(void)
{
    SteeringCenteringDiag d = base_snapshot();
    d.fsm_state = CENTERING_DONE;
    d.motor_owner = STEER_OWNER_EPS;
    d.restored_from_flash = true;
    d.pwm_requested = 0;
    d.pwm_applied_ch1 = 0;
    d.pwm_applied_ch2 = 0;
    SteerCenteringFrame out = roundtrip(&d);
    CHECK(d.abort_reason == STEER_DIAG_RESTORED_FROM_FLASH);
    check_roundtrip(&d, &out);
    CHECK(out.restored_from_flash == true);
    CHECK(out.motor_owner == STEER_OWNER_EPS);
    CHECK(out.pwm_requested == false);
}

int main(void)
{
    test_nominal_sweep();
    test_no_encoder_movement();
    test_relay_not_ready();
    test_center_already_active();
    test_aborted_safe();
    test_negative_delta();
    test_delta_clamp();
    test_restored_from_flash();

    printf("steering_centering_frame: %d run, %d failed\n",
           tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}
#else
int main(void) { return 0; }
#endif
