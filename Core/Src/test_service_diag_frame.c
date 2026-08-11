/**
  ****************************************************************************
  * @file    test_service_diag_frame.c
  * @brief   Host round-trip test for the 0x31B / 0x31C SERVICE_DIAG CAN
  *          frames (Bloque A, PR #445 Hito 1 — T2).
  *
  * Build (host):
  *   gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE -ICore/Inc \
  *       Core/Src/test_service_diag_frame.c -lm -o /tmp/t && /tmp/t
  ****************************************************************************
  */

#include "service_diag_frame.h"
#include "service_diag_session.h"

#include <stdio.h>
#include <string.h>

static int failed;
#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failed++; \
    } \
} while (0)

/* ---- 0x31B DIAG_SERVICE_SESSION -------------------------------------- */

static void test_session_round_trip_typical(void)
{
    ServiceDiagSessionFrame_t in;
    in.state            = SVCDIAG_STATE_STEPPING;
    in.step_index        = 3U;
    in.active_channel    = SVCDIAG_CH_RL;
    in.progress_pct      = 42U;
    in.reason            = SVCDIAG_REASON_NONE;
    in.origin_state_raw  = 2U;   /* opaque, not interpreted here */
    in.elapsed_sec       = 17U;
    in.active_pwm_pct    = 35U;

    uint8_t bytes[SERVICE_DIAG_SESSION_FRAME_DLC] = {0};
    uint8_t n = ServiceDiagSessionFrame_Pack(&in, bytes);
    CHECK(n == SERVICE_DIAG_SESSION_FRAME_DLC);
    CHECK(bytes[0] == (uint8_t)SVCDIAG_STATE_STEPPING);
    CHECK(bytes[1] == 3U);
    CHECK(bytes[2] == (uint8_t)SVCDIAG_CH_RL);
    CHECK(bytes[3] == 42U);
    CHECK(bytes[4] == (uint8_t)SVCDIAG_REASON_NONE);
    CHECK(bytes[5] == 2U);
    CHECK(bytes[6] == 17U);
    CHECK(bytes[7] == 35U);

    ServiceDiagSessionFrame_t out;
    memset(&out, 0xAA, sizeof(out));
    CHECK(ServiceDiagSessionFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.state           == in.state);
    CHECK(out.step_index       == in.step_index);
    CHECK(out.active_channel   == in.active_channel);
    CHECK(out.progress_pct     == in.progress_pct);
    CHECK(out.reason           == in.reason);
    CHECK(out.origin_state_raw == in.origin_state_raw);
    CHECK(out.elapsed_sec      == in.elapsed_sec);
    CHECK(out.active_pwm_pct   == in.active_pwm_pct);
}

static void test_session_round_trip_idle_silent_values(void)
{
    /* IDLE / no channel active: the fields the caller must present when the
     * session is not stepping (accessor defaults from service_diag_session.h). */
    ServiceDiagSessionFrame_t in;
    in.state            = SVCDIAG_STATE_IDLE;
    in.step_index        = 0U;
    in.active_channel    = SVCDIAG_CH_NONE;
    in.progress_pct      = 0U;
    in.reason            = SVCDIAG_REASON_NOT_CONFIRMED;
    in.origin_state_raw  = 0U;
    in.elapsed_sec       = 0U;
    in.active_pwm_pct    = 0U;

    uint8_t bytes[SERVICE_DIAG_SESSION_FRAME_DLC] = {0};
    CHECK(ServiceDiagSessionFrame_Pack(&in, bytes) == SERVICE_DIAG_SESSION_FRAME_DLC);

    ServiceDiagSessionFrame_t out;
    CHECK(ServiceDiagSessionFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.state         == SVCDIAG_STATE_IDLE);
    CHECK(out.active_channel == SVCDIAG_CH_NONE);
    CHECK(out.reason        == SVCDIAG_REASON_NOT_CONFIRMED);
}

static void test_session_boundary_values(void)
{
    /* Max values for every field, incl. progress/pwm saturation via the
     * shared SatPct helper (input out of 0..100 must clamp, never wrap). */
    ServiceDiagSessionFrame_t in;
    in.state            = SVCDIAG_STATE_ABORTED;   /* last enum member */
    in.step_index        = 255U;
    in.active_channel    = SVCDIAG_CH_NONE;         /* last enum member */
    in.progress_pct      = 250U;                    /* out of range -> sat 100 */
    in.reason            = SVCDIAG_REASON_WATCHDOG;  /* last enum member */
    in.origin_state_raw  = 255U;
    in.elapsed_sec       = 255U;
    in.active_pwm_pct    = 200U;                    /* out of range -> sat 100 */

    uint8_t bytes[SERVICE_DIAG_SESSION_FRAME_DLC] = {0};
    CHECK(ServiceDiagSessionFrame_Pack(&in, bytes) == SERVICE_DIAG_SESSION_FRAME_DLC);
    CHECK(bytes[3] == 100U);   /* progress_pct saturated */
    CHECK(bytes[7] == 100U);   /* active_pwm_pct saturated */

    ServiceDiagSessionFrame_t out;
    CHECK(ServiceDiagSessionFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.state           == SVCDIAG_STATE_ABORTED);
    CHECK(out.step_index       == 255U);
    CHECK(out.active_channel   == SVCDIAG_CH_NONE);
    CHECK(out.progress_pct     == 100U);
    CHECK(out.reason           == SVCDIAG_REASON_WATCHDOG);
    CHECK(out.origin_state_raw == 255U);
    CHECK(out.elapsed_sec      == 255U);
    CHECK(out.active_pwm_pct   == 100U);
}

static void test_session_invalid_dlc_rejected(void)
{
    ServiceDiagSessionFrame_t in;
    in.state = SVCDIAG_STATE_ARMED;
    in.step_index = 1U; in.active_channel = SVCDIAG_CH_FL;
    in.progress_pct = 0U; in.reason = SVCDIAG_REASON_NONE;
    in.origin_state_raw = 1U; in.elapsed_sec = 5U; in.active_pwm_pct = 0U;

    uint8_t bytes[SERVICE_DIAG_SESSION_FRAME_DLC] = {0};
    CHECK(ServiceDiagSessionFrame_Pack(&in, bytes) == SERVICE_DIAG_SESSION_FRAME_DLC);

    ServiceDiagSessionFrame_t out;
    memset(&out, 0x55, sizeof(out));
    /* Too short */
    CHECK(ServiceDiagSessionFrame_Unpack(bytes, 7U, &out) == false);
    /* Too long */
    CHECK(ServiceDiagSessionFrame_Unpack(bytes, 9U, &out) == false);
    /* Zero */
    CHECK(ServiceDiagSessionFrame_Unpack(bytes, 0U, &out) == false);
    /* Output must be untouched by a rejected decode. */
    CHECK(out.origin_state_raw == 0x55U);

    /* NULL guards. */
    CHECK(ServiceDiagSessionFrame_Unpack(NULL, sizeof(bytes), &out) == false);
    CHECK(ServiceDiagSessionFrame_Unpack(bytes, sizeof(bytes), NULL) == false);
    CHECK(ServiceDiagSessionFrame_Pack(NULL, bytes) == 0U);
    CHECK(ServiceDiagSessionFrame_Pack(&in, NULL) == 0U);
}

/* ---- 0x31C DIAG_TEST_RESULT ------------------------------------------ */

static void test_result_round_trip_typical(void)
{
    ServiceDiagTestResultFrame_t in;
    in.channel        = SVCDIAG_CH_FR;
    in.pwm_step_pct   = 50U;
    in.current_ma     = 4200U;
    in.pulses_per_sec = 37U;
    in.verdict        = SVCDIAG_STEP_PASS;
    in.step_index     = 2U;

    uint8_t bytes[SERVICE_DIAG_TEST_RESULT_FRAME_DLC] = {0};
    uint8_t n = ServiceDiagTestResultFrame_Pack(&in, bytes);
    CHECK(n == SERVICE_DIAG_TEST_RESULT_FRAME_DLC);
    CHECK(bytes[0] == (uint8_t)SVCDIAG_CH_FR);
    CHECK(bytes[1] == 50U);
    CHECK(bytes[2] == (4200U & 0xFFU));
    CHECK(bytes[3] == ((4200U >> 8) & 0xFFU));
    CHECK(bytes[4] == (37U & 0xFFU));
    CHECK(bytes[5] == ((37U >> 8) & 0xFFU));
    CHECK(bytes[6] == (uint8_t)SVCDIAG_STEP_PASS);
    CHECK(bytes[7] == 2U);

    ServiceDiagTestResultFrame_t out;
    memset(&out, 0xAA, sizeof(out));
    CHECK(ServiceDiagTestResultFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.channel        == in.channel);
    CHECK(out.pwm_step_pct   == in.pwm_step_pct);
    CHECK(out.current_ma     == in.current_ma);
    CHECK(out.pulses_per_sec == in.pulses_per_sec);
    CHECK(out.verdict        == in.verdict);
    CHECK(out.step_index     == in.step_index);
}

static void test_result_round_trip_overcurrent_fail(void)
{
    ServiceDiagTestResultFrame_t in;
    in.channel        = SVCDIAG_CH_STEERING;
    in.pwm_step_pct   = 28U;
    in.current_ma     = 16000U;
    in.pulses_per_sec = 0U;   /* steering has no wheel pulse rate */
    in.verdict        = SVCDIAG_STEP_FAIL_OVERCURRENT;
    in.step_index     = 1U;

    uint8_t bytes[SERVICE_DIAG_TEST_RESULT_FRAME_DLC] = {0};
    CHECK(ServiceDiagTestResultFrame_Pack(&in, bytes) == SERVICE_DIAG_TEST_RESULT_FRAME_DLC);

    ServiceDiagTestResultFrame_t out;
    CHECK(ServiceDiagTestResultFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.channel == SVCDIAG_CH_STEERING);
    CHECK(out.verdict == SVCDIAG_STEP_FAIL_OVERCURRENT);
    CHECK(out.pulses_per_sec == 0U);
    CHECK(out.current_ma == 16000U);
}

static void test_result_boundary_values(void)
{
    ServiceDiagTestResultFrame_t in;
    in.channel        = SVCDIAG_CH_NONE;          /* last enum member */
    in.pwm_step_pct   = 255U;                     /* out of range -> sat 100 */
    in.pulses_per_sec = 65535U;                    /* max u16 */
    in.verdict        = SVCDIAG_STEP_FAIL_OVERCURRENT; /* last enum member */
    in.step_index     = 255U;

    /* current_ma is a plain uint16_t field, so an out-of-range reading (e.g.
     * 70000 mA) must already be clamped by the caller via
     * ServiceDiagFrame_SatU16() before it is stored here — the struct field
     * itself cannot hold > 65535. Exercise that helper directly. */
    in.current_ma = ServiceDiagFrame_SatU16(70000);
    CHECK(in.current_ma == 65535U);

    uint8_t bytes[SERVICE_DIAG_TEST_RESULT_FRAME_DLC] = {0};
    CHECK(ServiceDiagTestResultFrame_Pack(&in, bytes) == SERVICE_DIAG_TEST_RESULT_FRAME_DLC);
    CHECK(bytes[1] == 100U);  /* pwm_step_pct saturated */

    ServiceDiagTestResultFrame_t out;
    CHECK(ServiceDiagTestResultFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.channel        == SVCDIAG_CH_NONE);
    CHECK(out.pwm_step_pct   == 100U);
    CHECK(out.current_ma     == 65535U);
    CHECK(out.pulses_per_sec == 65535U);
    CHECK(out.verdict        == SVCDIAG_STEP_FAIL_OVERCURRENT);
    CHECK(out.step_index     == 255U);
}

static void test_result_invalid_dlc_rejected(void)
{
    ServiceDiagTestResultFrame_t in;
    in.channel = SVCDIAG_CH_FL; in.pwm_step_pct = 10U;
    in.current_ma = 100U; in.pulses_per_sec = 5U;
    in.verdict = SVCDIAG_STEP_RUNNING; in.step_index = 1U;

    uint8_t bytes[SERVICE_DIAG_TEST_RESULT_FRAME_DLC] = {0};
    CHECK(ServiceDiagTestResultFrame_Pack(&in, bytes) == SERVICE_DIAG_TEST_RESULT_FRAME_DLC);

    ServiceDiagTestResultFrame_t out;
    memset(&out, 0x55, sizeof(out));
    CHECK(ServiceDiagTestResultFrame_Unpack(bytes, 7U, &out) == false);
    CHECK(ServiceDiagTestResultFrame_Unpack(bytes, 9U, &out) == false);
    CHECK(ServiceDiagTestResultFrame_Unpack(bytes, 0U, &out) == false);
    CHECK(out.current_ma == 0x5555U);   /* untouched by rejected decode */

    CHECK(ServiceDiagTestResultFrame_Unpack(NULL, sizeof(bytes), &out) == false);
    CHECK(ServiceDiagTestResultFrame_Unpack(bytes, sizeof(bytes), NULL) == false);
    CHECK(ServiceDiagTestResultFrame_Pack(NULL, bytes) == 0U);
    CHECK(ServiceDiagTestResultFrame_Pack(&in, NULL) == 0U);
}

/* ---- Every channel / verdict / reason / state enum value round-trips -- */

static void test_all_channels_round_trip(void)
{
    ServiceDiagChannel_t channels[] = {
        SVCDIAG_CH_FL, SVCDIAG_CH_FR, SVCDIAG_CH_RL, SVCDIAG_CH_RR,
        SVCDIAG_CH_STEERING, SVCDIAG_CH_NONE
    };
    for (size_t i = 0; i < sizeof(channels) / sizeof(channels[0]); i++) {
        ServiceDiagTestResultFrame_t in = {0};
        in.channel = channels[i];
        in.verdict = SVCDIAG_STEP_NONE;
        uint8_t bytes[SERVICE_DIAG_TEST_RESULT_FRAME_DLC] = {0};
        CHECK(ServiceDiagTestResultFrame_Pack(&in, bytes) == SERVICE_DIAG_TEST_RESULT_FRAME_DLC);
        ServiceDiagTestResultFrame_t out;
        CHECK(ServiceDiagTestResultFrame_Unpack(bytes, sizeof(bytes), &out));
        CHECK(out.channel == channels[i]);
    }
}

static void test_all_states_round_trip(void)
{
    ServiceDiagState_t states[] = {
        SVCDIAG_STATE_IDLE, SVCDIAG_STATE_ENTERING, SVCDIAG_STATE_ARMED,
        SVCDIAG_STATE_STEPPING, SVCDIAG_STATE_DEADTIME, SVCDIAG_STATE_ABORTED
    };
    for (size_t i = 0; i < sizeof(states) / sizeof(states[0]); i++) {
        ServiceDiagSessionFrame_t in = {0};
        in.state = states[i];
        in.active_channel = SVCDIAG_CH_NONE;
        uint8_t bytes[SERVICE_DIAG_SESSION_FRAME_DLC] = {0};
        CHECK(ServiceDiagSessionFrame_Pack(&in, bytes) == SERVICE_DIAG_SESSION_FRAME_DLC);
        ServiceDiagSessionFrame_t out;
        CHECK(ServiceDiagSessionFrame_Unpack(bytes, sizeof(bytes), &out));
        CHECK(out.state == states[i]);
    }
}

static void test_all_reasons_round_trip(void)
{
    ServiceDiagReason_t reasons[] = {
        SVCDIAG_REASON_NONE, SVCDIAG_REASON_OPERATOR, SVCDIAG_REASON_BOOT_STATE,
        SVCDIAG_REASON_GEAR, SVCDIAG_REASON_WHEELS_MOVING, SVCDIAG_REASON_NOT_CONFIRMED,
        SVCDIAG_REASON_BATTERY_LOW, SVCDIAG_REASON_BATTERY_WARN, SVCDIAG_REASON_ALREADY_ACTIVE,
        SVCDIAG_REASON_STATE_CHANGED, SVCDIAG_REASON_CAN_LOSS, SVCDIAG_REASON_GROUNDED_WHEEL,
        SVCDIAG_REASON_OVERCURRENT, SVCDIAG_REASON_SESSION_TIMEOUT, SVCDIAG_REASON_STEP_TIMEOUT,
        SVCDIAG_REASON_WATCHDOG
    };
    for (size_t i = 0; i < sizeof(reasons) / sizeof(reasons[0]); i++) {
        ServiceDiagSessionFrame_t in = {0};
        in.reason = reasons[i];
        in.active_channel = SVCDIAG_CH_NONE;
        uint8_t bytes[SERVICE_DIAG_SESSION_FRAME_DLC] = {0};
        CHECK(ServiceDiagSessionFrame_Pack(&in, bytes) == SERVICE_DIAG_SESSION_FRAME_DLC);
        ServiceDiagSessionFrame_t out;
        CHECK(ServiceDiagSessionFrame_Unpack(bytes, sizeof(bytes), &out));
        CHECK(out.reason == reasons[i]);
    }
}

static void test_all_verdicts_round_trip(void)
{
    ServiceDiagStepVerdict_t verdicts[] = {
        SVCDIAG_STEP_NONE, SVCDIAG_STEP_RUNNING, SVCDIAG_STEP_PASS,
        SVCDIAG_STEP_FAIL_OVERCURRENT
    };
    for (size_t i = 0; i < sizeof(verdicts) / sizeof(verdicts[0]); i++) {
        ServiceDiagTestResultFrame_t in = {0};
        in.verdict = verdicts[i];
        uint8_t bytes[SERVICE_DIAG_TEST_RESULT_FRAME_DLC] = {0};
        CHECK(ServiceDiagTestResultFrame_Pack(&in, bytes) == SERVICE_DIAG_TEST_RESULT_FRAME_DLC);
        ServiceDiagTestResultFrame_t out;
        CHECK(ServiceDiagTestResultFrame_Unpack(bytes, sizeof(bytes), &out));
        CHECK(out.verdict == verdicts[i]);
    }
}

int main(void)
{
    test_session_round_trip_typical();
    test_session_round_trip_idle_silent_values();
    test_session_boundary_values();
    test_session_invalid_dlc_rejected();

    test_result_round_trip_typical();
    test_result_round_trip_overcurrent_fail();
    test_result_boundary_values();
    test_result_invalid_dlc_rejected();

    test_all_channels_round_trip();
    test_all_states_round_trip();
    test_all_reasons_round_trip();
    test_all_verdicts_round_trip();

    if (failed != 0) {
        fprintf(stderr, "service_diag_frame: %d assertion(s) FAILED\n", failed);
        return 1;
    }
    puts("service_diag_frame: all assertions PASS");
    return 0;
}
