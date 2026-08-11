// Host cross-parity test: packs raw 0x31B/0x31C bytes using the STM32-side
// authoritative Core/Inc/service_diag_frame.h Pack() functions, then decodes
// them with the ESP32-side service_diag_view.h and asserts field-for-field
// equality — same pattern as test_traction_limit_diag_decode.cpp /
// test_relay_health_view.cpp.
#include <cstdio>
#include <cstring>

#include "service_diag_frame.h"
#include "service_diag_view.h"

static int failed;
#define CHECK(expr) do { \
    if (!(expr)) { \
        std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        ++failed; \
    } \
} while (0)

int main() {
    // ---- 0x31B DIAG_SERVICE_SESSION: a STEPPING snapshot ------------------
    ServiceDiagSessionFrame_t sessionIn{};
    sessionIn.state            = can::SVCDIAG_STATE_STEPPING;
    sessionIn.step_index       = 3U;
    sessionIn.active_channel   = can::SVCDIAG_CH_RR;
    sessionIn.progress_pct     = 55U;
    sessionIn.reason           = can::SVCDIAG_REASON_NONE;
    sessionIn.origin_state_raw = 1U; /* STANDBY, opaque to this header */
    sessionIn.elapsed_sec      = 12U;
    sessionIn.active_pwm_pct   = 30U;

    uint8_t sessionRaw[SERVICE_DIAG_SESSION_FRAME_DLC] = {0};
    CHECK(ServiceDiagSessionFrame_Pack(&sessionIn, sessionRaw) == SERVICE_DIAG_SESSION_FRAME_DLC);

    service_diag_view::SessionView sessionView{};
    CHECK(service_diag_view::decodeSession(sessionRaw, sizeof(sessionRaw), sessionView));
    CHECK(sessionView.state == can::SVCDIAG_STATE_STEPPING);
    CHECK(sessionView.stepIndex == 3U);
    CHECK(sessionView.activeChannel == can::SVCDIAG_CH_RR);
    CHECK(sessionView.progressPct == 55U);
    CHECK(sessionView.reason == can::SVCDIAG_REASON_NONE);
    CHECK(sessionView.originStateRaw == 1U);
    CHECK(sessionView.elapsedSec == 12U);
    CHECK(sessionView.activePwmPct == 30U);
    CHECK(service_diag_view::isActive(sessionView.state));
    CHECK(std::strcmp(service_diag_view::stateText(sessionView.state), "EJECUTANDO") == 0);
    CHECK(std::strcmp(service_diag_view::channelText(sessionView.activeChannel), "RR") == 0);

    // Wrong DLC must be rejected outright (no forward-compat short form).
    CHECK(!service_diag_view::decodeSession(sessionRaw, 3U, sessionView));
    CHECK(!service_diag_view::decodeSession(nullptr, 0U, sessionView));

    // ---- A rejected START: IDLE + a legible reason ------------------------
    ServiceDiagSessionFrame_t rejectIn{};
    rejectIn.state            = can::SVCDIAG_STATE_IDLE;
    rejectIn.step_index       = 0U;
    rejectIn.active_channel   = can::SVCDIAG_CH_NONE;
    rejectIn.progress_pct     = 0U;
    rejectIn.reason           = can::SVCDIAG_REASON_WHEELS_MOVING;
    rejectIn.origin_state_raw = 1U;
    rejectIn.elapsed_sec      = 0U;
    rejectIn.active_pwm_pct   = 0U;

    uint8_t rejectRaw[SERVICE_DIAG_SESSION_FRAME_DLC] = {0};
    CHECK(ServiceDiagSessionFrame_Pack(&rejectIn, rejectRaw) == SERVICE_DIAG_SESSION_FRAME_DLC);

    service_diag_view::SessionView rejectView{};
    CHECK(service_diag_view::decodeSession(rejectRaw, sizeof(rejectRaw), rejectView));
    CHECK(!service_diag_view::isActive(rejectView.state));
    CHECK(std::strcmp(service_diag_view::stateText(rejectView.state), "INACTIVO") == 0);
    CHECK(std::strcmp(service_diag_view::reasonText(rejectView.reason),
                      "RUEDAS EN MOVIMIENTO") == 0);

    // ---- 0x31C DIAG_TEST_RESULT: an overcurrent-failed step ---------------
    ServiceDiagTestResultFrame_t resultIn{};
    resultIn.channel        = can::SVCDIAG_CH_STEERING;
    resultIn.pwm_step_pct   = 25U;
    resultIn.current_ma     = 4200U;
    resultIn.pulses_per_sec = 0U;
    resultIn.verdict        = can::SVCDIAG_STEP_FAIL_OVERCURRENT;
    resultIn.step_index     = 3U;

    uint8_t resultRaw[SERVICE_DIAG_TEST_RESULT_FRAME_DLC] = {0};
    CHECK(ServiceDiagTestResultFrame_Pack(&resultIn, resultRaw) == SERVICE_DIAG_TEST_RESULT_FRAME_DLC);

    service_diag_view::TestResultView resultView{};
    CHECK(service_diag_view::decodeTestResult(resultRaw, sizeof(resultRaw), resultView));
    CHECK(resultView.channel == can::SVCDIAG_CH_STEERING);
    CHECK(resultView.pwmStepPct == 25U);
    CHECK(resultView.currentMa == 4200U);
    CHECK(resultView.pulsesPerSec == 0U);
    CHECK(resultView.verdict == can::SVCDIAG_STEP_FAIL_OVERCURRENT);
    CHECK(resultView.stepIndex == 3U);
    CHECK(std::strcmp(service_diag_view::channelText(resultView.channel), "DIR") == 0);
    CHECK(std::strcmp(service_diag_view::verdictText(resultView.verdict), "SOBRECORRIENTE") == 0);

    CHECK(!service_diag_view::decodeTestResult(resultRaw, 7U, resultView));
    CHECK(!service_diag_view::decodeTestResult(nullptr, 0U, resultView));

    // ---- Freshness (300 ms stale timeout while a session is believed live) -
    CHECK(service_diag_view::freshness(false, 5000U, 0U) ==
          service_diag_view::Freshness::NEVER_RECEIVED);
    CHECK(service_diag_view::freshness(true, 1200U, 1000U) ==
          service_diag_view::Freshness::VALID);
    /* Exactly 300 ms elapsed: boundary is inclusive, must be VALID. */
    CHECK(service_diag_view::freshness(true, 1300U, 1000U) ==
          service_diag_view::Freshness::VALID);
    CHECK(service_diag_view::freshness(true, 1301U, 1000U) ==
          service_diag_view::Freshness::STALE);

    if (failed != 0) return 1;
    std::puts("Service diag view decode tests: PASS");
    return 0;
}
