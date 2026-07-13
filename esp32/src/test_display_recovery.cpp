// test_display_recovery.cpp — audit Problem 2 (§4, §6).
//
// Host tests for the PURE Core-0 recovery choreography and the
// "PANTALLA RECUPERADA" banner formatter (display_recovery.h).  Together with
// test_display_supervisor.cpp (detection + retry policy) this proves the whole
// P2 supervisor policy at the host level, without any TFT / SPI / FreeRTOS.
//
// Build (from repo root):
//   g++ -std=c++17 -Wall -Wextra -Werror -Iesp32/src
//       esp32/src/test_display_recovery.cpp -o /tmp/test_display_recovery
//   /tmp/test_display_recovery

#include <cstdio>
#include <cstring>
#include <vector>

#include "display_recovery.h"

using namespace display;

static int tests_run = 0, tests_failed = 0;
#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

// The recovery steps must appear in exactly the audit's mandated order, and
// TFT re-initialisation must come only AFTER the panel has been fully isolated
// (both chip-selects HIGH, SPI closed) and the reset line pulsed.
static void test_sequence_order(void) {
    const RecoveryStep* seq = recoverySequence();

    // Exact order.
    const RecoveryStep expected[kRecoveryStepCount] = {
        RecoveryStep::STOP_RENDER_TOUCH,
        RecoveryStep::BLOCK_TFT_ACCESS,
        RecoveryStep::TFT_CS_HIGH,
        RecoveryStep::TOUCH_CS_HIGH,
        RecoveryStep::CLOSE_SPI,
        RecoveryStep::PULSE_GPIO38,
        RecoveryStep::TFT_INIT,
        RecoveryStep::SET_ROTATION_1,
        RecoveryStep::RESTORE_TOUCH_CAL,
        RecoveryStep::RESTORE_BACKLIGHT,
        RecoveryStep::INVALIDATE_CACHES,
        RecoveryStep::FORCE_FULL_REDRAW,
        RecoveryStep::VERIFY,
        RecoveryStep::RESUME,
    };
    for (uint8_t i = 0; i < kRecoveryStepCount; i++) {
        CHECK(seq[i] == expected[i]);
        CHECK(recoveryStepText(seq[i]) != nullptr);
    }

    // Helper: index of a step in the sequence.
    auto idx = [&](RecoveryStep s) -> int {
        for (int i = 0; i < kRecoveryStepCount; i++) if (seq[i] == s) return i;
        return -1;
    };

    // Isolation must precede re-init; re-init before redraw; verify before resume.
    CHECK(idx(RecoveryStep::STOP_RENDER_TOUCH) < idx(RecoveryStep::TFT_INIT));
    CHECK(idx(RecoveryStep::TFT_CS_HIGH)   < idx(RecoveryStep::TFT_INIT));
    CHECK(idx(RecoveryStep::TOUCH_CS_HIGH) < idx(RecoveryStep::TFT_INIT));
    CHECK(idx(RecoveryStep::CLOSE_SPI)     < idx(RecoveryStep::TFT_INIT));
    CHECK(idx(RecoveryStep::PULSE_GPIO38)  < idx(RecoveryStep::TFT_INIT));
    CHECK(idx(RecoveryStep::TFT_INIT)      < idx(RecoveryStep::SET_ROTATION_1));
    CHECK(idx(RecoveryStep::SET_ROTATION_1) < idx(RecoveryStep::FORCE_FULL_REDRAW));
    CHECK(idx(RecoveryStep::INVALIDATE_CACHES) < idx(RecoveryStep::FORCE_FULL_REDRAW));
    CHECK(idx(RecoveryStep::FORCE_FULL_REDRAW) < idx(RecoveryStep::VERIFY));
    CHECK(idx(RecoveryStep::VERIFY)        < idx(RecoveryStep::RESUME));
    CHECK(idx(RecoveryStep::RESUME) == kRecoveryStepCount - 1);
}

static void test_banner_fields(void) {
    RecoveryBannerInfo b{};
    b.cause            = Fault::RENDER_TIMEOUT;
    b.duration_ms      = 1234;
    b.attempts         = 2;
    b.render_continued = true;
    b.esp32_rebooted   = false;
    b.free_heap        = 54321;
    b.total_recoveries = 7;

    char buf[256];
    int written = formatRecoveryBanner(buf, sizeof(buf), b);
    CHECK(written > 0);
    CHECK(written < (int)sizeof(buf));      // no truncation

    // All mandated fields must be present.
    CHECK(strstr(buf, "PANTALLA RECUPERADA") != nullptr);
    CHECK(strstr(buf, "causa: RENDER TIMEOUT") != nullptr);
    CHECK(strstr(buf, "duracion: 1234 ms") != nullptr);
    CHECK(strstr(buf, "intentos: 2") != nullptr);
    CHECK(strstr(buf, "render continuo: si") != nullptr);
    CHECK(strstr(buf, "reboot ESP32: no") != nullptr);
    CHECK(strstr(buf, "heap: 54321") != nullptr);
    CHECK(strstr(buf, "recuperaciones totales: 7") != nullptr);
    CHECK(strstr(buf, "accion recomendada:") != nullptr);
}

// The banner must NEVER assert "white confirmed" when readback is unsupported.
static void test_banner_no_false_white_claim(void) {
    RecoveryBannerInfo b{};
    b.cause = Fault::READBACK_UNSUPPORTED;
    char buf[256];
    formatRecoveryBanner(buf, sizeof(buf), b);
    CHECK(strstr(buf, "causa: READBACK NO FIABLE") != nullptr);
    CHECK(strstr(buf, "confirmar visualmente") != nullptr);
    // Must not contain any unfounded "pantalla blanca confirmada" wording.
    CHECK(strstr(buf, "blanca confirmada") == nullptr);
}

// snprintf truncation contract: a tiny buffer is safely bounded and the
// return value signals the required size.
static void test_banner_truncation(void) {
    RecoveryBannerInfo b{};
    b.cause = Fault::LOW_MEMORY;
    b.free_heap = 100;
    char small[16];
    int need = formatRecoveryBanner(small, sizeof(small), b);
    CHECK(need >= (int)sizeof(small));       // truncated -> need >= n
    CHECK(small[sizeof(small) - 1] == '\0'); // still NUL-terminated
}

static void test_action_per_cause(void) {
    RecoveryBannerInfo b{};
    b.cause = Fault::LOW_MEMORY;
    CHECK(strstr(recoveryActionText(b), "heap") != nullptr);
    b.cause = Fault::ESP32_RESET;
    CHECK(strstr(recoveryActionText(b), "brownout") != nullptr);
    b.cause = Fault::TFT_STATUS_LOST;
    CHECK(strstr(recoveryActionText(b), "cableado") != nullptr);
}

// The trigger classification must honour the strict audit priority and must
// NEVER promote a plain stale render (no readback, no reset latch) to a
// confirmed white screen.
static void test_trigger_classification(void) {
    // Manual request wins over everything else.
    CHECK(classifyRecoveryTrigger(true, true, StatusRead::INVALID, true)
          == RecoveryTrigger::RECOVERY_MANUAL_REQUEST);
    // Reset latch beats status/render.
    CHECK(classifyRecoveryTrigger(false, true, StatusRead::INVALID, true)
          == RecoveryTrigger::TFT_RESET_SIGNAL_DETECTED);
    // Invalid status beats a stale render.
    CHECK(classifyRecoveryTrigger(false, false, StatusRead::INVALID, true)
          == RecoveryTrigger::TFT_CONTROLLER_STATUS_LOST);
    // Stale render with unsupported readback -> RENDER_STALLED, NOT white.
    CHECK(classifyRecoveryTrigger(false, false, StatusRead::UNSUPPORTED, true)
          == RecoveryTrigger::RENDER_STALLED);
    // No observable signal at all.
    CHECK(classifyRecoveryTrigger(false, false, StatusRead::UNSUPPORTED, false)
          == RecoveryTrigger::WHITE_SCREEN_NOT_OBSERVABLE);

    // Observability honesty: with no readback and no reset latch, white is
    // NOT observable.
    CHECK(whiteScreenObservable(false, false) == false);
    CHECK(whiteScreenObservable(true, false)  == true);
    CHECK(whiteScreenObservable(false, true)  == true);

    // Fault mapping never fabricates a confirmed white.
    CHECK(recoveryTriggerToFault(RecoveryTrigger::RENDER_STALLED) == Fault::RENDER_TIMEOUT);
    CHECK(recoveryTriggerToFault(RecoveryTrigger::TFT_RESET_SIGNAL_DETECTED)
          == Fault::TFT_RESET_PROBABLE);
    CHECK(recoveryTriggerToFault(RecoveryTrigger::RECOVERY_MANUAL_REQUEST)
          == Fault::TFT_RESET_PROBABLE);

    for (uint8_t i = 0; i <= (uint8_t)RecoveryTrigger::WHITE_SCREEN_NOT_OBSERVABLE; i++) {
        CHECK(recoveryTriggerText((RecoveryTrigger)i) != nullptr);
    }
}

// Cross-core handshake: Core 1 posts a request, only Core 0 can take it, no
// stacking while busy, and the result is delivered back exactly once.
static void test_request_mailbox(void) {
    RecoveryRequestMailbox mb;
    RecoveryTrigger out = RecoveryTrigger::NONE;

    // Nothing pending initially.
    CHECK(!mb.pending());
    CHECK(!mb.busy());
    CHECK(!mb.take(out));

    // NONE is never a valid request.
    CHECK(!mb.request(RecoveryTrigger::NONE));

    // Core 1 posts a request.
    CHECK(mb.request(RecoveryTrigger::RENDER_STALLED));
    CHECK(mb.pending());
    // A second request while one is pending is rejected (no stacking).
    CHECK(!mb.request(RecoveryTrigger::TFT_CONTROLLER_STATUS_LOST));

    // Core 0 takes it exactly once; now busy, nothing pending.
    CHECK(mb.take(out));
    CHECK(out == RecoveryTrigger::RENDER_STALLED);
    CHECK(!mb.pending());
    CHECK(mb.busy());
    CHECK(!mb.take(out));

    // No new requests accepted while a recovery is in flight.
    CHECK(!mb.request(RecoveryTrigger::RECOVERY_MANUAL_REQUEST));

    // Core 0 reports completion; Core 1 consumes the result once.
    mb.done(true, 4200);
    bool ok = false; uint32_t dur = 0;
    CHECK(mb.takeResult(ok, dur));
    CHECK(ok == true);
    CHECK(dur == 4200);
    CHECK(!mb.takeResult(ok, dur));   // consumed only once
    CHECK(!mb.busy());

    // After completion, new requests are accepted again.
    CHECK(mb.request(RecoveryTrigger::RECOVERY_MANUAL_REQUEST));
    CHECK(mb.pending());
}

// ---- Recovery runner (audit §4 order via mocks + §6 retries/final failure) --
// A recorder captures the exact order of hardware calls; the verify mock is
// scripted so we can drive success, retry-then-succeed, and exhausted-failure.
namespace {
struct RunnerMock {
    std::vector<RecoveryStep> steps;   // order of onStep callbacks
    std::vector<RecoveryStep> hw;      // order of REAL op invocations
    int  verifyCalls   = 0;
    int  verifySucceedOn = 1;          // 1 = first attempt succeeds
    bool tftInitBeforeIsolated = false;
    bool sawResetBeforeInit    = false;
    bool csHighBeforeReset     = false;
    bool resumed = false;
};
}  // namespace

static RunnerMock* g_rm = nullptr;
static void mrec(void* v, RecoveryStep s) { (void)v; g_rm->steps.push_back(s); }
static void mstop(void*)   { g_rm->hw.push_back(RecoveryStep::STOP_RENDER_TOUCH); }
static void mblock(void*)  { g_rm->hw.push_back(RecoveryStep::BLOCK_TFT_ACCESS); }
static void mtcs(void*)    { g_rm->hw.push_back(RecoveryStep::TFT_CS_HIGH); }
static void mtouch(void*)  { g_rm->hw.push_back(RecoveryStep::TOUCH_CS_HIGH); }
static void mspi(void*)    { g_rm->hw.push_back(RecoveryStep::CLOSE_SPI); }
static void mpulse(void*)  { g_rm->hw.push_back(RecoveryStep::PULSE_GPIO38); }
static void minit(void*)   { g_rm->hw.push_back(RecoveryStep::TFT_INIT); }
static void mrot(void*)    { g_rm->hw.push_back(RecoveryStep::SET_ROTATION_1); }
static void mcal(void*)    { g_rm->hw.push_back(RecoveryStep::RESTORE_TOUCH_CAL); }
static void mbl(void*)     { g_rm->hw.push_back(RecoveryStep::RESTORE_BACKLIGHT); }
static void minv(void*)    { g_rm->hw.push_back(RecoveryStep::INVALIDATE_CACHES); }
static void mredraw(void*) { g_rm->hw.push_back(RecoveryStep::FORCE_FULL_REDRAW); }
static bool mverify(void*) { g_rm->hw.push_back(RecoveryStep::VERIFY);
                             return (++g_rm->verifyCalls) >= g_rm->verifySucceedOn; }
static void mresume(void*) { g_rm->hw.push_back(RecoveryStep::RESUME); g_rm->resumed = true; }

static RecoveryOps makeMockOps() {
    RecoveryOps ops{};
    ops.stopRenderTouch=mstop; ops.blockBus=mblock; ops.tftCsHigh=mtcs;
    ops.touchCsHigh=mtouch; ops.closeSpi=mspi; ops.pulseReset=mpulse;
    ops.tftInit=minit; ops.setRotation=mrot; ops.restoreTouchCal=mcal;
    ops.restoreBacklight=mbl; ops.invalidateCaches=minv; ops.forceFullRedraw=mredraw;
    ops.verify=mverify; ops.resumeRenderTouch=mresume; ops.onStep=mrec;
    return ops;
}

// Full happy path: one attempt, verified, exact hardware order, resumed.
static void test_runner_order_and_success(void) {
    RunnerMock rm; g_rm = &rm; rm.verifySucceedOn = 1;
    RecoveryOps ops = makeMockOps();
    RecoveryOutcome oc = runRecovery(ops, nullptr, /*maxAttempts=*/3);

    CHECK(oc.verified == true);
    CHECK(oc.exhausted == false);
    CHECK(oc.attempts == 1);
    CHECK(rm.resumed == true);

    // The onStep recorder and the real op order must be identical and equal to
    // the canonical single-attempt sequence.
    CHECK(rm.steps.size() == (size_t)kRecoveryStepCount);
    CHECK(rm.hw.size()    == (size_t)kRecoveryStepCount);
    const RecoveryStep* seq = recoverySequence();
    bool same = (rm.hw.size() == (size_t)kRecoveryStepCount);
    for (size_t i = 0; same && i < rm.hw.size(); ++i) same = (rm.hw[i] == seq[i]);
    CHECK(same);

    // Ordering invariants: both CS HIGH + SPI closed BEFORE the reset pulse,
    // reset pulse BEFORE tft.init(), invalidate BEFORE full redraw.
    auto idx = [&](RecoveryStep s){ for (size_t i=0;i<rm.hw.size();++i) if (rm.hw[i]==s) return (int)i; return -1; };
    CHECK(idx(RecoveryStep::TFT_CS_HIGH)   < idx(RecoveryStep::PULSE_GPIO38));
    CHECK(idx(RecoveryStep::TOUCH_CS_HIGH) < idx(RecoveryStep::PULSE_GPIO38));
    CHECK(idx(RecoveryStep::CLOSE_SPI)     < idx(RecoveryStep::PULSE_GPIO38));
    CHECK(idx(RecoveryStep::PULSE_GPIO38)  < idx(RecoveryStep::TFT_INIT));
    CHECK(idx(RecoveryStep::INVALIDATE_CACHES) < idx(RecoveryStep::FORCE_FULL_REDRAW));
    CHECK(idx(RecoveryStep::VERIFY)        < idx(RecoveryStep::RESUME));
}

// Retry: verify fails once then succeeds → 2 attempts, reset/init repeated,
// isolation + resume happen exactly once, and it ends verified.
static void test_runner_retry_then_success(void) {
    RunnerMock rm; g_rm = &rm; rm.verifySucceedOn = 2;
    RecoveryOps ops = makeMockOps();
    RecoveryOutcome oc = runRecovery(ops, nullptr, /*maxAttempts=*/3);

    CHECK(oc.verified == true);
    CHECK(oc.exhausted == false);
    CHECK(oc.attempts == 2);

    auto count = [&](RecoveryStep s){ int n=0; for (auto x: rm.hw) if (x==s) ++n; return n; };
    CHECK(count(RecoveryStep::PULSE_GPIO38) == 2);   // reset pulsed each attempt
    CHECK(count(RecoveryStep::TFT_INIT)     == 2);
    CHECK(count(RecoveryStep::VERIFY)       == 2);
    CHECK(count(RecoveryStep::STOP_RENDER_TOUCH) == 1);  // isolation only once
    CHECK(count(RecoveryStep::CLOSE_SPI)    == 1);
    CHECK(count(RecoveryStep::RESUME)       == 1);       // resumed once at the end
    CHECK(rm.resumed == true);
}

// Final failure: verify never succeeds → attempts == maxAttempts, exhausted,
// NOT verified, yet render/touch is still resumed (degraded but alive).
static void test_runner_final_failure(void) {
    RunnerMock rm; g_rm = &rm; rm.verifySucceedOn = 99;  // never succeeds
    RecoveryOps ops = makeMockOps();
    RecoveryOutcome oc = runRecovery(ops, nullptr, /*maxAttempts=*/3);

    CHECK(oc.verified == false);
    CHECK(oc.exhausted == true);
    CHECK(oc.attempts == 3);
    CHECK(rm.verifyCalls == 3);
    CHECK(rm.resumed == true);   // still resumes after exhausting retries
}

int main() {
    test_sequence_order();
    test_banner_fields();
    test_banner_no_false_white_claim();
    test_banner_truncation();
    test_action_per_cause();
    test_trigger_classification();
    test_request_mailbox();
    test_runner_order_and_success();
    test_runner_retry_then_success();
    test_runner_final_failure();
    printf("display_recovery: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
