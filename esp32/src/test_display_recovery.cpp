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

int main() {
    test_sequence_order();
    test_banner_fields();
    test_banner_no_false_white_claim();
    test_banner_truncation();
    test_action_per_cause();
    printf("display_recovery: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
