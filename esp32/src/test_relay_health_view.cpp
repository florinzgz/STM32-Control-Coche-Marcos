// =============================================================================
// Host test for esp32/src/relay_health_view.h (0x317 DIAG_RELAY_HEALTH).
//
// Cross-verifies the ESP32 decoder against the REAL STM32 packer
// (Core/Inc/relay_health_frame.h) and the REAL classifier
// (Core/Src/relay_health_diag.c): build snapshot -> classify -> pack (STM32
// side) -> decode (ESP32 side) -> assert, plus the MOTIVO/SOLUCIÓN text maps.
//
// Build (host):
//   g++ -std=c++17 -Wall -Wextra -Werror -Iesp32/src -Iesp32/include -ICore/Inc
//       esp32/src/test_relay_health_view.cpp Core/Src/relay_health_diag.c
//       -o /tmp/t   (see firmware-validation.yml for the exact command)
// =============================================================================

#include "relay_health_view.h"

#include "relay_health_diag.h"
#include "relay_health_frame.h"

#include <cstdio>
#include <cstring>

static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg) do {                                     \
    ++g_checks;                                                    \
    if (!(cond)) { ++g_fails;                                      \
        std::printf("  FAIL: %s (line %d)\n", (msg), __LINE__); }  \
} while (0)

namespace rv = relay_health_view;

static RelayHealthDiag base_snapshot() {
    RelayHealthDiag d;
    std::memset(&d, 0, sizeof d);
    d.relay_commanded         = true;
    d.relay_sequence_complete = true;
    d.power_ready             = true;
    d.throttle_pct            = 40.0f;
    d.final_pwm_pct           = 40.0f;
    d.any_wheel_moving        = true;
    d.ina_ok_mask             = 0x0F;
    d.ina_expected_mask       = 0x0F;
    d.ina_valid_mask          = 0x0F;
    d.current_sample_age_ms   = 20;
    d.current_sum_abs         = 12.0f;
    d.current_signed_sum      = 12.0f;
    d.current_threshold       = 2.3f;
    d.post_relay_voltage_present = true;
    d.battery_consumption_rising = true;
    return d;
}

static void test_cross_decode() {
    std::printf("Scenario: STM32 pack -> ESP32 decode\n");

    // Rule A: wheels move under PWM, shunt reads 0 A -> CURRENT_SENSE_INVALID.
    RelayHealthDiag d = base_snapshot();
    d.current_sum_abs = 0.0f;
    d.current_signed_sum = 0.0f;
    d.diagnostic_reason = Relay_ClassifyHealth(&d);

    uint8_t bytes[8];
    RelayHealth_PackFrame(&d, bytes);

    rv::RelayHealthView v;
    CHECK(rv::decode(bytes, 8, v), "decode ok");
    CHECK(v.reason == can::RELAY_DIAG_CURRENT_SENSE_INVALID,
          "reason CURRENT_SENSE_INVALID matches");
    CHECK(v.relayCommanded && v.seqComplete && v.powerReady, "relay flags");
    CHECK(v.anyWheelMoving, "wheel moving flag");
    CHECK(v.currentValid, "current valid flag");
    CHECK(!v.currentStale && !v.inaMissing, "fresh + present");
    CHECK(v.currentSumCa == 0, "0 centi-A");
    CHECK(v.throttlePct == 40 && v.finalPwmPct == 40, "throttle/pwm");
    CHECK(v.sampleAgeMs == 20, "age");

    // Healthy current path.
    d = base_snapshot();
    d.diagnostic_reason = Relay_ClassifyHealth(&d);
    RelayHealth_PackFrame(&d, bytes);
    CHECK(rv::decode(bytes, 8, v), "decode ok 2");
    CHECK(v.reason == can::RELAY_DIAG_OK, "healthy -> OK");
    CHECK(v.currentSumCa == 1200, "12A -> 1200 centi-A");
    CHECK(rv::currentAmps(v) > 11.99f && rv::currentAmps(v) < 12.01f, "amps back");

    // Stale + missing.
    d = base_snapshot();
    d.current_sample_age_ms = 5000;
    d.ina_ok_mask = 0x07;   // CH3 absent
    d.diagnostic_reason = Relay_ClassifyHealth(&d);   // INA missing
    RelayHealth_PackFrame(&d, bytes);
    CHECK(rv::decode(bytes, 8, v), "decode ok 3");
    CHECK(v.reason == can::RELAY_DIAG_INA_MISSING, "INA missing reason");
    CHECK(v.currentStale, "stale flag");
    CHECK(v.inaMissing, "ina missing flag");
    CHECK(v.sampleAgeMs == 5000, "age 5000");

    // Short frame rejected.
    CHECK(!rv::decode(bytes, 4, v), "short frame rejected");
}

static void test_text_maps() {
    std::printf("Scenario: MOTIVO / SOLUCIÓN / confidence text\n");

    CHECK(std::strcmp(rv::reasonText(can::RELAY_DIAG_CURRENT_SENSE_INVALID),
                      "SENSOR CORRIENTE INVALIDO") == 0, "motivo sense invalid");
    CHECK(std::strcmp(rv::reasonText(can::RELAY_DIAG_OPEN_SUSPECTED),
                      "RELE ABIERTO SOSPECHA") == 0, "motivo suspected");
    CHECK(std::strcmp(rv::confidenceText(can::RELAY_DIAG_OPEN_CONFIRMED),
                      "CONFIRMADO") == 0, "confidence confirmed");
    CHECK(std::strcmp(rv::confidenceText(can::RELAY_DIAG_OPEN_SUSPECTED),
                      "PROBABLE") == 0, "confidence probable");
    CHECK(std::strcmp(rv::confidenceText(can::RELAY_DIAG_INCONCLUSIVE),
                      "INCONCLUSO") == 0, "confidence inconcluso");
    CHECK(std::strcmp(rv::solutionText(can::RELAY_DIAG_CURRENT_SENSE_INVALID),
                      "REVISAR SHUNT/R002/CABLEADO INA") == 0, "solucion sense");
    CHECK(rv::isFault(can::RELAY_DIAG_CURRENT_SENSE_INVALID), "is fault");
    CHECK(!rv::isFault(can::RELAY_DIAG_OK), "OK not fault");
    CHECK(!rv::isFault(can::RELAY_DIAG_INCONCLUSIVE), "inconclusive not fault");

    // Every reason has non-empty text (no gaps).
    for (uint8_t r = 0; r <= can::RELAY_DIAG_INCONCLUSIVE; ++r) {
        CHECK(rv::reasonText(r)[0] != '\0', "reasonText non-empty");
        CHECK(rv::solutionText(r)[0] != '\0', "solutionText non-empty");
    }
}

static void test_freshness() {
    std::printf("Scenario: freshness\n");
    CHECK(rv::freshness(false, 0, 0) == rv::Freshness::NEVER_RECEIVED, "never");
    CHECK(rv::freshness(true, 1000, 500) == rv::Freshness::VALID, "valid");
    CHECK(rv::freshness(true, 5000, 500) == rv::Freshness::STALE, "stale");
}

int main() {
    std::printf("=== test_relay_health_view ===\n");
    test_cross_decode();
    test_text_maps();
    test_freshness();
    std::printf("Checks: %d, Failures: %d\n", g_checks, g_fails);
    if (g_fails == 0) {
        std::printf("RESULT: PASS\n");
        return 0;
    }
    std::printf("RESULT: FAIL\n");
    return 1;
}
