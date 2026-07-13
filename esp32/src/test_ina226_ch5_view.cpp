// =============================================================================
// Host test for esp32/src/ina226_ch5_view.h (0x318 DIAG_INA_CH5, Problem 4).
//
// Cross-verifies the ESP32 decoder against the REAL STM32 packer
// (Core/Inc/ina226_ch5_frame.h) and the REAL classifier
// (Core/Src/ina226_channel_diag.c): build snapshot -> classify -> pack (STM32
// side) -> decode (ESP32 side) -> assert, plus the CH5 status / SOLUCIÓN maps.
// Proves a reversed (negative) steering current survives the round-trip
// SIGNED and is never flattened to zero, and that MISSING (no ACK) is a
// different state from n/d (no frame at all).
//
// Build (host):
//   g++ -std=c++17 -Wall -Wextra -Werror -Iesp32/src -Iesp32/include -ICore/Inc
//       esp32/src/test_ina226_ch5_view.cpp Core/Src/ina226_channel_diag.c
//       -o /tmp/t   (see firmware-validation.yml for the exact command)
// =============================================================================

#include "ina226_ch5_view.h"

#include "ina226_channel_diag.h"
#include "ina226_ch5_frame.h"

#include <cstdio>
#include <cstring>

static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg) do {                                     \
    ++g_checks;                                                    \
    if (!(cond)) { ++g_fails;                                      \
        std::printf("  FAIL: %s (line %d)\n", (msg), __LINE__); }  \
} while (0)

namespace v5 = ina226_ch5_view;

static Ina226ChannelDiag base_snapshot() {
    Ina226ChannelDiag d;
    std::memset(&d, 0, sizeof d);
    d.channel          = 5;
    d.mux_channel      = 5;
    d.expected_address = 0x40;
    d.detected_address = 0x40;
    d.mux_select_ok      = true;
    d.i2c_ack            = true;
    d.manufacturer_id_ok = true;
    d.die_id_ok          = true;
    d.config_write_ok    = true;
    d.config_readback_ok = true;
    d.shunt_read_ok      = true;
    d.bus_read_ok        = true;
    d.raw_shunt          = 1200;
    d.shunt_uv           = 3000;
    d.bus_mv             = 24000;
    d.signed_current_ma  = 2000;
    d.sample_age_ms      = 50;
    d.channel_powered    = true;
    return d;
}

static void test_cross_decode_ok() {
    std::printf("Scenario: STM32 pack -> ESP32 decode (OK)\n");

    Ina226ChannelDiag d = base_snapshot();
    d.fault_reason = Ina226_ClassifyChannel(&d);

    uint8_t bytes[8];
    Ina226Ch5_PackFrame(&d, bytes);

    v5::Ina226Ch5View v;
    CHECK(v5::decode(bytes, 8, v), "decode ok");
    CHECK(v.reason == can::INA_CH5_OK, "reason OK");
    CHECK(v.muxSelectOk && v.i2cAck && v.identityOk && v.configOk, "flags set");
    CHECK(v.shuntReadOk && v.busReadOk && v.channelPowered, "read flags set");
    CHECK(!v.stale, "not stale");
    CHECK(v.rawShunt == 1200, "raw shunt");
    CHECK(v.shuntMicroVolts == 3000, "shunt uV");
    CHECK(v.currentMilliAmps == 2000, "current mA");
    CHECK(v.busMilliVolts == 24000, "bus mV");
    CHECK(v.sampleAgeMs == 50, "age");
    CHECK(!v5::isFault(v.reason), "OK is not a fault");
}

static void test_cross_decode_reversed() {
    std::printf("Scenario: reversed current stays signed\n");

    Ina226ChannelDiag d = base_snapshot();
    d.raw_shunt         = -1600;   // -4000 µV
    d.shunt_uv          = -4000;
    d.signed_current_ma = -2667;
    d.fault_reason = Ina226_ClassifyChannel(&d);

    uint8_t bytes[8];
    Ina226Ch5_PackFrame(&d, bytes);

    v5::Ina226Ch5View v;
    CHECK(v5::decode(bytes, 8, v), "decode ok");
    CHECK(v.reason == can::INA_CH5_POLARITY_REVERSED, "reason POLARITY REVERSED");
    CHECK(v.rawShunt == -1600, "raw shunt negative");
    CHECK(v.shuntMicroVolts == -4000, "shunt uV negative (never zeroed)");
    CHECK(v.currentMilliAmps == -2667, "current mA negative (never zeroed)");
    CHECK(v5::currentAmps(v) < 0.0f, "amps negative");
    CHECK(std::strcmp(v5::statusText(v.reason), "CH5 POLARITY REVERSED") == 0,
          "status word");
}

static void test_cross_decode_missing_vs_nd() {
    std::printf("Scenario: MISSING (no ack) is not n/d (no frame)\n");

    Ina226ChannelDiag d = base_snapshot();
    d.i2c_ack = false;
    d.detected_address = 0;
    d.shunt_read_ok = false;
    d.bus_read_ok = false;
    d.fault_reason = Ina226_ClassifyChannel(&d);

    uint8_t bytes[8];
    Ina226Ch5_PackFrame(&d, bytes);

    v5::Ina226Ch5View v;
    CHECK(v5::decode(bytes, 8, v), "decode ok");
    CHECK(v.reason == can::INA_CH5_MISSING, "reason MISSING");
    CHECK(v.muxSelectOk, "mux still selected");
    CHECK(!v.i2cAck, "no ACK flag");
    CHECK(std::strcmp(v5::statusText(v.reason), "CH5 MISSING") == 0,
          "status MISSING");
    // n/d is the ABSENCE of any frame — a separate, explicit string.
    CHECK(std::strcmp(v5::notAvailableText(), "CH5 n/d (SIN CONTRATO CAN)") == 0,
          "n/d distinct text");
    CHECK(std::strcmp(v5::statusText(v.reason),
                      v5::notAvailableText()) != 0, "MISSING != n/d");
}

static void test_stale_and_never() {
    std::printf("Scenario: stale + never-sampled\n");

    // Stale: chip readable but old sample.
    Ina226ChannelDiag d = base_snapshot();
    d.sample_age_ms = INA226_DIAG_STALE_MS + 100;
    d.fault_reason = Ina226_ClassifyChannel(&d);
    uint8_t bytes[8];
    Ina226Ch5_PackFrame(&d, bytes);
    v5::Ina226Ch5View v;
    CHECK(v5::decode(bytes, 8, v), "decode ok");
    CHECK(v.reason == can::INA_CH5_STALE, "reason STALE");
    CHECK(v.stale, "stale flag");

    // Never sampled: age saturates at 0xFFFF.
    d = base_snapshot();
    d.sample_age_ms = 0xFFFFFFFFU;
    d.fault_reason = Ina226_ClassifyChannel(&d);
    Ina226Ch5_PackFrame(&d, bytes);
    CHECK(v5::decode(bytes, 8, v), "decode ok");
    CHECK(v.sampleAgeMs == 0xFFFFU, "age saturates");
    CHECK(v.stale, "never-sampled is stale");
}

static void test_present_no_shunt() {
    std::printf("Scenario: present but no shunt drop\n");

    Ina226ChannelDiag d = base_snapshot();
    d.raw_shunt         = 5;      // 12.5 µV < 50 µV floor
    d.shunt_uv          = 12;
    d.signed_current_ma = 8;
    d.fault_reason = Ina226_ClassifyChannel(&d);
    uint8_t bytes[8];
    Ina226Ch5_PackFrame(&d, bytes);
    v5::Ina226Ch5View v;
    CHECK(v5::decode(bytes, 8, v), "decode ok");
    CHECK(v.reason == can::INA_CH5_PRESENT_NO_SHUNT, "reason PRESENT NO SHUNT");
    CHECK(std::strcmp(v5::statusText(v.reason), "CH5 PRESENT NO SHUNT") == 0,
          "status word");
}

static void test_text_maps() {
    std::printf("Scenario: text maps non-empty + freshness\n");

    const uint8_t reasons[] = {
        can::INA_CH5_OK, can::INA_CH5_PRESENT_NO_SHUNT,
        can::INA_CH5_POLARITY_REVERSED, can::INA_CH5_STALE,
        can::INA_CH5_MUX_SELECT_FAIL, can::INA_CH5_MISSING,
        can::INA_CH5_WRONG_ID, can::INA_CH5_CONFIG_LOST,
        can::INA_CH5_READ_FAIL, can::INA_CH5_UNKNOWN
    };
    for (uint8_t r : reasons) {
        CHECK(v5::statusText(r)[0] != '\0', "status non-empty");
        CHECK(v5::solutionText(r)[0] != '\0', "solution non-empty");
    }
    CHECK(std::strcmp(v5::statusText(can::INA_CH5_WRONG_ID),
                      "CH5 WRONG ADDRESS") == 0, "wrong-id label");
    CHECK(std::strcmp(v5::statusText(can::INA_CH5_CONFIG_LOST),
                      "CH5 CONFIG FAIL") == 0, "config-fail label");

    // Freshness is millis()-wrap safe.
    CHECK(v5::freshness(false, 0, 0) == v5::Freshness::NEVER_RECEIVED,
          "invalid -> never");
    CHECK(v5::freshness(true, 500, 0) == v5::Freshness::VALID, "fresh -> valid");
    CHECK(v5::freshness(true, 5000, 0) == v5::Freshness::STALE, "old -> stale");
    // Wrap-around: now < stamp but within window.
    CHECK(v5::freshness(true, 5, 0xFFFFFFF0U) == v5::Freshness::VALID,
          "wrap valid");
}

int main() {
    test_cross_decode_ok();
    test_cross_decode_reversed();
    test_cross_decode_missing_vs_nd();
    test_stale_and_never();
    test_present_no_shunt();
    test_text_maps();

    std::printf("ina226_ch5_view: %d run, %d failed\n", g_checks, g_fails);
    std::printf("RESULT: %s\n", g_fails == 0 ? "PASS" : "FAIL");
    return g_fails == 0 ? 0 : 1;
}
