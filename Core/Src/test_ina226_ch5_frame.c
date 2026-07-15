/**
  ****************************************************************************
  * @file    test_ina226_ch5_frame.c
  * @brief   Host integration test for the steering-INA channel classifier and
  *          shared 0x318 pack/unpack contract.
  ****************************************************************************
  */

#include "ina226_channel_diag.h"
#include "ina226_ch5_frame.h"

#include <stdio.h>
#include <string.h>

static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg) do {                                      \
    g_checks++;                                                     \
    if (!(cond)) { g_fails++;                                      \
        printf("  FAIL: %s (line %d)\n", (msg), __LINE__); }        \
} while (0)

#ifndef INA226_CHANNEL_STEER_LOGICAL
#define INA226_CHANNEL_STEER_LOGICAL 5
#endif

static Ina226ChannelDiag base_snapshot(void)
{
    Ina226ChannelDiag d;
    memset(&d, 0, sizeof d);
    d.channel          = INA226_CHANNEL_STEER_LOGICAL;
    d.mux_channel      = INA226_CHANNEL_STEER_LOGICAL;
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
    d.current_expected   = true;
    return d;
}

static void test_classifier_scenarios(void)
{
    printf("test_classifier_scenarios\n");

    Ina226ChannelDiag d = base_snapshot();
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_OK, "healthy -> OK");

    d = base_snapshot();
    d.current_expected = false;
    d.raw_shunt = 0;
    d.shunt_uv = 0;
    d.signed_current_ma = 0;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_OK,
          "idle zero current -> OK");

    d = base_snapshot();
    d.i2c_ack = false;
    d.detected_address = 0;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_MISSING,
          "no ack -> MISSING");

    d = base_snapshot();
    d.mux_select_ok = false;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_MUX_SELECT_FAIL,
          "mux fail");

    d = base_snapshot();
    d.manufacturer_id_ok = false;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_WRONG_ID,
          "wrong id");

    d = base_snapshot();
    d.config_readback_ok = false;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_CONFIG_LOST,
          "config lost");

    d = base_snapshot();
    d.sample_age_ms = INA226_DIAG_STALE_MS + 10;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_STALE,
          "stale");

    d = base_snapshot();
    d.raw_shunt         = -1200;
    d.shunt_uv          = -3000;
    d.signed_current_ma = -2000;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_POLARITY_REVERSED,
          "polarity");

    d = base_snapshot();
    d.raw_shunt         = 5;
    d.shunt_uv          = 12;
    d.signed_current_ma = 8;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_PRESENT_NO_SHUNT,
          "no shunt under demand");
}

static void test_frame_roundtrip(void)
{
    printf("test_frame_roundtrip\n");

    Ina226ChannelDiag d = base_snapshot();
    d.raw_shunt = -1600;
    d.bus_mv    = 23500;
    d.sample_age_ms = 120;
    d.fault_reason = Ina226_ClassifyChannel(&d);

    uint8_t wire[8];
    Ina226Ch5_PackFrame(&d, wire);

    Ina226Ch5Frame f;
    memset(&f, 0, sizeof f);
    Ina226Ch5_UnpackFrame(wire, &f);

    CHECK(f.reason == (uint8_t)d.fault_reason, "reason preserved");
    CHECK(f.mux_select_ok == true, "mux flag");
    CHECK(f.i2c_ack == true, "ack flag");
    CHECK(f.identity_ok == true, "identity flag");
    CHECK(f.config_ok == true, "config flag");
    CHECK(f.shunt_read_ok == true, "shunt flag");
    CHECK(f.bus_read_ok == true, "bus flag");
    CHECK(f.channel_powered == true, "powered flag");
    CHECK(f.stale == false, "not stale");
    CHECK(f.raw_shunt == -1600, "raw shunt signed preserved");
    CHECK(f.shunt_uv == -4000, "shunt uV signed");
    CHECK(f.current_ma == -2667, "current mA signed");
    CHECK(f.bus_mv == 23500, "bus mV");
    CHECK(f.sample_age_ms == 120, "age");

    d = base_snapshot();
    d.sample_age_ms = 0xFFFFFFFFU;
    Ina226Ch5_PackFrame(&d, wire);
    Ina226Ch5_UnpackFrame(wire, &f);
    CHECK(f.sample_age_ms == 0xFFFFU, "never-sampled age saturates");
    CHECK(f.stale == true, "never-sampled is stale");

    d = base_snapshot();
    d.i2c_ack = false;
    d.shunt_read_ok = false;
    d.bus_read_ok = false;
    d.fault_reason = Ina226_ClassifyChannel(&d);
    Ina226Ch5_PackFrame(&d, wire);
    Ina226Ch5_UnpackFrame(wire, &f);
    CHECK(f.reason == INA226_CH_MISSING, "missing reason");
    CHECK(f.i2c_ack == false, "missing: no ack flag");
    CHECK(f.mux_select_ok == true, "missing: mux still selected");
}

int main(void)
{
    test_classifier_scenarios();
    test_frame_roundtrip();

    printf("ina226_ch5_frame: %d run, %d failed\n", g_checks, g_fails);
    printf("RESULT: %s\n", g_fails == 0 ? "PASS" : "FAIL");
    return g_fails == 0 ? 0 : 1;
}
