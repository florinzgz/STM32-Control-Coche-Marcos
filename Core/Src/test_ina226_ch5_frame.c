/**
  ****************************************************************************
  * @file    test_ina226_ch5_frame.c
  * @brief   Host integration test (Problem 4): drives the REAL steering-INA
  *          channel classifier (Ina226_ClassifyChannel) and the shared 0x318
  *          pack/unpack used by the firmware and the ESP32, over the audit's
  *          key scenarios: MISSING vs WRONG-ID vs CONFIG-FAIL vs
  *          PRESENT-NO-SHUNT vs POLARITY-REVERSED vs STALE vs OK, plus a full
  *          round-trip that proves a signed (reversed) current is never zeroed.
  *
  * Build (host):
  *   gcc -std=c11 -DHOST_TEST -ICore/Inc Core/Src/test_ina226_ch5_frame.c \
  *       Core/Src/ina226_channel_diag.c -lm -o /tmp/t && /tmp/t
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
    if (!(cond)) { g_fails++;                                       \
        printf("  FAIL: %s (line %d)\n", (msg), __LINE__); }        \
} while (0)

/* Local logical index constant (mux channel 5). */
#ifndef INA226_CHANNEL_STEER_LOGICAL
#define INA226_CHANNEL_STEER_LOGICAL 5
#endif

/* A present, healthy steering INA226 snapshot: mux selected, chip ACKs,
 * identity + config OK, both registers read, fresh, forward current. */
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
    d.raw_shunt          = 1200;                 /* 1200 * 2.5 = 3000 µV */
    d.shunt_uv           = 3000;
    d.bus_mv             = 24000;
    d.signed_current_ma  = 2000;                 /* 3000 µV / 1.5 mΩ */
    d.sample_age_ms      = 50;
    d.channel_powered    = true;
    return d;
}

static void test_classifier_scenarios(void)
{
    printf("test_classifier_scenarios\n");

    /* OK */
    Ina226ChannelDiag d = base_snapshot();
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_OK, "healthy -> OK");

    /* MISSING: no ACK (chip absent / unpowered) — must NOT be n/d. */
    d = base_snapshot();
    d.i2c_ack = false;
    d.detected_address = 0;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_MISSING, "no ack -> MISSING");

    /* MUX select failure (bus/mux problem, not a dead sensor). */
    d = base_snapshot();
    d.mux_select_ok = false;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_MUX_SELECT_FAIL, "mux fail");

    /* WRONG ID: acks but identity mismatch (wrong address device). */
    d = base_snapshot();
    d.manufacturer_id_ok = false;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_WRONG_ID, "wrong id");

    /* CONFIG FAIL: identity OK but config did not stick. */
    d = base_snapshot();
    d.config_readback_ok = false;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_CONFIG_LOST, "config lost");

    /* STALE: present & readable but old sample — never a valid 0 A. */
    d = base_snapshot();
    d.sample_age_ms = INA226_DIAG_STALE_MS + 10;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_STALE, "stale");

    /* POLARITY REVERSED: sizeable negative current under forward demand. */
    d = base_snapshot();
    d.raw_shunt         = -1200;
    d.shunt_uv          = -3000;
    d.signed_current_ma = -2000;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_POLARITY_REVERSED, "polarity");

    /* PRESENT NO SHUNT: chip fine, ~0 shunt drop (external shunt not wired). */
    d = base_snapshot();
    d.raw_shunt         = 5;      /* 5 * 2.5 = 12.5 µV < 50 µV floor */
    d.shunt_uv          = 12;
    d.signed_current_ma = 8;
    CHECK(Ina226_ClassifyChannel(&d) == INA226_CH_PRESENT_NO_SHUNT, "no shunt");
}

/* Verify the shared 0x318 packer/unpacker preserves every field and derives
 * signed µV / mA identically to the firmware — a reversed current stays
 * negative through the wire. */
static void test_frame_roundtrip(void)
{
    printf("test_frame_roundtrip\n");

    Ina226ChannelDiag d = base_snapshot();
    d.raw_shunt = -1600;             /* reversed: -1600 * 2.5 = -4000 µV */
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
    CHECK(f.shunt_uv == -4000, "shunt uV signed (never zeroed)");
    CHECK(f.current_ma == -2667, "current mA signed (never zeroed)");
    CHECK(f.bus_mv == 23500, "bus mV");
    CHECK(f.sample_age_ms == 120, "age");

    /* Never-sampled age saturates at 0xFFFF and sets the stale flag. */
    d = base_snapshot();
    d.sample_age_ms = 0xFFFFFFFFU;
    Ina226Ch5_PackFrame(&d, wire);
    Ina226Ch5_UnpackFrame(wire, &f);
    CHECK(f.sample_age_ms == 0xFFFFU, "never-sampled age saturates");
    CHECK(f.stale == true, "never-sampled is stale");

    /* MISSING keeps the flags cleared so the ESP32 can distinguish it from
     * n/d (which is the ABSENCE of this frame entirely). */
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
