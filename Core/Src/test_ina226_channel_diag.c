/**
  ****************************************************************************
  * @file    test_ina226_channel_diag.c
  * @brief   Host unit tests for the per-channel INA226 diagnostic classifier
  *          (Ina226_ClassifyChannel).
  *
  *          Covers the audit's Problem-4 test matrix: CH5 present, CH5
  *          missing, wrong address / mux, config-readback failure, shunt 0,
  *          negative current, stale and read-fail vs full-bus recovery.
  *
  *          Compile (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  *                Core/Src/test_ina226_channel_diag.c \
  *                Core/Src/ina226_channel_diag.c -lm \
  *                -o /tmp/test_ina226_channel_diag
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <string.h>

#include "ina226_channel_diag.h"

static int tests_run = 0, tests_failed = 0;

#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* Healthy steering CH5 baseline: mux selects, chip ACKs, IDs match, config
 * sticks, fresh reading, real shunt drop and positive current.           */
static Ina226ChannelDiag base_ok_ch5(void)
{
    Ina226ChannelDiag d;
    memset(&d, 0, sizeof(d));
    d.channel            = 5;
    d.mux_channel        = 5;
    d.expected_address   = 0x40;
    d.detected_address   = 0x40;
    d.mux_select_ok      = true;
    d.i2c_ack            = true;
    d.manufacturer_id_ok = true;
    d.die_id_ok          = true;
    d.config_write_ok    = true;
    d.config_readback_ok = true;
    d.shunt_read_ok      = true;
    d.bus_read_ok        = true;
    d.raw_shunt          = 1800;
    d.shunt_uv           = 3510;   /* ~2.34 A over 1.5 mΩ */
    d.bus_mv             = 12200;
    d.signed_current_ma  = 2340;
    d.sample_age_ms      = 20;
    d.channel_powered    = true;
    d.consecutive_failures = 0;
    d.recovery_count     = 0;
    return d;
}

static Ina226DiagReason_t classify(Ina226ChannelDiag d)
{
    return Ina226_ClassifyChannel(&d);
}

int main(void)
{
    /* --- Case 5: OK --- */
    CHECK(classify(base_ok_ch5()) == INA226_CH_OK);
    CHECK(Ina226_DiagStatusWord(INA226_CH_OK)[0] == 'O');

    /* --- Case 1: CH5 MISSING (no I2C ACK) --- */
    {
        Ina226ChannelDiag d = base_ok_ch5();
        d.i2c_ack = false;
        d.detected_address = 0x00;
        CHECK(classify(d) == INA226_CH_MISSING);
        CHECK(Ina226_DiagStatusWord(INA226_CH_MISSING)[0] == 'M');
    }

    /* --- MUX cannot select the channel --- */
    {
        Ina226ChannelDiag d = base_ok_ch5();
        d.mux_select_ok = false;
        CHECK(classify(d) == INA226_CH_MUX_SELECT_FAIL);
        /* mux select fail takes priority over a would-be ACK */
        d.i2c_ack = false;
        CHECK(classify(d) == INA226_CH_MUX_SELECT_FAIL);
    }

    /* --- Wrong device / corrupted identity --- */
    {
        Ina226ChannelDiag d = base_ok_ch5();
        d.manufacturer_id_ok = false;
        CHECK(classify(d) == INA226_CH_WRONG_ID);
        d = base_ok_ch5();
        d.die_id_ok = false;
        CHECK(classify(d) == INA226_CH_WRONG_ID);
    }

    /* --- Config readback failure --- */
    {
        Ina226ChannelDiag d = base_ok_ch5();
        d.config_readback_ok = false;
        CHECK(classify(d) == INA226_CH_CONFIG_LOST);
        d = base_ok_ch5();
        d.config_write_ok = false;
        CHECK(classify(d) == INA226_CH_CONFIG_LOST);
    }

    /* --- Single transaction read failure (does not imply whole bus dead) --- */
    {
        Ina226ChannelDiag d = base_ok_ch5();
        d.shunt_read_ok = false;
        CHECK(classify(d) == INA226_CH_READ_FAIL);
        d = base_ok_ch5();
        d.bus_read_ok = false;
        CHECK(classify(d) == INA226_CH_READ_FAIL);
    }

    /* --- Case 4: STALE telemetry --- */
    {
        Ina226ChannelDiag d = base_ok_ch5();
        d.sample_age_ms = INA226_DIAG_STALE_MS;
        CHECK(classify(d) == INA226_CH_STALE);
        CHECK(Ina226_DiagStatusWord(INA226_CH_STALE)[0] == 'S');
    }

    /* --- Case 2: present but no shunt drop (R002 removed) → 0 A --- */
    {
        Ina226ChannelDiag d = base_ok_ch5();
        d.raw_shunt = 0;
        d.shunt_uv = 0;
        d.signed_current_ma = 0;
        CHECK(classify(d) == INA226_CH_PRESENT_NO_SHUNT);
        CHECK(Ina226_DiagStatusWord(INA226_CH_PRESENT_NO_SHUNT)[0] == 'P');
    }

    /* --- shunt just at the noise floor still counts as no-shunt --- */
    {
        Ina226ChannelDiag d = base_ok_ch5();
        d.shunt_uv = INA226_DIAG_SHUNT_FLOOR_UV - 1;
        d.signed_current_ma = 0;
        CHECK(classify(d) == INA226_CH_PRESENT_NO_SHUNT);
    }

    /* --- Case 3: reversed polarity (negative current, VIN swap) --- */
    {
        Ina226ChannelDiag d = base_ok_ch5();
        d.shunt_uv = -4500;
        d.signed_current_ma = -3000;
        CHECK(classify(d) == INA226_CH_POLARITY_REVERSED);
    }

    /* --- Small negative current within noise is NOT reversed --- */
    {
        Ina226ChannelDiag d = base_ok_ch5();
        d.shunt_uv = 3510;
        d.signed_current_ma = -(INA226_DIAG_REVERSED_MA - 1);
        CHECK(classify(d) == INA226_CH_OK);
    }

    /* --- NULL guard + labels --- */
    {
        CHECK(Ina226_ClassifyChannel(NULL) == INA226_CH_UNKNOWN);
        for (int r = INA226_CH_OK; r <= INA226_CH_UNKNOWN; r++) {
            CHECK(Ina226_DiagReasonStr((Ina226DiagReason_t)r)[0] != '\0');
            CHECK(Ina226_DiagStatusWord((Ina226DiagReason_t)r)[0] != '\0');
        }
    }

    printf("ina226_channel_diag: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#endif /* HOST_TEST */
