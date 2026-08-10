/**
  ****************************************************************************
  * @file    test_wheel_equality_frame.c
  * @brief   Host round-trip test for the 0x31D DIAG_WHEEL_EQUALITY CAN frame
  *          (Bloque C6, PR #445 Hito 2 — wheel-equality / BTS7960 health).
  *
  * Build (host):
  *   gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE -ICore/Inc \
  *       Core/Src/test_wheel_equality_frame.c -lm -o /tmp/t && /tmp/t
  ****************************************************************************
  */

#include "wheel_equality_frame.h"

#include <stdio.h>
#include <string.h>

static int failed;
#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failed++; \
    } \
} while (0)

static void test_speed_round_trip_typical(void)
{
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.wheel = 2U;              /* RL */
    in.field_id = WHEQ_FIELD_SPEED;
    in.phase2_included = true;
    in.pulses_per_sec_25 = 610U;
    in.pulses_per_sec_50 = 1220U;
    in.normalized_speed_x1000 = 2000U;
    in.deviation_pct = 3U;

    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);
    CHECK((bytes[0] & 0x03U) == 2U);
    CHECK(((bytes[0] >> 2) & 0x03U) == WHEQ_FIELD_SPEED);
    CHECK((bytes[0] & (1U << 4)) != 0U);

    WheelEqualityFrame_t out;
    memset(&out, 0xAA, sizeof(out));
    CHECK(WheelEqualityFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.wheel == 2U);
    CHECK(out.field_id == WHEQ_FIELD_SPEED);
    CHECK(out.phase2_included == true);
    CHECK(out.pulses_per_sec_25 == 610U);
    CHECK(out.pulses_per_sec_50 == 1220U);
    CHECK(out.normalized_speed_x1000 == 2000U);
    CHECK(out.deviation_pct == 3U);
}

static void test_current_round_trip_typical(void)
{
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.wheel = 0U;              /* FL */
    in.field_id = WHEQ_FIELD_CURRENT;
    in.current_ma_25 = 3200U;
    in.current_ma_50 = 6100U;
    in.slope_ma_per_pct_x10 = 1160U;   /* 116.0 mA/% */
    in.probable_cause = 1U;            /* MECH_RESISTANCE */

    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);

    WheelEqualityFrame_t out;
    CHECK(WheelEqualityFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.wheel == 0U);
    CHECK(out.field_id == WHEQ_FIELD_CURRENT);
    CHECK(out.current_ma_25 == 3200U);
    CHECK(out.current_ma_50 == 6100U);
    CHECK(out.slope_ma_per_pct_x10 == 1160U);
    CHECK(out.probable_cause == 1U);
}

static void test_health_round_trip_typical(void)
{
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.wheel = 3U;              /* RR */
    in.field_id = WHEQ_FIELD_HEALTH;
    in.asymmetry_pct_x10 = 230U;   /* 23.0 % -> FAIL_HALFBRIDGE band */
    in.delta_temp_c_x10 = 180U;    /* 18.0 C rise */
    in.halfbridge_verdict = 2U;    /* FAIL_HALFBRIDGE */
    in.temp_present = true;

    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);

    WheelEqualityFrame_t out;
    CHECK(WheelEqualityFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.wheel == 3U);
    CHECK(out.field_id == WHEQ_FIELD_HEALTH);
    CHECK(out.asymmetry_pct_x10 == 230U);
    CHECK(out.delta_temp_c_x10 == 180U);
    CHECK(out.halfbridge_verdict == 2U);
    CHECK(out.temp_present == true);
}

static void test_verdict_round_trip_typical(void)
{
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.wheel = 1U;              /* FR */
    in.field_id = WHEQ_FIELD_VERDICT;
    in.wheel_verdict = 2U;      /* FAIL */
    in.driver_verdict = 1U;     /* SOSPECHOSO */
    in.phase2_ran = true;

    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);

    WheelEqualityFrame_t out;
    CHECK(WheelEqualityFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.wheel == 1U);
    CHECK(out.field_id == WHEQ_FIELD_VERDICT);
    CHECK(out.wheel_verdict == 2U);
    CHECK(out.driver_verdict == 1U);
    CHECK(out.phase2_ran == true);
}

static void test_boundary_values_saturate(void)
{
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.wheel = 3U;
    in.field_id = WHEQ_FIELD_SPEED;
    /* out of u16 range -> caller must saturate before storing */
    in.pulses_per_sec_25 = WheelEqFrame_SatU16(70000);
    CHECK(in.pulses_per_sec_25 == 65535U);
    in.deviation_pct = WheelEqFrame_SatU8(500);
    CHECK(in.deviation_pct == 255U);

    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);

    WheelEqualityFrame_t out;
    CHECK(WheelEqualityFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.pulses_per_sec_25 == 65535U);
    CHECK(out.deviation_pct == 255U);
}

static void test_invalid_dlc_and_null_rejected(void)
{
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.wheel = 0U; in.field_id = WHEQ_FIELD_SPEED;

    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);

    WheelEqualityFrame_t out;
    memset(&out, 0x55, sizeof(out));
    CHECK(WheelEqualityFrame_Unpack(bytes, 7U, &out) == false);
    CHECK(WheelEqualityFrame_Unpack(bytes, 9U, &out) == false);
    CHECK(WheelEqualityFrame_Unpack(bytes, 0U, &out) == false);
    CHECK(out.wheel == 0x55U);   /* untouched by a rejected decode */

    CHECK(WheelEqualityFrame_Unpack(NULL, sizeof(bytes), &out) == false);
    CHECK(WheelEqualityFrame_Unpack(bytes, sizeof(bytes), NULL) == false);
    CHECK(WheelEqualityFrame_Pack(NULL, bytes) == 0U);
    CHECK(WheelEqualityFrame_Pack(&in, NULL) == 0U);
}

static void test_invalid_wheel_and_field_id_rejected(void)
{
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.wheel = 4U;              /* out of range (0..3) */
    in.field_id = WHEQ_FIELD_SPEED;
    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == 0U);

    in.wheel = 0U;
    in.field_id = 4U;           /* out of range (0..3) */
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == 0U);
}

static void test_all_wheels_all_fields_round_trip(void)
{
    for (uint8_t w = 0U; w < 4U; w++) {
        for (uint8_t f = 0U; f < 4U; f++) {
            WheelEqualityFrame_t in;
            memset(&in, 0, sizeof(in));
            in.wheel = w;
            in.field_id = f;
            uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
            CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);
            WheelEqualityFrame_t out;
            CHECK(WheelEqualityFrame_Unpack(bytes, sizeof(bytes), &out));
            CHECK(out.wheel == w);
            CHECK(out.field_id == f);
        }
    }
}

int main(void)
{
    test_speed_round_trip_typical();
    test_current_round_trip_typical();
    test_health_round_trip_typical();
    test_verdict_round_trip_typical();
    test_boundary_values_saturate();
    test_invalid_dlc_and_null_rejected();
    test_invalid_wheel_and_field_id_rejected();
    test_all_wheels_all_fields_round_trip();

    if (failed != 0) {
        fprintf(stderr, "wheel_equality_frame: %d assertion(s) FAILED\n", failed);
        return 1;
    }
    puts("wheel_equality_frame: all assertions PASS");
    return 0;
}
