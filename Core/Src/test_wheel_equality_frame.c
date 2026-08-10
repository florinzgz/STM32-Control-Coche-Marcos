/**
  ****************************************************************************
  * @file    test_wheel_equality_frame.c
  * @brief   Host round-trip test for the 0x31D DIAG_WHEEL_EQUALITY CAN frame
  *          (Hito 2, PR #445).
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

/* ---- SPEED field --------------------------------------------------------*/
static void test_speed_field_round_trip_typical(void)
{
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.wheel                    = 2U;   /* RL */
    in.field_id                 = WHEQ_FIELD_SPEED;
    in.phase2_included          = true;
    in.pulses_per_sec_25        = 300U;
    in.pulses_per_sec_50        = 600U;
    in.normalized_speed_x1000   = 2000U;
    in.deviation_pct            = 3U;

    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);
    CHECK((bytes[0] & 0x03U) == 2U);
    CHECK(((bytes[0] >> 2) & 0x03U) == WHEQ_FIELD_SPEED);
    CHECK((bytes[0] & (1U << 4)) != 0U);

    WheelEqualityFrame_t out;
    memset(&out, 0xAA, sizeof(out));
    CHECK(WheelEqualityFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.wheel                  == in.wheel);
    CHECK(out.field_id               == in.field_id);
    CHECK(out.phase2_included        == in.phase2_included);
    CHECK(out.pulses_per_sec_25      == in.pulses_per_sec_25);
    CHECK(out.pulses_per_sec_50      == in.pulses_per_sec_50);
    CHECK(out.normalized_speed_x1000 == in.normalized_speed_x1000);
    CHECK(out.deviation_pct          == in.deviation_pct);
}

/* ---- CURRENT field -------------------------------------------------------*/
static void test_current_field_round_trip_typical(void)
{
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.wheel                 = 0U;  /* FL */
    in.field_id              = WHEQ_FIELD_CURRENT;
    in.phase2_included       = false;
    in.current_ma_25         = 2250U;
    in.current_ma_50         = 3500U;
    in.slope_ma_per_pct_x10  = 500U;
    in.probable_cause        = 1U;  /* WHEQ_CAUSE_MECHANICAL wire value    */

    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);

    WheelEqualityFrame_t out;
    memset(&out, 0xAA, sizeof(out));
    CHECK(WheelEqualityFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.wheel                 == in.wheel);
    CHECK(out.field_id              == in.field_id);
    CHECK(out.phase2_included       == in.phase2_included);
    CHECK(out.current_ma_25         == in.current_ma_25);
    CHECK(out.current_ma_50         == in.current_ma_50);
    CHECK(out.slope_ma_per_pct_x10  == in.slope_ma_per_pct_x10);
    CHECK(out.probable_cause        == in.probable_cause);
}

/* ---- HEALTH field --------------------------------------------------------*/
static void test_health_field_round_trip_typical(void)
{
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.wheel                = 3U;  /* RR */
    in.field_id             = WHEQ_FIELD_HEALTH;
    in.asymmetry_pct_x10    = 300U;  /* 30.0% */
    in.delta_temp_c_x10     = 45U;   /* 4.5 C */
    in.halfbridge_verdict   = 2U;    /* WHEQ_HALFBRIDGE_FAIL wire value    */
    in.temp_present         = true;

    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);

    WheelEqualityFrame_t out;
    memset(&out, 0xAA, sizeof(out));
    CHECK(WheelEqualityFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.wheel                == in.wheel);
    CHECK(out.field_id             == in.field_id);
    CHECK(out.asymmetry_pct_x10    == in.asymmetry_pct_x10);
    CHECK(out.delta_temp_c_x10     == in.delta_temp_c_x10);
    CHECK(out.halfbridge_verdict   == in.halfbridge_verdict);
    CHECK(out.temp_present         == in.temp_present);
}

/* ---- VERDICT field --------------------------------------------------------*/
static void test_verdict_field_round_trip_typical(void)
{
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.wheel                = 1U;  /* FR */
    in.field_id             = WHEQ_FIELD_VERDICT;
    in.wheel_verdict        = 3U;  /* WHEQ_WHEEL_VERDICT_FAIL wire value  */
    in.driver_verdict       = 2U;  /* WHEQ_DRIVER_SOSPECHOSO wire value   */
    in.phase2_ran           = true;
    in.driver_reason_mask   = 0x05U;  /* HALFBRIDGE | ELECTRICAL          */

    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);

    WheelEqualityFrame_t out;
    memset(&out, 0xAA, sizeof(out));
    CHECK(WheelEqualityFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.wheel                == in.wheel);
    CHECK(out.field_id             == in.field_id);
    CHECK(out.wheel_verdict        == in.wheel_verdict);
    CHECK(out.driver_verdict       == in.driver_verdict);
    CHECK(out.phase2_ran           == in.phase2_ran);
    CHECK(out.driver_reason_mask   == in.driver_reason_mask);
}

/* ---- Limit values (u16/u8 max) round-trip without wrapping, and the
 * saturating helpers themselves clamp correctly (mirrors
 * test_service_diag_frame.c's test_result_boundary_values pattern: the
 * struct fields are already narrow, so an out-of-range *sensor* reading
 * must be clamped by the caller via WheelEqFrame_SatU16/SatU8 BEFORE it is
 * stored here — exercise those helpers directly). ---------------------------*/
static void test_limit_values_round_trip(void)
{
    CHECK(WheelEqFrame_SatU16(70000) == 65535U);
    CHECK(WheelEqFrame_SatU16(-5) == 0U);
    CHECK(WheelEqFrame_SatU8(300) == 255U);
    CHECK(WheelEqFrame_SatU8(-5) == 0U);

    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.wheel                  = 0U;
    in.field_id               = WHEQ_FIELD_SPEED;
    in.pulses_per_sec_25      = WheelEqFrame_SatU16(999999);  /* clamped by caller */
    in.pulses_per_sec_50      = 65535U;  /* exact u16 max                  */
    in.normalized_speed_x1000 = 0U;      /* exact minimum                  */
    in.deviation_pct          = WheelEqFrame_SatU8(300);      /* clamped by caller */

    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);

    WheelEqualityFrame_t out;
    CHECK(WheelEqualityFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.pulses_per_sec_25 == 65535U);
    CHECK(out.pulses_per_sec_50 == 65535U);
    CHECK(out.normalized_speed_x1000 == 0U);
    CHECK(out.deviation_pct     == 255U);
}

/* Zero (the other boundary) packs/unpacks cleanly too. */
static void test_zero_values_round_trip(void)
{
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.field_id = WHEQ_FIELD_CURRENT;
    in.current_ma_25 = 0U;
    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);
    WheelEqualityFrame_t out;
    CHECK(WheelEqualityFrame_Unpack(bytes, sizeof(bytes), &out));
    CHECK(out.current_ma_25 == 0U);
}

/* ---- Wheel index and field_id round-trip for every valid combination ----*/
static void test_all_wheel_field_combinations(void)
{
    for (uint8_t w = 0; w < 4U; w++) {
        for (uint8_t f = 0; f < 4U; f++) {
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

/* ---- Invalid DLC must be rejected outright -------------------------------*/
static void test_invalid_dlc_rejected(void)
{
    uint8_t bytes7[7] = {0};
    uint8_t bytes9[9] = {0};
    WheelEqualityFrame_t out;
    memset(&out, 0, sizeof(out));

    CHECK(!WheelEqualityFrame_Unpack(bytes7, sizeof(bytes7), &out));
    CHECK(!WheelEqualityFrame_Unpack(bytes9, sizeof(bytes9), &out));
    CHECK(!WheelEqualityFrame_Unpack(NULL, 0U, &out));
}

/* ---- Invalid field_id / wheel index rejected on encode ------------------*/
static void test_invalid_field_id_rejected(void)
{
    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.wheel = 5U;      /* out of range (only 0..3 valid) */
    in.field_id = WHEQ_FIELD_SPEED;
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == 0U);

    memset(&in, 0, sizeof(in));
    in.wheel = 0U;
    in.field_id = 7U;   /* out of range (only 0..3 valid) */
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == 0U);
}

/* ---- NULL safety ----------------------------------------------------------*/
static void test_null_safety(void)
{
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(WheelEqualityFrame_Pack(NULL, bytes) == 0U);
    CHECK(WheelEqualityFrame_Pack(&in, NULL) == 0U);

    WheelEqualityFrame_t out;
    CHECK(!WheelEqualityFrame_Unpack(bytes, sizeof(bytes), NULL));
    CHECK(!WheelEqualityFrame_Unpack(NULL, sizeof(bytes), &out));
}

/* ---- Reserved bytes are always zero on encode ----------------------------*/
static void test_reserved_bytes_are_zero(void)
{
    WheelEqualityFrame_t in;
    memset(&in, 0, sizeof(in));
    in.field_id = WHEQ_FIELD_HEALTH;
    in.wheel = 0U;
    uint8_t bytes[WHEEL_EQUALITY_FRAME_DLC];
    memset(bytes, 0xAA, sizeof(bytes));
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);
    CHECK(bytes[7] == 0U);  /* HEALTH field's last byte is reserved        */

    memset(&in, 0, sizeof(in));
    in.field_id = WHEQ_FIELD_VERDICT;
    memset(bytes, 0xAA, sizeof(bytes));
    CHECK(WheelEqualityFrame_Pack(&in, bytes) == WHEEL_EQUALITY_FRAME_DLC);
    CHECK(bytes[5] == 0U && bytes[6] == 0U && bytes[7] == 0U);
}

int main(void)
{
    test_speed_field_round_trip_typical();
    test_current_field_round_trip_typical();
    test_health_field_round_trip_typical();
    test_verdict_field_round_trip_typical();
    test_limit_values_round_trip();
    test_zero_values_round_trip();
    test_all_wheel_field_combinations();
    test_invalid_dlc_rejected();
    test_invalid_field_id_rejected();
    test_null_safety();
    test_reserved_bytes_are_zero();

    if (failed) {
        printf("wheel_equality_frame: %d FAILED\n", failed);
        return 1;
    }
    printf("wheel_equality_frame: all tests passed\n");
    return 0;
}
