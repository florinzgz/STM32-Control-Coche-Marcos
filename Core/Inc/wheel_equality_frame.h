/**
  ****************************************************************************
  * @file    wheel_equality_frame.h
  * @brief   Pure pack/unpack contract for the 0x31D DIAG_WHEEL_EQUALITY CAN
  *          frame (Hito 2, PR #445 — wheel-equality / BTS7960 health test).
  *
  * Header-only pure functions (no HAL, no I/O, no flash): shared by the
  * STM32 sender (can_handler.c), the ESP32 receiver and the host round-trip
  * test so the wire layout can never drift between them — same pattern as
  * service_diag_frame.h (Hito 1).
  *
  * Deliberately does NOT include wheel_equality_test.h: that header pulls in
  * service_diag_session.h, which uses C11 `_Static_assert` and is not meant
  * to be compiled as a C++ translation unit.  Enum-valued bytes are
  * therefore carried as documented `uint8_t` (wire value), exactly like
  * service_diag_frame.h's `reason`/`state` fields — callers on both sides
  * cast to/from the real named enum (WheelEqWheelVerdict_t /
  * WheelEqDriverVerdict_t / WheelEqHalfBridgeVerdict_t / WheelEqCause_t on
  * the STM32 side) so a bare magic-number literal is never compared against
  * the wire byte.
  *
  * ONE CAN ID, FOUR "FIELD" SUB-FRAMES PER WHEEL
  * -----------------------------------------------------------------------
  * A single result for one wheel does not fit an 8-byte DLC (11 distinct
  * values are required per wheel by the HMI table: pulses/s @25%, pulses/s
  * @50%, normalized speed, deviation %, current, I/PWM slope, F/R
  * half-bridge asymmetry, thermal delta, wheel verdict, driver verdict,
  * probable cause).  Exactly like the existing 0x310 DRIVE_TUNING /
  * 0x30D field-stream frames, one wheel's full result is transmitted as a
  * short burst of frames sharing the same CAN ID, discriminated by a
  * `field_id` sub-code in byte0 alongside the wheel index — 4 field kinds x
  * 4 wheels = 16 frames per results burst, emitted once when Fase 1
  * completes and again (overwriting) if Fase 2 also runs.
  *
  * Common byte0 (ALL field kinds):
  *   bits0-1 : wheel            (0=FL,1=FR,2=RL,3=RR)
  *   bits2-3 : field_id         (0=SPEED,1=CURRENT,2=HEALTH,3=VERDICT)
  *   bit4    : phase2_included  (1 if this result reflects the Fase 2
  *                               simultaneous run, 0 if Fase 1 only)
  *   bits5-7 : reserved (0)
  *
  * FIELD_ID 0 — SPEED:
  *   b1-b2 : pulses_per_sec_25   (u16 LE, saturating; forward direction)
  *   b3-b4 : pulses_per_sec_50   (u16 LE, saturating; forward direction)
  *   b5-b6 : normalized_speed_x1000 (u16 LE, saturating; pulses/s per
  *                                   (pwm_pct x battery_V), x1000 for 3
  *                                   decimals of resolution; averaged over
  *                                   all 4 Fase 1 samples for the wheel)
  *   b7    : deviation_pct       (u8, 0..100 saturating; |value-median| /
  *                                median x 100, magnitude only)
  *
  * FIELD_ID 1 — CURRENT / SLOPE:
  *   b1-b2 : current_ma_25       (u16 LE, saturating; forward direction)
  *   b3-b4 : current_ma_50       (u16 LE, saturating; forward direction;
  *                                this is the value shown in the HMI's
  *                                single "corriente" column)
  *   b5-b6 : slope_ma_per_pct_x10 (u16 LE, saturating; (I50-I25)/(50-25)
  *                                 in mA per PWM-percent, x10 for 1 decimal)
  *   b7    : probable_cause      (WheelEqCause_t wire value, 0..4)
  *
  * FIELD_ID 2 — HEALTH (half-bridge asymmetry + thermal drift):
  *   b1-b2 : asymmetry_pct_x10   (u16 LE, saturating; forward-vs-reverse
  *                                normalized-speed/current asymmetry at the
  *                                50 % step, x10 for 1 decimal)
  *   b3-b4 : delta_temp_c_x10    (u16 LE, saturating; DS18B20 temperature
  *                                increase across the sequence, x10 for 1
  *                                decimal; 0 when temp_present is false)
  *   b5    : halfbridge_verdict  (WheelEqHalfBridgeVerdict_t wire value)
  *   b6    : temp_present        (0/1 — a DS18B20 is mapped to this wheel)
  *   b7    : reserved (0)
  *
  * FIELD_ID 3 — VERDICT:
  *   b1    : wheel_verdict       (WheelEqWheelVerdict_t wire value)
  *   b2    : driver_verdict      (WheelEqDriverVerdict_t wire value)
  *   b3    : phase2_ran          (0/1 — Fase 2 actually executed)
  *   b4    : driver_reason_mask  (WHEQ_DRIVER_REASON_* bitmask — the
  *                                concrete criterion(a) behind the driver
  *                                verdict: bit0 half-bridge, bit1 slope,
  *                                bit2 electrical-cause, bit3 thermal)
  *   b5-b7 : reserved (0)
  ****************************************************************************
  */

#ifndef WHEEL_EQUALITY_FRAME_H
#define WHEEL_EQUALITY_FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Transmitted/required DLC — the ONE authoritative value for this frame. */
#define WHEEL_EQUALITY_FRAME_DLC   8U

/** field_id wire values (byte0 bits2-3). */
#define WHEQ_FIELD_SPEED     0U
#define WHEQ_FIELD_CURRENT   1U
#define WHEQ_FIELD_HEALTH    2U
#define WHEQ_FIELD_VERDICT   3U

/** Decoded view of one 0x31D sub-frame (one wheel, one field_id). */
typedef struct {
    uint8_t  wheel;             /* 0=FL,1=FR,2=RL,3=RR                    */
    uint8_t  field_id;          /* WHEQ_FIELD_*                           */
    bool     phase2_included;

    /* FIELD_SPEED */
    uint16_t pulses_per_sec_25;
    uint16_t pulses_per_sec_50;
    uint16_t normalized_speed_x1000;
    uint8_t  deviation_pct;

    /* FIELD_CURRENT */
    uint16_t current_ma_25;
    uint16_t current_ma_50;
    uint16_t slope_ma_per_pct_x10;
    uint8_t  probable_cause;     /* WheelEqCause_t wire value              */

    /* FIELD_HEALTH */
    uint16_t asymmetry_pct_x10;
    uint16_t delta_temp_c_x10;
    uint8_t  halfbridge_verdict; /* WheelEqHalfBridgeVerdict_t wire value  */
    bool     temp_present;

    /* FIELD_VERDICT */
    uint8_t  wheel_verdict;      /* WheelEqWheelVerdict_t wire value       */
    uint8_t  driver_verdict;     /* WheelEqDriverVerdict_t wire value      */
    bool     phase2_ran;
    uint8_t  driver_reason_mask; /* WHEQ_DRIVER_REASON_* bitmask           */
} WheelEqualityFrame_t;

/* ---- Saturating helpers (host + firmware, no libm) ---------------------- */
static inline uint8_t WheelEqFrame_SatU8(int32_t v)
{
    if (v <= 0) return 0U;
    if (v >= 255) return 255U;
    return (uint8_t)v;
}

static inline uint16_t WheelEqFrame_SatU16(int32_t v)
{
    if (v <= 0) return 0U;
    if (v >= 65535) return 65535U;
    return (uint16_t)v;
}

/**
 * @brief  Pack a WheelEqualityFrame_t into 8 CAN bytes (0x31D).
 *
 * Only the fields relevant to @p in->field_id are consulted; the others are
 * ignored on encode (they still round-trip as whatever the caller left in
 * the struct on the DECODE side only for the matching field_id — see
 * WheelEqualityFrame_Unpack()).
 *
 * @param  in   Source view (must be non-NULL).
 * @param  out  Buffer of at least WHEEL_EQUALITY_FRAME_DLC bytes (non-NULL).
 * @return Number of bytes written (WHEEL_EQUALITY_FRAME_DLC), or 0 if
 *         @p in or @p out is NULL, or wheel/field_id is out of range.
 */
static inline uint8_t WheelEqualityFrame_Pack(
    const WheelEqualityFrame_t *in,
    uint8_t out[WHEEL_EQUALITY_FRAME_DLC])
{
    if (in == NULL || out == NULL) return 0U;
    if (in->wheel > 3U || in->field_id > 3U) return 0U;

    out[0] = (uint8_t)((in->wheel & 0x03U) |
                        ((in->field_id & 0x03U) << 2) |
                        (in->phase2_included ? (1U << 4) : 0U));

    switch (in->field_id) {
    case WHEQ_FIELD_SPEED: {
        uint16_t p25 = WheelEqFrame_SatU16(in->pulses_per_sec_25);
        uint16_t p50 = WheelEqFrame_SatU16(in->pulses_per_sec_50);
        uint16_t nsp = WheelEqFrame_SatU16(in->normalized_speed_x1000);
        out[1] = (uint8_t)(p25 & 0xFFU);
        out[2] = (uint8_t)((p25 >> 8) & 0xFFU);
        out[3] = (uint8_t)(p50 & 0xFFU);
        out[4] = (uint8_t)((p50 >> 8) & 0xFFU);
        out[5] = (uint8_t)(nsp & 0xFFU);
        out[6] = (uint8_t)((nsp >> 8) & 0xFFU);
        out[7] = WheelEqFrame_SatU8(in->deviation_pct);
        break;
    }
    case WHEQ_FIELD_CURRENT: {
        uint16_t i25   = WheelEqFrame_SatU16(in->current_ma_25);
        uint16_t i50   = WheelEqFrame_SatU16(in->current_ma_50);
        uint16_t slope = WheelEqFrame_SatU16(in->slope_ma_per_pct_x10);
        out[1] = (uint8_t)(i25 & 0xFFU);
        out[2] = (uint8_t)((i25 >> 8) & 0xFFU);
        out[3] = (uint8_t)(i50 & 0xFFU);
        out[4] = (uint8_t)((i50 >> 8) & 0xFFU);
        out[5] = (uint8_t)(slope & 0xFFU);
        out[6] = (uint8_t)((slope >> 8) & 0xFFU);
        out[7] = in->probable_cause;
        break;
    }
    case WHEQ_FIELD_HEALTH: {
        uint16_t asym = WheelEqFrame_SatU16(in->asymmetry_pct_x10);
        uint16_t dt   = WheelEqFrame_SatU16(in->delta_temp_c_x10);
        out[1] = (uint8_t)(asym & 0xFFU);
        out[2] = (uint8_t)((asym >> 8) & 0xFFU);
        out[3] = (uint8_t)(dt & 0xFFU);
        out[4] = (uint8_t)((dt >> 8) & 0xFFU);
        out[5] = in->halfbridge_verdict;
        out[6] = in->temp_present ? 1U : 0U;
        out[7] = 0U;
        break;
    }
    case WHEQ_FIELD_VERDICT: {
        out[1] = in->wheel_verdict;
        out[2] = in->driver_verdict;
        out[3] = in->phase2_ran ? 1U : 0U;
        out[4] = in->driver_reason_mask;
        out[5] = 0U; out[6] = 0U; out[7] = 0U;
        break;
    }
    default:
        return 0U;
    }
    return (uint8_t)WHEEL_EQUALITY_FRAME_DLC;
}

/**
 * @brief  Decode a received 0x31D payload.
 *
 * Only the fields belonging to the decoded field_id are populated; every
 * other field in @p out is zero-initialised first so the caller never sees
 * stale data from a previous decode.
 *
 * @param  data  Received payload (may be NULL only if len == 0).
 * @param  len   Received DLC.
 * @param  out   Decoded output (untouched when the frame is rejected).
 * @return true when the frame was accepted (len == WHEEL_EQUALITY_FRAME_DLC
 *         AND the embedded field_id is 0..3); false otherwise — the frame
 *         is rejected outright, there is no forward-compatible short form.
 */
static inline bool WheelEqualityFrame_Unpack(
    const uint8_t *data, size_t len, WheelEqualityFrame_t *out)
{
    if (data == NULL || out == NULL || len != WHEEL_EQUALITY_FRAME_DLC) {
        return false;
    }
    uint8_t field_id = (uint8_t)((data[0] >> 2) & 0x03U);
    if (field_id > 3U) return false;

    out->wheel             = (uint8_t)(data[0] & 0x03U);
    out->field_id          = field_id;
    out->phase2_included   = (data[0] & (1U << 4)) != 0U;

    out->pulses_per_sec_25 = 0U; out->pulses_per_sec_50 = 0U;
    out->normalized_speed_x1000 = 0U; out->deviation_pct = 0U;
    out->current_ma_25 = 0U; out->current_ma_50 = 0U;
    out->slope_ma_per_pct_x10 = 0U; out->probable_cause = 0U;
    out->asymmetry_pct_x10 = 0U; out->delta_temp_c_x10 = 0U;
    out->halfbridge_verdict = 0U; out->temp_present = false;
    out->wheel_verdict = 0U; out->driver_verdict = 0U; out->phase2_ran = false;
    out->driver_reason_mask = 0U;

    switch (field_id) {
    case WHEQ_FIELD_SPEED:
        out->pulses_per_sec_25 = (uint16_t)(data[1] | ((uint16_t)data[2] << 8));
        out->pulses_per_sec_50 = (uint16_t)(data[3] | ((uint16_t)data[4] << 8));
        out->normalized_speed_x1000 = (uint16_t)(data[5] | ((uint16_t)data[6] << 8));
        out->deviation_pct     = data[7];
        break;
    case WHEQ_FIELD_CURRENT:
        out->current_ma_25       = (uint16_t)(data[1] | ((uint16_t)data[2] << 8));
        out->current_ma_50       = (uint16_t)(data[3] | ((uint16_t)data[4] << 8));
        out->slope_ma_per_pct_x10 = (uint16_t)(data[5] | ((uint16_t)data[6] << 8));
        out->probable_cause      = data[7];
        break;
    case WHEQ_FIELD_HEALTH:
        out->asymmetry_pct_x10  = (uint16_t)(data[1] | ((uint16_t)data[2] << 8));
        out->delta_temp_c_x10   = (uint16_t)(data[3] | ((uint16_t)data[4] << 8));
        out->halfbridge_verdict = data[5];
        out->temp_present       = (data[6] != 0U);
        break;
    case WHEQ_FIELD_VERDICT:
        out->wheel_verdict       = data[1];
        out->driver_verdict      = data[2];
        out->phase2_ran          = (data[3] != 0U);
        out->driver_reason_mask  = data[4];
        break;
    default:
        return false;
    }
    return true;
}

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_EQUALITY_FRAME_H */
