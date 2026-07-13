/**
  ****************************************************************************
  * @file    relay_health_frame.h
  * @brief   Single source of truth for the 0x317 relay/current-sense health
  *          CAN frame (Problem 3).
  *
  * Serialises the classified RelayHealthDiag snapshot maintained by
  * safety_system.c so the ESP32 can show the REAL cause of a suspected relay
  * fault — and the numbers behind it — instead of a bare "RELAY OPEN".
  *
  * Header-only pure functions (no HAL, no I/O): shared by the STM32 sender
  * (can_handler.c), the ESP32 receiver, and the host round-trip test so the
  * wire layout can never drift between them.
  *
  * Wire layout (CAN id 0x317, DLC 8, STM32 -> ESP32, 1 Hz):
  *   b0    : diagnostic_reason (RelayDiagReason_t, 0..10)
  *   b1    : flags
  *             bit0 relay_commanded
  *             bit1 relay_sequence_complete
  *             bit2 power_ready
  *             bit3 any_wheel_moving
  *             bit4 current reading valid (all expected wheel INA valid)
  *             bit5 current sample stale (age >= RELAY_DIAG_CURRENT_STALE_MS)
  *             bit6 expected wheel INA missing
  *             bit7 polarity reversed
  *   b2-b3 : current_sum_abs in centi-amps (u16 LE, saturating)
  *   b4    : throttle_pct (0..100, u8 saturating)
  *   b5    : final_pwm_pct (0..100, u8 saturating)
  *   b6-b7 : current sample age ms (u16 LE, saturating)
  ****************************************************************************
  */

#ifndef RELAY_HEALTH_FRAME_H
#define RELAY_HEALTH_FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "relay_health_diag.h"

/* Flag bit positions in wire byte 1. */
#define RELAY_FRAME_FLAG_RELAY_CMD    (1U << 0)
#define RELAY_FRAME_FLAG_SEQ_COMPLETE (1U << 1)
#define RELAY_FRAME_FLAG_POWER_READY  (1U << 2)
#define RELAY_FRAME_FLAG_WHEEL_MOVING (1U << 3)
#define RELAY_FRAME_FLAG_CURRENT_VALID (1U << 4)
#define RELAY_FRAME_FLAG_CURRENT_STALE (1U << 5)
#define RELAY_FRAME_FLAG_INA_MISSING  (1U << 6)
#define RELAY_FRAME_FLAG_POLARITY_REV (1U << 7)

/* Decoded view of the 0x317 payload. */
typedef struct {
    uint8_t  reason;             /* RelayDiagReason_t */
    bool     relay_commanded;
    bool     relay_sequence_complete;
    bool     power_ready;
    bool     any_wheel_moving;
    bool     current_valid;
    bool     current_stale;
    bool     ina_missing;
    bool     polarity_reversed;
    uint16_t current_sum_ca;     /* centi-amps */
    uint8_t  throttle_pct;
    uint8_t  final_pwm_pct;
    uint16_t sample_age_ms;
} RelayHealthFrame;

/* Saturating helpers (host + firmware, no libm). */
static inline uint16_t RelayFrame_SatU16(float v) {
    if (v <= 0.0f) return 0U;
    if (v >= 65535.0f) return 65535U;
    return (uint16_t)(v + 0.5f);
}

static inline uint8_t RelayFrame_SatPct(float v) {
    if (v <= 0.0f) return 0U;
    if (v >= 100.0f) return 100U;
    return (uint8_t)(v + 0.5f);
}

/**
 * @brief  Pack a classified RelayHealthDiag snapshot into 8 CAN bytes.
 * @param  d     Source snapshot (must be non-NULL).
 * @param  out   8-byte destination buffer (must be non-NULL).
 */
static inline void RelayHealth_PackFrame(const RelayHealthDiag *d, uint8_t out[8]) {
    for (int i = 0; i < 8; i++) out[i] = 0U;
    if (d == NULL) return;

    out[0] = (uint8_t)d->diagnostic_reason;

    uint8_t expected = (uint8_t)(d->ina_expected_mask & RELAY_DIAG_WHEEL_MASK);
    bool ina_missing = (expected != 0U) &&
                       ((uint8_t)(expected & d->ina_ok_mask) != expected);
    bool current_valid = (expected != 0U) &&
                         ((uint8_t)(expected & d->ina_ok_mask & d->ina_valid_mask) == expected);
    bool current_stale = (d->current_sample_age_ms >= RELAY_DIAG_CURRENT_STALE_MS);

    uint8_t flags = 0U;
    if (d->relay_commanded)         flags |= RELAY_FRAME_FLAG_RELAY_CMD;
    if (d->relay_sequence_complete) flags |= RELAY_FRAME_FLAG_SEQ_COMPLETE;
    if (d->power_ready)             flags |= RELAY_FRAME_FLAG_POWER_READY;
    if (d->any_wheel_moving)        flags |= RELAY_FRAME_FLAG_WHEEL_MOVING;
    if (current_valid)              flags |= RELAY_FRAME_FLAG_CURRENT_VALID;
    if (current_stale)              flags |= RELAY_FRAME_FLAG_CURRENT_STALE;
    if (ina_missing)                flags |= RELAY_FRAME_FLAG_INA_MISSING;
    if (d->polarity_reversed)       flags |= RELAY_FRAME_FLAG_POLARITY_REV;
    out[1] = flags;

    uint16_t ca = RelayFrame_SatU16(d->current_sum_abs * 100.0f);
    out[2] = (uint8_t)(ca & 0xFFU);
    out[3] = (uint8_t)((ca >> 8) & 0xFFU);

    out[4] = RelayFrame_SatPct(d->throttle_pct);
    out[5] = RelayFrame_SatPct(d->final_pwm_pct);

    uint16_t age = (d->current_sample_age_ms > 65535U)
                 ? 65535U : (uint16_t)d->current_sample_age_ms;
    out[6] = (uint8_t)(age & 0xFFU);
    out[7] = (uint8_t)((age >> 8) & 0xFFU);
}

/**
 * @brief  Unpack 8 CAN bytes into a decoded RelayHealthFrame.
 */
static inline void RelayHealth_UnpackFrame(const uint8_t in[8], RelayHealthFrame *f) {
    if (f == NULL || in == NULL) return;

    f->reason                  = in[0];
    uint8_t flags              = in[1];
    f->relay_commanded         = (flags & RELAY_FRAME_FLAG_RELAY_CMD) != 0U;
    f->relay_sequence_complete = (flags & RELAY_FRAME_FLAG_SEQ_COMPLETE) != 0U;
    f->power_ready             = (flags & RELAY_FRAME_FLAG_POWER_READY) != 0U;
    f->any_wheel_moving        = (flags & RELAY_FRAME_FLAG_WHEEL_MOVING) != 0U;
    f->current_valid           = (flags & RELAY_FRAME_FLAG_CURRENT_VALID) != 0U;
    f->current_stale           = (flags & RELAY_FRAME_FLAG_CURRENT_STALE) != 0U;
    f->ina_missing             = (flags & RELAY_FRAME_FLAG_INA_MISSING) != 0U;
    f->polarity_reversed       = (flags & RELAY_FRAME_FLAG_POLARITY_REV) != 0U;
    f->current_sum_ca          = (uint16_t)(in[2] | ((uint16_t)in[3] << 8));
    f->throttle_pct            = in[4];
    f->final_pwm_pct           = in[5];
    f->sample_age_ms           = (uint16_t)(in[6] | ((uint16_t)in[7] << 8));
}

#ifdef __cplusplus
}
#endif

#endif /* RELAY_HEALTH_FRAME_H */
