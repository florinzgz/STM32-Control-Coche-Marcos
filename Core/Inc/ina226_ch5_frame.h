/**
  ****************************************************************************
  * @file    ina226_ch5_frame.h
  * @brief   Single source of truth for the 0x318 steering-INA (CH5) channel
  *          diagnostic CAN frame (Problem 4).
  *
  * Serialises the classified Ina226ChannelDiag snapshot maintained by
  * sensor_manager.c (Sensor_GetChannel5Diag) so the ESP32 can distinguish the
  * REAL state of the steering INA226 — MISSING / WRONG ADDRESS / CONFIG FAIL /
  * PRESENT NO SHUNT / POLARITY REVERSED / STALE / OK — instead of the opaque
  * "CH5 ST: n/d".  Crucially it carries a SIGNED shunt reading so a negative
  * (reversed-polarity) current is never flattened to zero, and the mere
  * presence of this frame lets the HMI separate "n/d" (no CAN contract) from a
  * genuine MISSING (chip did not ACK).
  *
  * Header-only pure functions (no HAL, no I/O): shared by the STM32 sender
  * (can_handler.c), the ESP32 receiver, and the host round-trip test so the
  * wire layout can never drift between them.
  *
  * Wire layout (CAN id 0x318, DLC 8, STM32 -> ESP32, 1 Hz):
  *   b0    : fault_reason (Ina226DiagReason_t, 0..9)
  *   b1    : flags / validity
  *             bit0 mux_select_ok      (TCA9548A selected the channel)
  *             bit1 i2c_ack            (INA226 acked at 0x40)
  *             bit2 identity_ok        (mfg id 0x5449 AND die id 0x2260)
  *             bit3 config_ok          (config register readback matched)
  *             bit4 shunt_read_ok
  *             bit5 bus_read_ok
  *             bit6 channel_powered    (steering branch expected energised)
  *             bit7 stale              (age >= INA226_DIAG_STALE_MS)
  *   b2-b3 : raw_shunt (int16 LE, two's complement) — the SIGNED shunt register.
  *             shunt_uv     = raw_shunt * INA226_CH5_SHUNT_LSB_UV   (2.5 µV/LSB)
  *             current_ma   = shunt_uv / INA226_CH5_SHUNT_MOHM       (1.5 mΩ)
  *           Both derived values keep the sign; never clamped to zero.
  *   b4-b5 : bus_mv (u16 LE, saturating)
  *   b6-b7 : sample_age_ms (u16 LE, saturating; 0xFFFF = never sampled)
  ****************************************************************************
  */

#ifndef INA226_CH5_FRAME_H
#define INA226_CH5_FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "ina226_channel_diag.h"

/* Derivation constants (steering INA226 uses the 50A/75mV = 1.5 mΩ shunt). */
#define INA226_CH5_SHUNT_LSB_UV   2.5f   /* µV per shunt-register LSB */
#define INA226_CH5_SHUNT_MOHM     1.5f   /* mΩ steering shunt         */

/* Flag bit positions in wire byte 1. */
#define INA226_CH5_FLAG_MUX_OK       (1U << 0)
#define INA226_CH5_FLAG_I2C_ACK      (1U << 1)
#define INA226_CH5_FLAG_IDENTITY_OK  (1U << 2)
#define INA226_CH5_FLAG_CONFIG_OK    (1U << 3)
#define INA226_CH5_FLAG_SHUNT_OK     (1U << 4)
#define INA226_CH5_FLAG_BUS_OK       (1U << 5)
#define INA226_CH5_FLAG_POWERED      (1U << 6)
#define INA226_CH5_FLAG_STALE        (1U << 7)

/* Decoded view of the 0x318 payload. */
typedef struct {
    uint8_t  reason;            /* Ina226DiagReason_t */
    bool     mux_select_ok;
    bool     i2c_ack;
    bool     identity_ok;
    bool     config_ok;
    bool     shunt_read_ok;
    bool     bus_read_ok;
    bool     channel_powered;
    bool     stale;
    int16_t  raw_shunt;         /* signed shunt register              */
    int32_t  shunt_uv;          /* signed µV (derived)                */
    int32_t  current_ma;        /* signed mA (derived, never zeroed)  */
    uint16_t bus_mv;
    uint16_t sample_age_ms;
} Ina226Ch5Frame;

/* Saturating helper (host + firmware, no libm). */
static inline uint16_t Ina226Ch5_SatU16(int32_t v) {
    if (v <= 0) return 0U;
    if (v >= 65535) return 65535U;
    return (uint16_t)v;
}

/* Round-to-nearest for signed floats (matches firmware lroundf, no libm). */
static inline int32_t Ina226Ch5_iRound(float v) {
    return (int32_t)(v < 0.0f ? (v - 0.5f) : (v + 0.5f));
}

/**
 * @brief  Pack a classified Ina226ChannelDiag snapshot into 8 CAN bytes.
 * @param  d     Source snapshot (must be non-NULL).
 * @param  out   8-byte destination buffer (must be non-NULL).
 */
static inline void Ina226Ch5_PackFrame(const Ina226ChannelDiag *d, uint8_t out[8]) {
    for (int i = 0; i < 8; i++) out[i] = 0U;
    if (d == NULL) return;

    out[0] = (uint8_t)d->fault_reason;

    uint8_t flags = 0U;
    if (d->mux_select_ok)                        flags |= INA226_CH5_FLAG_MUX_OK;
    if (d->i2c_ack)                              flags |= INA226_CH5_FLAG_I2C_ACK;
    if (d->manufacturer_id_ok && d->die_id_ok)   flags |= INA226_CH5_FLAG_IDENTITY_OK;
    if (d->config_readback_ok)                   flags |= INA226_CH5_FLAG_CONFIG_OK;
    if (d->shunt_read_ok)                        flags |= INA226_CH5_FLAG_SHUNT_OK;
    if (d->bus_read_ok)                          flags |= INA226_CH5_FLAG_BUS_OK;
    if (d->channel_powered)                      flags |= INA226_CH5_FLAG_POWERED;
    if (d->sample_age_ms >= INA226_DIAG_STALE_MS) flags |= INA226_CH5_FLAG_STALE;
    out[1] = flags;

    uint16_t raw = (uint16_t)d->raw_shunt;   /* preserve two's complement bits */
    out[2] = (uint8_t)(raw & 0xFFU);
    out[3] = (uint8_t)((raw >> 8) & 0xFFU);

    uint16_t bus = Ina226Ch5_SatU16(d->bus_mv);
    out[4] = (uint8_t)(bus & 0xFFU);
    out[5] = (uint8_t)((bus >> 8) & 0xFFU);

    uint16_t age = (d->sample_age_ms > 65535U) ? 65535U : (uint16_t)d->sample_age_ms;
    out[6] = (uint8_t)(age & 0xFFU);
    out[7] = (uint8_t)((age >> 8) & 0xFFU);
}

/**
 * @brief  Unpack 8 CAN bytes into a decoded Ina226Ch5Frame.
 *         Derives signed µV and signed mA from the raw shunt register; neither
 *         is ever clamped to zero.
 */
static inline void Ina226Ch5_UnpackFrame(const uint8_t in[8], Ina226Ch5Frame *f) {
    if (f == NULL || in == NULL) return;

    f->reason          = in[0];
    uint8_t flags      = in[1];
    f->mux_select_ok   = (flags & INA226_CH5_FLAG_MUX_OK) != 0U;
    f->i2c_ack         = (flags & INA226_CH5_FLAG_I2C_ACK) != 0U;
    f->identity_ok     = (flags & INA226_CH5_FLAG_IDENTITY_OK) != 0U;
    f->config_ok       = (flags & INA226_CH5_FLAG_CONFIG_OK) != 0U;
    f->shunt_read_ok   = (flags & INA226_CH5_FLAG_SHUNT_OK) != 0U;
    f->bus_read_ok     = (flags & INA226_CH5_FLAG_BUS_OK) != 0U;
    f->channel_powered = (flags & INA226_CH5_FLAG_POWERED) != 0U;
    f->stale           = (flags & INA226_CH5_FLAG_STALE) != 0U;

    f->raw_shunt   = (int16_t)(uint16_t)(in[2] | ((uint16_t)in[3] << 8));
    f->shunt_uv    = Ina226Ch5_iRound((float)f->raw_shunt * INA226_CH5_SHUNT_LSB_UV);
    f->current_ma  = Ina226Ch5_iRound((float)f->shunt_uv / INA226_CH5_SHUNT_MOHM);
    f->bus_mv      = (uint16_t)(in[4] | ((uint16_t)in[5] << 8));
    f->sample_age_ms = (uint16_t)(in[6] | ((uint16_t)in[7] << 8));
}

#ifdef __cplusplus
}
#endif

#endif /* INA226_CH5_FRAME_H */
