/**
  ****************************************************************************
  * @file    status_safety_frame.h
  * @brief   Single source of truth for the 0x203 STATUS_SAFETY CAN frame.
  *
  * Before this header the 0x203 payload existed in three places that had
  * drifted apart: `CAN_SendStatusSafety()` transmits DLC 6, `can_ids.h`
  * documents DLC 6, but `docs/CAN_CONTRACT_FINAL.md` (DLC 5),
  * `docs/CAN_PROTOCOL.md` (DLC 3) and `docs/HARDWARE_AND_SENSOR_MAP.md`
  * (3 bytes) each described a different, older revision.  A wire contract
  * with three definitions is not a contract, so the layout is defined ONCE
  * here as header-only pure functions shared by the STM32 sender
  * (can_handler.c), the ESP32 decoder and the host round-trip test.
  *
  * Wire layout (CAN id 0x203, DLC 6, STM32 -> ESP32, 10 Hz):
  *   b0 : abs_active        (0 / 1)
  *   b1 : tcs_active        (0 / 1)
  *   b2 : error_code        (Safety_Error_t)
  *   b3 : state             (SystemState_t / SYS_STATE_*)
  *   b4 : rx_errors         (saturating u8, FDCAN RX error counter)
  *   b5 : loop_peak_100us   (peak 100 Hz task duration, x100 us, sat. 255)
  *
  * Byte 5 IS part of the protocol (contract rev 1.4).  Receivers that only
  * consume bytes 0-4 remain valid: the decoder keeps a `>= 5` acceptance
  * guard and reports loop_peak_100us = 0 ("n/a") for a DLC-5 sender.
  ****************************************************************************
  */

#ifndef STATUS_SAFETY_FRAME_H
#define STATUS_SAFETY_FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/** Transmitted DLC — the ONE authoritative value for 0x203. */
#define STATUS_SAFETY_DLC          6U

/** Minimum DLC a receiver must accept (byte 5 is forward-compatible). */
#define STATUS_SAFETY_DLC_MIN      5U

/** Decoded view of the 0x203 payload. */
typedef struct {
    bool    abs_active;
    bool    tcs_active;
    uint8_t error_code;
    uint8_t state;
    uint8_t rx_errors;
    uint8_t loop_peak_100us;   /* 0 when the sender predates rev 1.4 */
} StatusSafetyFrame_t;

/**
 * @brief  Pack a STATUS_SAFETY payload into @p out.
 * @param  out  Buffer of at least STATUS_SAFETY_DLC bytes.
 * @return Number of bytes written (STATUS_SAFETY_DLC), or 0 if @p out is NULL.
 */
static inline uint8_t StatusSafetyFrame_Pack(const StatusSafetyFrame_t *in,
                                             uint8_t *out)
{
    if ((in == NULL) || (out == NULL)) {
        return 0U;
    }
    out[0] = in->abs_active ? 1U : 0U;
    out[1] = in->tcs_active ? 1U : 0U;
    out[2] = in->error_code;
    out[3] = in->state;
    out[4] = in->rx_errors;
    out[5] = in->loop_peak_100us;
    return (uint8_t)STATUS_SAFETY_DLC;
}

/**
 * @brief  Decode a received STATUS_SAFETY payload.
 * @param  data  Received payload.
 * @param  len   Received DLC.
 * @param  out   Decoded output (untouched when the frame is rejected).
 * @return true when the frame was accepted (len >= STATUS_SAFETY_DLC_MIN).
 */
static inline bool StatusSafetyFrame_Unpack(const uint8_t *data, size_t len,
                                            StatusSafetyFrame_t *out)
{
    if ((data == NULL) || (out == NULL) || (len < STATUS_SAFETY_DLC_MIN)) {
        return false;
    }
    out->abs_active     = (data[0] != 0U);
    out->tcs_active     = (data[1] != 0U);
    out->error_code     = data[2];
    out->state          = data[3];
    out->rx_errors      = data[4];
    /* Byte 5 only exists from contract rev 1.4 onwards. */
    out->loop_peak_100us = (len >= STATUS_SAFETY_DLC) ? data[5] : 0U;
    return true;
}

#ifdef __cplusplus
}
#endif

#endif /* STATUS_SAFETY_FRAME_H */
