/**
  ****************************************************************************
  * @file    service_diag_frame.h
  * @brief   Pure pack/unpack contract for the SERVICE_DIAG self-test CAN
  *          frames (Bloque A, PR #445 Hito 1): 0x31B session status and
  *          0x31C per-step test result.
  *
  * Header-only pure functions (no HAL, no I/O, no flash): shared by the
  * STM32 sender (can_handler.c), the ESP32 receiver and the host round-trip
  * test so the wire layout can never drift between them — same pattern as
  * traction_limit_frame.h / relay_health_frame.h / status_safety_frame.h.
  *
  * Deliberately does NOT include service_diag_session.h: that header uses
  * C11 `_Static_assert`, which does not compile under the ESP32's C++
  * toolchain, and traction_limit_frame.h precedent (included directly by
  * esp32/src/test_traction_limit_diag_decode.cpp) shows frame headers ARE
  * consumed as raw C++ translation units.  Enum-valued bytes are therefore
  * carried as documented `uint8_t` (wire value), exactly like
  * relay_health_frame.h's `reason` field — callers on both sides cast to/from
  * the real named enum (ServiceDiagState_t / ServiceDiagChannel_t /
  * ServiceDiagReason_t / ServiceDiagStepVerdict_t on the STM32 side) so a
  * bare magic-number literal is never compared against the wire byte.
  *
  * Wire layout (CAN id 0x31B DIAG_SERVICE_SESSION, DLC 8,
  *              STM32 -> ESP32, 10 Hz while a session is active,
  *              SILENT otherwise — no frame sent outside a session):
  *   b0 : state             (ServiceDiagState_t wire value, 0..5)
  *   b1 : step_index        (u8, count of RequestStep() calls so far)
  *   b2 : active_channel    (ServiceDiagChannel_t wire value, 0..5;
  *                           SVCDIAG_CH_NONE unless actually STEPPING)
  *   b3 : progress_pct      (0..100, 0 unless actually STEPPING)
  *   b4 : reason            (ServiceDiagReason_t wire value, 0..14 — single
  *                           highest-priority abort/reject cause)
  *   b5 : origin_state_raw  (opaque SystemState_t snapshot at Begin() time;
  *                           never interpreted by this header, forwarded
  *                           as-is for the HMI to render with its own
  *                           SystemState_t table)
  *   b6 : elapsed_sec       (session age in whole seconds, saturating u8)
  *   b7 : active_pwm_pct    (0..100, the PWM actually applied THIS cycle;
  *                           0 unless actually STEPPING)
  *
  * Wire layout (CAN id 0x31C DIAG_TEST_RESULT, DLC 8,
  *              STM32 -> ESP32, once per step close — i.e. once per
  *              STEPPING -> DEADTIME/ABORTED transition):
  *   b0    : channel          (ServiceDiagChannel_t wire value, 0..5)
  *   b1    : pwm_step_pct     (0..100, the clamped PWM commanded for the step)
  *   b2-b3 : current_ma       (u16 LE, peak/measured current in mA,
  *                             saturating)
  *   b4-b5 : pulses_per_sec   (u16 LE, wheel pulse rate during the step,
  *                             saturating; 0 for the steering channel)
  *   b6    : verdict          (ServiceDiagStepVerdict_t wire value, 0..3)
  *   b7    : step_index       (u8, correlates with the 0x31B step_index that
  *                             was active when this result was produced)
  ****************************************************************************
  */

#ifndef SERVICE_DIAG_FRAME_H
#define SERVICE_DIAG_FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Transmitted/required DLC — the ONE authoritative value for each frame. */
#define SERVICE_DIAG_SESSION_FRAME_DLC     8U
#define SERVICE_DIAG_TEST_RESULT_FRAME_DLC 8U

/** Decoded view of the 0x31B payload. */
typedef struct {
    uint8_t state;             /* ServiceDiagState_t wire value */
    uint8_t step_index;
    uint8_t active_channel;    /* ServiceDiagChannel_t wire value */
    uint8_t progress_pct;
    uint8_t reason;            /* ServiceDiagReason_t wire value */
    uint8_t origin_state_raw;
    uint8_t elapsed_sec;
    uint8_t active_pwm_pct;
} ServiceDiagSessionFrame_t;

/** Decoded view of the 0x31C payload. */
typedef struct {
    uint8_t  channel;          /* ServiceDiagChannel_t wire value */
    uint8_t  pwm_step_pct;
    uint16_t current_ma;
    uint16_t pulses_per_sec;
    uint8_t  verdict;          /* ServiceDiagStepVerdict_t wire value */
    uint8_t  step_index;
} ServiceDiagTestResultFrame_t;

/* ---- Saturating helpers (host + firmware, no libm) ---------------------- */
static inline uint8_t ServiceDiagFrame_SatPct(int32_t v)
{
    if (v <= 0) return 0U;
    if (v >= 100) return 100U;
    return (uint8_t)v;
}

static inline uint16_t ServiceDiagFrame_SatU16(int32_t v)
{
    if (v <= 0) return 0U;
    if (v >= 65535) return 65535U;
    return (uint16_t)v;
}

/**
 * @brief  Pack a ServiceDiagSessionFrame_t into 8 CAN bytes (0x31B).
 * @param  in   Source view (must be non-NULL).
 * @param  out  Buffer of at least SERVICE_DIAG_SESSION_FRAME_DLC bytes
 *              (must be non-NULL).
 * @return Number of bytes written (SERVICE_DIAG_SESSION_FRAME_DLC), or 0 if
 *         @p in or @p out is NULL.
 */
static inline uint8_t ServiceDiagSessionFrame_Pack(
    const ServiceDiagSessionFrame_t *in,
    uint8_t out[SERVICE_DIAG_SESSION_FRAME_DLC])
{
    if (in == NULL || out == NULL) return 0U;
    out[0] = in->state;
    out[1] = in->step_index;
    out[2] = in->active_channel;
    out[3] = ServiceDiagFrame_SatPct(in->progress_pct);
    out[4] = in->reason;
    out[5] = in->origin_state_raw;
    out[6] = in->elapsed_sec;
    out[7] = ServiceDiagFrame_SatPct(in->active_pwm_pct);
    return (uint8_t)SERVICE_DIAG_SESSION_FRAME_DLC;
}

/**
 * @brief  Decode a received 0x31B payload.
 * @param  data  Received payload (may be NULL only if len == 0).
 * @param  len   Received DLC.
 * @param  out   Decoded output (untouched when the frame is rejected).
 * @return true when the frame was accepted (len == SERVICE_DIAG_SESSION_FRAME_DLC);
 *         false on a NULL argument or a WRONG DLC — the frame is rejected
 *         outright, there is no forward-compatible short form for this
 *         frame (unlike 0x203's DLC_MIN, every one of these 8 bytes is
 *         load-bearing for the HMI's Bloque A submenu).
 */
static inline bool ServiceDiagSessionFrame_Unpack(
    const uint8_t *data, size_t len, ServiceDiagSessionFrame_t *out)
{
    if (data == NULL || out == NULL || len != SERVICE_DIAG_SESSION_FRAME_DLC) {
        return false;
    }
    out->state             = data[0];
    out->step_index        = data[1];
    out->active_channel    = data[2];
    out->progress_pct      = data[3];
    out->reason            = data[4];
    out->origin_state_raw  = data[5];
    out->elapsed_sec       = data[6];
    out->active_pwm_pct    = data[7];
    return true;
}

/**
 * @brief  Pack a ServiceDiagTestResultFrame_t into 8 CAN bytes (0x31C).
 * @param  in   Source view (must be non-NULL).
 * @param  out  Buffer of at least SERVICE_DIAG_TEST_RESULT_FRAME_DLC bytes
 *              (must be non-NULL).
 * @return Number of bytes written (SERVICE_DIAG_TEST_RESULT_FRAME_DLC), or 0
 *         if @p in or @p out is NULL.
 */
static inline uint8_t ServiceDiagTestResultFrame_Pack(
    const ServiceDiagTestResultFrame_t *in,
    uint8_t out[SERVICE_DIAG_TEST_RESULT_FRAME_DLC])
{
    if (in == NULL || out == NULL) return 0U;
    out[0] = in->channel;
    out[1] = ServiceDiagFrame_SatPct(in->pwm_step_pct);
    out[2] = (uint8_t)(in->current_ma & 0xFFU);
    out[3] = (uint8_t)((in->current_ma >> 8) & 0xFFU);
    out[4] = (uint8_t)(in->pulses_per_sec & 0xFFU);
    out[5] = (uint8_t)((in->pulses_per_sec >> 8) & 0xFFU);
    out[6] = in->verdict;
    out[7] = in->step_index;
    return (uint8_t)SERVICE_DIAG_TEST_RESULT_FRAME_DLC;
}

/**
 * @brief  Decode a received 0x31C payload.
 * @param  data  Received payload (may be NULL only if len == 0).
 * @param  len   Received DLC.
 * @param  out   Decoded output (untouched when the frame is rejected).
 * @return true when the frame was accepted (len == SERVICE_DIAG_TEST_RESULT_FRAME_DLC);
 *         false on a NULL argument or a WRONG DLC.
 */
static inline bool ServiceDiagTestResultFrame_Unpack(
    const uint8_t *data, size_t len, ServiceDiagTestResultFrame_t *out)
{
    if (data == NULL || out == NULL || len != SERVICE_DIAG_TEST_RESULT_FRAME_DLC) {
        return false;
    }
    out->channel        = data[0];
    out->pwm_step_pct   = data[1];
    out->current_ma     = (uint16_t)(data[2] | ((uint16_t)data[3] << 8));
    out->pulses_per_sec = (uint16_t)(data[4] | ((uint16_t)data[5] << 8));
    out->verdict        = data[6];
    out->step_index     = data[7];
    return true;
}

#ifdef __cplusplus
}
#endif

#endif /* SERVICE_DIAG_FRAME_H */
