/**
  ****************************************************************************
  * @file    encoder_reader.h
  * @brief   Read-only interface for E6B2-CWZ6C quadrature encoder (TIM2)
  *
  *          Hardware-integrated, not used for control.
  *          Exposes raw counter access for diagnostics and validation only.
  *          The encoder is already configured in quadrature mode by
  *          MX_TIM2_Init() and started by Motor_Init().
  ****************************************************************************
  */

#ifndef __ENCODER_READER_H
#define __ENCODER_READER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---- Read-only encoder access ---- */

/**
 * @brief  Return the current raw TIM2 counter value.
 *         No filtering, no scaling, no conversion.
 */
int32_t Encoder_GetRawCount(void);

/**
 * @brief  Return the change in raw count since the last call.
 *
 *         Must be called from a single context only (main loop).
 *         Not reentrant — shares internal state with
 *         Encoder_SendDiagnostic().
 */
int32_t Encoder_GetDelta(void);

/**
 * @brief  Reset the internal delta tracking state.
 *         Does NOT modify the hardware counter.
 */
void Encoder_Reset(void);

/**
 * @brief  Send raw encoder count and delta over CAN diagnostic channel.
 *         Uses CAN_ID_DIAG_ERROR (0x300) with subsystem tag.
 *         For validation only — not part of the control path.
 */
void Encoder_SendDiagnostic(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_READER_H */
