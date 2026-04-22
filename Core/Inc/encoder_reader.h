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

#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ---- Read-only encoder access (A/B quadrature channels) ---- */

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

/* ---- Encoder Z (index) channel — PB4 / EXTI4 ---- */
/*
 * The E6B2-CWZ6C Z output is NPN open-collector.  With the internal
 * pull-up (GPIO_PULLUP on PB4) the idle level is HIGH and a Z pulse
 * pulls the line LOW → FALLING edge trigger.
 *
 * Purpose: inter-revolution consistency check (drift / slip detection).
 * The position delta between consecutive Z pulses must equal ±ENCODER_CPR
 * (4800 counts).  A large deviation indicates encoder slippage or
 * A/B noise injection — useful in high-EMI environments (BTS7960 motors).
 *
 * The Z channel does NOT zero TIM2 or control steering centering.
 * Centering is handled by the inductive LJ12A3 sensor on PB5.
 *
 * ISR safety contract:
 *   – EncoderZ_IRQHandler() is O(1), no malloc, no blocking.
 *   – Debounce: pulses separated by < ENC_Z_MIN_INTERVAL_MS are silently
 *     dropped (rejects EMI spike trains; valid minimum ~30 ms at max RPM).
 *   – All volatile state is updated atomically (32-bit aligned stores on
 *     Cortex-M4 are single-instruction and thus interrupt-safe).
 */

/**
 * @brief  EXTI4 ISR body — call from EXTI4_IRQHandler in stm32g4xx_it.c.
 *         Records TIM2 counter, performs debounce, and flags encoder slip.
 */
void EncoderZ_IRQHandler(void);

/**
 * @brief  Total number of Z pulses accepted since boot.
 *         Useful for confirming the Z channel is electrically active.
 */
uint32_t Encoder_Z_GetPulseCount(void);

/**
 * @brief  TIM2 counter value captured at the most recent Z pulse.
 *         Returns 0 if no pulse has been received yet.
 */
int32_t Encoder_Z_GetLastPosition(void);

/**
 * @brief  Position error at the last Z pulse (current − expected count).
 *         Expected = previous Z position ± ENCODER_CPR (nearest multiple).
 *         Returns 0 if fewer than 2 pulses have been received.
 */
int32_t Encoder_Z_GetLastError(void);

/**
 * @brief  Returns true if the last Z pulse showed a position error larger
 *         than ENC_Z_SLIP_THRESHOLD.  Latching — call Encoder_Z_ClearSlip()
 *         to acknowledge.
 */
bool Encoder_Z_HasSlipped(void);

/**
 * @brief  Clear the slip latch (acknowledgement).
 */
void Encoder_Z_ClearSlip(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_READER_H */
