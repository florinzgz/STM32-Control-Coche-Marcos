/**
  ****************************************************************************
  * @file    encoder_reader.c
  * @brief   Read-only interface for E6B2-CWZ6C quadrature encoder (TIM2)
  *
  *          Hardware-integrated, not used for control.
  *          Provides raw counter access for diagnostics and validation.
  *          The encoder hardware (TIM2 in quadrature mode, 32-bit,
  *          4800 CPR) is initialised by MX_TIM2_Init() in main.c and
  *          started by Motor_Init() in motor_control.c.
  *
  *          This module does NOT:
  *            – filter or smooth the count
  *            – convert to speed, angle, or distance
  *            – feed into any control loop
  ****************************************************************************
  */

#include "encoder_reader.h"
#include "main.h"
#include "can_handler.h"

/* ---- Internal state for delta tracking ----
 * Initialised to 0, matching the TIM2 counter state after
 * Steering_Init() zeros it.  All public functions in this module
 * must be called from a single context (main loop) — they are
 * not reentrant and share enc_reader_prev without locking.      */
static int32_t enc_reader_prev = 0;

/* ==================================================================
 *  Public API
 * ================================================================== */

int32_t Encoder_GetRawCount(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

int32_t Encoder_GetDelta(void)
{
    int32_t current = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t delta   = current - enc_reader_prev;
    enc_reader_prev = current;
    return delta;
}

void Encoder_Reset(void)
{
    enc_reader_prev = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

void Encoder_SendDiagnostic(void)
{
    int32_t raw   = Encoder_GetRawCount();
    int32_t delta = Encoder_GetDelta();

    /* Clamp delta to int16 range for CAN packing */
    int16_t delta16;
    if (delta >  32767) delta16 =  32767;
    else if (delta < -32768) delta16 = -32768;
    else delta16 = (int16_t)delta;

    CAN_SendDiagnosticEncoder(raw, delta16);
}
