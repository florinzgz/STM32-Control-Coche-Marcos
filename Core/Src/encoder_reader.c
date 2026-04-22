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
  *
  *          Z (index) channel — PB4 / EXTI4:
  *            – Detects inter-revolution position drift / encoder slip.
  *            – Does NOT zero TIM2 (centering uses the PB5 inductive sensor).
  *            – Provides slip-detection diagnostics for high-EMI environments.
  ****************************************************************************
  */

#include "encoder_reader.h"
#include "main.h"
#include "can_handler.h"

/* TIM2 handle — defined in main.c.  Declared here as well so that the
 * build succeeds even if CubeMX regenerates main.h without the extern. */
extern TIM_HandleTypeDef htim2;

/* ---- Internal state for delta tracking ----
 * Initialised to 0, matching the TIM2 counter state after
 * Steering_Init() zeros it.  All public functions in this module
 * must be called from a single context (main loop) — they are
 * not reentrant and share enc_reader_prev without locking.      */
static int32_t enc_reader_prev = 0;

/* ==================================================================
 *  Encoder Z (index) channel — PB4 / EXTI4
 *
 *  Debounce: ENC_Z_MIN_INTERVAL_MS is the minimum time between two
 *  accepted Z pulses.  At the maximum steering motor speed the encoder
 *  shaft produces one Z pulse roughly every 30 ms (2000 RPM).  5 ms
 *  safely rejects EMI-induced spike trains while accepting all valid
 *  pulses.
 *
 *  Slip threshold: ENC_Z_SLIP_THRESHOLD is the maximum tolerated
 *  deviation of the inter-pulse delta from ±ENCODER_CPR (4800 counts).
 *  48 counts ≈ 1% of one revolution (3.6°).  Genuine encoder slip
 *  from EMI on the A/B lines produces much larger errors.
 *
 *  All volatile state is updated from the EXTI4 ISR only; all reads
 *  are from the main loop.  On Cortex-M4, 32-bit aligned store/load
 *  instructions are atomic, so no critical section is required for
 *  individual uint32_t / int32_t fields.
 * ================================================================== */
#define ENC_Z_MIN_INTERVAL_MS   5U    /* Debounce window (ms)            */
#define ENC_Z_SLIP_THRESHOLD    48    /* Max tolerated CPR error (counts)*/

static volatile uint32_t enc_z_pulse_count = 0;   /* Accepted Z pulses  */
static volatile int32_t  enc_z_last_pos    = 0;   /* TIM2 cnt at last Z */
static volatile int32_t  enc_z_last_error  = 0;   /* Deviation (counts) */
static volatile uint8_t  enc_z_slip        = 0;   /* Slip latch         */
static volatile uint32_t enc_z_last_tick   = 0;   /* Debounce timestamp */

/* ==================================================================
 *  Public API — A/B quadrature channels
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

/* ==================================================================
 *  Encoder Z (index) channel — ISR + public API
 * ================================================================== */

/**
 * @brief  EXTI4 ISR body for the encoder Z (index) pulse on PB4.
 *
 * Sequence:
 *   1. Debounce: reject pulses arriving within ENC_Z_MIN_INTERVAL_MS.
 *   2. Capture TIM2->CNT directly (ISR-safe register access).
 *   3. From the second pulse onward, compute the inter-pulse delta and
 *      compare to the nearest integer multiple of ENCODER_CPR.
 *      Any remainder larger than ENC_Z_SLIP_THRESHOLD latches enc_z_slip.
 *   4. Update enc_z_last_pos and enc_z_pulse_count.
 *
 * Complexity: O(1).  No malloc, no blocking, no HAL calls except
 * HAL_GetTick() (reads volatile uint32_t uwTick — always safe from ISR).
 */
void EncoderZ_IRQHandler(void)
{
    uint32_t now = HAL_GetTick();

    /* 1. Debounce — reject EMI spike trains */
    if ((now - enc_z_last_tick) < ENC_Z_MIN_INTERVAL_MS)
        return;
    enc_z_last_tick = now;

    /* 2. Capture current TIM2 counter (direct register — ISR-safe) */
    int32_t pos = (int32_t)TIM2->CNT;

    /* 3. Slip detection — only from the second pulse onward */
    if (enc_z_pulse_count > 0U)
    {
        /* Inter-pulse delta — signed, handles both directions */
        int32_t delta = pos - enc_z_last_pos;

        /* Normalise delta to the nearest integer multiple of ENCODER_CPR.
         * remainder = delta mod ENCODER_CPR, adjusted to [-CPR/2, +CPR/2].
         * A remainder of 0 means the pulse arrived exactly on schedule.    */
        int32_t cpr = (int32_t)ENCODER_CPR;
        int32_t remainder = delta % cpr;
        if (remainder >  cpr / 2) remainder -= cpr;
        if (remainder < -cpr / 2) remainder += cpr;

        enc_z_last_error = remainder;

        if (remainder < 0) remainder = -remainder;   /* |remainder| */
        if (remainder > ENC_Z_SLIP_THRESHOLD)
            enc_z_slip = 1U;
    }

    /* 4. Update state */
    enc_z_last_pos = pos;
    enc_z_pulse_count++;
}

/* ---- Public Z getters (main-loop context) ---- */

uint32_t Encoder_Z_GetPulseCount(void)   { return enc_z_pulse_count; }
int32_t  Encoder_Z_GetLastPosition(void) { return enc_z_last_pos;    }
int32_t  Encoder_Z_GetLastError(void)    { return enc_z_last_error;  }
bool     Encoder_Z_HasSlipped(void)      { return (enc_z_slip != 0U); }
void     Encoder_Z_ClearSlip(void)       { enc_z_slip = 0U;          }
