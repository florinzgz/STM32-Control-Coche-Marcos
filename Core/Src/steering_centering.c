/**
  ****************************************************************************
  * @file    steering_centering.c
  * @brief   Automatic steering centering — implementation
  *
  * Centering sequence:
  *   1. Begin sweeping LEFT at low PWM (~10 %).
  *   2. While sweeping, monitor:
  *      a) Inductive center sensor pulse → center found.
  *      b) Encoder stall (count unchanged for STALL_TIMEOUT_MS while
  *         motor is commanded) → end-of-travel reached.
  *      c) Total timeout (TOTAL_TIMEOUT_MS) → abort.
  *      d) Encoder range exceeded (MAX_CENTERING_COUNTS) → abort.
  *   3. If end-of-travel stall detected during LEFT sweep, reverse to
  *      sweep RIGHT.
  *   4. If end-of-travel stall detected during RIGHT sweep without
  *      center found → FAULT.
  *   5. When center pulse detected:
  *      a) Immediately stop the motor (PWM = 0, disable H-bridge).
  *      b) Zero the encoder counter at this position.
  *      c) Call Steering_SetCalibrated() to unlock normal operation.
  *      d) Transition to CENTERING_DONE.
  *   6. On any fault:
  *      a) Neutralise the steering motor.
  *      b) Latch SAFETY_ERROR_CENTERING.
  *      c) Transition system to SAFE state.
  *
  * Safety limits:
  *   - CENTERING_PWM           ~425  (10 % of 4249 full scale)
  *   - STALL_TIMEOUT_MS        300   (encoder frozen while driving)
  *   - TOTAL_TIMEOUT_MS        10000 (absolute deadline)
  *   - MAX_CENTERING_COUNTS    6000  (guard against runaway travel)
  *
  * Failure conditions:
  *   - Center pulse not detected after sweeping both directions.
  *   - Total timeout exceeded.
  *   - Encoder range exceeded.
  *   - Encoder fault (detected by Encoder_CheckHealth).
  ****************************************************************************
  */

#include "steering_centering.h"
#include "motor_control.h"
#include "sensor_manager.h"
#include "safety_system.h"
#include "main.h"
#include <math.h>

/* ---- Centering constants ----
 *
 * CENTERING_PWM: ~10 % of full PWM scale (4249 at 20 kHz center-aligned).
 * Intentionally LOW to avoid forcing the rack.  Positioning only, not power.
 *
 * STALL_TIMEOUT_MS: If the encoder does not change for this duration
 * while the motor is being driven, we infer end-of-travel (no
 * mechanical endstop assumption — purely encoder-based).
 *
 * TOTAL_TIMEOUT_MS: If centering is not complete within this time,
 * something is wrong.  Abort and enter SAFE state.
 *
 * MAX_CENTERING_COUNTS: Maximum encoder travel in one direction before
 * we conclude the center sensor is not going to be found (guard).      */

#define CENTERING_PWM              425U    /* ~10 % of 4249             */
#define STALL_TIMEOUT_MS           300U    /* End-of-travel stall (ms)  */
#define TOTAL_TIMEOUT_MS           10000U  /* Absolute deadline (ms)    */
#define MAX_CENTERING_COUNTS       6000    /* Safety range limit        */

/* ---- Module state ---- */
static CenteringState_t centering_state = CENTERING_IDLE;
static uint32_t centering_start_tick    = 0;   /* Tick when centering began   */
static int32_t  stall_prev_count        = 0;   /* Last encoder reading        */
static uint32_t stall_last_change_tick  = 0;   /* Tick when encoder last moved*/
static int32_t  sweep_origin_count      = 0;   /* Encoder at sweep start      */

extern TIM_HandleTypeDef htim2;

/* ---- Private helpers ---- */

/**
 * @brief  Abort centering: stop motor, latch fault, enter DEGRADED.
 *
 * Centering failure enters DEGRADED (not SAFE) to preserve the base
 * firmware's "drive-home" philosophy.  The vehicle can still operate
 * at reduced power with steering neutralised.  Traced to
 * limp_mode.cpp: !steeringCentered → LimpState::LIMP.
 */
static void Centering_Abort(void)
{
    Steering_Neutralize();
    centering_state = CENTERING_FAULT;
    Safety_SetError(SAFETY_ERROR_CENTERING);
    Safety_SetState(SYS_STATE_DEGRADED);
    Safety_SetDegradedLevel(DEGRADED_L1,
                            DEGRADED_REASON_CENTERING_FAIL);
}

/**
 * @brief  Complete centering: stop motor, zero encoder, mark calibrated.
 */
static void Centering_Complete(void)
{
    Steering_Neutralize();
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    Steering_SetCalibrated();
    SteeringCenter_ClearFlag();
    centering_state = CENTERING_DONE;
}

/**
 * @brief  Check if the encoder has stalled (end-of-travel inference).
 * @retval true if the encoder count has not changed for STALL_TIMEOUT_MS.
 */
static bool Centering_IsStalled(void)
{
    int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint32_t now  = HAL_GetTick();

    if (count != stall_prev_count) {
        stall_prev_count      = count;
        stall_last_change_tick = now;
        return false;
    }

    return ((now - stall_last_change_tick) >= STALL_TIMEOUT_MS);
}

/**
 * @brief  Check if the encoder has exceeded the safe centering range.
 * @retval true if |current_count - sweep_origin| > MAX_CENTERING_COUNTS.
 */
static bool Centering_RangeExceeded(void)
{
    int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t delta = count - sweep_origin_count;
    if (delta < 0) delta = -delta;
    return (delta > MAX_CENTERING_COUNTS);
}

/* ==================================================================
 *  Public API
 * ================================================================== */

void SteeringCentering_Init(void)
{
    centering_state = CENTERING_IDLE;
    SteeringCenter_ClearFlag();
}

void SteeringCentering_Step(void)
{
    /* Once centering is done or faulted, do nothing further. */
    if (centering_state == CENTERING_DONE ||
        centering_state == CENTERING_FAULT) {
        return;
    }

    /* If the encoder is already faulted, abort immediately. */
    if (Encoder_HasFault()) {
        Centering_Abort();
        return;
    }

    uint32_t now = HAL_GetTick();

    switch (centering_state) {

    /* ---- IDLE: first call starts centering ---- */
    case CENTERING_IDLE:
        SteeringCenter_ClearFlag();
        centering_start_tick   = now;
        stall_prev_count       = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        stall_last_change_tick = now;
        sweep_origin_count     = stall_prev_count;

        /* Begin sweeping LEFT (direction = reverse = true → negative) */
        Motor_SetPWM_Steering(CENTERING_PWM, true);
        centering_state = CENTERING_SWEEP_LEFT;
        break;

    /* ---- SWEEP LEFT ---- */
    case CENTERING_SWEEP_LEFT:
        /* 1. Center detected? */
        if (SteeringCenter_Detected()) {
            Centering_Complete();
            return;
        }

        /* 2. Total timeout? */
        if ((now - centering_start_tick) >= TOTAL_TIMEOUT_MS) {
            Centering_Abort();
            return;
        }

        /* 3. Range exceeded? */
        if (Centering_RangeExceeded()) {
            Centering_Abort();
            return;
        }

        /* 4. End-of-travel stall? → reverse to sweep right */
        if (Centering_IsStalled()) {
            sweep_origin_count     = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
            stall_prev_count       = sweep_origin_count;
            stall_last_change_tick = now;
            Motor_SetPWM_Steering(CENTERING_PWM, false);
            centering_state = CENTERING_SWEEP_RIGHT;
        }
        break;

    /* ---- SWEEP RIGHT ---- */
    case CENTERING_SWEEP_RIGHT:
        /* 1. Center detected? */
        if (SteeringCenter_Detected()) {
            Centering_Complete();
            return;
        }

        /* 2. Total timeout? */
        if ((now - centering_start_tick) >= TOTAL_TIMEOUT_MS) {
            Centering_Abort();
            return;
        }

        /* 3. Range exceeded? */
        if (Centering_RangeExceeded()) {
            Centering_Abort();
            return;
        }

        /* 4. End-of-travel stall? → center was never found */
        if (Centering_IsStalled()) {
            Centering_Abort();
        }
        break;

    default:
        break;
    }
}

bool SteeringCentering_IsComplete(void)
{
    return (centering_state == CENTERING_DONE);
}

bool SteeringCentering_HasFault(void)
{
    return (centering_state == CENTERING_FAULT);
}

CenteringState_t SteeringCentering_GetState(void)
{
    return centering_state;
}
