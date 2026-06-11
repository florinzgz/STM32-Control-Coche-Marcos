/**
  ****************************************************************************
  * @file    steering_z.h
  * @brief   Encoder Z (index) channel — SECONDARY center reference logic
  *
  * Implements the PB5 + Z dual-reference centering policy.
  *
  *   PB5 (LJ12A3 inductive, EXTI5) = PRIMARY physical/safety center
  *                                   reference.  Unchanged, mandatory.
  *   Z   (E6B2-CWZ6C index, PB4)   = SECONDARY precision reference, used
  *                                   only for verification, diagnostics and
  *                                   (optionally) sub-degree refinement.
  *
  * Hard rules enforced here:
  *   - Z is NEVER allowed to center the system on its own.
  *   - A Z pulse seen WITHOUT PB5 confirmation is NOT a center.
  *   - The Z offset is meaningful only AFTER PB5 confirms the center zone.
  *
  * This module is pure logic (no HAL, no flash, no ISR).  The numeric
  * inputs are produced by encoder_reader.c (Z capture) and
  * steering_centering.c (PB5 center event).  It is compiled both into the
  * firmware and into the host unit-test harness.
  ****************************************************************************
  */

#ifndef STEERING_Z_H
#define STEERING_Z_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ---- Combined PB5/Z validation status (CAN + HMI visible) ---- */
typedef enum {
    STEERING_Z_NOT_CALIBRATED = 0, /* No Z offset stored / computed yet      */
    STEERING_Z_OK             = 1, /* PB5 + Z agree within window            */
    STEERING_Z_NOT_SEEN       = 2, /* PB5 centered but no Z pulse captured   */
    STEERING_Z_OUT_OF_WINDOW  = 3, /* PB5 + Z disagree (> window, <= fault)  */
    STEERING_Z_MECH_OFFSET    = 4  /* PB5 + Z disagree strongly (> fault)    */
} SteeringZStatus_t;

/* ---- Pure helpers (no state) ---- */

/**
 * @brief  Normalise a raw count difference (z_position - center) into the
 *         signed residual nearest to zero, in the range
 *         (-ENCODER_CPR/2, +ENCODER_CPR/2].
 *
 *         Because the encoder turns 1:1 with the steering column and the Z
 *         pulse fires once per revolution, the Z pulses inside the ±355°
 *         travel sit at center_offset + k*CPR.  This collapses any of them
 *         to the single fundamental offset relative to center, so the
 *         "correct" Z (the one in the PB5 center zone) and a Z from an
 *         adjacent revolution map to the same residual.
 *
 * @param  raw_delta  z_position_counts - center_count (any revolution).
 * @retval Signed residual in counts, nearest multiple of CPR removed.
 */
int32_t SteeringZ_NormaliseOffset(int32_t raw_delta);

/**
 * @brief  Classify a Z offset (counts, already normalised) into a status,
 *         given whether PB5 confirmed the center and whether a Z pulse was
 *         actually captured.
 *
 *         Z without PB5 is never a center → STEERING_Z_NOT_SEEN is reserved
 *         for "PB5 ok, Z missing"; callers must not pass pb5_confirmed=false
 *         expecting an OK result.
 *
 * @param  pb5_confirmed  true if PB5 confirmed the physical center zone.
 * @param  z_seen         true if at least one Z pulse was captured.
 * @param  offset_counts  normalised Z offset relative to center.
 * @retval Combined validation status.
 */
SteeringZStatus_t SteeringZ_Classify(bool pb5_confirmed,
                                     bool z_seen,
                                     int32_t offset_counts);

/* ==================================================================
 *  Stateful runtime API (single-context: main loop)
 * ================================================================== */

/**
 * @brief  Reset all runtime Z-center state.  Called from module init.
 */
void SteeringZ_Init(void);

/**
 * @brief  Record the Z↔center offset at the moment PB5 confirms center.
 *
 *         MUST be called only after PB5 has detected the physical center
 *         (steering_centering.c, Centering_Complete).  Computes and stores
 *         the normalised offset and the combined validation status.
 *
 * @param  center_count   TIM2 count at the PB5 center detection (pre-zero).
 * @param  z_last_pos     TIM2 count captured at the most recent Z pulse.
 * @param  z_seen         true if at least one Z pulse has been captured.
 */
void SteeringZ_OnCenterConfirmed(int32_t center_count,
                                 int32_t z_last_pos,
                                 bool z_seen);

/**
 * @brief  Load a Z calibration restored from flash (boot path).
 * @param  offset_counts  stored z_center_offset_counts.
 * @param  valid          stored z_center_valid flag.
 */
void SteeringZ_LoadFromFlash(int32_t offset_counts, bool valid);

/**
 * @brief  Clear the stored Z calibration (HMI CLEAR action).
 */
void SteeringZ_ClearCalibration(void);

/* ---- Getters (telemetry / HMI / flash persistence) ---- */
int32_t           SteeringZ_GetOffset(void);     /* z_center_offset_counts   */
bool              SteeringZ_IsValid(void);        /* z_center_valid           */
SteeringZStatus_t SteeringZ_GetStatus(void);      /* combined PB5/Z status    */

#ifdef __cplusplus
}
#endif

#endif /* STEERING_Z_H */
