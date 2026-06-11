/**
  ****************************************************************************
  * @file    steering_z.c
  * @brief   Encoder Z (index) channel — SECONDARY center reference logic
  *
  * See steering_z.h for the dual-reference policy.  This translation unit
  * is pure logic (no HAL / flash / ISR) so it compiles unchanged on the
  * host unit-test harness.
  ****************************************************************************
  */

#include "steering_z.h"
#include "project_config.h"

/* ---- Runtime state (main-loop context only) ---- */
static int32_t           z_center_offset_counts = 0;
static bool              z_center_valid          = false;
static SteeringZStatus_t z_status                = STEERING_Z_NOT_CALIBRATED;

/* ==================================================================
 *  Pure helpers
 * ================================================================== */

int32_t SteeringZ_NormaliseOffset(int32_t raw_delta)
{
    int32_t cpr  = (int32_t)ENCODER_CPR;       /* 4800 */
    int32_t half = cpr / 2;                     /* 2400 */

    /* Reduce modulo CPR into [0, CPR) using a sign-safe remainder. */
    int32_t r = raw_delta % cpr;
    if (r < 0) r += cpr;                        /* r in [0, CPR)     */

    /* Fold the upper half to the negative side → (-half, +half]. */
    if (r > half) r -= cpr;
    return r;
}

SteeringZStatus_t SteeringZ_Classify(bool pb5_confirmed,
                                     bool z_seen,
                                     int32_t offset_counts)
{
    /* Z can never assert center without PB5. */
    if (!pb5_confirmed)
        return STEERING_Z_NOT_CALIBRATED;

    if (!z_seen)
        return STEERING_Z_NOT_SEEN;

    int32_t a = (offset_counts < 0) ? -offset_counts : offset_counts;

    if (a <= STEERING_Z_WINDOW_COUNTS)
        return STEERING_Z_OK;
    if (a <= STEERING_Z_FAULT_COUNTS)
        return STEERING_Z_OUT_OF_WINDOW;
    return STEERING_Z_MECH_OFFSET;
}

/* ==================================================================
 *  Stateful API
 * ================================================================== */

void SteeringZ_Init(void)
{
    z_center_offset_counts = 0;
    z_center_valid         = false;
    z_status               = STEERING_Z_NOT_CALIBRATED;
}

void SteeringZ_OnCenterConfirmed(int32_t center_count,
                                 int32_t z_last_pos,
                                 bool z_seen)
{
    /* PB5 has confirmed center (caller contract).  Compute the offset of
     * the nearest Z pulse relative to that center.  Folding modulo CPR
     * means whichever revolution's Z we last saw collapses to the same
     * fundamental offset, so we never confuse a Z from another turn. */
    int32_t offset = SteeringZ_NormaliseOffset(z_last_pos - center_count);

    z_status = SteeringZ_Classify(true, z_seen, offset);

    if (z_seen && (z_status == STEERING_Z_OK ||
                   z_status == STEERING_Z_OUT_OF_WINDOW)) {
        /* Store the offset when a Z is present and not grossly misaligned.
         * STEERING_Z_OK marks a high-precision calibration; OUT_OF_WINDOW
         * still records the offset for diagnostics but is not "valid". */
        z_center_offset_counts = offset;
        z_center_valid         = (z_status == STEERING_Z_OK);
    } else {
        /* No usable Z (not seen, or mechanical offset). Keep PB5 center;
         * Z simply not validated.  Never blocks operation. */
        z_center_offset_counts = z_seen ? offset : 0;
        z_center_valid         = false;
    }
}

void SteeringZ_LoadFromFlash(int32_t offset_counts, bool valid)
{
    z_center_offset_counts = offset_counts;
    z_center_valid         = valid;
    z_status               = valid ? STEERING_Z_OK : STEERING_Z_NOT_CALIBRATED;
}

void SteeringZ_ClearCalibration(void)
{
    z_center_offset_counts = 0;
    z_center_valid         = false;
    z_status               = STEERING_Z_NOT_CALIBRATED;
}

int32_t           SteeringZ_GetOffset(void) { return z_center_offset_counts; }
bool              SteeringZ_IsValid(void)   { return z_center_valid;          }
SteeringZStatus_t SteeringZ_GetStatus(void) { return z_status;                }
