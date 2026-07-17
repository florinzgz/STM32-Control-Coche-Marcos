/**
  ****************************************************************************
  * @file    standby_mode_sync_policy.c
  * @brief   HAL-free implementation of the STANDBY logical-mode-sync policy.
  *
  *          See standby_mode_sync_policy.h for the full contract.
  *          This translation unit has NO HAL dependency and compiles on host
  *          GCC unchanged.
  ****************************************************************************
  */

#include "standby_mode_sync_policy.h"
#include <math.h>   /* isnan(), isinf() */

/* Helper: return @p fallback when @p v is NaN or Inf (fail-safe). */
static float sp_sanitize(float v, float fallback)
{
    if (isnan(v) || isinf(v)) return fallback;
    return v;
}

StandbySyncResult_t StandbyModeSync_Evaluate(const StandbySyncInput_t *in)
{
    /* 1 — State must be exactly STANDBY. */
    if (in->state != STANDBY_SYNC_STATE_STANDBY)
        return STANDBY_SYNC_BLOCKED;

    /* 2 — Pedal released. */
    float pedal = sp_sanitize(in->pedalPct, 100.0f);
    if (pedal > STANDBY_MODE_SYNC_MAX_PEDAL_PCT)
        return STANDBY_SYNC_BLOCKED;

    /* 3 — Internal demand (TractionState_t->demandPct) is zero. */
    float demand = sp_sanitize(in->demandPct, 100.0f);
    if (demand > STANDBY_MODE_SYNC_DEMAND_EPSILON)
        return STANDBY_SYNC_BLOCKED;

    /* 4 — Effective demand (post-pipeline) is zero. */
    float eff = sp_sanitize(in->effectiveDemandPct, 100.0f);
    if (eff > STANDBY_MODE_SYNC_DEMAND_EPSILON)
        return STANDBY_SYNC_BLOCKED;

    /* 5 — Final applied PWM is zero (motors idle). */
    if (in->finalPwmPct != 0U)
        return STANDBY_SYNC_BLOCKED;

    /* 6 — Vehicle at safe (near-zero) speed. */
    float speed = sp_sanitize(in->avgSpeedKmh,
                              STANDBY_MODE_SYNC_MAX_SPEED_KMH + 1.0f);
    if (speed > STANDBY_MODE_SYNC_MAX_SPEED_KMH)
        return STANDBY_SYNC_BLOCKED;

    /* 7 — No active error or hazard latch. */
    if (in->errorOrHazardActive)
        return STANDBY_SYNC_BLOCKED;

    return STANDBY_SYNC_ALLOW_LOGICAL_ONLY;
}
