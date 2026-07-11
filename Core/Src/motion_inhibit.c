/**
 ****************************************************************************
 * @file    motion_inhibit.c
 * @brief   MOTION_INHIBIT_REASON classifier — see motion_inhibit.h.
 *
 *          Instrumentation only.  Contains NO control or safety policy;
 *          it just classifies a snapshot of the traction pipeline.
 ****************************************************************************
 */
#include "motion_inhibit.h"

#include <math.h>
#include <stddef.h>

/* Local helper: |x| for a possibly-NaN float, NaN treated as 0. */
static float mi_abs(float x)
{
    if (isnan(x) || isinf(x)) {
        return 0.0f;
    }
    return (x < 0.0f) ? -x : x;
}

uint16_t MotionInhibit_Evaluate(const MotionInhibitInputs *in)
{
    uint16_t reason = MOTION_INHIBIT_NONE;

    if (in == NULL) {
        return reason;
    }

    /* ---- Dominant gating reasons (root causes) ------------------------- */
    if (in->state == in->state_safe) {
        reason |= MOTION_INHIBIT_STATE_SAFE;
    }
    if (in->state == in->state_error) {
        reason |= MOTION_INHIBIT_STATE_ERROR;
    }
    /* Power-not-ready only inhibits while NOT already in SAFE/ERROR (those
     * states force outputs down and power off independently).             */
    if (!in->power_ready &&
        in->state != in->state_safe &&
        in->state != in->state_error) {
        reason |= MOTION_INHIBIT_POWER_NOT_READY;
    }
    if (in->gear == in->gear_park) {
        reason |= MOTION_INHIBIT_GEAR_PARK;
    }
    if (in->gear == in->gear_neutral) {
        reason |= MOTION_INHIBIT_GEAR_NEUTRAL;
    }

    /* Obstacle blocks forward motion only (reverse escape is allowed). */
    if (in->obstacle_forward_blocked && in->forward_gear) {
        reason |= MOTION_INHIBIT_OBSTACLE_BLOCK;
    }

    /* ---- Demand / PWM chain -------------------------------------------- */
    const float demand    = mi_abs(in->demand_pct);
    const float effective = mi_abs(in->effective_demand_pct);
    const bool  asking     = (demand >= MOTION_INHIBIT_DEMAND_EPS_PCT);

    if (!asking) {
        /* Operator is not requesting torque. */
        reason |= MOTION_INHIBIT_NO_DEMAND;
    } else if (effective < MOTION_INHIBIT_DEMAND_EPS_PCT) {
        /* Demand requested but scaled to (near) zero before reaching PWM —
         * e.g. obstacle scale, LIMP speed cap, or a demand-range anomaly. */
        reason |= MOTION_INHIBIT_DEMAND_ZEROED;
    } else if (in->final_pwm_max == 0U) {
        /* Demand survived scaling but the final PWM duty is still zero —
         * e.g. traction cap / per-motor temperature cutoff killed it, or
         * the trac-phase is not DRIVE.  This is the "40 % demand, 0 PWM"
         * signature we specifically want to catch on hardware.            */
        reason |= MOTION_INHIBIT_PWM_ZERO;
    }

    /* Torque-limited (informational): the vehicle can still move, but a
     * DEGRADED/LIMP torque cap is throttling it.                          */
    if (in->degraded_level > 0U) {
        reason |= MOTION_INHIBIT_TORQUE_LIMITED;
    }

    return reason;
}
