#ifndef EPS_OUTPUT_POLICY_H
#define EPS_OUTPUT_POLICY_H

#include <stdbool.h>
#include <math.h>

typedef struct {
    float pwm_pct;
    bool  coast;
} EpsOutputDecision_t;

/* Resolve a requested EPS output without touching hardware.
 * Requests inside the coast band physically release the H-bridge.  Minimum
 * drive compensation is applied only after genuine driver/return intent has
 * crossed that band, preventing a tiny request from becoming a permanent
 * ~8 percent steering effort near centre. */
static inline EpsOutputDecision_t
EpsOutput_Resolve(float pwm_pct, float coast_band_pct, float min_drive_pct)
{
    EpsOutputDecision_t out = { 0.0f, true };

    if (isnan(pwm_pct) || isinf(pwm_pct) ||
        isnan(coast_band_pct) || isinf(coast_band_pct) ||
        isnan(min_drive_pct) || isinf(min_drive_pct) ||
        coast_band_pct < 0.0f || min_drive_pct < 0.0f) {
        return out;
    }

    const float abs_pct = fabsf(pwm_pct);
    if (abs_pct <= 0.01f || abs_pct < coast_band_pct) {
        return out;
    }

    out.pwm_pct = pwm_pct;
    if (abs_pct < min_drive_pct) {
        out.pwm_pct = (pwm_pct > 0.0f) ? min_drive_pct : -min_drive_pct;
    }
    out.coast = false;
    return out;
}

#endif /* EPS_OUTPUT_POLICY_H */
