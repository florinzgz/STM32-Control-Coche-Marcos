#ifndef DRIVE_DYNAMICS_POLICY_H
#define DRIVE_DYNAMICS_POLICY_H

#include <stdbool.h>
#include <math.h>

#define DRIVE_TCS_MIN_REFERENCE_KMH       6.0f
#define DRIVE_TCS_SLIP_THRESHOLD_PCT     25.0f
#define DRIVE_TCS_INITIAL_REDUCTION       0.40f
#define DRIVE_TCS_REDUCTION_RATE_PER_S    1.00f
#define DRIVE_TCS_RECOVERY_RATE_PER_S     0.25f
#define DRIVE_TCS_MAX_REDUCTION           0.80f

#define DRIVE_DYNBRAKE_BASE_FACTOR        0.50f
#define DRIVE_DYNBRAKE_TARGET_FACTOR      0.20f
#define DRIVE_DYNBRAKE_FACTOR_RATIO       \
    (DRIVE_DYNBRAKE_TARGET_FACTOR / DRIVE_DYNBRAKE_BASE_FACTOR)
#define DRIVE_DYNBRAKE_MAX_PCT           30.0f

#define DRIVE_EPS_INTENT_OMEGA_LO_DPS      0.5f
#define DRIVE_EPS_INTENT_OMEGA_HI_DPS      4.0f

static inline float DriveDynamics_Clamp01(float value)
{
    if (!isfinite(value) || value <= 0.0f) return 0.0f;
    if (value >= 1.0f) return 1.0f;
    return value;
}

static inline bool DriveDynamics_TcsIsSlipping(float reference_kmh,
                                                float wheel_kmh)
{
    if (!isfinite(reference_kmh) || !isfinite(wheel_kmh) ||
        reference_kmh < DRIVE_TCS_MIN_REFERENCE_KMH || wheel_kmh < 0.0f) {
        return false;
    }

    const float slip_pct =
        ((wheel_kmh - reference_kmh) * 100.0f) / reference_kmh;
    return slip_pct > DRIVE_TCS_SLIP_THRESHOLD_PCT;
}

static inline float DriveDynamics_TcsReductionNext(float current,
                                                    bool slipping,
                                                    float dt_s)
{
    current = DriveDynamics_Clamp01(current);
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 1.0f) dt_s = 0.01f;

    if (slipping) {
        if (current < 0.01f) current = DRIVE_TCS_INITIAL_REDUCTION;
        else current += DRIVE_TCS_REDUCTION_RATE_PER_S * dt_s;
        if (current > DRIVE_TCS_MAX_REDUCTION) {
            current = DRIVE_TCS_MAX_REDUCTION;
        }
    } else {
        current -= DRIVE_TCS_RECOVERY_RATE_PER_S * dt_s;
        if (current < 0.0f) current = 0.0f;
    }
    return current;
}

static inline float DriveDynamics_DynbrakeLimitedPct(float base_pct)
{
    if (!isfinite(base_pct) || base_pct <= 0.0f) return 0.0f;
    float limited = base_pct * DRIVE_DYNBRAKE_FACTOR_RATIO;
    if (limited > DRIVE_DYNBRAKE_MAX_PCT) limited = DRIVE_DYNBRAKE_MAX_PCT;
    return limited;
}

static inline float DriveDynamics_EpsLambda(float abs_omega_dps)
{
    if (!isfinite(abs_omega_dps) ||
        abs_omega_dps <= DRIVE_EPS_INTENT_OMEGA_LO_DPS) {
        return 0.0f;
    }
    if (abs_omega_dps >= DRIVE_EPS_INTENT_OMEGA_HI_DPS) return 1.0f;

    const float t = (abs_omega_dps - DRIVE_EPS_INTENT_OMEGA_LO_DPS) /
                    (DRIVE_EPS_INTENT_OMEGA_HI_DPS -
                     DRIVE_EPS_INTENT_OMEGA_LO_DPS);
    return t * t * (3.0f - 2.0f * t);
}

#endif /* DRIVE_DYNAMICS_POLICY_H */
