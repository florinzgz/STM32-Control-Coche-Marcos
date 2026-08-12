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

/* BLOQUE 1 (dynbrake_store): motor_control.c's dynamic-brake factor is no
 * longer a fixed compile-time constant — it is the runtime-tunable
 * dynbrake_store.h::factor_pct (default 0.20, the same value this ratio
 * used to manufacture post-hoc from the historic 0.50 constant).  BASE and
 * TARGET are therefore equal so the ratio is neutral (1.0): the store is
 * now the single source of truth for the factor, and this file no longer
 * applies a second, hidden scale-down on top of it.  DRIVE_DYNBRAKE_MAX_PCT
 * remains the absolute field-safety backstop, applied downstream regardless
 * of the store's own (wider, 0-60) validated range — defense in depth. */
#define DRIVE_DYNBRAKE_BASE_FACTOR        1.00f
#define DRIVE_DYNBRAKE_TARGET_FACTOR      1.00f
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

/**
 * @param min_reference_kmh Effective (possibly service-tuned) value of
 *                           DRIVE_TCS_MIN_REFERENCE_KMH; pass the macro
 *                           itself when no runtime override applies.
 * @param slip_threshold_pct Effective value of DRIVE_TCS_SLIP_THRESHOLD_PCT.
 *
 * Kept as explicit parameters (rather than reading globals from inside the
 * inline body) so this function stays a pure, header-only, HAL-free unit
 * that test_drive_dynamics_policy.c can keep exercising standalone.  Callers
 * that own a runtime store (tcs_tuning_store.c via tcs_tuned.c) pass the
 * live effective values; everyone else may pass the compile-time macros.
 */
static inline bool DriveDynamics_TcsIsSlipping(float reference_kmh,
                                                float wheel_kmh,
                                                float min_reference_kmh,
                                                float slip_threshold_pct)
{
    if (!isfinite(reference_kmh) || !isfinite(wheel_kmh) ||
        reference_kmh < min_reference_kmh || wheel_kmh < 0.0f) {
        return false;
    }

    const float slip_pct =
        ((wheel_kmh - reference_kmh) * 100.0f) / reference_kmh;
    return slip_pct > slip_threshold_pct;
}

/**
 * @param initial_reduction    Effective DRIVE_TCS_INITIAL_REDUCTION.
 * @param reduction_rate_per_s Effective DRIVE_TCS_REDUCTION_RATE_PER_S.
 * @param recovery_rate_per_s  Effective DRIVE_TCS_RECOVERY_RATE_PER_S.
 * @param max_reduction        Effective DRIVE_TCS_MAX_REDUCTION.
 * See DriveDynamics_TcsIsSlipping() for why these are explicit parameters.
 */
static inline float DriveDynamics_TcsReductionNext(float current,
                                                    bool slipping,
                                                    float dt_s,
                                                    float initial_reduction,
                                                    float reduction_rate_per_s,
                                                    float recovery_rate_per_s,
                                                    float max_reduction)
{
    current = DriveDynamics_Clamp01(current);
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 1.0f) dt_s = 0.01f;

    if (slipping) {
        if (current < 0.01f) current = initial_reduction;
        else current += reduction_rate_per_s * dt_s;
        if (current > max_reduction) {
            current = max_reduction;
        }
    } else {
        current -= recovery_rate_per_s * dt_s;
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
