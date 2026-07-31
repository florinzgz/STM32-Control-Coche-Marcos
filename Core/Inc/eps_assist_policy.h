#ifndef EPS_ASSIST_POLICY_H
#define EPS_ASSIST_POLICY_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* Low-speed steering is deliberately assist-only.  Without a torque sensor,
 * active return torque at parking speed cannot distinguish a released wheel
 * from a driver who is turning slowly, so it can feel like an intermittent
 * brake.  Driver intent is latched briefly across encoder quantisation gaps. */
#define EPS_DRIVER_INTENT_ENTER_DPS              0.50f
#define EPS_DRIVER_INTENT_EXIT_DPS               0.15f
#define EPS_DRIVER_INTENT_HOLD_MS                 120U
#define EPS_ACTIVE_RETURN_MIN_KMH                 8.0f
#define EPS_DRIVER_ASSIST_COAST_MAX_PCT           1.0f
#define EPS_DRIVER_ASSIST_MARGIN_PCT              0.25f
#define EPS_DRIVER_ASSIST_BREAKAWAY_MAX_PCT       4.0f
#define EPS_DRIVER_ASSIST_SLEW_MAX_PCT            1.5f
#define EPS_LOW_SPEED_ASSIST_MAX_PCT              20.0f

typedef struct {
    bool     active;
    int8_t   direction;
    uint32_t last_motion_ms;
} EpsAssistState_t;

typedef struct {
    float pwm_pct;
    bool  coast;
    bool  driver_intent;
} EpsAssistDecision_t;

static inline void EpsAssist_Reset(EpsAssistState_t *state)
{
    if (state == NULL) return;
    state->active = false;
    state->direction = 0;
    state->last_motion_ms = 0U;
}

static inline bool EpsAssist_IsFiniteNonnegative(float value)
{
    return isfinite(value) && value >= 0.0f;
}

static inline EpsAssistDecision_t EpsAssist_Resolve(
    EpsAssistState_t *state,
    uint32_t now_ms,
    float omega_dps,
    float vehicle_speed_kmh,
    float assist_component_pct,
    float return_component_pct,
    float coast_band_pct)
{
    EpsAssistDecision_t out = { 0.0f, true, false };
    if (state == NULL || !isfinite(omega_dps) ||
        !EpsAssist_IsFiniteNonnegative(vehicle_speed_kmh) ||
        !isfinite(assist_component_pct) ||
        !isfinite(return_component_pct) ||
        !EpsAssist_IsFiniteNonnegative(coast_band_pct)) {
        if (state != NULL) EpsAssist_Reset(state);
        return out;
    }

    const float abs_omega = fabsf(omega_dps);
    if (abs_omega >= EPS_DRIVER_INTENT_ENTER_DPS) {
        state->active = true;
        state->direction = (omega_dps >= 0.0f) ? 1 : -1;
        state->last_motion_ms = now_ms;
    } else if (state->active) {
        if (abs_omega >= EPS_DRIVER_INTENT_EXIT_DPS) {
            state->direction = (omega_dps >= 0.0f) ? 1 : -1;
            state->last_motion_ms = now_ms;
        } else if ((uint32_t)(now_ms - state->last_motion_ms) >
                   EPS_DRIVER_INTENT_HOLD_MS) {
            EpsAssist_Reset(state);
        }
    }

    if (state->active && state->direction != 0) {
        float magnitude = fabsf(assist_component_pct);
        const float intent_coast =
            (coast_band_pct > EPS_DRIVER_ASSIST_COAST_MAX_PCT)
            ? EPS_DRIVER_ASSIST_COAST_MAX_PCT : coast_band_pct;
        const float genuine_intent_floor =
            intent_coast + EPS_DRIVER_ASSIST_MARGIN_PCT;
        if (magnitude < genuine_intent_floor) {
            magnitude = genuine_intent_floor;
        }
        if (vehicle_speed_kmh <= EPS_ACTIVE_RETURN_MIN_KMH &&
            magnitude > EPS_LOW_SPEED_ASSIST_MAX_PCT) {
            magnitude = EPS_LOW_SPEED_ASSIST_MAX_PCT;
        }
        out.pwm_pct = (state->direction > 0) ? magnitude : -magnitude;
        out.coast = false;
        out.driver_intent = true;
        return out;
    }

    /* No torque-sensor authority exists to distinguish a held wheel from a
     * released wheel while parked.  Below this speed, never self-centre or
     * damp against the driver's hands: physically coast instead. */
    if (vehicle_speed_kmh <= EPS_ACTIVE_RETURN_MIN_KMH) {
        return out;
    }

    out.pwm_pct = return_component_pct;
    out.coast = fabsf(return_component_pct) < coast_band_pct;
    return out;
}

static inline float EpsAssist_EffectiveCoastBand(float configured_pct,
                                                 bool driver_intent)
{
    if (!EpsAssist_IsFiniteNonnegative(configured_pct)) return 0.0f;
    if (driver_intent &&
        configured_pct > EPS_DRIVER_ASSIST_COAST_MAX_PCT) {
        return EPS_DRIVER_ASSIST_COAST_MAX_PCT;
    }
    return configured_pct;
}

static inline float EpsAssist_EffectiveMinDrive(float configured_pct,
                                                bool driver_intent)
{
    if (!EpsAssist_IsFiniteNonnegative(configured_pct)) return 0.0f;
    if (driver_intent &&
        configured_pct > EPS_DRIVER_ASSIST_BREAKAWAY_MAX_PCT) {
        return EPS_DRIVER_ASSIST_BREAKAWAY_MAX_PCT;
    }
    return configured_pct;
}

static inline float EpsAssist_EffectiveSlewRate(float configured_pct,
                                               bool driver_intent)
{
    if (!isfinite(configured_pct) || configured_pct <= 0.0f) return 0.1f;
    if (driver_intent && configured_pct > EPS_DRIVER_ASSIST_SLEW_MAX_PCT) {
        return EPS_DRIVER_ASSIST_SLEW_MAX_PCT;
    }
    return configured_pct;
}

#endif /* EPS_ASSIST_POLICY_H */
