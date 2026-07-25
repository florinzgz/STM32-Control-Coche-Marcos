#ifndef TRACTION_OUTPUT_POLICY_H
#define TRACTION_OUTPUT_POLICY_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define TRACTION_OUTPUT_WHEEL_COUNT 4U
#define TRACTION_OUTPUT_UNITY_EPSILON 1.0e-6f

/* These values intentionally mirror motor_mode_t without including the
 * hardware-heavy motor_control.h.  motor_control_patched.c asserts the mapping
 * at compile time before applying a plan to real BTS7960 outputs. */
enum {
    TRACTION_OUTPUT_MODE_COAST = 0U,
    TRACTION_OUTPUT_MODE_DRIVE = 1U,
    TRACTION_OUTPUT_MODE_BRAKE = 2U
};

enum {
    TRACTION_OUTPUT_FL = 0U,
    TRACTION_OUTPUT_FR = 1U,
    TRACTION_OUTPUT_RL = 2U,
    TRACTION_OUTPUT_RR = 3U
};

typedef struct {
    uint8_t mode[TRACTION_OUTPUT_WHEEL_COUNT];
    int8_t direction[TRACTION_OUTPUT_WHEEL_COUNT];
    uint16_t pwm[TRACTION_OUTPUT_WHEEL_COUNT];
} TractionOutputPlan;

static inline uint16_t TractionOutput_ClampPwm(uint32_t pwm,
                                               uint16_t pwm_max)
{
    return (pwm > pwm_max) ? pwm_max : (uint16_t)pwm;
}

/* Reduction-only cap.  NaN and non-positive scale fail closed to zero. */
static inline uint16_t TractionOutput_CapPwmByScale(uint16_t pwm,
                                                    float scale,
                                                    uint16_t pwm_max)
{
    pwm = TractionOutput_ClampPwm(pwm, pwm_max);
    if (!(scale > 0.0f)) return 0U;
    if (scale >= 1.0f) return pwm;
    return (uint16_t)((float)pwm * scale);
}

/* Equalisation must never raise a wheel above its resolved safe output. */
static inline uint16_t TractionOutput_MinPwm(
    const uint16_t pwm[TRACTION_OUTPUT_WHEEL_COUNT])
{
    uint16_t result = pwm[0];
    for (uint8_t i = 1U; i < TRACTION_OUTPUT_WHEEL_COUNT; ++i) {
        if (pwm[i] < result) result = pwm[i];
    }
    return result;
}

static inline bool TractionOutput_SameNonzeroDirection(
    const int8_t dir[TRACTION_OUTPUT_WHEEL_COUNT])
{
    if (dir[0] == 0) return false;
    for (uint8_t i = 1U; i < TRACTION_OUTPUT_WHEEL_COUNT; ++i) {
        if (dir[i] != dir[0]) return false;
    }
    return true;
}

/* Guard for straight-line 4x4 equalisation.  Every input must be finite and
 * each wheel scale must still be unity.  NaN/Inf therefore disable symmetry
 * enforcement instead of bypassing a safety limiter through float comparisons. */
static inline bool TractionOutput_StraightUnlimited(
    float steering_angle_deg,
    float deadband_deg,
    const float wheel_scale[TRACTION_OUTPUT_WHEEL_COUNT])
{
    if (!isfinite(steering_angle_deg) || !isfinite(deadband_deg) ||
        !(deadband_deg > 0.0f)) {
        return false;
    }
    if (fabsf(steering_angle_deg) >= deadband_deg) return false;

    for (uint8_t i = 0U; i < TRACTION_OUTPUT_WHEEL_COUNT; ++i) {
        if (!isfinite(wheel_scale[i]) ||
            fabsf(wheel_scale[i] - 1.0f) > TRACTION_OUTPUT_UNITY_EPSILON) {
            return false;
        }
    }
    return true;
}

static inline void TractionOutput_CopyPlan(
    const uint8_t mode[TRACTION_OUTPUT_WHEEL_COUNT],
    const int8_t direction[TRACTION_OUTPUT_WHEEL_COUNT],
    const uint16_t pwm[TRACTION_OUTPUT_WHEEL_COUNT],
    TractionOutputPlan *out)
{
    if (out == NULL) return;
    for (uint8_t i = 0U; i < TRACTION_OUTPUT_WHEEL_COUNT; ++i) {
        out->mode[i] = mode[i];
        out->direction[i] = direction[i];
        out->pwm[i] = pwm[i];
    }
}

/* Resolve the exact physical 4x4 plan consumed by motor_control_patched.c.
 * Outside the safe straight-line equalisation conditions, the shadow decisions
 * pass through unchanged.  Inside them, all wheels are reduced to the minimum
 * resolved PWM and receive one common non-zero direction. */
static inline void TractionOutput_Resolve4x4(
    const uint8_t mode[TRACTION_OUTPUT_WHEEL_COUNT],
    const int8_t direction[TRACTION_OUTPUT_WHEEL_COUNT],
    const uint16_t pwm[TRACTION_OUTPUT_WHEEL_COUNT],
    float steering_angle_deg,
    float deadband_deg,
    const float wheel_scale[TRACTION_OUTPUT_WHEEL_COUNT],
    TractionOutputPlan *out)
{
    TractionOutput_CopyPlan(mode, direction, pwm, out);
    if (out == NULL) return;

    bool all_drive = true;
    for (uint8_t i = 0U; i < TRACTION_OUTPUT_WHEEL_COUNT; ++i) {
        if (mode[i] != TRACTION_OUTPUT_MODE_DRIVE) {
            all_drive = false;
            break;
        }
    }

    if (!all_drive || !TractionOutput_SameNonzeroDirection(direction) ||
        !TractionOutput_StraightUnlimited(steering_angle_deg, deadband_deg,
                                          wheel_scale)) {
        return;
    }

    const uint16_t equal_pwm = TractionOutput_MinPwm(pwm);
    const int8_t equal_direction = direction[TRACTION_OUTPUT_FL] < 0 ? -1 : 1;
    for (uint8_t i = 0U; i < TRACTION_OUTPUT_WHEEL_COUNT; ++i) {
        out->mode[i] = TRACTION_OUTPUT_MODE_DRIVE;
        out->direction[i] = equal_direction;
        out->pwm[i] = equal_pwm;
    }
}

/* Resolve the installed rear-drive 4x2 plan.  FL/FR are logical left/right
 * decisions from the base controller; physical FL/FR always coast and the
 * result is routed to RL/RR with their own final safety scales.  Any unknown or
 * asymmetric mode fails closed to an all-coast plan and returns false. */
static inline bool TractionOutput_Resolve4x2Rear(
    const uint8_t logical_mode[TRACTION_OUTPUT_WHEEL_COUNT],
    const int8_t logical_direction[TRACTION_OUTPUT_WHEEL_COUNT],
    const uint16_t logical_pwm[TRACTION_OUTPUT_WHEEL_COUNT],
    const float wheel_scale[TRACTION_OUTPUT_WHEEL_COUNT],
    uint16_t pwm_max,
    TractionOutputPlan *out)
{
    if (out == NULL) return false;
    for (uint8_t i = 0U; i < TRACTION_OUTPUT_WHEEL_COUNT; ++i) {
        out->mode[i] = TRACTION_OUTPUT_MODE_COAST;
        out->direction[i] = 0;
        out->pwm[i] = 0U;
    }

    const uint8_t left_mode = logical_mode[TRACTION_OUTPUT_FL];
    const uint8_t right_mode = logical_mode[TRACTION_OUTPUT_FR];
    if (left_mode != right_mode) return false;

    if (left_mode == TRACTION_OUTPUT_MODE_COAST) return true;
    if (left_mode == TRACTION_OUTPUT_MODE_BRAKE) {
        out->mode[TRACTION_OUTPUT_RL] = TRACTION_OUTPUT_MODE_BRAKE;
        out->mode[TRACTION_OUTPUT_RR] = TRACTION_OUTPUT_MODE_BRAKE;
        return true;
    }
    if (left_mode != TRACTION_OUTPUT_MODE_DRIVE) return false;

    out->mode[TRACTION_OUTPUT_RL] = TRACTION_OUTPUT_MODE_DRIVE;
    out->mode[TRACTION_OUTPUT_RR] = TRACTION_OUTPUT_MODE_DRIVE;
    out->direction[TRACTION_OUTPUT_RL] = logical_direction[TRACTION_OUTPUT_FL];
    out->direction[TRACTION_OUTPUT_RR] = logical_direction[TRACTION_OUTPUT_FR];
    out->pwm[TRACTION_OUTPUT_RL] = TractionOutput_CapPwmByScale(
        logical_pwm[TRACTION_OUTPUT_FL], wheel_scale[TRACTION_OUTPUT_RL], pwm_max);
    out->pwm[TRACTION_OUTPUT_RR] = TractionOutput_CapPwmByScale(
        logical_pwm[TRACTION_OUTPUT_FR], wheel_scale[TRACTION_OUTPUT_RR], pwm_max);
    return true;
}

#endif /* TRACTION_OUTPUT_POLICY_H */
