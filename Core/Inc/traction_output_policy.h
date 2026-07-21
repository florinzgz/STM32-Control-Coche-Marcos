#ifndef TRACTION_OUTPUT_POLICY_H
#define TRACTION_OUTPUT_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#define TRACTION_OUTPUT_WHEEL_COUNT 4U

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
static inline uint16_t TractionOutput_MinPwm(const uint16_t pwm[4])
{
    uint16_t result = pwm[0];
    for (uint8_t i = 1U; i < TRACTION_OUTPUT_WHEEL_COUNT; ++i) {
        if (pwm[i] < result) result = pwm[i];
    }
    return result;
}

static inline bool TractionOutput_SameNonzeroDirection(const int8_t dir[4])
{
    if (dir[0] == 0) return false;
    for (uint8_t i = 1U; i < TRACTION_OUTPUT_WHEEL_COUNT; ++i) {
        if (dir[i] != dir[0]) return false;
    }
    return true;
}

#endif /* TRACTION_OUTPUT_POLICY_H */
