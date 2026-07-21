#ifndef BATTERY_SERVICE_POLICY_H
#define BATTERY_SERVICE_POLICY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>

#define BATT_SERVICE_POLICY_WHEEL_COUNT 4U

typedef struct {
    bool in_standby;
    bool in_active;
    bool in_degraded;
    bool degraded_is_battery_uv_warning;
    bool gear_is_park_or_neutral;
    float pedal_pct;
    uint8_t final_pwm_pct;
    uint16_t active_brake_pwm_ticks;
    float wheel_speed_kmh[BATT_SERVICE_POLICY_WHEEL_COUNT];
} BatteryServiceWriteInputs;

/* HAL-free policy used by the flash store and its host tests.
 * STANDBY remains the established unrestricted configuration state.
 * Outside STANDBY, every stationary-service condition must be proven. */
static inline bool BatteryServiceWrite_Evaluate(
    const BatteryServiceWriteInputs *in,
    float pedal_max_pct,
    float speed_max_kmh)
{
    if (in == NULL) return false;

    if (in->in_standby) return true;

    if (!in->in_active && !in->in_degraded) return false;
    if (in->in_degraded && !in->degraded_is_battery_uv_warning) return false;
    if (!in->gear_is_park_or_neutral) return false;

    if (!isfinite(in->pedal_pct) || in->pedal_pct >= pedal_max_pct) {
        return false;
    }

    if (in->final_pwm_pct != 0U || in->active_brake_pwm_ticks != 0U) {
        return false;
    }

    for (uint8_t i = 0U; i < BATT_SERVICE_POLICY_WHEEL_COUNT; ++i) {
        const float speed = in->wheel_speed_kmh[i];
        if (!isfinite(speed) || fabsf(speed) >= speed_max_kmh) {
            return false;
        }
    }

    return true;
}

#endif /* BATTERY_SERVICE_POLICY_H */
