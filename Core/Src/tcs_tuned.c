/**
  ****************************************************************************
  * @file    tcs_tuned.c
  * @brief   Quantisation-tolerant, time-based per-wheel TCS authority.
  *
  * main.c is routed here by build_sanity_checks.h.  The legacy and patched
  * implementations remain compiled for auditability, but this is the productive
  * 100 Hz entry point.  It never rewrites global traction demand and therefore
  * cannot feed the demand-anomaly detector.
  ****************************************************************************
  */

#include "safety_system.h"
#include "sensor_manager.h"
#include "motor_control.h"
#include "service_mode.h"
#include "drive_dynamics_policy.h"
#include <math.h>
#include <stdint.h>

static float s_tcs_reduction[4];
static uint32_t s_tcs_last_tick;

static void TcsTuned_SatInc(uint32_t *counter)
{
    if (counter && *counter < UINT32_MAX) ++(*counter);
}

static float TcsTuned_WheelSpeed(uint8_t wheel)
{
    switch (wheel) {
        case MOTOR_FL: return Wheel_GetSpeed_FL();
        case MOTOR_FR: return Wheel_GetSpeed_FR();
        case MOTOR_RL: return Wheel_GetSpeed_RL();
        case MOTOR_RR: return Wheel_GetSpeed_RR();
        default: return 0.0f;
    }
}

static bool TcsTuned_WheelHealthyDriven(uint8_t wheel)
{
    if (!Traction_IsWheelDriven(wheel)) return false;
    const ModuleID_t mod = (ModuleID_t)(MODULE_WHEEL_SPEED_FL + wheel);
    return ServiceMode_IsEnabled(mod) &&
           ServiceMode_GetFault(mod) == MODULE_FAULT_NONE;
}

static float TcsTuned_RobustReference(const float speed[4], uint8_t candidate)
{
    float values[3];
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < 4U; ++i) {
        if (i == candidate || !TcsTuned_WheelHealthyDriven(i)) continue;
        if (!isfinite(speed[i]) || speed[i] < 0.0f) continue;
        values[count++] = speed[i];
    }

    if (count == 0U) return 0.0f;
    if (count == 1U) return values[0];

    for (uint8_t i = 1U; i < count; ++i) {
        const float key = values[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && values[j] > key) {
            values[j + 1] = values[j];
            --j;
        }
        values[j + 1] = key;
    }

    return (count == 2U) ? values[0] : values[count / 2U];
}

void TCS_Update_Tuned(void)
{
    const uint32_t now = HAL_GetTick();
    float dt = (float)(now - s_tcs_last_tick) / 1000.0f;
    if (!isfinite(dt) || dt <= 0.0f || dt > 1.0f) dt = 0.01f;
    s_tcs_last_tick = now;

    if (!ServiceMode_IsEnabled(MODULE_TCS) ||
        !Safety_IsPowertrainEngaged()) {
        safety_status.tcs_active = false;
        safety_status.tcs_wheel_mask = 0U;
        for (uint8_t i = 0U; i < 4U; ++i) s_tcs_reduction[i] = 0.0f;
        return;
    }

    const float speed[4] = {
        TcsTuned_WheelSpeed(MOTOR_FL), TcsTuned_WheelSpeed(MOTOR_FR),
        TcsTuned_WheelSpeed(MOTOR_RL), TcsTuned_WheelSpeed(MOTOR_RR)
    };

    uint8_t slip_mask = 0U;
    for (uint8_t i = 0U; i < 4U; ++i) {
        if (!TcsTuned_WheelHealthyDriven(i)) {
            s_tcs_reduction[i] = 0.0f;
            continue;
        }

        const float reference = TcsTuned_RobustReference(speed, i);
        const bool slipping = DriveDynamics_TcsIsSlipping(reference, speed[i]);
        if (slipping) slip_mask |= (uint8_t)(1U << i);

        s_tcs_reduction[i] =
            DriveDynamics_TcsReductionNext(s_tcs_reduction[i], slipping, dt);
        const float scale = 1.0f - s_tcs_reduction[i];
        if (scale < safety_status.wheel_scale[i]) {
            safety_status.wheel_scale[i] = scale;
        }
    }

    safety_status.tcs_active = slip_mask != 0U;
    safety_status.tcs_wheel_mask = slip_mask;
    if (slip_mask != 0U) TcsTuned_SatInc(&safety_status.tcs_activation_count);
}
