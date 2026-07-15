/**
  ****************************************************************************
  * @file    sensor_manager_patched.c
  * @brief   Demand-aware CH5 INA226 classification wrapper.
  *
  * The diagnostic snapshot is still acquired by the production
  * sensor_manager.c.  Only the pure classifier call is decorated with the
  * final TIM3 steering PWM state, so a powered-but-idle channel correctly
  * reports 0 A as OK while a real drive command can still diagnose a missing
  * shunt or reversed VIN polarity.
  ****************************************************************************
  */

#include "sensor_manager.h"
#include "ina226_channel_diag.h"
#include "main.h"
#include <stddef.h>

extern TIM_HandleTypeDef htim3;

static Ina226DiagReason_t PR429_ClassifyInaWithDemand(
    const Ina226ChannelDiag *snapshot)
{
    if (snapshot == NULL) {
        return INA226_CH_UNKNOWN;
    }

    Ina226ChannelDiag qualified = *snapshot;
    qualified.current_expected = qualified.channel_powered &&
        ((__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1) > 0U) ||
         (__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2) > 0U));

    return Ina226_ClassifyChannel(&qualified);
}

#define Ina226_ClassifyChannel PR429_ClassifyInaWithDemand
#include "sensor_manager.c"
#undef Ina226_ClassifyChannel
