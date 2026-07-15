/**
  ****************************************************************************
  * @file    steering_centering_diag_patched.c
  * @brief   Correct local power-ready evidence during steering homing.
  *
  * Global Safety_IsPowerReady() describes the traction relay sequencer and is
  * intentionally false while homing runs in STANDBY.  The steering diagnostic
  * must instead use the locally commanded PC12 rail plus an active sweep state.
  ****************************************************************************
  */

#include "steering_centering.h"
#include "safety_system.h"
#include "main.h"

static bool PR429_SteeringHomingPowerReady(void)
{
    const CenteringState_t state = SteeringCentering_GetState();
    const bool sweeping = (state == CENTERING_SWEEP_LEFT) ||
                          (state == CENTERING_SWEEP_RIGHT);

    return sweeping &&
        (HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR) == GPIO_PIN_SET);
}

#define Safety_IsPowerReady PR429_SteeringHomingPowerReady
#include "steering_centering_diag.c"
#undef Safety_IsPowerReady
