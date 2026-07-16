/**
  ****************************************************************************
  * @file    steering_centering_patched.c
  * @brief   Steering-homing power-readiness reporting wrapper.
  *
  * The production centering FSM (steering_centering.c) is included verbatim.
  * Only the homing diagnostic's notion of "power ready" is wrapped so it
  * reports the LOCALLY controlled 12 V steering rail (PC12) during the sweep
  * rather than the unrelated global traction sequencer.
  *
  * NOTE (EPS isolation phase): a failed physical-centre search is now an
  * ISOLABLE ASSIST fault.  Centering_Abort() disconnects the steering motor
  * via Steering_DisableAssistFault() (PA6=PA7=0, PC4=LOW, PC12=OFF,
  * owner=NONE, EPS mechanical-only) and the vehicle stays ACTIVE with full
  * traction.  It no longer calls Safety_SetState()/Safety_SetDegradedLevel(),
  * so the former DEGRADED->LIMP_HOME redirect that lived here has been removed.
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

    /* WAIT_RAIL intentionally remains not-ready.  Once the FSM enters an
     * active sweep, PC12 being high is the real local power evidence. */
    return sweeping &&
        (HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR) == GPIO_PIN_SET);
}

#define Safety_IsPowerReady      PR429_SteeringHomingPowerReady
#include "steering_centering.c"
#undef Safety_IsPowerReady
