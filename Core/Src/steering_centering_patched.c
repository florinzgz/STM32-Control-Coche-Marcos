/**
  ****************************************************************************
  * @file    steering_centering_patched.c
  * @brief   Safe steering-homing policy and diagnostics for PR #429.
  *
  * The production centering FSM remains unchanged.  Its state transition and
  * power-readiness calls are wrapped so a failed physical-centre search
  * releases the 12 V steering bridge, enters walking-speed LIMP_HOME, and the
  * homing diagnostic reports the locally controlled PC12 rail rather than the
  * unrelated global traction sequencer.
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

static void PR429_CenteringSetState(SystemState_t requested)
{
    if (requested == SYS_STATE_DEGRADED &&
        Safety_GetError() == SAFETY_ERROR_CENTERING) {
        /* Steering_Neutralize() has already forced PWM=0 and EN low.  Remove
         * the actuator rail as a second independent release so a strapped or
         * faulty BTS7960 cannot electrically brake the wheel. */
        HAL_GPIO_WritePin(GPIOC, PIN_RELAY_STEER_PWR, GPIO_PIN_RESET);
        Safety_SetState(SYS_STATE_LIMP_HOME);
        return;
    }

    Safety_SetState(requested);
}

static void PR429_CenteringSetDegradedLevel(DegradedLevel_t level,
                                            DegradedReason_t reason)
{
    /* LIMP_HOME already has its own 20 % torque / walking-speed policy.  Do
     * not leave a contradictory L1 marker behind after the state redirect. */
    if (Safety_GetState() != SYS_STATE_LIMP_HOME) {
        Safety_SetDegradedLevel(level, reason);
    }
}

#define Safety_IsPowerReady      PR429_SteeringHomingPowerReady
#define Safety_SetState          PR429_CenteringSetState
#define Safety_SetDegradedLevel  PR429_CenteringSetDegradedLevel
#include "steering_centering.c"
#undef Safety_SetDegradedLevel
#undef Safety_SetState
#undef Safety_IsPowerReady
