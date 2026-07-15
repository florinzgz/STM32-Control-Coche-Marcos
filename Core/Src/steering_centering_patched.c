/**
  ****************************************************************************
  * @file    steering_centering_patched.c
  * @brief   Safe steering-homing failure policy for PR #429.
  *
  * The production centering FSM remains unchanged.  Its state transition calls
  * are wrapped so a failed physical-centre search releases the 12 V steering
  * bridge and enters the defined walking-speed LIMP_HOME mode instead of a
  * generic DEGRADED state with ambiguous steering power ownership.
  ****************************************************************************
  */

#include "steering_centering.h"
#include "safety_system.h"
#include "main.h"

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

#define Safety_SetState          PR429_CenteringSetState
#define Safety_SetDegradedLevel  PR429_CenteringSetDegradedLevel
#include "steering_centering.c"
#undef Safety_SetDegradedLevel
#undef Safety_SetState
