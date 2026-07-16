/**
  ****************************************************************************
  * @file    steering_eps.c
  * @brief   EPS local state authority + idempotent assist isolation.
  *
  * See steering_eps.h for the rationale.  This translation unit is HAL-free
  * and host-testable: the two hardware side effects it needs are delegated
  * to functions implemented by the pin-owning modules:
  *
  *   - Steering_Neutralize()   (motor_control.c) — PA6=0, PA7=0, PC4=LOW.
  *   - Steering_SteerPowerOff() (safety_system.c) — PC12 OFF, traction rail
  *                               and relay sequencer state untouched.
  *
  * Both are declared here as plain externs so no HAL header is pulled in.
  ****************************************************************************
  */

#include "steering_eps.h"

/* Hardware side effects — provided by the pin-owning production modules
 * (or by stubs in host tests).  Kept as externs so this unit stays HAL-free. */
extern void Steering_Neutralize(void);      /* PA6=0, PA7=0, PC4=LOW (coast) */
extern void Steering_SteerPowerOff(void);   /* PC12 OFF, traction untouched  */

/* ---- Module state (single source of truth) ---- */
static EpsState_t          s_state  = EPS_STATE_STARTING;
static EpsFaultReason_t    s_fault  = EPS_FAULT_NONE;
static SteeringMotorOwner_t s_owner = STEER_OWNER_NONE;

/* ---- Helpers ---- */

static bool eps_is_latched(void)
{
    return (s_state == EPS_STATE_MECHANICAL_ONLY) ||
           (s_state == EPS_STATE_ELECTRICAL_HAZARD);
}

/* ==================================================================
 *  Public API
 * ================================================================== */

void Steering_EpsInit(void)
{
    s_state = EPS_STATE_STARTING;
    s_fault = EPS_FAULT_NONE;
    s_owner = STEER_OWNER_NONE;
}

void Steering_DisableAssistFault(EpsFaultReason_t reason)
{
    /* Latch the cause the FIRST time only, so the root cause is preserved
     * across the idempotent repeat calls that follow.                     */
    if (s_fault == EPS_FAULT_NONE && reason != EPS_FAULT_NONE) {
        s_fault = reason;
    }

    /* ---- Ordered, idempotent motor shutdown ----
     * Steps 1-3: cancel orders + zero PA6/PA7 CCR + PC4 LOW.  Writing zeros
     * is safe to repeat — no pulses are generated.                        */
    Steering_Neutralize();

    /* Step 4: remove the steering actuator rail (PC12).  This is a second,
     * independent release so a strapped/faulty BTS7960 cannot electrically
     * brake the wheel.  It does NOT touch the traction relay or the relay
     * sequencer, so Safety_IsPowerReady() (traction) stays valid.          */
    Steering_SteerPowerOff();

    /* Steps 5-6: owner NONE, EPS mechanical-only (latched).  Never demote a
     * previously declared electrical hazard back to a plain assist loss.   */
    s_owner = STEER_OWNER_NONE;
    if (s_state != EPS_STATE_ELECTRICAL_HAZARD) {
        s_state = EPS_STATE_MECHANICAL_ONLY;
    }
}

void Steering_DeclareElectricalHazard(EpsFaultReason_t reason)
{
    /* Isolate first (idempotent), then escalate the local state so the HMI
     * and telemetry can distinguish a genuine electrical danger from a
     * plain, benign loss of assistance.                                    */
    Steering_DisableAssistFault(reason);
    s_state = EPS_STATE_ELECTRICAL_HAZARD;
}

void Steering_EpsSetHealthyState(EpsState_t s)
{
    /* A latched fault never auto-recovers within the same power cycle. */
    if (eps_is_latched()) {
        return;
    }
    if (s == EPS_STATE_STARTING ||
        s == EPS_STATE_CALIBRATING ||
        s == EPS_STATE_ACTIVE) {
        s_state = s;
    }
}

void Steering_EpsSetOwner(SteeringMotorOwner_t owner)
{
    if (eps_is_latched()) {
        s_owner = STEER_OWNER_NONE;   /* Forced — no owner may drive the motor */
        return;
    }
    s_owner = owner;
}

SteeringMotorOwner_t Steering_EpsGetOwner(void)
{
    return eps_is_latched() ? STEER_OWNER_NONE : s_owner;
}

EpsState_t Steering_GetEpsState(void)
{
    return s_state;
}

EpsFaultReason_t Steering_GetEpsFault(void)
{
    return s_fault;
}

bool Steering_IsMechanicalOnly(void)
{
    return eps_is_latched();
}

bool Steering_EpsIsAvailable(void)
{
    return !eps_is_latched();
}
