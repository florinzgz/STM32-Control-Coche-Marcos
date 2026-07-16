/**
  ****************************************************************************
  * @file    steering_eps.h
  * @brief   EPS (electric power steering) local state authority.
  *
  * The vehicle steering is MECHANICAL.  The steering motor only provides
  * ASSISTANCE.  Therefore ANY isolable failure of the assistance must
  * simply disconnect the motor and leave purely mechanical steering — it
  * must NOT degrade the global vehicle state, reduce traction, change the
  * gear, open traction relays or block the pedal.
  *
  * This module is the single source of truth for:
  *   - the functional EPS state (EpsState_t),
  *   - the latched isolation cause (EpsFaultReason_t),
  *   - the steering-motor owner (STEER_OWNER_NONE / CENTERING / EPS).
  *
  * The isolation function Steering_DisableAssistFault() performs the exact
  * ordered, idempotent shutdown of the steering motor without touching the
  * traction chain.  It may be called repeatedly without generating pulses,
  * re-energising PC12, re-enabling PC4, restarting centering, or flapping.
  *
  * Electrical-hazard escalation (SAFE/ERROR) is intentionally NOT performed
  * here: it belongs to the safety subsystem and is only justified when a
  * real electrical danger persists AFTER the motor has been isolated and
  * verified (e.g. steering current does not disappear with PA6=PA7=0,
  * PC4=LOW, PC12=OFF).  See Steering_DeclareElectricalHazard().
  ****************************************************************************
  */

#ifndef STEERING_EPS_H
#define STEERING_EPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Motor-owner enum lives in steering_centering.h (STEER_OWNER_NONE/…). */
#include "steering_centering.h"

/* ---- Functional EPS states -------------------------------------------- */
typedef enum {
    EPS_STATE_STARTING = 0,       /* Boot self-check, motor not yet assisting  */
    EPS_STATE_CALIBRATING,        /* Homing / centering sweep in progress      */
    EPS_STATE_ACTIVE,             /* Motor assisting correctly                 */
    EPS_STATE_MECHANICAL_ONLY,    /* Assist disconnected — pure mechanical      */
    EPS_STATE_ELECTRICAL_HAZARD   /* Non-isolable electrical danger             */
} EpsState_t;

/* ---- Explicit isolation causes ---------------------------------------- *
 * The numeric values are stable so they can be published on CAN and shown
 * on the HMI without ambiguity.                                            */
typedef enum {
    EPS_FAULT_NONE = 0,
    EPS_FAULT_CENTER_NOT_FOUND,       /* PB5 centre never confirmed / timeout  */
    EPS_FAULT_PB5,                    /* PB5 stuck / incoherent                */
    EPS_FAULT_ENCODER_AB,             /* Encoder A/B missing / incoherent      */
    EPS_FAULT_ENCODER_Z,             /* Encoder Z absent / incoherent          */
    EPS_FAULT_POSITION_MISMATCH,      /* PB5 vs Z / impossible position        */
    EPS_FAULT_CALIBRATION_INVALID,    /* Missing / corrupt / bad CRC cal       */
    EPS_FAULT_PARAMETERS_INVALID,     /* NaN/Inf/out-of-range EPS params       */
    EPS_FAULT_CH5_MISSING,            /* INA226 CH5 absent when mandatory      */
    EPS_FAULT_CH5_STALE,             /* INA226 CH5 sample stale                */
    EPS_FAULT_CH5_CONFIG,             /* INA226 CH5 configuration invalid      */
    EPS_FAULT_OVERCURRENT,            /* Steering overcurrent (isolable)       */
    EPS_FAULT_DIRECTION_POLARITY,     /* Assist pushing the wrong way          */
    EPS_FAULT_OSCILLATION,            /* Sustained oscillation                 */
    EPS_FAULT_DRIVER,                 /* Recoverable BTS7960 driver fault      */
    EPS_FAULT_POWER_CONFIRMATION,     /* Steering rail confirmation absent     */
    EPS_FAULT_OWNER_CONFLICT,         /* CENTERING vs EPS owner conflict       */
    EPS_FAULT_UNKNOWN                 /* Catch-all                              */
} EpsFaultReason_t;

/* ==================================================================
 *  Public API
 * ================================================================== */

/**
 * @brief  Initialise the EPS authority.  Call once during boot before the
 *         main loop.  Sets EPS_STATE_STARTING, owner NONE, no fault.
 *         Does NOT touch hardware.
 */
void Steering_EpsInit(void);

/**
 * @brief  Isolate the steering assistance because of an isolable fault.
 *
 *         Idempotent and latched.  Performs, in a safe order:
 *           1. cancel any centering/EPS order,
 *           2. zero PA6 (TIM3_CH1) and PA7 (TIM3_CH2) CCR,
 *           3. force PC4 (EN_STEER) LOW,
 *           4. force PC12 (steering rail) OFF,
 *           5. owner = STEER_OWNER_NONE,
 *           6. EPS state = EPS_STATE_MECHANICAL_ONLY (latched),
 *           7. record the cause (first non-NONE cause is kept).
 *
 *         It NEVER touches the traction demand, gear, traction relays or
 *         the pedal, and NEVER calls Safety_SetState(DEGRADED/LIMP_HOME/
 *         SAFE/ERROR).  Steps 2-4 are delegated to Steering_Neutralize()
 *         and Steering_SteerPowerOff() so the hardware writes live in the
 *         modules that own those pins.
 *
 * @param  reason  Isolation cause (latched on first call).
 */
void Steering_DisableAssistFault(EpsFaultReason_t reason);

/**
 * @brief  Declare a non-isolable electrical hazard.
 *
 *         Only for use AFTER Steering_DisableAssistFault() has been called
 *         and the danger (e.g. steering current) is verified to persist.
 *         Sets EPS_STATE_ELECTRICAL_HAZARD (latched).  The caller (safety
 *         subsystem) remains responsible for the SAFE/ERROR escalation and
 *         general power cut — this module only records the state so the
 *         HMI/telemetry can distinguish it from a plain assist loss.
 *
 * @param  reason  Underlying cause (latched if none yet recorded).
 */
void Steering_DeclareElectricalHazard(EpsFaultReason_t reason);

/**
 * @brief  Report a healthy assist state (STARTING → CALIBRATING → ACTIVE).
 *
 *         Ignored once the EPS has latched MECHANICAL_ONLY or
 *         ELECTRICAL_HAZARD: an isolated fault never auto-recovers within a
 *         power cycle.  Use to advance the state during a clean startup.
 *
 * @param  s  One of EPS_STATE_STARTING / CALIBRATING / ACTIVE.
 */
void Steering_EpsSetHealthyState(EpsState_t s);

/**
 * @brief  Set / query the steering-motor owner (single source of truth).
 *         While MECHANICAL_ONLY/ELECTRICAL_HAZARD is latched the owner is
 *         forced to STEER_OWNER_NONE and set requests are ignored.
 */
void                 Steering_EpsSetOwner(SteeringMotorOwner_t owner);
SteeringMotorOwner_t Steering_EpsGetOwner(void);

/** @brief  Current functional EPS state. */
EpsState_t Steering_GetEpsState(void);

/** @brief  Latched isolation cause (EPS_FAULT_NONE if healthy). */
EpsFaultReason_t Steering_GetEpsFault(void);

/**
 * @brief  true when the assist is disconnected and steering is purely
 *         mechanical (MECHANICAL_ONLY or ELECTRICAL_HAZARD).  The EPS
 *         control loop and the steering power relay must honour this.
 */
bool Steering_IsMechanicalOnly(void);

/** @brief  true when the assist may drive the motor (EPS available). */
bool Steering_EpsIsAvailable(void);

#ifdef __cplusplus
}
#endif

#endif /* STEERING_EPS_H */
