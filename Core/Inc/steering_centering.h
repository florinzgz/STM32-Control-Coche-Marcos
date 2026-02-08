/**
  ****************************************************************************
  * @file    steering_centering.h
  * @brief   Automatic steering centering without mechanical endstops
  *
  * At startup the steering position is unknown.  This module slowly
  * sweeps the steering rack left and right at LOW FORCE (limited PWM)
  * until the inductive center sensor (LJ12A3 detecting a screw at the
  * mechanical center) generates a pulse.  When the pulse is detected
  * the motor is stopped, the encoder counter is zeroed, and the
  * steering is marked as calibrated.
  *
  * Safety constraints:
  *   - Runs ONLY in BOOT or STANDBY state.
  *   - Motor drive is limited to CENTERING_PWM (≈10 % of full scale).
  *   - End-of-travel is inferred from encoder stall (no mechanical
  *     endstop assumption).
  *   - Any fault aborts centering, neutralises the motor, and latches
  *     SAFETY_ERROR_CENTERING to prevent transition to ACTIVE.
  *   - The encoder Z-index is NOT used.
  ****************************************************************************
  */

#ifndef __STEERING_CENTERING_H
#define __STEERING_CENTERING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---- Centering state machine ---- */
typedef enum {
    CENTERING_IDLE = 0,       /* Not started yet                        */
    CENTERING_SWEEP_LEFT,     /* Moving left at low PWM                 */
    CENTERING_SWEEP_RIGHT,    /* Moving right at low PWM                */
    CENTERING_DONE,           /* Center found, calibrated               */
    CENTERING_FAULT           /* Failed — motor neutralised, fault set  */
} CenteringState_t;

/* ---- Public API ---- */

/**
 * @brief  Initialise centering state.
 *         Must be called once during module init (before main loop).
 */
void SteeringCentering_Init(void);

/**
 * @brief  Non-blocking centering step — call at 100 Hz from the main
 *         loop while the system is in BOOT or STANDBY state.
 *
 *         Internally runs the sweep state machine, monitors the center
 *         inductive sensor, detects end-of-travel stalls, and handles
 *         timeout / encoder faults.
 */
void SteeringCentering_Step(void);

/**
 * @brief  Returns true once centering has completed successfully.
 */
bool SteeringCentering_IsComplete(void);

/**
 * @brief  Returns true if centering encountered a fault.
 */
bool SteeringCentering_HasFault(void);

/**
 * @brief  Returns the current centering state (for CAN diagnostics).
 */
CenteringState_t SteeringCentering_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* __STEERING_CENTERING_H */
