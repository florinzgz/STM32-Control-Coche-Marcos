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
  *   - The encoder Z-index (PB4) is NOT used for centering; it provides
 *     inter-revolution drift detection via encoder_reader.c.
  ****************************************************************************
  */

#ifndef STEERING_CENTERING_H
#define STEERING_CENTERING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---- Centering state machine ----
 *
 * Note: CENTERING_WAIT_RAIL is appended at the end of the enum on purpose
 * so the CAN-visible numeric values of the pre-existing states (IDLE=0,
 * SWEEP_LEFT=1, SWEEP_RIGHT=2, DONE=3, FAULT=4) remain stable.            */
typedef enum {
    CENTERING_IDLE = 0,       /* Not started yet                        */
    CENTERING_SWEEP_LEFT,     /* Moving left at low PWM                 */
    CENTERING_SWEEP_RIGHT,    /* Moving right at low PWM                */
    CENTERING_DONE,           /* Center found, calibrated               */
    CENTERING_FAULT,          /* Failed — motor neutralised, fault set  */
    CENTERING_WAIT_RAIL       /* DIR relay energised, waiting settle    */
} CenteringState_t;

/* ---- Steering motor ownership ----
 *
 * Exactly one subsystem may write the steering motor (PC4 EN_STEER,
 * PA6 TIM3_CH1, PA7 TIM3_CH2) in a given control cycle:
 *
 *   - STEER_OWNER_CENTERING: the homing FSM (SteeringCentering_Step)
 *     is the sole writer while it actively drives the sweep.
 *   - STEER_OWNER_EPS: the EPS torque-assist loop (Steering_ControlLoop)
 *     owns the motor once homing has released it, and is responsible for
 *     neutralising it while uncalibrated or in SAFE/ERROR.
 *
 * The two never run in the same cycle — this prevents the centering PWM
 * from being neutralised by the EPS loop before it can take effect.
 *
 *   - STEER_OWNER_NONE: nobody drives the motor.  Reported once the EPS
 *     assist has been isolated (EPS_STATE_MECHANICAL_ONLY): PA6=PA7=0,
 *     PC4=LOW, PC12=OFF and steering is purely mechanical.
 *
 * The numeric values are stable (CENTERING=0, EPS=1 kept as published on
 * CAN; NONE appended as 2) so existing telemetry consumers are unaffected. */
typedef enum {
    STEER_OWNER_CENTERING = 0,  /* SteeringCentering_Step drives the motor */
    STEER_OWNER_EPS,            /* Steering_ControlLoop drives the motor   */
    STEER_OWNER_NONE            /* Assist isolated — motor unowned/mech.   */
} SteeringMotorOwner_t;

/* ---- Homing PWM authority constants ----
 *
 * Moved here from steering_centering_patched.c (previously file-local) so
 * OTHER modules can reference the true, single-source-of-truth safety floor
 * at compile time instead of duplicating it as a magic number.
 *
 * Bloque A / V2 audit finding (PR #445): steering_service_store's
 * STEER_SVC_SEARCH_PWM_MAX was safe only "by construction" (1200 < 2336)
 * because nothing tied the two constants together or verified it at build
 * time.  steering_service_store.h now carries a _Static_assert against
 * HOMING_PWM_REDUCED_COUNTS so any future change that would break the
 * invariant fails the build instead of silently drifting.
 *
 * HOMING_PWM_REDUCED_COUNTS is the SAFETY FLOOR: full 100 % (HOMING_PWM_COUNTS)
 * authority is only ever applied while the CH5 current guard is confirmed
 * armed and healthy; whenever that guard is not trustworthy, homing falls
 * back to HOMING_PWM_REDUCED_PERCENT (55 %) of full scale.  Anything that
 * must stay below "reduced-authority homing" (e.g. the service-diagnostic
 * steering search PWM) must never exceed HOMING_PWM_REDUCED_COUNTS.        */
#define HOMING_PWM_COUNTS                4249U   /* TIM3 ARR (100 % authority) */
#define HOMING_PWM_REDUCED_PERCENT       55U      /* fallback when CH5 guard is not armed */
#define HOMING_PWM_REDUCED_COUNTS \
    ((uint16_t)(((uint32_t)HOMING_PWM_COUNTS * HOMING_PWM_REDUCED_PERCENT) / 100U))

/* Guarded for __cplusplus: this header is pulled in transitively by
 * steering_centering_frame.h -> steering_centering_diag.h (frame headers
 * are consumed as raw C++ translation units by ESP32 host tests, e.g.
 * test_frame_parity_cross.cpp — see service_diag_frame.h's file banner for
 * the documented convention). C11 `_Static_assert` is not valid C++
 * (static_assert is), so it must not be evaluated when __cplusplus is
 * defined; the ARM/C11 host-test builds still get the compile-time check. */
#ifndef __cplusplus
_Static_assert(HOMING_PWM_COUNTS == 4249U,
               "homing full authority must match TIM3 period");
#endif

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

/**
 * @brief  Decide which subsystem owns the steering motor this cycle AND
 *         commit that decision to the single owner authority.
 *
 *         The mutual-exclusion policy is: centering owns the motor only
 *         while it is actively homing (IDLE / WAIT_RAIL / SWEEP_LEFT /
 *         SWEEP_RIGHT) AND the system is still in a homing-capable state
 *         (BOOT or STANDBY).  In every other case — CENTERING_DONE,
 *         CENTERING_FAULT, or a system state such as SAFE/ERROR — the EPS
 *         control loop owns the motor (and neutralises it when required).
 *         A latched EPS isolation (MECHANICAL_ONLY / ELECTRICAL_HAZARD)
 *         overrides everything and yields STEER_OWNER_NONE.
 *
 *         SINGLE AUTHORITY: the decision is written to the EPS authority
 *         (steering_eps.c::s_owner) and the value it actually holds is
 *         returned, so the main loop, the 0x316 diagnostic and the CAN
 *         telemetry all observe one identical owner.  There is therefore no
 *         second, independent owner variable to drift out of sync.
 *
 * @param  state           Current centering FSM state.
 * @param  in_homing_state true if the system state permits homing
 *                         (SYS_STATE_BOOT or SYS_STATE_STANDBY).
 * @retval STEER_OWNER_CENTERING, STEER_OWNER_EPS or STEER_OWNER_NONE.
 */
SteeringMotorOwner_t SteeringCentering_DecideOwner(CenteringState_t state,
                                                   bool in_homing_state);


/**
 * @brief  Mark centering as complete using a stored flash calibration.
 *
 *         Called at boot when SteeringCal_ValidateAtBoot() passes.
 *         Sets the TIM2 counter to the stored center value and marks
 *         steering as calibrated, skipping the physical sweep.
 *
 *         Safety: this function is only called AFTER the center sensor
 *         has confirmed physical plausibility.  Flash alone never
 *         authorises this transition.
 *
 * @param  stored_center  The encoder count at center, read from flash.
 */
void SteeringCentering_MarkRestoredFromFlash(int32_t stored_center);

/* ---- Homing diagnostics (Problem 1 integration) ----
 *
 * SteeringCentering_UpdateDiag() builds a full SteeringCenteringDiag
 * telemetry snapshot from the LIVE hardware/FSM state, runs the pure
 * classifier (SteeringCentering_ClassifyDiag) on it, and caches the result.
 * It is pure instrumentation: it reads GPIO/timer/encoder state but never
 * drives the motor, relay, PWM or FSM.  Call it once per control cycle
 * (100 Hz) AFTER SteeringCentering_Step()/Steering_ControlLoop() so the
 * captured PWM/CCR reflect what was actually emitted this cycle.
 *
 * The pointer returned by SteeringCentering_GetDiag() is stable and always
 * valid; the struct is updated in place by SteeringCentering_UpdateDiag().
 */
struct SteeringCenteringDiag;   /* full definition in steering_centering_diag.h */

void SteeringCentering_UpdateDiag(void);
const struct SteeringCenteringDiag *SteeringCentering_GetDiag(void);

#ifdef __cplusplus
}
#endif

#endif /* STEERING_CENTERING_H */
