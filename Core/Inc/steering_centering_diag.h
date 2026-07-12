/**
  ****************************************************************************
  * @file    steering_centering_diag.h
  * @brief   Explicit steering-centering (homing) diagnostic classifier.
  *
  * Problem 1 of the audit: "the steering does not perform the automatic
  * centering sweep".  The firmware previously collapsed every homing
  * failure into a single opaque "Error 8" (SAFETY_ERROR_CENTERING), which
  * hides the real cause (no power, relay off, lost ownership, encoder not
  * moving, sensor already active, timeout, ...).
  *
  * This module captures a full telemetry snapshot of one homing cycle
  * (SteeringCenteringDiag) and, from that snapshot ALONE, derives a stable,
  * human-readable reason (SteerDiagReason_t).  The classifier is a PURE
  * function with no hardware access, so the whole decision matrix can be
  * unit-tested on the host (test_steering_centering_diag.c).
  *
  * IMPORTANT — this is instrumentation, not a behaviour change:
  *   - It does NOT drive the motor, relay or PWM.
  *   - It does NOT change CENTERING_PWM / STALL_TIMEOUT_MS /
  *     TOTAL_TIMEOUT_MS / MAX_CENTERING_COUNTS.
  *   - It only explains WHY the sweep did or did not progress so the cause
  *     can be shown on the HMI and confirmed physically before any pin,
  *     threshold or policy is touched.
  ****************************************************************************
  */

#ifndef STEERING_CENTERING_DIAG_H
#define STEERING_CENTERING_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "steering_centering.h"   /* CenteringState_t, SteeringMotorOwner_t */

/* ---- Thresholds mirrored from steering_centering.c ----
 *
 * These MUST stay in sync with the private #defines in
 * steering_centering.c.  They are duplicated here (not shared) on purpose
 * so the pure classifier has no dependency on the hardware module.  The
 * homing constants are frozen by the audit ("Mantener CENTERING_PWM=425,
 * STALL_TIMEOUT_MS=300, TOTAL_TIMEOUT_MS=10000 y MAX_CENTERING_COUNTS=6000
 * inicialmente"), so they are not expected to drift.                     */
#define STEER_DIAG_STALL_TIMEOUT_MS      300U
#define STEER_DIAG_TOTAL_TIMEOUT_MS      10000U
#define STEER_DIAG_MAX_CENTERING_COUNTS  6000

/* ---- System-state numeric codes ----
 *
 * Mirror of SystemState_t (safety_system.h) kept as plain values so the
 * classifier does not have to include the safety subsystem (which would
 * pull in HAL types).  Only the ordering/values matter and they are frozen
 * by the CAN contract.                                                    */
#define STEER_DIAG_SS_BOOT       0U
#define STEER_DIAG_SS_STANDBY    1U
#define STEER_DIAG_SS_ACTIVE     2U
#define STEER_DIAG_SS_DEGRADED   3U
#define STEER_DIAG_SS_SAFE       4U
#define STEER_DIAG_SS_ERROR      5U
#define STEER_DIAG_SS_LIMP_HOME  6U

/* ---- Stable, human-readable diagnostic reasons ----
 *
 * Numeric values are stable so they can be transported over CAN without
 * ambiguity.  Never reuse or reorder — append only.                       */
typedef enum {
    STEER_DIAG_OK = 0,                        /* Centering completed OK        */
    STEER_DIAG_RESTORED_FROM_FLASH,           /* Skipped sweep, flash restore  */
    STEER_DIAG_WAITING_POWER,                 /* Rail energising / not ready   */
    STEER_DIAG_CENTER_SENSOR_ALREADY_ACTIVE,  /* PB5 asserted before sweep     */
    STEER_DIAG_SWEEP_LEFT,                    /* Nominal — sweeping left        */
    STEER_DIAG_SWEEP_RIGHT,                   /* Nominal — sweeping right       */
    STEER_DIAG_NO_ENCODER_MOVEMENT,           /* PWM requested, encoder frozen  */
    STEER_DIAG_ENCODER_FAULT,                 /* Encoder health fault           */
    STEER_DIAG_RANGE_EXCEEDED,                /* Travel guard exceeded          */
    STEER_DIAG_TOTAL_TIMEOUT,                 /* Absolute deadline exceeded     */
    STEER_DIAG_LOST_HOMING_STATE,             /* Left BOOT/STANDBY or ownership */
    STEER_DIAG_RELAY_NOT_READY,               /* PC12 steering relay not on     */
    STEER_DIAG_MODULE_DISABLED,               /* Disabled in Service Mode       */
    STEER_DIAG_ABORTED_SAFE,                  /* System entered SAFE            */
    STEER_DIAG_ABORTED_ERROR,                 /* System entered ERROR           */
    STEER_DIAG_UNKNOWN                        /* Cause not classifiable         */
} SteerDiagReason_t;

/* ---- Full homing telemetry snapshot ----
 *
 * One instance describes a single control cycle of the homing FSM.  Every
 * field is captured from real hardware / module state by the caller; the
 * classifier reads them but never writes hardware.                        */
typedef struct {
    CenteringState_t     fsm_state;             /* SteeringCentering_GetState  */
    SteeringMotorOwner_t motor_owner;           /* DecideOwner result          */
    uint8_t              system_state;          /* STEER_DIAG_SS_*             */

    bool     center_sensor_raw;                 /* PB5 raw level               */
    bool     center_sensor_debounced;           /* PB5 debounced level         */
    bool     center_sensor_already_active;      /* PB5 active when sweep began */

    int32_t  encoder_count;                     /* TIM2 counter now            */
    int32_t  encoder_delta;                     /* count - sweep_origin        */

    bool     relay_steer_commanded;             /* PC12 EN commanded ON        */
    bool     power_ready;                        /* Safety_IsPowerReady()       */
    bool     enable_commanded;                  /* PC4 EN_STEER commanded      */

    uint16_t pwm_requested;                     /* CENTERING_PWM requested     */
    uint16_t pwm_applied_ch1;                   /* TIM3 CH1 CCR actually set   */
    uint16_t pwm_applied_ch2;                   /* TIM3 CH2 CCR actually set   */

    uint32_t elapsed_ms;                        /* ms since homing start       */
    uint32_t last_encoder_change_ms;            /* ms since encoder last moved */

    bool     encoder_fault;                     /* Encoder_HasFault()          */
    bool     restored_from_flash;               /* Marked done via flash       */
    bool     module_disabled;                   /* Disabled in Service Mode    */
    bool     fault_latched;                     /* SAFETY_ERROR_CENTERING set  */

    SteerDiagReason_t abort_reason;             /* Filled by classifier        */
} SteeringCenteringDiag;

/**
 * @brief  Classify a homing telemetry snapshot into an explicit reason.
 *
 *         Pure function (no side effects, no hardware access) so the full
 *         decision matrix is host-testable.  Does NOT read or depend on
 *         @p d->abort_reason (that field is the classifier's OUTPUT).
 *
 * @param  d  Telemetry snapshot for one homing cycle.
 * @retval The explicit diagnostic reason.
 */
SteerDiagReason_t SteeringCentering_ClassifyDiag(const SteeringCenteringDiag *d);

/**
 * @brief  Short, stable, ASCII label for a diagnostic reason.
 *         Never returns NULL.  Safe for snprintf into fixed HMI buffers.
 */
const char *SteeringCentering_DiagReasonStr(SteerDiagReason_t reason);

#ifdef __cplusplus
}
#endif

#endif /* STEERING_CENTERING_DIAG_H */
