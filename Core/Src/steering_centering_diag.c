/**
  ****************************************************************************
  * @file    steering_centering_diag.c
  * @brief   Pure classifier for steering-centering (homing) diagnostics.
  *
  * See steering_centering_diag.h for the design rationale.  The single
  * decision function turns a full telemetry snapshot into one explicit
  * reason, in strict priority order, so the HMI can show the real cause
  * instead of a generic "Error 8".
  ****************************************************************************
  */

#include "steering_centering_diag.h"

#include <stddef.h>

/* Absolute value without pulling <stdlib.h> abs (keeps this TU freestanding). */
static int32_t diag_abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static bool diag_homing_capable(uint8_t ss)
{
    return (ss == STEER_DIAG_SS_BOOT) || (ss == STEER_DIAG_SS_STANDBY);
}

SteerDiagReason_t SteeringCentering_ClassifyDiag(const SteeringCenteringDiag *d)
{
    if (d == NULL) {
        return STEER_DIAG_UNKNOWN;
    }

    /* --- Highest priority: hard aborts that override everything ---
     *
     * A latched encoder fault, or a system that has fallen into SAFE/ERROR,
     * is reported first because homing MUST NOT proceed and the physical
     * check is different from a mere "not started yet".                     */
    if (d->encoder_fault) {
        return STEER_DIAG_ENCODER_FAULT;
    }
    if (d->system_state == STEER_DIAG_SS_ERROR) {
        return STEER_DIAG_ABORTED_ERROR;
    }
    if (d->system_state == STEER_DIAG_SS_SAFE) {
        return STEER_DIAG_ABORTED_SAFE;
    }
    if (d->module_disabled) {
        return STEER_DIAG_MODULE_DISABLED;
    }

    /* --- Terminal FSM states --- */
    switch (d->fsm_state) {
    case CENTERING_DONE:
        return d->restored_from_flash ? STEER_DIAG_RESTORED_FROM_FLASH
                                      : STEER_DIAG_OK;

    case CENTERING_FAULT:
        /* Explain WHY it faulted from the numeric telemetry, in the same
         * priority the FSM itself evaluates the guards (range, then total
         * timeout, then stall / no-movement).                              */
        if (diag_abs32(d->encoder_delta) > STEER_DIAG_MAX_CENTERING_COUNTS) {
            return STEER_DIAG_RANGE_EXCEEDED;
        }
        if (d->elapsed_ms >= STEER_DIAG_TOTAL_TIMEOUT_MS) {
            return STEER_DIAG_TOTAL_TIMEOUT;
        }
        if (d->last_encoder_change_ms >= STEER_DIAG_STALL_TIMEOUT_MS) {
            return STEER_DIAG_NO_ENCODER_MOVEMENT;
        }
        return STEER_DIAG_UNKNOWN;

    default:
        break;
    }

    /* --- Active homing states: IDLE / WAIT_RAIL / SWEEP_LEFT / SWEEP_RIGHT ---
     *
     * If we are supposedly homing but the system has left BOOT/STANDBY, or
     * the motor ownership is no longer CENTERING, the sweep has silently
     * lost its authorisation — this is the "CENTRADO NO INICIADO / SISTEMA
     * SALIÓ DE STANDBY" case from the audit.                                */
    if (!diag_homing_capable(d->system_state) ||
        (d->motor_owner != STEER_OWNER_CENTERING)) {
        return STEER_DIAG_LOST_HOMING_STATE;
    }

    /* Power / relay readiness must come before movement checks: no encoder
     * movement is EXPECTED (not a fault) while the rail is still off.       */
    if (!d->relay_steer_commanded) {
        return STEER_DIAG_RELAY_NOT_READY;
    }
    if (!d->power_ready) {
        return STEER_DIAG_WAITING_POWER;
    }
    if ((d->fsm_state == CENTERING_IDLE) ||
        (d->fsm_state == CENTERING_WAIT_RAIL)) {
        return STEER_DIAG_WAITING_POWER;
    }

    /* Now genuinely sweeping. If PB5 was already asserted when the sweep
     * began, the physical centre reference is suspicious (metal permanently
     * in front of the inductive sensor) — surface it explicitly.           */
    if (d->center_sensor_already_active) {
        return STEER_DIAG_CENTER_SENSOR_ALREADY_ACTIVE;
    }

    /* Sweeping pathologies, same priority as the FSM guards. */
    if (diag_abs32(d->encoder_delta) > STEER_DIAG_MAX_CENTERING_COUNTS) {
        return STEER_DIAG_RANGE_EXCEEDED;
    }
    if (d->elapsed_ms >= STEER_DIAG_TOTAL_TIMEOUT_MS) {
        return STEER_DIAG_TOTAL_TIMEOUT;
    }
    /* PWM requested but the encoder has not moved for the stall window:
     * this is the key "DIRECCIÓN NO SE MUEVE / SIN MOVIMIENTO DEL ENCODER"
     * signature that points at the BTS7960 / EN / PWM / 12 V rail.         */
    if ((d->pwm_requested > 0U) &&
        (d->last_encoder_change_ms >= STEER_DIAG_STALL_TIMEOUT_MS)) {
        return STEER_DIAG_NO_ENCODER_MOVEMENT;
    }

    /* Nominal in-progress sweep. */
    if (d->fsm_state == CENTERING_SWEEP_LEFT) {
        return STEER_DIAG_SWEEP_LEFT;
    }
    if (d->fsm_state == CENTERING_SWEEP_RIGHT) {
        return STEER_DIAG_SWEEP_RIGHT;
    }

    return STEER_DIAG_UNKNOWN;
}

const char *SteeringCentering_DiagReasonStr(SteerDiagReason_t reason)
{
    switch (reason) {
    case STEER_DIAG_OK:                         return "OK";
    case STEER_DIAG_RESTORED_FROM_FLASH:        return "RESTORED FROM FLASH";
    case STEER_DIAG_WAITING_POWER:              return "WAITING POWER";
    case STEER_DIAG_CENTER_SENSOR_ALREADY_ACTIVE: return "CENTER SENSOR ACTIVE";
    case STEER_DIAG_SWEEP_LEFT:                 return "SWEEP LEFT";
    case STEER_DIAG_SWEEP_RIGHT:                return "SWEEP RIGHT";
    case STEER_DIAG_NO_ENCODER_MOVEMENT:        return "NO ENCODER MOVEMENT";
    case STEER_DIAG_ENCODER_FAULT:              return "ENCODER FAULT";
    case STEER_DIAG_RANGE_EXCEEDED:             return "RANGE EXCEEDED";
    case STEER_DIAG_TOTAL_TIMEOUT:              return "TOTAL TIMEOUT";
    case STEER_DIAG_LOST_HOMING_STATE:          return "LOST HOMING STATE";
    case STEER_DIAG_RELAY_NOT_READY:            return "RELAY NOT READY";
    case STEER_DIAG_MODULE_DISABLED:            return "MODULE DISABLED";
    case STEER_DIAG_ABORTED_SAFE:               return "ABORTED SAFE";
    case STEER_DIAG_ABORTED_ERROR:              return "ABORTED ERROR";
    case STEER_DIAG_UNKNOWN:
    default:                                    return "UNKNOWN";
    }
}
