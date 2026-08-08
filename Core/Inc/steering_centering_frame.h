/**
  ****************************************************************************
  * @file    steering_centering_frame.h
  * @brief   Pure pack/unpack for the 0x316 steering-homing diagnostic frame.
  *
  * Problem 1 of the audit: the steering-centering telemetry snapshot
  * (SteeringCenteringDiag) has to travel over CAN so the ESP32 can render
  * the real cause of a stuck homing sweep ("DIRECCIÓN NO SE MUEVE").
  *
  * The byte layout is defined ONCE here as header-only, side-effect-free
  * inline helpers so the STM32 sender (can_handler.c), the ESP32 receiver
  * and the host integration test all share exactly the same wire format and
  * it can be round-tripped on the host.  Mirrors the repo convention of pure
  * header helpers (motion_inhibit_view.h, wheel_traction.h).
  *
  * Wire layout (DLC 8), matching CAN_ID_DIAG_STEERING_CENTERING (0x316):
  *   Byte 0   diag reason (SteerDiagReason_t, 0..15)
  *   Byte 1   FSM state (low nibble) | motor owner (high nibble)
  *   Byte 2   flags: b0 PB5 raw, b1 PB5 debounced, b2 PB5 already-active,
  *            b3 PC12 relay commanded, b4 power ready, b5 PC4 EN commanded,
  *            b6 encoder fault, b7 restored-from-flash
  *   Byte 3   system state (low nibble) | b4 module disabled |
  *            b5 fault latched | b6 pwm requested (>0) | b7 current guard
  *            armed (CH5 end-stop guard able to observe pressure — Bloque 2
  *            audit fix; see steering_centering_diag.h current_guard_armed)
  *   Byte 4-5 PWM real (max CCR PA6/PA7, uint16 LE)
  *   Byte 6-7 encoder delta from sweep origin (int16 LE, clamped)
  ****************************************************************************
  */

#ifndef STEERING_CENTERING_FRAME_H
#define STEERING_CENTERING_FRAME_H

#include <stdbool.h>
#include <stdint.h>

#include "steering_centering_diag.h"   /* SteeringCenteringDiag */

/* ---- Decoded view of a 0x316 frame (receiver side) ---- */
typedef struct {
    uint8_t  reason;                /* SteerDiagReason_t                 */
    uint8_t  fsm_state;             /* CenteringState_t                  */
    uint8_t  motor_owner;           /* SteeringMotorOwner_t              */
    uint8_t  system_state;          /* STEER_DIAG_SS_*                   */

    bool     center_raw;            /* PB5 raw active                    */
    bool     center_debounced;      /* PB5 debounced active              */
    bool     center_already_active; /* PB5 active when sweep began       */
    bool     relay_commanded;       /* PC12 steering-rail relay ON       */
    bool     power_ready;           /* Safety_IsPowerReady()             */
    bool     enable_commanded;      /* PC4 EN_STEER commanded            */
    bool     encoder_fault;         /* Encoder health fault              */
    bool     restored_from_flash;   /* Homing skipped via flash          */
    bool     module_disabled;       /* Disabled in Service Mode          */
    bool     fault_latched;         /* SAFETY_ERROR_CENTERING latched    */
    bool     pwm_requested;         /* pwm_requested > 0                 */
    bool     current_guard_armed;   /* CH5 end-stop guard can observe    */

    uint16_t pwm_real;              /* max CCR PA6/PA7                    */
    int16_t  encoder_delta;         /* clamped delta from sweep origin   */
} SteerCenteringFrame;

/**
 * @brief  Serialise a classified homing snapshot into the 8-byte 0x316 frame.
 *         Pure: reads @p d, writes @p out[0..7], no side effects.
 */
static inline void SteerCentering_PackFrame(const SteeringCenteringDiag *d,
                                            uint8_t out[8])
{
    for (int i = 0; i < 8; i++) {
        out[i] = 0U;
    }
    if (d == 0) {
        return;
    }

    out[0] = (uint8_t)d->abort_reason;
    out[1] = (uint8_t)(((uint8_t)d->fsm_state & 0x0FU) |
                       (((uint8_t)d->motor_owner & 0x0FU) << 4));

    uint8_t flags = 0U;
    if (d->center_sensor_raw)            flags |= (uint8_t)(1U << 0);
    if (d->center_sensor_debounced)      flags |= (uint8_t)(1U << 1);
    if (d->center_sensor_already_active) flags |= (uint8_t)(1U << 2);
    if (d->relay_steer_commanded)        flags |= (uint8_t)(1U << 3);
    if (d->power_ready)                  flags |= (uint8_t)(1U << 4);
    if (d->enable_commanded)             flags |= (uint8_t)(1U << 5);
    if (d->encoder_fault)                flags |= (uint8_t)(1U << 6);
    if (d->restored_from_flash)          flags |= (uint8_t)(1U << 7);
    out[2] = flags;

    uint8_t st = (uint8_t)(d->system_state & 0x0FU);
    if (d->module_disabled)     st |= (uint8_t)(1U << 4);
    if (d->fault_latched)       st |= (uint8_t)(1U << 5);
    if (d->pwm_requested > 0U)  st |= (uint8_t)(1U << 6);
    if (d->current_guard_armed) st |= (uint8_t)(1U << 7);
    out[3] = st;

    uint16_t pwm_real = (d->pwm_applied_ch1 > d->pwm_applied_ch2)
                            ? d->pwm_applied_ch1 : d->pwm_applied_ch2;
    out[4] = (uint8_t)(pwm_real & 0xFFU);
    out[5] = (uint8_t)((pwm_real >> 8) & 0xFFU);

    int32_t delta = d->encoder_delta;
    if (delta >  32767)  delta =  32767;
    if (delta < -32768)  delta = -32768;
    uint16_t d16 = (uint16_t)(int16_t)delta;
    out[6] = (uint8_t)(d16 & 0xFFU);
    out[7] = (uint8_t)((d16 >> 8) & 0xFFU);
}

/**
 * @brief  Decode an 8-byte 0x316 frame into a flat SteerCenteringFrame.
 *         Pure: reads @p in[0..7], writes @p out, no side effects.
 */
static inline void SteerCentering_UnpackFrame(const uint8_t in[8],
                                              SteerCenteringFrame *out)
{
    if (out == 0 || in == 0) {
        return;
    }

    out->reason       = in[0];
    out->fsm_state    = (uint8_t)(in[1] & 0x0FU);
    out->motor_owner  = (uint8_t)((in[1] >> 4) & 0x0FU);

    out->center_raw            = (in[2] & (1U << 0)) != 0U;
    out->center_debounced      = (in[2] & (1U << 1)) != 0U;
    out->center_already_active = (in[2] & (1U << 2)) != 0U;
    out->relay_commanded       = (in[2] & (1U << 3)) != 0U;
    out->power_ready           = (in[2] & (1U << 4)) != 0U;
    out->enable_commanded      = (in[2] & (1U << 5)) != 0U;
    out->encoder_fault         = (in[2] & (1U << 6)) != 0U;
    out->restored_from_flash   = (in[2] & (1U << 7)) != 0U;

    out->system_state    = (uint8_t)(in[3] & 0x0FU);
    out->module_disabled = (in[3] & (1U << 4)) != 0U;
    out->fault_latched   = (in[3] & (1U << 5)) != 0U;
    out->pwm_requested   = (in[3] & (1U << 6)) != 0U;
    out->current_guard_armed = (in[3] & (1U << 7)) != 0U;

    out->pwm_real      = (uint16_t)((uint16_t)in[4] |
                                    ((uint16_t)in[5] << 8));
    out->encoder_delta = (int16_t)((uint16_t)in[6] |
                                   ((uint16_t)in[7] << 8));
}

#endif /* STEERING_CENTERING_FRAME_H */
