/**
  ****************************************************************************
  * @file    steering_output.h
  * @brief   Shared production shutdown of the physical steering-assist
  *          actuator (the electric ASSIST motor only — never the mechanical
  *          steering).
  *
  * A single low-level function performs the exact register-level coast of the
  * steering H-bridge so the identical hardware effect is used everywhere:
  *
  *     TIM3 CCR1 (PA6 / RPWM_STEER) = 0
  *     TIM3 CCR2 (PA7 / LPWM_STEER) = 0
  *     PC4 (EN_STEER)               = LOW  (coast, Hi-Z)
  *
  * Consumers:
  *   - motor_control.c  (Steering_Neutralize coast path),
  *   - steering_eps.c   (isolation authority Steering_DisableAssistFault),
  *   - host tests        (link this real function and assert real registers).
  *
  * It does NOT touch PC12 (steering rail) or the traction chain — the rail is
  * released separately by Steering_SteerPowerOff() (safety_system.c).
  ****************************************************************************
  */

#ifndef STEERING_OUTPUT_H
#define STEERING_OUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Coast the steering-assist motor at the register level: zero both
 *         TIM3 PWM compares (PA6/PA7) and force EN_STEER (PC4) LOW.
 *
 *         Idempotent — writing zeros generates no pulses and is safe to
 *         repeat every control cycle.  Never references PC12 or traction.
 */
void Steering_PhysicalOff(void);

#ifdef __cplusplus
}
#endif

#endif /* STEERING_OUTPUT_H */
