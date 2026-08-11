/**
  ****************************************************************************
  * @file    ackermann_diff.h
  * @brief   Ackermann per-wheel differential-torque correction — pure calc
  *
  * Extracted from motor_control.c's compute_ackermann_differential() (PR
  * #445, Bloque C testability fix — llamapreview P2 finding) so the
  * calculation can be host-tested directly, independent of motor_control.c's
  * HAL/CAN/ADC dependency graph.
  *
  * Historical bug (PR #445, Bloque C, fixed before this extraction):
  * compute_ackermann_differential() used to read the compile-time
  * WHEELBASE_M / TRACK_WIDTH_M macros (vehicle_physics.h) instead of the
  * runtime ackermann_wheelbase / ackermann_track statics, silently
  * nullifying Ackermann_SetGeometry() — geometry_store.c staged the values
  * correctly, but the differential math never consumed them.  No host test
  * exercised the calculation itself (only a recording stub of
  * Ackermann_SetGeometry() in test_geometry_store.c proved the *store*
  * called through — see test_ackermann_diff.c for the missing coverage).
  *
  * Taking wheelbase_m/track_m as explicit parameters — instead of reading
  * module statics or macros — makes it structurally impossible for this
  * calculation to silently fall back to a compile-time constant, and lets a
  * host test drive it directly with non-default geometry to prove the
  * values passed in are what actually shape the output.
  *
  * This module is intentionally free of any motor control, HAL, encoder,
  * or safety logic — mirrors the style of ackermann.c (front-wheel steering
  * angles), a separate and functionally unrelated calculation.
  ****************************************************************************
  */

#ifndef ACKERMANN_DIFF_H
#define ACKERMANN_DIFF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Wheel indices for diff_out[4].  Intentionally independent of
 * motor_control.h's MOTOR_FL/FR/RL/RR (kept HAL-free); motor_control.c
 * cross-checks the two enumerations are identical with a _Static_assert. */
enum {
    ACKERMANN_DIFF_FL = 0,
    ACKERMANN_DIFF_FR = 1,
    ACKERMANN_DIFF_RL = 2,
    ACKERMANN_DIFF_RR = 3
};

/* No correction below this steering angle (degrees) — straight-line driving. */
#define ACKERMANN_DEADBAND_DEG   2.0f

/* Maximum +/-differential fraction (15%) applied to either wheel side. */
#define ACKERMANN_MAX_DIFF       0.15f

/**
 * @brief  Compute per-wheel Ackermann differential-torque multipliers.
 *
 * Simplified Ackermann geometry: when the vehicle is turning, the inside
 * wheels trace a smaller radius arc than the outside wheels.  To prevent
 * inside-wheel scrubbing (understeer) and improve cornering stability, the
 * torque is biased toward the outside wheels and reduced on the inside
 * wheels.
 *
 *   R = wheelbase / tan(|steer_deg|)        (turn center radius)
 *   inside_mult  = 1 - (track/2) / R        (clamped >= 0)
 *   outside_mult = 1 + (track/2) / R        (clamped <= 1, REDUCTION ONLY)
 *
 * The correction fraction (track/2)/R is bounded to +/-ACKERMANN_MAX_DIFF
 * (15%) before being applied.  Below ACKERMANN_DEADBAND_DEG (2 degrees), no
 * correction is applied (diff_out is all 1.0f).  A |steer_deg| so large that
 * tan(angle) < 0.001f (nominally unreachable below 90 degrees) also leaves
 * diff_out at the all-1.0f default rather than risk a near-zero divisor.
 *
 * outside_mult is always clamped to <= 1.0f, so base torque is never
 * increased above nominal on any wheel — only the inside wheel is ever
 * reduced ("reduction-only" policy, coexists with ABS/TCS/obstacle_scale).
 *
 * Pure function: no NaN/Inf sanitisation and no side effects (unlike
 * motor_control.c's local sanitize_float(), this has no Safety_SetError()
 * dependency).  The caller is responsible for sanitising steer_deg before
 * the call and diff_out after, exactly as motor_control.c's wrapper does.
 *
 * @param steer_deg   Current steering angle, degrees (+ = left turn).
 * @param wheelbase_m  Wheelbase, metres — pass the runtime
 *                      ackermann_wheelbase static (Ackermann_SetGeometry()),
 *                      NOT the WHEELBASE_M compile-time macro.
 * @param track_m      Track width, metres — pass the runtime
 *                      ackermann_track static, NOT the TRACK_WIDTH_M macro.
 * @param diff_out[4]  Output: FL, FR, RL, RR multipliers (1.0 = no change).
 */
void AckermannDiff_Compute(float steer_deg, float wheelbase_m, float track_m,
                            float diff_out[4]);

#ifdef __cplusplus
}
#endif

#endif /* ACKERMANN_DIFF_H */
