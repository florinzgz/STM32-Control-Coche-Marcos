/**
  ****************************************************************************
  * @file    ackermann.h
  * @brief   Ackermann steering geometry — pure calculation module
  *
  * Self-contained Ackermann geometry computation.  Given a desired
  * road-wheel steering angle, computes the individual front-left and
  * front-right wheel angles so that all wheels share a common
  * instantaneous turn centre (no tyre scrub).
  *
  * Vehicle constants are taken from vehicle_physics.h (base firmware).
  ****************************************************************************
  */

#ifndef __ACKERMANN_H
#define __ACKERMANN_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Compute front-left and front-right road-wheel angles using
 *         Ackermann steering geometry.
 *
 * Standard Ackermann equations (bicycle-model extension):
 *
 *   R       = L / tan(|road_angle|)        (centreline turn radius)
 *   inner   = atan( L / (R - T/2) )        (inner wheel, larger angle)
 *   outer   = atan( L / (R + T/2) )        (outer wheel, smaller angle)
 *
 * where L = WHEELBASE_M, T = TRACK_WIDTH_M.
 *
 * Sign convention (positive = left turn):
 *   Left turn  → FL is the inner wheel, FR is the outer wheel.
 *   Right turn → FR is the inner wheel, FL is the outer wheel.
 *
 * Both output angles are clamped to ±MAX_STEER_DEG.
 * If road_angle_deg == 0, both outputs are exactly 0.
 *
 * @param road_angle_deg  Desired road-wheel angle in degrees (+ = left)
 * @param out_fl_deg      Pointer to receive front-left wheel angle (degrees)
 * @param out_fr_deg      Pointer to receive front-right wheel angle (degrees)
 */
void Ackermann_ComputeWheelAngles(float road_angle_deg,
                                   float *out_fl_deg,
                                   float *out_fr_deg);

#ifdef __cplusplus
}
#endif

#endif /* __ACKERMANN_H */
