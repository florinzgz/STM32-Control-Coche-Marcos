/**
  ****************************************************************************
  * @file    ackermann.c
  * @brief   Ackermann steering geometry — pure calculation module
  *
  * Implements the standard Ackermann equations to compute individual
  * front-left and front-right wheel angles from a single road-wheel
  * steering command.  Uses only the vehicle constants defined in
  * vehicle_physics.h (derived from the base firmware).
  *
  * This module is intentionally free of any motor control, encoder,
  * PID, or safety logic.
  ****************************************************************************
  */

#include "ackermann.h"
#include "vehicle_physics.h"
#include <math.h>

void Ackermann_ComputeWheelAngles(float road_angle_deg,
                                   float *out_fl_deg,
                                   float *out_fr_deg)
{
    /* Straight ahead: both wheels at zero */
    if (fabsf(road_angle_deg) < 0.01f) {
        *out_fl_deg = 0.0f;
        *out_fr_deg = 0.0f;
        return;
    }

    float abs_angle_rad = fabsf(road_angle_deg) * (float)M_PI / 180.0f;

    /* Centreline turn radius: R = L / tan(|road_angle|) */
    float R = WHEELBASE_M / tanf(abs_angle_rad);

    /* Inner and outer wheel turn radii */
    float R_inner = R - TRACK_WIDTH_M / 2.0f;
    float R_outer = R + TRACK_WIDTH_M / 2.0f;

    /* Individual wheel angles (always positive here, sign applied below) */
    float inner_deg = atanf(WHEELBASE_M / R_inner) * 180.0f / (float)M_PI;
    float outer_deg = atanf(WHEELBASE_M / R_outer) * 180.0f / (float)M_PI;

    /* Clamp per-wheel angles to MAX_STEER_DEG */
    if (inner_deg > MAX_STEER_DEG) inner_deg = MAX_STEER_DEG;
    if (outer_deg > MAX_STEER_DEG) outer_deg = MAX_STEER_DEG;

    /* Assign to FL / FR and apply sign based on turn direction.
     * Positive road_angle_deg = left turn:
     *   FL is inner (larger angle), FR is outer (smaller angle).
     * Negative road_angle_deg = right turn:
     *   FR is inner (larger angle), FL is outer (smaller angle).       */
    if (road_angle_deg > 0.0f) {
        *out_fl_deg =  inner_deg;
        *out_fr_deg =  outer_deg;
    } else {
        *out_fl_deg = -outer_deg;
        *out_fr_deg = -inner_deg;
    }
}
