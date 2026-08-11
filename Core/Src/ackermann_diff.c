/**
  ****************************************************************************
  * @file    ackermann_diff.c
  * @brief   Ackermann per-wheel differential-torque correction — pure calc
  *
  * See ackermann_diff.h for the full rationale and algorithm description.
  *
  * Byte-for-byte identical math to the pre-extraction
  * compute_ackermann_differential() (motor_control.c, PR #445 Bloque C) —
  * steer_deg/wheelbase_m/track_m are taken as explicit parameters instead of
  * reading module statics or macros.  No NaN/Inf sanitisation and no side
  * effects here (that stays in motor_control.c's wrapper, which owns the
  * Safety_SetError() dependency).
  ****************************************************************************
  */

#include "ackermann_diff.h"
#include <math.h>

void AckermannDiff_Compute(float steer_deg, float wheelbase_m, float track_m,
                            float diff_out[4])
{
    /* Default: no correction (straight line or below deadband) */
    diff_out[ACKERMANN_DIFF_FL] = 1.0f;
    diff_out[ACKERMANN_DIFF_FR] = 1.0f;
    diff_out[ACKERMANN_DIFF_RL] = 1.0f;
    diff_out[ACKERMANN_DIFF_RR] = 1.0f;

    float abs_angle = fabsf(steer_deg);
    if (abs_angle < ACKERMANN_DEADBAND_DEG) return;

    /* Compute turn radius from Ackermann geometry:
     * R = wheelbase / tan(|angle|) — distance from turn center
     * to the midpoint of the rear axle.                            */
    float angle_rad = abs_angle * (float)M_PI / 180.0f;
    float tan_angle = tanf(angle_rad);

    /* Guard against very small tan (near-zero angle already
     * filtered by deadband, but protect against float edge cases) */
    if (tan_angle < 0.001f) return;

    float R = wheelbase_m / tan_angle;

    /* Compute correction term: half_track / R.
     * This is the fractional velocity difference between inside
     * and outside wheels relative to the vehicle center speed.     */
    float half_track = track_m / 2.0f;
    float correction = half_track / R;

    /* Bound correction to maximum differential */
    if (correction > ACKERMANN_MAX_DIFF)
        correction = ACKERMANN_MAX_DIFF;

    /* Apply correction:
     *   Positive steer_deg = left turn → left wheels inside (slower)
     *   Negative steer_deg = right turn → right wheels inside (slower)
     *
     * inside_mult  = 1.0 - correction  (reduce inside wheels)
     * outside_mult = 1.0 + correction  (increase outside wheels)
     * Then clamp outside to 1.0 to never exceed base torque.       */
    float inside_mult  = 1.0f - correction;
    float outside_mult = 1.0f + correction;

    /* Clamp: never exceed 1.0 per wheel */
    if (outside_mult > 1.0f) outside_mult = 1.0f;
    if (inside_mult  < 0.0f) inside_mult  = 0.0f;

    if (steer_deg > 0.0f) {
        /* Left turn: left wheels are inside */
        diff_out[ACKERMANN_DIFF_FL] = inside_mult;
        diff_out[ACKERMANN_DIFF_FR] = outside_mult;
        diff_out[ACKERMANN_DIFF_RL] = inside_mult;
        diff_out[ACKERMANN_DIFF_RR] = outside_mult;
    } else {
        /* Right turn: right wheels are inside */
        diff_out[ACKERMANN_DIFF_FL] = outside_mult;
        diff_out[ACKERMANN_DIFF_FR] = inside_mult;
        diff_out[ACKERMANN_DIFF_RL] = outside_mult;
        diff_out[ACKERMANN_DIFF_RR] = inside_mult;
    }
}
