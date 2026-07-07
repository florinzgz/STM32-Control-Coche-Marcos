/**
  ****************************************************************************
  * @file    vehicle_physics.h
  * @brief   Physical and mechanical constants for the vehicle
  ****************************************************************************
  */

#ifndef VEHICLE_PHYSICS_H
#define VEHICLE_PHYSICS_H

/* ---- Wheel physical data ---- */
/* Measured circumference = 1110 mm (111 cm).  With WHEEL_PULSES_REV = 6
 * bolts this gives 1110 / 6 = 185 mm of travel per accepted pulse.        */
#define WHEEL_CIRCUM_MM         1110.0f                       /* Wheel circumference (mm) */
#define WHEEL_CIRCUMF_M         (WHEEL_CIRCUM_MM / 1000.0f)  /* Wheel circumference (m)  */

/* ---- Vehicle geometry ---- */
#define WHEELBASE_M             0.95f   /* Distance between front and rear axles (m) */
#define TRACK_WIDTH_M           0.70f   /* Distance between left and right wheels (m) */

/* ---- Steering limits ---- */
#define MAX_STEER_DEG           54.0f   /* Maximum road-wheel angle (degrees) */
#define STEERING_WHEEL_MAX_DEG  350.0f  /* Steering wheel mechanical travel (degrees) */

#endif /* VEHICLE_PHYSICS_H */
