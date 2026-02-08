/**
  ****************************************************************************
  * @file    vehicle_physics.h
  * @brief   Physical and mechanical constants for the vehicle
  ****************************************************************************
  */

#ifndef __VEHICLE_PHYSICS_H
#define __VEHICLE_PHYSICS_H

/* ---- Wheel physical data ---- */
#define WHEEL_CIRCUM_MM         1100.0f                       /* Wheel circumference (mm) */
#define WHEEL_CIRCUMF_M         (WHEEL_CIRCUM_MM / 1000.0f)  /* Wheel circumference (m)  */

/* ---- Vehicle geometry ---- */
#define WHEELBASE_M             0.95f   /* Distance between front and rear axles (m) */
#define TRACK_WIDTH_M           0.70f   /* Distance between left and right wheels (m) */

/* ---- Steering limits ---- */
#define MAX_STEER_DEG           54.0f   /* Maximum road-wheel angle (degrees) */
#define STEERING_WHEEL_MAX_DEG  350.0f  /* Steering wheel mechanical travel (degrees) */

#endif /* __VEHICLE_PHYSICS_H */
