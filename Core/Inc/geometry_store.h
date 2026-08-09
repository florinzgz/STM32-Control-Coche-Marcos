/**
  ****************************************************************************
  * @file    geometry_store.h
  * @brief   Persistent Ackermann geometry storage (NVM) — service diag C2
  *
  * Stores the runtime-tunable vehicle geometry used by the Ackermann
  * differential-torque correction in flash so it can be adjusted from the
  * ESP32-HMI SERVICE_AUTOTEST menu without re-flashing firmware.  Modelled
  * 1:1 on battery_limits_store.c.
  *
  * These were compile-time macros in vehicle_physics.h:
  *   WHEELBASE_M    0.95  range 0.50..1.50
  *   TRACK_WIDTH_M  0.70  range 0.40..1.20
  *
  * Invariant (unchanged, enforced by compute_ackermann_differential() in
  * motor_control.c -- NOT modified by this store):
  *   - Ackermann ONLY reduces: the outside wheel is always clamped to <=1.0,
  *     never boosted above the base demand.
  *   - The division-by-zero guard (tan_angle < 0.001f) and the
  *     ACKERMANN_DEADBAND_DEG deadband are untouched.
  *
  * Safety invariants:
  *   - Editable while a service-diag test is ACTIVE (staged in RAM only),
  *     same as C1/C5 -- geometry never actuates anything by itself.
  *   - On flash blank / CRC-invalid / out-of-range the module silently
  *     falls back to the compile-time defaults.  Boot is NEVER blocked.
  *
  * Flash layout:
  *   Page 113 (0x08071000, 4 KB) — dedicated to Ackermann geometry.
  *   Single slot with magic "GEO1" + CRC32 integrity check.
  ****************************************************************************
  */

#ifndef GEOMETRY_STORE_H
#define GEOMETRY_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "vehicle_physics.h"

/* ---- Hard validation ranges (matches the problem statement) ---- */
#define GEOMETRY_WHEELBASE_M_MIN     0.50f
#define GEOMETRY_WHEELBASE_M_MAX     1.50f
#define GEOMETRY_TRACK_WIDTH_M_MIN   0.40f
#define GEOMETRY_TRACK_WIDTH_M_MAX   1.20f

typedef struct {
    float wheelbase_m;
    float track_width_m;
} Geometry_t;

/* HAL-free single source used by production and host tests. */
static inline bool Geometry_ValidateValues(const Geometry_t *g)
{
    if (!g) return false;
    if (g->wheelbase_m < GEOMETRY_WHEELBASE_M_MIN ||
        g->wheelbase_m > GEOMETRY_WHEELBASE_M_MAX) return false;
    if (g->track_width_m < GEOMETRY_TRACK_WIDTH_M_MIN ||
        g->track_width_m > GEOMETRY_TRACK_WIDTH_M_MAX) return false;
    return true;
}

void GeometryStore_Init(void);
bool GeometryStore_IsValid(void);
void GeometryStore_GetDefaults(Geometry_t *out);
bool GeometryStore_Validate(const Geometry_t *g);

/** Effective (RAM-staged) values.  GeometryStore_Init()/Save() push these
 *  into Ackermann_SetGeometry() so motor_control.c's Traction_Update() and
 *  Ackermann_Compute() consume the live value immediately. */
void GeometryStore_GetEffective(Geometry_t *out);

/** Stage a candidate set in RAM AND apply it immediately via
 *  Ackermann_SetGeometry() (measure -> adjust -> measure loop, C2 is
 *  editable while a test is ACTIVE per Block C rules). */
bool GeometryStore_Stage(const Geometry_t *g);

void GeometryStore_GetStaged(Geometry_t *out);

/** Reverts the RAM stage (and the live Ackermann_SetGeometry() value) to the
 *  persisted (or default) values -- used on session abort/exit without SAVE. */
void GeometryStore_Revert(void);

/** Persist the currently staged geometry to flash. */
bool GeometryStore_Save(void);

/** Restores compile-time defaults into the RAM stage (and applies them
 *  immediately) -- the "restaurar defaults de este test" button. */
void GeometryStore_ResetToDefaults(void);

#ifdef __cplusplus
}
#endif

#endif /* GEOMETRY_STORE_H */
