/**
  ****************************************************************************
  * @file    eps_params.h
  * @brief   EPS (Electric Power Steering) calibration parameters
  *
  * Runtime-calibrable parameter structure for the torque-assist EPS
  * algorithm.  Parameters are stored in flash with checksum validation
  * and double-buffer safety.  The EPS control loop reads from the
  * active RAM copy; runtime changes take effect immediately without
  * reboot.
  *
  * API:
  *   EPS_Params_Init()          — load from flash or apply defaults
  *   EPS_Params_Get()           — read-only pointer to active params
  *   EPS_Params_Set()           — update a single field by index
  *   EPS_Params_Save()          — persist current RAM copy to flash
  *   EPS_Params_ResetDefaults() — revert to compiled defaults
  ****************************************************************************
  */

#ifndef EPS_PARAMS_H
#define EPS_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ---- Calibration parameter block ---- */
typedef struct {
    /* ---- Torque-assist algorithm gains ---- */
    float assist_strength;   /* Assist torque gain (driver intention)       */
    float center_strength;   /* Self-centering spring gain                  */
    float damping;           /* Velocity damping coefficient                */
    float friction_comp;     /* Static friction compensation torque         */
    float coast_band_pct;    /* Below this |PWM%| → coast (EN=LOW)         */
    float min_drive_pct;     /* Dead-zone jump: min PWM% when driving      */
    float assist_vs_speed;   /* Speed sensitivity for assist: g(v)=1/(1+v/X) */
    float return_vs_speed;   /* Speed sensitivity for return: h(v)=0.3+v/X */
    /* ---- Mechanical / output stage parameters ---- */
    float deadband_deg;      /* Backlash deadband (road-wheel °) — angles
                              * below this are treated as zero; default 1.8° */
    float max_pwm_pct;       /* Maximum PWM output clamp (0..100 %)        */
    float slew_rate_pct;     /* Slew-rate limit per control cycle (% of
                              * full PWM range); default ≈5.88 % (250 cts) */
    float center_offset_deg; /* Center-position correction (road-wheel °);
                              * positive shifts the neutral point right     */
} eps_params_t;

/* ---- Parameter indices for EPS_Params_Set() ---- */
typedef enum {
    EPS_PARAM_ASSIST_STRENGTH = 0,
    EPS_PARAM_CENTER_STRENGTH,
    EPS_PARAM_DAMPING,
    EPS_PARAM_FRICTION_COMP,
    EPS_PARAM_COAST_BAND_PCT,
    EPS_PARAM_MIN_DRIVE_PCT,
    EPS_PARAM_ASSIST_VS_SPEED,
    EPS_PARAM_RETURN_VS_SPEED,
    EPS_PARAM_DEADBAND_DEG,
    EPS_PARAM_MAX_PWM_PCT,
    EPS_PARAM_SLEW_RATE_PCT,
    EPS_PARAM_CENTER_OFFSET_DEG,
    EPS_PARAM_COUNT
} eps_param_id_t;

/* ---- SET_PARAM safety gate (Item A) ----------------------------------
 * A live EPS_Params_Set() reached over CAN (EPS_PARAM_OP_SET_PARAM) alters
 * steering-assist torque on the very next control-loop cycle.  Numeric
 * range validation in EPS_Params_Set() alone is NOT sufficient: an
 * in-range value can still be dangerous if applied while the vehicle is
 * moving or being driven.  EPS_Params_SetAllowed() gates the SET operation
 * on the vehicle being in a demonstrably safe, stationary state.
 *
 * ALL five conditions must hold for a SET to be accepted:
 *   1. system state == STANDBY,
 *   2. gear is PARK or NEUTRAL (no drive/reverse engaged),
 *   3. every wheel-speed magnitude <= EPS_SETPARAM_MAX_WHEEL_SPEED_KMH,
 *   4. traction demand magnitude  <  EPS_SETPARAM_MAX_DEMAND_PCT (≈zero),
 *   5. no dangerous active fault is latched.
 *
 * The predicate is PURE (no HAL / Safety dependency) so it links directly
 * into the host unit test; the CAN handler assembles the live snapshot. */
#define EPS_SETPARAM_MAX_WHEEL_SPEED_KMH  1.0f
#define EPS_SETPARAM_MAX_DEMAND_PCT       3.0f

typedef struct {
    uint8_t state;               /* SystemState_t value                     */
    uint8_t state_standby;       /* SYS_STATE_STANDBY enum value            */
    uint8_t gear;                /* GearPosition_t value                    */
    uint8_t gear_park;           /* GEAR_PARK enum value                    */
    uint8_t gear_neutral;        /* GEAR_NEUTRAL enum value                 */
    float   max_wheel_speed_kmh; /* max |wheel speed| across all wheels     */
    float   demand_pct;          /* traction demand (signed %)              */
    bool    dangerous_fault;     /* true if a dangerous active fault is set */
} eps_setparam_gate_t;

/* ---- Public API ---- */

/**
 * @brief  Initialise EPS parameters.
 *         Loads from flash if valid (checksum OK); otherwise applies
 *         compiled defaults.  Must be called once at startup before
 *         Steering_ControlLoop().
 */
void EPS_Params_Init(void);

/**
 * @brief  Get read-only pointer to the active parameter set.
 * @retval Pointer to the current eps_params_t (never NULL).
 */
const eps_params_t *EPS_Params_Get(void);

/**
 * @brief  Update a single parameter in the active RAM copy.
 *         Takes effect immediately on the next control loop iteration.
 * @param  id    Parameter index (eps_param_id_t).
 * @param  value New float value.
 * @retval true if accepted, false if id is out of range.
 */
bool EPS_Params_Set(eps_param_id_t id, float value);

/**
 * @brief  SET_PARAM safety gate (Item A).
 *         Returns true only when a live parameter change is safe to apply,
 *         i.e. the vehicle is in STANDBY, in PARK or NEUTRAL, stationary,
 *         with ~zero traction demand and no dangerous active fault.
 *         Pure function — no side effects, no globals; callers pass a
 *         snapshot of the current safety/motion state.
 * @param  g  Snapshot of the gating conditions (must not be NULL).
 * @retval true if a SET_PARAM may be applied, false otherwise.
 */
bool EPS_Params_SetAllowed(const eps_setparam_gate_t *g);

/**
 * @brief  Persist the current RAM parameter copy to flash.
 *         Uses checksum + double-buffer for write safety.
 * @retval true on success, false on flash write error.
 */
bool EPS_Params_Save(void);

/**
 * @brief  Reset all parameters to compiled defaults (RAM only).
 *         Call EPS_Params_Save() afterwards to persist.
 */
void EPS_Params_ResetDefaults(void);

/**
 * @brief  Return the compiled default parameter set.
 * @retval Pointer to the immutable default eps_params_t (never NULL).
 */
const eps_params_t *EPS_Params_GetDefaults(void);

/**
 * @brief  Returns true if a valid parameter block was loaded from flash.
 *         False means defaults were applied (no persisted calibration).
 */
bool EPS_Params_IsFlashValid(void);

/**
 * @brief  Read the authoritative server-side [min, max] limit for a
 *         parameter.  Exposes the eps_limits[] contract so the HMI editor
 *         ranges (esp32/src/eps_limits.h) can be cross-checked against the
 *         values a raw CAN EPS_PARAM_OP_SET_PARAM frame is validated with.
 * @param  id        Parameter index (< EPS_PARAM_COUNT).
 * @param  out_min   Receives the inclusive lower bound (may be NULL).
 * @param  out_max   Receives the inclusive upper bound (may be NULL).
 * @retval true on success, false if id is out of range.
 */
bool EPS_Params_GetLimit(eps_param_id_t id, float *out_min, float *out_max);

#ifdef __cplusplus
}
#endif

#endif /* EPS_PARAMS_H */
