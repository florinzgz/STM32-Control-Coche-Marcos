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

#ifndef __EPS_PARAMS_H
#define __EPS_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ---- Calibration parameter block ---- */
typedef struct {
    float assist_strength;   /* Assist torque gain (driver intention)       */
    float center_strength;   /* Self-centering spring gain                  */
    float damping;           /* Velocity damping coefficient                */
    float friction_comp;     /* Static friction compensation torque         */
    float coast_band_pct;    /* Below this |PWM%| → coast (EN=LOW)         */
    float min_drive_pct;     /* Dead-zone jump: min PWM% when driving      */
    float assist_vs_speed;   /* Speed sensitivity for assist: g(v)=1/(1+v/X) */
    float return_vs_speed;   /* Speed sensitivity for return: h(v)=0.3+v/X */
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
    EPS_PARAM_COUNT
} eps_param_id_t;

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

#ifdef __cplusplus
}
#endif

#endif /* __EPS_PARAMS_H */
