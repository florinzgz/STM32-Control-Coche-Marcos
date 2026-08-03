/**
  ****************************************************************************
  * @file    gear_limits_store.h
  * @brief   Persistent gear/lever power-limit + accel-response storage (NVM)
  *
  * Stores two per-gear profiles (D2 / D1 / R) in flash so they can be tuned
  * from the ESP32-HMI Engineering menu without re-flashing firmware:
  *
  *   1. POWER LIMIT  — caps the max traction demand for each gear; applied
  *      in motor_control.c::Traction_Update().  Defaults 100 / 60 / 60 %.
  *   2. ACCEL RESPONSE — softens (never amplifies) the demand signal per
  *      gear; applied in Traction_SetDemand() after EMA#2 and before the
  *      global ramp limiter.  Defaults 100 / 70 / 40 %.
  *
  * Both are integer percentages for each gear position:
  *
  *   D2 (GEAR_FORWARD_D2) — full-performance forward
  *   D1 (GEAR_FORWARD)    — low-gear forward
  *   R  (GEAR_REVERSE)    — reverse
  *
  * Safety invariants (mirrors pedal_cal_store.c):
  *   - Flash data alone NEVER authorises ACTIVE or clears startup_inhibit;
  *     it only scales an already-validated traction demand.
  *   - The accel-response factor is clamped to <= 1.0 and applied to
  *     positive demand only, so it can ONLY soften, never amplify.
  *   - On flash blank, CRC-invalid, or out-of-range, the module silently
  *     falls back to "no valid slot" and the caller keeps the compile-time
  *     defaults below.  Boot is NEVER blocked by a missing/corrupt slot.
  *   - Mutating and persisting a gear profile is permitted only in true
  *     SYS_STATE_STANDBY.  A pedal-calibration service lock never authorises
  *     a gear-limit write.
  *   - All ramp limiting, ABS/TCS wheel_scale[], temperature cut-off and
  *     Safety_GetPowerLimitFactor() gates in motor_control.c remain
  *     unchanged and apply on top of the gear scaling.
  *
  * On-flash format versioning:
  *   - v1 (magic "GLM1") stored power limits only (response in reserved
  *     bytes = 0).  v2 (magic "GLM2") adds the accel-response triple.
  *   - A valid v1 slot is migrated safely on read: the power limits are
  *     honoured and the compile-time response defaults are applied.  The
  *     slot is upgraded to v2 only when the operator next SAVEs.
  *
  * Flash layout:
  *   Page 122 (0x0807A000, 4 KB) — dedicated to gear profiles.
  *   Single 16-byte slot with magic + CRC32 integrity check.
  ****************************************************************************
  */

#ifndef GEAR_LIMITS_STORE_H
#define GEAR_LIMITS_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define GEAR_LIMIT_D2_MIN_PCT   30U
#define GEAR_LIMIT_D2_MAX_PCT   100U
#define GEAR_LIMIT_D1_MIN_PCT   20U
#define GEAR_LIMIT_D1_MAX_PCT   100U
#define GEAR_LIMIT_R_MIN_PCT    10U
#define GEAR_LIMIT_R_MAX_PCT    60U

/* Historic compile-time traction limits:
 *   D2 = 100 %, D1 = 60 %, R = 60 %. */
#define GEAR_LIMIT_D2_DEFAULT_PCT  100U
#define GEAR_LIMIT_D1_DEFAULT_PCT  60U
#define GEAR_LIMIT_R_DEFAULT_PCT   60U

#define GEAR_RESPONSE_D2_MIN_PCT   50U
#define GEAR_RESPONSE_D2_MAX_PCT   100U
#define GEAR_RESPONSE_D1_MIN_PCT   30U
#define GEAR_RESPONSE_D1_MAX_PCT   100U
#define GEAR_RESPONSE_R_MIN_PCT    20U
#define GEAR_RESPONSE_R_MAX_PCT    80U

#define GEAR_RESPONSE_D2_DEFAULT_PCT  100U
#define GEAR_RESPONSE_D1_DEFAULT_PCT  70U
#define GEAR_RESPONSE_R_DEFAULT_PCT   40U

_Static_assert(GEAR_LIMIT_D2_DEFAULT_PCT >= GEAR_LIMIT_D2_MIN_PCT &&
               GEAR_LIMIT_D2_DEFAULT_PCT <= GEAR_LIMIT_D2_MAX_PCT,
               "D2 default out of range");
_Static_assert(GEAR_LIMIT_D1_DEFAULT_PCT >= GEAR_LIMIT_D1_MIN_PCT &&
               GEAR_LIMIT_D1_DEFAULT_PCT <= GEAR_LIMIT_D1_MAX_PCT,
               "D1 default out of range");
_Static_assert(GEAR_LIMIT_R_DEFAULT_PCT >= GEAR_LIMIT_R_MIN_PCT &&
               GEAR_LIMIT_R_DEFAULT_PCT <= GEAR_LIMIT_R_MAX_PCT,
               "R default out of range");
_Static_assert(GEAR_RESPONSE_D2_DEFAULT_PCT >= GEAR_RESPONSE_D2_MIN_PCT &&
               GEAR_RESPONSE_D2_DEFAULT_PCT <= GEAR_RESPONSE_D2_MAX_PCT,
               "D2 response default out of range");
_Static_assert(GEAR_RESPONSE_D1_DEFAULT_PCT >= GEAR_RESPONSE_D1_MIN_PCT &&
               GEAR_RESPONSE_D1_DEFAULT_PCT <= GEAR_RESPONSE_D1_MAX_PCT,
               "D1 response default out of range");
_Static_assert(GEAR_RESPONSE_R_DEFAULT_PCT >= GEAR_RESPONSE_R_MIN_PCT &&
               GEAR_RESPONSE_R_DEFAULT_PCT <= GEAR_RESPONSE_R_MAX_PCT,
               "R response default out of range");
_Static_assert(GEAR_RESPONSE_D2_MAX_PCT <= 100U &&
               GEAR_RESPONSE_D1_MAX_PCT <= 100U &&
               GEAR_RESPONSE_R_MAX_PCT  <= 100U,
               "response factor must never amplify (<=100%)");

void GearLimitsStore_Init(void);
bool GearLimitsStore_IsValid(void);
void GearLimitsStore_GetStored(uint8_t *d2_pct, uint8_t *d1_pct, uint8_t *r_pct);
void GearLimitsStore_GetStoredResponse(uint8_t *d2_pct, uint8_t *d1_pct,
                                       uint8_t *r_pct);
bool GearLimitsStore_IsLegacyFormat(void);
bool GearLimitsStore_Validate(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct);
bool GearLimitsStore_ValidateResponse(uint8_t d2_pct, uint8_t d1_pct,
                                      uint8_t r_pct);

/** Persist a complete v2 profile.  The implementation accepts writes only
 * while Safety_GetState() == SYS_STATE_STANDBY and the safety error is clear. */
bool GearLimitsStore_Save(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct,
                          uint8_t d2_resp, uint8_t d1_resp, uint8_t r_resp);

#ifdef __cplusplus
}
#endif

#endif /* GEAR_LIMITS_STORE_H */
