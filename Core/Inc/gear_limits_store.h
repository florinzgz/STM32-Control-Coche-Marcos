/**
  ****************************************************************************
  * @file    gear_limits_store.h
  * @brief   Persistent gear/lever power-limit + accel-response storage (NVM)
  *
  * Stores two per-gear profiles (D2 / D1 / R) in flash so they can be tuned
  * from the ESP32-HMI Engineering menu without re-flashing firmware:
  *
  *   1. POWER LIMIT  — caps the max traction demand for each gear; applied
  *      in motor_control.c::Traction_Update().  Defaults 100 / 60 / 80 %.
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
  *   Single 16-byte slot with magic + CRC32 integrity check.  Modelled
  *   1:1 on pedal_cal_store.c so the flash-write primitive set is identical.
  ****************************************************************************
  */

#ifndef GEAR_LIMITS_STORE_H
#define GEAR_LIMITS_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---- Hard validation ranges (percent) ----------------------------
 * These are the single source of truth for the configurable gear power
 * limits.  motor_control.c and the CAN service handler validate against
 * the same macros so a value accepted by one path is accepted by all.
 *
 * Rationale (verified against motor_control.c::Traction_Update()):
 *   - D2 is full performance; never let it drop below a usable floor.
 *   - R (reverse) is capped low — reverse is always close-quarters
 *     manoeuvring, so the upper bound stays well below forward power.   */
#define GEAR_LIMIT_D2_MIN_PCT   30U
#define GEAR_LIMIT_D2_MAX_PCT   100U
#define GEAR_LIMIT_D1_MIN_PCT   20U
#define GEAR_LIMIT_D1_MAX_PCT   100U
#define GEAR_LIMIT_R_MIN_PCT    10U
#define GEAR_LIMIT_R_MAX_PCT    80U

/* ---- Compile-time defaults (percent) -----------------------------
 * These MIRROR the historic compile-time behaviour of motor_control.c:
 *   GEAR_POWER_FORWARD_D2_PCT = 1.00f -> 100 %
 *   GEAR_POWER_FORWARD_PCT    = 0.60f ->  60 %
 *   GEAR_POWER_REVERSE_PCT    = 0.80f ->  80 %
 * Keeping the defaults identical to the original macros guarantees that
 * a unit with no/blank/corrupt flash slot behaves exactly as before.    */
#define GEAR_LIMIT_D2_DEFAULT_PCT  100U
#define GEAR_LIMIT_D1_DEFAULT_PCT  60U
#define GEAR_LIMIT_R_DEFAULT_PCT   80U

/* ---- Hard validation ranges — accel RESPONSE (percent) -----------
 * The acceleration response profile softens (never amplifies) the pedal
 * demand signal per gear, applied in motor_control.c::Traction_SetDemand()
 * after EMA#2 and before the global ramp limiter.  These ranges are the
 * single source of truth shared by the flash store, motor_control.c and
 * the CAN service handler.
 *
 * Rationale (verified against motor_control.c::Traction_SetDemand()):
 *   - The factor multiplies a strictly positive demand target and is
 *     clamped to <= 1.0, so it can only soften, never amplify, demand.
 *   - D2 keeps a high floor (50 %) so full-performance forward stays
 *     responsive; R is capped at 80 % and floored at 20 % so reverse is
 *     always progressive but never frozen.                              */
#define GEAR_RESPONSE_D2_MIN_PCT   50U
#define GEAR_RESPONSE_D2_MAX_PCT   100U
#define GEAR_RESPONSE_D1_MIN_PCT   30U
#define GEAR_RESPONSE_D1_MAX_PCT   100U
#define GEAR_RESPONSE_R_MIN_PCT    20U
#define GEAR_RESPONSE_R_MAX_PCT    80U

/* ---- Compile-time defaults — accel RESPONSE (percent) ------------
 * Defaults requested by the task: D2 = 100 (normal), D1 = 70 (softer for
 * manoeuvres), R = 40 (very progressive for reverse).  When a unit has no
 * valid slot, or only a legacy power-only slot, these defaults apply so
 * behaviour is well-defined.                                            */
#define GEAR_RESPONSE_D2_DEFAULT_PCT  100U
#define GEAR_RESPONSE_D1_DEFAULT_PCT  70U
#define GEAR_RESPONSE_R_DEFAULT_PCT   40U

/* ---- Defensive coherence checks ---------------------------------- */
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
/* The response factor must never amplify demand: every legal value is
 * <= 100 %.  Guard the upper bounds so a future range edit cannot break
 * the "soften only" invariant. */
_Static_assert(GEAR_RESPONSE_D2_MAX_PCT <= 100U &&
               GEAR_RESPONSE_D1_MAX_PCT <= 100U &&
               GEAR_RESPONSE_R_MAX_PCT  <= 100U,
               "response factor must never amplify (<=100%)");

/**
 * @brief  Initialise the gear-limits store.  Reads flash and validates
 *         CRC / magic / range.  Does NOT apply the limits — the caller
 *         (main.c) decides when to call Traction_SetGearLimits() with the
 *         values returned by GearLimitsStore_GetStored().
 *
 *         On flash blank, CRC-invalid, or out-of-range, the module
 *         silently falls back to "no valid slot" — boot is never
 *         blocked.                                                       */
void GearLimitsStore_Init(void);

/**
 * @brief  Returns true if the flash slot was present and passed all
 *         integrity + range checks at boot.                             */
bool GearLimitsStore_IsValid(void);

/**
 * @brief  Read the stored D2 / D1 / R power-limit percentages.  Only
 *         meaningful when GearLimitsStore_IsValid() == true; otherwise
 *         outputs are zero. */
void GearLimitsStore_GetStored(uint8_t *d2_pct, uint8_t *d1_pct, uint8_t *r_pct);

/**
 * @brief  Read the stored D2 / D1 / R accel-response percentages.
 *
 *         Only meaningful when GearLimitsStore_IsValid() == true.  When a
 *         legacy (power-only) slot was loaded, the response values are the
 *         compile-time response defaults (D2 100 / D1 70 / R 40) — the old
 *         slot is NOT rewritten until the next SAVE.                       */
void GearLimitsStore_GetStoredResponse(uint8_t *d2_pct, uint8_t *d1_pct, uint8_t *r_pct);

/**
 * @brief  True when the loaded slot was a legacy power-only (v1) slot.
 *         The caller keeps the persisted power limits but applies the
 *         compile-time response defaults; the slot is upgraded to v2 only
 *         on the next explicit SAVE.                                       */
bool GearLimitsStore_IsLegacyFormat(void);

/**
 * @brief  Pure validator.  Returns true if (d2,d1,r) satisfy all hard
 *         power-limit range limits without touching flash.  Used by both
 *         the flash loader and the CAN SAVE sub-opcode handler.           */
bool GearLimitsStore_Validate(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct);

/**
 * @brief  Pure validator for the accel-response percentages.  Returns true
 *         if (d2,d1,r) satisfy all hard response range limits.            */
bool GearLimitsStore_ValidateResponse(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct);

/**
 * @brief  Persist new gear power-limit + accel-response percentages to flash.
 *
 * Write conditions (caller is responsible for the safety gates):
 *   - SYS_STATE_STANDBY, or a physically confirmed service lock
 *
 * Range conditions (validated internally via GearLimitsStore_Validate() and
 * GearLimitsStore_ValidateResponse()).
 *
 * Always writes the current (v2) on-flash format containing both the power
 * limits and the accel-response profile.
 *
 * @retval true on success, false on validation or flash error.
 */
bool GearLimitsStore_Save(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct,
                          uint8_t d2_resp, uint8_t d1_resp, uint8_t r_resp);

#ifdef __cplusplus
}
#endif

#endif /* GEAR_LIMITS_STORE_H */
