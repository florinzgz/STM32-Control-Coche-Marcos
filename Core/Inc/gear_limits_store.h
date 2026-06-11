/**
  ****************************************************************************
  * @file    gear_limits_store.h
  * @brief   Persistent gear/lever power-limit storage (flash NVM)
  *
  * Stores the per-gear traction power-limit percentages (D2 / D1 / R) in
  * flash so the limits applied in motor_control.c::Traction_Update() can be
  * tuned from the ESP32-HMI Engineering menu without re-flashing firmware.
  *
  * The limits are expressed as integer percentages (0..100) of the maximum
  * traction demand for each gear position:
  *
  *   D2 (GEAR_FORWARD_D2) — full-performance forward
  *   D1 (GEAR_FORWARD)    — low-gear forward
  *   R  (GEAR_REVERSE)    — reverse
  *
  * Safety invariants (mirrors pedal_cal_store.c):
  *   - Flash data alone NEVER authorises ACTIVE or clears startup_inhibit;
  *     it only scales an already-validated traction demand.
  *   - On flash blank, CRC-invalid, or out-of-range, the module silently
  *     falls back to "no valid slot" and the caller keeps the compile-time
  *     defaults below.  Boot is NEVER blocked by a missing/corrupt slot.
  *   - All ramp limiting, ABS/TCS wheel_scale[], temperature cut-off and
  *     Safety_GetPowerLimitFactor() gates in motor_control.c remain
  *     unchanged and apply on top of the gear scaling.
  *
  * Flash layout:
  *   Page 122 (0x0807A000, 4 KB) — dedicated to gear power limits.
  *   Single slot with magic + CRC32 integrity check.  Modelled 1:1 on
  *   pedal_cal_store.c so the flash-write primitive set is identical.
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
#define GEAR_LIMIT_R_MAX_PCT    60U

/* ---- Compile-time defaults (percent) -----------------------------
 * These MIRROR the historic compile-time behaviour of motor_control.c:
 *   GEAR_POWER_FORWARD_D2_PCT = 1.00f -> 100 %
 *   GEAR_POWER_FORWARD_PCT    = 0.60f ->  60 %
 *   GEAR_POWER_REVERSE_PCT    = 0.60f ->  60 %
 * Keeping the defaults identical to the original macros guarantees that
 * a unit with no/blank/corrupt flash slot behaves exactly as before.    */
#define GEAR_LIMIT_D2_DEFAULT_PCT  100U
#define GEAR_LIMIT_D1_DEFAULT_PCT  60U
#define GEAR_LIMIT_R_DEFAULT_PCT   60U

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
 * @brief  Read the stored D2 / D1 / R percentages.  Only meaningful when
 *         GearLimitsStore_IsValid() == true; otherwise outputs are zero. */
void GearLimitsStore_GetStored(uint8_t *d2_pct, uint8_t *d1_pct, uint8_t *r_pct);

/**
 * @brief  Pure validator.  Returns true if (d2,d1,r) satisfy all hard
 *         range limits without touching flash.  Used by both the flash
 *         loader and the CAN SAVE sub-opcode handler.                    */
bool GearLimitsStore_Validate(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct);

/**
 * @brief  Persist new gear-limit percentages to flash.
 *
 * Write conditions (caller is responsible for the safety gates):
 *   - Safety_GetState() == SYS_STATE_STANDBY
 *
 * Range conditions (validated internally via GearLimitsStore_Validate()).
 *
 * @retval true on success, false on validation or flash error.
 */
bool GearLimitsStore_Save(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct);

#ifdef __cplusplus
}
#endif

#endif /* GEAR_LIMITS_STORE_H */
