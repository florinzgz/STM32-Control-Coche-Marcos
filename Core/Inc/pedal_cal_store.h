/**
  ****************************************************************************
  * @file    pedal_cal_store.h
  * @brief   Persistent pedal-endpoint calibration storage (flash NVM)
  *
  * Stores the per-vehicle ADC counts for the released (MIN) and fully
  * pressed (MAX) accelerator pedal positions in flash, so that the
  * Pedal_RawToPercent() mapping is calibrated to the actual pedal /
  * voltage-divider hardware installed on this unit.
  *
  * Safety invariants:
  *   - Flash data alone NEVER unblocks startup_inhibit or authorises
  *     ACTIVE.  Calibration only affects the raw→% mapping.
  *   - If flash is blank, CRC-invalid, or out-of-range, the system
  *     falls back silently to compile-time defaults (150 / 2413).
  *     Boot is NEVER blocked by a missing/corrupt calibration slot.
  *   - All Pedal_Update() plausibility, EMA, rate-of-change, and
  *     FAULT_LO/HI gates remain unchanged and apply equally to the
  *     calibrated and default mappings.
  *
  * Flash layout:
  *   Page 124 (0x0807C000, 4 KB) — dedicated to pedal calibration.
  *   Single slot with magic + CRC32 integrity check.  Modelled 1:1 on
  *   steering_cal_store.c so the flash-write primitive set is identical.
  *
  * API:
  *   PedalCal_Init()          — load + validate flash slot at boot
  *   PedalCal_IsValid()       — query whether flash slot passed CRC/range
  *   PedalCal_GetStored()     — read stored adc_min / adc_max
  *   PedalCal_Save()          — persist new endpoints (validates first)
  *   PedalCal_Validate()      — pure validator (no flash access)
  ****************************************************************************
  */

#ifndef PEDAL_CAL_STORE_H
#define PEDAL_CAL_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---- Hard validation limits (mirror sensor_manager defaults) ----
 * adc_min must be at or above PEDAL_CAL_MIN_LIMIT to leave headroom
 * for the FAULT_LO open-wire detector.  adc_max must be at or below
 * PEDAL_CAL_MAX_LIMIT to stay clear of the FAULT_HI short-circuit
 * detector.  (adc_max - adc_min) must be >= PEDAL_CAL_RANGE_MIN so
 * the pedal % mapping has enough span to be useful.                 */
#define PEDAL_CAL_MIN_LIMIT    50U     /* adc_min >= 50              */
#define PEDAL_CAL_MAX_LIMIT    2600U   /* adc_max <= 2600            */
#define PEDAL_CAL_RANGE_MIN    800U    /* (adc_max - adc_min) >= 800 */

/* ---- Compile-time fallback endpoints ----
 * These mirror the PEDAL_ADC_MIN_DEFAULT / PEDAL_ADC_MAX_DEFAULT
 * constants in sensor_manager.c.  Re-exported here so callers that
 * need to persist or re-apply the defaults (e.g. the CAN RESET
 * sub-opcode handler) reference a single source of truth.            */
#define PEDAL_CAL_DEFAULT_MIN  150U
#define PEDAL_CAL_DEFAULT_MAX  2413U

/* ---- Defensive coherence checks ----------------------------------
 * Guarantee at compile time that the compile-time defaults satisfy
 * the same hard validation that PedalCal_Validate() enforces at
 * runtime.  Without these, a future tweak of any of the four macros
 * above could silently make PEDAL_CAL_OP_RESET_DEFAULTS impossible
 * (PedalCal_Save(150,2413) → Validate(150,2413) → false → ACK_REJECTED)
 * which would only surface as a runtime failure during service.    */
_Static_assert(PEDAL_CAL_DEFAULT_MIN >= PEDAL_CAL_MIN_LIMIT,
               "PEDAL_CAL_DEFAULT_MIN must clear PEDAL_CAL_MIN_LIMIT");
_Static_assert(PEDAL_CAL_DEFAULT_MAX <= PEDAL_CAL_MAX_LIMIT,
               "PEDAL_CAL_DEFAULT_MAX must stay below PEDAL_CAL_MAX_LIMIT");
_Static_assert((PEDAL_CAL_DEFAULT_MAX - PEDAL_CAL_DEFAULT_MIN)
                   >= PEDAL_CAL_RANGE_MIN,
               "Default pedal range below PEDAL_CAL_RANGE_MIN");

/**
 * @brief  Initialise the pedal calibration store module.
 *         Reads flash and validates CRC / magic / range.  Does NOT
 *         apply the calibration — the caller decides when to call
 *         Pedal_ApplyCalibration() with the values returned by
 *         PedalCal_GetStored().
 *
 *         On flash blank, CRC-invalid, or out-of-range, the module
 *         silently falls back to "no valid slot" — boot is never
 *         blocked.                                                    */
void PedalCal_Init(void);

/**
 * @brief  Returns true if the flash slot was present and passed all
 *         integrity + range checks at boot.                           */
bool PedalCal_IsValid(void);

/**
 * @brief  Read the stored adc_min / adc_max.  Only meaningful when
 *         PedalCal_IsValid() == true; otherwise outputs are zero.     */
void PedalCal_GetStored(uint16_t *adc_min, uint16_t *adc_max);

/**
 * @brief  Pure validator.  Returns true if (adc_min, adc_max) satisfy
 *         all hard limits without touching flash.  Used by both the
 *         flash loader and the CAN SAVE sub-opcode handler.           */
bool PedalCal_Validate(uint16_t adc_min, uint16_t adc_max);

/**
 * @brief  Persist new calibration endpoints to flash.
 *
 * Write conditions (caller is responsible for the safety gates):
 *   - Safety_GetState() == SYS_STATE_STANDBY
 *   - Startup_IsInhibited() == true
 *   - Pedal_GetPercent() < 3.0f
 *   - Pedal_IsPlausible() == true
 *   - All wheel speeds < 0.3 km/h
 *
 * Range conditions (validated internally):
 *   - PedalCal_Validate(adc_min, adc_max) must return true.
 *
 * @retval true on success, false on validation or flash error.
 */
bool PedalCal_Save(uint16_t adc_min, uint16_t adc_max);

#ifdef __cplusplus
}
#endif

#endif /* PEDAL_CAL_STORE_H */
