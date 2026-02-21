/**
  ****************************************************************************
  * @file    steering_cal_store.h
  * @brief   Persistent steering calibration storage (flash NVM)
  *
  * Stores the encoder count at the calibrated center position in flash
  * so that a centering sweep can be skipped on power-up when the
  * steering has not moved since the last shutdown.
  *
  * Safety invariant:
  *   Flash data alone NEVER authorises ACTIVE.  The physical center
  *   sensor must confirm plausibility at boot before the stored
  *   calibration is accepted.
  *
  * Flash layout:
  *   Page 126 (0x0807E000, 4 KB) — dedicated to steering calibration.
  *   Single slot with magic + CRC32 integrity check.
  *
  * API:
  *   SteeringCal_Init()            — load from flash (if valid)
  *   SteeringCal_Save()            — persist current calibration
  *   SteeringCal_ValidateAtBoot()  — compare stored vs live encoder +
  *                                   center-sensor agreement
  *   SteeringCal_IsRestoredValid() — query whether boot validation passed
  *   SteeringCal_GetStoredCenter() — read stored encoder center value
  ****************************************************************************
  */

#ifndef __STEERING_CAL_STORE_H
#define __STEERING_CAL_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---- Tolerance for encoder comparison at boot ----
 * If |current_encoder - stored_center| <= this value, the position
 * is considered unchanged since last shutdown.
 * 100 counts ≈ 7.5° of encoder shaft at 4800 CPR — allows for
 * minor thermal / mechanical drift while catching any deliberate
 * or accidental wheel movement.                                    */
#define STEERING_CAL_TOLERANCE_COUNTS  100

/**
 * @brief  Initialise the calibration store module.
 *         Reads flash and validates CRC / magic.  Does NOT perform
 *         boot validation (call SteeringCal_ValidateAtBoot for that).
 */
void SteeringCal_Init(void);

/**
 * @brief  Persist the current encoder center value to flash.
 *
 * Write conditions (caller must ensure):
 *   - Centering has finished successfully
 *   - Vehicle speed == 0
 *   - No safety errors
 *
 * @param  encoder_count_at_center  The TIM2 count at calibrated center.
 * @retval true on success, false on flash write error.
 */
bool SteeringCal_Save(int32_t encoder_count_at_center);

/**
 * @brief  Boot-time validation of stored calibration.
 *
 * Reads the current encoder counter and the center-sensor pin state,
 * then compares against the flash-stored center value.
 *
 * Rules:
 *   - Flash slot must be valid (magic + CRC)
 *   - |current_encoder - stored_center| <= STEERING_CAL_TOLERANCE_COUNTS
 *   - Physical center sensor must detect the screw (pin LOW with pull-up)
 *
 * @retval true  if all checks pass — centering sweep may be skipped.
 * @retval false if any check fails — normal centering required.
 */
bool SteeringCal_ValidateAtBoot(void);

/**
 * @brief  Query whether the last call to SteeringCal_ValidateAtBoot()
 *         returned true (i.e. stored calibration is usable).
 */
bool SteeringCal_IsRestoredValid(void);

/**
 * @brief  Get the stored encoder center value.
 *         Only meaningful when SteeringCal_IsRestoredValid() == true.
 */
int32_t SteeringCal_GetStoredCenter(void);

#ifdef __cplusplus
}
#endif

#endif /* __STEERING_CAL_STORE_H */
