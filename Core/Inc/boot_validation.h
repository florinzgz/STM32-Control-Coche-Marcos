/**
  ****************************************************************************
  * @file    boot_validation.h
  * @brief   Boot validation checklist — deterministic pre-ACTIVE gate
  *
  *  Implements a structured boot validation sequence executed during
  *  STANDBY.  All sensor subsystems must pass plausibility checks
  *  before the system is allowed to transition to ACTIVE.
  *
  *  Checks performed:
  *    - Temperature sensors within plausible range
  *    - Current sensors within plausible range
  *    - Encoder health valid (no fault)
  *    - Battery voltage above warning threshold
  *    - No persistent SAFETY_ERROR present
  *    - CAN bus not in bus-off state
  *
  *  Design constraints:
  *    - No blocking delays
  *    - No CAN contract changes
  *    - No watchdog configuration changes
  *    - No safety state machine transition changes
  ****************************************************************************
  */

#ifndef __BOOT_VALIDATION_H
#define __BOOT_VALIDATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---- Boot validation check flags ---- */
#define BOOT_CHECK_TEMP_PLAUSIBLE      (1U << 0)
#define BOOT_CHECK_CURRENT_PLAUSIBLE   (1U << 1)
#define BOOT_CHECK_ENCODER_HEALTHY     (1U << 2)
#define BOOT_CHECK_BATTERY_OK          (1U << 3)
#define BOOT_CHECK_NO_SAFETY_ERROR     (1U << 4)
#define BOOT_CHECK_CAN_NOT_BUSOFF      (1U << 5)

#define BOOT_CHECK_ALL_MASK  (BOOT_CHECK_TEMP_PLAUSIBLE    | \
                              BOOT_CHECK_CURRENT_PLAUSIBLE | \
                              BOOT_CHECK_ENCODER_HEALTHY   | \
                              BOOT_CHECK_BATTERY_OK        | \
                              BOOT_CHECK_NO_SAFETY_ERROR   | \
                              BOOT_CHECK_CAN_NOT_BUSOFF)

/* ---- Boot validation status ---- */
typedef struct {
    uint8_t  checks_passed;   /* Bitmask of passed checks (BOOT_CHECK_*) */
    uint8_t  checks_failed;   /* Bitmask of failed checks                */
    bool     validated;        /* true when all checks pass               */
} BootValidationStatus;

/**
 * @brief  Execute the boot validation checklist.
 *
 *         Non-blocking.  Reads current sensor values and safety state
 *         to evaluate each check.  Should be called periodically during
 *         STANDBY (e.g. from the 10 ms task loop).
 *
 *         Results are stored internally and queried via
 *         BootValidation_IsPassed().
 */
void BootValidation_Run(void);

/**
 * @brief  Query whether all boot validation checks have passed.
 * @return true if every check in the checklist passed on the last
 *         call to BootValidation_Run(), false otherwise.
 */
bool BootValidation_IsPassed(void);

/**
 * @brief  Get a read-only pointer to the current validation status.
 * @return Pointer to the internal BootValidationStatus struct.
 */
const BootValidationStatus* BootValidation_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_VALIDATION_H */
