/**
  ****************************************************************************
  * @file    tcs_tuning_store.h
  * @brief   Persistent TCS (anti-slip) tuning storage (NVM) — service diag C1
  *
  * Stores the runtime-tunable traction-control (TCS) parameters in flash so
  * they can be adjusted from the ESP32-HMI SERVICE_AUTOTEST menu without
  * re-flashing firmware.  Modelled 1:1 on battery_limits_store.c.
  *
  * These were compile-time macros in drive_dynamics_policy.h:
  *   DRIVE_TCS_MIN_REFERENCE_KMH    6.0   range  2..15
  *   DRIVE_TCS_SLIP_THRESHOLD_PCT  25.0   range 10..60
  *   DRIVE_TCS_INITIAL_REDUCTION    0.40  range 0.10..0.80
  *   DRIVE_TCS_REDUCTION_RATE_PER_S 1.00  range 0.20..3.00
  *   DRIVE_TCS_RECOVERY_RATE_PER_S  0.25  range 0.05..2.00
  *   DRIVE_TCS_MAX_REDUCTION        0.80  range 0.30..0.95
  *
  * The macros in drive_dynamics_policy.h remain the compile-time DEFAULTS
  * and are still asserted verbatim by CI (pr436-field-fixes.yml).  The
  * DriveDynamics_Tcs*() inline functions were changed to accept the
  * effective values as explicit parameters instead of reading the macros
  * directly, so this store's Get*() accessors supply the live (possibly
  * service-tuned) values to tcs_tuned.c while staying pure/header-only for
  * host tests that still exercise the macro defaults.
  *
  * Safety invariants:
  *   - Editable while a service-diag test is ACTIVE (staged in RAM only);
  *     SAVE persists, matching the "measure -> adjust -> measure" loop
  *     required by the service self-test (Block C rules).
  *   - PROHIBITED: any per-wheel trim/compensation.  These six values are
  *     GLOBAL to the TCS policy, applied identically to all four wheels.
  *   - On flash blank / CRC-invalid / out-of-range the module silently
  *     falls back to the compile-time defaults.  Boot is NEVER blocked.
  *
  * Flash layout:
  *   Page 112 (0x08070000, 4 KB) — dedicated to TCS tuning.
  *   Single slot with magic "TCS1" + CRC32 integrity check.
  ****************************************************************************
  */

#ifndef TCS_TUNING_STORE_H
#define TCS_TUNING_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "drive_dynamics_policy.h"

/* ---- Hard validation ranges (matches the problem statement) ---- */
#define TCS_TUNE_MIN_REFERENCE_KMH_MIN     2.0f
#define TCS_TUNE_MIN_REFERENCE_KMH_MAX    15.0f
#define TCS_TUNE_SLIP_THRESHOLD_PCT_MIN   10.0f
#define TCS_TUNE_SLIP_THRESHOLD_PCT_MAX   60.0f
#define TCS_TUNE_INITIAL_REDUCTION_MIN     0.10f
#define TCS_TUNE_INITIAL_REDUCTION_MAX     0.80f
#define TCS_TUNE_REDUCTION_RATE_MIN        0.20f
#define TCS_TUNE_REDUCTION_RATE_MAX        3.00f
#define TCS_TUNE_RECOVERY_RATE_MIN         0.05f
#define TCS_TUNE_RECOVERY_RATE_MAX         2.00f
#define TCS_TUNE_MAX_REDUCTION_MIN         0.30f
#define TCS_TUNE_MAX_REDUCTION_MAX         0.95f

typedef struct {
    float min_reference_kmh;
    float slip_threshold_pct;
    float initial_reduction;
    float reduction_rate_per_s;
    float recovery_rate_per_s;
    float max_reduction;
} TcsTuning_t;

/* HAL-free single source used by production and host tests. */
static inline bool TcsTuning_ValidateValues(const TcsTuning_t *t)
{
    if (!t) return false;
    if (t->min_reference_kmh < TCS_TUNE_MIN_REFERENCE_KMH_MIN ||
        t->min_reference_kmh > TCS_TUNE_MIN_REFERENCE_KMH_MAX) return false;
    if (t->slip_threshold_pct < TCS_TUNE_SLIP_THRESHOLD_PCT_MIN ||
        t->slip_threshold_pct > TCS_TUNE_SLIP_THRESHOLD_PCT_MAX) return false;
    if (t->initial_reduction < TCS_TUNE_INITIAL_REDUCTION_MIN ||
        t->initial_reduction > TCS_TUNE_INITIAL_REDUCTION_MAX) return false;
    if (t->reduction_rate_per_s < TCS_TUNE_REDUCTION_RATE_MIN ||
        t->reduction_rate_per_s > TCS_TUNE_REDUCTION_RATE_MAX) return false;
    if (t->recovery_rate_per_s < TCS_TUNE_RECOVERY_RATE_MIN ||
        t->recovery_rate_per_s > TCS_TUNE_RECOVERY_RATE_MAX) return false;
    if (t->max_reduction < TCS_TUNE_MAX_REDUCTION_MIN ||
        t->max_reduction > TCS_TUNE_MAX_REDUCTION_MAX) return false;
    if (t->max_reduction < t->initial_reduction) return false;
    return true;
}

void TcsTuningStore_Init(void);
bool TcsTuningStore_IsValid(void);
void TcsTuningStore_GetDefaults(TcsTuning_t *out);
bool TcsTuningStore_Validate(const TcsTuning_t *t);

/** Effective (RAM-staged) values actually consumed by tcs_tuned.c. */
float TcsTuningStore_GetMinReferenceKmh(void);
float TcsTuningStore_GetSlipThresholdPct(void);
float TcsTuningStore_GetInitialReduction(void);
float TcsTuningStore_GetReductionRatePerS(void);
float TcsTuningStore_GetRecoveryRatePerS(void);
float TcsTuningStore_GetMaxReduction(void);

/**
 * @brief Stage a candidate set in RAM only (no flash write).  Used by the
 *        service-diag SET_* sub-opcode so the operator can run the
 *        measure -> adjust -> measure loop without touching flash.
 *        C1 is editable while a test is ACTIVE (traction/geometry/wheel
 *        sensors are the only stores with this permission).
 */
bool TcsTuningStore_Stage(const TcsTuning_t *t);

/** Current staged (RAM) values, whether saved or not. */
void TcsTuningStore_GetStaged(TcsTuning_t *out);

/** Discards any unsaved staged edits and reverts to the persisted (or
 *  default, if flash is empty/invalid) values.  Used on session abort/exit
 *  without SAVE. */
void TcsTuningStore_Revert(void);

/**
 * @brief Persist the currently staged parameter set to flash.
 * Write conditions: identical write-rate-limit / no-op-elision guard as
 * battery_limits_store.c.  No system-state gate here: C1 is one of the
 * parameters explicitly editable DURING an active service-diag test
 * (Block C rule); the service session itself gates entry.
 */
bool TcsTuningStore_Save(void);

/** Restores compile-time defaults into the RAM stage (does not write flash
 *  until TcsTuningStore_Save() is called) -- the "restaurar defaults de
 *  este test" button. */
void TcsTuningStore_ResetToDefaults(void);

#ifdef __cplusplus
}
#endif

#endif /* TCS_TUNING_STORE_H */
