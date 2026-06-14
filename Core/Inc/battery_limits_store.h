/**
  ****************************************************************************
  * @file    battery_limits_store.h
  * @brief   Persistent battery-limit storage (NVM) — single source of truth
  *
  * Stores the runtime-tunable battery-voltage protection thresholds in flash
  * so they can be adjusted from the ESP32-HMI Engineering menu without
  * re-flashing firmware.  Modelled 1:1 on gear_limits_store.c.
  *
  * Voltages are stored as centivolts (V x 100) in a uint16 so the on-flash
  * slot stays integer-only and CRC-friendly; safety_system.c converts to
  * volts (float) at the point of use.
  *
  * Parameters (each seeded with the historic compile-time value so a unit
  * with no / blank / corrupt slot behaves EXACTLY like the original firmware):
  *
  *   - LowVWarning (V) : informational low-voltage warning point,
  *                       default 20.00 (telemetry/aviso only)
  *   - LowVLimit   (V) : derate (DEGRADED) threshold actually used by the
  *                       safety state machine, default 20.00
  *                       (was BATTERY_UV_WARNING_V = 20.0f)
  *   - LowVCutoff  (V) : SAFE cutoff threshold, default 18.00
  *                       (was BATTERY_UV_CRITICAL_V = 18.0f)
  *   - RecoveryV   (V) : SAFE -> STANDBY recovery threshold, default 18.50
  *                       (was BATTERY_UV_CRITICAL_V + BATTERY_UV_HYST_V)
  *   - VoltageFilter (ms): optional EMA/debounce time constant applied
  *                       before the compares, default 0 (no filtering →
  *                       byte-for-byte identical behaviour to today)
  *
  * Safety invariants:
  *   - The safety STATE MACHINE is NOT modified; only the threshold VALUES
  *     it compares against become runtime variables.  With the defaults the
  *     comparisons reproduce the historic #define values exactly.
  *   - Flash data alone NEVER authorises ACTIVE or clears startup_inhibit.
  *   - On flash blank / CRC-invalid / out-of-range the module silently falls
  *     back to "no valid slot" and the caller keeps the compile-time
  *     defaults below.  Boot is NEVER blocked.
  *
  * Flash layout:
  *   Page 120 (0x08078000, 4 KB) — dedicated to battery limits.
  *   Single 24-byte slot with magic "BAT1" + CRC32 integrity check.
  ****************************************************************************
  */

#ifndef BATTERY_LIMITS_STORE_H
#define BATTERY_LIMITS_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---- Fixed reference points (NOT runtime-configurable) -------------
 * The over-voltage warning point and the under-voltage hysteresis band
 * stay compile-time constants (they live in safety_system.c).  They are
 * mirrored here so the validators can enforce "Warning <= OV".          */
#define BATT_OV_WARNING_CV     3000U  /* 30.00 V — BATTERY_OV_WARNING_V   */

/* ---- Hard validation ranges (centivolts / ms) ----------------------
 * Single source of truth shared by the flash loader, safety_system.c and
 * the CAN service handler.  Bounds keep the thresholds inside physically
 * sane 6S Li-ion limits so a buggy / malicious caller cannot disable the
 * under-voltage protection.                                              */
#define BATT_WARNING_MIN_CV    1500U  /* 15.00 V                          */
#define BATT_WARNING_MAX_CV    3000U  /* 30.00 V (<= OV warning)          */
#define BATT_LIMIT_MIN_CV      1500U  /* 15.00 V                          */
#define BATT_LIMIT_MAX_CV      3000U  /* 30.00 V (<= OV warning)          */
#define BATT_CUTOFF_MIN_CV     1400U  /* 14.00 V                          */
#define BATT_CUTOFF_MAX_CV     2400U  /* 24.00 V                          */
#define BATT_RECOVERY_MIN_CV   1400U  /* 14.00 V                          */
#define BATT_RECOVERY_MAX_CV   2900U  /* 29.00 V                          */
#define BATT_FILTER_MIN_MS     0U
#define BATT_FILTER_MAX_MS     5000U

/* ---- Compile-time defaults — MIRROR the historic firmware behaviour --
 *   LowVWarning   = 20.00 V
 *   LowVLimit     = 20.00 V  (BATTERY_UV_WARNING_V — derate point)
 *   LowVCutoff    = 18.00 V  (BATTERY_UV_CRITICAL_V)
 *   RecoveryV     = 18.50 V  (BATTERY_UV_CRITICAL_V + BATTERY_UV_HYST_V)
 *   VoltageFilter = 0 ms     (no filtering)                              */
#define BATT_WARNING_DEFAULT_CV    2000U
#define BATT_LIMIT_DEFAULT_CV      2000U
#define BATT_CUTOFF_DEFAULT_CV     1800U
#define BATT_RECOVERY_DEFAULT_CV   1850U
#define BATT_FILTER_DEFAULT_MS     0U

/* ---- Defensive coherence checks (defaults must satisfy the validators) */
_Static_assert(BATT_WARNING_DEFAULT_CV  >  BATT_CUTOFF_DEFAULT_CV,
               "default Warning must be > Cutoff");
_Static_assert(BATT_LIMIT_DEFAULT_CV    >  BATT_CUTOFF_DEFAULT_CV,
               "default Limit must be > Cutoff");
_Static_assert(BATT_RECOVERY_DEFAULT_CV >  BATT_CUTOFF_DEFAULT_CV,
               "default Recovery must be > Cutoff");
_Static_assert(BATT_WARNING_DEFAULT_CV  <= BATT_OV_WARNING_CV,
               "default Warning must be <= OV warning");
_Static_assert(BATT_LIMIT_DEFAULT_CV    <= BATT_OV_WARNING_CV,
               "default Limit must be <= OV warning");

/**
 * @brief  Runtime battery-limit parameter set (centivolts / ms).  Plain POD.
 */
typedef struct {
    uint16_t warning_cv;    /* low-voltage warning (aviso) point          */
    uint16_t limit_cv;      /* derate (DEGRADED) threshold                */
    uint16_t cutoff_cv;     /* SAFE cutoff threshold                      */
    uint16_t recovery_cv;   /* SAFE -> STANDBY recovery threshold         */
    uint16_t filter_ms;     /* EMA/debounce time constant (0 = off)       */
} BatteryLimits_t;

/**
 * @brief  Initialise the battery-limits store.  Reads flash and validates
 *         CRC / magic / range.  Does NOT apply the values — the caller
 *         (main.c) applies them via Safety_SetBatteryLimits().  On flash
 *         blank / CRC-invalid / out-of-range, falls back silently.        */
void BatteryLimitsStore_Init(void);

/** @brief  True if the flash slot passed all integrity + range checks. */
bool BatteryLimitsStore_IsValid(void);

/** @brief  Read the stored parameter set.  Only meaningful when
 *          BatteryLimitsStore_IsValid() == true; otherwise outputs zero.  */
void BatteryLimitsStore_GetStored(BatteryLimits_t *out);

/** @brief  Fill @p out with the compile-time defaults (== historic firmware). */
void BatteryLimitsStore_GetDefaults(BatteryLimits_t *out);

/**
 * @brief  Pure validator (FASE 4).  Returns true iff every field is inside
 *         its hard range AND the coherence rules hold:
 *           Warning > Cutoff, Limit > Cutoff, Recovery > Cutoff,
 *           Warning <= OV, Limit <= OV.
 *         Touches no flash.  Shared by the flash loader and CAN handler.  */
bool BatteryLimitsStore_Validate(const BatteryLimits_t *b);

/**
 * @brief  Persist a new parameter set to flash.
 *
 * Write conditions:
 *   - Safety_GetState() == SYS_STATE_STANDBY (re-asserted internally).
 *   - The set must pass BatteryLimitsStore_Validate().
 *   - Flash-wear guards: no-op elision when unchanged + minimum interval.
 *
 * @retval true on success, false on validation / safety / flash error.
 */
bool BatteryLimitsStore_Save(const BatteryLimits_t *b);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_LIMITS_STORE_H */
