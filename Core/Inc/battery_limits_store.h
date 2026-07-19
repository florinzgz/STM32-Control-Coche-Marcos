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
  * Parameters (seeded for the installed 16.0–28.5 V traction pack):
  *
  *   - LowVWarning (V) : informational low-voltage warning point,
  *                       default 18.00
  *   - LowVLimit   (V) : derate (DEGRADED) threshold used by the safety
  *                       state machine, default 17.00
  *   - LowVCutoff  (V) : SAFE cutoff threshold, default 16.00
  *   - RecoveryV   (V) : SAFE -> STANDBY recovery threshold, default 17.00
  *   - VoltageFilter (ms): EMA/debounce time constant applied before the
  *                         compares, default 500 ms
  *
  * Safety invariants:
  *   - The safety STATE MACHINE is NOT modified; only the threshold VALUES
  *     it compares against become runtime variables.
  *   - Flash data alone NEVER authorises ACTIVE or clears startup_inhibit.
  *   - On flash blank / CRC-invalid / out-of-range the module silently falls
  *     back to the defaults below.  Boot is NEVER blocked.
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
 * The over-voltage warning point stays a compile-time protection limit.
 * It is mirrored here so validators can enforce Warning/Limit <= OV. */
#define BATT_OV_WARNING_CV     3000U  /* 30.00 V — BATTERY_OV_WARNING_V */

/* ---- Hard validation ranges (centivolts / ms) ----------------------
 * Bounds cover the installed pack while preventing a malformed command from
 * disabling undervoltage protection completely. */
#define BATT_WARNING_MIN_CV    1500U  /* 15.00 V */
#define BATT_WARNING_MAX_CV    3000U  /* 30.00 V (<= OV warning) */
#define BATT_LIMIT_MIN_CV      1500U  /* 15.00 V */
#define BATT_LIMIT_MAX_CV      3000U  /* 30.00 V (<= OV warning) */
#define BATT_CUTOFF_MIN_CV     1400U  /* 14.00 V */
#define BATT_CUTOFF_MAX_CV     2400U  /* 24.00 V */
#define BATT_RECOVERY_MIN_CV   1400U  /* 14.00 V */
#define BATT_RECOVERY_MAX_CV   2900U  /* 29.00 V */
#define BATT_FILTER_MIN_MS     0U
#define BATT_FILTER_MAX_MS     5000U

/* ---- Installed-pack defaults ---------------------------------------
 * Visual battery range on the ESP32 is 16.00 V = 0 % and 28.50 V = 100 %.
 * Protection thresholds intentionally retain margin above the 16.00 V floor. */
#define BATT_WARNING_DEFAULT_CV    1800U
#define BATT_LIMIT_DEFAULT_CV      1700U
#define BATT_CUTOFF_DEFAULT_CV     1600U
#define BATT_RECOVERY_DEFAULT_CV   1700U
#define BATT_FILTER_DEFAULT_MS      500U

/* ---- Defensive coherence checks (defaults must satisfy validators) */
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

typedef struct {
    uint16_t warning_cv;    /* low-voltage warning point */
    uint16_t limit_cv;      /* derate (DEGRADED) threshold */
    uint16_t cutoff_cv;     /* SAFE cutoff threshold */
    uint16_t recovery_cv;   /* SAFE -> STANDBY recovery threshold */
    uint16_t filter_ms;     /* EMA/debounce time constant (0 = off) */
} BatteryLimits_t;

void BatteryLimitsStore_Init(void);
bool BatteryLimitsStore_IsValid(void);
void BatteryLimitsStore_GetStored(BatteryLimits_t *out);
void BatteryLimitsStore_GetDefaults(BatteryLimits_t *out);

/**
 * @brief Pure validator.  Returns true iff every field is inside its hard
 *        range and the coherence rules hold:
 *        Warning > Cutoff, Limit > Cutoff, Recovery > Cutoff,
 *        Warning <= OV, Limit <= OV.
 */
bool BatteryLimitsStore_Validate(const BatteryLimits_t *b);

/**
 * @brief Persist a new parameter set to flash.
 *
 * Write conditions:
 *   - STANDBY, or a verified stationary service condition in ACTIVE/DEGRADED
 *     (pedal released, P/N, all wheels stopped and final PWM zero).
 *   - The set passes BatteryLimitsStore_Validate().
 *   - Flash-wear guards: no-op elision when unchanged + minimum interval.
 */
bool BatteryLimitsStore_Save(const BatteryLimits_t *b);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_LIMITS_STORE_H */
