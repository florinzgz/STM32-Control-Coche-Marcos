/**
  ****************************************************************************
  * @file    drive_tuning_store.h
  * @brief   Persistent drive-tuning storage (NVM) — single source of truth
  *
  * Stores the runtime-tunable traction "feel" parameters in flash so they
  * can be adjusted from the ESP32-HMI Engineering menu without re-flashing
  * firmware.  Modelled 1:1 on gear_limits_store.c / pedal_cal_store.c.
  *
  * Parameters (each seeded with the historic compile-time value so a unit
  * with no / blank / corrupt slot behaves EXACTLY like the original firmware):
  *
  *   - AccelRamp   (%/s)  : pedal ramp-up   rate          default 50
  *                          (was PEDAL_RAMP_UP_PCT_S   = 50.0f)
  *   - BrakeRamp   (%/s)  : pedal ramp-down rate          default 100
  *                          (was PEDAL_RAMP_DOWN_PCT_S = 100.0f)
  *   - ReverseRamp (%/s)  : pedal ramp-up rate used ONLY in GEAR_REVERSE,
  *                          default 50 (== AccelRamp, so reverse is unchanged)
  *   - CreepEnable (0/1)  : enable motor dead-zone (creep) compensation,
  *                          default 1 (ON, as today)
  *   - CreepPower  (%)    : dead-zone floor PWM% when creep is engaged,
  *                          default 8 (was MOTOR_DEADZONE_PCT = 8.0f)
  *   - CreepDelay  (ms)   : delay from pedal-leaves-0% before the creep
  *                          floor is (re)applied, default 0 (no delay → the
  *                          floor is applied immediately, exactly as today)
  *
  * Safety invariants (mirrors gear_limits_store.c):
  *   - Flash data alone NEVER authorises ACTIVE or clears startup_inhibit;
  *     it only shapes an already-validated traction demand.
  *   - The ramp rates can only limit (smooth) the demand; the dead-zone
  *     floor can only raise the *minimum* drive PWM, never the maximum.
  *   - On flash blank / CRC-invalid / out-of-range the module silently
  *     falls back to "no valid slot" and the caller keeps the compile-time
  *     defaults below.  Boot is NEVER blocked by a missing / corrupt slot.
  *
  * Flash layout:
  *   Page 121 (0x08079000, 4 KB) — dedicated to drive tuning.
  *   Single 16-byte slot with magic "DTN1" + CRC32 integrity check.
  ****************************************************************************
  */

#ifndef DRIVE_TUNING_STORE_H
#define DRIVE_TUNING_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---- Hard validation ranges ----------------------------------------
 * Single source of truth shared by the flash loader, motor_control.c and
 * the CAN service handler.  All ramp rates must be strictly > 0 (FASE 4).
 * Ranges are deliberately wide enough for tuning but bounded so a buggy or
 * malicious caller can never disable smoothing (ramp 0) or push the creep
 * floor past a safe walking-pace torque.                                 */
#define DRIVE_ACCEL_RAMP_MIN     1U     /* %/s — must be > 0              */
#define DRIVE_ACCEL_RAMP_MAX     200U   /* %/s                            */
#define DRIVE_BRAKE_RAMP_MIN     1U     /* %/s — must be > 0              */
#define DRIVE_BRAKE_RAMP_MAX     200U   /* %/s                            */
#define DRIVE_REVERSE_RAMP_MIN   1U     /* %/s — must be > 0              */
#define DRIVE_REVERSE_RAMP_MAX   200U   /* %/s                            */
#define DRIVE_CREEP_POWER_MIN    0U     /* % — 0 disables the floor       */
#define DRIVE_CREEP_POWER_MAX    20U    /* % — firmware cap (tope)        */
#define DRIVE_CREEP_DELAY_MIN    0U     /* ms                             */
#define DRIVE_CREEP_DELAY_MAX    5000U  /* ms                             */

/* ---- Compile-time defaults — MIRROR the historic firmware behaviour --
 *   AccelRamp   = PEDAL_RAMP_UP_PCT_S   = 50
 *   BrakeRamp   = PEDAL_RAMP_DOWN_PCT_S = 100
 *   ReverseRamp = AccelRamp             = 50  (reverse ramp-up unchanged)
 *   CreepEnable = 1 (dead-zone compensation always on, as today)
 *   CreepPower  = MOTOR_DEADZONE_PCT    = 8
 *   CreepDelay  = 0 ms (floor applied immediately, as today)
 * Keeping the defaults identical guarantees behaviour is unchanged until
 * the operator explicitly re-tunes.                                      */
#define DRIVE_ACCEL_RAMP_DEFAULT     50U
#define DRIVE_BRAKE_RAMP_DEFAULT     100U
#define DRIVE_REVERSE_RAMP_DEFAULT   50U
#define DRIVE_CREEP_ENABLE_DEFAULT   1U
#define DRIVE_CREEP_POWER_DEFAULT    8U
#define DRIVE_CREEP_DELAY_DEFAULT    0U

/* ---- Defensive coherence checks (defaults must be inside the ranges) - */
_Static_assert(DRIVE_ACCEL_RAMP_DEFAULT >= DRIVE_ACCEL_RAMP_MIN &&
               DRIVE_ACCEL_RAMP_DEFAULT <= DRIVE_ACCEL_RAMP_MAX,
               "AccelRamp default out of range");
_Static_assert(DRIVE_BRAKE_RAMP_DEFAULT >= DRIVE_BRAKE_RAMP_MIN &&
               DRIVE_BRAKE_RAMP_DEFAULT <= DRIVE_BRAKE_RAMP_MAX,
               "BrakeRamp default out of range");
_Static_assert(DRIVE_REVERSE_RAMP_DEFAULT >= DRIVE_REVERSE_RAMP_MIN &&
               DRIVE_REVERSE_RAMP_DEFAULT <= DRIVE_REVERSE_RAMP_MAX,
               "ReverseRamp default out of range");
_Static_assert(DRIVE_CREEP_POWER_DEFAULT >= DRIVE_CREEP_POWER_MIN &&
               DRIVE_CREEP_POWER_DEFAULT <= DRIVE_CREEP_POWER_MAX,
               "CreepPower default out of range");
_Static_assert(DRIVE_CREEP_DELAY_DEFAULT >= DRIVE_CREEP_DELAY_MIN &&
               DRIVE_CREEP_DELAY_DEFAULT <= DRIVE_CREEP_DELAY_MAX,
               "CreepDelay default out of range");
/* Ramp rates can never be zero — that would defeat the rate limiter. */
_Static_assert(DRIVE_ACCEL_RAMP_MIN >= 1U &&
               DRIVE_BRAKE_RAMP_MIN >= 1U &&
               DRIVE_REVERSE_RAMP_MIN >= 1U,
               "ramp rates must be strictly positive");

/**
 * @brief  Runtime drive-tuning parameter set.  Plain POD so it can be
 *         passed by value between the store, motor_control.c and the CAN
 *         handler without dynamic allocation.
 */
typedef struct {
    uint8_t  accel_ramp;    /* %/s ramp-up                                */
    uint8_t  brake_ramp;    /* %/s ramp-down                              */
    uint8_t  reverse_ramp;  /* %/s ramp-up in GEAR_REVERSE only           */
    uint8_t  creep_enable;  /* 0 = off, 1 = on                            */
    uint8_t  creep_power;   /* % dead-zone floor                          */
    uint16_t creep_delay;   /* ms delay before applying the creep floor   */
} DriveTuning_t;

/**
 * @brief  Initialise the drive-tuning store.  Reads flash and validates
 *         CRC / magic / range.  Does NOT apply the values — the caller
 *         (main.c) applies them via Traction_SetDriveTuning().  On flash
 *         blank / CRC-invalid / out-of-range, falls back silently to "no
 *         valid slot".  Boot is never blocked.                            */
void DriveTuningStore_Init(void);

/** @brief  True if the flash slot passed all integrity + range checks. */
bool DriveTuningStore_IsValid(void);

/** @brief  Read the stored parameter set.  Only meaningful when
 *          DriveTuningStore_IsValid() == true; otherwise outputs zero.   */
void DriveTuningStore_GetStored(DriveTuning_t *out);

/** @brief  Fill @p out with the compile-time defaults (== historic firmware). */
void DriveTuningStore_GetDefaults(DriveTuning_t *out);

/**
 * @brief  Pure validator.  Returns true iff every field satisfies the hard
 *         ranges above (FASE 4).  Touches no flash.  Shared by the flash
 *         loader and the CAN SET/SAVE handler.                            */
bool DriveTuningStore_Validate(const DriveTuning_t *t);

/**
 * @brief  Persist a new parameter set to flash.
 *
 * Write conditions:
 *   - Safety_GetState() == SYS_STATE_STANDBY (re-asserted internally).
 *   - The set must pass DriveTuningStore_Validate().
 *   - Flash-wear guards: no-op elision when unchanged + minimum interval.
 *
 * @retval true on success, false on validation / safety / flash error.
 */
bool DriveTuningStore_Save(const DriveTuning_t *t);

#ifdef __cplusplus
}
#endif

#endif /* DRIVE_TUNING_STORE_H */
