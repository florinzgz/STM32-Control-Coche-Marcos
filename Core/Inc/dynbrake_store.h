/**
  ****************************************************************************
  * @file    dynbrake_store.h
  * @brief   Persistent dynamic-braking tuning storage (NVM) — single source
  *          of truth
  *
  * Stores the runtime-tunable dynamic-braking ("engine braking" on pedal
  * release) parameters in flash so they can be adjusted from the ESP32-HMI
  * Engineering menu without re-flashing firmware.  Modelled 1:1 on
  * drive_tuning_store.c / gear_limits_store.c.
  *
  * Root cause addressed (instant pedal response without jerk on release):
  * the historic compile-time constants in motor_control.c
  * (DYNBRAKE_FACTOR=0.5, DYNBRAKE_MAX_PCT=60, DYNBRAKE_RAMP_DOWN_PCT_S=80)
  * were not field-adjustable, and the brake ENTRY (rise) had no dedicated
  * rate limit — it could jump straight to the clamped target in a single
  * cycle.  This store adds gentler defaults AND a new `ramp_up` field so the
  * entry is slope-limited exactly like the release.
  *
  * Parameters (each seeded with a SOFTER default than the historic firmware
  * so a unit with no / blank / corrupt slot brakes less aggressively than
  * before, never more):
  *
  *   - Factor    (x100, 0.00-1.00) : brake % per throttle-%/s of demand fall,
  *                                   default 0.20 (was DYNBRAKE_FACTOR=0.50)
  *   - MaxPct    (0-60)            : maximum opposing brake torque (%),
  *                                   default 25 (was DYNBRAKE_MAX_PCT=60)
  *   - MinSpeed  (x10 deci-km/h, 0.0-10.0) : disable below this speed,
  *                                   default 3.0 (unchanged)
  *   - RampDown  (%/s, 20-200)     : max brake RELEASE rate,
  *                                   default 80 (unchanged)
  *   - RampUp    (%/s, 20-200)     : max brake APPLICATION rate (NEW) —
  *                                   the fix that removes the jerk on a
  *                                   sudden pedal release, default 60
  *   - Enable    (0/1)             : 0 disables dynamic braking completely
  *                                   (vehicle coasts freely on release),
  *                                   default 1
  *
  * Safety invariants (mirrors drive_tuning_store.c / gear_limits_store.c):
  *   - Flash data alone NEVER authorises ACTIVE or clears startup_inhibit;
  *     it only shapes an already-validated traction demand (opposing torque
  *     magnitude and its slew rate).
  *   - Factor==0 or Enable==0 fully disables dynamic braking — no other code
  *     path may reintroduce it.
  *   - On flash blank / CRC-invalid / out-of-range the module silently falls
  *     back to "no valid slot" and the caller keeps the compile-time
  *     defaults below.  Boot is NEVER blocked.
  *
  * Flash layout:
  *   Page 111 (0x0806F000, 4 KB) — dedicated to dynamic-braking tuning.
  *   Single 16-byte slot with magic "DYB1" + CRC32 integrity check.
  ****************************************************************************
  */

#ifndef DYNBRAKE_STORE_H
#define DYNBRAKE_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---- Hard validation ranges ----------------------------------------
 * Single source of truth shared by the flash loader, motor_control.c and
 * the CAN service handler.  Ramp rates must be strictly > 0 (mirrors the
 * FASE 4 rule already applied to drive_tuning_store) so an operator can
 * never fully disable the slew limiter (which would reopen the jerk this
 * store exists to close).  Enable=0 or Factor=0 is the SUPPORTED way to
 * fully disable dynamic braking (coast), not ramp=0.                     */
#define DYNBRAKE_FACTOR_PCT_MIN       0U     /* x100 -> 0.00               */
#define DYNBRAKE_FACTOR_PCT_MAX       100U   /* x100 -> 1.00               */
#define DYNBRAKE_MAX_PCT_MIN          0U     /* %                          */
#define DYNBRAKE_MAX_PCT_MAX          60U    /* % — firmware cap (tope)    */
#define DYNBRAKE_MIN_SPEED_DKMH_MIN   0U     /* x10 -> 0.0 km/h            */
#define DYNBRAKE_MIN_SPEED_DKMH_MAX   100U   /* x10 -> 10.0 km/h           */
#define DYNBRAKE_RAMP_DOWN_MIN        20U    /* %/s — must be > 0          */
#define DYNBRAKE_RAMP_DOWN_MAX        200U   /* %/s                        */
#define DYNBRAKE_RAMP_UP_MIN          20U    /* %/s — must be > 0          */
#define DYNBRAKE_RAMP_UP_MAX          200U   /* %/s                        */
#define DYNBRAKE_ENABLE_MIN           0U
#define DYNBRAKE_ENABLE_MAX           1U

/* ---- Compile-time defaults — SOFTER than the historic firmware ------
 *   Factor    = 0.20  (was DYNBRAKE_FACTOR          = 0.50)
 *   MaxPct    = 25    (was DYNBRAKE_MAX_PCT         = 60)
 *   MinSpeed  = 3.0   (== DYNBRAKE_MIN_SPEED_KMH    = 3.0, unchanged)
 *   RampDown  = 80    (== DYNBRAKE_RAMP_DOWN_PCT_S  = 80,  unchanged)
 *   RampUp    = 60    (NEW — no historic equivalent; the entry used to
 *                       borrow the unrelated PEDAL_RAMP_DOWN_PCT_S=100 %/s
 *                       constant with an asymmetric hard-snap fallback)
 *   Enable    = 1     (dynamic braking stays on by default, as today)
 * A blank/corrupt slot therefore brakes LESS aggressively than the historic
 * firmware, never more, and the entry is always slope-limited.            */
#define DYNBRAKE_FACTOR_PCT_DEFAULT      20U
#define DYNBRAKE_MAX_PCT_DEFAULT         25U
#define DYNBRAKE_MIN_SPEED_DKMH_DEFAULT  30U
#define DYNBRAKE_RAMP_DOWN_DEFAULT       80U
#define DYNBRAKE_RAMP_UP_DEFAULT         60U
#define DYNBRAKE_ENABLE_DEFAULT          1U

/* ---- Defensive coherence checks (defaults must be inside the ranges) - */
_Static_assert(DYNBRAKE_FACTOR_PCT_DEFAULT >= DYNBRAKE_FACTOR_PCT_MIN &&
               DYNBRAKE_FACTOR_PCT_DEFAULT <= DYNBRAKE_FACTOR_PCT_MAX,
               "Factor default out of range");
_Static_assert(DYNBRAKE_MAX_PCT_DEFAULT >= DYNBRAKE_MAX_PCT_MIN &&
               DYNBRAKE_MAX_PCT_DEFAULT <= DYNBRAKE_MAX_PCT_MAX,
               "MaxPct default out of range");
_Static_assert(DYNBRAKE_MIN_SPEED_DKMH_DEFAULT >= DYNBRAKE_MIN_SPEED_DKMH_MIN &&
               DYNBRAKE_MIN_SPEED_DKMH_DEFAULT <= DYNBRAKE_MIN_SPEED_DKMH_MAX,
               "MinSpeed default out of range");
_Static_assert(DYNBRAKE_RAMP_DOWN_DEFAULT >= DYNBRAKE_RAMP_DOWN_MIN &&
               DYNBRAKE_RAMP_DOWN_DEFAULT <= DYNBRAKE_RAMP_DOWN_MAX,
               "RampDown default out of range");
_Static_assert(DYNBRAKE_RAMP_UP_DEFAULT >= DYNBRAKE_RAMP_UP_MIN &&
               DYNBRAKE_RAMP_UP_DEFAULT <= DYNBRAKE_RAMP_UP_MAX,
               "RampUp default out of range");
_Static_assert(DYNBRAKE_ENABLE_DEFAULT <= DYNBRAKE_ENABLE_MAX,
               "Enable default out of range");
/* Ramp rates can never be zero — that would defeat the slew limiter.
 * Enable=0 / Factor=0 are the supported ways to fully disable braking. */
_Static_assert(DYNBRAKE_RAMP_DOWN_MIN >= 1U && DYNBRAKE_RAMP_UP_MIN >= 1U,
               "dynbrake ramp rates must be strictly positive");
/* NOTE: dynbrake_max_pct's validated range is 0-60 (matches the historic
 * compile-time ceiling).  motor_control_patched.c additionally applies an
 * independent, fixed 30% absolute field-safety backstop downstream
 * (DriveDynamics_DynbrakeLimitedPct() / DRIVE_DYNBRAKE_MAX_PCT in
 * drive_dynamics_policy.h) as defense-in-depth; that backstop is
 * unaffected by this store and is not re-checked at compile time here. */

/**
 * @brief  Runtime dynamic-braking parameter set.  Plain POD so it can be
 *         passed by value between the store, motor_control.c and the CAN
 *         handler without dynamic allocation.
 */
typedef struct {
    uint8_t factor_pct;      /* x100 -> 0.00-1.00 brake % per throttle-%/s */
    uint8_t max_pct;         /* 0-60  maximum opposing brake torque (%)    */
    uint8_t min_speed_dkmh;  /* x10   -> 0.0-10.0 km/h disable threshold   */
    uint8_t ramp_down;       /* %/s   max brake RELEASE rate               */
    uint8_t ramp_up;         /* %/s   max brake APPLICATION rate           */
    uint8_t enable;          /* 0 = off (coast), 1 = on                    */
} DynBrakeTuning_t;

/**
 * @brief  Initialise the dynamic-braking tuning store.  Reads flash and
 *         validates CRC / magic / range.  Does NOT apply the values — the
 *         caller (main.c) applies them via Traction_SetDynBrakeTuning().
 *         On flash blank / CRC-invalid / out-of-range, falls back silently
 *         to "no valid slot".  Boot is never blocked.                     */
void DynBrakeStore_Init(void);

/** @brief  True if the flash slot passed all integrity + range checks. */
bool DynBrakeStore_IsValid(void);

/** @brief  Read the stored parameter set.  Only meaningful when
 *          DynBrakeStore_IsValid() == true; otherwise outputs zero.       */
void DynBrakeStore_GetStored(DynBrakeTuning_t *out);

/** @brief  Fill @p out with the compile-time defaults (softer than the
 *          historic firmware — see file header). */
void DynBrakeStore_GetDefaults(DynBrakeTuning_t *out);

/**
 * @brief  Pure validator.  Returns true iff every field satisfies the hard
 *         ranges above.  Touches no flash.  Shared by the flash loader and
 *         the CAN SET/SAVE handler.                                       */
bool DynBrakeStore_Validate(const DynBrakeTuning_t *t);

/**
 * @brief  Persist a new parameter set to flash.
 *
 * Write conditions:
 *   - Safety_GetState() == SYS_STATE_STANDBY (re-asserted internally).
 *   - The set must pass DynBrakeStore_Validate().
 *   - Flash-wear guards: no-op elision when unchanged + minimum interval.
 *
 * @retval true on success, false on validation / safety / flash error.
 */
bool DynBrakeStore_Save(const DynBrakeTuning_t *t);

#ifdef __cplusplus
}
#endif

#endif /* DYNBRAKE_STORE_H */
