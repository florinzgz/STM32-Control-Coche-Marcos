/**
  ****************************************************************************
  * @file    standby_mode_sync_policy.h
  * @brief   HAL-free, host-testable STANDBY logical-mode-sync gate policy.
  *
  *          Encapsulates EVERY precondition that must hold before a CMD_MODE
  *          is permitted to update the in-RAM 4x4 / tank-turn flags while the
  *          vehicle is in STANDBY (the §3 Option-A gate in can_handler.c /
  *          safety_system.c).
  *
  *          Safety_IsStandbyModeSyncAllowed() (safety_system.c) is the SOLE
  *          caller in firmware: it gathers all inputs and delegates the
  *          policy decision here.  Tests link this same .c file directly —
  *          there is NO replica.
  *
  *          Compile for host (from repo root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE -ICore/Inc -O2 -lm \
  *                Core/Src/standby_mode_sync_policy.c               \
  *                Core/Src/test_standby_mode_sync_gate.c             \
  *                -o /tmp/test_standby_mode_sync_gate
  ****************************************************************************
  */

#ifndef STANDBY_MODE_SYNC_POLICY_H
#define STANDBY_MODE_SYNC_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- State constant -------------------------------------------------------
 * Value MUST match SYS_STATE_STANDBY in safety_system.h (= 1).
 * Do not include safety_system.h here to keep this module HAL-free.        */
#define STANDBY_SYNC_STATE_STANDBY  1u

/* ---- Thresholds -----------------------------------------------------------
 * Shared with safety_system.c and tests via this header.                   */

/** Pedal: at or below this value the pedal is considered released (%). */
#define STANDBY_MODE_SYNC_MAX_PEDAL_PCT      3.0f

/** Internal / effective demand: at or below this ε the demand is zero (%). */
#define STANDBY_MODE_SYNC_DEMAND_EPSILON     0.5f

/** Speed: at or below this value the vehicle is considered stationary. */
#define STANDBY_MODE_SYNC_MAX_SPEED_KMH      1.0f

/* ---- Input bundle ---------------------------------------------------------
 * All scalar inputs gathered by Safety_IsStandbyModeSyncAllowed() and
 * forwarded as a single aggregate so the policy is pure (no HAL calls).    */
typedef struct {
    uint8_t state;               /**< SystemState_t cast to uint8_t.           */
    float   pedalPct;            /**< Pedal position 0-100 %.                  */
    float   demandPct;           /**< TractionState_t->demandPct 0-100 %.      */
    float   effectiveDemandPct;  /**< Traction_GetEffectiveDemandPct() result. */
    uint8_t finalPwmPct;         /**< Traction_GetFinalPwmPct() result.        */
    float   avgSpeedKmh;         /**< Average wheel speed km/h.                */
    bool    errorOrHazardActive; /**< True when a safety error / hazard latch
                                  *   is currently active (future-proofing).   */
} StandbySyncInput_t;

/* ---- Result ---------------------------------------------------------------*/
typedef enum {
    STANDBY_SYNC_ALLOW_LOGICAL_ONLY = 0, /**< All preconditions met.           */
    STANDBY_SYNC_BLOCKED            = 1  /**< At least one precondition failed.*/
} StandbySyncResult_t;

/* ---- Public API -----------------------------------------------------------*/

/**
 * Evaluate all preconditions for the STANDBY logical-mode-sync gate.
 *
 * Returns ALLOW_LOGICAL_ONLY only when:
 *   1. state == STANDBY_SYNC_STATE_STANDBY (exactly STANDBY, nothing else).
 *   2. pedalPct        ≤ STANDBY_MODE_SYNC_MAX_PEDAL_PCT  (pedal released).
 *   3. demandPct       ≤ STANDBY_MODE_SYNC_DEMAND_EPSILON (no stored demand).
 *   4. effectiveDemandPct ≤ STANDBY_MODE_SYNC_DEMAND_EPSILON (pipeline zero).
 *   5. finalPwmPct     == 0                                (motors idle).
 *   6. avgSpeedKmh     ≤ STANDBY_MODE_SYNC_MAX_SPEED_KMH  (vehicle still).
 *   7. !errorOrHazardActive                                (no active hazard).
 *
 * NaN / Inf float inputs are treated as unsafe (fail-safe direction).
 *
 * @param in  Non-NULL pointer to the input bundle.
 * @return    ALLOW_LOGICAL_ONLY or BLOCKED.
 */
StandbySyncResult_t StandbyModeSync_Evaluate(const StandbySyncInput_t *in);

#ifdef __cplusplus
}
#endif

#endif /* STANDBY_MODE_SYNC_POLICY_H */
