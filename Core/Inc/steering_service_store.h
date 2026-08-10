/**
  ****************************************************************************
  * @file    steering_service_store.h
  * @brief   Persistent steering diagnostic-homing parameters — service C4
  *
  * IMPORTANT SCOPE NOTE: the production auto-centering FSM in
  * steering_centering.c is intentionally FROZEN by a prior audit
  * ("Mantener CENTERING_PWM=425, STALL_TIMEOUT_MS=300, TOTAL_TIMEOUT_MS=10000
  * y MAX_CENTERING_COUNTS=6000 inicialmente" — see steering_centering_diag.h).
  * This store does NOT alter that production FSM.  It holds the parameters
  * used ONLY by the service-diagnostic steering test (Block B2), which runs
  * its OWN short, bounded sweep re-using the existing time-guard machinery
  * as a ceiling ("Barrido corto acotado por la guarda de tiempo ya
  * implementada" per the spec) without touching the frozen production
  * constants.
  *
  * Fields (all runtime-tunable, RAM-staged, flash-persisted):
  *   search_pwm_counts     100..1200   default 425   (matches CENTERING_PWM)
  *   sweep_time_guard_ms   500..5000   default 2000  (Block-A step ceiling)
  *   stall_timeout_ms      100..1000   default 300   (matches STEER_DIAG_*)
  *   total_timeout_ms      2000..20000 default 10000 (matches STEER_DIAG_*)
  *   max_centering_counts  1000..12000 default 6000  (matches STEER_DIAG_*)
  *   stop_current_ma       200..1500   default 1000  (mechanical-stop
  *                         detection threshold for the diagnostic sweep)
  *
  * Safety invariant (spec): "El maximo configurable NUNCA puede exceder el
  * valor seguro cuando la guarda de corriente no esta armada" -- enforced by
  * hard-capping the stop_current_ma validation range itself at
  * STEER_SVC_STOP_CURRENT_MA_MAX (1500 mA, well under
  * STEERING_ISOLATION_CURRENT_MAX_MA=1000+headroom / STEERING_OC_LIMIT_MA);
  * there is no way to stage or persist a value above this ceiling regardless
  * of whether the diagnostic test's own overcurrent guard is armed.
  *
  * Editable ONLY with actuators stopped inside the session (Block C rule:
  * C3/C4 require the test to be idle, unlike C1/C2/C5 which may be edited
  * live) -- enforced by the caller (service_diag_session.c), not this store.
  *
  * Flash layout: Page 115 (0x08073000, 4 KB), magic "STS1" + CRC32.
  ****************************************************************************
  */

#ifndef STEERING_SERVICE_STORE_H
#define STEERING_SERVICE_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "steering_centering.h"

#define STEER_SVC_SEARCH_PWM_MIN         100U
#define STEER_SVC_SEARCH_PWM_MAX         1200U
#define STEER_SVC_SWEEP_GUARD_MS_MIN     500U
#define STEER_SVC_SWEEP_GUARD_MS_MAX     5000U
#define STEER_SVC_STALL_TIMEOUT_MS_MIN   100U
#define STEER_SVC_STALL_TIMEOUT_MS_MAX   1000U
#define STEER_SVC_TOTAL_TIMEOUT_MS_MIN   2000U
#define STEER_SVC_TOTAL_TIMEOUT_MS_MAX   20000U
#define STEER_SVC_MAX_CENTERING_CNT_MIN  1000U
#define STEER_SVC_MAX_CENTERING_CNT_MAX  12000U
/* Hard, non-negotiable ceiling -- see safety invariant above. */
#define STEER_SVC_STOP_CURRENT_MA_MIN    200U
#define STEER_SVC_STOP_CURRENT_MA_MAX    1500U

/* ---- V2 audit fix (Bloque A, PR #445) -----------------------------------
 * "steering_service_store estrena consumidor de producción": the service-
 * diagnostic session (service_diag_session.c) is the FIRST production code
 * to actually drive an actuator from this store's staged/effective value.
 * Until now STEER_SVC_SEARCH_PWM_MAX (1200) being under the homing reduced-
 * authority floor (HOMING_PWM_REDUCED_COUNTS = 2336) was true only "by
 * construction" -- nobody verified it, and nothing would fail loudly if a
 * future edit broke the relationship.
 *
 * This is a SAFETY GUARD, not a convenience limit: STEER_SVC_SEARCH_PWM_MAX
 * bounds the only PWM a human-confirmed diagnostic session may ever command
 * on the steering motor outside the production homing FSM.  Raising it
 * requires re-validating against the homing CH5 overcurrent guard
 * (ENDSTOP_HARD_CURRENT_MA / ENDSTOP_STALL_CURRENT_MA in
 * steering_centering_patched.c) -- do not bump this constant without also
 * re-checking the homing end-stop pressure detector still fires in time. */
_Static_assert(STEER_SVC_SEARCH_PWM_MAX <= HOMING_PWM_REDUCED_COUNTS,
               "steering_service_store PWM ceiling must not exceed the "
               "homing reduced-authority safety floor");

typedef struct {
    uint16_t search_pwm_counts;
    uint16_t sweep_time_guard_ms;
    uint16_t stall_timeout_ms;
    uint16_t total_timeout_ms;
    uint16_t max_centering_counts;
    uint16_t stop_current_ma;
} SteeringServiceParams_t;

/* HAL-free single source used by production and host tests. */
static inline bool SteeringService_ValidateValues(const SteeringServiceParams_t *p)
{
    if (!p) return false;
    if (p->search_pwm_counts < STEER_SVC_SEARCH_PWM_MIN ||
        p->search_pwm_counts > STEER_SVC_SEARCH_PWM_MAX) return false;
    if (p->sweep_time_guard_ms < STEER_SVC_SWEEP_GUARD_MS_MIN ||
        p->sweep_time_guard_ms > STEER_SVC_SWEEP_GUARD_MS_MAX) return false;
    if (p->stall_timeout_ms < STEER_SVC_STALL_TIMEOUT_MS_MIN ||
        p->stall_timeout_ms > STEER_SVC_STALL_TIMEOUT_MS_MAX) return false;
    if (p->total_timeout_ms < STEER_SVC_TOTAL_TIMEOUT_MS_MIN ||
        p->total_timeout_ms > STEER_SVC_TOTAL_TIMEOUT_MS_MAX) return false;
    if (p->max_centering_counts < STEER_SVC_MAX_CENTERING_CNT_MIN ||
        p->max_centering_counts > STEER_SVC_MAX_CENTERING_CNT_MAX) return false;
    if (p->stop_current_ma < STEER_SVC_STOP_CURRENT_MA_MIN ||
        p->stop_current_ma > STEER_SVC_STOP_CURRENT_MA_MAX) return false;
    return true;
}

void SteeringServiceStore_Init(void);
bool SteeringServiceStore_IsValid(void);
void SteeringServiceStore_GetDefaults(SteeringServiceParams_t *out);
bool SteeringServiceStore_Validate(const SteeringServiceParams_t *p);

void SteeringServiceStore_GetEffective(SteeringServiceParams_t *out);
bool SteeringServiceStore_Stage(const SteeringServiceParams_t *p);
void SteeringServiceStore_GetStaged(SteeringServiceParams_t *out);
void SteeringServiceStore_Revert(void);
void SteeringServiceStore_ResetToDefaults(void);
bool SteeringServiceStore_Save(void);

/**
 * @brief  V2 audit (Bloque A): defense-in-depth CLAMPED accessor.
 *
 *         Returns the same staged/effective parameters as
 *         SteeringServiceStore_GetEffective(), but additionally re-clamps
 *         search_pwm_counts to HOMING_PWM_REDUCED_COUNTS at the point of
 *         USE.  SteeringServiceStore_Stage()/_ValidateValues() already
 *         reject anything above STEER_SVC_SEARCH_PWM_MAX (<= the floor by
 *         the _Static_assert above), so under normal operation this clamp
 *         is a no-op; it only engages if RAM/flash were ever corrupted in
 *         a way that bypassed the type-safe staging API (e.g. a stray
 *         write), guaranteeing the service session NEVER commands the
 *         steering motor above the homing safety floor no matter what the
 *         stored struct contains.  Any production consumer of
 *         search_pwm_counts (currently service_diag_session.c) MUST call
 *         this accessor, not SteeringServiceStore_GetEffective(), when the
 *         value is about to be applied to the motor.
 */
void SteeringServiceStore_GetEffectiveClamped(SteeringServiceParams_t *out);

#ifdef __cplusplus
}
#endif

#endif /* STEERING_SERVICE_STORE_H */

