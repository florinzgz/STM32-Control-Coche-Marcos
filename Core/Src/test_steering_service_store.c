/**
  ****************************************************************************
  * @file    test_steering_service_store.c
  * @brief   Host-compilable unit tests for steering_service_store.c
  *          (service C4 -- diagnostic-homing sweep parameters).
  *
  *          Covers:
  *            - GetDefaults() is bit-for-bit identical to STEER_DIAG_
  *              STALL_TIMEOUT_MS / TOTAL_TIMEOUT_MS / MAX_CENTERING_COUNTS
  *              in steering_centering_diag.h (audit V1, partial parity --
  *              see the NOT-a-mirror note below for the other 3 fields).
  *            - search_pwm_counts / sweep_time_guard_ms / stop_current_ma
  *              default to 425 / 2000 / 1000, which this store's OWN header
  *              (steering_service_store.h lines 6-36) documents as an
  *              INTENTIONALLY SEPARATE, lower-power "diagnostic sweep"
  *              concept -- NOT a mirror of the frozen production homing
  *              constants HOMING_PWM_COUNTS=4249, HOMING_SWEEP_TIMEOUT_MS=
  *              1000 or ENDSTOP_STALL/HARD_CURRENT_MA=8000/20000 in
  *              steering_centering_patched.c.  This test documents that
  *              distinction rather than asserting a false equivalence.
  *            - V2: STEER_SVC_SEARCH_PWM_MAX (1200) is a hard
  *              validation ceiling -- Stage()/Validate() REJECT (do not
  *              clamp) any candidate above it, so search_pwm_counts can
  *              never be set to a value anywhere near HOMING_PWM_COUNTS'
  *              reduced-cap floor (~2336 = 4249 * 55%), regardless of
  *              whether the CH5 current guard is armed.
  *            - Validate()/Stage() reject out-of-range values instead of
  *              silently clamping (audit V6c).  All 6 fields are uint16_t,
  *              so there is no NaN/Inf surface to test here (audit V6d
  *              N/A for this store).
  *            - Save() refuses to persist unless Safety_GetState() ==
  *              SYS_STATE_STANDBY (audit V3d), while Stage() remains usable
  *              at any system state.
  *
  *          Compile with host GCC (include production source):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -lm \
  *                Core/Src/test_steering_service_store.c \
  *                Core/Src/steering_service_store.c \
  *                -o test_steering_service_store
  *
  *          This file defines main() and is intended ONLY for host-side unit
  *          testing.  It is excluded from the STM32 firmware build via the
  *          HOST_TEST guard so that its main() does not collide with the
  *          firmware's main() in Core/Src/main.c at link time.
  ****************************************************************************
  */

#ifdef HOST_TEST

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "steering_service_store.h"
#include "steering_centering_diag.h"   /* STEER_DIAG_* macros (V1 parity) */

/* Safety_GetState() is called by SteeringServiceStore_Save(); provide a
 * stub whose return value the test can flip, to exercise the STANDBY-only
 * flash-write gate (rejects outside STANDBY, accepts inside STANDBY). */
#ifndef SAFETY_SYSTEM_H
typedef enum { SYS_STATE_BOOT = 0, SYS_STATE_STANDBY = 1, SYS_STATE_ACTIVE = 2 } SystemState_t;
typedef enum { SAFETY_ERROR_NONE = 0 } Safety_Error_t;
static SystemState_t s_stub_state = SYS_STATE_STANDBY;
SystemState_t Safety_GetState(void) { return s_stub_state; }
Safety_Error_t Safety_GetError(void) { return SAFETY_ERROR_NONE; }
#endif

/* ---- Test harness ---- */
static int tests_run    = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(expr) do {                                        \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_FALSE(expr) do {                                       \
    tests_run++;                                                      \
    if ((expr)) {                                                     \
        printf("FAIL %s:%d  !(%s)\n", __FILE__, __LINE__, #expr);     \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

int main(void)
{
    /* ---- V1: partial parity. Only stall/total/max-centering are meant to
     * mirror STEER_DIAG_* (this store's header says so explicitly); the
     * other 3 fields are a deliberately separate concept -- see file
     * banner above and steering_service_store.h lines 18-24. */
    SteeringServiceParams_t d;
    SteeringServiceStore_GetDefaults(&d);
    ASSERT_TRUE(d.stall_timeout_ms     == STEER_DIAG_STALL_TIMEOUT_MS);
    ASSERT_TRUE(d.total_timeout_ms     == STEER_DIAG_TOTAL_TIMEOUT_MS);
    ASSERT_TRUE(d.max_centering_counts == STEER_DIAG_MAX_CENTERING_COUNTS);
    /* NOT macro mirrors -- documented, separate diagnostic-only concept: */
    ASSERT_TRUE(d.search_pwm_counts    == 425U);
    ASSERT_TRUE(d.sweep_time_guard_ms  == 2000U);
    ASSERT_TRUE(d.stop_current_ma      == 1000U);
    ASSERT_TRUE(SteeringServiceStore_Validate(&d));

    /* ---- V2: STEER_SVC_SEARCH_PWM_MAX is a hard validation ceiling that
     * REJECTS (never clamps) any candidate above it -- structurally this
     * makes it impossible for search_pwm_counts to reach anywhere near the
     * production reduced-PWM floor (4249 * 55% ~= 2336), regardless of
     * whether the CH5 current guard is armed. ---- */
    ASSERT_TRUE(STEER_SVC_SEARCH_PWM_MAX < 2336U);
    /* Establish a known RAM-staged baseline first (Init() is intentionally
     * never called in this host test: it dereferences the raw flash
     * address, unmapped on a host process). ResetToDefaults() is RAM-only
     * and safe to call here. */
    SteeringServiceStore_ResetToDefaults();
    SteeringServiceParams_t over_pwm = d;
    over_pwm.search_pwm_counts = STEER_SVC_SEARCH_PWM_MAX + 1U;
    ASSERT_FALSE(SteeringServiceStore_Validate(&over_pwm));
    ASSERT_FALSE(SteeringServiceStore_Stage(&over_pwm));
    /* A rejected Stage() must not silently clamp -- the staged value must
     * remain whatever it was before the attempt (still the defaults). */
    SteeringServiceParams_t staged_after_reject;
    SteeringServiceStore_GetStaged(&staged_after_reject);
    ASSERT_TRUE(staged_after_reject.search_pwm_counts == 425U);

    /* ---- V6c: boundary + out-of-range rejection for every field
     * (uint16_t only -- no NaN/Inf surface, audit V6d is N/A here). ---- */
    SteeringServiceParams_t s;

    s = d; s.search_pwm_counts = STEER_SVC_SEARCH_PWM_MIN;
    ASSERT_TRUE(SteeringServiceStore_Validate(&s));
    s = d; s.search_pwm_counts = STEER_SVC_SEARCH_PWM_MIN - 1U;
    ASSERT_FALSE(SteeringServiceStore_Validate(&s));

    s = d; s.sweep_time_guard_ms = STEER_SVC_SWEEP_GUARD_MS_MIN;
    ASSERT_TRUE(SteeringServiceStore_Validate(&s));
    s = d; s.sweep_time_guard_ms = STEER_SVC_SWEEP_GUARD_MS_MIN - 1U;
    ASSERT_FALSE(SteeringServiceStore_Validate(&s));
    s = d; s.sweep_time_guard_ms = STEER_SVC_SWEEP_GUARD_MS_MAX + 1U;
    ASSERT_FALSE(SteeringServiceStore_Validate(&s));

    s = d; s.stall_timeout_ms = STEER_SVC_STALL_TIMEOUT_MS_MIN - 1U;
    ASSERT_FALSE(SteeringServiceStore_Validate(&s));
    s = d; s.stall_timeout_ms = STEER_SVC_STALL_TIMEOUT_MS_MAX + 1U;
    ASSERT_FALSE(SteeringServiceStore_Validate(&s));

    s = d; s.total_timeout_ms = STEER_SVC_TOTAL_TIMEOUT_MS_MIN - 1U;
    ASSERT_FALSE(SteeringServiceStore_Validate(&s));
    s = d; s.total_timeout_ms = STEER_SVC_TOTAL_TIMEOUT_MS_MAX + 1U;
    ASSERT_FALSE(SteeringServiceStore_Validate(&s));

    s = d; s.max_centering_counts = STEER_SVC_MAX_CENTERING_CNT_MIN - 1U;
    ASSERT_FALSE(SteeringServiceStore_Validate(&s));
    s = d; s.max_centering_counts = STEER_SVC_MAX_CENTERING_CNT_MAX + 1U;
    ASSERT_FALSE(SteeringServiceStore_Validate(&s));

    s = d; s.stop_current_ma = STEER_SVC_STOP_CURRENT_MA_MIN - 1U;
    ASSERT_FALSE(SteeringServiceStore_Validate(&s));
    s = d; s.stop_current_ma = STEER_SVC_STOP_CURRENT_MA_MAX + 1U;
    ASSERT_FALSE(SteeringServiceStore_Validate(&s));

    ASSERT_FALSE(SteeringServiceStore_Validate(NULL));

    /* ---- V3d: Save() must refuse to persist outside STANDBY, and must
     * succeed once STANDBY is restored; Stage() is unaffected. ---- */
    s_stub_state = SYS_STATE_ACTIVE;
    SteeringServiceParams_t tuned = d;
    tuned.search_pwm_counts = 600U;
    ASSERT_TRUE(SteeringServiceStore_Stage(&tuned));
    ASSERT_FALSE(SteeringServiceStore_Save());

    s_stub_state = SYS_STATE_STANDBY;
    ASSERT_TRUE(SteeringServiceStore_Save());

    /* GetEffective() reflects the staged (and now saved) value, consistent
     * with the sibling Bloque C stores. */
    SteeringServiceParams_t eff;
    SteeringServiceStore_GetEffective(&eff);
    ASSERT_TRUE(eff.search_pwm_counts == 600U);

    /* ---- V6b: Revert() discards unsaved staged edits and falls back to
     * the last SAVED value. ---- */
    SteeringServiceParams_t another = d;
    another.search_pwm_counts = 900U;
    ASSERT_TRUE(SteeringServiceStore_Stage(&another));
    SteeringServiceStore_Revert();
    SteeringServiceParams_t reverted;
    SteeringServiceStore_GetStaged(&reverted);
    ASSERT_TRUE(reverted.search_pwm_counts == 600U);   /* back to last SAVE, not 900 */

    /* ---- ResetToDefaults() restores the compile-time defaults (RAM only,
     * requires a subsequent Save() to persist -- same contract as every
     * sibling store). ---- */
    SteeringServiceStore_ResetToDefaults();
    SteeringServiceParams_t after_reset;
    SteeringServiceStore_GetStaged(&after_reset);
    ASSERT_TRUE(after_reset.search_pwm_counts == 425U);

    /* ---- V6e: no per-wheel trim/compensation exists in this struct --
     * six GLOBAL scalar fields only (steering has one motor, so this is
     * inherently non-per-wheel). ---- */
    ASSERT_TRUE(sizeof(SteeringServiceParams_t) == 6U * sizeof(uint16_t));

    printf("test_steering_service_store: %d run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}

#endif /* HOST_TEST */
