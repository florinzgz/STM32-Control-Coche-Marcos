/**
  ****************************************************************************
  * @file    test_tcs_tuning_store.c
  * @brief   Host-compilable unit tests for tcs_tuning_store.c (service C1).
  *
  *          Covers:
  *            - GetDefaults() is bit-for-bit identical to the legacy
  *              DRIVE_TCS_* macros in drive_dynamics_policy.h (audit V1).
  *            - Validate()/Stage() reject out-of-range, NaN and infinite
  *              values instead of silently clamping (audit V6c/V6d).
  *            - Save() refuses to persist unless Safety_GetState() ==
  *              SYS_STATE_STANDBY (audit V3d), while Stage() remains usable
  *              at any system state.
  *            - Revert() discards unsaved staged edits (audit V6b).
  *
  *          Compile with host GCC (include production source):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -lm \
  *                Core/Src/test_tcs_tuning_store.c \
  *                Core/Src/tcs_tuning_store.c \
  *                -o test_tcs_tuning_store
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
#include <math.h>

#include "tcs_tuning_store.h"

/* Safety_GetState() is called by TcsTuningStore_Save(); provide a stub
 * whose return value the test can flip, to exercise the STANDBY-only
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

static TcsTuning_t make_defaults(void)
{
    TcsTuning_t t;
    TcsTuningStore_GetDefaults(&t);
    return t;
}

int main(void)
{
    /* ---- V1: defaults must be bit-for-bit identical to the compile-time
     * macros they replace (a freshly-flashed car must behave exactly as
     * before this PR). ---- */
    TcsTuning_t d = make_defaults();
    ASSERT_TRUE(d.min_reference_kmh    == DRIVE_TCS_MIN_REFERENCE_KMH);
    ASSERT_TRUE(d.slip_threshold_pct   == DRIVE_TCS_SLIP_THRESHOLD_PCT);
    ASSERT_TRUE(d.initial_reduction    == DRIVE_TCS_INITIAL_REDUCTION);
    ASSERT_TRUE(d.reduction_rate_per_s == DRIVE_TCS_REDUCTION_RATE_PER_S);
    ASSERT_TRUE(d.recovery_rate_per_s  == DRIVE_TCS_RECOVERY_RATE_PER_S);
    ASSERT_TRUE(d.max_reduction        == DRIVE_TCS_MAX_REDUCTION);

    /* Defaults must always themselves validate (RESET DEFAULTS must never
     * be rejected by the store's own validator). */
    ASSERT_TRUE(TcsTuningStore_Validate(&d));

    /* ---- V6c/d: boundary + NaN/Inf/negative rejection ---- */
    TcsTuning_t t = d;
    t.min_reference_kmh = TCS_TUNE_MIN_REFERENCE_KMH_MIN;
    ASSERT_TRUE(TcsTuningStore_Validate(&t));
    t.min_reference_kmh = TCS_TUNE_MIN_REFERENCE_KMH_MAX;
    ASSERT_TRUE(TcsTuningStore_Validate(&t));
    t.min_reference_kmh = TCS_TUNE_MIN_REFERENCE_KMH_MIN - 0.01f;
    ASSERT_FALSE(TcsTuningStore_Validate(&t));
    t.min_reference_kmh = TCS_TUNE_MIN_REFERENCE_KMH_MAX + 0.01f;
    ASSERT_FALSE(TcsTuningStore_Validate(&t));

    /* max_reduction must never be below initial_reduction. */
    t = d;
    t.initial_reduction = 0.70f;
    t.max_reduction     = 0.60f;
    ASSERT_FALSE(TcsTuningStore_Validate(&t));

    /* NaN must be rejected on every float field (a NaN compared with
     * < or > is always false, so a naive range check would silently
     * accept it -- this is the audit V6d finding). */
    t = d; t.min_reference_kmh    = NAN; ASSERT_FALSE(TcsTuningStore_Validate(&t));
    t = d; t.slip_threshold_pct   = NAN; ASSERT_FALSE(TcsTuningStore_Validate(&t));
    t = d; t.initial_reduction    = NAN; ASSERT_FALSE(TcsTuningStore_Validate(&t));
    t = d; t.reduction_rate_per_s = NAN; ASSERT_FALSE(TcsTuningStore_Validate(&t));
    t = d; t.recovery_rate_per_s  = NAN; ASSERT_FALSE(TcsTuningStore_Validate(&t));
    t = d; t.max_reduction        = NAN; ASSERT_FALSE(TcsTuningStore_Validate(&t));

    /* +Inf / -Inf must be rejected too. */
    t = d; t.min_reference_kmh = INFINITY;  ASSERT_FALSE(TcsTuningStore_Validate(&t));
    t = d; t.min_reference_kmh = -INFINITY; ASSERT_FALSE(TcsTuningStore_Validate(&t));
    t = d; t.max_reduction     = INFINITY;  ASSERT_FALSE(TcsTuningStore_Validate(&t));

    /* Negative values are already rejected (every MIN is positive). */
    t = d; t.slip_threshold_pct = -1.0f; ASSERT_FALSE(TcsTuningStore_Validate(&t));

    /* NULL must be rejected, not dereferenced. */
    ASSERT_FALSE(TcsTuningStore_Validate(NULL));

    /* ---- Stage(): mirrors Validate(), explicit rejection not silent
     * clamp; a rejected Stage() must not corrupt the existing stage. ---- */
    ASSERT_TRUE(TcsTuningStore_Stage(&d));
    TcsTuning_t staged;
    TcsTuningStore_GetStaged(&staged);
    ASSERT_TRUE(staged.min_reference_kmh == d.min_reference_kmh);

    TcsTuning_t bad = d;
    bad.max_reduction = NAN;
    ASSERT_FALSE(TcsTuningStore_Stage(&bad));
    TcsTuningStore_GetStaged(&staged);
    ASSERT_TRUE(staged.min_reference_kmh == d.min_reference_kmh);  /* unchanged */

    /* ---- V3d: Save() must refuse to persist outside STANDBY, and must
     * succeed once STANDBY is restored; Stage() is unaffected by system
     * state (staging must remain usable during an active service test). ---- */
    s_stub_state = SYS_STATE_ACTIVE;
    TcsTuning_t tuned = d;
    tuned.slip_threshold_pct = 30.0f;
    ASSERT_TRUE(TcsTuningStore_Stage(&tuned));       /* staging always allowed */
    ASSERT_FALSE(TcsTuningStore_Save());             /* but save is refused    */

    s_stub_state = SYS_STATE_STANDBY;
    ASSERT_TRUE(TcsTuningStore_Save());              /* save now succeeds      */

    /* ---- V6b: Revert() discards unsaved staged edits, falling back to
     * the last persisted values. ---- */
    TcsTuning_t after_save;
    TcsTuningStore_GetStaged(&after_save);
    ASSERT_TRUE(after_save.slip_threshold_pct == 30.0f);

    TcsTuning_t another = d;
    another.slip_threshold_pct = 45.0f;
    ASSERT_TRUE(TcsTuningStore_Stage(&another));
    TcsTuningStore_Revert();
    TcsTuning_t reverted;
    TcsTuningStore_GetStaged(&reverted);
    ASSERT_TRUE(reverted.slip_threshold_pct == 30.0f);  /* back to last SAVE */

    /* ---- Effective (RAM-staged) getters mirror the staged struct (C1 is
     * one of the fields editable live, so Get*() must reflect Stage()
     * immediately without requiring Save()). ---- */
    TcsTuning_t live = d;
    live.recovery_rate_per_s = 1.0f;
    ASSERT_TRUE(TcsTuningStore_Stage(&live));
    ASSERT_TRUE(TcsTuningStore_GetRecoveryRatePerS() == 1.0f);

    /* ---- V6e: no per-wheel trim/compensation exists in this struct --
     * six GLOBAL scalar fields only, applied identically to all wheels. ---- */
    ASSERT_TRUE(sizeof(TcsTuning_t) == 6U * sizeof(float));

    printf("test_tcs_tuning_store: %d run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}

#endif /* HOST_TEST */
