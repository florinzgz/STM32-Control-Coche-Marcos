/**
  ****************************************************************************
  * @file    test_geometry_store.c
  * @brief   Host-compilable unit tests for geometry_store.c (service C2).
  *
  *          Covers:
  *            - GetDefaults() is bit-for-bit identical to WHEELBASE_M /
  *              TRACK_WIDTH_M in vehicle_physics.h (audit V1).
  *            - Validate()/Stage() reject out-of-range, NaN and infinite
  *              values instead of silently clamping (audit V6c/V6d).
  *            - Stage()/Save()/Revert()/ResetToDefaults() all call
  *              Ackermann_SetGeometry() with the new values, proving
  *              Ackermann_SetGeometry() now has an observable effect
  *              (audit V4c) via the real geometry_store.c production path.
  *            - Save() refuses to persist unless Safety_GetState() ==
  *              SYS_STATE_STANDBY (audit V3d), while Stage() remains usable
  *              at any system state.
  *
  *          Compile with host GCC (include production source):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -lm \
  *                Core/Src/test_geometry_store.c \
  *                Core/Src/geometry_store.c \
  *                -o test_geometry_store
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

#include "geometry_store.h"
#include "safety_system.h"   /* real SystemState_t / Safety_Error_t (V1) */

/* geometry_store.c calls Ackermann_SetGeometry() (declared in
 * motor_control.h, defined in motor_control.c) every time the staged
 * geometry changes.  Linking the real motor_control.c would pull in the
 * whole HAL/CAN/ADC dependency graph, so provide a minimal recording stub
 * instead -- this test is about geometry_store.c's own logic, not whether
 * the differential calculation itself consumes the staged values (that is
 * test_ackermann_diff.c's job: it links the extracted pure
 * AckermannDiff_Compute() with non-default geometry to prove the
 * calculation -- not just the store -- is wired to runtime values). */
static float s_last_wheelbase = -1.0f;
static float s_last_track     = -1.0f;
static float s_last_max_deg   = -1.0f;
static int   s_apply_calls    = 0;
void Ackermann_SetGeometry(float wheelbase_m, float track_m, float maxInnerDeg)
{
    s_last_wheelbase = wheelbase_m;
    s_last_track     = track_m;
    s_last_max_deg   = maxInnerDeg;
    s_apply_calls++;
}

/* Safety_GetState() is called by GeometryStore_Save(); safety_system.h
 * (included above) already declares its prototype and the SystemState_t
 * enum, so only the definition (a stub whose return value the test can
 * flip) is provided here, to exercise the STANDBY-only flash-write gate
 * (rejects outside STANDBY, accepts inside STANDBY) without linking the
 * real safety_system.c. */
static SystemState_t s_stub_state = SYS_STATE_STANDBY;
SystemState_t Safety_GetState(void) { return s_stub_state; }
Safety_Error_t Safety_GetError(void) { return SAFETY_ERROR_NONE; }

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
    /* ---- V1: defaults must be bit-for-bit identical to the compile-time
     * macros they replace. ---- */
    Geometry_t d;
    GeometryStore_GetDefaults(&d);
    ASSERT_TRUE(d.wheelbase_m   == WHEELBASE_M);
    ASSERT_TRUE(d.track_width_m == TRACK_WIDTH_M);
    ASSERT_TRUE(GeometryStore_Validate(&d));

    /* ---- V6c/d: boundary + NaN/Inf/negative rejection ---- */
    Geometry_t g = d;
    g.wheelbase_m = GEOMETRY_WHEELBASE_M_MIN;
    ASSERT_TRUE(GeometryStore_Validate(&g));
    g.wheelbase_m = GEOMETRY_WHEELBASE_M_MAX;
    ASSERT_TRUE(GeometryStore_Validate(&g));
    g.wheelbase_m = GEOMETRY_WHEELBASE_M_MIN - 0.001f;
    ASSERT_FALSE(GeometryStore_Validate(&g));
    g.wheelbase_m = GEOMETRY_WHEELBASE_M_MAX + 0.001f;
    ASSERT_FALSE(GeometryStore_Validate(&g));

    g = d; g.wheelbase_m   = NAN;      ASSERT_FALSE(GeometryStore_Validate(&g));
    g = d; g.track_width_m = NAN;      ASSERT_FALSE(GeometryStore_Validate(&g));
    g = d; g.wheelbase_m   = INFINITY; ASSERT_FALSE(GeometryStore_Validate(&g));
    g = d; g.track_width_m = -INFINITY; ASSERT_FALSE(GeometryStore_Validate(&g));
    g = d; g.wheelbase_m   = -0.5f;    ASSERT_FALSE(GeometryStore_Validate(&g));  /* negative */

    ASSERT_FALSE(GeometryStore_Validate(NULL));

    /* ---- V4c: Ackermann_SetGeometry() must now have an observable effect
     * -- Stage() must call it immediately with the new values (not the
     * macro defaults), proving the store is wired to the real production
     * entrypoint and not a dead copy. ---- */
    s_apply_calls = 0;
    Geometry_t custom = { .wheelbase_m = 1.10f, .track_width_m = 0.80f };
    ASSERT_TRUE(GeometryStore_Validate(&custom));
    ASSERT_TRUE(GeometryStore_Stage(&custom));
    ASSERT_TRUE(s_apply_calls == 1);
    ASSERT_TRUE(s_last_wheelbase == 1.10f);
    ASSERT_TRUE(s_last_track     == 0.80f);
    ASSERT_TRUE(s_last_max_deg   == MAX_STEER_DEG);

    /* A rejected Stage() must NOT call Ackermann_SetGeometry() (no silent
     * partial application of invalid geometry). */
    Geometry_t bad = { .wheelbase_m = NAN, .track_width_m = 0.80f };
    int calls_before = s_apply_calls;
    ASSERT_FALSE(GeometryStore_Stage(&bad));
    ASSERT_TRUE(s_apply_calls == calls_before);

    /* ---- V3d: Save() must refuse to persist outside STANDBY, and must
     * succeed once STANDBY is restored; Stage() is unaffected. ---- */
    s_stub_state = SYS_STATE_ACTIVE;
    Geometry_t tuned = { .wheelbase_m = 0.98f, .track_width_m = 0.72f };
    ASSERT_TRUE(GeometryStore_Stage(&tuned));
    ASSERT_FALSE(GeometryStore_Save());

    s_stub_state = SYS_STATE_STANDBY;
    ASSERT_TRUE(GeometryStore_Save());

    /* ---- V6b: Revert() discards unsaved staged edits and re-applies the
     * last persisted geometry via Ackermann_SetGeometry(). ---- */
    Geometry_t another = { .wheelbase_m = 1.20f, .track_width_m = 0.90f };
    ASSERT_TRUE(GeometryStore_Stage(&another));
    GeometryStore_Revert();
    Geometry_t reverted;
    GeometryStore_GetStaged(&reverted);
    ASSERT_TRUE(reverted.wheelbase_m   == 0.98f);
    ASSERT_TRUE(reverted.track_width_m == 0.72f);
    ASSERT_TRUE(s_last_wheelbase == 0.98f);  /* Revert() re-applied it */

    /* ---- ResetToDefaults() restores the compile-time macro values and
     * re-applies them. ---- */
    GeometryStore_ResetToDefaults();
    Geometry_t after_reset;
    GeometryStore_GetStaged(&after_reset);
    ASSERT_TRUE(after_reset.wheelbase_m   == WHEELBASE_M);
    ASSERT_TRUE(after_reset.track_width_m == TRACK_WIDTH_M);
    ASSERT_TRUE(s_last_wheelbase == WHEELBASE_M);

    printf("test_geometry_store: %d run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}

#endif /* HOST_TEST */
