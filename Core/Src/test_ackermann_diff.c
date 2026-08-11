/**
  ****************************************************************************
  * @file    test_ackermann_diff.c
  * @brief   Host-compilable unit tests for ackermann_diff.c (PR #445 Bloque
  *          C, llamapreview P2 finding).
  *
  *          Historical bug this closes: compute_ackermann_differential()
  *          (motor_control.c) used to read the compile-time WHEELBASE_M /
  *          TRACK_WIDTH_M macros instead of the runtime ackermann_wheelbase /
  *          ackermann_track statics, silently nullifying
  *          Ackermann_SetGeometry() (geometry_store.c).  test_geometry_store.c
  *          only proved the STORE calls through a recording stub of
  *          Ackermann_SetGeometry() -- nothing exercised the differential
  *          CALCULATION itself.  This file links the extracted pure
  *          AckermannDiff_Compute() (ackermann_diff.c) directly and drives it
  *          with non-default geometry to close that gap.
  *
  *          Covers:
  *            - Non-default geometry (wheelbase 1.20 / track 0.50) changes
  *              the per-wheel differential vs. the compile-time defaults at
  *              the same angle -- proves the calculation consumes whatever
  *              wheelbase_m/track_m it is given (the runtime statics at the
  *              real motor_control.c call site), not a compile-time macro.
  *              Wheelbase-only and track-only variations are also checked in
  *              isolation so neither parameter can be silently ignored.
  *            - Centred steering (0 deg) yields EXACTLY 1.000 on all four
  *              wheels regardless of geometry -- this is also the
  *              FAIL_ACKERMANN_OFFSET precondition the Hito 2 wheel-equality
  *              test (wheel_equality_test.c) depends on.
  *            - REDUCTION-ONLY clamp: the outside-wheel multiplier never
  *              exceeds 1.0 for any valid staged geometry (geometry_store.h
  *              GEOMETRY_*_MIN/MAX) or any angle 0-90 deg, and the inside
  *              wheel genuinely is reduced (the clamp is not trivially a
  *              no-op).
  *            - Deadband (<2 deg) and tan_angle<0.001f (>90 deg) guards both
  *              short-circuit to the all-1.0 default.
  *            - Default geometry (0.95/0.70) output matches the pre-fix
  *              values at 5 representative angles: 0 deg, within-deadband,
  *              an unclamped mid-angle, a clamped mid-angle, and
  *              MAX_STEER_DEG.
  *
  *          Compile with host GCC (include production source):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -lm \
  *                Core/Src/test_ackermann_diff.c \
  *                Core/Src/ackermann_diff.c \
  *                -o test_ackermann_diff
  *
  *          This file defines main() and is intended ONLY for host-side unit
  *          testing.  It is excluded from the STM32 firmware build via the
  *          HOST_TEST guard so that its main() does not collide with the
  *          firmware's main() in Core/Src/main.c at link time.
  ****************************************************************************
  */

#ifdef HOST_TEST

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "ackermann_diff.h"
#include "geometry_store.h"   /* GEOMETRY_WHEELBASE_M_MIN/MAX, TRACK_..._MIN/MAX (V6c range) */
#include "vehicle_physics.h"  /* WHEELBASE_M / TRACK_WIDTH_M / MAX_STEER_DEG defaults */

/* ---- Test harness (same shape as test_geometry_store.c et al.) ---- */
static int tests_run    = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(expr) do {                                        \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_NEAR(a, b, eps) do {                                        \
    tests_run++;                                                          \
    float _a = (a), _b = (b), _eps = (eps);                                \
    if (fabsf(_a - _b) > _eps) {                                           \
        printf("FAIL %s:%d  %s (%.8f) !~= %s (%.8f) eps=%.8f\n",           \
               __FILE__, __LINE__, #a, (double)_a, #b, (double)_b,         \
               (double)_eps);                                              \
        tests_failed++;                                                   \
    }                                                                      \
} while (0)

int main(void)
{
    float d[4];

    /* ================================================================
     * (1) Non-default geometry changes the differential
     *
     * wheelbase=1.20 / track=0.50 (neither equal to the compile-time
     * defaults WHEELBASE_M=0.95 / TRACK_WIDTH_M=0.70) at a fixed 10 deg
     * angle must differ from the default-geometry result at the SAME
     * angle.  This is the direct proof that AckermannDiff_Compute()
     * consumes wheelbase_m/track_m -- i.e. whatever
     * Ackermann_SetGeometry() staged into ackermann_wheelbase /
     * ackermann_track at the real motor_control.c call site -- and not
     * the WHEELBASE_M/TRACK_WIDTH_M macros.
     * ================================================================ */
    float d_nondefault[4], d_default[4];
    AckermannDiff_Compute(10.0f, 1.20f, 0.50f, d_nondefault);
    AckermannDiff_Compute(10.0f, WHEELBASE_M, TRACK_WIDTH_M, d_default);
    ASSERT_TRUE(fabsf(d_nondefault[ACKERMANN_DIFF_FL] - d_default[ACKERMANN_DIFF_FL]) > 0.01f);
    ASSERT_TRUE(fabsf(d_nondefault[ACKERMANN_DIFF_RL] - d_default[ACKERMANN_DIFF_RL]) > 0.01f);
    ASSERT_NEAR(d_nondefault[ACKERMANN_DIFF_FL], 0.96326524f, 1e-5f);
    ASSERT_NEAR(d_nondefault[ACKERMANN_DIFF_RL], 0.96326524f, 1e-5f);
    ASSERT_NEAR(d_nondefault[ACKERMANN_DIFF_FR], 1.0f, 1e-6f);
    ASSERT_NEAR(d_nondefault[ACKERMANN_DIFF_RR], 1.0f, 1e-6f);
    ASSERT_NEAR(d_default[ACKERMANN_DIFF_FL], 0.93503743f, 1e-5f);

    /* Right turn with non-default geometry: the inside/outside sides swap
     * to FR/RR, showing the same geometry sensitivity on the other axle
     * side. */
    AckermannDiff_Compute(-10.0f, 1.20f, 0.50f, d);
    ASSERT_NEAR(d[ACKERMANN_DIFF_FR], 0.96326524f, 1e-5f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_RR], 0.96326524f, 1e-5f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_FL], 1.0f, 1e-6f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_RL], 1.0f, 1e-6f);

    /* Wheelbase-only variation (track held at the default 0.70) must, on
     * its own, move the result away from the all-default baseline -- the
     * calculation cannot be secretly ignoring wheelbase_m while only
     * reacting to track_m. */
    float d_wb_only[4];
    AckermannDiff_Compute(10.0f, 1.20f, TRACK_WIDTH_M, d_wb_only);
    ASSERT_NEAR(d_wb_only[ACKERMANN_DIFF_FL], 0.94857132f, 1e-5f);
    ASSERT_TRUE(fabsf(d_wb_only[ACKERMANN_DIFF_FL] - d_default[ACKERMANN_DIFF_FL]) > 0.005f);

    /* Track-only variation (wheelbase held at the default 0.95) must,
     * likewise, move the result on its own -- the inverse blind spot. */
    float d_trk_only[4];
    AckermannDiff_Compute(10.0f, WHEELBASE_M, 0.50f, d_trk_only);
    ASSERT_NEAR(d_trk_only[ACKERMANN_DIFF_FL], 0.95359814f, 1e-5f);
    ASSERT_TRUE(fabsf(d_trk_only[ACKERMANN_DIFF_FL] - d_default[ACKERMANN_DIFF_FL]) > 0.005f);

    /* ================================================================
     * (2) Centred steering => EXACTLY 1.000 on all 4 wheels, regardless
     * of geometry.  This is also the precondition WheelEqTest_Begin()
     * (wheel_equality_test.c, WHEQ_WHEEL_VERDICT_FAIL_ACKERMANN_OFFSET)
     * relies on to avoid a false-positive precondition failure when the
     * steering wheel really is centred -- exact equality (not just
     * "within TRACTION_OUTPUT_UNITY_EPSILON") is used here because the
     * deadband branch assigns the 1.0f literal directly, with no
     * floating-point arithmetic involved.
     * ================================================================ */
    AckermannDiff_Compute(0.0f, 1.20f, 0.50f, d);
    ASSERT_TRUE(d[ACKERMANN_DIFF_FL] == 1.0f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_FR] == 1.0f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_RL] == 1.0f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_RR] == 1.0f);

    AckermannDiff_Compute(0.0f, WHEELBASE_M, TRACK_WIDTH_M, d);
    ASSERT_TRUE(d[ACKERMANN_DIFF_FL] == 1.0f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_FR] == 1.0f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_RL] == 1.0f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_RR] == 1.0f);

    /* ================================================================
     * (3) Reduction-only clamp
     *
     * The outside-wheel multiplier must never exceed 1.0, for ANY
     * geometry within the valid staged range (geometry_store.h
     * GEOMETRY_*_MIN/MAX -- the full range GeometryStore_Stage() will
     * accept) and ANY angle from 0 to 90 deg.  Also confirm the clamp
     * genuinely reduces the inside wheel below 1.0 somewhere in the sweep
     * (i.e. it is not a no-op that happens to never be exercised).
     * ================================================================ */
    static const float geoms[][2] = {
        { GEOMETRY_WHEELBASE_M_MIN, GEOMETRY_TRACK_WIDTH_M_MIN },
        { GEOMETRY_WHEELBASE_M_MIN, GEOMETRY_TRACK_WIDTH_M_MAX },
        { GEOMETRY_WHEELBASE_M_MAX, GEOMETRY_TRACK_WIDTH_M_MIN },
        { GEOMETRY_WHEELBASE_M_MAX, GEOMETRY_TRACK_WIDTH_M_MAX },
        { WHEELBASE_M,              TRACK_WIDTH_M },
        { 1.20f,                    0.50f },
    };
    bool saw_real_reduction = false;
    for (size_t g = 0; g < sizeof(geoms) / sizeof(geoms[0]); g++) {
        for (float ang = 0.0f; ang <= 90.0f; ang += 0.5f) {
            float dd[4];
            AckermannDiff_Compute(ang, geoms[g][0], geoms[g][1], dd);
            for (int i = 0; i < 4; i++) {
                ASSERT_TRUE(dd[i] <= 1.0f);
                ASSERT_TRUE(dd[i] >= 0.0f);
            }
            if (dd[ACKERMANN_DIFF_FL] < 0.999f) saw_real_reduction = true;
        }
        /* Mirror sweep on the negative (right-turn) side too. */
        for (float ang = -90.0f; ang <= 0.0f; ang += 0.5f) {
            float dd[4];
            AckermannDiff_Compute(ang, geoms[g][0], geoms[g][1], dd);
            for (int i = 0; i < 4; i++) {
                ASSERT_TRUE(dd[i] <= 1.0f);
                ASSERT_TRUE(dd[i] >= 0.0f);
            }
        }
    }
    ASSERT_TRUE(saw_real_reduction);

    /* ================================================================
     * (4) Deadband guard: below 2 deg (regardless of geometry), no
     * correction is applied at all -- and at exactly the 2 deg boundary
     * (strict '<' comparison in production) the correction DOES engage,
     * confirming the boundary has not drifted to '<=' or some other
     * off-by-one.
     * ================================================================ */
    AckermannDiff_Compute(1.999999f, 1.20f, 0.50f, d);
    ASSERT_TRUE(d[ACKERMANN_DIFF_FL] == 1.0f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_RL] == 1.0f);

    AckermannDiff_Compute(-1.9f, 1.20f, 0.50f, d);
    ASSERT_TRUE(d[ACKERMANN_DIFF_FR] == 1.0f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_RR] == 1.0f);

    AckermannDiff_Compute(2.0f, 1.20f, 0.50f, d);
    ASSERT_NEAR(d[ACKERMANN_DIFF_FL], 0.99272484f, 1e-5f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_FL] < 1.0f);

    /* ================================================================
     * (5) tan_angle < 0.001f guard: only reachable beyond 90 deg (tan()
     * goes negative there), never via the nominal 0..MAX_STEER_DEG (54
     * deg) steering range -- confirms the second, independent guard also
     * still short-circuits to the all-1.0 default.
     * ================================================================ */
    AckermannDiff_Compute(91.0f, 1.20f, 0.50f, d);
    ASSERT_TRUE(d[ACKERMANN_DIFF_FL] == 1.0f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_FR] == 1.0f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_RL] == 1.0f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_RR] == 1.0f);

    AckermannDiff_Compute(95.0f, WHEELBASE_M, TRACK_WIDTH_M, d);
    ASSERT_TRUE(d[ACKERMANN_DIFF_FL] == 1.0f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_FR] == 1.0f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_RL] == 1.0f);
    ASSERT_TRUE(d[ACKERMANN_DIFF_RR] == 1.0f);

    /* ================================================================
     * (6) Default-geometry parity: with WHEELBASE_M/TRACK_WIDTH_M
     * (0.95/0.70), the output must be identical to the pre-extraction
     * compute_ackermann_differential() at 5 representative angles: 0 deg,
     * within-deadband, an unclamped mid-angle, a clamped mid-angle, and
     * MAX_STEER_DEG.  Expected values were generated directly from this
     * same production algorithm before extraction (bit-identical logic).
     * ================================================================ */
    /* a) 0 deg */
    AckermannDiff_Compute(0.0f, WHEELBASE_M, TRACK_WIDTH_M, d);
    ASSERT_TRUE(d[ACKERMANN_DIFF_FL] == 1.0f && d[ACKERMANN_DIFF_FR] == 1.0f &&
                d[ACKERMANN_DIFF_RL] == 1.0f && d[ACKERMANN_DIFF_RR] == 1.0f);

    /* b) within deadband (1.5 deg) */
    AckermannDiff_Compute(1.5f, WHEELBASE_M, TRACK_WIDTH_M, d);
    ASSERT_TRUE(d[ACKERMANN_DIFF_FL] == 1.0f && d[ACKERMANN_DIFF_FR] == 1.0f &&
                d[ACKERMANN_DIFF_RL] == 1.0f && d[ACKERMANN_DIFF_RR] == 1.0f);

    /* c) unclamped mid-angle (10 deg) */
    AckermannDiff_Compute(10.0f, WHEELBASE_M, TRACK_WIDTH_M, d);
    ASSERT_NEAR(d[ACKERMANN_DIFF_FL], 0.93503743f, 1e-5f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_RL], 0.93503743f, 1e-5f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_FR], 1.0f, 1e-6f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_RR], 1.0f, 1e-6f);

    /* d) clamped mid-angle (30 deg -- correction saturates at ACKERMANN_MAX_DIFF) */
    AckermannDiff_Compute(30.0f, WHEELBASE_M, TRACK_WIDTH_M, d);
    ASSERT_NEAR(d[ACKERMANN_DIFF_FL], 0.85000002f, 1e-5f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_RL], 0.85000002f, 1e-5f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_FR], 1.0f, 1e-6f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_RR], 1.0f, 1e-6f);

    /* e) MAX_STEER_DEG (54 deg, left and right) */
    AckermannDiff_Compute(MAX_STEER_DEG, WHEELBASE_M, TRACK_WIDTH_M, d);
    ASSERT_NEAR(d[ACKERMANN_DIFF_FL], 0.85000002f, 1e-5f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_RL], 0.85000002f, 1e-5f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_FR], 1.0f, 1e-6f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_RR], 1.0f, 1e-6f);

    AckermannDiff_Compute(-MAX_STEER_DEG, WHEELBASE_M, TRACK_WIDTH_M, d);
    ASSERT_NEAR(d[ACKERMANN_DIFF_FR], 0.85000002f, 1e-5f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_RR], 0.85000002f, 1e-5f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_FL], 1.0f, 1e-6f);
    ASSERT_NEAR(d[ACKERMANN_DIFF_RL], 1.0f, 1e-6f);

    printf("test_ackermann_diff: %d run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}

#endif /* HOST_TEST */
