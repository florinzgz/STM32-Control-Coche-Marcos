/**
  ****************************************************************************
  * @file    test_steering_z.c
  * @brief   Host unit tests for the PB5 + encoder-Z dual center-reference
  *          logic (steering_z.c).  Pure logic — links the real module.
  *
  *          Compile with host GCC:
  *            gcc -std=c11 -DHOST_TEST -Ianalysis_artifacts/stubs -ICore/Inc \
  *                -O2 Core/Src/test_steering_z.c Core/Src/steering_z.c \
  *                -o /tmp/test_steering_z
  *
  *          Covers the safety-critical rules:
  *            - Z alone (no PB5) never reports a center.
  *            - PB5 + Z within window → OK (high precision).
  *            - PB5 without Z → NOT_SEEN (still operable).
  *            - PB5 + Z out of window / mechanical offset.
  *            - Offset normalisation collapses adjacent-revolution Z.
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "steering_z.h"
#include "project_config.h"

static int tests_run = 0, tests_failed = 0;

#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ---- Offset normalisation ---- */
static void test_normalise(void)
{
    CHECK(SteeringZ_NormaliseOffset(0)            == 0);
    CHECK(SteeringZ_NormaliseOffset(ENCODER_CPR)  == 0);     /* full rev */
    CHECK(SteeringZ_NormaliseOffset(ENCODER_CPR + 5) == 5);  /* +1 rev   */
    CHECK(SteeringZ_NormaliseOffset(-5)           == -5);
    CHECK(SteeringZ_NormaliseOffset(2 * ENCODER_CPR - 7) == -7);
    /* A Z one full revolution away maps to the same fundamental offset. */
    CHECK(SteeringZ_NormaliseOffset(7)            ==
          SteeringZ_NormaliseOffset(7 + ENCODER_CPR));
    CHECK(SteeringZ_NormaliseOffset(7)            ==
          SteeringZ_NormaliseOffset(7 - ENCODER_CPR));
}

/* ---- Classification rules ---- */
static void test_classify(void)
{
    /* Z without PB5 is NEVER a center. */
    CHECK(SteeringZ_Classify(false, true, 0)   == STEERING_Z_NOT_CALIBRATED);
    CHECK(SteeringZ_Classify(false, true, 5)   == STEERING_Z_NOT_CALIBRATED);

    /* PB5 confirmed, no Z pulse → operable but Z not seen. */
    CHECK(SteeringZ_Classify(true, false, 0)   == STEERING_Z_NOT_SEEN);

    /* PB5 + Z within window → OK. */
    CHECK(SteeringZ_Classify(true, true, 0)    == STEERING_Z_OK);
    CHECK(SteeringZ_Classify(true, true, STEERING_Z_WINDOW_COUNTS)  == STEERING_Z_OK);
    CHECK(SteeringZ_Classify(true, true, -STEERING_Z_WINDOW_COUNTS) == STEERING_Z_OK);

    /* Beyond window but within fault → out of window. */
    CHECK(SteeringZ_Classify(true, true, STEERING_Z_WINDOW_COUNTS + 1) == STEERING_Z_OUT_OF_WINDOW);
    CHECK(SteeringZ_Classify(true, true, STEERING_Z_FAULT_COUNTS)      == STEERING_Z_OUT_OF_WINDOW);

    /* Beyond fault threshold → mechanical offset. */
    CHECK(SteeringZ_Classify(true, true, STEERING_Z_FAULT_COUNTS + 1)  == STEERING_Z_MECH_OFFSET);
    CHECK(SteeringZ_Classify(true, true, -(STEERING_Z_FAULT_COUNTS + 1)) == STEERING_Z_MECH_OFFSET);
}

/* ---- Stateful center-confirmation ---- */
static void test_on_center_confirmed(void)
{
    /* PB5 + Z within window → valid, high-precision offset stored. */
    SteeringZ_Init();
    SteeringZ_OnCenterConfirmed(1000, 1007, true);   /* offset = +7 */
    CHECK(SteeringZ_GetStatus() == STEERING_Z_OK);
    CHECK(SteeringZ_IsValid());
    CHECK(SteeringZ_GetOffset() == 7);

    /* Z from an adjacent revolution collapses to the same offset. */
    SteeringZ_Init();
    SteeringZ_OnCenterConfirmed(1000, 1000 + ENCODER_CPR + 7, true);
    CHECK(SteeringZ_GetStatus() == STEERING_Z_OK);
    CHECK(SteeringZ_GetOffset() == 7);

    /* PB5 + Z out of window → recorded but NOT valid. */
    SteeringZ_Init();
    SteeringZ_OnCenterConfirmed(1000, 1000 + 30, true);
    CHECK(SteeringZ_GetStatus() == STEERING_Z_OUT_OF_WINDOW);
    CHECK(!SteeringZ_IsValid());
    CHECK(SteeringZ_GetOffset() == 30);

    /* PB5 but no Z pulse → not seen, not valid, never blocks. */
    SteeringZ_Init();
    SteeringZ_OnCenterConfirmed(1000, 0, false);
    CHECK(SteeringZ_GetStatus() == STEERING_Z_NOT_SEEN);
    CHECK(!SteeringZ_IsValid());

    /* PB5 + Z mechanical offset → flagged, not valid. */
    SteeringZ_Init();
    SteeringZ_OnCenterConfirmed(1000, 1000 + 60, true);
    CHECK(SteeringZ_GetStatus() == STEERING_Z_MECH_OFFSET);
    CHECK(!SteeringZ_IsValid());
}

/* ---- Flash load / clear ---- */
static void test_load_and_clear(void)
{
    SteeringZ_Init();
    SteeringZ_LoadFromFlash(12, true);
    CHECK(SteeringZ_GetOffset() == 12);
    CHECK(SteeringZ_IsValid());
    CHECK(SteeringZ_GetStatus() == STEERING_Z_OK);

    SteeringZ_LoadFromFlash(0, false);     /* v1→v2 migration: Z not valid */
    CHECK(!SteeringZ_IsValid());
    CHECK(SteeringZ_GetStatus() == STEERING_Z_NOT_CALIBRATED);

    SteeringZ_LoadFromFlash(50, true);
    SteeringZ_ClearCalibration();
    CHECK(SteeringZ_GetOffset() == 0);
    CHECK(!SteeringZ_IsValid());
    CHECK(SteeringZ_GetStatus() == STEERING_Z_NOT_CALIBRATED);
}

int main(void)
{
    test_normalise();
    test_classify();
    test_on_center_confirmed();
    test_load_and_clear();

    printf("\n%d tests run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}

#endif /* HOST_TEST */
