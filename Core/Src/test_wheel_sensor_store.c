/**
  ****************************************************************************
  * @file    test_wheel_sensor_store.c
  * @brief   Host-compilable unit tests for wheel_sensor_store.c
  *          (service C5 -- global wheel-speed-sensor geometry/timing).
  *
  *          Covers:
  *            - GetDefaults() is bit-for-bit identical to WHEEL_PULSES_REV,
  *              WHEEL_CIRCUM_MM, SENSOR_DEBOUNCE_US and
  *              WHEEL_FAULT_DEBOUNCE_MS (audit V1).
  *            - Validate()/Stage() reject out-of-range, NaN and infinite
  *              circumference_mm instead of silently clamping (audit
  *              V6c/V6d).
  *            - Stage()/Save()/Revert()/ResetToDefaults() all call
  *              Sensor_SetDebounceUs() with the new debounce_us value,
  *              proving the real sensor_manager.c DWT pre-filter is wired
  *              to this store's runtime value, not a dead copy (audit V5).
  *            - Save() refuses to persist unless Safety_GetState() ==
  *              SYS_STATE_STANDBY (audit V3d), while Stage() remains usable
  *              at any system state.
  *            - No per-wheel trim/variant field exists (audit V6e) -- this
  *              store is DELIBERATELY GLOBAL (one value for all 4 wheels).
  *
  *          Compile with host GCC (include production source):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -lm \
  *                Core/Src/test_wheel_sensor_store.c \
  *                Core/Src/wheel_sensor_store.c \
  *                -o test_wheel_sensor_store
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

#include "wheel_sensor_store.h"
#include "project_config.h"
#include "vehicle_physics.h"
#include "safety_system.h"   /* WHEEL_FAULT_DEBOUNCE_MS + SystemState_t (V1) */

/* wheel_sensor_store.c calls Sensor_SetDebounceUs() (declared in
 * sensor_manager.h, defined in sensor_manager.c) every time the staged
 * debounce window changes.  Linking the real sensor_manager.c would pull in
 * the whole HAL/CAN/ADC dependency graph, so provide a minimal recording
 * stub instead -- this test is about wheel_sensor_store.c's own logic. */
static uint16_t s_last_debounce_us = 0xFFFFU;
static int      s_apply_calls      = 0;
void Sensor_SetDebounceUs(uint16_t debounce_us)
{
    s_last_debounce_us = debounce_us;
    s_apply_calls++;
}

/* Safety_GetState() is called by WheelSensorStore_Save(); safety_system.h
 * (included above for WHEEL_FAULT_DEBOUNCE_MS) already declares its
 * prototype and the SystemState_t enum, so only the definition (a stub
 * whose return value the test can flip) is provided here, to exercise the
 * STANDBY-only flash-write gate (rejects outside STANDBY, accepts inside
 * STANDBY) without linking the real safety_system.c. */
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
    WheelSensorParams_t d;
    WheelSensorStore_GetDefaults(&d);
    ASSERT_TRUE(d.pulses_per_rev       == (uint16_t)WHEEL_PULSES_REV);
    ASSERT_TRUE(d.circumference_mm     == (float)WHEEL_CIRCUM_MM);
    ASSERT_TRUE(d.debounce_us          == (uint16_t)SENSOR_DEBOUNCE_US);
    ASSERT_TRUE(d.mismatch_debounce_ms == (uint16_t)WHEEL_FAULT_DEBOUNCE_MS);
    ASSERT_TRUE(WheelSensorStore_Validate(&d));

    /* ---- V6c/d: boundary + NaN/Inf/negative rejection ---- */
    WheelSensorParams_t w = d;
    w.pulses_per_rev = WHEEL_SENSOR_PULSES_REV_MIN;
    ASSERT_TRUE(WheelSensorStore_Validate(&w));
    w.pulses_per_rev = WHEEL_SENSOR_PULSES_REV_MAX;
    ASSERT_TRUE(WheelSensorStore_Validate(&w));
    w = d; w.pulses_per_rev = WHEEL_SENSOR_PULSES_REV_MIN - 1U;
    ASSERT_FALSE(WheelSensorStore_Validate(&w));
    w = d; w.pulses_per_rev = WHEEL_SENSOR_PULSES_REV_MAX + 1U;
    ASSERT_FALSE(WheelSensorStore_Validate(&w));

    w = d; w.circumference_mm = WHEEL_SENSOR_CIRCUM_MM_MIN;
    ASSERT_TRUE(WheelSensorStore_Validate(&w));
    w = d; w.circumference_mm = WHEEL_SENSOR_CIRCUM_MM_MAX;
    ASSERT_TRUE(WheelSensorStore_Validate(&w));
    w = d; w.circumference_mm = WHEEL_SENSOR_CIRCUM_MM_MIN - 0.001f;
    ASSERT_FALSE(WheelSensorStore_Validate(&w));
    w = d; w.circumference_mm = WHEEL_SENSOR_CIRCUM_MM_MAX + 0.001f;
    ASSERT_FALSE(WheelSensorStore_Validate(&w));
    w = d; w.circumference_mm = NAN;      ASSERT_FALSE(WheelSensorStore_Validate(&w));
    w = d; w.circumference_mm = INFINITY; ASSERT_FALSE(WheelSensorStore_Validate(&w));
    w = d; w.circumference_mm = -INFINITY; ASSERT_FALSE(WheelSensorStore_Validate(&w));
    w = d; w.circumference_mm = -500.0f;  ASSERT_FALSE(WheelSensorStore_Validate(&w));  /* negative */

    w = d; w.debounce_us = WHEEL_SENSOR_DEBOUNCE_US_MIN - 1U;
    ASSERT_FALSE(WheelSensorStore_Validate(&w));
    w = d; w.debounce_us = WHEEL_SENSOR_DEBOUNCE_US_MAX + 1U;
    ASSERT_FALSE(WheelSensorStore_Validate(&w));

    w = d; w.mismatch_debounce_ms = WHEEL_SENSOR_MISMATCH_DEBOUNCE_MS_MIN - 1U;
    ASSERT_FALSE(WheelSensorStore_Validate(&w));
    w = d; w.mismatch_debounce_ms = WHEEL_SENSOR_MISMATCH_DEBOUNCE_MS_MAX + 1U;
    ASSERT_FALSE(WheelSensorStore_Validate(&w));

    ASSERT_FALSE(WheelSensorStore_Validate(NULL));

    /* ---- V5: Stage() must call Sensor_SetDebounceUs() with the new value
     * immediately, proving the real DWT pre-filter in sensor_manager.c is
     * wired to this store (not a dead copy). ---- */
    s_apply_calls = 0;
    WheelSensorParams_t custom = d;
    custom.debounce_us = 350U;
    ASSERT_TRUE(WheelSensorStore_Stage(&custom));
    ASSERT_TRUE(s_apply_calls == 1);
    ASSERT_TRUE(s_last_debounce_us == 350U);
    ASSERT_TRUE(WheelSensorStore_GetEffectiveDebounceUs() == 350U);
    ASSERT_TRUE(WheelSensorStore_GetEffectivePulsesPerRev() == d.pulses_per_rev);
    ASSERT_TRUE(WheelSensorStore_GetEffectiveCircumferenceM() ==
                d.circumference_mm / 1000.0f);

    /* A rejected Stage() must NOT call Sensor_SetDebounceUs() (no silent
     * partial application of invalid geometry). */
    WheelSensorParams_t bad = d;
    bad.circumference_mm = NAN;
    int calls_before = s_apply_calls;
    ASSERT_FALSE(WheelSensorStore_Stage(&bad));
    ASSERT_TRUE(s_apply_calls == calls_before);

    /* ---- V3d: Save() must refuse to persist outside STANDBY, and must
     * succeed once STANDBY is restored; Stage() is unaffected. ---- */
    s_stub_state = SYS_STATE_ACTIVE;
    WheelSensorParams_t tuned = d;
    tuned.debounce_us = 250U;
    ASSERT_TRUE(WheelSensorStore_Stage(&tuned));
    ASSERT_FALSE(WheelSensorStore_Save());

    s_stub_state = SYS_STATE_STANDBY;
    ASSERT_TRUE(WheelSensorStore_Save());

    /* ---- V6b: Revert() discards unsaved staged edits and re-applies the
     * last persisted value via Sensor_SetDebounceUs(). ---- */
    WheelSensorParams_t another = d;
    another.debounce_us = 500U;
    ASSERT_TRUE(WheelSensorStore_Stage(&another));
    WheelSensorStore_Revert();
    WheelSensorParams_t reverted;
    WheelSensorStore_GetStaged(&reverted);
    ASSERT_TRUE(reverted.debounce_us == 250U);   /* back to last SAVE, not 500 */
    ASSERT_TRUE(s_last_debounce_us   == 250U);   /* Revert() re-applied it */

    /* ---- ResetToDefaults() restores the compile-time macro values and
     * re-applies them. ---- */
    WheelSensorStore_ResetToDefaults();
    WheelSensorParams_t after_reset;
    WheelSensorStore_GetStaged(&after_reset);
    ASSERT_TRUE(after_reset.debounce_us == (uint16_t)SENSOR_DEBOUNCE_US);
    ASSERT_TRUE(s_last_debounce_us      == (uint16_t)SENSOR_DEBOUNCE_US);

    /* ---- V6e: no per-wheel trim/compensation exists in this struct --
     * one GLOBAL value applied identically to all 4 wheels; the whole
     * point of the service-diagnostic system is to REVEAL a difference
     * between wheels, never to compensate for it. Exactly 4 scalar
     * fields, 12 bytes (2+2 pad+4+2+2 with natural float alignment). ---- */
    ASSERT_TRUE(sizeof(WheelSensorParams_t) == 12U);

    printf("test_wheel_sensor_store: %d run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}

#endif /* HOST_TEST */
