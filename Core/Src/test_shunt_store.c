/**
  ****************************************************************************
  * @file    test_shunt_store.c
  * @brief   Host-compilable unit tests for shunt_store.c (service C3).
  *
  *          Covers:
  *            - GetDefaults() is bit-for-bit identical to
  *              INA226_SHUNT_MOHM_BATTERY / INA226_SHUNT_MOHM_MOTOR per
  *              channel (audit V1).
  *            - Validate()/Stage() reject out-of-range, NaN and infinite
  *              values instead of silently clamping (audit V6c/V6d).
  *            - GetEffectiveMohm() returns the RAM-staged value the same
  *              cycle Stage() is called, consistent with every sibling
  *              Bloque C store's GetEffective*() (audit V5: confirms the
  *              real, wired consumer -- sensor_manager.c's Current_ReadAll()
  *              -- sees a staged calibration change immediately, and Save()
  *              additionally persists it to flash for reboot survival).
  *            - Save() refuses to persist unless Safety_GetState() ==
  *              SYS_STATE_STANDBY (audit V3d), while Stage() remains usable
  *              at any system state.
  *
  *          Compile with host GCC (include production source):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -lm \
  *                Core/Src/test_shunt_store.c \
  *                Core/Src/shunt_store.c \
  *                -o test_shunt_store
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

#include "shunt_store.h"
#include "project_config.h"
#include "safety_system.h"   /* real SystemState_t / Safety_Error_t (V1) */

/* Safety_GetState() is called by ShuntStore_Save(); safety_system.h
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
     * per-channel macros they replace. ---- */
    ShuntCal_t d;
    ShuntStore_GetDefaults(&d);
    for (uint32_t i = 0; i < SHUNT_STORE_NUM_CHANNELS; i++) {
        float expected = (i == INA226_CHANNEL_BATTERY) ? (float)INA226_SHUNT_MOHM_BATTERY
                                                        : (float)INA226_SHUNT_MOHM_MOTOR;
        ASSERT_TRUE(d.mohm[i] == expected);
    }
    ASSERT_TRUE(ShuntStore_Validate(&d));

    /* ---- V6c/d: boundary + NaN/Inf/negative rejection (every channel) ---- */
    ShuntCal_t s = d;
    s.mohm[0] = SHUNT_STORE_MOHM_MIN;
    ASSERT_TRUE(ShuntStore_Validate(&s));
    s.mohm[0] = SHUNT_STORE_MOHM_MAX;
    ASSERT_TRUE(ShuntStore_Validate(&s));
    s.mohm[0] = SHUNT_STORE_MOHM_MIN - 0.001f;
    ASSERT_FALSE(ShuntStore_Validate(&s));
    s.mohm[0] = SHUNT_STORE_MOHM_MAX + 0.001f;
    ASSERT_FALSE(ShuntStore_Validate(&s));

    for (uint32_t i = 0; i < SHUNT_STORE_NUM_CHANNELS; i++) {
        s = d; s.mohm[i] = NAN;      ASSERT_FALSE(ShuntStore_Validate(&s));
        s = d; s.mohm[i] = INFINITY; ASSERT_FALSE(ShuntStore_Validate(&s));
        s = d; s.mohm[i] = -1.0f;    ASSERT_FALSE(ShuntStore_Validate(&s));  /* negative */
    }

    ASSERT_FALSE(ShuntStore_Validate(NULL));

    /* ---- V5: GetEffectiveMohm() reflects the RAM-staged value the SAME
     * cycle Stage() is called -- this is the real, wired production
     * behaviour: sensor_manager.c's Current_ReadAll() reads
     * ShuntStore_GetEffectiveMohm() every cycle, so a staged calibration
     * change is visible immediately, exactly like every sibling Bloque C
     * store's GetEffective*(). No Save()/Init() needed for this part. ---- */
    ShuntCal_t staged_only = d;
    staged_only.mohm[0] = 2.5f;   /* well within range, deliberately different */
    ASSERT_TRUE(ShuntStore_Stage(&staged_only));
    ASSERT_TRUE(ShuntStore_GetEffectiveMohm(0) == 2.5f);

    /* Out-of-range channel index must fail safe (never divide by zero
     * downstream): falls back to the motor default, not 0. */
    ASSERT_TRUE(ShuntStore_GetEffectiveMohm(SHUNT_STORE_NUM_CHANNELS) ==
                (float)INA226_SHUNT_MOHM_MOTOR);

    /* ---- V3d: Save() must refuse to persist outside STANDBY (Stage()'s
     * RAM effect above is untouched by the rejection), and must succeed
     * once STANDBY is restored (single real write -- avoids the
     * pre-existing, unrelated SHUNT_WRITE_MIN_INTERVAL_MS rate limiter
     * that a second back-to-back content-changing Save() would trip;
     * Init() is intentionally never called in this host test since it
     * dereferences the raw flash address, unmapped on a host process). ---- */
    s_stub_state = SYS_STATE_ACTIVE;
    ASSERT_FALSE(ShuntStore_Save());
    ASSERT_TRUE(ShuntStore_GetEffectiveMohm(0) == 2.5f);   /* still staged, unaffected */

    s_stub_state = SYS_STATE_STANDBY;
    ASSERT_TRUE(ShuntStore_Save());
    ASSERT_TRUE(ShuntStore_GetEffectiveMohm(0) == 2.5f);   /* now persisted too */

    /* ---- V6b: Revert() discards unsaved staged edits and falls back to
     * the last SAVED value (RAM-only, no further Save() calls needed --
     * avoids the write rate-limiter). ---- */
    ShuntCal_t another = d;
    another.mohm[0] = 4.0f;
    ASSERT_TRUE(ShuntStore_Stage(&another));
    ASSERT_TRUE(ShuntStore_GetEffectiveMohm(0) == 4.0f);  /* staged takes effect again */
    ShuntStore_Revert();
    ShuntCal_t reverted;
    ShuntStore_GetStaged(&reverted);
    ASSERT_TRUE(reverted.mohm[0] == 2.5f);   /* back to last SAVE, not 4.0 */
    ASSERT_TRUE(ShuntStore_GetEffectiveMohm(0) == 2.5f);

    printf("test_shunt_store: %d run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}

#endif /* HOST_TEST */
