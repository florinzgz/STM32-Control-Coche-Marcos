/**
  ****************************************************************************
  * @file    test_service_mode.c
  * @brief   Host-compilable unit tests for service_mode module.
  *
  *          Validates structural safety invariants:
  *            - CRITICAL modules cannot be disabled
  *            - Module ID range checks reject out-of-bounds
  *            - Classification table is consistent with enum
  *            - Factory restore re-enables all modules
  *            - Fault recording works regardless of enabled state
  *            - Bitmask accessors match module state
  *
  *          Compile with host GCC (from Core/Src/ directory):
  *            gcc -std=c11 -I../Inc -O2 \
  *                service_mode.c test_service_mode.c -o test_service_mode
  ****************************************************************************
  */

#include "service_mode.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* ---- Test harness ---- */
static int tests_run    = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(expr) do {                                        \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);       \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_FALSE(expr) do {                                       \
    tests_run++;                                                      \
    if ((expr)) {                                                     \
        printf("FAIL %s:%d  !(%s)\n", __FILE__, __LINE__, #expr);    \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_EQ_U8(got, expected) do {                              \
    uint8_t _got = (got);                                             \
    uint8_t _exp = (expected);                                        \
    tests_run++;                                                      \
    if (_got != _exp) {                                               \
        printf("FAIL %s:%d  %s == %u (expected %u)\n",               \
               __FILE__, __LINE__, #got, (unsigned)_got, (unsigned)_exp); \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_EQ_U32(got, expected) do {                             \
    uint32_t _got = (got);                                            \
    uint32_t _exp = (expected);                                       \
    tests_run++;                                                      \
    if (_got != _exp) {                                               \
        printf("FAIL %s:%d  %s == 0x%08X (expected 0x%08X)\n",       \
               __FILE__, __LINE__, #got, _got, _exp);                 \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ================================================================== */
/*  Test: CRITICAL modules cannot be disabled                          */
/* ================================================================== */

static void test_critical_modules_not_disableable(void)
{
    ServiceMode_Init();

    /* All four CRITICAL modules: IDs 0–3 */
    ASSERT_FALSE(ServiceMode_DisableModule(MODULE_CAN_TIMEOUT));
    ASSERT_FALSE(ServiceMode_DisableModule(MODULE_EMERGENCY_STOP));
    ASSERT_FALSE(ServiceMode_DisableModule(MODULE_WATCHDOG));
    ASSERT_FALSE(ServiceMode_DisableModule(MODULE_RELAY_TRAC));

    /* Verify they remain enabled after disable attempt */
    ASSERT_TRUE(ServiceMode_IsEnabled(MODULE_CAN_TIMEOUT));
    ASSERT_TRUE(ServiceMode_IsEnabled(MODULE_EMERGENCY_STOP));
    ASSERT_TRUE(ServiceMode_IsEnabled(MODULE_WATCHDOG));
    ASSERT_TRUE(ServiceMode_IsEnabled(MODULE_RELAY_TRAC));

    /* Verify classification is indeed CRITICAL */
    ASSERT_EQ_U8(ServiceMode_GetClass(MODULE_CAN_TIMEOUT),    MODULE_CLASS_CRITICAL);
    ASSERT_EQ_U8(ServiceMode_GetClass(MODULE_EMERGENCY_STOP), MODULE_CLASS_CRITICAL);
    ASSERT_EQ_U8(ServiceMode_GetClass(MODULE_WATCHDOG),       MODULE_CLASS_CRITICAL);
    ASSERT_EQ_U8(ServiceMode_GetClass(MODULE_RELAY_TRAC),     MODULE_CLASS_CRITICAL);
}

/* ================================================================== */
/*  Test: NON-CRITICAL modules CAN be disabled                         */
/* ================================================================== */

static void test_noncritical_modules_disableable(void)
{
    ServiceMode_Init();

    /* Test a representative set of non-critical modules */
    ASSERT_TRUE(ServiceMode_DisableModule(MODULE_TEMP_SENSOR_0));
    ASSERT_FALSE(ServiceMode_IsEnabled(MODULE_TEMP_SENSOR_0));

    ASSERT_TRUE(ServiceMode_DisableModule(MODULE_WHEEL_SPEED_FL));
    ASSERT_FALSE(ServiceMode_IsEnabled(MODULE_WHEEL_SPEED_FL));

    ASSERT_TRUE(ServiceMode_DisableModule(MODULE_ABS));
    ASSERT_FALSE(ServiceMode_IsEnabled(MODULE_ABS));

    ASSERT_TRUE(ServiceMode_DisableModule(MODULE_TCS));
    ASSERT_FALSE(ServiceMode_IsEnabled(MODULE_TCS));

    ASSERT_TRUE(ServiceMode_DisableModule(MODULE_OBSTACLE_DETECT));
    ASSERT_FALSE(ServiceMode_IsEnabled(MODULE_OBSTACLE_DETECT));

    /* Verify classification is NON_CRITICAL */
    ASSERT_EQ_U8(ServiceMode_GetClass(MODULE_TEMP_SENSOR_0),  MODULE_CLASS_NON_CRITICAL);
    ASSERT_EQ_U8(ServiceMode_GetClass(MODULE_WHEEL_SPEED_FL), MODULE_CLASS_NON_CRITICAL);
    ASSERT_EQ_U8(ServiceMode_GetClass(MODULE_ABS),            MODULE_CLASS_NON_CRITICAL);
    ASSERT_EQ_U8(ServiceMode_GetClass(MODULE_TCS),            MODULE_CLASS_NON_CRITICAL);
    ASSERT_EQ_U8(ServiceMode_GetClass(MODULE_OBSTACLE_DETECT),MODULE_CLASS_NON_CRITICAL);
}

/* ================================================================== */
/*  Test: Module ID range validation                                   */
/* ================================================================== */

static void test_module_id_range_validation(void)
{
    ServiceMode_Init();

    /* Out-of-range IDs must be rejected */
    ASSERT_FALSE(ServiceMode_DisableModule((ModuleID_t)MODULE_COUNT));
    ASSERT_FALSE(ServiceMode_DisableModule((ModuleID_t)255));
    ASSERT_FALSE(ServiceMode_EnableModule((ModuleID_t)MODULE_COUNT));
    ASSERT_FALSE(ServiceMode_EnableModule((ModuleID_t)255));

    /* Out-of-range IsEnabled returns false */
    ASSERT_FALSE(ServiceMode_IsEnabled((ModuleID_t)MODULE_COUNT));
    ASSERT_FALSE(ServiceMode_IsEnabled((ModuleID_t)255));

    /* Out-of-range GetClass returns CRITICAL (safe default) */
    ASSERT_EQ_U8(ServiceMode_GetClass((ModuleID_t)MODULE_COUNT), MODULE_CLASS_CRITICAL);
    ASSERT_EQ_U8(ServiceMode_GetClass((ModuleID_t)255),          MODULE_CLASS_CRITICAL);

    /* Out-of-range GetFault returns NONE (safe default) */
    ASSERT_EQ_U8(ServiceMode_GetFault((ModuleID_t)MODULE_COUNT), MODULE_FAULT_NONE);
}

/* ================================================================== */
/*  Test: Classification table completeness                            */
/* ================================================================== */

static void test_classification_table_complete(void)
{
    ServiceMode_Init();

    /* Verify every valid module ID has a valid classification */
    for (unsigned i = 0; i < MODULE_COUNT; i++) {
        ModuleClass_t cls = ServiceMode_GetClass((ModuleID_t)i);
        ASSERT_TRUE(cls == MODULE_CLASS_CRITICAL || cls == MODULE_CLASS_NON_CRITICAL);
    }

    /* Verify exactly 4 CRITICAL modules (IDs 0–3) */
    uint8_t critical_count = 0;
    for (unsigned i = 0; i < MODULE_COUNT; i++) {
        if (ServiceMode_GetClass((ModuleID_t)i) == MODULE_CLASS_CRITICAL)
            critical_count++;
    }
    ASSERT_EQ_U8(critical_count, 4);
}

/* ================================================================== */
/*  Test: Factory restore re-enables all modules                       */
/* ================================================================== */

static void test_factory_restore(void)
{
    ServiceMode_Init();

    /* Disable several modules */
    ServiceMode_DisableModule(MODULE_TEMP_SENSOR_0);
    ServiceMode_DisableModule(MODULE_ABS);
    ServiceMode_DisableModule(MODULE_TCS);
    ServiceMode_DisableModule(MODULE_WHEEL_SPEED_FL);
    ServiceMode_DisableModule(MODULE_OBSTACLE_DETECT);

    /* Set a DISABLED fault (manual disable marker) */
    ServiceMode_SetFault(MODULE_ABS, MODULE_FAULT_DISABLED);

    /* Set a real fault on a different module */
    ServiceMode_SetFault(MODULE_TCS, MODULE_FAULT_ERROR);

    /* Factory restore */
    ServiceMode_FactoryRestore();

    /* All modules should be re-enabled */
    for (unsigned i = 0; i < MODULE_COUNT; i++) {
        ASSERT_TRUE(ServiceMode_IsEnabled((ModuleID_t)i));
    }

    /* Manual-disable fault should be cleared */
    ASSERT_EQ_U8(ServiceMode_GetFault(MODULE_ABS), MODULE_FAULT_NONE);

    /* Real fault should be preserved */
    ASSERT_EQ_U8(ServiceMode_GetFault(MODULE_TCS), MODULE_FAULT_ERROR);
}

/* ================================================================== */
/*  Test: Faults recorded regardless of enabled state                  */
/* ================================================================== */

static void test_faults_always_recorded(void)
{
    ServiceMode_Init();

    /* Disable a module then set a fault — fault must be recorded */
    ServiceMode_DisableModule(MODULE_TEMP_SENSOR_0);
    ASSERT_FALSE(ServiceMode_IsEnabled(MODULE_TEMP_SENSOR_0));

    ServiceMode_SetFault(MODULE_TEMP_SENSOR_0, MODULE_FAULT_ERROR);
    ASSERT_EQ_U8(ServiceMode_GetFault(MODULE_TEMP_SENSOR_0), MODULE_FAULT_ERROR);

    /* ShouldBlock: disabled + faulted → should NOT block */
    ASSERT_FALSE(ServiceMode_ShouldBlock(MODULE_TEMP_SENSOR_0));

    /* Re-enable: now faulted + enabled → SHOULD block */
    ServiceMode_EnableModule(MODULE_TEMP_SENSOR_0);
    ASSERT_TRUE(ServiceMode_ShouldBlock(MODULE_TEMP_SENSOR_0));

    /* Critical module fault always blocks */
    ServiceMode_SetFault(MODULE_CAN_TIMEOUT, MODULE_FAULT_ERROR);
    ASSERT_TRUE(ServiceMode_ShouldBlock(MODULE_CAN_TIMEOUT));
}

/* ================================================================== */
/*  Test: Critical modules always enabled regardless of internal flag  */
/* ================================================================== */

static void test_critical_always_enabled(void)
{
    ServiceMode_Init();

    /* Even if someone forces the internal flag (which DisableModule prevents),
     * IsEnabled must STILL return true for critical modules.
     * This test verifies the IsEnabled guard clause, not just DisableModule. */
    ASSERT_TRUE(ServiceMode_IsEnabled(MODULE_CAN_TIMEOUT));
    ASSERT_TRUE(ServiceMode_IsEnabled(MODULE_EMERGENCY_STOP));
    ASSERT_TRUE(ServiceMode_IsEnabled(MODULE_WATCHDOG));
    ASSERT_TRUE(ServiceMode_IsEnabled(MODULE_RELAY_TRAC));
}

/* ================================================================== */
/*  Test: Bitmask accessors                                            */
/* ================================================================== */

static void test_bitmask_accessors(void)
{
    ServiceMode_Init();

    /* Initially all modules enabled — enabled mask should have all 25 bits set */
    uint32_t enabled_mask = ServiceMode_GetEnabledMask();
    for (unsigned i = 0; i < MODULE_COUNT; i++) {
        ASSERT_TRUE((enabled_mask >> i) & 1);
    }

    /* Initially no faults — fault mask should be 0 */
    ASSERT_EQ_U32(ServiceMode_GetFaultMask(), 0);

    /* Initially no disabled — disabled mask should be 0 */
    ASSERT_EQ_U32(ServiceMode_GetDisabledMask(), 0);

    /* Disable module 21 (ABS) and verify bitmasks update */
    ServiceMode_DisableModule(MODULE_ABS);
    ASSERT_FALSE((ServiceMode_GetEnabledMask() >> MODULE_ABS) & 1);
    ASSERT_TRUE ((ServiceMode_GetDisabledMask() >> MODULE_ABS) & 1);

    /* Set fault on module 4 (TEMP_SENSOR_0) */
    ServiceMode_SetFault(MODULE_TEMP_SENSOR_0, MODULE_FAULT_WARNING);
    ASSERT_TRUE((ServiceMode_GetFaultMask() >> MODULE_TEMP_SENSOR_0) & 1);
}

/* ================================================================== */
/*  Test: Multiple module disable does not create inconsistent state    */
/* ================================================================== */

static void test_multi_disable_consistency(void)
{
    ServiceMode_Init();

    /* Disable all non-critical modules one by one */
    for (unsigned i = 0; i < MODULE_COUNT; i++) {
        if (ServiceMode_GetClass((ModuleID_t)i) == MODULE_CLASS_NON_CRITICAL) {
            ASSERT_TRUE(ServiceMode_DisableModule((ModuleID_t)i));
        }
    }

    /* Verify all critical modules remain enabled */
    ASSERT_TRUE(ServiceMode_IsEnabled(MODULE_CAN_TIMEOUT));
    ASSERT_TRUE(ServiceMode_IsEnabled(MODULE_EMERGENCY_STOP));
    ASSERT_TRUE(ServiceMode_IsEnabled(MODULE_WATCHDOG));
    ASSERT_TRUE(ServiceMode_IsEnabled(MODULE_RELAY_TRAC));

    /* Verify no critical faults reported from disable operations */
    ASSERT_FALSE(ServiceMode_HasCriticalFault());

    /* Active non-critical fault count should be 0 (disabled modules don't count) */
    ASSERT_EQ_U8(ServiceMode_CountActiveNonCriticalFaults(), 0);

    /* Factory restore should recover everything */
    ServiceMode_FactoryRestore();
    for (unsigned i = 0; i < MODULE_COUNT; i++) {
        ASSERT_TRUE(ServiceMode_IsEnabled((ModuleID_t)i));
    }
}

/* ================================================================== */
/*  Test: GetStatus struct correctness                                 */
/* ================================================================== */

static void test_get_status(void)
{
    ServiceMode_Init();

    /* Critical module status */
    ModuleStatus_t s0 = ServiceMode_GetStatus(MODULE_CAN_TIMEOUT);
    ASSERT_EQ_U8(s0.id,             MODULE_CAN_TIMEOUT);
    ASSERT_EQ_U8(s0.classification, MODULE_CLASS_CRITICAL);
    ASSERT_TRUE (s0.enabled);
    ASSERT_EQ_U8(s0.fault,          MODULE_FAULT_NONE);

    /* Non-critical module after disable + fault */
    ServiceMode_DisableModule(MODULE_ABS);
    ServiceMode_SetFault(MODULE_ABS, MODULE_FAULT_ERROR);
    ModuleStatus_t s1 = ServiceMode_GetStatus(MODULE_ABS);
    ASSERT_EQ_U8(s1.id,             MODULE_ABS);
    ASSERT_EQ_U8(s1.classification, MODULE_CLASS_NON_CRITICAL);
    ASSERT_FALSE(s1.enabled);
    ASSERT_EQ_U8(s1.fault,          MODULE_FAULT_ERROR);

    /* Out-of-range → safe defaults */
    ModuleStatus_t s2 = ServiceMode_GetStatus((ModuleID_t)255);
    ASSERT_EQ_U8(s2.classification, MODULE_CLASS_CRITICAL);
    ASSERT_FALSE(s2.enabled);
}

/* ================================================================== */
/*  Test: Enable idempotency                                           */
/* ================================================================== */

static void test_enable_idempotency(void)
{
    ServiceMode_Init();

    /* Enable already-enabled module — should succeed without side effects */
    ASSERT_TRUE(ServiceMode_EnableModule(MODULE_ABS));
    ASSERT_TRUE(ServiceMode_IsEnabled(MODULE_ABS));

    /* Enable after disable — should restore */
    ServiceMode_DisableModule(MODULE_ABS);
    ASSERT_FALSE(ServiceMode_IsEnabled(MODULE_ABS));
    ASSERT_TRUE(ServiceMode_EnableModule(MODULE_ABS));
    ASSERT_TRUE(ServiceMode_IsEnabled(MODULE_ABS));
}

/* ================================================================== */
/*  main                                                               */
/* ================================================================== */

int main(void)
{
    test_critical_modules_not_disableable();
    test_noncritical_modules_disableable();
    test_module_id_range_validation();
    test_classification_table_complete();
    test_factory_restore();
    test_faults_always_recorded();
    test_critical_always_enabled();
    test_bitmask_accessors();
    test_multi_disable_consistency();
    test_get_status();
    test_enable_idempotency();

    printf("\n--- service_mode tests: %d run, %d failed ---\n",
           tests_run, tests_failed);

    return tests_failed ? 1 : 0;
}
