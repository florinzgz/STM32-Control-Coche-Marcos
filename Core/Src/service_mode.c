/**
  ****************************************************************************
  * @file    service_mode.c
  * @brief   Service Mode / Module Disable implementation
  *
  *  Traced to base firmware (FULL-FIRMWARE-Coche-Marcos):
  *    limp_mode.cpp    — NORMAL/DEGRADED/LIMP/CRITICAL state machine
  *    car_sensors.cpp  — cfg.tempSensorsEnabled, cfg.currentSensorsEnabled,
  *                       cfg.wheelSensorsEnabled (per-subsystem enable)
  *    temperature.cpp  — sensorOk[] per-sensor tracking
  *    relays.cpp       — consecutiveErrors escalation logic
  ****************************************************************************
  */

#include "service_mode.h"

/* ================================================================== */
/*  Module registry (static, never changes at runtime)                 */
/* ================================================================== */

/**
 * Classification table.  CRITICAL modules can never be disabled.
 * Traced to base firmware:
 *   - CAN timeout, emergency stop, watchdog, relay = always-on safety
 *   - Temperature, current, wheel, steering, ABS, TCS = optional/degradable
 *     (car_sensors.cpp: cfg.tempSensorsEnabled, cfg.wheelSensorsEnabled)
 */
static const ModuleClass_t module_class[MODULE_COUNT] = {
    /* CRITICAL */
    [MODULE_CAN_TIMEOUT]      = MODULE_CLASS_CRITICAL,
    [MODULE_EMERGENCY_STOP]   = MODULE_CLASS_CRITICAL,
    [MODULE_WATCHDOG]         = MODULE_CLASS_CRITICAL,
    [MODULE_RELAY_MAIN]       = MODULE_CLASS_CRITICAL,

    /* NON-CRITICAL — temperature sensors (base: cfg.tempSensorsEnabled) */
    [MODULE_TEMP_SENSOR_0]    = MODULE_CLASS_NON_CRITICAL,
    [MODULE_TEMP_SENSOR_1]    = MODULE_CLASS_NON_CRITICAL,
    [MODULE_TEMP_SENSOR_2]    = MODULE_CLASS_NON_CRITICAL,
    [MODULE_TEMP_SENSOR_3]    = MODULE_CLASS_NON_CRITICAL,
    [MODULE_TEMP_SENSOR_4]    = MODULE_CLASS_NON_CRITICAL,

    /* NON-CRITICAL — current sensors (base: cfg.currentSensorsEnabled) */
    [MODULE_CURRENT_SENSOR_0] = MODULE_CLASS_NON_CRITICAL,
    [MODULE_CURRENT_SENSOR_1] = MODULE_CLASS_NON_CRITICAL,
    [MODULE_CURRENT_SENSOR_2] = MODULE_CLASS_NON_CRITICAL,
    [MODULE_CURRENT_SENSOR_3] = MODULE_CLASS_NON_CRITICAL,
    [MODULE_CURRENT_SENSOR_4] = MODULE_CLASS_NON_CRITICAL,
    [MODULE_CURRENT_SENSOR_5] = MODULE_CLASS_NON_CRITICAL,

    /* NON-CRITICAL — wheel speed (base: cfg.wheelSensorsEnabled) */
    [MODULE_WHEEL_SPEED_FL]   = MODULE_CLASS_NON_CRITICAL,
    [MODULE_WHEEL_SPEED_FR]   = MODULE_CLASS_NON_CRITICAL,
    [MODULE_WHEEL_SPEED_RL]   = MODULE_CLASS_NON_CRITICAL,
    [MODULE_WHEEL_SPEED_RR]   = MODULE_CLASS_NON_CRITICAL,

    /* NON-CRITICAL — steering sub-features */
    [MODULE_STEER_CENTER]     = MODULE_CLASS_NON_CRITICAL,
    [MODULE_STEER_ENCODER]    = MODULE_CLASS_NON_CRITICAL,

    /* NON-CRITICAL — safety sub-features */
    [MODULE_ABS]              = MODULE_CLASS_NON_CRITICAL,
    [MODULE_TCS]              = MODULE_CLASS_NON_CRITICAL,
    [MODULE_ACKERMANN]        = MODULE_CLASS_NON_CRITICAL,

    /* NON-CRITICAL — obstacle detection (ESP32-side, STM32 tolerates absence) */
    [MODULE_OBSTACLE_DETECT]  = MODULE_CLASS_NON_CRITICAL,
};

/* ================================================================== */
/*  Runtime state                                                      */
/* ================================================================== */

static bool          module_enabled[MODULE_COUNT];
static ModuleFault_t module_fault[MODULE_COUNT];

/* ================================================================== */
/*  Initialisation                                                     */
/* ================================================================== */

void ServiceMode_Init(void)
{
    for (uint8_t i = 0; i < MODULE_COUNT; i++) {
        module_enabled[i] = true;
        module_fault[i]   = MODULE_FAULT_NONE;
    }
}

/* ================================================================== */
/*  Classification query                                               */
/* ================================================================== */

ModuleClass_t ServiceMode_GetClass(ModuleID_t id)
{
    if ((uint8_t)id >= MODULE_COUNT) return MODULE_CLASS_CRITICAL;
    return module_class[id];
}

/* ================================================================== */
/*  Enable / Disable                                                   */
/* ================================================================== */

bool ServiceMode_IsEnabled(ModuleID_t id)
{
    if ((uint8_t)id >= MODULE_COUNT) return false;
    /* Critical modules are always enabled regardless of flag */
    if (module_class[id] == MODULE_CLASS_CRITICAL) return true;
    return module_enabled[id];
}

bool ServiceMode_DisableModule(ModuleID_t id)
{
    if ((uint8_t)id >= MODULE_COUNT) return false;
    /* CRITICAL modules MUST NOT be disableable — safety constraint */
    if (module_class[id] == MODULE_CLASS_CRITICAL) return false;
    module_enabled[id] = false;
    return true;
}

bool ServiceMode_EnableModule(ModuleID_t id)
{
    if ((uint8_t)id >= MODULE_COUNT) return false;
    module_enabled[id] = true;
    return true;
}

/* ================================================================== */
/*  Fault management                                                   */
/* ================================================================== */

void ServiceMode_SetFault(ModuleID_t id, ModuleFault_t fault)
{
    if ((uint8_t)id >= MODULE_COUNT) return;
    /* Faults are ALWAYS recorded, even for disabled modules.
     * This ensures fault transparency (requirement 1). */
    module_fault[id] = fault;
}

void ServiceMode_ClearFault(ModuleID_t id)
{
    if ((uint8_t)id >= MODULE_COUNT) return;
    module_fault[id] = MODULE_FAULT_NONE;
}

ModuleFault_t ServiceMode_GetFault(ModuleID_t id)
{
    if ((uint8_t)id >= MODULE_COUNT) return MODULE_FAULT_NONE;
    return module_fault[id];
}

/* ================================================================== */
/*  Status query                                                       */
/* ================================================================== */

ModuleStatus_t ServiceMode_GetStatus(ModuleID_t id)
{
    ModuleStatus_t status;
    status.id             = id;
    status.classification = MODULE_CLASS_CRITICAL;
    status.enabled        = false;
    status.fault          = MODULE_FAULT_NONE;

    if ((uint8_t)id < MODULE_COUNT) {
        status.classification = module_class[id];
        status.enabled        = ServiceMode_IsEnabled(id);
        status.fault          = module_fault[id];
    }
    return status;
}

bool ServiceMode_ShouldBlock(ModuleID_t id)
{
    if ((uint8_t)id >= MODULE_COUNT) return false;

    ModuleFault_t fault = module_fault[id];
    if (fault == MODULE_FAULT_NONE) return false;

    /* Critical modules ALWAYS block on fault */
    if (module_class[id] == MODULE_CLASS_CRITICAL) return true;

    /* Non-critical modules block ONLY if still enabled.
     * Disabled non-critical modules report faults but do NOT block.
     * Traced to base firmware car_sensors.cpp:
     *   if (!cfg.tempSensorsEnabled) { return 0.0f; }  — skip check */
    return module_enabled[id];
}

bool ServiceMode_HasCriticalFault(void)
{
    for (uint8_t i = 0; i < MODULE_COUNT; i++) {
        if (module_class[i] == MODULE_CLASS_CRITICAL &&
            module_fault[i] != MODULE_FAULT_NONE) {
            return true;
        }
    }
    return false;
}

uint8_t ServiceMode_CountActiveNonCriticalFaults(void)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < MODULE_COUNT; i++) {
        if (module_class[i] == MODULE_CLASS_NON_CRITICAL &&
            module_fault[i] != MODULE_FAULT_NONE &&
            module_enabled[i]) {
            count++;
        }
    }
    return count;
}

/* ================================================================== */
/*  Factory restore                                                    */
/* ================================================================== */

void ServiceMode_FactoryRestore(void)
{
    for (uint8_t i = 0; i < MODULE_COUNT; i++) {
        module_enabled[i] = true;
        /* Clear manual-disable faults but preserve real faults.
         * Real faults will reappear on the next check cycle. */
        if (module_fault[i] == MODULE_FAULT_DISABLED) {
            module_fault[i] = MODULE_FAULT_NONE;
        }
    }
}

/* ================================================================== */
/*  Bitmask accessors for CAN transmission                             */
/* ================================================================== */

uint32_t ServiceMode_GetEnabledMask(void)
{
    uint32_t mask = 0;
    for (uint8_t i = 0; i < MODULE_COUNT && i < 32; i++) {
        if (ServiceMode_IsEnabled((ModuleID_t)i)) {
            mask |= (1U << i);
        }
    }
    return mask;
}

uint32_t ServiceMode_GetFaultMask(void)
{
    uint32_t mask = 0;
    for (uint8_t i = 0; i < MODULE_COUNT && i < 32; i++) {
        if (module_fault[i] != MODULE_FAULT_NONE) {
            mask |= (1U << i);
        }
    }
    return mask;
}

uint32_t ServiceMode_GetDisabledMask(void)
{
    uint32_t mask = 0;
    for (uint8_t i = 0; i < MODULE_COUNT && i < 32; i++) {
        if (!module_enabled[i] &&
            module_class[i] == MODULE_CLASS_NON_CRITICAL) {
            mask |= (1U << i);
        }
    }
    return mask;
}
