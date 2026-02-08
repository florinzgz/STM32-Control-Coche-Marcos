/**
  ****************************************************************************
  * @file    service_mode.h
  * @brief   Service Mode / Module Disable system
  *
  *  Implements the SERVICE / LIMP philosophy from the base firmware
  *  (FULL-FIRMWARE-Coche-Marcos, src/system/limp_mode.cpp):
  *
  *    - Non-critical sensor faults do NOT block the vehicle
  *    - The vehicle remains drivable with reduced performance
  *    - The operator is informed of the exact fault
  *    - Faulty modules can be temporarily disabled
  *    - The vehicle can be driven home and repaired later
  *
  *  Module classification:
  *    CRITICAL    — never disableable, always force SAFE/ERROR
  *    NON_CRITICAL — can be disabled by user, enters DEGRADED
  *
  *  Traced to base firmware:
  *    limp_mode.cpp  — NORMAL/DEGRADED/LIMP/CRITICAL states
  *    car_sensors.cpp — cfg.tempSensorsEnabled, cfg.currentSensorsEnabled,
  *                      cfg.wheelSensorsEnabled (per-subsystem enable flags)
  *    relays.cpp      — consecutiveErrors escalation
  *    temperature.cpp — individual sensor OK tracking (sensorOk[])
  ****************************************************************************
  */

#ifndef __SERVICE_MODE_H
#define __SERVICE_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ================================================================== */
/*  Module IDs                                                         */
/* ================================================================== */

/**
 * @brief Unique module identifiers.
 *
 * Each module maps to a sensor subsystem or safety feature.
 * IDs are stable and never reordered (used in CAN protocol).
 */
typedef enum {
    /* --- CRITICAL modules (NOT disableable) --- */
    MODULE_CAN_TIMEOUT      = 0,   /* CAN heartbeat watchdog           */
    MODULE_EMERGENCY_STOP   = 1,   /* Hardware emergency stop           */
    MODULE_WATCHDOG         = 2,   /* IWDG watchdog                     */
    MODULE_RELAY_MAIN       = 3,   /* Main power relay                  */

    /* --- NON-CRITICAL modules (disableable) --- */
    MODULE_TEMP_SENSOR_0    = 4,   /* DS18B20 temperature sensor 0      */
    MODULE_TEMP_SENSOR_1    = 5,   /* DS18B20 temperature sensor 1      */
    MODULE_TEMP_SENSOR_2    = 6,   /* DS18B20 temperature sensor 2      */
    MODULE_TEMP_SENSOR_3    = 7,   /* DS18B20 temperature sensor 3      */
    MODULE_TEMP_SENSOR_4    = 8,   /* DS18B20 temperature sensor 4      */
    MODULE_CURRENT_SENSOR_0 = 9,   /* INA226 current sensor 0 (FL)      */
    MODULE_CURRENT_SENSOR_1 = 10,  /* INA226 current sensor 1 (FR)      */
    MODULE_CURRENT_SENSOR_2 = 11,  /* INA226 current sensor 2 (RL)      */
    MODULE_CURRENT_SENSOR_3 = 12,  /* INA226 current sensor 3 (RR)      */
    MODULE_CURRENT_SENSOR_4 = 13,  /* INA226 current sensor 4 (battery) */
    MODULE_CURRENT_SENSOR_5 = 14,  /* INA226 current sensor 5 (steer)   */
    MODULE_WHEEL_SPEED_FL   = 15,  /* Wheel speed sensor FL             */
    MODULE_WHEEL_SPEED_FR   = 16,  /* Wheel speed sensor FR             */
    MODULE_WHEEL_SPEED_RL   = 17,  /* Wheel speed sensor RL             */
    MODULE_WHEEL_SPEED_RR   = 18,  /* Wheel speed sensor RR             */
    MODULE_STEER_CENTER     = 19,  /* Steering centering sensor          */
    MODULE_STEER_ENCODER    = 20,  /* Steering encoder                   */
    MODULE_ABS              = 21,  /* Anti-lock braking                  */
    MODULE_TCS              = 22,  /* Traction control                   */
    MODULE_ACKERMANN        = 23,  /* Ackermann steering correction      */
    MODULE_OBSTACLE_DETECT  = 24,  /* Obstacle detection (ESP32-side)    */

    MODULE_COUNT            = 25
} ModuleID_t;

/* ================================================================== */
/*  Module classification                                              */
/* ================================================================== */

typedef enum {
    MODULE_CLASS_CRITICAL     = 0,  /* Never disableable                 */
    MODULE_CLASS_NON_CRITICAL = 1   /* Can be disabled by user           */
} ModuleClass_t;

/* ================================================================== */
/*  Per-module fault state                                             */
/* ================================================================== */

typedef enum {
    MODULE_FAULT_NONE       = 0,   /* No fault                          */
    MODULE_FAULT_WARNING    = 1,   /* Out of range / degraded reading   */
    MODULE_FAULT_ERROR      = 2,   /* Sensor not responding / invalid   */
    MODULE_FAULT_DISABLED   = 3    /* Manually disabled by user         */
} ModuleFault_t;

/* ================================================================== */
/*  Per-module status (readable by ESP32 via CAN)                      */
/* ================================================================== */

typedef struct {
    ModuleID_t    id;
    ModuleClass_t classification;
    bool          enabled;         /* true = active, false = disabled    */
    ModuleFault_t fault;           /* Current fault state                */
} ModuleStatus_t;

/* ================================================================== */
/*  Public API                                                         */
/* ================================================================== */

/**
 * @brief  Initialise service mode system.
 *         Sets all modules to enabled state and clears faults.
 */
void ServiceMode_Init(void);

/**
 * @brief  Get classification of a module (CRITICAL or NON_CRITICAL).
 * @param  id  Module identifier
 * @return Module classification
 */
ModuleClass_t ServiceMode_GetClass(ModuleID_t id);

/**
 * @brief  Check if a module is currently enabled.
 * @param  id  Module identifier
 * @return true if enabled (or critical), false if disabled
 */
bool ServiceMode_IsEnabled(ModuleID_t id);

/**
 * @brief  Disable a non-critical module.
 *         Critical modules cannot be disabled (returns false).
 *         Disabled modules still report faults but do not block the system.
 * @param  id  Module identifier
 * @return true if successfully disabled, false if critical/invalid
 */
bool ServiceMode_DisableModule(ModuleID_t id);

/**
 * @brief  Enable a previously disabled module.
 * @param  id  Module identifier
 * @return true if successfully enabled, false if invalid
 */
bool ServiceMode_EnableModule(ModuleID_t id);

/**
 * @brief  Set the fault state of a module.
 *         Faults are always recorded regardless of enable/disable state.
 * @param  id     Module identifier
 * @param  fault  Fault type
 */
void ServiceMode_SetFault(ModuleID_t id, ModuleFault_t fault);

/**
 * @brief  Clear the fault state of a module.
 * @param  id  Module identifier
 */
void ServiceMode_ClearFault(ModuleID_t id);

/**
 * @brief  Get the current fault state of a module.
 * @param  id  Module identifier
 * @return Current fault (NONE/WARNING/ERROR/DISABLED)
 */
ModuleFault_t ServiceMode_GetFault(ModuleID_t id);

/**
 * @brief  Get full status of a module.
 * @param  id  Module identifier
 * @return ModuleStatus_t with all fields populated
 */
ModuleStatus_t ServiceMode_GetStatus(ModuleID_t id);

/**
 * @brief  Check whether a faulted module should block the system.
 *
 *         A module blocks the system (returns true) when:
 *           - It is CRITICAL and has a fault, OR
 *           - It is NON_CRITICAL, has a fault, AND is still enabled
 *
 *         A disabled non-critical module with a fault does NOT block.
 *
 * @param  id  Module identifier
 * @return true if the module's fault should affect system state
 */
bool ServiceMode_ShouldBlock(ModuleID_t id);

/**
 * @brief  Check if any CRITICAL module has a fault.
 * @return true if at least one critical module is faulted
 */
bool ServiceMode_HasCriticalFault(void);

/**
 * @brief  Count the number of active (blocking) non-critical faults.
 * @return Number of non-critical modules that are enabled and faulted
 */
uint8_t ServiceMode_CountActiveNonCriticalFaults(void);

/**
 * @brief  Factory restore: re-enable all modules and clear manual disables.
 *         Faults that are still present will reappear on next check cycle.
 */
void ServiceMode_FactoryRestore(void);

/**
 * @brief  Get a 32-bit bitmask of module enable states.
 *         Bit N = 1 means module N is enabled.
 *         Transmitted to ESP32 via CAN for service menu display.
 * @return Bitmask of enabled modules
 */
uint32_t ServiceMode_GetEnabledMask(void);

/**
 * @brief  Get a 32-bit bitmask of module fault states.
 *         Bit N = 1 means module N has a fault (WARNING or ERROR).
 *         Transmitted to ESP32 via CAN for fault display.
 * @return Bitmask of faulted modules
 */
uint32_t ServiceMode_GetFaultMask(void);

/**
 * @brief  Get a 32-bit bitmask of disabled modules.
 *         Bit N = 1 means module N is manually disabled.
 * @return Bitmask of disabled modules
 */
uint32_t ServiceMode_GetDisabledMask(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVICE_MODE_H */
