/**
  ****************************************************************************
  * @file    boot_validation.c
  * @brief   Boot validation checklist implementation
  *
  *  Deterministic pre-ACTIVE gate executed during STANDBY.
  *  Each check is non-blocking and reads current sensor / safety state.
  *
  *  Thresholds reuse existing constants defined in safety_system.c
  *  and sensor_manager.h to avoid duplication.
  ****************************************************************************
  */

#include "boot_validation.h"
#include "safety_system.h"
#include "sensor_manager.h"
#include "motor_control.h"
#include "can_handler.h"
#include "service_mode.h"

/* Plausibility thresholds — aligned with safety_system.c definitions.
 * Temperature: DS18B20 range is -55 °C to +125 °C; we accept
 *              -40 °C to +125 °C as plausible (same as SENSOR_TEMP_*).
 * Current:     Anything negative or > 50 A is implausible.
 * Battery:     Must be above the warning threshold (20.0 V).          */
#define BOOT_TEMP_MIN_C         (-40.0f)
#define BOOT_TEMP_MAX_C         125.0f
#define BOOT_CURRENT_MIN_A      (-1.0f)
#define BOOT_CURRENT_MAX_A      50.0f
#define BOOT_BATTERY_MIN_V      20.0f

/* Number of temperature / current sensors to check */
#define BOOT_NUM_TEMP_SENSORS   5
#define BOOT_NUM_CURRENT_SENSORS 4

/* ---- Module state ---- */
static BootValidationStatus boot_status = {0};

/* ---- Internal check helpers ---- */

/**
 * @brief  Check that all temperature sensors report a plausible value.
 *         A reading of exactly 0.0 °C on ALL sensors is treated as
 *         "sensors not yet read" — implausible (hardware default).
 */
static bool check_temperature_plausible(void)
{
    uint8_t zero_count = 0;
    uint8_t enabled_count = 0;
    for (uint8_t i = 0; i < BOOT_NUM_TEMP_SENSORS; i++) {
        if (!ServiceMode_IsEnabled(MODULE_TEMP_SENSOR_0 + i))
            continue;   /* Disabled sensors are excluded from check */
        enabled_count++;
        float t = Temperature_Get(i);
        if (t < BOOT_TEMP_MIN_C || t > BOOT_TEMP_MAX_C)
            return false;
        if (t == 0.0f)
            zero_count++;
    }
    /* If every enabled sensor reads exactly 0.0 °C → likely uninit */
    if (enabled_count > 0 && zero_count == enabled_count)
        return false;
    return true;
}

/**
 * @brief  Check that motor current sensors report a plausible value.
 */
static bool check_current_plausible(void)
{
    for (uint8_t i = 0; i < BOOT_NUM_CURRENT_SENSORS; i++) {
        if (!ServiceMode_IsEnabled(MODULE_CURRENT_SENSOR_0 + i))
            continue;
        float c = Current_GetAmps(i);
        if (c < BOOT_CURRENT_MIN_A || c > BOOT_CURRENT_MAX_A)
            return false;
    }
    return true;
}

/**
 * @brief  Check that the steering encoder has no active fault.
 */
static bool check_encoder_healthy(void)
{
    return !Encoder_HasFault();
}

/**
 * @brief  Check that battery voltage is above the warning threshold.
 *         Uses bus voltage from the battery INA226 channel (index 4).
 */
static bool check_battery_ok(void)
{
    float v = Voltage_GetBus(4);
    return (v >= BOOT_BATTERY_MIN_V);
}

/**
 * @brief  Check that no persistent safety error is present.
 */
static bool check_no_safety_error(void)
{
    return (Safety_GetError() == SAFETY_ERROR_NONE);
}

/**
 * @brief  Check that the CAN bus is not in bus-off state.
 */
static bool check_can_not_busoff(void)
{
    return !CAN_IsBusOff();
}

/* ================================================================== */
/*  Public API                                                         */
/* ================================================================== */

void BootValidation_Run(void)
{
    boot_status.checks_passed = 0;
    boot_status.checks_failed = 0;

    /* Temperature plausibility */
    if (check_temperature_plausible())
        boot_status.checks_passed |= BOOT_CHECK_TEMP_PLAUSIBLE;
    else
        boot_status.checks_failed |= BOOT_CHECK_TEMP_PLAUSIBLE;

    /* Current plausibility */
    if (check_current_plausible())
        boot_status.checks_passed |= BOOT_CHECK_CURRENT_PLAUSIBLE;
    else
        boot_status.checks_failed |= BOOT_CHECK_CURRENT_PLAUSIBLE;

    /* Encoder health */
    if (check_encoder_healthy())
        boot_status.checks_passed |= BOOT_CHECK_ENCODER_HEALTHY;
    else
        boot_status.checks_failed |= BOOT_CHECK_ENCODER_HEALTHY;

    /* Battery voltage */
    if (check_battery_ok())
        boot_status.checks_passed |= BOOT_CHECK_BATTERY_OK;
    else
        boot_status.checks_failed |= BOOT_CHECK_BATTERY_OK;

    /* No safety error */
    if (check_no_safety_error())
        boot_status.checks_passed |= BOOT_CHECK_NO_SAFETY_ERROR;
    else
        boot_status.checks_failed |= BOOT_CHECK_NO_SAFETY_ERROR;

    /* CAN not bus-off */
    if (check_can_not_busoff())
        boot_status.checks_passed |= BOOT_CHECK_CAN_NOT_BUSOFF;
    else
        boot_status.checks_failed |= BOOT_CHECK_CAN_NOT_BUSOFF;

    /* Overall result */
    boot_status.validated =
        (boot_status.checks_passed == BOOT_CHECK_ALL_MASK);

    /* Log failed checks via ServiceMode if validation fails.
     * This does NOT force SAFE — the system stays in STANDBY
     * until all checks pass.  Per-sensor faults are logged
     * to identify the exact failing sensor.                       */
    if (!boot_status.validated && boot_status.checks_failed != 0) {
        if (boot_status.checks_failed & BOOT_CHECK_TEMP_PLAUSIBLE) {
            for (uint8_t i = 0; i < BOOT_NUM_TEMP_SENSORS; i++) {
                if (!ServiceMode_IsEnabled(MODULE_TEMP_SENSOR_0 + i))
                    continue;
                float t = Temperature_Get(i);
                if (t < BOOT_TEMP_MIN_C || t > BOOT_TEMP_MAX_C)
                    ServiceMode_SetFault(MODULE_TEMP_SENSOR_0 + i, MODULE_FAULT_WARNING);
            }
        }
        if (boot_status.checks_failed & BOOT_CHECK_CURRENT_PLAUSIBLE) {
            for (uint8_t i = 0; i < BOOT_NUM_CURRENT_SENSORS; i++) {
                if (!ServiceMode_IsEnabled(MODULE_CURRENT_SENSOR_0 + i))
                    continue;
                float c = Current_GetAmps(i);
                if (c < BOOT_CURRENT_MIN_A || c > BOOT_CURRENT_MAX_A)
                    ServiceMode_SetFault(MODULE_CURRENT_SENSOR_0 + i, MODULE_FAULT_WARNING);
            }
        }
        if (boot_status.checks_failed & BOOT_CHECK_ENCODER_HEALTHY)
            ServiceMode_SetFault(MODULE_STEER_ENCODER, MODULE_FAULT_WARNING);
    }
}

bool BootValidation_IsPassed(void)
{
    return boot_status.validated;
}

const BootValidationStatus* BootValidation_GetStatus(void)
{
    return &boot_status;
}
