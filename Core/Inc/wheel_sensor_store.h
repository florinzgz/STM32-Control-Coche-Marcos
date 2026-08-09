/**
  ****************************************************************************
  * @file    wheel_sensor_store.h
  * @brief   Persistent GLOBAL wheel-sensor geometry/timing — service diag C5
  *
  * Stores the runtime-tunable wheel-speed-sensor parameters in flash so they
  * can be adjusted from the ESP32-HMI SERVICE_AUTOTEST menu without
  * re-flashing firmware.  Modelled 1:1 on battery_limits_store.c.
  *
  * DELIBERATELY GLOBAL (single value applied to all 4 wheels): the four
  * wheels share identical hardware (same sensor, same bolt count, same
  * wheel/tyre) so there is exactly ONE correct value for each field.  There
  * is NO per-wheel variant of any field here and none may be added — the
  * whole point of the service-diagnostic system is to REVEAL a difference
  * between wheels, never to compensate for it (see PROHIBIDO in Block C).
  *
  * These were compile-time macros:
  *   WHEEL_PULSES_REV        6      (project_config.h)   pulses per revolution
  *   WHEEL_CIRCUM_MM         1110.0 (vehicle_physics.h)   wheel circumference
  *   SENSOR_DEBOUNCE_US      200    (project_config.h)    DWT debounce window
  *   WHEEL_FAULT_DEBOUNCE_MS 1000   (safety_system.h)     mismatch-fault debounce
  *
  * pulses_per_rev / circumference_mm / debounce_us are wired live into
  * sensor_manager.c (Wheel_ComputeSpeed() and the ISR DWT pre-filter via
  * Sensor_SetDebounceUs()).  mismatch_debounce_ms is persisted/validated here
  * and is consumed by the service-diagnostic session's own "pulses on a
  * wheel that is not being actuated" abort check (Block A) -- it is
  * DELIBERATELY NOT wired into safety_system.c's production WHEEL_FAULT_
  * DEBOUNCE_MS fault-latch path, which stays on its audited compile-time
  * constant to avoid widening that safety-critical file's dependency graph.
  *
  * Safety invariants:
  *   - Editable while a service-diag test is ACTIVE (staged in RAM only),
  *     same as C1/C2 -- these are measurement-geometry parameters, they do
  *     not actuate anything by themselves.
  *   - On flash blank / CRC-invalid / out-of-range the module silently
  *     falls back to the compile-time defaults.  Boot is NEVER blocked.
  *
  * Flash layout:
  *   Page 116 (0x08074000, 4 KB) — dedicated to wheel-sensor geometry.
  *   Single slot with magic "WSN1" + CRC32 integrity check.
  ****************************************************************************
  */

#ifndef WHEEL_SENSOR_STORE_H
#define WHEEL_SENSOR_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define WHEEL_SENSOR_PULSES_REV_MIN         1U
#define WHEEL_SENSOR_PULSES_REV_MAX         50U
#define WHEEL_SENSOR_CIRCUM_MM_MIN          500.0f
#define WHEEL_SENSOR_CIRCUM_MM_MAX          2000.0f
#define WHEEL_SENSOR_DEBOUNCE_US_MIN        50U
#define WHEEL_SENSOR_DEBOUNCE_US_MAX        2000U
#define WHEEL_SENSOR_MISMATCH_DEBOUNCE_MS_MIN  100U
#define WHEEL_SENSOR_MISMATCH_DEBOUNCE_MS_MAX  5000U

typedef struct {
    uint16_t pulses_per_rev;
    float    circumference_mm;
    uint16_t debounce_us;
    uint16_t mismatch_debounce_ms;
} WheelSensorParams_t;

/* HAL-free single source used by production and host tests. */
static inline bool WheelSensor_ValidateValues(const WheelSensorParams_t *w)
{
    if (!w) return false;
    if (w->pulses_per_rev < WHEEL_SENSOR_PULSES_REV_MIN ||
        w->pulses_per_rev > WHEEL_SENSOR_PULSES_REV_MAX) return false;
    if (w->circumference_mm < WHEEL_SENSOR_CIRCUM_MM_MIN ||
        w->circumference_mm > WHEEL_SENSOR_CIRCUM_MM_MAX) return false;
    if (w->debounce_us < WHEEL_SENSOR_DEBOUNCE_US_MIN ||
        w->debounce_us > WHEEL_SENSOR_DEBOUNCE_US_MAX) return false;
    if (w->mismatch_debounce_ms < WHEEL_SENSOR_MISMATCH_DEBOUNCE_MS_MIN ||
        w->mismatch_debounce_ms > WHEEL_SENSOR_MISMATCH_DEBOUNCE_MS_MAX) return false;
    return true;
}

void WheelSensorStore_Init(void);
bool WheelSensorStore_IsValid(void);
void WheelSensorStore_GetDefaults(WheelSensorParams_t *out);
bool WheelSensorStore_Validate(const WheelSensorParams_t *w);

/** Effective (RAM-staged) values -- consumed by sensor_manager.c's wheel
 *  speed calculation / DWT debounce and safety_system.c's mismatch debounce. */
void WheelSensorStore_GetEffective(WheelSensorParams_t *out);
uint16_t WheelSensorStore_GetEffectivePulsesPerRev(void);
float    WheelSensorStore_GetEffectiveCircumferenceM(void);
uint16_t WheelSensorStore_GetEffectiveDebounceUs(void);
uint16_t WheelSensorStore_GetEffectiveMismatchDebounceMs(void);

bool WheelSensorStore_Stage(const WheelSensorParams_t *w);
void WheelSensorStore_GetStaged(WheelSensorParams_t *out);
void WheelSensorStore_Revert(void);
void WheelSensorStore_ResetToDefaults(void);
bool WheelSensorStore_Save(void);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_SENSOR_STORE_H */
