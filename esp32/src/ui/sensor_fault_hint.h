// =============================================================================
// ESP32-S3 HMI — Sensor-fault cause hint for SAFETY_ERROR_SENSOR_FAULT (code 4)
//
// SAFETY_ERROR_SENSOR_FAULT is a single generic error code raised by the
// STM32 from several unrelated plausibility checks (Core/Src/safety_system.c
// Safety_CheckSensors()): pedal dual-ADC cross-validation, INA226 current
// sensors, DS18B20 temperature sensors, and wheel-speed sensors.  The STM32
// already reports exactly which module is at fault via two channels that are
// decoded into VehicleData but were never surfaced on the Error screen:
//   - CAN 0x20B STATUS_PEDAL     -> PedalData.plausible / .contradictory
//   - CAN 0x301/0x303 SERVICE_*  -> ServiceData.faultMask / .disabledMask
//     (bit order mirrors ModuleID_t in Core/Inc/service_mode.h)
//
// This header turns those already-available fields into a short "Cause: ..."
// line so the operator can tell a pedal fault apart from an INA226 channel,
// a temperature sensor, or a wheel sensor, instead of a bare "SENSOR FAULT".
//
// Priority rationale:
//   - Pedal is checked FIRST because Safety_CheckSensors() validates pedal
//     plausibility before the temp/current/wheel loop and returns
//     immediately on a pedal fault — a fresh pedal fault is therefore the
//     most likely live cause of the CURRENT SENSOR_FAULT.
//   - INA226 current-sensor faults are always MODULE_FAULT_ERROR (single
//     call site in Safety_CheckSensors()), so a live bit is a high-confidence
//     match.  Temperature and wheel-speed bits can also be
//     MODULE_FAULT_WARNING from a diagnostic-only cross-check that does NOT
//     itself raise SENSOR_FAULT — the 0x301 mask cannot distinguish WARNING
//     from ERROR, so — like the neighbouring wheel-diag hint — this is a
//     best-effort HINT, not a guaranteed root cause.
//   - A module bit that is ALSO set in the disabled mask (0x303) is skipped:
//     a manually-disabled module is excluded from Safety_CheckSensors() and
//     cannot be the live cause even if its fault bit is still latched from
//     before it was disabled.
//
// This header is intentionally free of Arduino/TFT dependencies so the
// mapping can be exercised on host g++ (see
// esp32/src/test_sensor_fault_hint.cpp).
// =============================================================================

#ifndef UI_SENSOR_FAULT_HINT_H
#define UI_SENSOR_FAULT_HINT_H

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace ui {

// Which category of sensor the hint identified as the likely SENSOR_FAULT
// cause.  NONE means the caller should keep its existing plain generic text.
enum class SensorFaultCause : uint8_t {
    NONE,
    PEDAL,
    CURRENT,
    TEMP,
    WHEEL,
};

// Safety_Error_t code for SENSOR_FAULT (Core/Inc/safety_system.h) — the only
// code this hint applies to.
inline constexpr uint8_t SAFETY_ERROR_SENSOR_FAULT_CODE = 4;

// Freshness window for the pedal/service inputs — matches the 2 s convention
// already used for the 0x313 wheel-diag hint (see wheelDiagValid_ in
// error_screen.cpp).  Callers compute the boolean once (not a raw age) so a
// continuously-advancing clock never forces a redraw every frame.
inline constexpr unsigned long SENSOR_FAULT_HINT_STALE_MS = 2000UL;

// Bit positions within ServiceData.faultMask / .disabledMask (CAN 0x301 /
// 0x303), mirroring ModuleID_t order in Core/Inc/service_mode.h.  Kept as
// local named bit indices rather than a shared enum — matches the existing
// convention in engineering_screen.cpp's MODULE_NAMES table, which indexes
// by the same raw bit order.
inline constexpr uint8_t MODBIT_TEMP_SENSOR_0    = 4;   // .. +4 = TEMP_SENSOR_4
inline constexpr uint8_t MODBIT_CURRENT_SENSOR_0 = 9;   // .. +5 = CURRENT_SENSOR_5 (steer)
inline constexpr uint8_t MODBIT_WHEEL_SPEED_FL   = 15;  // .. +3 = WHEEL_SPEED_RR

inline constexpr uint8_t TEMP_SENSOR_COUNT     = 5;
inline constexpr uint8_t CURRENT_SENSOR_COUNT  = 6;
inline constexpr uint8_t WHEEL_SENSOR_COUNT    = 4;

// -------------------------------------------------------------------------
// buildSensorFaultHint — compose the "Cause: ..." hint into out[0..n).
//
//   safetyErrorCode     vehicle::SafetyData.errorCode; only acts on code 4.
//   pedalExtended       PedalData.extended (fault bits present, DLC >= 4).
//   pedalPlausible      PedalData.plausible (0x20B b1 bit0).
//   pedalContradictory  PedalData.contradictory (0x20B b1 bit1).
//   pedalFresh          age of the 0x20B frame <= SENSOR_FAULT_HINT_STALE_MS.
//   serviceFaultMask    ServiceData.faultMask (0x301).
//   serviceDisabledMask ServiceData.disabledMask (0x303).
//   serviceFresh        age of the 0x301 frame <= SENSOR_FAULT_HINT_STALE_MS.
//
// Decision order (highest priority first):
//   1. safetyErrorCode != 4          -> NONE, out untouched (not applicable)
//   2. fresh pedal fault             -> PEDAL   "Cause: Pedal CONTRADICTORY/IMPLAUSIBLE"
//   3. fresh live INA226 channel bit -> CURRENT "Cause: INA226 CH<i> (<axis>)"
//   4. fresh live temp sensor bit    -> TEMP    "Cause: Temp Sensor <i>"
//   5. fresh live wheel speed bit    -> WHEEL   "Cause: Wheel <axis> (no detail)"
//   6. otherwise                     -> NONE, out untouched
//
// "live" = set in faultMask but NOT in disabledMask.  Output is always
// NUL-terminated and bounded by n (no overflow).
// -------------------------------------------------------------------------
inline SensorFaultCause buildSensorFaultHint(
        uint8_t safetyErrorCode,
        bool pedalExtended, bool pedalPlausible, bool pedalContradictory, bool pedalFresh,
        uint32_t serviceFaultMask, uint32_t serviceDisabledMask, bool serviceFresh,
        char* out, size_t n) {
    if (out == nullptr || n == 0) return SensorFaultCause::NONE;
    out[0] = '\0';

    if (safetyErrorCode != SAFETY_ERROR_SENSOR_FAULT_CODE) {
        return SensorFaultCause::NONE;
    }

    // 1. Pedal dual-ADC plausibility — checked first by the STM32 (returns
    //    immediately, before the temp/current/wheel loop), so a fresh pedal
    //    fault is the most likely live cause.
    if (pedalExtended && pedalFresh && !pedalPlausible) {
        snprintf(out, n, "Cause: Pedal %s",
                 pedalContradictory ? "CONTRADICTORY" : "IMPLAUSIBLE");
        return SensorFaultCause::PEDAL;
    }

    if (serviceFresh) {
        const uint32_t live = serviceFaultMask & ~serviceDisabledMask;

        // 2. INA226 current-sensor channel (always MODULE_FAULT_ERROR when set).
        static const char* const kCurrentLbl[CURRENT_SENSOR_COUNT] = {
            "FL", "FR", "RL", "RR", "Battery", "Steering"
        };
        for (uint8_t i = 0; i < CURRENT_SENSOR_COUNT; ++i) {
            if (live & (1UL << (MODBIT_CURRENT_SENSOR_0 + i))) {
                snprintf(out, n, "Cause: INA226 CH%u (%s)", i, kCurrentLbl[i]);
                return SensorFaultCause::CURRENT;
            }
        }

        // 3. DS18B20 temperature sensor.
        for (uint8_t i = 0; i < TEMP_SENSOR_COUNT; ++i) {
            if (live & (1UL << (MODBIT_TEMP_SENSOR_0 + i))) {
                snprintf(out, n, "Cause: Temp Sensor %u", i);
                return SensorFaultCause::TEMP;
            }
        }

        // 4. Wheel-speed sensor fallback — used only when the more detailed
        //    0x313-based wheel hint has no fresh data to show (see
        //    error_screen.cpp, which tries that hint first).
        static const char* const kWheelLbl[WHEEL_SENSOR_COUNT] = {
            "FL", "FR", "RL", "RR"
        };
        for (uint8_t i = 0; i < WHEEL_SENSOR_COUNT; ++i) {
            if (live & (1UL << (MODBIT_WHEEL_SPEED_FL + i))) {
                snprintf(out, n, "Cause: Wheel %s (no detail)", kWheelLbl[i]);
                return SensorFaultCause::WHEEL;
            }
        }
    }

    return SensorFaultCause::NONE;
}

}  // namespace ui

#endif  // UI_SENSOR_FAULT_HINT_H
