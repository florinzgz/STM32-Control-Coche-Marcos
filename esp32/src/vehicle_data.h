// =============================================================================
// ESP32-S3 HMI — Vehicle Data Store
//
// Passive data container for decoded CAN telemetry.
// All values are populated by can_rx and read by UI modules.
// No logic, no thresholds, no decisions.
//
// Reference: docs/CAN_CONTRACT_FINAL.md rev 1.0
// =============================================================================

#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H

#include <cstdint>
#include <array>
#include "can_ids.h"

namespace vehicle {

// Number of wheels: FL, FR, RL, RR
inline constexpr uint8_t NUM_WHEELS = 4;
// Number of temperature sensors
inline constexpr uint8_t NUM_TEMP_SENSORS = 5;

// -------------------------------------------------------------------------
// Heartbeat data from STM32 (0x001)
// -------------------------------------------------------------------------
struct HeartbeatData {
    uint8_t           aliveCounter = 0;
    can::SystemState  systemState  = can::SystemState::BOOT;
    uint8_t           faultFlags   = 0;
    uint8_t           reserved     = 0;
    unsigned long     timestampMs  = 0;
};

// -------------------------------------------------------------------------
// Wheel speeds (0x200) — raw uint16 in 0.1 km/h units
// -------------------------------------------------------------------------
struct SpeedData {
    std::array<uint16_t, NUM_WHEELS> raw{};   // FL, FR, RL, RR
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Motor currents (0x201) — raw uint16 in 0.01 A units
// -------------------------------------------------------------------------
struct CurrentData {
    std::array<uint16_t, NUM_WHEELS> raw{};   // FL, FR, RL, RR
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Temperature sensors (0x202) — int8 °C
// -------------------------------------------------------------------------
struct TempData {
    std::array<int8_t, NUM_TEMP_SENSORS> temps{};
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Safety status (0x203)
// -------------------------------------------------------------------------
struct SafetyData {
    uint8_t absActive  = 0;
    uint8_t tcsActive  = 0;
    uint8_t errorCode  = 0;
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Steering status (0x204) — raw int16 in 0.1° units
// -------------------------------------------------------------------------
struct SteeringData {
    int16_t angleRaw   = 0;    // 0.1° units
    uint8_t calibrated = 0;
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Traction scale per wheel (0x205) — 0–100 %
// -------------------------------------------------------------------------
struct TractionData {
    std::array<uint8_t, NUM_WHEELS> scale{};  // FL, FR, RL, RR
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Temperature map (0x206) — int8 °C, mapped: FL, FR, RL, RR, Ambient
// -------------------------------------------------------------------------
struct TempMapData {
    std::array<int8_t, NUM_TEMP_SENSORS> temps{};  // FL, FR, RL, RR, Ambient
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Diagnostic error (0x300)
// -------------------------------------------------------------------------
struct DiagData {
    uint8_t errorCode  = 0;
    uint8_t subsystem  = 0;
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Service status (0x301–0x303) — 32-bit bitmasks
// -------------------------------------------------------------------------
struct ServiceData {
    uint32_t faultMask    = 0;   // 0x301
    uint32_t enabledMask  = 0;   // 0x302
    uint32_t disabledMask = 0;   // 0x303
    unsigned long faultTimestampMs    = 0;
    unsigned long enabledTimestampMs  = 0;
    unsigned long disabledTimestampMs = 0;
};

// -------------------------------------------------------------------------
// Battery bus data (0x207) — 24V main battery via INA226 100A shunt
// Displayed in the upper-right corner of the drive screen.
// -------------------------------------------------------------------------
struct BatteryData {
    uint16_t currentRaw = 0;   // 0.01 A units
    uint16_t voltageRaw = 0;   // 0.01 V units
    unsigned long timestampMs = 0;
};

// =========================================================================
// VehicleData — central read/write store
// =========================================================================
class VehicleData {
public:
    // Setters (used by can_rx)
    void setHeartbeat(const HeartbeatData& d)  { heartbeat_ = d; }
    void setSpeed(const SpeedData& d)          { speed_ = d; }
    void setCurrent(const CurrentData& d)      { current_ = d; }
    void setTemp(const TempData& d)            { temp_ = d; }
    void setSafety(const SafetyData& d)        { safety_ = d; }
    void setSteering(const SteeringData& d)    { steering_ = d; }
    void setTraction(const TractionData& d)    { traction_ = d; }
    void setTempMap(const TempMapData& d)      { tempMap_ = d; }
    void setDiag(const DiagData& d)            { diag_ = d; }
    void setBattery(const BatteryData& d)      { battery_ = d; }

    void setServiceFaults(uint32_t mask, unsigned long ts)   { service_.faultMask = mask;    service_.faultTimestampMs = ts; }
    void setServiceEnabled(uint32_t mask, unsigned long ts)  { service_.enabledMask = mask;  service_.enabledTimestampMs = ts; }
    void setServiceDisabled(uint32_t mask, unsigned long ts) { service_.disabledMask = mask; service_.disabledTimestampMs = ts; }

    // Const getters (used by UI)
    const HeartbeatData& heartbeat() const { return heartbeat_; }
    const SpeedData&     speed()     const { return speed_; }
    const CurrentData&   current()   const { return current_; }
    const TempData&      temp()      const { return temp_; }
    const SafetyData&    safety()    const { return safety_; }
    const SteeringData&  steering()  const { return steering_; }
    const TractionData&  traction()  const { return traction_; }
    const TempMapData&   tempMap()   const { return tempMap_; }
    const DiagData&      diag()      const { return diag_; }
    const BatteryData&   battery()   const { return battery_; }
    const ServiceData&   service()   const { return service_; }

private:
    HeartbeatData heartbeat_;
    SpeedData     speed_;
    CurrentData   current_;
    TempData      temp_;
    SafetyData    safety_;
    SteeringData  steering_;
    TractionData  traction_;
    TempMapData   tempMap_;
    DiagData      diag_;
    BatteryData   battery_;
    ServiceData   service_;
};

} // namespace vehicle

#endif // VEHICLE_DATA_H
