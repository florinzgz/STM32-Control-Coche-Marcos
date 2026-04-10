// =============================================================================
// ESP32-S3 HMI — Safe Screen
//
// Shown when system_state = SAFE (4).
// Prominent safety warning banner. Read-only telemetry visible:
// wheel speeds, motor currents, temperatures, steering angle.
// All controls disabled. No heap allocation.
//
// Reference: docs/HMI_STATE_MODEL.md §2.5
// =============================================================================

#ifndef SAFE_SCREEN_H
#define SAFE_SCREEN_H

#include "screen.h"
#include <cstdint>
#include <array>

class SafeScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data) override;
    void draw()    override;

private:
    bool    needsRedraw_    = true;
    uint8_t faultFlags_     = 0;
    uint8_t prevFaultFlags_ = 0xFF;
    uint8_t errorCode_      = 0;
    uint8_t prevErrorCode_  = 0xFF;

    // Read-only telemetry (HMI_STATE_MODEL §2.5)
    std::array<uint16_t, 4> wheelSpeed_     = {};   // 0.1 km/h units
    std::array<uint16_t, 4> prevWheelSpeed_ = {};
    std::array<uint16_t, 4> motorCurrent_   = {};   // 0.01 A units
    std::array<uint16_t, 4> prevMotorCurrent_ = {};
    std::array<int8_t, 5>   temps_          = {};   // °C
    std::array<int8_t, 5>   prevTemps_      = {};
    int16_t steeringAngle_     = 0;                  // 0.1° units
    int16_t prevSteeringAngle_ = 0;
};

#endif // SAFE_SCREEN_H
