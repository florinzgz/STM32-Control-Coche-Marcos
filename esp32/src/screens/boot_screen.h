// =============================================================================
// ESP32-S3 HMI — Boot Screen
//
// Static splash screen shown during system_state = BOOT (0).
// Displays firmware name and CAN link status.
// No interactive elements. No heap allocation.
//
// Reference: docs/HMI_STATE_MODEL.md §2.1
// =============================================================================

#ifndef BOOT_SCREEN_H
#define BOOT_SCREEN_H

#include "screen.h"
#include "../sensors/obstacle_sensor.h"

class BootScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data) override;
    void draw()    override;

private:
    bool needsRedraw_ = true;
    bool canLinked_    = false;
    bool prevCanLinked_ = false;
    obstacle_sensor::SensorStatus sensorStatus_     = obstacle_sensor::SensorStatus::WAITING;
    obstacle_sensor::SensorStatus prevSensorStatus_  = obstacle_sensor::SensorStatus::WAITING;
};

#endif // BOOT_SCREEN_H
