// =============================================================================
// ESP32-S3 HMI — Standby Screen
//
// Shown when system_state = STANDBY (1).
// Displays "System Ready" with CAN link status and temperatures.
// No drive controls. No heap allocation.
//
// Reference: docs/HMI_STATE_MODEL.md §2.2
// =============================================================================

#ifndef STANDBY_SCREEN_H
#define STANDBY_SCREEN_H

#include "screen.h"
#include <cstdint>

class StandbyScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data) override;
    void draw()    override;

private:
    bool    needsRedraw_   = true;
    uint8_t faultFlags_    = 0;
    uint8_t prevFaultFlags_ = 0xFF;
    int8_t  temps_[5]      = {};
    int8_t  prevTemps_[5]  = {};
};

#endif // STANDBY_SCREEN_H
