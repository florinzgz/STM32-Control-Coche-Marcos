// =============================================================================
// ESP32-S3 HMI — Safe Screen
//
// Shown when system_state = SAFE (4).
// Prominent safety warning banner. Limited telemetry visible.
// All controls disabled. No heap allocation.
//
// Reference: docs/HMI_STATE_MODEL.md §2.5
// =============================================================================

#ifndef SAFE_SCREEN_H
#define SAFE_SCREEN_H

#include "screen.h"
#include <cstdint>

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
};

#endif // SAFE_SCREEN_H
