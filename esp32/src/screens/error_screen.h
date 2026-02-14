// =============================================================================
// ESP32-S3 HMI — Error Screen
//
// Shown when system_state = ERROR (5).
// Red full-screen overlay with "SYSTEM ERROR" banner.
// Manual reset required. All controls disabled.
// Telemetry frozen at last known values. No heap allocation.
//
// Reference: docs/HMI_STATE_MODEL.md §2.6
// =============================================================================

#ifndef ERROR_SCREEN_H
#define ERROR_SCREEN_H

#include "screen.h"
#include <cstdint>

class ErrorScreen : public Screen {
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
    uint8_t diagCode_       = 0;
    uint8_t prevDiagCode_   = 0xFF;
    uint8_t diagSubsystem_  = 0;
};

#endif // ERROR_SCREEN_H
