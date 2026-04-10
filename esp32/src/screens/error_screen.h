// =============================================================================
// ESP32-S3 HMI — Error Screen
//
// Shown when system_state = ERROR (5).
// Red full-screen overlay with "SYSTEM ERROR" banner.
// Displays full human-readable fault flags, safety error name, diagnostic
// subsystem name, and elapsed time (permanence) since the error occurred.
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
    static const char* safetyErrorName(uint8_t code);
    static const char* diagSubsystemName(uint8_t sub);

    bool     needsRedraw_    = true;
    bool     canLost_        = false;   // CAN communication lost (ESP32-detected)
    bool     prevCanLost_    = false;   // previous value for redraw detection
    uint8_t  faultFlags_     = 0;
    uint8_t  prevFaultFlags_ = 0xFF;
    uint8_t  errorCode_      = 0;
    uint8_t  prevErrorCode_  = 0xFF;
    uint8_t  diagCode_       = 0;
    uint8_t  prevDiagCode_   = 0xFF;
    uint8_t  diagSubsystem_  = 0;
    uint8_t  prevDiagSubsystem_ = 0xFF;
    uint32_t errorEntryMs_   = 0;      // millis() when error screen entered
    uint32_t prevElapsedSec_ = 0xFFFFFFFF;
};

#endif // ERROR_SCREEN_H
