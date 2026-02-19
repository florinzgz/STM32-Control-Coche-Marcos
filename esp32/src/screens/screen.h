// =============================================================================
// ESP32-S3 HMI — Screen Base Class
//
// Abstract interface for all HMI screens.
// No graphics library, no TFT — pure interface for future implementation.
//
// Reference: docs/ESP32_FIRMWARE_DESIGN.md rev 1.1
// =============================================================================

#ifndef SCREEN_BASE_H
#define SCREEN_BASE_H

#include "vehicle_data.h"

class Screen {
public:
    virtual ~Screen() = default;

    /// Called when this screen becomes active
    virtual void onEnter() = 0;

    /// Called when this screen is about to be replaced
    virtual void onExit() = 0;

    /// Called every loop iteration with latest vehicle data
    virtual void update(const vehicle::VehicleData& data) = 0;

    /// Called every loop iteration to render (stub — no TFT yet)
    virtual void draw() = 0;
};

#endif // SCREEN_BASE_H
