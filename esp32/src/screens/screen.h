// =============================================================================
// ESP32-S3 HMI — Screen Base Class
//
// Abstract interface for all HMI screens.
// Subclasses implement rendering via TFT_eSPI.
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

    /// Called every loop iteration with latest vehicle data.
    /// @param data   Frozen CAN snapshot for this frame
    /// @param frameTimeMs  Wall-clock time captured once per frame (millis()).
    ///        All timing logic in update() MUST use this value instead of
    ///        calling millis() directly, ensuring deterministic behavior:
    ///        same (data, frameTimeMs) ⇒ same derived state ALWAYS.
    virtual void update(const vehicle::VehicleData& data, unsigned long frameTimeMs) = 0;

    /// Called at the frame-limited rate to render the screen via TFT_eSPI
    virtual void draw() = 0;
};

#endif // SCREEN_BASE_H
