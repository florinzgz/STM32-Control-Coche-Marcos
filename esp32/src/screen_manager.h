// =============================================================================
// ESP32-S3 HMI — Screen Manager
//
// State machine that selects the active Screen based on
// vehicleData.heartbeat().systemState (byte 1 of CAN 0x001).
//
// Integrates frame limiter: update() is called every loop,
// but draw() only executes at the target frame rate (20 FPS).
//
// Reference: docs/HMI_STATE_MODEL.md
//            docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef SCREEN_MANAGER_H
#define SCREEN_MANAGER_H

#include "can_ids.h"
#include "vehicle_data.h"
#include "screens/screen.h"
#include "screens/boot_screen.h"
#include "screens/standby_screen.h"
#include "screens/drive_screen.h"
#include "screens/safe_screen.h"
#include "screens/error_screen.h"
#include "ui/frame_limiter.h"

class ScreenManager {
public:
    ScreenManager();

    /// Call once per loop after can_rx::poll().
    /// Detects state changes and forwards data to the active screen.
    /// draw() is only called at the frame-limited rate.
    void update(const vehicle::VehicleData& data);

private:
    Screen* screenForState(can::SystemState state);

    BootScreen    bootScreen_;
    StandbyScreen standbyScreen_;
    DriveScreen   driveScreen_;
    SafeScreen    safeScreen_;
    ErrorScreen   errorScreen_;

    Screen*           currentScreen_;
    can::SystemState  currentState_;
    ui::FrameLimiter  frameLimiter_;
};

#endif // SCREEN_MANAGER_H
