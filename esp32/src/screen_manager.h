// =============================================================================
// ESP32-S3 HMI — Screen Manager
//
// State machine that selects the active Screen based on
// vehicleData.heartbeat().systemState (byte 1 of CAN 0x001).
//
// Integrates frame limiter: update() is called every loop,
// but draw() only executes at the target frame rate (20 FPS).
//
// Engineering menu access:
//   1. Long-press (3 s) on the battery icon → PIN entry screen.
//   2. Enter the correct 4-digit PIN (8989) on the keypad.
//   3. On success → EngineeringScreen.
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
#include "screens/engineering_screen.h"
#include "screens/pin_screen.h"
#include "ui/frame_limiter.h"

class ScreenManager {
public:
    ScreenManager();

    /// Call once per loop after can_rx::poll().
    /// Detects state changes and forwards data to the active screen.
    /// draw() is only called at the frame-limited rate.
    void update(const vehicle::VehicleData& data);

    /// Forward a TAP event.  Routes to PIN / engineering screen when active.
    void onTouch(int16_t x, int16_t y);

    /// Forward a LONG_PRESS event.  Activates PIN screen if battery icon pressed.
    void onLongPress(int16_t x, int16_t y);

    /// Returns true while PIN entry or engineering screen is showing.
    /// Use to suppress normal top-bar touch handling in main loop.
    bool isBlockingInput() const { return pinActive_ || engineeringActive_; }

    /// Returns true when the current screen is the initial boot screen
    /// (COCHE MARCOS / HMI v1.0 / CAN: WAITING / SENSOR: WAITING).
    bool isInitialScreen() const;

private:
    Screen* screenForState(can::SystemState state);
    void    activatePinScreen();

    BootScreen         bootScreen_;
    StandbyScreen      standbyScreen_;
    DriveScreen        driveScreen_;
    SafeScreen         safeScreen_;
    ErrorScreen        errorScreen_;
    EngineeringScreen  engineeringScreen_;
    PinScreen          pinScreen_;

    Screen*           currentScreen_;
    can::SystemState  currentState_;
    ui::FrameLimiter  frameLimiter_;

    bool pinActive_         = false;  // true while PIN screen is shown
    bool engineeringActive_ = false;  // true while engineering screen is shown
    bool canLost_           = false;  // true when STM32 heartbeat lost > CAN_LOSS_TIMEOUT_MS
};

#endif // SCREEN_MANAGER_H
