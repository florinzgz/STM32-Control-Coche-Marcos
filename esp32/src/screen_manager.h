// =============================================================================
// ESP32-S3 HMI — Screen Manager
//
// State machine that selects the active Screen based on
// vehicleData.heartbeat().systemState (byte 1 of CAN 0x001).
//
// Integrates frame limiter: update() is called every loop,
// but draw() only executes at the target frame rate (20 FPS).
//
// TILE-BASED DIRTY REGION RENDER ENGINE:
//   CAN RX (Core 1) → VehicleData → mutex copy → localVD snapshot (Core 0)
//   → screenManager.update(snapshot):
//       1. screen.update(snapshot) → copies into cur_*, computes tile hashes
//       2. TileSet::updateHash() → compares hash → marks dirty if changed
//       3. frameLimiter_.shouldDraw() gates at 20 FPS
//       4. screen.draw() iterates tiles, only renders dirty ones
//   Result: each rendered frame uses data from a single consistent time instant.
//   CAN updates arriving mid-render do NOT affect the current frame.
//   Only tiles whose content hash changed since last frame are redrawn.
//
// ANTI-FLICKER STRATEGY:
//   - Static layer (car body, labels, outlines) drawn once in onEnter()
//   - Dynamic layer uses tile dirty flags: only changed tiles are repainted
//   - Tile hash comparison: FNV-1a hash per tile for fast skip decision
//   - Text updates use setTextPadding() to overwrite in a single SPI pass
//     instead of fillRect+drawString (which causes visible blank flash)
//   - Bar widgets (pedal, battery, obstacle) use differential update:
//     only the portion that changed is painted, not clear+redraw
//   - CAN-loss overlay: partial redraw of banner tile only, not full screen
//   - Overlay tiles restore underlying tiles when dismissed
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
#include "screens/touch_calibration_screen.h"
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
    bool isBlockingInput() const { return pinActive_ || engineeringActive_ || touchCalActive_; }

    /// Returns true while the touch-calibration wizard is active.  When
    /// true, main.cpp must NOT feed taps from the (potentially miscalibrated)
    /// touch driver into the normal touch_handler/screen pipeline — the
    /// wizard owns the touch flow exclusively.
    bool isTouchCalActive() const { return touchCalActive_; }

    /// Launch the touch-calibration wizard.  Called automatically once on
    /// first boot from main.cpp, and on demand from the engineering menu.
    /// @param firstBoot  When true, the wizard hides the CANCEL button and
    ///                   forces the user to complete a calibration.
    void requestTouchWizard(bool firstBoot);

    /// Returns true when the current screen is the initial boot screen
    /// (COCHE MARCOS / HMI v1.0 / CAN: WAITING / SENSOR: WAITING).
    bool isInitialScreen() const;

    /// Returns true if the DriveScreen is active and showing the tank confirm dialog.
    bool isTankConfirmActive() const;

    /// Show the tank turn confirmation dialog on the DriveScreen.
    void showTankConfirm();

    /// Forward a touch to the tank confirm dialog.
    /// Returns: 0=not consumed, 1=YES (confirmed), 2=NO (cancelled).
    uint8_t handleTankConfirmTouch(int16_t x, int16_t y);

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
    TouchCalibrationScreen touchCalScreen_;

    Screen*           currentScreen_;
    can::SystemState  currentState_;
    ui::FrameLimiter  frameLimiter_;

    bool pinActive_         = false;  // true while PIN screen is shown
    bool engineeringActive_ = false;  // true while engineering screen is shown
    bool touchCalActive_    = false;  // true while touch-calibration wizard is shown
    bool canLost_           = false;  // true when STM32 heartbeat lost > CAN_LOSS_TIMEOUT_MS

    // Frame time monotonicity tracking (V10 contract)
    unsigned long prevFrameTimeMs_ = 0;
};

#endif // SCREEN_MANAGER_H
