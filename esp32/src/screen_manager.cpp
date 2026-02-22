// =============================================================================
// ESP32-S3 HMI — Screen Manager Implementation
//
// Frame-limited rendering: update() runs every loop iteration,
// but draw() only runs when the frame limiter allows (20 FPS).
// On screen transitions, the frame limiter is forced to allow
// immediate redraw.
//
// Secret code "8989" detection: 4 taps alternating left/right halves
// of the screen within a timeout window activate the engineering menu.
// =============================================================================

#include "screen_manager.h"
#include "ui/runtime_monitor.h"
#include "ui/ui_common.h"

// Secret code timeout — code resets if taps are too slow
static constexpr unsigned long SECRET_CODE_TIMEOUT_MS = 2000;

// Expected tap sequence: alternating left-right-left-right (4 taps)
// Represents "8989" where left half = 8, right half = 9
static constexpr bool SECRET_SEQUENCE[4] = { true, false, true, false }; // true=left, false=right

ScreenManager::ScreenManager()
    : currentScreen_(&bootScreen_)
    , currentState_(can::SystemState::BOOT)
    , frameLimiter_()
    , secretCodePos_(0)
    , secretLastMs_(0)
    , engineeringActive_(false)
{
    currentScreen_->onEnter();
}

void ScreenManager::update(const vehicle::VehicleData& data) {
    // If engineering screen is active, bypass state-based transitions
    if (engineeringActive_) {
        currentScreen_->update(data);
        if (frameLimiter_.shouldDraw()) {
            RTMON_FRAME_BEGIN();
            RTMON_RENDER_BEGIN();
            currentScreen_->draw();
            RTMON_RENDER_END();
            RTMON_FRAME_END();
        }
        return;
    }

    can::SystemState newState = data.heartbeat().systemState;

    if (newState != currentState_) {
        currentScreen_->onExit();
        currentState_  = newState;
        currentScreen_ = screenForState(newState);
        currentScreen_->onEnter();
        frameLimiter_.forceNextFrame();  // Immediate redraw on transition
    }

    // Always update data (fast, no rendering)
    currentScreen_->update(data);

    // Only draw at frame-limited rate
    if (frameLimiter_.shouldDraw()) {
        RTMON_FRAME_BEGIN();
        RTMON_RENDER_BEGIN();
        currentScreen_->draw();
        RTMON_RENDER_END();
        RTMON_FRAME_END();
    }
}

void ScreenManager::onTouch(int16_t x, int16_t y) {
    // If engineering screen is active, dispatch touch to it
    if (engineeringActive_) {
        engineeringScreen_.handleTouch(x, y);
        // Check if user requested exit from engineering mode
        if (engineeringScreen_.exitRequested()) {
            engineeringScreen_.clearExitRequest();
            engineeringActive_ = false;
            currentScreen_->onExit();
            currentScreen_ = screenForState(currentState_);
            currentScreen_->onEnter();
            frameLimiter_.forceNextFrame();
            Serial.println("[ENG] Engineering menu deactivated");
        }
        return;
    }

    // Check for secret code
    checkSecretCode(x);
}

void ScreenManager::checkSecretCode(int16_t x) {
    unsigned long now = millis();

    // Timeout — reset code
    if (secretCodePos_ > 0 &&
        (now - secretLastMs_) > SECRET_CODE_TIMEOUT_MS) {
        secretCodePos_ = 0;
    }

    bool isLeft = (x < ui::SCREEN_W / 2);

    // Check if current tap matches expected position in sequence
    if (isLeft == SECRET_SEQUENCE[secretCodePos_]) {
        secretCodePos_++;
        secretLastMs_ = now;

        if (secretCodePos_ >= SECRET_CODE_LEN) {
            // Code complete — activate engineering screen
            secretCodePos_ = 0;
            engineeringActive_ = true;
            currentScreen_->onExit();
            currentScreen_ = &engineeringScreen_;
            currentScreen_->onEnter();
            frameLimiter_.forceNextFrame();
            Serial.println("[ENG] Engineering menu activated");
        }
    } else {
        // Wrong tap — reset
        secretCodePos_ = 0;
    }
}

Screen* ScreenManager::screenForState(can::SystemState state) {
    switch (state) {
        case can::SystemState::BOOT:      return &bootScreen_;
        case can::SystemState::STANDBY:   return &standbyScreen_;
        case can::SystemState::ACTIVE:    return &driveScreen_;
        case can::SystemState::DEGRADED:  return &driveScreen_;
        case can::SystemState::LIMP_HOME: return &driveScreen_;
        case can::SystemState::SAFE:      return &safeScreen_;
        case can::SystemState::ERROR:     return &errorScreen_;
        default:                          return &errorScreen_;
    }
}
