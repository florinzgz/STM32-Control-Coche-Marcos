// =============================================================================
// ESP32-S3 HMI — Screen Manager Implementation
//
// Frame-limited rendering: update() runs every loop iteration,
// but draw() only runs when the frame limiter allows (20 FPS).
// On screen transitions, the frame limiter is forced to allow
// immediate redraw.
//
// Engineering menu access flow:
//   Long-press (3 s) on battery icon → PIN screen → enter 8989 → engineering.
// =============================================================================

#include "screen_manager.h"
#include "ui/runtime_monitor.h"
#include "ui/ui_common.h"

ScreenManager::ScreenManager()
    : currentScreen_(&bootScreen_)
    , currentState_(can::SystemState::BOOT)
    , frameLimiter_()
    , pinActive_(false)
    , engineeringActive_(false)
{
    currentScreen_->onEnter();
}

void ScreenManager::update(const vehicle::VehicleData& data) {
    // ---- Engineering screen active ----
    if (engineeringActive_) {
        currentScreen_->update(data);
        if (frameLimiter_.shouldDraw()) {
            RTMON_FRAME_BEGIN();
            RTMON_RENDER_BEGIN();
            currentScreen_->draw();
            RTMON_RENDER_END();
            RTMON_FRAME_END();
        }
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

    // ---- PIN screen active ----
    if (pinActive_) {
        pinScreen_.update(data);
        if (frameLimiter_.shouldDraw()) {
            RTMON_FRAME_BEGIN();
            RTMON_RENDER_BEGIN();
            pinScreen_.draw();
            RTMON_RENDER_END();
            RTMON_FRAME_END();
        }
        if (pinScreen_.isConfirmed()) {
            // Correct PIN — enter engineering
            pinActive_         = false;
            engineeringActive_ = true;
            currentScreen_->onExit();
            currentScreen_ = &engineeringScreen_;
            currentScreen_->onEnter();
            frameLimiter_.forceNextFrame();
            Serial.println("[ENG] Engineering menu activated");
        } else if (pinScreen_.isCancelled()) {
            // User cancelled — back to normal screen
            pinActive_ = false;
            pinScreen_.reset();
            currentScreen_->onEnter();   // Force full redraw
            frameLimiter_.forceNextFrame();
            Serial.println("[PIN] Cancelled");
        }
        return;
    }

    // ---- Normal state-machine ----
    can::SystemState newState = data.heartbeat().systemState;

    if (newState != currentState_) {
        currentScreen_->onExit();
        currentState_  = newState;
        currentScreen_ = screenForState(newState);
        currentScreen_->onEnter();
        frameLimiter_.forceNextFrame();  // Immediate redraw on transition
    }

    currentScreen_->update(data);

    if (frameLimiter_.shouldDraw()) {
        RTMON_FRAME_BEGIN();
        RTMON_RENDER_BEGIN();
        currentScreen_->draw();
        RTMON_RENDER_END();
        RTMON_FRAME_END();
    }
}

void ScreenManager::onTouch(int16_t x, int16_t y) {
    if (engineeringActive_) {
        engineeringScreen_.handleTouch(x, y);
        return;
    }

    if (pinActive_) {
        pinScreen_.handleTouch(x, y);
        return;
    }

    // Normal screens: no tap-based secret code anymore
    (void)x;
    (void)y;
}

void ScreenManager::onLongPress(int16_t x, int16_t y) {
    // Only activate when neither overlay is already shown
    if (engineeringActive_ || pinActive_) return;

    // Hit-test the battery icon (ui::BAT_X, BAT_Y, BAT_W, BAT_H)
    if (x >= ui::BAT_X && x <= ui::BAT_X + ui::BAT_W &&
        y >= ui::BAT_Y && y <= ui::BAT_Y + ui::BAT_H) {
        activatePinScreen();
    }
}

void ScreenManager::activatePinScreen() {
    pinActive_ = true;
    pinScreen_.onEnter();
    frameLimiter_.forceNextFrame();
    Serial.println("[PIN] Battery long press → PIN screen");
}

bool ScreenManager::isInitialScreen() const {
    return currentState_ == can::SystemState::BOOT;
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
