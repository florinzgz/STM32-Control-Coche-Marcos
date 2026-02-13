// =============================================================================
// ESP32-S3 HMI — Screen Manager Implementation
//
// Frame-limited rendering: update() runs every loop iteration,
// but draw() only runs when the frame limiter allows (20 FPS).
// On screen transitions, the frame limiter is forced to allow
// immediate redraw.
// =============================================================================

#include "screen_manager.h"

ScreenManager::ScreenManager()
    : currentScreen_(&bootScreen_)
    , currentState_(can::SystemState::BOOT)
    , frameLimiter_()
{
    currentScreen_->onEnter();
}

void ScreenManager::update(const vehicle::VehicleData& data) {
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
        currentScreen_->draw();
    }
}

Screen* ScreenManager::screenForState(can::SystemState state) {
    switch (state) {
        case can::SystemState::BOOT:     return &bootScreen_;
        case can::SystemState::STANDBY:  return &standbyScreen_;
        case can::SystemState::ACTIVE:   return &driveScreen_;
        case can::SystemState::DEGRADED: return &driveScreen_;
        case can::SystemState::SAFE:     return &safeScreen_;
        case can::SystemState::ERROR:    return &errorScreen_;
        default:                         return &errorScreen_;
    }
}
