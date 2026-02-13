// =============================================================================
// ESP32-S3 HMI — Screen Manager Implementation
// =============================================================================

#include "screen_manager.h"

ScreenManager::ScreenManager()
    : currentScreen_(&bootScreen_)
    , currentState_(can::SystemState::BOOT)
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
    }

    currentScreen_->update(data);
    currentScreen_->draw();
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
