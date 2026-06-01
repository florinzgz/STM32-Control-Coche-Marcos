// =============================================================================
// ESP32-S3 HMI — Screen Manager Implementation
//
// Frame-limited rendering: update() runs every loop iteration,
// but draw() only runs when the frame limiter allows (20 FPS).
// On screen transitions, the frame limiter is forced to allow
// immediate redraw.
//
// Engineering menu access flow:
//   Long-press (3 s) on any free screen area → PIN screen → enter 8989 → engineering.
//   (Long-pressing the battery icon still works — it is a subset of the screen.)
// =============================================================================

#include "screen_manager.h"
#include "ui/runtime_monitor.h"
#include "ui/ui_common.h"
#include "touch_calibration.h"
#include <TFT_eSPI.h>
#include <Arduino.h>

extern TFT_eSPI tft;

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
    // Capture wall-clock time ONCE per frame. All screen update() logic
    // uses this injected value instead of calling millis() directly,
    // ensuring deterministic behavior: same (data, frameTimeMs) ⇒ same UI.
    const unsigned long frameTimeMs = millis();

    // Frame Time Contract (V10): assert monotonicity.
    // millis() on ESP32 is monotonically non-decreasing; a backward jump
    // would indicate a systemic bug (timer reset, memory corruption).
    // Unsigned subtraction handles the 2^32 wrap correctly (~49.7 days).
#if UI_TILE_DEBUG
    if (frameTimeMs < prevFrameTimeMs_ &&
        (prevFrameTimeMs_ - frameTimeMs) < 0x80000000UL) {
        // Not a wrap — genuine backward jump (should never happen)
        Serial.printf("[FRAME] WARNING: frameTimeMs went backwards: %lu → %lu\n",
                      prevFrameTimeMs_, frameTimeMs);
    }
#endif
    prevFrameTimeMs_ = frameTimeMs;

    // ---- Touch-calibration wizard active ----
    // Highest priority: when the wizard is up, no other screen logic runs.
    // The wizard owns the entire touch & display pipeline until SAVE/CANCEL.
    if (touchCalActive_) {
        RTMON_UI_BEGIN();
        touchCalScreen_.update(data, frameTimeMs);
        RTMON_UI_END();
        if (frameLimiter_.shouldDraw()) {
            RTMON_FRAME_BEGIN();
            RTMON_RENDER_BEGIN();
            touchCalScreen_.draw();
            RTMON_RENDER_END();
            RTMON_FRAME_END();
        }
        if (touchCalScreen_.isSaved()) {
            // Apply the new calibration to the live driver in-place.
            // Cast away const because TFT_eSPI::setTouch takes a non-const
            // pointer despite not modifying the data.
            uint16_t cal[touch_calibration::CAL_DATA_LEN];
            memcpy(cal, touchCalScreen_.result(), sizeof(cal));
            tft.setTouch(cal);
            Serial.println("[TOUCH_CAL] New calibration applied to TFT_eSPI");
            touchCalActive_ = false;
            touchCalScreen_.clearResultFlags();
            currentScreen_->onEnter();    // force full redraw of underlying screen
            frameLimiter_.forceNextFrame();
        } else if (touchCalScreen_.isCancelled()) {
            // Mark first-boot done even on cancel from the engineering menu
            // so the wizard never auto-launches again — only by request.
            touch_calibration::markFirstBootDone();
            touchCalActive_ = false;
            touchCalScreen_.clearResultFlags();
            currentScreen_->onEnter();
            frameLimiter_.forceNextFrame();
            Serial.println("[TOUCH_CAL] Wizard exited without saving");
        }
        return;
    }

    // ---- Engineering screen active ----
    if (engineeringActive_) {
        RTMON_UI_BEGIN();
        currentScreen_->update(data, frameTimeMs);
        RTMON_UI_END();
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

            // If the exit was triggered by the "TOUCH CALIBRATION" entry,
            // launch the wizard immediately (same frame) so the user does
            // not see a flash of the underlying screen.
            if (engineeringScreen_.consumeTouchCalRequest()) {
                requestTouchWizard(/*firstBoot=*/false);
            }
        }
        return;
    }

    // ---- PIN screen active ----
    if (pinActive_) {
        RTMON_UI_BEGIN();
        pinScreen_.update(data, frameTimeMs);
        RTMON_UI_END();
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

    // ---- CAN-loss detection ----
    // If the STM32 heartbeat was once received (timestampMs > 0) but is now
    // older than CAN_LOSS_TIMEOUT_MS, the CAN cable is likely disconnected.
    // Force transition to the ERROR screen so the user gets visual feedback.
    // On the BOOT screen the existing "CAN: WAITING..." indicator suffices.
    {
        unsigned long hbTs = data.heartbeat().timestampMs;
        if (hbTs > 0) {
            unsigned long age = frameTimeMs - hbTs;
            bool stale = (age > can::CAN_LOSS_TIMEOUT_MS);

            if (stale && currentState_ != can::SystemState::BOOT) {
                if (!canLost_) {
                    canLost_ = true;
                    Serial.printf("[SCREEN] CAN lost — no heartbeat for >%lu ms\n",
                                  (unsigned long)can::CAN_LOSS_TIMEOUT_MS);
                }
                newState = can::SystemState::ERROR;
            } else if (canLost_ && !stale) {
                canLost_ = false;
                Serial.println("[SCREEN] CAN communication restored");
            }
        }
    }

    if (newState != currentState_) {
        currentScreen_->onExit();
        currentState_  = newState;
        currentScreen_ = screenForState(newState);
        currentScreen_->onEnter();
        frameLimiter_.forceNextFrame();  // Immediate redraw on transition
    }

    RTMON_UI_BEGIN();
    currentScreen_->update(data, frameTimeMs);
    RTMON_UI_END();

    if (frameLimiter_.shouldDraw()) {
        RTMON_FRAME_BEGIN();
        RTMON_RENDER_BEGIN();
        currentScreen_->draw();
        RTMON_RENDER_END();
        RTMON_FRAME_END();
    }
}

void ScreenManager::onTouch(int16_t x, int16_t y) {
    if (touchCalActive_) {
        touchCalScreen_.handleTouch(x, y);
        return;
    }

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
    // Only activate when no overlay is already shown (ignore long-press while
    // the PIN, engineering or touch-calibration screens own the pipeline).
    if (engineeringActive_ || pinActive_ || touchCalActive_) return;

    // Global gesture: a 3 s long-press on ANY free area of the screen opens the
    // PIN entry.  The battery-icon region is a subset of the full screen, so
    // the previous "long-press the battery icon" gesture still works for
    // backward compatibility.  Because dispatch is not state-gated, this also
    // works in Safe Mode where the battery icon may not be visible/accessible.
    (void)x;
    (void)y;
    activatePinScreen();
}

void ScreenManager::activatePinScreen() {
    pinActive_ = true;
    pinScreen_.onEnter();
    frameLimiter_.forceNextFrame();
    Serial.println("[PIN] Global long press → PIN screen");
}

void ScreenManager::requestTouchWizard(bool firstBoot) {
    if (touchCalActive_) return;   // already active — idempotent

    // Fully replace the active screen with the wizard.  The previous
    // screen is paused; on wizard exit we re-enter it (forces a redraw).
    touchCalActive_ = true;
    touchCalScreen_.setFirstBoot(firstBoot);
    touchCalScreen_.onEnter();
    frameLimiter_.forceNextFrame();
    Serial.printf("[TOUCH_CAL] Wizard requested (firstBoot=%d)\n",
                  static_cast<int>(firstBoot));
}

bool ScreenManager::isInitialScreen() const {
    return currentState_ == can::SystemState::BOOT;
}

bool ScreenManager::isTankConfirmActive() const {
    // Only relevant when drive screen is active (ACTIVE/DEGRADED/LIMP_HOME)
    if (currentScreen_ != &driveScreen_) return false;
    return driveScreen_.isTankConfirmVisible();
}

void ScreenManager::showTankConfirm() {
    if (currentScreen_ == &driveScreen_) {
        driveScreen_.showTankConfirm();
    }
}

uint8_t ScreenManager::handleTankConfirmTouch(int16_t x, int16_t y) {
    if (currentScreen_ != &driveScreen_) return 0;
    return driveScreen_.handleTankConfirmTouch(x, y);
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
