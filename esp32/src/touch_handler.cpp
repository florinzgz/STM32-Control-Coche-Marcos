// =============================================================================
// ESP32-S3 HMI — Touch Handler Implementation
//
// Centralized touch input with debounce, tap/long-press detection.
// Consumes raw XPT2046 coordinates and produces TouchEvent structs.
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#include "touch_handler.h"
#include <Arduino.h>

namespace touch {

// Module state
static Config       cfg_;
static bool         initialized_    = false;
static bool         currentPressed_ = false;
static bool         prevPressed_    = false;
static int16_t      curX_           = 0;
static int16_t      curY_           = 0;
static unsigned long pressStartMs_  = 0;
static int16_t      pressStartX_    = 0;
static int16_t      pressStartY_    = 0;
static unsigned long lastTapMs_     = 0;
static bool         longPressEmitted_ = false;
static TouchEvent   pendingEvent_   = {};

void init(const Config& cfg) {
    cfg_             = cfg;
    initialized_     = true;
    currentPressed_  = false;
    prevPressed_     = false;
    curX_            = 0;
    curY_            = 0;
    pressStartMs_    = 0;
    pressStartX_     = 0;
    pressStartY_     = 0;
    lastTapMs_       = 0;
    longPressEmitted_ = false;
    pendingEvent_    = {};
    Serial.println("[TOUCH] Handler initialized");
}

void update(bool isTouched, int16_t rawX, int16_t rawY) {
    if (!initialized_) return;

    unsigned long now = millis();
    prevPressed_    = currentPressed_;
    currentPressed_ = isTouched;

    if (isTouched) {
        curX_ = rawX;
        curY_ = rawY;
    }

    // Rising edge — finger down
    if (isTouched && !prevPressed_) {
        pressStartMs_     = now;
        pressStartX_      = rawX;
        pressStartY_      = rawY;
        longPressEmitted_ = false;
    }

    // Held — check for long press
    if (isTouched && prevPressed_ && !longPressEmitted_) {
        // Anti false-positive guard: if the finger drifts too far from where
        // it first landed, treat it as a swipe/drag and restart the timer
        // (re-anchoring at the current point).  This protects the global
        // long-press gesture (open Engineering PIN) from accidental swipes
        // without ever blocking render or touch.
        int16_t dx = static_cast<int16_t>(curX_ - pressStartX_);
        int16_t dy = static_cast<int16_t>(curY_ - pressStartY_);
        if (dx < 0) dx = static_cast<int16_t>(-dx);
        if (dy < 0) dy = static_cast<int16_t>(-dy);
        if (dx > cfg_.moveCancelPx || dy > cfg_.moveCancelPx) {
            pressStartMs_ = now;
            pressStartX_  = curX_;
            pressStartY_  = curY_;
        } else if ((now - pressStartMs_) >= cfg_.longPressMs) {
            pendingEvent_ = { EventType::LONG_PRESS, curX_, curY_ };
            longPressEmitted_ = true;
        }
    }

    // Falling edge — finger up
    if (!isTouched && prevPressed_) {
        if (!longPressEmitted_) {
            // Short tap — apply debounce
            if ((now - lastTapMs_) >= cfg_.debounceMs) {
                pendingEvent_ = { EventType::TAP, curX_, curY_ };
                lastTapMs_    = now;
            }
        }
        // Always emit release if no other event was generated
        if (pendingEvent_.type == EventType::NONE) {
            pendingEvent_ = { EventType::RELEASE, curX_, curY_ };
        }
    }
}

TouchEvent getEvent() {
    TouchEvent evt = pendingEvent_;
    pendingEvent_ = {};
    return evt;
}

bool isPressed() {
    return currentPressed_;
}

void getPosition(int16_t& x, int16_t& y) {
    x = curX_;
    y = curY_;
}

} // namespace touch
