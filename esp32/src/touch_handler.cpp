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

// Corner long-press tracker (anchored to the zone, independent of the global
// long-press drift logic). See touch_handler.h Config corner fields.
static unsigned long cornerStartMs_ = 0;
static bool          cornerTracking_ = false;
static bool          cornerEmitted_  = false;

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
    cornerStartMs_   = 0;
    cornerTracking_  = false;
    cornerEmitted_   = false;
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
        // Reset corner tracker on every fresh contact so its emitted-latch
        // (used to suppress the tap-on-release) only spans a single press.
        cornerTracking_   = false;
        cornerEmitted_    = false;
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

    // Robust corner long-press (Engineering-access fallback).
    // Anchored to the top-right zone: drift INSIDE the zone never cancels it,
    // only leaving the zone (or releasing) does.  This survives noisy or
    // miscalibrated resistive panels that keep resetting the global gesture.
    // Guarded so it never overwrites a LONG_PRESS already produced this frame.
    {
        bool inCorner = isTouched &&
            curX_ >= cfg_.cornerX0 && curX_ <= cfg_.cornerX1 &&
            curY_ >= cfg_.cornerY0 && curY_ <= cfg_.cornerY1;
        if (inCorner) {
            if (!cornerTracking_) {
                cornerTracking_ = true;
                cornerStartMs_  = now;
            } else if (!cornerEmitted_ &&
                       (now - cornerStartMs_) >= cfg_.cornerLongPressMs) {
                if (pendingEvent_.type == EventType::NONE) {
                    pendingEvent_ = { EventType::CORNER_LONG_PRESS, curX_, curY_ };
                }
                cornerEmitted_ = true;
            }
        } else {
            // Left the zone — allow the timer to restart on re-entry.  Do NOT
            // clear cornerEmitted_ here: it must stay latched until the next
            // finger-down so the release below can suppress a spurious tap.
            cornerTracking_ = false;
        }
    }

    // Falling edge — finger up
    if (!isTouched && prevPressed_) {
        if (!longPressEmitted_ && !cornerEmitted_) {
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
