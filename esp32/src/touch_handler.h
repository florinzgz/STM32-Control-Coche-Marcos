// =============================================================================
// ESP32-S3 HMI — Touch Handler
//
// Centralized touch input management with debounce and dispatch.
// Reads XPT2046 via TFT_eSPI getTouch(), processes events, and
// dispatches to the active screen or global UI elements.
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef TOUCH_HANDLER_H
#define TOUCH_HANDLER_H

#include <cstdint>

namespace touch {

// -------------------------------------------------------------------------
// Touch event types
// -------------------------------------------------------------------------
enum class EventType : uint8_t {
    NONE = 0,
    TAP,           // Short press and release
    LONG_PRESS,    // Press held > LONG_PRESS_MS
    RELEASE        // Finger lifted
};

// -------------------------------------------------------------------------
// Touch event data
// -------------------------------------------------------------------------
struct TouchEvent {
    EventType type = EventType::NONE;
    int16_t   x    = 0;
    int16_t   y    = 0;
};

// -------------------------------------------------------------------------
// Configuration
// -------------------------------------------------------------------------
struct Config {
    uint32_t debounceMs    = 200;    // Minimum ms between taps
    uint32_t longPressMs   = 3000;   // Duration for long press detection
    int16_t  moveCancelPx  = 40;     // Max finger drift (px) before the
                                     // long-press timer restarts. Guards the
                                     // global long-press gesture against
                                     // accidental swipes / false positives.
};

/// Initialize touch handler.  Call once from setup().
void init(const Config& cfg = Config{});

/// Update touch state.  Call from loop() every iteration.
/// @param isTouched  Result of tft.getTouch()
/// @param rawX       Raw touch X coordinate
/// @param rawY       Raw touch Y coordinate
void update(bool isTouched, int16_t rawX, int16_t rawY);

/// Get the latest touch event (consumed on read — returns NONE after).
TouchEvent getEvent();

/// Check if touch is currently pressed (raw state).
bool isPressed();

/// Get current touch coordinates (valid only when isPressed).
void getPosition(int16_t& x, int16_t& y);

} // namespace touch

#endif // TOUCH_HANDLER_H
