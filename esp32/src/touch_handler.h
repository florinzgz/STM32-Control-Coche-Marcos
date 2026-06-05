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
    CORNER_LONG_PRESS, // Press held inside the top-right corner zone (robust
                       // Engineering-access fallback; see Config corner fields)
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

    // ---- Robust corner long-press (Engineering-access fallback) ----
    // A touch that REMAINS inside this top-right rectangle (screen coords)
    // for cornerLongPressMs emits CORNER_LONG_PRESS.  Unlike the global
    // long-press, it re-anchors to the zone instead of the initial contact
    // point, so finger drift / resistive-panel noise inside the corner never
    // cancels it — only leaving the zone does.  The default zone covers the
    // battery icon (BAT_X=405..470, BAT_Y=6..34) plus margin.
    int16_t  cornerX0          = 380;   // left   edge of corner zone
    int16_t  cornerY0          = 0;     // top    edge of corner zone
    int16_t  cornerX1          = 480;   // right  edge of corner zone
    int16_t  cornerY1          = 80;    // bottom edge of corner zone
    uint32_t cornerLongPressMs = 3000;  // hold duration inside the zone
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
