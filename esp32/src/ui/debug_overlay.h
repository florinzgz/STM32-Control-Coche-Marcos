// =============================================================================
// ESP32-S3 HMI — Debug Overlay
//
// Hidden debug overlay activated by holding touch for 3 seconds.
// Displays runtime performance stats over the current screen.
// Uses existing TFT — no new libraries, no sprites.
//
// Compiles out when RUNTIME_MONITOR is 0.
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef DEBUG_OVERLAY_H
#define DEBUG_OVERLAY_H

#include "runtime_monitor.h"

#if RUNTIME_MONITOR

#include <TFT_eSPI.h>

namespace rtmon {

class DebugOverlay {
public:
    /// Call every loop iteration with current touch state.
    /// touchDown: true if screen is currently being touched
    /// Returns true if overlay is currently visible.
    static bool update(bool touchDown);

    /// Draw overlay if visible. Call after screen draw().
    /// Uses direct TFT calls on existing instance.
    static void draw(TFT_eSPI& tft);

    /// Returns true if overlay is currently visible
    static bool isVisible();

private:
    // Overlay dimensions
    static constexpr int16_t OVL_X      = 80;
    static constexpr int16_t OVL_Y      = 60;
    static constexpr int16_t OVL_W      = 320;
    static constexpr int16_t OVL_H      = 200;
    static constexpr int16_t LINE_H     = 20;
    static constexpr int16_t MARGIN     = 10;

    static bool     visible_;
    static bool     prevTouchDown_;
    static uint32_t touchStartMs_;
    static uint32_t lastUpdateMs_;

    static constexpr uint32_t HOLD_THRESHOLD_MS = 3000;  // 3 seconds
    static constexpr uint32_t REFRESH_MS        = 500;   // Update every 500 ms
};

} // namespace rtmon

#define RTMON_OVERLAY_UPDATE(touch)   rtmon::DebugOverlay::update(touch)
#define RTMON_OVERLAY_DRAW(tft)       rtmon::DebugOverlay::draw(tft)
#define RTMON_OVERLAY_VISIBLE()       rtmon::DebugOverlay::isVisible()

#else  // RUNTIME_MONITOR == 0

#define RTMON_OVERLAY_UPDATE(touch)   (false)
#define RTMON_OVERLAY_DRAW(tft)       ((void)0)
#define RTMON_OVERLAY_VISIBLE()       (false)

#endif // RUNTIME_MONITOR

#endif // DEBUG_OVERLAY_H
