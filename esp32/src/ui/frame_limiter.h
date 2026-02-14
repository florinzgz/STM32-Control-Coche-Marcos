// =============================================================================
// ESP32-S3 HMI — Frame Limiter
//
// Deterministic frame rate control. Ensures draw() is only called
// at the target interval (default 50 ms = 20 FPS).
// Uses millis() comparison — never blocks.
//
// Reference: docs/HMI_RENDERING_STRATEGY.md §2.2
// =============================================================================

#ifndef FRAME_LIMITER_H
#define FRAME_LIMITER_H

#include <Arduino.h>

namespace ui {

class FrameLimiter {
public:
    static constexpr unsigned long DEFAULT_INTERVAL_MS = 50;  // 20 FPS

    explicit FrameLimiter(unsigned long intervalMs = DEFAULT_INTERVAL_MS)
        : intervalMs_(intervalMs)
        , lastFrameMs_(0)
    {}

    /// Returns true if enough time has elapsed for a new frame.
    /// Call this before draw(). Non-blocking.
    bool shouldDraw() {
        unsigned long now = millis();
        if (now - lastFrameMs_ >= intervalMs_) {
            lastFrameMs_ = now;
            return true;
        }
        return false;
    }

    /// Force next shouldDraw() to return true (e.g., on screen enter).
    void forceNextFrame() {
        lastFrameMs_ = 0;
    }

private:
    unsigned long intervalMs_;
    unsigned long lastFrameMs_;
};

} // namespace ui

#endif // FRAME_LIMITER_H
