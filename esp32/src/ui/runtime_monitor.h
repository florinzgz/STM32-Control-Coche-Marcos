// =============================================================================
// ESP32-S3 HMI — Runtime Monitor
//
// Lightweight instrumentation layer for measuring Drive Screen performance.
// Tracks frame timing, zone redraw counts, and phase durations.
//
// Enable with: #define RUNTIME_MONITOR 1
// Disable with: #define RUNTIME_MONITOR 0  (zero overhead when disabled)
//
// Constraints:
//   - NO heap allocation
//   - NO String class
//   - Fixed-size ring buffer (120 frames)
//   - Overhead < 50 µs per frame
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef RUNTIME_MONITOR_H
#define RUNTIME_MONITOR_H

// Master switch — set to 0 to compile out all instrumentation
#ifndef RUNTIME_MONITOR
#define RUNTIME_MONITOR 1
#endif

#include <cstdint>

#if RUNTIME_MONITOR

#include <Arduino.h>

namespace rtmon {

// -------------------------------------------------------------------------
// Ring buffer size for frame time history
// -------------------------------------------------------------------------
inline constexpr uint8_t RING_SIZE = 120;

// -------------------------------------------------------------------------
// Blocking threshold — any phase exceeding this is flagged
// -------------------------------------------------------------------------
inline constexpr uint32_t BLOCKING_THRESHOLD_US = 4000;  // 4 ms

// -------------------------------------------------------------------------
// Zone identifiers for redraw tracking
// -------------------------------------------------------------------------
enum class Zone : uint8_t {
    TOP_BAR   = 0,
    OBSTACLE  = 1,
    CAR       = 2,
    SPEED     = 3,
    PEDAL     = 4,
    GEAR      = 5,
    COUNT     = 6   // sentinel — number of zones
};

// -------------------------------------------------------------------------
// Aggregated statistics snapshot (returned by getStats())
// -------------------------------------------------------------------------
struct RuntimeStats {
    // Frame timing (microseconds)
    uint32_t frameTimeUs;       // last frame time
    uint32_t maxFrameUs;        // max since reset
    uint32_t minFrameUs;        // min since reset
    uint32_t avgFrameUs;        // moving average over last RING_SIZE frames
    uint16_t fps;               // frames per second (computed)

    // Redraw counts (per second)
    uint16_t zoneRedraws[static_cast<uint8_t>(Zone::COUNT)];
    uint16_t fullRedraws;       // unintentional full screen redraws

    // Phase timing — max durations (microseconds)
    uint32_t canMaxUs;          // max CAN receive processing time
    uint32_t uiUpdateMaxUs;     // max UI update time
    uint32_t renderMaxUs;       // max render (draw) time

    // Blocking flags
    bool     canBlocking;       // CAN phase exceeded threshold
    bool     uiBlocking;        // UI update exceeded threshold
    bool     renderBlocking;    // Render phase exceeded threshold
};

// -------------------------------------------------------------------------
// Runtime Monitor — singleton-style static class
// -------------------------------------------------------------------------
class RuntimeMonitor {
public:
    // ---- Frame timing ----

    /// Call at the start of each draw() frame
    static void frameBegin();

    /// Call at the end of each draw() frame
    static void frameEnd();

    // ---- Phase timing (CAN, UI update, render) ----

    /// Call before/after CAN receive processing
    static void canBegin();
    static void canEnd();

    /// Call before/after UI update (screen.update())
    static void uiBegin();
    static void uiEnd();

    /// Call before/after rendering (screen.draw())
    static void renderBegin();
    static void renderEnd();

    // ---- Zone redraw tracking ----

    /// Record that a specific zone was redrawn this frame
    static void zoneRedraw(Zone zone);

    /// Record that a full screen redraw occurred
    static void fullRedraw();

    // ---- Stats retrieval ----

    /// Get current aggregated statistics (non-blocking, ~5 µs)
    static RuntimeStats getStats();

    /// Reset all counters (call on screen transitions)
    static void reset();

    /// Print stats to Serial in compact format
    /// Format: [RT] fps=.. avg=.. max=.. redraw=.. full=.. can=..
    static void logToSerial();

private:
    // Ring buffer for frame times
    static uint32_t  ringBuf_[RING_SIZE];
    static uint8_t   ringIdx_;
    static uint8_t   ringCount_;

    // Frame timing state
    static uint32_t  frameStartUs_;
    static uint32_t  lastFrameUs_;
    static uint32_t  maxFrameUs_;
    static uint32_t  minFrameUs_;

    // Phase timing state
    static uint32_t  canStartUs_;
    static uint32_t  canMaxUs_;
    static bool      canBlocking_;

    static uint32_t  uiStartUs_;
    static uint32_t  uiMaxUs_;
    static bool      uiBlocking_;

    static uint32_t  renderStartUs_;
    static uint32_t  renderMaxUs_;
    static bool      renderBlocking_;

    // Zone redraw counters (accumulated between getStats() calls)
    static uint16_t  zoneCounts_[static_cast<uint8_t>(Zone::COUNT)];
    static uint16_t  fullRedrawCount_;

    // FPS calculation
    static uint32_t  fpsCounterStart_;
    static uint16_t  fpsFrameCount_;
    static uint16_t  fpsCurrent_;
};

} // namespace rtmon

// -------------------------------------------------------------------------
// Convenience macros — compile to nothing when RUNTIME_MONITOR is 0
// -------------------------------------------------------------------------
#define RTMON_FRAME_BEGIN()      rtmon::RuntimeMonitor::frameBegin()
#define RTMON_FRAME_END()        rtmon::RuntimeMonitor::frameEnd()
#define RTMON_CAN_BEGIN()        rtmon::RuntimeMonitor::canBegin()
#define RTMON_CAN_END()          rtmon::RuntimeMonitor::canEnd()
#define RTMON_UI_BEGIN()         rtmon::RuntimeMonitor::uiBegin()
#define RTMON_UI_END()           rtmon::RuntimeMonitor::uiEnd()
#define RTMON_RENDER_BEGIN()     rtmon::RuntimeMonitor::renderBegin()
#define RTMON_RENDER_END()       rtmon::RuntimeMonitor::renderEnd()
#define RTMON_ZONE_REDRAW(z)     rtmon::RuntimeMonitor::zoneRedraw(z)
#define RTMON_FULL_REDRAW()      rtmon::RuntimeMonitor::fullRedraw()
#define RTMON_LOG()              rtmon::RuntimeMonitor::logToSerial()
#define RTMON_RESET()            rtmon::RuntimeMonitor::reset()

#else  // RUNTIME_MONITOR == 0

// -------------------------------------------------------------------------
// All macros compile to nothing — zero overhead
// -------------------------------------------------------------------------
#define RTMON_FRAME_BEGIN()      ((void)0)
#define RTMON_FRAME_END()        ((void)0)
#define RTMON_CAN_BEGIN()        ((void)0)
#define RTMON_CAN_END()          ((void)0)
#define RTMON_UI_BEGIN()         ((void)0)
#define RTMON_UI_END()           ((void)0)
#define RTMON_RENDER_BEGIN()     ((void)0)
#define RTMON_RENDER_END()       ((void)0)
#define RTMON_ZONE_REDRAW(z)     ((void)0)
#define RTMON_FULL_REDRAW()      ((void)0)
#define RTMON_LOG()              ((void)0)
#define RTMON_RESET()            ((void)0)

#endif // RUNTIME_MONITOR

#endif // RUNTIME_MONITOR_H
