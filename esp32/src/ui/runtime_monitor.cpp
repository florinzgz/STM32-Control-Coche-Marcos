// =============================================================================
// ESP32-S3 HMI — Runtime Monitor Implementation
//
// All static storage is BSS-initialized (zero cost at startup).
// Ring buffer stores last 120 frame times for moving average.
// FPS computed from frame counter over 1-second window.
// Phase timing uses micros() for µs-precision measurement.
//
// Total overhead: ~20–30 µs per frame (two micros() calls + array write).
// =============================================================================

#include "runtime_monitor.h"

#if RUNTIME_MONITOR

#include <cstdio>
#include <cstring>

namespace rtmon {

// -------------------------------------------------------------------------
// Static member definitions (BSS-initialized to zero)
// -------------------------------------------------------------------------
uint32_t  RuntimeMonitor::ringBuf_[RING_SIZE]  = {};
uint8_t   RuntimeMonitor::ringIdx_             = 0;
uint8_t   RuntimeMonitor::ringCount_           = 0;

uint32_t  RuntimeMonitor::frameStartUs_        = 0;
uint32_t  RuntimeMonitor::lastFrameUs_         = 0;
uint32_t  RuntimeMonitor::maxFrameUs_          = 0;
uint32_t  RuntimeMonitor::minFrameUs_          = UINT32_MAX;

uint32_t  RuntimeMonitor::canStartUs_          = 0;
uint32_t  RuntimeMonitor::canMaxUs_            = 0;
bool      RuntimeMonitor::canBlocking_         = false;

uint32_t  RuntimeMonitor::uiStartUs_           = 0;
uint32_t  RuntimeMonitor::uiMaxUs_             = 0;
bool      RuntimeMonitor::uiBlocking_          = false;

uint32_t  RuntimeMonitor::renderStartUs_       = 0;
uint32_t  RuntimeMonitor::renderMaxUs_         = 0;
bool      RuntimeMonitor::renderBlocking_      = false;

uint16_t  RuntimeMonitor::zoneCounts_[static_cast<uint8_t>(Zone::COUNT)] = {};
uint16_t  RuntimeMonitor::fullRedrawCount_     = 0;

uint32_t  RuntimeMonitor::fpsCounterStart_     = 0;
uint16_t  RuntimeMonitor::fpsFrameCount_       = 0;
uint16_t  RuntimeMonitor::fpsCurrent_          = 0;

// -------------------------------------------------------------------------
// Frame timing
// -------------------------------------------------------------------------

void RuntimeMonitor::frameBegin() {
    frameStartUs_ = micros();
}

void RuntimeMonitor::frameEnd() {
    uint32_t elapsed = micros() - frameStartUs_;
    lastFrameUs_ = elapsed;

    // Update min/max
    if (elapsed > maxFrameUs_) maxFrameUs_ = elapsed;
    if (elapsed < minFrameUs_) minFrameUs_ = elapsed;

    // Store in ring buffer
    ringBuf_[ringIdx_] = elapsed;
    ringIdx_ = (ringIdx_ + 1) % RING_SIZE;
    if (ringCount_ < RING_SIZE) ringCount_++;

    // FPS counter
    fpsFrameCount_++;
    uint32_t now = millis();
    if (fpsCounterStart_ == 0) fpsCounterStart_ = now;
    uint32_t fpsElapsed = now - fpsCounterStart_;
    if (fpsElapsed >= 1000) {
        fpsCurrent_ = static_cast<uint16_t>(
            (static_cast<uint32_t>(fpsFrameCount_) * 1000) / fpsElapsed);
        fpsFrameCount_ = 0;
        fpsCounterStart_ = now;
    }
}

// -------------------------------------------------------------------------
// Phase timing — CAN receive
// -------------------------------------------------------------------------

void RuntimeMonitor::canBegin() {
    canStartUs_ = micros();
}

void RuntimeMonitor::canEnd() {
    uint32_t elapsed = micros() - canStartUs_;
    if (elapsed > canMaxUs_) canMaxUs_ = elapsed;
    if (elapsed > BLOCKING_THRESHOLD_US) canBlocking_ = true;
}

// -------------------------------------------------------------------------
// Phase timing — UI update
// -------------------------------------------------------------------------

void RuntimeMonitor::uiBegin() {
    uiStartUs_ = micros();
}

void RuntimeMonitor::uiEnd() {
    uint32_t elapsed = micros() - uiStartUs_;
    if (elapsed > uiMaxUs_) uiMaxUs_ = elapsed;
    if (elapsed > BLOCKING_THRESHOLD_US) uiBlocking_ = true;
}

// -------------------------------------------------------------------------
// Phase timing — Render (draw)
// -------------------------------------------------------------------------

void RuntimeMonitor::renderBegin() {
    renderStartUs_ = micros();
}

void RuntimeMonitor::renderEnd() {
    uint32_t elapsed = micros() - renderStartUs_;
    if (elapsed > renderMaxUs_) renderMaxUs_ = elapsed;
    if (elapsed > BLOCKING_THRESHOLD_US) renderBlocking_ = true;
}

// -------------------------------------------------------------------------
// Zone redraw tracking
// -------------------------------------------------------------------------

void RuntimeMonitor::zoneRedraw(Zone zone) {
    uint8_t idx = static_cast<uint8_t>(zone);
    if (idx < static_cast<uint8_t>(Zone::COUNT)) {
        zoneCounts_[idx]++;
    }
}

void RuntimeMonitor::fullRedraw() {
    fullRedrawCount_++;
}

// -------------------------------------------------------------------------
// Stats retrieval — computes moving average from ring buffer
// -------------------------------------------------------------------------

RuntimeStats RuntimeMonitor::getStats() {
    RuntimeStats s;
    s.frameTimeUs = lastFrameUs_;
    s.maxFrameUs  = maxFrameUs_;
    s.minFrameUs  = (minFrameUs_ == UINT32_MAX) ? 0 : minFrameUs_;
    s.fps         = fpsCurrent_;

    // Moving average over ring buffer
    if (ringCount_ > 0) {
        uint32_t sum = 0;
        for (uint8_t i = 0; i < ringCount_; ++i) {
            sum += ringBuf_[i];
        }
        s.avgFrameUs = sum / ringCount_;
    } else {
        s.avgFrameUs = 0;
    }

    // Zone redraws
    memcpy(s.zoneRedraws, zoneCounts_, sizeof(s.zoneRedraws));
    s.fullRedraws = fullRedrawCount_;

    // Phase timing
    s.canMaxUs      = canMaxUs_;
    s.uiUpdateMaxUs = uiMaxUs_;
    s.renderMaxUs   = renderMaxUs_;

    // Blocking flags
    s.canBlocking    = canBlocking_;
    s.uiBlocking     = uiBlocking_;
    s.renderBlocking = renderBlocking_;

    return s;
}

// -------------------------------------------------------------------------
// Reset all counters
// -------------------------------------------------------------------------

void RuntimeMonitor::reset() {
    memset(ringBuf_, 0, sizeof(ringBuf_));
    ringIdx_    = 0;
    ringCount_  = 0;

    lastFrameUs_ = 0;
    maxFrameUs_  = 0;
    minFrameUs_  = UINT32_MAX;

    canMaxUs_      = 0;
    canBlocking_   = false;
    uiMaxUs_       = 0;
    uiBlocking_    = false;
    renderMaxUs_   = 0;
    renderBlocking_ = false;

    memset(zoneCounts_, 0, sizeof(zoneCounts_));
    fullRedrawCount_ = 0;

    fpsCounterStart_ = 0;
    fpsFrameCount_   = 0;
    fpsCurrent_      = 0;
}

// -------------------------------------------------------------------------
// Serial logging — compact single-line format
// -------------------------------------------------------------------------

void RuntimeMonitor::logToSerial() {
    RuntimeStats s = getStats();

    // Count total zone redraws
    uint16_t totalRedraws = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(Zone::COUNT); ++i) {
        totalRedraws += s.zoneRedraws[i];
    }

    char buf[128];
    snprintf(buf, sizeof(buf),
        "[RT] fps=%u avg=%lu max=%lu min=%lu redraw=%u full=%u can=%lu",
        s.fps,
        (unsigned long)s.avgFrameUs,
        (unsigned long)s.maxFrameUs,
        (unsigned long)s.minFrameUs,
        totalRedraws,
        s.fullRedraws,
        (unsigned long)s.canMaxUs);

    Serial.println(buf);

    if (s.canBlocking || s.uiBlocking || s.renderBlocking) {
        char warnBuf[96];
        snprintf(warnBuf, sizeof(warnBuf),
            "[RT] WARN blocking: can=%s ui=%s render=%s",
            s.canBlocking    ? "YES" : "no",
            s.uiBlocking     ? "YES" : "no",
            s.renderBlocking ? "YES" : "no");
        Serial.println(warnBuf);
    }
}

} // namespace rtmon

#endif // RUNTIME_MONITOR
