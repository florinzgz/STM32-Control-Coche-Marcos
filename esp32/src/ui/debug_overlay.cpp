// =============================================================================
// ESP32-S3 HMI — Debug Overlay Implementation
//
// Draws performance stats on a semi-transparent overlay box.
// Activated by holding touch for 3 seconds, dismissed by tapping again.
// No heap allocation, no String, no sprites.
// =============================================================================

#include "debug_overlay.h"

#if RUNTIME_MONITOR

#include "ui_common.h"
#include <cstdio>

namespace rtmon {

// -------------------------------------------------------------------------
// Static member definitions
// -------------------------------------------------------------------------
bool     DebugOverlay::visible_        = false;
uint32_t DebugOverlay::lastUpdateMs_   = 0;

// -------------------------------------------------------------------------
// Update — bookkeeping only.  Visibility is controlled explicitly from the
// Engineering menu (toggle()/setVisible()); the overlay deliberately does
// NOT react to the global long-press gesture, which is reserved for opening
// the PIN/Engineering screen.
// -------------------------------------------------------------------------
bool DebugOverlay::update(bool touchDown, uint32_t frameTimeMs) {
    (void)touchDown;
    (void)frameTimeMs;
    return visible_;
}

// -------------------------------------------------------------------------
// Explicit visibility control (Engineering menu)
// -------------------------------------------------------------------------
void DebugOverlay::setVisible(bool v) {
    visible_ = v;
    if (visible_) {
        lastUpdateMs_ = 0;  // Force immediate draw on next draw() call
    }
}

void DebugOverlay::toggle() {
    setVisible(!visible_);
}

// -------------------------------------------------------------------------
// Draw overlay — only when visible, throttled to REFRESH_MS
// -------------------------------------------------------------------------
void DebugOverlay::draw(TFT_eSPI& tft, uint32_t frameTimeMs) {
    if (!visible_) return;

    uint32_t now = frameTimeMs;
    if (lastUpdateMs_ != 0 && (now - lastUpdateMs_ < REFRESH_MS)) return;
    lastUpdateMs_ = now;

    RuntimeStats s = RuntimeMonitor::getStats();

    // Draw overlay background
    tft.fillRect(OVL_X, OVL_Y, OVL_W, OVL_H, ui::COL_BLACK);
    tft.drawRect(OVL_X, OVL_Y, OVL_W, OVL_H, ui::COL_CYAN);

    // Title
    tft.setTextColor(ui::COL_CYAN, ui::COL_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);

    int16_t x = OVL_X + MARGIN;
    int16_t y = OVL_Y + MARGIN;

    tft.drawString("=== DEBUG OVERLAY ===", x, y);
    y += LINE_H;

    // Stats lines
    char buf[48];

    snprintf(buf, sizeof(buf), "FPS:          %u", s.fps);
    tft.setTextColor(ui::COL_WHITE, ui::COL_BLACK);
    tft.drawString(buf, x, y);
    y += LINE_H;

    snprintf(buf, sizeof(buf), "AVG frame:    %lu us", (unsigned long)s.avgFrameUs);
    tft.drawString(buf, x, y);
    y += LINE_H;

    snprintf(buf, sizeof(buf), "MAX frame:    %lu us", (unsigned long)s.maxFrameUs);
    tft.setTextColor(s.maxFrameUs > BLOCKING_THRESHOLD_US ? ui::COL_RED : ui::COL_WHITE,
                     ui::COL_BLACK);
    tft.drawString(buf, x, y);
    y += LINE_H;

    snprintf(buf, sizeof(buf), "MIN frame:    %lu us", (unsigned long)s.minFrameUs);
    tft.setTextColor(ui::COL_WHITE, ui::COL_BLACK);
    tft.drawString(buf, x, y);
    y += LINE_H;

    // Dirty redraws per zone (total)
    uint16_t totalRedraws = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(Zone::COUNT); ++i) {
        totalRedraws += s.zoneRedraws[i];
    }
    snprintf(buf, sizeof(buf), "Dirty redraw: %u", totalRedraws);
    tft.drawString(buf, x, y);
    y += LINE_H;

    snprintf(buf, sizeof(buf), "Full redraws: %u", s.fullRedraws);
    tft.setTextColor(s.fullRedraws > FULL_REDRAW_WARN ? ui::COL_AMBER : ui::COL_WHITE,
                     ui::COL_BLACK);
    tft.drawString(buf, x, y);
    y += LINE_H;

    snprintf(buf, sizeof(buf), "CAN max:      %lu us", (unsigned long)s.canMaxUs);
    tft.setTextColor(s.canBlocking ? ui::COL_RED : ui::COL_WHITE,
                     ui::COL_BLACK);
    tft.drawString(buf, x, y);
    y += LINE_H;

    snprintf(buf, sizeof(buf), "Render max:   %lu us", (unsigned long)s.renderMaxUs);
    tft.setTextColor(s.renderBlocking ? ui::COL_RED : ui::COL_WHITE,
                     ui::COL_BLACK);
    tft.drawString(buf, x, y);
    y += LINE_H;

    snprintf(buf, sizeof(buf), "Loop max:     %lu us", (unsigned long)s.loopMaxUs);
    tft.setTextColor(s.loopBlocking ? ui::COL_RED : ui::COL_WHITE,
                     ui::COL_BLACK);
    tft.drawString(buf, x, y);

    // Reset text state
    tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}

// -------------------------------------------------------------------------
// Visibility query
// -------------------------------------------------------------------------
bool DebugOverlay::isVisible() {
    return visible_;
}

} // namespace rtmon

#endif // RUNTIME_MONITOR
