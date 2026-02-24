// =============================================================================
// ESP32-S3 HMI — Light Control Buttons Implementation
//
// Two buttons in the top bar:
//   Front (LED_FRONT_X): headlight / low-beam icon
//     ON  = yellow background, white headlight circle + 3 angled rays →
//     OFF = dark background, gray headlight circle + gray rays →
//
//   Rear (LED_REAR_X): brake / tail-light icon
//     ON  = red background, 3 bright white horizontal bars ═══
//     OFF = dark background, gray 3-bar pattern
// =============================================================================

#include "led_toggle.h"
#include "render_trace.h"

namespace ui {

// -------------------------------------------------------------------------
// drawStatic — draw both buttons in OFF state at screen entry
// -------------------------------------------------------------------------
void LedToggle::drawStatic(TFT_eSPI& tft) {
    drawFrontButton(tft, false);
    drawRearButton(tft, false);
}

// -------------------------------------------------------------------------
// draw — update only changed buttons
// -------------------------------------------------------------------------
void LedToggle::draw(TFT_eSPI& tft,
                     bool curFront, bool prevFront,
                     bool curRear,  bool prevRear) {
    if (curFront != prevFront) drawFrontButton(tft, curFront);
    if (curRear  != prevRear)  drawRearButton(tft, curRear);
}

// -------------------------------------------------------------------------
// hitTestFront — touch hit test for front light button
// -------------------------------------------------------------------------
bool LedToggle::hitTestFront(int16_t touchX, int16_t touchY) {
    return (touchX >= LED_FRONT_X &&
            touchX <= LED_FRONT_X + LED_FRONT_W &&
            touchY >= LED_FRONT_Y &&
            touchY <= LED_FRONT_Y + LED_FRONT_H);
}

// -------------------------------------------------------------------------
// hitTestRear — touch hit test for rear light button
// -------------------------------------------------------------------------
bool LedToggle::hitTestRear(int16_t touchX, int16_t touchY) {
    return (touchX >= LED_REAR_X &&
            touchX <= LED_REAR_X + LED_REAR_W &&
            touchY >= LED_REAR_Y &&
            touchY <= LED_REAR_Y + LED_REAR_H);
}

// -------------------------------------------------------------------------
// drawFrontButton — headlight / low-beam icon
//   Layout inside LED_FRONT_W×LED_FRONT_H:
//     Small filled circle on right = headlight housing
//     3 angled lines going left    = low-beam pattern
// -------------------------------------------------------------------------
void LedToggle::drawFrontButton(TFT_eSPI& tft, bool active) {
    uint16_t bgCol   = active ? COL_YELLOW   : COL_BG;
    uint16_t bordCol = active ? COL_WHITE    : COL_GRAY;
    uint16_t rayCol  = active ? COL_WHITE    : COL_DARK_GRAY;
    uint16_t houseCol = active ? COL_HEADLIGHT  : COL_GRAY;  // warm yellow when on, gray when off

    // Background + border
    tft.fillRect(LED_FRONT_X, LED_FRONT_Y, LED_FRONT_W, LED_FRONT_H, bgCol);
    RTRACE_FILL_RECT(LED_FRONT_X, LED_FRONT_Y, LED_FRONT_W, LED_FRONT_H, bgCol);
    tft.drawRect(LED_FRONT_X, LED_FRONT_Y, LED_FRONT_W, LED_FRONT_H, bordCol);
    RTRACE_DRAW_RECT(LED_FRONT_X, LED_FRONT_Y, LED_FRONT_W, LED_FRONT_H, bordCol);

    // Headlight housing circle (right side of button)
    int16_t cx = LED_FRONT_X + LED_FRONT_W - 10;
    int16_t cy = LED_FRONT_Y + LED_FRONT_H / 2;
    tft.fillCircle(cx, cy, 5, houseCol);

    // Low-beam rays: 3 lines going left at angles (-15°, 0°, +15°)
    // ray 1: straight left
    tft.drawLine(cx - 5, cy, LED_FRONT_X + 4, cy, rayCol);
    // ray 2: upper-left (~-15°)
    tft.drawLine(cx - 5, cy - 1, LED_FRONT_X + 4, cy - 4, rayCol);
    // ray 3: lower-left (~+15°)
    tft.drawLine(cx - 5, cy + 1, LED_FRONT_X + 4, cy + 4, rayCol);

    // Label "F" at bottom-right corner
    tft.setTextColor(active ? COL_BLACK : COL_GRAY, bgCol);
    tft.setTextSize(1);
    tft.setTextDatum(BR_DATUM);
    tft.drawString("F", LED_FRONT_X + LED_FRONT_W - 2, LED_FRONT_Y + LED_FRONT_H - 1);
    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// drawRearButton — brake / tail-light icon
//   Layout inside LED_REAR_W×LED_REAR_H:
//     3 horizontal bright bars stacked = tail/brake light clusters
// -------------------------------------------------------------------------
void LedToggle::drawRearButton(TFT_eSPI& tft, bool active) {
    uint16_t bgCol   = active ? COL_RED     : COL_BG;
    uint16_t bordCol = active ? COL_WHITE   : COL_GRAY;
    uint16_t barCol  = active ? COL_WHITE   : COL_DARK_GRAY;

    // Background + border
    tft.fillRect(LED_REAR_X, LED_REAR_Y, LED_REAR_W, LED_REAR_H, bgCol);
    RTRACE_FILL_RECT(LED_REAR_X, LED_REAR_Y, LED_REAR_W, LED_REAR_H, bgCol);
    tft.drawRect(LED_REAR_X, LED_REAR_Y, LED_REAR_W, LED_REAR_H, bordCol);
    RTRACE_DRAW_RECT(LED_REAR_X, LED_REAR_Y, LED_REAR_W, LED_REAR_H, bordCol);

    // 3 horizontal bars representing rear light clusters
    int16_t barX = LED_REAR_X + 5;
    int16_t barW = LED_REAR_W - 10;
    int16_t barH = 4;
    int16_t y1 = LED_REAR_Y + 5;
    int16_t y2 = LED_REAR_Y + LED_REAR_H / 2 - 2;
    int16_t y3 = LED_REAR_Y + LED_REAR_H - 9;

    tft.fillRect(barX, y1, barW, barH, barCol);
    tft.fillRect(barX, y2, barW, barH, barCol);
    tft.fillRect(barX, y3, barW, barH, barCol);

    // Label "R" at bottom-right corner
    tft.setTextColor(active ? COL_WHITE : COL_GRAY, bgCol);
    tft.setTextSize(1);
    tft.setTextDatum(BR_DATUM);
    tft.drawString("R", LED_REAR_X + LED_REAR_W - 2, LED_REAR_Y + LED_REAR_H - 1);
    tft.setTextDatum(TL_DATUM);
}

} // namespace ui
