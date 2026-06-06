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
// redraw — force a full repaint of both buttons (overlay restore path)
// -------------------------------------------------------------------------
void LedToggle::redraw(TFT_eSPI& tft, bool front, bool rear) {
    drawFrontButton(tft, front);
    drawRearButton(tft, rear);
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
    uint16_t bgCol    = active ? COL_YELLOW    : COL_GEAR_OFF;
    uint16_t bordCol  = active ? COL_WHITE      : COL_DIAL_RING;
    uint16_t rayCol   = active ? COL_BLACK      : COL_DARK_GRAY;
    uint16_t houseCol = active ? COL_BLACK      : COL_GRAY;

    // Rounded button face (clear the cell first to avoid stale corners).
    tft.fillRect(LED_FRONT_X, LED_FRONT_Y, LED_FRONT_W, LED_FRONT_H, COL_BG);
    RTRACE_FILL_RECT(LED_FRONT_X, LED_FRONT_Y, LED_FRONT_W, LED_FRONT_H, COL_BG);
    tft.fillRoundRect(LED_FRONT_X, LED_FRONT_Y, LED_FRONT_W, LED_FRONT_H, 5, bgCol);
    RTRACE_FILL_RECT(LED_FRONT_X, LED_FRONT_Y, LED_FRONT_W, LED_FRONT_H, bgCol);
    tft.drawRoundRect(LED_FRONT_X, LED_FRONT_Y, LED_FRONT_W, LED_FRONT_H, 5, bordCol);
    RTRACE_DRAW_RECT(LED_FRONT_X, LED_FRONT_Y, LED_FRONT_W, LED_FRONT_H, bordCol);

    // Headlight = filled half-disc (the classic "D" lamp shape) on the left,
    // projecting three beams of increasing length to the right.  Reads as a
    // headlight without any label.
    int16_t lx = LED_FRONT_X + 12;
    int16_t cy = LED_FRONT_Y + LED_FRONT_H / 2;
    tft.fillCircle(lx, cy, 7, houseCol);          // filled disc (matches trace)
    tft.fillRect(lx - 8, cy - 7, 8, 15, bgCol);   // flatten the left side → "D"
    tft.drawLine(lx, cy - 7, lx, cy + 7, houseCol);
    RTRACE_FILL_CIRCLE(lx, cy, 7, houseCol);

    // Three projecting beams (short, medium, long).
    int16_t bx = lx + 9;
    tft.drawLine(bx, cy - 4, bx + 6,  cy - 4, rayCol);
    tft.drawLine(bx, cy,     bx + 10, cy,     rayCol);
    tft.drawLine(bx, cy + 4, bx + 6,  cy + 4, rayCol);
    RTRACE_LINE(bx, cy, bx + 10, cy, rayCol);
}

// -------------------------------------------------------------------------
// drawRearButton — brake / tail-light icon
//   Rounded lamp cluster: a red tail-lamp block with bright horizontal
//   segments; glows brightly when active.
// -------------------------------------------------------------------------
void LedToggle::drawRearButton(TFT_eSPI& tft, bool active) {
    uint16_t bgCol   = active ? COL_RED       : COL_GEAR_OFF;
    uint16_t bordCol = active ? COL_WHITE      : COL_DIAL_RING;
    uint16_t barCol  = active ? COL_WHITE      : COL_DARK_GRAY;

    // Rounded button face.
    tft.fillRect(LED_REAR_X, LED_REAR_Y, LED_REAR_W, LED_REAR_H, COL_BG);
    RTRACE_FILL_RECT(LED_REAR_X, LED_REAR_Y, LED_REAR_W, LED_REAR_H, COL_BG);
    tft.fillRoundRect(LED_REAR_X, LED_REAR_Y, LED_REAR_W, LED_REAR_H, 5, bgCol);
    RTRACE_FILL_RECT(LED_REAR_X, LED_REAR_Y, LED_REAR_W, LED_REAR_H, bgCol);
    tft.drawRoundRect(LED_REAR_X, LED_REAR_Y, LED_REAR_W, LED_REAR_H, 5, bordCol);
    RTRACE_DRAW_RECT(LED_REAR_X, LED_REAR_Y, LED_REAR_W, LED_REAR_H, bordCol);

    // Tail-lamp cluster: a rounded lamp housing with three bright bars — the
    // universal "rear lights" read.
    int16_t lampX = LED_REAR_X + 6;
    int16_t lampW = LED_REAR_W - 12;
    int16_t barH  = 4;
    int16_t y1 = LED_REAR_Y + 6;
    int16_t y2 = LED_REAR_Y + LED_REAR_H / 2 - 2;
    int16_t y3 = LED_REAR_Y + LED_REAR_H - 10;
    tft.fillRoundRect(lampX, y1, lampW, barH, 2, barCol);
    tft.fillRoundRect(lampX, y2, lampW, barH, 2, barCol);
    tft.fillRoundRect(lampX, y3, lampW, barH, 2, barCol);
    RTRACE_FILL_RECT(lampX, y1, lampW, barH, barCol);
    RTRACE_FILL_RECT(lampX, y2, lampW, barH, barCol);
    RTRACE_FILL_RECT(lampX, y3, lampW, barH, barCol);
}

} // namespace ui
