// =============================================================================
// ESP32-S3 HMI — Car Renderer Implementation
//
// Draws a realistic top-down vehicle silhouette inspired by automotive
// dashboard UIs.  The body is tapered (narrower at front/rear bumpers,
// wider at the cabin), with a visible windshield, rear window, side
// mirrors, headlights, tail-lights, and a driveshaft+differential
// representation connecting front and rear axles.
//
// All geometry uses only TFT_eSPI primitives available on every board:
//   drawLine, drawRect, fillRect, drawCircle, fillCircle, fillTriangle
// No sprites, no heap, no floating-point in the static path.
// =============================================================================

#include "car_renderer.h"
#include "render_trace.h"
#include <cstdio>
#include <cmath>

namespace ui {

// =========================================================================
// Body geometry helpers — all derived from the existing CAR_BODY_* layout
// constants in ui_common.h so the car stays centred in the same area.
// =========================================================================

// Centre X of the car body
static constexpr int16_t CX = CAR_BODY_X + CAR_BODY_W / 2;   // 240

// Body vertical anchors
static constexpr int16_t BODY_TOP    = CAR_BODY_Y;            // 105
static constexpr int16_t BODY_BOT    = CAR_BODY_Y + CAR_BODY_H; // 215

// Half-widths at key sections (the body tapers towards the bumpers)
static constexpr int16_t HW_BUMPER   = 30;   // narrow at bumpers
static constexpr int16_t HW_FENDER   = 44;   // widest at wheel-arches
static constexpr int16_t HW_CABIN    = 42;   // cabin width

// Vertical Y positions for body sections (top to bottom = front to rear)
static constexpr int16_t Y_FRONT_BUMPER = BODY_TOP;
static constexpr int16_t Y_HOOD_START   = BODY_TOP + 8;
static constexpr int16_t Y_HOOD_END     = BODY_TOP + 20;   // front fender line
static constexpr int16_t Y_WINDSHIELD   = BODY_TOP + 26;
static constexpr int16_t Y_CABIN_FRONT  = BODY_TOP + 34;
static constexpr int16_t Y_CABIN_REAR   = BODY_BOT - 34;
static constexpr int16_t Y_REAR_WINDOW  = BODY_BOT - 26;
static constexpr int16_t Y_TRUNK_START  = BODY_BOT - 20;
static constexpr int16_t Y_TRUNK_END    = BODY_BOT - 8;
static constexpr int16_t Y_REAR_BUMPER  = BODY_BOT;

// Headlight / tail-light dimensions
static constexpr int16_t HL_W = 10;
static constexpr int16_t HL_H = 4;

// -------------------------------------------------------------------------
// drawBodyOutline — tapered top-down car silhouette using line segments
//
// The outline is drawn as a series of connected straight line segments,
// mirrored on the left and right sides.  This produces a shape similar
// to the reference image: narrow front bumper → wider hood → fenders →
// cabin → narrower trunk → narrow rear bumper.
// -------------------------------------------------------------------------
static void drawBodyOutline(TFT_eSPI& tft, uint16_t col) {
    // Left-side outline points (from front bumper going clockwise)
    // Each point is {x, y} where x is offset from centre.
    struct Pt { int16_t x; int16_t y; };
    static constexpr Pt left[] = {
        { -HW_BUMPER,  Y_FRONT_BUMPER },   // 0  front bumper left
        { -HW_BUMPER,  Y_HOOD_START   },   // 1  bumper→hood transition
        { -HW_FENDER,  Y_HOOD_END     },   // 2  front fender (widest)
        { -HW_FENDER,  Y_WINDSHIELD   },   // 3  A-pillar top
        { -HW_CABIN,   Y_CABIN_FRONT  },   // 4  cabin front
        { -HW_CABIN,   Y_CABIN_REAR   },   // 5  cabin rear
        { -HW_FENDER,  Y_REAR_WINDOW  },   // 6  C-pillar
        { -HW_FENDER,  Y_TRUNK_START  },   // 7  rear fender (widest)
        { -HW_BUMPER,  Y_TRUNK_END    },   // 8  trunk→bumper transition
        { -HW_BUMPER,  Y_REAR_BUMPER  },   // 9  rear bumper left
    };
    static constexpr uint8_t N = sizeof(left) / sizeof(left[0]);

    // Draw left side outline
    for (uint8_t i = 0; i + 1 < N; ++i) {
        int16_t x0 = CX + left[i].x,     y0 = left[i].y;
        int16_t x1 = CX + left[i + 1].x, y1 = left[i + 1].y;
        tft.drawLine(x0, y0, x1, y1, col);
        RTRACE_LINE(x0, y0, x1, y1, col);
        // Double-width outline
        tft.drawLine(x0 + 1, y0, x1 + 1, y1, col);
    }

    // Draw right side (mirror: negate x offset)
    for (uint8_t i = 0; i + 1 < N; ++i) {
        int16_t x0 = CX - left[i].x,     y0 = left[i].y;
        int16_t x1 = CX - left[i + 1].x, y1 = left[i + 1].y;
        tft.drawLine(x0, y0, x1, y1, col);
        RTRACE_LINE(x0, y0, x1, y1, col);
        tft.drawLine(x0 - 1, y0, x1 - 1, y1, col);
    }

    // Front bumper (horizontal top line)
    tft.drawLine(CX - HW_BUMPER, Y_FRONT_BUMPER,
                 CX + HW_BUMPER, Y_FRONT_BUMPER, col);
    RTRACE_LINE(CX - HW_BUMPER, Y_FRONT_BUMPER,
                CX + HW_BUMPER, Y_FRONT_BUMPER, col);
    tft.drawLine(CX - HW_BUMPER, Y_FRONT_BUMPER + 1,
                 CX + HW_BUMPER, Y_FRONT_BUMPER + 1, col);

    // Rear bumper (horizontal bottom line)
    tft.drawLine(CX - HW_BUMPER, Y_REAR_BUMPER,
                 CX + HW_BUMPER, Y_REAR_BUMPER, col);
    RTRACE_LINE(CX - HW_BUMPER, Y_REAR_BUMPER,
                CX + HW_BUMPER, Y_REAR_BUMPER, col);
    tft.drawLine(CX - HW_BUMPER, Y_REAR_BUMPER - 1,
                 CX + HW_BUMPER, Y_REAR_BUMPER - 1, col);
}

// -------------------------------------------------------------------------
// Static car body outline — drawn once per screen enter
// -------------------------------------------------------------------------
void CarRenderer::drawStatic(TFT_eSPI& tft) {

    // --- Car body silhouette (tapered outline) ---------------------------
    drawBodyOutline(tft, COL_CYAN);

    // --- Windshield (curved top edge of cabin glass) ---------------------
    {
        int16_t wL = CX - HW_CABIN + 4;
        int16_t wR = CX + HW_CABIN - 4;
        int16_t wY = Y_WINDSHIELD;
        // Slightly curved: 3 line segments forming a shallow arc
        int16_t midL = wL + (wR - wL) / 3;
        int16_t midR = wL + 2 * (wR - wL) / 3;
        tft.drawLine(wL, wY + 2, midL, wY, COL_GRAY);
        RTRACE_LINE(wL, wY + 2, midL, wY, COL_GRAY);
        tft.drawLine(midL, wY, midR, wY, COL_GRAY);
        RTRACE_LINE(midL, wY, midR, wY, COL_GRAY);
        tft.drawLine(midR, wY, wR, wY + 2, COL_GRAY);
        RTRACE_LINE(midR, wY, wR, wY + 2, COL_GRAY);
        // Second line for thickness
        tft.drawLine(wL, wY + 3, midL, wY + 1, COL_GRAY);
        tft.drawLine(midL, wY + 1, midR, wY + 1, COL_GRAY);
        tft.drawLine(midR, wY + 1, wR, wY + 3, COL_GRAY);
    }

    // --- Rear window (similar arc) ---------------------------------------
    {
        int16_t wL = CX - HW_CABIN + 4;
        int16_t wR = CX + HW_CABIN - 4;
        int16_t wY = Y_REAR_WINDOW;
        int16_t midL = wL + (wR - wL) / 3;
        int16_t midR = wL + 2 * (wR - wL) / 3;
        tft.drawLine(wL, wY - 2, midL, wY, COL_GRAY);
        RTRACE_LINE(wL, wY - 2, midL, wY, COL_GRAY);
        tft.drawLine(midL, wY, midR, wY, COL_GRAY);
        RTRACE_LINE(midL, wY, midR, wY, COL_GRAY);
        tft.drawLine(midR, wY, wR, wY - 2, COL_GRAY);
        RTRACE_LINE(midR, wY, wR, wY - 2, COL_GRAY);
        tft.drawLine(wL, wY - 1, midL, wY + 1, COL_GRAY);
        tft.drawLine(midL, wY + 1, midR, wY + 1, COL_GRAY);
        tft.drawLine(midR, wY + 1, wR, wY - 1, COL_GRAY);
    }

    // --- Headlights (front, small filled rectangles at bumper corners) ---
    {
        int16_t hlY = Y_FRONT_BUMPER + 2;
        // Left headlight
        tft.fillRect(CX - HW_BUMPER + 2, hlY, HL_W, HL_H, COL_HEADLIGHT);
        RTRACE_FILL_RECT(CX - HW_BUMPER + 2, hlY, HL_W, HL_H, COL_HEADLIGHT);
        // Right headlight
        tft.fillRect(CX + HW_BUMPER - HL_W - 2, hlY, HL_W, HL_H, COL_HEADLIGHT);
        RTRACE_FILL_RECT(CX + HW_BUMPER - HL_W - 2, hlY, HL_W, HL_H, COL_HEADLIGHT);
    }

    // --- Tail-lights (rear, red rectangles at rear bumper corners) -------
    {
        int16_t tlY = Y_REAR_BUMPER - HL_H - 2;
        tft.fillRect(CX - HW_BUMPER + 2, tlY, HL_W, HL_H, COL_RED);
        RTRACE_FILL_RECT(CX - HW_BUMPER + 2, tlY, HL_W, HL_H, COL_RED);
        tft.fillRect(CX + HW_BUMPER - HL_W - 2, tlY, HL_W, HL_H, COL_RED);
        RTRACE_FILL_RECT(CX + HW_BUMPER - HL_W - 2, tlY, HL_W, HL_H, COL_RED);
    }

    // --- Side mirrors (small triangles at cabin level) -------------------
    {
        int16_t mY = Y_CABIN_FRONT + 2;
        // Left mirror
        int16_t lx = CX - HW_FENDER - 2;
        tft.fillRect(lx - 4, mY, 5, 6, COL_CYAN);
        RTRACE_FILL_RECT(lx - 4, mY, 5, 6, COL_CYAN);
        // Right mirror
        int16_t rx = CX + HW_FENDER + 2;
        tft.fillRect(rx, mY, 5, 6, COL_CYAN);
        RTRACE_FILL_RECT(rx, mY, 5, 6, COL_CYAN);
    }

    // --- Driveshaft (vertical centre line from front axle to rear axle) --
    {
        int16_t frontAxleY = WHL_FL_Y + WHEEL_H / 2;
        int16_t rearAxleY  = WHL_RL_Y + WHEEL_H / 2;
        // Main shaft
        tft.drawLine(CX, frontAxleY, CX, rearAxleY, COL_ORANGE);
        RTRACE_LINE(CX, frontAxleY, CX, rearAxleY, COL_ORANGE);
        tft.drawLine(CX + 1, frontAxleY, CX + 1, rearAxleY, COL_ORANGE);
        // Centre differential box
        int16_t diffY = (frontAxleY + rearAxleY) / 2 - 4;
        tft.fillRect(CX - 4, diffY, 9, 9, COL_ORANGE);
        RTRACE_FILL_RECT(CX - 4, diffY, 9, 9, COL_ORANGE);
        tft.drawRect(CX - 4, diffY, 9, 9, COL_WHITE);
        RTRACE_DRAW_RECT(CX - 4, diffY, 9, 9, COL_WHITE);
    }

    // --- Front axle (connects left wheel → driveshaft → right wheel) -----
    {
        int16_t ay = WHL_FL_Y + WHEEL_H / 2;
        tft.drawLine(WHL_FL_X + WHEEL_W, ay, WHL_FR_X, ay, COL_ORANGE);
        RTRACE_LINE(WHL_FL_X + WHEEL_W, ay, WHL_FR_X, ay, COL_ORANGE);
        tft.drawLine(WHL_FL_X + WHEEL_W, ay + 1, WHL_FR_X, ay + 1, COL_ORANGE);
        // Front diff box
        tft.fillRect(CX - 3, ay - 3, 7, 7, COL_ORANGE);
        RTRACE_FILL_RECT(CX - 3, ay - 3, 7, 7, COL_ORANGE);
        tft.drawRect(CX - 3, ay - 3, 7, 7, COL_WHITE);
        RTRACE_DRAW_RECT(CX - 3, ay - 3, 7, 7, COL_WHITE);
    }

    // --- Rear axle -------------------------------------------------------
    {
        int16_t ay = WHL_RL_Y + WHEEL_H / 2;
        tft.drawLine(WHL_RL_X + WHEEL_W, ay, WHL_RR_X, ay, COL_ORANGE);
        RTRACE_LINE(WHL_RL_X + WHEEL_W, ay, WHL_RR_X, ay, COL_ORANGE);
        tft.drawLine(WHL_RL_X + WHEEL_W, ay + 1, WHL_RR_X, ay + 1, COL_ORANGE);
        // Rear diff box
        tft.fillRect(CX - 3, ay - 3, 7, 7, COL_ORANGE);
        RTRACE_FILL_RECT(CX - 3, ay - 3, 7, 7, COL_ORANGE);
        tft.drawRect(CX - 3, ay - 3, 7, 7, COL_WHITE);
        RTRACE_DRAW_RECT(CX - 3, ay - 3, 7, 7, COL_WHITE);
    }

    // --- Steering circle outline -----------------------------------------
    tft.drawCircle(STEER_CX, STEER_CY, STEER_RADIUS, COL_GRAY);
    RTRACE_CIRCLE(STEER_CX, STEER_CY, STEER_RADIUS, COL_GRAY);
    tft.drawCircle(STEER_CX, STEER_CY, STEER_RADIUS + 1, COL_GRAY);
    RTRACE_CIRCLE(STEER_CX, STEER_CY, STEER_RADIUS + 1, COL_GRAY);
}

// -------------------------------------------------------------------------
// Draw/update all 4 wheels + their torque/temp labels
//
// Wheel order: 0=FL (left), 1=FR (right), 2=RL (left), 3=RR (right)
// Labels are placed to the left for FL/RL, to the right for FR/RR,
// matching the reference image layout.
// -------------------------------------------------------------------------
void CarRenderer::drawWheels(TFT_eSPI& tft,
                             const vehicle::TractionData& traction,
                             const vehicle::TempMapData& tempMap,
                             const uint8_t prevTraction[4],
                             const int8_t prevTemp[4]) {
    static constexpr int16_t wx[4] = { WHL_FL_X, WHL_FR_X, WHL_RL_X, WHL_RR_X };
    static constexpr int16_t wy[4] = { WHL_FL_Y, WHL_FR_Y, WHL_RL_Y, WHL_RR_Y };
    // rightSide: FR(1) and RR(3) have labels on the right
    static constexpr bool rs[4] = { false, true, false, true };

    for (uint8_t i = 0; i < 4; ++i) {
        if (traction.scale[i] != prevTraction[i] ||
            tempMap.temps[i] != prevTemp[i]) {
            drawWheel(tft, wx[i], wy[i], traction.scale[i], tempMap.temps[i]);
            drawWheelLabel(tft, wx[i], wy[i],
                           traction.scale[i], tempMap.temps[i], rs[i]);
        }
    }
}

// -------------------------------------------------------------------------
// Draw a single wheel with tread-line detail
//
// Layout (WHEEL_W × WHEEL_H = 44 × 44):
//   Outer: cyan/white rounded outline
//   Inner: dark fill + 5 horizontal tread bars
//   Centre: small arrow showing rotation direction
//   Text: torque % and temperature drawn *outside* the wheel (beside it)
//         to keep the tyre graphic clean — matching the reference image.
// -------------------------------------------------------------------------
void CarRenderer::drawWheel(TFT_eSPI& tft,
                            int16_t x, int16_t y,
                            uint8_t torquePct, int8_t tempC) {
    uint16_t treadCol = torqueColor(torquePct);

    // Clear entire wheel + label area
    // Labels sit to the left or right of the wheel (see drawWheels)
    tft.fillRect(x, y, WHEEL_W, WHEEL_H, COL_BG);
    RTRACE_FILL_RECT(x, y, WHEEL_W, WHEEL_H, COL_BG);

    // Outer rounded tyre outline
    static constexpr int16_t R = 4;  // corner radius
    tft.drawRoundRect(x, y, WHEEL_W, WHEEL_H, R, COL_CYAN);
    tft.drawRoundRect(x + 1, y + 1, WHEEL_W - 2, WHEEL_H - 2, R - 1, COL_CYAN);
    RTRACE_DRAW_RECT(x, y, WHEEL_W, WHEEL_H, COL_CYAN);

    // Dark inner fill (tyre rubber)
    tft.fillRect(x + 3, y + 3, WHEEL_W - 6, WHEEL_H - 6, COL_DARK_GRAY);
    RTRACE_FILL_RECT(x + 3, y + 3, WHEEL_W - 6, WHEEL_H - 6, COL_DARK_GRAY);

    // Horizontal tread lines (5 bars across the tyre face)
    {
        static constexpr uint8_t NUM_TREADS = 5;
        int16_t innerH = WHEEL_H - 8;        // usable height inside outline
        int16_t spacing = innerH / (NUM_TREADS + 1);
        int16_t barH = 2;
        for (uint8_t t = 1; t <= NUM_TREADS; ++t) {
            int16_t ty = y + 4 + t * spacing - barH / 2;
            tft.fillRect(x + 5, ty, WHEEL_W - 10, barH, treadCol);
        }
        RTRACE_FILL_RECT(x + 5, y + 4, WHEEL_W - 10, innerH, treadCol);
    }

    // Direction arrow (small upward triangle in centre)
    {
        int16_t cx = x + WHEEL_W / 2;
        int16_t cy = y + WHEEL_H / 2;
        // Arrow pointing up (front of car)
        tft.fillTriangle(cx, cy - 5, cx - 4, cy + 3, cx + 4, cy + 3, COL_CYAN);
        RTRACE_FILL_RECT(cx - 4, cy - 5, 9, 9, COL_CYAN);  // trace approx
    }

}

// -------------------------------------------------------------------------
// Draw torque/temp label text next to a wheel
//
// Left-side wheels (FL, RL): label to the LEFT of the wheel
// Right-side wheels (FR, RR): label to the RIGHT of the wheel
// -------------------------------------------------------------------------
void CarRenderer::drawWheelLabel(TFT_eSPI& tft,
                                 int16_t wx, int16_t wy,
                                 uint8_t torquePct, int8_t tempC,
                                 bool rightSide) {
    uint16_t torqueCol = torqueColor(torquePct);
    char buf[FMT_BUF_SMALL];

    if (rightSide) {
        // Label to the right of the wheel
        int16_t lx = wx + WHEEL_W + 4;
        int16_t ly = wy + 4;
        // Clear label area
        tft.fillRect(lx, ly - 2, 46, 28, COL_BG);
        RTRACE_FILL_RECT(lx, ly - 2, 46, 28, COL_BG);
        // Torque %
        snprintf(buf, sizeof(buf), "%u%%", torquePct);
        tft.setTextColor(torqueCol, COL_BG);
        tft.setTextSize(2);
        tft.setTextDatum(TL_DATUM);
        tft.drawString(buf, lx, ly);
        RTRACE_TEXT(lx, ly, buf, torqueCol, COL_BG, 2, TL_DATUM);
        // Temperature
        snprintf(buf, sizeof(buf), "%d\xC2\xB0""C", tempC);
        tft.setTextColor(COL_CYAN, COL_BG);
        tft.setTextSize(1);
        tft.drawString(buf, lx + 2, ly + 18);
        RTRACE_TEXT(lx + 2, ly + 18, buf, COL_CYAN, COL_BG, 1, TL_DATUM);
    } else {
        // Label to the left of the wheel
        int16_t lx = wx - 50;
        int16_t ly = wy + 4;
        // Clear label area
        tft.fillRect(lx, ly - 2, 46, 28, COL_BG);
        RTRACE_FILL_RECT(lx, ly - 2, 46, 28, COL_BG);
        // Torque %
        snprintf(buf, sizeof(buf), "%u%%", torquePct);
        tft.setTextColor(torqueCol, COL_BG);
        tft.setTextSize(2);
        tft.setTextDatum(TR_DATUM);
        tft.drawString(buf, lx + 46, ly);
        RTRACE_TEXT(lx + 46, ly, buf, torqueCol, COL_BG, 2, TR_DATUM);
        // Temperature
        snprintf(buf, sizeof(buf), "%d\xC2\xB0""C", tempC);
        tft.setTextColor(COL_CYAN, COL_BG);
        tft.setTextSize(1);
        tft.drawString(buf, lx + 44, ly + 18);
        RTRACE_TEXT(lx + 44, ly + 18, buf, COL_CYAN, COL_BG, 1, TR_DATUM);
    }

    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}

// -------------------------------------------------------------------------
// Draw steering angle indicator
// -------------------------------------------------------------------------
void CarRenderer::drawSteering(TFT_eSPI& tft,
                               int16_t angleRaw,
                               int16_t prevAngleRaw) {
    if (angleRaw == prevAngleRaw) return;

    // Erase previous line
    drawSteerLine(tft, prevAngleRaw, COL_BG);

    // Draw new line
    drawSteerLine(tft, angleRaw, COL_CYAN);

    // Small center dot
    tft.fillCircle(STEER_CX, STEER_CY, 3, COL_WHITE);
    RTRACE_FILL_CIRCLE(STEER_CX, STEER_CY, 3, COL_WHITE);
}

// -------------------------------------------------------------------------
// Draw a rotation indicator line from center
// -------------------------------------------------------------------------
void CarRenderer::drawSteerLine(TFT_eSPI& tft,
                                int16_t angleRaw, uint16_t color) {
    // angleRaw is in 0.1° units, convert to radians
    // 0° = straight up, positive = clockwise (right turn)
    float angleDeg = static_cast<float>(angleRaw) * 0.1f;

    // Clamp to ±45° for display
    if (angleDeg > 45.0f) angleDeg = 45.0f;
    if (angleDeg < -45.0f) angleDeg = -45.0f;

    // Convert to radians, offset by -90° so 0° points up
    float angleRad = (angleDeg - 90.0f) * 3.14159265f / 180.0f;

    int16_t ex = STEER_CX + static_cast<int16_t>(cosf(angleRad) *
                 static_cast<float>(STEER_RADIUS - 2));
    int16_t ey = STEER_CY + static_cast<int16_t>(sinf(angleRad) *
                 static_cast<float>(STEER_RADIUS - 2));

    tft.drawLine(STEER_CX, STEER_CY, ex, ey, color);
    RTRACE_LINE(STEER_CX, STEER_CY, ex, ey, color);

    // Redraw circle outline to fix any erasure artifacts
    if (color == COL_BG) {
        tft.drawCircle(STEER_CX, STEER_CY, STEER_RADIUS, COL_GRAY);
        RTRACE_CIRCLE(STEER_CX, STEER_CY, STEER_RADIUS, COL_GRAY);
    }
}

} // namespace ui
