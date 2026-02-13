// =============================================================================
// ESP32-S3 HMI — Car Renderer Implementation
// =============================================================================

#include "car_renderer.h"
#include <cstdio>
#include <cmath>

namespace ui {

// -------------------------------------------------------------------------
// Static car body outline — drawn once per screen enter
// -------------------------------------------------------------------------
void CarRenderer::drawStatic(TFT_eSPI& tft) {
    // Car body rectangle
    tft.drawRect(CAR_BODY_X, CAR_BODY_Y, CAR_BODY_W, CAR_BODY_H, COL_WHITE);
    tft.drawRect(CAR_BODY_X + 1, CAR_BODY_Y + 1,
                 CAR_BODY_W - 2, CAR_BODY_H - 2, COL_WHITE);

    // Windshield (top of car body)
    int16_t wsY = CAR_BODY_Y + 15;
    tft.drawLine(CAR_BODY_X + 10, wsY,
                 CAR_BODY_X + CAR_BODY_W - 10, wsY, COL_GRAY);
    tft.drawLine(CAR_BODY_X + 10, wsY + 1,
                 CAR_BODY_X + CAR_BODY_W - 10, wsY + 1, COL_GRAY);

    // Rear window (bottom of car body)
    int16_t rwY = CAR_BODY_Y + CAR_BODY_H - 18;
    tft.drawLine(CAR_BODY_X + 10, rwY,
                 CAR_BODY_X + CAR_BODY_W - 10, rwY, COL_GRAY);
    tft.drawLine(CAR_BODY_X + 10, rwY + 1,
                 CAR_BODY_X + CAR_BODY_W - 10, rwY + 1, COL_GRAY);

    // Axle lines connecting wheels
    // Front axle
    tft.drawLine(WHL_FL_X + WHEEL_W, WHL_FL_Y + WHEEL_H / 2,
                 WHL_FR_X, WHL_FR_Y + WHEEL_H / 2, COL_DARK_GRAY);
    // Rear axle
    tft.drawLine(WHL_RL_X + WHEEL_W, WHL_RL_Y + WHEEL_H / 2,
                 WHL_RR_X, WHL_RR_Y + WHEEL_H / 2, COL_DARK_GRAY);

    // Steering circle outline
    tft.drawCircle(STEER_CX, STEER_CY, STEER_RADIUS, COL_GRAY);
    tft.drawCircle(STEER_CX, STEER_CY, STEER_RADIUS + 1, COL_GRAY);
}

// -------------------------------------------------------------------------
// Draw/update all 4 wheels
// -------------------------------------------------------------------------
void CarRenderer::drawWheels(TFT_eSPI& tft,
                             const vehicle::TractionData& traction,
                             const vehicle::TempMapData& tempMap,
                             const uint8_t prevTraction[4],
                             const int8_t prevTemp[4]) {
    static constexpr int16_t wx[4] = { WHL_FL_X, WHL_FR_X, WHL_RL_X, WHL_RR_X };
    static constexpr int16_t wy[4] = { WHL_FL_Y, WHL_FR_Y, WHL_RL_Y, WHL_RR_Y };

    for (uint8_t i = 0; i < 4; ++i) {
        if (traction.scale[i] != prevTraction[i] ||
            tempMap.temps[i] != prevTemp[i]) {
            drawWheel(tft, wx[i], wy[i], traction.scale[i], tempMap.temps[i]);
        }
    }
}

// -------------------------------------------------------------------------
// Draw a single wheel box
// -------------------------------------------------------------------------
void CarRenderer::drawWheel(TFT_eSPI& tft,
                            int16_t x, int16_t y,
                            uint8_t torquePct, int8_t tempC) {
    uint16_t fillCol = torqueColor(torquePct);

    // Clear wheel area
    tft.fillRect(x, y, WHEEL_W, WHEEL_H, COL_BG);

    // Fill with torque color (inner area)
    tft.fillRect(x + 2, y + 2, WHEEL_W - 4, WHEEL_H - 4, fillCol);

    // Draw outline
    tft.drawRect(x, y, WHEEL_W, WHEEL_H, COL_WHITE);

    // Torque percentage text (centered in wheel)
    char buf[FMT_BUF_SMALL];
    snprintf(buf, sizeof(buf), "%u%%", torquePct);
    tft.setTextColor(COL_BLACK, fillCol);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(buf, x + WHEEL_W / 2, y + WHEEL_H / 2 - 8);

    // Temperature text (below torque)
    snprintf(buf, sizeof(buf), "%dC", tempC);
    tft.drawString(buf, x + WHEEL_W / 2, y + WHEEL_H / 2 + 8);

    tft.setTextDatum(TL_DATUM);  // Reset datum
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

    // Redraw circle outline to fix any erasure artifacts
    if (color == COL_BG) {
        tft.drawCircle(STEER_CX, STEER_CY, STEER_RADIUS, COL_GRAY);
    }
}

} // namespace ui
