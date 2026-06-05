// =============================================================================
// ESP32-S3 HMI — Car Renderer Implementation
//
// Premium top-down vehicle for the drive dashboard.  Goal: read instantly as a
// premium 4x4 / instrument cluster, NOT as a CAD or debug schematic.
//
//   * Ghost vehicle body: a rounded, gradient-shaded silhouette drawn BEHIND
//     the wheels so the tyres protrude on the sides (wide 4x4 track).
//   * Shaded tyres (the visual hero): each tyre is layered greys faking a
//     cylindrical sidewall, with tread notches, a sheen band and a contact
//     shadow.  A thin colour cap on the rim edge encodes status, so the tyre
//     itself still looks like rubber.
//   * Stylised steering wheel: rim + 3 spokes + hub, between the front tyres.
//   * Faint driveline: thin, dark axles + driveshaft INSIDE the body — a
//     subtle technical accent, never dominant.
//   * Steering arc gauge (right side): a green→orange arc + numeric angle.
//
// Body, steering wheel, driveline and the gauge bezel are STATIC (drawn once
// on screen enter).  Only the tyres and the steering arc repaint per frame.
//
// All geometry uses only TFT_eSPI primitives available on every board:
//   drawLine, drawRect, fillRect, drawRoundRect, fillRoundRect, drawCircle,
//   fillCircle, fillTriangle.  No sprites, no heap, no String.
// =============================================================================

#include "car_renderer.h"
#include "render_trace.h"
#include "ui_config.h"
#include <cstdio>
#include <cmath>

namespace ui {

// =========================================================================
// Body geometry — derived from the existing CAR_* / WHL_* layout constants so
// the vehicle stays centred in the same car area and the wheels keep their
// fixed positions (referenced by the WHEELS tile + side data labels).
// =========================================================================

// Centre X of the vehicle (matches wheel symmetry)
static constexpr int16_t CX = CAR_BODY_X + CAR_BODY_W / 2;   // 240

// Ghost body box — narrower than the wheel track so the tyres stick out on
// both sides (the "wide 4x4" reading).  Inner edges of the wheels are at
// x≈174 (left) and x≈306 (right); the body lives between them.
static constexpr int16_t BODY_W   = 116;
static constexpr int16_t BODY_X   = CX - BODY_W / 2;          // 182
static constexpr int16_t BODY_TOP = WHL_FL_Y - 4;            // 91  (just above front wheels)
static constexpr int16_t BODY_BOT = WHL_RL_Y + WHEEL_H + 4;  // 228 (just below rear wheels)
static constexpr int16_t BODY_H   = BODY_BOT - BODY_TOP;     // 137
static constexpr int16_t BODY_R   = 22;                       // corner radius

// Steering wheel centre — between the front tyres, on the body.
static constexpr int16_t SW_CX = CX;
static constexpr int16_t SW_CY = WHL_FL_Y + WHEEL_H / 2 - 5;  // 112
static constexpr int16_t SW_R  = 15;

// Axle Y positions (aligned with wheel centres so the driveline looks attached)
static constexpr int16_t AXLE_F_Y = WHL_FL_Y + WHEEL_H / 2;   // 117
static constexpr int16_t AXLE_R_Y = WHL_RL_Y + WHEEL_H / 2;   // 202

// -------------------------------------------------------------------------
// drawVehicleBody — static ghost body with a vertical metallic gradient
// -------------------------------------------------------------------------
void CarRenderer::drawVehicleBody(TFT_eSPI& tft) {
    // Soft drop shadow under the body (offset down/right)
    tft.fillRoundRect(BODY_X + 3, BODY_TOP + 2, BODY_W, BODY_H, BODY_R, COL_BODY_DARK);
    RTRACE_FILL_RECT(BODY_X + 3, BODY_TOP + 2, BODY_W, BODY_H, COL_BODY_DARK);

    // Base body (darkest band — the lower/rear gradient)
    tft.fillRoundRect(BODY_X, BODY_TOP, BODY_W, BODY_H, BODY_R, COL_BODY_DARK);
    RTRACE_FILL_RECT(BODY_X, BODY_TOP, BODY_W, BODY_H, COL_BODY_DARK);

    // Mid band covering the upper ~62 % (rounded top corners match the body)
    int16_t midH = static_cast<int16_t>(BODY_H * 62 / 100);
    tft.fillRoundRect(BODY_X, BODY_TOP, BODY_W, midH, BODY_R, COL_BODY_MID);
    RTRACE_FILL_RECT(BODY_X, BODY_TOP, BODY_W, midH, COL_BODY_MID);

    // Top sheen band (~32 %)
    int16_t topH = static_cast<int16_t>(BODY_H * 32 / 100);
    tft.fillRoundRect(BODY_X, BODY_TOP, BODY_W, topH, BODY_R, COL_BODY_LIGHT);
    RTRACE_FILL_RECT(BODY_X, BODY_TOP, BODY_W, topH, COL_BODY_LIGHT);

    // Bright reflective edge along the very top
    tft.drawLine(BODY_X + BODY_R, BODY_TOP + 1, BODY_X + BODY_W - BODY_R, BODY_TOP + 1, COL_BODY_EDGE);
    RTRACE_LINE(BODY_X + BODY_R, BODY_TOP + 1, BODY_X + BODY_W - BODY_R, BODY_TOP + 1, COL_BODY_EDGE);

    // Cabin / roof hint — a faint darker rounded panel in the middle gives the
    // "car" reading without any technical windshield/mirror line work.
    int16_t cabX = BODY_X + 16;
    int16_t cabW = BODY_W - 32;
    int16_t cabY = BODY_TOP + 34;
    int16_t cabH = BODY_H - 70;
    tft.fillRoundRect(cabX, cabY, cabW, cabH, 12, COL_BODY_DARK);
    RTRACE_FILL_RECT(cabX, cabY, cabW, cabH, COL_BODY_DARK);

    // Body outline for a crisp silhouette
    tft.drawRoundRect(BODY_X, BODY_TOP, BODY_W, BODY_H, BODY_R, COL_RIM);
    RTRACE_DRAW_RECT(BODY_X, BODY_TOP, BODY_W, BODY_H, COL_RIM);
}

// -------------------------------------------------------------------------
// drawDriveline — faint, de-emphasised axles + driveshaft (technical accent)
// -------------------------------------------------------------------------
void CarRenderer::drawDriveline(TFT_eSPI& tft) {
    // Front axle (inside the body, between the front wheels)
    tft.drawLine(BODY_X + 6, AXLE_F_Y, BODY_X + BODY_W - 6, AXLE_F_Y, COL_DARK_GRAY);
    RTRACE_LINE(BODY_X + 6, AXLE_F_Y, BODY_X + BODY_W - 6, AXLE_F_Y, COL_DARK_GRAY);

    // Rear axle
    tft.drawLine(BODY_X + 6, AXLE_R_Y, BODY_X + BODY_W - 6, AXLE_R_Y, COL_DARK_GRAY);
    RTRACE_LINE(BODY_X + 6, AXLE_R_Y, BODY_X + BODY_W - 6, AXLE_R_Y, COL_DARK_GRAY);

    // Central driveshaft
    tft.drawLine(CX, AXLE_F_Y, CX, AXLE_R_Y, COL_DARK_GRAY);
    RTRACE_LINE(CX, AXLE_F_Y, CX, AXLE_R_Y, COL_DARK_GRAY);

    // Subtle differential markers (small grey dots, no CAD boxes)
    tft.fillCircle(CX, AXLE_F_Y, 3, COL_GRAY);
    RTRACE_FILL_CIRCLE(CX, AXLE_F_Y, 3, COL_GRAY);
    tft.fillCircle(CX, AXLE_R_Y, 3, COL_GRAY);
    RTRACE_FILL_CIRCLE(CX, AXLE_R_Y, 3, COL_GRAY);
    tft.fillCircle(CX, (AXLE_F_Y + AXLE_R_Y) / 2, 4, COL_GRAY);   // transfer case
    RTRACE_FILL_CIRCLE(CX, (AXLE_F_Y + AXLE_R_Y) / 2, 4, COL_GRAY);
}

// -------------------------------------------------------------------------
// drawSteeringWheel — stylised metallic wheel (rim + 3 spokes + hub)
// -------------------------------------------------------------------------
void CarRenderer::drawSteeringWheel(TFT_eSPI& tft) {
    // Rim (double circle for a thicker metallic look)
    tft.drawCircle(SW_CX, SW_CY, SW_R, COL_RIM);
    tft.drawCircle(SW_CX, SW_CY, SW_R - 1, COL_RIM);
    RTRACE_CIRCLE(SW_CX, SW_CY, SW_R, COL_RIM);

    // Three spokes (Mercedes-like) at 90°, 210°, 330°
    static constexpr float DEG2RAD = 3.14159265f / 180.0f;
    static constexpr float spokeDeg[3] = { 90.0f, 210.0f, 330.0f };
    for (float d : spokeDeg) {
        float a = d * DEG2RAD;
        int16_t ex = SW_CX + static_cast<int16_t>(cosf(a) * (SW_R - 3));
        int16_t ey = SW_CY + static_cast<int16_t>(sinf(a) * (SW_R - 3));
        tft.drawLine(SW_CX, SW_CY, ex, ey, COL_RIM);
        RTRACE_LINE(SW_CX, SW_CY, ex, ey, COL_RIM);
    }

    // Hub
    tft.fillCircle(SW_CX, SW_CY, 4, COL_RIM);
    RTRACE_FILL_CIRCLE(SW_CX, SW_CY, 4, COL_RIM);
    tft.drawCircle(SW_CX, SW_CY, 4, COL_WHITE);
    RTRACE_CIRCLE(SW_CX, SW_CY, 4, COL_WHITE);
}

// -------------------------------------------------------------------------
// drawStatic — static vehicle (body + driveline + steering wheel + gauge bezel)
// Drawn once per screen enter.
// -------------------------------------------------------------------------
void CarRenderer::drawStatic(TFT_eSPI& tft) {
    // Order matters: body first, faint driveline on top of body, steering
    // wheel on top so it is never occluded by the axle accent.
    drawVehicleBody(tft);
    drawDriveline(tft);
    drawSteeringWheel(tft);

    // Steering arc gauge bezel (right side of the car area).  The dynamic arc
    // + needle + numeric angle are painted by drawSteering() inside this ring.
    tft.drawCircle(STEER_CX, STEER_CY, STEER_RADIUS, COL_GRAY);
    tft.drawCircle(STEER_CX, STEER_CY, STEER_RADIUS + 1, COL_GRAY);
    RTRACE_CIRCLE(STEER_CX, STEER_CY, STEER_RADIUS, COL_GRAY);
}

// -------------------------------------------------------------------------
// Draw/update all 4 wheels + their torque/temp labels
//
// Wheel order: 0=FL (left), 1=FR (right), 2=RL (left), 3=RR (right)
// Labels are placed to the left for FL/RL, to the right for FR/RR.
// -------------------------------------------------------------------------
void CarRenderer::drawWheels(TFT_eSPI& tft,
                             const vehicle::TractionData& traction,
                             const vehicle::TempMapData& tempMap,
                             const uint8_t prevTraction[4],
                             const int8_t prevTemp[4],
                             bool tractionValid,
                             bool tempValid) {
    static constexpr int16_t wx[4] = { WHL_FL_X, WHL_FR_X, WHL_RL_X, WHL_RR_X };
    static constexpr int16_t wy[4] = { WHL_FL_Y, WHL_FR_Y, WHL_RL_Y, WHL_RR_Y };
    // rightSide: FR(1) and RR(3) have labels on the right
    static constexpr bool rs[4] = { false, true, false, true };

    for (uint8_t i = 0; i < 4; ++i) {
        if (traction.scale[i] != prevTraction[i] ||
            tempMap.temps[i] != prevTemp[i]) {
            drawWheel(tft, wx[i], wy[i], traction.scale[i], tempMap.temps[i],
                      tractionValid);
            drawWheelLabel(tft, wx[i], wy[i],
                           traction.scale[i], tempMap.temps[i], rs[i],
                           tractionValid, tempValid);
        }
    }
}

// -------------------------------------------------------------------------
// Draw a single shaded tyre (top-down) within the WHEEL_W × WHEEL_H box.
//
// The tyre is drawn as a vertical rounded shape (taller than wide) centred in
// the box, with layered greys faking a cylindrical sidewall, tread notches, a
// sheen band and a contact shadow.  A thin colour cap on the top/bottom rim
// edge encodes status (torque level / staleness) without colouring the rubber.
// -------------------------------------------------------------------------
void CarRenderer::drawWheel(TFT_eSPI& tft,
                            int16_t x, int16_t y,
                            uint8_t torquePct, int8_t tempC,
                            bool tractionValid) {
    (void)tempC;

    // Status colour: torque level when valid, neutral grey when stale/unknown.
    uint16_t statusCol = tractionValid ? torqueColor(torquePct) : COL_GRAY;

    // Clear the whole wheel box (labels live outside this box — see drawWheels)
    tft.fillRect(x, y, WHEEL_W, WHEEL_H, COL_BG);
    RTRACE_FILL_RECT(x, y, WHEEL_W, WHEEL_H, COL_BG);

    // Tyre footprint inside the box (vertical rounded rect)
    static constexpr int16_t TW = 26;          // tyre width
    static constexpr int16_t TH = WHEEL_H - 2;  // tyre height (42)
    int16_t tx = x + (WHEEL_W - TW) / 2;        // centred horizontally
    int16_t ty = y + 1;

    // Contact shadow (offset down/right, dark)
    tft.fillRoundRect(tx + 2, ty + 3, TW, TH, 7, COL_TYRE_DARK);
    RTRACE_FILL_RECT(tx + 2, ty + 3, TW, TH, COL_TYRE_DARK);

    // Tyre outer (dark rubber)
    tft.fillRoundRect(tx, ty, TW, TH, 7, COL_TYRE_DARK);
    RTRACE_FILL_RECT(tx, ty, TW, TH, COL_TYRE_DARK);

    // Sidewall mid tone
    tft.fillRoundRect(tx + 3, ty + 2, TW - 6, TH - 4, 5, COL_TYRE_MID);
    RTRACE_FILL_RECT(tx + 3, ty + 2, TW - 6, TH - 4, COL_TYRE_MID);

    // Central sheen band (fakes the rounded cross-section highlight)
    tft.fillRect(tx + (TW / 2) - 3, ty + 3, 6, TH - 6, COL_TYRE_LIGHT);
    RTRACE_FILL_RECT(tx + (TW / 2) - 3, ty + 3, 6, TH - 6, COL_TYRE_LIGHT);

    // Tread notches (short dark horizontal bars across the tyre face)
    for (int16_t ny = ty + 5; ny < ty + TH - 4; ny += 7) {
        tft.fillRect(tx + 3, ny, TW - 6, 2, COL_TYRE_DARK);
    }
    RTRACE_FILL_RECT(tx + 3, ty + 5, TW - 6, TH - 10, COL_TYRE_DARK);  // trace approx

    // Hub hint (small alloy centre)
    int16_t hcx = x + WHEEL_W / 2;
    int16_t hcy = y + WHEEL_H / 2;
    tft.fillCircle(hcx, hcy, 3, COL_RIM);
    RTRACE_FILL_CIRCLE(hcx, hcy, 3, COL_RIM);

    // Status colour caps on the top and bottom rim edges
    tft.fillRect(tx + 4, ty, TW - 8, 2, statusCol);
    RTRACE_FILL_RECT(tx + 4, ty, TW - 8, 2, statusCol);
    tft.fillRect(tx + 4, ty + TH - 2, TW - 8, 2, statusCol);
    RTRACE_FILL_RECT(tx + 4, ty + TH - 2, TW - 8, 2, statusCol);
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
                                 bool rightSide,
                                 bool tractionValid,
                                 bool tempValid) {
    // Stale telemetry must not be rendered as a real value (e.g. "100%" /
    // "0°C"): show a neutral placeholder and a muted colour instead.
    uint16_t torqueCol = tractionValid ? torqueColor(torquePct) : COL_GRAY;
    // Smart 5-level temperature colour (blue→green→yellow→orange→red).
    uint16_t tempCol   = tempValid ? tempColorFull(tempC) : COL_GRAY;
    char buf[FMT_BUF_SMALL];

    if (rightSide) {
        // Label to the right of the wheel
        int16_t lx = wx + WHEEL_W + 4;
        int16_t ly = wy + 4;
        // Torque %
        if (tractionValid) snprintf(buf, sizeof(buf), "%u%%", torquePct);
        else               snprintf(buf, sizeof(buf), "--");
        tft.setTextColor(torqueCol, COL_BG);
        tft.setTextSize(2);
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(cfg::PAD_WHEEL_LABEL);
        tft.drawString(buf, lx, ly);
        RTRACE_TEXT(lx, ly, buf, torqueCol, COL_BG, 2, TL_DATUM);
        // Temperature
        if (tempValid) snprintf(buf, sizeof(buf), "%d\xC2\xB0""C", tempC);
        else           snprintf(buf, sizeof(buf), "N/A");
        tft.setTextColor(tempCol, COL_BG);
        tft.setTextSize(1);
        tft.setTextPadding(cfg::PAD_WHEEL_LABEL);
        tft.drawString(buf, lx + 2, ly + 18);
        RTRACE_TEXT(lx + 2, ly + 18, buf, tempCol, COL_BG, 1, TL_DATUM);
        tft.setTextPadding(0);
    } else {
        // Label to the left of the wheel
        int16_t lx = wx - 50;
        int16_t ly = wy + 4;
        // Torque %
        if (tractionValid) snprintf(buf, sizeof(buf), "%u%%", torquePct);
        else               snprintf(buf, sizeof(buf), "--");
        tft.setTextColor(torqueCol, COL_BG);
        tft.setTextSize(2);
        tft.setTextDatum(TR_DATUM);
        tft.setTextPadding(cfg::PAD_WHEEL_LABEL);
        tft.drawString(buf, lx + 46, ly);
        RTRACE_TEXT(lx + 46, ly, buf, torqueCol, COL_BG, 2, TR_DATUM);
        // Temperature
        if (tempValid) snprintf(buf, sizeof(buf), "%d\xC2\xB0""C", tempC);
        else           snprintf(buf, sizeof(buf), "N/A");
        tft.setTextColor(tempCol, COL_BG);
        tft.setTextSize(1);
        tft.setTextPadding(cfg::PAD_WHEEL_LABEL);
        tft.drawString(buf, lx + 44, ly + 18);
        RTRACE_TEXT(lx + 44, ly + 18, buf, tempCol, COL_BG, 1, TR_DATUM);
        tft.setTextPadding(0);
    }

    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}

// -------------------------------------------------------------------------
// Draw steering angle indicator — green→orange arc + needle + numeric angle.
// Repaints only when the angle changed.
// -------------------------------------------------------------------------
void CarRenderer::drawSteering(TFT_eSPI& tft,
                               int16_t angleRaw,
                               int16_t prevAngleRaw) {
    if (angleRaw == prevAngleRaw) return;

    // Colour: green when centred, orange once turning (matches the reference).
    int16_t absDeg = static_cast<int16_t>((angleRaw < 0 ? -angleRaw : angleRaw) / 10);
    uint16_t col = (absDeg <= 3) ? COL_GREEN : COL_ORANGE;

    drawSteerArc(tft, angleRaw, col);
}

// -------------------------------------------------------------------------
// drawSteerArc — clear the gauge interior, redraw bezel, then paint the arc,
// needle and numeric angle for the given raw angle / colour.
// -------------------------------------------------------------------------
void CarRenderer::drawSteerArc(TFT_eSPI& tft,
                               int16_t angleRaw, uint16_t color) {
    // Clear gauge interior.
    tft.fillCircle(STEER_CX, STEER_CY, STEER_RADIUS - 1, COL_BG);
    RTRACE_FILL_CIRCLE(STEER_CX, STEER_CY, STEER_RADIUS - 1, COL_BG);

    // Redraw the static bezel (fixes any erased pixels on the ring).
    tft.drawCircle(STEER_CX, STEER_CY, STEER_RADIUS, COL_GRAY);
    tft.drawCircle(STEER_CX, STEER_CY, STEER_RADIUS + 1, COL_GRAY);
    RTRACE_CIRCLE(STEER_CX, STEER_CY, STEER_RADIUS, COL_GRAY);

    // angleRaw is in 0.1° units; 0° = straight up, positive = right turn.
    float angleDeg = static_cast<float>(angleRaw) * 0.1f;
    if (angleDeg >  45.0f) angleDeg =  45.0f;
    if (angleDeg < -45.0f) angleDeg = -45.0f;

    static constexpr float DEG2RAD = 3.14159265f / 180.0f;

    // Sweep arc from top (0°) to the current angle, as short radial ticks near
    // the rim — reads as a filling arc without per-pixel arc math.
    float   mag   = (angleDeg < 0.0f) ? -angleDeg : angleDeg;
    int16_t steps = static_cast<int16_t>(mag);
    int16_t dir   = (angleDeg < 0.0f) ? -1 : 1;
    for (int16_t d = 0; d <= steps; d += 3) {
        float a = (static_cast<float>(dir * d) - 90.0f) * DEG2RAD;
        int16_t r1 = STEER_RADIUS - 5;
        int16_t r2 = STEER_RADIUS - 1;
        int16_t x1 = STEER_CX + static_cast<int16_t>(cosf(a) * r1);
        int16_t y1 = STEER_CY + static_cast<int16_t>(sinf(a) * r1);
        int16_t x2 = STEER_CX + static_cast<int16_t>(cosf(a) * r2);
        int16_t y2 = STEER_CY + static_cast<int16_t>(sinf(a) * r2);
        tft.drawLine(x1, y1, x2, y2, color);
        RTRACE_LINE(x1, y1, x2, y2, color);
    }

    // Needle from centre to the current angle.
    float na = (angleDeg - 90.0f) * DEG2RAD;
    int16_t ex = STEER_CX + static_cast<int16_t>(cosf(na) * (STEER_RADIUS - 4));
    int16_t ey = STEER_CY + static_cast<int16_t>(sinf(na) * (STEER_RADIUS - 4));
    tft.drawLine(STEER_CX, STEER_CY, ex, ey, color);
    RTRACE_LINE(STEER_CX, STEER_CY, ex, ey, color);

    // Centre hub
    tft.fillCircle(STEER_CX, STEER_CY, 3, COL_WHITE);
    RTRACE_FILL_CIRCLE(STEER_CX, STEER_CY, 3, COL_WHITE);

    // Numeric angle below the gauge (0.1° → whole degrees).
    char buf[FMT_BUF_SMALL];
    int16_t deg = static_cast<int16_t>(angleRaw / 10);
    snprintf(buf, sizeof(buf), "%d\xC2\xB0", deg);
    tft.setTextColor(color, COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(TC_DATUM);
    tft.setTextPadding(cfg::PAD_RPM);
    tft.drawString(buf, STEER_CX, STEER_CY + STEER_RADIUS + 4);
    RTRACE_TEXT(STEER_CX, STEER_CY + STEER_RADIUS + 4, buf, color, COL_BG, 1, TC_DATUM);
    tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);
}

} // namespace ui
