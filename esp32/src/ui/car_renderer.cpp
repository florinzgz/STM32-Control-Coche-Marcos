// =============================================================================
// ESP32-S3 HMI — Car Renderer Implementation (FASE 3.5)
//
// Premium top-down cockpit.  See car_renderer.h for the design intent.
//
//   * Static : solid gradient-shaded SUV body (volume + soft shadow) and the
//              recessed cockpit pod ring.  No driveline / wireframe.
//   * Tyres  : four shaded capsules at the corners with % / C / relative load
//              bar; the hardest-working wheel is highlighted.
//   * Steering: ONE large 3-spoke wheel that rotates with the reconstructed
//              steering-wheel angle, plus a dual (Steering Wheel / Wheel Angle)
//              read-out.  Replaces the old hub wheel AND the separate gauge.
//
// Only TFT_eSPI primitives — no sprites, no heap, no String.
// =============================================================================

#include "car_renderer.h"
#include "render_trace.h"
#include "ui_config.h"
#include <cstdio>
#include <cmath>

namespace ui {

// =========================================================================
// Geometry (derived from ui_common.h layout constants)
// =========================================================================
static constexpr int16_t CX = SWHEEL_CX;                 // 240, vehicle centre

// Premium body silhouette (top-down).  Wider, solid, with rounded shoulders.
static constexpr int16_t BODY_X   = 150;
static constexpr int16_t BODY_W   = 180;                 // 150..330
static constexpr int16_t BODY_TOP = 84;
static constexpr int16_t BODY_H   = 144;                 // 84..228
static constexpr int16_t BODY_R   = 28;

// Cockpit pod well (flat dark region owned by the dynamic steering tile — it
// is re-cleared each angle change so no body gradient lives underneath).
static constexpr int16_t PW_X = 172;
static constexpr int16_t PW_W = 136;                     // 172..308
static constexpr int16_t PW_Y = 88;
static constexpr int16_t PW_H = 132;                     // 88..220
static constexpr int16_t PW_R = 18;

static constexpr float DEG2RAD = 3.14159265f / 180.0f;

// -------------------------------------------------------------------------
// drawVehicleBody — solid, gradient-shaded premium SUV silhouette
// -------------------------------------------------------------------------
void CarRenderer::drawVehicleBody(TFT_eSPI& tft) {
    // Soft drop shadow (offset down/right) for depth.
    tft.fillRoundRect(BODY_X + 4, BODY_TOP + 4, BODY_W, BODY_H, BODY_R, COL_BODY_DARK);
    RTRACE_FILL_RECT(BODY_X + 4, BODY_TOP + 4, BODY_W, BODY_H, COL_BODY_DARK);

    // Base body (darkest band — lower/rear).
    tft.fillRoundRect(BODY_X, BODY_TOP, BODY_W, BODY_H, BODY_R, COL_BODY_DARK);
    RTRACE_FILL_RECT(BODY_X, BODY_TOP, BODY_W, BODY_H, COL_BODY_DARK);

    // Mid band over the upper ~64 % (metallic gradient).
    int16_t midH = static_cast<int16_t>(BODY_H * 64 / 100);
    tft.fillRoundRect(BODY_X, BODY_TOP, BODY_W, midH, BODY_R, COL_BODY_MID);
    RTRACE_FILL_RECT(BODY_X, BODY_TOP, BODY_W, midH, COL_BODY_MID);

    // Top sheen band (~34 %).
    int16_t topH = static_cast<int16_t>(BODY_H * 34 / 100);
    tft.fillRoundRect(BODY_X, BODY_TOP, BODY_W, topH, BODY_R, COL_BODY_LIGHT);
    RTRACE_FILL_RECT(BODY_X, BODY_TOP, BODY_W, topH, COL_BODY_LIGHT);

    // Bright reflective edge along the very top (front of the vehicle).
    tft.drawLine(BODY_X + BODY_R, BODY_TOP + 1, BODY_X + BODY_W - BODY_R, BODY_TOP + 1, COL_BODY_EDGE);
    RTRACE_LINE(BODY_X + BODY_R, BODY_TOP + 1, BODY_X + BODY_W - BODY_R, BODY_TOP + 1, COL_BODY_EDGE);

    // Headlight hints near the front corners (warm accents — reads as a car,
    // not a diagram).  Single soft blocks, no technical line work.
    tft.fillRoundRect(BODY_X + 14, BODY_TOP + 6, 22, 8, 3, COL_HEADLIGHT);
    RTRACE_FILL_RECT(BODY_X + 14, BODY_TOP + 6, 22, 8, COL_HEADLIGHT);
    tft.fillRoundRect(BODY_X + BODY_W - 36, BODY_TOP + 6, 22, 8, 3, COL_HEADLIGHT);
    RTRACE_FILL_RECT(BODY_X + BODY_W - 36, BODY_TOP + 6, 22, 8, COL_HEADLIGHT);

    // Crisp silhouette outline.
    tft.drawRoundRect(BODY_X, BODY_TOP, BODY_W, BODY_H, BODY_R, COL_RIM);
    RTRACE_DRAW_RECT(BODY_X, BODY_TOP, BODY_W, BODY_H, COL_RIM);
}

// -------------------------------------------------------------------------
// drawCockpitPodStatic — recessed dark well the steering wheel sits in
// -------------------------------------------------------------------------
void CarRenderer::drawCockpitPodStatic(TFT_eSPI& tft) {
    // Flat dark cockpit panel (the dynamic steering tile re-clears this rect).
    tft.fillRoundRect(PW_X, PW_Y, PW_W, PW_H, PW_R, COL_DIAL_FACE);
    RTRACE_FILL_RECT(PW_X, PW_Y, PW_W, PW_H, COL_DIAL_FACE);
    tft.drawRoundRect(PW_X, PW_Y, PW_W, PW_H, PW_R, COL_DIAL_RING);
    RTRACE_DRAW_RECT(PW_X, PW_Y, PW_W, PW_H, COL_DIAL_RING);
}

// -------------------------------------------------------------------------
// drawStatic — static vehicle (body + cockpit pod ring).  Drawn once.
// -------------------------------------------------------------------------
void CarRenderer::drawStatic(TFT_eSPI& tft) {
    drawVehicleBody(tft);
    drawCockpitPodStatic(tft);
}

// -------------------------------------------------------------------------
// drawWheels — repaint all four corner capsules.
//
// Wheel order: 0=FL, 1=FR, 2=RL, 3=RR.  The relative "hardest-working" wheel
// highlight depends on all four values, so every capsule is redrawn together.
// -------------------------------------------------------------------------
void CarRenderer::drawWheels(TFT_eSPI& tft,
                             const vehicle::TractionData& traction,
                             const vehicle::TempMapData& tempMap,
                             const uint8_t prevTraction[4],
                             const int8_t prevTemp[4],
                             bool tractionValid,
                             bool tempValid) {
    (void)prevTraction;
    (void)prevTemp;

    static constexpr int16_t cx[4]   = { WCAP_L_X, WCAP_R_X, WCAP_L_X, WCAP_R_X };
    static constexpr int16_t cy[4]   = { WCAP_TOP_Y, WCAP_TOP_Y, WCAP_BOT_Y, WCAP_BOT_Y };
    static constexpr bool    left[4] = { true, false, true, false };
    static const char*       lbl[4]  = { "FL", "FR", "RL", "RR" };

    // Hardest-working wheel: the max torque across valid wheels.  Ties all
    // highlight; if all read 0 (or stale) nothing is highlighted.
    uint8_t maxLoad = 0;
    if (tractionValid) {
        for (uint8_t i = 0; i < 4; ++i)
            if (traction.scale[i] > maxLoad) maxLoad = traction.scale[i];
    }

    for (uint8_t i = 0; i < 4; ++i) {
        bool isMax = tractionValid && maxLoad > 0 &&
                     traction.scale[i] == maxLoad;
        drawWheelCapsule(tft, cx[i], cy[i], lbl[i],
                         traction.scale[i], tempMap.temps[i],
                         left[i], isMax, tractionValid, tempValid);
    }
}

// -------------------------------------------------------------------------
// drawWheelCapsule — one tyre + % + C + relative 4-segment load bar.
// -------------------------------------------------------------------------
void CarRenderer::drawWheelCapsule(TFT_eSPI& tft,
                                   int16_t x, int16_t y,
                                   const char* label,
                                   uint8_t torquePct, int8_t tempC,
                                   bool leftSide,
                                   bool isMaxLoad,
                                   bool tractionValid,
                                   bool tempValid) {
    // Clear the capsule region.
    tft.fillRect(x, y, WCAP_W, WCAP_H, COL_BG);
    RTRACE_FILL_RECT(x, y, WCAP_W, WCAP_H, COL_BG);

    uint16_t loadCol = tractionValid ? torqueColor(torquePct) : COL_GRAY;
    uint16_t tempCol = tempValid    ? tempColorFull(tempC)    : COL_GRAY;

    // ---- Tyre graphic (inner side of the capsule, toward the vehicle) ----
    static constexpr int16_t TW = 26;
    static constexpr int16_t TH = WCAP_H - 12;            // 50
    int16_t tx = leftSide ? (x + WCAP_W - TW - 6) : (x + 6);
    int16_t ty = y + 6;

    // Contact shadow.
    tft.fillRoundRect(tx + 2, ty + 3, TW, TH, 7, COL_TYRE_DARK);
    RTRACE_FILL_RECT(tx + 2, ty + 3, TW, TH, COL_TYRE_DARK);
    // Outer rubber.
    tft.fillRoundRect(tx, ty, TW, TH, 7, COL_TYRE_DARK);
    RTRACE_FILL_RECT(tx, ty, TW, TH, COL_TYRE_DARK);
    // Sidewall mid tone.
    tft.fillRoundRect(tx + 3, ty + 2, TW - 6, TH - 4, 5, COL_TYRE_MID);
    RTRACE_FILL_RECT(tx + 3, ty + 2, TW - 6, TH - 4, COL_TYRE_MID);
    // Central sheen band.
    tft.fillRect(tx + (TW / 2) - 3, ty + 3, 6, TH - 6, COL_TYRE_LIGHT);
    RTRACE_FILL_RECT(tx + (TW / 2) - 3, ty + 3, 6, TH - 6, COL_TYRE_LIGHT);
    // Tread notches.
    for (int16_t ny = ty + 5; ny < ty + TH - 4; ny += 7) {
        tft.fillRect(tx + 3, ny, TW - 6, 2, COL_TYRE_DARK);
    }
    RTRACE_FILL_RECT(tx + 3, ty + 5, TW - 6, TH - 10, COL_TYRE_DARK);
    // Alloy hub hint.
    tft.fillCircle(tx + TW / 2, ty + TH / 2, 3, COL_RIM);
    RTRACE_FILL_CIRCLE(tx + TW / 2, ty + TH / 2, 3, COL_RIM);
    // Status colour caps (top/bottom rim edge).
    tft.fillRect(tx + 4, ty, TW - 8, 2, loadCol);
    RTRACE_FILL_RECT(tx + 4, ty, TW - 8, 2, loadCol);
    tft.fillRect(tx + 4, ty + TH - 2, TW - 8, 2, loadCol);
    RTRACE_FILL_RECT(tx + 4, ty + TH - 2, TW - 8, 2, loadCol);

    // Hardest-working highlight: amber ring around the tyre + a marker dot.
    if (isMaxLoad) {
        tft.drawRoundRect(tx - 2, ty - 2, TW + 4, TH + 4, 8, COL_AMBER);
        RTRACE_DRAW_RECT(tx - 2, ty - 2, TW + 4, TH + 4, COL_AMBER);
        tft.drawRoundRect(tx - 3, ty - 3, TW + 6, TH + 6, 9, COL_AMBER);
    }

    // ---- Text block (outer side of the capsule) ----
    int16_t txt = leftSide ? (x + 4) : (x + TW + 16);
    uint16_t lblCol = isMaxLoad ? COL_AMBER : COL_GRAY;

    char buf[FMT_BUF_SMALL];

    // Label + (optional) MAX flag.
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
    tft.setTextColor(lblCol, COL_BG);
    tft.drawString(label, txt, y + 1);
    RTRACE_TEXT(txt, y + 1, label, lblCol, COL_BG, 1, TL_DATUM);
    if (isMaxLoad) {
        tft.setTextColor(COL_AMBER, COL_BG);
        tft.drawString("MAX", txt + 20, y + 1);
        RTRACE_TEXT(txt + 20, y + 1, "MAX", COL_AMBER, COL_BG, 1, TL_DATUM);
    }

    // Torque %.
    if (tractionValid) snprintf(buf, sizeof(buf), "%u%%", torquePct);
    else               snprintf(buf, sizeof(buf), "--");
    tft.setTextColor(loadCol, COL_BG);
    tft.setTextSize(2);
    tft.drawString(buf, txt, y + 12);
    RTRACE_TEXT(txt, y + 12, buf, loadCol, COL_BG, 2, TL_DATUM);

    // Temperature.
    if (tempValid) snprintf(buf, sizeof(buf), "%d\xC2\xB0""C", tempC);
    else           snprintf(buf, sizeof(buf), "N/A");
    tft.setTextColor(tempCol, COL_BG);
    tft.setTextSize(1);
    tft.drawString(buf, txt, y + 32);
    RTRACE_TEXT(txt, y + 32, buf, tempCol, COL_BG, 1, TL_DATUM);

    // ---- Relative 4-segment load bar ----
    // Absolute %, four 25 %-segments (mirrors the approved mockup "FL ▰▰▰▱").
    // Real data is never invented: if all read 100 %, four segments show.
    static constexpr int16_t SEG_W = 19;
    static constexpr int16_t SEG_H = 8;
    static constexpr int16_t SEG_GAP = 2;
    int16_t barY = y + WCAP_H - SEG_H - 1;
    uint8_t filled = tractionValid
                     ? static_cast<uint8_t>((torquePct * 4 + 50) / 100)
                     : 0;
    if (filled > 4) filled = 4;
    for (uint8_t s = 0; s < 4; ++s) {
        int16_t sx = txt + s * (SEG_W + SEG_GAP);
        uint16_t segCol = (s < filled) ? loadCol : COL_DARK_GRAY;
        tft.fillRoundRect(sx, barY, SEG_W, SEG_H, 2, segCol);
        RTRACE_FILL_RECT(sx, barY, SEG_W, SEG_H, segCol);
    }

    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}

// -------------------------------------------------------------------------
// drawSteering — repaint the single rotating wheel only if the angle changed.
// -------------------------------------------------------------------------
void CarRenderer::drawSteering(TFT_eSPI& tft,
                               int16_t angleRaw,
                               int16_t prevAngleRaw) {
    if (angleRaw == prevAngleRaw) return;
    drawSteeringWheelDynamic(tft, angleRaw);
}

// -------------------------------------------------------------------------
// drawSteeringWheelDynamic — clear the cockpit pod, draw the rotating
// 3-spoke wheel, a progressive direction arc and the dual angle read-out.
//
// Presentation only.  angleRaw = road-wheel angle in 0.1 deg (CAN 0x204).
//   Wheel Angle    = angleRaw / 10           (real, firmware +-54)
//   Steering Wheel = WheelAngle x 6.48        (reconstructed column, ~+-350)
// -------------------------------------------------------------------------
void CarRenderer::drawSteeringWheelDynamic(TFT_eSPI& tft, int16_t angleRaw) {
    // Clamp the road-wheel angle to the firmware envelope (+-54 deg).
    int16_t wheelTenth = angleRaw;
    int16_t lim = static_cast<int16_t>(STEER_MAX_WHEEL_DEG * 10);   // 540
    if (wheelTenth >  lim) wheelTenth =  lim;
    if (wheelTenth < -lim) wheelTenth = -lim;

    // Reconstructed steering-wheel (column) angle, clamped for display.
    int16_t colDeg = steeringWheelDeg(wheelTenth);
    if (colDeg >  350) colDeg =  350;
    if (colDeg < -350) colDeg = -350;

    const int16_t cx = SWHEEL_CX;
    const int16_t cy = SWHEEL_CY;
    const int16_t R  = SWHEEL_R;

    // ---- Re-clear the cockpit pod (flat — no gradient underneath) ----
    tft.fillRoundRect(PW_X, PW_Y, PW_W, PW_H, PW_R, COL_DIAL_FACE);
    RTRACE_FILL_RECT(PW_X, PW_Y, PW_W, PW_H, COL_DIAL_FACE);
    tft.drawRoundRect(PW_X, PW_Y, PW_W, PW_H, PW_R, COL_DIAL_RING);
    RTRACE_DRAW_RECT(PW_X, PW_Y, PW_W, PW_H, COL_DIAL_RING);

    // Recessed disc behind the wheel.
    tft.fillCircle(cx, cy, SWHEEL_POD_R, COL_BG);
    RTRACE_FILL_CIRCLE(cx, cy, SWHEEL_POD_R, COL_BG);
    tft.drawCircle(cx, cy, SWHEEL_POD_R, COL_DIAL_RING);
    RTRACE_CIRCLE(cx, cy, SWHEEL_POD_R, COL_DIAL_RING);

    // Turn-intensity colour (kept consistent across the read-out).
    int16_t absWheelDeg = (wheelTenth < 0 ? -wheelTenth : wheelTenth) / 10;
    uint16_t turnCol = (absWheelDeg <= 3)  ? COL_GREEN
                     : (absWheelDeg <= 30) ? COL_AMBER
                                           : COL_ORANGE;

    // ---- Progressive direction arc on the disc rim ----
    // Sweeps from top (-90 deg) by an amount proportional to the column angle
    // (scaled into a readable +-150 deg visual arc), showing both magnitude
    // and direction.
    int16_t arcSpan = static_cast<int16_t>(
        static_cast<int32_t>(colDeg) * 150 / 350);       // +-150 visual
    int16_t arcStep = (arcSpan < 0) ? -4 : 4;
    int16_t r1 = SWHEEL_POD_R - 4, r2 = SWHEEL_POD_R - 1;
    for (int16_t d = 0; (arcStep > 0) ? (d <= arcSpan) : (d >= arcSpan); d += arcStep) {
        float a = (static_cast<float>(d) - 90.0f) * DEG2RAD;
        int16_t x1 = cx + static_cast<int16_t>(cosf(a) * r1);
        int16_t y1 = cy + static_cast<int16_t>(sinf(a) * r1);
        int16_t x2 = cx + static_cast<int16_t>(cosf(a) * r2);
        int16_t y2 = cy + static_cast<int16_t>(sinf(a) * r2);
        tft.drawLine(x1, y1, x2, y2, turnCol);
        RTRACE_LINE(x1, y1, x2, y2, turnCol);
    }

    // ---- Steering wheel rim (thick, metallic) ----
    tft.drawCircle(cx, cy, R,     COL_RIM);
    tft.drawCircle(cx, cy, R - 1, COL_RIM);
    tft.drawCircle(cx, cy, R - 2, COL_RIM);
    tft.drawCircle(cx, cy, R - 3, COL_TYRE_DARK);   // inner shadow groove
    RTRACE_CIRCLE(cx, cy, R, COL_RIM);

    // ---- Three rotating spokes (Mercedes-style: two upper + one lower) ----
    // Base angles in screen space (y-down): upper-left, upper-right, bottom.
    static constexpr float spokeBase[3] = { 200.0f, 340.0f, 90.0f };
    float rot = static_cast<float>(colDeg);             // CW positive = right
    int16_t hubR = 7;
    for (float base : spokeBase) {
        float a = (base + rot) * DEG2RAD;
        float ca = cosf(a), sa = sinf(a);
        int16_t ex = cx + static_cast<int16_t>(ca * (R - 4));
        int16_t ey = cy + static_cast<int16_t>(sa * (R - 4));
        int16_t sx = cx + static_cast<int16_t>(ca * hubR);
        int16_t sy = cy + static_cast<int16_t>(sa * hubR);
        // Thicken the spoke with a perpendicular offset (~3 px).
        int16_t px = static_cast<int16_t>(-sa * 1.5f);
        int16_t py = static_cast<int16_t>( ca * 1.5f);
        tft.fillTriangle(sx + px, sy + py, sx - px, sy - py, ex, ey, COL_RIM);
        tft.drawLine(sx, sy, ex, ey, COL_RIM);
        RTRACE_LINE(sx, sy, ex, ey, COL_RIM);
    }

    // ---- Hub with AMG-style accent ----
    tft.fillCircle(cx, cy, hubR, COL_RIM);
    RTRACE_FILL_CIRCLE(cx, cy, hubR, COL_RIM);
    tft.fillCircle(cx, cy, hubR - 2, COL_DIAL_FACE);
    tft.fillCircle(cx, cy, 2, turnCol);

    // ---- Dual read-out (magnitudes kept separate) ----
    char buf[FMT_BUF_MED];

    tft.setTextDatum(TC_DATUM);

    // Caption.
    tft.setTextSize(1);
    tft.setTextColor(COL_GRAY, COL_DIAL_FACE);
    tft.drawString("STEERING WHEEL", cx, SWHEEL_TXT_Y1);
    RTRACE_TEXT(cx, SWHEEL_TXT_Y1, "STEERING WHEEL", COL_GRAY, COL_DIAL_FACE, 1, TC_DATUM);

    // Reconstructed steering-wheel angle (big).
    snprintf(buf, sizeof(buf), "%+d\xC2\xB0", colDeg);
    tft.setTextSize(2);
    tft.setTextColor(turnCol, COL_DIAL_FACE);
    tft.drawString(buf, cx, SWHEEL_VAL_Y);
    RTRACE_TEXT(cx, SWHEEL_VAL_Y, buf, turnCol, COL_DIAL_FACE, 2, TC_DATUM);

    // Real road-wheel angle (secondary, one decimal).
    int16_t aw = (wheelTenth < 0 ? -wheelTenth : wheelTenth);
    snprintf(buf, sizeof(buf), "WHEEL ANGLE %s%d.%d\xC2\xB0",
             wheelTenth < 0 ? "-" : "+", aw / 10, aw % 10);
    tft.setTextSize(1);
    tft.setTextColor(COL_CYAN, COL_DIAL_FACE);
    tft.drawString(buf, cx, SWHEEL_TXT_Y2);
    RTRACE_TEXT(cx, SWHEEL_TXT_Y2, buf, COL_CYAN, COL_DIAL_FACE, 1, TC_DATUM);

    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}

} // namespace ui
