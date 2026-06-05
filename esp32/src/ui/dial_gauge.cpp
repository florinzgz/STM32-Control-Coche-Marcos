// =============================================================================
// ESP32-S3 HMI — Analog Dial Gauge Implementation
// =============================================================================

#include "dial_gauge.h"
#include "render_trace.h"
#include <cmath>

namespace ui {

static constexpr float DG_DEG2RAD = 3.14159265f / 180.0f;

// Map a gauge angle (deg, 0 = up, clockwise positive) to a screen point at the
// given radius around (cx, cy).
static inline void dgPolar(int16_t cx, int16_t cy, float r, float deg,
                           int16_t& x, int16_t& y) {
    float a = deg * DG_DEG2RAD;
    x = static_cast<int16_t>(cx + sinf(a) * r);
    y = static_cast<int16_t>(cy - cosf(a) * r);
}

// Zoned progress colour: green (lower) → amber (mid) → red (upper).
static inline uint16_t dgZoneColor(float frac) {
    if (frac < 0.6f) return COL_GREEN;
    if (frac < 0.85f) return COL_AMBER;
    return COL_RED;
}

// -------------------------------------------------------------------------
// drawStatic — face disc, bezel ring and evenly spaced major ticks.
// -------------------------------------------------------------------------
void DialGauge::drawStatic(TFT_eSPI& tft, int16_t cx, int16_t cy, int16_t r) {
    // Face disc (subtle dark interior).
    tft.fillCircle(cx, cy, r - 1, COL_DIAL_FACE);
    RTRACE_FILL_CIRCLE(cx, cy, r - 1, COL_DIAL_FACE);

    // Double bezel ring for a thicker metallic look.
    tft.drawCircle(cx, cy, r, COL_DIAL_RING);
    tft.drawCircle(cx, cy, r - 1, COL_DIAL_RING);
    RTRACE_CIRCLE(cx, cy, r, COL_DIAL_RING);

    // Major ticks along the 270° sweep (7 ticks: 0, 1/6 … 1).
    static constexpr int16_t NUM_TICKS = 7;
    for (int16_t i = 0; i < NUM_TICKS; ++i) {
        float frac = static_cast<float>(i) / static_cast<float>(NUM_TICKS - 1);
        float deg  = static_cast<float>(DIAL_START_DEG) +
                     frac * static_cast<float>(DIAL_SWEEP_DEG);
        int16_t x1, y1, x2, y2;
        dgPolar(cx, cy, static_cast<float>(r - 5), deg, x1, y1);
        dgPolar(cx, cy, static_cast<float>(r - 2), deg, x2, y2);
        tft.drawLine(x1, y1, x2, y2, COL_DIAL_TICK);
        RTRACE_LINE(x1, y1, x2, y2, COL_DIAL_TICK);
    }
}

// -------------------------------------------------------------------------
// draw — dynamic interior: clear, progress arc, needle, hub, value + unit.
// -------------------------------------------------------------------------
void DialGauge::draw(TFT_eSPI& tft, int16_t cx, int16_t cy, int16_t r,
                     uint16_t value, uint16_t maxValue,
                     uint16_t needleColor, bool zoned,
                     const char* valueText, const char* unit) {
    if (maxValue == 0) maxValue = 1;
    if (value > maxValue) value = maxValue;
    float frac = static_cast<float>(value) / static_cast<float>(maxValue);

    // Clear the dynamic interior (everything inside the static ticks).
    tft.fillCircle(cx, cy, r - 6, COL_DIAL_FACE);
    RTRACE_FILL_CIRCLE(cx, cy, r - 6, COL_DIAL_FACE);

    // Colour progress arc — short radial segments from 0 up to the value.
    float endDeg = frac * static_cast<float>(DIAL_SWEEP_DEG);
    for (float d = 0.0f; d <= endDeg; d += 6.0f) {
        float segFrac = (DIAL_SWEEP_DEG > 0)
                        ? (d / static_cast<float>(DIAL_SWEEP_DEG)) : 0.0f;
        uint16_t segCol = zoned ? dgZoneColor(segFrac) : needleColor;
        float deg = static_cast<float>(DIAL_START_DEG) + d;
        int16_t x1, y1, x2, y2;
        dgPolar(cx, cy, static_cast<float>(r - 10), deg, x1, y1);
        dgPolar(cx, cy, static_cast<float>(r - 7),  deg, x2, y2);
        tft.drawLine(x1, y1, x2, y2, segCol);
        RTRACE_LINE(x1, y1, x2, y2, segCol);
    }

    // Needle — slim triangle from the hub to the value angle.
    float needleDeg = static_cast<float>(DIAL_START_DEG) + endDeg;
    int16_t tipX, tipY;
    dgPolar(cx, cy, static_cast<float>(r - 12), needleDeg, tipX, tipY);
    int16_t baseLX, baseLY, baseRX, baseRY;
    dgPolar(cx, cy, 3.0f, needleDeg - 90.0f, baseLX, baseLY);
    dgPolar(cx, cy, 3.0f, needleDeg + 90.0f, baseRX, baseRY);
    tft.fillTriangle(tipX, tipY, baseLX, baseLY, baseRX, baseRY, needleColor);
    RTRACE_LINE(cx, cy, tipX, tipY, needleColor);

    // Hub.
    tft.fillCircle(cx, cy, 3, COL_WHITE);
    RTRACE_FILL_CIRCLE(cx, cy, 3, COL_WHITE);

    // Digital value in the bottom gap, unit just below it.
    tft.setTextColor(COL_WHITE, COL_DIAL_FACE);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.setTextPadding(r);
    tft.drawString(valueText, cx, cy + 15);
    RTRACE_TEXT(cx, cy + 15, valueText, COL_WHITE, COL_DIAL_FACE, 2, MC_DATUM);
    tft.setTextPadding(0);

    tft.setTextColor(COL_GRAY, COL_DIAL_FACE);
    tft.setTextSize(1);
    tft.drawString(unit, cx, cy + 30);
    RTRACE_TEXT(cx, cy + 30, unit, COL_GRAY, COL_DIAL_FACE, 1, MC_DATUM);

    tft.setTextDatum(TL_DATUM);
}

} // namespace ui
