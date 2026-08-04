// =============================================================================
// ESP32-S3 HMI — Gear Display Implementation (premium centre cluster)
//
// Renders the P / R / N / D1 / D2 selector as a row of compact, rounded
// "pills" centred between the two analog dials.  The active gear is shown as a
// bright filled pill with a glow border and a colour matched to its role:
//   P -> cyan, R -> red, N -> amber, D1/D2 -> green.
// Inactive gears are dim dark pills with grey lettering, so the active gear
// reads instantly from across the cabin (OEM cluster behaviour).
// =============================================================================

#include "gear_display.h"
#include "render_trace.h"
#include <cstdio>

namespace ui {

constexpr const char* GearDisplay::GEAR_LABELS[NUM_GEARS];

// File-local copy of the labels for the free-function pill renderer
// (GearDisplay::GEAR_LABELS is private).
static const char* const PILL_LABELS[NUM_GEARS] = { "P", "R", "N", "D1", "D2" };

// Per-gear accent colour for the active pill.
static inline uint16_t gearAccent(uint8_t idx) {
    switch (idx) {
        case 0: return COL_CYAN;    // P
        case 1: return COL_RED;     // R
        case 2: return COL_AMBER;   // N
        default: return COL_GREEN;  // D1 / D2
    }
}

// Left edge of pill i.
static inline int16_t gearCellX(uint8_t i) {
    return DGEAR_START_X + i * (DGEAR_CELL_W + DGEAR_CELL_GAP);
}

// -------------------------------------------------------------------------
// Draw a single gear pill in active or inactive style.
// -------------------------------------------------------------------------
static void drawGearPill(TFT_eSPI& tft, uint8_t idx, bool active) {
    int16_t x  = gearCellX(idx);
    int16_t cx = x + DGEAR_CELL_W / 2;
    int16_t cy = DGEAR_Y + DGEAR_H / 2;

    uint16_t fill   = active ? gearAccent(idx) : COL_GEAR_OFF;
    uint16_t border = active ? COL_WHITE : COL_GEAR_EDGE;
    uint16_t text   = active ? COL_BLACK : COL_GRAY;

    tft.fillRoundRect(x, DGEAR_Y, DGEAR_CELL_W, DGEAR_H, 6, fill);
    RTRACE_FILL_RECT(x, DGEAR_Y, DGEAR_CELL_W, DGEAR_H, fill);
    tft.drawRoundRect(x, DGEAR_Y, DGEAR_CELL_W, DGEAR_H, 6, border);
    RTRACE_DRAW_RECT(x, DGEAR_Y, DGEAR_CELL_W, DGEAR_H, border);

    tft.setTextColor(text, fill);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(PILL_LABELS[idx], cx, cy);
    RTRACE_TEXT(cx, cy, PILL_LABELS[idx], text, fill, 2, MC_DATUM);
}

// -------------------------------------------------------------------------
// Draw all gear pills in inactive state (call once on screen enter).
// -------------------------------------------------------------------------
void GearDisplay::drawStatic(TFT_eSPI& tft) {
    for (uint8_t i = 0; i < NUM_GEARS; ++i) {
        drawGearPill(tft, i, false);
    }
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}

// -------------------------------------------------------------------------
// Update applied/requested state.  The whole compact row is repainted because
// a mismatch is safety-significant and must never be hidden by differential
// pill updates.
// -------------------------------------------------------------------------
void GearDisplay::draw(TFT_eSPI& tft,
                       Gear applied, Gear previousApplied,
                       bool appliedValid, bool previousAppliedValid,
                       Gear requested, Gear previousRequested) {
    if (applied == previousApplied &&
        appliedValid == previousAppliedValid &&
        requested == previousRequested) {
        return;
    }

    const uint8_t appliedIdx = static_cast<uint8_t>(applied);
    for (uint8_t i = 0; i < NUM_GEARS; ++i) {
        drawGearPill(tft, i, appliedValid && i == appliedIdx);
    }

    char status[40];
    const uint8_t reqIdx = static_cast<uint8_t>(requested);
    const char* reqText = reqIdx < NUM_GEARS ? PILL_LABELS[reqIdx] : "?";
    const char* appText = appliedValid && appliedIdx < NUM_GEARS
                            ? PILL_LABELS[appliedIdx] : "--";
    snprintf(status, sizeof(status), "LEVER %s  APPLIED %s", reqText, appText);
    const bool mismatch = !appliedValid || requested != applied;
    tft.fillRect(CLUSTER_X, DGEAR_Y + DGEAR_H + 1, CLUSTER_W, 9, COL_BG);
    tft.setTextDatum(TC_DATUM);
    tft.setTextSize(1);
    tft.setTextColor(!appliedValid ? COL_AMBER :
                     (mismatch ? COL_RED : COL_GREEN), COL_BG);
    tft.drawString(status, CLUSTER_X + CLUSTER_W / 2, DGEAR_Y + DGEAR_H + 1);
    tft.setTextDatum(TL_DATUM);
}

} // namespace ui
