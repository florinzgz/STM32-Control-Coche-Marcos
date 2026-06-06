// =============================================================================
// ESP32-S3 HMI — Battery Indicator Implementation
// =============================================================================

#include "battery_indicator.h"
#include "render_trace.h"
#include "ui_config.h"
#include <cstdio>

namespace ui {

// 24V lead-acid/LiFePO4 pack: 18.0V = 0%, 25.2V = 100%
static constexpr uint16_t BATT_MIN_RAW = cfg::BATT_VOLTAGE_MIN_RAW;
static constexpr uint16_t BATT_MAX_RAW = cfg::BATT_VOLTAGE_MAX_RAW;

// -------------------------------------------------------------------------
// Static outline
// -------------------------------------------------------------------------
void BatteryIndicator::drawStatic(TFT_eSPI& tft) {
    // Battery icon outline
    tft.drawRect(BAT_X, BAT_Y, BAT_W - 6, BAT_H, COL_WHITE);
    RTRACE_DRAW_RECT(BAT_X, BAT_Y, BAT_W - 6, BAT_H, COL_WHITE);
    // Battery terminal nub
    tft.fillRect(BAT_X + BAT_W - 6, BAT_Y + 8, 6, 12, COL_WHITE);
    RTRACE_FILL_RECT(BAT_X + BAT_W - 6, BAT_Y + 8, 6, 12, COL_WHITE);
}

// -------------------------------------------------------------------------
// Update percentage display
//
// Full single-pass repaint whenever the tile is invalidated (clear → frame →
// fill → text).  Flicker is avoided upstream by the tile-hash dirty engine —
// the battery tile is only marked dirty when the voltage/staleness actually
// changes or when an overlay (fault strip, ACK pill, etc.) clipped the area
// and explicitly invalidates it.  Crucially there is NO early-return here:
// an invalidated tile must always be fully repainted, otherwise an overlay
// that cleared the area would leave the battery blank with no text (the
// "battery disappears" symptom).
//
// prevVoltageRaw / prevStale are no longer used for suppression (the hash
// engine owns that decision); they are kept in the signature for ABI
// compatibility with the caller.
// -------------------------------------------------------------------------
void BatteryIndicator::draw(TFT_eSPI& tft,
                            uint16_t voltageRaw,
                            uint16_t prevVoltageRaw,
                            bool stale,
                            bool prevStale) {
    (void)prevVoltageRaw;
    (void)prevStale;

    int16_t innerX = BAT_X + 2;
    int16_t innerY = BAT_Y + 2;
    int16_t innerW = BAT_W - 10;
    int16_t innerH = BAT_H - 4;
    int16_t barInner = innerW - 4;

    // ---- Repair the static frame first ----
    // An overlay (e.g. the fault strip at y28..38) can clip the lower edge of
    // the battery outline.  Redrawing the outline + terminal nub here makes the
    // repaint self-contained so the marco is always intact after a restore.
    tft.drawRect(BAT_X, BAT_Y, BAT_W - 6, BAT_H, COL_WHITE);
    RTRACE_DRAW_RECT(BAT_X, BAT_Y, BAT_W - 6, BAT_H, COL_WHITE);
    tft.fillRect(BAT_X + BAT_W - 6, BAT_Y + 8, 6, 12, COL_WHITE);
    RTRACE_FILL_RECT(BAT_X + BAT_W - 6, BAT_Y + 8, 6, 12, COL_WHITE);

    // ---- Stale / unknown reading ----
    // When the battery frame (0x207) is older than the CAN-loss timeout, the
    // last voltage is no longer trustworthy.  Show "--" rather than a frozen
    // value (e.g. a stuck "100%").
    if (stale) {
        tft.fillRect(innerX, innerY, innerW, innerH, COL_BG);
        RTRACE_FILL_RECT(innerX, innerY, innerW, innerH, COL_BG);
        tft.setTextColor(COL_GRAY, COL_BG);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.setTextPadding(innerW - 4);
        tft.drawString("--", BAT_X + (BAT_W - 6) / 2, BAT_Y + BAT_H / 2);
        RTRACE_TEXT(BAT_X + (BAT_W - 6) / 2, BAT_Y + BAT_H / 2, "--",
                    COL_GRAY, COL_BG, 1, MC_DATUM);
        tft.setTextPadding(0);
        tft.setTextDatum(TL_DATUM);
        return;
    }

    uint8_t pct     = voltageToPercent(voltageRaw);

    // ---- Single clean pass (no overlapping partial fills) ----------------
    // The whole interior is rebuilt once per invalidation: clear → coloured
    // fill → text.  Calculation is unchanged.

    // Colour by level.
    uint16_t col;
    if (pct > cfg::BATT_COLOR_MID) {
        col = COL_GREEN;
    } else if (pct > cfg::BATT_COLOR_LOW) {
        col = COL_YELLOW;
    } else {
        col = COL_RED;
    }

    int16_t fillW = static_cast<int16_t>(
        (static_cast<int32_t>(pct) * barInner) / 100);

    // Clear the whole interior, then paint the charge fill in one pass.
    tft.fillRect(innerX, innerY, innerW, innerH, COL_BG);
    RTRACE_FILL_RECT(innerX, innerY, innerW, innerH, COL_BG);
    if (fillW > 0) {
        tft.fillRect(innerX, innerY, fillW + 2, innerH, col);
        RTRACE_FILL_RECT(innerX, innerY, fillW + 2, innerH, col);
    }

    // Percentage text centred over the bar.  Dark glyphs once the fill reaches
    // past the text centre, light glyphs otherwise (deterministic, no
    // cross-frame state needed because the interior is fully rebuilt here).
    // textCentre is measured in the inner-bar frame (from innerX) so it matches
    // the coordinate system fillW is computed in.
    char buf[FMT_BUF_SMALL];
    snprintf(buf, sizeof(buf), "%u%%", pct);
    int16_t textCentre = innerW / 2;          // relative to innerX (== fill origin)
    bool useDark = (fillW + 2) >= textCentre;
    uint16_t txtBg = useDark ? col : COL_BG;
    uint16_t txtFg = useDark ? COL_BLACK : COL_WHITE;
    tft.setTextColor(txtFg, txtBg);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.setTextPadding(innerW - 4);
    tft.drawString(buf, BAT_X + (BAT_W - 6) / 2, BAT_Y + BAT_H / 2);
    RTRACE_TEXT(BAT_X + (BAT_W - 6) / 2, BAT_Y + BAT_H / 2, buf,
                txtFg, txtBg, 1, MC_DATUM);
    tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// Voltage to percentage conversion
// -------------------------------------------------------------------------
uint8_t BatteryIndicator::voltageToPercent(uint16_t voltageRaw) {
    if (voltageRaw <= BATT_MIN_RAW) return 0;
    if (voltageRaw >= BATT_MAX_RAW) return 100;

    uint32_t range = BATT_MAX_RAW - BATT_MIN_RAW;
    uint32_t delta = voltageRaw - BATT_MIN_RAW;
    return static_cast<uint8_t>((delta * 100) / range);
}

} // namespace ui
