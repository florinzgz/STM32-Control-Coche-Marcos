// =============================================================================
// ESP32-S3 HMI — Mode Icons Implementation
// =============================================================================

#include "mode_icons.h"
#include "render_trace.h"

namespace ui {

// -------------------------------------------------------------------------
// Draw all icons in default (inactive) state
// -------------------------------------------------------------------------
void ModeIcons::drawStatic(TFT_eSPI& tft) {
    drawIcon(tft, ICON_4X4_X, "4x4", false);
    drawIcon(tft, ICON_4X2_X, "4x2", false);
    drawIcon(tft, ICON_360_X, "360", false);
}

// -------------------------------------------------------------------------
// Update icon highlights
// -------------------------------------------------------------------------
void ModeIcons::draw(TFT_eSPI& tft,
                     const ModeState& current,
                     const ModeState& previous) {
    // 4x4 icon
    if (current.is4x4 != previous.is4x4) {
        drawIcon(tft, ICON_4X4_X, "4x4", current.is4x4);
    }

    // 4x2 icon (inverse of 4x4)
    bool cur4x2 = !current.is4x4;
    bool prev4x2 = !previous.is4x4;
    if (cur4x2 != prev4x2) {
        drawIcon(tft, ICON_4X2_X, "4x2", cur4x2);
    }

    // 360° icon
    if (current.isTankTurn != previous.isTankTurn) {
        drawIcon(tft, ICON_360_X, "360", current.isTankTurn);
    }
}

// -------------------------------------------------------------------------
// Hit test for touch input — only 360° icon is touchable.
// 4x4/4x2 are display-only (controlled by physical traction switch).
// -------------------------------------------------------------------------
uint8_t ModeIcons::hitTest(int16_t touchX, int16_t touchY) {
    if (touchY < ICON_Y || touchY > ICON_Y + ICON_H) return 0;

    // Only 360° tank turn is still touch-selectable
    if (touchX >= ICON_360_X && touchX <= ICON_360_X + ICON_W) return 3;

    return 0;
}

// -------------------------------------------------------------------------
// Draw a single icon.
//
// Two visual languages, same geometry (touch zones unchanged):
//   * 4x4 / 4x2  → drivetrain STATUS BADGES (rounded, flat, a left accent
//                  stripe when active).  Deliberately do NOT look pressable.
//   * 360        → interactive BUTTON (persistent accent outline + a small
//                  rotation glyph) so it clearly reads as tappable.
// -------------------------------------------------------------------------
void ModeIcons::drawIcon(TFT_eSPI& tft, int16_t x, const char* label,
                         bool active) {
    const bool isButton = (label[0] == '3');   // only the "360" icon

    // Always clear the full icon cell first (prevents stale pixels when the
    // rounded styles leave the square corners untouched).
    tft.fillRect(x, ICON_Y, ICON_W, ICON_H, COL_BG);
    RTRACE_FILL_RECT(x, ICON_Y, ICON_W, ICON_H, COL_BG);

    if (isButton) {
        // ---- Interactive 360° button ----
        uint16_t face   = active ? COL_CYAN : COL_GEAR_OFF;
        uint16_t txtCol = active ? COL_BLACK : COL_CYAN;

        tft.fillRoundRect(x, ICON_Y, ICON_W, ICON_H, 5, face);
        RTRACE_FILL_RECT(x, ICON_Y, ICON_W, ICON_H, face);
        // Persistent accent outline = "this is tappable".
        tft.drawRoundRect(x, ICON_Y, ICON_W, ICON_H, 5, COL_CYAN);
        RTRACE_DRAW_RECT(x, ICON_Y, ICON_W, ICON_H, COL_CYAN);
        tft.drawRoundRect(x + 1, ICON_Y + 1, ICON_W - 2, ICON_H - 2, 4,
                          active ? COL_WHITE : COL_DIAL_RING);

        // Small rotation arc hint on the left, then the "360" label.
        int16_t gx = x + 9;
        int16_t gy = ICON_Y + ICON_H / 2;
        tft.drawCircle(gx, gy, 5, txtCol);
        tft.drawCircle(gx, gy, 4, txtCol);
        RTRACE_CIRCLE(gx, gy, 5, txtCol);
        tft.fillTriangle(gx + 5, gy - 6, gx + 5, gy, gx + 9, gy - 3, txtCol);

        tft.setTextColor(txtCol, face);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("360", x + ICON_W / 2 + 6, gy);
        RTRACE_TEXT(x + ICON_W / 2 + 6, gy, "360", txtCol, face, 1, MC_DATUM);
        tft.setTextDatum(TL_DATUM);
        return;
    }

    // ---- Drivetrain status badge (4x4 / 4x2) ----
    uint16_t face   = active ? COL_DIAL_FACE : COL_BG;
    uint16_t txtCol = active ? COL_WHITE : COL_DARK_GRAY;
    uint16_t edge   = active ? COL_GREEN : COL_DIAL_RING;

    tft.fillRoundRect(x, ICON_Y, ICON_W, ICON_H, 4, face);
    RTRACE_FILL_RECT(x, ICON_Y, ICON_W, ICON_H, face);
    tft.drawRoundRect(x, ICON_Y, ICON_W, ICON_H, 4, edge);
    RTRACE_DRAW_RECT(x, ICON_Y, ICON_W, ICON_H, edge);

    // Left accent stripe marks the active drivetrain mode (status, not button).
    if (active) {
        tft.fillRect(x + 2, ICON_Y + 3, 3, ICON_H - 6, COL_GREEN);
        RTRACE_FILL_RECT(x + 2, ICON_Y + 3, 3, ICON_H - 6, COL_GREEN);
    }

    tft.setTextColor(txtCol, face);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(label, x + ICON_W / 2 + 2, ICON_Y + ICON_H / 2);
    RTRACE_TEXT(x + ICON_W / 2 + 2, ICON_Y + ICON_H / 2, label,
                txtCol, face, 1, MC_DATUM);
    tft.setTextDatum(TL_DATUM);
}

} // namespace ui
