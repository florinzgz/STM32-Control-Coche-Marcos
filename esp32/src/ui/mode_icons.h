// =============================================================================
// ESP32-S3 HMI — Mode Icons
//
// Touch icons for drive modes: 4x4, 4x2, 360° axis rotation.
// Drawn as simple bordered rectangles with text labels.
// Active mode is highlighted with cyan, inactive with gray.
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef MODE_ICONS_H
#define MODE_ICONS_H

#include <TFT_eSPI.h>
#include "ui_common.h"

namespace ui {

/// Active drive mode flags (from CMD_MODE byte 0)
struct ModeState {
    bool is4x4     = false;   // bit 0 of mode_flags
    bool isTankTurn = false;  // bit 1 of mode_flags
};

class ModeIcons {
public:
    /// Draw all three mode icons (call once on screen enter)
    static void drawStatic(TFT_eSPI& tft);

    /// Update icon highlights based on mode state.
    /// Only redraws changed icons.
    static void draw(TFT_eSPI& tft,
                     const ModeState& current,
                     const ModeState& previous);

    /// Check if a touch point hits a mode icon.
    /// Returns: 0 = no hit, 1 = 4x4, 2 = 4x2, 3 = 360°
    static uint8_t hitTest(int16_t touchX, int16_t touchY);

private:
    static void drawIcon(TFT_eSPI& tft, int16_t x, const char* label,
                         bool active);
};

} // namespace ui

#endif // MODE_ICONS_H
