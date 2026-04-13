// =============================================================================
// ESP32-S3 HMI — Relay Status Indicator Widget
//
// Compact 3-letter indicator showing real-time relay GPIO command state:
//   M = MAIN (PC10), T = TRACTION (PC11), D = DIRECTION (PC12)
//
// Color logic:
//   Green  — relay commanded ON and sequence complete
//   Amber  — relay commanded ON but sequence in progress (bit7 = 0)
//   Gray   — relay commanded OFF
//
// Displayed in DriveScreen gear bar zone (right side).
// Data source: HeartbeatData.relayStatus (CAN 0x001, byte 5).
//
// Rendering cost: single text call with 3 colored characters.
// No dynamic allocation, O(1) per frame.
// =============================================================================

#ifndef RELAY_INDICATOR_H
#define RELAY_INDICATOR_H

#include <TFT_eSPI.h>
#include <cstdint>
#include "ui_common.h"
#include "ui_config.h"

namespace ui {

// Layout constants for relay indicator in DriveScreen gear bar
namespace cfg {
    inline constexpr int16_t REL_IND_X     = 430;   // Right side of gear bar
    inline constexpr int16_t REL_IND_Y     = GEAR_Y;
    inline constexpr int16_t REL_IND_W     = 50;
    inline constexpr int16_t REL_IND_H     = GEAR_H;
} // namespace cfg

class RelayIndicator {
public:
    /// Draw relay indicator.  Only redraws if relayStatus byte changed.
    /// relayStatus: bit0=MAIN, bit1=TRAC, bit2=DIR, bit7=SEQ_COMPLETE
    static void draw(TFT_eSPI& tft, uint8_t relayStatus, uint8_t prevRelayStatus) {
        if (relayStatus == prevRelayStatus) return;

        const bool seqComplete = (relayStatus & 0x80U) != 0;
        const bool mainOn      = (relayStatus & 0x01U) != 0;
        const bool tracOn      = (relayStatus & 0x02U) != 0;
        const bool dirOn       = (relayStatus & 0x04U) != 0;

        // Clear background
        tft.fillRect(cfg::REL_IND_X, cfg::REL_IND_Y,
                     cfg::REL_IND_W, cfg::REL_IND_H, COL_BG);

        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);

        // Y center (text size 1 is 8px high, bar is 20px, so offset by 6)
        const int16_t textY = cfg::REL_IND_Y + (cfg::REL_IND_H - 8) / 2;

        // Draw "M" for MAIN relay
        tft.setTextColor(relayColor(mainOn, seqComplete), COL_BG);
        tft.drawChar('M', cfg::REL_IND_X, textY);

        // Draw "T" for TRACTION relay
        tft.setTextColor(relayColor(tracOn, seqComplete), COL_BG);
        tft.drawChar('T', cfg::REL_IND_X + 14, textY);

        // Draw "D" for DIRECTION relay
        tft.setTextColor(relayColor(dirOn, seqComplete), COL_BG);
        tft.drawChar('D', cfg::REL_IND_X + 28, textY);
    }

    /// Static label — draw "REL" prefix (call once on screen enter).
    /// Intentionally empty: M/T/D letters are self-explanatory in the gear bar
    /// context. Kept for API parity with GearDisplay, ModeIcons, etc.
    static void drawStatic(TFT_eSPI& /* tft */) {}

private:
    /// Color logic for a single relay letter
    static uint16_t relayColor(bool relayOn, bool seqComplete) {
        if (!relayOn) return COL_GRAY;
        return seqComplete ? COL_GREEN : COL_AMBER;
    }
};

} // namespace ui

#endif // RELAY_INDICATOR_H
