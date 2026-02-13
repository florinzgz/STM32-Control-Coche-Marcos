// =============================================================================
// ESP32-S3 HMI — Safe Screen Implementation
// =============================================================================

#include "safe_screen.h"
#include "ui/ui_common.h"
#include <TFT_eSPI.h>
#include <cstdio>

extern TFT_eSPI tft;

void SafeScreen::onEnter() {
    needsRedraw_ = true;
    faultFlags_  = 0;
    prevFaultFlags_ = 0xFF;
    errorCode_   = 0;
    prevErrorCode_ = 0xFF;
}

void SafeScreen::onExit() {}

void SafeScreen::update(const vehicle::VehicleData& data) {
    faultFlags_ = data.heartbeat().faultFlags;
    errorCode_  = data.safety().errorCode;
}

void SafeScreen::draw() {
    if (needsRedraw_) {
        needsRedraw_ = false;

        tft.fillScreen(ui::COL_BG);

        // Full-width SAFE MODE banner
        tft.fillRect(0, 0, ui::SCREEN_W, 60, ui::COL_AMBER);
        tft.setTextColor(ui::COL_BLACK, ui::COL_AMBER);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("SAFE MODE", ui::SCREEN_W / 2, 30);

        // Explanation
        tft.setTextSize(1);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.drawString("Actuators inhibited", ui::SCREEN_W / 2, 90);
        tft.drawString("Controls disabled", ui::SCREEN_W / 2, 110);

        // Labels
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("FAULT FLAGS", ui::SCREEN_W / 2, 170);
        tft.drawString("ERROR CODE", ui::SCREEN_W / 2, 260);

        tft.setTextDatum(TL_DATUM);

        prevFaultFlags_ = faultFlags_ + 1;
        prevErrorCode_  = errorCode_ + 1;
    }

    // Fault flags (partial redraw)
    if (faultFlags_ != prevFaultFlags_) {
        prevFaultFlags_ = faultFlags_;

        char buf[ui::FMT_BUF_MED];
        snprintf(buf, sizeof(buf), "0x%02X", faultFlags_);

        tft.fillRect(60, 195, 200, 30, ui::COL_BG);
        tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(buf, ui::SCREEN_W / 2, 210);
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
    }

    // Error code (partial redraw)
    if (errorCode_ != prevErrorCode_) {
        prevErrorCode_ = errorCode_;

        char buf[ui::FMT_BUF_MED];
        snprintf(buf, sizeof(buf), "Code: %u", errorCode_);

        tft.fillRect(60, 285, 200, 30, ui::COL_BG);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(buf, ui::SCREEN_W / 2, 300);
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
    }
}
