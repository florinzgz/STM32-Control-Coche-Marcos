// =============================================================================
// ESP32-S3 HMI — Safe Screen Implementation
// =============================================================================

#include "safe_screen.h"
#include "ui/ui_common.h"
#include "ui/render_trace.h"
#include <TFT_eSPI.h>
#include <cstdio>

extern TFT_eSPI tft;

void SafeScreen::onEnter() {
    RTRACE_BEGIN_SCREEN("safe");
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

        RTRACE_SET_LAYER(0);
        tft.fillScreen(ui::COL_BG);
        RTRACE_FILL_SCREEN(ui::COL_BG);

        RTRACE_SET_LAYER(1);

        // Full-width SAFE MODE banner
        tft.fillRect(0, 0, ui::SCREEN_W, 60, ui::COL_AMBER);
        RTRACE_FILL_RECT(0, 0, ui::SCREEN_W, 60, ui::COL_AMBER);
        tft.setTextColor(ui::COL_BLACK, ui::COL_AMBER);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("SAFE MODE", ui::SCREEN_W / 2, 30);
        RTRACE_TEXT(ui::SCREEN_W / 2, 30, "SAFE MODE",
                    ui::COL_BLACK, ui::COL_AMBER, 3, MC_DATUM);

        // Explanation
        tft.setTextSize(1);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.drawString("Actuators inhibited", ui::SCREEN_W / 2, 90);
        RTRACE_TEXT(ui::SCREEN_W / 2, 90, "Actuators inhibited",
                    ui::COL_WHITE, ui::COL_BG, 1, MC_DATUM);
        tft.drawString("Controls disabled", ui::SCREEN_W / 2, 110);
        RTRACE_TEXT(ui::SCREEN_W / 2, 110, "Controls disabled",
                    ui::COL_WHITE, ui::COL_BG, 1, MC_DATUM);

        // Labels
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("FAULT FLAGS", ui::SCREEN_W / 2, 170);
        RTRACE_TEXT(ui::SCREEN_W / 2, 170, "FAULT FLAGS",
                    ui::COL_GRAY, ui::COL_BG, 1, MC_DATUM);
        tft.drawString("ERROR CODE", ui::SCREEN_W / 2, 260);
        RTRACE_TEXT(ui::SCREEN_W / 2, 260, "ERROR CODE",
                    ui::COL_GRAY, ui::COL_BG, 1, MC_DATUM);

        tft.setTextDatum(TL_DATUM);

        prevFaultFlags_ = faultFlags_ + 1;
        prevErrorCode_  = errorCode_ + 1;
    }

    RTRACE_SET_LAYER(2);

    // Fault flags (partial redraw)
    if (faultFlags_ != prevFaultFlags_) {
        prevFaultFlags_ = faultFlags_;

        char buf[ui::FMT_BUF_MED];
        snprintf(buf, sizeof(buf), "0x%02X", faultFlags_);

        tft.fillRect(60, 195, 200, 30, ui::COL_BG);
        RTRACE_FILL_RECT(60, 195, 200, 30, ui::COL_BG);
        tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(buf, ui::SCREEN_W / 2, 210);
        RTRACE_TEXT(ui::SCREEN_W / 2, 210, buf,
                    ui::COL_AMBER, ui::COL_BG, 2, MC_DATUM);
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
    }

    // Error code (partial redraw)
    if (errorCode_ != prevErrorCode_) {
        prevErrorCode_ = errorCode_;

        char buf[ui::FMT_BUF_MED];
        snprintf(buf, sizeof(buf), "Code: %u", errorCode_);

        tft.fillRect(60, 285, 200, 30, ui::COL_BG);
        RTRACE_FILL_RECT(60, 285, 200, 30, ui::COL_BG);
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(buf, ui::SCREEN_W / 2, 300);
        RTRACE_TEXT(ui::SCREEN_W / 2, 300, buf,
                    ui::COL_WHITE, ui::COL_BG, 2, MC_DATUM);
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
    }

    RTRACE_DUMP_IF_PENDING();
}
