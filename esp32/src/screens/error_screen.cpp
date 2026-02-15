// =============================================================================
// ESP32-S3 HMI — Error Screen Implementation
// =============================================================================

#include "error_screen.h"
#include "ui/ui_common.h"
#include <TFT_eSPI.h>
#include <cstdio>

extern TFT_eSPI tft;

void ErrorScreen::onEnter() {
    needsRedraw_ = true;
    faultFlags_  = 0;
    prevFaultFlags_ = 0xFF;
    errorCode_   = 0;
    prevErrorCode_ = 0xFF;
    diagCode_    = 0;
    prevDiagCode_ = 0xFF;
    diagSubsystem_ = 0;
}

void ErrorScreen::onExit() {}

void ErrorScreen::update(const vehicle::VehicleData& data) {
    faultFlags_    = data.heartbeat().faultFlags;
    errorCode_     = data.safety().errorCode;
    diagCode_      = data.diag().errorCode;
    diagSubsystem_ = data.diag().subsystem;
}

void ErrorScreen::draw() {
    if (needsRedraw_) {
        needsRedraw_ = false;

        // Red background for error state
        tft.fillScreen(ui::COL_RED);

        // SYSTEM ERROR banner
        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("SYSTEM ERROR", ui::SCREEN_W / 2, 40);

        // Manual reset instruction
        tft.setTextSize(1);
        tft.drawString("Manual reset required", ui::SCREEN_W / 2, 80);

        // Labels
        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.drawString("FAULT FLAGS", ui::SCREEN_W / 2, 110);
        tft.drawString("SAFETY ERROR", ui::SCREEN_W / 2, 170);
        tft.drawString("DIAGNOSTIC", ui::SCREEN_W / 2, 230);

        tft.setTextDatum(TL_DATUM);

        prevFaultFlags_ = faultFlags_ + 1;
        prevErrorCode_  = errorCode_ + 1;
        prevDiagCode_   = diagCode_ + 1;
    }

    // Fault flags
    if (faultFlags_ != prevFaultFlags_) {
        prevFaultFlags_ = faultFlags_;

        char buf[ui::FMT_BUF_MED];
        snprintf(buf, sizeof(buf), "0x%02X", faultFlags_);

        tft.fillRect(60, 125, 200, 30, ui::COL_RED);
        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(buf, ui::SCREEN_W / 2, 140);
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
    }

    // Safety error code
    if (errorCode_ != prevErrorCode_) {
        prevErrorCode_ = errorCode_;

        char buf[ui::FMT_BUF_MED];
        snprintf(buf, sizeof(buf), "Code: %u", errorCode_);

        tft.fillRect(60, 185, 200, 30, ui::COL_RED);
        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(buf, ui::SCREEN_W / 2, 200);
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
    }

    // Diagnostic info
    if (diagCode_ != prevDiagCode_) {
        prevDiagCode_ = diagCode_;

        char buf[ui::FMT_BUF_LARGE];
        snprintf(buf, sizeof(buf), "E:%u S:%u", diagCode_, diagSubsystem_);

        tft.fillRect(60, 245, 200, 30, ui::COL_RED);
        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(buf, ui::SCREEN_W / 2, 260);
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
    }
}
