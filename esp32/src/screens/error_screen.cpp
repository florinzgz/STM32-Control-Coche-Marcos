// =============================================================================
// ESP32-S3 HMI — Error Screen Implementation
// =============================================================================

#include "error_screen.h"
#include "ui/ui_common.h"
#include "ui/render_trace.h"
#include <TFT_eSPI.h>
#include <cstdio>

extern TFT_eSPI tft;

void ErrorScreen::onEnter() {
    RTRACE_BEGIN_SCREEN("error");
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

        RTRACE_SET_LAYER(0);
        // Red background for error state
        tft.fillScreen(ui::COL_RED);
        RTRACE_FILL_SCREEN(ui::COL_RED);

        RTRACE_SET_LAYER(1);

        // SYSTEM ERROR banner
        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("SYSTEM ERROR", ui::SCREEN_W / 2, 40);
        RTRACE_TEXT(ui::SCREEN_W / 2, 40, "SYSTEM ERROR",
                    ui::COL_WHITE, ui::COL_RED, 3, MC_DATUM);

        // Manual reset instruction
        tft.setTextSize(1);
        tft.drawString("Manual reset required", ui::SCREEN_W / 2, 80);
        RTRACE_TEXT(ui::SCREEN_W / 2, 80, "Manual reset required",
                    ui::COL_WHITE, ui::COL_RED, 1, MC_DATUM);

        // Labels
        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.drawString("FAULT FLAGS", ui::SCREEN_W / 2, 110);
        RTRACE_TEXT(ui::SCREEN_W / 2, 110, "FAULT FLAGS",
                    ui::COL_WHITE, ui::COL_RED, 1, MC_DATUM);
        tft.drawString("SAFETY ERROR", ui::SCREEN_W / 2, 170);
        RTRACE_TEXT(ui::SCREEN_W / 2, 170, "SAFETY ERROR",
                    ui::COL_WHITE, ui::COL_RED, 1, MC_DATUM);
        tft.drawString("DIAGNOSTIC", ui::SCREEN_W / 2, 230);
        RTRACE_TEXT(ui::SCREEN_W / 2, 230, "DIAGNOSTIC",
                    ui::COL_WHITE, ui::COL_RED, 1, MC_DATUM);

        tft.setTextDatum(TL_DATUM);

        prevFaultFlags_ = faultFlags_ + 1;
        prevErrorCode_  = errorCode_ + 1;
        prevDiagCode_   = diagCode_ + 1;
    }

    RTRACE_SET_LAYER(2);

    // Fault flags
    if (faultFlags_ != prevFaultFlags_) {
        prevFaultFlags_ = faultFlags_;

        char buf[ui::FMT_BUF_MED];
        snprintf(buf, sizeof(buf), "0x%02X", faultFlags_);

        tft.fillRect(60, 125, 200, 30, ui::COL_RED);
        RTRACE_FILL_RECT(60, 125, 200, 30, ui::COL_RED);
        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(buf, ui::SCREEN_W / 2, 140);
        RTRACE_TEXT(ui::SCREEN_W / 2, 140, buf,
                    ui::COL_WHITE, ui::COL_RED, 2, MC_DATUM);
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
    }

    // Safety error code
    if (errorCode_ != prevErrorCode_) {
        prevErrorCode_ = errorCode_;

        char buf[ui::FMT_BUF_MED];
        snprintf(buf, sizeof(buf), "Code: %u", errorCode_);

        tft.fillRect(60, 185, 200, 30, ui::COL_RED);
        RTRACE_FILL_RECT(60, 185, 200, 30, ui::COL_RED);
        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(buf, ui::SCREEN_W / 2, 200);
        RTRACE_TEXT(ui::SCREEN_W / 2, 200, buf,
                    ui::COL_WHITE, ui::COL_RED, 2, MC_DATUM);
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
    }

    // Diagnostic info
    if (diagCode_ != prevDiagCode_) {
        prevDiagCode_ = diagCode_;

        char buf[ui::FMT_BUF_LARGE];
        snprintf(buf, sizeof(buf), "E:%u S:%u", diagCode_, diagSubsystem_);

        tft.fillRect(60, 245, 200, 30, ui::COL_RED);
        RTRACE_FILL_RECT(60, 245, 200, 30, ui::COL_RED);
        tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(buf, ui::SCREEN_W / 2, 260);
        RTRACE_TEXT(ui::SCREEN_W / 2, 260, buf,
                    ui::COL_WHITE, ui::COL_RED, 2, MC_DATUM);
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
    }

    RTRACE_DUMP_IF_PENDING();
}
