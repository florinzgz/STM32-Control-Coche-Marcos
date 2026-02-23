// =============================================================================
// ESP32-S3 HMI — Standby Screen Implementation
// =============================================================================

#include "standby_screen.h"
#include "ui/ui_common.h"
#include "ui/render_trace.h"
#include <TFT_eSPI.h>
#include <cstdio>
#include <cstring>

extern TFT_eSPI tft;

void StandbyScreen::onEnter() {
    RTRACE_BEGIN_SCREEN("standby");
    needsRedraw_ = true;
    faultFlags_  = 0;
    prevFaultFlags_ = 0xFF;
    memset(temps_, 0, sizeof(temps_));
    memset(prevTemps_, 0x7F, sizeof(prevTemps_));
}

void StandbyScreen::onExit() {}

void StandbyScreen::update(const vehicle::VehicleData& data) {
    faultFlags_ = data.heartbeat().faultFlags;
    for (uint8_t i = 0; i < 5; ++i) {
        temps_[i] = data.temp().temps[i];
    }
}

void StandbyScreen::draw() {
    if (needsRedraw_) {
        needsRedraw_ = false;

        RTRACE_SET_LAYER(0);
        tft.fillScreen(ui::COL_BG);
        RTRACE_FILL_SCREEN(ui::COL_BG);

        RTRACE_SET_LAYER(1);

        // System Ready header
        tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("READY", ui::SCREEN_W / 2, 60);
        RTRACE_TEXT(ui::SCREEN_W / 2, 60, "READY",
                    ui::COL_GREEN, ui::COL_BG, 3, MC_DATUM);

        // CAN linked indicator
        tft.setTextSize(1);
        tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
        tft.drawString("CAN: LINKED", ui::SCREEN_W / 2, 100);
        RTRACE_TEXT(ui::SCREEN_W / 2, 100, "CAN: LINKED",
                    ui::COL_GREEN, ui::COL_BG, 1, MC_DATUM);

        // Temperature header
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("TEMPERATURES", ui::SCREEN_W / 2, 160);
        RTRACE_TEXT(ui::SCREEN_W / 2, 160, "TEMPERATURES",
                    ui::COL_GRAY, ui::COL_BG, 1, MC_DATUM);

        // Temperature labels
        tft.setTextDatum(TL_DATUM);
        static const char* tempLabels[5] = { "FL:", "FR:", "RL:", "RR:", "AMB:" };
        for (uint8_t i = 0; i < 5; ++i) {
            tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
            tft.drawString(tempLabels[i], 80, 185 + i * 22);
            RTRACE_TEXT(80, 185 + i * 22, tempLabels[i],
                        ui::COL_GRAY, ui::COL_BG, 1, TL_DATUM);
        }

        // Fault flags header
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("FAULT FLAGS", ui::SCREEN_W / 2, 290);
        RTRACE_TEXT(ui::SCREEN_W / 2, 290, "FAULT FLAGS",
                    ui::COL_GRAY, ui::COL_BG, 1, MC_DATUM);

        tft.setTextDatum(TL_DATUM);

        prevFaultFlags_ = faultFlags_ + 1;  // Force redraw
        memset(prevTemps_, 0x7F, sizeof(prevTemps_));
    }

    RTRACE_SET_LAYER(2);

    // Temperature values (partial redraw)
    for (uint8_t i = 0; i < 5; ++i) {
        if (temps_[i] != prevTemps_[i]) {
            prevTemps_[i] = temps_[i];

            char buf[ui::FMT_BUF_SMALL];
            snprintf(buf, sizeof(buf), "%3d C", temps_[i]);

            tft.fillRect(140, 185 + i * 22, 80, 16, ui::COL_BG);
            RTRACE_FILL_RECT(140, 185 + i * 22, 80, 16, ui::COL_BG);
            tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
            tft.setTextSize(1);
            tft.drawString(buf, 140, 185 + i * 22);
            RTRACE_TEXT(140, 185 + i * 22, buf,
                        ui::COL_WHITE, ui::COL_BG, 1, TL_DATUM);
        }
    }

    // Fault flags (partial redraw)
    if (faultFlags_ != prevFaultFlags_) {
        prevFaultFlags_ = faultFlags_;

        tft.fillRect(40, 300, 240, 20, ui::COL_BG);
        RTRACE_FILL_RECT(40, 300, 240, 20, ui::COL_BG);

        tft.setTextDatum(MC_DATUM);
        if (faultFlags_ == 0) {
            tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
            tft.setTextSize(1);
            tft.drawString("NO FAULTS", ui::SCREEN_W / 2, 308);
            RTRACE_TEXT(ui::SCREEN_W / 2, 308, "NO FAULTS",
                        ui::COL_GREEN, ui::COL_BG, 1, MC_DATUM);
        } else {
            char buf[ui::FMT_BUF_MED];
            snprintf(buf, sizeof(buf), "FLAGS: 0x%02X", faultFlags_);
            tft.setTextColor(ui::COL_AMBER, ui::COL_BG);
            tft.setTextSize(1);
            tft.drawString(buf, ui::SCREEN_W / 2, 308);
            RTRACE_TEXT(ui::SCREEN_W / 2, 308, buf,
                        ui::COL_AMBER, ui::COL_BG, 1, MC_DATUM);
        }
        tft.setTextDatum(TL_DATUM);
    }

    RTRACE_DUMP_IF_PENDING();
}
