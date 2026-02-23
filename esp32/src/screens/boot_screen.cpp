// =============================================================================
// ESP32-S3 HMI — Boot Screen Implementation
// =============================================================================

#include "boot_screen.h"
#include "ui/ui_common.h"
#include "ui/render_trace.h"
#include "hmi/obstacle_indicator.h"
#include "sensors/obstacle_sensor.h"
#include <TFT_eSPI.h>

extern TFT_eSPI tft;

void BootScreen::onEnter() {
    RTRACE_BEGIN_SCREEN("boot");
    needsRedraw_ = true;
    canLinked_    = false;
    prevCanLinked_ = false;
    sensorStatus_     = obstacle_sensor::SensorStatus::WAITING;
    prevSensorStatus_ = obstacle_sensor::SensorStatus::WAITING;
}

void BootScreen::onExit() {}

void BootScreen::update(const vehicle::VehicleData& data) {
    // CAN link is considered active if heartbeat timestamp is recent
    unsigned long now = millis();
    canLinked_ = (data.heartbeat().timestampMs > 0 &&
                  (now - data.heartbeat().timestampMs) < 500);

    // Obstacle sensor status
    sensorStatus_ = obstacle_sensor::getReading().status;
}

void BootScreen::draw() {
    if (needsRedraw_) {
        needsRedraw_ = false;

        RTRACE_SET_LAYER(0);
        tft.fillScreen(ui::COL_BG);
        RTRACE_FILL_SCREEN(ui::COL_BG);

        RTRACE_SET_LAYER(1);

        // Title
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("COCHE", ui::SCREEN_W / 2, ui::SCREEN_H / 2 - 40);
        RTRACE_TEXT(ui::SCREEN_W / 2, ui::SCREEN_H / 2 - 40, "COCHE",
                    ui::COL_WHITE, ui::COL_BG, 3, MC_DATUM);
        tft.drawString("MARCOS", ui::SCREEN_W / 2, ui::SCREEN_H / 2);
        RTRACE_TEXT(ui::SCREEN_W / 2, ui::SCREEN_H / 2, "MARCOS",
                    ui::COL_WHITE, ui::COL_BG, 3, MC_DATUM);

        // Subtitle
        tft.setTextSize(1);
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("HMI v1.0", ui::SCREEN_W / 2, ui::SCREEN_H / 2 + 40);
        RTRACE_TEXT(ui::SCREEN_W / 2, ui::SCREEN_H / 2 + 40, "HMI v1.0",
                    ui::COL_GRAY, ui::COL_BG, 1, MC_DATUM);

        prevCanLinked_ = !canLinked_;  // Force status redraw
        prevSensorStatus_ = (sensorStatus_ == obstacle_sensor::SensorStatus::WAITING)
                            ? obstacle_sensor::SensorStatus::VALID
                            : obstacle_sensor::SensorStatus::WAITING;  // Force redraw
    }

    // CAN link status (partial redraw)
    if (canLinked_ != prevCanLinked_) {
        prevCanLinked_ = canLinked_;

        int16_t statusY = ui::SCREEN_H / 2 + 70;
        RTRACE_SET_LAYER(2);
        tft.fillRect(0, statusY - 10, ui::SCREEN_W, 20, ui::COL_BG);
        RTRACE_FILL_RECT(0, statusY - 10, ui::SCREEN_W, 20, ui::COL_BG);

        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        if (canLinked_) {
            tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
            tft.drawString("CAN: LINKED", ui::SCREEN_W / 2, statusY);
            RTRACE_TEXT(ui::SCREEN_W / 2, statusY, "CAN: LINKED",
                        ui::COL_GREEN, ui::COL_BG, 1, MC_DATUM);
        } else {
            tft.setTextColor(ui::COL_RED, ui::COL_BG);
            tft.drawString("CAN: WAITING...", ui::SCREEN_W / 2, statusY);
            RTRACE_TEXT(ui::SCREEN_W / 2, statusY, "CAN: WAITING...",
                        ui::COL_RED, ui::COL_BG, 1, MC_DATUM);
        }
        tft.setTextDatum(TL_DATUM);
    }

    // Obstacle sensor status (partial redraw)
    RTRACE_SET_LAYER(2);
    hmi::ObstacleIndicator::draw(tft, sensorStatus_, prevSensorStatus_);
    prevSensorStatus_ = sensorStatus_;

    RTRACE_DUMP_IF_PENDING();
}
