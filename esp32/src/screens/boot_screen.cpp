// =============================================================================
// ESP32-S3 HMI — Boot Screen Implementation
// =============================================================================

#include "boot_screen.h"
#include "ui/ui_common.h"
#include "hmi/obstacle_indicator.h"
#include "sensors/obstacle_sensor.h"
#include <TFT_eSPI.h>

extern TFT_eSPI tft;

void BootScreen::onEnter() {
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

        tft.fillScreen(ui::COL_BG);

        // Title
        tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("COCHE", ui::SCREEN_W / 2, ui::SCREEN_H / 2 - 40);
        tft.drawString("MARCOS", ui::SCREEN_W / 2, ui::SCREEN_H / 2);

        // Subtitle
        tft.setTextSize(1);
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.drawString("HMI v1.0", ui::SCREEN_W / 2, ui::SCREEN_H / 2 + 40);

        prevCanLinked_ = !canLinked_;  // Force status redraw
        prevSensorStatus_ = (sensorStatus_ == obstacle_sensor::SensorStatus::WAITING)
                            ? obstacle_sensor::SensorStatus::VALID
                            : obstacle_sensor::SensorStatus::WAITING;  // Force redraw
    }

    // CAN link status (partial redraw)
    if (canLinked_ != prevCanLinked_) {
        prevCanLinked_ = canLinked_;

        int16_t statusY = ui::SCREEN_H / 2 + 70;
        tft.fillRect(0, statusY - 10, ui::SCREEN_W, 20, ui::COL_BG);

        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        if (canLinked_) {
            tft.setTextColor(ui::COL_GREEN, ui::COL_BG);
            tft.drawString("CAN: LINKED", ui::SCREEN_W / 2, statusY);
        } else {
            tft.setTextColor(ui::COL_RED, ui::COL_BG);
            tft.drawString("CAN: WAITING...", ui::SCREEN_W / 2, statusY);
        }
        tft.setTextDatum(TL_DATUM);
    }

    // Obstacle sensor status (partial redraw)
    hmi::ObstacleIndicator::draw(tft, sensorStatus_, prevSensorStatus_);
    prevSensorStatus_ = sensorStatus_;
}
