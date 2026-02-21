// =============================================================================
// ESP32-S3 HMI — Drive Screen Implementation
//
// Implements the main driving dashboard with full telemetry display.
// Uses partial redraw: only elements that changed since the previous
// frame are redrawn, keeping render time <5 ms.
//
// Layout zones (480×320 landscape):
//   Top bar (0–40):      [4x4] [4x2] [360°]              [BAT XX%]
//   Sensor (40–85):      frontal obstacle distance + proximity bar
//   Center (85–230):     car body + 4 wheels (torque/temp) + steering gauge
//   Speed (230–270):     large centered speed (km/h)
//   Pedal (270–300):     pedal bar 0–100% with gradient
//   Gears (300–320):     [P] [R] [N] [D1] [D2] — flat text
//
// No String class. No heap allocation. No recursion.
// All format buffers are fixed-size stack arrays with snprintf().
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#include "drive_screen.h"
#include "ui/car_renderer.h"
#include "ui/pedal_bar.h"
#include "ui/gear_display.h"
#include "ui/battery_indicator.h"
#include "ui/mode_icons.h"
#include "ui/led_toggle.h"
#include "ui/obstacle_sensor.h"
#include "ui/runtime_monitor.h"
#include <cstdio>
#include <cstring>

// External TFT instance (initialized in main.cpp)
extern TFT_eSPI tft;

// -------------------------------------------------------------------------
// onEnter — called when transitioning to this screen
// -------------------------------------------------------------------------
void DriveScreen::onEnter() {
    needsFullRedraw_ = true;

    // Zero out previous values to force full redraw
    memset(prevTraction_, 0, sizeof(prevTraction_));
    memset(prevTemp_, 0, sizeof(prevTemp_));
    prevSteeringRaw_ = 0;
    prevSpeedAvgRaw_ = 0;
    prevBattVoltRaw_ = 0;
    prevPedalPct_    = 0;
    prevGear_        = ui::Gear::P;
    prevMode_        = {};
    prevObstacleCm_  = 0;
    prevLedOn_       = false;
}

// -------------------------------------------------------------------------
// onExit — called when leaving this screen
// -------------------------------------------------------------------------
void DriveScreen::onExit() {
    // Nothing to clean up — no dynamic resources
}

// -------------------------------------------------------------------------
// update — read vehicle data into current-frame cache
// -------------------------------------------------------------------------
void DriveScreen::update(const vehicle::VehicleData& data) {
    // Traction (torque per wheel)
    for (uint8_t i = 0; i < 4; ++i) {
        curTraction_[i] = data.traction().scale[i];
    }

    // Temperature per wheel (first 4 of tempMap)
    for (uint8_t i = 0; i < 4; ++i) {
        curTemp_[i] = data.tempMap().temps[i];
    }

    // Steering angle
    curSteeringRaw_ = data.steering().angleRaw;

    // Average speed (all 4 wheels, raw 0.1 km/h units)
    // Max sum: 4 × 65535 = 262140, fits in uint32_t
    // Max average: 65535, fits in uint16_t
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        sum += data.speed().raw[i];
    }
    curSpeedAvgRaw_ = static_cast<uint16_t>(sum / 4);

    // Battery voltage
    curBattVoltRaw_ = data.battery().voltageRaw;

    // Pedal/throttle — derived from traction average as display hint
    // (actual throttle command is sent separately via CMD_THROTTLE)
    uint16_t tractionSum = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        tractionSum += data.traction().scale[i];
    }
    curPedalPct_ = static_cast<uint8_t>(tractionSum / 4);

    // Gear — the STM32 does not echo gear back via CAN; CMD_MODE is ESP32→STM32
    // only. Default to N (neutral/unknown) since no gear has been commanded yet.
    curGear_ = ui::Gear::N;

    // Mode flags
    curMode_.is4x4 = false;
    curMode_.isTankTurn = false;

    // Obstacle sensor
    curObstacleCm_ = data.obstacle().distanceCm;

    // LED relay state from STM32
    curLedOn_ = data.lights().relayOn;
}

// -------------------------------------------------------------------------
// draw — render changed elements to TFT
// -------------------------------------------------------------------------
void DriveScreen::draw() {
    if (needsFullRedraw_) {
        needsFullRedraw_ = false;
        RTMON_FULL_REDRAW();

        // Clear entire screen
        tft.fillScreen(ui::COL_BG);

        // Draw all static elements (in layout order top→bottom)
        ui::ModeIcons::drawStatic(tft);
        ui::LedToggle::drawStatic(tft);
        ui::BatteryIndicator::drawStatic(tft);
        ui::ObstacleSensor::drawStatic(tft);
        ui::CarRenderer::drawStatic(tft);
        ui::PedalBar::drawStatic(tft);
        ui::GearDisplay::drawStatic(tft);

        // Speed label (below speed value)
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.setTextSize(1);
        tft.setTextDatum(TC_DATUM);
        tft.drawString("km/h", ui::SCREEN_W / 2, ui::SPEED_Y + 26);
        tft.setTextDatum(TL_DATUM);

        // "360°" label centered above steering gauge
        tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
        tft.setTextSize(1);
        tft.setTextDatum(TC_DATUM);
        tft.drawString("360", ui::STEER_CX, ui::STEER_CY - ui::STEER_RADIUS - 12);
        tft.setTextDatum(TL_DATUM);

        // Force draw of all dynamic elements
        prevSpeedAvgRaw_ = curSpeedAvgRaw_ + 1;  // Force mismatch
        prevBattVoltRaw_ = curBattVoltRaw_ + 1;
        prevPedalPct_    = curPedalPct_ + 1;
        prevGear_        = (curGear_ == ui::Gear::P) ? ui::Gear::N : ui::Gear::P;
        prevSteeringRaw_ = curSteeringRaw_ + 10;
        prevObstacleCm_  = curObstacleCm_ + 1;
        for (uint8_t i = 0; i < 4; ++i) {
            prevTraction_[i] = curTraction_[i] + 1;
            prevTemp_[i]     = curTemp_[i] + 1;
        }
        prevMode_.is4x4     = !curMode_.is4x4;
        prevMode_.isTankTurn = !curMode_.isTankTurn;
        prevLedOn_          = !curLedOn_;
    }

    // Partial redraw: only changed elements

    // Speed (in its own zone, 230–270px)
    if (curSpeedAvgRaw_ != prevSpeedAvgRaw_) {
        RTMON_ZONE_REDRAW(rtmon::Zone::SPEED);
    }
    drawSpeed();

    // Obstacle sensor (40–85px)
    if (curObstacleCm_ != prevObstacleCm_) {
        RTMON_ZONE_REDRAW(rtmon::Zone::OBSTACLE);
    }
    ui::ObstacleSensor::draw(tft, curObstacleCm_, prevObstacleCm_);

    // Wheels (torque + temperature)
    {
        bool carDirty = false;
        for (uint8_t i = 0; i < 4; ++i) {
            if (curTraction_[i] != prevTraction_[i] || curTemp_[i] != prevTemp_[i]) {
                carDirty = true;
                break;
            }
        }
        if (carDirty || curSteeringRaw_ != prevSteeringRaw_) {
            RTMON_ZONE_REDRAW(rtmon::Zone::CAR);
        }
    }
    ui::CarRenderer::drawWheels(tft, vehicle::TractionData{
        {curTraction_[0], curTraction_[1], curTraction_[2], curTraction_[3]}, 0},
        vehicle::TempMapData{
        {curTemp_[0], curTemp_[1], curTemp_[2], curTemp_[3], 0}, 0},
        prevTraction_, prevTemp_);

    // Steering circular gauge (right side)
    ui::CarRenderer::drawSteering(tft, curSteeringRaw_, prevSteeringRaw_);

    // Battery (part of top bar zone)
    if (curBattVoltRaw_ != prevBattVoltRaw_) {
        RTMON_ZONE_REDRAW(rtmon::Zone::TOP_BAR);
    }
    ui::BatteryIndicator::draw(tft, curBattVoltRaw_, prevBattVoltRaw_);

    // Gear
    if (curGear_ != prevGear_) {
        RTMON_ZONE_REDRAW(rtmon::Zone::GEAR);
    }
    ui::GearDisplay::draw(tft, curGear_, prevGear_);

    // Pedal bar
    if (curPedalPct_ != prevPedalPct_) {
        RTMON_ZONE_REDRAW(rtmon::Zone::PEDAL);
    }
    ui::PedalBar::draw(tft, curPedalPct_, prevPedalPct_);

    // Mode icons (part of top bar zone)
    if (curMode_.is4x4 != prevMode_.is4x4 || curMode_.isTankTurn != prevMode_.isTankTurn) {
        RTMON_ZONE_REDRAW(rtmon::Zone::TOP_BAR);
    }
    ui::ModeIcons::draw(tft, curMode_, prevMode_);

    // LED toggle button (part of top bar zone)
    if (curLedOn_ != prevLedOn_) {
        RTMON_ZONE_REDRAW(rtmon::Zone::TOP_BAR);
    }
    ui::LedToggle::draw(tft, curLedOn_, prevLedOn_);

    // Copy current values to previous for next frame
    memcpy(prevTraction_, curTraction_, sizeof(prevTraction_));
    memcpy(prevTemp_, curTemp_, sizeof(prevTemp_));
    prevSteeringRaw_ = curSteeringRaw_;
    prevSpeedAvgRaw_ = curSpeedAvgRaw_;
    prevBattVoltRaw_ = curBattVoltRaw_;
    prevPedalPct_    = curPedalPct_;
    prevGear_        = curGear_;
    prevMode_        = curMode_;
    prevObstacleCm_  = curObstacleCm_;
    prevLedOn_       = curLedOn_;
}

// -------------------------------------------------------------------------
// Speed display helper — in its own zone (230–270px), NOT inside car
// -------------------------------------------------------------------------
void DriveScreen::drawSpeed() {
    if (curSpeedAvgRaw_ == prevSpeedAvgRaw_) return;

    // Convert raw (0.1 km/h) to display
    uint16_t intPart  = curSpeedAvgRaw_ / 10;
    uint16_t fracPart = curSpeedAvgRaw_ % 10;

    char buf[ui::FMT_BUF_MED];
    snprintf(buf, sizeof(buf), "%u.%u", intPart, fracPart);

    // Clear speed area
    tft.fillRect(0, ui::SPEED_Y, ui::SCREEN_W, 24, ui::COL_BG);

    // Draw speed value — large centered text
    tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
    tft.setTextSize(3);
    tft.setTextDatum(TC_DATUM);
    tft.drawString(buf, ui::SCREEN_W / 2, ui::SPEED_Y);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}
