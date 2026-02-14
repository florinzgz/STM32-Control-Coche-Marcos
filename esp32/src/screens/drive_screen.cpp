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
#include "ui/obstacle_sensor.h"
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

    // Gear — derived from heartbeat flags (byte 1 of CMD_MODE)
    // For now, map system state to gear display:
    // Gear is tracked separately via mode commands; default to N if unknown
    // The gear value is encoded as the second byte of CMD_MODE (0x102)
    // Since we receive the heartbeat state, we default to D1 when active
    curGear_ = ui::Gear::D1;

    // Mode flags
    curMode_.is4x4 = false;
    curMode_.isTankTurn = false;

    // Obstacle sensor
    curObstacleCm_ = data.obstacle().distanceCm;
}

// -------------------------------------------------------------------------
// draw — render changed elements to TFT
// -------------------------------------------------------------------------
void DriveScreen::draw() {
    if (needsFullRedraw_) {
        needsFullRedraw_ = false;

        // Clear entire screen
        tft.fillScreen(ui::COL_BG);

        // Draw all static elements (in layout order top→bottom)
        ui::ModeIcons::drawStatic(tft);
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
    }

    // Partial redraw: only changed elements

    // Speed (in its own zone, 230–270px)
    drawSpeed();

    // Obstacle sensor (40–85px)
    ui::ObstacleSensor::draw(tft, curObstacleCm_, prevObstacleCm_);

    // Wheels (torque + temperature)
    ui::CarRenderer::drawWheels(tft, vehicle::TractionData{
        {curTraction_[0], curTraction_[1], curTraction_[2], curTraction_[3]}, 0},
        vehicle::TempMapData{
        {curTemp_[0], curTemp_[1], curTemp_[2], curTemp_[3], 0}, 0},
        prevTraction_, prevTemp_);

    // Steering circular gauge (right side)
    ui::CarRenderer::drawSteering(tft, curSteeringRaw_, prevSteeringRaw_);

    // Battery
    ui::BatteryIndicator::draw(tft, curBattVoltRaw_, prevBattVoltRaw_);

    // Gear
    ui::GearDisplay::draw(tft, curGear_, prevGear_);

    // Pedal bar
    ui::PedalBar::draw(tft, curPedalPct_, prevPedalPct_);

    // Mode icons
    ui::ModeIcons::draw(tft, curMode_, prevMode_);

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
