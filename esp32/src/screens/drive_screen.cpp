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
#include "ui/render_trace.h"
#include "ui/runtime_monitor.h"
#include "shifter_input.h"
#include <Arduino.h>
#include <cstdio>
#include <cstring>

// External TFT instance (initialized in main.cpp)
extern TFT_eSPI tft;

// ACK visual feedback constants
static constexpr int16_t ACK_X = 200;   // Centered in top bar
static constexpr int16_t ACK_Y = 2;
static constexpr int16_t ACK_W = 80;
static constexpr int16_t ACK_H = 16;
static constexpr unsigned long ACK_DISPLAY_DURATION_MS = 1500;

// -------------------------------------------------------------------------
// onEnter — called when transitioning to this screen
// -------------------------------------------------------------------------
void DriveScreen::onEnter() {
    RTRACE_BEGIN_SCREEN("drive");
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
    prevFrontLedOn_  = false;
    prevRearLedOn_   = false;

    // Reset ACK indicator state
    ackLastShownMs_    = 0;
    ackTrackedAckMs_   = 0;
    ackTrackedTmoMs_   = 0;
    ackDisplayResult_  = 0;
    ackIndicatorDirty_ = false;
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

    // Gear — read from physical shifter via MCP23017
    {
        uint8_t raw = shifter::getGearRaw();
        // Map shifter::Gear (0-4) to ui::Gear enum
        switch (raw) {
            case 0: curGear_ = ui::Gear::P;  break;
            case 1: curGear_ = ui::Gear::R;  break;
            case 2: curGear_ = ui::Gear::N;  break;
            case 3: curGear_ = ui::Gear::D1; break;
            case 4: curGear_ = ui::Gear::D2; break;
            default: curGear_ = ui::Gear::N; break;
        }
    }

    // Mode flags — read from vehicle data (confirmed by STM32 heartbeat echo)
    {
        uint8_t flags = data.mode().modeFlags;
        curMode_.is4x4     = (flags & 0x01) != 0;
        curMode_.isTankTurn = (flags & 0x02) != 0;
    }

    // Obstacle sensor
    curObstacleCm_ = data.obstacle().distanceCm;

    // LED relay states from STM32
    curFrontLedOn_ = data.lights().frontRelayOn;
    curRearLedOn_  = data.lights().rearRelayOn;

    // ACK visual feedback: detect new ACK or timeout events
    {
        unsigned long now = millis();
        const auto& ad = data.ack();

        // New ACK received from STM32
        if (ad.timestampMs > 0 && ad.timestampMs > ackTrackedAckMs_) {
            ackTrackedAckMs_ = ad.timestampMs;
            ackDisplayResult_ = (ad.result == can::AckResult::OK) ? 1 : 2;
            ackLastShownMs_   = now;
            ackIndicatorDirty_ = true;
        }

        // New ACK timeout detected
        unsigned long tmo = data.ackTimeoutMs();
        if (tmo > 0 && tmo > ackTrackedTmoMs_) {
            ackTrackedTmoMs_  = tmo;
            ackDisplayResult_ = 3;
            ackLastShownMs_   = now;
            ackIndicatorDirty_ = true;
        }

        // Auto-clear after display duration
        if (ackDisplayResult_ != 0 && (now - ackLastShownMs_) >= ACK_DISPLAY_DURATION_MS) {
            ackDisplayResult_ = 0;
            ackIndicatorDirty_ = true;
        }
    }
}

// -------------------------------------------------------------------------
// draw — render changed elements to TFT
// -------------------------------------------------------------------------
void DriveScreen::draw() {
    if (needsFullRedraw_) {
        needsFullRedraw_ = false;
        RTMON_FULL_REDRAW();

        // Clear entire screen
        RTRACE_SET_LAYER(0);
        tft.fillScreen(ui::COL_BG);
        RTRACE_FILL_SCREEN(ui::COL_BG);

        RTRACE_SET_LAYER(1);

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
        RTRACE_TEXT(ui::SCREEN_W / 2, ui::SPEED_Y + 26, "km/h",
                    ui::COL_GRAY, ui::COL_BG, 1, TC_DATUM);
        tft.setTextDatum(TL_DATUM);

        // "360°" label centered above steering gauge
        tft.setTextColor(ui::COL_CYAN, ui::COL_BG);
        tft.setTextSize(1);
        tft.setTextDatum(TC_DATUM);
        tft.drawString("360", ui::STEER_CX, ui::STEER_CY - ui::STEER_RADIUS - 12);
        RTRACE_TEXT(ui::STEER_CX, ui::STEER_CY - ui::STEER_RADIUS - 12, "360",
                    ui::COL_CYAN, ui::COL_BG, 1, TC_DATUM);
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
        prevFrontLedOn_     = !curFrontLedOn_;
        prevRearLedOn_      = !curRearLedOn_;
    }

    // Partial redraw: only changed elements
    RTRACE_SET_LAYER(2);

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

    // LED toggle buttons (part of top bar zone)
    if (curFrontLedOn_ != prevFrontLedOn_ || curRearLedOn_ != prevRearLedOn_) {
        RTMON_ZONE_REDRAW(rtmon::Zone::TOP_BAR);
    }
    ui::LedToggle::draw(tft, curFrontLedOn_, prevFrontLedOn_,
                             curRearLedOn_,  prevRearLedOn_);

    // ACK visual feedback indicator (brief text near top bar)
    drawAckIndicator();

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
    prevFrontLedOn_  = curFrontLedOn_;
    prevRearLedOn_   = curRearLedOn_;

    RTRACE_DUMP_IF_PENDING();
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
    RTRACE_FILL_RECT(0, ui::SPEED_Y, ui::SCREEN_W, 24, ui::COL_BG);

    // Draw speed value — large centered text
    tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
    tft.setTextSize(3);
    tft.setTextDatum(TC_DATUM);
    tft.drawString(buf, ui::SCREEN_W / 2, ui::SPEED_Y);
    RTRACE_TEXT(ui::SCREEN_W / 2, ui::SPEED_Y, buf,
                ui::COL_WHITE, ui::COL_BG, 3, TC_DATUM);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}

// -------------------------------------------------------------------------
// ACK visual feedback — brief indicator in top bar after mode/gear command
// -------------------------------------------------------------------------
void DriveScreen::drawAckIndicator() {
    if (!ackIndicatorDirty_) return;
    ackIndicatorDirty_ = false;

    // Clear indicator area
    tft.fillRect(ACK_X, ACK_Y, ACK_W, ACK_H, ui::COL_BG);
    RTRACE_FILL_RECT(ACK_X, ACK_Y, ACK_W, ACK_H, ui::COL_BG);

    if (ackDisplayResult_ == 0) return;  // Nothing to show

    const char* text;
    uint16_t color;
    switch (ackDisplayResult_) {
        case 1:  text = "OK";       color = ui::COL_GREEN;  break;
        case 2:  text = "REJECTED"; color = ui::COL_RED;    break;
        case 3:  text = "TIMEOUT";  color = ui::COL_YELLOW; break;
        default: return;
    }

    tft.setTextColor(color, ui::COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(TC_DATUM);
    tft.drawString(text, ACK_X + ACK_W / 2, ACK_Y + 2);
    RTRACE_TEXT(ACK_X + ACK_W / 2, ACK_Y + 2, text,
                color, ui::COL_BG, 1, TC_DATUM);
    tft.setTextDatum(TL_DATUM);
}
