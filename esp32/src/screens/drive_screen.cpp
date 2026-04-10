// =============================================================================
// ESP32-S3 HMI — Drive Screen Implementation (Tile-Based Dirty Region Engine)
//
// Implements the main driving dashboard using a tile-based render architecture.
// Each screen region is a logical tile with its own dirty flag and content
// hash. Only tiles whose source data changed since the last frame are redrawn.
//
// Tile map (480×320 landscape):
//   DTILE_MODE_ICONS  [  0, 0,180, 40]  — 4x4/4x2/360° icons
//   DTILE_LED_TOGGLE  [180, 0,120, 40]  — front/rear LED buttons
//   DTILE_ACK         [200, 0, 80, 20]  — ACK indicator (overlaps LED area)
//   DTILE_BATTERY     [405, 0, 75, 40]  — battery icon + percentage
//   DTILE_OBSTACLE    [  0,40,480, 45]  — frontal sensor distance + bar
//   DTILE_WHEELS      [  0,85,370,145]  — 4 wheels + labels
//   DTILE_STEERING    [370,85,110,145]  — steering circular gauge
//   DTILE_SPEED       [  0,230,480,40]  — large speed display
//   DTILE_PEDAL       [  0,270,480,30]  — pedal bar + percentage
//   DTILE_GEAR        [  0,300,480,20]  — gear selector P/R/N/D1/D2
//   DTILE_DEGRADED    [  0, 40,480,18]  — overlay: degraded/limp banner
//   DTILE_FAULTS      [  0, 28,480,10]  — overlay: fault indicators
//
// Pipeline per frame:
//   1. update(snapshot) → latch cur_* from frozen VehicleData
//   2. Compute FNV-1a hash per tile → updateHash() → dirty if changed
//   3. draw() → skip clean tiles, render only dirty ones
//   4. Tiles marked clean after render; prev_* updated for next frame
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
#include "can_ids.h"
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

// Degraded/limp mode overlay layout (HMI_STATE_MODEL §2.4)
static constexpr int16_t DEG_BANNER_X = 0;
static constexpr int16_t DEG_BANNER_Y = 40;
static constexpr int16_t DEG_BANNER_W = 480;
static constexpr int16_t DEG_BANNER_H = 18;

// Fault overlay area (bottom margin of top bar, right of mode icons)
static constexpr int16_t FAULT_OVERLAY_Y = 28;
static constexpr int16_t FAULT_OVERLAY_H = 10;

// -------------------------------------------------------------------------
// onEnter — called when transitioning to this screen
// -------------------------------------------------------------------------
void DriveScreen::onEnter() {
    RTRACE_BEGIN_SCREEN("drive");
    needsFullRedraw_ = true;

    // ---- Initialize tile regions ----
    tiles_.setRect(DTILE_SPEED,      0,   ui::SPEED_Y,  ui::SCREEN_W, ui::SPEED_H);
    tiles_.setRect(DTILE_OBSTACLE,   0,   ui::SENSOR_Y, ui::SCREEN_W, ui::SENSOR_H);
    tiles_.setRect(DTILE_WHEELS,     0,   ui::CAR_AREA_Y, 370, ui::CAR_AREA_H);
    tiles_.setRect(DTILE_STEERING,   370, ui::CAR_AREA_Y, 110, ui::CAR_AREA_H);
    tiles_.setRect(DTILE_BATTERY,    ui::BAT_X, 0, 75, ui::TOP_BAR_H);
    tiles_.setRect(DTILE_GEAR,       0,   ui::GEAR_Y, ui::SCREEN_W, ui::GEAR_H);
    tiles_.setRect(DTILE_PEDAL,      0,   ui::PEDAL_Y, ui::SCREEN_W, ui::PEDAL_H);
    tiles_.setRect(DTILE_MODE_ICONS, 0,   0, 180, ui::TOP_BAR_H);
    tiles_.setRect(DTILE_LED_TOGGLE, 180, 0, 120, ui::TOP_BAR_H);
    tiles_.setRect(DTILE_DEGRADED,   DEG_BANNER_X, DEG_BANNER_Y, DEG_BANNER_W, DEG_BANNER_H);
    tiles_.setRect(DTILE_FAULTS,     0,   FAULT_OVERLAY_Y, ui::SCREEN_W, FAULT_OVERLAY_H);
    tiles_.setRect(DTILE_ACK,        ACK_X, ACK_Y, ACK_W, ACK_H);

    // Invalidate all tile hashes — forces full redraw of every tile
    tiles_.invalidateAll();

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

    // Reset degraded/fault overlay state
    curSystemState_  = can::SystemState::ACTIVE;
    prevSystemState_ = can::SystemState::ACTIVE;
    curFaultFlags_   = 0;
    prevFaultFlags_  = 0;
}

// -------------------------------------------------------------------------
// onExit — called when leaving this screen
// -------------------------------------------------------------------------
void DriveScreen::onExit() {
    // Nothing to clean up — no dynamic resources
}

// -------------------------------------------------------------------------
// update — read vehicle data into current-frame cache + compute tile hashes
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

    // System state for degraded/limp overlay (HMI_STATE_MODEL §2.4)
    curSystemState_ = data.heartbeat().systemState;

    // Fault flags for visual overlays (HMI_STATE_MODEL §4.1)
    curFaultFlags_ = data.heartbeat().faultFlags;

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

    // ---- Compute tile hashes for dirty detection ----
    // Each tile gets a compact FNV-1a hash of its source data.
    // If the hash matches the previous frame, the tile is skipped.

    // SPEED tile
    tiles_.updateHash(DTILE_SPEED, ui::tileHashVal(curSpeedAvgRaw_));

    // OBSTACLE tile
    tiles_.updateHash(DTILE_OBSTACLE, ui::tileHashVal(curObstacleCm_));

    // WHEELS tile — hash all 4 traction + 4 temp values together.
    // Uses threshold filtering: small CAN jitter doesn't trigger redraw.
    {
        ui::TileHash wh = ui::FNV_OFFSET;
        for (uint8_t i = 0; i < 4; ++i) {
            // Apply threshold: only count change if Δtorque > 2 or Δtemp ≥ 1
            uint8_t dt = (curTraction_[i] > prevTraction_[i])
                         ? (curTraction_[i] - prevTraction_[i])
                         : (prevTraction_[i] - curTraction_[i]);
            int8_t  dT = (curTemp_[i] > prevTemp_[i])
                         ? static_cast<int8_t>(curTemp_[i] - prevTemp_[i])
                         : static_cast<int8_t>(prevTemp_[i] - curTemp_[i]);

            // Use drawn value (prev) if below threshold, current if above
            uint8_t drawTraction = (dt > 2) ? curTraction_[i] : prevTraction_[i];
            int8_t  drawTemp     = (dT >= 1) ? curTemp_[i] : prevTemp_[i];
            wh = ui::tileHashFeed(wh, drawTraction);
            wh = ui::tileHashFeed(wh, drawTemp);
        }
        tiles_.updateHash(DTILE_WHEELS, wh);
    }

    // STEERING tile
    tiles_.updateHash(DTILE_STEERING, ui::tileHashVal(curSteeringRaw_));

    // BATTERY tile
    tiles_.updateHash(DTILE_BATTERY, ui::tileHashVal(curBattVoltRaw_));

    // GEAR tile
    tiles_.updateHash(DTILE_GEAR, ui::tileHashVal(curGear_));

    // PEDAL tile
    tiles_.updateHash(DTILE_PEDAL, ui::tileHashVal(curPedalPct_));

    // MODE_ICONS tile
    {
        ui::TileHash mh = ui::tileHashVal(curMode_.is4x4);
        mh = ui::tileHashFeed(mh, curMode_.isTankTurn);
        tiles_.updateHash(DTILE_MODE_ICONS, mh);
    }

    // LED_TOGGLE tile
    {
        ui::TileHash lh = ui::tileHashVal(curFrontLedOn_);
        lh = ui::tileHashFeed(lh, curRearLedOn_);
        tiles_.updateHash(DTILE_LED_TOGGLE, lh);
    }

    // DEGRADED overlay tile
    tiles_.updateHash(DTILE_DEGRADED, ui::tileHashVal(curSystemState_));

    // FAULTS overlay tile
    tiles_.updateHash(DTILE_FAULTS, ui::tileHashVal(curFaultFlags_));

    // ACK tile — uses dirty flag directly (event-driven, not hash-based)
    if (ackIndicatorDirty_) {
        tiles_.markDirty(DTILE_ACK);
    }
}

// -------------------------------------------------------------------------
// draw — tile-based render: only dirty tiles are redrawn
// -------------------------------------------------------------------------
void DriveScreen::draw() {
    if (needsFullRedraw_) {
        needsFullRedraw_ = false;
        RTMON_FULL_REDRAW();

        // ---- STATIC LAYER (drawn once per screen enter) ----
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

        // Force all prev_* to differ from cur_* so every tile renders
        prevSpeedAvgRaw_ = curSpeedAvgRaw_ + 1;
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
        prevSystemState_    = (curSystemState_ == can::SystemState::ACTIVE)
                              ? can::SystemState::DEGRADED : can::SystemState::ACTIVE;
        prevFaultFlags_     = curFaultFlags_ ^ 0xFF;

        // Mark all tiles dirty for initial dynamic render pass
        tiles_.markAllDirty();
    }

    // ---- DYNAMIC LAYER — tile-based dirty region rendering ----
    RTRACE_SET_LAYER(2);

    // TILE: Speed (230–270px)
    if (tiles_.isDirty(DTILE_SPEED)) {
        RTMON_ZONE_REDRAW(rtmon::Zone::SPEED);
        drawSpeed();
        tiles_.markClean(DTILE_SPEED);
    }

    // TILE: Obstacle sensor (40–85px)
    if (tiles_.isDirty(DTILE_OBSTACLE)) {
        RTMON_ZONE_REDRAW(rtmon::Zone::OBSTACLE);
        ui::ObstacleSensor::draw(tft, curObstacleCm_, prevObstacleCm_);
        tiles_.markClean(DTILE_OBSTACLE);
    }

    // TILE: Wheels — uses threshold filtering from update() hash
    if (tiles_.isDirty(DTILE_WHEELS)) {
        RTMON_ZONE_REDRAW(rtmon::Zone::CAR);

        // Compute draw values with threshold filtering
        uint8_t drawTraction[4];
        int8_t  drawTemp[4];

        for (uint8_t i = 0; i < 4; ++i) {
            uint8_t dt = (curTraction_[i] > prevTraction_[i])
                         ? (curTraction_[i] - prevTraction_[i])
                         : (prevTraction_[i] - curTraction_[i]);
            int8_t  dT = (curTemp_[i] > prevTemp_[i])
                         ? static_cast<int8_t>(curTemp_[i] - prevTemp_[i])
                         : static_cast<int8_t>(prevTemp_[i] - curTemp_[i]);

            drawTraction[i] = (dt > 2) ? curTraction_[i] : prevTraction_[i];
            drawTemp[i]     = (dT >= 1) ? curTemp_[i]    : prevTemp_[i];
        }

        ui::CarRenderer::drawWheels(tft, vehicle::TractionData{
            {drawTraction[0], drawTraction[1], drawTraction[2], drawTraction[3]}, 0},
            vehicle::TempMapData{
            {drawTemp[0], drawTemp[1], drawTemp[2], drawTemp[3], 0}, 0},
            prevTraction_, prevTemp_);

        // Update prev to what was actually drawn (not raw CAN values)
        memcpy(prevTraction_, drawTraction, sizeof(prevTraction_));
        memcpy(prevTemp_, drawTemp, sizeof(prevTemp_));
        tiles_.markClean(DTILE_WHEELS);
    }

    // TILE: Steering circular gauge (right side)
    if (tiles_.isDirty(DTILE_STEERING)) {
        ui::CarRenderer::drawSteering(tft, curSteeringRaw_, prevSteeringRaw_);
        tiles_.markClean(DTILE_STEERING);
    }

    // TILE: Battery (top-right)
    if (tiles_.isDirty(DTILE_BATTERY)) {
        RTMON_ZONE_REDRAW(rtmon::Zone::TOP_BAR);
        ui::BatteryIndicator::draw(tft, curBattVoltRaw_, prevBattVoltRaw_);
        tiles_.markClean(DTILE_BATTERY);
    }

    // TILE: Gear
    if (tiles_.isDirty(DTILE_GEAR)) {
        RTMON_ZONE_REDRAW(rtmon::Zone::GEAR);
        ui::GearDisplay::draw(tft, curGear_, prevGear_);
        tiles_.markClean(DTILE_GEAR);
    }

    // TILE: Pedal bar
    if (tiles_.isDirty(DTILE_PEDAL)) {
        RTMON_ZONE_REDRAW(rtmon::Zone::PEDAL);
        ui::PedalBar::draw(tft, curPedalPct_, prevPedalPct_);
        tiles_.markClean(DTILE_PEDAL);
    }

    // TILE: Mode icons
    if (tiles_.isDirty(DTILE_MODE_ICONS)) {
        RTMON_ZONE_REDRAW(rtmon::Zone::TOP_BAR);
        ui::ModeIcons::draw(tft, curMode_, prevMode_);
        tiles_.markClean(DTILE_MODE_ICONS);
    }

    // TILE: LED toggle buttons
    if (tiles_.isDirty(DTILE_LED_TOGGLE)) {
        RTMON_ZONE_REDRAW(rtmon::Zone::TOP_BAR);
        ui::LedToggle::draw(tft, curFrontLedOn_, prevFrontLedOn_,
                                 curRearLedOn_,  prevRearLedOn_);
        tiles_.markClean(DTILE_LED_TOGGLE);
    }

    // TILE: Degraded/limp mode overlay (HMI_STATE_MODEL §2.4)
    if (tiles_.isDirty(DTILE_DEGRADED)) {
        drawDegradedOverlay();
        tiles_.markClean(DTILE_DEGRADED);
    }

    // TILE: Fault flag visual overlays (HMI_STATE_MODEL §4.1)
    if (tiles_.isDirty(DTILE_FAULTS)) {
        drawFaultOverlays();
        tiles_.markClean(DTILE_FAULTS);
    }

    // TILE: ACK visual feedback indicator (event-driven)
    if (tiles_.isDirty(DTILE_ACK)) {
        drawAckIndicator();
        tiles_.markClean(DTILE_ACK);
    }

    // Copy current values to previous for next frame
    // (traction/temp prev values managed in the wheel tile above — threshold logic)
    prevSteeringRaw_ = curSteeringRaw_;
    prevSpeedAvgRaw_ = curSpeedAvgRaw_;
    prevBattVoltRaw_ = curBattVoltRaw_;
    prevPedalPct_    = curPedalPct_;
    prevGear_        = curGear_;
    prevMode_        = curMode_;
    prevObstacleCm_  = curObstacleCm_;
    prevFrontLedOn_  = curFrontLedOn_;
    prevRearLedOn_   = curRearLedOn_;
    prevSystemState_ = curSystemState_;
    prevFaultFlags_  = curFaultFlags_;

    RTRACE_DUMP_IF_PENDING();
}

// -------------------------------------------------------------------------
// Speed display helper — in its own zone (230–270px), NOT inside car
//
// Anti-flicker: uses setTextPadding() to overwrite previous text in a
// single SPI transaction instead of fillRect+drawString (which causes
// a visible blank flash between clear and redraw).
// -------------------------------------------------------------------------
void DriveScreen::drawSpeed() {
    // Convert raw (0.1 km/h) to display
    uint16_t intPart  = curSpeedAvgRaw_ / 10;
    uint16_t fracPart = curSpeedAvgRaw_ % 10;

    char buf[ui::FMT_BUF_MED];
    snprintf(buf, sizeof(buf), "%u.%u", intPart, fracPart);

    // Draw speed value — padded to fixed width eliminates flicker
    tft.setTextColor(ui::COL_WHITE, ui::COL_BG);
    tft.setTextSize(3);
    tft.setTextDatum(TC_DATUM);
    tft.setTextPadding(120);   // covers max "999.9" at size 3
    tft.drawString(buf, ui::SCREEN_W / 2, ui::SPEED_Y);
    RTRACE_TEXT(ui::SCREEN_W / 2, ui::SPEED_Y, buf,
                ui::COL_WHITE, ui::COL_BG, 3, TC_DATUM);
    tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}

// -------------------------------------------------------------------------
// ACK visual feedback — brief indicator in top bar after mode/gear command
// Called only when DTILE_ACK is dirty (event-driven).
// -------------------------------------------------------------------------
void DriveScreen::drawAckIndicator() {
    ackIndicatorDirty_ = false;

    const char* text = "";
    uint16_t color = ui::COL_BG;
    switch (ackDisplayResult_) {
        case 1:  text = "OK";       color = ui::COL_GREEN;  break;
        case 2:  text = "REJECTED"; color = ui::COL_RED;    break;
        case 3:  text = "TIMEOUT";  color = ui::COL_YELLOW; break;
        default: text = "";         color = ui::COL_BG;     break;
    }

    // Anti-flicker: setTextPadding overwrites the previous text in one pass
    tft.setTextColor(color, ui::COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(TC_DATUM);
    tft.setTextPadding(ACK_W);
    tft.drawString(text, ACK_X + ACK_W / 2, ACK_Y + 2);
    RTRACE_TEXT(ACK_X + ACK_W / 2, ACK_Y + 2, text,
                color, ui::COL_BG, 1, TC_DATUM);
    tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// Degraded / Limp-Home mode overlay (HMI_STATE_MODEL §2.4)
//
// When system_state is DEGRADED(3) or LIMP_HOME(6), draws an amber banner
// just below the top bar to alert the driver.
// Called only when DTILE_DEGRADED is dirty (system state changed).
// -------------------------------------------------------------------------
void DriveScreen::drawDegradedOverlay() {
    // Clear the banner area regardless (remove old banner if state changed)
    tft.fillRect(DEG_BANNER_X, DEG_BANNER_Y, DEG_BANNER_W, DEG_BANNER_H, ui::COL_BG);
    RTRACE_FILL_RECT(DEG_BANNER_X, DEG_BANNER_Y, DEG_BANNER_W, DEG_BANNER_H, ui::COL_BG);

    const char* bannerText = nullptr;

    if (curSystemState_ == can::SystemState::DEGRADED) {
        bannerText = "DEGRADED MODE - 40% POWER";
    } else if (curSystemState_ == can::SystemState::LIMP_HOME) {
        bannerText = "LIMP HOME - REDUCED SPEED";
    }

    if (bannerText != nullptr) {
        tft.fillRect(DEG_BANNER_X, DEG_BANNER_Y, DEG_BANNER_W, DEG_BANNER_H, ui::COL_AMBER);
        RTRACE_FILL_RECT(DEG_BANNER_X, DEG_BANNER_Y, DEG_BANNER_W, DEG_BANNER_H, ui::COL_AMBER);
        tft.setTextColor(ui::COL_BLACK, ui::COL_AMBER);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(bannerText, DEG_BANNER_W / 2, DEG_BANNER_Y + DEG_BANNER_H / 2);
        RTRACE_TEXT(DEG_BANNER_W / 2, DEG_BANNER_Y + DEG_BANNER_H / 2, bannerText,
                    ui::COL_BLACK, ui::COL_AMBER, 1, MC_DATUM);
        tft.setTextDatum(TL_DATUM);
    }
}

// -------------------------------------------------------------------------
// Fault flag visual overlays (HMI_STATE_MODEL §4.1)
//
// Draws compact fault indicators below the top bar.
// ABS/TCS are informational (green/cyan), faults are amber/red.
// Called only when DTILE_FAULTS is dirty (fault flags changed).
// -------------------------------------------------------------------------
void DriveScreen::drawFaultOverlays() {
    // Clear the fault overlay strip — needed because multiple labels are
    // drawn at variable positions; setTextPadding alone cannot clear the
    // entire strip when the active set of faults changes.
    tft.fillRect(0, FAULT_OVERLAY_Y, ui::SCREEN_W, FAULT_OVERLAY_H, ui::COL_BG);
    RTRACE_FILL_RECT(0, FAULT_OVERLAY_Y, ui::SCREEN_W, FAULT_OVERLAY_H, ui::COL_BG);

    if (curFaultFlags_ == 0) return;  // No faults active

    // Build compact fault indicator string
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);

    int16_t x = 4;
    static constexpr int16_t LABEL_GAP = 4;

    // Fault indicators — amber/red for faults, cyan for informational
    struct FaultEntry {
        uint8_t     mask;
        const char* label;
        uint16_t    color;
    };
    static constexpr FaultEntry entries[] = {
        { 0x01, "CAN TMO",     ui::COL_AMBER  },   // Bit 0: CAN_TIMEOUT
        { 0x02, "OVERTEMP",    ui::COL_AMBER  },   // Bit 1
        { 0x04, "OVERCURR",    ui::COL_AMBER  },   // Bit 2
        { 0x08, "ENC FAULT",   ui::COL_AMBER  },   // Bit 3
        { 0x10, "WHL SENS",    ui::COL_AMBER  },   // Bit 4
        { 0x20, "ABS",         ui::COL_CYAN   },   // Bit 5 (informational)
        { 0x40, "TCS",         ui::COL_CYAN   },   // Bit 6 (informational)
        { 0x80, "CENTER",      ui::COL_AMBER  },   // Bit 7
    };

    for (const auto& e : entries) {
        if (curFaultFlags_ & e.mask) {
            tft.setTextColor(e.color, ui::COL_BG);
            tft.drawString(e.label, x, FAULT_OVERLAY_Y);
            RTRACE_TEXT(x, FAULT_OVERLAY_Y, e.label,
                        e.color, ui::COL_BG, 1, TL_DATUM);
            x += static_cast<int16_t>(tft.textWidth(e.label) + LABEL_GAP);
        }
    }
}
