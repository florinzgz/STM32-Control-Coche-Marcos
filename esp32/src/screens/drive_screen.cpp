// =============================================================================
// ESP32-S3 HMI — Drive Screen Implementation (Tile-Based Dirty Region Engine)
//
// Implements the main driving dashboard using a tile-based render architecture.
// Each screen region is a logical tile with its own dirty flag and content
// hash. Only tiles whose source data changed since the last frame are redrawn.
//
// Tile map (480×320 landscape) — dimensions defined in ui_config.h:
//   DTILE_MODE_ICONS  — 4x4/4x2/360° icons (top bar left)
//   DTILE_LED_TOGGLE  — front/rear LED buttons (top bar left-center)
//   DTILE_ACK         — ACK indicator (overlaps LED area, overlay layer)
//   DTILE_BATTERY     — battery icon + percentage (top bar right)
//   DTILE_OBSTACLE    — frontal sensor distance + bar
//   DTILE_WHEELS      — 4 wheels + labels
//   DTILE_STEERING    — steering circular gauge
//   DTILE_SPEED       — large speed display
//   DTILE_PEDAL       — pedal bar + percentage
//   DTILE_GEAR        — gear selector P/R/N/D1/D2
//   DTILE_DEGRADED    — overlay: degraded/limp banner
//   DTILE_FAULTS      — overlay: fault indicators
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
#include "ui/ui_config.h"
#include "shifter_input.h"
#include "can_ids.h"
#include <Arduino.h>
#include <cstdio>
#include <cstring>

// External TFT instance (initialized in main.cpp)
extern TFT_eSPI tft;

// Wheel threshold filtering constants (from centralized config)
static constexpr uint8_t TRACTION_THRESHOLD = ui::cfg::THR_TRACTION_DELTA;
static constexpr int8_t  TEMP_THRESHOLD     = ui::cfg::THR_TEMP_DELTA;

/// Apply threshold filtering to a single wheel's traction + temperature.
/// Returns true if either value crossed its threshold.
static inline bool wheelThresholdFilter(
    uint8_t curTraction, uint8_t prevTraction, uint8_t& drawTraction,
    int8_t  curTemp,     int8_t  prevTemp,     int8_t& drawTemp) {
    uint8_t dt = (curTraction > prevTraction)
                 ? static_cast<uint8_t>(curTraction - prevTraction)
                 : static_cast<uint8_t>(prevTraction - curTraction);
    int8_t  dT = (curTemp > prevTemp)
                 ? static_cast<int8_t>(curTemp - prevTemp)
                 : static_cast<int8_t>(prevTemp - curTemp);

    drawTraction = (dt > TRACTION_THRESHOLD) ? curTraction : prevTraction;
    drawTemp     = (dT >= TEMP_THRESHOLD)    ? curTemp     : prevTemp;

    return (dt > TRACTION_THRESHOLD) || (dT >= TEMP_THRESHOLD);
}

// -------------------------------------------------------------------------
// onEnter — called when transitioning to this screen
// -------------------------------------------------------------------------
void DriveScreen::onEnter() {
    RTRACE_BEGIN_SCREEN("drive");
    needsFullRedraw_ = true;

    // ---- Initialize tile regions (dimensions from ui_config.h) ----
    tiles_.setRect(DTILE_SPEED,      0,   ui::SPEED_Y,  ui::SCREEN_W, ui::SPEED_H);
    tiles_.setRect(DTILE_OBSTACLE,   0,   ui::SENSOR_Y, ui::SCREEN_W, ui::SENSOR_H);
    tiles_.setRect(DTILE_WHEELS,     0,   ui::CAR_AREA_Y,
                   ui::cfg::DTILE_WHEELS_W, ui::CAR_AREA_H);
    tiles_.setRect(DTILE_STEERING,   ui::cfg::DTILE_STEERING_X, ui::CAR_AREA_Y,
                   ui::cfg::DTILE_STEERING_W, ui::CAR_AREA_H);
    tiles_.setRect(DTILE_BATTERY,    ui::BAT_X, 0,
                   ui::cfg::DTILE_BATTERY_W, ui::TOP_BAR_H);
    tiles_.setRect(DTILE_GEAR,       0,   ui::GEAR_Y, ui::SCREEN_W, ui::GEAR_H);
    tiles_.setRect(DTILE_PEDAL,      0,   ui::PEDAL_Y, ui::SCREEN_W, ui::PEDAL_H);
    tiles_.setRect(DTILE_MODE_ICONS, ui::cfg::DTILE_MODE_ICONS_X, 0,
                   ui::cfg::DTILE_MODE_ICONS_W, ui::TOP_BAR_H);
    tiles_.setRect(DTILE_LED_TOGGLE, ui::cfg::DTILE_LED_TOGGLE_X, 0,
                   ui::cfg::DTILE_LED_TOGGLE_W, ui::TOP_BAR_H);
    tiles_.setRect(DTILE_DEGRADED,   ui::cfg::OVL_DEGRADED_X, ui::cfg::OVL_DEGRADED_Y,
                                     ui::cfg::OVL_DEGRADED_W, ui::cfg::OVL_DEGRADED_H);
    tiles_.setRect(DTILE_FAULTS,     0,   ui::cfg::OVL_FAULT_Y, ui::SCREEN_W, ui::cfg::OVL_FAULT_H);
    tiles_.setRect(DTILE_ACK,        ui::cfg::ACK_X, ui::cfg::ACK_Y,
                                     ui::cfg::ACK_W, ui::cfg::ACK_H);

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

    // Reset overlay visibility tracking
    prevDegradedVisible_ = false;
    prevFaultsVisible_   = false;
    prevAckVisible_      = false;
    curDegradedVisible_  = false;
    curFaultsVisible_    = false;
    curAckVisible_       = false;

    // Reset precomputed wheel draw values
    memset(drawTraction_, 0, sizeof(drawTraction_));
    memset(drawTemp_, 0, sizeof(drawTemp_));
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
void DriveScreen::update(const vehicle::VehicleData& data, unsigned long frameTimeMs) {
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
        const auto& ad = data.ack();

        // New ACK received from STM32
        if (ad.timestampMs > 0 && ad.timestampMs > ackTrackedAckMs_) {
            ackTrackedAckMs_ = ad.timestampMs;
            ackDisplayResult_ = (ad.result == can::AckResult::OK) ? 1 : 2;
            ackLastShownMs_   = frameTimeMs;
            ackIndicatorDirty_ = true;
        }

        // New ACK timeout detected
        unsigned long tmo = data.ackTimeoutMs();
        if (tmo > 0 && tmo > ackTrackedTmoMs_) {
            ackTrackedTmoMs_  = tmo;
            ackDisplayResult_ = 3;
            ackLastShownMs_   = frameTimeMs;
            ackIndicatorDirty_ = true;
        }

        // Auto-clear after display duration
        if (ackDisplayResult_ != 0 && (frameTimeMs - ackLastShownMs_) >= ui::cfg::ACK_DISPLAY_DURATION_MS) {
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
    // Precomputed draw values are stored for use in draw() phase.
    {
        ui::TileHash wh = ui::FNV_OFFSET;
        for (uint8_t i = 0; i < 4; ++i) {
            wheelThresholdFilter(curTraction_[i], prevTraction_[i], drawTraction_[i],
                                 curTemp_[i], prevTemp_[i], drawTemp_[i]);
            wh = ui::tileHashFeed(wh, drawTraction_[i]);
            wh = ui::tileHashFeed(wh, drawTemp_[i]);
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

    // DEGRADED overlay tile — hash the display state (whether banner is shown),
    // not the raw system state, to avoid unnecessary redraws on transitions
    // between states that both don't trigger the overlay (e.g. ACTIVE→SAFE).
    // Visibility is precomputed here; draw() MUST NOT recompute it.
    curDegradedVisible_ = (curSystemState_ == can::SystemState::DEGRADED ||
                           curSystemState_ == can::SystemState::LIMP_HOME);
    tiles_.updateHash(DTILE_DEGRADED, ui::tileHashVal(curDegradedVisible_));

    // FAULTS overlay tile — visibility precomputed for draw()
    curFaultsVisible_ = (curFaultFlags_ != 0);
    tiles_.updateHash(DTILE_FAULTS, ui::tileHashVal(curFaultFlags_));

    // ACK tile — uses dirty flag directly (event-driven, not hash-based)
    // Visibility precomputed for overlay invalidation in draw()
    curAckVisible_ = (ackDisplayResult_ != 0);
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

    // TILE: Wheels — uses precomputed threshold-filtered values from update()
    if (tiles_.isDirty(DTILE_WHEELS)) {
        RTMON_ZONE_REDRAW(rtmon::Zone::CAR);

        ui::CarRenderer::drawWheels(tft, vehicle::TractionData{
            {drawTraction_[0], drawTraction_[1], drawTraction_[2], drawTraction_[3]}, 0},
            vehicle::TempMapData{
            {drawTemp_[0], drawTemp_[1], drawTemp_[2], drawTemp_[3], 0}, 0},
            prevTraction_, prevTemp_);

        // Update prev to what was actually drawn (not raw CAN values)
        memcpy(prevTraction_, drawTraction_, sizeof(prevTraction_));
        memcpy(prevTemp_, drawTemp_, sizeof(prevTemp_));
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

    // ---- OVERLAY LAYER (Z-order 2) — rendered after base tiles ----
    // Overlay visibility was precomputed in update(). draw() only consumes it.

    // TILE: Degraded/limp mode overlay (HMI_STATE_MODEL §2.4)
    if (tiles_.isDirty(DTILE_DEGRADED)) {
        drawDegradedOverlay();
        // Overlay invalidation: when banner is removed, restore underlying OBSTACLE tile
        if (prevDegradedVisible_ && !curDegradedVisible_) {
            tiles_.markDirty(DTILE_OBSTACLE);
        }
        prevDegradedVisible_ = curDegradedVisible_;
        tiles_.markClean(DTILE_DEGRADED);
    }

    // TILE: Fault flag visual overlays (HMI_STATE_MODEL §4.1)
    if (tiles_.isDirty(DTILE_FAULTS)) {
        drawFaultOverlays();
        // Overlay invalidation: when faults are cleared, restore underlying top-bar tiles
        if (prevFaultsVisible_ && !curFaultsVisible_) {
            tiles_.markDirty(DTILE_MODE_ICONS);
            tiles_.markDirty(DTILE_LED_TOGGLE);
            tiles_.markDirty(DTILE_BATTERY);
        }
        prevFaultsVisible_ = curFaultsVisible_;
        tiles_.markClean(DTILE_FAULTS);
    }

    // TILE: ACK visual feedback indicator (event-driven)
    if (tiles_.isDirty(DTILE_ACK)) {
        drawAckIndicator();
        ackIndicatorDirty_ = false;   // Clear event flag AFTER render (draw purity)
        // Overlay invalidation: when ACK clears, restore underlying LED toggle tile
        if (prevAckVisible_ && !curAckVisible_) {
            tiles_.markDirty(DTILE_LED_TOGGLE);
        }
        prevAckVisible_ = curAckVisible_;
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
    tft.setTextPadding(ui::cfg::PAD_SPEED);
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
    // NOTE: ackIndicatorDirty_ is cleared by the caller (draw()) AFTER this
    // returns, preserving draw-phase purity — draw helpers must not modify
    // event-driven flags consumed by update().

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
    tft.setTextPadding(ui::cfg::PAD_ACK);
    tft.drawString(text, ui::cfg::ACK_X + ui::cfg::ACK_W / 2, ui::cfg::ACK_Y + 2);
    RTRACE_TEXT(ui::cfg::ACK_X + ui::cfg::ACK_W / 2, ui::cfg::ACK_Y + 2, text,
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
    tft.fillRect(ui::cfg::OVL_DEGRADED_X, ui::cfg::OVL_DEGRADED_Y,
                 ui::cfg::OVL_DEGRADED_W, ui::cfg::OVL_DEGRADED_H, ui::COL_BG);
    RTRACE_FILL_RECT(ui::cfg::OVL_DEGRADED_X, ui::cfg::OVL_DEGRADED_Y,
                     ui::cfg::OVL_DEGRADED_W, ui::cfg::OVL_DEGRADED_H, ui::COL_BG);

    const char* bannerText = nullptr;

    if (curSystemState_ == can::SystemState::DEGRADED) {
        bannerText = "DEGRADED MODE - 40% POWER";
    } else if (curSystemState_ == can::SystemState::LIMP_HOME) {
        bannerText = "LIMP HOME - REDUCED SPEED";
    }

    if (bannerText != nullptr) {
        tft.fillRect(ui::cfg::OVL_DEGRADED_X, ui::cfg::OVL_DEGRADED_Y,
                     ui::cfg::OVL_DEGRADED_W, ui::cfg::OVL_DEGRADED_H, ui::COL_AMBER);
        RTRACE_FILL_RECT(ui::cfg::OVL_DEGRADED_X, ui::cfg::OVL_DEGRADED_Y,
                         ui::cfg::OVL_DEGRADED_W, ui::cfg::OVL_DEGRADED_H, ui::COL_AMBER);
        tft.setTextColor(ui::COL_BLACK, ui::COL_AMBER);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(bannerText, ui::cfg::OVL_DEGRADED_W / 2,
                        ui::cfg::OVL_DEGRADED_Y + ui::cfg::OVL_DEGRADED_H / 2);
        RTRACE_TEXT(ui::cfg::OVL_DEGRADED_W / 2,
                    ui::cfg::OVL_DEGRADED_Y + ui::cfg::OVL_DEGRADED_H / 2, bannerText,
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
    tft.fillRect(0, ui::cfg::OVL_FAULT_Y, ui::SCREEN_W, ui::cfg::OVL_FAULT_H, ui::COL_BG);
    RTRACE_FILL_RECT(0, ui::cfg::OVL_FAULT_Y, ui::SCREEN_W, ui::cfg::OVL_FAULT_H, ui::COL_BG);

    if (curFaultFlags_ == 0) return;  // No faults active

    // Build compact fault indicator string
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);

    int16_t x = 4;

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
            tft.drawString(e.label, x, ui::cfg::OVL_FAULT_Y);
            RTRACE_TEXT(x, ui::cfg::OVL_FAULT_Y, e.label,
                        e.color, ui::COL_BG, 1, TL_DATUM);
            x += static_cast<int16_t>(tft.textWidth(e.label) + ui::cfg::OVL_FAULT_LABEL_GAP);
        }
    }
}
