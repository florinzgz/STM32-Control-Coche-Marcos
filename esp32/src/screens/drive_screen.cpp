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

    // ---- Initialize tile regions (dimensions from ui_common.h) ----
    // Speed & RPM analog dials live in the bottom corners.
    tiles_.setRect(DTILE_SPEED,    ui::SPEED_DIAL_CX - ui::DIAL_R - 2,
                                   ui::SPEED_DIAL_CY - ui::DIAL_R - 2,
                                   2 * ui::DIAL_R + 4, 2 * ui::DIAL_R + 4);
    tiles_.setRect(DTILE_RPM,      ui::RPM_DIAL_CX - ui::DIAL_R - 2,
                                   ui::RPM_DIAL_CY - ui::DIAL_R - 2,
                                   2 * ui::DIAL_R + 4, 2 * ui::DIAL_R + 4);
    tiles_.setRect(DTILE_OBSTACLE,   0,   ui::SENSOR_Y, ui::SCREEN_W, ui::SENSOR_H);
    tiles_.setRect(DTILE_WHEELS,     0,   ui::CAR_AREA_Y,
                   ui::SCREEN_W, ui::CAR_AREA_H);
    tiles_.setRect(DTILE_STEERING,   ui::STEER_TILE_X, ui::CAR_AREA_Y,
                   ui::STEER_TILE_W, ui::CAR_AREA_H);
    tiles_.setRect(DTILE_BATTERY,    ui::BAT_X, 0,
                   ui::cfg::DTILE_BATTERY_W, ui::TOP_BAR_H);
    tiles_.setRect(DTILE_CAN,        ui::CAN_IND_X, 0, ui::CAN_IND_W, ui::TOP_BAR_H);
    tiles_.setRect(DTILE_AMBIENT,    ui::AMB_X, 0, ui::AMB_W, ui::TOP_BAR_H);
    tiles_.setRect(DTILE_GEAR,       ui::CLUSTER_X, ui::DGEAR_Y - 2,
                                     ui::CLUSTER_W, ui::DGEAR_H + 4);
    tiles_.setRect(DTILE_PEDAL,      ui::DTHR_X - 14, ui::DTHR_Y - 2,
                                     ui::DTHR_W + 20, ui::DTHR_H + 4);
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
    prevRpmAvg_      = 0;
    prevBattVoltRaw_ = 0;
    prevBattStale_   = false;
    prevCanOk_       = true;
    prevAmbientTemp_ = 0;
    prevPedalPct_    = 0;
    prevGear_        = ui::Gear::P;
    prevMode_        = {};
    prevObstacleCm_  = 0;
    prevFrontLedOn_  = false;
    prevRearLedOn_   = false;
    prevRelayStatus_ = 0;

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

    // Reset hash failsafe counter
    failsafeFrameCount_ = 0;

    // Reset tank turn confirmation dialog
    tankConfirmVisible_ = false;
    tankConfirmDirty_   = true;
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

    // Ambient temperature (TempMapData.temps[4], CAN 0x206) — top-bar read-out.
    curAmbientTemp_ = data.tempMap().temps[4];

    // Wheel telemetry staleness: a never-received (timestampMs==0) or expired
    // traction/temp frame must not be shown as a real reading.  When stale the
    // WHEELS tile renders "--" / "N/A" instead of leftover defaults (the 0x205
    // scale defaults to 100 and tempMap to 0 °C before any frame arrives).
    {
        unsigned long tts = data.traction().timestampMs;
        curTractionStale_ = (tts == 0) ||
                            ((frameTimeMs - tts) > can::CAN_LOSS_TIMEOUT_MS);
        unsigned long mts = data.tempMap().timestampMs;
        curTempStale_ = (mts == 0) ||
                        ((frameTimeMs - mts) > can::CAN_LOSS_TIMEOUT_MS);
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

    // Wheel RPM from average speed:
    //   RPM = speed_kmh / (3.6 × WHEEL_CIRCUMF_M) × 60
    //       = speed_kmh × 60 / (3.6 × 1.1)
    //       = speed_kmh × 60 / 3.96
    //   With raw in 0.1 km/h units (speed_kmh = raw / 10):
    //       RPM = (raw / 10) × 60 / 3.96 = raw × 6 / 3.96
    //           = raw × 600 / 396 = raw × 100 / 66  (simplified)
    //   Max overflow: 65535 × 100 = 6 553 500, fits in uint32_t.
    //   Integer truncation ≤ 1 RPM — acceptable for display.
    {
        uint32_t rpm = static_cast<uint32_t>(curSpeedAvgRaw_) * 100 / 66;
        if (rpm > ui::cfg::RPM_DISPLAY_MAX) rpm = ui::cfg::RPM_DISPLAY_MAX;
        curRpmAvg_ = static_cast<uint16_t>(rpm);
    }

    // Battery voltage
    curBattVoltRaw_ = data.battery().voltageRaw;

    // Battery staleness: the 0x207 frame is no longer trustworthy if it was
    // never received or is older than the CAN-loss timeout.  A stale reading
    // is shown as "--" (see BatteryIndicator::draw) instead of a frozen value
    // such as a misleading "100%" left over from before the link dropped.
    {
        unsigned long bts = data.battery().timestampMs;
        curBattStale_ = (bts == 0) ||
                        ((frameTimeMs - bts) > can::CAN_LOSS_TIMEOUT_MS);
    }

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

    // LED relay states from STM32 (0x20A).  When the lights frame is stale
    // (never received or older than the CAN-loss timeout) we must NOT keep
    // showing the last "ON" state — a frozen indicator made the dash look
    // like the lights were energised when they were not.  Fall back to the
    // de-energised (OFF) representation, which is the safe assumption when
    // the relay state is unknown.
    {
        unsigned long lts = data.lights().timestampMs;
        bool lightsStale = (lts == 0) ||
                           ((frameTimeMs - lts) > can::CAN_LOSS_TIMEOUT_MS);
        curFrontLedOn_ = lightsStale ? false : data.lights().frontRelayOn;
        curRearLedOn_  = lightsStale ? false : data.lights().rearRelayOn;
    }

    // Relay status (heartbeat byte 5)
    curRelayStatus_ = data.heartbeat().relayStatus;

    // System state for degraded/limp overlay (HMI_STATE_MODEL §2.4)
    curSystemState_ = data.heartbeat().systemState;

    // Fault flags for visual overlays (HMI_STATE_MODEL §4.1)
    curFaultFlags_ = data.heartbeat().faultFlags;

    // CAN-link health for the top-bar indicator: the heartbeat (0x001) is the
    // STM32 liveness beacon.  A never-received or expired heartbeat means the
    // CAN link is down — shown as a red "CAN" indicator (presentation only;
    // no control logic is derived from this).
    {
        unsigned long hts = data.heartbeat().timestampMs;
        curCanOk_ = (hts != 0) &&
                    ((frameTimeMs - hts) <= can::CAN_LOSS_TIMEOUT_MS);
    }

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

    // RPM tile (derived from average speed)
    tiles_.updateHash(DTILE_RPM, ui::tileHashVal(curRpmAvg_));

    // CAN link tile
    tiles_.updateHash(DTILE_CAN, ui::tileHashVal(curCanOk_ ? 1u : 0u));

    // AMBIENT temperature tile (value + staleness of the temp-map frame)
    {
        ui::TileHash ah = ui::tileHashVal(static_cast<uint8_t>(curAmbientTemp_));
        ah = ui::tileHashFeed(ah, curTempStale_ ? 1u : 0u);
        tiles_.updateHash(DTILE_AMBIENT, ah);
    }

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
        wh = ui::tileHashFeed(wh, curTractionStale_ ? 1u : 0u);
        wh = ui::tileHashFeed(wh, curTempStale_ ? 1u : 0u);
        tiles_.updateHash(DTILE_WHEELS, wh);
    }

    // STEERING tile
    tiles_.updateHash(DTILE_STEERING, ui::tileHashVal(curSteeringRaw_));

    // BATTERY tile
    {
        ui::TileHash bh = ui::tileHashVal(curBattVoltRaw_);
        bh = ui::tileHashFeed(bh, curBattStale_ ? 1u : 0u);
        tiles_.updateHash(DTILE_BATTERY, bh);
    }

    // GEAR tile (includes relay indicator — they share the same vertical strip)
    {
        ui::TileHash gh = ui::tileHashVal(curGear_);
        gh = ui::tileHashFeed(gh, curRelayStatus_);
        tiles_.updateHash(DTILE_GEAR, gh);
    }

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

    // ---- Hash failsafe: staggered forced redraw of critical tiles (V10) ----
    // Distribute forced redraws across the interval to avoid SPI spikes.
    // Each critical tile redraws at a different frame offset within the cycle.
    ++failsafeFrameCount_;
    if (failsafeFrameCount_ >= ui::cfg::HASH_FAILSAFE_INTERVAL) {
        failsafeFrameCount_ = 0;
    }
    {
        constexpr uint16_t STAGGER = ui::cfg::HASH_FAILSAFE_INTERVAL / 4;
        if (failsafeFrameCount_ == 0)               tiles_.forceRedraw(DTILE_SPEED);
        if (failsafeFrameCount_ == STAGGER)          tiles_.forceRedraw(DTILE_FAULTS);
        if (failsafeFrameCount_ == STAGGER * 2)      tiles_.forceRedraw(DTILE_DEGRADED);
        if (failsafeFrameCount_ == STAGGER * 3)      tiles_.forceRedraw(DTILE_BATTERY);
    }

    // ---- Critical tile policy (V10): fault-condition override ----
    // When faults are active, SPEED and FAULTS tiles bypass hash suppression
    // every frame. Ensures fault visualization is always current.
    if (curFaultFlags_ != 0) {
        tiles_.forceRedraw(DTILE_SPEED);
        tiles_.forceRedraw(DTILE_FAULTS);
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

        // Top-bar accent separator (thin cyan rule under the status row).
        tft.drawLine(0, ui::TOP_BAR_H - 1, ui::SCREEN_W - 1, ui::TOP_BAR_H - 1, ui::COL_ACCENT);
        RTRACE_LINE(0, ui::TOP_BAR_H - 1, ui::SCREEN_W - 1, ui::TOP_BAR_H - 1, ui::COL_ACCENT);

        // CAN link indicator static label.
        tft.setTextColor(ui::COL_GRAY, ui::COL_BG);
        tft.setTextSize(1);
        tft.setTextDatum(ML_DATUM);
        tft.drawString("CAN", ui::CAN_IND_X + 12, ui::CAN_IND_Y + ui::CAN_IND_H / 2);
        RTRACE_TEXT(ui::CAN_IND_X + 12, ui::CAN_IND_Y + ui::CAN_IND_H / 2, "CAN",
                    ui::COL_GRAY, ui::COL_BG, 1, ML_DATUM);
        tft.setTextDatum(TL_DATUM);

        // Analog dial bezels (face + ring + ticks). Needles/values are dynamic.
        ui::DialGauge::drawStatic(tft, ui::SPEED_DIAL_CX, ui::SPEED_DIAL_CY, ui::DIAL_R);
        ui::DialGauge::drawStatic(tft, ui::RPM_DIAL_CX,   ui::RPM_DIAL_CY,   ui::DIAL_R);

        // Force all prev_* to differ from cur_* so every tile renders
        prevSpeedAvgRaw_ = curSpeedAvgRaw_ + 1;
        prevRpmAvg_      = curRpmAvg_ + 1;
        prevBattVoltRaw_ = curBattVoltRaw_ + 1;
        prevCanOk_       = !curCanOk_;
        prevAmbientTemp_ = curAmbientTemp_ + 1;
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

    // TILE: Speed analog dial (bottom-left)
    if (tiles_.isDirty(DTILE_SPEED)) {
        RTMON_ZONE_REDRAW(rtmon::Zone::SPEED);
        drawSpeedDial();
        tiles_.markClean(DTILE_SPEED);
    }

    // TILE: RPM analog dial (bottom-right)
    if (tiles_.isDirty(DTILE_RPM)) {
        RTMON_ZONE_REDRAW(rtmon::Zone::SPEED);
        drawRpmDial();
        tiles_.markClean(DTILE_RPM);
    }

    // TILE: CAN link status (top bar)
    if (tiles_.isDirty(DTILE_CAN)) {
        RTMON_ZONE_REDRAW(rtmon::Zone::TOP_BAR);
        drawCanStatus();
        tiles_.markClean(DTILE_CAN);
    }

    // TILE: Ambient temperature (top bar)
    if (tiles_.isDirty(DTILE_AMBIENT)) {
        RTMON_ZONE_REDRAW(rtmon::Zone::TOP_BAR);
        drawAmbientTemp();
        prevAmbientTemp_ = curAmbientTemp_;
        tiles_.markClean(DTILE_AMBIENT);
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

        // drawWheels always repaints all four capsules, and the tile hash folds
        // in the stale flags, so a stale↔live transition is already a guaranteed
        // redraw — no prev-cache invalidation needed here.
        ui::CarRenderer::drawWheels(tft, vehicle::TractionData{
            {drawTraction_[0], drawTraction_[1], drawTraction_[2], drawTraction_[3]}, 0},
            vehicle::TempMapData{
            {drawTemp_[0], drawTemp_[1], drawTemp_[2], drawTemp_[3], 0}, 0},
            !curTractionStale_, !curTempStale_);

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
        ui::BatteryIndicator::draw(tft, curBattVoltRaw_, prevBattVoltRaw_,
                                   curBattStale_, prevBattStale_);
        prevBattStale_ = curBattStale_;
        tiles_.markClean(DTILE_BATTERY);
    }

    // TILE: Gear + Relay indicator (share same tile region)
    if (tiles_.isDirty(DTILE_GEAR)) {
        RTMON_ZONE_REDRAW(rtmon::Zone::GEAR);
        ui::GearDisplay::draw(tft, curGear_, prevGear_);
        ui::RelayIndicator::draw(tft, curRelayStatus_, prevRelayStatus_);
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
        // Overlay invalidation: the fault strip (OVL_FAULT_Y..+H) clips the
        // lower edge of every top-bar widget.  When the faults clear, force a
        // full repaint of the clipped widgets so no ghost pixels remain.  Mode
        // icons and LED buttons need their force-redraw entry points (their
        // differential draw() skips when the underlying state is unchanged);
        // CAN, ambient and battery fully repaint whenever their tile is dirty.
        if (prevFaultsVisible_ && !curFaultsVisible_) {
            ui::ModeIcons::redraw(tft, curMode_);
            ui::LedToggle::redraw(tft, curFrontLedOn_, curRearLedOn_);
            drawCanStatus();
            drawAmbientTemp();
            ui::BatteryIndicator::draw(tft, curBattVoltRaw_, prevBattVoltRaw_,
                                       curBattStale_, prevBattStale_);
        }
        prevFaultsVisible_ = curFaultsVisible_;
        tiles_.markClean(DTILE_FAULTS);
    }

    // TILE: ACK visual feedback indicator (event-driven)
    if (tiles_.isDirty(DTILE_ACK)) {
        drawAckIndicator();
        // Overlay invalidation: the ACK pill (ACK_X..ACK_X+ACK_W) paints over
        // both LED toggle buttons.  When the pill clears it fills its area with
        // the background, erasing the buttons underneath.  The relay state has
        // not changed, so a plain markDirty + differential LedToggle::draw would
        // skip the repaint and leave a ghost rectangle over the LEDs.  Force a
        // full repaint of both buttons at their current state to restore them.
        if (prevAckVisible_ && !curAckVisible_) {
            ui::LedToggle::redraw(tft, curFrontLedOn_, curRearLedOn_);
        }
        prevAckVisible_ = curAckVisible_;
        tiles_.markClean(DTILE_ACK);
        ackIndicatorDirty_ = false;   // Clear event flag AFTER render + markClean (flag safety §16)
    }

    // Copy current values to previous for next frame
    // (traction/temp prev values managed in the wheel tile above — threshold logic)
    prevSteeringRaw_ = curSteeringRaw_;
    prevSpeedAvgRaw_ = curSpeedAvgRaw_;
    prevRpmAvg_      = curRpmAvg_;
    prevCanOk_       = curCanOk_;
    prevBattVoltRaw_ = curBattVoltRaw_;
    prevPedalPct_    = curPedalPct_;
    prevGear_        = curGear_;
    prevMode_        = curMode_;
    prevObstacleCm_  = curObstacleCm_;
    prevFrontLedOn_  = curFrontLedOn_;
    prevRearLedOn_   = curRearLedOn_;
    prevRelayStatus_ = curRelayStatus_;
    prevSystemState_ = curSystemState_;
    prevFaultFlags_  = curFaultFlags_;

    // ---- SYSTEM LAYER (Z-order 3) — confirmation dialogs ----
    if (tankConfirmVisible_ && tankConfirmDirty_) {
        drawTankConfirmBar();
        tankConfirmDirty_ = false;
    }

    RTRACE_DUMP_IF_PENDING();
}

// -------------------------------------------------------------------------
// Speed analog dial — needle + digital read-out (bottom-left).
// The dial fully clears and repaints its interior, so no prev_* is needed.
// -------------------------------------------------------------------------
void DriveScreen::drawSpeedDial() {
    // Raw is 0.1 km/h; the dial scales 0..SPEED_DIAL_MAX (=40.0 km/h).
    char buf[ui::FMT_BUF_SMALL];
    snprintf(buf, sizeof(buf), "%u", static_cast<unsigned>(curSpeedAvgRaw_ / 10));
    ui::DialGauge::draw(tft, ui::SPEED_DIAL_CX, ui::SPEED_DIAL_CY, ui::DIAL_R,
                        curSpeedAvgRaw_, ui::SPEED_DIAL_MAX,
                        ui::COL_NEEDLE, /*zoned=*/false, buf, "km/h");
}

// -------------------------------------------------------------------------
// RPM analog dial — needle + digital read-out (bottom-right).
// Progress arc is zoned (green→amber→red) like an OEM tachometer.
// -------------------------------------------------------------------------
void DriveScreen::drawRpmDial() {
    char buf[ui::FMT_BUF_SMALL];
    snprintf(buf, sizeof(buf), "%u", static_cast<unsigned>(curRpmAvg_));
    ui::DialGauge::draw(tft, ui::RPM_DIAL_CX, ui::RPM_DIAL_CY, ui::DIAL_R,
                        curRpmAvg_, ui::cfg::RPM_DISPLAY_MAX,
                        ui::COL_NEEDLE_RPM, /*zoned=*/true, buf, "rpm");
}

// -------------------------------------------------------------------------
// CAN link status — colored dot next to the static "CAN" label (top bar).
// Green = heartbeat fresh, red = link lost.  Iconographic, minimal text.
// -------------------------------------------------------------------------
void DriveScreen::drawCanStatus() {
    uint16_t col = curCanOk_ ? ui::COL_GREEN : ui::COL_RED;
    int16_t cx = ui::CAN_IND_X + 4;
    int16_t cy = ui::CAN_IND_Y + ui::CAN_IND_H / 2;

    // Status dot.
    tft.fillCircle(cx, cy, 4, col);
    RTRACE_FILL_CIRCLE(cx, cy, 4, col);
    tft.drawCircle(cx, cy, 4, ui::COL_WHITE);
    RTRACE_CIRCLE(cx, cy, 4, ui::COL_WHITE);

    // Compact link state to the right of the "CAN" label.
    const char* txt = curCanOk_ ? "OK" : "--";
    tft.setTextColor(col, ui::COL_BG);
    tft.setTextSize(1);
    tft.setTextDatum(ML_DATUM);
    tft.setTextPadding(ui::cfg::PAD_CAN_STATE);
    tft.drawString(txt, ui::CAN_IND_X + 38, cy);
    RTRACE_TEXT(ui::CAN_IND_X + 38, cy, txt, col, ui::COL_BG, 1, ML_DATUM);
    tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);
}

// -------------------------------------------------------------------------
// Ambient temperature read-out (top bar) — TempMapData.temps[4] (CAN 0x206).
// Presentation only: shows "NN°C" colour-coded, or "--" when the temp-map
// frame is stale.  No separate "AMB" caption is drawn — the °C suffix and the
// top-bar position make the ambient reading self-explanatory.
// -------------------------------------------------------------------------
void DriveScreen::drawAmbientTemp() {
    int16_t cy = ui::AMB_Y + ui::AMB_H / 2;

    char buf[8];
    uint16_t col;
    if (curTempStale_) {
        snprintf(buf, sizeof(buf), "--\xC2\xB0""C");
        col = ui::COL_GRAY;
    } else {
        snprintf(buf, sizeof(buf), "%d\xC2\xB0""C", static_cast<int>(curAmbientTemp_));
        col = ui::tempColorFull(curAmbientTemp_);
    }

    tft.setTextColor(col, ui::COL_BG);
    tft.setTextSize(2);
    tft.setTextDatum(ML_DATUM);
    tft.setTextPadding(ui::AMB_W);
    tft.drawString(buf, ui::AMB_X, cy);
    RTRACE_TEXT(ui::AMB_X, cy, buf, col, ui::COL_BG, 2, ML_DATUM);
    tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}
// A dark rounded bar with a deep-red accent block and silver "AMG" wordmark,
// echoing OEM AMG / performance clusters.  Pure presentation.
// -------------------------------------------------------------------------
void DriveScreen::drawAmgBar() {
    // Bar body.
    tft.fillRoundRect(ui::AMG_X, ui::AMG_Y, ui::AMG_W, ui::AMG_H, 5, ui::COL_GEAR_OFF);
    RTRACE_FILL_RECT(ui::AMG_X, ui::AMG_Y, ui::AMG_W, ui::AMG_H, ui::COL_GEAR_OFF);
    tft.drawRoundRect(ui::AMG_X, ui::AMG_Y, ui::AMG_W, ui::AMG_H, 5, ui::COL_GEAR_EDGE);
    RTRACE_DRAW_RECT(ui::AMG_X, ui::AMG_Y, ui::AMG_W, ui::AMG_H, ui::COL_GEAR_EDGE);

    // Red accent block on the left edge.
    tft.fillRect(ui::AMG_X + 3, ui::AMG_Y + 3, 6, ui::AMG_H - 6, ui::COL_AMG_RED);
    RTRACE_FILL_RECT(ui::AMG_X + 3, ui::AMG_Y + 3, 6, ui::AMG_H - 6, ui::COL_AMG_RED);

    // Silver wordmark.
    tft.setTextColor(ui::COL_AMG_SILVER, ui::COL_GEAR_OFF);
    tft.setTextSize(2);
    tft.setTextDatum(ML_DATUM);
    tft.drawString("AMG", ui::AMG_X + 18, ui::AMG_Y + ui::AMG_H / 2);
    RTRACE_TEXT(ui::AMG_X + 18, ui::AMG_Y + ui::AMG_H / 2, "AMG",
                ui::COL_AMG_SILVER, ui::COL_GEAR_OFF, 2, ML_DATUM);

    // Small "4MATIC" tag to the right (subtle).
    tft.setTextColor(ui::COL_GRAY, ui::COL_GEAR_OFF);
    tft.setTextSize(1);
    tft.setTextDatum(MR_DATUM);
    tft.drawString("4MATIC", ui::AMG_X + ui::AMG_W - 8, ui::AMG_Y + ui::AMG_H / 2);
    RTRACE_TEXT(ui::AMG_X + ui::AMG_W - 8, ui::AMG_Y + ui::AMG_H / 2, "4MATIC",
                ui::COL_GRAY, ui::COL_GEAR_OFF, 1, MR_DATUM);

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
        case 1:  text = "OK";  color = ui::COL_GREEN;  break;
        case 2:  text = "REJ"; color = ui::COL_RED;    break;
        case 3:  text = "TMO"; color = ui::COL_YELLOW; break;
        default: text = "";    color = ui::COL_BG;     break;
    }

    const int16_t bx = ui::cfg::ACK_X;
    const int16_t by = ui::cfg::ACK_Y;
    const int16_t bw = ui::cfg::ACK_W;
    const int16_t bh = ui::cfg::ACK_H;
    const int16_t cy = by + bh / 2;

    // Always clear the pill area first (single background fill, no flash).
    tft.fillRect(bx, by, bw, bh, ui::COL_BG);
    RTRACE_FILL_RECT(bx, by, bw, bh, ui::COL_BG);

    if (ackDisplayResult_ != 0) {
        // Colored rounded pill conveys status iconographically.
        tft.fillRoundRect(bx, by, bw, bh, bh / 2, color);
        RTRACE_FILL_RECT(bx, by, bw, bh, color);
        tft.setTextColor(ui::COL_BLACK, color);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(text, bx + bw / 2, cy);
        RTRACE_TEXT(bx + bw / 2, cy, text, ui::COL_BLACK, color, 1, MC_DATUM);
        tft.setTextDatum(TL_DATUM);
    }
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
    uint16_t bannerCol = ui::COL_AMBER;

    if (curSystemState_ == can::SystemState::DEGRADED) {
        bannerText = "DEGRADED  40%";
        bannerCol  = ui::COL_AMBER;
    } else if (curSystemState_ == can::SystemState::LIMP_HOME) {
        bannerText = "LIMP HOME";
        bannerCol  = ui::COL_RED;
    }

    if (bannerText != nullptr) {
        const int16_t bx = ui::cfg::OVL_DEGRADED_X;
        const int16_t by = ui::cfg::OVL_DEGRADED_Y;
        const int16_t bw = ui::cfg::OVL_DEGRADED_W;
        const int16_t bh = ui::cfg::OVL_DEGRADED_H;
        const int16_t cy = by + bh / 2;

        // Rounded banner body with a colored border for an OEM look.
        tft.fillRoundRect(bx, by, bw, bh, 6, ui::COL_GEAR_OFF);
        RTRACE_FILL_RECT(bx, by, bw, bh, ui::COL_GEAR_OFF);
        tft.drawRoundRect(bx, by, bw, bh, 6, bannerCol);
        RTRACE_DRAW_RECT(bx, by, bw, bh, bannerCol);

        // Warning triangle icon (left).
        const int16_t ix = bx + 16;
        tft.fillTriangle(ix, cy - 8, ix - 8, cy + 7, ix + 8, cy + 7, bannerCol);
        RTRACE_LINE(ix, cy - 8, ix - 8, cy + 7, bannerCol);
        tft.setTextColor(ui::COL_BLACK, bannerCol);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("!", ix, cy + 1);
        RTRACE_TEXT(ix, cy + 1, "!", ui::COL_BLACK, bannerCol, 1, MC_DATUM);

        // Banner text.
        tft.setTextColor(bannerCol, ui::COL_GEAR_OFF);
        tft.setTextSize(2);
        tft.setTextDatum(ML_DATUM);
        tft.drawString(bannerText, bx + 32, cy);
        RTRACE_TEXT(bx + 32, cy, bannerText, bannerCol, ui::COL_GEAR_OFF, 2, ML_DATUM);
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
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

// -------------------------------------------------------------------------
// Tank Turn Confirmation Bar — modal overlay (SYSTEM layer Z-order 3)
//
// Drawn when the user taps the 360° icon. Asks for confirmation before
// activating/deactivating tank turn mode to prevent accidental taps.
// -------------------------------------------------------------------------
void DriveScreen::drawTankConfirmBar() {
    using namespace ui::cfg;

    // Semi-transparent background bar
    tft.fillRect(TANK_CONFIRM_BAR_X, TANK_CONFIRM_BAR_Y,
                 TANK_CONFIRM_BAR_W, TANK_CONFIRM_BAR_H, ui::COL_DARK_GRAY);
    RTRACE_FILL_RECT(TANK_CONFIRM_BAR_X, TANK_CONFIRM_BAR_Y,
                     TANK_CONFIRM_BAR_W, TANK_CONFIRM_BAR_H, ui::COL_DARK_GRAY);
    tft.drawRect(TANK_CONFIRM_BAR_X, TANK_CONFIRM_BAR_Y,
                 TANK_CONFIRM_BAR_W, TANK_CONFIRM_BAR_H, ui::COL_AMBER);
    RTRACE_DRAW_RECT(TANK_CONFIRM_BAR_X, TANK_CONFIRM_BAR_Y,
                     TANK_CONFIRM_BAR_W, TANK_CONFIRM_BAR_H, ui::COL_AMBER);

    // Question text
    const char* question = curMode_.isTankTurn
                            ? "DESACTIVAR TANK TURN?"
                            : "ACTIVAR TANK TURN?";
    tft.setTextColor(ui::COL_WHITE, ui::COL_DARK_GRAY);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(question,
                   TANK_CONFIRM_BAR_X + TANK_CONFIRM_BAR_W / 2,
                   TANK_CONFIRM_BAR_Y + 12);
    RTRACE_TEXT(TANK_CONFIRM_BAR_X + TANK_CONFIRM_BAR_W / 2,
                TANK_CONFIRM_BAR_Y + 12, question,
                ui::COL_WHITE, ui::COL_DARK_GRAY, 1, MC_DATUM);

    // SÍ button (green)
    tft.fillRect(TANK_CONFIRM_YES_X, TANK_CONFIRM_YES_Y,
                 TANK_CONFIRM_YES_W, TANK_CONFIRM_YES_H, ui::COL_GREEN);
    RTRACE_FILL_RECT(TANK_CONFIRM_YES_X, TANK_CONFIRM_YES_Y,
                     TANK_CONFIRM_YES_W, TANK_CONFIRM_YES_H, ui::COL_GREEN);
    tft.setTextColor(ui::COL_BLACK, ui::COL_GREEN);
    tft.setTextSize(2);
    tft.drawString("SI", TANK_CONFIRM_YES_X + TANK_CONFIRM_YES_W / 2,
                   TANK_CONFIRM_YES_Y + TANK_CONFIRM_YES_H / 2);
    RTRACE_TEXT(TANK_CONFIRM_YES_X + TANK_CONFIRM_YES_W / 2,
                TANK_CONFIRM_YES_Y + TANK_CONFIRM_YES_H / 2, "SI",
                ui::COL_BLACK, ui::COL_GREEN, 2, MC_DATUM);

    // NO button (red)
    tft.fillRect(TANK_CONFIRM_NO_X, TANK_CONFIRM_NO_Y,
                 TANK_CONFIRM_NO_W, TANK_CONFIRM_NO_H, ui::COL_RED);
    RTRACE_FILL_RECT(TANK_CONFIRM_NO_X, TANK_CONFIRM_NO_Y,
                     TANK_CONFIRM_NO_W, TANK_CONFIRM_NO_H, ui::COL_RED);
    tft.setTextColor(ui::COL_WHITE, ui::COL_RED);
    tft.setTextSize(2);
    tft.drawString("NO", TANK_CONFIRM_NO_X + TANK_CONFIRM_NO_W / 2,
                   TANK_CONFIRM_NO_Y + TANK_CONFIRM_NO_H / 2);
    RTRACE_TEXT(TANK_CONFIRM_NO_X + TANK_CONFIRM_NO_W / 2,
                TANK_CONFIRM_NO_Y + TANK_CONFIRM_NO_H / 2, "NO",
                ui::COL_WHITE, ui::COL_RED, 2, MC_DATUM);

    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
}

void DriveScreen::showTankConfirm() {
    tankConfirmVisible_ = true;
    tankConfirmDirty_   = true;
}

uint8_t DriveScreen::handleTankConfirmTouch(int16_t x, int16_t y) {
    using namespace ui::cfg;

    if (!tankConfirmVisible_) return 0;

    // SÍ button hit test
    if (x >= TANK_CONFIRM_YES_X && x <= TANK_CONFIRM_YES_X + TANK_CONFIRM_YES_W &&
        y >= TANK_CONFIRM_YES_Y && y <= TANK_CONFIRM_YES_Y + TANK_CONFIRM_YES_H) {
        tankConfirmVisible_ = false;
        // Force redraw of tiles covered by the confirm bar
        needsFullRedraw_ = true;
        return 1;  // YES
    }

    // NO button hit test
    if (x >= TANK_CONFIRM_NO_X && x <= TANK_CONFIRM_NO_X + TANK_CONFIRM_NO_W &&
        y >= TANK_CONFIRM_NO_Y && y <= TANK_CONFIRM_NO_Y + TANK_CONFIRM_NO_H) {
        tankConfirmVisible_ = false;
        // Force redraw of tiles covered by the confirm bar
        needsFullRedraw_ = true;
        return 2;  // NO
    }

    // Touch is inside the confirm bar area but not on a button — consume it
    if (x >= TANK_CONFIRM_BAR_X && x <= TANK_CONFIRM_BAR_X + TANK_CONFIRM_BAR_W &&
        y >= TANK_CONFIRM_BAR_Y && y <= TANK_CONFIRM_BAR_Y + TANK_CONFIRM_BAR_H) {
        return 0;  // consumed but no action
    }

    // Touch outside the confirm bar — dismiss (treat as NO)
    tankConfirmVisible_ = false;
    needsFullRedraw_ = true;
    return 2;  // dismiss = NO
}
