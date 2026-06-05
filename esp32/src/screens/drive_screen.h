// =============================================================================
// ESP32-S3 HMI — Drive Screen
//
// Full operational display with live telemetry.
// Drawn when system_state is ACTIVE (2) or DEGRADED (3).
//
// TILE-BASED DIRTY REGION ENGINE:
//   The screen is divided into independent tiles, each with its own dirty
//   flag and content hash. Only tiles whose data has changed since the last
//   rendered frame are redrawn. This minimises SPI transactions and
//   eliminates visual flicker from unnecessary overdraw.
//
// Tile map (480×320 landscape):
//   TILE_MODE_ICONS   — top bar left (4x4, 4x2, 360°)
//   TILE_LED_TOGGLE   — top bar left-center (front/rear LED buttons)
//   TILE_ACK          — top bar center (command ACK indicator)
//   TILE_BATTERY      — top bar right (battery percentage + bar)
//   TILE_OBSTACLE     — sensor zone (40–85 px)
//   TILE_WHEELS       — car area wheels (4 wheel rects + labels)
//   TILE_STEERING     — car area steering gauge (right side)
//   TILE_SPEED        — large centered speed display (230–270 px)
//   TILE_PEDAL        — pedal bar (270–300 px)
//   TILE_GEAR         — gear display (300–320 px)
//   TILE_DEGRADED     — overlay: degraded/limp mode banner
//   TILE_FAULTS       — overlay: fault flag indicators
//
// Pipeline per frame:
//   1. update(snapshot) → compute tile hashes from frozen VehicleData
//   2. TileSet::updateHash() → compare hash → mark dirty if changed
//   3. draw() → iterate tile set, only render dirty tiles
//   4. markClean() after each tile render
//
// No heap allocation in update()/draw(). All formatting on stack.
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef DRIVE_SCREEN_H
#define DRIVE_SCREEN_H

#include "screen.h"
#include "ui/ui_common.h"
#include "ui/ui_config.h"
#include "ui/tile_engine.h"
#include "ui/gear_display.h"
#include "ui/mode_icons.h"
#include "ui/led_toggle.h"
#include "ui/relay_indicator.h"
#include "can_ids.h"
#include <cstdint>

/// Tile indices for DriveScreen
enum DriveTile : uint8_t {
    DTILE_SPEED = 0,
    DTILE_OBSTACLE,
    DTILE_WHEELS,
    DTILE_STEERING,
    DTILE_BATTERY,
    DTILE_GEAR,
    DTILE_PEDAL,
    DTILE_MODE_ICONS,
    DTILE_LED_TOGGLE,
    DTILE_DEGRADED,
    DTILE_FAULTS,
    DTILE_ACK,
    DTILE_COUNT
};

class DriveScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data, unsigned long frameTimeMs) override;
    void draw()    override;

private:
    // Tile set — manages dirty flags and hashes for all drive screen regions
    ui::TileSet<DTILE_COUNT> tiles_;

    // Cached previous-frame values for dirty detection
    uint8_t  prevTraction_[4]    = {};
    int8_t   prevTemp_[4]        = {};
    int16_t  prevSteeringRaw_    = 0;
    uint16_t prevSpeedAvgRaw_    = 0;
    uint16_t prevBattVoltRaw_    = 0;
    uint8_t  prevPedalPct_       = 0;
    ui::Gear prevGear_           = ui::Gear::P;
    ui::ModeState prevMode_      = {};
    uint16_t prevObstacleCm_     = 0;
    bool     prevFrontLedOn_     = false;
    bool     prevRearLedOn_      = false;
    uint8_t  prevRelayStatus_    = 0;

    // Current frame values (populated in update, used in draw)
    uint8_t  curTraction_[4]     = {};
    int8_t   curTemp_[4]         = {};
    int16_t  curSteeringRaw_     = 0;
    uint16_t curSpeedAvgRaw_     = 0;
    uint16_t curRpmAvg_          = 0;
    uint16_t curBattVoltRaw_     = 0;
    uint8_t  curPedalPct_        = 0;
    ui::Gear curGear_            = ui::Gear::P;
    ui::ModeState curMode_       = {};
    uint16_t curObstacleCm_      = 0;
    bool     curFrontLedOn_      = false;
    bool     curRearLedOn_       = false;
    uint8_t  curRelayStatus_     = 0;

    // Battery reading staleness (0x207 older than CAN-loss timeout → show "--")
    bool     curBattStale_       = false;
    bool     prevBattStale_      = false;

    // Wheel telemetry staleness: traction (0x205) and temp map (0x206) older
    // than the CAN-loss timeout → render "--" / "N/A" instead of a frozen
    // default (e.g. 100% torque or 0 °C) that misrepresents the motors.
    bool     curTractionStale_   = false;
    bool     prevTractionStale_  = false;
    bool     curTempStale_       = false;
    bool     prevTempStale_      = false;

    bool     needsFullRedraw_    = true;

    // Precomputed wheel draw values (threshold-filtered in update phase)
    uint8_t  drawTraction_[4]    = {};
    int8_t   drawTemp_[4]        = {};

    // System state for degraded/limp overlays (HMI_STATE_MODEL §2.4)
    can::SystemState curSystemState_  = can::SystemState::ACTIVE;
    can::SystemState prevSystemState_ = can::SystemState::ACTIVE;

    // Fault flags for visual overlays (HMI_STATE_MODEL §4.1)
    uint8_t curFaultFlags_  = 0;
    uint8_t prevFaultFlags_ = 0;

    // Overlay visibility tracking for invalidation chain.
    // When an overlay transitions from visible→invisible, underlying
    // tiles are marked dirty to restore their content.
    // These are precomputed in update() — draw() MUST NOT recompute them.
    bool prevDegradedVisible_ = false;
    bool prevFaultsVisible_   = false;
    bool prevAckVisible_      = false;
    bool curDegradedVisible_  = false;   // precomputed in update()
    bool curFaultsVisible_    = false;   // precomputed in update()
    bool curAckVisible_       = false;   // precomputed in update()

    // ACK visual feedback state
    unsigned long ackLastShownMs_   = 0;   // frameTimeMs when indicator was last shown
    unsigned long ackTrackedAckMs_  = 0;   // last ack().timestampMs we processed
    unsigned long ackTrackedTmoMs_  = 0;   // last ackTimeoutMs() we processed
    uint8_t       ackDisplayResult_ = 0;   // 0=none, 1=OK, 2=rejected, 3=timeout
    bool          ackIndicatorDirty_ = false;

    // Hash failsafe: frame counter for periodic forced redraw of critical tiles
    uint16_t failsafeFrameCount_ = 0;

    // ---- Tank Turn Confirmation Dialog ----
    // When the user taps the 360° icon, a confirmation bar is shown
    // instead of immediately toggling tank turn mode.
    bool     tankConfirmVisible_  = false;   // true while confirm bar is shown
    bool     tankConfirmDirty_    = true;    // needs redraw

    void drawSpeed();
    void drawAckIndicator();
    void drawDegradedOverlay();
    void drawFaultOverlays();
    void drawTankConfirmBar();

public:
    /// Show the tank turn confirmation bar overlay.
    void showTankConfirm();

    /// Process a touch during confirmation. Returns:
    ///   0 = not consumed (touch outside confirm bar)
    ///   1 = YES — user confirmed, ready to toggle
    ///   2 = NO — user cancelled
    uint8_t handleTankConfirmTouch(int16_t x, int16_t y);

    /// Returns true if the tank confirm bar is currently visible.
    bool isTankConfirmVisible() const { return tankConfirmVisible_; }
};

#endif // DRIVE_SCREEN_H
