// =============================================================================
// ESP32-S3 HMI — Standby Screen
//
// Shown when system_state = STANDBY (1).
// Displays "System Ready" with CAN link status and temperatures.
//
// TILE-BASED DIRTY REGION ENGINE:
//   YTILE_TEMPS   — temperature values (5 sensors)
//   YTILE_FAULTS  — fault flags display
//
// Reference: docs/HMI_STATE_MODEL.md §2.2
// =============================================================================

#ifndef STANDBY_SCREEN_H
#define STANDBY_SCREEN_H

#include "screen.h"
#include "ui/tile_engine.h"
#include <cstdint>

/// Tile indices for StandbyScreen
enum StandbyTile : uint8_t {
    YTILE_TEMPS = 0,
    YTILE_FAULTS,
    YTILE_COUNT
};

class StandbyScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data, unsigned long frameTimeMs) override;
    void draw()    override;

private:
    ui::TileSet<YTILE_COUNT> tiles_;

    bool    needsRedraw_   = true;
    uint8_t faultFlags_    = 0;
    uint8_t prevFaultFlags_ = 0xFF;
    int8_t  temps_[5]      = {};
    int8_t  prevTemps_[5]  = {};
};

#endif // STANDBY_SCREEN_H
