// =============================================================================
// ESP32-S3 HMI — Drive Screen
//
// Full operational display with live telemetry.
// Drawn when system_state is ACTIVE (2) or DEGRADED (3).
//
// Layout (480×320 landscape):
//   Top bar (0–40):    mode icons (4x4, 4x2, 360°) + battery percentage
//   Sensor (40–85):    frontal obstacle distance + proximity bar
//   Center (85–230):   car top-view with 4 wheels (torque %, temp, color)
//                      steering circular gauge (right side)
//                      "360°" label above steering gauge
//   Speed (230–270):   large speed value centered
//   Pedal (270–300):   pedal bar (0-100%, gradient, percentage text)
//   Gears (300–320):   gear display (P, R, N, D1, D2) — flat text
//
// No heap allocation in update()/draw(). All formatting on stack.
// Partial redraw only — dirty flags track which elements changed.
//
// Reference: docs/HMI_RENDERING_STRATEGY.md
// =============================================================================

#ifndef DRIVE_SCREEN_H
#define DRIVE_SCREEN_H

#include "screen.h"
#include "ui/ui_common.h"
#include "ui/gear_display.h"
#include "ui/mode_icons.h"
#include <cstdint>

class DriveScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data) override;
    void draw()    override;

private:
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

    // Current frame values (populated in update, used in draw)
    uint8_t  curTraction_[4]     = {};
    int8_t   curTemp_[4]         = {};
    int16_t  curSteeringRaw_     = 0;
    uint16_t curSpeedAvgRaw_     = 0;
    uint16_t curBattVoltRaw_     = 0;
    uint8_t  curPedalPct_        = 0;
    ui::Gear curGear_            = ui::Gear::P;
    ui::ModeState curMode_       = {};
    uint16_t curObstacleCm_      = 0;

    bool     needsFullRedraw_    = true;

    void drawSpeed();
};

#endif // DRIVE_SCREEN_H
