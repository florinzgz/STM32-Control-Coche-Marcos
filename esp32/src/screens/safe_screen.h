// =============================================================================
// ESP32-S3 HMI — Safe Screen
//
// Shown when system_state = SAFE (4).
// Prominent safety warning banner. Read-only telemetry visible:
// wheel speeds, motor currents, temperatures, steering angle.
// Extended passive visualization: gear, obstacle, LED status, relay.
//
// TILE-BASED DIRTY REGION ENGINE:
//   STILE_FAULTS    — fault flags
//   STILE_ERROR     — error code
//   STILE_SPEEDS    — wheel speed values (4 wheels)
//   STILE_CURRENTS  — motor current values (4 wheels)
//   STILE_TEMPS     — temperature values (5 sensors)
//   STILE_STEERING  — steering angle + visual direction indicator
//   STILE_OBSTACLE  — obstacle sensor distance bar
//   STILE_LED_STAT  — LED system status (front/rear/turn)
//   STILE_GEAR      — gear position bar
//   STILE_RELAY     — relay status indicator (M T D)
//   STILE_I2C       — I2C bus diagnostic (mux + per-channel INA226 health)
//
// Reference: docs/HMI_STATE_MODEL.md §2.5
// =============================================================================

#ifndef SAFE_SCREEN_H
#define SAFE_SCREEN_H

#include "screen.h"
#include "ui/tile_engine.h"
#include "ui/gear_display.h"
#include "led_controller.h"
#include <cstdint>
#include <array>

/// Tile indices for SafeScreen
enum SafeTile : uint8_t {
    STILE_FAULTS = 0,
    STILE_ERROR,
    STILE_SPEEDS,
    STILE_CURRENTS,
    STILE_TEMPS,
    STILE_STEERING,
    STILE_OBSTACLE,
    STILE_LED_STAT,
    STILE_GEAR,
    STILE_RELAY,
    STILE_I2C,
    STILE_COUNT
};

class SafeScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data, unsigned long frameTimeMs) override;
    void draw()    override;

private:
    ui::TileSet<STILE_COUNT> tiles_;

    bool    needsRedraw_    = true;
    uint8_t faultFlags_     = 0;
    uint8_t prevFaultFlags_ = 0xFF;
    uint8_t errorCode_      = 0;
    uint8_t prevErrorCode_  = 0xFF;

    // Read-only telemetry (HMI_STATE_MODEL §2.5)
    std::array<uint16_t, 4> wheelSpeed_     = {};   // 0.1 km/h units
    std::array<uint16_t, 4> prevWheelSpeed_ = {};
    std::array<uint16_t, 4> motorCurrent_   = {};   // 0.01 A units
    std::array<uint16_t, 4> prevMotorCurrent_ = {};
    std::array<int8_t, 5>   temps_          = {};   // °C
    std::array<int8_t, 5>   prevTemps_      = {};
    int16_t steeringAngle_     = 0;                  // 0.1° units
    int16_t prevSteeringAngle_ = 0;

    // Extended passive visualization (UI-only, no functional impact)
    ui::Gear curGear_        = ui::Gear::N;
    ui::Gear prevGear_       = ui::Gear::P;   // Force initial draw

    uint16_t obstacleCm_     = 0;
    uint16_t prevObstacleCm_ = 0xFFFF;        // Force initial draw

    bool     frontLedOn_      = false;
    bool     rearLedOn_       = false;
    led_ctrl::TurnSignal turnSignal_ = led_ctrl::TurnSignal::OFF;
    bool     prevFrontLedOn_  = true;          // Force initial draw
    bool     prevRearLedOn_   = true;
    led_ctrl::TurnSignal prevTurnSignal_ = led_ctrl::TurnSignal::LEFT;

    uint8_t  relayStatus_     = 0;
    uint8_t  prevRelayStatus_ = 0xFF;          // Force initial draw

    // I2C bus diagnostic (passive, read-only) — mux + per-channel INA226
    bool     i2cValid_        = false;
    bool     i2cMuxPresent_   = false;
    uint8_t  i2cInaMask_      = 0;
    uint8_t  i2cFailCount_    = 0;
    uint8_t  i2cRecovery_     = 0;
    bool     i2cEverOk_       = false;
    // Phase 1 diagnostic: age of the last 0x309 frame + derived flags.
    // `i2cStale_` is true when a frame was once received but is now older
    // than the STALE threshold (STM32 stopped sending).  `i2cHbAlive_`
    // tells a "STM32 firmware too old to send 0x309" case (heartbeat
    // alive, no 0x309) apart from a dead CAN link.
    unsigned long i2cAgeMs_   = 0;
    bool     i2cStale_        = false;
    bool     i2cHbAlive_      = false;
    // Force initial draw with mismatched previous-state snapshot
    bool     previ2cValid_      = true;
    bool     previ2cMuxPresent_ = true;
    uint8_t  previ2cInaMask_    = 0xFF;
    uint8_t  previ2cFailCount_  = 0xFF;
    uint8_t  previ2cRecovery_   = 0xFF;
    bool     previ2cEverOk_     = true;
};

#endif // SAFE_SCREEN_H
