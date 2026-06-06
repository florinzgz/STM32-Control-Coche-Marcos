// =============================================================================
// ESP32-S3 HMI — Boot Screen
//
// Static splash screen shown during system_state = BOOT (0).
// Displays firmware name, CAN link status, and CAN diagnostic panel.
//
// TILE-BASED DIRTY REGION ENGINE:
//   BTILE_CAN_STATUS  — CAN link status (LINKED / WAITING)
//   BTILE_SENSOR      — obstacle sensor status
//   BTILE_DIAGNOSTICS — CAN diagnostic panel (ESP32 bus, STM32 heartbeat, etc.)
//
// Reference: docs/HMI_STATE_MODEL.md §2.1
// =============================================================================

#ifndef BOOT_SCREEN_H
#define BOOT_SCREEN_H

#include "screen.h"
#include "ui/tile_engine.h"
#include "../sensors/obstacle_sensor.h"
#include <cstdint>

/// Tile indices for BootScreen
enum BootTile : uint8_t {
    BTILE_CAN_STATUS = 0,
    BTILE_SENSOR,
    BTILE_DIAGNOSTICS,
    BTILE_COUNT
};

class BootScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data, unsigned long frameTimeMs) override;
    void draw()    override;

private:
    static constexpr uint8_t DIAG_LINE_COUNT = 5;
    static constexpr uint8_t DIAG_TEXT_MAX   = 64;

    ui::TileSet<BTILE_COUNT> tiles_;

    bool needsRedraw_ = true;
    bool canLinked_    = false;
    bool prevCanLinked_ = false;
    obstacle_sensor::SensorStatus sensorStatus_     = obstacle_sensor::SensorStatus::WAITING;
    obstacle_sensor::SensorStatus prevSensorStatus_  = obstacle_sensor::SensorStatus::WAITING;
    uint16_t sensorDistanceMm_     = 0;
    uint16_t prevSensorDistanceMm_ = 0;

    // CAN diagnostic state (captured in update, rendered in draw)
    uint8_t  diagBusState_    = 0xFF;   // twai_state_t stored as uint8_t
    uint32_t diagTxErr_       = 0;
    uint32_t diagRxErr_       = 0;
    uint32_t diagTxFail_      = 0;
    uint32_t diagRxMiss_      = 0;
    uint32_t diagBusErr_      = 0;
    uint16_t diagRxFlags_     = 0;      // Bitmask: which STM32 frame types received

    // STM32 heartbeat diagnostic state (freeze detection + status)
    bool     diagStm32HbValid_      = false;  // Heartbeat received recently?
    bool     diagStm32Frozen_       = false;  // Alive counter not incrementing?
    uint8_t  diagStm32Alive_        = 0;      // Latest alive counter value
    uint8_t  diagStm32PrevAlive_    = 0xFF;   // Previous alive counter for freeze detect
    unsigned long diagStm32AliveChangedMs_ = 0; // Timestamp when counter last changed
    uint8_t  diagStm32State_        = 0xFF;   // System state from heartbeat
    uint8_t  diagStm32Faults_       = 0;      // Fault flags byte from heartbeat
    uint8_t  diagStm32Error_        = 0;      // Error code byte from heartbeat

    // Throttled diagnostics + line-level dirty rectangles
    unsigned long diagNextUpdateMs_ = 0;
    uint8_t  diagDirtyMask_         = 0x1F;
    char     diagLineText_[DIAG_LINE_COUNT][DIAG_TEXT_MAX] = {};
    uint16_t diagLineColor_[DIAG_LINE_COUNT] = {};
};

#endif // BOOT_SCREEN_H
