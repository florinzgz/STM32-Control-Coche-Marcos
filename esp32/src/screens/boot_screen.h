// =============================================================================
// ESP32-S3 HMI — Boot Screen
//
// Static splash screen shown during system_state = BOOT (0).
// Displays firmware name, CAN link status, and CAN diagnostic panel.
// No interactive elements. No heap allocation.
//
// The diagnostic panel shows:
//   - TWAI bus state and error counters
//   - ESP32→STM32 TX frame summary (= what STM32 receives)
//   - STM32→ESP32 RX frame summary (= what STM32 transmits)
//   - Bus error counters (tx_fail, rx_miss, bus_err)
//
// Reference: docs/HMI_STATE_MODEL.md §2.1
// =============================================================================

#ifndef BOOT_SCREEN_H
#define BOOT_SCREEN_H

#include "screen.h"
#include "../sensors/obstacle_sensor.h"

class BootScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data) override;
    void draw()    override;

private:
    bool needsRedraw_ = true;
    bool canLinked_    = false;
    bool prevCanLinked_ = false;
    obstacle_sensor::SensorStatus sensorStatus_     = obstacle_sensor::SensorStatus::WAITING;
    obstacle_sensor::SensorStatus prevSensorStatus_  = obstacle_sensor::SensorStatus::WAITING;

    // CAN diagnostic state (captured in update, rendered in draw)
    bool     diagNeedsRedraw_ = true;
    uint8_t  diagBusState_    = 0xFF;   // twai_state_t stored as uint8_t
    uint32_t diagTxErr_       = 0;
    uint32_t diagRxErr_       = 0;
    uint32_t diagTxFail_      = 0;
    uint32_t diagRxMiss_      = 0;
    uint32_t diagBusErr_      = 0;
    uint16_t diagRxFlags_     = 0;      // Bitmask: which STM32 frame types received
    bool     diagObsActive_   = false;  // Obstacle sensor TX active
};

#endif // BOOT_SCREEN_H
