// =============================================================================
// ESP32-S3 — CAN Obstacle TX Module (implementation)
//
// Transmits CAN 0x208 (OBSTACLE_DISTANCE) matching the frozen protocol.
// Stops transmitting when sensor is in WAITING state (warmup) or
// completely unavailable — STM32 timeout path handles fail-safe.
//
// Reference: docs/CAN_CONTRACT_FINAL.md rev 1.3
// =============================================================================

#include "can_obstacle.h"
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include "sensors/obstacle_sensor.h"
#include "can_ids.h"

namespace can_obstacle {

// -------------------------------------------------------------------------
// Module state
// -------------------------------------------------------------------------
static unsigned long lastTxMs_  = 0;
static uint8_t       counter_   = 0;
static bool          initialized_ = false;

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init() {
    lastTxMs_    = 0;
    counter_     = 0;
    initialized_ = true;
    Serial.println("[CAN_OBS] Obstacle TX initialized");
}

void update() {
    if (!initialized_) return;

    unsigned long now = millis();
    if (now - lastTxMs_ < can::OBSTACLE_RATE_MS) return;
    lastTxMs_ = now;

    obstacle_sensor::Reading rd = obstacle_sensor::getReading();

    // Failsafe: do not send frame if sensor is in warmup or uninitialized.
    // STM32 will naturally enter its 500 ms CAN timeout path.
    if (rd.status == obstacle_sensor::SensorStatus::WAITING) {
        return;
    }

    // Build CAN frame matching frozen protocol
    CanFrame frame = {};
    frame.identifier       = can::OBSTACLE_DISTANCE;  // 0x208
    frame.extd             = 0;
    frame.data_length_code = 5;

    // Bytes 0-1: distance_mm (uint16 LE)
    frame.data[0] = static_cast<uint8_t>(rd.distance_mm & 0xFF);
    frame.data[1] = static_cast<uint8_t>((rd.distance_mm >> 8) & 0xFF);

    // Byte 2: zone (0–3)
    frame.data[2] = rd.zone;

    // Byte 3: sensor_health (0=unhealthy/stuck, 1=healthy)
    frame.data[3] = (rd.healthy && !rd.stuck) ? 1 : 0;

    // Byte 4: rolling counter (0–255)
    frame.data[4] = counter_++;

    ESP32Can.writeFrame(frame);
}

} // namespace can_obstacle
