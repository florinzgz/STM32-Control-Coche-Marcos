// =============================================================================
// ESP32-S3 — CAN Obstacle TX Module (implementation)
//
// Transmits CAN 0x208 (OBSTACLE_DISTANCE) matching the frozen protocol.
// Also transmits CAN 0x209 (OBSTACLE_SAFETY) at 100 ms interval with
// the ESP32's computed obstacle safety state (informational).
//
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
static unsigned long lastTxMs_      = 0;
static unsigned long lastSafetyMs_  = 0;
static uint8_t       counter_       = 0;
static bool          initialized_   = false;

// 0x209 transmit rate (100 ms, matching can_ids.h comment)
static constexpr unsigned long SAFETY_RATE_MS = 100;

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init() {
    lastTxMs_     = 0;
    lastSafetyMs_ = 0;
    counter_      = 0;
    initialized_  = true;
    Serial.println("[CAN_OBS] Obstacle TX initialized");
}

void update() {
    if (!initialized_) return;

    unsigned long now = millis();
    obstacle_sensor::Reading rd = obstacle_sensor::getReading();

    // Failsafe: do not send frames if sensor is in warmup or uninitialized.
    // STM32 will naturally enter its 500 ms CAN timeout path.
    if (rd.status == obstacle_sensor::SensorStatus::WAITING) {
        return;
    }

    // ---- 0x208: Obstacle Distance (66 ms) ----
    if (now - lastTxMs_ >= can::OBSTACLE_RATE_MS) {
        lastTxMs_ = now;

        CanFrame frame = {};
        frame.identifier       = can::OBSTACLE_DISTANCE;  // 0x208
        frame.extd             = 0;
        frame.data_length_code = 5;

        // Bytes 0-1: distance_mm (uint16 LE)
        frame.data[0] = static_cast<uint8_t>(rd.distance_mm & 0xFF);
        frame.data[1] = static_cast<uint8_t>((rd.distance_mm >> 8) & 0xFF);

        // Byte 2: zone (0–4)
        frame.data[2] = rd.zone;

        // Byte 3: sensor_health (0=unhealthy/stuck, 1=healthy)
        frame.data[3] = (rd.healthy && !rd.stuck) ? 1 : 0;

        // Byte 4: rolling counter (0–255)
        frame.data[4] = counter_++;

        ESP32Can.writeFrame(frame);
    }

    // ---- 0x209: Obstacle Safety State (100 ms) ----
    if (now - lastSafetyMs_ >= SAFETY_RATE_MS) {
        lastSafetyMs_ = now;

        CanFrame frame = {};
        frame.identifier       = can::OBSTACLE_SAFETY;  // 0x209
        frame.extd             = 0;
        frame.data_length_code = 4;

        // Byte 0: zone (0–4)
        frame.data[0] = rd.zone;

        // Byte 1: sensor status (0=WAITING, 1=INVALID, 2=VALID)
        frame.data[1] = static_cast<uint8_t>(rd.status);

        // Byte 2: stuck flag (0=OK, 1=stuck)
        frame.data[2] = rd.stuck ? 1 : 0;

        // Byte 3: reserved
        frame.data[3] = 0;

        ESP32Can.writeFrame(frame);
    }
}

} // namespace can_obstacle
