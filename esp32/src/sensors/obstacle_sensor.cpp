// =============================================================================
// ESP32-S3 — Obstacle Sensor Driver (implementation)
//
// Reads TOFSense-M 8×8 LiDAR sensor via UART1.
// Parses NLink_TOFSense_M_Frame0 protocol (400-byte frames).
// Validates readings against physical range, detects stuck sensor,
// and provides a warmup period for sensor stabilization.
//
// Sensor: TOFSense-M S (Nooploop), UART 921600 bps, active output mode.
// Matches STM32 expectations for CAN 0x208 payload.
//
// Reference: TOFSense-M User Manual V3.0
//            https://ftp.nooploop.com/downloads/tofsense/TOFSense-M_User_Manual_V3.0_en.pdf
// =============================================================================

#include "obstacle_sensor.h"
#include <Arduino.h>
#include <HardwareSerial.h>

namespace obstacle_sensor {

// -------------------------------------------------------------------------
// NLink_TOFSense_M_Frame0 protocol constants
// -------------------------------------------------------------------------
static constexpr uint8_t  FRAME_HEADER      = 0x57;
static constexpr uint16_t FRAME_LENGTH      = 400;    // Total frame size (bytes)
static constexpr uint8_t  PIXEL_COUNT_8X8   = 64;     // 8×8 matrix
static constexpr uint8_t  BYTES_PER_PIXEL   = 6;      // 3 (distance) + 1 (signal) + 1 (status) + 1 (reserved)
// NLink protocol distance unit: 1/256 mm per LSB
static constexpr int32_t DISTANCE_UNITS_PER_MM = 256;

// Frame offsets
static constexpr uint16_t OFF_HEADER        = 0;      // 0x57
static constexpr uint16_t OFF_FUNCTION_MARK = 1;      // Function mark
static constexpr uint16_t OFF_RESERVED1     = 2;      // Reserved
static constexpr uint16_t OFF_ID            = 3;      // Sensor ID
static constexpr uint16_t OFF_SYSTEM_TIME   = 4;      // System time (4 bytes, LE)
static constexpr uint16_t OFF_PIXEL_DATA    = 8;      // Start of pixel data
// Pixel data: 64 pixels × 6 bytes = 384 bytes (offset 8..391)
// Checksum at offset 392 (8-bit sum mod 256 of bytes 0..391)

// -------------------------------------------------------------------------
// Module state
// -------------------------------------------------------------------------
static Config       cfg_;
static Reading      reading_;
static unsigned long initTimeMs_       = 0;
static unsigned long lastValidMs_      = 0;
static uint16_t      prevDistanceMm_   = 0;
static unsigned long stuckSinceMs_     = 0;
static bool          stuckActive_      = false;
static bool          initialized_      = false;

// UART receive buffer
static uint8_t       rxBuf_[FRAME_LENGTH];
static uint16_t      rxIdx_            = 0;

// UART1 for TOFSense-M
static HardwareSerial tofSerial(1);

// -------------------------------------------------------------------------
// Zone mapping — matches STM32 distance tiers (safety_system.c)
//   < 200 mm  → zone 4 (emergency, scale=0.0)
//   200–500   → zone 3 (critical,  scale=0.3)
//   500–1000  → zone 2 (warning,   scale=0.7)
//   1000–1500 → zone 1 (caution,   scale=0.85)
//   > 1500    → zone 0 (normal,    scale=1.0)
// -------------------------------------------------------------------------
static constexpr uint16_t ZONE_EMERGENCY_MM = 200;
static constexpr uint16_t ZONE_CRITICAL_MM  = 500;
static constexpr uint16_t ZONE_WARNING_MM   = 1000;
static constexpr uint16_t ZONE_CAUTION_MM   = 1500;

static uint8_t distanceToZone(uint16_t mm) {
    if (mm < ZONE_EMERGENCY_MM) return 4;
    if (mm < ZONE_CRITICAL_MM)  return 3;
    if (mm < ZONE_WARNING_MM)   return 2;
    if (mm < ZONE_CAUTION_MM)   return 1;
    return 0;
}

// -------------------------------------------------------------------------
// Parse a complete NLink_TOFSense_M_Frame0 and extract minimum distance
// Returns distance in mm, or 0 on parse failure.
// -------------------------------------------------------------------------
static uint16_t parseFrame(const uint8_t* buf, uint16_t len) {
    if (len < FRAME_LENGTH) return 0;
    if (buf[OFF_HEADER] != FRAME_HEADER) return 0;

    // Verify checksum: 8-bit sum of all bytes except the checksum byte itself
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < FRAME_LENGTH - 1; i++) {
        checksum += buf[i];
    }
    if (checksum != buf[FRAME_LENGTH - 1]) return 0;

    // Extract minimum valid distance across all 64 pixels.
    // Each pixel: 3 bytes distance (signed int24 LE, unit = 1/256 mm),
    //             1 byte signal strength,
    //             1 byte status (0 = valid),
    //             1 byte reserved.
    uint32_t minDistMm = 0xFFFFFFFF;
    bool anyValid = false;

    for (uint8_t px = 0; px < PIXEL_COUNT_8X8; px++) {
        uint16_t base = OFF_PIXEL_DATA + (uint16_t)(px * BYTES_PER_PIXEL);
        if (base + BYTES_PER_PIXEL > FRAME_LENGTH) break;

        uint8_t status = buf[base + 4];
        if (status != 0) continue;  // Skip invalid pixels

        // Distance: 3-byte signed little-endian, unit = 1/256 mm per LSB
        int32_t raw = (int32_t)buf[base]
                    | ((int32_t)buf[base + 1] << 8)
                    | ((int32_t)buf[base + 2] << 16);
        // Sign-extend 24-bit value to 32-bit: check bit 23 (sign bit)
        if (raw & 0x800000) raw |= (int32_t)0xFF000000;

        if (raw <= 0) continue;  // Negative or zero distance — skip

        uint32_t distMm = (uint32_t)(raw / DISTANCE_UNITS_PER_MM);
        if (distMm < minDistMm) {
            minDistMm = distMm;
            anyValid = true;
        }
    }

    if (!anyValid) return 0;
    if (minDistMm > 0xFFFF) return 0xFFFF;
    return (uint16_t)minDistMm;
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init(const Config& cfg) {
    cfg_ = cfg;

    // Initialize UART1 for TOFSense-M
    tofSerial.begin(cfg_.baudRate, SERIAL_8N1, cfg_.rxPin, cfg_.txPin);

    initTimeMs_    = millis();
    lastValidMs_   = 0;
    prevDistanceMm_ = 0;
    stuckSinceMs_  = 0;
    stuckActive_   = false;
    initialized_   = true;
    rxIdx_         = 0;

    reading_ = Reading{};  // Reset to defaults

    Serial.println("[OBSTACLE] TOFSense-M initialized (UART1, 921600 bps)");
}

void update(float vehicleSpeedKmh) {
    if (!initialized_) return;

    unsigned long now = millis();

    // Warmup period — report WAITING
    if (now - initTimeMs_ < cfg_.warmupMs) {
        reading_.status  = SensorStatus::WAITING;
        reading_.healthy = false;
        reading_.stuck   = false;
        return;
    }

    // Read available UART bytes and parse frames
    uint16_t measuredMm = 0;
    bool gotFrame = false;

    while (tofSerial.available() > 0) {
        uint8_t byte = (uint8_t)tofSerial.read();

        // Synchronize on header byte
        if (rxIdx_ == 0 && byte != FRAME_HEADER) {
            continue;  // Discard until we find the header
        }

        rxBuf_[rxIdx_++] = byte;

        // Complete frame received
        if (rxIdx_ >= FRAME_LENGTH) {
            uint16_t dist = parseFrame(rxBuf_, rxIdx_);
            if (dist > 0) {
                measuredMm = dist;
                gotFrame = true;
            }
            rxIdx_ = 0;  // Reset for next frame
        }
    }

    if (!gotFrame) {
        // No valid frame received — check timeout
        if (lastValidMs_ > 0 && (now - lastValidMs_) > cfg_.frameTimeoutMs) {
            reading_.status  = SensorStatus::INVALID;
            reading_.healthy = false;
        }
        return;
    }

    // Range validation
    bool inRange = (measuredMm >= cfg_.minRangeMm && measuredMm <= cfg_.maxRangeMm);

    if (!inRange) {
        reading_.healthy = false;
        reading_.status  = SensorStatus::INVALID;
        return;
    }

    // Valid reading — update distance
    reading_.distance_mm = measuredMm;
    reading_.zone        = distanceToZone(measuredMm);
    lastValidMs_         = now;

    // Stuck-sensor detection:
    // If distance unchanged (within threshold) while vehicle moving > threshold
    uint16_t diff = (measuredMm > prevDistanceMm_)
                    ? (measuredMm - prevDistanceMm_)
                    : (prevDistanceMm_ - measuredMm);

    if (diff < cfg_.stuckThresholdMm && vehicleSpeedKmh > cfg_.minSpeedForStuck) {
        if (stuckSinceMs_ == 0) {
            stuckSinceMs_ = now;
        }
        if (now - stuckSinceMs_ >= cfg_.stuckDurationMs) {
            stuckActive_ = true;
        }
    } else {
        stuckSinceMs_ = 0;
        stuckActive_  = false;
    }

    prevDistanceMm_ = measuredMm;

    reading_.stuck   = stuckActive_;
    reading_.healthy = !stuckActive_;
    reading_.status  = stuckActive_ ? SensorStatus::INVALID : SensorStatus::VALID;
}

Reading getReading() {
    return reading_;
}

} // namespace obstacle_sensor
