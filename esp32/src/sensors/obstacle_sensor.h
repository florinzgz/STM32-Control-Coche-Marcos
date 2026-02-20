// =============================================================================
// ESP32-S3 — Obstacle Sensor Driver
//
// Reads distance from ultrasonic sensor (HC-SR04 compatible) connected to
// ESP32 GPIO.  Provides validated readings with stuck-sensor detection,
// warmup filtering, and physical-range validation.
//
// Sampling rate: >= 20 Hz (configurable via SAMPLE_INTERVAL_MS).
// Output: distance_mm, zone, health flag, stuck flag, sensor status.
//
// Reference: docs/FAIL_OPERATIONAL_MIGRATION_AUDIT.md — Step 5
//            docs/CAN_CONTRACT_FINAL.md rev 1.3 (0x208 payload)
// =============================================================================

#ifndef OBSTACLE_SENSOR_H_DRIVER
#define OBSTACLE_SENSOR_H_DRIVER

#include <cstdint>

namespace obstacle_sensor {

// -------------------------------------------------------------------------
// Sensor status reported to HMI boot screen
// -------------------------------------------------------------------------
enum class SensorStatus : uint8_t {
    WAITING,     // Warmup period — not yet producing valid data
    INVALID,     // Timeout, out-of-range, or stuck
    VALID        // Healthy reading within physical range
};

// -------------------------------------------------------------------------
// Validated sensor reading — consumed by can_obstacle for 0x208 frame
// -------------------------------------------------------------------------
struct Reading {
    uint16_t     distance_mm  = 0;      // Measured distance (mm), 0 = no reading
    uint8_t      zone         = 0;      // Distance zone (0–3): 0=far, 1=warn, 2=crit, 3=emergency
    bool         healthy      = false;  // true if reading is valid and plausible
    bool         stuck        = false;  // true if stuck-sensor condition detected
    SensorStatus status       = SensorStatus::WAITING;
};

// -------------------------------------------------------------------------
// Configuration — GPIO pins for ultrasonic sensor
// -------------------------------------------------------------------------
struct Config {
    int      trigPin            = 6;     // GPIO for TRIG output
    int      echoPin            = 7;     // GPIO for ECHO input
    uint32_t sampleIntervalMs   = 40;    // Sampling interval (40 ms = 25 Hz, >= 20 Hz)
    uint32_t warmupMs           = 1000;  // Warmup period after init (ms)
    uint16_t minRangeMm         = 20;    // Minimum physical range (mm)
    uint16_t maxRangeMm         = 4000;  // Maximum physical range (mm)
    uint32_t sensorTimeoutMs    = 100;   // Max time to wait for echo (ms)
    uint32_t stuckDurationMs    = 1000;  // Duration for stuck detection (ms)
    uint16_t stuckThresholdMm   = 10;    // Change threshold for stuck detection (mm)
    float    minSpeedForStuck   = 1.0f;  // Vehicle speed threshold (km/h) for stuck detection
};

/// Initialize sensor hardware.  Call once from setup().
void init(const Config& cfg = Config{});

/// Update sensor reading.  Call from loop() every iteration.
/// @param vehicleSpeedKmh  Current vehicle speed for stuck detection.
void update(float vehicleSpeedKmh);

/// Get the latest validated reading.
Reading getReading();

} // namespace obstacle_sensor

#endif // OBSTACLE_SENSOR_H_DRIVER
