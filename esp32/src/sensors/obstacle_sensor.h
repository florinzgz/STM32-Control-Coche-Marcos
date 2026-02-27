// =============================================================================
// ESP32-S3 — Obstacle Sensor Driver (TOFSense-M by Nooploop)
//
// Reads distance from TOFSense-M 8×8 LiDAR sensor via UART.
// Parses NLink_TOFSense_M_Frame0 protocol (400-byte frames at 921600 bps).
// Provides validated readings with stuck-sensor detection,
// warmup filtering, and physical-range validation.
//
// Sensor output rate: ~10 Hz (active mode).
// Output: distance_mm, zone, health flag, stuck flag, sensor status.
//
// Hardware: TOFSense-M S (Nooploop) — GH1.25 4-pin connector
//   VCC (3.3 V or 5 V), GND, RX (not used), TX → ESP32 RX (GPIO 18)
//
// Reference: TOFSense-M User Manual V3.0
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
    uint8_t      zone         = 0;      // Distance zone (0–4): 0=far, 1=caution, 2=warn, 3=crit, 4=emergency
    bool         healthy      = false;  // true if reading is valid and plausible
    bool         stuck        = false;  // true if stuck-sensor condition detected
    SensorStatus status       = SensorStatus::WAITING;
};

// -------------------------------------------------------------------------
// Configuration — UART for TOFSense-M LiDAR sensor
// -------------------------------------------------------------------------
struct Config {
    int      rxPin             = 18;    // GPIO for UART1 RX (sensor TX → ESP32 RX)
    int      txPin             = -1;    // GPIO for UART1 TX (-1 = not connected, receive-only)
    uint32_t baudRate          = 921600; // TOFSense-M default baud rate
    uint32_t warmupMs          = 1000;  // Warmup period after init (ms)
    uint16_t minRangeMm        = 20;    // Minimum physical range (mm)
    uint16_t maxRangeMm        = 4000;  // Maximum physical range (mm)
    uint32_t frameTimeoutMs    = 500;   // Max time without valid frame before INVALID
    uint32_t stuckDurationMs   = 1000;  // Duration for stuck detection (ms)
    uint16_t stuckThresholdMm  = 10;    // Change threshold for stuck detection (mm)
    float    minSpeedForStuck  = 1.0f;  // Vehicle speed threshold (km/h) for stuck detection
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
