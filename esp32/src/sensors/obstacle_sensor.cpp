// =============================================================================
// ESP32-S3 — Obstacle Sensor Driver (implementation)
//
// Reads HC-SR04-compatible ultrasonic sensor via TRIG/ECHO GPIO.
// Validates readings against physical range, detects stuck sensor,
// and provides a warmup period for sensor stabilization.
//
// Sampling: >= 20 Hz (default 25 Hz, configurable).
// Matches STM32 expectations for CAN 0x208 payload.
//
// Reference: docs/FAIL_OPERATIONAL_MIGRATION_AUDIT.md — Step 5
// =============================================================================

#include "obstacle_sensor.h"
#include <Arduino.h>

namespace obstacle_sensor {

// -------------------------------------------------------------------------
// Module state
// -------------------------------------------------------------------------
static Config       cfg_;
static Reading      reading_;
static unsigned long initTimeMs_       = 0;
static unsigned long lastSampleMs_     = 0;
static unsigned long lastValidMs_      = 0;
static uint16_t      prevDistanceMm_   = 0;
static unsigned long stuckSinceMs_     = 0;
static bool          stuckActive_      = false;
static bool          initialized_      = false;

// -------------------------------------------------------------------------
// Zone mapping — matches STM32 distance tiers (safety_system.c)
//   < 200 mm  → zone 3 (emergency, scale=0.0)
//   200–500   → zone 2 (critical,  scale=0.3)
//   500–1000  → zone 1 (warning,   scale=0.7)
//   > 1000    → zone 0 (normal,    scale=1.0)
// -------------------------------------------------------------------------
static uint8_t distanceToZone(uint16_t mm) {
    if (mm < 200)  return 3;
    if (mm < 500)  return 2;
    if (mm < 1000) return 1;
    return 0;
}

// -------------------------------------------------------------------------
// Perform a single ultrasonic measurement
// Returns distance in mm, or 0 on timeout/failure.
// -------------------------------------------------------------------------
static uint16_t measureOnce() {
    // Send 10 µs trigger pulse
    digitalWrite(cfg_.trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(cfg_.trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(cfg_.trigPin, LOW);

    // Measure echo pulse duration (µs)
    unsigned long timeoutUs = static_cast<unsigned long>(cfg_.sensorTimeoutMs) * 1000UL;
    unsigned long durationUs = pulseIn(cfg_.echoPin, HIGH, timeoutUs);

    if (durationUs == 0) {
        return 0;  // Timeout — no echo received
    }

    // Speed of sound: ~343 m/s at 20°C → 0.343 mm/µs
    // Round-trip: distance_mm = durationUs * 0.343 / 2 = durationUs * 0.1715
    // Integer approximation: distance_mm = durationUs * 1715 / 10000
    uint32_t distMm = (static_cast<uint32_t>(durationUs) * 1715UL) / 10000UL;

    if (distMm > 0xFFFF) return 0xFFFF;
    return static_cast<uint16_t>(distMm);
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init(const Config& cfg) {
    cfg_ = cfg;
    pinMode(cfg_.trigPin, OUTPUT);
    pinMode(cfg_.echoPin, INPUT);
    digitalWrite(cfg_.trigPin, LOW);

    initTimeMs_    = millis();
    lastSampleMs_  = 0;
    lastValidMs_   = 0;
    prevDistanceMm_ = 0;
    stuckSinceMs_  = 0;
    stuckActive_   = false;
    initialized_   = true;

    reading_ = Reading{};  // Reset to defaults

    Serial.println("[OBSTACLE] Sensor initialized");
}

void update(float vehicleSpeedKmh) {
    if (!initialized_) return;

    unsigned long now = millis();

    // Rate-limit sampling
    if (now - lastSampleMs_ < cfg_.sampleIntervalMs) return;
    lastSampleMs_ = now;

    // Warmup period — report WAITING
    if (now - initTimeMs_ < cfg_.warmupMs) {
        reading_.status  = SensorStatus::WAITING;
        reading_.healthy = false;
        reading_.stuck   = false;
        return;
    }

    // Take measurement
    uint16_t rawMm = measureOnce();

    // Range validation
    bool inRange = (rawMm >= cfg_.minRangeMm && rawMm <= cfg_.maxRangeMm);

    if (rawMm == 0 || !inRange) {
        // Sensor timeout or out-of-range
        if (lastValidMs_ > 0 && (now - lastValidMs_) > cfg_.sensorTimeoutMs) {
            reading_.status  = SensorStatus::INVALID;
            reading_.healthy = false;
        }
        // Keep last known distance but mark unhealthy
        reading_.healthy = false;
        reading_.status  = SensorStatus::INVALID;
        return;
    }

    // Valid reading — update distance
    reading_.distance_mm = rawMm;
    reading_.zone        = distanceToZone(rawMm);
    lastValidMs_         = now;

    // Stuck-sensor detection:
    // If distance unchanged (within threshold) while vehicle moving > threshold
    uint16_t diff = (rawMm > prevDistanceMm_)
                    ? (rawMm - prevDistanceMm_)
                    : (prevDistanceMm_ - rawMm);

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

    prevDistanceMm_ = rawMm;

    reading_.stuck   = stuckActive_;
    reading_.healthy = !stuckActive_;
    reading_.status  = stuckActive_ ? SensorStatus::INVALID : SensorStatus::VALID;
}

Reading getReading() {
    return reading_;
}

} // namespace obstacle_sensor
