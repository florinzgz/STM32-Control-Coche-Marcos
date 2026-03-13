// =============================================================================
// ESP32-S3 — Shifter Input Driver (MCP23017 I2C) — Implementation
//
// Reads gear selector position from MCP23017 Port A pins (active-low).
// Pin mapping:
//   GPA0 = Park, GPA1 = Reverse, GPA2 = Neutral,
//   GPA3 = Drive (D1), GPA4 = Drive2 (D2)
//
// If no pin is active or multiple pins are active, defaults to Neutral.
//
// Reference: docs/CAN_CONTRACT_FINAL.md §4.5
// =============================================================================

#include "shifter_input.h"
#include <Wire.h>
#include <Arduino.h>

namespace shifter {

// MCP23017 register addresses (BANK=0 mode, default)
static constexpr uint8_t REG_IODIRA   = 0x00;  // I/O Direction A
static constexpr uint8_t REG_GPPUA    = 0x0C;  // Pull-Up A
static constexpr uint8_t REG_GPIOA    = 0x12;  // GPIO A read

// Pin masks for each gear position on Port A
static constexpr uint8_t PIN_PARK     = (1 << 0);  // GPA0
static constexpr uint8_t PIN_REVERSE  = (1 << 1);  // GPA1
static constexpr uint8_t PIN_NEUTRAL  = (1 << 2);  // GPA2
static constexpr uint8_t PIN_FORWARD  = (1 << 3);  // GPA3
static constexpr uint8_t PIN_FWD_D2   = (1 << 4);  // GPA4
static constexpr uint8_t GEAR_MASK    = 0x1F;       // Bits 0-4

// I2C error handling constants
static constexpr uint8_t  ERROR_THRESHOLD  = 5;     // Consecutive errors before backoff
                                                     // (5 × 50ms pollMs = 250ms to trigger)
static constexpr uint32_t BACKOFF_POLL_MS  = 1000;  // Poll interval during backoff (ms)

// Module state
static Config       cfg_;
static Gear         currentGear_   = Gear::NEUTRAL;
static unsigned long lastPollMs_   = 0;
static bool         initialized_   = false;
static uint8_t      errorCount_    = 0;               // Consecutive I2C error counter
static bool         connected_     = false;            // MCP23017 responding on I2C

// -------------------------------------------------------------------------
// Write a single register to MCP23017
// -------------------------------------------------------------------------
static bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(cfg_.i2cAddr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

// -------------------------------------------------------------------------
// Read a single register from MCP23017
// Returns 0xFF on error.  Sets connected_ to false on I2C failure.
// -------------------------------------------------------------------------
static uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(cfg_.i2cAddr);
    Wire.write(reg);
    uint8_t err = Wire.endTransmission(false);
    if (err != 0) {
        return 0xFF;  // Error — skip requestFrom to avoid log spam
    }
    uint8_t n = Wire.requestFrom(cfg_.i2cAddr, (uint8_t)1);
    if (n == 0 || !Wire.available()) {
        return 0xFF;  // Error — no data received
    }
    return Wire.read();
}

// -------------------------------------------------------------------------
// Decode gear from port value (active-low, one-hot)
// -------------------------------------------------------------------------
static Gear decodeGear(uint8_t portVal) {
    // Invert (active-low) and mask relevant bits
    uint8_t active = (~portVal) & GEAR_MASK;

    // Require exactly one bit set (valid one-hot encoding)
    // __builtin_popcount works for single byte via implicit int promotion
    if (__builtin_popcount(active) != 1) {
        return Gear::NEUTRAL;  // Invalid or ambiguous → default Neutral
    }

    if (active & PIN_PARK)    return Gear::PARK;
    if (active & PIN_REVERSE) return Gear::REVERSE;
    if (active & PIN_NEUTRAL) return Gear::NEUTRAL;
    if (active & PIN_FORWARD) return Gear::FORWARD;
    if (active & PIN_FWD_D2)  return Gear::FORWARD_D2;

    return Gear::NEUTRAL;
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init(const Config& cfg) {
    cfg_ = cfg;

    Wire.begin(cfg_.sdaPin, cfg_.sclPin);
    Wire.setClock(400000);  // 400 kHz I2C (Fast Mode)

    // Probe the MCP23017: try to configure Port A
    bool ok = writeReg(REG_IODIRA, GEAR_MASK);
    if (ok) {
        ok = writeReg(REG_GPPUA, GEAR_MASK);
    }

    connected_   = ok;
    errorCount_  = ok ? 0 : ERROR_THRESHOLD;
    initialized_ = true;
    lastPollMs_  = 0;
    currentGear_ = Gear::NEUTRAL;

    if (ok) {
        Serial.println("[SHIFTER] MCP23017 initialized");
    } else {
        Serial.println("[SHIFTER] MCP23017 NOT detected — backoff active");
    }
}

void update() {
    if (!initialized_) return;

    unsigned long now = millis();

    // Use longer poll interval when the device is in error backoff
    uint32_t interval = (errorCount_ >= ERROR_THRESHOLD) ? BACKOFF_POLL_MS : cfg_.pollMs;
    if (now - lastPollMs_ < interval) return;
    lastPollMs_ = now;

    uint8_t portVal = readReg(REG_GPIOA);

    if (portVal == 0xFF) {
        // I2C read failed
        if (errorCount_ < ERROR_THRESHOLD) {
            errorCount_++;
            if (errorCount_ == ERROR_THRESHOLD) {
                // Transition to disconnected — log once
                connected_ = false;
                Serial.println("[SHIFTER] MCP23017 I2C error — backoff active");
            }
        }
        // Set currentGear_ to NEUTRAL during error
        currentGear_ = Gear::NEUTRAL;
        return;
    }

    // Successful read — recover from error state if needed
    if (errorCount_ > 0) {
        if (!connected_) {
            // Re-initialize registers after reconnection
            writeReg(REG_IODIRA, GEAR_MASK);
            writeReg(REG_GPPUA, GEAR_MASK);
            Serial.println("[SHIFTER] MCP23017 reconnected");
        }
        errorCount_ = 0;
        connected_  = true;
    }

    currentGear_ = decodeGear(portVal);
}

Gear getGear() {
    if (!initialized_) return Gear::NEUTRAL;
    return currentGear_;
}

uint8_t getGearRaw() {
    if (!initialized_) return static_cast<uint8_t>(Gear::NEUTRAL);
    return static_cast<uint8_t>(currentGear_);
}

bool isConnected() {
    return initialized_ && connected_;
}

} // namespace shifter
