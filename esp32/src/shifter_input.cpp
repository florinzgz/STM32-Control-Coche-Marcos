// =============================================================================
// ESP32-S3 — Shifter Input Driver (MCP23017 I2C) — Implementation
//
// Reads gear selector position from MCP23017 Port A pins (active-low).
//
// The lever has DRY CONTACTS (no internal voltage) and 5 wires:
//   1 common wire (blue) — connect to GND
//   4 signal wires       — connect to MCP23017 GPA pins (pull-ups provide bias)
//
// Physical lever order (top → bottom) and confirmed wire colours:
//   GPA0 — P  (Park)    — blue + purple
//   GPA1 — D2 (Drive 2) — blue + green
//   GPA2 — D1 (Drive 1) — blue + yellow
//   GPA3 — R  (Reverse) — blue + white
//
// NEUTRAL has NO dedicated contact wire.  It is detected implicitly:
// when none of the four contacts are closed all four GPA pins stay HIGH
// (pulled up) and decodeGear() returns NEUTRAL.
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

// Pin masks for each gear position on Port A — confirmed wire colours
// Common wire (blue) → GND.  Signal wires → GPA0-GPA3 with pull-ups.
static constexpr uint8_t PIN_PARK     = (1 << 0);  // GPA0 — blue+purple  (P)
static constexpr uint8_t PIN_FWD_D2   = (1 << 1);  // GPA1 — blue+green   (D2)
static constexpr uint8_t PIN_FORWARD  = (1 << 2);  // GPA2 — blue+yellow  (D1)
static constexpr uint8_t PIN_REVERSE  = (1 << 3);  // GPA3 — blue+white   (R)
// Neutral has no contact wire — detected when no pin is active (see decodeGear)
static constexpr uint8_t GEAR_MASK    = 0x0F;       // Bits 0-3 only

// I2C error handling constants
static constexpr uint8_t  ERROR_THRESHOLD  = 5;     // Consecutive errors before backoff
                                                     // (5 × 50ms pollMs = 250ms to trigger)
static constexpr uint32_t BACKOFF_POLL_MS  = 1000;  // Poll interval during backoff (ms)

// Module state
static Config       cfg_;
static Gear         currentGear_   = Gear::NEUTRAL;
static Gear         pendingGear_   = Gear::NEUTRAL;        // F5: candidate awaiting confirmation
static uint8_t      pendingCount_  = 0;                     // F5: consecutive identical samples
static constexpr uint8_t DEBOUNCE_SAMPLES = 2;              // F5: required identical samples
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
//
// Returns NEUTRAL when no pin is active — this is the normal resting state
// for the N position (no physical contact wire).
// -------------------------------------------------------------------------
static Gear decodeGear(uint8_t portVal) {
    // Invert (active-low) and mask relevant bits
    uint8_t active = (~portVal) & GEAR_MASK;

    // Zero active bits = Neutral (lever in N, no contact closed)
    // More than one active bit = invalid/transition → default to Neutral
    if (__builtin_popcount(active) != 1) {
        return Gear::NEUTRAL;
    }

    if (active & PIN_PARK)    return Gear::PARK;
    if (active & PIN_FWD_D2)  return Gear::FORWARD_D2;
    if (active & PIN_FORWARD) return Gear::FORWARD;
    if (active & PIN_REVERSE) return Gear::REVERSE;

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
    pendingGear_  = Gear::NEUTRAL;   // F5: reset debounce state
    pendingCount_ = 0;

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

    // When in backoff, use address-only probe (endTransmission with STOP)
    // instead of readReg() to avoid ESP32 Wire.requestFrom() error spam
    // when the MCP23017 is not connected.
    if (errorCount_ >= ERROR_THRESHOLD) {
        Wire.beginTransmission(cfg_.i2cAddr);
        if (Wire.endTransmission() != 0) {
            currentGear_ = Gear::NEUTRAL;
            pendingGear_  = Gear::NEUTRAL;   // F5: reset debounce on fail-safe
            pendingCount_ = 0;
            return;  // Device still absent — stay in backoff
        }
        // Device responded — re-initialize and resume
        if (!writeReg(REG_IODIRA, GEAR_MASK) || !writeReg(REG_GPPUA, GEAR_MASK)) {
            currentGear_ = Gear::NEUTRAL;
            pendingGear_  = Gear::NEUTRAL;   // F5: reset debounce on fail-safe
            pendingCount_ = 0;
            return;  // Re-init failed — stay in backoff
        }
        errorCount_ = 0;
        connected_  = true;
        Serial.println("[SHIFTER] MCP23017 reconnected");
        uint8_t portVal = readReg(REG_GPIOA);
        // F5: after reconnect, force the NEUTRAL fail-safe and restart the
        // debounce window — never publish a non-NEUTRAL gear on a single
        // post-recovery sample.
        currentGear_ = Gear::NEUTRAL;
        pendingGear_  = (portVal != 0xFF) ? decodeGear(portVal) : Gear::NEUTRAL;
        pendingCount_ = 1;
        return;
    }

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
        // Set currentGear_ to NEUTRAL during error and clear debounce state
        currentGear_  = Gear::NEUTRAL;
        pendingGear_  = Gear::NEUTRAL;
        pendingCount_ = 0;
        return;
    }

    // Successful read — recover from minor error streak
    if (errorCount_ > 0) {
        errorCount_ = 0;
        connected_  = true;
    }

    // F5 — true debounce:
    //   Only commit a decoded gear to currentGear_ after observing
    //   DEBOUNCE_SAMPLES (=2) consecutive identical samples.  Any
    //   sample that differs from the pending candidate restarts the
    //   counter.  This filters single-poll glitches without any heap
    //   allocation, RAM cost = 2 bytes (pendingGear_ + pendingCount_),
    //   and preserves the 50 ms polling cadence and the one-hot
    //   NEUTRAL fail-safe in decodeGear().
    Gear sample = decodeGear(portVal);
    if (sample == pendingGear_) {
        if (pendingCount_ < DEBOUNCE_SAMPLES) {
            pendingCount_++;
        }
    } else {
        pendingGear_  = sample;
        pendingCount_ = 1;
    }
    if (pendingCount_ >= DEBOUNCE_SAMPLES) {
        currentGear_ = pendingGear_;
    }
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
