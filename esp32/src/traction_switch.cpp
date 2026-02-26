// =============================================================================
// ESP32-S3 — Traction Switch Driver — Implementation
//
// DPDT rocker switch (ON-ON latching) used as SPDT for 2WD/4WD selection.
// Two wires: common → GPIO 15, output → GND.
//
// Wiring logic:
//   Switch position A (4WD): GPIO pulled LOW  (connected to GND)
//   Switch position B (2WD): GPIO pulled HIGH (internal pull-up, open)
//
// Safety design:
//   - Pull-up ensures a disconnected cable reads as 2WD (safer default)
//   - Debounce requires STABLE_COUNT consecutive identical readings
//   - Speed gate prevents traction change while vehicle is moving
//   - Last valid state is retained on electrical fault (floating)
//
// Reference: docs/CAN_CONTRACT_FINAL.md §4.5
// =============================================================================

#include "traction_switch.h"
#include <Arduino.h>

namespace traction_sw {

// Module state
static Config       cfg_;
static Mode         confirmedMode_    = Mode::TWO_WD;   // Last confirmed mode
static Mode         candidateMode_    = Mode::TWO_WD;   // Mode being debounced
static uint8_t      stableCounter_    = 0;               // Consecutive stable readings
static unsigned long lastPollMs_      = 0;
static unsigned long lastChangeMs_    = 0;               // Timestamp of last debounce start
static bool         changed_          = false;            // Flag: mode changed since last clear
static bool         initialized_      = false;
static bool         speedBlocked_     = false;            // True if change was blocked by speed

// -------------------------------------------------------------------------
// Read raw switch state and convert to Mode
// -------------------------------------------------------------------------
static Mode readRaw() {
    int level = digitalRead(cfg_.gpioPin);
    // LOW = connected to GND = 4WD, HIGH = pull-up (open) = 2WD
    return (level == LOW) ? Mode::FOUR_WD : Mode::TWO_WD;
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init(const Config& cfg) {
    cfg_ = cfg;

    // Configure GPIO with internal pull-up
    // Pull-up ensures disconnected cable → HIGH → 2WD (safe default)
    pinMode(cfg_.gpioPin, INPUT_PULLUP);

    // Read initial state immediately (no debounce on first read)
    confirmedMode_ = readRaw();
    candidateMode_ = confirmedMode_;
    stableCounter_ = 0;
    changed_       = true;   // Signal initial state for CAN publish on boot
                               // (STM32 needs to know traction mode at startup)
    lastPollMs_    = millis();
    lastChangeMs_  = 0;
    speedBlocked_  = false;
    initialized_   = true;

    Serial.printf("[TRACTION] Switch initialized on GPIO %d — mode: %s\n",
                  cfg_.gpioPin,
                  confirmedMode_ == Mode::FOUR_WD ? "4WD" : "2WD");
}

void update(float vehicleSpeedKmh) {
    if (!initialized_) return;

    unsigned long now = millis();
    if (now - lastPollMs_ < cfg_.pollMs) return;
    lastPollMs_ = now;

    Mode rawMode = readRaw();

    // ---- Debounce logic ----
    // If reading matches candidate, increment stable counter
    if (rawMode == candidateMode_) {
        if (stableCounter_ < cfg_.stableCount) {
            stableCounter_++;
        }
    } else {
        // New reading differs from candidate: start new debounce
        candidateMode_ = rawMode;
        stableCounter_ = 1;
        lastChangeMs_  = now;
    }

    // ---- Check if debounce is complete ----
    if (stableCounter_ >= cfg_.stableCount &&
        (now - lastChangeMs_) >= cfg_.debounceMs &&
        candidateMode_ != confirmedMode_) {

        // ---- Speed gate ----
        // Do not allow traction change while vehicle is moving > threshold
        if (vehicleSpeedKmh > cfg_.speedThreshKmh) {
            if (!speedBlocked_) {
                Serial.printf("[TRACTION] Change blocked: speed %.1f km/h > %.1f km/h\n",
                              vehicleSpeedKmh, cfg_.speedThreshKmh);
                speedBlocked_ = true;
            }
            // Keep current confirmed mode, do NOT update
            return;
        }

        // ---- Accept new mode ----
        confirmedMode_ = candidateMode_;
        changed_       = true;
        speedBlocked_  = false;

        Serial.printf("[TRACTION] Mode changed → %s\n",
                      confirmedMode_ == Mode::FOUR_WD ? "4WD" : "2WD");
    }

    // Clear speed-blocked flag when speed drops below threshold
    if (speedBlocked_ && vehicleSpeedKmh <= cfg_.speedThreshKmh) {
        speedBlocked_ = false;
    }
}

Mode getMode() {
    if (!initialized_) return Mode::TWO_WD;
    return confirmedMode_;
}

bool hasChanged() {
    return changed_;
}

void clearChanged() {
    changed_ = false;
}

bool is4WD() {
    return confirmedMode_ == Mode::FOUR_WD;
}

uint8_t getModeFlag() {
    return is4WD() ? 0x01 : 0x00;  // MODE_FLAG_4X4 = 0x01
}

} // namespace traction_sw
