// =============================================================================
// ESP32-S3 HMI — Power Manager Implementation
//
// Ignition key detection with debouncing and power sequencing.
// The external delay relay hardware keeps 12V alive after key-off,
// giving the ESP32 time to play the farewell audio and shut down
// gracefully before power is finally cut.
//
// Sequence:
//   Key ON  → POWER_HOLD → STARTING (welcome audio) → RUNNING
//   Key OFF → SHUTTING_DOWN (farewell audio, 3s) → OFF (release hold)
// =============================================================================

#include "power_manager.h"
#include <Arduino.h>

namespace power_mgr {

static PowerState state = PowerState::OFF;

// Key debounce state
static bool     keyRawLast       = false;
static bool     keyDebounced     = false;
static unsigned long keyChangeMs = 0;

// State transition timestamp
static unsigned long stateEntryMs = 0;

// -------------------------------------------------------------------------
// Debounced key reading
// -------------------------------------------------------------------------
static bool readKeyDebounced() {
    bool raw = (digitalRead(PIN_IGNITION_SENSE) == HIGH);
    unsigned long now = millis();

    if (raw != keyRawLast) {
        keyChangeMs = now;
        keyRawLast  = raw;
    }

    if ((now - keyChangeMs) >= DEBOUNCE_MS) {
        keyDebounced = keyRawLast;
    }

    return keyDebounced;
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init() {
    pinMode(PIN_IGNITION_SENSE, INPUT_PULLDOWN);
    pinMode(PIN_POWER_HOLD, OUTPUT);
    digitalWrite(PIN_POWER_HOLD, LOW);

    state        = PowerState::OFF;
    stateEntryMs = millis();
    keyRawLast   = false;
    keyDebounced = false;
    keyChangeMs  = millis();

    // Check if key is already on at boot (power-on with key turned)
    if (digitalRead(PIN_IGNITION_SENSE) == HIGH) {
        keyDebounced = true;
        keyRawLast   = true;
    }

    Serial.println("[PWR] Power manager initialized");
}

void update() {
    bool keyOn = readKeyDebounced();
    unsigned long now = millis();
    unsigned long elapsed = now - stateEntryMs;

    switch (state) {
        case PowerState::OFF:
            // Transition: key turned ON
            if (keyOn) {
                digitalWrite(PIN_POWER_HOLD, HIGH);
                state        = PowerState::POWER_HOLD;
                stateEntryMs = now;
                Serial.println("[PWR] Key ON → POWER_HOLD");
            }
            break;

        case PowerState::POWER_HOLD:
            // Immediate transition to STARTING
            state        = PowerState::STARTING;
            stateEntryMs = now;
            Serial.println("[PWR] POWER_HOLD → STARTING");
            break;

        case PowerState::STARTING:
            // Wait for startup delay (audio welcome plays during this time)
            if (elapsed >= STARTUP_DELAY_MS) {
                state        = PowerState::RUNNING;
                stateEntryMs = now;
                Serial.println("[PWR] STARTING → RUNNING");
            }
            // If key turned off during startup, go to shutdown
            if (!keyOn) {
                state        = PowerState::SHUTTING_DOWN;
                stateEntryMs = now;
                Serial.println("[PWR] STARTING → SHUTTING_DOWN (key off)");
            }
            break;

        case PowerState::RUNNING:
            // Normal operation — check for key off
            if (!keyOn) {
                state        = PowerState::SHUTTING_DOWN;
                stateEntryMs = now;
                Serial.println("[PWR] RUNNING → SHUTTING_DOWN (key off)");
            }
            break;

        case PowerState::SHUTTING_DOWN:
            // Farewell audio plays during this window.
            // After SHUTDOWN_DELAY_MS, release power hold.
            // The delay relay keeps hardware power for additional time.
            if (elapsed >= SHUTDOWN_DELAY_MS) {
                digitalWrite(PIN_POWER_HOLD, LOW);
                state        = PowerState::OFF;
                stateEntryMs = now;
                Serial.println("[PWR] SHUTTING_DOWN → OFF (power released)");
            }
            // If key turned back on during shutdown, return to running
            if (keyOn) {
                state        = PowerState::RUNNING;
                stateEntryMs = now;
                Serial.println("[PWR] SHUTTING_DOWN → RUNNING (key back on)");
            }
            break;
    }
}

PowerState getState() {
    return state;
}

bool isShutdownComplete() {
    return (state == PowerState::OFF);
}

bool isRunning() {
    return (state == PowerState::RUNNING);
}

bool isKeyOn() {
    return keyDebounced;
}

} // namespace power_mgr
