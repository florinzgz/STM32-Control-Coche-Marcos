// =============================================================================
// ESP32-S3 HMI — Audio Relay Module (Implementation)
//
// 2-channel 5V optocoupled relay driver for speaker source switching.
// Implements the non-blocking state machine described in relay_audio.h.
//
// State transitions:
//
//   requestOn()        establishment timer         release()
//   IDLE ──────────► ACTIVATING ──────────────► ACTIVE ──────────► RELEASING
//                                                   ▲                  │
//                       requestOn() (cancel)        └──────────────────┘
//                       RELEASING ─────────────────────────────────────►
//                                                         cooldown timer
//                                                   RELEASING ─────────► IDLE
//
// See relay_audio.h for full design rationale and GPIO selection.
// =============================================================================

#include "relay_audio.h"
#include <Arduino.h>

namespace relay_audio {

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------

enum class State : uint8_t {
    IDLE,        // Relay OFF — GPIO HIGH
    ACTIVATING,  // Relay ON  — GPIO LOW, waiting for contact establishment
    ACTIVE,      // Relay ON  — GPIO LOW, ready for DFPlayer audio
    RELEASING    // Relay ON  — GPIO LOW, cooldown after audio ends
};

static State         state_    = State::IDLE;
static unsigned long stateMs_  = 0;

// ---------------------------------------------------------------------------
// Public API implementation
// ---------------------------------------------------------------------------

void init() {
    pinMode(PIN_AUDIO_RELAY, OUTPUT);
    digitalWrite(PIN_AUDIO_RELAY, HIGH);  // Relay OFF (active LOW)
    state_   = State::IDLE;
    stateMs_ = 0;
    Serial.println("[RELAY] Audio relay initialized (GPIO 11, active-LOW, IDLE)");
}

void requestOn() {
    switch (state_) {
        case State::IDLE:
            // Relay was off — turn it on and start the establishment timer
            digitalWrite(PIN_AUDIO_RELAY, LOW);
            state_   = State::ACTIVATING;
            stateMs_ = millis();
            break;

        case State::RELEASING:
            // New audio request arrived during the release cooldown.
            // The relay contact is still closed, so no re-establishment is
            // needed — cancel the cooldown and go straight back to ACTIVE.
            state_   = State::ACTIVE;
            stateMs_ = millis();
            break;

        case State::ACTIVATING:
        case State::ACTIVE:
            // Already heading to (or at) ACTIVE — nothing to do
            break;
    }
}

void release() {
    if (state_ == State::ACTIVE || state_ == State::ACTIVATING) {
        state_   = State::RELEASING;
        stateMs_ = millis();
    }
    // IDLE and RELEASING are left unchanged
}

bool isReady() {
    return state_ == State::ACTIVE;
}

void update() {
    unsigned long now = millis();

    switch (state_) {
        case State::IDLE:
            break;  // Nothing to do

        case State::ACTIVATING:
            // Wait for relay contact to fully establish
            if ((now - stateMs_) >= RELAY_ESTABLISH_MS) {
                state_ = State::ACTIVE;
            }
            break;

        case State::ACTIVE:
            break;  // Relay ON, ready — audio::update() drives playback

        case State::RELEASING:
            // Keep relay ON during cooldown to absorb any audio tail
            if ((now - stateMs_) >= RELAY_RELEASE_MS) {
                digitalWrite(PIN_AUDIO_RELAY, HIGH);  // Relay OFF
                state_ = State::IDLE;
            }
            break;
    }
}

} // namespace relay_audio
