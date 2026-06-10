// =============================================================================
// ESP32-S3 HMI — Audio Relay Module
//
// Controls a 2-channel 5V optocoupled relay that switches the speaker output
// between two sources:
//   - Normal state  (relay OFF) → Car radio output
//   - Active state  (relay ON)  → DFPlayer Mini amplifier output
//
// The relay command path is active-HIGH at the ESP32 GPIO when using ULN2803A:
//   GPIO HIGH -> ULN sink ON -> Songle IN3 pulled LOW -> relay ON
//   GPIO LOW  -> ULN sink OFF -> Songle IN3 pull-up to 5V -> relay OFF
//
// Pin selection: GPIO 11
//   - Not a strapping pin (only GPIO 0/3/45/46 are strapping on ESP32-S3)
//   - No conflict with Flash (GPIO 26–32) or PSRAM (GPIO 33–37) on N16R8
//   - Not a USB pin (GPIO 19/20 are USB D–/D+)
//   - Not used by any other firmware module
//   - Available on the ESP32-S3-DevKitC-1 right-side header (G11)
//   - GPIO level during boot is INPUT (high-impedance); the optocoupler's
//     built-in current-limiting resistor keeps the relay OFF until
//     relay_audio::requestOn() drives it HIGH during playback
//
// State machine (non-blocking, millis-based):
//   IDLE       → Relay OFF (GPIO LOW), no audio pending
//   ACTIVATING → Relay ON  (GPIO HIGH), waiting RELAY_ESTABLISH_MS for
//                the relay contact to fully close (~20 ms)
//   ACTIVE     → Relay ON  (GPIO HIGH), DFPlayer output connected to speaker
//   RELEASING  → Relay ON  (GPIO HIGH), audio ended, RELAY_RELEASE_MS
//                cooldown before deactivation to prevent speaker click
//
// Integration:
//   audio::update()  calls relay_audio::update()  every loop iteration.
//   audio::init()    calls relay_audio::init()     once at startup.
//   The audio subsystem is the master; the relay module is the slave.
//
// Reference: docs/AUDIO_RELAY_INTEGRATION.md
// =============================================================================

#ifndef RELAY_AUDIO_H
#define RELAY_AUDIO_H

#include <cstdint>

namespace relay_audio {

// ---- Hardware assignment ------------------------------------------------
/// GPIO pin for audio relay control (active HIGH at ESP32 side with ULN2803A).
/// GPIO 11: safe for ESP32-S3 — see header comment for full justification.
inline constexpr int PIN_AUDIO_RELAY = 11;

// ---- Timing constants ---------------------------------------------------
/// Time (ms) for relay contact to fully close after coil energisation.
/// Keeps DFPlayer audio muted until the speaker path is established.
inline constexpr unsigned long RELAY_ESTABLISH_MS = 20;

/// Cooldown (ms) after audio ends before the relay is de-energised.
/// Prevents an audible "click" from the relay switching mid-audio tail.
inline constexpr unsigned long RELAY_RELEASE_MS = 150;

/// Hard safety watchdog (ms): maximum time the relay coil may stay
/// energised in a single activation cycle before update() forces it OFF.
///
/// Calculation: RELAY_ESTABLISH_MS (20) + MAX_PLAY_DURATION_MS (5000) +
///              RELAY_RELEASE_MS (150) + 1830 ms safety margin = 7000 ms.
///
/// This is a defence-in-depth guard: under normal operation the relay
/// is released by the audio::update() playing timeout well before this
/// limit is reached.  It catches corner cases where the audio timeout
/// fails to call release() (e.g. logic error elsewhere in the system).
/// Note: when consecutive sounds are played back-to-back the relay stays
/// energised across them; the watchdog is reset on each new sound start
/// (requestOn() resets activationMs_).
inline constexpr unsigned long RELAY_MAX_ON_MS = 7000;

// ---- Public API ---------------------------------------------------------

/// Initialise relay GPIO pin and drive it LOW (relay OFF).
/// Call once from audio::init() before the main loop begins.
void init();

/// Request relay activation (audio playback is about to start).
///  - IDLE      → begins ACTIVATING sequence (GPIO HIGH, starts timer)
///  - RELEASING → cancels cooldown and returns to ACTIVE immediately
///    (relay contact is already closed — no re-establishment needed)
///  - ACTIVATING / ACTIVE → no-op (idempotent)
void requestOn();

/// Signal that the current audio playback has ended.
/// Transitions ACTIVE or ACTIVATING → RELEASING.
/// The relay is kept ON for RELAY_RELEASE_MS before final de-energisation.
void release();

/// Force relay OFF immediately, bypassing the normal RELEASING cooldown.
/// Drives GPIO LOW and resets the state machine to IDLE.
///
/// Use from emergency handlers or explicit shutdown sequences where
/// latency is unacceptable.  A small audible click may result.
void forceOff();

/// Tick the relay state machine.
/// Must be called on every audio::update() iteration.
/// Non-blocking: all timing via millis(), no delay() calls.
void update();

/// Returns true only when the relay is fully established (state == ACTIVE).
/// audio::update() waits for this before sending the DFPlayer play command.
bool isReady();

} // namespace relay_audio

#endif // RELAY_AUDIO_H
