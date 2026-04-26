// =============================================================================
// ESP32-S3 HMI — Power Manager
//
// Manages ignition key detection and power sequencing.
// The user has an external delay relay that keeps 12V alive for a
// configurable window after the ignition key is turned off, allowing
// the audio system to play a farewell sound before final power-down.
//
// GPIO 40: Ignition key sense via PC817 optocoupler — INVERTED logic
//          LOW  = key ON  (LED conducts → transistor saturated)
//          HIGH = key OFF (LED off, line pulled up by PC817 board, ~10 kΩ)
// GPIO 41: Power hold output  (HIGH = request power stay on)
//
// External wiring (PC817 8-channel isolation board, 1 free channel):
//   +12 V (ignition key) ── 1 kΩ ¼ W ──→ IN+ (LED anode) of PC817 channel
//   GND  (vehicle)        ─────────────→ IN- (LED cathode)
//   OUT  (collector)      ─────────────→ GPIO 40 (already pulled up to 3.3 V
//                                          on the module by an onboard ~10 kΩ)
//   GND  (3.3 V side)     ──── shared with ESP32 / STM32 GND
// The 1 kΩ resistor (NOT the 330 Ω used for the LJ12A3 wheel sensors) is
// mandatory for continuous-duty operation: at 12 V it limits LED current to
// ~10.8 mA, well within the PC817 nominal 20 mA, preserving CTR over time.
//
// State machine:
//   OFF → POWER_HOLD → STARTING → RUNNING → SHUTTING_DOWN → OFF
//
// Reference: FIRMWARE_MIGRATION_AUDIT.md Step 2
// =============================================================================

#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <cstdint>

namespace power_mgr {

/// GPIO assignments for ignition key detection
inline constexpr int PIN_IGNITION_SENSE = 40;   // Input: key position
inline constexpr int PIN_POWER_HOLD     = 41;   // Output: keep power alive

/// Timing constants
inline constexpr unsigned long DEBOUNCE_MS        = 50;    // Key debounce
inline constexpr unsigned long STARTUP_DELAY_MS   = 200;   // Delay before audio welcome
inline constexpr unsigned long SHUTDOWN_DELAY_MS  = 3000;  // Time for farewell audio

/// Power states
enum class PowerState : uint8_t {
    OFF             = 0,   // System powered down
    POWER_HOLD      = 1,   // Key detected, holding power
    STARTING        = 2,   // Playing welcome, initializing
    RUNNING         = 3,   // Normal operation
    SHUTTING_DOWN   = 4    // Key off detected, farewell playing
};

/// Initialize GPIO pins and state machine
void init();

/// Update state machine — call from main loop
void update();

/// Get current power state
PowerState getState();

/// Returns true if shutdown sequence is complete (audio finished, safe to lose power)
bool isShutdownComplete();

/// Returns true if the system is fully running and ready for normal operation
bool isRunning();

/// Returns true if key is currently in ON position (debounced)
bool isKeyOn();

} // namespace power_mgr

#endif // POWER_MANAGER_H
