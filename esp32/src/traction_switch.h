// =============================================================================
// ESP32-S3 — Traction Switch Driver (DPDT Rocker, used as SPDT)
//
// Reads the physical 2WD/4WD rocker switch position via GPIO.
// The switch has 2 wires: common (→ GPIO) and one output (→ GND).
// When the switch connects GPIO to GND → 4WD (LOW).
// When the switch disconnects GPIO from GND → 2WD (HIGH via pull-up).
//
// Features:
//   - Internal pull-up on GPIO for fail-safe (floating = 2WD)
//   - Software debounce (50 ms, requires consecutive stable readings)
//   - Speed gate: rejects traction changes when vehicle speed > 0.5 km/h
//   - Floating detection: retains last valid state on cable disconnect
//   - Reports mode change flag for CAN transmission
//
// Output: 4WD flag matching CMD_MODE byte 0 bit 0 (can::MODE_FLAG_4X4)
//
// Reference: docs/CAN_CONTRACT_FINAL.md §4.5 — CMD_MODE byte 0
// =============================================================================

#ifndef TRACTION_SWITCH_H
#define TRACTION_SWITCH_H

#include <cstdint>

namespace traction_sw {

// -------------------------------------------------------------------------
// Configuration
// -------------------------------------------------------------------------
struct Config {
    int      gpioPin         = 15;    // GPIO for switch input (pull-up)
    uint32_t debounceMs      = 50;    // Debounce time (ms)
    uint8_t  stableCount     = 3;     // Consecutive stable readings required
    uint32_t pollMs          = 10;    // Polling interval (ms)
    float    speedThreshKmh  = 0.5f;  // Max speed for traction change (km/h)
};

// -------------------------------------------------------------------------
// Traction mode
// -------------------------------------------------------------------------
enum class Mode : uint8_t {
    TWO_WD = 0,   // 2WD (front-wheel drive)
    FOUR_WD = 1   // 4WD (all-wheel drive)
};

/// Initialize GPIO and internal state.  Call once from setup().
void init(const Config& cfg = Config{});

/// Poll switch state and apply debounce + speed gate.
/// Call from loop() every iteration.
/// @param vehicleSpeedKmh  Average vehicle speed for speed gate.
void update(float vehicleSpeedKmh);

/// Get the current confirmed traction mode.
Mode getMode();

/// Returns true if the traction mode changed since the last call
/// to clearChanged().  Used to trigger CAN transmission.
bool hasChanged();

/// Clear the changed flag after CAN transmission.
void clearChanged();

/// Returns true if 4WD is active.
bool is4WD();

/// Get the mode as CAN mode flag value (0x00 or 0x01).
uint8_t getModeFlag();

} // namespace traction_sw

#endif // TRACTION_SWITCH_H
