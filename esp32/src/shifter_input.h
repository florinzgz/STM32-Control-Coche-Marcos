// =============================================================================
// ESP32-S3 — Shifter Input Driver (MCP23017 I2C)
//
// Reads the physical gear selector position (P/R/N/D1/D2) via MCP23017
// I2C I/O expander.  Each position is connected to a separate GPIO pin
// on Port A of the MCP23017 (active-low with internal pull-ups).
//
// Output: gear position enum matching STM32 GearPosition_t encoding.
//
// Reference: docs/CAN_CONTRACT_FINAL.md §4.5 — CMD_MODE byte 1 (gear)
// =============================================================================

#ifndef SHIFTER_INPUT_H
#define SHIFTER_INPUT_H

#include <cstdint>

namespace shifter {

// -------------------------------------------------------------------------
// Gear positions — matches STM32 GearPosition_t in motor_control.h
// -------------------------------------------------------------------------
enum class Gear : uint8_t {
    PARK       = 0,
    REVERSE    = 1,
    NEUTRAL    = 2,
    FORWARD    = 3,
    FORWARD_D2 = 4
};

// -------------------------------------------------------------------------
// Configuration
// -------------------------------------------------------------------------
struct Config {
    int      sdaPin       = 8;     // I2C SDA GPIO (available on ESP32-S3)
    int      sclPin       = 9;     // I2C SCL GPIO (available on ESP32-S3)
    uint8_t  i2cAddr      = 0x20;  // MCP23017 default I2C address
    uint32_t pollMs       = 50;    // Polling interval (ms)
};

/// Initialize MCP23017 hardware.  Call once from setup().
void init(const Config& cfg = Config{});

/// Poll gear selector and update internal state.  Call from loop().
void update();

/// Get the currently selected gear.
Gear getGear();

/// Get the raw gear value as uint8_t for CAN transmission.
uint8_t getGearRaw();

} // namespace shifter

#endif // SHIFTER_INPUT_H
