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

/// Returns true when the MCP23017 is responding on I2C.
bool isConnected();

// -------------------------------------------------------------------------
// Live diagnostic snapshot (read-only, cached) — for the Engineering menu.
//
// IMPORTANT: getDiag() performs NO I2C of its own.  It returns a snapshot of
// values cached by update() (which runs on the main loop / Core 1).  This
// keeps the Engineering screen (rendered on Core 0) off the shared Wire bus,
// avoiding a cross-core I2C race with the shifter poller.
// -------------------------------------------------------------------------
struct Diag {
    uint8_t       i2cAddr      = 0x20;   // configured MCP23017 address
    bool          initialized  = false;  // init() has run
    bool          connected    = false;  // MCP23017 currently responding
    uint8_t       iodirA       = 0x00;   // configured Port A direction (0x0F expected)
    uint8_t       gppuA        = 0x00;   // configured Port A pull-ups  (0x0F expected)
    uint8_t       gpioRaw      = 0xFF;   // last raw GPIOA read (0xFF = error/no data)
    uint8_t       gearDecoded  = 2;      // current decoded gear (Gear as uint8_t)
    uint8_t       errorCount   = 0;      // consecutive I2C error counter
    uint16_t      recoveryCount = 0;     // number of I2C bus-recovery attempts
    unsigned long lastValidMs  = 0;      // millis() of last successful GPIOA read (0 = never)
};

/// Get a cached, read-only diagnostic snapshot of the MCP23017 shifter.
Diag getDiag();

} // namespace shifter

#endif // SHIFTER_INPUT_H
