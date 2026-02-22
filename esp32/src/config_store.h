// =============================================================================
// ESP32-S3 HMI — Config Store (NVS Persistence)
//
// Wrapper around ESP32 NVS (Non-Volatile Storage) for persisting HMI
// configuration across reboots.  Validates data with CRC32.
//
// Stored items:
//   - Preferred drive mode (4x4/4x2/tank)
//   - Display brightness
//   - LED state
//   - Audio volume
//
// No external library needed — uses ESP32 native Preferences API.
//
// Reference: docs/ESP32_FIRMWARE_DESIGN.md
// =============================================================================

#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <cstdint>

namespace config_store {

// -------------------------------------------------------------------------
// Persisted configuration structure
// -------------------------------------------------------------------------
struct Config {
    uint8_t  driveMode     = 0;      // Mode flags (bit 0=4x4, bit 1=tank)
    uint8_t  brightness    = 100;    // Display brightness (0-255)
    bool     ledEnabled    = false;  // LED relay default state
    uint8_t  audioVolume   = 15;     // DFPlayer volume (0-30)
    uint32_t crc32         = 0;      // CRC32 validation
};

/// Initialize NVS and load saved config.  Call once from setup().
void init();

/// Load config from NVS.  Returns true if valid config was found.
bool load(Config& cfg);

/// Save config to NVS.  Returns true on success.
bool save(const Config& cfg);

/// Get the currently loaded config (read-only).
const Config& get();

/// Update and save a specific field.
void setDriveMode(uint8_t mode);
void setBrightness(uint8_t brightness);
void setLedEnabled(bool enabled);
void setAudioVolume(uint8_t volume);

/// Reset config to factory defaults and save.
void factoryReset();

} // namespace config_store

#endif // CONFIG_STORE_H
