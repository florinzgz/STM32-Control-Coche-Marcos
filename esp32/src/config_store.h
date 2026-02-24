// =============================================================================
// ESP32-S3 HMI — Config Store (NVS Persistence)
//
// Wrapper around ESP32 NVS (Non-Volatile Storage) for persisting HMI
// configuration across reboots.  Validates data with CRC32.
//
// Stored items:
//   - Preferred drive mode (4x4/4x2/tank)
//   - Display brightness
//   - Front and rear LED relay states
//   - Audio volume
//   - INA226 channel-to-position mapping (6 entries, 0-5)
//   - DS18B20 sensor-to-position mapping (5 entries, 0-4)
//
// No external library needed — uses ESP32 native Preferences API.
//
// Reference: docs/ESP32_FIRMWARE_DESIGN.md
// =============================================================================

#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <cstdint>

namespace config_store {

// Number of INA226 channels (must match NUM_INA226 in STM32 main.h)
inline constexpr uint8_t NUM_INA226_CH   = 6;
// Number of DS18B20 sensors (must match NUM_DS18B20 in STM32 main.h)
inline constexpr uint8_t NUM_TEMP_SENS   = 5;

// -------------------------------------------------------------------------
// Sensor position label indices (used in mapping arrays)
//   INA226: 0=FL_Motor 1=FR_Motor 2=RL_Motor 3=RR_Motor 4=Battery 5=Steering
//   DS18B20: 0=FL_Wheel 1=FR_Wheel 2=RL_Wheel 3=RR_Wheel 4=Ambient
// -------------------------------------------------------------------------
inline constexpr uint8_t SENSOR_POS_UNSET = 0xFF;  // not yet assigned

// -------------------------------------------------------------------------
// Persisted configuration structure
// -------------------------------------------------------------------------
struct Config {
    uint8_t  driveMode          = 0;      // Mode flags (bit 0=4x4, bit 1=tank)
    uint8_t  brightness         = 100;    // Display brightness (0-255)
    bool     frontLedEnabled    = false;  // Front LED relay default state
    bool     rearLedEnabled     = false;  // Rear LED relay default state
    uint8_t  audioVolume        = 15;     // DFPlayer volume (0-30)
    // INA226 mapping: ina226Map[physCh] = positionIndex (0-5, 0xFF=unset)
    uint8_t  ina226Map[NUM_INA226_CH]  = {0, 1, 2, 3, 4, 5};
    // DS18B20 mapping: tempMap[physIdx] = positionIndex (0-4, 0xFF=unset)
    uint8_t  tempSensorMap[NUM_TEMP_SENS] = {0, 1, 2, 3, 4};
    uint32_t crc32              = 0;      // CRC32 validation
};

/// Initialize NVS and load saved config.  Call once from setup().
void init();

/// Load config from NVS.  Returns true if valid config was found.
bool load(Config& cfg);

/// Save config to NVS.  Returns true on success.
bool save(const Config& cfg);

/// Get the currently loaded config (read-only).
const Config& get();

/// Update a specific field (marks config dirty; call flush() to persist).
void setDriveMode(uint8_t mode);
void setBrightness(uint8_t brightness);
void setFrontLedEnabled(bool enabled);
void setRearLedEnabled(bool enabled);
void setAudioVolume(uint8_t volume);
void setIna226Map(const uint8_t map[NUM_INA226_CH]);
void setTempSensorMap(const uint8_t map[NUM_TEMP_SENS]);

/// Flush pending changes to NVS if dirty.  Call periodically from loop().
void flush();

/// Returns true if there are unsaved changes.
bool isDirty();

/// Reset config to factory defaults and save.
void factoryReset();

} // namespace config_store

#endif // CONFIG_STORE_H
