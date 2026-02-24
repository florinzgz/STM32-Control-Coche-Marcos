// =============================================================================
// ESP32-S3 HMI — Config Store (NVS Persistence) — Implementation
//
// Uses ESP32 Preferences library (NVS wrapper) for persistent storage.
// CRC32 validation ensures data integrity across firmware updates.
// =============================================================================

#include "config_store.h"
#include <Preferences.h>
#include <Arduino.h>
#include <cstring>

namespace config_store {

// NVS namespace and keys
static constexpr const char* NVS_NAMESPACE   = "hmi_cfg";
static constexpr const char* KEY_MODE        = "mode";
static constexpr const char* KEY_BRIGHT      = "bright";
static constexpr const char* KEY_LED_FRONT   = "led_f";
static constexpr const char* KEY_LED_REAR    = "led_r";
static constexpr const char* KEY_VOLUME      = "vol";
static constexpr const char* KEY_INA_MAP     = "ina_map";  // 6-byte blob
static constexpr const char* KEY_TEMP_MAP    = "tmp_map";  // 5-byte blob
static constexpr const char* KEY_CRC         = "crc";

// Module state
static Preferences prefs_;
static Config      currentCfg_;
static bool        initialized_ = false;
static bool        dirty_       = false;     // Unsaved changes pending

// -------------------------------------------------------------------------
// CRC32 computation (standard CRC-32/ISO-HDLC)
// -------------------------------------------------------------------------
static uint32_t computeCrc32(const Config& cfg) {
    // Hash the data fields (not including the crc32 field itself)
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&cfg);
    size_t len = offsetof(Config, crc32);

    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFF;
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init() {
    initialized_ = true;

    Config loaded;
    if (load(loaded)) {
        currentCfg_ = loaded;
        Serial.println("[NVS] Config loaded successfully");
    } else {
        Serial.println("[NVS] No valid config — using defaults");
        currentCfg_ = Config{};
        save(currentCfg_);
    }
}

bool load(Config& cfg) {
    prefs_.begin(NVS_NAMESPACE, true);  // Read-only

    cfg.driveMode       = prefs_.getUChar(KEY_MODE, 0);
    cfg.brightness      = prefs_.getUChar(KEY_BRIGHT, 100);
    cfg.frontLedEnabled = prefs_.getBool(KEY_LED_FRONT, false);
    cfg.rearLedEnabled  = prefs_.getBool(KEY_LED_REAR, false);
    cfg.audioVolume     = prefs_.getUChar(KEY_VOLUME, 15);

    // Load INA226 mapping (6-byte blob)
    uint8_t inaDefault[NUM_INA226_CH] = {0, 1, 2, 3, 4, 5};
    size_t inaLen = prefs_.getBytes(KEY_INA_MAP, cfg.ina226Map, NUM_INA226_CH);
    if (inaLen != NUM_INA226_CH) {
        memcpy(cfg.ina226Map, inaDefault, NUM_INA226_CH);
    }

    // Load temp sensor mapping (5-byte blob)
    uint8_t tmpDefault[NUM_TEMP_SENS] = {0, 1, 2, 3, 4};
    size_t tmpLen = prefs_.getBytes(KEY_TEMP_MAP, cfg.tempSensorMap, NUM_TEMP_SENS);
    if (tmpLen != NUM_TEMP_SENS) {
        memcpy(cfg.tempSensorMap, tmpDefault, NUM_TEMP_SENS);
    }

    uint32_t storedCrc = prefs_.getULong(KEY_CRC, 0);

    prefs_.end();

    // Validate CRC
    cfg.crc32 = 0;
    uint32_t computed = computeCrc32(cfg);
    cfg.crc32 = storedCrc;

    if (computed != storedCrc) {
        Serial.printf("[NVS] CRC mismatch: stored=0x%08lX computed=0x%08lX\n",
                      (unsigned long)storedCrc, (unsigned long)computed);
        return false;
    }

    return true;
}

bool save(const Config& cfg) {
    Config toSave = cfg;
    toSave.crc32 = 0;
    toSave.crc32 = computeCrc32(toSave);

    prefs_.begin(NVS_NAMESPACE, false);  // Read-write

    prefs_.putUChar(KEY_MODE, toSave.driveMode);
    prefs_.putUChar(KEY_BRIGHT, toSave.brightness);
    prefs_.putBool(KEY_LED_FRONT, toSave.frontLedEnabled);
    prefs_.putBool(KEY_LED_REAR, toSave.rearLedEnabled);
    prefs_.putUChar(KEY_VOLUME, toSave.audioVolume);
    prefs_.putBytes(KEY_INA_MAP, toSave.ina226Map, NUM_INA226_CH);
    prefs_.putBytes(KEY_TEMP_MAP, toSave.tempSensorMap, NUM_TEMP_SENS);
    prefs_.putULong(KEY_CRC, toSave.crc32);

    prefs_.end();

    Serial.println("[NVS] Config saved");
    return true;
}

const Config& get() {
    return currentCfg_;
}

void setDriveMode(uint8_t mode) {
    currentCfg_.driveMode = mode & 0x03;  // Only bits 0-1 are valid
    dirty_ = true;
}

void setBrightness(uint8_t brightness) {
    currentCfg_.brightness = brightness;  // Full 0-255 range valid
    dirty_ = true;
}

void setFrontLedEnabled(bool enabled) {
    currentCfg_.frontLedEnabled = enabled;
    dirty_ = true;
}

void setRearLedEnabled(bool enabled) {
    currentCfg_.rearLedEnabled = enabled;
    dirty_ = true;
}

void setAudioVolume(uint8_t volume) {
    if (volume > 30) volume = 30;
    currentCfg_.audioVolume = volume;
    dirty_ = true;
}

void setIna226Map(const uint8_t map[NUM_INA226_CH]) {
    memcpy(currentCfg_.ina226Map, map, NUM_INA226_CH);
    dirty_ = true;
}

void setTempSensorMap(const uint8_t map[NUM_TEMP_SENS]) {
    memcpy(currentCfg_.tempSensorMap, map, NUM_TEMP_SENS);
    dirty_ = true;
}

void flush() {
    if (!dirty_) return;
    save(currentCfg_);
    dirty_ = false;
}

bool isDirty() {
    return dirty_;
}

void factoryReset() {
    prefs_.begin(NVS_NAMESPACE, false);
    prefs_.clear();
    prefs_.end();

    currentCfg_ = Config{};
    save(currentCfg_);
    Serial.println("[NVS] Factory reset complete");
}

} // namespace config_store
