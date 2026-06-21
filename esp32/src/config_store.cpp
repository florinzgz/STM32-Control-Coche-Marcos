// =============================================================================
// ESP32-S3 HMI — Config Store (NVS Persistence) — Implementation
//
// Uses ESP32 Preferences library (NVS wrapper) for persistent storage.
// CRC32 validation ensures data integrity across firmware updates.
// =============================================================================

#include "config_store.h"
#include "led_controller.h"
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
static constexpr const char* KEY_RUNTIME     = "runtime";  // uint32 seconds
static constexpr const char* KEY_MAINT_INT   = "maint_h";  // uint32 hours
static constexpr const char* KEY_MAINT_ACK   = "maint_ack";// uint8
static constexpr const char* KEY_LED_MODE    = "led_mode"; // uint8
static constexpr const char* KEY_CRC         = "crc";

// Fault log NVS keys (separate namespace to avoid bloating main config writes)
static constexpr const char* NVS_FAULTLOG_NS = "hmi_flog";
static constexpr const char* KEY_FLOG_COUNT  = "fl_cnt";   // uint8 entry count
static constexpr const char* KEY_FLOG_WRITE  = "fl_wr";    // uint8 write index (ring)
static constexpr const char* KEY_FLOG_DATA   = "fl_data";  // blob: FAULT_LOG_MAX_ENTRIES * sizeof(FaultLogEntry)

// Module state
static Preferences prefs_;
static Config      currentCfg_;
static bool        initialized_ = false;
static bool        dirty_       = false;     // Unsaved changes pending

// Fault log ring buffer (kept in memory, persisted on each logFault call)
static FaultLogEntry faultLog_[FAULT_LOG_MAX_ENTRIES] = {};
static uint8_t       faultLogCount_    = 0;
static uint8_t       faultLogWriteIdx_ = 0;

// Duplicate suppression: last logged state (errorCode + faultFlags)
// Prevents log flooding when a fault persists across multiple CAN frames.
static uint8_t  lastLoggedErrorCode_  = 0;
static uint8_t  lastLoggedFaultFlags_ = 0;
static uint32_t lastLoggedUptimeMs_   = 0;
static constexpr uint32_t FLOG_MIN_INTERVAL_MS = 1000;  // minimum 1 s between logs

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

// Forward declarations (internal helpers defined below)
static void loadFaultLog();

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

    // Load fault log from separate NVS namespace
    loadFaultLog();
}

bool load(Config& cfg) {
    prefs_.begin(NVS_NAMESPACE, true);  // Read-only

    cfg.driveMode       = prefs_.getUChar(KEY_MODE, 0);
    cfg.brightness      = prefs_.getUChar(KEY_BRIGHT, 255);
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

    // Load maintenance fields
    cfg.runtimeSeconds     = prefs_.getULong(KEY_RUNTIME, 0);
    cfg.maintIntervalHours = prefs_.getULong(KEY_MAINT_INT, MAINT_DEFAULT_INTERVAL_HOURS);
    cfg.maintAcknowledged  = prefs_.getUChar(KEY_MAINT_ACK, 0);
    cfg.ledMode            = prefs_.getUChar(KEY_LED_MODE, 0);

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
    prefs_.putULong(KEY_RUNTIME, toSave.runtimeSeconds);
    prefs_.putULong(KEY_MAINT_INT, toSave.maintIntervalHours);
    prefs_.putUChar(KEY_MAINT_ACK, toSave.maintAcknowledged);
    prefs_.putUChar(KEY_LED_MODE, toSave.ledMode);
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
    // Bounds-check every entry before touching the live config.  Valid values
    // are a position index 0..NUM_INA226_CH-1, or 0xFF (unset sentinel — see
    // Config::ina226Map doc).  A single out-of-range byte rejects the whole
    // update so a corrupt/garbage source can never overwrite a good mapping.
    // The NVS blob format is unchanged; nothing is erased on rejection.
    if (map == nullptr) {
        return;
    }
    for (uint8_t i = 0; i < NUM_INA226_CH; ++i) {
        if (map[i] >= NUM_INA226_CH && map[i] != 0xFF) {
            Serial.printf("[CFG] setIna226Map rejected: map[%u]=%u out of range\n",
                          (unsigned)i, (unsigned)map[i]);
            return;
        }
    }
    memcpy(currentCfg_.ina226Map, map, NUM_INA226_CH);
    dirty_ = true;
}

void setTempSensorMap(const uint8_t map[NUM_TEMP_SENS]) {
    // Bounds-check every entry before touching the live config.  Valid values
    // are a position index 0..NUM_TEMP_SENS-1, or 0xFF (unset sentinel).  A
    // single out-of-range byte rejects the whole update; the existing mapping
    // and NVS blob are left untouched.
    if (map == nullptr) {
        return;
    }
    for (uint8_t i = 0; i < NUM_TEMP_SENS; ++i) {
        if (map[i] >= NUM_TEMP_SENS && map[i] != 0xFF) {
            Serial.printf("[CFG] setTempSensorMap rejected: map[%u]=%u out of range\n",
                          (unsigned)i, (unsigned)map[i]);
            return;
        }
    }
    memcpy(currentCfg_.tempSensorMap, map, NUM_TEMP_SENS);
    dirty_ = true;
}

void setRuntimeSeconds(uint32_t seconds) {
    currentCfg_.runtimeSeconds = seconds;
    dirty_ = true;
}

void setMaintIntervalHours(uint32_t hours) {
    currentCfg_.maintIntervalHours = hours;
    dirty_ = true;
}

void setMaintAcknowledged(uint8_t ack) {
    currentCfg_.maintAcknowledged = ack;
    dirty_ = true;
}

void setLedMode(uint8_t mode) {
    if (mode >= led_ctrl::DECOR_MODE_COUNT) mode = 0;  // clamp to valid DecorMode range
    currentCfg_.ledMode = mode;
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
    clearFaultLog();
    Serial.println("[NVS] Factory reset complete");
}

// -------------------------------------------------------------------------
// Fault Log — persistent ring buffer in separate NVS namespace
// -------------------------------------------------------------------------

static void loadFaultLog() {
    Preferences fprefs;
    fprefs.begin(NVS_FAULTLOG_NS, true);  // read-only
    faultLogCount_    = fprefs.getUChar(KEY_FLOG_COUNT, 0);
    faultLogWriteIdx_ = fprefs.getUChar(KEY_FLOG_WRITE, 0);

    // Defensive bounds clamping (power-loss / corruption safety §1.3)
    if (faultLogCount_ > FAULT_LOG_MAX_ENTRIES) faultLogCount_ = 0;
    if (faultLogWriteIdx_ >= FAULT_LOG_MAX_ENTRIES) faultLogWriteIdx_ = 0;

    // Cross-check: when buffer is not full, writeIdx must equal count.
    // If inconsistent (power loss mid-write), reset to safe state.
    if (faultLogCount_ < FAULT_LOG_MAX_ENTRIES &&
        faultLogWriteIdx_ != faultLogCount_) {
        faultLogCount_    = 0;
        faultLogWriteIdx_ = 0;
    }

    size_t expected = FAULT_LOG_MAX_ENTRIES * sizeof(FaultLogEntry);
    size_t actual   = fprefs.getBytes(KEY_FLOG_DATA, faultLog_, expected);
    if (actual != expected) {
        // Corrupt or first boot — reset
        memset(faultLog_, 0, sizeof(faultLog_));
        faultLogCount_    = 0;
        faultLogWriteIdx_ = 0;
    }
    fprefs.end();
    Serial.printf("[NVS] Fault log loaded: %u entries\n", faultLogCount_);
}

static void saveFaultLog() {
    Preferences fprefs;
    fprefs.begin(NVS_FAULTLOG_NS, false);  // read-write
    fprefs.putUChar(KEY_FLOG_COUNT, faultLogCount_);
    fprefs.putUChar(KEY_FLOG_WRITE, faultLogWriteIdx_);
    fprefs.putBytes(KEY_FLOG_DATA, faultLog_,
                    FAULT_LOG_MAX_ENTRIES * sizeof(FaultLogEntry));
    fprefs.end();
}

void logFault(const FaultLogEntry& entry) {
    // --- Duplicate suppression (§1.1) ---
    // Only log on rising edge: errorCode changed OR faultFlags changed meaningfully.
    bool isDuplicate = (entry.errorCode  == lastLoggedErrorCode_ &&
                        entry.faultFlags == lastLoggedFaultFlags_);

    if (isDuplicate) {
        return;  // Exact same state — suppress
    }

    // --- Write rate protection / debounce (§1.2, §2) ---
    // Rate-limit faultFlags toggling for the SAME errorCode to filter CAN noise.
    // New error codes always pass through immediately to avoid missing transitions.
    // Unsigned subtraction is safe for millis() wrap (~49.7 days):
    // if uptime < last (e.g. reboot), delta wraps to a large value → not "too soon".
    if (entry.errorCode == lastLoggedErrorCode_ && lastLoggedUptimeMs_ != 0) {
        uint32_t delta = entry.uptimeMs - lastLoggedUptimeMs_;
        if (delta < FLOG_MIN_INTERVAL_MS) {
            return;  // Same error, flags changed but too soon — debounce
        }
    }

    faultLog_[faultLogWriteIdx_] = entry;
    // Advance write pointer (overwrites oldest entry when full — circular buffer)
    faultLogWriteIdx_ = (faultLogWriteIdx_ + 1) % FAULT_LOG_MAX_ENTRIES;
    if (faultLogCount_ < FAULT_LOG_MAX_ENTRIES) {
        faultLogCount_++;
    }

    // Update suppression state
    lastLoggedErrorCode_  = entry.errorCode;
    lastLoggedFaultFlags_ = entry.faultFlags;
    lastLoggedUptimeMs_   = entry.uptimeMs;

    saveFaultLog();
    Serial.printf("[NVS] Fault logged: code=%u flags=0x%02X sub=%u state=%u\n",
                  entry.errorCode, entry.faultFlags,
                  entry.subsystem, entry.systemState);
}

uint8_t getFaultLogCount() {
    return faultLogCount_;
}

bool getFaultLogEntry(uint8_t index, FaultLogEntry& out) {
    if (index >= faultLogCount_ || faultLogCount_ == 0) return false;
    // Ring buffer: oldest entry is at (writeIdx - count) mod MAX
    uint8_t oldest = (faultLogWriteIdx_ + FAULT_LOG_MAX_ENTRIES - faultLogCount_)
                     % FAULT_LOG_MAX_ENTRIES;
    uint8_t actual = (oldest + index) % FAULT_LOG_MAX_ENTRIES;
    // Defensive bounds clamp (§1.3 — should never fire, but protects against corruption)
    if (actual >= FAULT_LOG_MAX_ENTRIES) actual = 0;
    out = faultLog_[actual];
    return true;
}

void clearFaultLog() {
    memset(faultLog_, 0, sizeof(faultLog_));
    faultLogCount_    = 0;
    faultLogWriteIdx_ = 0;
    Preferences fprefs;
    fprefs.begin(NVS_FAULTLOG_NS, false);
    fprefs.clear();
    fprefs.end();
    Serial.println("[NVS] Fault log cleared");
}

// -------------------------------------------------------------------------
// Maintenance status helpers
// -------------------------------------------------------------------------

bool isMaintenanceDue() {
    uint32_t thresholdSec = currentCfg_.maintIntervalHours * 3600UL;
    return currentCfg_.runtimeSeconds >= thresholdSec;
}

void acknowledgeMaintenance() {
    currentCfg_.maintAcknowledged = 1;
    dirty_ = true;
}

void resetMaintenanceCounter() {
    currentCfg_.runtimeSeconds    = 0;
    currentCfg_.maintAcknowledged = 0;
    dirty_ = true;
}

} // namespace config_store
