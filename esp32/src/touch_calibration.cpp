// =============================================================================
// ESP32-S3 HMI — Touch Calibration Persistence (NVS) — Implementation
//
// Uses the Arduino Preferences library (NVS wrapper) — same approach as
// `config_store` for fault log + main HMI config.  Storage namespace is
// dedicated (`touch_cal`) so that `config_store::factoryReset()` does NOT
// wipe touch calibration accidentally.
//
// All public APIs are blocking but very short (Preferences::begin/end pairs);
// they are called from `setup()` and from the wizard screen, never from the
// CAN poll path or from the render-frame critical section.
// =============================================================================

#include "touch_calibration.h"
#include <Preferences.h>
#include <Arduino.h>
#include <cstddef>
#include <cstring>

namespace touch_calibration {

// -------------------------------------------------------------------------
// NVS storage layout
// -------------------------------------------------------------------------
static constexpr const char*    NVS_NAMESPACE   = "touch_cal";
static constexpr const char*    KEY_DATA        = "data";        // blob
static constexpr const char*    KEY_FIRST_DONE  = "first_done";  // bool
static constexpr uint32_t       MAGIC           = 0x54434C31UL;  // "TCL1"

// XPT2046 raw value range
static constexpr uint16_t       RAW_MAX         = 4095;

// Persisted record (POD — we hash the bytes directly).  The layout is
// fixed once written; do NOT reorder fields without bumping MAGIC,
// otherwise old blobs would be silently misread (CRC would still reject
// them, but explicit versioning is clearer).
//
// Explicit 2-byte padding (`_pad2`) keeps the layout deterministic across
// compilers — the natural alignment of `crc32` is 4 bytes, so without
// `_pad2` the compiler would insert anonymous padding that we cannot
// guarantee to be zeroed on every code path (it IS zeroed by `Record rec = {};`
// today, but this makes the on-disk image self-describing and stable).
struct Record {
    uint32_t magic;       // offset  0
    uint16_t xMin;        // offset  4
    uint16_t xMax;        // offset  6
    uint16_t yMin;        // offset  8
    uint16_t yMax;        // offset 10
    uint8_t  rotation;    // offset 12
    uint8_t  _pad;        // offset 13
    uint16_t _pad2;       // offset 14
    uint32_t crc32;       // offset 16
};

static_assert(sizeof(Record) == 20, "touch_calibration::Record size mismatch");
static_assert(offsetof(Record, crc32) == 16,
              "touch_calibration::Record CRC offset mismatch");

// -------------------------------------------------------------------------
// CRC-32/ISO-HDLC — same polynomial as config_store, kept local to avoid
// adding a cross-module dependency for a 12-byte hash.
// -------------------------------------------------------------------------
static uint32_t computeCrc32(const Record& rec) {
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&rec);
    const size_t   len  = offsetof(Record, crc32);

    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 1U) {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

// -------------------------------------------------------------------------
// Validation helper — pure, no NVS access
// -------------------------------------------------------------------------
static bool isValid(const Record& rec) {
    if (rec.magic != MAGIC) return false;

    Record copy = rec;
    copy.crc32  = 0;
    if (computeCrc32(copy) != rec.crc32) return false;

    // xMin/yMin are raw XPT2046 ADC values (0-4095).
    // xMax/yMax store the pre-computed axis RANGES (max_raw - min_raw) as
    // returned by TFT_eSPI::calibrateTouch() in parameters[1] and [3].
    // They are NOT the raw maximum values, so we only bound-check xMin/yMin.
    if (rec.xMin > RAW_MAX || rec.yMin > RAW_MAX) return false;

    // Range must be large enough to be a meaningful calibration.
    if (rec.xMax < MIN_RANGE || rec.yMax < MIN_RANGE) return false;

    // rec.rotation stores the TFT_eSPI calibration flags bitmask returned in
    // parameters[4] by calibrateTouch():
    //   bit 0 = rotate (swap X/Y axes)
    //   bit 1 = invert_x
    //   bit 2 = invert_y
    // Valid range is 0-7 (3 bits).  This is NOT the display rotation mode
    // (TFT_ROTATION) — do not compare against it.
    if (rec.rotation > 7) return false;

    return true;
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------
void init() {
    // Touching the namespace creates it on first boot so subsequent reads
    // do not generate noisy "namespace not found" log lines.
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, false);
    prefs.end();
}

bool loadValid(uint16_t out[CAL_DATA_LEN]) {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, true);  // read-only

    Record rec = {};
    size_t got = prefs.getBytes(KEY_DATA, &rec, sizeof(rec));
    prefs.end();

    if (got != sizeof(rec)) {
        // Either no entry (first boot) or partial write — both treated as
        // "invalid"; caller falls back to compile-time TOUCH_CALIBRATION.
        return false;
    }
    if (!isValid(rec)) {
        Serial.println("[TOUCH_CAL] Stored calibration invalid — using fallback");
        return false;
    }

    out[0] = rec.xMin;
    out[1] = rec.xMax;
    out[2] = rec.yMin;
    out[3] = rec.yMax;
    out[4] = rec.rotation;
    Serial.printf("[TOUCH_CAL] Loaded: { %u, %u, %u, %u, %u }\n",
                  out[0], out[1], out[2], out[3], out[4]);
    return true;
}

bool save(const uint16_t data[CAL_DATA_LEN]) {
    Record rec = {};
    rec.magic    = MAGIC;
    rec.xMin     = data[0];
    rec.xMax     = data[1];
    rec.yMin     = data[2];
    rec.yMax     = data[3];
    rec.rotation = static_cast<uint8_t>(data[4]);
    rec._pad     = 0;
    rec._pad2    = 0;     // explicit (already zeroed by value-init above; kept for clarity)
    rec.crc32    = 0;
    rec.crc32    = computeCrc32(rec);

    // Sanity-check before committing — refuse to persist garbage that
    // would be rejected on next boot anyway.
    if (!isValid(rec)) {
        Serial.println("[TOUCH_CAL] Refusing to save invalid calibration");
        return false;
    }

    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, false);  // read-write
    size_t written = prefs.putBytes(KEY_DATA, &rec, sizeof(rec));
    prefs.putBool(KEY_FIRST_DONE, true);
    prefs.end();

    if (written != sizeof(rec)) {
        Serial.printf("[TOUCH_CAL] Save FAILED: wrote %u/%u bytes\n",
                      (unsigned)written, (unsigned)sizeof(rec));
        return false;
    }

    Serial.printf("[TOUCH_CAL] Saved: { %u, %u, %u, %u, %u }\n",
                  data[0], data[1], data[2], data[3], data[4]);
    return true;
}

void clear() {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, false);
    prefs.remove(KEY_DATA);
    // first_done is intentionally NOT removed here.
    prefs.end();
    Serial.println("[TOUCH_CAL] Calibration data cleared (first_done preserved)");
}

void factoryReset() {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, false);
    prefs.clear();
    prefs.end();
    Serial.println("[TOUCH_CAL] Factory reset — wizard re-armed for next boot");
}

bool firstBootDone() {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, true);
    bool done = prefs.getBool(KEY_FIRST_DONE, false);
    prefs.end();
    return done;
}

void markFirstBootDone() {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putBool(KEY_FIRST_DONE, true);
    prefs.end();
}

} // namespace touch_calibration
