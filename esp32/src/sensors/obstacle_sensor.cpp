// =============================================================================
// ESP32-S3 — Obstacle Sensor Driver (implementation)
//
// Reads TOFSense-M 8×8 LiDAR sensor via UART1.
// Parses NLink_TOFSense_M_Frame0 protocol (400-byte frames).
// Validates readings against physical range, detects stuck sensor,
// and provides a warmup period for sensor stabilization.
//
// Sensor: TOFSense-M S (Nooploop), UART 921600 bps, active output mode.
// Matches STM32 expectations for CAN 0x208 payload.
//
// Reference: TOFSense-M User Manual V3.0
//            https://ftp.nooploop.com/downloads/tofsense/TOFSense-M_User_Manual_V3.0_en.pdf
// =============================================================================

#include "obstacle_sensor.h"
#include <Arduino.h>
#include <HardwareSerial.h>

namespace obstacle_sensor {

// -------------------------------------------------------------------------
// NLink_TOFSense_M_Frame0 protocol constants
// Reference: TOFSense-M User Manual V3.0 (Nooploop)
//            https://github.com/nooploop-dev/autorobo_a — nlink_tofsensem_frame0.c
// -------------------------------------------------------------------------
static constexpr uint8_t  FRAME_HEADER      = 0x57;
static constexpr uint8_t  FUNCTION_MARK     = 0x01;   // Frame0 function mark
static constexpr uint16_t FRAME_LENGTH      = 400;    // Total frame size (bytes) for 8×8 mode
static constexpr uint8_t  PIXEL_COUNT_8X8   = 64;     // 8×8 matrix
static constexpr uint8_t  BYTES_PER_PIXEL   = 6;      // 3 (distance) + 1 (dis_status) + 2 (signal_strength)
// Minimum number of valid pixels required to trust a frame's distance data.
// When the TOFSense-M is in LONG RANGE mode and an object is within the
// sensor's blind zone (< ~50 mm), most pixels saturate (dis_status ≠ 0)
// but a few may report valid distances at 2000–3000 mm due to phase
// wrap-around artifacts.  This produces the "data reversed" symptom:
// very close objects appear far.  Requiring at least MIN_VALID_PIXELS
// valid pixels rejects these artifacts and triggers emergency-close
// (NO_VALID_PIXELS → minRangeMm) instead.
static constexpr uint8_t  MIN_VALID_PIXELS  = 4;
// NLink protocol distance unit: 1 mm per LSB (raw int24 value IS in mm).
// The official Nooploop code (nlink_tofsensem_frame0.c) divides by 1000.0f
// to produce a float in meters; we keep the raw mm value for our uint16
// Reading::distance_mm field.

// Frame offsets (per official Nooploop struct ntsm_frame0_raw_t)
static constexpr uint16_t OFF_HEADER        = 0;      // 0x57
static constexpr uint16_t OFF_FUNCTION_MARK = 1;      // 0x01 (Frame0)
static constexpr uint16_t OFF_RESERVED      = 2;      // Reserved
static constexpr uint16_t OFF_ID            = 3;      // Sensor ID
static constexpr uint16_t OFF_SYSTEM_TIME   = 4;      // System time (4 bytes, LE)
static constexpr uint16_t OFF_PIXEL_COUNT   = 8;      // Pixel count (64 for 8×8)
static constexpr uint16_t OFF_PIXEL_DATA    = 9;      // Start of pixel data
// Pixel data: 64 pixels × 6 bytes = 384 bytes (offset 9..392)
// Reserved1: 6 bytes (offset 393..398)
// Checksum at offset 399 (8-bit sum mod 256 of bytes 0..398)

// -------------------------------------------------------------------------
// Module state
// -------------------------------------------------------------------------
static Config       cfg_;
static Reading      reading_;
static unsigned long initTimeMs_       = 0;
static unsigned long lastValidMs_      = 0;
static uint16_t      prevDistanceMm_   = 0;
static unsigned long stuckSinceMs_     = 0;
static bool          stuckActive_      = false;
static bool          initialized_      = false;
static bool          warmupDone_       = false;

// UART receive buffer
static uint8_t       rxBuf_[FRAME_LENGTH];
static uint16_t      rxIdx_            = 0;

// UART1 for TOFSense-M
static HardwareSerial tofSerial(1);

// -------------------------------------------------------------------------
// Diagnostic counters — logged every DIAG_INTERVAL_MS to Serial for
// debugging wiring / signal-level problems (e.g. wrong voltage divider
// resistor values → marginal signal → intermittent checksum failures).
// -------------------------------------------------------------------------
static constexpr unsigned long DIAG_INTERVAL_MS = 5000;
static constexpr uint32_t DIAG_MIN_FRAMES_FOR_RANGE_CHECK      = 10;   // Minimum OK frames before checking range mode
static constexpr uint16_t SHORT_RANGE_DETECTION_THRESHOLD_MM   = 1500; // If max distance < this, likely SHORT RANGE mode
static unsigned long lastDiagMs_      = 0;
static uint32_t diagFramesOk_        = 0;  // Frames parsed successfully
static uint32_t diagChecksumFail_    = 0;  // Checksum mismatches
static uint32_t diagHeaderFail_      = 0;  // Wrong header or function mark
static uint32_t diagAllPixelsInvalid_= 0;  // All 64 dis_status ≠ 0
static uint32_t diagBytesDiscarded_  = 0;  // Bytes skipped waiting for 0x57
static uint16_t diagMaxDistMm_       = 0;  // Maximum distance seen in current diag interval
static bool     diagShortRangeWarned_= false; // Only warn about SHORT RANGE once per session

// -------------------------------------------------------------------------
// Zone mapping — matches STM32 distance tiers (safety_system.c)
//   < 200 mm  → zone 4 (emergency, scale=0.0)
//   200–500   → zone 3 (critical,  scale=0.3)
//   500–1000  → zone 2 (warning,   scale=0.7)
//   1000–1500 → zone 1 (caution,   scale=0.85)
//   > 1500    → zone 0 (normal,    scale=1.0)
// -------------------------------------------------------------------------
static constexpr uint16_t ZONE_EMERGENCY_MM = 200;
static constexpr uint16_t ZONE_CRITICAL_MM  = 500;
static constexpr uint16_t ZONE_WARNING_MM   = 1000;
static constexpr uint16_t ZONE_CAUTION_MM   = 1500;

static uint8_t distanceToZone(uint16_t mm) {
    if (mm < ZONE_EMERGENCY_MM) return 4;
    if (mm < ZONE_CRITICAL_MM)  return 3;
    if (mm < ZONE_WARNING_MM)   return 2;
    if (mm < ZONE_CAUTION_MM)   return 1;
    return 0;
}

// -------------------------------------------------------------------------
// Parse result — allows update() to distinguish failure modes for diag
// -------------------------------------------------------------------------
enum class ParseResult : uint8_t {
    OK,                // Successful parse
    TOO_SHORT,         // Buffer too short
    BAD_HEADER,        // Wrong header or function_mark
    BAD_CHECKSUM,      // 8-bit sum mismatch
    NO_VALID_PIXELS    // All 64 pixels have dis_status ≠ 0
};

// -------------------------------------------------------------------------
// Parse a complete NLink_TOFSense_M_Frame0 and extract minimum distance
// Returns distance in mm via outDistMm, or 0 on failure.
//
// Per-pixel layout (6 bytes, per official Nooploop struct ntsm_frame0_pixel_raw_t):
//   [0-2]  dis:              3-byte signed int24 LE, unit = mm
//                            (official code divides by 1000.0f to get meters)
//   [3]    dis_status:       0 = valid measurement
//   [4-5]  signal_strength:  uint16 LE
// -------------------------------------------------------------------------
static ParseResult parseFrame(const uint8_t* buf, uint16_t len, uint16_t& outDistMm) {
    outDistMm = 0;
    if (len < FRAME_LENGTH) return ParseResult::TOO_SHORT;
    if (buf[OFF_HEADER] != FRAME_HEADER || buf[OFF_FUNCTION_MARK] != FUNCTION_MARK)
        return ParseResult::BAD_HEADER;

    // Verify checksum: 8-bit sum of all bytes except the checksum byte itself
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < FRAME_LENGTH - 1; i++) {
        checksum += buf[i];
    }
    if (checksum != buf[FRAME_LENGTH - 1]) return ParseResult::BAD_CHECKSUM;

    // Extract minimum valid distance across all 64 pixels.
    uint32_t minDistMm = 0xFFFFFFFF;
    uint8_t validCount = 0;

    for (uint8_t px = 0; px < PIXEL_COUNT_8X8; px++) {
        uint16_t base = OFF_PIXEL_DATA + (uint16_t)(px * BYTES_PER_PIXEL);
        if (base + BYTES_PER_PIXEL > FRAME_LENGTH) break;

        uint8_t status = buf[base + 3];
        if (status != 0) continue;  // Skip invalid pixels

        // Distance: 3-byte signed little-endian, unit = mm per LSB
        // (per official Nooploop nlink_tofsensem_frame0.c — NLINK_ParseInt24)
        int32_t raw = (int32_t)buf[base]
                    | ((int32_t)buf[base + 1] << 8)
                    | ((int32_t)buf[base + 2] << 16);
        // Sign-extend 24-bit value to 32-bit: check bit 23 (sign bit)
        if (raw & 0x800000) raw |= (int32_t)0xFF000000;

        if (raw <= 0) continue;  // Negative or zero distance — skip

        uint32_t distMm = (uint32_t)raw;  // Already in mm
        validCount++;
        if (distMm < minDistMm) {
            minDistMm = distMm;
        }
    }

    if (validCount == 0) return ParseResult::NO_VALID_PIXELS;

    // When very few pixels are valid while most are invalid, the valid
    // ones are likely phase wrap-around artifacts from an object within
    // the sensor's blind zone (< ~50 mm).  Treat as emergency-close
    // instead of reporting the misleading high distances.
    if (validCount < MIN_VALID_PIXELS) return ParseResult::NO_VALID_PIXELS;

    outDistMm = (minDistMm > 0xFFFF) ? (uint16_t)0xFFFF : (uint16_t)minDistMm;
    return ParseResult::OK;
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init(const Config& cfg) {
    cfg_ = cfg;

    // Set UART RX ring-buffer size BEFORE begin().
    // Default ESP32 buffer is 256 bytes — too small for the 400-byte
    // TOFSense-M frames at 921600 bps.  With a 256-byte buffer the ring
    // buffer fills in ~2.8 ms; any main-loop cycle longer than that
    // (display update, CAN processing, etc.) causes bytes to be lost,
    // corrupting the frame and triggering checksum failures that make
    // the sensor oscillate between VALID and INVALID.
    // 2048 bytes ≈ 5 full frames → ~22 ms of headroom.
    tofSerial.setRxBufferSize(cfg_.rxBufSize);

    // Initialize UART1 for TOFSense-M
    tofSerial.begin(cfg_.baudRate, SERIAL_8N1, cfg_.rxPin, cfg_.txPin);

    initTimeMs_    = millis();
    lastValidMs_   = 0;
    prevDistanceMm_ = 0;
    stuckSinceMs_  = 0;
    stuckActive_   = false;
    warmupDone_    = false;
    initialized_   = true;
    rxIdx_         = 0;

    // Reset diagnostic counters
    lastDiagMs_         = millis();
    diagFramesOk_       = 0;
    diagChecksumFail_   = 0;
    diagHeaderFail_     = 0;
    diagAllPixelsInvalid_ = 0;
    diagBytesDiscarded_ = 0;
    diagMaxDistMm_      = 0;
    diagShortRangeWarned_ = false;

    reading_ = Reading{};  // Reset to defaults

    Serial.printf("[OBSTACLE] TOFSense-M init (UART1, %lu bps, rxBuf %u, rxPin %d, txPin %d)\n",
                  (unsigned long)cfg_.baudRate, (unsigned)cfg_.rxBufSize,
                  cfg_.rxPin, cfg_.txPin);
    if (cfg_.txPin < 0) {
        Serial.println("[OBSTACLE] TX pin not connected — config commands "
                       "(setRangeMode, configureLongRange, saveConfig, etc.) "
                       "will not work. If the sensor is not in LONG RANGE mode, "
                       "connect ESP32 TX (e.g. GPIO 17) to sensor RX and set "
                       "Config::txPin, or use NAssistant to configure the sensor.");
    }
}

void update(float vehicleSpeedKmh) {
    if (!initialized_) return;

    unsigned long now = millis();

    // Warmup period — report WAITING
    if (now - initTimeMs_ < cfg_.warmupMs) {
        reading_.status  = SensorStatus::WAITING;
        reading_.healthy = false;
        reading_.stuck   = false;
        return;
    }

    // First call after warmup: flush stale UART data that accumulated
    // while update() was returning early during the warmup period.
    // Without this flush, the first post-warmup read contains a partial
    // frame (buffer overflow during warmup), causing false sync and
    // cascading checksum failures until re-alignment.
    if (!warmupDone_) {
        while (tofSerial.available() > 0) {
            tofSerial.read();
        }
        rxIdx_ = 0;
        warmupDone_ = true;
    }

    // Read all available UART bytes and parse frames.
    // When multiple frames are queued (e.g. after a display refresh or CAN
    // burst), processing them all and keeping only the last valid result is
    // both simpler and more robust than discarding bytes at arbitrary
    // positions (which risks breaking frame alignment and causing cascading
    // checksum failures — the root cause of intermittent INVALID readings).
    uint16_t measuredMm = 0;
    bool gotFrame = false;

    while (tofSerial.available() > 0) {
        uint8_t byte = (uint8_t)tofSerial.read();

        // Synchronize on header byte
        if (rxIdx_ == 0 && byte != FRAME_HEADER) {
            diagBytesDiscarded_++;
            continue;  // Discard until we find the header
        }

        // Validate function_mark as second sync byte.  The header value
        // 0x57 can appear naturally inside pixel distance data (~1.5×
        // per frame).  Checking byte[1]==0x01 immediately avoids locking
        // onto a false header, which would consume 400 bytes before
        // parseFrame() rejects it — potentially missing the real frame.
        if (rxIdx_ == 1 && byte != FUNCTION_MARK) {
            // Not a valid frame start — reset.
            // If this byte is itself a header, keep it as byte[0].
            if (byte == FRAME_HEADER) {
                rxBuf_[0] = byte;
                // rxIdx_ stays at 1
            } else {
                rxIdx_ = 0;
                diagBytesDiscarded_++;
            }
            continue;
        }

        rxBuf_[rxIdx_++] = byte;

        // Complete frame received
        if (rxIdx_ >= FRAME_LENGTH) {
            uint16_t dist = 0;
            ParseResult pr = parseFrame(rxBuf_, rxIdx_, dist);
            switch (pr) {
                case ParseResult::OK:
                    measuredMm = dist;
                    gotFrame = true;
                    diagFramesOk_++;
                    if (dist > diagMaxDistMm_) diagMaxDistMm_ = dist;
                    break;
                case ParseResult::BAD_HEADER:
                    diagHeaderFail_++;
                    break;
                case ParseResult::BAD_CHECKSUM:
                    diagChecksumFail_++;
                    break;
                case ParseResult::NO_VALID_PIXELS:
                    diagAllPixelsInvalid_++;
                    // All pixels report dis_status ≠ 0.  The TOFSense-M
                    // cannot measure distances below ~50 mm; when an object
                    // is that close every pixel saturates and reports
                    // invalid.  Treat this as "too close to measure" —
                    // an emergency-close detection at minRangeMm — instead
                    // of letting the frame-timeout mark the sensor INVALID
                    // (which would look like a sensor fault, not an
                    // obstacle).
                    measuredMm = cfg_.minRangeMm;
                    gotFrame   = true;
                    break;
                default:
                    break;
            }
            rxIdx_ = 0;  // Reset for next frame
        }
    }

    // Periodic diagnostic output (every DIAG_INTERVAL_MS)
    if (now - lastDiagMs_ >= DIAG_INTERVAL_MS) {
        uint32_t totalFail = diagChecksumFail_ + diagHeaderFail_ + diagAllPixelsInvalid_;
        if (diagFramesOk_ > 0 || totalFail > 0 || diagBytesDiscarded_ > 0) {
            Serial.printf("[OBSTACLE] Diag %lus: OK=%lu cksumFail=%lu hdrFail=%lu "
                          "noPixels=%lu discarded=%lu\n",
                          (unsigned long)(DIAG_INTERVAL_MS / 1000),
                          (unsigned long)diagFramesOk_,
                          (unsigned long)diagChecksumFail_,
                          (unsigned long)diagHeaderFail_,
                          (unsigned long)diagAllPixelsInvalid_,
                          (unsigned long)diagBytesDiscarded_);
            if (diagChecksumFail_ > diagFramesOk_ && diagChecksumFail_ > 0) {
                Serial.println("[OBSTACLE] WARNING: Most frames fail checksum. "
                               "Check voltage divider resistor values — if you "
                               "measure ~2.1V on GPIO 18, R1 (series) is likely "
                               "3.3kohm instead of 1kohm. Expected ~2.9V with "
                               "correct R1=1kohm (series) + R2=4.7kohm (to GND). "
                               "See TOFSENSE_M_WIRING_GUIDE.md section 5");
            }
            // Detect "always 20 mm" symptom: when all (or most) frames have
            // NO valid pixels, the driver returns minRangeMm (20 mm).  This
            // almost always means the sensor is in SHORT RANGE mode and there
            // is nothing within its ~1.3 m range — every pixel saturates and
            // reports dis_status ≠ 0.  Warn with actionable instructions.
            if (!diagShortRangeWarned_
                    && diagAllPixelsInvalid_ > DIAG_MIN_FRAMES_FOR_RANGE_CHECK
                    && diagFramesOk_ == 0) {
                Serial.printf("[OBSTACLE] WARNING: All %lu frames returned "
                              "NO valid pixels (distance stuck at %u mm). "
                              "Sensor is likely in SHORT RANGE mode with no "
                              "obstacle within ~1.3 m.  Fix: call "
                              "configureLongRange() with txPin connected, "
                              "or use NAssistant to switch to Long Range mode.\n",
                              (unsigned long)diagAllPixelsInvalid_,
                              (unsigned)cfg_.minRangeMm);
                diagShortRangeWarned_ = true;
            }
            if (!diagShortRangeWarned_
                    && diagFramesOk_ > DIAG_MIN_FRAMES_FOR_RANGE_CHECK
                    && diagMaxDistMm_ > 0
                    && diagMaxDistMm_ < SHORT_RANGE_DETECTION_THRESHOLD_MM) {
                Serial.printf("[OBSTACLE] WARNING: Max distance seen = %u mm "
                              "(expected up to 4000 mm in LONG RANGE mode). "
                              "Sensor may still be in SHORT RANGE / HIGH PRECISION "
                              "mode. Fix: connect ESP32 TX to sensor RX, set "
                              "Config::txPin, and call configureLongRange(); or use "
                              "NAssistant to switch to Long Range mode.\n",
                              (unsigned)diagMaxDistMm_);
                diagShortRangeWarned_ = true;  // Only warn once per session
            }
        } else if (reading_.status != SensorStatus::WAITING) {
            Serial.println("[OBSTACLE] Diag: no UART data received — check wiring "
                           "and baud rate (expected 921600 bps factory default). "
                           "If sensor was reconfigured with NAssistant, reset it "
                           "to 921600 or update Config::baudRate to match");
        }
        // Reset counters for next interval
        diagFramesOk_        = 0;
        diagChecksumFail_    = 0;
        diagHeaderFail_      = 0;
        diagAllPixelsInvalid_= 0;
        diagBytesDiscarded_  = 0;
        diagMaxDistMm_       = 0;
        lastDiagMs_          = now;
    }

    if (!gotFrame) {
        // No valid frame received — check timeout
        if (lastValidMs_ > 0 && (now - lastValidMs_) > cfg_.frameTimeoutMs) {
            reading_.status  = SensorStatus::INVALID;
            reading_.healthy = false;
        }
        return;
    }

    // Range validation
    // Above-maximum distances indicate no obstacle within the sensor's
    // effective range — this is normal operation (open air / distant
    // background).  Clamp to maxRangeMm so the reading stays VALID at
    // zone 0 (no obstacle), instead of marking it INVALID (which would
    // look like a sensor fault to the STM32 and trigger fallback mode).
    if (measuredMm > cfg_.maxRangeMm) {
        measuredMm = cfg_.maxRangeMm;
    }
    // Clamp below-minimum distances (object too close for accurate
    // measurement).  Report as minRangeMm so the reading enters the
    // emergency zone instead of being marked INVALID.
    if (measuredMm < cfg_.minRangeMm) {
        measuredMm = cfg_.minRangeMm;
    }

    // Valid reading — update distance
    reading_.distance_mm = measuredMm;
    reading_.zone        = distanceToZone(measuredMm);
    lastValidMs_         = now;

    // Stuck-sensor detection:
    // If distance unchanged (within threshold) while vehicle moving > threshold
    uint16_t diff = (measuredMm > prevDistanceMm_)
                    ? (measuredMm - prevDistanceMm_)
                    : (prevDistanceMm_ - measuredMm);

    if (diff < cfg_.stuckThresholdMm && vehicleSpeedKmh > cfg_.minSpeedForStuck) {
        if (stuckSinceMs_ == 0) {
            stuckSinceMs_ = now;
        }
        if (now - stuckSinceMs_ >= cfg_.stuckDurationMs) {
            stuckActive_ = true;
        }
    } else {
        stuckSinceMs_ = 0;
        stuckActive_  = false;
    }

    prevDistanceMm_ = measuredMm;

    reading_.stuck   = stuckActive_;
    reading_.healthy = !stuckActive_;
    reading_.status  = stuckActive_ ? SensorStatus::INVALID : SensorStatus::VALID;
}

Reading getReading() {
    return reading_;
}

// =========================================================================
// NLink 0x5A configuration command implementation
// =========================================================================

uint16_t buildCommand(uint8_t cmdId, const uint8_t* payload,
                      uint8_t payloadLen, uint8_t* outBuf,
                      uint16_t outBufSize) {
    // Frame: [0x5A] [len] [cmdId] [payload...] [checksum]
    // Total length = 1 (header) + 1 (len) + 1 (cmd) + payloadLen + 1 (checksum)
    uint16_t totalLen = (uint16_t)(3 + payloadLen + 1);
    if (totalLen > outBufSize) return 0;

    outBuf[0] = CMD_HEADER;                  // 0x5A
    outBuf[1] = (uint8_t)totalLen;           // Total frame length
    outBuf[2] = cmdId;                       // Command ID
    for (uint8_t i = 0; i < payloadLen; i++) {
        outBuf[3 + i] = payload[i];
    }
    // Checksum: sum of all bytes except the checksum byte itself
    uint8_t sum = 0;
    for (uint16_t i = 0; i < totalLen - 1; i++) {
        sum += outBuf[i];
    }
    outBuf[totalLen - 1] = sum;
    return totalLen;
}

// Internal helper: send a command frame over UART TX
static bool sendCmd(const uint8_t* frame, uint16_t len) {
    if (!initialized_) return false;
    if (cfg_.txPin < 0) return false;   // TX pin not connected
    tofSerial.write(frame, len);
    return true;
}

bool setRangeMode(RangeMode mode) {
    uint8_t buf[8];
    uint8_t payload = static_cast<uint8_t>(mode);
    uint16_t len = buildCommand(static_cast<uint8_t>(CmdId::SET_RANGE_MODE),
                                &payload, 1, buf, sizeof(buf));
    if (len == 0) return false;
    return sendCmd(buf, len);
}

bool setBaudRate(BaudRateCode baud) {
    uint8_t buf[8];
    uint8_t payload = static_cast<uint8_t>(baud);
    uint16_t len = buildCommand(static_cast<uint8_t>(CmdId::SET_BAUD_RATE),
                                &payload, 1, buf, sizeof(buf));
    if (len == 0) return false;
    return sendCmd(buf, len);
}

bool setFrameRate(uint8_t hz) {
    if (hz == 0) return false;          // 0 Hz is not a valid frame rate
    uint8_t buf[8];
    uint16_t len = buildCommand(static_cast<uint8_t>(CmdId::SET_FRAME_RATE),
                                &hz, 1, buf, sizeof(buf));
    if (len == 0) return false;
    return sendCmd(buf, len);
}

bool setOutputMode(OutputMode mode) {
    uint8_t buf[8];
    uint8_t payload = static_cast<uint8_t>(mode);
    uint16_t len = buildCommand(static_cast<uint8_t>(CmdId::SET_OUTPUT_MODE),
                                &payload, 1, buf, sizeof(buf));
    if (len == 0) return false;
    return sendCmd(buf, len);
}

bool saveConfig() {
    uint8_t buf[8];
    uint16_t len = buildCommand(static_cast<uint8_t>(CmdId::SAVE_CONFIG),
                                nullptr, 0, buf, sizeof(buf));
    if (len == 0) return false;
    return sendCmd(buf, len);
}

// Delay between NLink 0x5A configuration commands (ms).
// The TOFSense-M needs time to process each command before accepting the next.
// Without this delay, sending multiple commands back-to-back may cause the
// sensor to miss commands after the first one — resulting in partial
// configuration (e.g. LONG RANGE set but not saved, or save issued before
// the range-mode change is applied internally).
// 100 ms is conservative; the sensor typically processes in < 50 ms.
static constexpr unsigned long CMD_INTER_DELAY_MS = 100;

bool configureLongRange() {
    if (!setRangeMode(RangeMode::LONG_RANGE))  return false;
    delay(CMD_INTER_DELAY_MS);
    if (!setOutputMode(OutputMode::ACTIVE))     return false;
    delay(CMD_INTER_DELAY_MS);
    if (!setFrameRate(10))                      return false;
    delay(CMD_INTER_DELAY_MS);
    if (!saveConfig())                          return false;
    return true;
}

} // namespace obstacle_sensor
