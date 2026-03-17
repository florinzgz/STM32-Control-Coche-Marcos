// =============================================================================
// ESP32-S3 — Obstacle Sensor Driver (implementation)
//
// Reads TOFSense-M 8×8 LiDAR sensor via UART1.
// Supports two frame formats with auto-detection:
//   - Compact format: 257 bytes (1 header + 64×4 pixel data)
//   - Frame0 format:  400 bytes (9-byte header + 64×6 pixel data + 6 reserved + 1 checksum)
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
// TOFSense-M protocol constants
// Two frame formats are supported with auto-detection:
//   - Compact: 257 bytes (1 header + 64×4 pixel data), no checksum
//   - Frame0:  400 bytes (9-byte header + 64×6 pixel data + 6 reserved + 1 checksum)
// Per-pixel layout (both formats):
//   [0-1] pixel_id: uint16 LE
//   [2-3] distance: uint16 LE, mm
//   [4-5] extra/signal: uint16 LE (Frame0 only)
// Pixel validity: 0 < distance < INVALID_DIST_THRESHOLD (65000)
// -------------------------------------------------------------------------
static constexpr uint8_t  FRAME_HEADER      = 0x57;
static constexpr uint8_t  FUNCTION_MARK     = 0x01;   // Frame0 function mark
static constexpr uint16_t FRAME0_LENGTH     = 400;    // Frame0 (legacy) total size
static constexpr uint16_t COMPACT_FRAME_LENGTH = 257; // Compact: 1 + 64×4
static constexpr uint8_t  PIXEL_COUNT_8X8   = 64;     // 8×8 matrix
static constexpr uint8_t  FRAME0_BYTES_PER_PIXEL  = 6;  // Frame0: ID(2) + Dist(2) + Extra(2)
static constexpr uint8_t  COMPACT_BYTES_PER_PIXEL = 4;  // Compact: ID(2) + Dist(2)
static constexpr uint16_t INVALID_DIST_THRESHOLD  = 65000; // Distances >= this are "no return"
// Minimum number of valid pixels required to trust a frame's distance data.
// When the TOFSense-M is in LONG RANGE mode and an object is within the
// sensor's blind zone (< ~50 mm), most pixels report invalid distances
// but a few may report valid distances at 2000–3000 mm due to phase
// wrap-around artifacts.  This produces the "data reversed" symptom:
// very close objects appear far.  Requiring at least MIN_VALID_PIXELS
// valid pixels rejects these artifacts and triggers emergency-close
// (NO_VALID_PIXELS → minRangeMm) instead.
static constexpr uint8_t  MIN_VALID_PIXELS  = 4;
// Maximum allowed distance spread (max − min) among valid pixels in a
// single frame.  When an object is very close (< ~50 mm) or the sensor
// is covered, the TOFSense-M may report MORE than MIN_VALID_PIXELS
// pixels with valid distances, but their values scatter wildly
// (e.g. 121, 273, 1000 mm or 1700, 2500 mm) due to phase wrap-around.
// A real obstacle produces consistent pixel distances (spread < 300 mm
// even for curved surfaces).  Frames exceeding this threshold are
// treated as wrap-around artifacts → emergency-close (minRangeMm).
static constexpr uint16_t MAX_PIXEL_DISPERSION_MM = 500;

// Frame offsets
static constexpr uint16_t OFF_HEADER             = 0;
static constexpr uint16_t OFF_FUNCTION_MARK      = 1;
static constexpr uint16_t FRAME0_OFF_PIXEL_COUNT = 8;
static constexpr uint16_t FRAME0_OFF_PIXEL_DATA  = 9;
static constexpr uint16_t COMPACT_OFF_PIXEL_DATA = 1;  // Right after header

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

// Frame format auto-detection state
enum class FrameMode : uint8_t { AUTO, COMPACT, FRAME0 };
static FrameMode detectedMode_ = FrameMode::AUTO;

// UART receive buffer (sized for the larger frame format)
static uint8_t       rxBuf_[FRAME0_LENGTH];
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
static uint32_t diagAllPixelsInvalid_= 0;  // All 64 pixels invalid (dist==0 or >=65000)
static uint32_t diagHighDispersion_  = 0;  // Valid pixels but spread > MAX_PIXEL_DISPERSION_MM
static uint32_t diagBytesDiscarded_  = 0;  // Bytes skipped waiting for 0x57
static uint16_t diagMaxDistMm_       = 0;  // Maximum distance seen in current diag interval
static bool     diagShortRangeWarned_= false; // Only warn about SHORT RANGE once per session

// -------------------------------------------------------------------------
// Auto-recovery state — retries configureLongRange() when the sensor is
// stuck in SHORT RANGE mode (sustained all-pixels-invalid frames).
// -------------------------------------------------------------------------
// Number of consecutive frames where all pixels were invalid or had high
// dispersion.  Reset to 0 whenever a ParseResult::OK frame arrives.
static uint32_t consecutiveInvalidFrames_ = 0;
// Number of auto-recovery attempts already performed.
static uint8_t  autoRecoveryAttempts_     = 0;
// Maximum number of automatic configureLongRange() retries.
static constexpr uint8_t  MAX_AUTO_RECOVERY_ATTEMPTS    = 10;
// Number of consecutive invalid frames before triggering auto-recovery.
// At 10 Hz frame rate, 30 frames ≈ 3 seconds of sustained failure.
static constexpr uint32_t AUTO_RECOVERY_FRAME_THRESHOLD = 30;

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
    NO_VALID_PIXELS,   // All 64 pixels invalid (dist==0 or >=65000), or fewer than MIN_VALID_PIXELS
    HIGH_DISPERSION    // Valid pixels present but distance spread exceeds MAX_PIXEL_DISPERSION_MM (wrap-around)
};

// -------------------------------------------------------------------------
// Parse a complete TOFSense-M frame and extract minimum distance.
// Supports both Frame0 (400-byte) and Compact (257-byte) formats via
// parameterized pixel stride and offset.
//
// Per-pixel layout (both formats):
//   [0-1]  pixel_id:  uint16 LE
//   [2-3]  distance:  uint16 LE, mm
//   [4-5]  extra/signal: uint16 LE (Frame0 only)
//
// Pixel validity: 0 < distance < INVALID_DIST_THRESHOLD (65000)
// -------------------------------------------------------------------------
static ParseResult parseFrame(const uint8_t* buf, uint16_t len,
                               uint8_t bytesPerPixel, uint16_t pixelDataOffset,
                               bool checkChecksum, uint16_t frameLen,
                               uint16_t& outDistMm) {
    outDistMm = 0;
    if (len < frameLen) return ParseResult::TOO_SHORT;

    // Header validation
    if (buf[OFF_HEADER] != FRAME_HEADER) return ParseResult::BAD_HEADER;

    // For Frame0: validate function mark and checksum
    if (checkChecksum) {
        if (buf[OFF_FUNCTION_MARK] != FUNCTION_MARK) return ParseResult::BAD_HEADER;
        uint8_t checksum = 0;
        for (uint16_t i = 0; i < frameLen - 1; i++) checksum += buf[i];
        if (checksum != buf[frameLen - 1]) return ParseResult::BAD_CHECKSUM;
    }

    // Extract minimum valid distance across all 64 pixels
    uint32_t minDistMm = 0xFFFFFFFF;
    uint32_t maxDistMm = 0;
    uint8_t validCount = 0;

    for (uint8_t px = 0; px < PIXEL_COUNT_8X8; px++) {
        uint16_t base = pixelDataOffset + (uint16_t)(px * bytesPerPixel);
        if (base + bytesPerPixel > frameLen) break;

        // Distance: uint16 LE at bytes 2-3 of each pixel group
        uint16_t dist = (uint16_t)buf[base + 2] | ((uint16_t)buf[base + 3] << 8);

        // Valid if 0 < dist < INVALID_DIST_THRESHOLD
        if (dist == 0 || dist >= INVALID_DIST_THRESHOLD) continue;

        uint32_t distMm = (uint32_t)dist;
        validCount++;
        if (distMm < minDistMm) minDistMm = distMm;
        if (distMm > maxDistMm) maxDistMm = distMm;
    }

    if (validCount == 0) return ParseResult::NO_VALID_PIXELS;

    // When very few pixels are valid while most are invalid, the valid
    // ones are likely phase wrap-around artifacts from an object within
    // the sensor's blind zone (< ~50 mm).  Treat as emergency-close.
    if (validCount < MIN_VALID_PIXELS) return ParseResult::NO_VALID_PIXELS;

    // High-dispersion check: when valid pixels disagree widely (spread
    // > MAX_PIXEL_DISPERSION_MM), the readings are inconsistent and
    // likely caused by phase wrap-around at close range.
    if ((maxDistMm - minDistMm) > MAX_PIXEL_DISPERSION_MM) {
        return ParseResult::HIGH_DISPERSION;
    }

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
    detectedMode_  = FrameMode::AUTO;

    // Reset diagnostic counters
    lastDiagMs_         = millis();
    diagFramesOk_       = 0;
    diagChecksumFail_   = 0;
    diagHeaderFail_     = 0;
    diagAllPixelsInvalid_ = 0;
    diagHighDispersion_   = 0;
    diagBytesDiscarded_ = 0;
    diagMaxDistMm_      = 0;
    diagShortRangeWarned_ = false;
    consecutiveInvalidFrames_ = 0;
    autoRecoveryAttempts_     = 0;

    reading_ = Reading{};  // Reset to defaults

    Serial.printf("[OBSTACLE] TOFSense-M init (UART1, %lu bps, rxBuf %u, rxPin %d, txPin %d, auto-detect Frame0/Compact)\n",
                  (unsigned long)cfg_.baudRate, (unsigned)cfg_.rxBufSize,
                  cfg_.rxPin, cfg_.txPin);
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

        // Sync on header byte
        if (rxIdx_ == 0 && byte != FRAME_HEADER) {
            diagBytesDiscarded_++;
            continue;
        }

        rxBuf_[rxIdx_++] = byte;

        // Determine current frame's target length based on detected mode
        uint16_t targetLen = 0;
        bool isFrame0 = false;

        if (detectedMode_ == FrameMode::COMPACT) {
            targetLen = COMPACT_FRAME_LENGTH;
            isFrame0 = false;
        } else if (detectedMode_ == FrameMode::FRAME0) {
            targetLen = FRAME0_LENGTH;
            isFrame0 = true;
        } else {
            // AUTO mode: decide based on header bytes once we have 9 bytes
            if (rxIdx_ >= 9) {
                if (rxBuf_[1] == FUNCTION_MARK && rxBuf_[2] == 0xFF && rxBuf_[8] == 0x40) {
                    targetLen = FRAME0_LENGTH;
                    isFrame0 = true;
                } else {
                    targetLen = COMPACT_FRAME_LENGTH;
                    isFrame0 = false;
                }
            } else {
                continue;  // Need more bytes to detect format
            }
        }

        // Complete frame received
        if (targetLen > 0 && rxIdx_ >= targetLen) {
            uint16_t dist = 0;
            ParseResult pr;
            if (isFrame0) {
                pr = parseFrame(rxBuf_, rxIdx_,
                               FRAME0_BYTES_PER_PIXEL, FRAME0_OFF_PIXEL_DATA,
                               true, FRAME0_LENGTH, dist);
            } else {
                pr = parseFrame(rxBuf_, rxIdx_,
                               COMPACT_BYTES_PER_PIXEL, COMPACT_OFF_PIXEL_DATA,
                               false, COMPACT_FRAME_LENGTH, dist);
            }

            switch (pr) {
                case ParseResult::OK:
                    measuredMm = dist;
                    gotFrame = true;
                    diagFramesOk_++;
                    consecutiveInvalidFrames_ = 0;
                    if (dist > diagMaxDistMm_) diagMaxDistMm_ = dist;
                    if (detectedMode_ == FrameMode::AUTO) {
                        detectedMode_ = isFrame0 ? FrameMode::FRAME0 : FrameMode::COMPACT;
                    }
                    break;
                case ParseResult::NO_VALID_PIXELS:
                    diagAllPixelsInvalid_++;
                    consecutiveInvalidFrames_++;
                    measuredMm = cfg_.minRangeMm;
                    gotFrame   = true;
                    if (detectedMode_ == FrameMode::AUTO) {
                        detectedMode_ = isFrame0 ? FrameMode::FRAME0 : FrameMode::COMPACT;
                    }
                    break;
                case ParseResult::HIGH_DISPERSION:
                    diagHighDispersion_++;
                    consecutiveInvalidFrames_++;
                    measuredMm = cfg_.minRangeMm;
                    gotFrame   = true;
                    if (detectedMode_ == FrameMode::AUTO) {
                        detectedMode_ = isFrame0 ? FrameMode::FRAME0 : FrameMode::COMPACT;
                    }
                    break;
                case ParseResult::BAD_HEADER:
                    diagHeaderFail_++;
                    break;
                case ParseResult::BAD_CHECKSUM:
                    diagChecksumFail_++;
                    break;
                default:
                    break;
            }
            rxIdx_ = 0;
        }
    }

    // Auto-recovery: if the sensor keeps returning all-invalid frames for
    // an extended period (AUTO_RECOVERY_FRAME_THRESHOLD consecutive frames),
    // it is likely still in SHORT RANGE mode.  Retry configureLongRange()
    // up to MAX_AUTO_RECOVERY_ATTEMPTS times.  This handles the case where
    // the initial configureLongRange() in setup() was sent before the
    // sensor's command processor was ready (sensor still booting).
    if (consecutiveInvalidFrames_ >= AUTO_RECOVERY_FRAME_THRESHOLD
            && autoRecoveryAttempts_ < MAX_AUTO_RECOVERY_ATTEMPTS
            && cfg_.txPin >= 0) {
        autoRecoveryAttempts_++;
        Serial.printf("[OBSTACLE] Auto-recovery attempt %u/%u: "
                      "re-sending configureLongRange() "
                      "(%lu consecutive invalid frames)\n",
                      (unsigned)autoRecoveryAttempts_,
                      (unsigned)MAX_AUTO_RECOVERY_ATTEMPTS,
                      (unsigned long)consecutiveInvalidFrames_);
        configureLongRange();
        // Reset counter to provide a 30-frame (~3 second) backoff before
        // the next retry attempt.  Without this reset, the ≥ 30 condition
        // would trigger on every subsequent update() call.
        consecutiveInvalidFrames_ = 0;
    }

    // Periodic diagnostic output (every DIAG_INTERVAL_MS)
    if (now - lastDiagMs_ >= DIAG_INTERVAL_MS) {
        uint32_t totalFail = diagChecksumFail_ + diagHeaderFail_ + diagAllPixelsInvalid_ + diagHighDispersion_;
        if (diagFramesOk_ > 0 || totalFail > 0 || diagBytesDiscarded_ > 0) {
            Serial.printf("[OBSTACLE] Diag %lus: OK=%lu cksumFail=%lu hdrFail=%lu "
                          "noPixels=%lu highDisp=%lu discarded=%lu\n",
                          (unsigned long)(DIAG_INTERVAL_MS / 1000),
                          (unsigned long)diagFramesOk_,
                          (unsigned long)diagChecksumFail_,
                          (unsigned long)diagHeaderFail_,
                          (unsigned long)diagAllPixelsInvalid_,
                          (unsigned long)diagHighDispersion_,
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
            // is nothing within its ~1.3 m range — every pixel reports
            // an invalid distance.  Warn with actionable instructions.
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
        diagHighDispersion_  = 0;
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

uint8_t getAutoRecoveryAttempts() {
    return autoRecoveryAttempts_;
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
    if (payloadLen > 0 && payload == nullptr) return 0;

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
