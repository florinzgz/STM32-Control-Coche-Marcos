// =============================================================================
// ESP32-S3 — Obstacle Sensor Driver (implementation)
//
// Reads TOFSense-M 8×8 LiDAR sensor via UART1.
// ONLY the multi-pixel frame format is supported:
//   [0x57][0x01][0xFF][ID][TIME×4][0x40][64×6B pixels][0xFF×6][CHK]
//
// Validates readings against physical range, detects stuck sensor,
// and provides a warmup period for sensor stabilization.
// Computes per-frame pixel statistics (min/max/avg/validCount/dispersion).
// Rejects frames with too few valid pixels or excessive dispersion.
//
// Sensor: TOFSense-M 8×8 (Nooploop), UART 921600 bps, active output mode.
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
// TOFSense-M 8×8 protocol constants
//
// Frame layout (function_mark = 0x01): 9 + 64*6 + 6 + 1 = 400 bytes
//   [0]     header         = 0x57
//   [1]     function_mark  = 0x01
//   [2]     reserved       = 0xFF
//   [3]     sensor_id
//   [4-7]   system_time    (uint32 LE, ms)
//   [8]     num_pixels     (0x40 = 64 for 8×8)
//   [9..]   pixel data     64 × 6 bytes:
//             [0-2] distance  (uint24 LE, micrometers)
//             [3]   status    (0x00 = valid)
//             [4-5] signal    (uint16 LE)
//   [393-398] end_marker   6 × 0xFF
//   [399]   checksum       (sum of bytes [0..398] & 0xFF)
// -------------------------------------------------------------------------
static constexpr uint8_t  FRAME_HEADER         = 0x57;
static constexpr uint8_t  FUNCTION_MARK_MP     = 0x01;   // Multi-pixel function mark

// Multi-pixel frame layout
static constexpr uint8_t  MP_NUM_PIXELS        = 64;     // 8×8 matrix
static constexpr uint8_t  MP_BYTES_PER_PIXEL   = 6;      // 3 dist + 1 status + 2 signal
static constexpr uint8_t  MP_END_MARKER_LEN    = 6;      // 6 × 0xFF
static constexpr uint16_t MP_HEADER_LEN        = 9;      // header(1)+mark(1)+reserved(1)+id(1)+time(4)+numpix(1)
static constexpr uint16_t MP_FRAME_LENGTH      = MP_HEADER_LEN
                                                + MP_NUM_PIXELS * MP_BYTES_PER_PIXEL
                                                + MP_END_MARKER_LEN
                                                + 1;      // checksum (= 400)
static constexpr uint8_t  MP_PIXEL_STATUS_VALID = 0x00;  // Status byte: 0 = valid pixel

// Frame byte offset for function mark
static constexpr uint16_t OFF_FUNCTION_MARK = 1;

// Pixel validation thresholds
static constexpr uint8_t  MIN_VALID_PIXELS           = 4;     // Minimum valid pixels for a usable frame
static constexpr uint16_t MAX_PIXEL_DISPERSION_MM    = 3000;  // Max spread (maxDist-minDist) in mm

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

// UART receive buffer — holds a full multi-pixel frame (400 bytes) + headroom
static uint8_t       rxBuf_[MP_FRAME_LENGTH + 16];
static uint16_t      rxIdx_            = 0;

// UART1 for TOFSense-M
static HardwareSerial tofSerial(1);

// -------------------------------------------------------------------------
// Diagnostic counters — logged every DIAG_INTERVAL_MS
// -------------------------------------------------------------------------
static constexpr unsigned long DIAG_INTERVAL_MS = 5000;
static unsigned long lastDiagMs_      = 0;
static uint32_t diagFramesOk_        = 0;
static uint32_t diagChecksumFail_    = 0;
static uint32_t diagHeaderFail_      = 0;
static uint32_t diagNoTarget_        = 0;
static uint32_t diagTooFewPixels_    = 0;
static uint32_t diagHighDispersion_  = 0;
static uint32_t diagBytesDiscarded_  = 0;
static uint16_t diagMaxDistMm_       = 0;

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
    OK,                // Successful parse — valid frame with enough pixels
    TOO_SHORT,         // Buffer too short
    BAD_HEADER,        // Wrong header or function_mark or pixel count
    BAD_CHECKSUM,      // 8-bit sum mismatch
    NO_TARGET,         // All pixels invalid or zero distance
    TOO_FEW_PIXELS,    // Valid pixels < MIN_VALID_PIXELS
    HIGH_DISPERSION    // Pixel spread > MAX_PIXEL_DISPERSION_MM
};

// -------------------------------------------------------------------------
// Pixel statistics computed during frame parsing
// -------------------------------------------------------------------------
struct PixelStats {
    uint16_t minDist_mm   = 0;
    uint16_t maxDist_mm   = 0;
    uint16_t avgDist_mm   = 0;
    uint8_t  validCount   = 0;
    uint16_t dispersion_mm= 0;
};

// -------------------------------------------------------------------------
// Parse a complete TOFSense-M 8×8 multi-pixel frame.
//
// Returns minimum valid pixel distance in mm (closest obstacle).
// Also fills PixelStats with per-frame statistics.
// -------------------------------------------------------------------------
static ParseResult parseMultiPixelFrame(const uint8_t* buf, uint16_t len,
                                         uint16_t& outDistMm,
                                         PixelStats& stats) {
    outDistMm = 0;
    stats = PixelStats{};

    if (len < MP_FRAME_LENGTH) return ParseResult::TOO_SHORT;

    // Header validation
    if (buf[0] != FRAME_HEADER) return ParseResult::BAD_HEADER;
    if (buf[1] != FUNCTION_MARK_MP) return ParseResult::BAD_HEADER;

    // Pixel count must be 64
    if (buf[8] != MP_NUM_PIXELS) return ParseResult::BAD_HEADER;

    // Checksum: sum of all bytes except the last, & 0xFF
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < MP_FRAME_LENGTH - 1; i++) checksum += buf[i];
    if (checksum != buf[MP_FRAME_LENGTH - 1]) return ParseResult::BAD_CHECKSUM;

    // Iterate 64 pixels — compute statistics
    uint32_t minDistUm = UINT32_MAX;
    uint32_t maxDistUm = 0;
    uint64_t sumDistUm = 0;
    uint8_t  validPixels = 0;

    for (uint8_t px = 0; px < MP_NUM_PIXELS; px++) {
        uint16_t off = MP_HEADER_LEN + px * MP_BYTES_PER_PIXEL;
        uint8_t status = buf[off + 3];
        if (status != MP_PIXEL_STATUS_VALID) continue;

        uint32_t distUm = (uint32_t)buf[off]
                        | ((uint32_t)buf[off + 1] << 8)
                        | ((uint32_t)buf[off + 2] << 16);
        if (distUm == 0) continue;  // Skip zero-distance pixels (no target)

        validPixels++;
        sumDistUm += distUm;
        if (distUm < minDistUm) minDistUm = distUm;
        if (distUm > maxDistUm) maxDistUm = distUm;
    }

    if (validPixels == 0) return ParseResult::NO_TARGET;

    // Convert µm → mm
    uint32_t minMm = minDistUm / 1000;
    uint32_t maxMm = maxDistUm / 1000;
    uint32_t avgMm = (uint32_t)(sumDistUm / validPixels / 1000);

    // Avoid reporting 0 mm (= no-target sentinel)
    if (minMm == 0) minMm = 1;

    // Fill stats
    stats.validCount    = validPixels;
    stats.minDist_mm    = (minMm > UINT16_MAX) ? UINT16_MAX : (uint16_t)minMm;
    stats.maxDist_mm    = (maxMm > UINT16_MAX) ? UINT16_MAX : (uint16_t)maxMm;
    stats.avgDist_mm    = (avgMm > UINT16_MAX) ? UINT16_MAX : (uint16_t)avgMm;
    stats.dispersion_mm = (stats.maxDist_mm >= stats.minDist_mm)
                        ? (stats.maxDist_mm - stats.minDist_mm)
                        : 0;

    // Quality gates
    if (validPixels < MIN_VALID_PIXELS) return ParseResult::TOO_FEW_PIXELS;
    if (stats.dispersion_mm > MAX_PIXEL_DISPERSION_MM) return ParseResult::HIGH_DISPERSION;

    outDistMm = stats.minDist_mm;
    return ParseResult::OK;
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init(const Config& cfg) {
    cfg_ = cfg;

    // Set UART RX ring-buffer size BEFORE begin().
    tofSerial.setRxBufferSize(cfg_.rxBufSize);

    // Initialize UART1 for TOFSense-M 8×8
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
    diagNoTarget_       = 0;
    diagTooFewPixels_   = 0;
    diagHighDispersion_ = 0;
    diagBytesDiscarded_ = 0;
    diagMaxDistMm_      = 0;

    reading_ = Reading{};  // Reset to defaults

    Serial.printf("[OBSTACLE] TOFSense-M 8x8 init (UART1, %lu bps, rxBuf %u, "
                  "rxPin %d, txPin %d, range %u–%u mm)\n",
                  (unsigned long)cfg_.baudRate, (unsigned)cfg_.rxBufSize,
                  cfg_.rxPin, cfg_.txPin,
                  (unsigned)cfg_.minRangeMm, (unsigned)cfg_.maxRangeMm);
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

    // First call after warmup: flush stale UART data
    if (!warmupDone_) {
        while (tofSerial.available() > 0) {
            tofSerial.read();
        }
        rxIdx_ = 0;
        warmupDone_ = true;
    }

    // Read all available UART bytes and parse frames.
    // Process all queued frames and keep only the last valid result.
    uint16_t measuredMm = 0;
    bool gotFrame = false;
    PixelStats lastStats{};

    while (tofSerial.available() > 0) {
        uint8_t byte = (uint8_t)tofSerial.read();

        // Sync on header byte
        if (rxIdx_ == 0 && byte != FRAME_HEADER) {
            diagBytesDiscarded_++;
            continue;
        }

        // Bounds check: prevent writing past rxBuf_.
        if (rxIdx_ >= sizeof(rxBuf_)) {
            rxIdx_ = 0;
            diagBytesDiscarded_++;
            if (byte == FRAME_HEADER) {
                rxBuf_[rxIdx_++] = byte;
            }
            continue;
        }

        rxBuf_[rxIdx_++] = byte;

        // After receiving function mark (byte[1]), verify it's 0x01
        if (rxIdx_ == 2 && rxBuf_[OFF_FUNCTION_MARK] != FUNCTION_MARK_MP) {
            // Not a multi-pixel frame — reject and resync
            diagHeaderFail_++;
            rxIdx_ = 0;
            // If this byte itself is a header, start over
            if (byte == FRAME_HEADER) {
                rxBuf_[rxIdx_++] = byte;
            }
            continue;
        }

        // Not enough bytes yet — keep accumulating
        if (rxIdx_ < MP_FRAME_LENGTH) continue;

        // Complete frame received — attempt parse
        uint16_t dist = 0;
        PixelStats stats{};
        ParseResult pr = parseMultiPixelFrame(rxBuf_, rxIdx_, dist, stats);

        switch (pr) {
            case ParseResult::OK:
                measuredMm = dist;
                gotFrame = true;
                lastStats = stats;
                diagFramesOk_++;
                if (dist > diagMaxDistMm_) diagMaxDistMm_ = dist;
                break;
            case ParseResult::NO_TARGET:
                diagNoTarget_++;
                // No valid pixels → emergency close fallback
                measuredMm = cfg_.minRangeMm;
                gotFrame = true;
                lastStats = stats;
                break;
            case ParseResult::TOO_FEW_PIXELS:
                diagTooFewPixels_++;
                // Insufficient data quality → emergency close fallback
                measuredMm = cfg_.minRangeMm;
                gotFrame = true;
                lastStats = stats;
                break;
            case ParseResult::HIGH_DISPERSION:
                diagHighDispersion_++;
                // Noisy/unreliable data — use the min distance as a
                // conservative safety measure
                measuredMm = stats.minDist_mm;
                gotFrame = true;
                lastStats = stats;
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

        // Frame consumed (or rejected). Reset for next frame.
        // On BAD_CHECKSUM or BAD_HEADER, scan for next 0x57.
        if (pr == ParseResult::BAD_CHECKSUM || pr == ParseResult::BAD_HEADER) {
            uint16_t resyncIdx = 0;
            for (uint16_t scan = 1; scan < rxIdx_; scan++) {
                if (rxBuf_[scan] == FRAME_HEADER) {
                    resyncIdx = rxIdx_ - scan;
                    for (uint16_t j = 0; j < resyncIdx; j++) {
                        rxBuf_[j] = rxBuf_[scan + j];
                    }
                    break;
                }
            }
            rxIdx_ = resyncIdx;
        } else {
            rxIdx_ = 0;
        }
    }

    // Periodic diagnostic output
    if (now - lastDiagMs_ >= DIAG_INTERVAL_MS) {
        uint32_t total = diagFramesOk_ + diagChecksumFail_ + diagHeaderFail_
                       + diagNoTarget_ + diagTooFewPixels_ + diagHighDispersion_;
        if (total > 0 || diagBytesDiscarded_ > 0) {
            Serial.printf("[OBSTACLE] Diag %lus: OK=%lu cksumFail=%lu hdrFail=%lu "
                          "noTarget=%lu fewPx=%lu highDisp=%lu discarded=%lu maxDist=%u\n",
                          (unsigned long)(DIAG_INTERVAL_MS / 1000),
                          (unsigned long)diagFramesOk_,
                          (unsigned long)diagChecksumFail_,
                          (unsigned long)diagHeaderFail_,
                          (unsigned long)diagNoTarget_,
                          (unsigned long)diagTooFewPixels_,
                          (unsigned long)diagHighDispersion_,
                          (unsigned long)diagBytesDiscarded_,
                          (unsigned)diagMaxDistMm_);
            if (diagChecksumFail_ > diagFramesOk_ && diagChecksumFail_ > 0) {
                Serial.println("[OBSTACLE] WARNING: Most frames fail checksum. "
                               "Check voltage divider and cable length.");
            }
        } else if (reading_.status != SensorStatus::WAITING) {
            Serial.println("[OBSTACLE] Diag: no UART data — check wiring "
                           "and baud rate (expected 921600 bps)");
        }
        // Reset counters
        diagFramesOk_        = 0;
        diagChecksumFail_    = 0;
        diagHeaderFail_      = 0;
        diagNoTarget_        = 0;
        diagTooFewPixels_    = 0;
        diagHighDispersion_  = 0;
        diagBytesDiscarded_  = 0;
        diagMaxDistMm_       = 0;
        lastDiagMs_          = now;
    }

    if (!gotFrame) {
        // No frame received — check timeout
        unsigned long refMs = lastValidMs_ > 0
                            ? lastValidMs_
                            : (initTimeMs_ + cfg_.warmupMs);
        if ((now - refMs) > cfg_.frameTimeoutMs) {
            reading_.status  = SensorStatus::INVALID;
            reading_.healthy = false;
        }
        return;
    }

    // Range validation
    if (measuredMm > cfg_.maxRangeMm) {
        measuredMm = cfg_.maxRangeMm;
    }
    if (measuredMm < cfg_.minRangeMm) {
        measuredMm = cfg_.minRangeMm;
    }

    // Valid reading — update state
    reading_.distance_mm  = measuredMm;
    reading_.zone         = distanceToZone(measuredMm);
    reading_.minDist_mm   = lastStats.minDist_mm;
    reading_.maxDist_mm   = lastStats.maxDist_mm;
    reading_.avgDist_mm   = lastStats.avgDist_mm;
    reading_.validCount   = lastStats.validCount;
    reading_.dispersion_mm= lastStats.dispersion_mm;
    lastValidMs_          = now;

    // Stuck-sensor detection
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

    reading_.updateCount++;
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
    if (payloadLen > 0 && payload == nullptr) return 0;
    uint16_t totalLen = (uint16_t)(3 + payloadLen + 1);
    if (totalLen > outBufSize) return 0;

    outBuf[0] = CMD_HEADER;
    outBuf[1] = (uint8_t)totalLen;
    outBuf[2] = cmdId;
    for (uint8_t i = 0; i < payloadLen; i++) {
        outBuf[3 + i] = payload[i];
    }
    uint8_t sum = 0;
    for (uint16_t i = 0; i < totalLen - 1; i++) {
        sum += outBuf[i];
    }
    outBuf[totalLen - 1] = sum;
    return totalLen;
}

static bool sendCmd(const uint8_t* frame, uint16_t len) {
    if (!initialized_) return false;
    if (cfg_.txPin < 0) return false;
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
    if (hz == 0) return false;
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
