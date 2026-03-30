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
#if OBSTACLE_SENSOR_ENABLED
#include <HardwareSerial.h>
#endif

namespace obstacle_sensor {

// =========================================================================
// SENSOR DISABLED MODE (OBSTACLE_SENSOR_ENABLED == 0):
// - Zero CPU usage: update() returns immediately, no function calls
// - Zero UART activity: no HardwareSerial constructed, no begin/read/write
// - Zero buffer allocation: rxBuf_ and diagnostic counters not instantiated
// - Safe default output for CAN/UI: distance=0, healthy=false, status=INVALID
// - CAN 0x208/0x209 frames report health=0, status=INVALID (sensor absent)
// - UI shows "---" (no valid distance)
// - No periodic logs — single startup message only
// =========================================================================

// -------------------------------------------------------------------------
// Module state — always available (both enabled and disabled modes)
// -------------------------------------------------------------------------
static Config  cfg_;
static Reading reading_;
static bool    initialized_ = false;

#if OBSTACLE_SENSOR_ENABLED

#if SENSOR_TYPE == SENSOR_TYPE_TFMINI
// =========================================================================
// TF-Mini Plus protocol constants
//
// Frame layout: 9 bytes
//   [0]   header1    = 0x59
//   [1]   header2    = 0x59
//   [2]   dist_L     distance low byte (cm)
//   [3]   dist_H     distance high byte (cm)
//   [4]   str_L      signal strength low byte
//   [5]   str_H      signal strength high byte
//   [6]   temp_L     temperature low byte (reserved)
//   [7]   temp_H     temperature high byte (reserved)
//   [8]   checksum   sum of bytes [0..7] & 0xFF
//
// Distance: uint16 LE in cm → convert to mm for the Reading struct.
// Strength < 100 or == 65535: reject (unreliable / saturated).
// Distance == 0 or == 65535: reject (out-of-range sentinel).
// =========================================================================
static constexpr uint8_t  TFM_HEADER          = 0x59;
static constexpr uint8_t  TFM_FRAME_LENGTH    = 9;
static constexpr uint16_t TFM_MIN_STRENGTH    = 100;
static constexpr uint16_t TFM_DIST_INVALID    = 0xFFFF;  // 65535
// Non-blocking: max bytes read per update() call.
// At 115200 baud / 100 Hz, ~9 bytes arrive every 10 ms.
// 32 bytes covers ~3 frames worth of data — ample for loop jitter.
static constexpr uint16_t TFM_MAX_BYTES_PER_UPDATE = 32;

#else  // SENSOR_TYPE_TOFSENSE

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

// UART receive buffer — holds a full multi-pixel frame (400 bytes) + headroom
static uint8_t       rxBuf_[MP_FRAME_LENGTH + 16];
static uint16_t      rxIdx_            = 0;

#endif  // SENSOR_TYPE (TOFSense-M constants/buffer)

// -------------------------------------------------------------------------
// Sensor-active state — shared across sensor types
// -------------------------------------------------------------------------
static unsigned long initTimeMs_       = 0;
static unsigned long lastValidMs_      = 0;
static uint16_t      prevDistanceMm_   = 0;
static unsigned long stuckSinceMs_     = 0;
static bool          stuckActive_      = false;
static bool          warmupDone_       = false;

#if SENSOR_TYPE == SENSOR_TYPE_TFMINI
// TF-Mini Plus: 9-byte frame buffer (no large rxBuf_ needed)
static uint8_t       tfmBuf_[TFM_FRAME_LENGTH];
static uint8_t       tfmIdx_           = 0;
#endif

// UART1 for sensor
static HardwareSerial sensorSerial(1);

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
static uint16_t diagUartHWM_         = 0;  // UART available() high-water mark

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

#if SENSOR_TYPE == SENSOR_TYPE_TFMINI
// =========================================================================
// TF-Mini Plus frame parser
//
// Parse a complete 9-byte TF-Mini Plus frame.
// Returns true if valid, with distance in mm written to outDistMm.
//
// Frame: [0x59][0x59][DIST_L][DIST_H][STR_L][STR_H][TEMP_L][TEMP_H][CHK]
// Checksum: sum of bytes [0..7] & 0xFF must equal byte [8]
// Distance: uint16 LE in cm → mm
// Strength: uint16 LE, reject < TFM_MIN_STRENGTH
// Reject distance == 0 or == 65535
// =========================================================================
static bool parseTfMiniFrame(const uint8_t* buf, uint16_t& outDistMm) {
    outDistMm = 0;

    // Header validation
    if (buf[0] != TFM_HEADER || buf[1] != TFM_HEADER) return false;

    // Checksum: sum of bytes [0..7] & 0xFF
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 8; i++) checksum += buf[i];
    if (checksum != buf[8]) return false;

    // Distance in cm (uint16 LE)
    uint16_t distCm = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);

    // Signal strength (uint16 LE)
    uint16_t strength = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

    // Reject invalid readings
    if (distCm == 0)                       return false;  // No target
    if (distCm == TFM_DIST_INVALID)        return false;  // Out-of-range sentinel
    if (strength < TFM_MIN_STRENGTH)       return false;  // Unreliable signal

    // Convert cm → mm
    outDistMm = distCm * 10;
    return true;
}

#else  // SENSOR_TYPE_TOFSENSE

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

#endif  // SENSOR_TYPE (parser selection)

#endif // OBSTACLE_SENSOR_ENABLED — end of sensor-active declarations

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init(const Config& cfg) {
    cfg_ = cfg;

#if OBSTACLE_SENSOR_ENABLED
    // Set UART RX ring-buffer size BEFORE begin().
    sensorSerial.setRxBufferSize(cfg_.rxBufSize);

    // Initialize UART1
    sensorSerial.begin(cfg_.baudRate, SERIAL_8N1, cfg_.rxPin, cfg_.txPin);

    initTimeMs_    = millis();
    lastValidMs_   = 0;
    prevDistanceMm_ = 0;
    stuckSinceMs_  = 0;
    stuckActive_   = false;
    warmupDone_    = false;
    initialized_   = true;

#if SENSOR_TYPE == SENSOR_TYPE_TFMINI
    tfmIdx_        = 0;
#else
    rxIdx_         = 0;
#endif

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
    diagUartHWM_        = 0;

    reading_ = Reading{};  // Reset to defaults

#if SENSOR_TYPE == SENSOR_TYPE_TFMINI
    Serial.printf("[OBSTACLE] TF-Mini Plus init (UART1, %lu bps, rxBuf %u, "
                  "rxPin %d, range %u–%u mm)\n",
                  (unsigned long)cfg_.baudRate, (unsigned)cfg_.rxBufSize,
                  cfg_.rxPin,
                  (unsigned)cfg_.minRangeMm, (unsigned)cfg_.maxRangeMm);
#else
    Serial.printf("[OBSTACLE] TOFSense-M 8x8 init (UART1, %lu bps, rxBuf %u, "
                  "rxPin %d, txPin %d, range %u–%u mm)\n",
                  (unsigned long)cfg_.baudRate, (unsigned)cfg_.rxBufSize,
                  cfg_.rxPin, cfg_.txPin,
                  (unsigned)cfg_.minRangeMm, (unsigned)cfg_.maxRangeMm);
#endif
#else
    // Sensor hardware removed — no UART init, no reads, no parsing.
    // getReading() returns safe defaults (distance=0, healthy=false, INVALID).
    initialized_ = false;
    reading_ = Reading{};
    reading_.status  = SensorStatus::INVALID;
    reading_.healthy = false;
    Serial.println("[OBSTACLE] Sensor DISABLED (OBSTACLE_SENSOR_ENABLED=0). "
                   "No UART configured. Returning safe defaults.");
#endif
}

void update(float vehicleSpeedKmh) {
#if !OBSTACLE_SENSOR_ENABLED
    // Sensor disabled — no UART reads, no parsing, no blocking.
    // reading_ keeps its safe defaults set in init().
    (void)vehicleSpeedKmh;
    return;
#else
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
        while (sensorSerial.available() > 0) {
            sensorSerial.read();
        }
#if SENSOR_TYPE == SENSOR_TYPE_TFMINI
        tfmIdx_ = 0;
#else
        rxIdx_ = 0;
#endif
        warmupDone_ = true;
    }

    // Read available UART bytes and parse frames.
    uint16_t measuredMm = 0;
    bool gotFrame = false;

#if SENSOR_TYPE == SENSOR_TYPE_TFMINI
    // ---- TF-Mini Plus: 9-byte frames, non-blocking ----
    // Read max TFM_MAX_BYTES_PER_UPDATE bytes per call (32).
    // Parse at most 1 valid frame per call — exit immediately after.
    uint16_t bytesProcessed = 0;

    while (sensorSerial.available() > 0 &&
           bytesProcessed < TFM_MAX_BYTES_PER_UPDATE) {
        uint8_t b = (uint8_t)sensorSerial.read();
        ++bytesProcessed;

        // Sync: first byte must be 0x59
        if (tfmIdx_ == 0 && b != TFM_HEADER) {
            diagBytesDiscarded_++;
            continue;
        }
        // Second byte must also be 0x59
        if (tfmIdx_ == 1 && b != TFM_HEADER) {
            diagHeaderFail_++;
            tfmIdx_ = 0;
            // Re-check: if this byte is 0x59, it could be the start
            if (b == TFM_HEADER) {
                tfmBuf_[tfmIdx_++] = b;
            }
            continue;
        }

        tfmBuf_[tfmIdx_++] = b;

        if (tfmIdx_ < TFM_FRAME_LENGTH) continue;

        // Complete 9-byte frame — attempt parse
        uint16_t dist = 0;
        if (parseTfMiniFrame(tfmBuf_, dist)) {
            measuredMm = dist;
            gotFrame = true;
            diagFramesOk_++;
            if (dist > diagMaxDistMm_) diagMaxDistMm_ = dist;
        } else {
            // Distinguish checksum vs data rejection for diagnostics
            uint8_t cksum = 0;
            for (uint8_t i = 0; i < 8; i++) cksum += tfmBuf_[i];
            if (cksum != tfmBuf_[8]) {
                diagChecksumFail_++;
            } else {
                diagNoTarget_++;  // Valid checksum but rejected data
            }
        }

        tfmIdx_ = 0;
        // Parse max 1 frame per update() call for deterministic timing
        if (gotFrame) break;
    }

#else  // SENSOR_TYPE_TOFSENSE
    PixelStats lastStats{};
    // Cap: 4 frames worth of bytes — drains accumulated data after loop
    // jitter while still bounding CPU time per call.
    static constexpr uint16_t MAX_BYTES_PER_UPDATE = MP_FRAME_LENGTH * 4;
    uint16_t bytesProcessed = 0;

    // Track UART buffer high-water mark for overflow diagnostics
    {
        int avail = sensorSerial.available();
        if (avail > 0 && static_cast<uint16_t>(avail) > diagUartHWM_) {
            diagUartHWM_ = static_cast<uint16_t>(avail);
        }
    }

    while (sensorSerial.available() > 0 && bytesProcessed < MAX_BYTES_PER_UPDATE) {
        uint8_t byte = (uint8_t)sensorSerial.read();
        ++bytesProcessed;

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
        // On BAD_CHECKSUM or BAD_HEADER, scan for next 0x57+0x01 pair.
        // Checking both header AND function mark prevents false resync
        // on 0x57 bytes that appear naturally in pixel distance data,
        // avoiding cascading checksum failures.
        if (pr == ParseResult::BAD_CHECKSUM || pr == ParseResult::BAD_HEADER) {
            uint16_t resyncIdx = 0;
            // Scan for 0x57+0x01 pair (header + function mark)
            for (uint16_t scan = 1; scan + 1 < rxIdx_; scan++) {
                if (rxBuf_[scan] == FRAME_HEADER &&
                    rxBuf_[scan + 1] == FUNCTION_MARK_MP) {
                    resyncIdx = rxIdx_ - scan;
                    for (uint16_t j = 0; j < resyncIdx; j++) {
                        rxBuf_[j] = rxBuf_[scan + j];
                    }
                    break;
                }
            }
            // If no pair found, check if the very last byte is a header
            // (function mark not yet received — keep it for next read)
            if (resyncIdx == 0 && rxIdx_ > 0 &&
                rxBuf_[rxIdx_ - 1] == FRAME_HEADER) {
                rxBuf_[0] = FRAME_HEADER;
                resyncIdx = 1;
            }
            rxIdx_ = resyncIdx;
        } else {
            rxIdx_ = 0;
        }
    }
#endif  // SENSOR_TYPE (update loop)

    // Periodic diagnostic output
    if (now - lastDiagMs_ >= DIAG_INTERVAL_MS) {
        uint32_t total = diagFramesOk_ + diagChecksumFail_ + diagHeaderFail_
                       + diagNoTarget_ + diagTooFewPixels_ + diagHighDispersion_;
        if (total > 0 || diagBytesDiscarded_ > 0) {
            Serial.printf("[OBSTACLE] Diag %lus: OK=%lu cksumFail=%lu hdrFail=%lu "
                          "noTarget=%lu fewPx=%lu highDisp=%lu discarded=%lu "
                          "maxDist=%u uartHWM=%u\n",
                          (unsigned long)(DIAG_INTERVAL_MS / 1000),
                          (unsigned long)diagFramesOk_,
                          (unsigned long)diagChecksumFail_,
                          (unsigned long)diagHeaderFail_,
                          (unsigned long)diagNoTarget_,
                          (unsigned long)diagTooFewPixels_,
                          (unsigned long)diagHighDispersion_,
                          (unsigned long)diagBytesDiscarded_,
                          (unsigned)diagMaxDistMm_,
                          (unsigned)diagUartHWM_);
            if (diagChecksumFail_ > diagFramesOk_ && diagChecksumFail_ > 0) {
                Serial.println("[OBSTACLE] WARNING: Most frames fail checksum. "
                               "Check voltage divider and cable length.");
            }
        } else if (reading_.status != SensorStatus::WAITING) {
#if SENSOR_TYPE == SENSOR_TYPE_TFMINI
            Serial.println("[OBSTACLE] Diag: no UART data — check wiring "
                           "and baud rate (expected 115200 bps)");
#else
            Serial.println("[OBSTACLE] Diag: no UART data — check wiring "
                           "and baud rate (expected 921600 bps)");
#endif
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
        diagUartHWM_         = 0;
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
#if SENSOR_TYPE == SENSOR_TYPE_TFMINI
    // TF-Mini Plus: single-point sensor — all pixel statistics report the
    // same single distance value.
    reading_.minDist_mm    = measuredMm;
    reading_.maxDist_mm    = measuredMm;
    reading_.avgDist_mm    = measuredMm;
    reading_.validCount    = 1;
    reading_.dispersion_mm = 0;
#else
    reading_.minDist_mm   = lastStats.minDist_mm;
    reading_.maxDist_mm   = lastStats.maxDist_mm;
    reading_.avgDist_mm   = lastStats.avgDist_mm;
    reading_.validCount   = lastStats.validCount;
    reading_.dispersion_mm= lastStats.dispersion_mm;
#endif
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
#endif  // OBSTACLE_SENSOR_ENABLED
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

#if OBSTACLE_SENSOR_ENABLED

static bool sendCmd(const uint8_t* frame, uint16_t len) {
    if (!initialized_) return false;
    if (cfg_.txPin < 0) return false;
    sensorSerial.write(frame, len);
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

#else // !OBSTACLE_SENSOR_ENABLED — sensor commands unavailable

// cppcheck-suppress unusedFunction
bool setRangeMode(RangeMode)    { return false; }
// cppcheck-suppress unusedFunction
bool setBaudRate(BaudRateCode)  { return false; }
// cppcheck-suppress unusedFunction
bool setFrameRate(uint8_t)      { return false; }
// cppcheck-suppress unusedFunction
bool setOutputMode(OutputMode)  { return false; }
// cppcheck-suppress unusedFunction
bool saveConfig()               { return false; }
// cppcheck-suppress unusedFunction
bool configureLongRange()       { return false; }

#endif // OBSTACLE_SENSOR_ENABLED

} // namespace obstacle_sensor
