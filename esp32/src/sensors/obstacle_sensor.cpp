// =============================================================================
// ESP32-S3 — Obstacle Sensor Driver (implementation)
//
// Reads TOFSense-M LiDAR sensor via UART1.
// Supports two frame formats:
//   Single-point (7 bytes):  [0x57][0x00][DIST_L][DIST_H][SIG_L][SIG_H][CHK]
//   Multi-pixel  (400 bytes): [0x57][0x01][0xFF][ID][TIME×4][0x40][64×6B pixels][0xFF×6][CHK]
// Validates readings against physical range, detects stuck sensor,
// and provides a warmup period for sensor stabilization.
//
// Sensor: TOFSense-M (Nooploop), UART 921600 bps, active output mode.
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
//
// Single-point frame (function_mark = 0x00): 7 bytes
//   [0]   header         = 0x57
//   [1]   function_mark  = 0x00
//   [2]   distance_L     (uint16 LE low byte, mm)
//   [3]   distance_H     (uint16 LE high byte, mm)
//   [4]   signal_L       (uint16 LE low byte)
//   [5]   signal_H       (uint16 LE high byte)
//   [6]   checksum       (sum of bytes [0..5] & 0xFF)
//
// Multi-pixel frame (function_mark = 0x01): 9 + N*6 + 7 bytes
//   [0]     header         = 0x57
//   [1]     function_mark  = 0x01
//   [2]     reserved       = 0xFF
//   [3]     sensor_id
//   [4-7]   system_time    (uint32 LE, ms)
//   [8]     num_pixels     (0x40 = 64 for 8×8)
//   [9..]   pixel data     N × 6 bytes:
//             [0-2] distance  (uint24 LE, micrometers)
//             [3]   status    (0x00 = valid)
//             [4-5] signal    (uint16 LE)
//   [..]    end_marker     6 × 0xFF
//   [last]  checksum       (sum of all preceding bytes & 0xFF)
//
// Distance validity (single-point): distance > 0 means valid.
//                                   distance == 0 means no target.
// Distance validity (multi-pixel):  status == 0 means valid pixel.
// -------------------------------------------------------------------------
static constexpr uint8_t  FRAME_HEADER         = 0x57;
static constexpr uint8_t  FUNCTION_MARK_MS     = 0x00;   // Single-point function mark
static constexpr uint8_t  FUNCTION_MARK_MP     = 0x01;   // Multi-pixel function mark
static constexpr uint16_t MS_FRAME_LENGTH      = 7;      // Single-point frame size

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

// Frame byte offsets
static constexpr uint16_t OFF_HEADER           = 0;
static constexpr uint16_t OFF_FUNCTION_MARK    = 1;
static constexpr uint16_t OFF_DISTANCE         = 2;   // uint16 LE (2 bytes)
static constexpr uint16_t OFF_SIGNAL           = 4;   // uint16 LE (2 bytes)
static constexpr uint16_t OFF_CHECKSUM         = 6;

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

// UART receive buffer — accumulates bytes for the current frame being
// assembled.  Needs to hold a full multi-pixel frame (400 bytes) plus
// some headroom for resync.
static uint8_t       rxBuf_[MP_FRAME_LENGTH + 16];
static uint16_t      rxIdx_            = 0;

// UART1 for TOFSense-M S
static HardwareSerial tofSerial(1);

// -------------------------------------------------------------------------
// Diagnostic counters — logged every DIAG_INTERVAL_MS to Serial for
// debugging wiring / signal-level problems.
// -------------------------------------------------------------------------
static constexpr unsigned long DIAG_INTERVAL_MS = 5000;
static constexpr uint32_t DIAG_MIN_FRAMES_FOR_RANGE_CHECK      = 10;
static constexpr uint16_t SHORT_RANGE_DETECTION_THRESHOLD_MM   = 1500;
static unsigned long lastDiagMs_      = 0;
static uint32_t diagFramesOk_        = 0;  // Frames parsed successfully
static uint32_t diagChecksumFail_    = 0;  // Checksum mismatches
static uint32_t diagHeaderFail_      = 0;  // Wrong header or function mark
static uint32_t diagNoTarget_        = 0;  // Distance == 0 (no target in range)
static uint32_t diagBytesDiscarded_  = 0;  // Bytes skipped waiting for 0x57
static uint16_t diagMaxDistMm_       = 0;  // Maximum distance seen in current diag interval
static bool     diagShortRangeWarned_= false;

// -------------------------------------------------------------------------
// Auto-recovery state — retries configureLongRange() when the sensor is
// stuck in SHORT RANGE mode (sustained no-target frames).
// -------------------------------------------------------------------------
static uint32_t consecutiveInvalidFrames_ = 0;
static uint8_t  autoRecoveryAttempts_     = 0;
static constexpr uint8_t  MAX_AUTO_RECOVERY_ATTEMPTS    = 10;
static constexpr uint32_t AUTO_RECOVERY_FRAME_THRESHOLD = 30;
static constexpr uint32_t SENSOR_FAULT_FRAME_THRESHOLD  = 60;
static bool     sensorFaultActive_        = false;

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
    OK,                // Successful parse — valid distance > 0
    TOO_SHORT,         // Buffer too short
    BAD_HEADER,        // Wrong header or function_mark
    BAD_CHECKSUM,      // 8-bit sum mismatch
    NO_TARGET          // Distance == 0 — no target detected
};

// -------------------------------------------------------------------------
// Parse a complete TOFSense-M S single-point frame.
//
// Frame layout (7 bytes):
//   [0] header       = 0x57
//   [1] func_mark    = 0x00
//   [2] distance_L   uint8
//   [3] distance_H   uint8
//   [4] signal_L     uint8
//   [5] signal_H     uint8
//   [6] checksum     sum([0..5]) & 0xFF
// -------------------------------------------------------------------------
static ParseResult parseFrame(const uint8_t* buf, uint16_t len,
                               uint16_t& outDistMm) {
    outDistMm = 0;
    if (len < MS_FRAME_LENGTH) return ParseResult::TOO_SHORT;

    // Header validation
    if (buf[OFF_HEADER] != FRAME_HEADER) return ParseResult::BAD_HEADER;
    if (buf[OFF_FUNCTION_MARK] != FUNCTION_MARK_MS) return ParseResult::BAD_HEADER;

    // Checksum: sum of bytes [0..5] must equal byte [6]
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < MS_FRAME_LENGTH - 1; i++) checksum += buf[i];
    if (checksum != buf[OFF_CHECKSUM]) return ParseResult::BAD_CHECKSUM;

    // Distance: uint16 LE at bytes [2-3]
    uint16_t dist = (uint16_t)buf[OFF_DISTANCE] | ((uint16_t)buf[OFF_DISTANCE + 1] << 8);

    // distance == 0 means no target detected
    if (dist == 0) return ParseResult::NO_TARGET;

    outDistMm = dist;
    return ParseResult::OK;
}

// -------------------------------------------------------------------------
// Parse a complete TOFSense-M multi-pixel frame (8×8 matrix).
//
// Frame layout (400 bytes for 64 pixels):
//   [0]      header        = 0x57
//   [1]      func_mark     = 0x01
//   [2]      reserved      = 0xFF
//   [3]      sensor_id
//   [4-7]    system_time   uint32 LE (ms)
//   [8]      num_pixels    (0x40 = 64)
//   [9..]    pixel data    64 × 6 bytes:
//              [0-2] distance  uint24 LE (micrometers)
//              [3]   status    0x00 = valid
//              [4-5] signal    uint16 LE
//   [393-398] end_marker   6 × 0xFF
//   [399]    checksum      sum of bytes [0..398] & 0xFF
//
// Returns minimum valid pixel distance in mm (closest obstacle).
// -------------------------------------------------------------------------
static ParseResult parseMultiPixelFrame(const uint8_t* buf, uint16_t len,
                                         uint16_t& outDistMm) {
    outDistMm = 0;
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

    // Extract minimum valid-pixel distance (closest obstacle in the 8×8 FOV)
    uint32_t minDistUm = UINT32_MAX;
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
        if (distUm < minDistUm) minDistUm = distUm;
    }

    if (validPixels == 0) return ParseResult::NO_TARGET;

    // Convert µm → mm (integer division, truncate)
    uint32_t distMm = minDistUm / 1000;
    if (distMm > UINT16_MAX) distMm = UINT16_MAX;
    outDistMm = (uint16_t)distMm;
    if (outDistMm == 0) outDistMm = 1;  // Avoid reporting 0 (= no-target)

    return ParseResult::OK;
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void init(const Config& cfg) {
    cfg_ = cfg;

    // Set UART RX ring-buffer size BEFORE begin().
    tofSerial.setRxBufferSize(cfg_.rxBufSize);

    // Initialize UART1 for TOFSense-M S
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
    diagBytesDiscarded_ = 0;
    diagMaxDistMm_      = 0;
    diagShortRangeWarned_ = false;
    consecutiveInvalidFrames_ = 0;
    autoRecoveryAttempts_     = 0;
    sensorFaultActive_        = false;

    reading_ = Reading{};  // Reset to defaults

    Serial.printf("[OBSTACLE] TOFSense-M init (UART1, %lu bps, rxBuf %u, rxPin %d, txPin %d, single-point + multi-pixel%s)\n",
                  (unsigned long)cfg_.baudRate, (unsigned)cfg_.rxBufSize,
                  cfg_.rxPin, cfg_.txPin,
                  cfg_.shortRangeMode ? ", SHORT RANGE read-only" : "");
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

        // Determine expected frame length once we have the function mark (byte [1])
        uint16_t expectedLen = 0;
        if (rxIdx_ >= 2) {
            if (rxBuf_[1] == FUNCTION_MARK_MS) {
                expectedLen = MS_FRAME_LENGTH;       // 7 bytes (single-point)
            } else if (rxBuf_[1] == FUNCTION_MARK_MP) {
                expectedLen = MP_FRAME_LENGTH;       // 400 bytes (multi-pixel)
            }
            // Unknown function mark: will be handled below when we have enough bytes
        }

        // Not enough bytes yet for the detected frame type — keep accumulating
        if (expectedLen > 0 && rxIdx_ < expectedLen) continue;

        // If function mark is unknown and we have at least 7 bytes, treat as error
        if (expectedLen == 0 && rxIdx_ >= MS_FRAME_LENGTH) {
            // Unknown function mark — reject
            diagHeaderFail_++;
            // Resync: scan for next 0x57
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
            continue;
        }

        // Still accumulating and don't know length yet (only 1 byte = header)
        if (expectedLen == 0) continue;

        // Complete frame received — attempt parse
        uint16_t dist = 0;
        ParseResult pr;

        if (rxBuf_[1] == FUNCTION_MARK_MP) {
            pr = parseMultiPixelFrame(rxBuf_, rxIdx_, dist);
        } else {
            pr = parseFrame(rxBuf_, rxIdx_, dist);
        }

        switch (pr) {
            case ParseResult::OK:
                if (sensorFaultActive_) {
                    Serial.println("[OBSTACLE] Recovery: valid distance received — "
                                   "sensor fault cleared.");
                    sensorFaultActive_ = false;
                }
                measuredMm = dist;
                gotFrame = true;
                diagFramesOk_++;
                consecutiveInvalidFrames_ = 0;
                if (dist > diagMaxDistMm_) diagMaxDistMm_ = dist;
                break;
            case ParseResult::NO_TARGET:
                diagNoTarget_++;
                if (cfg_.shortRangeMode) {
                    // SHORT RANGE: no target = nothing within ~1.3 m = safe.
                    // Report maxRangeMm (zone 0) instead of emergency-close.
                    measuredMm = cfg_.maxRangeMm;
                } else {
                    // LONG RANGE: no target → emergency-close fallback.
                    consecutiveInvalidFrames_++;
                    measuredMm = cfg_.minRangeMm;
                }
                gotFrame = true;
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

        /* Frame consumed (or rejected).  Reset for next frame.
         *
         * On BAD_CHECKSUM or BAD_HEADER, scan forward for another
         * 0x57 that could be the real start of the next frame.   */
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

    // Auto-recovery: if the sensor keeps returning no-target frames for
    // an extended period, retry configureLongRange().
    if (consecutiveInvalidFrames_ >= AUTO_RECOVERY_FRAME_THRESHOLD
            && autoRecoveryAttempts_ < MAX_AUTO_RECOVERY_ATTEMPTS
            && cfg_.txPin >= 0) {
        autoRecoveryAttempts_++;
        Serial.printf("[OBSTACLE] Auto-recovery attempt %u/%u: "
                      "re-sending configureLongRange() "
                      "(%lu consecutive no-target frames)\n",
                      (unsigned)autoRecoveryAttempts_,
                      (unsigned)MAX_AUTO_RECOVERY_ATTEMPTS,
                      (unsigned long)consecutiveInvalidFrames_);
        configureLongRange();
        consecutiveInvalidFrames_ = 0;
    }

    // Periodic diagnostic output (every DIAG_INTERVAL_MS)
    if (now - lastDiagMs_ >= DIAG_INTERVAL_MS) {
        uint32_t totalFail = diagChecksumFail_ + diagHeaderFail_ + diagNoTarget_;
        if (diagFramesOk_ > 0 || totalFail > 0 || diagBytesDiscarded_ > 0) {
            Serial.printf("[OBSTACLE] Diag %lus: OK=%lu cksumFail=%lu hdrFail=%lu "
                          "noTarget=%lu discarded=%lu\n",
                          (unsigned long)(DIAG_INTERVAL_MS / 1000),
                          (unsigned long)diagFramesOk_,
                          (unsigned long)diagChecksumFail_,
                          (unsigned long)diagHeaderFail_,
                          (unsigned long)diagNoTarget_,
                          (unsigned long)diagBytesDiscarded_);
            if (diagChecksumFail_ > diagFramesOk_ && diagChecksumFail_ > 0) {
                Serial.println("[OBSTACLE] WARNING: Most frames fail checksum. "
                               "Check voltage divider resistor values — if you "
                               "measure ~2.1V on GPIO 18, R1 (series) is likely "
                               "3.3kohm instead of 1kohm. Expected ~2.9V with "
                               "correct R1=1kohm (series) + R2=4.7kohm (to GND). "
                               "See TOFSENSE_M_WIRING_GUIDE.md section 5");
            }
            if (!diagShortRangeWarned_
                    && diagNoTarget_ > DIAG_MIN_FRAMES_FOR_RANGE_CHECK
                    && diagFramesOk_ == 0) {
                if (cfg_.shortRangeMode) {
                    Serial.printf("[OBSTACLE] INFO: All %lu frames had no target "
                                  "— normal for SHORT RANGE read-only mode "
                                  "with no obstacle within ~%.1f m. Reporting %u mm "
                                  "(safe/clear).\n",
                                  (unsigned long)diagNoTarget_,
                                  (float)cfg_.maxRangeMm / 1000.0f,
                                  (unsigned)cfg_.maxRangeMm);
                } else {
                    Serial.printf("[OBSTACLE] WARNING: All %lu frames returned "
                                  "no target (distance stuck at %u mm). "
                                  "Sensor is likely in SHORT RANGE mode with no "
                                  "obstacle within ~1.3 m.  Fix: call "
                                  "configureLongRange() with txPin connected, "
                                  "or use NAssistant to switch to Long Range mode.\n",
                                  (unsigned long)diagNoTarget_,
                                  (unsigned)cfg_.minRangeMm);
                }
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
                diagShortRangeWarned_ = true;
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
        diagNoTarget_        = 0;
        diagBytesDiscarded_  = 0;
        diagMaxDistMm_       = 0;
        lastDiagMs_          = now;
    }

    if (!gotFrame) {
        // No valid frame received — check timeout.
        // Two cases:
        //   1) Had a valid frame before → timeout from lastValidMs_.
        //   2) Never received any frame → timeout from end of warmup.
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

    // Valid reading — update distance
    reading_.distance_mm = measuredMm;
    reading_.zone        = distanceToZone(measuredMm);
    lastValidMs_         = now;

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

    // ---- Sensor fault override ----
    bool recoveryPossible = (autoRecoveryAttempts_ < MAX_AUTO_RECOVERY_ATTEMPTS)
                            && (cfg_.txPin >= 0);
    if (!cfg_.shortRangeMode
            && !recoveryPossible
            && consecutiveInvalidFrames_ >= SENSOR_FAULT_FRAME_THRESHOLD) {
        if (!sensorFaultActive_) {
            sensorFaultActive_ = true;
            Serial.printf("[OBSTACLE] FAULT: Sensor declared permanently invalid — "
                          "%lu consecutive no-target frames, %u/%u recovery attempts, "
                          "txPin=%d. "
                          "Fix: set Config::txPin to the ESP32 TX GPIO wired to "
                          "sensor RX and reboot, or use NAssistant to configure "
                          "Long Range mode.\n",
                          (unsigned long)consecutiveInvalidFrames_,
                          (unsigned)autoRecoveryAttempts_,
                          (unsigned)MAX_AUTO_RECOVERY_ATTEMPTS,
                          cfg_.txPin);
        }
        reading_.healthy = false;
        reading_.status  = SensorStatus::INVALID;
    }

    reading_.updateCount++;
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
    if (payloadLen > 0 && payload == nullptr) return 0;
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
