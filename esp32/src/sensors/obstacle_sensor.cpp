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
#include <cstring>

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
// NLink protocol distance unit: 1 µm per LSB (divide by 1000 for mm)
static constexpr int32_t DISTANCE_UNITS_PER_MM = 1000;

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
static unsigned long lastDiagMs_      = 0;
static uint32_t diagFramesOk_        = 0;  // Frames parsed successfully
static uint32_t diagChecksumFail_    = 0;  // Checksum mismatches
static uint32_t diagHeaderFail_      = 0;  // Wrong header or function mark
static uint32_t diagAllPixelsInvalid_= 0;  // All 64 dis_status ≠ 0
static uint32_t diagBytesDiscarded_  = 0;  // Bytes skipped waiting for 0x57

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
// Per-pixel layout (6 bytes, per official Nooploop struct):
//   [0-2]  dis:              3-byte signed int24 LE, unit = µm (/1000 = mm)
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
    bool anyValid = false;

    for (uint8_t px = 0; px < PIXEL_COUNT_8X8; px++) {
        uint16_t base = OFF_PIXEL_DATA + (uint16_t)(px * BYTES_PER_PIXEL);
        if (base + BYTES_PER_PIXEL > FRAME_LENGTH) break;

        uint8_t status = buf[base + 3];
        if (status != 0) continue;  // Skip invalid pixels

        // Distance: 3-byte signed little-endian, unit = µm per LSB
        int32_t raw = (int32_t)buf[base]
                    | ((int32_t)buf[base + 1] << 8)
                    | ((int32_t)buf[base + 2] << 16);
        // Sign-extend 24-bit value to 32-bit: check bit 23 (sign bit)
        if (raw & 0x800000) raw |= (int32_t)0xFF000000;

        if (raw <= 0) continue;  // Negative or zero distance — skip

        uint32_t distMm = (uint32_t)(raw / DISTANCE_UNITS_PER_MM);
        if (distMm < minDistMm) {
            minDistMm = distMm;
            anyValid = true;
        }
    }

    if (!anyValid) return ParseResult::NO_VALID_PIXELS;
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

    reading_ = Reading{};  // Reset to defaults

    Serial.printf("[OBSTACLE] TOFSense-M init (UART1, %lu bps, rxBuf %u)\n",
                  (unsigned long)cfg_.baudRate, (unsigned)cfg_.rxBufSize);
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

    // ---- Overflow / freshness management ----
    // If more than 2 full frames have accumulated in the UART RX ring-buffer
    // while update() wasn't called (display refresh, CAN burst, etc.), the
    // oldest data is stale.  Discard everything except the last frame's worth
    // so we always process the most recent sensor output.  This prevents:
    //  (a) byte-by-byte processing from blocking the main loop for too long,
    //  (b) cascading checksum failures from buffer-overflow byte loss.
    if (rxIdx_ == 0) {
        int avail = tofSerial.available();
        if (avail > (int)(FRAME_LENGTH * 2)) {
            int toDiscard = avail - (int)FRAME_LENGTH;
            for (int i = 0; i < toDiscard; i++) {
                tofSerial.read();
            }
            diagBytesDiscarded_ += (uint32_t)toDiscard;
        }
    }

    // Read available UART bytes and parse frames.
    // Limit to MAX_BYTES_PER_UPDATE to prevent this loop from blocking the
    // main loop when multiple frames are queued.  Any remaining bytes stay
    // in the UART ring-buffer and are picked up on the next call.
    static constexpr uint16_t MAX_BYTES_PER_UPDATE = FRAME_LENGTH * 3;
    uint16_t bytesProcessed = 0;
    uint16_t measuredMm = 0;
    bool gotFrame = false;

    while (tofSerial.available() > 0 && bytesProcessed < MAX_BYTES_PER_UPDATE) {
        uint8_t byte = (uint8_t)tofSerial.read();
        bytesProcessed++;

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
            // After a failed parse, scan the consumed buffer for the next
            // valid {0x57, 0x01} header.  Without this resync a single
            // corrupted frame costs up to 400 extra bytes of re-alignment
            // (almost an entire additional frame period at 10 Hz), which is
            // the main driver behind the intermittent INVALID transitions
            // the user observes at medium-to-long range.
            if (pr == ParseResult::BAD_CHECKSUM || pr == ParseResult::BAD_HEADER) {
                bool resynced = false;
                for (uint16_t i = 1; i < FRAME_LENGTH - 1; i++) {
                    if (rxBuf_[i] == FRAME_HEADER && rxBuf_[i + 1] == FUNCTION_MARK) {
                        uint16_t remaining = FRAME_LENGTH - i;
                        memmove(rxBuf_, rxBuf_ + i, remaining);
                        rxIdx_ = remaining;
                        resynced = true;
                        break;
                    }
                }
                if (!resynced) {
                    rxIdx_ = 0;
                }
            } else {
                rxIdx_ = 0;  // Successful parse — reset for next frame
            }
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
    if (measuredMm > cfg_.maxRangeMm) {
        // Above maximum range — likely no target or sensor noise
        reading_.healthy = false;
        reading_.status  = SensorStatus::INVALID;
        return;
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

} // namespace obstacle_sensor
