// =============================================================================
// ESP32-S3 — Obstacle Sensor Driver (TOFSense-M 8×8 by Nooploop)
//
// Reads distance from TOFSense-M (8×8 matrix) LiDAR sensor via UART.
// This driver supports ONLY the multi-pixel frame format:
//
//   Multi-pixel (TOFSense-M 8×8) — 400-byte Frame0:
//     [0x57] [0x01] [0xFF] [ID] [TIME×4] [NUM_PIX=0x40]
//     followed by 64 × 6-byte pixel blocks:
//       [DIST_L] [DIST_H] [DIST_UH] [STATUS] [SIG_L] [SIG_H]
//     Distance in µm (uint24 LE, divide by 1000 for mm).
//     Status 0x00 = valid pixel.
//     Trailer: 6 × 0xFF + 1-byte checksum (sum of all preceding & 0xFF).
//
// The driver computes:
//   - minDist:    minimum valid-pixel distance (closest obstacle)
//   - maxDist:    maximum valid-pixel distance
//   - avgDist:    mean valid-pixel distance
//   - validCount: number of pixels with status == 0x00 and distance > 0
//   - dispersion: maxDist − minDist (spread across the 8×8 FOV)
//
// The reported distance is minDist (closest point in the FOV).
// Frames are rejected if validCount < MIN_VALID_PIXELS (4) or
// dispersion > MAX_PIXEL_DISPERSION_MM (3000 mm).
//
// Parses at 921600 bps (factory default), 8N1.
// Provides validated readings with stuck-sensor detection,
// warmup filtering, and physical-range validation.
//
// UART configuration:
//   UART1, RX = GPIO 18 (sensor TX → ESP32 RX), TX = -1 (no commands)
//   Baud rate = 921600 bps, 8N1
//
// Frame layout (400 bytes):
//   [0]    header         = 0x57
//   [1]    function_mark  = 0x01 (multi-pixel)
//   [2]    reserved       = 0xFF
//   [3]    sensor_id
//   [4-7]  system_time    uint32 LE (ms)
//   [8]    num_pixels     (0x40 = 64 for 8×8)
//   [9..N] pixel data     64 × 6 bytes each:
//            [0-2] distance  uint24 LE (µm)
//            [3]   status    0x00 = valid
//            [4-5] signal    uint16 LE
//   [N+1..N+6] end marker  6 × 0xFF
//   [N+7]  checksum        sum of bytes [0..N+6] & 0xFF
//
// Hardware: TOFSense-M (Nooploop) — GH1.25 4-pin connector
//   Pin1=VCC (5 V required), Pin2=GND, Pin3=RX (not used), Pin4=TX → ESP32 RX
//   UART IO level: 3.5–3.6 V → protection REQUIRED (voltage divider or level shifter).
//   See docs/TOFSENSE_M_WIRING_GUIDE.md for wiring diagrams.
//
// Reference: TOFSense-M User Manual V3.0
//            https://ftp.nooploop.com/downloads/tofsense/TOFSense-M_User_Manual_V3.0_en.pdf
//            docs/CAN_CONTRACT_FINAL.md rev 1.3 (0x208 payload)
//
// ---- SENSOR ENABLE/DISABLE ------------------------------------------------
// Set OBSTACLE_SENSOR_ENABLED to 0 to disable all UART and parsing logic.
// When disabled:
//   - init() logs a message but does NOT configure UART
//   - update() is a no-op (returns immediately, no blocking)
//   - getReading() returns safe defaults (distance=0, healthy=false,
//     status=INVALID) — CAN module stops sending frames, UI shows "---"
//
// Set to 1 to re-enable when the sensor hardware is reconnected.
//
// Future: replace TOFSense-M with Benewake TF-Mini Plus (UART, 115200 baud).
//         See distance_sensor.h for the common interface abstraction.
// -------------------------------------------------------------------------
// =============================================================================

#ifndef OBSTACLE_SENSOR_H_DRIVER
#define OBSTACLE_SENSOR_H_DRIVER

// ---- Compile-time sensor enable flag ----------------------------------------
// 0 = sensor hardware removed (all UART/parsing disabled, safe defaults)
// 1 = sensor connected and active
#ifndef OBSTACLE_SENSOR_ENABLED
#define OBSTACLE_SENSOR_ENABLED 0
#endif

// ---- Sensor type selection --------------------------------------------------
// Select which physical sensor hardware is connected.
// Only one type may be active at a time.
#define SENSOR_TYPE_TOFSENSE  0   // Nooploop TOFSense-M 8×8 (921600 baud, 400-byte frames)
#define SENSOR_TYPE_TFMINI    1   // Benewake TF-Mini Plus   (115200 baud, 9-byte frames)

#ifndef SENSOR_TYPE
#define SENSOR_TYPE  SENSOR_TYPE_TFMINI   // Default: TF-Mini Plus (next planned sensor)
#endif                                    // When OBSTACLE_SENSOR_ENABLED=0 (current state),
                                          // this default has no effect — all sensor code is
                                          // compiled out.  It only matters when re-enabling.

// ---- TO ENABLE TF-MINI PLUS: ------------------------------------------------
// 1. Set OBSTACLE_SENSOR_ENABLED = 1 in this file (or via -D compiler flag)
// 2. Set SENSOR_TYPE = SENSOR_TYPE_TFMINI (already the default)
// 3. Connect sensor TX to ESP32 RX (GPIO 18 default), 3.3V logic — no divider
// 4. Provide 5V power to sensor (Pin 1), GND (Pin 2)
// 5. Ensure baud rate = 115200 (TF-Mini Plus factory default)
// 6. Flash firmware (pio run -t upload)
// 7. Verify logs: "[OBSTACLE] TF-Mini Plus init ..." should appear on Serial
// 8. Verify CAN data: 0x208 frames with valid distance (health=1, status=VALID)
// ---- END ACTIVATION INSTRUCTIONS -------------------------------------------

#include <cstdint>

namespace obstacle_sensor {

// -------------------------------------------------------------------------
// Sensor status reported to HMI boot screen
// -------------------------------------------------------------------------
enum class SensorStatus : uint8_t {
    WAITING,     // Warmup period — not yet producing valid data
    INVALID,     // Timeout, too few pixels, excessive dispersion, or stuck
    VALID        // Healthy reading within physical range
};

// -------------------------------------------------------------------------
// Validated sensor reading — consumed by can_obstacle for 0x208 frame
// -------------------------------------------------------------------------
struct Reading {
    uint16_t     distance_mm  = 0;      // Minimum valid-pixel distance (mm), 0 = no reading
    uint8_t      zone         = 0;      // Distance zone (0–4): 0=far, 1=caution, 2=warn, 3=crit, 4=emergency
    bool         healthy      = false;  // true if reading is valid and plausible
    bool         stuck        = false;  // true if stuck-sensor condition detected
    SensorStatus status       = SensorStatus::WAITING;
    uint32_t     updateCount  = 0;      // Increments on each valid frame (telemetry/diagnostics)
    // Pixel statistics from last valid frame
    uint16_t     minDist_mm   = 0;      // Minimum valid-pixel distance (mm)
    uint16_t     maxDist_mm   = 0;      // Maximum valid-pixel distance (mm)
    uint16_t     avgDist_mm   = 0;      // Mean valid-pixel distance (mm)
    uint8_t      validCount   = 0;      // Number of valid pixels (0–64)
    uint16_t     dispersion_mm= 0;      // maxDist − minDist (mm)
};

// -------------------------------------------------------------------------
// Configuration — UART for obstacle distance sensor
// Defaults depend on SENSOR_TYPE (TF-Mini Plus vs TOFSense-M).
// -------------------------------------------------------------------------
struct Config {
    int      rxPin             = 18;    // GPIO for UART1 RX (sensor TX → ESP32 RX)
    int      txPin             = -1;    // GPIO for UART1 TX (-1 = not connected, receive-only)
#if SENSOR_TYPE == SENSOR_TYPE_TFMINI
    uint32_t baudRate          = 115200; // TF-Mini Plus factory default
    uint16_t rxBufSize         = 256;    // 9-byte frames — 256 bytes is ample
    uint32_t warmupMs          = 500;    // TF-Mini Plus stabilises faster
    uint16_t minRangeMm        = 100;    // TF-Mini Plus minimum range (10 cm)
    uint16_t maxRangeMm        = 12000;  // TF-Mini Plus maximum range (12 m)
#else  // SENSOR_TYPE_TOFSENSE
    uint32_t baudRate          = 921600; // TOFSense-M factory default
    uint16_t rxBufSize         = 4096;   // UART RX ring-buffer (~44 ms at 921600 baud)
    uint32_t warmupMs          = 1000;   // Warmup period after init (ms)
    uint16_t minRangeMm        = 20;     // Minimum physical range (mm)
    uint16_t maxRangeMm        = 4000;   // Maximum physical range (mm)
#endif
    uint32_t frameTimeoutMs    = 500;    // Max time without valid frame before INVALID
    uint32_t stuckDurationMs   = 1000;   // Duration for stuck detection (ms)
    uint16_t stuckThresholdMm  = 10;     // Change threshold for stuck detection (mm)
    float    minSpeedForStuck  = 1.0f;   // Vehicle speed threshold (km/h) for stuck detection
};

/// Initialize sensor hardware.  Call once from setup().
void init(const Config& cfg = Config{});

/// Update sensor reading.  Call from loop() every iteration.
/// @param vehicleSpeedKmh  Current vehicle speed for stuck detection.
void update(float vehicleSpeedKmh);

/// Get the latest validated reading.
Reading getReading();

// =========================================================================
// NLink 0x5A command protocol — for sensor configuration via UART TX.
// Requires Config::txPin ≥ 0 (TX wired to sensor RX).
// =========================================================================

static constexpr uint8_t CMD_HEADER = 0x5A;

enum class RangeMode : uint8_t {
    SHORT_RANGE = 0x00,
    LONG_RANGE  = 0x01
};

enum class BaudRateCode : uint8_t {
    BAUD_9600    = 0x00,
    BAUD_19200   = 0x01,
    BAUD_38400   = 0x02,
    BAUD_57600   = 0x03,
    BAUD_115200  = 0x04,
    BAUD_230400  = 0x05,
    BAUD_460800  = 0x06,
    BAUD_921600  = 0x07
};

enum class OutputMode : uint8_t {
    ACTIVE = 0x00,
    QUERY  = 0x01
};

enum class CmdId : uint8_t {
    SET_SENSOR_ID   = 0x01,
    SET_OUTPUT_MODE = 0x02,
    SET_BAUD_RATE   = 0x03,
    SET_OUTPUT_IF   = 0x04,
    SET_FRAME_RATE  = 0x05,
    SET_RESOLUTION  = 0x06,
    SET_RANGE_MODE  = 0x07,
    SAVE_CONFIG     = 0x08
};

uint16_t buildCommand(uint8_t cmdId, const uint8_t* payload,
                      uint8_t payloadLen, uint8_t* outBuf,
                      uint16_t outBufSize);

bool setRangeMode(RangeMode mode);
bool setBaudRate(BaudRateCode baud);
bool setFrameRate(uint8_t hz);
bool setOutputMode(OutputMode mode);
bool saveConfig();
bool configureLongRange();

} // namespace obstacle_sensor

#endif // OBSTACLE_SENSOR_H_DRIVER
