// =============================================================================
// ESP32-S3 — Obstacle Sensor Driver (TOFSense-M by Nooploop)
//
// Reads distance from TOFSense-M 8×8 LiDAR sensor via UART.
// Parses NLink_TOFSense_M_Frame0 protocol (400-byte frames at 921600 bps).
// Provides validated readings with stuck-sensor detection,
// warmup filtering, and physical-range validation.
//
// Also provides a configuration API (NLink 0x5A command protocol) to
// change sensor parameters via UART, including range mode, baud rate,
// frame rate, output mode, and persist settings to flash.
//
// KNOWN ISSUE — ~1368 mm maximum distance:
//   If the sensor reports a maximum of ~1300–1500 mm instead of its full
//   4000 mm range, the most likely cause is that it is configured in
//   "Short Range / High Precision" mode.  This mode limits the maximum
//   measurable distance to ~1.3–1.5 m in exchange for better accuracy.
//   FIX: switch to Long Range mode:
//       obstacle_sensor::setRangeMode(RangeMode::LONG_RANGE);
//       obstacle_sensor::saveConfig();
//   Or use the NAssistant PC tool to change the measurement mode.
//   The UART data frame parser itself is correct (verified by unit tests
//   for 2000 mm and 4000 mm distances).
//
// Per-pixel layout (6 bytes each, per Nooploop reference):
//   [0-2]  dis:              3-byte signed int24 LE, unit = µm (/1000 = mm)
//   [3]    dis_status:       0 = valid measurement
//   [4-5]  signal_strength:  uint16 LE
//
// Sensor output rate: ~10 Hz (active mode).
// Output: distance_mm, zone, health flag, stuck flag, sensor status.
//
// Hardware: TOFSense-M S (Nooploop) — GH1.25 4-pin connector
//   Pin1=VCC (5 V required), Pin2=GND, Pin3=RX (not used), Pin4=TX → ESP32 RX (GPIO 18)
//   VCC must be 5 V (sensor will not work at 3.3 V).
//   UART IO level: datasheet says 3.3 V TTL, but real measurements show
//   3.5–3.6 V on TX.  ESP32-S3 absolute max is 3.6 V → protection REQUIRED:
//     Option 1: voltage divider (1 kΩ + 4.7 kΩ to GND) → ~2.9 V
//     Option 2: BSS138-based level shifter (NOT TXS0108E) → 3.3 V
//   See docs/TOFSENSE_M_WIRING_GUIDE.md for wiring diagrams.
//
// Baudrate recommendations:
//   921600 bps (factory default) works well with the 2048-byte RX buffer
//   and is recommended for real-time obstacle detection.  If you experience
//   frequent checksum failures (check [OBSTACLE] Diag output), try:
//     1. Verify wiring (voltage divider values, cable length < 30 cm)
//     2. Lower to 460800 or 230400 bps (use setBaudRate() + saveConfig())
//     3. Reduce frame rate to 5 Hz (use setFrameRate(5) + saveConfig())
//   Lower baudrates reduce throughput but improve signal integrity on
//   longer cables or electrically noisy environments.
//
// Reference: TOFSense-M User Manual V3.0
//            https://ftp.nooploop.com/downloads/tofsense/TOFSense-M_User_Manual_V3.0_en.pdf
//            docs/CAN_CONTRACT_FINAL.md rev 1.3 (0x208 payload)
// =============================================================================

#ifndef OBSTACLE_SENSOR_H_DRIVER
#define OBSTACLE_SENSOR_H_DRIVER

#include <cstdint>

namespace obstacle_sensor {

// -------------------------------------------------------------------------
// Sensor status reported to HMI boot screen
// -------------------------------------------------------------------------
enum class SensorStatus : uint8_t {
    WAITING,     // Warmup period — not yet producing valid data
    INVALID,     // Timeout, out-of-range, or stuck
    VALID        // Healthy reading within physical range
};

// -------------------------------------------------------------------------
// Validated sensor reading — consumed by can_obstacle for 0x208 frame
// -------------------------------------------------------------------------
struct Reading {
    uint16_t     distance_mm  = 0;      // Measured distance (mm), 0 = no reading
    uint8_t      zone         = 0;      // Distance zone (0–4): 0=far, 1=caution, 2=warn, 3=crit, 4=emergency
    bool         healthy      = false;  // true if reading is valid and plausible
    bool         stuck        = false;  // true if stuck-sensor condition detected
    SensorStatus status       = SensorStatus::WAITING;
};

// -------------------------------------------------------------------------
// Configuration — UART for TOFSense-M LiDAR sensor
// -------------------------------------------------------------------------
struct Config {
    int      rxPin             = 18;    // GPIO for UART1 RX (sensor TX → ESP32 RX)
    int      txPin             = -1;    // GPIO for UART1 TX (-1 = not connected, receive-only)
    uint32_t baudRate          = 921600; // TOFSense-M default baud rate (must match sensor config; changeable via NAssistant)
    uint16_t rxBufSize         = 2048;  // UART RX ring-buffer (must be > 400; default ESP32 256 is too small)
    uint32_t warmupMs          = 1000;  // Warmup period after init (ms)
    uint16_t minRangeMm        = 20;    // Minimum physical range (mm)
    uint16_t maxRangeMm        = 4000;  // Maximum physical range (mm)
    uint32_t frameTimeoutMs    = 500;   // Max time without valid frame before INVALID
    uint32_t stuckDurationMs   = 1000;  // Duration for stuck detection (ms)
    uint16_t stuckThresholdMm  = 10;    // Change threshold for stuck detection (mm)
    float    minSpeedForStuck  = 1.0f;  // Vehicle speed threshold (km/h) for stuck detection
};

/// Initialize sensor hardware.  Call once from setup().
void init(const Config& cfg = Config{});

/// Update sensor reading.  Call from loop() every iteration.
/// @param vehicleSpeedKmh  Current vehicle speed for stuck detection.
void update(float vehicleSpeedKmh);

/// Get the latest validated reading.
Reading getReading();

// =========================================================================
// Sensor configuration via NLink protocol (command header 0x5A)
//
// ROOT CAUSE OF ~1368 mm RANGE LIMIT:
//   The TOFSense-M ships (or may be reconfigured via NAssistant) in
//   "Short Range / High Precision" mode, which limits maximum distance
//   to ~1.3–1.5 m.  To reach the full 4 m range, switch to "Long Range"
//   mode using setRangeMode(RangeMode::LONG_RANGE) followed by
//   saveConfig().
//
// IMPORTANT:
//   • Sending commands requires a TX pin connected to the sensor's RX.
//     Set Config::txPin to a valid GPIO (default is -1 = not connected).
//   • After changing baudrate, call init() again with the new baudRate
//     in Config, otherwise UART communication will be lost.
//   • Always call saveConfig() after changing settings to persist them
//     in the sensor's flash memory (survives power cycle).
//   • Configuration can also be done via the NAssistant PC tool
//     (recommended for initial setup).
//
// Command frame format (host → sensor, per TOFSense-M User Manual V3.0):
//   Byte 0:      0x5A (command header)
//   Byte 1:      Length (total bytes including header and checksum)
//   Byte 2:      Command ID
//   Bytes 3..N-2: Payload (command-specific)
//   Byte N-1:    Checksum (sum of bytes [0..N-2] & 0xFF)
//
// Reference: TOFSense-M User Manual V3.0, Section 6 — UART Configuration
//            https://ftp.nooploop.com/downloads/tofsense/TOFSense-M_User_Manual_V3.0_en.pdf
// =========================================================================

/// NLink 0x5A command protocol constants
static constexpr uint8_t CMD_HEADER = 0x5A;

/// Measurement range mode — controls max distance vs. precision trade-off.
/// The sensor in SHORT_RANGE mode caps at ~1.3–1.5 m (high precision).
/// Switch to LONG_RANGE for full 4 m capability.
enum class RangeMode : uint8_t {
    SHORT_RANGE = 0x00,   ///< High precision, max ~1.3–1.5 m
    LONG_RANGE  = 0x01    ///< Standard precision, max ~4 m
};

/// UART baud rate codes (index sent in the command payload).
enum class BaudRateCode : uint8_t {
    BAUD_9600    = 0x00,
    BAUD_19200   = 0x01,
    BAUD_38400   = 0x02,
    BAUD_57600   = 0x03,
    BAUD_115200  = 0x04,
    BAUD_230400  = 0x05,
    BAUD_460800  = 0x06,
    BAUD_921600  = 0x07   ///< Factory default
};

/// Sensor output mode.
enum class OutputMode : uint8_t {
    ACTIVE = 0x00,  ///< Continuous output at configured frame rate
    QUERY  = 0x01   ///< Output one frame per query command
};

/// Command IDs for the 0x5A configuration protocol
/// (per TOFSense-M User Manual V3.0, Section 6.2)
enum class CmdId : uint8_t {
    SET_SENSOR_ID   = 0x01,   ///< Set module ID
    SET_OUTPUT_MODE = 0x02,   ///< Active / query
    SET_BAUD_RATE   = 0x03,   ///< UART baud rate
    SET_OUTPUT_IF   = 0x04,   ///< UART / CAN interface
    SET_FRAME_RATE  = 0x05,   ///< Output frequency (Hz)
    SET_RESOLUTION  = 0x06,   ///< 4×4 or 8×8
    SET_RANGE_MODE  = 0x07,   ///< Short range / long range
    SAVE_CONFIG     = 0x08    ///< Persist settings to flash
};

// -------------------------------------------------------------------------
// Build a raw NLink 0x5A command frame.
//
// Writes [0x5A, totalLen, cmdId, payload..., checksum] into `outBuf`.
// Returns total frame length written, or 0 if outBuf is too small.
//
// @param cmdId       Command ID (CmdId enum value)
// @param payload     Pointer to payload bytes (may be nullptr if payloadLen==0)
// @param payloadLen  Number of payload bytes
// @param outBuf      Destination buffer (must be >= payloadLen + 3)
// @param outBufSize  Size of outBuf
// -------------------------------------------------------------------------
uint16_t buildCommand(uint8_t cmdId, const uint8_t* payload,
                      uint8_t payloadLen, uint8_t* outBuf,
                      uint16_t outBufSize);

// -------------------------------------------------------------------------
// High-level configuration API
//
// All functions return true if the command was sent successfully.
// The sensor must have TX pin connected (Config::txPin ≠ -1).
// Call saveConfig() after changing settings to persist them.
// -------------------------------------------------------------------------

/// Switch between short-range (high precision) and long-range modes.
/// This is the fix for the ~1368 mm range limit.
bool setRangeMode(RangeMode mode);

/// Change UART baud rate.  After calling this, you must re-init with
/// the matching Config::baudRate or communication will be lost.
bool setBaudRate(BaudRateCode baud);

/// Set output frame rate in Hz (1–15 for 8×8, 1–60 for 4×4).
bool setFrameRate(uint8_t hz);

/// Set output mode (active continuous or query-on-demand).
bool setOutputMode(OutputMode mode);

/// Save current configuration to sensor flash memory.
/// Settings persist across power cycles after this command.
bool saveConfig();

} // namespace obstacle_sensor

#endif // OBSTACLE_SENSOR_H_DRIVER
