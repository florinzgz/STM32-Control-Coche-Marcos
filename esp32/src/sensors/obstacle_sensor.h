// =============================================================================
// ESP32-S3 — Obstacle Sensor Driver (TF-Mini Plus / TOFSense-M 8×8)
//
// Active sensor: Benewake TF-Mini Plus (SENSOR_TYPE_TFMINI)
//   - UART 115200 bps, 9-byte frames, 100 Hz output
//   - Single-point distance in cm (uint16 LE), converted to mm
//   - Signal strength validation, checksum, stuck detection
//   - Range: 100–12000 mm (10 cm – 12 m)
//
// Also supports (compile-time): Nooploop TOFSense-M 8×8
//   - UART 921600 bps, 400-byte multi-pixel frames
//   - 64 pixels, per-pixel statistics, quality gates
//
// The driver computes:
//   - distance_mm: validated distance to nearest obstacle (raw, not filtered)
//   - zone:        0–4 classification matching STM32 safety tiers
//                  (50 cm minimum-distance policy):
//                    < 500 mm   → 4 emergency (scale 0.0, forward blocked)
//                    500–1000   → 3 critical  (scale 0.3)
//                    1000–1500  → 2 warning   (scale 0.7)
//                    1500–2000  → 1 caution   (scale 0.85)
//                    ≥ 2000     → 0 alert / normal
//                  An EMA filter (α=0.3) is applied before classification
//                  to suppress boundary flicker under LiDAR noise.
//   - stuck:       sensor-stuck detection (unchanged distance while moving)
//   - healthy:     composite health flag for CAN reporting
//
// UART configuration:
//   UART1, RX = GPIO 18 (sensor TX → ESP32 RX), TX = -1 (no commands)
//
// Reference: docs/TFMINI_PLUS_WIRING_GUIDE.md
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
// -------------------------------------------------------------------------
// =============================================================================

#ifndef OBSTACLE_SENSOR_H_DRIVER
#define OBSTACLE_SENSOR_H_DRIVER

// ---- Compile-time sensor enable flag ----------------------------------------
// 0 = sensor hardware removed (all UART/parsing disabled, safe defaults)
// 1 = sensor connected and active (TF-Mini Plus on GPIO 18, 115200 baud)
#ifndef OBSTACLE_SENSOR_ENABLED
#define OBSTACLE_SENSOR_ENABLED 1
#endif

// ---- Sensor type selection --------------------------------------------------
// Select which physical sensor hardware is connected.
// Only one type may be active at a time.
#define SENSOR_TYPE_TOFSENSE  0   // Nooploop TOFSense-M 8×8 (921600 baud, 400-byte frames)
#define SENSOR_TYPE_TFMINI    1   // Benewake TF-Mini Plus   (115200 baud, 9-byte frames)

#ifndef SENSOR_TYPE
#define SENSOR_TYPE  SENSOR_TYPE_TFMINI   // Active: Benewake TF-Mini Plus (115200 baud, 9-byte frames)
#endif

// ---- TF-MINI PLUS — CURRENTLY ACTIVE ----------------------------------------
// Sensor is ENABLED with the following configuration:
// 1. OBSTACLE_SENSOR_ENABLED = 1 (active)
// 2. SENSOR_TYPE = SENSOR_TYPE_TFMINI (TF-Mini Plus)
// 3. Connect sensor TX to ESP32 RX (GPIO 18 default), 3.3V logic — no divider
// 4. Provide 5V power to sensor (red wire), GND (black wire)
// 5. Ensure baud rate = 115200 (TF-Mini Plus factory default)
// 6. Flash firmware (pio run -t upload)
// 7. Verify logs: "[OBSTACLE] TF-Mini Plus init ..." should appear on Serial
// 8. Verify CAN data: 0x208 frames with valid distance (health=1, status=VALID)
//
// To DISABLE: set OBSTACLE_SENSOR_ENABLED = 0 (or -DOBSTACLE_SENSOR_ENABLED=0)
// ---- END CONFIGURATION STATUS -----------------------------------------------

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
    uint8_t      zone         = 0;      // Distance zone (0–4): 0=alert/normal (≥2000 mm),
                                        // 1=caution (1500–2000), 2=warning (1000–1500),
                                        // 3=critical (500–1000), 4=emergency (<500 mm, 50 cm policy)
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
    uint16_t rxBufSize         = 512;    // 9-byte frames @ 100 Hz → 900 B/s; 512 B ≈ 570 ms headroom
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
