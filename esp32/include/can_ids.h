// =============================================================================
// ESP32-S3 HMI — CAN Bus ID and Protocol Definitions
//
// Source of truth: docs/CAN_CONTRACT_FINAL.md rev 1.2
// Status:          FROZEN — do not modify without a new contract revision
//
// These values are mirrored from the CAN contract document.
// They are NOT imported from STM32 headers.
// =============================================================================

#ifndef CAN_IDS_H
#define CAN_IDS_H

#include <cstdint>

namespace can {

// -------------------------------------------------------------------------
// CAN Bus Parameters (CAN_CONTRACT_FINAL.md §2)
// -------------------------------------------------------------------------
inline constexpr uint32_t CAN_BITRATE          = 500000;   // 500 kbps
inline constexpr uint8_t  CAN_MAX_PAYLOAD      = 8;        // Classic CAN

// -------------------------------------------------------------------------
// ESP32 → STM32  Commands / Heartbeat (§3.1)
// -------------------------------------------------------------------------
inline constexpr uint32_t HEARTBEAT_ESP32       = 0x011;    // DLC —, 100 ms
inline constexpr uint32_t CMD_THROTTLE          = 0x100;    // DLC 1, 50 ms
inline constexpr uint32_t CMD_STEERING          = 0x101;    // DLC 2, 50 ms
inline constexpr uint32_t CMD_MODE              = 0x102;    // DLC 2 (byte0=mode flags, byte1=gear), on-demand

// -------------------------------------------------------------------------
// STM32 → ESP32  Command Acknowledgment (§3.5)
// -------------------------------------------------------------------------
inline constexpr uint32_t CMD_ACK               = 0x103;    // DLC 3, on-demand (after CMD_MODE / SERVICE_CMD)

// -------------------------------------------------------------------------
// STM32 → ESP32  Status / Heartbeat (§3.2)
// -------------------------------------------------------------------------
inline constexpr uint32_t HEARTBEAT_STM32       = 0x001;    // DLC 4, 100 ms
inline constexpr uint32_t STATUS_SPEED          = 0x200;    // DLC 8, 100 ms
inline constexpr uint32_t STATUS_CURRENT        = 0x201;    // DLC 8, 100 ms
inline constexpr uint32_t STATUS_TEMP           = 0x202;    // DLC 5, 1000 ms
inline constexpr uint32_t STATUS_SAFETY         = 0x203;    // DLC 3, 100 ms
inline constexpr uint32_t STATUS_STEERING       = 0x204;    // DLC 3, 100 ms
inline constexpr uint32_t STATUS_TRACTION       = 0x205;    // DLC 4, 100 ms
inline constexpr uint32_t STATUS_TEMP_MAP       = 0x206;    // DLC 5, 1000 ms
inline constexpr uint32_t STATUS_BATTERY        = 0x207;    // DLC 4, 100 ms  battery 24V bus current + voltage

// -------------------------------------------------------------------------
// ESP32 → STM32  Obstacle Data (CAN_CONTRACT_FINAL.md §3.4)
// -------------------------------------------------------------------------
inline constexpr uint32_t OBSTACLE_DISTANCE      = 0x208;    // DLC 5, 66 ms  obstacle distance + zone + health + counter
inline constexpr uint32_t OBSTACLE_SAFETY         = 0x209;    // DLC 8, 100 ms obstacle safety state (informational)

// -------------------------------------------------------------------------
// Bidirectional  Diagnostic (§3.3)
// -------------------------------------------------------------------------
inline constexpr uint32_t DIAG_ERROR            = 0x300;    // DLC 2, on-demand

// -------------------------------------------------------------------------
// Service Mode (docs/SERVICE_MODE.md, Core/Inc/can_handler.h)
// -------------------------------------------------------------------------
inline constexpr uint32_t SERVICE_FAULTS         = 0x301;   // STM32→ESP32, DLC 4, 1000 ms
inline constexpr uint32_t SERVICE_ENABLED        = 0x302;   // STM32→ESP32, DLC 4, 1000 ms
inline constexpr uint32_t SERVICE_DISABLED       = 0x303;   // STM32→ESP32, DLC 4, 1000 ms
inline constexpr uint32_t SERVICE_CMD            = 0x110;   // ESP32→STM32, DLC 2, on-demand

// -------------------------------------------------------------------------
// System States — HEARTBEAT_STM32 byte 1 (§6)
// -------------------------------------------------------------------------
enum class SystemState : uint8_t {
    BOOT     = 0,    // Power-on, peripherals initializing
    STANDBY  = 1,    // Ready, waiting for ESP32 heartbeat
    ACTIVE   = 2,    // Normal operation, commands accepted
    DEGRADED = 3,    // Limp / degraded — commands accepted with reduced limits
    SAFE     = 4,    // Fault detected, actuators inhibited
    ERROR    = 5     // Unrecoverable fault, manual reset required
};

// -------------------------------------------------------------------------
// Fault Flags — HEARTBEAT_STM32 byte 2 bitmask (§6)
// -------------------------------------------------------------------------
enum class FaultFlag : uint8_t {
    CAN_TIMEOUT       = 0x01,   // Bit 0: ESP32 heartbeat lost > 250 ms
    TEMP_OVERLOAD     = 0x02,   // Bit 1: Motor temperature > 90 °C
    CURRENT_OVERLOAD  = 0x04,   // Bit 2: Motor current > 25 A
    ENCODER_ERROR     = 0x08,   // Bit 3: Encoder / sensor fault
    WHEEL_SENSOR      = 0x10,   // Bit 4: Wheel speed sensor fault
    ABS_ACTIVE        = 0x20,   // Bit 5: ABS intervening
    TCS_ACTIVE        = 0x40,   // Bit 6: TCS intervening
    CENTERING         = 0x80    // Bit 7: Steering centering failed
};

// -------------------------------------------------------------------------
// Safety Error Codes — STATUS_SAFETY byte 2 / DIAG_ERROR byte 0 (§7)
// -------------------------------------------------------------------------
enum class SafetyError : uint8_t {
    NONE             = 0,
    OVERCURRENT      = 1,
    OVERTEMP         = 2,
    CAN_TIMEOUT      = 3,
    SENSOR_FAULT     = 4,
    MOTOR_STALL      = 5,   // Reserved — not implemented in current firmware
    EMERGENCY_STOP   = 6,
    WATCHDOG         = 7,
    CENTERING        = 8,       // Steering centering failed
    BATTERY_UV_WARN  = 9,       // Battery voltage < 20.0 V
    BATTERY_UV_CRIT  = 10,      // Battery voltage < 18.0 V
    I2C_FAILURE      = 11,      // I2C bus locked / unrecoverable
    OBSTACLE         = 12       // Obstacle emergency or CAN timeout
};

// -------------------------------------------------------------------------
// Command ACK Result Codes — CMD_ACK byte 1 (§4.17)
// -------------------------------------------------------------------------
enum class AckResult : uint8_t {
    OK                  = 0,   // Command accepted and applied
    REJECTED            = 1,   // Command rejected (speed too high, etc.)
    INVALID             = 2,   // Command payload invalid / malformed
    BLOCKED_BY_SAFETY   = 3    // Command blocked by safety system state
};

// -------------------------------------------------------------------------
// Diagnostic Subsystem IDs — DIAG_ERROR byte 1 (§4.14)
// -------------------------------------------------------------------------
enum class DiagSubsystem : uint8_t {
    GLOBAL  = 0,
    MOTOR   = 1,
    SENSOR  = 2,
    CAN_BUS = 3
};

// -------------------------------------------------------------------------
// Timing Constants (§6)
// -------------------------------------------------------------------------
inline constexpr uint32_t HEARTBEAT_INTERVAL_MS = 100;   // Both directions
inline constexpr uint32_t HEARTBEAT_TIMEOUT_MS  = 250;   // STM32 watchdog
inline constexpr uint32_t CMD_THROTTLE_RATE_MS  = 50;
inline constexpr uint32_t CMD_STEERING_RATE_MS  = 50;
inline constexpr uint32_t STATUS_FAST_RATE_MS   = 100;   // Speed, current, safety, steering, traction
inline constexpr uint32_t STATUS_SLOW_RATE_MS   = 1000;  // Temperature
inline constexpr uint32_t OBSTACLE_RATE_MS      = 66;    // Obstacle distance (15 Hz)
inline constexpr uint32_t OBSTACLE_TIMEOUT_MS   = 500;   // STM32 obstacle CAN timeout (fail-safe)
inline constexpr uint32_t ACK_TIMEOUT_MS        = 200;   // ESP32 command ACK timeout (UI update)

// -------------------------------------------------------------------------
// Drive Mode Flags — CMD_MODE byte 0 (§4.5)
// -------------------------------------------------------------------------
inline constexpr uint8_t MODE_FLAG_4X4       = 0x01;   // Bit 0: 1 = 4×4, 0 = 4×2
inline constexpr uint8_t MODE_FLAG_TANK_TURN = 0x02;   // Bit 1: 1 = tank turn enabled

// -------------------------------------------------------------------------
// Service Commands — SERVICE_CMD byte 0 (docs/SERVICE_MODE.md)
// -------------------------------------------------------------------------
inline constexpr uint8_t SERVICE_ACTION_DISABLE        = 0x00;
inline constexpr uint8_t SERVICE_ACTION_ENABLE         = 0x01;
inline constexpr uint8_t SERVICE_ACTION_FACTORY_RESTORE = 0xFF;

} // namespace can

#endif // CAN_IDS_H
