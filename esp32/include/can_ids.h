// =============================================================================
// ESP32-S3 HMI — CAN Bus ID and Protocol Definitions
//
// Source of truth: docs/CAN_CONTRACT_FINAL.md rev 1.4
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
inline constexpr uint32_t HEARTBEAT_ESP32       = 0x011;    // DLC 1, 100 ms
inline constexpr uint32_t CMD_THROTTLE          = 0x100;    // DLC 1, 50 ms
inline constexpr uint32_t CMD_STEERING          = 0x101;    // DLC 2, 50 ms
inline constexpr uint32_t CMD_MODE              = 0x102;    // DLC 2 (byte0=mode flags, byte1=gear), on-demand
inline constexpr uint32_t CMD_RC_OVERRIDE       = 0x10A;    // DLC 5, 50 ms
                                                            // ESP32 → STM32 RC override demand.
                                                            // byte0: flags (bit0=override_active / CH10=REMOTE)
                                                            // byte1: throttle 0..100
                                                            // byte2..3: int16 LE steering 1/10°
                                                            // byte4: seq rolling counter
                                                            // 200 ms STM32-side watchdog → failsafe to local pedal.
inline constexpr uint32_t CMD_LED               = 0x120;    // DLC 2 (byte0=front relay, byte1=rear relay), on-demand
inline constexpr uint32_t CMD_SYSTEM_SHUTDOWN   = 0x130;    // DLC 0 or 1, on-demand
                                                            // Pre-power-cut safe-state handshake.
                                                            // Sent once when ignition key is turned OFF,
                                                            // before the external delay relay cuts power.
                                                            // Idempotent on the STM32 side. No ACK.

// -------------------------------------------------------------------------
// STM32 → ESP32  Command Acknowledgment (§3.5)
// -------------------------------------------------------------------------
inline constexpr uint32_t CMD_ACK               = 0x103;    // DLC 3, on-demand (after CMD_MODE / SERVICE_CMD)

// -------------------------------------------------------------------------
// STM32 → ESP32  Status / Heartbeat (§3.2)
// -------------------------------------------------------------------------
inline constexpr uint32_t HEARTBEAT_STM32       = 0x001;    // DLC 6, 100 ms (byte5=relay status; 3-bit layout: bit0=reserved/0, bit1=TRAC, bit2=DIR, bit7=SEQ)
inline constexpr uint32_t STATUS_SPEED          = 0x200;    // DLC 8, 100 ms
inline constexpr uint32_t STATUS_CURRENT        = 0x201;    // DLC 8, 100 ms
inline constexpr uint32_t STATUS_TEMP           = 0x202;    // DLC 5, 1000 ms
inline constexpr uint32_t STATUS_SAFETY         = 0x203;    // DLC 6, 100 ms  (ABS,TCS,error_code,state,rx_err,loop_peak_100us)
inline constexpr uint32_t STATUS_STEERING       = 0x204;    // DLC 3, 100 ms
inline constexpr uint32_t STATUS_TRACTION       = 0x205;    // DLC 4, 100 ms
inline constexpr uint32_t STATUS_TEMP_MAP       = 0x206;    // DLC 5, 1000 ms
inline constexpr uint32_t STATUS_BATTERY        = 0x207;    // DLC 4, 100 ms  battery 24V bus current + voltage

// -------------------------------------------------------------------------
// ESP32 → STM32  Obstacle Data (CAN_CONTRACT_FINAL.md §3.4)
// -------------------------------------------------------------------------
inline constexpr uint32_t OBSTACLE_DISTANCE      = 0x208;    // DLC 5, 66 ms  obstacle distance + zone + health + counter
inline constexpr uint32_t OBSTACLE_SAFETY         = 0x209;    // DLC 4, 100 ms obstacle safety state (informational)

// -------------------------------------------------------------------------
// STM32 → ESP32  LED / Lights Status (Audit Step 6)
// -------------------------------------------------------------------------
inline constexpr uint32_t STATUS_LIGHTS          = 0x20A;    // DLC 2, 1000 ms  byte0=front relay, byte1=rear relay
inline constexpr uint32_t STATUS_PEDAL           = 0x20B;    // DLC 1, 100 ms   byte0=Hall pedal position % (telemetry only)
// -------------------------------------------------------------------------
// ESP32 → STM32  LED relay command (§3.1)
//   Byte 0: front relay (0=OFF, 1=ON)
//   Byte 1: rear  relay (0=OFF, 1=ON)
// -------------------------------------------------------------------------
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
inline constexpr uint32_t ERROR_LOG_ENTRY        = 0x304;   // STM32→ESP32, DLC 8, on-demand
inline constexpr uint32_t ERROR_LOG_HEADER       = 0x305;   // STM32→ESP32, DLC 8, 1000 ms
inline constexpr uint32_t DIAG_DEBOUNCE          = 0x306;   // STM32→ESP32, DLC 8, 1000 ms — DWT-debounce filtered counts (4× wheel u16 LE)
inline constexpr uint32_t DIAG_DEBOUNCE_STEER    = 0x307;   // STM32→ESP32, DLC 4, 1000 ms — DWT-debounce filtered count (steer u32 LE)
inline constexpr uint32_t DIAG_PEDAL_CAL         = 0x308;   // STM32→ESP32, DLC 8, on-demand (10 Hz × 1 s after QUERY) — pedal calibration telemetry
inline constexpr uint32_t DIAG_I2C               = 0x309;   // STM32→ESP32, DLC 5, 1000 ms — I2C topology diag: mux present + per-channel INA226 health
inline constexpr uint32_t DIAG_CAN_META          = 0x30A;   // STM32→ESP32, DLC 8, 1000 ms — CAN/0x309 delivery meta-diag (call/tick/tx-ok/err/fifo-drops)
inline constexpr uint32_t DIAG_I2C_SCAN          = 0x30B;   // STM32→ESP32, DLC 8, on-demand — I2C service-mode scan (mux/INA probe, SDA/SCL levels, recovery)
inline constexpr uint32_t DIAG_FDCAN             = 0x30C;   // STM32→ESP32, DLC 6, on-demand — FDCAN error-counter dump (TEC/REC/LEC/state)
inline constexpr uint32_t DIAG_GEAR_LIMITS       = 0x30D;   // STM32→ESP32, DLC 8, on-demand (10 Hz × 1 s after QUERY) — gear power-limit telemetry
inline constexpr uint32_t DIAG_STEERING_Z        = 0x30E;   // STM32→ESP32, DLC 8, on-demand (10 Hz × 1 s after QUERY) — PB5 + encoder-Z dual center-reference diagnostic
inline constexpr uint32_t SERVICE_CMD            = 0x110;   // ESP32→STM32, DLC 2, on-demand
inline constexpr uint32_t CMD_SENSOR_MAP_TEMP    = 0x112;   // ESP32→STM32, DLC 5, on-demand  DS18B20 physIdx→role mapping

// -------------------------------------------------------------------------
// System States — HEARTBEAT_STM32 byte 1 (§6)
// -------------------------------------------------------------------------
enum class SystemState : uint8_t {
    BOOT      = 0,    // Power-on, peripherals initializing
    STANDBY   = 1,    // Ready, waiting for ESP32 heartbeat
    ACTIVE    = 2,    // Normal operation, commands accepted
    DEGRADED  = 3,    // Limp / degraded — commands accepted with reduced limits
    SAFE      = 4,    // Hardware danger, actuators inhibited
    ERROR     = 5,    // Unrecoverable fault, manual reset required
    LIMP_HOME = 6     // CAN-loss degraded — local pedal, walking speed, steering OK
};

// -------------------------------------------------------------------------
// Fault Flags — HEARTBEAT_STM32 byte 2 bitmask (§6)
// -------------------------------------------------------------------------
enum class FaultFlag : uint8_t {
    CAN_TIMEOUT       = 0x01,   // Bit 0: ESP32 heartbeat lost > 250 ms
    TEMP_OVERLOAD     = 0x02,   // Bit 1: Motor temperature ≥ 80 °C (warning or critical)
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
    OBSTACLE         = 12,      // Obstacle emergency or CAN timeout
    CAN_BUSOFF       = 13,      // FDCAN bus-off condition detected
    BATTERY_OV_WARN  = 14,      // Battery voltage > 30.0 V
    BATTERY_OV_CRIT  = 15,      // Battery voltage > 35.0 V
    RELAY_OPEN       = 16       // Relay health: insufficient motor current
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
inline constexpr uint32_t CAN_LOSS_TIMEOUT_MS   = 1500;  // ESP32 HMI: force error screen after no HB

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
inline constexpr uint8_t SERVICE_ACTION_RELAY_OVERRIDE = 0xE0;  // Engineering relay override (byte1=mask)
inline constexpr uint8_t SERVICE_ACTION_FACTORY_RESTORE = 0xFF;

// Individual factory-default reset commands (byte 0 of SERVICE_CMD)
// Byte 1 is unused (0x00) for these commands.
inline constexpr uint8_t SERVICE_ACTION_RESET_STEERING_PID   = 0xF0;
inline constexpr uint8_t SERVICE_ACTION_RESET_WHEEL_SENSORS  = 0xF1;
inline constexpr uint8_t SERVICE_ACTION_RESET_INA226_SHUNTS  = 0xF2;
inline constexpr uint8_t SERVICE_ACTION_RESET_TRACTION_FORCE = 0xF3;
inline constexpr uint8_t SERVICE_ACTION_RESET_STEERING_FORCE = 0xF4;
inline constexpr uint8_t SERVICE_ACTION_PEDAL_CAL            = 0xF5;  // Persistent pedal endpoint calibration (byte 1 = sub-opcode)
inline constexpr uint8_t SERVICE_ACTION_I2C_SERVICE         = 0xF6;  // I2C service-mode scan (probe mux/INA, SDA/SCL levels, recovery) → 0x30B + 0x30C
inline constexpr uint8_t SERVICE_ACTION_GEAR_LIMITS         = 0xF7;  // Gear power-limit config (byte 1 = sub-opcode, byte 2 = percent) → 0x30D
inline constexpr uint8_t SERVICE_ACTION_STEERING_Z          = 0xF8;  // PB5 + encoder-Z center diagnostic/calibration (byte 1 = sub-opcode) → 0x30E
inline constexpr uint8_t SERVICE_ACTION_CLEAR_ERROR_LOG      = 0xFE;

// I2C service-mode scan terminal phase — 0x30B (DIAG_I2C_SCAN) byte 5.
// Additive diagnostic mirroring Core/Inc/can_handler.h I2C_SCAN_PHASE_*.
// Lets the HMI name the fault (bus busy / TCA missing / TCA ack) instead of
// a bare "SCAN TIMEOUT".
inline constexpr uint8_t I2C_SCAN_PHASE_UNKNOWN     = 0x00;  // not determined
inline constexpr uint8_t I2C_SCAN_PHASE_BUS_BUSY    = 0x01;  // SDA idle low — bus stuck
inline constexpr uint8_t I2C_SCAN_PHASE_TCA_MISSING = 0x02;  // bus OK but TCA9548A 0x70 !ACK
inline constexpr uint8_t I2C_SCAN_PHASE_TCA_ACK     = 0x03;  // TCA9548A ACKed (mux present)

// Pedal-calibration sub-opcodes — byte 1 when byte 0 == SERVICE_ACTION_PEDAL_CAL
//   0x01 CAPTURE_MIN    Capture pedal-released ADC into pending MIN
//   0x02 CAPTURE_MAX    Capture pedal-pressed  ADC into pending MAX
//   0x03 SAVE           Validate pending pair + persist to STM32 flash
//   0x04 RESET_DEFAULTS Restore compile-time defaults (50 / 4000)   pedal direct 0–3.3 V, no divider
//   0x05 QUERY          Request a 1 s burst of 0x308 telemetry at 10 Hz
inline constexpr uint8_t PEDAL_CAL_OP_CAPTURE_MIN    = 0x01;
inline constexpr uint8_t PEDAL_CAL_OP_CAPTURE_MAX    = 0x02;
inline constexpr uint8_t PEDAL_CAL_OP_SAVE           = 0x03;
inline constexpr uint8_t PEDAL_CAL_OP_RESET_DEFAULTS = 0x04;
inline constexpr uint8_t PEDAL_CAL_OP_QUERY          = 0x05;

// Gear power-limit + accel-response sub-opcodes — byte 1 when byte 0 == SERVICE_ACTION_GEAR_LIMITS
//   For SET_* sub-opcodes byte 2 carries the new percentage (0..100); the
//   value is staged in a RAM-only "pending" set and is NOT applied or
//   persisted until SAVE.  SET/SAVE/RESET require the STM32 in STANDBY.
//   0x01 SET_D2          Stage pending D2 power limit  (byte 2 = percent)
//   0x02 SET_D1          Stage pending D1 power limit  (byte 2 = percent)
//   0x03 SET_R           Stage pending R  power limit  (byte 2 = percent)
//   0x04 SAVE            Validate pending set + persist to STM32 flash + apply
//   0x05 RESET_DEFAULTS  Restore defaults (power 100/60/60, response 100/70/40)
//   0x06 QUERY           Request a burst of 0x30D telemetry (power + response)
//   0x07 SET_D2_RESPONSE Stage pending D2 accel response (byte 2 = percent)
//   0x08 SET_D1_RESPONSE Stage pending D1 accel response (byte 2 = percent)
//   0x09 SET_R_RESPONSE  Stage pending R  accel response (byte 2 = percent)
inline constexpr uint8_t GEAR_LIMIT_OP_SET_D2          = 0x01;
inline constexpr uint8_t GEAR_LIMIT_OP_SET_D1          = 0x02;
inline constexpr uint8_t GEAR_LIMIT_OP_SET_R           = 0x03;
inline constexpr uint8_t GEAR_LIMIT_OP_SAVE            = 0x04;
inline constexpr uint8_t GEAR_LIMIT_OP_RESET_DEFAULTS  = 0x05;
inline constexpr uint8_t GEAR_LIMIT_OP_QUERY           = 0x06;
inline constexpr uint8_t GEAR_LIMIT_OP_SET_D2_RESPONSE = 0x07;
inline constexpr uint8_t GEAR_LIMIT_OP_SET_D1_RESPONSE = 0x08;
inline constexpr uint8_t GEAR_LIMIT_OP_SET_R_RESPONSE  = 0x09;

// Steering-Z dual-reference sub-opcodes — byte 1 when byte 0 == SERVICE_ACTION_STEERING_Z
//   PB5 stays the primary/safety center reference; Z is secondary precision.
//   0x01 QUERY      Request a 1 s burst of 0x30E telemetry at 10 Hz
//   0x02 CALIBRATE  Recompute + persist the Z↔center offset.  Requires PB5 to
//                   currently detect center AND the STM32 in BOOT/STANDBY.
//   0x03 CLEAR      Clear the stored Z calibration (PB5 center kept).  Requires
//                   BOOT/STANDBY; the HMI must enforce a double-confirm.
inline constexpr uint8_t STEER_Z_OP_QUERY     = 0x01;
inline constexpr uint8_t STEER_Z_OP_CALIBRATE = 0x02;
inline constexpr uint8_t STEER_Z_OP_CLEAR     = 0x03;

// EPS parameter tuning sub-opcodes — byte 1 when byte 0 == SERVICE_ACTION_EPS_PARAMS (0xF9)
// SET_PARAM applies immediately to the STM32 RAM copy (real-time tuning; no state gate).
// SAVE and RESET require the STM32 in STANDBY.
//   0x01 SET_PARAM  Immediate: byte2=param_id, bytes3-6=float LE value
//   0x02 SAVE       Persist active RAM copy to STM32 flash (STANDBY only)
//   0x03 RESET      Revert to compiled defaults, RAM only (STANDBY only)
//   0x04 QUERY      Request a 1 s burst of 0x30F telemetry at 10 Hz
inline constexpr uint32_t DIAG_EPS_PARAMS        = 0x30F;  // STM32→ESP32, DLC 8, on-demand
inline constexpr uint8_t  SERVICE_ACTION_EPS_PARAMS = 0xF9;
inline constexpr uint8_t  EPS_PARAM_OP_SET_PARAM  = 0x01;
inline constexpr uint8_t  EPS_PARAM_OP_SAVE       = 0x02;
inline constexpr uint8_t  EPS_PARAM_OP_RESET      = 0x03;
inline constexpr uint8_t  EPS_PARAM_OP_QUERY      = 0x04;

// EPS parameter IDs (eps_param_id_t mirror — MUST match Core/Inc/eps_params.h)
inline constexpr uint8_t EPS_PARAM_ASSIST_STRENGTH  = 0;
inline constexpr uint8_t EPS_PARAM_CENTER_STRENGTH  = 1;
inline constexpr uint8_t EPS_PARAM_DAMPING          = 2;
inline constexpr uint8_t EPS_PARAM_FRICTION_COMP    = 3;
inline constexpr uint8_t EPS_PARAM_COAST_BAND_PCT   = 4;
inline constexpr uint8_t EPS_PARAM_MIN_DRIVE_PCT    = 5;
inline constexpr uint8_t EPS_PARAM_ASSIST_VS_SPEED  = 6;
inline constexpr uint8_t EPS_PARAM_RETURN_VS_SPEED  = 7;
inline constexpr uint8_t EPS_PARAM_DEADBAND_DEG     = 8;
inline constexpr uint8_t EPS_PARAM_MAX_PWM_PCT      = 9;
inline constexpr uint8_t EPS_PARAM_SLEW_RATE_PCT    = 10;
inline constexpr uint8_t EPS_PARAM_CENTER_OFFSET_DEG = 11;
inline constexpr uint8_t EPS_PARAM_COUNT            = 12;

// Gear power-limit valid ranges (percent) — MUST mirror gear_limits_store.h
// on the STM32 so the HMI never sends a value the firmware would reject.
inline constexpr uint8_t GEAR_LIMIT_D2_MIN_PCT = 30;
inline constexpr uint8_t GEAR_LIMIT_D2_MAX_PCT = 100;
inline constexpr uint8_t GEAR_LIMIT_D1_MIN_PCT = 20;
inline constexpr uint8_t GEAR_LIMIT_D1_MAX_PCT = 100;
inline constexpr uint8_t GEAR_LIMIT_R_MIN_PCT  = 10;
inline constexpr uint8_t GEAR_LIMIT_R_MAX_PCT  = 60;
inline constexpr uint8_t GEAR_LIMIT_D2_DEFAULT_PCT = 100;
inline constexpr uint8_t GEAR_LIMIT_D1_DEFAULT_PCT = 60;
inline constexpr uint8_t GEAR_LIMIT_R_DEFAULT_PCT  = 60;

// Gear accel-response valid ranges (percent) — MUST mirror gear_limits_store.h.
// The response factor softens (never amplifies) demand, so every max is <=100.
inline constexpr uint8_t GEAR_RESPONSE_D2_MIN_PCT = 50;
inline constexpr uint8_t GEAR_RESPONSE_D2_MAX_PCT = 100;
inline constexpr uint8_t GEAR_RESPONSE_D1_MIN_PCT = 30;
inline constexpr uint8_t GEAR_RESPONSE_D1_MAX_PCT = 100;
inline constexpr uint8_t GEAR_RESPONSE_R_MIN_PCT  = 20;
inline constexpr uint8_t GEAR_RESPONSE_R_MAX_PCT  = 80;
inline constexpr uint8_t GEAR_RESPONSE_D2_DEFAULT_PCT = 100;
inline constexpr uint8_t GEAR_RESPONSE_D1_DEFAULT_PCT = 70;
inline constexpr uint8_t GEAR_RESPONSE_R_DEFAULT_PCT  = 40;

// -------------------------------------------------------------------------
// Drive-tuning configuration (CAN_CONTRACT_FINAL.md §4.20) — ESP32 → STM32
//   SERVICE_CMD (0x110) byte0 = SERVICE_ACTION_DRIVE_TUNING (0xFA),
//   byte1 = sub-opcode, bytes2-3 = uint16 LE value for SET_* ops.
//   STM32 replies with CMD_ACK (0x103, cmd_id_low = 0x10) and, after QUERY,
//   a DIAG_DRIVE_TUNING (0x310) field-stream burst.
// -------------------------------------------------------------------------
inline constexpr uint32_t DIAG_DRIVE_TUNING        = 0x310;  // STM32→ESP32, DLC 8, on-demand burst
inline constexpr uint8_t  SERVICE_ACTION_DRIVE_TUNING = 0xFA;
// Sub-opcodes (byte 1) — MUST mirror Core/Inc/can_handler.h DRIVE_TUNE_OP_*.
inline constexpr uint8_t DRIVE_TUNE_OP_SET_ACCEL_RAMP   = 0x01;  // DLC 4
inline constexpr uint8_t DRIVE_TUNE_OP_SET_BRAKE_RAMP   = 0x02;  // DLC 4
inline constexpr uint8_t DRIVE_TUNE_OP_SET_REVERSE_RAMP = 0x03;  // DLC 4
inline constexpr uint8_t DRIVE_TUNE_OP_SET_CREEP_ENABLE = 0x04;  // DLC 4
inline constexpr uint8_t DRIVE_TUNE_OP_SET_CREEP_POWER  = 0x05;  // DLC 4
inline constexpr uint8_t DRIVE_TUNE_OP_SET_CREEP_DELAY  = 0x06;  // DLC 4
inline constexpr uint8_t DRIVE_TUNE_OP_SAVE             = 0x07;  // DLC 2 (STANDBY)
inline constexpr uint8_t DRIVE_TUNE_OP_RESET_DEFAULTS   = 0x08;  // DLC 2 (STANDBY)
inline constexpr uint8_t DRIVE_TUNE_OP_QUERY            = 0x09;  // DLC 2 (read-only)
// Telemetry field ids (0x310 byte 1) — MUST mirror DRIVE_TUNE_FIELD_*.
inline constexpr uint8_t DRIVE_TUNE_FIELD_ACCEL_RAMP   = 0x01;
inline constexpr uint8_t DRIVE_TUNE_FIELD_BRAKE_RAMP   = 0x02;
inline constexpr uint8_t DRIVE_TUNE_FIELD_REVERSE_RAMP = 0x03;
inline constexpr uint8_t DRIVE_TUNE_FIELD_CREEP_ENABLE = 0x04;
inline constexpr uint8_t DRIVE_TUNE_FIELD_CREEP_POWER  = 0x05;
inline constexpr uint8_t DRIVE_TUNE_FIELD_CREEP_DELAY  = 0x06;
inline constexpr uint8_t DRIVE_TUNE_FIELD_COUNT        = 6;
// Hard validation ranges — MUST mirror Core/Inc/drive_tuning_store.h so the
// HMI never sends a value the firmware would reject (reject-keep-previous).
inline constexpr uint16_t DRIVE_ACCEL_RAMP_MIN   = 1;
inline constexpr uint16_t DRIVE_ACCEL_RAMP_MAX   = 200;
inline constexpr uint16_t DRIVE_BRAKE_RAMP_MIN   = 1;
inline constexpr uint16_t DRIVE_BRAKE_RAMP_MAX   = 200;
inline constexpr uint16_t DRIVE_REVERSE_RAMP_MIN = 1;
inline constexpr uint16_t DRIVE_REVERSE_RAMP_MAX = 200;
inline constexpr uint16_t DRIVE_CREEP_POWER_MIN  = 0;
inline constexpr uint16_t DRIVE_CREEP_POWER_MAX  = 20;
inline constexpr uint16_t DRIVE_CREEP_DELAY_MIN  = 0;
inline constexpr uint16_t DRIVE_CREEP_DELAY_MAX  = 5000;
inline constexpr uint16_t DRIVE_ACCEL_RAMP_DEFAULT   = 50;
inline constexpr uint16_t DRIVE_BRAKE_RAMP_DEFAULT   = 100;
inline constexpr uint16_t DRIVE_REVERSE_RAMP_DEFAULT = 50;
inline constexpr uint16_t DRIVE_CREEP_ENABLE_DEFAULT = 1;
inline constexpr uint16_t DRIVE_CREEP_POWER_DEFAULT  = 8;
inline constexpr uint16_t DRIVE_CREEP_DELAY_DEFAULT  = 0;

// -------------------------------------------------------------------------
// Battery-limit configuration (CAN_CONTRACT_FINAL.md §4.21) — ESP32 → STM32
//   SERVICE_CMD (0x110) byte0 = SERVICE_ACTION_BATTERY_LIMITS (0xFB),
//   byte1 = sub-opcode, bytes2-3 = uint16 LE value (centivolts = V×100, or ms
//   for the filter) for SET_* ops.  STM32 replies with CMD_ACK (0x103,
//   cmd_id_low = 0x10) and, after QUERY, a DIAG_BATTERY_LIMITS (0x311) burst.
// -------------------------------------------------------------------------
inline constexpr uint32_t DIAG_BATTERY_LIMITS         = 0x311;  // STM32→ESP32, DLC 8, on-demand burst
inline constexpr uint8_t  SERVICE_ACTION_BATTERY_LIMITS = 0xFB;
// Sub-opcodes (byte 1) — MUST mirror Core/Inc/can_handler.h BATT_LIM_OP_*.
inline constexpr uint8_t BATT_LIM_OP_SET_WARNING    = 0x01;  // DLC 4
inline constexpr uint8_t BATT_LIM_OP_SET_LIMIT      = 0x02;  // DLC 4
inline constexpr uint8_t BATT_LIM_OP_SET_CUTOFF     = 0x03;  // DLC 4
inline constexpr uint8_t BATT_LIM_OP_SET_RECOVERY   = 0x04;  // DLC 4
inline constexpr uint8_t BATT_LIM_OP_SET_FILTER     = 0x05;  // DLC 4
inline constexpr uint8_t BATT_LIM_OP_SAVE           = 0x06;  // DLC 2 (STANDBY)
inline constexpr uint8_t BATT_LIM_OP_RESET_DEFAULTS = 0x07;  // DLC 2 (STANDBY)
inline constexpr uint8_t BATT_LIM_OP_QUERY          = 0x08;  // DLC 2 (read-only)
// Telemetry field ids (0x311 byte 1) — MUST mirror BATT_LIM_FIELD_*.
inline constexpr uint8_t BATT_LIM_FIELD_WARNING  = 0x01;
inline constexpr uint8_t BATT_LIM_FIELD_LIMIT    = 0x02;
inline constexpr uint8_t BATT_LIM_FIELD_CUTOFF   = 0x03;
inline constexpr uint8_t BATT_LIM_FIELD_RECOVERY = 0x04;
inline constexpr uint8_t BATT_LIM_FIELD_FILTER   = 0x05;
inline constexpr uint8_t BATT_LIM_FIELD_COUNT    = 5;
// Hard validation ranges (centivolts / ms) — MUST mirror
// Core/Inc/battery_limits_store.h.  Coherence (Warning/Limit/Recovery > Cutoff,
// Warning/Limit <= OV) is also enforced locally before SAVE (FASE 7).
inline constexpr uint16_t BATT_OV_WARNING_CV   = 3000;
inline constexpr uint16_t BATT_WARNING_MIN_CV  = 1500;
inline constexpr uint16_t BATT_WARNING_MAX_CV  = 3000;
inline constexpr uint16_t BATT_LIMIT_MIN_CV    = 1500;
inline constexpr uint16_t BATT_LIMIT_MAX_CV    = 3000;
inline constexpr uint16_t BATT_CUTOFF_MIN_CV   = 1400;
inline constexpr uint16_t BATT_CUTOFF_MAX_CV   = 2400;
inline constexpr uint16_t BATT_RECOVERY_MIN_CV = 1400;
inline constexpr uint16_t BATT_RECOVERY_MAX_CV = 2900;
inline constexpr uint16_t BATT_FILTER_MIN_MS   = 0;
inline constexpr uint16_t BATT_FILTER_MAX_MS   = 5000;
inline constexpr uint16_t BATT_WARNING_DEFAULT_CV  = 2000;
inline constexpr uint16_t BATT_LIMIT_DEFAULT_CV    = 2000;
inline constexpr uint16_t BATT_CUTOFF_DEFAULT_CV   = 1800;
inline constexpr uint16_t BATT_RECOVERY_DEFAULT_CV = 1850;
inline constexpr uint16_t BATT_FILTER_DEFAULT_MS   = 0;

} // namespace can

#endif // CAN_IDS_H
