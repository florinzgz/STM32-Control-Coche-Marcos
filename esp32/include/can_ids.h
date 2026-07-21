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
inline constexpr uint32_t STATUS_PEDAL           = 0x20B;    // DLC 4, 100 ms   b0=Hall pedal % ; b1=fault flags(bit0 plausible,bit1 contradictory) ; b2-3=raw ADC LE (telemetry only)
inline constexpr uint32_t STATUS_WHEEL_EFFORT    = 0x20C;    // DLC 4, 100 ms   per-wheel FINAL applied PWM % (b0=FL,b1=FR,b2=RL,b3=RR). Real applied effort, NOT the 0x205 ABS/TCS limit.
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
inline constexpr uint32_t DIAG_I2C               = 0x309;   // STM32→ESP32, DLC 8, 1000 ms — I2C topology diag: mux + per-channel INA226 health (byte5=expected mask, byte6=i2c_last_read_ms, byte7=reserved)
inline constexpr uint32_t DIAG_CAN_META          = 0x30A;   // STM32→ESP32, DLC 8, 1000 ms — CAN/0x309 delivery meta-diag (call/tick/tx-ok/err/fifo-drops)
inline constexpr uint32_t DIAG_I2C_SCAN          = 0x30B;   // STM32→ESP32, DLC 8, on-demand — I2C service-mode scan (mux/INA probe, SDA/SCL levels, recovery)
inline constexpr uint32_t DIAG_FDCAN             = 0x30C;   // STM32→ESP32, DLC 6, on-demand — FDCAN error-counter dump (TEC/REC/LEC/state)
inline constexpr uint32_t DIAG_GEAR_LIMITS       = 0x30D;   // STM32→ESP32, DLC 8, on-demand (10 Hz × 1 s after QUERY) — gear power-limit telemetry
inline constexpr uint32_t DIAG_STEERING_Z        = 0x30E;   // STM32→ESP32, DLC 8, on-demand (10 Hz × 1 s after QUERY) — PB5 + encoder-Z dual center-reference diagnostic
inline constexpr uint32_t SERVICE_CMD            = 0x110;   // ESP32→STM32, DLC 2, on-demand
inline constexpr uint32_t CMD_SENSOR_MAP_TEMP    = 0x112;   // ESP32→STM32, DLC 5, on-demand  DS18B20 physIdx→role mapping

inline constexpr uint16_t PEDCAL_REJECT_NOT_STANDBY           = 0x0001u;
inline constexpr uint16_t PEDCAL_REJECT_STARTUP_NOT_INHIBITED = 0x0002u;
inline constexpr uint16_t PEDCAL_REJECT_PEDAL_NOT_RELEASED    = 0x0004u;
inline constexpr uint16_t PEDCAL_REJECT_PEDAL_NOT_PLAUSIBLE   = 0x0008u;
inline constexpr uint16_t PEDCAL_REJECT_WHEELS_MOVING         = 0x0010u;
inline constexpr uint16_t PEDCAL_REJECT_PENDING_INCOMPLETE    = 0x0020u;
inline constexpr uint16_t PEDCAL_REJECT_MIN_GT_MAX            = 0x0040u;
inline constexpr uint16_t PEDCAL_REJECT_RANGE_TOO_SMALL       = 0x0080u;
inline constexpr uint16_t PEDCAL_REJECT_MAX_TOO_HIGH          = 0x0100u;
inline constexpr uint16_t PEDCAL_REJECT_RANGE_INVALID         = 0x0200u;
inline constexpr uint16_t PEDCAL_REJECT_FLASH_ERROR           = 0x0400u;
inline constexpr uint16_t PEDCAL_REJECT_SAMPLE_UNSTABLE       = 0x0800u;
inline constexpr uint16_t PEDCAL_REJECT_CAPTURE_TIMEOUT       = 0x1000u;
inline constexpr uint16_t PEDCAL_REJECT_CAPTURE_BUSY          = 0x2000u;

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
// The productive calibration is a single guided PedalCalSession FSM; HMI
// buttons map onto Begin / RequestSave / Abort (audit P5):
//   0x01 CAPTURE_MIN    BEGIN the guided session (auto-captures MIN/MAX)
//   0x02 CAPTURE_MAX    Advisory only — MAX auto-captures on a pressed pedal
//   0x03 SAVE           RequestSave: validate + persist + apply + readback verify
//   0x04 RESET_DEFAULTS Restore compile-time defaults (50 / 4000)   pedal direct 0–3.3 V, no divider
//   0x05 QUERY          Request a 1 s burst of 0x308/0x319 telemetry at 10 Hz
//   0x06 ABORT          Cancel the running session (operator abort)
inline constexpr uint8_t PEDAL_CAL_OP_CAPTURE_MIN    = 0x01;
inline constexpr uint8_t PEDAL_CAL_OP_CAPTURE_MAX    = 0x02;
inline constexpr uint8_t PEDAL_CAL_OP_SAVE           = 0x03;
inline constexpr uint8_t PEDAL_CAL_OP_RESET_DEFAULTS = 0x04;
inline constexpr uint8_t PEDAL_CAL_OP_QUERY          = 0x05;
inline constexpr uint8_t PEDAL_CAL_OP_ABORT          = 0x06;

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
inline constexpr uint16_t BATT_WARNING_DEFAULT_CV  = 1800;
inline constexpr uint16_t BATT_LIMIT_DEFAULT_CV    = 1700;
inline constexpr uint16_t BATT_CUTOFF_DEFAULT_CV   = 1600;
inline constexpr uint16_t BATT_RECOVERY_DEFAULT_CV = 1700;
inline constexpr uint16_t BATT_FILTER_DEFAULT_MS   = 500;

// -------------------------------------------------------------------------
// Boot/Reset Diagnostic (0x312) — STM32→ESP32, 1 Hz, DLC 8
//   Byte 0-3: HAL_GetTick() uptime in ms (uint32 LE)
//   Byte 4:   RCC reset-cause bitmask (mirrors RESET_CAUSE_* in STM32 main.h)
//               bit0 = POWERON   bit1 = SOFTWARE  bit2 = IWDG
//               bit3 = WWDG      bit4 = BROWNOUT  bit5 = PIN
//   Byte 5:   tx_queue_depth      — CAN software TX ring occupancy now (0..31)
//   Byte 6:   tx_queue_depth_max  — CAN software TX ring high-water mark (0..31)
//   Byte 7:   reserved (0)
// -------------------------------------------------------------------------
inline constexpr uint32_t DIAG_BOOT_RESET = 0x312;  // STM32→ESP32, DLC 8, 1000 ms — boot/reset diagnostic

// Reset-cause bit masks (byte 4 of DIAG_BOOT_RESET) — mirror RESET_CAUSE_* in main.h
inline constexpr uint8_t RESET_CAUSE_POWERON   = (1U << 0);
inline constexpr uint8_t RESET_CAUSE_SOFTWARE  = (1U << 1);
inline constexpr uint8_t RESET_CAUSE_IWDG      = (1U << 2);
inline constexpr uint8_t RESET_CAUSE_WWDG      = (1U << 3);
inline constexpr uint8_t RESET_CAUSE_BROWNOUT  = (1U << 4);
inline constexpr uint8_t RESET_CAUSE_PIN       = (1U << 5);

// -------------------------------------------------------------------------
// Per-wheel Speed-Sensor Fault-Reason Diagnostic (0x313) — STM32→ESP32, 1 Hz, DLC 8
//   Byte 0-3: reason FL/FR/RL/RR (WHEEL_DIAG_REASON_* code, 0-8).  While a
//             channel fault is latched this carries the reason captured at
//             latch time (the culprit), not the self-healed live reason.
//   Byte 4:   reason STEER/CENTER — WHEEL_DIAG_REASON_DISABLED_STATE when the
//             steering-encoder module is off (encoder unwired), else OK
//   Byte 5:   gpio_level_mask  (bit0 FL, bit1 FR, bit2 RL, bit3 RR, bit4 STEER)
//   Byte 6:   active_fault_mask (bit0 FL, bit1 FR, bit2 RL, bit3 RR, bit4 STEER)
//   Byte 7:   flags/sequence (bit0 powertrain_engaged, bit1 manual_movement,
//             bit2 wheel_fault_debouncing, bit3 wheel_fault_latched,
//             bits4-7 sequence counter)
// Mirrors Core/Src/can_handler.c CAN_SendWheelSensorDiag() / WheelDiag_t.
// -------------------------------------------------------------------------
inline constexpr uint32_t DIAG_WHEEL_SENSOR = 0x313;  // STM32→ESP32, DLC 8, 1000 ms — per-wheel fault reason

// Wheel-diagnostic reason codes (0x313 bytes 0-4) — MUST mirror WheelDiag_t
// in Core/Inc/safety_system.h (0-7) plus the UNKNOWN wire code (8).
inline constexpr uint8_t WHEEL_DIAG_REASON_OK              = 0;
inline constexpr uint8_t WHEEL_DIAG_REASON_NO_PULSE       = 1;
inline constexpr uint8_t WHEEL_DIAG_REASON_STUCK_HIGH     = 2;
inline constexpr uint8_t WHEEL_DIAG_REASON_STUCK_LOW      = 3;
inline constexpr uint8_t WHEEL_DIAG_REASON_MISMATCH       = 4;
inline constexpr uint8_t WHEEL_DIAG_REASON_IMPOSSIBLE_RATE = 5;
inline constexpr uint8_t WHEEL_DIAG_REASON_MANUAL_MOVEMENT = 6;
inline constexpr uint8_t WHEEL_DIAG_REASON_DISABLED_STATE  = 7;
inline constexpr uint8_t WHEEL_DIAG_REASON_UNKNOWN         = 8;

// Flag bits (0x313 byte 7 low nibble).
inline constexpr uint8_t WHEEL_DIAG_FLAG_POWERTRAIN   = (1U << 0);
inline constexpr uint8_t WHEEL_DIAG_FLAG_MANUAL       = (1U << 1);
inline constexpr uint8_t WHEEL_DIAG_FLAG_DEBOUNCING   = (1U << 2);
inline constexpr uint8_t WHEEL_DIAG_FLAG_LATCHED      = (1U << 3);

// Per-wheel VALID (accepted) pulse-count diagnostic (0x314) — STM32→ESP32, 1 Hz, DLC 8.
// Bytes 0-1/2-3/4-5/6-7 = FL/FR/RL/RR accepted-edge counts (uint16 LE, saturated
// 0xFFFF).  VALID = edges that PASSED the DWT 200 us pre-filter and drive the
// speed/distance estimate, as opposed to the REJECTED (bounce/EMI) counts in
// 0x306 (DIAG_DEBOUNCE).  A full wheel revolution ≈ 6 valid pulses. Report-only.
inline constexpr uint32_t DIAG_WHEEL_PULSES = 0x314;  // STM32→ESP32, DLC 8, 1000 ms — per-wheel valid pulse counts

// MOTION_INHIBIT_REASON diagnostic (0x315) — STM32→ESP32, 10 Hz, DLC 8.
// Instrumentation only: explains why the traction chain is (or is not)
// producing torque.  Bytes: 0-1 reason bitfield (uint16 LE), 2 system state,
// 3 gear, 4 operator demand % (int8), 5 effective demand % (int8), 6 max final
// PWM duty % (uint8), 7 flags (bit0 power_ready, bit1 obstacle_forward_blocked,
// bits2-3 relay-seq phase [0 idle,1 in-progress,2 complete] — commanded
// GPIO/sequencer state only, NO physical relay-contact feedback,
// bits4-7 degraded level).
inline constexpr uint32_t DIAG_MOTION_INHIBIT = 0x315;  // STM32→ESP32, DLC 8, 100 ms — motion-inhibit reason

// MOTION_INHIBIT_REASON bits (0x315 bytes 0-1) — mirror of Core/Inc/motion_inhibit.h
inline constexpr uint16_t MOTION_INHIBIT_NONE            = 0x0000;
inline constexpr uint16_t MOTION_INHIBIT_STATE_SAFE      = 0x0001;
inline constexpr uint16_t MOTION_INHIBIT_STATE_ERROR     = 0x0002;
inline constexpr uint16_t MOTION_INHIBIT_POWER_NOT_READY = 0x0004;
inline constexpr uint16_t MOTION_INHIBIT_GEAR_PARK       = 0x0008;
inline constexpr uint16_t MOTION_INHIBIT_GEAR_NEUTRAL    = 0x0010;
inline constexpr uint16_t MOTION_INHIBIT_NO_DEMAND       = 0x0020;
inline constexpr uint16_t MOTION_INHIBIT_DEMAND_ZEROED   = 0x0040;
inline constexpr uint16_t MOTION_INHIBIT_OBSTACLE_BLOCK  = 0x0080;
inline constexpr uint16_t MOTION_INHIBIT_PWM_ZERO        = 0x0100;
inline constexpr uint16_t MOTION_INHIBIT_TORQUE_LIMITED  = 0x0200;
inline constexpr uint16_t MOTION_INHIBIT_STARTUP_INHIBIT = 0x0400;
inline constexpr uint16_t MOTION_INHIBIT_PEDAL_FAULT     = 0x0800;
inline constexpr uint16_t MOTION_INHIBIT_SAFETY_SCALE_ZERO = 0x1000;
inline constexpr uint16_t MOTION_INHIBIT_BATTERY_CUTOFF  = 0x2000;
inline constexpr uint16_t MOTION_INHIBIT_THERMAL_OVERCURRENT = 0x4000;
inline constexpr uint16_t MOTION_INHIBIT_SERVICE_DISABLED = 0x8000;

// Relay-sequence phase (0x315 byte 7 bits 2-3) — commanded state only.
inline constexpr uint8_t MOTION_INHIBIT_RELAY_SEQ_IDLE        = 0;
inline constexpr uint8_t MOTION_INHIBIT_RELAY_SEQ_IN_PROGRESS = 1;
inline constexpr uint8_t MOTION_INHIBIT_RELAY_SEQ_COMPLETE    = 2;

// =====================================================================
// Steering-homing (centering) diagnostic (0x316) — STM32→ESP32, 1 Hz, DLC 8.
// Mirrors Core/Inc/steering_centering_frame.h.  Instrumentation only: it
// explains WHY the automatic centering sweep did/did not progress so the HMI
// can render "DIRECCIÓN NO SE MUEVE" with the real cause instead of "Error 8".
//   Byte 0   diag reason (STEER_DIAG_* below, 0..15)
//   Byte 1   FSM state (low nibble) | motor owner (high nibble)
//   Byte 2   flags: b0 PB5 raw, b1 PB5 debounced, b2 PB5 already-active,
//            b3 PC12 relay commanded, b4 power ready, b5 PC4 EN commanded,
//            b6 encoder fault, b7 restored-from-flash
//   Byte 3   system state (low nibble) | b4 module disabled | b5 fault
//            latched | b6 pwm requested (>0)
//   Byte 4-5 PWM real (max CCR PA6/PA7, uint16 LE)
//   Byte 6-7 encoder delta from sweep origin (int16 LE)
// =====================================================================
inline constexpr uint32_t DIAG_STEERING_CENTERING = 0x316;  // STM32→ESP32, DLC 8, 1000 ms

// Steering diagnostic reason codes (0x316 byte 0) — mirror SteerDiagReason_t.
inline constexpr uint8_t STEER_DIAG_OK                        = 0;
inline constexpr uint8_t STEER_DIAG_RESTORED_FROM_FLASH       = 1;
inline constexpr uint8_t STEER_DIAG_WAITING_POWER             = 2;
inline constexpr uint8_t STEER_DIAG_CENTER_SENSOR_ACTIVE      = 3;
inline constexpr uint8_t STEER_DIAG_SWEEP_LEFT                = 4;
inline constexpr uint8_t STEER_DIAG_SWEEP_RIGHT               = 5;
inline constexpr uint8_t STEER_DIAG_NO_ENCODER_MOVEMENT       = 6;
inline constexpr uint8_t STEER_DIAG_ENCODER_FAULT             = 7;
inline constexpr uint8_t STEER_DIAG_RANGE_EXCEEDED            = 8;
inline constexpr uint8_t STEER_DIAG_TOTAL_TIMEOUT             = 9;
inline constexpr uint8_t STEER_DIAG_LOST_HOMING_STATE         = 10;
inline constexpr uint8_t STEER_DIAG_RELAY_NOT_READY           = 11;
inline constexpr uint8_t STEER_DIAG_MODULE_DISABLED           = 12;
inline constexpr uint8_t STEER_DIAG_ABORTED_SAFE             = 13;
inline constexpr uint8_t STEER_DIAG_ABORTED_ERROR            = 14;
inline constexpr uint8_t STEER_DIAG_UNKNOWN                   = 15;

// Flag bits (0x316 byte 2).
inline constexpr uint8_t STEER_DIAG_FLAG_PB5_RAW        = 1 << 0;
inline constexpr uint8_t STEER_DIAG_FLAG_PB5_DEBOUNCED  = 1 << 1;
inline constexpr uint8_t STEER_DIAG_FLAG_PB5_ACTIVE     = 1 << 2;
inline constexpr uint8_t STEER_DIAG_FLAG_RELAY_PC12     = 1 << 3;
inline constexpr uint8_t STEER_DIAG_FLAG_POWER_READY    = 1 << 4;
inline constexpr uint8_t STEER_DIAG_FLAG_EN_PC4         = 1 << 5;
inline constexpr uint8_t STEER_DIAG_FLAG_ENCODER_FAULT  = 1 << 6;
inline constexpr uint8_t STEER_DIAG_FLAG_RESTORED_FLASH = 1 << 7;

// System-state / status bits (0x316 byte 3).
inline constexpr uint8_t STEER_DIAG_STATE_MASK          = 0x0F;
inline constexpr uint8_t STEER_DIAG_STATUS_MODULE_DISABLED = 1 << 4;
inline constexpr uint8_t STEER_DIAG_STATUS_FAULT_LATCHED   = 1 << 5;
inline constexpr uint8_t STEER_DIAG_STATUS_PWM_REQUESTED   = 1 << 6;

// -------------------------------------------------------------------------
// 0x317 DIAG_RELAY_HEALTH — traction relay / current-sense health (Problem 3)
// STM32→ESP32, DLC 8, 1000 ms.  Evidence-graded cause so the HMI shows
// CURRENT SENSE INVALID vs RELAY OPEN SUSPECTED instead of a bare RELAY OPEN.
// Reason codes mirror RelayDiagReason_t in Core/Inc/relay_health_diag.h.
// -------------------------------------------------------------------------
inline constexpr uint32_t DIAG_RELAY_HEALTH = 0x317;  // STM32→ESP32, DLC 8, 1000 ms

inline constexpr uint8_t RELAY_DIAG_OK                  = 0;
inline constexpr uint8_t RELAY_DIAG_OPEN_CONFIRMED      = 1;
inline constexpr uint8_t RELAY_DIAG_OPEN_SUSPECTED      = 2;
inline constexpr uint8_t RELAY_DIAG_CURRENT_SENSE_INVALID = 3;
inline constexpr uint8_t RELAY_DIAG_SHUNT_OPEN          = 4;
inline constexpr uint8_t RELAY_DIAG_SHUNT_BYPASSED      = 5;
inline constexpr uint8_t RELAY_DIAG_POLARITY_REVERSED   = 6;
inline constexpr uint8_t RELAY_DIAG_DATA_STALE          = 7;
inline constexpr uint8_t RELAY_DIAG_INA_MISSING         = 8;
inline constexpr uint8_t RELAY_DIAG_SCALE_INVALID       = 9;
inline constexpr uint8_t RELAY_DIAG_INCONCLUSIVE        = 10;

// Byte 1 flag bits (mirror RELAY_FRAME_FLAG_* in Core/Inc/relay_health_frame.h).
inline constexpr uint8_t RELAY_DIAG_FLAG_RELAY_CMD     = 1 << 0;
inline constexpr uint8_t RELAY_DIAG_FLAG_SEQ_COMPLETE  = 1 << 1;
inline constexpr uint8_t RELAY_DIAG_FLAG_POWER_READY   = 1 << 2;
inline constexpr uint8_t RELAY_DIAG_FLAG_WHEEL_MOVING  = 1 << 3;
inline constexpr uint8_t RELAY_DIAG_FLAG_CURRENT_VALID = 1 << 4;
inline constexpr uint8_t RELAY_DIAG_FLAG_CURRENT_STALE = 1 << 5;
inline constexpr uint8_t RELAY_DIAG_FLAG_INA_MISSING   = 1 << 6;
inline constexpr uint8_t RELAY_DIAG_FLAG_POLARITY_REV  = 1 << 7;

// -------------------------------------------------------------------------
// 0x318 DIAG_INA_CH5 — steering INA226 (CH5) channel diagnostic (Problem 4)
// STM32→ESP32, DLC 8, 1000 ms.  Separates a genuinely MISSING chip (no ACK)
// from a transport gap ("n/d" = ABSENCE of this frame), a lost config, an
// open/bypassed shunt, or a reversed-polarity wiring fault — and carries a
// SIGNED shunt register so a negative current is never flattened to zero.
// Reason codes mirror Ina226DiagReason_t in Core/Inc/ina226_channel_diag.h.
// -------------------------------------------------------------------------
inline constexpr uint32_t DIAG_INA_CH5 = 0x318;  // STM32→ESP32, DLC 8, 1000 ms

// 0x319 DIAG_PEDAL_CAL_SESSION — guided PedalCalSession status (audit P5).
// STM32→ESP32, DLC 8, on state change + ~10 Hz while active.  Mirrors
// Core/Src/can_handler.c pedalcal_send_session_status().
//   b0 state (PedalCalState), b1 flags (bit0 active, bit1 have_min,
//   bit2 have_max, bit3 completed, bit4 aborted, bit5 entry-ok,
//   bit6 operator-cancel abort, bit7 movement-lock-lost abort),
//   b2-3 reason mask (LE, low 16 bits), b4-5 adc_min (LE), b6-7 adc_max (LE).
// The extended OPERATOR / LOCK_LOST abort causes live above bit 15 in the
// firmware reason word and are surfaced ONLY via flag bits 6/7.
inline constexpr uint32_t DIAG_PEDAL_CAL_SESSION = 0x319;

// 0x319 byte-1 flag bits.
inline constexpr uint8_t PEDCAL_SESS_FLAG_ACTIVE        = 0x01u;
inline constexpr uint8_t PEDCAL_SESS_FLAG_HAVE_MIN      = 0x02u;
inline constexpr uint8_t PEDCAL_SESS_FLAG_HAVE_MAX      = 0x04u;
inline constexpr uint8_t PEDCAL_SESS_FLAG_COMPLETED     = 0x08u;
inline constexpr uint8_t PEDCAL_SESS_FLAG_ABORTED       = 0x10u;
inline constexpr uint8_t PEDCAL_SESS_FLAG_ENTRY_OK      = 0x20u;
inline constexpr uint8_t PEDCAL_SESS_FLAG_ABORT_OPERATOR  = 0x40u;
inline constexpr uint8_t PEDCAL_SESS_FLAG_ABORT_LOCK_LOST = 0x80u;

// PedalCalSession state enum (mirror of Core/Inc/pedal_cal_session.h PedalCalState).
inline constexpr uint8_t PEDCAL_SESS_IDLE                 = 0;
inline constexpr uint8_t PEDCAL_SESS_ENTERING             = 1;
inline constexpr uint8_t PEDCAL_SESS_WAIT_RELEASED        = 2;
inline constexpr uint8_t PEDCAL_SESS_CAPTURING_MIN        = 3;
inline constexpr uint8_t PEDCAL_SESS_WAIT_FULL_PRESS      = 4;
inline constexpr uint8_t PEDCAL_SESS_CAPTURING_MAX        = 5;
inline constexpr uint8_t PEDCAL_SESS_WAIT_RELEASE_FOR_SAVE= 6;
inline constexpr uint8_t PEDCAL_SESS_READY_TO_SAVE        = 7;
inline constexpr uint8_t PEDCAL_SESS_SAVING              = 8;
inline constexpr uint8_t PEDCAL_SESS_COMPLETED           = 9;
inline constexpr uint8_t PEDCAL_SESS_ABORTED             = 10;

// PedalCalSession reason bitmask (mirror of pedal_cal_session.h).
inline constexpr uint16_t PEDCAL_SESS_BLOCK_NOT_STANDBY       = 0x0001u;
inline constexpr uint16_t PEDCAL_SESS_BLOCK_GEAR             = 0x0002u;
inline constexpr uint16_t PEDCAL_SESS_BLOCK_WHEELS_MOVING    = 0x0004u;
inline constexpr uint16_t PEDCAL_SESS_BLOCK_PEDAL_IMPLAUSIBLE= 0x0008u;
inline constexpr uint16_t PEDCAL_SESS_BLOCK_CRITICAL_ERROR   = 0x0010u;
inline constexpr uint16_t PEDCAL_SESS_BLOCK_TRACTION_LIVE    = 0x0020u;
inline constexpr uint16_t PEDCAL_SESS_ABORT_SAFE            = 0x0040u;
inline constexpr uint16_t PEDCAL_SESS_ABORT_ERROR           = 0x0080u;
inline constexpr uint16_t PEDCAL_SESS_ABORT_EMERGENCY       = 0x0100u;
inline constexpr uint16_t PEDCAL_SESS_ABORT_MOVEMENT        = 0x0200u;
inline constexpr uint16_t PEDCAL_SESS_ABORT_CAN_LOSS        = 0x0400u;
inline constexpr uint16_t PEDCAL_SESS_ABORT_TIMEOUT         = 0x0800u;
inline constexpr uint16_t PEDCAL_SESS_FAIL_MIN_GE_MAX       = 0x1000u;
inline constexpr uint16_t PEDCAL_SESS_FAIL_RANGE_SMALL      = 0x2000u;
inline constexpr uint16_t PEDCAL_SESS_FAIL_UNSTABLE         = 0x4000u;
inline constexpr uint16_t PEDCAL_SESS_FAIL_READBACK         = 0x8000u;

inline constexpr uint8_t INA_CH5_OK                = 0;
inline constexpr uint8_t INA_CH5_PRESENT_NO_SHUNT  = 1;
inline constexpr uint8_t INA_CH5_POLARITY_REVERSED = 2;
inline constexpr uint8_t INA_CH5_STALE             = 3;
inline constexpr uint8_t INA_CH5_MUX_SELECT_FAIL   = 4;
inline constexpr uint8_t INA_CH5_MISSING           = 5;
inline constexpr uint8_t INA_CH5_WRONG_ID          = 6;
inline constexpr uint8_t INA_CH5_CONFIG_LOST       = 7;
inline constexpr uint8_t INA_CH5_READ_FAIL         = 8;
inline constexpr uint8_t INA_CH5_UNKNOWN           = 9;

// Byte 1 flag bits (mirror INA226_CH5_FLAG_* in Core/Inc/ina226_ch5_frame.h).
inline constexpr uint8_t INA_CH5_FLAG_MUX_OK      = 1 << 0;
inline constexpr uint8_t INA_CH5_FLAG_I2C_ACK     = 1 << 1;
inline constexpr uint8_t INA_CH5_FLAG_IDENTITY_OK = 1 << 2;
inline constexpr uint8_t INA_CH5_FLAG_CONFIG_OK   = 1 << 3;
inline constexpr uint8_t INA_CH5_FLAG_SHUNT_OK    = 1 << 4;
inline constexpr uint8_t INA_CH5_FLAG_BUS_OK      = 1 << 5;
inline constexpr uint8_t INA_CH5_FLAG_POWERED     = 1 << 6;
inline constexpr uint8_t INA_CH5_FLAG_STALE       = 1 << 7;

} // namespace can

#endif // CAN_IDS_H
