/**
  ****************************************************************************
  * @file    can_handler.h
  * @brief   CAN bus handler for ESP32-S3 HMI communication
  *          Protocol: 500 kbps, CAN 2.0A (11-bit IDs)
  ****************************************************************************
  */

#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "can_init_diag.h"
#include <stdbool.h>
#include <stdint.h>

/* CAN test frame ID — used by CAN_TestTransmit() and the Rx test filter */
#define CAN_ID_TEST_FRAME   0x123

/* CAN Message IDs (ESP32 ↔ STM32) */
#define CAN_ID_HEARTBEAT_STM32    0x001  // STM32 → ESP32 (100ms, DLC 6: byte5=relay status)
#define CAN_ID_HEARTBEAT_ESP32    0x011  // ESP32 → STM32 (100ms)
#define CAN_ID_CMD_THROTTLE       0x100  // ESP32 → STM32 (50ms)
#define CAN_ID_CMD_STEERING       0x101  // ESP32 → STM32 (50ms)
#define CAN_ID_CMD_MODE           0x102  // ESP32 → STM32 (on-demand)
#define CAN_ID_CMD_RC_OVERRIDE    0x10A  // ESP32 → STM32 (50ms) RC override demand
                                          // DLC 5: byte0=flags (bit0=override_active),
                                          // byte1=throttle 0..100, byte2..3=int16 LE
                                          // steer 1/10°, byte4=seq.  See rc_arbiter.h.
                                          // 200 ms watchdog → failsafe to local pedal.
#define CAN_ID_CMD_LED            0x120  // ESP32 → STM32 (on-demand) LED relay control
#define CAN_ID_CMD_SYSTEM_SHUTDOWN 0x130 // ESP32 → STM32 (on-demand) pre-power-cut safe-state request
                                          // Payload: empty or 1 byte (ignored).
                                          // Idempotent. Reuses existing safety primitives;
                                          // forces PWM=0, EN=LOW, relays OFF, state=SAFE.
                                          // If the frame never arrives, behaviour is unchanged
                                          // (hardware delay relay still cuts power).
#define CAN_ID_STATUS_SPEED       0x200  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_CURRENT     0x201  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_TEMP        0x202  // STM32 → ESP32 (1000ms)
#define CAN_ID_STATUS_SAFETY      0x203  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_STEERING    0x204  // STM32 → ESP32 (100ms)
#define CAN_ID_STATUS_TRACTION    0x205  // STM32 → ESP32 (100ms) per-wheel ABS/TCS LIMIT (traction PERMITTED, NOT applied torque)
#define CAN_ID_STATUS_TEMP_MAP    0x206  // STM32 → ESP32 (1000ms) explicit temp sensor map
#define CAN_ID_STATUS_BATTERY     0x207  // STM32 → ESP32 (100ms) battery 24V bus current + voltage
#define CAN_ID_OBSTACLE_DISTANCE  0x208  // ESP32 → STM32 (66ms) obstacle distance + zone + health
#define CAN_ID_OBSTACLE_SAFETY    0x209  // ESP32 → STM32 (100ms) obstacle safety state
#define CAN_ID_STATUS_LIGHTS      0x20A  // STM32 → ESP32 (1000ms) LED relay + light state
#define CAN_ID_STATUS_PEDAL       0x20B  // STM32 → ESP32 (100ms) Hall pedal position % (telemetry only)
#define CAN_ID_STATUS_WHEEL_EFFORT 0x20C // STM32 → ESP32 (100ms) per-wheel FINAL PWM duty actually applied to each BTS7960
                                         //   Byte 0: FL final PWM 0-100 %
                                         //   Byte 1: FR final PWM 0-100 %
                                         //   Byte 2: RL final PWM 0-100 %
                                         //   Byte 3: RR final PWM 0-100 %
                                         //   COAST/BRAKE/disabled motor → 0.  This is the REAL applied
                                         //   effort (post pedal/ramp/gear/ABS/TCS/jerk/DRIVE-BRAKE-COAST),
                                         //   NOT the ABS/TCS "permitted" scale of 0x205.
#define CAN_ID_DIAG_ERROR         0x300  // Both directions (on-demand)
#define CAN_ID_SERVICE_FAULTS     0x301  // STM32 → ESP32 (1000ms) fault bitmask
#define CAN_ID_SERVICE_ENABLED    0x302  // STM32 → ESP32 (1000ms) enabled bitmask
#define CAN_ID_SERVICE_DISABLED   0x303  // STM32 → ESP32 (1000ms) disabled bitmask
#define CAN_ID_ERROR_LOG_ENTRY    0x304  // STM32 → ESP32 (on-demand) error log entry
#define CAN_ID_ERROR_LOG_HEADER   0x305  // STM32 → ESP32 (1000ms) error log count + total
#define CAN_ID_DIAG_DEBOUNCE      0x306  // STM32 → ESP32 (1000ms) DWT-debounce filtered counts (4× wheel u16 LE)
#define CAN_ID_DIAG_DEBOUNCE_STEER 0x307 // STM32 → ESP32 (1000ms) DWT-debounce filtered count (steer u32 LE)
#define CAN_ID_DIAG_PEDAL_CAL     0x308  // STM32 → ESP32 (on-demand, 10 Hz for 1 s after QUERY) pedal calibration telemetry
#define CAN_ID_DIAG_I2C           0x309  // STM32 → ESP32 (1000ms) I2C topology diag: mux + per-channel INA226 health
#define CAN_ID_DIAG_CAN_META      0x30A  // STM32 → ESP32 (1000ms) CAN/0x309 delivery meta-diagnostic (additive)
#define CAN_ID_DIAG_I2C_SCAN      0x30B  // STM32 → ESP32 (on-demand) I2C service-mode scan report (additive)
#define CAN_ID_DIAG_FDCAN         0x30C  // STM32 → ESP32 (on-demand) FDCAN error-counter dump (additive)
#define CAN_ID_DIAG_GEAR_LIMITS   0x30D  // STM32 → ESP32 (on-demand, after QUERY) gear power-limit + accel-response telemetry (frame-kind in byte0 bit4)
#define CAN_ID_DIAG_STEERING_Z    0x30E  // STM32 → ESP32 (on-demand, after QUERY) PB5 + encoder-Z dual center-reference diagnostic
#define CAN_ID_DIAG_EPS_PARAMS    0x30F  // STM32 → ESP32 (on-demand, after QUERY) EPS parameter + live-state telemetry
#define CAN_ID_DIAG_DRIVE_TUNING  0x310  // STM32 → ESP32 (on-demand, after QUERY) drive-tuning (ramp/creep) field-stream telemetry
#define CAN_ID_DIAG_BATTERY_LIMITS 0x311 // STM32 → ESP32 (on-demand, after QUERY) battery-limit (voltage threshold) field-stream telemetry
#define CAN_ID_DIAG_BOOT_RESET    0x312  // STM32 → ESP32 (1000ms) boot/reset diagnostic: uptime_ms + RCC reset-cause bitmask
                                         //   Byte 0-3: HAL_GetTick() uptime in ms (uint32 LE)
                                         //   Byte 4:   reset_cause bitmask (RESET_CAUSE_* from main.h)
                                         //             bit0=POWERON  bit1=SOFTWARE bit2=IWDG
                                         //             bit3=WWDG     bit4=BROWNOUT  bit5=PIN
                                         //   Byte 5:   tx_queue_depth (software TX ring occupancy now, 0..31)
                                         //   Byte 6:   tx_queue_depth_max (software TX ring high-water, 0..31)
                                         //   Byte 7:   reserved (0)
#define CAN_ID_DIAG_WHEEL_SENSOR  0x313  // STM32 → ESP32 (1000ms) per-wheel speed-sensor fault-reason diagnostic
                                         //   Byte 0: reason FL   (WheelDiag_t code, 0-8: see WHEEL_DIAG_*)
                                         //   Byte 1: reason FR
                                         //   Byte 2: reason RL
                                         //   Byte 3: reason RR
                                         //   Byte 4: reason STEER/CENTER (reserved, currently 0=OK)
                                         //   Byte 5: gpio_level_mask  bit0 FL,bit1 FR,bit2 RL,bit3 RR,bit4 STEER
                                         //   Byte 6: active_fault_mask bit0 FL,bit1 FR,bit2 RL,bit3 RR,bit4 STEER
                                         //   Byte 7: flags/sequence bit0 powertrain_engaged, bit1 manual_movement,
                                         //           bit2 wheel_fault_debouncing, bit3 wheel_fault_latched,
                                         //           bits4-7 sequence counter (wraps 0-15)
                                         //   Reason codes: 0=OK 1=NO_PULSE 2=STUCK_HIGH 3=STUCK_LOW 4=MISMATCH
                                         //                 5=IMPOSSIBLE_RATE 6=MANUAL_MOVEMENT 7=DISABLED_STATE 8=UNKNOWN
#define CAN_ID_DIAG_WHEEL_PULSES  0x314  // STM32 → ESP32 (1000ms) per-wheel VALID (accepted) pulse counts
                                         //   Byte 0-1: FL valid pulses (uint16 LE, saturated 0xFFFF)
                                         //   Byte 2-3: FR valid pulses (uint16 LE, saturated 0xFFFF)
                                         //   Byte 4-5: RL valid pulses (uint16 LE, saturated 0xFFFF)
                                         //   Byte 6-7: RR valid pulses (uint16 LE, saturated 0xFFFF)
                                         //   Valid = accepted EXTI edges that PASSED the DWT 200us pre-filter
                                         //   (i.e. real wheel pulses used for speed/distance), NOT the REJECTED
                                         //   (bounce/EMI) counts reported by 0x306. Diagnostic only.
#define CAN_ID_DIAG_MOTION_INHIBIT 0x315 // STM32 → ESP32 (100ms) MOTION_INHIBIT_REASON — why the traction
                                         //   chain is (or is not) producing torque.  Instrumentation only.
                                         //   Byte 0-1: reason bitfield (uint16 LE, MOTION_INHIBIT_* bits)
                                         //   Byte 2:   system state (SYS_STATE_*)
                                         //   Byte 3:   current gear (GearPosition_t)
                                         //   Byte 4:   operator demand %   (int8, signed, -100..100)
                                         //   Byte 5:   effective demand %   (int8, signed, after scaling)
                                         //   Byte 6:   max final PWM duty %  (uint8, 0..100)
                                         //   Byte 7:   flags bit0 power_ready, bit1 obstacle_forward_blocked,
                                         //             bits2-3 relay-seq phase (0 idle,1 in-progress,2 complete;
                                         //             commanded GPIO/sequencer state only, no contact feedback),
                                         //             bits4-7 degraded level (0=none,1..3)
#define CAN_ID_DIAG_STEERING_CENTERING 0x316 // STM32 → ESP32 (1000ms) steering homing telemetry —
                                         //   WHY the automatic centering sweep did/did not progress.
                                         //   Instrumentation only (drives nothing).
                                         //   Byte 0:   diag reason (SteerDiagReason_t, 0..15)
                                         //   Byte 1:   FSM state (low nibble, CenteringState_t) |
                                         //             motor owner (high nibble, SteeringMotorOwner_t)
                                         //   Byte 2:   flags bit0 PB5 raw active, bit1 PB5 debounced,
                                         //             bit2 PB5 already-active-at-sweep-start,
                                         //             bit3 PC12 relay commanded, bit4 power ready,
                                         //             bit5 PC4 EN_STEER commanded, bit6 encoder fault,
                                         //             bit7 restored-from-flash
                                         //   Byte 3:   system state (low nibble, STEER_DIAG_SS_*) |
                                         //             bit4 module disabled | bit5 fault latched |
                                         //             bit6 pwm requested (>0)
                                         //   Byte 4-5: PWM real (max CCR PA6/PA7, uint16 LE)
                                         //   Byte 6-7: encoder delta from sweep origin (int16 LE, clamped)
#define CAN_ID_DIAG_RELAY_HEALTH  0x317 // STM32 → ESP32 (1000ms) traction relay / current-sense health.
                                         //   Evidence-graded cause so the HMI shows CURRENT SENSE INVALID
                                         //   vs RELAY OPEN SUSPECTED instead of a bare "RELAY OPEN".
                                         //   Byte 0:   diag reason (RelayDiagReason_t, 0..10)
                                         //   Byte 1:   flags bit0 relay commanded, bit1 seq complete,
                                         //             bit2 power ready, bit3 any wheel moving,
                                         //             bit4 current valid, bit5 current stale,
                                         //             bit6 expected wheel INA missing, bit7 polarity rev
                                         //   Byte 2-3: sum |CH0..3| current in centi-amps (uint16 LE)
                                         //   Byte 4:   throttle % (0..100)
                                         //   Byte 5:   final PWM/traction demand % (0..100)
                                         //   Byte 6-7: current sample age ms (uint16 LE)
#define CAN_ID_DIAG_INA_CH5       0x318 // STM32 → ESP32 (1000ms) steering INA226 (CH5) channel diagnostic.
                                         //   Separates MISSING (no ACK) from n/d (no contract) and keeps a
                                         //   SIGNED shunt so a reversed current is never zeroed.
                                         //   Byte 0:   fault reason (Ina226DiagReason_t, 0..9)
                                         //   Byte 1:   flags bit0 mux select ok, bit1 i2c ack,
                                         //             bit2 identity ok, bit3 config ok, bit4 shunt read ok,
                                         //             bit5 bus read ok, bit6 channel powered, bit7 stale
                                         //   Byte 2-3: raw shunt register (int16 LE, signed two's complement)
                                         //   Byte 4-5: bus voltage mV (uint16 LE)
                                         //   Byte 6-7: sample age ms (uint16 LE, 0xFFFF = never)
#define CAN_ID_DIAG_PEDAL_CAL_SESSION 0x319 // STM32 → ESP32 (on-demand/while active) PedalCalSession state+reason
                                         //   Byte 0:   session state (PedalCalState 0..10)
                                         //   Byte 1:   flags bit0 active, bit1 have_min, bit2 have_max,
                                         //             bit3 completed, bit4 aborted, bit5 entry-guards-ok
                                         //   Byte 2-3: reason bitmask (PEDAL_CAL_SESS_* u16 LE)
                                         //   Byte 4-5: captured adc_min (u16 LE)
                                         //   Byte 6-7: captured adc_max (u16 LE)
#define CAN_ID_DIAG_TRACTION_LIMITS 0x31A // STM32 → ESP32 (1000ms) post-demand traction limiting factors
                                         //   Byte 0: obstacle_scale percent (0..100)
                                         //   Byte 1: degraded traction_cap percent (0..100)
                                         //   Byte 2: brake_release ramp percent (0..100)
                                         //   Byte 3: ObstacleState_t (0 NO_SENSOR,1 NORMAL,2 CONFIRMING,
                                         //           3 ACTIVE,4 CLEARING,5 SENSOR_FAULT)
                                         //   Instrumentation only; drives no control path.
#define CAN_ID_SERVICE_CMD              0x110  // ESP32 → STM32 (on-demand) module control
#define CAN_ID_CMD_SENSOR_MAP_TEMP      0x112  // ESP32 → STM32 (on-demand) DS18B20 physIdx→role map (DLC 5)
#define CAN_ID_CMD_ACK                  0x103  // STM32 → ESP32 (on-demand) command acknowledgment

/* Service command action codes (SERVICE_CMD byte 0) */
#define SERVICE_ACTION_DISABLE             0x00
#define SERVICE_ACTION_ENABLE              0x01
#define SERVICE_ACTION_RELAY_OVERRIDE      0xE0  /* Engineering relay override (byte1=mask) */
#define SERVICE_ACTION_RESET_STEERING_PID  0xF0
#define SERVICE_ACTION_RESET_WHEEL_SENSORS 0xF1
#define SERVICE_ACTION_RESET_INA226_SHUNTS 0xF2
#define SERVICE_ACTION_RESET_TRACTION_FORCE 0xF3
#define SERVICE_ACTION_RESET_STEERING_FORCE 0xF4
#define SERVICE_ACTION_CLEAR_ERROR_LOG     0xFE
#define SERVICE_ACTION_PEDAL_CAL           0xF5  /* Pedal endpoint calibration (byte1 = sub-opcode) */
#define SERVICE_ACTION_I2C_SERVICE         0xF6  /* I2C service-mode scan: probe mux/INA, SDA/SCL levels, recovery */
#define SERVICE_ACTION_GEAR_LIMITS         0xF7  /* Gear power-limit config (byte1 = sub-opcode, byte2 = percent) */
#define SERVICE_ACTION_STEERING_Z          0xF8  /* PB5 + encoder-Z center diagnostic/calibration (byte1 = sub-opcode) */
#define SERVICE_ACTION_EPS_PARAMS          0xF9  /* EPS runtime parameter tuning (byte1 = sub-opcode) → 0x30F        */
#define SERVICE_ACTION_DRIVE_TUNING        0xFA  /* Drive-tuning ramp/creep config (byte1 = sub-opcode) → 0x310      */
#define SERVICE_ACTION_BATTERY_LIMITS      0xFB  /* Battery voltage-threshold config (byte1 = sub-opcode) → 0x311    */
#define SERVICE_ACTION_FACTORY_RESTORE     0xFF

/* ---- I2C service-mode scan terminal phase (0x30B byte 5) ----------------
 * Additive diagnostic: a compact code summarising the terminal I2C condition
 * reached during Sensor_RunI2CServiceScan(), so the HMI can display "TCA
 * MISSING" / "I2C BUS BUSY" / "TCA ACK" instead of a bare "SCAN TIMEOUT".
 * Derived from fields already present in Sensor_I2cScanResult_t.           */
#define I2C_SCAN_PHASE_UNKNOWN      0x00  /* not determined                        */
#define I2C_SCAN_PHASE_BUS_BUSY     0x01  /* SDA idle low — bus stuck (recovery run)*/
#define I2C_SCAN_PHASE_TCA_MISSING  0x02  /* bus idle OK but TCA9548A 0x70 !ACK     */
#define I2C_SCAN_PHASE_TCA_ACK      0x03  /* TCA9548A ACKed (mux present)           */

/* ---- Pedal-calibration sub-opcodes (byte1 when byte0 == 0xF5) ----
 * The productive calibration is a single guided PedalCalSession FSM
 * (pedal_cal_session.c).  HMI buttons map onto Begin / RequestSave / Abort:
 * 0x01 CAPTURE_MIN    BEGIN the guided session (it then captures MIN once the
 *                     pedal is released, prompts a full press, captures MAX,
 *                     and waits released before SAVE).
 * 0x02 CAPTURE_MAX    Advisory only — MAX is captured automatically when the
 *                     pedal is objectively pressed; requests a status burst.
 * 0x03 SAVE           RequestSave: validate + persist + apply + readback-verify
 * 0x04 RESET_DEFAULTS Erase flash slot + restore 50 / 4000
 * 0x05 QUERY          Request a 1 s burst of 0x308 telemetry at 10 Hz
 * 0x06 ABORT          Cancel the running session (operator abort)            */
#define PEDAL_CAL_OP_CAPTURE_MIN    0x01U
#define PEDAL_CAL_OP_CAPTURE_MAX    0x02U
#define PEDAL_CAL_OP_SAVE           0x03U
#define PEDAL_CAL_OP_RESET_DEFAULTS 0x04U
#define PEDAL_CAL_OP_QUERY          0x05U
#define PEDAL_CAL_OP_ABORT          0x06U

/* ---- Pedal-calibration reject-reason bitmask (0x308 diagnostic frame) ---- */
#define PEDCAL_REJECT_NOT_STANDBY            0x0001U
#define PEDCAL_REJECT_STARTUP_NOT_INHIBITED  0x0002U
#define PEDCAL_REJECT_PEDAL_NOT_RELEASED     0x0004U
#define PEDCAL_REJECT_PEDAL_NOT_PLAUSIBLE    0x0008U
#define PEDCAL_REJECT_WHEELS_MOVING          0x0010U
#define PEDCAL_REJECT_PENDING_INCOMPLETE     0x0020U
#define PEDCAL_REJECT_MIN_GT_MAX             0x0040U
#define PEDCAL_REJECT_RANGE_TOO_SMALL        0x0080U
#define PEDCAL_REJECT_MAX_TOO_HIGH           0x0100U
#define PEDCAL_REJECT_RANGE_INVALID          0x0200U
#define PEDCAL_REJECT_FLASH_ERROR            0x0400U
#define PEDCAL_REJECT_SAMPLE_UNSTABLE        0x0800U
#define PEDCAL_REJECT_CAPTURE_TIMEOUT        0x1000U
#define PEDCAL_REJECT_CAPTURE_BUSY           0x2000U

/* ---- Gear power-limit + accel-response sub-opcodes (byte1 when byte0 == 0xF7) ----
 * For SET_* sub-opcodes byte2 carries the new percentage (0..100); the
 * value updates a RAM-only "pending" set and is NOT applied or persisted
 * until SAVE.  All SET/SAVE/RESET sub-opcodes require SYS_STATE_STANDBY.
 *   0x01 SET_D2          Stage pending D2 power limit  (byte2 = percent)
 *   0x02 SET_D1          Stage pending D1 power limit  (byte2 = percent)
 *   0x03 SET_R           Stage pending R  power limit  (byte2 = percent)
 *   0x04 SAVE            Validate pending set + persist to flash + apply
 *   0x05 RESET_DEFAULTS  Restore + persist compile-time defaults
 *                        (power 100/60/60, response 100/70/40)
 *   0x06 QUERY           Request a 1 s burst of 0x30D telemetry at 10 Hz
 *   0x07 SET_D2_RESPONSE Stage pending D2 accel response (byte2 = percent)
 *   0x08 SET_D1_RESPONSE Stage pending D1 accel response (byte2 = percent)
 *   0x09 SET_R_RESPONSE  Stage pending R  accel response (byte2 = percent) */
#define GEAR_LIMIT_OP_SET_D2          0x01U
#define GEAR_LIMIT_OP_SET_D1          0x02U
#define GEAR_LIMIT_OP_SET_R           0x03U
#define GEAR_LIMIT_OP_SAVE            0x04U
#define GEAR_LIMIT_OP_RESET_DEFAULTS  0x05U
#define GEAR_LIMIT_OP_QUERY           0x06U
#define GEAR_LIMIT_OP_SET_D2_RESPONSE 0x07U
#define GEAR_LIMIT_OP_SET_D1_RESPONSE 0x08U
#define GEAR_LIMIT_OP_SET_R_RESPONSE  0x09U

/* ---- Steering-Z dual-reference sub-opcodes (byte1 when byte0 == 0xF8) ----
 * PB5 stays the primary/safety center reference; Z is secondary precision.
 *   0x01 QUERY        Request a 1 s burst of 0x30E telemetry at 10 Hz
 *   0x02 CALIBRATE    Recompute + persist the Z↔center offset.  Requires
 *                     PB5 to currently detect center AND BOOT/STANDBY.
 *   0x03 CLEAR        Clear the stored Z calibration (PB5 center kept).
 *                     Requires BOOT/STANDBY (ESP32 enforces double-confirm).  */
#define STEER_Z_OP_QUERY      0x01U
#define STEER_Z_OP_CALIBRATE  0x02U
#define STEER_Z_OP_CLEAR      0x03U

/* ---- EPS runtime parameter tuning sub-opcodes (byte1 when byte0 == 0xF9) ----
 * SET_PARAM applies immediately to the RAM active copy; no safety gate.
 * SAVE and RESET require SYS_STATE_STANDBY.
 *
 * 0x30F frame kinds (byte0 bits 7-4):
 *   kind 0: gains A   — assist_strength×1000, center_strength×1000, damping×1000
 *   kind 1: gains B   — friction_comp×1000, coast_band_pct×100, min_drive_pct×100
 *   kind 2: speeds    — assist_vs_speed×10, return_vs_speed×10, deadband_deg×100
 *   kind 3: mechanical — max_pwm_pct×100, slew_rate_pct×100, center_offset_deg×100
 *   kind 4: live state — enc_raw(int16), angle×10, motor_effort×10, steer_state
 * byte0 bits 3-0 flags: bit0=flash_valid, bit1=sys_in_standby
 *
 *   0x01 SET_PARAM  Immediate: byte2=param_id(eps_param_id_t), bytes3-6=float LE
 *   0x02 SAVE       Persist active RAM copy to flash (STANDBY only)
 *   0x03 RESET      Revert to compiled defaults, RAM only (STANDBY only)
 *   0x04 QUERY      Request a 1 s burst of 0x30F telemetry at 10 Hz          */
#define EPS_PARAM_OP_SET_PARAM   0x01U
#define EPS_PARAM_OP_SAVE        0x02U
#define EPS_PARAM_OP_RESET       0x03U
#define EPS_PARAM_OP_QUERY       0x04U

/* ---- Drive-tuning sub-opcodes (byte1 when byte0 == 0xFA) ------------
 * For SET_* sub-opcodes the new value is carried as a uint16 little-endian
 * in byte2 (LSB) + byte3 (MSB).  SET_* stages a RAM-only "pending" set; the
 * value is NOT applied or persisted until SAVE.  All SET/SAVE/RESET sub-
 * opcodes require SYS_STATE_STANDBY; QUERY is read-only and exempt.
 *   0x01 SET_ACCEL_RAMP    Stage pending accel ramp   (%/s)
 *   0x02 SET_BRAKE_RAMP    Stage pending brake ramp   (%/s)
 *   0x03 SET_REVERSE_RAMP  Stage pending reverse ramp (%/s, GEAR_REVERSE only)
 *   0x04 SET_CREEP_ENABLE  Stage pending creep enable (0/1)
 *   0x05 SET_CREEP_POWER   Stage pending creep floor  (%)
 *   0x06 SET_CREEP_DELAY   Stage pending creep delay  (ms)
 *   0x07 SAVE              Validate pending set + persist to flash + apply
 *   0x08 RESET_DEFAULTS    Restore + persist compile-time defaults
 *   0x09 QUERY             Request a 1 s burst of 0x310 telemetry at 10 Hz   */
#define DRIVE_TUNE_OP_SET_ACCEL_RAMP   0x01U
#define DRIVE_TUNE_OP_SET_BRAKE_RAMP   0x02U
#define DRIVE_TUNE_OP_SET_REVERSE_RAMP 0x03U
#define DRIVE_TUNE_OP_SET_CREEP_ENABLE 0x04U
#define DRIVE_TUNE_OP_SET_CREEP_POWER  0x05U
#define DRIVE_TUNE_OP_SET_CREEP_DELAY  0x06U
#define DRIVE_TUNE_OP_SAVE             0x07U
#define DRIVE_TUNE_OP_RESET_DEFAULTS   0x08U
#define DRIVE_TUNE_OP_QUERY            0x09U

/* Drive-tuning telemetry (0x310) field ids — one frame per field, see
 * the 0x310 frame layout comment in can_handler.c.                       */
#define DRIVE_TUNE_FIELD_ACCEL_RAMP    0x01U
#define DRIVE_TUNE_FIELD_BRAKE_RAMP    0x02U
#define DRIVE_TUNE_FIELD_REVERSE_RAMP  0x03U
#define DRIVE_TUNE_FIELD_CREEP_ENABLE  0x04U
#define DRIVE_TUNE_FIELD_CREEP_POWER   0x05U
#define DRIVE_TUNE_FIELD_CREEP_DELAY   0x06U
#define DRIVE_TUNE_FIELD_COUNT         6U

/* ---- Battery-limit sub-opcodes (byte1 when byte0 == 0xFB) -----------
 * Values are centivolts (V×100) or ms, carried as uint16 little-endian in
 * byte2 (LSB) + byte3 (MSB).  SET_* stages a RAM-only "pending" set; nothing
 * is applied or persisted until SAVE.  All SET/SAVE/RESET require STANDBY.
 *   0x01 SET_WARNING       Stage pending low-voltage warning (cV)
 *   0x02 SET_LIMIT         Stage pending derate (DEGRADED) threshold (cV)
 *   0x03 SET_CUTOFF        Stage pending SAFE cutoff threshold (cV)
 *   0x04 SET_RECOVERY      Stage pending SAFE→STANDBY recovery (cV)
 *   0x05 SET_FILTER        Stage pending voltage filter time constant (ms)
 *   0x06 SAVE              Validate pending set + persist to flash + apply
 *   0x07 RESET_DEFAULTS    Restore + persist compile-time defaults
 *   0x08 QUERY             Request a 1 s burst of 0x311 telemetry at 10 Hz   */
#define BATT_LIM_OP_SET_WARNING   0x01U
#define BATT_LIM_OP_SET_LIMIT     0x02U
#define BATT_LIM_OP_SET_CUTOFF    0x03U
#define BATT_LIM_OP_SET_RECOVERY  0x04U
#define BATT_LIM_OP_SET_FILTER    0x05U
#define BATT_LIM_OP_SAVE          0x06U
#define BATT_LIM_OP_RESET_DEFAULTS 0x07U
#define BATT_LIM_OP_QUERY         0x08U

/* Battery-limit telemetry (0x311) field ids — one frame per field. */
#define BATT_LIM_FIELD_WARNING    0x01U
#define BATT_LIM_FIELD_LIMIT      0x02U
#define BATT_LIM_FIELD_CUTOFF     0x03U
#define BATT_LIM_FIELD_RECOVERY   0x04U
#define BATT_LIM_FIELD_FILTER     0x05U
#define BATT_LIM_FIELD_COUNT      5U


typedef enum {
    ACK_OK                  = 0,   /* Command accepted and applied           */
    ACK_REJECTED            = 1,   /* Command rejected (speed too high, etc) */
    ACK_INVALID             = 2,   /* Command payload invalid / malformed    */
    ACK_BLOCKED_BY_SAFETY   = 3    /* Command blocked by safety system state */
} CAN_AckResult_t;

/* Timeouts */
#define CAN_TIMEOUT_HEARTBEAT_MS  250    // Heartbeat timeout
#define CAN_TIMEOUT_OBSTACLE_MS   500    // Obstacle data timeout (fail-safe)

/* CAN bus-off recovery configuration */
#define CAN_BUSOFF_RETRY_INTERVAL_MS  500   /* Non-blocking retry interval      */
#define CAN_BUSOFF_MAX_RETRIES        10    /* Max FDCAN re-init attempts before giving up; system stays in LIMP_HOME (NOT ERROR) */

/* Sustained heartbeat window required to CONFIRM bus-off recovery.
 * After the FDCAN peripheral is re-initialised (RUNNING again) the bus is
 * NOT yet declared recovered: valid ESP32 heartbeats must be present
 * continuously for this long before the fault is cleared and the retry
 * counter is reset.  Merely observing RUNNING is insufficient — a
 * persistent physical fault would otherwise loop forever without ever
 * reaching CAN_BUSOFF_MAX_RETRIES.  Chosen as several heartbeat periods
 * (heartbeat rate 100 ms, liveness timeout CAN_TIMEOUT_HEARTBEAT_MS).   */
#define CAN_BUSOFF_RECOVERY_WINDOW_MS 1000U

/* FIFO overflow escalation threshold.
 * After this many cumulative message-lost events, CAN_ProcessMessages()
 * escalates to DEGRADED_L1.  A single overflow is a transient; sustained
 * overflow indicates bus overload or a stuck processing loop.             */
#define CAN_FIFO_OVERFLOW_DEGRADE_THRESHOLD  5U

/* CAN Statistics */
typedef struct {
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t tx_errors;
    uint32_t rx_errors;
    uint32_t last_heartbeat_esp32;
    uint32_t busoff_count;                  /* Total bus-off events detected     */
    uint32_t fifo_overflow_count;           /* FIFO message-lost events          */
    /* ---- Frame health monitoring (Part 3) ---- */
    uint32_t rx_count_prev;                 /* rx_count snapshot at last 1-second tick */
    uint32_t rx_frames_per_sec;             /* Computed: frames received in last 1 s   */
    uint32_t rx_rate_tick;                  /* Timestamp of last FPS computation        */
} CAN_Stats_t;

/* CAN bus error diagnostics — readable via SWD debugger.
 * Updated every 10 ms by CAN_CheckBusOff().                          */
typedef struct {
    uint8_t  last_error_code;   /* FDCAN PSR.LEC: 0=none 1=stuff 2=form
                                 * 3=ack 4=bit1(rec) 5=bit0(dom) 6=CRC */
    uint8_t  error_passive;     /* 1 = Error Passive state              */
    uint8_t  bus_off;           /* 1 = Bus-Off state                    */
    uint8_t  warning;           /* 1 = error warning threshold exceeded */
    uint8_t  tec;               /* Transmit Error Counter (0-255)       */
    uint8_t  rec;               /* Receive Error Counter  (0-255)       */
    /* ---- TX validation (Part 5) ---- */
    uint8_t  tx_nack_flag;      /* 1 = repeated TX failures detected    */
    uint8_t  tx_consec_fail;    /* Consecutive TX failures (0-255)      */
} CAN_Diag_t;

/* ---- CAN/0x309 delivery meta-diagnostic (additive, report-only) ----
 * Answers the observability questions A–D from the 0x309 audit without
 * touching any control or safety path:
 *   A) diag309_call_count   — times CAN_SendI2CDiag() actually ran
 *   B) tick_1000ms_count    — iterations of the main-loop 1 Hz block
 *   C) diag309_tx_ok / err  — TransmitFrame() result for 0x309 specifically
 *   D) tx_fifo_full_drops   — frames dropped because the TX FIFO was full
 *   E) hb_tx_count          — heartbeat (0x001) frames successfully queued (tracked locally)
 *   F) hb_tx_err            — heartbeat frames that failed to queue (TX busy); surfaced on 0x30A byte7 bits7:1
 *   G) tx_queue_depth[_max] — software TX ring occupancy now / high-water; surfaced on 0x312 bytes 5-6
 * All counters saturate; never wrap to 0.  0x30A currently surfaces A–D and F. */
typedef struct {
    uint32_t diag309_call_count;   /* A: CAN_SendI2CDiag() invocations     */
    uint32_t tick_1000ms_count;    /* B: 1 Hz scheduler-block iterations   */
    uint32_t diag309_tx_ok;        /* C: 0x309 queued to TX FIFO OK        */
    uint32_t diag309_tx_err;       /* C: 0x309 TransmitFrame() != HAL_OK   */
    uint32_t tx_fifo_full_drops;   /* D: any frame dropped (FIFO full)     */
    uint32_t hb_tx_count;          /* E: heartbeat 0x001 frames queued OK  */
    uint32_t hb_tx_err;            /* F: heartbeat 0x001 TX failures       */
    uint8_t  tx_queue_depth;       /* G: software TX ring occupancy (now)  */
    uint8_t  tx_queue_depth_max;   /* G: software TX ring high-water mark  */
} CAN_TxMeta_t;

/* CAN_InitDiag_t is defined in can_init_diag.h (included above) to allow
 * stm32g4xx_hal_msp.c to record MspInit diagnostics without depending on
 * the full CAN handler API.                                              */

/* Function prototypes */
void CAN_Init(void);
void CAN_TestTransmit(void);
void CAN_SendHeartbeat(void);
void CAN_SendStatusSpeed(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr);
void CAN_SendStatusCurrent(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr);
void CAN_SendStatusTemp(int8_t t1, int8_t t2, int8_t t3, int8_t t4, int8_t t5);
void CAN_SendStatusSafety(bool abs, bool tcs, uint8_t error_code,
                          uint8_t loop_peak_100us);
void CAN_SendStatusSteering(int16_t angle, bool calibrated);
void CAN_SendStatusTraction(void);
void CAN_SendStatusWheelEffort(void);
void CAN_SendStatusTempMap(void);
void CAN_SendStatusBattery(void);
void CAN_SendStatusLights(void);
void CAN_SendStatusPedal(uint8_t pedal_pct);
void CAN_SendError(uint8_t error_code, uint8_t subsystem);
void CAN_SendDiagnosticEncoder(int32_t raw_count, int16_t delta);
void CAN_SendCommandAck(uint8_t cmd_id_low, CAN_AckResult_t result);
void CAN_SendServiceStatus(void);
void CAN_SendErrorLogHeader(void);
void CAN_SendDebounceDiag(void);    /* 1 Hz DWT-debounce filter EMI counters (0x306 + 0x307) */
void CAN_SendI2CDiag(void);         /* 1 Hz I2C topology diagnostic (0x309): mux + per-channel INA226 */
void CAN_SendCanMetaDiag(void);     /* 1 Hz CAN/0x309 delivery meta-diagnostic (0x30A) */
void CAN_SendBootResetDiag(void);   /* 1 Hz boot/reset diagnostic (0x312): uptime_ms + RCC reset-cause */
void CAN_SendWheelSensorDiag(void); /* 1 Hz per-wheel speed-sensor fault-reason diagnostic (0x313) */
void CAN_SendMotionInhibit(void);   /* 10 Hz MOTION_INHIBIT_REASON instrumentation (0x315) */
void CAN_SendTractionLimitDiag(void); /* 1 Hz obstacle/cap/brake-ramp telemetry (0x31A) */
void CAN_SendSteeringCenteringDiag(void); /* 1 Hz steering homing telemetry (0x316) */
void CAN_SendRelayHealthDiag(void);       /* 1 Hz relay/current-sense health (0x317) */
void CAN_SendIna226Ch5Diag(void);         /* 1 Hz steering INA226 CH5 diagnostic (0x318) */
void CAN_SendI2CScanReport(void);   /* On-demand I2C service-mode scan report (0x30B) */
void CAN_SendFdcanDiag(void);       /* On-demand FDCAN error-counter dump (0x30C) */
void CAN_ProcessMessages(void);
/* Drain the software TX queue into the FDCAN hardware FIFO.  Non-blocking;
 * call once per main-loop iteration so queued frames keep flowing even when
 * no new frames are produced.  Additive — never blocks or gates safety.    */
void CAN_TxPump(void);
bool CAN_IsESP32Alive(void);
bool CAN_IsGlobalSilent(void);
void CAN_CheckBusOff(void);
bool CAN_IsBusOff(void);
void CAN_UpdateFrameRate(void);     /* Call every ~1 s to compute rx FPS  */

/* Drives the on-demand 0x308 pedal-calibration telemetry burst.
 * Call once per 100 ms tick — the function is a no-op while no
 * burst is in progress, so it is safe to call unconditionally and
 * has zero impact on backward-compatible nodes that ignore 0x308. */
void CAN_PedalCalBurstUpdate(void);

/* Drives the guided PedalCalSession FSM (audit P5).  Call once per 50 ms
 * main-loop tick, immediately after Pedal_Update().  It builds the live
 * PedalCalConds from safety/sensor state, advances PedalCalSession_Update(),
 * enforces safe outputs (Traction_SetDemand(0)) while a session is active, and
 * publishes the 0x319 session-status frame.  No-op while the session is IDLE. */
void CAN_PedalCalCaptureTick(void);

/* Drives the on-demand 0x30D gear power-limit telemetry burst.
 * Call once per 100 ms tick — the function is a no-op while no burst
 * is in progress, so it is safe to call unconditionally and has zero
 * impact on backward-compatible nodes that ignore 0x30D.            */
void CAN_GearLimitsBurstUpdate(void);

/* Drives the on-demand 0x30E steering-Z center-reference telemetry burst.
 * Call once per 100 ms tick — the function is a no-op while no burst is
 * in progress, so it is safe to call unconditionally and has zero impact
 * on backward-compatible nodes that ignore 0x30E.                       */
void CAN_SteeringZBurstUpdate(void);

/* Drives the on-demand 0x30F EPS parameter + live-state telemetry burst.
 * Call once per 100 ms tick.  No-op while no burst is in progress; has zero
 * impact on backward-compatible nodes that ignore 0x30F.                */
void CAN_EPS_ParamsBurstUpdate(void);

/* Drives the on-demand 0x310 drive-tuning (ramp/creep) telemetry burst.
 * Call once per 100 ms tick.  No-op while no burst is in progress; has zero
 * impact on backward-compatible nodes that ignore 0x310.                */
void CAN_DriveTuningBurstUpdate(void);

/* Drives the on-demand 0x311 battery-limit telemetry burst.
 * Call once per 100 ms tick.  No-op while no burst is in progress; has zero
 * impact on backward-compatible nodes that ignore 0x311.                */
void CAN_BatteryLimitsBurstUpdate(void);

/* LED relay states — front (PB10) and rear (PB11) — toggled via CAN 0x120 */
void LED_Relay_Set(bool on);          /* front relay */
bool LED_Relay_Get(void);             /* front relay state */
void LED_Relay_Rear_Set(bool on);     /* rear relay */
bool LED_Relay_Rear_Get(void);        /* rear relay state */

extern CAN_Stats_t    can_stats;
extern CAN_Diag_t     can_diag;
extern CAN_TxMeta_t   can_txmeta;
extern FDCAN_HandleTypeDef hfdcan1;

/* Debug-visible global CAN buffers (volatile for debugger inspection) */
extern volatile uint8_t               g_CAN_RxData[8];
extern volatile FDCAN_RxHeaderTypeDef  g_CAN_RxHeader;
extern volatile uint8_t               g_CAN_TxData[8];

#ifdef __cplusplus
}
#endif

#endif /* CAN_HANDLER_H */
