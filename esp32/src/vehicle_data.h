// =============================================================================
// ESP32-S3 HMI — Vehicle Data Store
//
// Passive data container for decoded CAN telemetry.
// All values are populated by can_rx and read by UI modules.
// No logic, no thresholds, no decisions.
//
// RENDER PIPELINE (deterministic frame latch):
//   CAN INPUT → can_rx::poll() → VehicleData (shadow store, Core 1)
//            → mutex copy to renderVD (shared) → localVD (render task, Core 0)
//            → screen.update(localVD) copies into cur_* members (frame latch)
//            → screen.draw() uses only cur_*/prev_* (immutable during render)
//
// The localVD copy in the render task acts as a frozen snapshot — CAN updates
// arriving during the render frame do NOT affect the in-progress draw.
// This guarantees frame-consistent, zero-jitter display output.
//
// Reference: docs/CAN_CONTRACT_FINAL.md rev 1.0
// =============================================================================

#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H

#include <cstdint>
#include <array>
#include "can_ids.h"

namespace vehicle {

// Number of wheels: FL, FR, RL, RR
inline constexpr uint8_t NUM_WHEELS = 4;
// Number of temperature sensors
inline constexpr uint8_t NUM_TEMP_SENSORS = 5;

// -------------------------------------------------------------------------
// Heartbeat data from STM32 (0x001)
// -------------------------------------------------------------------------
struct HeartbeatData {
    uint8_t           aliveCounter = 0;
    can::SystemState  systemState  = can::SystemState::BOOT;
    uint8_t           faultFlags   = 0;
    uint8_t           errorCode    = 0;
    uint8_t           statusFlags  = 0;   // bit 0: startup inhibit, bit 1: 4x4, bit 2: tank, bits 3-5: DS18B20 count
    uint8_t           relayStatus  = 0;   // bit 0: reserved (always 0), bit 1: TRAC, bit 2: DIR, bit 7: SEQ_COMPLETE
    unsigned long     timestampMs  = 0;
};

// -------------------------------------------------------------------------
// Wheel speeds (0x200) — raw uint16 in 0.1 km/h units
// -------------------------------------------------------------------------
struct SpeedData {
    std::array<uint16_t, NUM_WHEELS> raw{};   // FL, FR, RL, RR
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Motor currents (0x201) — raw uint16 in 0.01 A units
// -------------------------------------------------------------------------
struct CurrentData {
    std::array<uint16_t, NUM_WHEELS> raw{};   // FL, FR, RL, RR
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Temperature sensors (0x202) — int8 °C
// -------------------------------------------------------------------------
struct TempData {
    std::array<int8_t, NUM_TEMP_SENSORS> temps{};
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Safety status (0x203)
// -------------------------------------------------------------------------
struct SafetyData {
    uint8_t absActive  = 0;
    uint8_t tcsActive  = 0;
    uint8_t errorCode  = 0;
    uint8_t peakLoop100us = 0;   // 0x203 byte 5 — STM32 100 Hz task peak (×100 µs, sat. 255 = 25.5 ms)
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Steering status (0x204) — raw int16 in 0.1° units
// -------------------------------------------------------------------------
struct SteeringData {
    int16_t angleRaw   = 0;    // 0.1° units
    uint8_t calibrated = 0;
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Traction scale per wheel (0x205) — 0–100 %
// -------------------------------------------------------------------------
struct TractionData {
    std::array<uint8_t, NUM_WHEELS> scale{};  // FL, FR, RL, RR
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Temperature map (0x206) — int8 °C, mapped: FL, FR, RL, RR, Ambient
// -------------------------------------------------------------------------
struct TempMapData {
    std::array<int8_t, NUM_TEMP_SENSORS> temps{};  // FL, FR, RL, RR, Ambient
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Diagnostic error (0x300)
// -------------------------------------------------------------------------
struct DiagData {
    uint8_t errorCode  = 0;
    uint8_t subsystem  = 0;
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Service status (0x301–0x303) — 32-bit bitmasks
// -------------------------------------------------------------------------
struct ServiceData {
    uint32_t faultMask    = 0;   // 0x301
    uint32_t enabledMask  = 0;   // 0x302
    uint32_t disabledMask = 0;   // 0x303
    unsigned long faultTimestampMs    = 0;
    unsigned long enabledTimestampMs  = 0;
    unsigned long disabledTimestampMs = 0;
};

// -------------------------------------------------------------------------
// Battery bus data (0x207) — 24V main battery via INA226 100A shunt
// Displayed in the upper-right corner of the drive screen.
// -------------------------------------------------------------------------
struct BatteryData {
    uint16_t currentRaw = 0;   // 0.01 A units
    uint16_t voltageRaw = 0;   // 0.01 V units
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Command ACK (0x103) — STM32 acknowledgment of ESP32 command
// -------------------------------------------------------------------------
struct AckData {
    uint8_t           cmdIdLow    = 0;     // Low byte of acknowledged CAN ID
    can::AckResult    result      = can::AckResult::OK;
    can::SystemState  systemState = can::SystemState::BOOT;
    unsigned long     timestampMs = 0;
};

// -------------------------------------------------------------------------
// Front obstacle sensor (0x208) — TF-Mini Plus LiDAR distance in cm
// -------------------------------------------------------------------------
struct ObstacleData {
    uint16_t distanceCm  = 0;   // 0 = no reading, max ~1200 cm (TF-Mini Plus 12 m range)
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// LED / Lights status (0x20A) — relay states from STM32
//   Byte 0: front relay (headlights / front LED strip)
//   Byte 1: rear relay  (tail / brake LED strip)
// -------------------------------------------------------------------------
struct LightsData {
    bool frontRelayOn       = false;   // true = front LED relay active
    bool rearRelayOn        = false;   // true = rear LED relay active
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Pedal position telemetry (0x20B) — live Hall pedal travel from STM32.
//   Byte 0: pedal position % (0 = released, 100 = full travel)
// Telemetry only: drives the HMI THROTTLE bar.  This is the physical pedal
// reading, distinct from the per-wheel torque/TCS scale in TractionData
// (0x205), which must NOT be used as a throttle indicator.
// -------------------------------------------------------------------------
struct PedalData {
    uint8_t percent         = 0;       // 0..100 Hall pedal travel
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Pedal calibration telemetry (0x308) — on-demand burst from STM32 after
// SERVICE_ACTION_PEDAL_CAL/QUERY.  Decoded layout matches can_handler.c
// pedalcal_send_status().  STM32 alternates PENDING / STORED frame
// variants within the burst (bit 6 of flags selects the variant); the
// decoder maintains separate slots for each.
//   flags bit 0: pending MIN captured
//   flags bit 1: pending MAX captured
//   flags bit 2: pending pair validates OK (range + Δ)
//   flags bit 3: stored slot valid
//   flags bit 4: safety gates satisfied (STANDBY + inhibited + pedal<3% + …)
//   flags bit 5: pedal currently plausible
//   flags bit 6: 0 = bytes 3-6 are PENDING; 1 = bytes 3-6 are STORED
//   rawAdc:      live 12-bit ADC reading (post EMA)
//   storedMin/Max:  endpoints currently held in STM32 flash
//   pendingMin/Max: STM32 RAM-only pending endpoints (0 when not set)
//   pedalPercent:   0..100 saturating
// -------------------------------------------------------------------------
struct PedalCalData {
    uint8_t  flags        = 0;
    uint16_t rawAdc       = 0;
    uint16_t storedMin    = 0;
    uint16_t storedMax    = 0;
    uint16_t pendingMin   = 0;
    uint16_t pendingMax   = 0;
    uint8_t  pedalPercent = 0;
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Gear power-limit telemetry (0x30D) — on-demand (10 Hz × 1 s after QUERY)
// Active = limits currently applied by the STM32 motor controller.
// Pending = unsaved edit staged on the STM32 (mirrors the UI edit state).
// -------------------------------------------------------------------------
struct GearLimitsData {
    uint8_t  flags        = 0;    // bit0 stored-valid, bit1 pending-differs,
                                  // bit2 safety-ok(STANDBY), bit3 pending-valid,
                                  // bit4 frame-kind(0=power,1=response)
    uint8_t  activeD2     = 0;    // currently applied D2 power limit (percent)
    uint8_t  activeD1     = 0;
    uint8_t  activeR      = 0;
    uint8_t  pendingD2    = 0;    // staged (unsaved) D2 power limit (percent)
    uint8_t  pendingD1    = 0;
    uint8_t  pendingR     = 0;
    uint8_t  systemState  = 0;
    // Accel-response profile (v2) — carried by the RESPONSE frame (flags bit4=1).
    uint8_t  activeRespD2  = 0;   // currently applied D2 accel response (percent)
    uint8_t  activeRespD1  = 0;
    uint8_t  activeRespR   = 0;
    uint8_t  pendingRespD2 = 0;   // staged (unsaved) D2 accel response (percent)
    uint8_t  pendingRespD1 = 0;
    uint8_t  pendingRespR  = 0;
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Steering PB5 + encoder-Z dual center-reference diagnostic (0x30E)
// PB5 (LJ12A3, EXTI5) is the PRIMARY physical/safety center reference; the
// encoder Z (index) pulse on PB4 is a SECONDARY precision reference that can
// NEVER center on its own.  Diagnostic only — never gates HMI control.
// Frame layout mirrors Core/Src/can_handler.c steerz_send_status().
// -------------------------------------------------------------------------
struct SteeringZData {
    uint8_t  flags        = 0;   // bit0-2 status, bit3 PB5 live, bit4 Z valid,
                                 // bit5 Z slip
    uint8_t  status       = 0;   // 0 NOT_CAL,1 OK,2 NOT_SEEN,3 OUT_OF_WINDOW,
                                 // 4 MECH_OFFSET (== flags & 0x07)
    bool     pb5Live      = false;
    bool     zValid       = false;
    bool     zSlip        = false;
    uint8_t  zPulseCount  = 0;   // saturated 0..255
    int16_t  zLastPos     = 0;   // TIM2 count at last Z pulse
    int16_t  zOffset      = 0;   // Z↔center offset (counts)
    int8_t   zLastError   = 0;   // last inter-pulse error (counts)
    uint8_t  zTolerance   = 0;   // window used (counts)
    unsigned long timestampMs = 0;
};
// Counters of edge pulses rejected by the STM32 DWT 200 µs pre-filter.
// Wheel counters arrive truncated/saturated to uint16; steering is full uint32.
// -------------------------------------------------------------------------
struct DebounceDiagData {
    std::array<uint16_t, NUM_WHEELS> wheelFiltered{};   // FL, FR, RL, RR (saturated u16)
    uint32_t      steerFiltered = 0;
    unsigned long timestampMs   = 0;
};

// -------------------------------------------------------------------------
// I2C topology diagnostic (0x309) — report-only
// Lets the HMI Safe Mode screen tell a missing TCA9548A multiplexer (0x70)
// apart from a missing/dead INA226 (0x40) on a specific mux channel.
// `valid` is false until the first 0x309 frame is received (show "NO DATA").
// -------------------------------------------------------------------------
struct I2cDiagData {
    bool     muxPresent    = false;   // TCA9548A 0x70 acked on the STM32 side
    uint8_t  inaOkMask     = 0;       // bit i = INA226 ch i acked (FL,FR,RL,RR,BAT,STEER)
    uint8_t  inaExpectedMask = 0x3F;  // bit i = ch i's branch is powered this phase,
                                      // so its INA226 should answer. Defaults to all
                                      // channels (pre-extension STM32 FW with DLC 5
                                      // omits this byte → keep legacy FAIL behaviour).
    uint8_t  failCount     = 0;       // failed I2C transactions in last STM32 cycle
    uint8_t  recoveryCount = 0;       // sticky bus-recovery attempt counter
    bool     everOk        = false;   // latched: at least one INA seen healthy
    bool     valid         = false;   // a 0x309 frame has been received
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// CAN/0x309 delivery meta-diagnostic (0x30A) — report-only
// Proves whether the 0x309 frame is generated, queued and accepted by the
// FDCAN TX FIFO, independent of the I2C bus state (audit questions A–D).
// -------------------------------------------------------------------------
struct CanMetaData {
    uint16_t diag309CallCount = 0;   // [A] CAN_SendI2CDiag() invocations (saturated)
    uint16_t tick1000msCount  = 0;   // [B] 1 Hz scheduler-block iterations (saturated)
    uint8_t  diag309TxOk      = 0;   // [C] 0x309 queued OK (saturated)
    uint8_t  diag309TxErr     = 0;   // [C] 0x309 TransmitFrame() failed (saturated)
    uint8_t  txFifoFullDrops  = 0;   // [D] frames dropped, TX FIFO full (saturated)
    bool     fdcanInitOk      = false;
    bool     valid            = false;
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// I2C service-mode scan report (0x30B) — report-only, on-demand
// Active probe of the I2C topology triggered by SERVICE_CMD 0xF6.
// -------------------------------------------------------------------------
struct I2cScanData {
    bool     sclIdleHigh       = false;  // SCL line idle high (pull-up OK)
    bool     sdaIdleHigh       = false;  // SDA line idle high (not stuck)
    bool     recoveryAttempted = false;  // bus recovery was run
    bool     recoverySuccess   = false;  // SDA released after recovery
    bool     muxPresent        = false;  // TCA9548A 0x70 acked
    uint8_t  inaPresentMask    = 0;      // bit0..5 = INA226 0x40 acked behind ch0..5
    uint8_t  failCount         = 0;
    uint8_t  recoveryAttempts  = 0;
    uint8_t  scanPhase         = 0;      // 0x30B byte5 — I2C_SCAN_PHASE_* terminal phase
    bool     valid             = false;
    unsigned long timestampMs  = 0;
};

// -------------------------------------------------------------------------
// FDCAN error-counter dump (0x30C) — report-only, on-demand
// -------------------------------------------------------------------------
struct FdcanDiagData {
    uint8_t  lastErrorCode = 0;   // PSR.LEC
    bool     errorPassive  = false;
    bool     busOff        = false;
    bool     warning       = false;
    uint8_t  tec           = 0;    // Transmit Error Counter
    uint8_t  rec           = 0;    // Receive Error Counter
    uint8_t  txNackFlag    = 0;
    uint8_t  txConsecFail  = 0;
    bool     valid         = false;
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Active drive mode — set locally by ESP32 touch (eventually from STM32 CAN echo)
// -------------------------------------------------------------------------
struct ModeData {
    uint8_t modeFlags       = 0;       // bit 0 = 4x4, bit 1 = tank turn
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// EPS parameter telemetry (0x30F) — Engineering Menu tuning + live state
// All float fields are decoded from the scaled int16 CAN frames.
// -------------------------------------------------------------------------
struct EpsParamsData {
    // kind 0: gains A
    float assistStrength  = 0.45f;   // assist_strength (default matches firmware)
    float centerStrength  = 0.30f;
    float damping         = 0.10f;
    // kind 1: gains B
    float frictionComp    = 0.05f;
    float coastBandPct    = 3.0f;
    float minDrivePct     = 8.0f;
    // kind 2: speed params + deadband
    float assistVsSpeed   = 18.0f;
    float returnVsSpeed   = 35.0f;
    float deadbandDeg     = 1.8f;
    // kind 3: mechanical params
    float maxPwmPct       = 60.0f;
    float slewRatePct     = 5.883f;
    float centerOffsetDeg = 0.0f;
    // kind 4: live state
    int16_t       encRaw        = 0;     // raw TIM2 count
    float         angleDeg      = 0.0f;  // road-wheel angle (°)
    float         motorEffortPct = 0.0f; // 0..100 %
    uint8_t       steerState    = 0;     // 0=uncal, 1=cal, 2=enc_fault
    // flags (present in all kinds, from most-recent frame)
    bool          flashValid    = false;
    bool          sysInStandby  = false;
    bool          valid         = false;          // true after first 0x30F frame
    unsigned long timestampMs   = 0;
    // Track which kinds have been received (bitmask bits 0-4)
    uint8_t       kindsReceived = 0;
};

// -------------------------------------------------------------------------
// Drive-tuning telemetry (0x310) — on-demand field-stream burst after
// SERVICE_ACTION_DRIVE_TUNING/QUERY (0xFA op 0x09).  One CAN frame per field;
// the decoder preserves the other fields across frames so a QUERY (which
// streams all 6 fields ×10) leaves the editor fully populated.  "active" =
// the ramp/creep values the STM32 traction pipeline applies right now;
// "pending" = an unsaved edit staged on the STM32 (RAM only).  Units mirror
// the CAN contract: ramps %/s, CreepEnable 0/1, CreepPower %, CreepDelay ms.
// -------------------------------------------------------------------------
struct DriveTuningData {
    uint8_t  flags        = 0;   // bit0 stored-valid, bit1 pending-differs,
                                 // bit2 safety-ok(STANDBY), bit3 pending-valid
    uint16_t accelRamp    = can::DRIVE_ACCEL_RAMP_DEFAULT;    // %/s
    uint16_t brakeRamp    = can::DRIVE_BRAKE_RAMP_DEFAULT;    // %/s
    uint16_t reverseRamp  = can::DRIVE_REVERSE_RAMP_DEFAULT;  // %/s
    uint16_t creepEnable  = can::DRIVE_CREEP_ENABLE_DEFAULT;  // 0/1
    uint16_t creepPower   = can::DRIVE_CREEP_POWER_DEFAULT;   // %
    uint16_t creepDelay   = can::DRIVE_CREEP_DELAY_DEFAULT;   // ms
    // Staged (unsaved) pending mirror, per field.
    uint16_t pendAccelRamp   = can::DRIVE_ACCEL_RAMP_DEFAULT;
    uint16_t pendBrakeRamp   = can::DRIVE_BRAKE_RAMP_DEFAULT;
    uint16_t pendReverseRamp = can::DRIVE_REVERSE_RAMP_DEFAULT;
    uint16_t pendCreepEnable = can::DRIVE_CREEP_ENABLE_DEFAULT;
    uint16_t pendCreepPower  = can::DRIVE_CREEP_POWER_DEFAULT;
    uint16_t pendCreepDelay  = can::DRIVE_CREEP_DELAY_DEFAULT;
    uint8_t  systemState  = 0;
    uint8_t  fieldsSeen   = 0;   // bitmask: bit (field-1) set once received
    bool     valid        = false;  // true after first 0x310 frame
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Battery-limit telemetry (0x311) — on-demand field-stream burst after
// SERVICE_ACTION_BATTERY_LIMITS/QUERY (0xFB op 0x08).  Same per-field layout
// as 0x310; values are centivolts (V×100) or ms for the filter.  Diagnostic /
// configuration only — the STM32 safety state machine is never altered.
// -------------------------------------------------------------------------
struct BatteryLimitsData {
    uint8_t  flags        = 0;   // bit0 stored-valid, bit1 pending-differs,
                                 // bit2 safety-ok(STANDBY), bit3 pending-valid
    uint16_t warningCv    = can::BATT_WARNING_DEFAULT_CV;   // cV
    uint16_t limitCv      = can::BATT_LIMIT_DEFAULT_CV;     // cV
    uint16_t cutoffCv     = can::BATT_CUTOFF_DEFAULT_CV;    // cV
    uint16_t recoveryCv   = can::BATT_RECOVERY_DEFAULT_CV;  // cV
    uint16_t filterMs     = can::BATT_FILTER_DEFAULT_MS;    // ms
    uint16_t pendWarningCv  = can::BATT_WARNING_DEFAULT_CV;
    uint16_t pendLimitCv    = can::BATT_LIMIT_DEFAULT_CV;
    uint16_t pendCutoffCv   = can::BATT_CUTOFF_DEFAULT_CV;
    uint16_t pendRecoveryCv = can::BATT_RECOVERY_DEFAULT_CV;
    uint16_t pendFilterMs   = can::BATT_FILTER_DEFAULT_MS;
    uint8_t  systemState  = 0;
    uint8_t  fieldsSeen   = 0;
    bool     valid        = false;  // true after first 0x311 frame
    unsigned long timestampMs = 0;
};

// -------------------------------------------------------------------------
// Drive/battery live diagnostic view (FASE 8) — read-only aggregation.
// NOTE: the STM32 firmware does NOT emit a dedicated live drive/battery
// operating-point stream.  This view is therefore ASSEMBLED on demand by the
// Engineering screen from telemetry that DOES exist on the bus:
//   - pedalPercent      ← STATUS_PEDAL (0x20B)
//   - batteryRawVoltage ← STATUS_BATTERY (0x207)
//   - warning/cutoff    ← STATUS_SAFETY / heartbeat error_code (9/10)
// Every field the firmware does not transmit is marked invalid (`*Valid`
// = false) and rendered as "N/A".  No value is ever estimated or invented.
// -------------------------------------------------------------------------
struct DriveBatteryDiagData {
    uint8_t  pedalPercent          = 0;     bool pedalValid          = false;
    uint16_t requestedPower        = 0;     bool requestedPowerValid = false;
    uint16_t limitedPower          = 0;     bool limitedPowerValid   = false;
    uint16_t rampOutput            = 0;     bool rampOutputValid     = false;
    uint16_t batteryRawCv          = 0;     bool batteryRawValid     = false;  // cV
    uint16_t batteryFilteredCv     = 0;     bool batteryFilteredValid = false; // cV
    bool     warningActive         = false; bool warningValid        = false;
    bool     limitingActive        = false; bool limitingValid       = false;
    bool     cutoffActive          = false; bool cutoffValid         = false;
    bool     recoveryActive        = false; bool recoveryValid       = false;
    unsigned long lastUpdateAgeMs  = 0;     bool ageValid            = false;
};

// =========================================================================
// VehicleData — central read/write store
// =========================================================================
class VehicleData {
public:
    // Setters (used by can_rx)
    void setHeartbeat(const HeartbeatData& d)  { heartbeat_ = d; }
    void setSpeed(const SpeedData& d)          { speed_ = d; }
    void setCurrent(const CurrentData& d)      { current_ = d; }
    void setTemp(const TempData& d)            { temp_ = d; }
    void setSafety(const SafetyData& d)        { safety_ = d; }
    void setSteering(const SteeringData& d)    { steering_ = d; }
    void setTraction(const TractionData& d)    { traction_ = d; }
    void setTempMap(const TempMapData& d)      { tempMap_ = d; }
    void setDiag(const DiagData& d)            { diag_ = d; }
    void setBattery(const BatteryData& d)      { battery_ = d; }
    void setAck(const AckData& d)              { ack_ = d; }
    void setAckTimeout(unsigned long ts)       { ackTimeoutMs_ = ts; }
    void setObstacle(const ObstacleData& d)    { obstacle_ = d; }
    void setLights(const LightsData& d)        { lights_ = d; }
    void setPedal(const PedalData& d)          { pedal_ = d; }
    void setMode(const ModeData& d)            { mode_ = d; }
    void setDebounceDiag(const DebounceDiagData& d) { debounceDiag_ = d; }
    void setPedalCal(const PedalCalData& d)         { pedalCal_ = d; }
    void setGearLimits(const GearLimitsData& d)     { gearLimits_ = d; }
    void setI2cDiag(const I2cDiagData& d)           { i2cDiag_ = d; }
    void setCanMeta(const CanMetaData& d)           { canMeta_ = d; }
    void setI2cScan(const I2cScanData& d)           { i2cScan_ = d; }
    void setFdcanDiag(const FdcanDiagData& d)       { fdcanDiag_ = d; }
    void setSteeringZ(const SteeringZData& d)       { steeringZ_ = d; }
    void setEpsParams(const EpsParamsData& d)       { epsParams_ = d; }
    void setDriveTuning(const DriveTuningData& d)   { driveTuning_ = d; }
    void setBatteryLimits(const BatteryLimitsData& d) { batteryLimits_ = d; }

    void setServiceFaults(uint32_t mask, unsigned long ts)   { service_.faultMask = mask;    service_.faultTimestampMs = ts; }
    void setServiceEnabled(uint32_t mask, unsigned long ts)  { service_.enabledMask = mask;  service_.enabledTimestampMs = ts; }
    void setServiceDisabled(uint32_t mask, unsigned long ts) { service_.disabledMask = mask; service_.disabledTimestampMs = ts; }

    // Const getters (used by UI)
    const HeartbeatData& heartbeat() const { return heartbeat_; }
    const SpeedData&     speed()     const { return speed_; }
    const CurrentData&   current()   const { return current_; }
    const TempData&      temp()      const { return temp_; }
    const SafetyData&    safety()    const { return safety_; }
    const SteeringData&  steering()  const { return steering_; }
    const TractionData&  traction()  const { return traction_; }
    const TempMapData&   tempMap()   const { return tempMap_; }
    const DiagData&      diag()      const { return diag_; }
    const BatteryData&   battery()   const { return battery_; }
    const ServiceData&   service()   const { return service_; }
    const AckData&       ack()       const { return ack_; }
    unsigned long        ackTimeoutMs() const { return ackTimeoutMs_; }
    const ObstacleData&  obstacle()  const { return obstacle_; }
    const LightsData&    lights()    const { return lights_; }
    const PedalData&     pedal()     const { return pedal_; }
    const ModeData&      mode()      const { return mode_; }
    const DebounceDiagData& debounceDiag() const { return debounceDiag_; }
    const PedalCalData&     pedalCal()     const { return pedalCal_; }
    const GearLimitsData&   gearLimits()   const { return gearLimits_; }
    const I2cDiagData&      i2cDiag()      const { return i2cDiag_; }
    const CanMetaData&      canMeta()      const { return canMeta_; }
    const I2cScanData&      i2cScan()      const { return i2cScan_; }
    const FdcanDiagData&    fdcanDiag()    const { return fdcanDiag_; }
    const SteeringZData&    steeringZ()    const { return steeringZ_; }
    const EpsParamsData&    epsParams()    const { return epsParams_; }
    const DriveTuningData&  driveTuning()  const { return driveTuning_; }
    const BatteryLimitsData& batteryLimits() const { return batteryLimits_; }

private:
    HeartbeatData heartbeat_;
    SpeedData     speed_;
    CurrentData   current_;
    TempData      temp_;
    SafetyData    safety_;
    SteeringData  steering_;
    TractionData  traction_;
    TempMapData   tempMap_;
    DiagData      diag_;
    BatteryData   battery_;
    ServiceData   service_;
    AckData       ack_;
    unsigned long ackTimeoutMs_ = 0;
    ObstacleData  obstacle_;
    LightsData    lights_;
    PedalData     pedal_;
    ModeData      mode_;
    DebounceDiagData debounceDiag_;
    PedalCalData     pedalCal_;
    GearLimitsData   gearLimits_;
    I2cDiagData      i2cDiag_;
    CanMetaData      canMeta_;
    I2cScanData      i2cScan_;
    FdcanDiagData    fdcanDiag_;
    SteeringZData    steeringZ_;
    EpsParamsData    epsParams_;
    DriveTuningData  driveTuning_;
    BatteryLimitsData batteryLimits_;
};

} // namespace vehicle

#endif // VEHICLE_DATA_H
