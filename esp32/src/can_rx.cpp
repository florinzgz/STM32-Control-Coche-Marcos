// =============================================================================
// ESP32-S3 HMI — CAN RX Module (implementation)
//
// Decodes CAN frames EXACTLY as specified in CAN_CONTRACT_FINAL.md rev 1.4.
// Pushes decoded values into the VehicleData store.
// Unknown CAN IDs are silently ignored.
//
// Reference: docs/CAN_CONTRACT_FINAL.md rev 1.4
//            docs/SERVICE_MODE.md (0x301–0x303)
// =============================================================================

#include "can_rx.h"
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include "can_ids.h"

namespace can_rx {

// -------------------------------------------------------------------------
// Internal helpers — little-endian 16/32-bit extraction
// -------------------------------------------------------------------------
static inline uint16_t readU16LE(const uint8_t* buf) {
    return static_cast<uint16_t>(buf[0]) |
           (static_cast<uint16_t>(buf[1]) << 8);
}

static inline int16_t readS16LE(const uint8_t* buf) {
    return static_cast<int16_t>(readU16LE(buf));
}

static inline uint32_t readU32LE(const uint8_t* buf) {
    return static_cast<uint32_t>(buf[0])       |
           (static_cast<uint32_t>(buf[1]) << 8)  |
           (static_cast<uint32_t>(buf[2]) << 16) |
           (static_cast<uint32_t>(buf[3]) << 24);
}

// -------------------------------------------------------------------------
// Frame decoders — one per CAN ID
// -------------------------------------------------------------------------

static void decodeHeartbeat(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 5) return;
    vehicle::HeartbeatData hb;
    hb.aliveCounter = f.data[0];
    hb.systemState  = static_cast<can::SystemState>(f.data[1]);
    hb.faultFlags   = f.data[2];
    hb.errorCode    = f.data[3];
    hb.statusFlags  = f.data[4];
    // Byte 5: relay status (DLC >= 6) — backward compatible with DLC 5
    hb.relayStatus  = (f.data_length_code >= 6) ? f.data[5] : 0;
    hb.timestampMs  = millis();
    data.setHeartbeat(hb);
}

static void decodeSpeed(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 8) return;
    vehicle::SpeedData sd;
    sd.raw[0] = readU16LE(&f.data[0]);   // FL
    sd.raw[1] = readU16LE(&f.data[2]);   // FR
    sd.raw[2] = readU16LE(&f.data[4]);   // RL
    sd.raw[3] = readU16LE(&f.data[6]);   // RR
    sd.timestampMs = millis();
    data.setSpeed(sd);
}

static void decodeCurrent(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 8) return;
    vehicle::CurrentData cd;
    cd.raw[0] = readU16LE(&f.data[0]);   // FL
    cd.raw[1] = readU16LE(&f.data[2]);   // FR
    cd.raw[2] = readU16LE(&f.data[4]);   // RL
    cd.raw[3] = readU16LE(&f.data[6]);   // RR
    cd.timestampMs = millis();
    data.setCurrent(cd);
}

static void decodeTemp(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 5) return;
    vehicle::TempData td;
    for (uint8_t i = 0; i < vehicle::NUM_TEMP_SENSORS; ++i) {
        td.temps[i] = static_cast<int8_t>(f.data[i]);
    }
    td.timestampMs = millis();
    data.setTemp(td);
}

static void decodeSafety(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 5) return;
    vehicle::SafetyData sd;
    sd.absActive  = f.data[0];
    sd.tcsActive  = f.data[1];
    sd.errorCode  = f.data[2];
    // Byte 3 (STM32 system state): intentionally not stored — state is already
    // available in HeartbeatData::systemState (0x200) which is more authoritative.
    // Byte 4 (rx_errors counter): intentionally not stored — no ESP32 consumer
    // currently requires per-packet CAN error counts from this frame.
    // Byte 5 (peak 100 Hz task duration ×100 µs): forward-compatible — only
    // populated when DLC ≥ 6.  Pre-extension STM32 firmware (DLC 5) leaves
    // peakLoop100us at zero, which the HMI renders as "n/a".
    if (f.data_length_code >= 6) {
        sd.peakLoop100us = f.data[5];
    }
    sd.timestampMs = millis();
    data.setSafety(sd);
}

static void decodeSteering(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 3) return;
    vehicle::SteeringData sd;
    sd.angleRaw   = readS16LE(&f.data[0]);  // 0.1° units
    sd.calibrated = f.data[2];
    sd.timestampMs = millis();
    data.setSteering(sd);
}

static void decodeTraction(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 4) return;
    vehicle::TractionData td;
    for (uint8_t i = 0; i < vehicle::NUM_WHEELS; ++i) {
        td.scale[i] = (f.data[i] <= 100) ? f.data[i] : 100;
    }
    td.timestampMs = millis();
    data.setTraction(td);
}

static void decodeTempMap(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 5) return;
    vehicle::TempMapData tm;
    for (uint8_t i = 0; i < vehicle::NUM_TEMP_SENSORS; ++i) {
        tm.temps[i] = static_cast<int8_t>(f.data[i]);
    }
    tm.timestampMs = millis();
    data.setTempMap(tm);
}

static void decodeDiagError(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 2) return;
    vehicle::DiagData dd;
    dd.errorCode  = f.data[0];
    dd.subsystem  = f.data[1];
    dd.timestampMs = millis();
    data.setDiag(dd);
}

static void decodeServiceFaults(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 4) return;
    data.setServiceFaults(readU32LE(&f.data[0]), millis());
}

static void decodeServiceEnabled(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 4) return;
    data.setServiceEnabled(readU32LE(&f.data[0]), millis());
}

static void decodeServiceDisabled(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 4) return;
    data.setServiceDisabled(readU32LE(&f.data[0]), millis());
}

static void decodeBattery(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 4) return;
    vehicle::BatteryData bd;
    bd.currentRaw  = readU16LE(&f.data[0]);   // 0.01 A units
    bd.voltageRaw  = readU16LE(&f.data[2]);   // 0.01 V units
    bd.timestampMs = millis();
    data.setBattery(bd);
}

static void decodeCommandAck(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 3) return;
    vehicle::AckData ad;
    ad.cmdIdLow    = f.data[0];
    ad.result      = static_cast<can::AckResult>(f.data[1]);
    ad.systemState = static_cast<can::SystemState>(f.data[2]);
    ad.timestampMs = millis();
    data.setAck(ad);
}

static void decodeLights(const CanFrame& f, vehicle::VehicleData& data) {
    // DLC >= 1: accept single-byte commands (byte0=front) for backward compatibility.
    // DLC >= 2: byte1 = rear relay.
    if (f.data_length_code < 1) return;
    vehicle::LightsData ld;
    ld.frontRelayOn = (f.data[0] != 0);
    ld.rearRelayOn  = (f.data_length_code >= 2) ? (f.data[1] != 0) : false;
    ld.timestampMs  = millis();
    data.setLights(ld);
}

static void decodeDebounceDiag(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 8) return;
    vehicle::DebounceDiagData dd = data.debounceDiag();   // preserve steer count
    dd.wheelFiltered[0] = readU16LE(&f.data[0]);   // FL
    dd.wheelFiltered[1] = readU16LE(&f.data[2]);   // FR
    dd.wheelFiltered[2] = readU16LE(&f.data[4]);   // RL
    dd.wheelFiltered[3] = readU16LE(&f.data[6]);   // RR
    dd.timestampMs      = millis();
    data.setDebounceDiag(dd);
}

static void decodeDebounceDiagSteer(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 4) return;
    vehicle::DebounceDiagData dd = data.debounceDiag();   // preserve wheel counts
    dd.steerFiltered = readU32LE(&f.data[0]);
    dd.timestampMs   = millis();
    data.setDebounceDiag(dd);
}

// 0x308 DIAG_PEDAL_CAL — pedal calibration telemetry (DLC 8).
// Frame layout mirrors Core/Src/can_handler.c pedalcal_send_status().
// Bit 6 of flags selects whether bytes 3-6 carry the STORED or PENDING
// endpoint pair; the decoder preserves the other pair across frames so
// the UI always has the most recent value for both.
static void decodePedalCal(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 8) return;
    vehicle::PedalCalData pc = data.pedalCal();   // preserve untouched pair
    pc.flags        = f.data[0];
    pc.rawAdc       = readU16LE(&f.data[1]);
    uint16_t mn     = readU16LE(&f.data[3]);
    uint16_t mx     = readU16LE(&f.data[5]);
    if (pc.flags & 0x40U) {
        pc.storedMin = mn;
        pc.storedMax = mx;
    } else {
        pc.pendingMin = mn;
        pc.pendingMax = mx;
    }
    pc.pedalPercent = f.data[7];
    pc.timestampMs  = millis();
    data.setPedalCal(pc);
}

// 0x309 DIAG_I2C — I2C topology diagnostic (DLC 5, extended to DLC 6).
// Frame layout mirrors Core/Src/can_handler.c CAN_SendI2CDiag().
//   byte0=mux_present, byte1=ina_ok_mask, byte2=fail_count,
//   byte3=recovery_attempts, byte4=flags (bit0 = ever OK),
//   byte5=ina_expected_mask (bit i = ch i's branch powered this phase).
//   byte5 is populated when DLC >= 6.  Pre-extension STM32 firmware (DLC 5)
//   leaves the expected mask at its default (all channels) so the legacy
//   FAIL-on-missing rendering is preserved.
//
// Per-ID RX counters (audit questions E/F): record every 0x309 frame seen on
// the bus and the last DLC, and count frames dropped for a short DLC.  This
// lets the operator tell "0 frames ever" (STM32 not emitting / CAN link down)
// apart from "frames arrive but with the wrong DLC" (decode rejects them).
static uint32_t s_rx0x309Count      = 0;  // total 0x309 frames seen (any DLC)
static uint32_t s_dropped0x309Dlc   = 0;  // 0x309 frames rejected for DLC < 5
static uint8_t  s_last0x309Dlc      = 0;  // DLC of the most recent 0x309 frame

static void decodeI2cDiag(const CanFrame& f, vehicle::VehicleData& data) {
    ++s_rx0x309Count;
    s_last0x309Dlc = f.data_length_code;
    if (f.data_length_code < 5) {
        ++s_dropped0x309Dlc;
        return;
    }
    vehicle::I2cDiagData id;
    id.muxPresent    = (f.data[0] != 0);
    id.inaOkMask     = f.data[1];
    id.failCount     = f.data[2];
    id.recoveryCount = f.data[3];
    id.everOk        = (f.data[4] & 0x01U) != 0;
    // byte5 (DLC >= 6): per-channel "expected powered" mask.  When absent
    // (legacy DLC-5 firmware), keep the default all-channels mask so a missing
    // INA still renders as FAIL rather than being silently hidden.
    if (f.data_length_code >= 6) {
        id.inaExpectedMask = f.data[5];
    }
    id.valid         = true;
    id.timestampMs   = millis();
    data.setI2cDiag(id);
}

// 0x30A DIAG_CAN_META — CAN/0x309 delivery meta-diagnostic (DLC 8).
// Frame layout mirrors Core/Src/can_handler.c CAN_SendCanMetaDiag().
static void decodeCanMeta(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 8) return;
    vehicle::CanMetaData m;
    m.diag309CallCount = (uint16_t)(f.data[0] | (f.data[1] << 8));
    m.tick1000msCount  = (uint16_t)(f.data[2] | (f.data[3] << 8));
    m.diag309TxOk      = f.data[4];
    m.diag309TxErr     = f.data[5];
    m.txFifoFullDrops  = f.data[6];
    m.fdcanInitOk      = (f.data[7] & 0x01U) != 0;
    m.valid            = true;
    m.timestampMs      = millis();
    data.setCanMeta(m);
}

// 0x30B DIAG_I2C_SCAN — I2C service-mode scan report (DLC 8).
// Frame layout mirrors Core/Src/can_handler.c CAN_SendI2CScanReport().
static void decodeI2cScan(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 5) return;
    vehicle::I2cScanData s;
    s.sclIdleHigh       = (f.data[0] & 0x01U) != 0;
    s.sdaIdleHigh       = (f.data[0] & 0x02U) != 0;
    s.recoveryAttempted = (f.data[0] & 0x04U) != 0;
    s.recoverySuccess   = (f.data[0] & 0x08U) != 0;
    s.muxPresent        = (f.data[1] != 0);
    s.inaPresentMask    = f.data[2];
    s.failCount         = f.data[3];
    s.recoveryAttempts  = f.data[4];
    s.valid             = true;
    s.timestampMs       = millis();
    data.setI2cScan(s);
}

// 0x30C DIAG_FDCAN — FDCAN error-counter dump (DLC 6).
// Frame layout mirrors Core/Src/can_handler.c CAN_SendFdcanDiag().
static void decodeFdcanDiag(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 6) return;
    vehicle::FdcanDiagData d;
    d.lastErrorCode = f.data[0];
    d.errorPassive  = (f.data[1] & 0x01U) != 0;
    d.busOff        = (f.data[1] & 0x02U) != 0;
    d.warning       = (f.data[1] & 0x04U) != 0;
    d.tec           = f.data[2];
    d.rec           = f.data[3];
    d.txNackFlag    = f.data[4];
    d.txConsecFail  = f.data[5];
    d.valid         = true;
    d.timestampMs   = millis();
    data.setFdcanDiag(d);
}

// -------------------------------------------------------------------------
// Debug: RX frame counter and first-frame logging
// -------------------------------------------------------------------------
static uint32_t rxFrameCount  = 0;  // Total CAN frames received since boot
static uint32_t lastRxLogMs   = 0;  // Timestamp of last periodic RX log

// Log the first CAN_RX_DEBUG_FIRST_N frames after boot for diagnostics
static constexpr uint32_t CAN_RX_DEBUG_FIRST_N = 10;

// Periodic RX statistics log interval (ms)
static constexpr uint32_t CAN_RX_STATS_INTERVAL_MS = 10000;

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void poll(vehicle::VehicleData& data) {
    CanFrame frame;
    while (ESP32Can.readFrame(frame, 0)) {
        ++rxFrameCount;

        // Debug: log first N received frames so the serial console shows
        // CAN bus activity (or lack thereof) immediately after power-on.
        if (rxFrameCount <= CAN_RX_DEBUG_FIRST_N) {
            Serial.printf("[CAN-RX] #%lu ID=0x%03lX DLC=%u data=[",
                          (unsigned long)rxFrameCount,
                          (unsigned long)frame.identifier,
                          (unsigned)frame.data_length_code);
            for (uint8_t i = 0; i < frame.data_length_code && i < 8; ++i) {
                Serial.printf("%s%02X", i ? " " : "", frame.data[i]);
            }
            Serial.println("]");
        }

        switch (frame.identifier) {
            case can::HEARTBEAT_STM32:  decodeHeartbeat(frame, data);      break;
            case can::STATUS_SPEED:     decodeSpeed(frame, data);          break;
            case can::STATUS_CURRENT:   decodeCurrent(frame, data);        break;
            case can::STATUS_TEMP:      decodeTemp(frame, data);           break;
            case can::STATUS_SAFETY:    decodeSafety(frame, data);         break;
            case can::STATUS_STEERING:  decodeSteering(frame, data);       break;
            case can::STATUS_TRACTION:  decodeTraction(frame, data);       break;
            case can::STATUS_TEMP_MAP:  decodeTempMap(frame, data);        break;
            case can::STATUS_BATTERY:   decodeBattery(frame, data);        break;
            case can::CMD_ACK:          decodeCommandAck(frame, data);     break;
            case can::DIAG_ERROR:       decodeDiagError(frame, data);      break;
            case can::STATUS_LIGHTS:    decodeLights(frame, data);         break;
            case can::SERVICE_FAULTS:   decodeServiceFaults(frame, data);  break;
            case can::SERVICE_ENABLED:  decodeServiceEnabled(frame, data); break;
            case can::SERVICE_DISABLED: decodeServiceDisabled(frame, data);break;
            case can::DIAG_DEBOUNCE:        decodeDebounceDiag(frame, data);      break;
            case can::DIAG_DEBOUNCE_STEER:  decodeDebounceDiagSteer(frame, data); break;
            case can::DIAG_PEDAL_CAL:       decodePedalCal(frame, data);          break;
            case can::DIAG_I2C:             decodeI2cDiag(frame, data);           break;
            case can::DIAG_CAN_META:        decodeCanMeta(frame, data);           break;
            case can::DIAG_I2C_SCAN:        decodeI2cScan(frame, data);           break;
            case can::DIAG_FDCAN:           decodeFdcanDiag(frame, data);         break;
            default:
                // Unknown CAN ID — silently ignored
                break;
        }
    }

    // Periodic RX statistics log (every CAN_RX_STATS_INTERVAL_MS).
    // Unsigned subtraction handles millis() wrap-around (~49 days) correctly.
    unsigned long nowMs = millis();
    if (nowMs - lastRxLogMs >= CAN_RX_STATS_INTERVAL_MS) {
        lastRxLogMs = nowMs;
        Serial.printf("[CAN-RX] total_frames=%lu\n",
                      (unsigned long)rxFrameCount);
    }
}

// -------------------------------------------------------------------------
// Per-ID 0x309 RX counters (audit questions E/F) — exposed for the
// engineering CAN/I2C diagnostic submenu and the Safe Mode hint logic.
// -------------------------------------------------------------------------
uint32_t rx0x309Count()       { return s_rx0x309Count; }
uint32_t dropped0x309Dlc()    { return s_dropped0x309Dlc; }
uint8_t  last0x309Dlc()       { return s_last0x309Dlc; }

} // namespace can_rx
