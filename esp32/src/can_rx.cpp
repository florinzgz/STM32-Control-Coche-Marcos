// =============================================================================
// ESP32-S3 HMI — CAN RX Module (implementation)
//
// Decodes CAN frames EXACTLY as specified in CAN_CONTRACT_FINAL.md rev 1.0.
// Pushes decoded values into the VehicleData store.
// Unknown CAN IDs are silently ignored.
//
// Reference: docs/CAN_CONTRACT_FINAL.md rev 1.0
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
    if (f.data_length_code < 4) return;
    vehicle::HeartbeatData hb;
    hb.aliveCounter = f.data[0];
    hb.systemState  = static_cast<can::SystemState>(f.data[1]);
    hb.faultFlags   = f.data[2];
    hb.reserved     = f.data[3];
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
    if (f.data_length_code < 3) return;
    vehicle::SafetyData sd;
    sd.absActive  = f.data[0];
    sd.tcsActive  = f.data[1];
    sd.errorCode  = f.data[2];
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
    td.scale[0] = f.data[0];   // FL
    td.scale[1] = f.data[1];   // FR
    td.scale[2] = f.data[2];   // RL
    td.scale[3] = f.data[3];   // RR
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

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

void poll(vehicle::VehicleData& data) {
    CanFrame frame;
    while (ESP32Can.readFrame(frame, 0)) {
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
            case can::DIAG_ERROR:       decodeDiagError(frame, data);      break;
            case can::SERVICE_FAULTS:   decodeServiceFaults(frame, data);  break;
            case can::SERVICE_ENABLED:  decodeServiceEnabled(frame, data); break;
            case can::SERVICE_DISABLED: decodeServiceDisabled(frame, data);break;
            default:
                // Unknown CAN ID — silently ignored
                break;
        }
    }
}

} // namespace can_rx
