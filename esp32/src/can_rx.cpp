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

static uint32_t s_rx0x001Count = 0;  // total 0x001 heartbeat frames seen
static uint32_t s_rx0x103Count = 0;  // total 0x103 CMD_ACK frames seen

static void decodeHeartbeat(const CanFrame& f, vehicle::VehicleData& data) {
    ++s_rx0x001Count;
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

// 0x207 STATUS_BATTERY reception counters (points 4+5 — battery diag)
static uint32_t s_rx0x207Count     = 0;  // total 0x207 frames seen (any DLC)
static uint32_t s_dropped0x207Dlc  = 0;  // 0x207 frames with DLC < 4 (discarded)
static uint8_t  s_last0x207Dlc     = 0;  // DLC of the most recent 0x207 frame

static void decodeBattery(const CanFrame& f, vehicle::VehicleData& data) {
    const unsigned long nowMs = millis();
    ++s_rx0x207Count;
    s_last0x207Dlc = f.data_length_code;
    const bool shortDlc = (f.data_length_code < 4);
    if (shortDlc) { ++s_dropped0x207Dlc; }
    if (!shortDlc) {
        vehicle::BatteryData bd;
        bd.currentRaw  = readU16LE(&f.data[0]);   // 0.01 A units
        bd.voltageRaw  = readU16LE(&f.data[2]);   // 0.01 V units
        bd.timestampMs = nowMs;
        data.setBattery(bd);
    }
    /* Publish 0x207 diagnostic counters on EVERY reception — including
     * short-DLC frames that are discarded — so the HMI drop counter stays
     * visible even when all incoming 0x207 frames are malformed (otherwise
     * the battery screen could show NO PACKET/STALE while frames arrive).  */
    vehicle::Batt207DiagData diag;
    diag.rxCount     = s_rx0x207Count;
    diag.lastDlc     = s_last0x207Dlc;
    diag.droppedDlc  = s_dropped0x207Dlc;
    diag.valid       = true;
    diag.timestampMs = nowMs;
    data.setBatt207Diag(diag);
}

static void decodeCommandAck(const CanFrame& f, vehicle::VehicleData& data) {
    ++s_rx0x103Count;
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

static void decodePedal(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 1) return;
    vehicle::PedalData pd;
    pd.percent     = (f.data[0] <= 100) ? f.data[0] : 100;
    // Extended layout (DLC >= 4, additive): fault flags + raw ADC.  Legacy
    // single-byte frames leave plausible=true/contradictory=false defaults.
    if (f.data_length_code >= 4) {
        pd.plausible     = (f.data[1] & 0x01u) != 0;
        pd.contradictory = (f.data[1] & 0x02u) != 0;
        pd.rawAdc        = static_cast<uint16_t>(f.data[2] | (f.data[3] << 8));
        pd.extended      = true;
    }
    pd.timestampMs = millis();
    data.setPedal(pd);
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

static void decodeWheelPulses(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 8) return;
    vehicle::DebounceDiagData dd = data.debounceDiag();   // preserve rejected counts
    dd.wheelValid[0] = readU16LE(&f.data[0]);   // FL
    dd.wheelValid[1] = readU16LE(&f.data[2]);   // FR
    dd.wheelValid[2] = readU16LE(&f.data[4]);   // RL
    dd.wheelValid[3] = readU16LE(&f.data[6]);   // RR
    dd.validTimestampMs = millis();
    data.setDebounceDiag(dd);
}

// 0x308 DIAG_PEDAL_CAL — pedal calibration telemetry (DLC 8).
// Frame layout mirrors Core/Src/can_handler.c pedalcal_send_status().
// Bit 7 distinguishes pair frames from extended diagnostic frames.
// Bit 6 selects STORED vs PENDING only on pair frames.  The decoder
// preserves untouched fields across burst variants so the UI always has
// the latest pair values plus the latest raw2/diff/reject diagnostic.
static void decodePedalCal(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 8) return;
    vehicle::PedalCalData pc = data.pedalCal();   // preserve untouched pair
    pc.flags        = f.data[0];
    if (pc.flags & 0x80U) {
        pc.rawAdc2      = readU16LE(&f.data[1]);
        pc.diffRaw      = readU16LE(&f.data[3]);
        pc.rejectReason = readU16LE(&f.data[5]);
    } else {
        pc.rawAdc   = readU16LE(&f.data[1]);
        uint16_t mn = readU16LE(&f.data[3]);
        uint16_t mx = readU16LE(&f.data[5]);
        if (pc.flags & 0x40U) {
            pc.storedMin = mn;
            pc.storedMax = mx;
        } else {
            pc.pendingMin = mn;
            pc.pendingMax = mx;
        }
    }
    pc.pedalPercent = f.data[7];
    pc.timestampMs  = millis();
    data.setPedalCal(pc);
}

// 0x309 DIAG_I2C — I2C topology diagnostic (DLC 8; legacy DLC 5/6 accepted).
// Frame layout mirrors Core/Src/can_handler.c CAN_SendI2CDiag().
//   byte0=mux_present, byte1=ina_ok_mask, byte2=fail_count,
//   byte3=recovery_attempts, byte4=flags (bit0 = ever OK),
//   byte5=ina_expected_mask (bit i = ch i's branch powered this phase),
//   byte6=i2c_last_read_ms (last Current_ReadAll() duration, saturated 255),
//   byte7=reserved.
//   byte5 is populated when DLC >= 6; byte6 when DLC >= 7.  Pre-extension STM32
//   firmware (DLC 5) leaves the expected mask at its default (all channels) so
//   the legacy FAIL-on-missing rendering is preserved.
//
// Per-ID RX counters (audit questions E/F): record every 0x309 frame seen on
// the bus and the last DLC, and count frames dropped for a short DLC.  This
// lets the operator tell "0 frames ever" (STM32 not emitting / CAN link down)
// apart from "frames arrive but with the wrong DLC" (decode rejects them).
static uint32_t s_rx0x309Count      = 0;  // total 0x309 frames seen (any DLC)
static uint32_t s_dropped0x309Dlc   = 0;  // 0x309 frames rejected for DLC < 5
static uint8_t  s_last0x309Dlc      = 0;  // DLC of the most recent 0x309 frame
static uint32_t s_rx0x30BCount      = 0;  // total 0x30B frames seen (any DLC)
static uint32_t s_dropped0x30BDlc   = 0;  // 0x30B frames rejected for DLC < 5
static uint8_t  s_last0x30BDlc      = 0;  // DLC of the most recent 0x30B frame
static uint32_t s_rx0x30CCount      = 0;  // total 0x30C frames seen (any DLC)

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
    // byte6 (DLC >= 7): duration of last Current_ReadAll() in ms (saturated 255).
    // Zero when absent (legacy firmware with DLC <= 6).
    if (f.data_length_code >= 7) {
        id.i2cReadMs = f.data[6];
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
    // bits 7:1 of byte 7: hb_tx_err (heartbeat TX failures, saturated at 127)
    m.hbTxErr          = (uint8_t)((f.data[7] >> 1) & 0x7FU);
    m.valid            = true;
    m.timestampMs      = millis();
    data.setCanMeta(m);
}

// 0x312 DIAG_BOOT_RESET — boot/reset diagnostic (DLC 8, 1 Hz).
// Frame layout mirrors Core/Src/can_handler.c CAN_SendBootResetDiag().
static void decodeBootReset(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 5) return;
    vehicle::BootResetData br;
    br.uptimeMs   = readU32LE(&f.data[0]);  // bytes 0-3: HAL_GetTick() uptime
    br.resetCause = f.data[4];              // byte 4: RESET_CAUSE_* bitmask
    // bytes 5-6: software TX queue depth (now / high-water). Present only on
    // DLC-8 frames; older/short frames leave these at their 0 defaults.
    if (f.data_length_code >= 7) {
        br.txQueueDepth    = f.data[5];
        br.txQueueDepthMax = f.data[6];
    }
    br.valid      = true;
    br.timestampMs = millis();
    data.setBootReset(br);
}

// 0x313 DIAG_WHEEL_SENSOR — per-wheel speed-sensor fault-reason diagnostic.
// Frame layout mirrors Core/Src/can_handler.c CAN_SendWheelSensorDiag().
// Accepts DLC >= 7 (reasons + gpio + fault mask) and DLC 8 (adds flags byte).
// Short frames are counted and dropped so a truncated payload never renders
// stale/garbage reasons.
static uint32_t s_rx0x313Count    = 0;  // total 0x313 frames seen (any DLC)
static uint32_t s_dropped0x313Dlc = 0;  // 0x313 frames rejected for DLC < 7
static uint8_t  s_last0x313Dlc    = 0;  // DLC of the most recent 0x313 frame

static void decodeWheelSensorDiag(const CanFrame& f, vehicle::VehicleData& data) {
    ++s_rx0x313Count;
    s_last0x313Dlc = f.data_length_code;
    if (f.data_length_code < 7) { ++s_dropped0x313Dlc; return; }

    vehicle::WheelSensorDiagData wd;
    for (uint8_t i = 0; i < 5; ++i) {
        wd.reason[i] = f.data[i];           // FL,FR,RL,RR,STEER reason codes
    }
    wd.gpioMask  = f.data[5];
    wd.faultMask = f.data[6];
    // byte 7 (flags/sequence) is present only on DLC-8 frames; leave 0 otherwise.
    if (f.data_length_code >= 8) {
        wd.flags = f.data[7];
    }
    wd.valid      = true;
    wd.timestampMs = millis();
    data.setWheelSensorDiag(wd);
}

// 0x315 DIAG_MOTION_INHIBIT — MOTION_INHIBIT_REASON instrumentation (DLC 8).
// Frame layout mirrors Core/Src/can_handler.c CAN_SendMotionInhibit().
static void decodeMotionInhibit(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 8) return;
    vehicle::MotionInhibitData mi;
    mi.reason         = readU16LE(&f.data[0]);
    mi.systemState    = f.data[2];
    mi.gear           = f.data[3];
    mi.demandPct      = (int8_t)f.data[4];
    mi.effectivePct   = (int8_t)f.data[5];
    mi.finalPwmPct    = f.data[6];
    mi.powerReady     = (f.data[7] & 0x01U) != 0;
    mi.obstacleFwdBlk = (f.data[7] & 0x02U) != 0;
    mi.degradedLevel  = (uint8_t)((f.data[7] >> 4) & 0x0FU);
    mi.valid          = true;
    mi.timestampMs    = millis();
    data.setMotionInhibit(mi);
}

// 0x30B DIAG_I2C_SCAN — I2C service-mode scan report (DLC 8).
// Frame layout mirrors Core/Src/can_handler.c CAN_SendI2CScanReport().
static void decodeI2cScan(const CanFrame& f, vehicle::VehicleData& data) {
    ++s_rx0x30BCount;
    s_last0x30BDlc = f.data_length_code;
    if (f.data_length_code < 5) {
        ++s_dropped0x30BDlc;
        return;
    }
    vehicle::I2cScanData s;
    s.sclIdleHigh       = (f.data[0] & 0x01U) != 0;
    s.sdaIdleHigh       = (f.data[0] & 0x02U) != 0;
    s.recoveryAttempted = (f.data[0] & 0x04U) != 0;
    s.recoverySuccess   = (f.data[0] & 0x08U) != 0;
    s.muxPresent        = (f.data[1] != 0);
    s.inaPresentMask    = f.data[2];
    s.failCount         = f.data[3];
    s.recoveryAttempts  = f.data[4];
    // Byte 5 (additive): terminal I2C scan phase.  Only present on DLC>=6
    // frames; older/short frames leave it at the default (UNKNOWN=0).
    s.scanPhase         = (f.data_length_code >= 6) ? f.data[5] : can::I2C_SCAN_PHASE_UNKNOWN;
    s.valid             = true;
    s.timestampMs       = millis();
    data.setI2cScan(s);
}

// 0x30C DIAG_FDCAN — FDCAN error-counter dump (DLC 6).
// Frame layout mirrors Core/Src/can_handler.c CAN_SendFdcanDiag().
static void decodeFdcanDiag(const CanFrame& f, vehicle::VehicleData& data) {
    ++s_rx0x30CCount;
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

// 0x30D DIAG_GEAR_LIMITS — gear power-limit + accel-response telemetry (DLC 8).
// Two interleaved frame kinds share this ID, distinguished by flags bit4:
//   bit4 = 0  POWER   frame: bytes 1-3 active power, 4-6 pending power
//   bit4 = 1  RESPONSE frame: bytes 1-3 active response %, 4-6 pending response %
// Each frame updates only its half; the other half is preserved so a QUERY
// (which sends both frames back-to-back) leaves the editor fully populated.
// Frame layout mirrors Core/Src/can_handler.c gearlim_send_status_kind().
static void decodeGearLimits(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 8) return;
    vehicle::GearLimitsData d = data.gearLimits();  // preserve the other half
    const bool responseFrame = (f.data[0] & 0x10) != 0;
    d.flags       = f.data[0];
    d.systemState = f.data[7];
    if (responseFrame) {
        d.activeRespD2  = f.data[1];
        d.activeRespD1  = f.data[2];
        d.activeRespR   = f.data[3];
        d.pendingRespD2 = f.data[4];
        d.pendingRespD1 = f.data[5];
        d.pendingRespR  = f.data[6];
    } else {
        d.activeD2  = f.data[1];
        d.activeD1  = f.data[2];
        d.activeR   = f.data[3];
        d.pendingD2 = f.data[4];
        d.pendingD1 = f.data[5];
        d.pendingR  = f.data[6];
    }
    d.timestampMs = millis();
    data.setGearLimits(d);
}

// 0x30E DIAG_STEERING_Z — PB5 + encoder-Z dual center-reference diagnostic (DLC 8).
// PB5 (LJ12A3) is the PRIMARY physical/safety center reference; the encoder Z
// (index) pulse is a SECONDARY precision reference and can NEVER center alone.
// Frame layout mirrors Core/Src/can_handler.c steerz_send_status().
static void decodeSteeringZ(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 8) return;
    vehicle::SteeringZData d;
    d.flags       = f.data[0];
    d.status      = (uint8_t)(f.data[0] & 0x07);
    d.pb5Live     = (f.data[0] & 0x08) != 0;
    d.zValid      = (f.data[0] & 0x10) != 0;
    d.zSlip       = (f.data[0] & 0x20) != 0;
    d.zPulseCount = f.data[1];
    d.zLastPos    = (int16_t)((uint16_t)f.data[2] | ((uint16_t)f.data[3] << 8));
    d.zOffset     = (int16_t)((uint16_t)f.data[4] | ((uint16_t)f.data[5] << 8));
    d.zLastError  = (int8_t)f.data[6];
    d.zTolerance  = f.data[7];
    d.timestampMs = millis();
    data.setSteeringZ(d);
}

// 0x30F DIAG_EPS_PARAMS — EPS parameter + live-state telemetry (DLC 8).
// Frame layout mirrors Core/Src/can_handler.c eps_send_all_kinds().
// Five frame kinds share the 0x30F ID; kind is byte0 bits 7-4.
static void decodeEpsParams(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 8) return;
    vehicle::EpsParamsData d = data.epsParams();  // preserve other kinds
    const uint8_t kind  = (uint8_t)(f.data[0] >> 4);
    const uint8_t flags = (uint8_t)(f.data[0] & 0x0FU);
    d.flashValid   = (flags & 0x01U) != 0;
    d.sysInStandby = (flags & 0x02U) != 0;

    auto rd16 = [&](int i) -> int16_t {
        return (int16_t)((uint16_t)f.data[i] | ((uint16_t)f.data[i + 1] << 8));
    };

    switch (kind) {
    case 0:
        d.assistStrength = rd16(1) / 1000.0f;
        d.centerStrength = rd16(3) / 1000.0f;
        d.damping        = rd16(5) / 1000.0f;
        d.kindsReceived |= 0x01U;
        break;
    case 1:
        d.frictionComp = rd16(1) / 1000.0f;
        d.coastBandPct = rd16(3) /  100.0f;
        d.minDrivePct  = rd16(5) /  100.0f;
        d.kindsReceived |= 0x02U;
        break;
    case 2:
        d.assistVsSpeed = rd16(1) /  10.0f;
        d.returnVsSpeed = rd16(3) /  10.0f;
        d.deadbandDeg   = rd16(5) / 100.0f;
        d.kindsReceived |= 0x04U;
        break;
    case 3:
        d.maxPwmPct       = rd16(1) / 100.0f;
        d.slewRatePct     = rd16(3) / 100.0f;
        d.centerOffsetDeg = rd16(5) / 100.0f;
        d.kindsReceived |= 0x08U;
        break;
    case 4:
        d.encRaw         = rd16(1);
        d.angleDeg       = rd16(3) / 10.0f;
        d.motorEffortPct = rd16(5) / 10.0f;
        d.steerState     = f.data[7];
        d.kindsReceived |= 0x10U;
        break;
    default:
        return;
    }
    d.valid       = true;
    d.timestampMs = millis();
    data.setEpsParams(d);
}

// 0x310 DIAG_DRIVE_TUNING — drive-tuning (ramp/creep) field-stream (DLC 8).
// One CAN frame per field; a QUERY emits 10 sweeps × all 6 fields at 10 Hz.
// Frame layout mirrors Core/Src/can_handler.c drvtune_send_field():
//   byte0=flags, byte1=field id (1..6), byte2-3=active u16 LE,
//   byte4-5=pending u16 LE, byte6=system_state, byte7=field count.
// Each frame updates only its field; the others are preserved so the editor
// stays fully populated.  Unknown field ids are ignored (partial data kept).
static void decodeDriveTuning(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 8) return;
    vehicle::DriveTuningData d = data.driveTuning();  // preserve other fields
    const uint8_t  field  = f.data[1];
    const uint16_t actVal = readU16LE(&f.data[2]);
    const uint16_t pndVal = readU16LE(&f.data[4]);
    switch (field) {
    case can::DRIVE_TUNE_FIELD_ACCEL_RAMP:
        d.accelRamp = actVal; d.pendAccelRamp = pndVal; break;
    case can::DRIVE_TUNE_FIELD_BRAKE_RAMP:
        d.brakeRamp = actVal; d.pendBrakeRamp = pndVal; break;
    case can::DRIVE_TUNE_FIELD_REVERSE_RAMP:
        d.reverseRamp = actVal; d.pendReverseRamp = pndVal; break;
    case can::DRIVE_TUNE_FIELD_CREEP_ENABLE:
        d.creepEnable = actVal; d.pendCreepEnable = pndVal; break;
    case can::DRIVE_TUNE_FIELD_CREEP_POWER:
        d.creepPower = actVal; d.pendCreepPower = pndVal; break;
    case can::DRIVE_TUNE_FIELD_CREEP_DELAY:
        d.creepDelay = actVal; d.pendCreepDelay = pndVal; break;
    default:
        return;   // unknown field — keep partial data, no timestamp bump
    }
    d.flags       = f.data[0];
    d.systemState = f.data[6];
    if (field >= 1 && field <= 8) d.fieldsSeen |= (uint8_t)(1U << (field - 1));
    d.valid       = true;
    d.timestampMs = millis();
    data.setDriveTuning(d);
}

// 0x311 DIAG_BATTERY_LIMITS — battery voltage-limit field-stream (DLC 8).
// One CAN frame per field; a QUERY emits 10 sweeps × all 5 fields at 10 Hz.
// Same per-field layout as 0x310; values are centivolts (V×100) or ms.
// Frame layout mirrors Core/Src/can_handler.c battlim_send_field().
static void decodeBatteryLimits(const CanFrame& f, vehicle::VehicleData& data) {
    if (f.data_length_code < 8) return;
    vehicle::BatteryLimitsData d = data.batteryLimits();  // preserve other fields
    const uint8_t  field  = f.data[1];
    const uint16_t actVal = readU16LE(&f.data[2]);
    const uint16_t pndVal = readU16LE(&f.data[4]);
    switch (field) {
    case can::BATT_LIM_FIELD_WARNING:
        d.warningCv = actVal; d.pendWarningCv = pndVal; break;
    case can::BATT_LIM_FIELD_LIMIT:
        d.limitCv = actVal; d.pendLimitCv = pndVal; break;
    case can::BATT_LIM_FIELD_CUTOFF:
        d.cutoffCv = actVal; d.pendCutoffCv = pndVal; break;
    case can::BATT_LIM_FIELD_RECOVERY:
        d.recoveryCv = actVal; d.pendRecoveryCv = pndVal; break;
    case can::BATT_LIM_FIELD_FILTER:
        d.filterMs = actVal; d.pendFilterMs = pndVal; break;
    default:
        return;   // unknown field — keep partial data
    }
    d.flags       = f.data[0];
    d.systemState = f.data[6];
    if (field >= 1 && field <= 8) d.fieldsSeen |= (uint8_t)(1U << (field - 1));
    d.valid       = true;
    d.timestampMs = millis();
    data.setBatteryLimits(d);
}
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
            case can::STATUS_PEDAL:     decodePedal(frame, data);          break;
            case can::SERVICE_FAULTS:   decodeServiceFaults(frame, data);  break;
            case can::SERVICE_ENABLED:  decodeServiceEnabled(frame, data); break;
            case can::SERVICE_DISABLED: decodeServiceDisabled(frame, data);break;
            case can::DIAG_DEBOUNCE:        decodeDebounceDiag(frame, data);      break;
            case can::DIAG_DEBOUNCE_STEER:  decodeDebounceDiagSteer(frame, data); break;
            case can::DIAG_WHEEL_PULSES:    decodeWheelPulses(frame, data);       break;
            case can::DIAG_PEDAL_CAL:       decodePedalCal(frame, data);          break;
            case can::DIAG_I2C:             decodeI2cDiag(frame, data);           break;
            case can::DIAG_CAN_META:        decodeCanMeta(frame, data);           break;
            case can::DIAG_BOOT_RESET:      decodeBootReset(frame, data);         break;
            case can::DIAG_WHEEL_SENSOR:    decodeWheelSensorDiag(frame, data);   break;
            case can::DIAG_MOTION_INHIBIT:  decodeMotionInhibit(frame, data);     break;
            case can::DIAG_I2C_SCAN:        decodeI2cScan(frame, data);           break;
            case can::DIAG_FDCAN:           decodeFdcanDiag(frame, data);         break;
            case can::DIAG_GEAR_LIMITS:     decodeGearLimits(frame, data);        break;
            case can::DIAG_STEERING_Z:      decodeSteeringZ(frame, data);         break;
            case can::DIAG_EPS_PARAMS:      decodeEpsParams(frame, data);         break;
            case can::DIAG_DRIVE_TUNING:    decodeDriveTuning(frame, data);       break;
            case can::DIAG_BATTERY_LIMITS:  decodeBatteryLimits(frame, data);     break;
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
uint32_t rx0x30BCount()       { return s_rx0x30BCount; }
uint32_t dropped0x30BDlc()    { return s_dropped0x30BDlc; }
uint8_t  last0x30BDlc()       { return s_last0x30BDlc; }
uint32_t rx0x30CCount()       { return s_rx0x30CCount; }
uint32_t rx0x001Count()       { return s_rx0x001Count; }
uint32_t rx0x103Count()       { return s_rx0x103Count; }
uint32_t rx0x207Count()       { return s_rx0x207Count; }
uint32_t dropped0x207Dlc()    { return s_dropped0x207Dlc; }
uint8_t  last0x207Dlc()       { return s_last0x207Dlc; }

uint32_t rx0x313Count()       { return s_rx0x313Count; }
uint32_t dropped0x313Dlc()    { return s_dropped0x313Dlc; }
uint8_t  last0x313Dlc()       { return s_last0x313Dlc; }

} // namespace can_rx
