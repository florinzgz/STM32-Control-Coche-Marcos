// =============================================================================
// ESP32-S3 HMI — Wheel-diagnostic hint line for Pedal Calibration
//
// Compact, non-invasive helper that turns the 0x313 DIAG_WHEEL_SENSOR frame
// (already decoded into WheelSensorDiagData) into a single "WD: ..." line the
// Pedal Calibration screen shows under "Safety gate".  It explains *why* the
// safety gate is BLOCKED when the cause is a wheel-sensor fault — without
// changing any safety logic (the gate itself is enforced server-side on STM32).
//
// This header is intentionally free of Arduino/TFT dependencies so the mapping
// can be exercised on host g++ (see esp32/src/test_wheel_diag_hint.cpp).
//
// Short mnemonics (per the HMI interpretation spec for this line):
//   OK -> OK   NO_PULSE -> NP     STUCK_HIGH -> S-HI   STUCK_LOW -> S-LO
//   MISMATCH -> MIS   IMPOSSIBLE_RATE -> RATE   MANUAL_MOVEMENT -> MAN
//   DISABLED_STATE -> DIS   UNKNOWN -> UNK
// =============================================================================

#ifndef UI_WHEEL_DIAG_HINT_H
#define UI_WHEEL_DIAG_HINT_H

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "can_ids.h"
#include "ui_common.h"

namespace ui {

// Severity class of the hint line → drives the text colour.  Kept separate from
// the RGB565 value so the mapping stays host-testable without TFT constants.
enum class WheelHintKind : uint8_t {
    OK,      // all channels OK/DISABLED  → green
    INFO,    // manual hand-movement      → cyan (expected, never a red alarm)
    WARN,    // historic DTC, no live flt → amber
    FAULT,   // active per-channel fault  → red
    STALE,   // 0x313 older than 2 s      → soft amber
    NODATA   // 0x313 never seen          → gray
};

// Compact reason mnemonic for the Pedal-Calibration wheel-diag hint line.
inline const char* wheelReasonShort(uint8_t reason) {
    switch (reason) {
        case can::WHEEL_DIAG_REASON_OK:               return "OK";
        case can::WHEEL_DIAG_REASON_NO_PULSE:         return "NP";
        case can::WHEEL_DIAG_REASON_STUCK_HIGH:       return "S-HI";
        case can::WHEEL_DIAG_REASON_STUCK_LOW:        return "S-LO";
        case can::WHEEL_DIAG_REASON_MISMATCH:         return "MIS";
        case can::WHEEL_DIAG_REASON_IMPOSSIBLE_RATE:  return "RATE";
        case can::WHEEL_DIAG_REASON_MANUAL_MOVEMENT:  return "MAN";
        case can::WHEEL_DIAG_REASON_DISABLED_STATE:   return "DIS";
        default:                                      return "UNK";
    }
}

// RGB565 colour for a given hint severity.
inline uint16_t wheelHintColor(WheelHintKind k) {
    switch (k) {
        case WheelHintKind::OK:     return ui::COL_GREEN;
        case WheelHintKind::INFO:   return ui::COL_CYAN;
        case WheelHintKind::WARN:   return ui::COL_AMBER;
        case WheelHintKind::FAULT:  return ui::COL_RED;
        case WheelHintKind::STALE:  return ui::COL_AMBER;
        case WheelHintKind::NODATA: return ui::COL_GRAY;
    }
    return ui::COL_GRAY;
}

// -------------------------------------------------------------------------
// buildWheelBlockText — compose the "WD: ..." hint into out[0..n).
//
//   reason[5]         FL,FR,RL,RR,STEER reason codes (WHEEL_DIAG_REASON_*)
//   faultMask         per-channel active-fault bitmask (bit0 FL .. bit4 ST)
//   valid             WheelSensorDiagData.valid (0x313 ever seen)
//   ageMs             now - timestampMs of the last 0x313 frame
//   hasSensorFaultDtc a SENSOR_FAULT (0x10) DTC is latched (heartbeat/0x313)
//
// Decision order (highest priority first):
//   1. !valid                 -> "WD: NO DATA"   (NODATA)
//   2. ageMs > 2000 ms        -> "WD: STALE"     (STALE)
//   3. faultMask != 0         -> "WD: FR MIS ..."(FAULT)  one segment/channel
//   4. hasSensorFaultDtc      -> "WD: CLR DTC"   (WARN)   historic, not live
//   5. any reason == MANUAL   -> "WD: MAN"       (INFO)   hand-spin, not a fault
//   6. otherwise (OK/DIS)     -> "WD: OK"        (OK)
//
// Never renders MANUAL_MOVEMENT or DISABLED_STATE as a red alarm.  Output is
// always NUL-terminated and bounded by n (no overflow).
// -------------------------------------------------------------------------
inline WheelHintKind buildWheelBlockText(
        const uint8_t reason[5], uint8_t faultMask,
        bool valid, unsigned long ageMs, bool hasSensorFaultDtc,
        char* out, size_t n) {
    if (out == nullptr || n == 0) return WheelHintKind::NODATA;

    // 1. No 0x313 frame has been received yet.
    if (!valid) {
        snprintf(out, n, "WD: NO DATA");
        return WheelHintKind::NODATA;
    }

    // 2. Frame is stale (>2 s without a fresh 0x313).
    if (ageMs > 2000UL) {
        snprintf(out, n, "WD: STALE");
        return WheelHintKind::STALE;
    }

    // 3. One or more channels report an active fault — name each of them.
    //    Append each " CH REASON" segment with a bounded snprintf that writes
    //    at out+used with the exact remaining space (n-used).  snprintf always
    //    NUL-terminates and never writes past the buffer, so this stays safe
    //    even if n is smaller than the composed line (silent truncation only,
    //    never overflow).  We stop as soon as the buffer is full.
    if (faultMask != 0) {
        static const char* const kLbl[5] = { "FL", "FR", "RL", "RR", "ST" };
        size_t used = 0;
        int w = snprintf(out, n, "WD:");
        if (w > 0) {
            used = (static_cast<size_t>(w) < n) ? static_cast<size_t>(w) : (n - 1);
        }
        for (uint8_t i = 0; i < 5; ++i) {
            if ((faultMask & (1U << i)) == 0) continue;
            if (used + 1 >= n) break;  // no room left (keep the NUL)
            int m = snprintf(out + used, n - used, " %s %s",
                             kLbl[i], wheelReasonShort(reason[i]));
            if (m < 0) break;
            if (static_cast<size_t>(m) >= n - used) {
                used = n - 1;  // truncated: buffer is now full → stop
                break;
            }
            used += static_cast<size_t>(m);
        }
        return WheelHintKind::FAULT;
    }

    // 4. Historic SENSOR_FAULT DTC latched but no live channel fault: the
    //    operator can clear it (RESET WHEEL SENSORS / reboot) — not a live stop.
    if (hasSensorFaultDtc) {
        snprintf(out, n, "WD: CLR DTC");
        return WheelHintKind::WARN;
    }

    // 5. Manual hand-movement on a wheel (expected — never a red alarm).
    for (uint8_t i = 0; i < 5; ++i) {
        if (reason[i] == can::WHEEL_DIAG_REASON_MANUAL_MOVEMENT) {
            snprintf(out, n, "WD: MAN");
            return WheelHintKind::INFO;
        }
    }

    // 6. Everything OK / DISABLED.
    snprintf(out, n, "WD: OK");
    return WheelHintKind::OK;
}

}  // namespace ui

#endif  // UI_WHEEL_DIAG_HINT_H
