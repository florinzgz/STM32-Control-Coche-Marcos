// =============================================================================
// ESP32-S3 HMI — 0x315 DIAG_MOTION_INHIBIT presentation helpers (pure)
//
// Pure, host-testable conversions used by the Engineering-screen
// "MOTION INHIBIT DIAG" page.  Deliberately free of Arduino / TFT / TWAI
// dependencies so the exact same code is exercised by the host unit test
// (esp32/src/test_motion_inhibit_view.cpp) and by the firmware.
//
//   * reason bitmask  -> ordered list of active reason labels
//   * relay phase code -> human text (IDLE / IN PROGRESS / COMPLETE / UNKNOWN)
//   * frame age        -> VALID / STALE / NEVER RECEIVED  (millis()-wrap safe)
//   * systemState      -> text + "does this state permit motion?" derivation
//   * gear             -> text
//
// No dynamic allocation; every string producer writes into a caller-supplied
// fixed buffer with a bounded snprintf.  The MOTION_INHIBIT_* / RELAY_SEQ_*
// bit values are the single definitions in esp32/include/can_ids.h.
// =============================================================================

#ifndef MOTION_INHIBIT_VIEW_H
#define MOTION_INHIBIT_VIEW_H

#include <cstdint>
#include <cstddef>
#include <cstdio>

#include "can_ids.h"

namespace motion_inhibit_view {

// -------------------------------------------------------------------------
// STALE timeout.
//
// 0x315 is transmitted at 10 Hz (100 ms nominal period, see can_ids.h
// DIAG_MOTION_INHIBIT).  A frame is considered STALE once THREE nominal
// periods have elapsed without a fresh timestamp, i.e. 300 ms.  This
// tolerates a single dropped frame plus scheduling jitter while still
// flagging a genuinely silent link within a third of a second.
// -------------------------------------------------------------------------
inline constexpr uint32_t FRAME_PERIOD_MS  = 100U;
inline constexpr uint32_t STALE_TIMEOUT_MS = 3U * FRAME_PERIOD_MS;  // 300 ms

// Maximum number of reason bits (16-bit mask).
inline constexpr uint8_t REASON_MAX = 16U;

// -------------------------------------------------------------------------
// Freshness of the last received 0x315 frame.
// -------------------------------------------------------------------------
enum class Freshness : uint8_t {
    NEVER_RECEIVED = 0,   // no frame has ever been decoded (valid flag false)
    VALID          = 1,   // last frame within the STALE timeout
    STALE          = 2    // a frame was received but is older than the timeout
};

// millis()-wrap-safe age of the last frame.  Fixed-width uint32_t matches the
// 32-bit ESP32 millis() domain; unsigned subtraction yields the correct
// elapsed time across a 32-bit millis() wrap.  (The caller truncates its
// `unsigned long` millis()/timestamp to uint32_t, which is exact on the
// 32-bit target and consistent on host.)
inline uint32_t ageMs(uint32_t nowMs, uint32_t stampMs) {
    return nowMs - stampMs;
}

// Classify freshness.  `valid` is MotionInhibitData::valid — false means no
// frame has ever been decoded, which must map to NEVER_RECEIVED regardless of
// the timestamp fields.
inline Freshness freshness(bool valid,
                           uint32_t nowMs,
                           uint32_t stampMs,
                           uint32_t timeoutMs = STALE_TIMEOUT_MS) {
    if (!valid) {
        return Freshness::NEVER_RECEIVED;
    }
    return (ageMs(nowMs, stampMs) <= timeoutMs) ? Freshness::VALID
                                                : Freshness::STALE;
}

inline const char* freshnessText(Freshness f) {
    switch (f) {
        case Freshness::VALID:          return "VALID";
        case Freshness::STALE:          return "STALE";
        case Freshness::NEVER_RECEIVED: return "NEVER RECEIVED";
    }
    return "NEVER RECEIVED";
}

// -------------------------------------------------------------------------
// Relay-sequence phase (0x315 byte7 bits2-3) — commanded/sequencer state only,
// NOT physical relay-contact feedback.
// -------------------------------------------------------------------------
inline const char* relayPhaseText(uint8_t phase) {
    switch (phase) {
        case can::MOTION_INHIBIT_RELAY_SEQ_IDLE:        return "IDLE";
        case can::MOTION_INHIBIT_RELAY_SEQ_IN_PROGRESS: return "IN PROGRESS";
        case can::MOTION_INHIBIT_RELAY_SEQ_COMPLETE:    return "COMPLETE";
        default:                                        return "UNKNOWN";
    }
}

// -------------------------------------------------------------------------
// System state (0x315 byte2, SystemState enum).  `permitsMotion` is derived on
// the HMI (Section 2 item 4): BOOT/STANDBY/SAFE/ERROR/unknown never permit
// traction, so the page can surface a "NO MOTION STATE" indication without any
// change to the 8-byte protocol.
// -------------------------------------------------------------------------
inline const char* systemStateText(uint8_t s) {
    switch (s) {
        case 0: return "BOOT";
        case 1: return "STANDBY";
        case 2: return "ACTIVE";
        case 3: return "DEGRADED";
        case 4: return "SAFE";
        case 5: return "ERROR";
        case 6: return "LIMP HOME";
        default: return "UNKNOWN";
    }
}

// True only for states in which the traction chain is allowed to move
// (ACTIVE, DEGRADED, LIMP_HOME).  Everything else — including unknown codes —
// is a no-motion state.
inline bool stateAllowsMotion(uint8_t s) {
    return (s == 2 /*ACTIVE*/) || (s == 3 /*DEGRADED*/) || (s == 6 /*LIMP_HOME*/);
}

inline const char* gearText(uint8_t g) {
    switch (g) {
        case 0: return "PARK";
        case 1: return "REVERSE";
        case 2: return "NEUTRAL";
        case 3: return "FORWARD";
        case 4: return "FORWARD D2";
        default: return "UNKNOWN";
    }
}

// -------------------------------------------------------------------------
// Reason bitmask -> ordered list of labels.
//
// The table is ordered from bit0 (0x0001) to bit15 (0x8000) so the produced
// list is deterministic.  Labels are kept short enough to fit the fixed
// telemetry column and phrased honestly:
//   * BATTERY LIMIT/CUT — the 8-byte frame cannot distinguish a power-limiting
//     warning from a hard cutoff, so the label does not over-claim "cutoff".
//   * SAFETY SCALE 0    — groups ABS/TCS/per-wheel/driver-enable scale collapse
//     (the protocol cannot separate them in 8 bytes).
// -------------------------------------------------------------------------
struct ReasonEntry {
    uint16_t    bit;
    const char* label;
};

inline const ReasonEntry* reasonTable(uint8_t& countOut) {
    static const ReasonEntry kTable[REASON_MAX] = {
        { can::MOTION_INHIBIT_STATE_SAFE,          "SAFE STATE"      },
        { can::MOTION_INHIBIT_STATE_ERROR,         "ERROR STATE"     },
        { can::MOTION_INHIBIT_POWER_NOT_READY,     "POWER NOT READY" },
        { can::MOTION_INHIBIT_GEAR_PARK,           "GEAR PARK"       },
        { can::MOTION_INHIBIT_GEAR_NEUTRAL,        "GEAR NEUTRAL"    },
        { can::MOTION_INHIBIT_NO_DEMAND,           "NO DEMAND"       },
        { can::MOTION_INHIBIT_DEMAND_ZEROED,       "DEMAND ZEROED"   },
        { can::MOTION_INHIBIT_OBSTACLE_BLOCK,      "OBSTACLE BLOCK"  },
        { can::MOTION_INHIBIT_PWM_ZERO,            "PWM ZERO"        },
        { can::MOTION_INHIBIT_TORQUE_LIMITED,      "TORQUE LIMITED"  },
        { can::MOTION_INHIBIT_STARTUP_INHIBIT,     "STARTUP INHIBIT" },
        { can::MOTION_INHIBIT_PEDAL_FAULT,         "PEDAL FAULT"     },
        { can::MOTION_INHIBIT_SAFETY_SCALE_ZERO,   "SAFETY SCALE 0"  },
        { can::MOTION_INHIBIT_BATTERY_CUTOFF,      "BATTERY LIMIT/CUT" },
        { can::MOTION_INHIBIT_THERMAL_OVERCURRENT, "THERMAL/OVERCURR"  },
        { can::MOTION_INHIBIT_SERVICE_DISABLED,    "SERVICE DISABLED"  },
    };
    countOut = REASON_MAX;
    return kTable;
}

// Fill `out[]` with pointers to the labels of every set bit in `mask`, in
// table order, up to `maxLabels`.  Returns the number of labels written.
inline uint8_t reasonLabels(uint16_t mask, const char* out[], uint8_t maxLabels) {
    uint8_t tableCount = 0;
    const ReasonEntry* table = reasonTable(tableCount);
    uint8_t n = 0;
    for (uint8_t i = 0; i < tableCount && n < maxLabels; ++i) {
        if ((mask & table[i].bit) != 0U) {
            out[n++] = table[i].label;
        }
    }
    return n;
}

// Render the active-reason list into a single bounded, NUL-terminated buffer,
// separated by ", ".  A 0x0000 mask yields "NONE".  Never writes past
// bufLen-1; truncation is safe (snprintf-bounded).  Returns the number of
// active reasons (independent of truncation).
inline uint8_t reasonListToBuffer(uint16_t mask, char* buf, size_t bufLen) {
    if (buf == nullptr || bufLen == 0U) {
        return 0U;
    }
    const char* labels[REASON_MAX];
    const uint8_t n = reasonLabels(mask, labels, REASON_MAX);
    if (n == 0U) {
        snprintf(buf, bufLen, "NONE");
        return 0U;
    }
    size_t pos = 0;
    buf[0] = '\0';
    for (uint8_t i = 0; i < n && pos < (bufLen - 1U); ++i) {
        const int w = snprintf(buf + pos, bufLen - pos, "%s%s",
                               (i == 0) ? "" : ", ", labels[i]);
        if (w <= 0) {
            break;
        }
        pos += (size_t)w;
        if (pos >= bufLen) {          // snprintf reported truncation
            pos = bufLen - 1U;
            break;
        }
    }
    return n;
}

} // namespace motion_inhibit_view

#endif // MOTION_INHIBIT_VIEW_H
