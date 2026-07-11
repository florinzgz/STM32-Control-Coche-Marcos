// =============================================================================
// ESP32 HMI — host unit test for the 0x315 DIAG_MOTION_INHIBIT decoder.
//
// Self-contained replica of decodeMotionInhibit() (esp32/src/can_rx.cpp) so the
// MOTION_INHIBIT_REASON decode and short-DLC handling can be validated on host
// g++ without Arduino/TWAI.
//
// Build & run (from repository root):
//   g++ -std=c++17 -DHOST_TEST esp32/src/test_motion_inhibit_decode.cpp -o /tmp/t && /tmp/t
//
// NOTE: has its own main(); excluded from the firmware build via
// build_src_filter in esp32/platformio.ini.
// =============================================================================

#include <cstdint>
#include <cstdio>

// ---- Minimal CanFrame stub (mirrors the fields the decoder reads) ----
struct CanFrame {
    uint32_t identifier       = 0;
    uint8_t  data_length_code = 0;
    uint8_t  data[8]          = {0};
};

// ---- MOTION_INHIBIT_* bits (mirror can_ids.h) ----
namespace can {
constexpr uint16_t MOTION_INHIBIT_STATE_SAFE      = 0x0001;
constexpr uint16_t MOTION_INHIBIT_STATE_ERROR     = 0x0002;
constexpr uint16_t MOTION_INHIBIT_POWER_NOT_READY = 0x0004;
constexpr uint16_t MOTION_INHIBIT_GEAR_PARK       = 0x0008;
constexpr uint16_t MOTION_INHIBIT_GEAR_NEUTRAL    = 0x0010;
constexpr uint16_t MOTION_INHIBIT_NO_DEMAND       = 0x0020;
constexpr uint16_t MOTION_INHIBIT_DEMAND_ZEROED   = 0x0040;
constexpr uint16_t MOTION_INHIBIT_OBSTACLE_BLOCK  = 0x0080;
constexpr uint16_t MOTION_INHIBIT_PWM_ZERO        = 0x0100;
constexpr uint16_t MOTION_INHIBIT_TORQUE_LIMITED  = 0x0200;
constexpr uint16_t MOTION_INHIBIT_STARTUP_INHIBIT = 0x0400;
constexpr uint16_t MOTION_INHIBIT_PEDAL_FAULT     = 0x0800;
constexpr uint16_t MOTION_INHIBIT_SAFETY_SCALE_ZERO = 0x1000;
constexpr uint16_t MOTION_INHIBIT_BATTERY_CUTOFF  = 0x2000;
constexpr uint16_t MOTION_INHIBIT_THERMAL_OVERCURRENT = 0x4000;
constexpr uint16_t MOTION_INHIBIT_SERVICE_DISABLED = 0x8000;
constexpr uint8_t  MOTION_INHIBIT_RELAY_SEQ_IDLE        = 0;
constexpr uint8_t  MOTION_INHIBIT_RELAY_SEQ_IN_PROGRESS = 1;
constexpr uint8_t  MOTION_INHIBIT_RELAY_SEQ_COMPLETE    = 2;
}

// ---- Decoded store (mirror MotionInhibitData) ----
struct MotionInhibitData {
    uint16_t reason        = 0;
    uint8_t  systemState   = 0;
    uint8_t  gear          = 0;
    int8_t   demandPct     = 0;
    int8_t   effectivePct  = 0;
    uint8_t  finalPwmPct   = 0;
    bool     powerReady    = false;
    bool     obstacleFwdBlk = false;
    uint8_t  relaySeqPhase = 0;
    uint8_t  degradedLevel = 0;
    bool     valid         = false;
    unsigned long timestampMs = 0;
};

static uint16_t readU16LE(const uint8_t* p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

// ---- Replica of decodeMotionInhibit() ----
static bool decodeMotionInhibit(const CanFrame& f, MotionInhibitData& out) {
    if (f.data_length_code < 8) return false;
    MotionInhibitData mi;
    mi.reason         = readU16LE(&f.data[0]);
    mi.systemState    = f.data[2];
    mi.gear           = f.data[3];
    mi.demandPct      = (int8_t)f.data[4];
    mi.effectivePct   = (int8_t)f.data[5];
    mi.finalPwmPct    = f.data[6];
    mi.powerReady     = (f.data[7] & 0x01U) != 0;
    mi.obstacleFwdBlk = (f.data[7] & 0x02U) != 0;
    mi.relaySeqPhase  = (uint8_t)((f.data[7] >> 2) & 0x03U);
    mi.degradedLevel  = (uint8_t)((f.data[7] >> 4) & 0x0FU);
    mi.valid          = true;
    mi.timestampMs    = 1234;
    out = mi;
    return true;
}

// ---- Test framework ----
static int tests_run = 0, tests_failed = 0;
#define CHECK(cond, msg) do {                                     \
        tests_run++;                                              \
        if (!(cond)) { tests_failed++;                            \
            std::printf("  FAIL: %s\n", msg); }                   \
        else { std::printf("  ok:   %s\n", msg); }                \
    } while (0)

int main() {
    std::printf("=== test_motion_inhibit_decode ===\n");

    // 1. Full DLC-8 frame: DEGRADED, demand survived, PWM zero.
    {
        CanFrame f;
        f.identifier = 0x315;
        f.data_length_code = 8;
        uint16_t reason = can::MOTION_INHIBIT_PWM_ZERO | can::MOTION_INHIBIT_TORQUE_LIMITED;
        f.data[0] = (uint8_t)(reason & 0xFF);
        f.data[1] = (uint8_t)((reason >> 8) & 0xFF);
        f.data[2] = 4;                 // SYS_STATE_DEGRADED
        f.data[3] = 3;                 // GEAR_FORWARD (D1)
        f.data[4] = (uint8_t)(int8_t)40;   // operator demand 40 %
        f.data[5] = (uint8_t)(int8_t)16;   // effective 16 %
        f.data[6] = 0;                 // final PWM 0 %
        f.data[7] = 0x01 | (uint8_t)(1 << 4);  // power_ready, degraded level 1
        MotionInhibitData mi;
        const bool ok = decodeMotionInhibit(f, mi);
        CHECK(ok && mi.valid, "DLC8: decodes valid");
        CHECK(mi.reason == reason, "DLC8: reason bitfield decoded");
        CHECK(mi.reason & can::MOTION_INHIBIT_PWM_ZERO, "DLC8: PWM_ZERO bit set");
        CHECK(mi.reason & can::MOTION_INHIBIT_TORQUE_LIMITED, "DLC8: TORQUE_LIMITED bit set");
        CHECK(mi.systemState == 4, "DLC8: system state decoded");
        CHECK(mi.gear == 3, "DLC8: gear decoded");
        CHECK(mi.demandPct == 40, "DLC8: operator demand decoded");
        CHECK(mi.effectivePct == 16, "DLC8: effective demand decoded");
        CHECK(mi.finalPwmPct == 0, "DLC8: final PWM decoded");
        CHECK(mi.powerReady, "DLC8: power_ready flag decoded");
        CHECK(!mi.obstacleFwdBlk, "DLC8: obstacle flag clear");
        CHECK(mi.degradedLevel == 1, "DLC8: degraded level decoded");
    }

    // 2. Negative (signed) demand decodes correctly.
    {
        CanFrame f;
        f.data_length_code = 8;
        f.data[4] = (uint8_t)(int8_t)(-25);  // dynamic braking demand
        f.data[5] = (uint8_t)(int8_t)(-10);
        f.data[7] = 0x02;                     // obstacle_forward_blocked
        MotionInhibitData mi;
        const bool ok = decodeMotionInhibit(f, mi);
        CHECK(ok, "signed: decoded");
        CHECK(mi.demandPct == -25, "signed: negative demand");
        CHECK(mi.effectivePct == -10, "signed: negative effective");
        CHECK(mi.obstacleFwdBlk, "signed: obstacle flag decoded");
        CHECK(!mi.powerReady, "signed: power_ready clear");
    }

    // 3. Short DLC (<8) is dropped, store not updated.
    {
        CanFrame f;
        f.data_length_code = 6;
        MotionInhibitData mi;
        const bool ok = decodeMotionInhibit(f, mi);
        CHECK(!ok && !mi.valid, "shortDLC: not decoded");
    }

    // 4. Relay-sequence phase (byte7 bits2-3) decodes independently of the
    //    power_ready / obstacle / degraded fields sharing the same byte.
    {
        CanFrame f;
        f.data_length_code = 8;
        // in-progress (1) + power_ready + degraded level 2
        f.data[7] = 0x01 | (uint8_t)(can::MOTION_INHIBIT_RELAY_SEQ_IN_PROGRESS << 2)
                         | (uint8_t)(2 << 4);
        MotionInhibitData mi;
        CHECK(decodeMotionInhibit(f, mi), "relaySeq: decoded");
        CHECK(mi.relaySeqPhase == can::MOTION_INHIBIT_RELAY_SEQ_IN_PROGRESS,
              "relaySeq: in-progress decoded");
        CHECK(mi.powerReady, "relaySeq: power_ready still decoded");
        CHECK(mi.degradedLevel == 2, "relaySeq: degraded level still decoded");

        f.data[7] = (uint8_t)(can::MOTION_INHIBIT_RELAY_SEQ_COMPLETE << 2);
        CHECK(decodeMotionInhibit(f, mi), "relaySeq: complete decoded");
        CHECK(mi.relaySeqPhase == can::MOTION_INHIBIT_RELAY_SEQ_COMPLETE,
              "relaySeq: complete value");

        f.data[7] = 0x00;  // idle
        CHECK(decodeMotionInhibit(f, mi), "relaySeq: idle decoded");
        CHECK(mi.relaySeqPhase == can::MOTION_INHIBIT_RELAY_SEQ_IDLE,
              "relaySeq: idle value");
    }

    // 5. Extended reason bits (Item 5) round-trip through bytes 0-1.
    {
        CanFrame f;
        f.data_length_code = 8;
        uint16_t reason = can::MOTION_INHIBIT_STARTUP_INHIBIT
                        | can::MOTION_INHIBIT_BATTERY_CUTOFF
                        | can::MOTION_INHIBIT_SERVICE_DISABLED;
        f.data[0] = (uint8_t)(reason & 0xFF);
        f.data[1] = (uint8_t)((reason >> 8) & 0xFF);
        f.data[7] = 0x00;
        MotionInhibitData mi;
        CHECK(decodeMotionInhibit(f, mi), "extReason: decoded");
        CHECK(mi.reason & can::MOTION_INHIBIT_STARTUP_INHIBIT, "extReason: startup bit");
        CHECK(mi.reason & can::MOTION_INHIBIT_BATTERY_CUTOFF,  "extReason: battery bit");
        CHECK(mi.reason & can::MOTION_INHIBIT_SERVICE_DISABLED,"extReason: service bit");
    }

    std::printf("=== %d run, %d failed ===\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
