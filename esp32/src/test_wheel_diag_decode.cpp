// =============================================================================
// ESP32 HMI — host unit test for the 0x313 DIAG_WHEEL_SENSOR decoder.
//
// Self-contained replica of decodeWheelSensorDiag() (esp32/src/can_rx.cpp) and
// the drawWheelDiagBlock() reason-label mapping, so the wheel-diagnostic decode
// and short-DLC handling can be validated on host g++ without Arduino/TWAI.
//
// Build & run (from repository root):
//   g++ -std=c++17 -DHOST_TEST esp32/src/test_wheel_diag_decode.cpp \
//       -o /tmp/test_wheel_diag_decode && /tmp/test_wheel_diag_decode
//
// NOTE: has its own main(); excluded from the firmware build via
// build_src_filter in esp32/platformio.ini.
// =============================================================================

#include <cstdint>
#include <cstdio>
#include <cstring>

// ---- Minimal CanFrame stub (mirrors the fields the decoder reads) ----
struct CanFrame {
    uint32_t identifier       = 0;
    uint8_t  data_length_code = 0;
    uint8_t  data[8]          = {0};
};

// ---- Reason codes / flag bits (mirror can_ids.h) ----
namespace can {
constexpr uint8_t WHEEL_DIAG_REASON_OK              = 0;
constexpr uint8_t WHEEL_DIAG_REASON_NO_PULSE        = 1;
constexpr uint8_t WHEEL_DIAG_REASON_STUCK_HIGH      = 2;
constexpr uint8_t WHEEL_DIAG_REASON_STUCK_LOW       = 3;
constexpr uint8_t WHEEL_DIAG_REASON_MISMATCH        = 4;
constexpr uint8_t WHEEL_DIAG_REASON_IMPOSSIBLE_RATE = 5;
constexpr uint8_t WHEEL_DIAG_REASON_MANUAL_MOVEMENT = 6;
constexpr uint8_t WHEEL_DIAG_REASON_DISABLED_STATE  = 7;
constexpr uint8_t WHEEL_DIAG_REASON_UNKNOWN         = 8;
constexpr uint8_t WHEEL_DIAG_FLAG_POWERTRAIN = (1U << 0);
constexpr uint8_t WHEEL_DIAG_FLAG_MANUAL     = (1U << 1);
constexpr uint8_t WHEEL_DIAG_FLAG_DEBOUNCING = (1U << 2);
constexpr uint8_t WHEEL_DIAG_FLAG_LATCHED    = (1U << 3);
}

// ---- Decoded store (mirror WheelSensorDiagData) ----
struct WheelSensorDiagData {
    uint8_t reason[5] = {0,0,0,0,0};
    uint8_t gpioMask  = 0;
    uint8_t faultMask = 0;
    uint8_t flags     = 0;
    bool    valid     = false;
    unsigned long timestampMs = 0;
};

static uint32_t s_rx0x313Count    = 0;
static uint32_t s_dropped0x313Dlc = 0;
static uint8_t  s_last0x313Dlc    = 0;

// ---- Replica of decodeWheelSensorDiag() ----
static bool decodeWheelSensorDiag(const CanFrame& f, WheelSensorDiagData& out) {
    ++s_rx0x313Count;
    s_last0x313Dlc = f.data_length_code;
    if (f.data_length_code < 7) { ++s_dropped0x313Dlc; return false; }

    WheelSensorDiagData wd;
    for (uint8_t i = 0; i < 5; ++i) wd.reason[i] = f.data[i];
    wd.gpioMask  = f.data[5];
    wd.faultMask = f.data[6];
    if (f.data_length_code >= 8) wd.flags = f.data[7];
    wd.valid = true;
    wd.timestampMs = 1234;
    out = wd;
    return true;
}

// ---- Replica of drawWheelDiagBlock() label lookup (wheelDiagReasonEs) ----
static const char* reasonLabel(uint8_t r) {
    switch (r) {
        case can::WHEEL_DIAG_REASON_OK:              return "OK";
        case can::WHEEL_DIAG_REASON_NO_PULSE:        return "SIN PULSO";
        case can::WHEEL_DIAG_REASON_STUCK_HIGH:      return "PEG.ALTO";
        case can::WHEEL_DIAG_REASON_STUCK_LOW:       return "PEG.BAJO";
        case can::WHEEL_DIAG_REASON_MISMATCH:        return "DISCREPA";
        case can::WHEEL_DIAG_REASON_IMPOSSIBLE_RATE: return "IMPOSIBLE";
        case can::WHEEL_DIAG_REASON_MANUAL_MOVEMENT: return "MANUAL";
        case can::WHEEL_DIAG_REASON_DISABLED_STATE:  return "DESHAB.";
        default:                                     return "?";
    }
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
    std::printf("=== test_wheel_diag_decode ===\n");

    // 1. Full DLC-8 frame decodes all fields.
    {
        CanFrame f;
        f.identifier = 0x313;
        f.data_length_code = 8;
        f.data[0] = can::WHEEL_DIAG_REASON_OK;
        f.data[1] = can::WHEEL_DIAG_REASON_MANUAL_MOVEMENT;
        f.data[2] = can::WHEEL_DIAG_REASON_MISMATCH;
        f.data[3] = can::WHEEL_DIAG_REASON_STUCK_LOW;
        f.data[4] = can::WHEEL_DIAG_REASON_OK;   // steer reserved
        f.data[5] = 0x05;                        // FL,RL gpio high
        f.data[6] = 0x08;                        // RR fault active
        f.data[7] = can::WHEEL_DIAG_FLAG_MANUAL | can::WHEEL_DIAG_FLAG_DEBOUNCING
                  | (uint8_t)(0x03 << 4);        // seq=3
        WheelSensorDiagData wd;
        const bool ok = decodeWheelSensorDiag(f, wd);
        CHECK(ok && wd.valid, "DLC8: decodes valid");
        CHECK(wd.reason[0]==0 && wd.reason[1]==6 && wd.reason[2]==4 && wd.reason[3]==3,
              "DLC8: FL/FR/RL/RR reasons decoded");
        CHECK(wd.gpioMask==0x05, "DLC8: gpio mask decoded");
        CHECK(wd.faultMask==0x08, "DLC8: fault mask decoded (RR)");
        CHECK((wd.flags & can::WHEEL_DIAG_FLAG_MANUAL)!=0, "DLC8: manual flag decoded");
        CHECK(((wd.flags>>4)&0x0F)==3, "DLC8: sequence decoded");
    }

    // 2. DLC-7 frame decodes reasons + masks, flags default 0.
    {
        CanFrame f;
        f.data_length_code = 7;
        f.data[0] = can::WHEEL_DIAG_REASON_NO_PULSE;
        f.data[5] = 0x00;
        f.data[6] = 0x01;
        WheelSensorDiagData wd;
        const bool ok = decodeWheelSensorDiag(f, wd);
        CHECK(ok && wd.valid, "DLC7: decodes valid");
        CHECK(wd.reason[0]==1, "DLC7: FL reason NO_PULSE");
        CHECK(wd.faultMask==0x01, "DLC7: fault mask decoded");
        CHECK(wd.flags==0, "DLC7: flags default 0 (byte7 absent)");
    }

    // 3. Short DLC (<7) is dropped and counted, store not updated.
    {
        const uint32_t before = s_dropped0x313Dlc;
        CanFrame f;
        f.data_length_code = 4;
        WheelSensorDiagData wd;
        const bool ok = decodeWheelSensorDiag(f, wd);
        CHECK(!ok && !wd.valid, "shortDLC: not decoded");
        CHECK(s_dropped0x313Dlc == before + 1, "shortDLC: drop counter incremented");
        CHECK(s_last0x313Dlc == 4, "shortDLC: last DLC recorded");
    }

    // 4. Reason labels map correctly to readable Spanish, out-of-range -> "?".
    {
        CHECK(std::strcmp(reasonLabel(0), "OK")==0, "label OK");
        CHECK(std::strcmp(reasonLabel(1), "SIN PULSO")==0, "label NO_PULSE=SIN PULSO");
        CHECK(std::strcmp(reasonLabel(6), "MANUAL")==0, "label MANUAL_MOVEMENT=MANUAL");
        CHECK(std::strcmp(reasonLabel(4), "DISCREPA")==0, "label MISMATCH=DISCREPA");
        CHECK(std::strcmp(reasonLabel(7), "DESHAB.")==0, "label DISABLED_STATE=DESHAB.");
        CHECK(std::strcmp(reasonLabel(8), "?")==0, "label UNKNOWN=?");
        CHECK(std::strcmp(reasonLabel(200), "?")==0, "label out-of-range clamps to ?");
    }

    std::printf("=== %d run, %d failed ===\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
