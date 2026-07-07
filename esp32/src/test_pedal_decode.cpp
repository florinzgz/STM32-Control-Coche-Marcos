// =============================================================================
// ESP32 HMI — host unit test for the extended 0x20B STATUS_PEDAL decoder.
//
// Self-contained replica of decodePedal() (esp32/src/can_rx.cpp) so the
// additive DLC1 -> DLC4 pedal telemetry (percent + fault flags + raw ADC) and
// legacy short-DLC handling can be validated on host g++ without Arduino/TWAI.
//
// Build & run (from repository root):
//   g++ -std=c++17 -DHOST_TEST esp32/src/test_pedal_decode.cpp \
//       -o /tmp/test_pedal_decode && /tmp/test_pedal_decode
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

// ---- Decoded store (mirror PedalData) ----
struct PedalData {
    uint8_t  percent      = 0;
    bool     plausible    = true;
    bool     contradictory = false;
    uint16_t rawAdc       = 0;
    bool     extended     = false;
    unsigned long timestampMs = 0;
};

// ---- Replica of decodePedal() ----
static bool decodePedal(const CanFrame& f, PedalData& out) {
    if (f.data_length_code < 1) return false;
    PedalData pd;
    pd.percent = (f.data[0] <= 100) ? f.data[0] : 100;
    if (f.data_length_code >= 4) {
        pd.plausible     = (f.data[1] & 0x01u) != 0;
        pd.contradictory = (f.data[1] & 0x02u) != 0;
        pd.rawAdc        = static_cast<uint16_t>(f.data[2] | (f.data[3] << 8));
        pd.extended      = true;
    }
    pd.timestampMs = 1234;
    out = pd;
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
    std::printf("=== test_pedal_decode ===\n");

    // 1. Legacy single-byte frame: percent only, plausible defaults true.
    {
        CanFrame f;
        f.identifier = 0x20B;
        f.data_length_code = 1;
        f.data[0] = 42;
        PedalData pd;
        const bool ok = decodePedal(f, pd);
        CHECK(ok, "DLC1: decodes");
        CHECK(pd.percent == 42, "DLC1: percent decoded");
        CHECK(pd.plausible && !pd.contradictory, "DLC1: fault defaults ok");
        CHECK(!pd.extended, "DLC1: not extended");
    }

    // 2. Percent clamps to 100.
    {
        CanFrame f;
        f.data_length_code = 1;
        f.data[0] = 200;
        PedalData pd;
        decodePedal(f, pd);
        CHECK(pd.percent == 100, "percent clamps to 100");
    }

    // 3. Extended DLC4 healthy: plausible=1, no contradiction, raw ADC LE.
    {
        CanFrame f;
        f.data_length_code = 4;
        f.data[0] = 55;
        f.data[1] = 0x01;          // plausible, not contradictory
        f.data[2] = 0x34;          // raw ADC low
        f.data[3] = 0x12;          // raw ADC high -> 0x1234
        PedalData pd;
        decodePedal(f, pd);
        CHECK(pd.percent == 55, "DLC4: percent decoded");
        CHECK(pd.plausible, "DLC4: plausible set");
        CHECK(!pd.contradictory, "DLC4: not contradictory");
        CHECK(pd.rawAdc == 0x1234, "DLC4: raw ADC LE decoded");
        CHECK(pd.extended, "DLC4: extended flag set");
    }

    // 4. Extended DLC4 fault: implausible + contradictory reported.
    {
        CanFrame f;
        f.data_length_code = 4;
        f.data[0] = 0;
        f.data[1] = 0x02;          // bit0=0 -> not plausible, bit1=1 -> contradictory
        f.data[2] = 0xFF;
        f.data[3] = 0x0F;          // 0x0FFF
        PedalData pd;
        decodePedal(f, pd);
        CHECK(!pd.plausible, "DLC4 fault: not plausible");
        CHECK(pd.contradictory, "DLC4 fault: contradictory");
        CHECK(pd.rawAdc == 0x0FFF, "DLC4 fault: raw ADC decoded");
    }

    // 5. Zero-length frame is rejected.
    {
        CanFrame f;
        f.data_length_code = 0;
        PedalData pd;
        const bool ok = decodePedal(f, pd);
        CHECK(!ok, "DLC0: rejected");
    }

    std::printf("=== %d run, %d failed ===\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
