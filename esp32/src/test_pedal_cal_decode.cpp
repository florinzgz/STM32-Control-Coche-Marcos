// =============================================================================
// ESP32 HMI — host unit test for the extended 0x308 DIAG_PEDAL_CAL decoder.
//
// Build & run (from repository root):
//   g++ -std=c++17 -DHOST_TEST esp32/src/test_pedal_cal_decode.cpp \
//       -o /tmp/test_pedal_cal_decode && /tmp/test_pedal_cal_decode
// =============================================================================

#include <cstdint>
#include <cstdio>

struct CanFrame {
    uint32_t identifier       = 0;
    uint8_t  data_length_code = 0;
    uint8_t  data[8]          = {0};
};

struct PedalCalData {
    uint8_t  flags        = 0;
    uint16_t rawAdc       = 0;
    uint16_t rawAdc2      = 0;
    uint16_t diffRaw      = 0;
    uint16_t rejectReason = 0;
    uint16_t storedMin    = 0;
    uint16_t storedMax    = 0;
    uint16_t pendingMin   = 0;
    uint16_t pendingMax   = 0;
    uint8_t  pedalPercent = 0;
};

static inline uint16_t readU16LE(const uint8_t* buf) {
    return static_cast<uint16_t>(buf[0]) |
           (static_cast<uint16_t>(buf[1]) << 8);
}

static bool decodePedalCal(const CanFrame& f, PedalCalData& out) {
    if (f.data_length_code < 8) return false;
    PedalCalData pc = out;
    pc.flags = f.data[0];
    if (pc.flags & 0x80U) {
        pc.rawAdc2      = readU16LE(&f.data[1]);
        pc.diffRaw      = readU16LE(&f.data[3]);
        pc.rejectReason = readU16LE(&f.data[5]);
    } else {
        pc.rawAdc = readU16LE(&f.data[1]);
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
    out = pc;
    return true;
}

static int tests_run = 0, tests_failed = 0;
#define CHECK(cond, msg) do {                                     \
        tests_run++;                                              \
        if (!(cond)) { tests_failed++;                            \
            std::printf("  FAIL: %s\n", msg); }                   \
        else { std::printf("  ok:   %s\n", msg); }                \
    } while (0)

int main() {
    std::printf("=== test_pedal_cal_decode ===\n");

    PedalCalData pd;

    {
        CanFrame f{};
        f.data_length_code = 8;
        f.data[0] = 0x00;
        f.data[1] = 0x34; f.data[2] = 0x12;
        f.data[3] = 0x00; f.data[4] = 0x00;
        f.data[5] = 0xA0; f.data[6] = 0x0F;
        f.data[7] = 2;
        CHECK(decodePedalCal(f, pd), "pending frame decodes");
        CHECK(pd.rawAdc == 0x1234, "pending frame rawAdc");
        CHECK(pd.pendingMin == 0, "pending frame min accepts 0");
        CHECK(pd.pendingMax == 4000, "pending frame max decoded");
        CHECK(pd.pedalPercent == 2, "pending frame pedal percent");
    }

    {
        CanFrame f{};
        f.data_length_code = 8;
        f.data[0] = 0x40;
        f.data[1] = 0x78; f.data[2] = 0x56;
        f.data[3] = 50;   f.data[4] = 0x00;
        f.data[5] = 0xA0; f.data[6] = 0x0F;
        f.data[7] = 0;
        CHECK(decodePedalCal(f, pd), "stored frame decodes");
        CHECK(pd.storedMin == 50, "stored frame min decoded");
        CHECK(pd.storedMax == 4000, "stored frame max decoded");
        CHECK(pd.pendingMin == 0, "stored frame preserves pending min");
    }

    {
        CanFrame f{};
        f.data_length_code = 8;
        f.data[0] = 0x80;
        f.data[1] = 0x30; f.data[2] = 0x12;
        f.data[3] = 0x04; f.data[4] = 0x00;
        f.data[5] = 0xA4; f.data[6] = 0x00;
        f.data[7] = 1;
        CHECK(decodePedalCal(f, pd), "diag frame decodes");
        CHECK(pd.rawAdc2 == 0x1230, "diag frame rawAdc2");
        CHECK(pd.diffRaw == 4, "diag frame diffRaw");
        CHECK(pd.rejectReason == 0x00A4, "diag frame rejectReason");
        CHECK(pd.rawAdc == 0x5678, "diag frame preserves last rawAdc");
    }

    {
        CanFrame f{};
        f.data_length_code = 0;
        CHECK(!decodePedalCal(f, pd), "short frame rejected");
    }

    std::printf("=== %d run, %d failed ===\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
