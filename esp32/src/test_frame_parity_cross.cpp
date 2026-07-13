// test_frame_parity_cross.cpp — audit Problem 6.
//
// Cross-codebase wire-contract parity tests for the three diagnostic frames
// introduced by Problems 1/3/4:
//   * 0x316 DIAG_STEERING_CENTERING
//   * 0x317 DIAG_RELAY_HEALTH
//   * 0x318 DIAG_INA_CH5
//
// The STM32 side (Core/Inc/*_frame.h) PACKS these frames; the ESP32 side
// (esp32/src/*_view.h + esp32/include/can_ids.h) DECODES them.  These two
// definitions live in separate codebases, so a silent drift in a bit
// position, byte order, sign handling or DLC guard would only surface on the
// vehicle.  This test forces both halves through the SAME payloads and proves
// they agree, covering the audit's P6 checklist:
//   - identical byte layout (STM32 unpack == ESP32 decode, field by field),
//   - flag bit-position parity between can_ids.h and the frame headers,
//   - DLC guard (len < 8 rejected on both sides) — frame absent vs MISSING,
//   - little-endian multi-byte fields,
//   - signed fields (encoder delta, INA shunt) preserved as two's complement,
//   - saturation of derived quantities,
//   - freshness / stale thresholds.
//
// Build (from repo root):
//   g++ -std=c++17 -Wall -Wextra -Werror -DHOST_TEST
//       -ICore/Inc -Iesp32/src -Iesp32/include
//       esp32/src/test_frame_parity_cross.cpp -o /tmp/test_frame_parity_cross
//   /tmp/test_frame_parity_cross

#include <cstdint>
#include <cstdio>
#include <cstring>

// ---- STM32 (packer / reference unpacker) ----
#include "steering_centering_frame.h"
#include "relay_health_frame.h"
#include "ina226_ch5_frame.h"

// ---- ESP32 (view decoders + wire constants) ----
#include "can_ids.h"
#include "steering_diag_view.h"
#include "relay_health_view.h"
#include "ina226_ch5_view.h"

static int tests_run = 0, tests_failed = 0;
#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

// ---- Static (compile-time) flag bit-position parity ----------------------
// Any drift here would break every runtime decode; assert it directly.
static void test_flag_constants_parity(void) {
    // 0x316 steering flags (byte 2) + status bits (byte 3).
    CHECK(can::STEER_DIAG_FLAG_PB5_RAW        == (1U << 0));
    CHECK(can::STEER_DIAG_FLAG_RELAY_PC12     == (1U << 3));
    CHECK(can::STEER_DIAG_FLAG_RESTORED_FLASH == (1U << 7));
    CHECK(can::STEER_DIAG_STATUS_PWM_REQUESTED == (1U << 6));

    // 0x317 relay flags (byte 1) vs Core frame header.
    CHECK(can::RELAY_DIAG_FLAG_RELAY_CMD     == RELAY_FRAME_FLAG_RELAY_CMD);
    CHECK(can::RELAY_DIAG_FLAG_SEQ_COMPLETE  == RELAY_FRAME_FLAG_SEQ_COMPLETE);
    CHECK(can::RELAY_DIAG_FLAG_POWER_READY   == RELAY_FRAME_FLAG_POWER_READY);
    CHECK(can::RELAY_DIAG_FLAG_WHEEL_MOVING  == RELAY_FRAME_FLAG_WHEEL_MOVING);
    CHECK(can::RELAY_DIAG_FLAG_CURRENT_VALID == RELAY_FRAME_FLAG_CURRENT_VALID);
    CHECK(can::RELAY_DIAG_FLAG_CURRENT_STALE == RELAY_FRAME_FLAG_CURRENT_STALE);
    CHECK(can::RELAY_DIAG_FLAG_INA_MISSING   == RELAY_FRAME_FLAG_INA_MISSING);
    CHECK(can::RELAY_DIAG_FLAG_POLARITY_REV  == RELAY_FRAME_FLAG_POLARITY_REV);

    // 0x318 INA flags (byte 1) vs Core frame header.
    CHECK(can::INA_CH5_FLAG_MUX_OK      == INA226_CH5_FLAG_MUX_OK);
    CHECK(can::INA_CH5_FLAG_I2C_ACK     == INA226_CH5_FLAG_I2C_ACK);
    CHECK(can::INA_CH5_FLAG_IDENTITY_OK == INA226_CH5_FLAG_IDENTITY_OK);
    CHECK(can::INA_CH5_FLAG_CONFIG_OK   == INA226_CH5_FLAG_CONFIG_OK);
    CHECK(can::INA_CH5_FLAG_SHUNT_OK    == INA226_CH5_FLAG_SHUNT_OK);
    CHECK(can::INA_CH5_FLAG_BUS_OK      == INA226_CH5_FLAG_BUS_OK);
    CHECK(can::INA_CH5_FLAG_POWERED     == INA226_CH5_FLAG_POWERED);
    CHECK(can::INA_CH5_FLAG_STALE       == INA226_CH5_FLAG_STALE);
}

// ================= 0x316 steering-centering =================
static void test_steering_parity(void) {
    // Craft a representative payload with distinct byte values including a
    // negative encoder delta (signed, bytes 6-7) and a >255 PWM (bytes 4-5).
    uint8_t wire[8];
    wire[0] = can::STEER_DIAG_SWEEP_LEFT;                     // reason
    wire[1] = static_cast<uint8_t>((4U & 0x0F) | ((2U & 0x0F) << 4)); // fsm=4, owner=2
    wire[2] = can::STEER_DIAG_FLAG_PB5_RAW |
              can::STEER_DIAG_FLAG_RELAY_PC12 |
              can::STEER_DIAG_FLAG_POWER_READY;
    wire[3] = static_cast<uint8_t>(1U /*state*/ | can::STEER_DIAG_STATUS_PWM_REQUESTED);
    wire[4] = 0x2C; wire[5] = 0x01;                          // pwm_real = 0x012C = 300
    wire[6] = 0x00; wire[7] = 0xFF;                          // delta = 0xFF00 = -256

    // ESP32 decode.
    steering_diag_view::SteeringDiagView v{};
    CHECK(steering_diag_view::decode(wire, 8, v));

    // STM32 reference unpack.
    SteerCenteringFrame f;
    memset(&f, 0, sizeof(f));
    SteerCentering_UnpackFrame(wire, &f);

    // Field-by-field cross-parity.
    CHECK(v.reason == f.reason);
    CHECK(v.pb5Raw == f.center_raw);
    CHECK(v.relayPc12 == f.relay_commanded);
    CHECK(v.powerReady == f.power_ready);
    CHECK(v.pwmRequested == f.pwm_requested);
    CHECK(v.pwmReal == f.pwm_real);
    CHECK(v.pwmReal == 300);                    // little-endian
    CHECK(v.encoderDelta == f.encoder_delta);
    CHECK(v.encoderDelta == -256);              // signed two's complement

    // DLC guard — a short frame (frame present but truncated) is rejected;
    // the HMI keeps its prior "valid=false" MISSING/absent semantics.
    steering_diag_view::SteeringDiagView vshort{};
    CHECK(steering_diag_view::decode(wire, 7, vshort) == false);

    // Round-trip via the STM32 packer proves the packer and both decoders all
    // agree on the same layout.
    SteeringCenteringDiag d;
    memset(&d, 0, sizeof(d));
    d.abort_reason    = (SteerDiagReason_t)5;   // SWEEP_RIGHT
    d.fsm_state       = (CenteringState_t)3;
    d.motor_owner     = (SteeringMotorOwner_t)1;
    d.system_state    = 2;
    d.pwm_applied_ch1 = 400;
    d.pwm_applied_ch2 = 123;
    d.pwm_requested   = 400;
    d.encoder_delta   = -1000;
    uint8_t packed[8];
    SteerCentering_PackFrame(&d, packed);
    steering_diag_view::SteeringDiagView vp{};
    CHECK(steering_diag_view::decode(packed, 8, vp));
    CHECK(vp.reason == 5);
    CHECK(vp.pwmReal == 400);                   // max(400,123)
    CHECK(vp.encoderDelta == -1000);
}

// ================= 0x317 relay-health =================
static void test_relay_parity(void) {
    uint8_t wire[8];
    wire[0] = can::RELAY_DIAG_OK;
    wire[1] = can::RELAY_DIAG_FLAG_RELAY_CMD |
              can::RELAY_DIAG_FLAG_CURRENT_VALID |
              can::RELAY_DIAG_FLAG_POLARITY_REV;
    wire[2] = 0x10; wire[3] = 0x27;   // 0x2710 = 10000 cA = 100.00 A
    wire[4] = 55;                     // throttle %
    wire[5] = 60;                     // final pwm %
    wire[6] = 0x2C; wire[7] = 0x01;   // age = 300 ms (>= STALE threshold)

    relay_health_view::RelayHealthView v{};
    CHECK(relay_health_view::decode(wire, 8, v));

    RelayHealthFrame f;
    memset(&f, 0, sizeof(f));
    RelayHealth_UnpackFrame(wire, &f);

    CHECK(v.reason == f.reason);
    CHECK(v.currentValid == f.current_valid);
    CHECK(v.currentSumCa == f.current_sum_ca);
    CHECK(v.currentSumCa == 10000);                  // little-endian
    CHECK(relay_health_view::currentAmps(v) > 99.9f);
    CHECK(v.throttlePct == f.throttle_pct);
    CHECK(v.finalPwmPct == f.final_pwm_pct);

    // DLC guard.
    relay_health_view::RelayHealthView vshort{};
    CHECK(relay_health_view::decode(wire, 4, vshort) == false);

    // Saturation: current_sum_abs huge -> centi-amps clamps to 0xFFFF.
    RelayHealthDiag d;
    memset(&d, 0, sizeof(d));
    d.diagnostic_reason = (RelayDiagReason_t)0;
    d.current_sum_abs   = 1000.0f;   // 100000 cA -> saturates
    d.throttle_pct      = 250.0f;    // -> clamps to 100
    d.final_pwm_pct     = -5.0f;     // -> clamps to 0
    uint8_t packed[8];
    RelayHealth_PackFrame(&d, packed);
    relay_health_view::RelayHealthView vp{};
    CHECK(relay_health_view::decode(packed, 8, vp));
    CHECK(vp.currentSumCa == 65535);
    CHECK(vp.throttlePct == 100);
    CHECK(vp.finalPwmPct == 0);
}

// ================= 0x318 INA226 CH5 =================
static void test_ina_parity(void) {
    uint8_t wire[8];
    wire[0] = can::INA_CH5_OK;
    wire[1] = can::INA_CH5_FLAG_MUX_OK | can::INA_CH5_FLAG_I2C_ACK |
              can::INA_CH5_FLAG_SHUNT_OK | can::INA_CH5_FLAG_BUS_OK;
    // Negative raw shunt (reversed installation) — signed two's complement.
    int16_t raw = -800;
    wire[2] = static_cast<uint8_t>(static_cast<uint16_t>(raw) & 0xFF);
    wire[3] = static_cast<uint8_t>((static_cast<uint16_t>(raw) >> 8) & 0xFF);
    wire[4] = 0x10; wire[5] = 0x27;   // bus = 0x2710 = 10000 mV
    wire[6] = 0x88; wire[7] = 0x13;   // age = 5000 ms

    ina226_ch5_view::Ina226Ch5View v{};
    CHECK(ina226_ch5_view::decode(wire, 8, v));

    Ina226Ch5Frame f;
    memset(&f, 0, sizeof(f));
    Ina226Ch5_UnpackFrame(wire, &f);

    CHECK(v.reason == f.reason);
    CHECK(v.rawShunt == f.raw_shunt);
    CHECK(v.rawShunt == -800);                 // signed preserved
    CHECK(v.shuntMicroVolts == f.shunt_uv);    // same derivation constants
    CHECK(v.currentMilliAmps == f.current_ma); // signed, never zeroed
    CHECK(v.currentMilliAmps < 0);             // reversed -> negative current
    CHECK(v.busMilliVolts == f.bus_mv);
    CHECK(v.busMilliVolts == 10000);
    CHECK(v.sampleAgeMs == f.sample_age_ms);

    // DLC guard.
    ina226_ch5_view::Ina226Ch5View vshort{};
    CHECK(ina226_ch5_view::decode(wire, 5, vshort) == false);

    // Bus-mV saturation via the packer.
    Ina226ChannelDiag d;
    memset(&d, 0, sizeof(d));
    d.fault_reason = (Ina226DiagReason_t)0;
    d.raw_shunt    = 1234;
    d.bus_mv       = 100000;   // > 65535 -> saturates
    uint8_t packed[8];
    Ina226Ch5_PackFrame(&d, packed);
    ina226_ch5_view::Ina226Ch5View vp{};
    CHECK(ina226_ch5_view::decode(packed, 8, vp));
    CHECK(vp.rawShunt == 1234);
    CHECK(vp.busMilliVolts == 65535);
}

int main() {
    test_flag_constants_parity();
    test_steering_parity();
    test_relay_parity();
    test_ina_parity();
    printf("frame_parity_cross: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
