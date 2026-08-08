// =============================================================================
// Host unit test — 0x316 DIAG_STEERING_CENTERING presentation helpers.
//
// Exercises esp32/src/steering_diag_view.h in isolation (no Arduino / TFT):
//   * decode() of a real 8-byte payload into SteeringDiagView
//   * reason -> MOTIVO / ACCIÓN text mapping
//   * FSM / owner / system-state text
//   * freshness classification (VALID / STALE / NEVER RECEIVED)
//
// Build (from repository root):
//   g++ -std=c++17 -Wall -Wextra -Iesp32/src -Iesp32/include
//       esp32/src/test_steering_diag_view.cpp -o /tmp/test_steering_diag_view
//   /tmp/test_steering_diag_view
// =============================================================================

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "steering_diag_view.h"

using namespace steering_diag_view;

static int tests_run = 0;
static int tests_failed = 0;

#define CHECK(expr) do {                                          \
    tests_run++;                                                  \
    if (!(expr)) {                                                \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);    \
        tests_failed++;                                           \
    }                                                             \
} while (0)

#define CHECK_STR(a, b) do {                                      \
    tests_run++;                                                  \
    if (strcmp((a), (b)) != 0) {                                  \
        printf("FAIL %s:%d  \"%s\" != \"%s\"\n",                  \
               __FILE__, __LINE__, (a), (b));                     \
        tests_failed++;                                           \
    }                                                             \
} while (0)

// Build the 8-byte 0x316 payload for the "PWM requested but encoder frozen"
// scenario — the key "DIRECCIÓN NO SE MUEVE" signature.
static void test_decode_no_movement() {
    uint8_t data[8] = {0};
    data[0] = can::STEER_DIAG_NO_ENCODER_MOVEMENT;
    data[1] = static_cast<uint8_t>(1 /*SWEEP_LEFT*/ | (0 /*CENTERING*/ << 4));
    data[2] = static_cast<uint8_t>(can::STEER_DIAG_FLAG_RELAY_PC12 |
                                   can::STEER_DIAG_FLAG_POWER_READY |
                                   can::STEER_DIAG_FLAG_EN_PC4);
    data[3] = static_cast<uint8_t>(1 /*STANDBY*/ |
                                   can::STEER_DIAG_STATUS_PWM_REQUESTED |
                                   can::STEER_DIAG_STATUS_CURRENT_GUARD_ARMED);
    data[4] = 425 & 0xFF;         // PWM real low
    data[5] = (425 >> 8) & 0xFF;  // PWM real high
    data[6] = 0;                  // encoder delta = 0 (frozen)
    data[7] = 0;

    SteeringDiagView v;
    CHECK(decode(data, 8, v));
    CHECK(v.reason == can::STEER_DIAG_NO_ENCODER_MOVEMENT);
    CHECK(v.fsmState == 1);
    CHECK(v.motorOwner == 0);
    CHECK(v.systemState == 1);
    CHECK(v.relayPc12);
    CHECK(v.powerReady);
    CHECK(v.enPc4);
    CHECK(v.pwmRequested);
    CHECK(v.currentGuardArmed);
    CHECK(v.pwmReal == 425);
    CHECK(v.encoderDelta == 0);
    CHECK(isStuck(v.reason));
    CHECK_STR(reasonText(v.reason), "SIN MOV. ENCODER");
    CHECK_STR(actionText(v.reason), "REVISAR BTS7960/EN/12V");
    CHECK_STR(fsmText(v.fsmState), "SWEEP LEFT");
    CHECK_STR(ownerText(v.motorOwner), "CENTERING");
    CHECK_STR(systemStateText(v.systemState), "STANDBY");
}

// Negative encoder delta (sweeping right) must decode as signed.
static void test_decode_negative_delta() {
    uint8_t data[8] = {0};
    data[0] = can::STEER_DIAG_SWEEP_RIGHT;
    data[1] = 2 /*SWEEP_RIGHT*/;
    int16_t delta = -1500;
    uint16_t u = static_cast<uint16_t>(delta);
    data[6] = static_cast<uint8_t>(u & 0xFF);
    data[7] = static_cast<uint8_t>((u >> 8) & 0xFF);

    SteeringDiagView v;
    CHECK(decode(data, 8, v));
    CHECK(v.encoderDelta == -1500);
    CHECK(!isStuck(v.reason));
    CHECK_STR(fsmText(v.fsmState), "SWEEP RIGHT");
}

// Relay-not-ready clears PC12 and power flags.
static void test_relay_not_ready() {
    uint8_t data[8] = {0};
    data[0] = can::STEER_DIAG_RELAY_NOT_READY;
    data[3] = 1;  // STANDBY, no flags set
    SteeringDiagView v;
    CHECK(decode(data, 8, v));
    CHECK(!v.relayPc12);
    CHECK(!v.powerReady);
    CHECK(!v.currentGuardArmed);
    CHECK(isStuck(v.reason));
    CHECK_STR(reasonText(v.reason), "RELE PC12 OFF");
    CHECK_STR(actionText(v.reason), "REVISAR RELE PC12");
}

// Restored-from-flash: DONE, EPS owner, restored flag set, not stuck.
static void test_restored_flash() {
    uint8_t data[8] = {0};
    data[0] = can::STEER_DIAG_RESTORED_FROM_FLASH;
    data[1] = static_cast<uint8_t>(3 /*DONE*/ | (1 /*EPS*/ << 4));
    data[2] = can::STEER_DIAG_FLAG_RESTORED_FLASH;
    SteeringDiagView v;
    CHECK(decode(data, 8, v));
    CHECK(v.restoredFromFlash);
    CHECK(!v.pwmRequested);
    CHECK(!isStuck(v.reason));
    CHECK_STR(fsmText(v.fsmState), "DONE");
    CHECK_STR(ownerText(v.motorOwner), "EPS");
    CHECK_STR(actionText(v.reason), "NINGUNA");
}

// Short frames are rejected and leave the struct zeroed.
static void test_short_frame() {
    uint8_t data[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    SteeringDiagView v;
    CHECK(!decode(data, 4, v));
    CHECK(v.reason == 0);
    CHECK(!decode(nullptr, 8, v));
}

// Freshness classification.
static void test_freshness() {
    CHECK(freshness(false, 5000, 0) == Freshness::NEVER_RECEIVED);
    CHECK(freshness(true, 1000, 500) == Freshness::VALID);
    CHECK(freshness(true, 5000, 500) == Freshness::STALE);
    // millis() wrap: now just after wrap, stamp just before.
    CHECK(freshness(true, 100, 0xFFFFFF00u) == Freshness::VALID);
    CHECK_STR(freshnessText(Freshness::STALE), "STALE");
}

int main() {
    test_decode_no_movement();
    test_decode_negative_delta();
    test_relay_not_ready();
    test_restored_flash();
    test_short_frame();
    test_freshness();

    printf("steering_diag_view: %d run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}
