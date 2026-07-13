// test_pedal_cal_session_view.cpp — audit P5/P6.
//
// Host tests for the ESP32-side presentation of the guided PedalCalSession
// (0x319 DIAG_PEDAL_CAL_SESSION).  Exercises the REAL production helpers in
// pedal_cal_session_view.h (state/reason text, freshness, active) and a decode
// of the 0x319 wire layout matching can_rx.cpp decodePedalCalSession().
//
// Build (from repo root):
//   g++ -std=c++17 -Wall -Wextra -Werror -Iesp32/src -Iesp32/include
//       esp32/src/test_pedal_cal_session_view.cpp -o /tmp/test_pcsv
//   /tmp/test_pcsv

#include <cstdio>
#include <cstdint>
#include <cstring>

#include "pedal_cal_session_view.h"
#include "can_ids.h"

static int tests_run = 0, tests_failed = 0;
#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

using namespace pedalcal;

// ---- Decode mirror of can_rx.cpp decodePedalCalSession() ------------------
struct SessFrame { uint8_t dlc; uint8_t d[8]; };
struct SessData { uint8_t state, flags; uint16_t reason, adcMin, adcMax; bool ok; };
static uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static SessData decode(const SessFrame& f) {
    SessData s{};
    if (f.dlc < 8) { s.ok = false; return s; }
    s.state  = f.d[0];
    s.flags  = f.d[1];
    s.reason = rd16(&f.d[2]);
    s.adcMin = rd16(&f.d[4]);
    s.adcMax = rd16(&f.d[6]);
    s.ok = true;
    return s;
}

// State text mirrors the STM32 enum values exactly.
static void test_state_text(void) {
    CHECK(strcmp(sessionStateText(can::PEDCAL_SESS_IDLE),          "IDLE") == 0);
    CHECK(strcmp(sessionStateText(can::PEDCAL_SESS_CAPTURING_MIN), "CAPTURING MIN") == 0);
    CHECK(strcmp(sessionStateText(can::PEDCAL_SESS_CAPTURING_MAX), "CAPTURING MAX") == 0);
    CHECK(strcmp(sessionStateText(can::PEDCAL_SESS_READY_TO_SAVE), "READY TO SAVE") == 0);
    CHECK(strcmp(sessionStateText(can::PEDCAL_SESS_COMPLETED),     "COMPLETED") == 0);
    CHECK(strcmp(sessionStateText(can::PEDCAL_SESS_ABORTED),       "ABORTED") == 0);
    CHECK(strcmp(sessionStateText(200),                           "UNKNOWN") == 0);
}

// active() true only for the in-progress states.
static void test_active(void) {
    CHECK(sessionActive(can::PEDCAL_SESS_IDLE)      == false);
    CHECK(sessionActive(can::PEDCAL_SESS_COMPLETED) == false);
    CHECK(sessionActive(can::PEDCAL_SESS_ABORTED)   == false);
    CHECK(sessionActive(can::PEDCAL_SESS_CAPTURING_MIN)  == true);
    CHECK(sessionActive(can::PEDCAL_SESS_READY_TO_SAVE)  == true);
    CHECK(sessionActive(can::PEDCAL_SESS_SAVING)         == true);
}

// Reason text: OK, single bits, and priority (emergency wins over movement).
static void test_reason_text(void) {
    CHECK(strcmp(sessionReasonText(0x0000), "OK") == 0);
    CHECK(strcmp(sessionReasonText(can::PEDCAL_SESS_BLOCK_NOT_STANDBY),   "NOT STANDBY") == 0);
    CHECK(strcmp(sessionReasonText(can::PEDCAL_SESS_BLOCK_GEAR),          "SHIFT TO P/N") == 0);
    CHECK(strcmp(sessionReasonText(can::PEDCAL_SESS_ABORT_MOVEMENT),      "VEHICLE MOVING") == 0);
    CHECK(strcmp(sessionReasonText(can::PEDCAL_SESS_ABORT_EMERGENCY),     "EMERGENCY STOP") == 0);
    CHECK(strcmp(sessionReasonText(can::PEDCAL_SESS_FAIL_READBACK),       "SAVE FAILED") == 0);
    // Emergency is higher priority than a concurrent movement bit.
    CHECK(strcmp(sessionReasonText(can::PEDCAL_SESS_ABORT_EMERGENCY |
                                   can::PEDCAL_SESS_ABORT_MOVEMENT),     "EMERGENCY STOP") == 0);
}

// Freshness: NEVER when no frame, FRESH within window, STALE after timeout.
static void test_freshness(void) {
    CHECK(sessionFreshness(0, 5000) == Freshness::NEVER);
    CHECK(sessionFreshness(4000, 4500) == Freshness::FRESH);
    CHECK(sessionFreshness(1000, 1000 + kSessionStaleMs) == Freshness::FRESH);
    CHECK(sessionFreshness(1000, 1000 + kSessionStaleMs + 1) == Freshness::STALE);
}

// 0x319 decode: a READY_TO_SAVE frame with captured endpoints round-trips.
static void test_decode_frame(void) {
    SessFrame f{};
    f.dlc = 8;
    f.d[0] = can::PEDCAL_SESS_READY_TO_SAVE;
    f.d[1] = 0x01 | 0x02 | 0x04;   // active | have_min | have_max
    uint16_t reason = 0x0000;
    f.d[2] = reason & 0xFF; f.d[3] = reason >> 8;
    f.d[4] = 60 & 0xFF;     f.d[5] = (60 >> 8);      // adc_min = 60
    f.d[6] = 3900 & 0xFF;   f.d[7] = (3900 >> 8);    // adc_max = 3900

    SessData s = decode(f);
    CHECK(s.ok);
    CHECK(s.state == can::PEDCAL_SESS_READY_TO_SAVE);
    CHECK((s.flags & 0x02) != 0);
    CHECK((s.flags & 0x04) != 0);
    CHECK(s.reason == 0);
    CHECK(s.adcMin == 60);
    CHECK(s.adcMax == 3900);
    CHECK(sessionActive(s.state) == true);

    // A short frame is rejected.
    SessFrame shortF{}; shortF.dlc = 7;
    CHECK(decode(shortF).ok == false);
}

// Extended reason text: operator-cancel and lock-lost come from flag bits 6/7
// (they live above bit 15 in the firmware reason word), with the correct
// priority relative to the hard safety aborts.
static void test_reason_text_ex(void) {
    using namespace pedalcal;
    // Operator cancel: benign, only when no higher-priority cause is set.
    CHECK(strcmp(sessionReasonTextEx(0x0000u, kFlagAbortOperator),
                 "OPERATOR CANCEL") == 0);
    // Lock lost is safety-relevant and outranks a co-set operator bit.
    CHECK(strcmp(sessionReasonTextEx(0x0000u,
                 (uint8_t)(kFlagAbortLockLost | kFlagAbortOperator)),
                 "LOCK LOST") == 0);
    // Emergency still dominates over lock-lost.
    CHECK(strcmp(sessionReasonTextEx(0x0100u, kFlagAbortLockLost),
                 "EMERGENCY STOP") == 0);
    // No bits at all -> OK.
    CHECK(strcmp(sessionReasonTextEx(0x0000u, 0x00u), "OK") == 0);
    // Falls back to the low-16 reason text when no flag bit is set.
    CHECK(strcmp(sessionReasonTextEx(0x2000u, 0x00u), "RANGE TOO SMALL") == 0);
}

// The pedal is expected pressed only in PRESS FULLY / CAPTURING MAX, so the
// 0x308 "pedal not released" gate must not read as BLOCKED there (audit P5.5).
static void test_pedal_expected_pressed(void) {
    using namespace pedalcal;
    CHECK(pedalExpectedPressed(can::PEDCAL_SESS_WAIT_FULL_PRESS) == true);
    CHECK(pedalExpectedPressed(can::PEDCAL_SESS_CAPTURING_MAX)   == true);
    CHECK(pedalExpectedPressed(can::PEDCAL_SESS_CAPTURING_MIN)   == false);
    CHECK(pedalExpectedPressed(can::PEDCAL_SESS_READY_TO_SAVE)   == false);
    CHECK(pedalExpectedPressed(can::PEDCAL_SESS_IDLE)            == false);
}

int main() {
    test_state_text();
    test_active();
    test_reason_text();
    test_reason_text_ex();
    test_pedal_expected_pressed();
    test_freshness();
    test_decode_frame();
    printf("pedal_cal_session_view: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
