// Host cross-parity test: packs raw 0x31D bytes using the STM32-side
// authoritative Core/Inc/wheel_equality_frame.h Pack() function, then
// decodes them with the ESP32-side wheel_equality_view.h and asserts
// field-for-field equality — same pattern as test_service_diag_view.cpp /
// test_traction_limit_diag_decode.cpp.
//
// NOTE: wheel_equality_frame.h #defines WHEQ_FIELD_SPEED/CURRENT/HEALTH/
// VERDICT as plain macros (needed for its own Pack/Unpack switch). Those
// exact names are ALSO defined as `can::WHEQ_FIELD_*` constexpr in
// can_ids.h. Once this translation unit includes wheel_equality_frame.h,
// writing the qualified form "can::WHEQ_FIELD_SPEED" would have the bare
// macro substituted first (breaking the "::"), so this file intentionally
// uses the bare macro name for those four constants everywhere and reserves
// the "can::" prefix for identifiers with no colliding STM32-side macro
// (WHEQ_WHEEL_*, WHEQ_*_VERDICT_*, WHEQ_CAUSE_*, WHEQ_DRIVER_REASON_*).
#include <cstdio>
#include <cstring>

#include "wheel_equality_view.h"
#include "wheel_equality_frame.h"

static int failed;
#define CHECK(expr) do { \
    if (!(expr)) { \
        std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        ++failed; \
    } \
} while (0)

static uint8_t packField(const WheelEqualityFrame_t& in, uint8_t raw[WHEEL_EQUALITY_FRAME_DLC]) {
    return WheelEqualityFrame_Pack(&in, raw);
}

int main() {
    // ---- FIELD_SPEED round-trip (FL, Fase 1 only) --------------------------
    WheelEqualityFrame_t speedIn{};
    speedIn.wheel                  = can::WHEQ_WHEEL_FL;
    speedIn.field_id               = WHEQ_FIELD_SPEED;
    speedIn.phase2_included        = false;
    speedIn.pulses_per_sec_25      = 1234U;
    speedIn.pulses_per_sec_50      = 2345U;
    speedIn.normalized_speed_x1000 = 987U;
    speedIn.deviation_pct          = 12U;

    uint8_t speedRaw[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(packField(speedIn, speedRaw) == WHEEL_EQUALITY_FRAME_DLC);

    wheel_equality_view::FrameView speedView{};
    CHECK(wheel_equality_view::decode(speedRaw, sizeof(speedRaw), speedView));
    CHECK(speedView.wheel == can::WHEQ_WHEEL_FL);
    CHECK(speedView.fieldId == WHEQ_FIELD_SPEED);
    CHECK(speedView.phase2Included == false);
    CHECK(speedView.pulsesPerSec25 == 1234U);
    CHECK(speedView.pulsesPerSec50 == 2345U);
    CHECK(speedView.normalizedSpeedX1000 == 987U);
    CHECK(speedView.deviationPct == 12U);

    // ---- FIELD_CURRENT round-trip (FR, Fase 2 included) --------------------
    WheelEqualityFrame_t currentIn{};
    currentIn.wheel                = can::WHEQ_WHEEL_FR;
    currentIn.field_id             = WHEQ_FIELD_CURRENT;
    currentIn.phase2_included      = true;
    currentIn.current_ma_25        = 500U;
    currentIn.current_ma_50        = 1500U;
    currentIn.slope_ma_per_pct_x10 = 250U;   // 25.0 mA per PWM-percent
    currentIn.probable_cause       = can::WHEQ_CAUSE_MECHANICAL;

    uint8_t currentRaw[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(packField(currentIn, currentRaw) == WHEEL_EQUALITY_FRAME_DLC);

    wheel_equality_view::FrameView currentView{};
    CHECK(wheel_equality_view::decode(currentRaw, sizeof(currentRaw), currentView));
    CHECK(currentView.wheel == can::WHEQ_WHEEL_FR);
    CHECK(currentView.fieldId == WHEQ_FIELD_CURRENT);
    CHECK(currentView.phase2Included == true);
    CHECK(currentView.currentMa25 == 500U);
    CHECK(currentView.currentMa50 == 1500U);
    CHECK(currentView.slopeMaPerPctX10 == 250U);
    CHECK(currentView.probableCause == can::WHEQ_CAUSE_MECHANICAL);
    CHECK(std::strcmp(wheel_equality_view::causeText(currentView.probableCause),
                       "RESISTENCIA MECANICA (rodamiento/freno/reductora/rozamiento)") == 0);
    CHECK(std::strcmp(wheel_equality_view::causeShortText(currentView.probableCause),
                       "MECANICA") == 0);

    // ---- FIELD_HEALTH round-trip (RL, DS18B20 present, half-bridge WARN) ---
    WheelEqualityFrame_t healthIn{};
    healthIn.wheel               = can::WHEQ_WHEEL_RL;
    healthIn.field_id            = WHEQ_FIELD_HEALTH;
    healthIn.asymmetry_pct_x10   = 125U;   // 12.5 %
    healthIn.delta_temp_c_x10    = 87U;    // 8.7 C
    healthIn.halfbridge_verdict  = can::WHEQ_HALFBRIDGE_WARN;
    healthIn.temp_present        = true;

    uint8_t healthRaw[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(packField(healthIn, healthRaw) == WHEEL_EQUALITY_FRAME_DLC);

    wheel_equality_view::FrameView healthView{};
    CHECK(wheel_equality_view::decode(healthRaw, sizeof(healthRaw), healthView));
    CHECK(healthView.wheel == can::WHEQ_WHEEL_RL);
    CHECK(healthView.fieldId == WHEQ_FIELD_HEALTH);
    CHECK(healthView.asymmetryPctX10 == 125U);
    CHECK(healthView.deltaTempCX10 == 87U);
    CHECK(healthView.halfBridgeVerdict == can::WHEQ_HALFBRIDGE_WARN);
    CHECK(healthView.tempPresent == true);
    CHECK(std::strcmp(wheel_equality_view::halfBridgeVerdictText(healthView.halfBridgeVerdict),
                       "WARN_HALFBRIDGE") == 0);

    // ---- FIELD_VERDICT round-trip (RR, FAIL/DEGRADADO, Fase 2 ran) ---------
    WheelEqualityFrame_t verdictIn{};
    verdictIn.wheel              = can::WHEQ_WHEEL_RR;
    verdictIn.field_id           = WHEQ_FIELD_VERDICT;
    verdictIn.wheel_verdict      = can::WHEQ_WHEEL_VERDICT_FAIL;
    verdictIn.driver_verdict     = can::WHEQ_DRIVER_DEGRADADO;
    verdictIn.phase2_ran         = true;
    verdictIn.driver_reason_mask = can::WHEQ_DRIVER_REASON_HALFBRIDGE | can::WHEQ_DRIVER_REASON_THERMAL;

    uint8_t verdictRaw[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(packField(verdictIn, verdictRaw) == WHEEL_EQUALITY_FRAME_DLC);

    wheel_equality_view::FrameView verdictView{};
    CHECK(wheel_equality_view::decode(verdictRaw, sizeof(verdictRaw), verdictView));
    CHECK(verdictView.wheel == can::WHEQ_WHEEL_RR);
    CHECK(verdictView.fieldId == WHEQ_FIELD_VERDICT);
    CHECK(verdictView.wheelVerdict == can::WHEQ_WHEEL_VERDICT_FAIL);
    CHECK(verdictView.driverVerdict == can::WHEQ_DRIVER_DEGRADADO);
    CHECK(verdictView.phase2Ran == true);
    CHECK(verdictView.driverReasonMask ==
          (can::WHEQ_DRIVER_REASON_HALFBRIDGE | can::WHEQ_DRIVER_REASON_THERMAL));
    CHECK(wheel_equality_view::isWheelVerdictFault(verdictView.wheelVerdict));
    CHECK(std::strcmp(wheel_equality_view::wheelVerdictText(verdictView.wheelVerdict), "FAIL") == 0);
    CHECK(std::strcmp(wheel_equality_view::driverVerdictText(verdictView.driverVerdict),
                       "DEGRADADO") == 0);

    // ---- Extreme representable values (max u16 / max u8 wire fields) -------
    WheelEqualityFrame_t satIn{};
    satIn.wheel                  = can::WHEQ_WHEEL_FL;
    satIn.field_id                = WHEQ_FIELD_SPEED;
    satIn.pulses_per_sec_25       = 65535U;    // exact max u16
    satIn.pulses_per_sec_50       = 65535U;    // exact max u16
    satIn.normalized_speed_x1000  = 0U;        // exact min
    satIn.deviation_pct           = 255U;       // exact max u8

    uint8_t satRaw[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(packField(satIn, satRaw) == WHEEL_EQUALITY_FRAME_DLC);
    wheel_equality_view::FrameView satView{};
    CHECK(wheel_equality_view::decode(satRaw, sizeof(satRaw), satView));
    CHECK(satView.pulsesPerSec25 == 65535U);
    CHECK(satView.pulsesPerSec50 == 65535U);
    CHECK(satView.normalizedSpeedX1000 == 0U);
    CHECK(satView.deviationPct == 255U);

    // ---- Ackermann-offset wheel verdict + FAIL half-bridge (rare paths) ----
    WheelEqualityFrame_t ackerIn{};
    ackerIn.wheel          = can::WHEQ_WHEEL_FR;
    ackerIn.field_id       = WHEQ_FIELD_VERDICT;
    ackerIn.wheel_verdict  = can::WHEQ_WHEEL_VERDICT_FAIL_ACKERMANN_OFFSET;
    ackerIn.driver_verdict = can::WHEQ_DRIVER_SOSPECHOSO;

    uint8_t ackerRaw[WHEEL_EQUALITY_FRAME_DLC] = {0};
    CHECK(packField(ackerIn, ackerRaw) == WHEEL_EQUALITY_FRAME_DLC);
    wheel_equality_view::FrameView ackerView{};
    CHECK(wheel_equality_view::decode(ackerRaw, sizeof(ackerRaw), ackerView));
    CHECK(ackerView.wheelVerdict == can::WHEQ_WHEEL_VERDICT_FAIL_ACKERMANN_OFFSET);
    CHECK(wheel_equality_view::isWheelVerdictFault(ackerView.wheelVerdict));
    CHECK(std::strcmp(wheel_equality_view::wheelVerdictText(ackerView.wheelVerdict),
                       "FAIL_ACKERMANN_OFFSET") == 0);
    CHECK(std::strcmp(wheel_equality_view::wheelVerdictShortText(ackerView.wheelVerdict),
                       "F.ACKER") == 0);
    CHECK(std::strcmp(wheel_equality_view::driverVerdictText(ackerView.driverVerdict),
                       "SOSPECHOSO") == 0);

    // ---- PASS / SANO / NONE-cause legibility (no defects) -------------------
    CHECK(!wheel_equality_view::isWheelVerdictFault(can::WHEQ_WHEEL_VERDICT_PASS));
    CHECK(std::strcmp(wheel_equality_view::wheelVerdictText(can::WHEQ_WHEEL_VERDICT_PASS),
                       "PASS") == 0);
    CHECK(std::strcmp(wheel_equality_view::driverVerdictText(can::WHEQ_DRIVER_SANO), "SANO") == 0);
    CHECK(std::strcmp(wheel_equality_view::causeText(can::WHEQ_CAUSE_NONE), "-") == 0);
    CHECK(std::strcmp(wheel_equality_view::causeText(can::WHEQ_CAUSE_ELECTRICAL),
                       "FALTA DE PAR ELECTRICO (escobillas/conexion/driver/caida tension)") == 0);
    CHECK(std::strcmp(wheel_equality_view::causeText(can::WHEQ_CAUSE_SENSOR),
                       "SENSOR (no la rueda)") == 0);
    CHECK(std::strcmp(wheel_equality_view::causeText(can::WHEQ_CAUSE_OTHERS_BRAKED),
                       "REVISAR SI LAS OTRAS TRES ESTAN FRENADAS") == 0);
    CHECK(std::strcmp(wheel_equality_view::halfBridgeVerdictText(can::WHEQ_HALFBRIDGE_PASS),
                       "PASS") == 0);
    CHECK(std::strcmp(wheel_equality_view::halfBridgeVerdictText(can::WHEQ_HALFBRIDGE_FAIL),
                       "FAIL_HALFBRIDGE") == 0);

    // ---- Unknown enum values must fall back to "?" (never a bare number) ---
    CHECK(std::strcmp(wheel_equality_view::wheelVerdictText(99U), "?") == 0);
    CHECK(std::strcmp(wheel_equality_view::driverVerdictText(99U), "?") == 0);
    CHECK(std::strcmp(wheel_equality_view::causeText(99U), "?") == 0);
    CHECK(std::strcmp(wheel_equality_view::halfBridgeVerdictText(99U), "?") == 0);

    // ---- Invalid DLC must be rejected outright (no forward-compat form) ----
    wheel_equality_view::FrameView rejectView{};
    CHECK(!wheel_equality_view::decode(speedRaw, 7U, rejectView));
    CHECK(!wheel_equality_view::decode(speedRaw, 9U, rejectView));
    CHECK(!wheel_equality_view::decode(speedRaw, 0U, rejectView));
    CHECK(!wheel_equality_view::decode(nullptr, 0U, rejectView));
    CHECK(!wheel_equality_view::decode(nullptr, WHEEL_EQUALITY_FRAME_DLC, rejectView));

    // ---- Freshness: exact 3000/3001 ms boundary (mandatory per spec) -------
    CHECK(wheel_equality_view::freshness(false, 5000U, 0U) ==
          wheel_equality_view::Freshness::NEVER_RECEIVED);
    CHECK(wheel_equality_view::freshness(true, 2999U, 0U) ==
          wheel_equality_view::Freshness::VALID);
    /* Exactly 3000 ms elapsed: boundary is inclusive, must be VALID. */
    CHECK(wheel_equality_view::freshness(true, 3000U, 0U) ==
          wheel_equality_view::Freshness::VALID);
    CHECK(wheel_equality_view::freshness(true, 3001U, 0U) ==
          wheel_equality_view::Freshness::STALE);

    if (failed != 0) return 1;
    std::puts("Wheel equality view decode tests: PASS");
    return 0;
}
