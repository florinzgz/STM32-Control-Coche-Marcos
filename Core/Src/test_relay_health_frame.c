/**
  ****************************************************************************
  * @file    test_relay_health_frame.c
  * @brief   Host integration test (Problem 3): drives the REAL relay-health
  *          classifier (Relay_ClassifyHealth) and the shared 0x317 pack/unpack
  *          used by the firmware and the ESP32, over the audit's key scenarios:
  *          manual/inertia movement, stale current, INA missing, polarity,
  *          relay-open suspected, and a full round-trip of every field.
  *
  * Build (host):
  *   gcc -std=c11 -DHOST_TEST -ICore/Inc Core/Src/test_relay_health_frame.c \
  *       Core/Src/relay_health_diag.c -o /tmp/t && /tmp/t
  ****************************************************************************
  */

#include "relay_health_diag.h"
#include "relay_health_frame.h"

#include <stdio.h>
#include <string.h>

static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg) do {                                      \
    g_checks++;                                                     \
    if (!(cond)) { g_fails++;                                       \
        printf("  FAIL: %s (line %d)\n", (msg), __LINE__); }        \
} while (0)

/* A plausibly-healthy base snapshot: relay closed, demand present, all four
 * wheel INA present + valid + fresh, some current flowing. */
static RelayHealthDiag base_snapshot(void)
{
    RelayHealthDiag d;
    memset(&d, 0, sizeof d);
    d.relay_commanded         = true;
    d.relay_sequence_complete = true;
    d.power_ready             = true;
    d.throttle_pct            = 40.0f;
    d.traction_demand_pct     = 40.0f;
    d.effective_demand_pct    = 40.0f;
    d.final_pwm_pct           = 40.0f;
    d.average_speed           = 5.0f;
    d.wheel_speed[0] = d.wheel_speed[1] = 5.0f;
    d.wheel_speed[2] = d.wheel_speed[3] = 5.0f;
    d.any_wheel_moving        = true;
    d.ina_ok_mask             = 0x0F;
    d.ina_expected_mask       = 0x0F;
    d.ina_valid_mask          = 0x0F;
    d.current_sample_age_ms   = 20;
    d.current_ch[0] = d.current_ch[1] = 3.0f;
    d.current_ch[2] = d.current_ch[3] = 3.0f;
    d.current_sum_abs         = 12.0f;
    d.current_signed_sum      = 12.0f;
    d.current_threshold       = 2.3f;
    d.battery_voltage         = 24.0f;
    d.post_relay_voltage_present = true;
    d.battery_consumption_rising = true;
    return d;
}

static void test_classification(void)
{
    printf("Scenario: classification rules\n");

    /* Healthy: current confirms the relay is passing power. */
    RelayHealthDiag d = base_snapshot();
    CHECK(Relay_ClassifyHealth(&d) == RELAY_HEALTH_OK, "healthy -> OK");

    /* Rule A: wheels move under PWM but shunt reads ~0 A (fresh, valid). */
    d = base_snapshot();
    d.current_ch[0] = d.current_ch[1] = 0.0f;
    d.current_ch[2] = d.current_ch[3] = 0.0f;
    d.current_sum_abs = 0.0f;
    d.current_signed_sum = 0.0f;
    CHECK(Relay_ClassifyHealth(&d) == CURRENT_SENSE_INVALID,
          "moving + 0A fresh -> CURRENT_SENSE_INVALID");

    /* Rule B: PWM, relay closed, valid sensors, 0 A, no wheel motion. */
    d = base_snapshot();
    d.average_speed = 0.0f;
    d.wheel_speed[0] = d.wheel_speed[1] = 0.0f;
    d.wheel_speed[2] = d.wheel_speed[3] = 0.0f;
    d.any_wheel_moving = false;
    d.current_sum_abs = 0.0f;
    d.current_signed_sum = 0.0f;
    /* keep independent evidence "present" so it stays SUSPECTED not CONFIRMED */
    CHECK(Relay_ClassifyHealth(&d) == RELAY_OPEN_SUSPECTED,
          "PWM + no motion + 0A + evidence-present -> RELAY_OPEN_SUSPECTED");

    /* Rule C: same, but NO independent evidence -> CONFIRMED (pure classifier).
     * Firmware downgrades this to SUSPECTED because the HW has no post-relay
     * sensor; that downgrade is covered by the safety_system integration.   */
    d.post_relay_voltage_present = false;
    d.battery_consumption_rising = false;
    CHECK(Relay_ClassifyHealth(&d) == RELAY_OPEN_CONFIRMED,
          "PWM + no motion + 0A + no-evidence -> RELAY_OPEN_CONFIRMED");

    /* Stale current must NOT be read as 0 A. */
    d = base_snapshot();
    d.current_sum_abs = 0.0f;
    d.current_sample_age_ms = RELAY_DIAG_CURRENT_STALE_MS;
    CHECK(Relay_ClassifyHealth(&d) == CURRENT_DATA_STALE,
          "stale sample -> CURRENT_DATA_STALE");

    /* Missing expected wheel INA. */
    d = base_snapshot();
    d.ina_ok_mask = 0x07;   /* CH3 absent */
    CHECK(Relay_ClassifyHealth(&d) == CURRENT_INA_MISSING,
          "expected INA absent -> CURRENT_INA_MISSING");

    /* Manual hand-spin with relay NOT commanded: cannot judge. */
    d = base_snapshot();
    d.relay_commanded = false;
    d.relay_sequence_complete = false;
    d.power_ready = false;
    d.throttle_pct = 0.0f;
    d.final_pwm_pct = 0.0f;
    CHECK(Relay_ClassifyHealth(&d) == RELAY_DIAG_INCONCLUSIVE,
          "manual spin, relay off, no demand -> INCONCLUSIVE");

    /* Reversed installation: large |current| but wrong sign under forward PWM. */
    d = base_snapshot();
    d.current_signed_sum = -12.0f;
    CHECK(Relay_ClassifyHealth(&d) == CURRENT_POLARITY_REVERSED,
          "signed current opposite demand -> CURRENT_POLARITY_REVERSED");
}

static void test_roundtrip(void)
{
    printf("Scenario: 0x317 pack/unpack round-trip\n");

    RelayHealthDiag d = base_snapshot();
    d.current_ch[0] = d.current_ch[1] = 0.0f;
    d.current_ch[2] = d.current_ch[3] = 0.0f;
    d.current_sum_abs = 0.0f;
    d.current_signed_sum = 0.0f;
    d.diagnostic_reason = Relay_ClassifyHealth(&d); /* CURRENT_SENSE_INVALID */

    uint8_t bytes[8];
    RelayHealth_PackFrame(&d, bytes);

    RelayHealthFrame f;
    RelayHealth_UnpackFrame(bytes, &f);

    CHECK(f.reason == (uint8_t)CURRENT_SENSE_INVALID, "reason survives");
    CHECK(f.relay_commanded, "relay_commanded flag");
    CHECK(f.relay_sequence_complete, "seq flag");
    CHECK(f.power_ready, "power_ready flag");
    CHECK(f.any_wheel_moving, "any_wheel_moving flag");
    CHECK(f.current_valid, "current_valid flag");
    CHECK(!f.current_stale, "not stale");
    CHECK(!f.ina_missing, "ina present");
    CHECK(f.current_sum_ca == 0, "sum 0 centi-A");
    CHECK(f.throttle_pct == 40, "throttle 40%");
    CHECK(f.final_pwm_pct == 40, "pwm 40%");
    CHECK(f.sample_age_ms == 20, "age 20 ms");

    /* Current magnitude + saturation. */
    d = base_snapshot();               /* sum_abs = 12.0 A -> 1200 centi-A */
    RelayHealth_PackFrame(&d, bytes);
    RelayHealth_UnpackFrame(bytes, &f);
    CHECK(f.current_sum_ca == 1200, "12.0A -> 1200 centi-A");
    CHECK(f.reason == 0, "unclassified snapshot reason 0");

    d.current_sum_abs = 10000.0f;      /* saturates centi-A field */
    RelayHealth_PackFrame(&d, bytes);
    RelayHealth_UnpackFrame(bytes, &f);
    CHECK(f.current_sum_ca == 65535, "current centi-A saturates");

    /* Stale + missing flags. */
    d = base_snapshot();
    d.current_sample_age_ms = 5000;
    d.ina_ok_mask = 0x07;
    RelayHealth_PackFrame(&d, bytes);
    RelayHealth_UnpackFrame(bytes, &f);
    CHECK(f.current_stale, "stale flag set");
    CHECK(f.ina_missing, "ina_missing flag set");
    CHECK(f.sample_age_ms == 5000, "age 5000 ms");
    CHECK(!f.current_valid, "current not valid when CH3 missing");
}

int main(void)
{
    printf("=== test_relay_health_frame ===\n");
    test_classification();
    test_roundtrip();
    printf("Checks: %d, Failures: %d\n", g_checks, g_fails);
    if (g_fails == 0) {
        printf("RESULT: PASS\n");
        return 0;
    }
    printf("RESULT: FAIL\n");
    return 1;
}
