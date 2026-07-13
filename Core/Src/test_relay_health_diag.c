/**
  ****************************************************************************
  * @file    test_relay_health_diag.c
  * @brief   Host unit tests for the traction-relay health classifier
  *          (Relay_ClassifyHealth).
  *
  *          Covers the audit's Problem-3 test matrix: valid current, INA
  *          missing, stale data, motors moving with 0 A, PWM=0, relay not
  *          ready, relay suspected, relay confirmed (only with sufficient
  *          evidence), current-sense invalid and polarity reversed.
  *
  *          Compile (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  *                Core/Src/test_relay_health_diag.c \
  *                Core/Src/relay_health_diag.c -lm \
  *                -o /tmp/test_relay_health_diag
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <string.h>

#include "relay_health_diag.h"

static int tests_run = 0, tests_failed = 0;

#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* Healthy driving baseline: relay closed, demand present, 4 wheel INAs OK,
 * fresh valid readings, real current flowing, wheels turning.            */
static RelayHealthDiag base_ok(void)
{
    RelayHealthDiag d;
    memset(&d, 0, sizeof(d));
    d.relay_commanded          = true;
    d.relay_sequence_complete  = true;
    d.power_ready              = true;
    d.throttle_pct             = 42.0f;
    d.traction_demand_pct      = 29.0f;
    d.effective_demand_pct     = 29.0f;
    d.final_pwm_pct            = 18.0f;
    d.average_speed            = 3.2f;
    d.wheel_speed[0] = d.wheel_speed[1] = 3.2f;
    d.wheel_speed[2] = d.wheel_speed[3] = 3.2f;
    d.any_wheel_moving         = true;
    d.ina_ok_mask              = 0x1F;   /* CH0-3 + battery CH4 */
    d.ina_expected_mask        = 0x0F;   /* wheels expected     */
    d.ina_valid_mask           = 0x0F;
    d.current_sample_age_ms    = 12;
    d.current_ch[0] = d.current_ch[1] = 2.0f;
    d.current_ch[2] = d.current_ch[3] = 2.0f;
    d.current_sum_abs          = 8.0f;
    d.current_signed_sum       = 8.0f;
    d.current_threshold        = 2.34f;
    d.battery_voltage          = 24.0f;
    d.post_relay_voltage_present = true;
    d.battery_consumption_rising = true;
    d.scale_invalid            = false;
    d.polarity_reversed        = false;
    d.debounce_count           = 0;
    d.elapsed_since_demand_ms  = 500;
    return d;
}

static RelayDiagReason_t classify(RelayHealthDiag d)
{
    return Relay_ClassifyHealth(&d);
}

int main(void)
{
    /* --- Healthy: current confirms relay is passing power --- */
    CHECK(classify(base_ok()) == RELAY_HEALTH_OK);

    /* --- Preconditions not met → inconclusive --- */
    {
        RelayHealthDiag d = base_ok();
        d.relay_commanded = false;
        CHECK(classify(d) == RELAY_DIAG_INCONCLUSIVE);
        d = base_ok(); d.relay_sequence_complete = false;
        CHECK(classify(d) == RELAY_DIAG_INCONCLUSIVE);
        d = base_ok(); d.power_ready = false;
        CHECK(classify(d) == RELAY_DIAG_INCONCLUSIVE);
        d = base_ok(); d.throttle_pct = 0.0f; d.final_pwm_pct = 0.0f;
        CHECK(classify(d) == RELAY_DIAG_INCONCLUSIVE);
    }

    /* --- Rule A: motors move + PWM>0 + 0 A → CURRENT_SENSE_INVALID --- */
    {
        RelayHealthDiag d = base_ok();
        d.current_ch[0] = d.current_ch[1] = d.current_ch[2] = d.current_ch[3] = 0.0f;
        d.current_sum_abs = 0.0f;
        d.current_signed_sum = 0.0f;
        CHECK(classify(d) == CURRENT_SENSE_INVALID);
    }

    /* --- Rule B: PWM>0, valid sensors, 0 A, no motion → SUSPECTED --- */
    {
        RelayHealthDiag d = base_ok();
        d.current_sum_abs = 0.0f;
        d.current_signed_sum = 0.0f;
        d.any_wheel_moving = false;
        d.average_speed = 0.0f;
        memset(d.wheel_speed, 0, sizeof(d.wheel_speed));
        /* independent evidence says power IS present → only suspected */
        d.post_relay_voltage_present = true;
        d.battery_consumption_rising = true;
        CHECK(classify(d) == RELAY_OPEN_SUSPECTED);
    }

    /* --- Rule C: same but NO independent power evidence → CONFIRMED --- */
    {
        RelayHealthDiag d = base_ok();
        d.current_sum_abs = 0.0f;
        d.current_signed_sum = 0.0f;
        d.any_wheel_moving = false;
        d.average_speed = 0.0f;
        memset(d.wheel_speed, 0, sizeof(d.wheel_speed));
        d.post_relay_voltage_present = false;
        d.battery_consumption_rising = false;
        CHECK(classify(d) == RELAY_OPEN_CONFIRMED);
    }

    /* --- INA missing dominates the 0 A conclusion --- */
    {
        RelayHealthDiag d = base_ok();
        d.current_sum_abs = 0.0f;
        d.ina_ok_mask = 0x0E;   /* CH0 missing */
        CHECK(classify(d) == CURRENT_INA_MISSING);
    }

    /* --- Stale data is not "0 A valid" --- */
    {
        RelayHealthDiag d = base_ok();
        d.current_sum_abs = 0.0f;
        d.current_sample_age_ms = RELAY_DIAG_CURRENT_STALE_MS;
        CHECK(classify(d) == CURRENT_DATA_STALE);
    }

    /* --- Scale invalid --- */
    {
        RelayHealthDiag d = base_ok();
        d.current_sum_abs = 0.0f;
        d.scale_invalid = true;
        CHECK(classify(d) == CURRENT_SCALE_INVALID);
    }

    /* --- One wheel channel reading invalid --- */
    {
        RelayHealthDiag d = base_ok();
        d.current_sum_abs = 0.0f;
        d.ina_valid_mask = 0x0E;   /* CH0 invalid */
        CHECK(classify(d) == CURRENT_SENSE_INVALID);
    }

    /* --- Explicit reversed-polarity flag --- */
    {
        RelayHealthDiag d = base_ok();
        d.polarity_reversed = true;
        CHECK(classify(d) == CURRENT_POLARITY_REVERSED);
    }

    /* --- Reversed installation: large |current| but wrong sign --- */
    {
        RelayHealthDiag d = base_ok();
        d.current_signed_sum = -8.0f;   /* forward demand, negative sum */
        CHECK(classify(d) == CURRENT_POLARITY_REVERSED);
    }

    /* --- No wheel INA enabled at all → inconclusive --- */
    {
        RelayHealthDiag d = base_ok();
        d.ina_expected_mask = 0x00;
        CHECK(classify(d) == RELAY_DIAG_INCONCLUSIVE);
    }

    /* --- NULL guard + labels --- */
    {
        CHECK(Relay_ClassifyHealth(NULL) == RELAY_DIAG_INCONCLUSIVE);
        for (int r = RELAY_HEALTH_OK; r <= RELAY_DIAG_INCONCLUSIVE; r++) {
            CHECK(Relay_DiagReasonStr((RelayDiagReason_t)r)[0] != '\0');
            CHECK(Relay_DiagConfidenceStr((RelayDiagReason_t)r)[0] != '\0');
        }
        CHECK(Relay_DiagConfidenceStr(RELAY_OPEN_CONFIRMED)[0] == 'C'); /* CONFIRMADO */
    }

    printf("relay_health_diag: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#endif /* HOST_TEST */
