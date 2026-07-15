/**
 * @file test_relay_health_diag.c
 * @brief Host tests for the evidence-graded relay/current classifier.
 */
#ifdef HOST_TEST
#include <stdio.h>
#include <string.h>
#include "relay_health_diag.h"

static int tests_run, tests_failed;
#define CHECK(e) do { tests_run++; if (!(e)) { tests_failed++; \
    printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #e); } } while (0)

static RelayHealthDiag base_ok(void)
{
    RelayHealthDiag d;
    memset(&d, 0, sizeof d);
    d.relay_commanded = true;
    d.relay_sequence_complete = true;
    d.power_ready = true;
    d.throttle_pct = 42.0f;
    d.traction_demand_pct = 29.0f;
    d.effective_demand_pct = 29.0f;
    d.final_pwm_pct = 18.0f;
    d.average_speed = 3.2f;
    d.wheel_speed[0] = d.wheel_speed[1] = 3.2f;
    d.wheel_speed[2] = d.wheel_speed[3] = 3.2f;
    d.any_wheel_moving = true;
    d.ina_ok_mask = 0x1FU;
    d.ina_expected_mask = 0x0FU;
    d.ina_valid_mask = 0x0FU;
    d.current_sample_age_ms = 12U;
    d.current_ch[0] = d.current_ch[1] = 2.0f;
    d.current_ch[2] = d.current_ch[3] = 2.0f;
    d.current_sum_abs = 8.0f;
    d.current_signed_sum = 8.0f;
    d.current_threshold = 2.34f;
    d.battery_voltage = 24.0f;
    d.post_relay_voltage_present = true;
    d.battery_consumption_rising = true;
    return d;
}

static RelayDiagReason_t classify(RelayHealthDiag d)
{
    return Relay_ClassifyHealth(&d);
}

int main(void)
{
    CHECK(classify(base_ok()) == RELAY_HEALTH_OK);

    /* Raised wheels can spin with only a few hundred mA.  This is measurable
     * current and must not be treated as a zero-current relay failure merely
     * because it is below the adaptive load expectation. */
    {
        RelayHealthDiag d = base_ok();
        d.current_ch[0] = d.current_ch[1] = 0.08f;
        d.current_ch[2] = d.current_ch[3] = 0.08f;
        d.current_sum_abs = 0.32f;
        d.current_signed_sum = 0.32f;
        CHECK(classify(d) == RELAY_HEALTH_OK);
    }

    /* No real output means the relay cannot be judged. */
    {
        RelayHealthDiag d = base_ok();
        d.final_pwm_pct = 0.0f;
        d.effective_demand_pct = 0.0f;
        d.current_sum_abs = 0.0f;
        d.current_signed_sum = 0.0f;
        CHECK(classify(d) == RELAY_DIAG_INCONCLUSIVE);
    }

    /* Wheels moving under PWM proves the relay is passing energy; a genuine
     * zero reading is a current-sense problem, never RELAY OPEN. */
    {
        RelayHealthDiag d = base_ok();
        memset(d.current_ch, 0, sizeof d.current_ch);
        d.current_sum_abs = 0.0f;
        d.current_signed_sum = 0.0f;
        CHECK(classify(d) == CURRENT_SENSE_INVALID);
        CHECK(classify(d) != RELAY_OPEN_CONFIRMED);
        CHECK(classify(d) != RELAY_OPEN_SUSPECTED);
    }

    /* With no motion and independent evidence that power exists: suspected. */
    {
        RelayHealthDiag d = base_ok();
        d.current_sum_abs = 0.0f;
        d.current_signed_sum = 0.0f;
        d.any_wheel_moving = false;
        d.post_relay_voltage_present = true;
        d.battery_consumption_rising = true;
        CHECK(classify(d) == RELAY_OPEN_SUSPECTED);
    }

    /* Only absent independent evidence permits the pure classifier's
     * CONFIRMED result.  The real vehicle currently downgrades this because it
     * has no post-relay feedback sensor. */
    {
        RelayHealthDiag d = base_ok();
        d.current_sum_abs = 0.0f;
        d.current_signed_sum = 0.0f;
        d.any_wheel_moving = false;
        d.post_relay_voltage_present = false;
        d.battery_consumption_rising = false;
        CHECK(classify(d) == RELAY_OPEN_CONFIRMED);
    }

    {
        RelayHealthDiag d = base_ok();
        d.ina_ok_mask = 0x0EU;
        d.current_sum_abs = 0.0f;
        CHECK(classify(d) == CURRENT_INA_MISSING);
    }
    {
        RelayHealthDiag d = base_ok();
        d.current_sample_age_ms = RELAY_DIAG_CURRENT_STALE_MS;
        CHECK(classify(d) == CURRENT_DATA_STALE);
    }
    {
        RelayHealthDiag d = base_ok();
        d.ina_valid_mask = 0x0EU;
        CHECK(classify(d) == CURRENT_SENSE_INVALID);
    }
    {
        RelayHealthDiag d = base_ok();
        d.current_signed_sum = -8.0f;
        CHECK(classify(d) == CURRENT_POLARITY_REVERSED);
    }
    {
        RelayHealthDiag d = base_ok();
        d.ina_expected_mask = 0U;
        CHECK(classify(d) == RELAY_DIAG_INCONCLUSIVE);
    }

    CHECK(Relay_ClassifyHealth(NULL) == RELAY_DIAG_INCONCLUSIVE);
    for (int r = RELAY_HEALTH_OK; r <= RELAY_DIAG_INCONCLUSIVE; ++r) {
        CHECK(Relay_DiagReasonStr((RelayDiagReason_t)r)[0] != '\0');
        CHECK(Relay_DiagConfidenceStr((RelayDiagReason_t)r)[0] != '\0');
    }

    printf("relay_health_diag: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
#endif
