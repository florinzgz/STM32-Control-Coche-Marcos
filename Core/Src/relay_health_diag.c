/**
  ****************************************************************************
  * @file    relay_health_diag.c
  * @brief   Pure classifier for traction-relay / current-sense health.
  *
  * Implements the audit's decision rules:
  *   A. Wheels move + real PWM>0 + genuinely zero CH0–CH3 current
  *      → CURRENT_SENSE_INVALID, never relay open.
  *   B. Real PWM>0 + relay ON + valid sensors + zero current + no motion
  *      → RELAY_OPEN_SUSPECTED.
  *   C. Confirmation additionally requires independent power-path evidence.
  ****************************************************************************
  */

#include "relay_health_diag.h"

#include <stddef.h>

static float diag_fabs(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* A relay-open conclusion requires genuinely near-zero current, not merely
 * current below the adaptive load expectation.  Wheels raised from the floor
 * can legitimately spin below 1 A; treating the 1.5–4 A adaptive threshold as
 * "zero" caused false diagnostics during no-load tests. */
#define RELAY_DIAG_ZERO_CURRENT_A 0.10f

static bool diag_current_effectively_zero(const RelayHealthDiag *d)
{
    return d->current_sum_abs < RELAY_DIAG_ZERO_CURRENT_A;
}

RelayDiagReason_t Relay_ClassifyHealth(const RelayHealthDiag *d)
{
    if (d == NULL) {
        return RELAY_DIAG_INCONCLUSIVE;
    }

    bool demand_present = (d->throttle_pct > 0.0f) ||
                          (d->final_pwm_pct > 0.0f);
    if (!d->relay_commanded || !d->relay_sequence_complete ||
        !d->power_ready || !demand_present) {
        return RELAY_DIAG_INCONCLUSIVE;
    }

    uint8_t expected_wheels =
        (uint8_t)(d->ina_expected_mask & RELAY_DIAG_WHEEL_MASK);
    if (expected_wheels == 0U) {
        return RELAY_DIAG_INCONCLUSIVE;
    }
    if ((uint8_t)(expected_wheels & d->ina_ok_mask) != expected_wheels) {
        return CURRENT_INA_MISSING;
    }
    if (d->current_sample_age_ms >= RELAY_DIAG_CURRENT_STALE_MS) {
        return CURRENT_DATA_STALE;
    }
    if (d->scale_invalid) {
        return CURRENT_SCALE_INVALID;
    }
    if ((uint8_t)(expected_wheels & d->ina_valid_mask) != expected_wheels) {
        return CURRENT_SENSE_INVALID;
    }
    if (d->polarity_reversed) {
        return CURRENT_POLARITY_REVERSED;
    }

    if (!diag_current_effectively_zero(d)) {
        if ((d->final_pwm_pct > 0.0f) &&
            (d->current_signed_sum < 0.0f) &&
            (diag_fabs(d->current_signed_sum) >= d->current_threshold)) {
            return CURRENT_POLARITY_REVERSED;
        }
        return RELAY_HEALTH_OK;
    }

    /* Motion under real PWM proves that the traction power path is passing
     * energy.  A zero-current report is therefore a current-sense fault. */
    if ((d->final_pwm_pct > 0.0f) && d->any_wheel_moving) {
        return CURRENT_SENSE_INVALID;
    }

    if ((d->final_pwm_pct > 0.0f) && !d->any_wheel_moving) {
        if (!d->post_relay_voltage_present &&
            !d->battery_consumption_rising) {
            return RELAY_OPEN_CONFIRMED;
        }
        return RELAY_OPEN_SUSPECTED;
    }

    return RELAY_DIAG_INCONCLUSIVE;
}

const char *Relay_DiagReasonStr(RelayDiagReason_t reason)
{
    switch (reason) {
    case RELAY_HEALTH_OK:            return "OK";
    case RELAY_OPEN_CONFIRMED:       return "RELAY OPEN CONFIRMED";
    case RELAY_OPEN_SUSPECTED:       return "RELAY OPEN SUSPECTED";
    case CURRENT_SENSE_INVALID:      return "CURRENT SENSE INVALID";
    case CURRENT_SHUNT_OPEN:         return "SHUNT OPEN";
    case CURRENT_SHUNT_BYPASSED:     return "SHUNT BYPASSED";
    case CURRENT_POLARITY_REVERSED:  return "POLARITY REVERSED";
    case CURRENT_DATA_STALE:         return "CURRENT DATA STALE";
    case CURRENT_INA_MISSING:        return "INA MISSING";
    case CURRENT_SCALE_INVALID:      return "CURRENT SCALE INVALID";
    case RELAY_DIAG_INCONCLUSIVE:
    default:                         return "INCONCLUSIVE";
    }
}

const char *Relay_DiagConfidenceStr(RelayDiagReason_t reason)
{
    switch (reason) {
    case RELAY_OPEN_CONFIRMED:
        return "CONFIRMADO";
    case RELAY_HEALTH_OK:
    case RELAY_OPEN_SUSPECTED:
    case CURRENT_SENSE_INVALID:
    case CURRENT_SHUNT_OPEN:
    case CURRENT_SHUNT_BYPASSED:
    case CURRENT_POLARITY_REVERSED:
    case CURRENT_DATA_STALE:
    case CURRENT_INA_MISSING:
    case CURRENT_SCALE_INVALID:
        return "PROBABLE";
    case RELAY_DIAG_INCONCLUSIVE:
    default:
        return "INCONCLUSO";
    }
}
