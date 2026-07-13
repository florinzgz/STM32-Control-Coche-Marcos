/**
  ****************************************************************************
  * @file    relay_health_diag.c
  * @brief   Pure classifier for traction-relay / current-sense health.
  *
  * Implements the audit's decision rules (Problem 3):
  *   A. Wheels move + real PWM>0 + CH0–CH3 = 0 A  → CURRENT_SENSE_INVALID
  *      (NOT relay open — a fully open relay cannot power turning motors).
  *   B. Real PWM>0 + relay ON + valid sensors + 0 current + no wheel moving
  *      → RELAY_OPEN_SUSPECTED.
  *   C. Only with additional evidence (no post-relay voltage, no rising
  *      battery consumption) → RELAY_OPEN_CONFIRMED.
  *
  * No protection is disabled here; this is instrumentation only.
  ****************************************************************************
  */

#include "relay_health_diag.h"

#include <stddef.h>

/* |x| without libm. */
static float diag_fabs(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* Treat "no measurable current" as anything below the detection threshold
 * (or a small floor when the threshold itself is not yet populated). */
static bool diag_current_effectively_zero(const RelayHealthDiag *d)
{
    float floor_a = (d->current_threshold > 0.1f) ? d->current_threshold : 0.5f;
    return (d->current_sum_abs < floor_a);
}

RelayDiagReason_t Relay_ClassifyHealth(const RelayHealthDiag *d)
{
    if (d == NULL) {
        return RELAY_DIAG_INCONCLUSIVE;
    }

    /* --- Preconditions: without a closed-relay command, completed sequence,
     * power-ready and actual demand, the relay state cannot be judged. --- */
    bool demand_present = (d->throttle_pct > 0.0f) || (d->final_pwm_pct > 0.0f);
    if (!d->relay_commanded || !d->relay_sequence_complete ||
        !d->power_ready || !demand_present) {
        return RELAY_DIAG_INCONCLUSIVE;
    }

    /* --- Current-sense validity gate (must precede any 0 A conclusion) ---
     * A missing / stale / invalid reading is NOT "0 A"; classifying those
     * as relay-open would hide the real (sensor) fault.                    */
    uint8_t expected_wheels = (uint8_t)(d->ina_expected_mask & RELAY_DIAG_WHEEL_MASK);
    if (expected_wheels == 0U) {
        /* No wheel INA enabled at all — cannot use current for diagnosis. */
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

    /* --- Readings are trustworthy from here on. --- */

    /* Healthy: measurable current confirms the relay is passing power. */
    if (!diag_current_effectively_zero(d)) {
        /* Guard against a reversed installation that still sums a large
         * |current| but with the wrong sign under forward demand.          */
        if ((d->final_pwm_pct > 0.0f) &&
            (d->current_signed_sum < 0.0f) &&
            (diag_fabs(d->current_signed_sum) >= d->current_threshold)) {
            return CURRENT_POLARITY_REVERSED;
        }
        return RELAY_HEALTH_OK;
    }

    /* --- No measurable current with valid sensors. --- */

    /* Rule A: motion confirmed under real PWM ⇒ the relay is clearly
     * passing power; the current measurement is what is wrong.            */
    if ((d->final_pwm_pct > 0.0f) && d->any_wheel_moving) {
        return CURRENT_SENSE_INVALID;
    }

    /* Rule B/C: real PWM, relay commanded, valid sensors, no current AND
     * no wheel motion ⇒ the relay may genuinely be open.                  */
    if ((d->final_pwm_pct > 0.0f) && !d->any_wheel_moving) {
        /* Rule C: confirm only with independent evidence that no power is
         * reaching the motors (no voltage after the relay AND the battery
         * shows no extra consumption).                                     */
        if (!d->post_relay_voltage_present && !d->battery_consumption_rising) {
            return RELAY_OPEN_CONFIRMED;
        }
        return RELAY_OPEN_SUSPECTED;
    }

    /* PWM not actually applied yet (e.g. demand present but limited to 0) —
     * cannot conclude anything about the relay.                            */
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
