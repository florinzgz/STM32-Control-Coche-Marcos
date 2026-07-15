/**
  ****************************************************************************
  * @file    ina226_channel_diag.c
  * @brief   Pure per-channel INA226 diagnostic classifier.
  *
  * Turns a per-channel I2C probe snapshot into one explicit reason, in the
  * same order a real probe discovers faults:
  *   mux select → I2C ACK → identity (MFG/DIE) → config → register reads →
  *   freshness → demand-qualified shunt/polarity checks → OK.
  *
  * A healthy, powered but idle steering channel is expected to read 0 A and
  * approximately 0 µV.  Shunt and polarity conclusions are therefore allowed
  * only when the final steering PWM proves that current is genuinely expected.
  ****************************************************************************
  */

#include "ina226_channel_diag.h"

#include <stddef.h>

static int32_t diag_abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

Ina226DiagReason_t Ina226_ClassifyChannel(const Ina226ChannelDiag *d)
{
    if (d == NULL) {
        return INA226_CH_UNKNOWN;
    }

    /* 1. The multiplexer must select the channel first. If it cannot, we
     *    cannot even reach the INA — this is a MUX/bus problem, not a dead
     *    sensor. */
    if (!d->mux_select_ok) {
        return INA226_CH_MUX_SELECT_FAIL;
    }

    /* 2. No ACK at the expected address ⇒ genuinely missing. */
    if (!d->i2c_ack) {
        return INA226_CH_MISSING;
    }

    /* 3. ACKs but is not an INA226 (or identity reads are corrupt). */
    if (!d->manufacturer_id_ok || !d->die_id_ok) {
        return INA226_CH_WRONG_ID;
    }

    /* 4. Identity OK but configuration did not stick. */
    if (!d->config_write_ok || !d->config_readback_ok) {
        return INA226_CH_CONFIG_LOST;
    }

    /* 5. A measurement register read failed this cycle. */
    if (!d->shunt_read_ok || !d->bus_read_ok) {
        return INA226_CH_READ_FAIL;
    }

    /* 6. The chip is present and readable, but the sample is old ⇒ STALE,
     *    never a trustworthy 0 A. */
    if (d->sample_age_ms >= INA226_DIAG_STALE_MS) {
        return INA226_CH_STALE;
    }

    /* 7. Polarity and shunt-continuity diagnoses require real output demand.
     *    At idle, 0 A / 0 µV are correct and must remain INA226_CH_OK. */
    if (d->current_expected &&
        d->signed_current_ma <= -INA226_DIAG_REVERSED_MA) {
        return INA226_CH_POLARITY_REVERSED;
    }

    if (d->current_expected &&
        diag_abs32(d->shunt_uv) < INA226_DIAG_SHUNT_FLOOR_UV) {
        return INA226_CH_PRESENT_NO_SHUNT;
    }

    /* 8. Everything observable checks out. */
    return INA226_CH_OK;
}

const char *Ina226_DiagReasonStr(Ina226DiagReason_t reason)
{
    switch (reason) {
    case INA226_CH_OK:                return "OK";
    case INA226_CH_PRESENT_NO_SHUNT:  return "PRESENT NO SHUNT";
    case INA226_CH_POLARITY_REVERSED: return "POLARITY REVERSED";
    case INA226_CH_STALE:             return "STALE";
    case INA226_CH_MUX_SELECT_FAIL:   return "MUX SELECT FAIL";
    case INA226_CH_MISSING:           return "MISSING";
    case INA226_CH_WRONG_ID:          return "WRONG ID";
    case INA226_CH_CONFIG_LOST:       return "CONFIG LOST";
    case INA226_CH_READ_FAIL:         return "READ FAIL";
    case INA226_CH_UNKNOWN:
    default:                          return "UNKNOWN";
    }
}

const char *Ina226_DiagStatusWord(Ina226DiagReason_t reason)
{
    switch (reason) {
    case INA226_CH_OK:
        return "OK";
    case INA226_CH_PRESENT_NO_SHUNT:
    case INA226_CH_POLARITY_REVERSED:
        return "PRESENT";
    case INA226_CH_STALE:
        return "STALE";
    case INA226_CH_MUX_SELECT_FAIL:
    case INA226_CH_MISSING:
    case INA226_CH_WRONG_ID:
    case INA226_CH_CONFIG_LOST:
    case INA226_CH_READ_FAIL:
        return "MISSING";
    case INA226_CH_UNKNOWN:
    default:
        return "n/d";
    }
}
