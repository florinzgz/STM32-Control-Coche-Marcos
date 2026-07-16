/**
  ****************************************************************************
  * @file    ina226_channel_diag.h
  * @brief   Explicit per-channel INA226 diagnostic classifier.
  *
  * Problem 4 of the audit: the steering INA226 (CH5, MUX channel 5,
  * address 0x40) appears dead — "INA OK 0x1F / EXPECTED 0x3F / fail:2 rec:0"
  * on the STM32 and "CH5 ST: n/d" on the ESP32.
  *
  * The audit demands separating TWO different problems that both look like a
  * "dead sensor":
  *   A. "n/d" — the CAN contract simply does not carry CH5 current, so the
  *      ESP32 has no telemetry (a transport gap, not a dead chip).
  *   B. bit-5 genuinely missing from INA OK — the STM32 expects CH5 but the
  *      I2C read does not ACK.
  *
  * This module classifies a full per-channel probe snapshot
  * (Ina226ChannelDiag) into one explicit reason (Ina226DiagReason_t), so the
  * HMI can distinguish MISSING / PRESENT-NO-SHUNT / POLARITY / STALE / OK
  * and show the correct physical check.  Pure function, host-testable.
  *
  * Datasheet-verified identity registers (TI INA226, SLYS…):
  *   Manufacturer ID (0xFE) = 0x5449 ("TI")
  *   Die ID          (0xFF) = 0x2260
  ****************************************************************************
  */

#ifndef INA226_CHANNEL_DIAG_H
#define INA226_CHANNEL_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Datasheet-verified INA226 identity constants. */
#define INA226_MANUFACTURER_ID   0x5449U   /* register 0xFE */
#define INA226_DIE_ID            0x2260U   /* register 0xFF */

/* Telemetry older than this is STALE (never reported as a valid 0 A). */
#define INA226_DIAG_STALE_MS     500U

/* A shunt drop below this (in µV) while current is genuinely expected means
 * "no shunt voltage" — the classic external-shunt / R002 wiring fault.     */
#define INA226_DIAG_SHUNT_FLOOR_UV   50   /* ~50 µV ≈ noise floor */

/* A signed current more negative than this (mA) while current is genuinely
 * expected indicates reversed VIN+/VIN− wiring.                            */
#define INA226_DIAG_REVERSED_MA      100

/* ---- Explicit per-channel diagnosis (stable, CAN-transportable) ---- */
typedef enum {
    INA226_CH_OK = 0,                 /* Present, valid, no proven fault     */
    INA226_CH_PRESENT_NO_SHUNT,       /* Present but ~0 shunt drop           */
    INA226_CH_POLARITY_REVERSED,      /* Present, negative current (VIN swap) */
    INA226_CH_STALE,                  /* Telemetry not updated               */
    INA226_CH_MUX_SELECT_FAIL,        /* TCA9548A did not select the channel  */
    INA226_CH_MISSING,                /* No I2C ACK at the expected address   */
    INA226_CH_WRONG_ID,               /* ACKs but MFG/DIE id mismatch         */
    INA226_CH_CONFIG_LOST,            /* Config write/readback mismatch       */
    INA226_CH_READ_FAIL,              /* Shunt/bus register read failed       */
    INA226_CH_UNKNOWN                 /* Not classifiable                     */
} Ina226DiagReason_t;

/* ---- Full per-channel probe snapshot ---- */
typedef struct {
    uint8_t  channel;                 /* Logical INA index (0..5)            */
    uint8_t  mux_channel;             /* TCA9548A channel index              */
    uint8_t  expected_address;        /* 0x40                                */
    uint8_t  detected_address;        /* Address that ACKed (0 if none)      */

    bool     mux_select_ok;           /* TCA9548A channel select ACKed       */
    bool     i2c_ack;                 /* INA226 ACKed at expected address    */
    bool     manufacturer_id_ok;      /* 0xFE == 0x5449                      */
    bool     die_id_ok;               /* 0xFF == 0x2260                      */
    bool     config_write_ok;         /* Config register write ACKed         */
    bool     config_readback_ok;      /* Config register read back matches   */
    bool     shunt_read_ok;           /* Shunt register read ok              */
    bool     bus_read_ok;             /* Bus register read ok                */

    int16_t  raw_shunt;               /* Raw shunt register                  */
    int32_t  shunt_uv;                /* Shunt drop (µV)                     */
    int32_t  bus_mv;                  /* Bus voltage (mV)                    */
    int32_t  signed_current_ma;       /* Signed current (mA)                 */

    uint32_t sample_age_ms;           /* Age of newest sample                */

    /* Real acquisition identity (NOT derived from the reader's current tick).
     * sample_sequence increments EXACTLY ONCE per new, valid acquisition of
     * this channel (a full ACK + shunt + bus read).  A consumer polling faster
     * than the acquisition rate (e.g. the 100 Hz supervisor over a 20 Hz
     * INA226 read) sees the SAME value until a genuinely new sample lands, so
     * it can tell a fresh reading from a re-read of the previous one.  The
     * uint32_t wraps after 2^32 valid samples (>6 years at 20 Hz); consumers
     * MUST compare for inequality (seq != captured), never magnitude.        */
    uint32_t sample_sequence;         /* ++ once per new valid acquisition   */
    uint32_t last_valid_tick_ms;      /* Tick of the newest valid acquisition */

    bool     channel_powered;         /* Power branch expected energised     */
    bool     current_expected;        /* Real steering PWM is being emitted  */

    uint32_t consecutive_failures;    /* Per-channel consecutive fails       */
    uint32_t recovery_count;          /* Bus recoveries attributed to ch     */

    Ina226DiagReason_t fault_reason;  /* Filled by classifier                */
} Ina226ChannelDiag;

/**
 * @brief  Classify a per-channel INA226 probe snapshot.
 *         Pure function; ignores @p d->fault_reason (its output).
 */
Ina226DiagReason_t Ina226_ClassifyChannel(const Ina226ChannelDiag *d);

/** @brief Short ASCII label; never NULL. */
const char *Ina226_DiagReasonStr(Ina226DiagReason_t reason);

/** @brief One-word ESP32 "CH5 ST:" status: OK/MISSING/PRESENT/STALE/n/d. */
const char *Ina226_DiagStatusWord(Ina226DiagReason_t reason);

#ifdef __cplusplus
}
#endif

#endif /* INA226_CHANNEL_DIAG_H */
