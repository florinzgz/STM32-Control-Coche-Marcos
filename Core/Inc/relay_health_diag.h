/**
  ****************************************************************************
  * @file    relay_health_diag.h
  * @brief   Explicit traction-relay health classifier.
  *
  * Problem 3 of the audit: pressing the pedal makes the traction motors
  * turn, yet the HMI shows "RELAY OPEN" while INA226 CH0–CH3 read 0 A.
  *
  * A fully open traction relay CANNOT power motors that are visibly turning
  * under PWM.  So when movement is confirmed, the fault must NOT be reported
  * as RELAY OPEN — it is a CURRENT-SENSE problem (shunt/R002/VIN wiring).
  *
  * This module turns a full telemetry snapshot (RelayHealthDiag) into one
  * explicit, evidence-graded reason (RelayDiagReason_t) using the audit's
  * rules A/B/C.  It is a PURE function: no hardware access, no side effects,
  * fully host-testable.  It does NOT disable any protection — it only
  * classifies the cause so the HMI can show real numbers and require a
  * physical check before anything is changed.
  ****************************************************************************
  */

#ifndef RELAY_HEALTH_DIAG_H
#define RELAY_HEALTH_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Current telemetry older than this is treated as STALE (not "0 A valid"). */
#define RELAY_DIAG_CURRENT_STALE_MS   300U

/* Wheel current channels (INA226 CH0–CH3) occupy mask bits 0..3. */
#define RELAY_DIAG_WHEEL_MASK         0x0FU

/* ---- Evidence-graded relay / current-sense diagnosis ----
 * Values are stable (CAN-transportable); append only.                     */
typedef enum {
    RELAY_HEALTH_OK = 0,             /* Demand present, current confirms power */
    RELAY_OPEN_CONFIRMED,            /* No current, no motion + hard evidence  */
    RELAY_OPEN_SUSPECTED,            /* No current, no motion, sensors valid   */
    CURRENT_SENSE_INVALID,           /* Motors move + PWM>0 but ~0 A measured  */
    CURRENT_SHUNT_OPEN,              /* Chip present, no shunt drop            */
    CURRENT_SHUNT_BYPASSED,          /* Power path bypasses the shunt          */
    CURRENT_POLARITY_REVERSED,       /* Signed current opposite to demand      */
    CURRENT_DATA_STALE,              /* Telemetry too old to trust             */
    CURRENT_INA_MISSING,             /* Expected wheel INA not on the bus      */
    CURRENT_SCALE_INVALID,           /* Cal/scale invalid — value meaningless  */
    RELAY_DIAG_INCONCLUSIVE          /* Preconditions not met to decide        */
} RelayDiagReason_t;

/* ---- Full relay/current telemetry snapshot ---- */
typedef struct {
    /* Relay & power path */
    bool     relay_commanded;         /* Traction relay commanded ON          */
    bool     relay_sequence_complete; /* RELAY_SEQ_COMPLETE                    */
    bool     power_ready;             /* Safety_IsPowerReady()                 */

    /* Demand chain */
    float    throttle_pct;            /* Pedal_GetPercent()                    */
    float    traction_demand_pct;     /* Post-safety traction demand           */
    float    effective_demand_pct;    /* After LIMP/ABS/TCS limits             */
    float    final_pwm_pct;           /* PWM actually emitted to the BTS7960   */

    /* Motion */
    float    average_speed;           /* Mean wheel speed (km/h)               */
    float    wheel_speed[4];          /* FL, FR, RL, RR (km/h)                 */
    bool     any_wheel_moving;        /* At least one wheel above stall speed  */

    /* Current sense (INA226 CH0–CH3) */
    uint8_t  ina_ok_mask;             /* Bit i: INA i answered on I2C          */
    uint8_t  ina_expected_mask;       /* Bit i: INA i expected (Service Mode)  */
    uint8_t  ina_valid_mask;          /* Bit i: reading valid (finite/scale)   */
    uint32_t current_sample_age_ms;   /* Age of newest current sample          */
    float    current_ch[4];           /* Signed per-channel current (A)        */
    float    current_sum_abs;         /* Sum of |CH0..3| (A)                   */
    float    current_signed_sum;      /* Sum of signed CH0..3 (A)              */
    float    current_threshold;       /* Adaptive detection threshold (A)      */
    float    battery_voltage;         /* CH4 bus voltage (V)                   */

    /* Extra confirming evidence (rule C) */
    bool     post_relay_voltage_present; /* Voltage measured after the relay   */
    bool     battery_consumption_rising; /* CH4 shows load being drawn         */
    bool     scale_invalid;              /* Explicit cal/scale invalid flag    */
    bool     polarity_reversed;          /* Explicit reversed-polarity flag    */

    /* Timing / debounce */
    uint8_t  debounce_count;          /* Consecutive low-current cycles        */
    uint32_t elapsed_since_demand_ms; /* Since demand onset                    */

    RelayDiagReason_t diagnostic_reason; /* Filled by classifier               */
} RelayHealthDiag;

/**
 * @brief  Classify a relay/current snapshot into an explicit reason.
 *         Pure function; ignores @p d->diagnostic_reason (its output).
 */
RelayDiagReason_t Relay_ClassifyHealth(const RelayHealthDiag *d);

/** @brief Short ASCII label; never NULL. */
const char *Relay_DiagReasonStr(RelayDiagReason_t reason);

/** @brief Evidence grade word for the HMI: CONFIRMED / PROBABLE / INCONCLUSO. */
const char *Relay_DiagConfidenceStr(RelayDiagReason_t reason);

#ifdef __cplusplus
}
#endif

#endif /* RELAY_HEALTH_DIAG_H */
