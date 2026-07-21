/**
  ****************************************************************************
  * @file    steering_supervisor.h
  * @brief   EPS assist supervisor — connects the REAL steering detectors to
  *          the EPS isolation policy (steering_eps.c).
  *
  * The steering is MECHANICAL; the motor only ASSISTS.  Any isolable failure
  * of the assist must disconnect the motor and leave purely mechanical
  * steering (EPS_STATE_MECHANICAL_ONLY) WITHOUT touching traction, gear, the
  * traction relays or the pedal.  Only a proven, non-isolable electrical
  * danger escalates to ELECTRICAL_HAZARD and hands SAFE/ERROR to the safety
  * subsystem.
  *
  * This module owns the decision logic for the audit items:
  *   1. INA226 CH5 current-sensor diagnosis (MISSING / STALE / CONFIG /
  *      POLARITY REVERSED / MUX FAILURE) → Steering_DisableAssistFault().
  *   2. The real overcurrent state machine (isolate → non-blocking wait for a
  *      fresh CH5 sample → keep MECHANICAL_ONLY if the current fell, or
  *      declare ELECTRICAL_HAZARD + request SAFE/ERROR if it persists).
  *   3. The encoder-Z policy (optional → diagnostic only; mandatory → an
  *      absent/incoherent Z isolates the assist).
  *   4. Fresh device (no params) → authorised defaults, versus existing flash
  *      with corrupt CRC/structure → EPS_FAULT_PARAMETERS_INVALID.
  *   5. Persistent invalid calibration that centering could not recover →
  *      EPS_FAULT_CALIBRATION_INVALID.
  *
  * The decision helpers are PURE (no HAL, no globals) so they are fully
  * host-testable, and SteeringSupervisor_Apply() drives the real, idempotent
  * steering_eps.c isolation API.  The production data-gathering glue lives in
  * steering_supervisor_io.c.
  ****************************************************************************
  */

#ifndef STEERING_SUPERVISOR_H
#define STEERING_SUPERVISOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "steering_eps.h"
#include "ina226_channel_diag.h"

/* ---- Overcurrent tuning ---------------------------------------------- *
 * The steering assist motor is monitored through INA226 CH5 (1.5 mΩ shunt).
 *
 * TWO physically distinct thresholds are required (audit section 6):
 *
 *  - STEERING_ACTIVE_OVERCURRENT_MA: the ceiling for the motor while it is
 *    genuinely working.  Mirrors the documented 25 A per-motor limit
 *    (safety_system.c MAX_CURRENT_A) expressed in mA.  Crossing it while a
 *    valid CH5 sample is present starts the isolation sequence.
 *
 *  - STEERING_ISOLATION_CURRENT_MAX_MA: the MAXIMUM current that may still
 *    flow through CH5 AFTER the PC12 relay has been commanded open and still
 *    be considered "isolated / harmless".  It must budget the INA226 offset,
 *    normal residual/quiescent draw, the pre-relay electronics (incl. the
 *    BTS7960 if it sits before the relay), shunt tolerance and noise.  There
 *    is NO physical measurement of this residual in the repository yet, so the
 *    value below is a PLACEHOLDER and MUST NOT be treated as calibrated.
 *
 *    STEERING_ISOLATION_THRESHOLD_CALIBRATED gates its use:
 *      0 (default, UNCALIBRATED): a post-isolation current above the residual
 *        placeholder NEVER escalates to ELECTRICAL_HAZARD / SAFE.  The assist
 *        stays MECHANICAL_ONLY and the diagnostic reports THRESHOLD
 *        UNCALIBRATED.  (Calibration procedure: see docs/EPS_MECHANICAL_ONLY_POLICY.md.)
 *      1 (CALIBRATED): a fresh valid post-isolation sample still above
 *        STEERING_ISOLATION_CURRENT_MAX_MA proves a non-isolable hazard and
 *        the FSM may escalate to ELECTRICAL_HAZARD / SAFE.
 */
#ifndef STEERING_OC_LIMIT_MA
#define STEERING_OC_LIMIT_MA          25000
#endif

#ifndef STEERING_ACTIVE_OVERCURRENT_MA
#define STEERING_ACTIVE_OVERCURRENT_MA   STEERING_OC_LIMIT_MA
#endif

#ifndef STEERING_ISOLATION_CURRENT_MAX_MA
/* PLACEHOLDER residual ceiling — NOT physically calibrated.  Only consulted
 * once STEERING_ISOLATION_THRESHOLD_CALIBRATED becomes 1.                    */
#define STEERING_ISOLATION_CURRENT_MAX_MA   1000
#endif

#ifndef STEERING_ISOLATION_THRESHOLD_CALIBRATED
#define STEERING_ISOLATION_THRESHOLD_CALIBRATED   0
#endif

/* How long the FSM waits (non-blocking) for a genuinely NEW valid CH5 sample
 * to arrive after isolating the motor.  If no fresh valid sample confirms the
 * outcome within this window the danger is NOT assumed: the assist stays
 * isolated (MECHANICAL_ONLY) and the isolation is reported as unconfirmed
 * (ISOLATED_UNCONFIRMED).  A confirm timeout NEVER auto-escalates to
 * ELECTRICAL_HAZARD / SAFE — missing information is not proof of a hazard.   */
#ifndef STEERING_OC_CONFIRM_MS
#define STEERING_OC_CONFIRM_MS        200U
#endif

/* A CH5 isolable fault must persist this many REAL CH5 acquisitions before it
 * isolates the assist, so a single transient I2C hiccup never trips it.  The
 * debounce advances per real acquisition (probe_sequence), NOT per 100 Hz
 * supervisor cycle, so one failed 20 Hz probe re-read across five 100 Hz
 * cycles counts as ONE failure, never five.                                  */
#ifndef STEERING_CH5_FAULT_DEBOUNCE
#define STEERING_CH5_FAULT_DEBOUNCE   3U
#endif

/* Default encoder-Z policy: Z is OPTIONAL (a missing/incoherent Z is a
 * diagnostic only, never an assist isolation).  Define STEERING_Z_REQUIRED=1
 * at build time to make Z mandatory for the assist.                         */
#ifndef STEERING_Z_REQUIRED
#define STEERING_Z_REQUIRED           0
#endif

/* ======================================================================
 *  Pure decision helpers (no state, no HAL) — host-testable
 * ====================================================================== */

/**
 * @brief  Map an INA226 CH5 diagnosis to the EPS isolation cause.
 *
 *         MISSING / MUX FAILURE / read-fail (no usable sensor)  → CH5_MISSING
 *         STALE                                                 → CH5_STALE
 *         CONFIG lost / wrong id (invalid configuration)        → CH5_CONFIG
 *         POLARITY REVERSED                                     → DIRECTION_POLARITY
 *         Everything benign (OK / present-no-shunt)             → EPS_FAULT_NONE
 *
 * @param  reason  Classifier output (Ina226DiagReason_t).
 * @retval EPS isolation cause, or EPS_FAULT_NONE when no isolation is due.
 */
EpsFaultReason_t SteeringSupervisor_Ch5ToEpsFault(Ina226DiagReason_t reason);

/**
 * @brief  EPS parameter-store policy (audit item 4).
 *
 *         fresh device (no persisted slot) → EPS_FAULT_NONE (defaults are
 *         authorised); an EXISTING flash slot that is present but structurally
 *         corrupt / bad CRC → EPS_FAULT_PARAMETERS_INVALID (never silently run
 *         on defaults over real corruption).
 *
 * @param  flash_present  a persisted parameter slot exists in flash.
 * @param  flash_valid    the persisted slot is structurally valid (CRC ok).
 */
EpsFaultReason_t SteeringSupervisor_ParamsPolicy(bool flash_present,
                                                 bool flash_valid);

/**
 * @brief  Steering calibration policy (audit item 5).
 *
 *         An EXISTING calibration slot that is corrupt, OR a calibration that
 *         is invalid AND that the centering FSM has finished without being
 *         able to recover a valid centre → EPS_FAULT_CALIBRATION_INVALID.
 *         A fresh device whose centering has not finished yet is not a fault.
 *
 * @param  flash_present       a calibration slot exists in flash.
 * @param  flash_corrupt       the calibration slot is present but corrupt.
 * @param  cal_valid           a valid calibration is currently available.
 * @param  centering_finished  the homing FSM has reached DONE or FAULT.
 * @param  centering_recovered the homing FSM produced a valid centre.
 */
EpsFaultReason_t SteeringSupervisor_CalPolicy(bool flash_present,
                                              bool flash_corrupt,
                                              bool cal_valid,
                                              bool centering_finished,
                                              bool centering_recovered);

/**
 * @brief  Encoder-Z policy (audit item 3).
 *
 *         When Z is optional → EPS_FAULT_NONE (diagnostic only).  When Z is
 *         mandatory → an absent (NOT_SEEN) or incoherent (OUT_OF_WINDOW /
 *         MECH_OFFSET) Z once the centre is confirmed isolates the assist with
 *         EPS_FAULT_ENCODER_Z.
 *
 * @param  z_required     policy: Z is mandatory for the assist.
 * @param  center_known   PB5 centre has been confirmed (Z only matters then).
 * @param  z_status       SteeringZStatus_t value (see steering_z.h).
 */
EpsFaultReason_t SteeringSupervisor_ZPolicy(bool z_required,
                                            bool center_known,
                                            int z_status);

/* ======================================================================
 *  Overcurrent state machine (audit item 2) — pure, host-testable
 * ====================================================================== */

typedef enum {
    OC_STATE_NORMAL = 0,          /* No overcurrent seen                      */
    OC_STATE_CONFIRM_WAIT,        /* Isolated, waiting for a fresh CH5 sample  */
    OC_STATE_ISOLATED_UNCONFIRMED,/* Isolated, no fresh sample yet (no SAFE)   */
    OC_STATE_MECHANICAL_CONFIRMED,/* Terminal: current fell, isolable, done    */
    OC_STATE_HAZARD               /* Terminal: current persisted → hazard      */
} OcState_t;

typedef enum {
    OC_ACTION_NONE = 0,            /* Nothing to do this cycle                 */
    OC_ACTION_ISOLATE,             /* Isolate the motor (DisableAssistFault OC) */
    OC_ACTION_KEEP_MECHANICAL,     /* Current fell — stay MECHANICAL_ONLY      */
    OC_ACTION_ISOLATION_UNCONFIRMED,/* No fresh sample — stay isolated, no SAFE */
    OC_ACTION_ESCALATE_HAZARD      /* Persisted — DeclareElectricalHazard+SAFE */
} OcAction_t;

typedef struct {
    OcState_t state;
    int32_t   active_limit_ma;    /* Working-motor overcurrent ceiling (mA)    */
    int32_t   isolation_limit_ma; /* Post-isolation residual ceiling (mA)      */
    bool      isolation_calibrated;/* Residual ceiling is physically calibrated */
    uint32_t  confirm_ms;         /* Non-blocking confirm window (ms)          */
    uint32_t  isolate_sample_id;  /* CH5 sample id captured at isolation       */
    uint32_t  isolate_tick_ms;    /* Tick at isolation (for the timeout)       */
} OcFsm_t;

/** @brief  Reset the overcurrent FSM.
 *  @param  f                    FSM state.
 *  @param  active_limit_ma      working-motor overcurrent ceiling (mA).
 *  @param  isolation_limit_ma   post-isolation residual ceiling (mA).
 *  @param  isolation_calibrated residual ceiling is physically calibrated.
 *  @param  confirm_ms           non-blocking confirm window (ms).            */
void SteeringSupervisor_OcInit(OcFsm_t *f, int32_t active_limit_ma,
                               int32_t isolation_limit_ma,
                               bool isolation_calibrated, uint32_t confirm_ms);

/**
 * @brief  Advance the overcurrent FSM one cycle (non-blocking).
 *
 * @param  f            FSM state.
 * @param  current_ma   Signed CH5 current this cycle (mA).
 * @param  sample_id    Monotonic id that CHANGES on every new valid CH5
 *                      sample (production passes the real acquisition
 *                      sequence Ina226ChannelDiag::sample_sequence).
 * @param  sample_valid the CH5 read this cycle is valid.
 * @param  now_ms       Current tick (HAL_GetTick equivalent).
 * @retval The action the caller must perform on the real hardware.
 */
OcAction_t SteeringSupervisor_OcStep(OcFsm_t *f, int32_t current_ma,
                                     uint32_t sample_id, bool sample_valid,
                                     uint32_t now_ms);

/* ======================================================================
 *  Stateful supervisor — drives the real EPS isolation API
 * ====================================================================== */

/* Snapshot of every real detector the supervisor consults in one cycle. */
typedef struct {
    /* INA226 CH5 (steering current sensor). */
    Ina226DiagReason_t ch5_reason;   /* Sensor_GetChannel5Diag()->fault_reason */
    int32_t            ch5_current_ma;/* signed_current_ma                     */
    uint32_t           ch5_sample_id; /* real acquisition sequence (changes on new)*/
    bool               ch5_sample_valid;/* shunt+bus+ack read ok this sample    */
    uint32_t           ch5_probe_id;  /* real acquisition ATTEMPT sequence:
                                       * ++ once per Sensor_UpdateChannel5Diag
                                       * (valid OR invalid).  The CH5 isolable-
                                       * fault debounce advances only when this
                                       * changes, so one failed 20 Hz probe seen
                                       * across five 100 Hz cycles counts ONCE. */

    /* EPS parameter store. */
    bool params_flash_present;
    bool params_flash_valid;

    /* Steering calibration store. */
    bool cal_flash_present;
    bool cal_flash_corrupt;
    bool cal_valid;
    bool centering_finished;
    bool centering_recovered;

    /* Encoder Z. */
    bool z_required;
    bool z_center_known;
    int  z_status;                   /* SteeringZStatus_t                     */

    uint32_t now_ms;                 /* HAL_GetTick() this cycle              */
} SteeringSupervisorInputs;

/** @brief  Initialise the supervisor (FSM + latches).  Call once at boot. */
void SteeringSupervisor_Init(void);

/**
 * @brief  Evaluate every real detector and drive the EPS isolation API.
 *
 *         Calls the idempotent Steering_DisableAssistFault() for any isolable
 *         cause and, on a proven persistent overcurrent, calls
 *         Steering_DeclareElectricalHazard() and latches an internal
 *         SAFE/ERROR-escalation request (see SteeringSupervisor_WantsSafe()).
 *         Never touches traction, gear, the traction relays or the pedal.
 */
void SteeringSupervisor_Apply(const SteeringSupervisorInputs *in);

/** @brief  true once a proven electrical hazard requires the safety subsystem
 *          to escalate to SAFE/ERROR.  Latched.                             */
bool SteeringSupervisor_WantsSafe(void);

/** @brief  The last isolation cause the supervisor asserted (diagnostics). */
EpsFaultReason_t SteeringSupervisor_LastCause(void);

/** @brief  Current overcurrent FSM state (diagnostics/tests). */
OcState_t SteeringSupervisor_OcState(void);

/** @brief true only while the OC FSM needs a fresh post-isolation CH5
 *         sample.  Unrelated EPS latches must not start an OC sequence. */
bool SteeringSupervisor_NeedsPostIsolationSample(void);

/* ======================================================================
 *  Production entry point (steering_supervisor_io.c)
 * ====================================================================== */

/**
 * @brief  Gather the real steering detectors and run one supervisor cycle.
 *         Called from the 100 Hz main task after the steering motor writer.
 *         Defined in steering_supervisor_io.c (depends on HAL/globals).
 */
void SteeringSupervisor_Service(void);

#ifdef __cplusplus
}
#endif

#endif /* STEERING_SUPERVISOR_H */
