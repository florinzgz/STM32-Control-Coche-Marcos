/**
  ****************************************************************************
  * @file    pedal_cal_session.h
  * @brief   Explicit, safe accelerator-pedal calibration session FSM.
  *
  * Problem 5 of the audit: the previous CAPTURE MIN/MAX handling was
  * self-contradictory.  Both sub-opcodes went through a single safety gate
  * that rejected whenever `Pedal_GetPercent() >= 3 %`, which made CAPTURE
  * MAX (which REQUIRES a fully pressed pedal) impossible to ever complete.
  * The gate also depended on `Startup_IsInhibited()` (the transient 400 ms
  * startup window), coupling calibration to an unrelated boot latch.
  *
  * This module is the PURE decision core of a proper calibration session.
  * It owns an explicit finite-state machine:
  *
  *     IDLE
  *      -> ENTERING              (entry guards satisfied)
  *      -> WAIT_RELEASED         (operator must release the pedal)
  *      -> CAPTURING_MIN         (8 stable samples, pedal released)
  *      -> WAIT_FULL_PRESS       (operator arms CAPTURE MAX with the button)
  *      -> CAPTURING_MAX         (8 stable RAW samples, raw>MIN, raw-MIN>=range)
  *      -> WAIT_RELEASE_FOR_SAVE (operator must release again before saving)
  *      -> READY_TO_SAVE         (MIN<MAX and range validated)
  *      -> SAVING                (persist + apply + readback verify)
  *      -> COMPLETED
  *      -> ABORTED               (any abort/failure cause)
  *
  * Safety model (enforced for the WHOLE session, every tick):
  *   Entry is permitted ONLY when: STANDBY, gear Park or Neutral, all wheels
  *   < 0.3 km/h, pedal plausible, no critical error, and traction + power
  *   relays already inhibited.  It does NOT depend on startup_inhibit.
  *   Throughout the session the caller keeps Traction_SetDemand(0), PWM at
  *   zero and the traction enables safe; ACTIVE / movement are never allowed.
  *   The session self-aborts on SAFE, ERROR, emergency, movement, CAN loss or
  *   timeout.
  *
  * Phase capture rules (audit §5):
  *   CAPTURE MIN : pedal must be RELEASED, 8 stable samples.
  *   CAPTURE MAX : the operator ARMS the capture with the CAPTURE MAX button
  *                 (PedalCalSession_ArmCaptureMax); MAX is then locked from the
  *                 RAW ADC — 8 stable samples with raw > MIN and
  *                 (raw - MIN) >= range_min.  It does NOT use a percent threshold
  *                 (Pedal_GetPercent >= 80 %) that would depend on a possibly-
  *                 wrong old calibration.
  *   SAVE        : pedal must be RELEASED again, MIN < MAX, range >= 800,
  *                 persist, apply, then read the value back and verify.
  *
  * The module performs NO hardware access and NO delays: persistence, apply
  * and readback are injected as callbacks so the whole policy is host-testable
  * (test_pedal_cal_session.c).  The technical reason bitmask is preserved for
  * CAN telemetry; PedalCalSession_ReasonText() renders each cause as human
  * text for the HMI.
  ****************************************************************************
  */

#ifndef PEDAL_CAL_SESSION_H
#define PEDAL_CAL_SESSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---- Explicit session states (audit §5) ---------------------------------
 * Values are stable / CAN-transportable; append only.                      */
typedef enum {
    PEDAL_CAL_IDLE = 0,
    PEDAL_CAL_ENTERING,
    PEDAL_CAL_WAIT_RELEASED,
    PEDAL_CAL_CAPTURING_MIN,
    PEDAL_CAL_WAIT_FULL_PRESS,
    PEDAL_CAL_CAPTURING_MAX,
    PEDAL_CAL_WAIT_RELEASE_FOR_SAVE,
    PEDAL_CAL_READY_TO_SAVE,
    PEDAL_CAL_SAVING,
    PEDAL_CAL_COMPLETED,
    PEDAL_CAL_ABORTED,
} PedalCalState;

/* ---- Reason bitmask (technical) -----------------------------------------
 * Kept as a mask so several concurrent causes can be reported on CAN; the
 * HMI shows PedalCalSession_ReasonText() for the highest-priority bit.      */
#define PEDAL_CAL_SESS_OK                 0x0000U
/* Entry blocks */
#define PEDAL_CAL_BLOCK_NOT_STANDBY       0x0001U
#define PEDAL_CAL_BLOCK_GEAR              0x0002U  /* not Park/Neutral        */
#define PEDAL_CAL_BLOCK_WHEELS_MOVING     0x0004U
#define PEDAL_CAL_BLOCK_PEDAL_IMPLAUSIBLE 0x0008U
#define PEDAL_CAL_BLOCK_CRITICAL_ERROR    0x0010U
#define PEDAL_CAL_BLOCK_TRACTION_LIVE     0x0020U  /* relays not inhibited    */
/* Runtime aborts */
#define PEDAL_CAL_ABORT_SAFE              0x0040U
#define PEDAL_CAL_ABORT_ERROR             0x0080U
#define PEDAL_CAL_ABORT_EMERGENCY         0x0100U
#define PEDAL_CAL_ABORT_MOVEMENT          0x0200U
#define PEDAL_CAL_ABORT_CAN_LOSS          0x0400U
#define PEDAL_CAL_ABORT_TIMEOUT           0x0800U
/* Save-time failures */
#define PEDAL_CAL_FAIL_MIN_GE_MAX         0x1000U
#define PEDAL_CAL_FAIL_RANGE_SMALL        0x2000U
#define PEDAL_CAL_FAIL_UNSTABLE           0x4000U
#define PEDAL_CAL_FAIL_READBACK           0x8000U
/* Extended causes (audit P5 final).  These sit ABOVE the 16-bit CAN reason
 * mask so the 0x319 reason field (bytes 2-3) stays a stable 16-bit contract.
 * They are surfaced instead through dedicated 0x319 byte-1 flag bits so the
 * HMI can still tell an operator cancel apart from a real emergency, and a
 * lost movement lock apart from an ordinary movement abort.                 */
#define PEDAL_CAL_ABORT_OPERATOR          0x00010000U  /* operator pressed ABORT */
#define PEDAL_CAL_ABORT_LOCK_LOST         0x00020000U  /* traction lock no longer safe */

/* ---- Per-tick condition snapshot (all facts the FSM needs) ---------------
 * The caller fills this from the live safety / sensor state; the module
 * never reads hardware itself.                                             */
typedef struct {
    uint32_t now_ms;              /* monotonic time                          */
    bool     in_standby;          /* Safety_GetState() == SYS_STATE_STANDBY  */
    bool     gear_park_or_neutral;/* shifter in Park or Neutral              */
    bool     wheels_moving;       /* any wheel speed >= 0.3 km/h             */
    bool     pedal_plausible;     /* Pedal_IsPlausible()                     */
    bool     pedal_released;      /* pedal percent < released_pct            */
    bool     pedal_pressed_full;  /* DEPRECATED (audit P5): percent >= full_press_pct.
                                   * No longer used to latch CAPTURE MAX — kept only
                                   * for ABI/telemetry compatibility.  MAX is now
                                   * decided from the raw ADC, not a percent derived
                                   * from a possibly-wrong old calibration.        */
    uint16_t pedal_raw;           /* current raw ADC sample                  */
    bool     critical_error;      /* an ERROR-level fault is latched         */
    bool     safe_state;          /* system is in SAFE                       */
    bool     emergency;           /* emergency-stop asserted                 */
    bool     can_loss;            /* CAN heartbeat lost                      */
    bool     traction_inhibited;  /* traction + power relays inhibited (entry)*/
    bool     traction_locked;     /* audit P5: live confirmation that the real
                                   * movement lock is still safe THIS tick —
                                   * demand 0, PWM 0, traction enables LOW and
                                   * the traction relay OFF.  Losing it while a
                                   * session runs aborts with ABORT_LOCK_LOST. */
} PedalCalConds;

/* ---- Injected persistence / apply / readback hooks -----------------------
 * All may be NULL in host tests that don't exercise SAVE.  validate() must
 * mirror PedalCal_Validate(); persist()/apply()/readback() wrap the flash
 * store and the pedal pipeline.                                            */
typedef struct {
    bool (*validate)(uint16_t adc_min, uint16_t adc_max);
    bool (*persist)(uint16_t adc_min, uint16_t adc_max);
    void (*apply)(uint16_t adc_min, uint16_t adc_max);
    bool (*readback)(uint16_t *adc_min, uint16_t *adc_max);
} PedalCalSessionHooks;

/* ---- Tunables ----------------------------------------------------------- */
typedef struct {
    uint8_t  stable_samples;   /* consecutive in-zone samples to lock (8)    */
    uint16_t stable_tol;       /* max spread (counts) across the samples     */
    uint16_t range_min;        /* required (max-min) span (>= 800)           */
    uint32_t phase_timeout_ms; /* no operator progress within a phase        */
    uint32_t capture_timeout_ms;/* a single capture must lock within this    */
} PedalCalSessionCfg;

/* ---- Session object (opaque-ish; POD so tests can inspect it) ------------ */
typedef struct {
    PedalCalSessionCfg  cfg;
    PedalCalSessionHooks hooks;
    PedalCalState       state;
    uint32_t            reason;        /* technical bitmask (latched); 32-bit so
                                        * the extended ABORT_OPERATOR / LOCK_LOST
                                        * causes fit above the 16-bit CAN field.  */

    /* Captured endpoints (RAM only until SAVE). */
    uint16_t            adc_min;
    bool                have_min;
    uint16_t            adc_max;
    bool                have_max;
    bool                max_armed;     /* audit P5: CAPTURE MAX button explicitly
                                        * armed the pressed-pedal capture.        */

    /* Rolling capture window. */
    uint16_t            samples[16];   /* >= stable_samples                   */
    uint8_t             sample_count;

    /* Timing. */
    uint32_t            phase_start_ms;
    uint32_t            session_start_ms;
} PedalCalSession;

/* Default configuration (matches the historical 8-sample / >=800 contract). */
PedalCalSessionCfg PedalCalSession_DefaultCfg(void);

/* Initialise a session to IDLE with the given config + hooks. */
void PedalCalSession_Init(PedalCalSession *s,
                          const PedalCalSessionCfg *cfg,
                          const PedalCalSessionHooks *hooks);

/* Operator request to start a session.  Returns true if it was accepted
 * (IDLE -> ENTERING); on rejection the entry-block reason is latched and the
 * state stays IDLE.  Guards: STANDBY, Park/Neutral, wheels stopped, pedal
 * plausible, no critical error, traction inhibited.                        */
bool PedalCalSession_Begin(PedalCalSession *s, const PedalCalConds *c);

/* Periodic tick — advances waits, collects capture samples and enforces the
 * always-on abort conditions.  Returns the new state.                      */
PedalCalState PedalCalSession_Update(PedalCalSession *s, const PedalCalConds *c);

/* Operator request to commit (only valid in READY_TO_SAVE, pedal released).
 * Runs validate -> persist -> apply -> readback-verify; on success the state
 * becomes COMPLETED, otherwise ABORTED with a FAIL_* reason.               */
void PedalCalSession_RequestSave(PedalCalSession *s, const PedalCalConds *c);

/* Operator request to ARM the CAPTURE MAX capture (audit P5).  Only meaningful
 * in WAIT_FULL_PRESS: it lets the FSM proceed to CAPTURING_MAX so MAX is
 * captured from a stable RAW ADC reading (raw > MIN and raw-MIN >= range_min),
 * NOT from a percent threshold derived from a possibly-wrong old calibration.
 * No-op in any other state.                                                 */
void PedalCalSession_ArmCaptureMax(PedalCalSession *s);

/* Operator/host abort at any time. */
void PedalCalSession_Abort(PedalCalSession *s, uint32_t reason_bit);

/* ---- Accessors ---------------------------------------------------------- */
static inline PedalCalState PedalCalSession_State(const PedalCalSession *s) { return s->state; }
static inline uint32_t      PedalCalSession_Reason(const PedalCalSession *s) { return s->reason; }
static inline bool          PedalCalSession_Active(const PedalCalSession *s) {
    return s->state != PEDAL_CAL_IDLE &&
           s->state != PEDAL_CAL_COMPLETED &&
           s->state != PEDAL_CAL_ABORTED;
}

/* Human-readable label for a state (HMI). */
const char *PedalCalSession_StateText(PedalCalState st);

/* Human-readable text for the highest-priority reason bit set (HMI).
 * Returns "OK" when the mask is zero.                                       */
const char *PedalCalSession_ReasonText(uint32_t reason_mask);

#ifdef __cplusplus
}
#endif

#endif /* PEDAL_CAL_SESSION_H */
