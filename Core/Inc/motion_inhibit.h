/**
 ****************************************************************************
 * @file    motion_inhibit.h
 * @brief   MOTION_INHIBIT_REASON — pure, host-testable classifier that
 *          explains WHY the traction chain is (or is not) producing torque.
 *
 *          This is an INSTRUMENTATION-ONLY module.  It changes NO safety
 *          policy: it merely observes the values already computed by the
 *          traction/safety pipeline (system state, power-ready gate, gear,
 *          obstacle block, pedal demand, post-scaling effective demand and
 *          the final PWM duty actually written to the BTS7960 drivers) and
 *          folds them into a single bitfield that is emitted over CAN
 *          (0x315 DIAG_MOTION_INHIBIT) so the exact blocking stage can be
 *          captured on hardware.
 *
 *          The classic symptom this is designed to diagnose is "DEGRADED
 *          40 % with real demand/PWM at zero": the bitfield distinguishes
 *          between "the operator is not asking for torque" (NO_DEMAND),
 *          "demand was asked but zeroed before PWM" (DEMAND_ZEROED) and
 *          "demand survived but the final PWM is zero" (PWM_ZERO), plus the
 *          gating reason (state / power / gear / obstacle).
 *
 *          The module has NO firmware dependency so the logic is linked
 *          directly into a host unit test.
 ****************************************************************************
 */
#ifndef MOTION_INHIBIT_H
#define MOTION_INHIBIT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- MOTION_INHIBIT_REASON bitfield (16-bit, emitted over 0x315) ---------
 * A set bit means that stage is currently contributing to keeping the
 * vehicle from moving (or limiting it).  Several bits may be set at once;
 * consumers should treat the LOW bits (state/power/gear) as the dominant
 * root cause when present.                                                 */
#define MOTION_INHIBIT_NONE           0x0000U
#define MOTION_INHIBIT_STATE_SAFE     0x0001U /* SYS_STATE_SAFE — all torque inhibited      */
#define MOTION_INHIBIT_STATE_ERROR    0x0002U /* SYS_STATE_ERROR — all torque inhibited     */
#define MOTION_INHIBIT_POWER_NOT_READY 0x0004U /* Relay power-up sequence not yet complete   */
#define MOTION_INHIBIT_GEAR_PARK      0x0008U /* Gear P — passive brake, no drive accepted  */
#define MOTION_INHIBIT_GEAR_NEUTRAL   0x0010U /* Gear N — coast, no drive accepted          */
#define MOTION_INHIBIT_NO_DEMAND      0x0020U /* Operator demand below the motion threshold */
#define MOTION_INHIBIT_DEMAND_ZEROED  0x0040U /* Demand requested but zeroed before PWM     */
#define MOTION_INHIBIT_OBSTACLE_BLOCK 0x0080U /* Forward motion blocked by obstacle sensor  */
#define MOTION_INHIBIT_PWM_ZERO       0x0100U /* Demand survived but final PWM duty is zero */
#define MOTION_INHIBIT_TORQUE_LIMITED 0x0200U /* DEGRADED/LIMP torque cap active (moving OK)*/
#define MOTION_INHIBIT_STARTUP_INHIBIT 0x0400U /* startup_inhibit still latched (pre-arm)    */
#define MOTION_INHIBIT_PEDAL_FAULT    0x0800U /* Pedal implausible/contradictory — demand 0 */
#define MOTION_INHIBIT_SAFETY_SCALE_ZERO 0x1000U /* Per-wheel safety/ABS/TCS scale collapsed to 0 */
#define MOTION_INHIBIT_BATTERY_CUTOFF 0x2000U /* Battery under/over-voltage limit or cutoff  */
#define MOTION_INHIBIT_THERMAL_OVERCURRENT 0x4000U /* Over-temperature or over-current cutoff */
#define MOTION_INHIBIT_SERVICE_DISABLED 0x8000U /* Traction relay module disabled via service */

/* Relay-sequence phase (NOT a reason bit — carried separately in 0x315 byte 7
 * bits 2-3).  Reflects the COMMANDED GPIO/sequencer state only; the firmware
 * has NO physical relay-contact feedback.                                    */
#define MOTION_INHIBIT_RELAY_SEQ_IDLE      0U /* No power-up sequence in progress          */
#define MOTION_INHIBIT_RELAY_SEQ_IN_PROGRESS 1U /* Traction relay energised, settling      */
#define MOTION_INHIBIT_RELAY_SEQ_COMPLETE  2U /* Sequence finished, power-ready            */

/* Demand (percent of full scale) at/below which the operator is considered
 * to be requesting no motion.  Matches the powertrain-engaged threshold
 * used elsewhere in the safety system (|demand| >= 3 %).                    */
#define MOTION_INHIBIT_DEMAND_EPS_PCT  3.0f

/**
 * Snapshot of the traction pipeline for one evaluation cycle.  All fields
 * are read straight from values the pipeline already computed — this module
 * never recomputes control math.
 */
typedef struct {
    uint8_t  state;                    /* SystemState_t value (SYS_STATE_*)          */
    bool     power_ready;              /* Safety_IsPowerReady()                      */
    uint8_t  gear;                     /* current_gear (GEAR_*)                      */
    uint8_t  gear_park;                /* GEAR_PARK enum value (for comparison)      */
    uint8_t  gear_neutral;            /* GEAR_NEUTRAL enum value (for comparison)   */
    uint8_t  state_safe;              /* SYS_STATE_SAFE enum value                  */
    uint8_t  state_error;             /* SYS_STATE_ERROR enum value                 */
    bool     obstacle_forward_blocked; /* Obstacle_IsForwardBlocked()                */
    bool     forward_gear;             /* true when the selected gear drives forward */
    float    demand_pct;               /* Operator demand before scaling (signed %)  */
    float    effective_demand_pct;     /* Demand after all scaling (signed %)        */
    uint16_t final_pwm_max;            /* Max final PWM duty (ticks) across wheels   */
    uint8_t  degraded_level;           /* 0 = none; >0 = DEGRADED level (torque cap) */
    /* ---- Additional observability inputs (each a pre-computed boolean read
     * straight from the pipeline; the classifier only folds them into bits) -- */
    bool     startup_inhibit;          /* Startup_IsInhibited()                      */
    bool     pedal_fault;              /* !Pedal_IsPlausible() (implausible/contra.) */
    bool     safety_scale_zero;        /* All per-wheel safety scales collapsed to 0 */
    bool     battery_cutoff;           /* Battery UV/OV limit or cutoff active        */
    bool     thermal_overcurrent;      /* Over-temperature or over-current cutoff     */
    bool     service_disabled;         /* Traction relay module disabled via service */
} MotionInhibitInputs;

/**
 * Fold the snapshot into a MOTION_INHIBIT_REASON bitfield.
 * Pure function — no side effects, no globals.
 */
uint16_t MotionInhibit_Evaluate(const MotionInhibitInputs *in);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_INHIBIT_H */
