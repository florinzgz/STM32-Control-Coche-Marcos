/**
  ****************************************************************************
  * @file    rc_arbiter.h
  * @brief   RC override arbiter (Modo Control Remoto Clásico con failsafe)
  *
  *          Multiplexor between local sources (pedal + local steering) and
  *          remote-control demands carried on CAN_ID_CMD_RC_OVERRIDE (0x10A).
  *
  *          Decision rule (single point of truth):
  *
  *              if  (now - last_rx_ms) < RC_OVERRIDE_TIMEOUT_MS
  *              AND last_frame.override_flag == 1
  *                  → use RC values
  *              else
  *                  → use local values
  *
  *          The arbiter NEVER calls Traction_SetDemand() or Steering_SetAngle()
  *          itself.  It only returns the arbitrated value; the main scheduler
  *          and the CAN handler keep the existing safety pipeline
  *          (Safety_Validate*, Startup_IsInhibited, LIMP_HOME) DOWNSTREAM.
  *
  *          Failsafe is automatic:
  *           - RC turned off / out of RF range / battery dead   → ESP32 stops
  *                emitting 0x10A → arbiter timeout in ≤ 200 ms → local control
  *           - ESP32 dies / CAN bus loss                        → no 0x10A
  *                ever arrives → local control (default)
  *           - CH10 switch released                             → ESP32 sends
  *                override_flag=0 → arbiter returns local immediately
  ****************************************************************************
  */

#ifndef RC_ARBITER_H
#define RC_ARBITER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Strict failsafe window.  If no valid 0x10A is parsed within this many
 * milliseconds, the arbiter falls back to local sources unconditionally.
 * 200 ms = 4× nominal 50 ms cadence → tolerates a 3-frame burst loss
 * before reverting to local control.                                     */
#define RC_OVERRIDE_TIMEOUT_MS  200U

/* DLC of CAN_ID_CMD_RC_OVERRIDE (0x10A) frame.
 *   byte0: flags          bit0 = override_active (CH10 = REMOTE)
 *                         bits 1..7 reserved (must be 0)
 *   byte1: throttle_pct   0..100  (>100 → frame rejected)
 *   byte2: steer_lo       int16 LE, signed 1/10 degree
 *   byte3: steer_hi
 *   byte4: seq            rolling counter (informational; freeze tolerated)
 */
#define RC_OVERRIDE_DLC          5U

/* Initialize arbiter state.  Safe to call multiple times.                */
void RcArbiter_Init(void);

/* Called by can_handler.c upon receipt of CAN_ID_CMD_RC_OVERRIDE.
 * Validates length and value ranges; on success stores demands and
 * refreshes the watchdog timestamp.  Rejects malformed frames silently
 * (does NOT refresh timestamp), so corrupt traffic cannot keep the
 * arbiter active.
 *
 * payload : raw CAN data (≥ RC_OVERRIDE_DLC bytes)
 * len     : DLC of the received frame
 * now_ms  : HAL_GetTick() at receipt time                                */
void RcArbiter_OnFrame(const uint8_t* payload, uint8_t len, uint32_t now_ms);

/* True iff the last validated frame is recent (< RC_OVERRIDE_TIMEOUT_MS)
 * AND its override_flag is set.  This is the gate consumed by main.c
 * and the 0x101 steering handler to decide whose value to apply.        */
bool RcArbiter_IsActive(uint32_t now_ms);

/* Returns the throttle percentage (0..100) the traction pipeline should
 * use this 50 ms tick.  Returns local_pct when the arbiter is inactive,
 * otherwise returns the last validated RC throttle.                     */
float RcArbiter_GetThrottle(float local_pct, uint32_t now_ms);

/* Returns the steering angle (deg, signed) the steering controller
 * should use.  Returns local_deg when inactive, otherwise the last
 * validated RC angle.                                                    */
float RcArbiter_GetSteering(float local_deg, uint32_t now_ms);

/* Diagnostics — read-only snapshot for logs / unit tests.                */
typedef struct {
    uint32_t last_rx_ms;        /* HAL_GetTick() of last valid frame      */
    uint32_t accepted_frames;   /* Frames that passed range validation    */
    uint32_t rejected_frames;   /* DLC short or value out of range        */
    float    last_throttle_pct; /* Last accepted throttle (0..100)        */
    float    last_steer_deg;    /* Last accepted steering (deg, signed)   */
    uint8_t  last_flags;        /* Last accepted flags byte               */
    uint8_t  last_seq;          /* Last accepted sequence counter         */
} RcArbiterStats_t;

void RcArbiter_GetStats(RcArbiterStats_t* out);

#ifdef __cplusplus
}
#endif

#endif /* RC_ARBITER_H */
