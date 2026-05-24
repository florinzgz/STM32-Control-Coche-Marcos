/**
  ****************************************************************************
  * @file    rc_arbiter.c
  * @brief   RC override arbiter implementation — see rc_arbiter.h for the
  *          decision rule and failsafe contract.
  *
  *          Threading model: the arbiter state is read/written from two
  *          contexts — the 50 ms main scheduler (readers) and
  *          CAN_ProcessMessages() (writer).  Both run on the same
  *          cooperative main loop; there is no preemption.  Therefore no
  *          locking is needed.  All accesses go through this TU so a
  *          future move to interrupt context would only require adding
  *          `volatile` qualifiers here.
  ****************************************************************************
  */

#include "rc_arbiter.h"

#include <string.h>

/* ---- Internal state ---------------------------------------------------- */

static struct {
    uint32_t last_rx_ms;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    float    throttle_pct;   /* 0..100 */
    float    steer_deg;      /* signed degrees */
    uint8_t  flags;          /* bit0 = override_active */
    uint8_t  seq;
    bool     initialised;
} s_rc;

/* ---- Helpers ----------------------------------------------------------- */

static inline bool rc_is_fresh(uint32_t now_ms)
{
    /* (uint32_t)(now - last) handles wrap correctly because both are
     * uint32_t milliseconds from HAL_GetTick().                          */
    return (s_rc.accepted_frames != 0U) &&
           ((uint32_t)(now_ms - s_rc.last_rx_ms) < RC_OVERRIDE_TIMEOUT_MS);
}

/* ---- Public API -------------------------------------------------------- */

void RcArbiter_Init(void)
{
    memset(&s_rc, 0, sizeof(s_rc));
    s_rc.initialised = true;
}

void RcArbiter_OnFrame(const uint8_t* payload, uint8_t len, uint32_t now_ms)
{
    if (payload == NULL || len < RC_OVERRIDE_DLC) {
        s_rc.rejected_frames++;
        return;
    }

    /* byte0: flags — only bit0 carries meaning; reserved bits ignored
     * intentionally (forward-compat).  We DO NOT reject on unknown bits
     * so future flag additions remain backward-compatible.              */
    uint8_t flags = payload[0];

    /* byte1: throttle 0..100, reject out of range to prevent injection. */
    uint8_t thr = payload[1];
    if (thr > 100U) {
        s_rc.rejected_frames++;
        return;
    }

    /* byte2..3: int16 LE steering in 1/10 degree, clamped to ±30°.
     * Out-of-range frames are rejected (not clamped) to surface a
     * misconfigured ESP32.                                              */
    int16_t deg10 = (int16_t)((uint16_t)payload[2] |
                              ((uint16_t)payload[3] << 8));
    if (deg10 < -300 || deg10 > 300) {
        s_rc.rejected_frames++;
        return;
    }

    /* All validations passed → commit. */
    s_rc.flags         = flags;
    s_rc.throttle_pct  = (float)thr;
    s_rc.steer_deg     = (float)deg10 / 10.0f;
    s_rc.seq           = payload[4];
    s_rc.last_rx_ms    = now_ms;
    s_rc.accepted_frames++;
}

bool RcArbiter_IsActive(uint32_t now_ms)
{
    if (!rc_is_fresh(now_ms)) {
        return false;
    }
    return (s_rc.flags & 0x01U) != 0U;
}

float RcArbiter_GetThrottle(float local_pct, uint32_t now_ms)
{
    if (RcArbiter_IsActive(now_ms)) {
        return s_rc.throttle_pct;
    }
    return local_pct;
}

float RcArbiter_GetSteering(float local_deg, uint32_t now_ms)
{
    if (RcArbiter_IsActive(now_ms)) {
        return s_rc.steer_deg;
    }
    return local_deg;
}

void RcArbiter_GetStats(RcArbiterStats_t* out)
{
    if (out == NULL) return;
    out->last_rx_ms        = s_rc.last_rx_ms;
    out->accepted_frames   = s_rc.accepted_frames;
    out->rejected_frames   = s_rc.rejected_frames;
    out->last_throttle_pct = s_rc.throttle_pct;
    out->last_steer_deg    = s_rc.steer_deg;
    out->last_flags        = s_rc.flags;
    out->last_seq          = s_rc.seq;
}
