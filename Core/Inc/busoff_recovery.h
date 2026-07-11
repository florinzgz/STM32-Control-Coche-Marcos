/**
  ****************************************************************************
  * @file    busoff_recovery.h
  * @brief   CAN bus-off recovery confirmation window (pure, host-testable).
  *
  * After the FDCAN peripheral is re-initialised following a bus-off, the
  * controller returns to the RUNNING (operational) state.  RUNNING alone
  * does NOT prove the bus is healthy: if the underlying fault persists
  * (short, stuck-dominant node, wiring), the controller will simply go
  * bus-off again.  Declaring recovery on RUNNING (and resetting the retry
  * counter) therefore produces an infinite recovery loop that never
  * escalates and prematurely clears the fault.
  *
  * This module implements a small state machine that requires valid
  * ESP32 heartbeats to be sustained *continuously* over a stable
  * confirmation window before recovery is declared confirmed.  It has no
  * HAL dependency so the window logic can be unit-tested on the host.
  ****************************************************************************
  */

#ifndef BUSOFF_RECOVERY_H
#define BUSOFF_RECOVERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  Recovery confirmation-window state.
 *
 * @note   Treat the fields as opaque; use the functions below.
 */
typedef struct {
    bool     probation;        /* true from Begin() until confirmed/reset   */
    bool     streak;           /* currently inside a continuous-alive streak */
    uint32_t streak_start_ms;  /* tick when the current alive streak began   */
} BusOffRecoveryWindow_t;

/**
 * @brief  Reset the window to the idle (no-probation) state.
 */
static inline void BusOffRecoveryWindow_Reset(BusOffRecoveryWindow_t *w)
{
    w->probation       = false;
    w->streak          = false;
    w->streak_start_ms = 0U;
}

/**
 * @brief  Begin the confirmation probation window.
 *         Call immediately after the FDCAN peripheral has been
 *         re-initialised and returned to RUNNING.
 * @param  now_ms  Current millisecond tick.
 */
static inline void BusOffRecoveryWindow_Begin(BusOffRecoveryWindow_t *w,
                                              uint32_t now_ms)
{
    w->probation       = true;
    w->streak          = false;
    w->streak_start_ms = now_ms;
}

/**
 * @brief  True while awaiting recovery confirmation.
 */
static inline bool BusOffRecoveryWindow_InProbation(const BusOffRecoveryWindow_t *w)
{
    return w->probation;
}

/**
 * @brief  Advance the window one cycle.
 *
 *         Recovery is confirmed only when heartbeats have been alive
 *         *continuously* for at least @p window_ms.  Any observation of a
 *         non-alive heartbeat resets the streak, so the full window must
 *         be re-accumulated — a single valid frame (or a flapping bus)
 *         cannot confirm recovery.
 *
 * @param  w                 Window state.
 * @param  now_ms            Current millisecond tick.
 * @param  heartbeat_alive   True if a valid heartbeat is currently present.
 * @param  window_ms         Required continuous-alive duration.
 * @retval true   exactly on the cycle recovery becomes confirmed (the
 *                window then leaves probation).
 * @retval false  otherwise (including when not in probation).
 */
bool BusOffRecoveryWindow_Update(BusOffRecoveryWindow_t *w,
                                 uint32_t now_ms,
                                 bool     heartbeat_alive,
                                 uint32_t window_ms);

#ifdef __cplusplus
}
#endif

#endif /* BUSOFF_RECOVERY_H */
