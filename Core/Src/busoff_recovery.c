/**
  ****************************************************************************
  * @file    busoff_recovery.c
  * @brief   CAN bus-off recovery confirmation window (pure logic).
  *
  * See busoff_recovery.h for the rationale.  This translation unit holds
  * only the non-inline Update() so the module can be compiled into the
  * firmware and linked by the host unit test (test_busoff_recovery.c)
  * without any HAL dependency.
  ****************************************************************************
  */

#include "busoff_recovery.h"

bool BusOffRecoveryWindow_Update(BusOffRecoveryWindow_t *w,
                                 uint32_t now_ms,
                                 bool     heartbeat_alive,
                                 uint32_t window_ms)
{
    if (!w->probation) {
        return false;
    }

    if (heartbeat_alive) {
        /* Start a new continuous-alive streak if one is not running. */
        if (!w->streak) {
            w->streak          = true;
            w->streak_start_ms = now_ms;
        }
        /* Confirm once the streak has been sustained for the full window.
         * Unsigned modular subtraction is safe across the HAL_GetTick()
         * 49.7-day wrap. */
        if ((uint32_t)(now_ms - w->streak_start_ms) >= window_ms) {
            w->probation = false;
            w->streak    = false;
            return true;
        }
    } else {
        /* Heartbeat gap — the bus is not proven healthy.  Drop the streak
         * so the full window must be re-accumulated once heartbeats
         * return. */
        w->streak = false;
    }

    return false;
}
