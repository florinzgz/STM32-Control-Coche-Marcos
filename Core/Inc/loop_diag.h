/**
 * @file    loop_diag.h
 * @brief   Main-loop timing diagnostic — peak-hold of 100 Hz task duration.
 *
 * Pure-additive observational telemetry.  No control-flow effect.  The
 * 10 ms task block in `main.c` records its own duration via
 * `LoopDiag_RecordTaskUs(us)` after measuring `DWT->CYCCNT` deltas;
 * `LoopDiag_GetAndResetPeak100us()` is read once per 100 ms TX cycle by
 * `CAN_SendStatusSafety` and packed into byte 5 of `STATUS_SAFETY`
 * (0x203) so the HMI can observe worst-case task runtime.
 *
 * Units intentionally scaled to 100 µs / count (cap 255 → 25.5 ms).
 * The 100 Hz task budget is 10 ms; readings approaching 8 ms are
 * already a sign of unhealthy timing.
 *
 * Citations: roadmap "additive/observational" item #1; `STATUS_SAFETY`
 * frame already extended once before (DLC 3 → 5) — same pattern.
 */
#ifndef LOOP_DIAG_H
#define LOOP_DIAG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Submit the most recent 100 Hz task duration in microseconds.
 *
 * Atomically updates the windowed peak.  Safe to call from main-loop
 * context only (no ISR re-entry guard required).  NaN/Inf-proof since
 * the input is uint32_t.
 */
void LoopDiag_RecordTaskUs(uint32_t task_us);

/**
 * @brief   Snapshot the peak-hold value (in 100 µs units) and clear it.
 *
 * Returns a value in the range [0, 255].  Saturates at 255 for any
 * task longer than 25.5 ms (already off-spec; HMI shows "≥25.5 ms").
 * Caller must invoke this exactly once per TX cycle.
 */
uint8_t LoopDiag_GetAndResetPeak100us(void);

/**
 * @brief   Read the peak-hold value without clearing — for unit tests.
 */
uint32_t LoopDiag_PeekPeakUs(void);

#ifdef __cplusplus
}
#endif

#endif /* LOOP_DIAG_H */
