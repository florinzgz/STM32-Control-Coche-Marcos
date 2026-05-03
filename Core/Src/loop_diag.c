/**
 * @file    loop_diag.c
 * @brief   Implementation of main-loop timing diagnostic.
 *
 * Single 32-bit windowed peak in microseconds, saturated at the 32-bit
 * range (any reasonable 100 Hz task runs in < 10 ms = 10 000 µs).
 *
 * Concurrency: read-modify-write happens only on the main-loop thread
 * (the recorder is invoked from the 100 Hz block; the snapshot is
 * invoked from the 100 ms CAN TX block which lives in the same loop).
 * No ISR writes to this state, so no locking is required.
 */
#include "loop_diag.h"

static uint32_t s_peak_window_us = 0U;

void LoopDiag_RecordTaskUs(uint32_t task_us)
{
    if (task_us > s_peak_window_us) {
        s_peak_window_us = task_us;
    }
}

uint8_t LoopDiag_GetAndResetPeak100us(void)
{
    uint32_t peak = s_peak_window_us;
    s_peak_window_us = 0U;

    /* Convert µs → 100 µs units, saturating at 255 (= 25.5 ms). */
    uint32_t units = peak / 100U;
    if (units > 255U) {
        units = 255U;
    }
    return (uint8_t)units;
}

uint32_t LoopDiag_PeekPeakUs(void)
{
    return s_peak_window_us;
}
