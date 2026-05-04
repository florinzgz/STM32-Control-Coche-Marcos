/**
 * @file    test_loop_diag.c
 * @brief   Host tests for loop_diag peak-hold telemetry.
 *
 * Compile:
 *   gcc -std=c11 -DHOST_TEST -ICore/Inc -O2 \
 *       Core/Src/test_loop_diag.c Core/Src/loop_diag.c -o /tmp/test_loop_diag
 *
 * This file defines main() and is intended ONLY for host-side unit testing.
 * It is excluded from the STM32 firmware build via the HOST_TEST guard so
 * that its main() does not collide with the firmware's main() in
 * Core/Src/main.c at link time.
 */
#ifdef HOST_TEST

#include "loop_diag.h"
#include <stdio.h>
#include <stdint.h>

static int  tests_run    = 0;
static int  tests_failed = 0;

#define ASSERT_EQ(actual, expected) do {                                   \
    tests_run++;                                                           \
    uint32_t _a = (uint32_t)(actual);                                      \
    uint32_t _e = (uint32_t)(expected);                                    \
    if (_a != _e) {                                                        \
        printf("FAIL %s:%d  %s = %u, expected %u\n",                       \
               __FILE__, __LINE__, #actual, (unsigned)_a, (unsigned)_e);   \
        tests_failed++;                                                    \
    }                                                                      \
} while (0)

static void reset_state(void) {
    /* Drain any leftover peak from the previous test. */
    (void)LoopDiag_GetAndResetPeak100us();
}

/* Initial state: no record, peak is zero, snapshot returns 0. */
static void test_initial_zero(void) {
    reset_state();
    ASSERT_EQ(LoopDiag_PeekPeakUs(), 0U);
    ASSERT_EQ(LoopDiag_GetAndResetPeak100us(), 0U);
}

/* Single record, value < 100 µs → 100 µs units = 0. */
static void test_sub_100us(void) {
    reset_state();
    LoopDiag_RecordTaskUs(50U);
    ASSERT_EQ(LoopDiag_PeekPeakUs(), 50U);
    ASSERT_EQ(LoopDiag_GetAndResetPeak100us(), 0U);
    /* After reset, peek is zero. */
    ASSERT_EQ(LoopDiag_PeekPeakUs(), 0U);
}

/* Peak-hold semantics: max(record1, record2). */
static void test_peak_hold_max(void) {
    reset_state();
    LoopDiag_RecordTaskUs(8000U);   /* 8 ms */
    LoopDiag_RecordTaskUs(3000U);   /* 3 ms — must NOT lower the peak */
    ASSERT_EQ(LoopDiag_PeekPeakUs(), 8000U);
    LoopDiag_RecordTaskUs(9500U);   /* 9.5 ms — must raise the peak */
    ASSERT_EQ(LoopDiag_PeekPeakUs(), 9500U);
    /* 9500 / 100 = 95. */
    ASSERT_EQ(LoopDiag_GetAndResetPeak100us(), 95U);
    /* Peak cleared after snapshot. */
    ASSERT_EQ(LoopDiag_PeekPeakUs(), 0U);
}

/* Conversion µs → 100 µs: integer-division truncation. */
static void test_unit_conversion(void) {
    reset_state();
    LoopDiag_RecordTaskUs(199U);       /* 199 µs → 1 unit (truncates) */
    ASSERT_EQ(LoopDiag_GetAndResetPeak100us(), 1U);
    LoopDiag_RecordTaskUs(200U);       /* exact 2 units */
    ASSERT_EQ(LoopDiag_GetAndResetPeak100us(), 2U);
    LoopDiag_RecordTaskUs(10000U);     /* 10 ms exact = 100 units */
    ASSERT_EQ(LoopDiag_GetAndResetPeak100us(), 100U);
}

/* Saturation at 255 (= 25.5 ms). */
static void test_saturation(void) {
    reset_state();
    LoopDiag_RecordTaskUs(25500U);
    ASSERT_EQ(LoopDiag_GetAndResetPeak100us(), 255U);
    LoopDiag_RecordTaskUs(25600U);   /* > 25.5 ms — saturates */
    ASSERT_EQ(LoopDiag_GetAndResetPeak100us(), 255U);
    LoopDiag_RecordTaskUs(0xFFFFFFFFU);  /* uint32 max — must saturate */
    ASSERT_EQ(LoopDiag_GetAndResetPeak100us(), 255U);
}

/* Reset-after-snapshot: a fresh small reading after a saturated one
 * does NOT carry any peak from before. */
static void test_window_resets(void) {
    reset_state();
    LoopDiag_RecordTaskUs(20000U);                  /* 20 ms */
    ASSERT_EQ(LoopDiag_GetAndResetPeak100us(), 200U);
    /* Window now empty. */
    ASSERT_EQ(LoopDiag_PeekPeakUs(), 0U);
    LoopDiag_RecordTaskUs(500U);                    /* 0.5 ms */
    ASSERT_EQ(LoopDiag_GetAndResetPeak100us(), 5U); /* 500 / 100 */
}

int main(void) {
    test_initial_zero();
    test_sub_100us();
    test_peak_hold_max();
    test_unit_conversion();
    test_saturation();
    test_window_resets();

    printf("--- loop_diag tests: %d run, %d failed ---\n",
           tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}

#endif /* HOST_TEST */
