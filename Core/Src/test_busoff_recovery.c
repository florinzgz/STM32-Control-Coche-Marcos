/**
  ****************************************************************************
  * @file    test_busoff_recovery.c
  * @brief   Host-compilable unit tests for the CAN bus-off recovery
  *          confirmation window (busoff_recovery.c).
  *
  *          Verifies that recovery is declared ONLY after valid heartbeats
  *          are sustained continuously over the confirmation window, and
  *          never merely because the peripheral returned to RUNNING.
  *
  *          Compile with host GCC (from repo root):
  *            gcc -std=c11 -DHOST_TEST -ICore/Inc -O2 \
  *                Core/Src/test_busoff_recovery.c Core/Src/busoff_recovery.c \
  *                -o /tmp/test_busoff_recovery && /tmp/test_busoff_recovery
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "busoff_recovery.h"

static int tests_run    = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(expr) do {                                        \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);       \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_FALSE(expr) ASSERT_TRUE(!(expr))

#define WIN_MS 1000U

/* Not in probation → Update always returns false regardless of input. */
static void test_idle_never_confirms(void)
{
    BusOffRecoveryWindow_t w;
    BusOffRecoveryWindow_Reset(&w);
    ASSERT_FALSE(BusOffRecoveryWindow_InProbation(&w));
    for (uint32_t t = 0; t < 5U * WIN_MS; t += 100U) {
        ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, t, true, WIN_MS));
    }
}

/* RUNNING (Begin) alone must NOT confirm; a single alive frame must not
 * confirm; recovery is declared only after the full window elapses. */
static void test_running_alone_does_not_confirm(void)
{
    BusOffRecoveryWindow_t w;
    BusOffRecoveryWindow_Begin(&w, 0U);
    ASSERT_TRUE(BusOffRecoveryWindow_InProbation(&w));

    /* First alive frame at t=0 starts the streak but cannot confirm. */
    ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, 0U, true, WIN_MS));
    /* Just before the window elapses → still not confirmed. */
    ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, WIN_MS - 1U, true, WIN_MS));
    /* Exactly at the window → confirmed. */
    ASSERT_TRUE(BusOffRecoveryWindow_Update(&w, WIN_MS, true, WIN_MS));
    /* Probation cleared after confirmation. */
    ASSERT_FALSE(BusOffRecoveryWindow_InProbation(&w));
}

/* Confirmation fires exactly once; further updates stay false. */
static void test_confirms_once(void)
{
    BusOffRecoveryWindow_t w;
    BusOffRecoveryWindow_Begin(&w, 100U);
    ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, 100U, true, WIN_MS));
    ASSERT_TRUE(BusOffRecoveryWindow_Update(&w, 100U + WIN_MS, true, WIN_MS));
    /* Subsequent updates do not re-confirm. */
    ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, 100U + 2U * WIN_MS, true, WIN_MS));
    ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, 100U + 3U * WIN_MS, true, WIN_MS));
}

/* A heartbeat gap in the middle of the window resets the streak: the full
 * window must be re-accumulated from the moment heartbeats return. */
static void test_gap_resets_streak(void)
{
    BusOffRecoveryWindow_t w;
    BusOffRecoveryWindow_Begin(&w, 0U);

    /* Accumulate most of the window... */
    ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, 0U,   true, WIN_MS));
    ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, 900U, true, WIN_MS));
    /* ...then a heartbeat gap drops the streak. */
    ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, 950U, false, WIN_MS));
    /* Heartbeats return; the OLD elapsed time must NOT count. */
    ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, 1000U, true, WIN_MS)); /* streak restart */
    ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, 1999U, true, WIN_MS)); /* 999ms < window */
    ASSERT_TRUE(BusOffRecoveryWindow_Update(&w,  2000U, true, WIN_MS)); /* full new window */
}

/* Flapping alive/dead never confirms (each dead frame resets the streak). */
static void test_flapping_never_confirms(void)
{
    BusOffRecoveryWindow_t w;
    BusOffRecoveryWindow_Begin(&w, 0U);
    bool alive = true;
    for (uint32_t t = 0; t < 10U * WIN_MS; t += 100U) {
        /* alive for one tick, dead the next — streak can never reach WIN_MS */
        ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, t, alive, WIN_MS));
        alive = !alive;
    }
    /* Still awaiting confirmation. */
    ASSERT_TRUE(BusOffRecoveryWindow_InProbation(&w));
}

/* Re-Begin after a reset (fresh bus-off during probation) starts clean. */
static void test_rebegin_after_reset(void)
{
    BusOffRecoveryWindow_t w;
    BusOffRecoveryWindow_Begin(&w, 0U);
    ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, 0U, true, WIN_MS));
    /* Simulate a fresh bus-off mid-probation. */
    BusOffRecoveryWindow_Reset(&w);
    ASSERT_FALSE(BusOffRecoveryWindow_InProbation(&w));
    /* New probation window opens later. */
    BusOffRecoveryWindow_Begin(&w, 5000U);
    ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, 5000U,          true, WIN_MS));
    ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, 5000U + 999U,   true, WIN_MS));
    ASSERT_TRUE(BusOffRecoveryWindow_Update(&w,  5000U + WIN_MS, true, WIN_MS));
}

/* Tick wrap-around (49.7-day HAL_GetTick wrap) is handled by modular math. */
static void test_tick_wraparound(void)
{
    BusOffRecoveryWindow_t w;
    uint32_t start = 0xFFFFFF00U;   /* 256 ms before wrap */
    BusOffRecoveryWindow_Begin(&w, start);
    ASSERT_FALSE(BusOffRecoveryWindow_Update(&w, start, true, WIN_MS));
    uint32_t later = start + WIN_MS; /* wraps past 0 */
    ASSERT_TRUE(BusOffRecoveryWindow_Update(&w, later, true, WIN_MS));
}

int main(void)
{
    test_idle_never_confirms();
    test_running_alone_does_not_confirm();
    test_confirms_once();
    test_gap_resets_streak();
    test_flapping_never_confirms();
    test_rebegin_after_reset();
    test_tick_wraparound();

    printf("\n--- busoff_recovery tests: %d run, %d failed ---\n",
           tests_run, tests_failed);

    return tests_failed ? 1 : 0;
}

#endif /* HOST_TEST */
