// =============================================================================
// Host test — TWAI (CAN) BUS_OFF stable-heartbeat recovery policy
//
// Reproduces the photographed ESP32 oscillation
//     BUS_OFF → RECOVERING → STOPPED → RUNNING → BUS_OFF
// and proves the recovery policy:
//   - does NOT reset counters merely on RUNNING / STOPPED→RUNNING;
//   - confirms recovery only after sustained advancing heartbeats;
//   - preserves the lifetime BUS_OFF event count;
//   - uses fast bounded retries then slow unbounded backoff;
//   - always recovers after a physical reconnection;
//   - is wrap-safe across the millis() rollover.
// =============================================================================

#include <cstdint>
#include <cstdio>
#include "twai_recovery.h"

using twai_recovery::TwaiBusOffRecovery;
using twai_recovery::TwaiState;
using twai_recovery::TwaiAction;

static int g_run = 0;
static int g_fail = 0;

#define CHECK(cond, msg) do {                                   \
    ++g_run;                                                    \
    if (!(cond)) { ++g_fail; printf("FAIL: %s\n", (msg)); }     \
    else         { printf("PASS: %s\n", (msg)); }               \
} while (0)

// Test parameters (small windows for fast, deterministic tests).
static constexpr uint32_t FAST = 500;
static constexpr uint32_t SLOW = 5000;
static constexpr uint8_t  MAXF = 4;
static constexpr uint32_t WIN  = 1000;

static TwaiBusOffRecovery makeFsm() {
    return TwaiBusOffRecovery(FAST, SLOW, MAXF, WIN);
}

// --- Healthy running never acts and never counts a bus-off -------------------
static void test_healthy_running_idle() {
    TwaiBusOffRecovery fsm = makeFsm();
    for (uint32_t t = 0; t < 5000; t += 250) {
        CHECK(fsm.update(t, TwaiState::RUNNING, true) == TwaiAction::NONE,
              "healthy RUNNING is idle");
    }
    CHECK(fsm.lifetimeBusOffCount() == 0, "no bus-off counted while healthy");
    CHECK(!fsm.episodeActive(), "no episode active while healthy");
}

// --- BUS_OFF triggers recovery and counts exactly one lifetime event ---------
static void test_busoff_triggers_recovery_once() {
    TwaiBusOffRecovery fsm = makeFsm();
    uint32_t t = 0;
    CHECK(fsm.update(t, TwaiState::BUS_OFF, false) == TwaiAction::INITIATE_RECOVERY,
          "first BUS_OFF tick initiates recovery");
    CHECK(fsm.lifetimeBusOffCount() == 1, "lifetime bus-off counted once");
    // Staying in BUS_OFF for many ticks must NOT re-count the event.
    for (int i = 0; i < 5; ++i) { t += 100; fsm.update(t, TwaiState::BUS_OFF, false); }
    CHECK(fsm.lifetimeBusOffCount() == 1, "staying BUS_OFF does not re-count event");
}

// --- Fast bounded retries then slow unbounded backoff ------------------------
static void test_fast_then_slow_backoff() {
    TwaiBusOffRecovery fsm = makeFsm();
    uint32_t t = 0;
    int fastActions = 0;
    // First tick fires immediately, then MAXF-1 more at the FAST interval.
    CHECK(fsm.update(t, TwaiState::BUS_OFF, false) == TwaiAction::INITIATE_RECOVERY,
          "attempt 1 immediate");
    fastActions = 1;
    for (int i = 0; i < MAXF; ++i) {
        // Just before the fast interval: no action.
        t += FAST - 100;
        CHECK(fsm.update(t, TwaiState::BUS_OFF, false) == TwaiAction::NONE,
              "no retry before fast interval elapses");
        // After the fast interval elapses.
        t += 100;
        if (fsm.update(t, TwaiState::BUS_OFF, false) == TwaiAction::INITIATE_RECOVERY)
            ++fastActions;
    }
    CHECK(fsm.attempts() >= MAXF, "reached the fast-retry budget");
    // Now in slow backoff: a FAST-sized gap is NOT enough to retry.
    t += FAST + 50;
    CHECK(fsm.update(t, TwaiState::BUS_OFF, false) == TwaiAction::NONE,
          "slow backoff ignores a fast-sized gap");
    // A full SLOW interval does retry (unbounded).
    t += SLOW;
    CHECK(fsm.update(t, TwaiState::BUS_OFF, false) == TwaiAction::INITIATE_RECOVERY,
          "slow backoff retries after the slow interval");
}

// --- STOPPED restarts the driver during an episode ---------------------------
static void test_stopped_starts_driver() {
    TwaiBusOffRecovery fsm = makeFsm();
    uint32_t t = 0;
    fsm.update(t, TwaiState::BUS_OFF, false);
    t += 100;
    fsm.update(t, TwaiState::RECOVERING, false);
    t += 100;
    CHECK(fsm.update(t, TwaiState::STOPPED, false) == TwaiAction::START,
          "STOPPED during episode requests twai_start()");
}

// --- RUNNING after recovery does NOT confirm immediately ---------------------
static void test_running_does_not_confirm_immediately() {
    TwaiBusOffRecovery fsm = makeFsm();
    uint32_t t = 0;
    fsm.update(t, TwaiState::BUS_OFF, false);
    uint8_t attemptsBefore = fsm.attempts();
    t += 100; fsm.update(t, TwaiState::STOPPED, false);
    t += 100;
    // First RUNNING tick: probation, not confirmed, counters preserved.
    CHECK(fsm.update(t, TwaiState::RUNNING, true) == TwaiAction::NONE,
          "RUNNING alone does not confirm recovery");
    CHECK(fsm.inProbation(), "RUNNING after episode is probation");
    CHECK(fsm.episodeActive(), "episode still active during probation");
    CHECK(fsm.attempts() == attemptsBefore,
          "attempt counter NOT reset merely on RUNNING");
}

// --- Sustained heartbeats confirm recovery exactly once ----------------------
static void test_sustained_heartbeat_confirms() {
    TwaiBusOffRecovery fsm = makeFsm();
    uint32_t t = 0;
    fsm.update(t, TwaiState::BUS_OFF, false);
    t += 100; fsm.update(t, TwaiState::STOPPED, false);
    // Enter probation.
    t += 100; fsm.update(t, TwaiState::RUNNING, true);
    uint32_t streakStart = t;
    TwaiAction act = TwaiAction::NONE;
    // Sustain heartbeats until the window elapses.
    while ((uint32_t)(t - streakStart) < WIN) {
        t += 100;
        act = fsm.update(t, TwaiState::RUNNING, true);
        if (act == TwaiAction::CONFIRMED) break;
    }
    CHECK(act == TwaiAction::CONFIRMED, "recovery confirmed after stable window");
    CHECK(!fsm.episodeActive(), "episode cleared after confirmation");
    CHECK(fsm.attempts() == 0, "attempts reset after confirmation");
    CHECK(fsm.confirmedRecoveries() == 1, "one confirmed recovery recorded");
    // Steady state after confirmation is idle.
    t += 250;
    CHECK(fsm.update(t, TwaiState::RUNNING, true) == TwaiAction::NONE,
          "idle after confirmation");
}

// --- Heartbeat gap during probation restarts the stable window ---------------
static void test_heartbeat_gap_restarts_window() {
    TwaiBusOffRecovery fsm = makeFsm();
    uint32_t t = 0;
    fsm.update(t, TwaiState::BUS_OFF, false);
    t += 100; fsm.update(t, TwaiState::STOPPED, false);
    // Accumulate most of the window with heartbeats alive.
    t += 100; fsm.update(t, TwaiState::RUNNING, true);
    t += (WIN - 200); fsm.update(t, TwaiState::RUNNING, true);
    // Heartbeat drops just before confirmation — window must restart.
    t += 100; TwaiAction a = fsm.update(t, TwaiState::RUNNING, false);
    CHECK(a == TwaiAction::NONE, "heartbeat gap prevents confirmation");
    CHECK(fsm.episodeActive(), "still unconfirmed after gap");
    // Heartbeats return: must accumulate the FULL window again.
    uint32_t restart = t + 100;
    t = restart; fsm.update(t, TwaiState::RUNNING, true);
    t += (WIN - 200);
    CHECK(fsm.update(t, TwaiState::RUNNING, true) == TwaiAction::NONE,
          "partial window after gap does not confirm");
    t += 300;
    CHECK(fsm.update(t, TwaiState::RUNNING, true) == TwaiAction::CONFIRMED,
          "full window after gap confirms");
}

// --- Photographed oscillation: RUNNING flap without a stable window ----------
static void test_oscillation_never_confirms() {
    TwaiBusOffRecovery fsm = makeFsm();
    uint32_t t = 0;
    // BUS_OFF → RECOVERING → STOPPED → RUNNING (briefly) → BUS_OFF … repeated.
    for (int cycle = 0; cycle < 4; ++cycle) {
        fsm.update(t, TwaiState::BUS_OFF, false);       t += 100;
        fsm.update(t, TwaiState::RECOVERING, false);    t += 100;
        fsm.update(t, TwaiState::STOPPED, false);       t += 100;
        // RUNNING for only 300 ms (< WIN) with no real heartbeat, then off.
        fsm.update(t, TwaiState::RUNNING, false);       t += 300;
    }
    CHECK(fsm.confirmedRecoveries() == 0, "oscillation never falsely confirms");
    CHECK(fsm.episodeActive(), "still trying to recover under oscillation");
    CHECK(fsm.lifetimeBusOffCount() == 4, "each bus-off event counted");
}

// --- Auto-recovers after a physical reconnection (even past fast budget) ------
static void test_auto_recover_after_reconnect() {
    TwaiBusOffRecovery fsm = makeFsm();
    uint32_t t = 0;
    // Long disconnection: many BUS_OFF ticks well past the fast budget.
    int actions = 0;
    for (int i = 0; i < 200; ++i) {
        if (fsm.update(t, TwaiState::BUS_OFF, false) == TwaiAction::INITIATE_RECOVERY)
            ++actions;
        t += 250;
    }
    CHECK(actions > MAXF, "keeps retrying (unbounded) during long disconnection");
    // Cable reconnected: controller recovers and heartbeats resume.
    fsm.update(t, TwaiState::STOPPED, false); t += 100;
    uint32_t streakStart = t;
    TwaiAction act = TwaiAction::NONE;
    fsm.update(t, TwaiState::RUNNING, true);
    while ((uint32_t)(t - streakStart) < WIN + 200) {
        t += 100;
        act = fsm.update(t, TwaiState::RUNNING, true);
        if (act == TwaiAction::CONFIRMED) break;
    }
    CHECK(act == TwaiAction::CONFIRMED, "auto-recovers after reconnection");
}

// --- Lifetime count is preserved across a full recovery ----------------------
static void test_lifetime_preserved() {
    TwaiBusOffRecovery fsm = makeFsm();
    uint32_t t = 0;
    fsm.update(t, TwaiState::BUS_OFF, false);
    t += 100; fsm.update(t, TwaiState::STOPPED, false);
    t += 100; fsm.update(t, TwaiState::RUNNING, true);
    uint32_t s = t;
    while ((uint32_t)(t - s) < WIN + 100) { t += 100; fsm.update(t, TwaiState::RUNNING, true); }
    CHECK(fsm.lifetimeBusOffCount() == 1, "lifetime count survives recovery");
    // A brand new bus-off increments to 2.
    t += 500; fsm.update(t, TwaiState::BUS_OFF, false);
    CHECK(fsm.lifetimeBusOffCount() == 2, "new bus-off increments lifetime count");
}

// --- Wrap-safe across the millis() 32-bit rollover ---------------------------
static void test_wrap_safe() {
    TwaiBusOffRecovery fsm = makeFsm();
    uint32_t t = 0xFFFFFF00u;  // ~256 ms before wrap
    fsm.update(t, TwaiState::BUS_OFF, false);
    t += 100; fsm.update(t, TwaiState::STOPPED, false);  // wraps here
    t += 100; fsm.update(t, TwaiState::RUNNING, true);   // t now small (post-wrap)
    uint32_t s = t;
    TwaiAction act = TwaiAction::NONE;
    while ((uint32_t)(t - s) < WIN + 200) {
        t += 100;
        act = fsm.update(t, TwaiState::RUNNING, true);
        if (act == TwaiAction::CONFIRMED) break;
    }
    CHECK(act == TwaiAction::CONFIRMED, "confirmation works across millis() wrap");
}

int main() {
    test_healthy_running_idle();
    test_busoff_triggers_recovery_once();
    test_fast_then_slow_backoff();
    test_stopped_starts_driver();
    test_running_does_not_confirm_immediately();
    test_sustained_heartbeat_confirms();
    test_heartbeat_gap_restarts_window();
    test_oscillation_never_confirms();
    test_auto_recover_after_reconnect();
    test_lifetime_preserved();
    test_wrap_safe();
    printf("--- twai_recovery tests: %d run, %d failed ---\n", g_run, g_fail);
    return g_fail ? 1 : 0;
}
