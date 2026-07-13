// test_display_supervisor.cpp — host tests for the TFT display supervisor FSM.
//
// Covers the audit's Problem-2 test matrix (§7 A):
//   - normal heartbeat,
//   - isolated slow frame (no false detection),
//   - render stalled (staleness),
//   - repeated invalid status,
//   - readback unsupported (reset inferred),
//   - successful recovery,
//   - failed recovery,
//   - attempt cap,
//   - backoff,
//   - persistence of the last reason,
//   - no false detection from a single slow frame.
//
// Compile (from repository root):
//   g++ -std=c++17 -Wall -Wextra -Iesp32/src esp32/src/test_display_supervisor.cpp -o /tmp/test_display_supervisor
//   /tmp/test_display_supervisor

#include "display_supervisor.h"

#include <cstdio>

using namespace display;

static int tests_run = 0, tests_failed = 0;
#define CHECK(expr) do {                                             \
    tests_run++;                                                     \
    if (!(expr)) {                                                   \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);       \
        tests_failed++;                                              \
    }                                                                \
} while (0)

// Helper: drive a full successful recovery from CONFIRMED_LOST/REQUESTED.
static void driveRecovery(Supervisor& s, uint32_t& t, bool ok) {
    // Must be in RECOVERY_REQUESTED to advance.
    CHECK(s.state() == State::RECOVERY_REQUESTED);
    s.advanceRecovery(t); CHECK(s.state() == State::RESETTING);
    s.advanceRecovery(t); CHECK(s.state() == State::REINITIALIZING);
    s.advanceRecovery(t); CHECK(s.state() == State::REDRAWING);
    t += 50;
    s.completeRecovery(t, ok);
}

int main() {
    const uint32_t HEAP_OK = 200000;

    // ---- Normal heartbeat: fresh frames keep OK ----
    {
        Supervisor s;
        uint32_t t = 1000;
        for (int i = 0; i < 20; i++) {
            t += 50;                 // 20 FPS
            State st = s.update(t, t, StatusRead::VALID, HEAP_OK);
            CHECK(st == State::OK);
        }
    }

    // ---- Isolated slow frame must NOT trip detection ----
    {
        Supervisor s;
        uint32_t t = 1000;
        // Steady frames...
        for (int i = 0; i < 5; i++) { t += 50; s.update(t, t, StatusRead::VALID, HEAP_OK); }
        // One slow frame: 600 ms since last complete (over the 500 ms floor),
        // a single sample -> at most SUSPECT, never CONFIRMED.
        uint32_t last_complete = t;
        t += 600;
        State st = s.update(t, last_complete, StatusRead::VALID, HEAP_OK);
        CHECK(st == State::OK || st == State::SUSPECT);
        CHECK(st != State::CONFIRMED_LOST);
        // Next frame completes normally -> back to OK, no false alarm.
        t += 50;
        CHECK(s.update(t, t, StatusRead::VALID, HEAP_OK) == State::OK);
    }

    // ---- Render fully stalled -> SUSPECT -> CONFIRMED_LOST (RENDER_TIMEOUT) ----
    {
        Supervisor s;
        uint32_t t = 1000;
        uint32_t last_complete = t;   // render never completes again
        State st = State::OK;
        for (int i = 0; i < 6; i++) {
            t += 200;
            st = s.update(t, last_complete, StatusRead::VALID, HEAP_OK);
        }
        CHECK(st == State::RECOVERY_REQUESTED || st == State::CONFIRMED_LOST);
        CHECK(s.lastFault() == Fault::RENDER_TIMEOUT);
    }

    // ---- Repeated invalid status -> TFT_STATUS_LOST ----
    {
        Supervisor s;
        uint32_t t = 1000;
        State st = State::OK;
        for (int i = 0; i < 8; i++) {
            t += 100;
            // frames are "fresh" but status regs read garbage
            st = s.update(t, t, StatusRead::INVALID, HEAP_OK);
        }
        CHECK(st == State::RECOVERY_REQUESTED || st == State::CONFIRMED_LOST);
        CHECK(s.lastFault() == Fault::TFT_STATUS_LOST);
    }

    // ---- Readback unsupported + render stalled -> TFT_RESET_PROBABLE ----
    {
        Supervisor s;
        uint32_t t = 1000;
        uint32_t last_complete = t;
        State st = State::OK;
        for (int i = 0; i < 6; i++) {
            t += 200;
            st = s.update(t, last_complete, StatusRead::UNSUPPORTED, HEAP_OK);
        }
        CHECK(st == State::RECOVERY_REQUESTED || st == State::CONFIRMED_LOST);
        CHECK(s.lastFault() == Fault::TFT_RESET_PROBABLE);
    }

    // ---- Low memory classification ----
    {
        Supervisor s;
        uint32_t t = 1000;
        State st = State::OK;
        for (int i = 0; i < 6; i++) {
            t += 100;
            st = s.update(t, t, StatusRead::VALID, /*free_heap=*/1000);
        }
        CHECK(st == State::RECOVERY_REQUESTED || st == State::CONFIRMED_LOST);
        CHECK(s.lastFault() == Fault::LOW_MEMORY);
    }

    // ---- Successful recovery resets counters, records duration ----
    {
        Supervisor s;
        uint32_t t = 1000;
        uint32_t last_complete = t;
        while (s.state() != State::RECOVERY_REQUESTED) {
            t += 200;
            s.update(t, last_complete, StatusRead::VALID, HEAP_OK);
        }
        driveRecovery(s, t, /*ok=*/true);
        CHECK(s.state() == State::RECOVERED);
        CHECK(s.recoveryCount() == 1);
        CHECK(s.recoveryFailCount() == 0);
        CHECK(s.attempts() == 0);
        CHECK(s.lastFaultDurationMs() > 0);
        // A subsequent healthy frame returns to OK.
        t += 50;
        CHECK(s.update(t, t, StatusRead::VALID, HEAP_OK) == State::OK);
    }

    // ---- Failed recovery: retries, backoff, then attempt cap ----
    {
        Config cfg; cfg.max_attempts = 3; cfg.backoff_base_ms = 1000;
        Supervisor s(cfg);
        uint32_t t = 1000;
        uint32_t last_complete = t;
        // Reach the first recovery request.
        while (s.state() != State::RECOVERY_REQUESTED) {
            t += 200;
            s.update(t, last_complete, StatusRead::VALID, HEAP_OK);
        }
        // Attempt 1 fails -> backoff, CONFIRMED_LOST.
        driveRecovery(s, t, /*ok=*/false);
        CHECK(s.recoveryFailCount() == 1);
        CHECK(s.attempts() == 1);
        CHECK(s.state() == State::CONFIRMED_LOST);
        // During backoff, update() must NOT immediately re-request.
        State during = s.update(t + 10, last_complete, StatusRead::VALID, HEAP_OK);
        CHECK(during == State::CONFIRMED_LOST);
        // After the backoff window, a new request opens.
        t += cfg.backoff_base_ms + 50;
        State after = s.update(t, last_complete, StatusRead::VALID, HEAP_OK);
        CHECK(after == State::RECOVERY_REQUESTED);
        // Attempt 2 fails.
        driveRecovery(s, t, false);
        CHECK(s.attempts() == 2);
        // Backoff, then attempt 3 fails -> terminal RECOVERY_FAILED.
        t += (uint32_t)s.attempts() * cfg.backoff_base_ms + 50;
        while (s.state() == State::CONFIRMED_LOST) {
            t += 50;
            s.update(t, last_complete, StatusRead::VALID, HEAP_OK);
        }
        CHECK(s.state() == State::RECOVERY_REQUESTED);
        driveRecovery(s, t, false);
        CHECK(s.state() == State::RECOVERY_FAILED);
        CHECK(s.attempts() == 3);
        // Terminal: further updates stay FAILED (no infinite loop), reason kept.
        CHECK(s.update(t + 500, last_complete, StatusRead::VALID, HEAP_OK)
              == State::RECOVERY_FAILED);
        CHECK(s.lastFault() == Fault::RENDER_TIMEOUT);
    }

    // ---- forceFault (e.g. persisted ESP32 reset) surfaces after reboot ----
    {
        Supervisor s;
        s.forceFault(5000, Fault::ESP32_RESET);
        CHECK(s.lastFault() == Fault::ESP32_RESET);
        CHECK(s.state() == State::RECOVERY_REQUESTED ||
              s.state() == State::CONFIRMED_LOST);
    }

    printf("display_supervisor: %d run, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
