#include <cstdint>
#include <cstdio>

#include "can/can_heartbeat_guard_policy.h"

static int checks = 0;
static int failures = 0;

#define CHECK(expr) do { ++checks; if (!(expr)) { ++failures; \
    std::printf("FAIL line %d: %s\n", __LINE__, #expr); } } while (0)

int main() {
    using namespace can_heartbeat;

    GuardPolicy p;
    p.reset(1000U);

    // Healthy loop: the guard remains silent and does not duplicate 0x011.
    p.notifyLoopAlive(1050U);
    p.observeQueue(0U);
    CHECK(!p.shouldAttempt(1100U));
    CHECK(p.stats().backupAttempts == 0U);

    // A loop/NVS stall is detected before the STM32 250 ms watchdog expires.
    CHECK(p.shouldAttempt(1170U));  // 120 ms since last kick
    CHECK(p.stats().loopStallEvents == 1U);
    p.noteAttempt(1170U, true);
    CHECK(p.stats().backupSuccess == 1U);
    CHECK(!p.retryPending());

    // Rate limiting: no heartbeat flood while the main loop remains stalled.
    CHECK(!p.shouldAttempt(1200U));
    CHECK(p.shouldAttempt(1250U));

    // A full TX queue latches congestion; inject once it drains.
    p.reset(2000U);
    p.notifyLoopAlive(2050U);
    p.observeQueue(5U);
    CHECK(p.congestionLatched());
    CHECK(p.stats().congestionEvents == 1U);
    p.observeQueue(2U);
    CHECK(p.shouldAttempt(2130U));
    p.noteAttempt(2130U, true);
    CHECK(!p.congestionLatched());

    // Explicit TX drop follows the same bounded recovery path.
    p.reset(3000U);
    p.notifyTxDrop();
    p.observeQueue(0U);
    CHECK(p.shouldAttempt(3080U));
    p.noteAttempt(3080U, false);
    CHECK(p.retryPending());
    CHECK(!p.shouldAttempt(3089U));
    CHECK(p.shouldAttempt(3090U));
    p.noteRetry();
    p.noteAttempt(3090U, true);
    CHECK(p.stats().backupFailures == 1U);
    CHECK(p.stats().retries == 1U);
    CHECK(p.stats().backupSuccess == 1U);

    // Unsigned subtraction keeps stall detection correct across millis wrap.
    p.reset(0xFFFFFFF0U);
    p.notifyLoopAlive(0xFFFFFFF5U);
    p.observeQueue(0U);
    CHECK(!p.shouldAttempt(0x00000040U));
    CHECK(p.shouldAttempt(0x00000080U));

    std::printf("can_heartbeat_guard: %d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
