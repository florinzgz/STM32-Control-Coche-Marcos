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

    // Absolute 100 ms producer: independent from main-loop liveness.
    CHECK(!p.shouldAttempt(1099U));
    CHECK(p.shouldAttempt(1100U));
    p.noteAttempt(1100U, true);
    CHECK(p.stats().attempts == 1U);
    CHECK(p.stats().successes == 1U);
    CHECK(p.stats().failures == 0U);
    CHECK(p.stats().lastSuccessMs == 1100U);
    CHECK(p.stats().maxSuccessGapMs == 100U);
    CHECK(!p.retryPending());

    // A blocked Arduino loop cannot suppress the dedicated producer.
    CHECK(!p.shouldAttempt(1199U));
    CHECK(p.shouldAttempt(1200U));
    p.noteAttempt(1200U, true);
    CHECK(p.stats().successes == 2U);

    // Loop kicks are instrumentation only.  The first kick also records the
    // 201 ms elapsed since reset, then later kicks track their own smaller gap.
    p.notifyLoopAlive(1201U);
    p.notifyLoopAlive(1260U);
    CHECK(p.stats().maxObservedLoopGapMs == 201U);
    CHECK(!p.shouldAttempt(1299U));
    CHECK(p.shouldAttempt(1300U));

    // Queue-full/timeout: counter/time success must not advance; retry at 10 ms.
    p.noteAttempt(1300U, false);
    p.noteQueueFull();
    CHECK(p.stats().failures == 1U);
    CHECK(p.stats().queueFullObservations == 1U);
    CHECK(p.stats().lastSuccessMs == 1200U);
    CHECK(p.retryPending());
    CHECK(!p.shouldAttempt(1309U));
    CHECK(p.shouldAttempt(1310U));
    p.noteAttempt(1310U, true);
    CHECK(!p.retryPending());
    CHECK(p.stats().successes == 3U);
    CHECK(p.stats().maxSuccessGapMs == 110U);

    // Schedule remains absolute: successful retry does not drift cadence.
    CHECK(!p.shouldAttempt(1399U));
    CHECK(p.shouldAttempt(1400U));

    // Long controller outage is observable and produces a warning gap.
    p.noteAttempt(1400U, false);
    p.noteSkippedNotRunning();
    p.noteSkippedNotRunning();
    CHECK(p.stats().skippedNotRunning == 2U);
    CHECK(p.shouldAttempt(1410U));
    p.noteAttempt(1410U, false);
    CHECK(p.shouldAttempt(1420U));
    p.noteAttempt(1600U, true);
    CHECK(p.stats().warningGapEvents == 1U);
    CHECK(p.stats().maxSuccessGapMs == 290U);

    // Drops reported by other CAN producers are diagnostics only.
    p.notifyTxDrop();
    p.notifyTxDrop();
    CHECK(p.stats().txDropNotifications == 2U);

    // Current successful-heartbeat age is wrap-safe.
    CHECK(p.currentSuccessGapMs(1650U) == 50U);

    GuardPolicy wrap;
    wrap.reset(0xFFFFFFF0U);
    CHECK(!wrap.shouldAttempt(0x00000040U));  // 80 ms after reset
    CHECK(wrap.shouldAttempt(0x00000054U));   // 100 ms after reset
    wrap.noteAttempt(0x00000054U, true);
    CHECK(wrap.stats().successes == 1U);
    CHECK(wrap.stats().maxSuccessGapMs == 100U);
    CHECK(wrap.currentSuccessGapMs(0x00000086U) == 50U);

    std::printf("can_heartbeat_guard: %d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
