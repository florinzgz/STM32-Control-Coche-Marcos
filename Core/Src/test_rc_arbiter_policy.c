#ifdef HOST_TEST
#include "rc_arbiter.h"
#include <math.h>
#include <stdio.h>
static int n, f;
#define T(x) do { ++n; if (!(x)) { ++f; printf("FAIL %d: %s\n", __LINE__, #x); } } while (0)
#define NEAR(a,b) (fabsf((a)-(b)) < 0.001f)
int main(void) {
    uint8_t frame[5] = {1U, 80U, 100U, 0U, 1U};
    RcArbiter_Init();
    RcArbiter_OnFrame(frame, 5U, 100U);
#if RC_OVERRIDE_ENABLED
    T(RcArbiter_IsActive(100U));
    T(NEAR(RcArbiter_GetThrottle(12.0f, 100U), 80.0f));
    T(NEAR(RcArbiter_GetSteering(-2.0f, 100U), 10.0f));
    RcArbiterStats_t before;
    RcArbiter_GetStats(&before);
    RcArbiter_OnFrame(frame, 5U, 150U); /* replayed sequence */
    RcArbiterStats_t after;
    RcArbiter_GetStats(&after);
    T(after.accepted_frames == before.accepted_frames);
    T(after.rejected_frames == before.rejected_frames + 1U);
    frame[0] = 0x81U;
    frame[4] = 2U;
    RcArbiter_OnFrame(frame, 5U, 160U);
    RcArbiter_GetStats(&after);
    T(after.accepted_frames == before.accepted_frames);
    frame[0] = 0x01U;
    frame[4] = 200U; /* backwards/stale modulo window from seq=1 */
    RcArbiter_OnFrame(frame, 5U, 170U);
    RcArbiter_GetStats(&after);
    T(after.accepted_frames == before.accepted_frames);
    frame[4] = 4U; /* forward progress with two dropped frames */
    RcArbiter_OnFrame(frame, 5U, 180U);
    RcArbiter_GetStats(&after);
    T(after.accepted_frames == before.accepted_frames + 1U);
#else
    T(!RcArbiter_IsActive(100U));
    T(NEAR(RcArbiter_GetThrottle(12.0f, 100U), 12.0f));
    T(NEAR(RcArbiter_GetSteering(-2.0f, 100U), -2.0f));
    RcArbiterStats_t stats;
    RcArbiter_GetStats(&stats);
    T(stats.accepted_frames == 0U);
    T(stats.rejected_frames == 1U);
#endif
    printf("test_rc_arbiter_policy: %d run, %d failed (enabled=%d)\n", n, f, RC_OVERRIDE_ENABLED);
    return f != 0;
}
#endif
