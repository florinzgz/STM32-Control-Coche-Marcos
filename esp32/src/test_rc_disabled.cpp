/**
 ****************************************************************************
 * @file    test_rc_disabled.cpp
 * @brief   Host-compilable test that verifies REMOTE_CONTROL_ENABLED=0 stubs
 *          are fail-safe: no remote authority, zero demand, zero steering,
 *          neutral gear, no active override and no executable RC path in
 *          the binary.
 *
 *          Compile and run on host (from repository root):
 *            g++ -std=c++17 -DREMOTE_CONTROL_ENABLED=0 \
 *                -Iesp32/src -Iesp32/test_stubs \
 *                esp32/src/test_rc_disabled.cpp \
 *                -o /tmp/test_rc_disabled && /tmp/test_rc_disabled
 *
 *          PlatformIO scans esp32/src automatically.  The ARDUINO guard keeps
 *          this host-only main() out of the production firmware while the
 *          standalone host command above continues to compile and execute it.
 *
 *          Verified:
 *            1. remote_control::isActive()          == false
 *            2. remote_control::isRemoteSelected() == false
 *            3. remote_control::getThrottlePct()   == 0.0f
 *            4. remote_control::getSteeringDeg()   == 0.0f
 *            5. remote_control::getRequestedGear() == 2  (NEUTRAL)
 *            6. remote_control::getDriveMode()     == 0  (2WD, tank OFF)
 *            7. remote_control::isLightsOn()       == false
 *            8. remote_control::isKillSwitchActive() == true (fail-safe default)
 *            9. remote_control::getState()         == State::IDLE
 ****************************************************************************
 */

#if !defined(ARDUINO)

#include <cstdio>
#include <cmath>
#include "remote_control.h"

static int failed;
#define CHECK(expr) do { \
    if (!(expr)) { \
        std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        ++failed; \
    } \
} while (0)

int main()
{
    /* No initialization needed — the stubs are stateless constexpr inlines.
     * Calling init() anyway verifies it is a no-op and does not fault.    */
    remote_control::init();
    remote_control::update();

    /* 1. Remote authority must be inactive when the module is compiled out. */
    CHECK(remote_control::isActive()         == false);
    CHECK(remote_control::isRemoteSelected() == false);

    /* 2. Demand and steering must be exactly zero — no residual output. */
    CHECK(std::fpclassify(remote_control::getThrottlePct()) == FP_ZERO ||
          remote_control::getThrottlePct() == 0.0f);
    CHECK(std::fpclassify(remote_control::getSteeringDeg()) == FP_ZERO ||
          remote_control::getSteeringDeg() == 0.0f);

    /* 3. Gear must be neutral (value 2) — safe default when RC is absent. */
    CHECK(remote_control::getRequestedGear() == 2U);

    /* 4. Drive mode must be 2WD with tank OFF (bits 0-1 = 0). */
    CHECK(remote_control::getDriveMode() == 0U);

    /* 5. Lights and audio volume must be off/zero. */
    CHECK(remote_control::isLightsOn()     == false);
    CHECK(remote_control::getAudioVolume() == 0U);

    /* 6. Kill switch defaults to active (fail-safe: no receiver = no authority). */
    CHECK(remote_control::isKillSwitchActive() == true);

    /* 7. FSM state must be IDLE (no transition without a real receiver). */
    CHECK(remote_control::getState() == remote_control::State::IDLE);

    if (failed != 0) return 1;
    std::puts("RC disabled stubs: PASS");
    return 0;
}

#endif /* !defined(ARDUINO) */
