#!/usr/bin/env python3
"""Apply the final verified PR441 review corrections.

Temporary integration helper. The committing workflow removes this file.
"""
from pathlib import Path


def replace_once(path: str, old: str, new: str) -> None:
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{path}: expected one anchor, found {count}")
    p.write_text(text.replace(old, new, 1), encoding="utf-8")


# 1. Publish service ownership only after both physical lock predicates pass.
replace_once(
    "Core/Src/can_handler.c",
    '''/* Enter the physical calibration lock.  The flag is raised before power-down
 * so the 10 ms relay/traction guards cannot race and re-energise the vehicle. */
static bool pedalcal_service_enter(void)
{
    PedalCalConds c;
    pedalcal_build_conds(&c);
    if (!pedalcal_service_request_allowed(&c)) return false;

    pedalcal_service_active = true;
    Traction_SetAxisRotation(false);
    Steering_Neutralize();
    Relay_PowerDown();
    const bool en_pwm_locked = Traction_CalibrationLock();
    const bool relay_off = ((Safety_GetRelayStatusByte() & (1U << 1)) == 0U);
    if (!en_pwm_locked || !relay_off) {
        pedalcal_service_active = false;
        return false;
    }
    return true;
}
''',
    '''/* Enter the physical calibration lock.  Ownership is published only after
 * both the BTS7960 output lock and the traction-relay OFF state are confirmed,
 * so CAN_PedalCalServiceActive() can never report an unowned/pending lock. */
static bool pedalcal_service_enter(void)
{
    PedalCalConds c;
    pedalcal_build_conds(&c);
    if (!pedalcal_service_request_allowed(&c)) return false;

    Traction_SetAxisRotation(false);
    Steering_Neutralize();
    Relay_PowerDown();
    const bool en_pwm_locked = Traction_CalibrationLock();
    const bool relay_off = ((Safety_GetRelayStatusByte() & (1U << 1)) == 0U);
    if (!en_pwm_locked || !relay_off) {
        return false;
    }

    pedalcal_service_active = true;
    /* Re-assert after publication so the periodic guards and this synchronous
     * entry path agree on the same already-confirmed physical state. */
    Relay_PowerDown();
    (void)Traction_CalibrationLock();
    return true;
}
''')

# 2. Permit flash persistence only in STANDBY or a confirmed, clean service lock.
replace_once(
    "Core/Src/pedal_cal_store.c",
    '#include "safety_system.h"\n#include <string.h>\n',
    '''#include "safety_system.h"
#include <string.h>

/* Optional dependency for host fixtures; present in the productive image.
 * The service flag is published only after the physical lock is confirmed. */
extern bool CAN_PedalCalServiceActive(void) __attribute__((weak));

static bool pcal_service_lock_confirmed(void)
{
    return CAN_PedalCalServiceActive != 0 &&
           CAN_PedalCalServiceActive();
}
''')
replace_once(
    "Core/Src/pedal_cal_store.c",
    '''    /* Defense in depth: never persist while actuators may be live.
     * The CAN dispatcher (pedalcal_safety_ok in can_handler.c) already
     * blocks any caller path outside SYS_STATE_STANDBY, but the same
     * gate is re-asserted at the persistence boundary so that any
     * future caller (service-mode shortcut, host test fixture, etc.)
     * cannot accidentally erase page 124 while the vehicle is in
     * ACTIVE / DEGRADED / LIMP_HOME state.                            */
    if (Safety_GetState() != SYS_STATE_STANDBY)
        return false;
''',
    '''    /* Defense in depth: persistence is permitted only in true STANDBY or
     * while the confirmed pedal-calibration service lock owns a clean ACTIVE /
     * DEGRADED recovery state.  SAFE, ERROR, LIMP_HOME and any active safety
     * error remain hard blocks even if a stale caller attempts SAVE. */
    const SystemState_t state = Safety_GetState();
    const bool service_authorized =
        pcal_service_lock_confirmed() &&
        (state == SYS_STATE_ACTIVE || state == SYS_STATE_DEGRADED) &&
        Safety_GetError() == SAFETY_ERROR_NONE;
    if (state != SYS_STATE_STANDBY && !service_authorized)
        return false;
''')

# 3. Distinguish rearmable no-response failure from hard INVALID failure.
replace_once(
    "esp32/src/mode_sync.h",
    '''            failed_  = false;   // new intent — allow a fresh sync attempt
            blocked_ = false;   // and retry immediately, not after the cooldown
            episodeArmed_ = false;  // §4: a new intent starts a fresh age timer
''',
    '''            failed_  = false;   // new intent — allow a fresh sync attempt
            rearmableFailure_ = false;
            blocked_ = false;   // and retry immediately, not after the cooldown
            episodeArmed_ = false;  // §4: a new intent starts a fresh age timer
''')
replace_once(
    "esp32/src/mode_sync.h",
    '''            failed_         = false;
            blocked_        = false;
            blockedArmed_   = false;
            confirmedValid_ = false;
''',
    '''            failed_         = false;
            rearmableFailure_ = false;
            blocked_        = false;
            blockedArmed_   = false;
            confirmedValid_ = false;
''')
replace_once(
    "esp32/src/mode_sync.h",
    '''            pending_      = false;
            blocked_      = false;
            blockedArmed_ = false;
            episodeArmed_ = false;   // §4: in sync — next miss restarts the timer
''',
    '''            pending_      = false;
            failed_       = false;
            rearmableFailure_ = false;
            blocked_      = false;
            blockedArmed_ = false;
            episodeArmed_ = false;   // §4: in sync — next miss restarts the timer
''')
replace_once(
    "esp32/src/mode_sync.h",
    '''                pending_ = false;
                failed_  = true;
                return Action::NONE;
''',
    '''                pending_ = false;
                failed_  = true;
                rearmableFailure_ = true;  // only this failure class may re-arm
                return Action::NONE;
''')
replace_once(
    "esp32/src/mode_sync.h",
    '''                failed_         = false;
                blocked_        = false;
                blockedArmed_   = false;
                break;
            case AckResult::INVALID:
                failed_       = true;    // hard error — do not spam the bus
                blocked_      = false;
                blockedArmed_ = false;
''',
    '''                failed_         = false;
                rearmableFailure_ = false;
                blocked_        = false;
                blockedArmed_   = false;
                break;
            case AckResult::INVALID:
                failed_       = true;    // hard error — do not spam the bus
                rearmableFailure_ = false;
                blocked_      = false;
                blockedArmed_ = false;
''')
replace_once(
    "esp32/src/mode_sync.h",
    '''                blocked_      = true;
                blockedArmed_ = false;
                failed_       = false;
                blockReason_  = result;
''',
    '''                blocked_      = true;
                blockedArmed_ = false;
                failed_       = false;
                rearmableFailure_ = false;
                blockReason_  = result;
''')
replace_once(
    "esp32/src/mode_sync.h",
    '''            pending_ = false;
            failed_  = false;
        }
        if (echoModeFlags == desired_) {
''',
    '''            pending_ = false;
            failed_  = false;
            rearmableFailure_ = false;
        }
        if (echoModeFlags == desired_) {
''')
replace_once(
    "esp32/src/mode_sync.h",
    '''            blocked_      = false;
            blockedArmed_ = false;
        }
''',
    '''            blocked_      = false;
            blockedArmed_ = false;
            failed_       = false;
            rearmableFailure_ = false;
        }
''')
replace_once(
    "esp32/src/mode_sync.h",
    '''        failed_         = false;
        blocked_        = false;
        blockedArmed_   = false;
''',
    '''        failed_         = false;
        rearmableFailure_ = false;
        blocked_        = false;
        blockedArmed_   = false;
''')
replace_once(
    "esp32/src/mode_sync.h",
    '''    void rearmFailedAttempt() {
        if (!failed_) return;
        failed_       = false;
        pending_      = false;
        blocked_      = false;
        blockedArmed_ = false;
        retries_      = 0;
    }
''',
    '''    void rearmFailedAttempt() {
        if (!failed_ || !rearmableFailure_) return;
        failed_       = false;
        rearmableFailure_ = false;
        pending_      = false;
        blocked_      = false;
        blockedArmed_ = false;
        retries_      = 0;
    }
''')
replace_once(
    "esp32/src/mode_sync.h",
    '''    bool    failed()         const { return failed_; }
    /** True while a temporary ACK (REJECTED/BLOCKED) is pending its retry. */
''',
    '''    bool    failed()         const { return failed_; }
    /** True only for a bounded no-response failure that the owner may re-arm. */
    bool    rearmableFailure() const { return failed_ && rearmableFailure_; }
    /** True while a temporary ACK (REJECTED/BLOCKED) is pending its retry. */
''')
replace_once(
    "esp32/src/mode_sync.h",
    '''    bool     pending_     = false;
    bool     failed_      = false;
    bool     blocked_     = false;  ///< A temporary ACK is awaiting its retry.
''',
    '''    bool     pending_     = false;
    bool     failed_      = false;
    bool     rearmableFailure_ = false; ///< Exhausted no-response burst only.
    bool     blocked_     = false;  ///< A temporary ACK is awaiting its retry.
''')

# Owner-side cooldown starts when the current rearmable failure appears.
replace_once(
    "esp32/src/main.cpp",
    '''static unsigned long g_lastModeFailureRearmMs = 0;
static constexpr unsigned long MODE_SYNC_FAILURE_REARM_MS = 2000;
''',
    '''static unsigned long g_modeFailureSinceMs = 0;
static bool          g_modeFailureTimerArmed = false;
static constexpr unsigned long MODE_SYNC_FAILURE_REARM_MS = 2000;
''')
replace_once(
    "esp32/src/main.cpp",
    '''            if (g_modeSync.failed() && stm32IsAlive &&
                !g_modeSync.inSync() &&
                (now - g_lastModeFailureRearmMs) >= MODE_SYNC_FAILURE_REARM_MS) {
                g_lastModeFailureRearmMs = now;
                g_modeSync.rearmFailedAttempt();
                Serial.println("[MODESYNC] no-response burst re-armed");
            }
''',
    '''            if (g_modeSync.rearmableFailure() && stm32IsAlive &&
                !g_modeSync.inSync()) {
                if (!g_modeFailureTimerArmed) {
                    g_modeFailureSinceMs = now;
                    g_modeFailureTimerArmed = true;
                } else if ((now - g_modeFailureSinceMs) >= MODE_SYNC_FAILURE_REARM_MS) {
                    g_modeSync.rearmFailedAttempt();
                    g_modeFailureTimerArmed = false;
                    Serial.println("[MODESYNC] no-response burst re-armed");
                }
            } else {
                g_modeFailureTimerArmed = false;
            }
''')
replace_once(
    "esp32/src/main.cpp",
    '''    // Initialize remote control parser (FlySky FS-iA6B iBUS, GPIO 16 RX).
    // Enabled by default (REMOTE_CONTROL_ENABLED=1 in platformio.ini);
    // compiles to an inline no-op when the flag is set to 0.  See
''',
    '''    // Initialize remote control parser (FlySky FS-iA6B iBUS, GPIO 16 RX).
    // Disabled in the current bench build (REMOTE_CONTROL_ENABLED=0);
    // compiles to an inline no-op while the receiver is excluded.  See
''')

# Tests: no-response is rearmable; INVALID remains hard-latched.
replace_once(
    "esp32/src/test_mode_sync.cpp",
    '''    ASSERT(ms.failed());

    ms.rearmFailedAttempt();
''',
    '''    ASSERT(ms.failed());
    ASSERT(ms.rearmableFailure());

    ms.rearmFailedAttempt();
''')
replace_once(
    "esp32/src/test_mode_sync.cpp",
    '''static void test_failed_burst_can_be_rearmed() {
''',
    '''static void test_invalid_failure_cannot_be_rearmed() {
    ModeSync ms(ACK_TIMEOUT_MS, MAX_RETRIES);
    ms.setDesired(0x03);
    ASSERT(ms.update(0, true) == Action::SEND);
    ms.onAck(AckRes::INVALID);
    ASSERT(ms.failed());
    ASSERT(!ms.rearmableFailure());

    ms.rearmFailedAttempt();
    ASSERT(ms.failed());
    ASSERT(ms.update(ACK_TIMEOUT_MS + 1, true) == Action::NONE);
}

static void test_failed_burst_can_be_rearmed() {
''')
replace_once(
    "esp32/src/test_mode_sync.cpp",
    '''    test_failed_burst_can_be_rearmed();
    test_late_ack_confirms();
''',
    '''    test_failed_burst_can_be_rearmed();
    test_invalid_failure_cannot_be_rearmed();
    test_late_ack_confirms();
''')
