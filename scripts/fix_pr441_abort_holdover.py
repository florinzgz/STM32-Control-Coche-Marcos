#!/usr/bin/env python3
"""Apply final PR441 safety hold and CAN-recovery corrections.

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


replace_once(
    "Core/Inc/can_handler.h",
    '''/* True while the pedal-calibration service lock owns the powertrain.
 * The lock is entered only after verifying P/N, stopped wheels, released and
 * plausible pedal, healthy CAN and no active safety error.  While true, the
 * relay sequencer and productive traction wrapper must keep all traction
 * outputs in physical COAST and both power relays de-energised. */
bool CAN_PedalCalServiceActive(void);
''',
    '''/* True while any pedal-calibration guard phase owns the powertrain:
 * PENDING entry, confirmed ACTIVE session, or post-session HOLD.  While true,
 * the relay sequencer and productive traction wrapper must keep all traction
 * outputs in physical COAST and both power relays de-energised. */
bool CAN_PedalCalServiceActive(void);

/* True only after the physical lock was confirmed and while the guided session
 * owns it.  Persistence uses this stricter predicate; PENDING/HOLD never
 * authorize a flash erase/program operation. */
bool CAN_PedalCalServiceConfirmed(void);
''')

replace_once(
    "Core/Src/can_handler.c",
    '''static bool            pedalcal_service_active = false;

bool CAN_PedalCalServiceActive(void)
{
    return pedalcal_service_active;
}
''',
    '''static bool            pedalcal_service_active = false;
static bool            pedalcal_service_pending = false;
static bool            pedalcal_service_hold = false;

bool CAN_PedalCalServiceActive(void)
{
    return pedalcal_service_pending || pedalcal_service_active ||
           pedalcal_service_hold;
}

bool CAN_PedalCalServiceConfirmed(void)
{
    return pedalcal_service_active;
}
''')

replace_once(
    "Core/Src/can_handler.c",
    '''    c->in_standby           = (st == SYS_STATE_STANDBY) ||
                              pedalcal_service_active;
''',
    '''    c->in_standby           = (st == SYS_STATE_STANDBY) ||
                              CAN_PedalCalServiceConfirmed();
''')
replace_once(
    "Core/Src/can_handler.c",
    '''    c->traction_inhibited   = (Traction_GetMotionInhibit() != 0U) ||
                              (st == SYS_STATE_STANDBY) ||
                              pedalcal_service_active;
''',
    '''    c->traction_inhibited   = (Traction_GetMotionInhibit() != 0U) ||
                              (st == SYS_STATE_STANDBY) ||
                              CAN_PedalCalServiceActive();
''')
replace_once(
    "Core/Src/can_handler.c",
    '''    return state_ok &&
           Safety_GetError() == SAFETY_ERROR_NONE &&
''',
    '''    return state_ok &&
           !pedalcal_service_pending && !pedalcal_service_active &&
           !pedalcal_service_hold &&
           Safety_GetError() == SAFETY_ERROR_NONE &&
''')

replace_once(
    "Core/Src/can_handler.c",
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

static void pedalcal_service_exit(void)
{
    pedalcal_service_active = false;
}
''',
    '''/* Enter through a PENDING guard so periodic relay/traction tasks cannot
 * re-energise outputs while the synchronous physical checks are in progress.
 * Only the confirmed ACTIVE phase authorizes persistence. */
static bool pedalcal_service_enter(void)
{
    PedalCalConds c;
    pedalcal_build_conds(&c);
    if (!pedalcal_service_request_allowed(&c)) return false;

    pedalcal_service_pending = true;
    Traction_SetAxisRotation(false);
    Steering_Neutralize();
    Relay_PowerDown();
    const bool en_pwm_locked = Traction_CalibrationLock();
    const bool relay_off = ((Safety_GetRelayStatusByte() & (1U << 1)) == 0U);
    if (!en_pwm_locked || !relay_off) {
        pedalcal_service_pending = false;
        pedalcal_service_hold = true;
        Relay_PowerDown();
        (void)Traction_CalibrationLock();
        return false;
    }

    pedalcal_service_active = true;
    pedalcal_service_pending = false;
    Relay_PowerDown();
    (void)Traction_CalibrationLock();
    return true;
}

/* Never release directly back to a drive-capable state.  Terminal completion,
 * operator abort, runtime abort and failed entry all transition to HOLD.  HOLD
 * keeps relays OFF and all BTS7960 outputs in COAST until P/N, stopped wheels
 * and a plausible released pedal are observed together. */
static void pedalcal_service_exit(void)
{
    pedalcal_service_pending = false;
    pedalcal_service_active = false;
    pedalcal_service_hold = true;
    Relay_PowerDown();
    (void)Traction_CalibrationLock();
}

static void pedalcal_service_hold_update(void)
{
    if (!pedalcal_service_hold) return;

    Relay_PowerDown();
    const bool output_locked = Traction_CalibrationLock();
    const bool relay_off = ((Safety_GetRelayStatusByte() & (1U << 1)) == 0U);

    PedalCalConds c;
    pedalcal_build_conds(&c);
    const bool safe_rearm = output_locked && relay_off &&
                            c.gear_park_or_neutral && !c.wheels_moving &&
                            c.pedal_plausible && c.pedal_released &&
                            !c.critical_error && !c.safe_state &&
                            !c.emergency && !c.can_loss;
    if (safe_rearm) {
        pedalcal_service_hold = false;
    }
}
''')

replace_once(
    "Core/Src/can_handler.c",
    '''    case PEDAL_CAL_OP_CAPTURE_MIN: {   /* BEGIN the guided session */
        /* Enter a physical service lock first.  This permits calibration from
''',
    '''    case PEDAL_CAL_OP_CAPTURE_MIN: {   /* BEGIN the guided session */
        /* A repeated BEGIN must never tear down an already-owned lock. */
        if (PedalCalSession_Active(&pedalcal_session)) {
            pedalcal_start_burst();
            pedalcal_send_session_status();
            CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
            return;
        }
        /* Enter a physical service lock first.  This permits calibration from
''')

replace_once(
    "Core/Src/can_handler.c",
    '''void CAN_PedalCalCaptureTick(void)
{
    pedalcal_session_lazy_init();
    if (!PedalCalSession_Active(&pedalcal_session)) return;
''',
    '''void CAN_PedalCalCaptureTick(void)
{
    pedalcal_session_lazy_init();
    pedalcal_service_hold_update();
    if (!PedalCalSession_Active(&pedalcal_session)) return;
''')

replace_once(
    "Core/Src/pedal_cal_store.c",
    '''/* Optional dependency for host fixtures; present in the productive image.
 * The service flag is published only after the physical lock is confirmed. */
extern bool CAN_PedalCalServiceActive(void) __attribute__((weak));

static bool pcal_service_lock_confirmed(void)
{
    return CAN_PedalCalServiceActive != 0 &&
           CAN_PedalCalServiceActive();
}
#include <stddef.h>
''',
    '''#include <stddef.h>

/* Optional dependency for host fixtures; present in the productive image.
 * Only the confirmed ACTIVE phase can authorize persistence. */
extern bool CAN_PedalCalServiceConfirmed(void) __attribute__((weak));

static bool pcal_service_lock_confirmed(void)
{
    return CAN_PedalCalServiceConfirmed != 0 &&
           CAN_PedalCalServiceConfirmed();
}

static bool pcal_write_context_authorized(bool standby_context)
{
    const SystemState_t state = Safety_GetState();
    if (standby_context) return state == SYS_STATE_STANDBY;
    return pcal_service_lock_confirmed() &&
           (state == SYS_STATE_ACTIVE || state == SYS_STATE_DEGRADED) &&
           Safety_GetError() == SAFETY_ERROR_NONE;
}
''')

replace_once(
    "Core/Src/pedal_cal_store.c",
    '''    const SystemState_t state = Safety_GetState();
    const bool service_authorized =
        pcal_service_lock_confirmed() &&
        (state == SYS_STATE_ACTIVE || state == SYS_STATE_DEGRADED) &&
        Safety_GetError() == SAFETY_ERROR_NONE;
    if (state != SYS_STATE_STANDBY && !service_authorized)
        return false;
''',
    '''    const bool standby_context = (Safety_GetState() == SYS_STATE_STANDBY);
    if (!pcal_write_context_authorized(standby_context))
        return false;
''')

replace_once(
    "Core/Src/pedal_cal_store.c",
    '''    /* Unlock flash */
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
''',
    '''    /* Revalidate immediately before entering the erase/program window. */
    if (!pcal_write_context_authorized(standby_context))
        return false;

    /* Unlock flash */
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
''')
replace_once(
    "Core/Src/pedal_cal_store.c",
    '''    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    /* Write the slot (double-word aligned).
''',
    '''    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }
    if (!pcal_write_context_authorized(standby_context)) {
        HAL_FLASH_Lock();
        return false;
    }

    /* Write the slot (double-word aligned).
''')
replace_once(
    "Core/Src/pedal_cal_store.c",
    '''    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
''',
    '''    for (uint32_t i = 0; i < dword_count; i++) {
        if (!pcal_write_context_authorized(standby_context)) {
            HAL_FLASH_Lock();
            return false;
        }
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
''')
replace_once(
    "Core/Src/pedal_cal_store.c",
    '''    HAL_FLASH_Lock();

    /* Update RAM state + rate-limit bookkeeping (matches the pattern
''',
    '''    HAL_FLASH_Lock();
    if (!pcal_write_context_authorized(standby_context))
        return false;

    /* Update RAM state + rate-limit bookkeeping (matches the pattern
''')

replace_once(
    "Core/Src/safety_system_patched.c",
    '''        if (heartbeat_stale || bus_off) {
            PR_EnterCanHoldover();
            return;
        }
''',
    '''        if (heartbeat_stale || bus_off) {
            recovery_pending = 0U;
            recovery_clean_since = 0U;
            PR_EnterCanHoldover();
            return;
        }
''')
replace_once(
    "Core/Src/safety_system_patched.c",
    '''                if (PR_IsCanOnlyError(safety_error)) {
                    Safety_ClearError(safety_error);
                }
            }
        } else {
''',
    '''                if (PR_IsCanOnlyError(safety_error)) {
                    Safety_ClearError(safety_error);
                }
            }
            if (pr_can_holdover_active) {
                recovery_pending = 0U;
                recovery_clean_since = 0U;
                return;
            }
        } else {
''')

replace_once(
    "esp32/src/mode_sync.h",
    '''    /* Re-arm a bounded no-response failure after an owner-controlled cooldown.
''',
    '''    /** Re-arm a bounded no-response failure after an owner-controlled cooldown.
''')
