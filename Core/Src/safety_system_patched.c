/**
  ****************************************************************************
  * @file    safety_system_patched.c
  * @brief   Field-validated safety-policy corrections layered over production.
  *
  * Corrections kept in this narrow wrapper:
  *  - drain FDCAN RX before the 250 ms watchdog decision;
  *  - keep an already-moving vehicle in its current drive state while CAN
  *    recovers, preserving the local pedal, applied gear, relay and PWM ramp;
  *  - keep wheel-sensor availability local (never global SENSOR_FAULT);
  *  - treat valid wheel-speed divergence as slip/TCS, not sensor failure;
  *  - reduce torque per slipping wheel, never all wheels globally.
  ****************************************************************************
  */

#include <stdbool.h>
#include <stdint.h>

#ifdef HOST_TEST
#include "standby_mode_sync_policy.c"
#endif

static bool pr_mask_wheel_inputs = false;

#define Wheel_GetSpeed_FL        PR_Wheel_GetSpeed_FL
#define Wheel_GetSpeed_FR        PR_Wheel_GetSpeed_FR
#define Wheel_GetSpeed_RL        PR_Wheel_GetSpeed_RL
#define Wheel_GetSpeed_RR        PR_Wheel_GetSpeed_RR
#define Wheel_IsStale            PR_Wheel_IsStale
#define Wheel_GetGpioLevel       PR_Wheel_GetGpioLevel

/* Rename only the productive entry points replaced below.  Internal safety
 * calls keep their legacy names inside this translation unit; external calls
 * (notably CAN_CheckBusOff()) are routed through the wrappers defined later. */
#define Safety_SetError            Safety_SetError_Legacy
#define Safety_SetState            Safety_SetState_Legacy
#define Safety_CheckRelayHealth    Safety_CheckRelayHealth_Legacy
#define Relay_SequencerUpdate      Relay_SequencerUpdate_Legacy
#define Safety_RelayOverrideUpdate Safety_RelayOverrideUpdate_Legacy
#define Safety_CheckCANTimeout     Safety_CheckCANTimeout_Legacy
#define Safety_CheckSensors        Safety_CheckSensors_Legacy
#define TCS_Update                 TCS_Update_Legacy
#include "safety_system.c"
#undef TCS_Update
#undef Safety_CheckSensors
#undef Safety_CheckCANTimeout
#undef Safety_RelayOverrideUpdate
#undef Relay_SequencerUpdate
#undef Safety_CheckRelayHealth
#undef Safety_SetState
#undef Safety_SetError

#undef Wheel_GetGpioLevel
#undef Wheel_IsStale
#undef Wheel_GetSpeed_RR
#undef Wheel_GetSpeed_RL
#undef Wheel_GetSpeed_FR
#undef Wheel_GetSpeed_FL

/* sensor_manager.h was seen with the forwarding macros active. */
extern float    Wheel_GetSpeed_FL(void);
extern float    Wheel_GetSpeed_FR(void);
extern float    Wheel_GetSpeed_RL(void);
extern float    Wheel_GetSpeed_RR(void);
extern bool     Wheel_IsStale(uint8_t idx);
extern uint8_t  Wheel_GetGpioLevel(uint8_t idx);
#ifndef HOST_TEST
extern uint32_t Wheel_GetPulseCount(uint8_t idx);
#else
/* Safety integration tests stub the historic wheel API but do not need the new
 * pulse-recovery evidence.  A weak zero source keeps those tests linkable;
 * any test that supplies a real/strong Wheel_GetPulseCount overrides it. */
__attribute__((weak)) uint32_t Wheel_GetPulseCount(uint8_t idx)
{
    (void)idx;
    return 0U;
}
#endif

float PR_Wheel_GetSpeed_FL(void)
{
    return pr_mask_wheel_inputs ? 0.0f : Wheel_GetSpeed_FL();
}

float PR_Wheel_GetSpeed_FR(void)
{
    return pr_mask_wheel_inputs ? 0.0f : Wheel_GetSpeed_FR();
}

float PR_Wheel_GetSpeed_RL(void)
{
    return pr_mask_wheel_inputs ? 0.0f : Wheel_GetSpeed_RL();
}

float PR_Wheel_GetSpeed_RR(void)
{
    return pr_mask_wheel_inputs ? 0.0f : Wheel_GetSpeed_RR();
}

bool PR_Wheel_IsStale(uint8_t idx)
{
    return pr_mask_wheel_inputs ? false : Wheel_IsStale(idx);
}

uint8_t PR_Wheel_GetGpioLevel(uint8_t idx)
{
    return pr_mask_wheel_inputs ? 1U : Wheel_GetGpioLevel(idx);
}

/* -------------------------------------------------------------------------
 * Seamless CAN holdover
 * -------------------------------------------------------------------------
 * The STM32 owns the physical pedal, traction control, gear already applied,
 * wheel/current/temperature protection and the traction relay.  Therefore a
 * temporary loss of the ESP32/HMI link is not, by itself, a reason to make a
 * running vehicle jump from >40 % demand to the old 40 % LIMP_HOME ceiling.
 *
 * Policy while already ACTIVE/DEGRADED:
 *   - remain in the same state and preserve the current pedal/PWM ramp;
 *   - RC override expires independently after 200 ms and control falls back to
 *     the physical STM32 pedal;
 *   - freeze the already-applied gear/mode (no new CAN command can arrive);
 *   - report MODULE_CAN_TIMEOUT as a diagnostic warning;
 *   - continue FDCAN bus-off recovery in the background;
 *   - clear the warning only after 500 ms of stable heartbeat and no bus-off.
 *
 * Real hardware hazards (overcurrent, battery, temperature, pedal
 * contradiction, obstacle emergency, EPS electrical hazard, emergency stop)
 * retain all of their existing state transitions.  Only CAN-only requests to
 * enter LIMP_HOME are suppressed for a vehicle that was already drive-capable.
 */
#define PR_CAN_RECOVERY_STABLE_MS 500U

static bool          pr_can_holdover_active          = false;
static bool          pr_can_limp_transition_pending  = false;
static uint32_t      pr_can_recovery_since           = 0U;
static SystemState_t pr_can_holdover_origin          = SYS_STATE_ACTIVE;

static bool PR_IsDriveState(SystemState_t state)
{
    return state == SYS_STATE_ACTIVE || state == SYS_STATE_DEGRADED;
}

static bool PR_IsCanOnlyError(Safety_Error_t error)
{
    return error == SAFETY_ERROR_CAN_TIMEOUT ||
           error == SAFETY_ERROR_CAN_BUSOFF;
}

static bool PR_IsBusOffNow(void)
{
#ifndef HOST_TEST
    return CAN_IsBusOff();
#else
    return false;
#endif
}

static void PR_EnterCanHoldover(void)
{
    if (!pr_can_holdover_active) {
        pr_can_holdover_origin = system_state;
    }
    pr_can_holdover_active = true;
    pr_can_recovery_since = 0U;
    ServiceMode_SetFault(MODULE_CAN_TIMEOUT, MODULE_FAULT_WARNING);
}

/* External CAN paths call Safety_SetError() before requesting LIMP_HOME.  Keep
 * CAN loss diagnostic-only while the vehicle is already moving, but delegate
 * every non-CAN fault unchanged to the productive safety implementation. */
void Safety_SetError(Safety_Error_t error)
{
    if (PR_IsCanOnlyError(error) && PR_IsDriveState(system_state)) {
        PR_EnterCanHoldover();
        pr_can_limp_transition_pending = true;
        return;
    }

    pr_can_limp_transition_pending = false;
    Safety_SetError_Legacy(error);
}

/* Consume only the LIMP_HOME transition paired with the CAN-only error above.
 * This precise one-shot guard cannot hide a pedal/sensor/electrical transition
 * that happens during the same holdover: any non-CAN Safety_SetError() clears
 * the pending flag and the legacy state transition proceeds normally. */
void Safety_SetState(SystemState_t new_state)
{
    if (new_state == SYS_STATE_LIMP_HOME &&
        pr_can_limp_transition_pending &&
        PR_IsDriveState(system_state)) {
        pr_can_limp_transition_pending = false;
        PR_EnterCanHoldover();
        return;
    }

    pr_can_limp_transition_pending = false;
    Safety_SetState_Legacy(new_state);
}

/* Consume any heartbeat already waiting in FIFO0 before evaluating its age.
 * During a running CAN-only outage, never change state, gear, demand or relay.
 * STANDBY/SAFE/LIMP_HOME and all non-CAN recovery behaviour remain delegated to
 * the productive implementation. */
void Safety_CheckCANTimeout(void)
{
#ifndef HOST_TEST
    CAN_ProcessMessages();
    CAN_TxPump();
#endif

    const uint32_t now = HAL_GetTick();
    const bool heartbeat_stale = (now - last_can_rx_time) > CAN_TIMEOUT_MS;
    const bool bus_off = PR_IsBusOffNow();

    if (PR_IsDriveState(system_state) || pr_can_holdover_active) {
        if (heartbeat_stale || bus_off) {
            PR_EnterCanHoldover();
            return;
        }

        /* Link is physically back.  Keep the current state and torque while
         * confirming stability; no NEUTRAL injection and no ramp reset. */
        if (pr_can_holdover_active) {
            if (pr_can_recovery_since == 0U) {
                pr_can_recovery_since = now;
            } else if ((now - pr_can_recovery_since) >=
                       PR_CAN_RECOVERY_STABLE_MS) {
                pr_can_holdover_active = false;
                pr_can_limp_transition_pending = false;
                pr_can_recovery_since = 0U;
                ServiceMode_ClearFault(MODULE_CAN_TIMEOUT);

                /* Defensive cleanup for a CAN error left by an older path or
                 * an earlier firmware image; non-CAN errors are untouched. */
                if (PR_IsCanOnlyError(safety_error)) {
                    Safety_ClearError(safety_error);
                }
            }
        } else {
            ServiceMode_ClearFault(MODULE_CAN_TIMEOUT);
            if (PR_IsCanOnlyError(safety_error)) {
                Safety_ClearError(safety_error);
            }
        }
        return;
    }

    /* Not an already-running vehicle: preserve the established boot, STANDBY,
     * SAFE and genuine LIMP_HOME policies exactly. */
    Safety_CheckCANTimeout_Legacy();
}

#define PR_WHEEL_RECOVERY_PULSES  2U
static uint32_t pr_wheel_pulse_at_fault[NUM_WHEELS] = {0};

static float PR_WheelSpeed(uint8_t idx)
{
    switch (idx) {
        case 0U: return Wheel_GetSpeed_FL();
        case 1U: return Wheel_GetSpeed_FR();
        case 2U: return Wheel_GetSpeed_RL();
        case 3U: return Wheel_GetSpeed_RR();
        default: return 0.0f;
    }
}

static bool PR_WheelIsDriven(uint8_t idx)
{
    const TractionState_t *ts = Traction_GetState();
    if (ts == NULL || idx >= NUM_WHEELS) return false;
    if (ts->mode4x4 || ts->axisRotation) return true;

    /* Follow the mode actually implemented today.  The separate 4x2 rear-drive
     * conversion must change motor_control and this helper in the same commit. */
    return idx < 2U;
}

static bool PR_IsWheelReason(WheelDiag_t reason)
{
    return reason == WHEEL_DIAG_NO_PULSE      ||
           reason == WHEEL_DIAG_STUCK_HIGH    ||
           reason == WHEEL_DIAG_STUCK_LOW     ||
           reason == WHEEL_DIAG_MISMATCH      ||
           reason == WHEEL_DIAG_IMPOSSIBLE_RATE;
}

static void PR_UpdateWheelDiagnostics(void)
{
    const bool powertrain = Safety_PowertrainEngaged();
    const bool pwm_active = Traction_GetFinalPwmPct() > 3U;
    const uint32_t now = HAL_GetTick();
    float spd[NUM_WHEELS];
    bool any_moving = false;

    for (uint8_t i = 0U; i < NUM_WHEELS; ++i) {
        spd[i] = PR_WheelSpeed(i);
        if (isfinite(spd[i]) && spd[i] > 1.0f) any_moving = true;
    }

    for (uint8_t i = 0U; i < NUM_WHEELS; ++i) {
        const ModuleID_t mod = (ModuleID_t)(MODULE_WHEEL_SPEED_FL + i);

        if (!ServiceMode_IsEnabled(mod)) {
            wheel_diag[i] = WHEEL_DIAG_DISABLED_STATE;
            wheel_mismatch_since[i] = 0U;
            continue;
        }

        if (!isfinite(spd[i]) || spd[i] < 0.0f ||
            spd[i] > SENSOR_SPEED_MAX_KMH) {
            ServiceMode_SetFault(mod, MODULE_FAULT_ERROR);
            wheel_diag[i] = WHEEL_DIAG_IMPOSSIBLE_RATE;
            wheel_latched_reason[i] = WHEEL_DIAG_IMPOSSIBLE_RATE;
            wheel_mismatch_since[i] = 0U;
            pr_wheel_pulse_at_fault[i] = Wheel_GetPulseCount(i);
            continue;
        }

        const bool stale = Wheel_IsStale(i) && spd[i] == 0.0f;
        const bool driven = PR_WheelIsDriven(i);
        const bool motion_expected = powertrain && pwm_active && driven;

        if (stale && (any_moving || motion_expected)) {
            if (!powertrain) {
                wheel_diag[i] = WHEEL_DIAG_MANUAL_MOVEMENT;
                wheel_mismatch_since[i] = 0U;
            } else if (!driven) {
                wheel_diag[i] = WHEEL_DIAG_OK;
                wheel_mismatch_since[i] = 0U;
            } else {
                if (wheel_mismatch_since[i] == 0U) {
                    wheel_mismatch_since[i] = now;
                }
                if ((now - wheel_mismatch_since[i]) >= WHEEL_FAULT_DEBOUNCE_MS) {
                    const uint8_t level = Wheel_GetGpioLevel(i);
                    wheel_diag[i] = (level == 1U) ? WHEEL_DIAG_STUCK_HIGH :
                                    (level == 0U) ? WHEEL_DIAG_STUCK_LOW  :
                                                    WHEEL_DIAG_NO_PULSE;
                    wheel_latched_reason[i] = wheel_diag[i];
                    pr_wheel_pulse_at_fault[i] = Wheel_GetPulseCount(i);
                    /* Local warning only: ABS/TCS exclude the channel; the
                     * vehicle, pedal and gripping wheels remain available. */
                    ServiceMode_SetFault(mod, MODULE_FAULT_WARNING);
                } else {
                    wheel_diag[i] = WHEEL_DIAG_MISMATCH;
                }
            }
            continue;
        }

        /* Valid pulses plus a faster wheel means real slip.  Do not fault it. */
        wheel_diag[i] = WHEEL_DIAG_OK;
        wheel_mismatch_since[i] = 0U;

        if (ServiceMode_GetFault(mod) != MODULE_FAULT_NONE &&
            PR_IsWheelReason(wheel_latched_reason[i])) {
            const uint32_t pulse_now = Wheel_GetPulseCount(i);
            if ((uint32_t)(pulse_now - pr_wheel_pulse_at_fault[i]) >=
                PR_WHEEL_RECOVERY_PULSES) {
                ServiceMode_ClearFault(mod);
                wheel_latched_reason[i] = WHEEL_DIAG_OK;
                pr_wheel_pulse_at_fault[i] = pulse_now;
            }
        }
    }
}

void Safety_CheckSensors(void)
{
    /* Preserve production temperature/current/pedal checks.  Mask wheel values
     * only inside the legacy aggregate so they cannot create a global DTC. */
    pr_mask_wheel_inputs = true;
    Safety_CheckSensors_Legacy();
    pr_mask_wheel_inputs = false;
    PR_UpdateWheelDiagnostics();
}

static bool PR_WheelHealthyDriven(uint8_t idx)
{
    const ModuleID_t mod = (ModuleID_t)(MODULE_WHEEL_SPEED_FL + idx);
    return PR_WheelIsDriven(idx) &&
           ServiceMode_IsEnabled(mod) &&
           ServiceMode_GetFault(mod) == MODULE_FAULT_NONE;
}

static float PR_RobustReference(const float spd[NUM_WHEELS], uint8_t candidate)
{
    float v[NUM_WHEELS - 1U];
    uint8_t n = 0U;

    for (uint8_t i = 0U; i < NUM_WHEELS; ++i) {
        if (i == candidate || !PR_WheelHealthyDriven(i)) continue;
        if (!isfinite(spd[i]) || spd[i] < 0.0f) continue;
        v[n++] = spd[i];
    }

    if (n == 0U) return 0.0f;
    if (n == 1U) return v[0];

    for (uint8_t i = 1U; i < n; ++i) {
        const float key = v[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && v[j] > key) {
            v[j + 1] = v[j];
            --j;
        }
        v[j + 1] = key;
    }

    return (n == 2U) ? v[0] : v[n / 2U];
}

void TCS_Update(void)
{
    if (!ServiceMode_IsEnabled(MODULE_TCS) || !Safety_PowertrainEngaged()) {
        safety_status.tcs_active = false;
        safety_status.tcs_wheel_mask = 0U;
        for (uint8_t i = 0U; i < NUM_WHEELS; ++i) tcs_reduction[i] = 0.0f;
        tcs_last_tick = HAL_GetTick();
        return;
    }

    float spd[NUM_WHEELS] = {
        Wheel_GetSpeed_FL(), Wheel_GetSpeed_FR(),
        Wheel_GetSpeed_RL(), Wheel_GetSpeed_RR()
    };

    const uint32_t now = HAL_GetTick();
    float dt = (float)(now - tcs_last_tick) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;
    tcs_last_tick = now;

    uint8_t slip_mask = 0U;
    for (uint8_t i = 0U; i < NUM_WHEELS; ++i) {
        if (!PR_WheelHealthyDriven(i)) {
            tcs_reduction[i] = 0.0f;
            continue;
        }

        const float reference = PR_RobustReference(spd, i);
        bool slipping = false;
        if (reference >= 3.0f && isfinite(spd[i])) {
            const float slip_pct = ((spd[i] - reference) * 100.0f) / reference;
            slipping = slip_pct > (float)TCS_SLIP_THRESHOLD;
        }

        if (slipping) {
            slip_mask |= (uint8_t)(1U << i);
            if (tcs_reduction[i] < 0.01f) {
                tcs_reduction[i] = TCS_INITIAL_REDUCTION;
            } else {
                tcs_reduction[i] += TCS_SMOOTH_REDUCTION;
            }
            if (tcs_reduction[i] > TCS_MAX_REDUCTION) {
                tcs_reduction[i] = TCS_MAX_REDUCTION;
            }
        } else if (tcs_reduction[i] > 0.0f) {
            tcs_reduction[i] -= TCS_RECOVERY_RATE_PER_S * dt;
            if (tcs_reduction[i] < 0.0f) tcs_reduction[i] = 0.0f;
        }

        const float tcs_scale = 1.0f - tcs_reduction[i];
        if (tcs_scale < safety_status.wheel_scale[i]) {
            safety_status.wheel_scale[i] = tcs_scale;
        }
    }

    safety_status.tcs_active = slip_mask != 0U;
    safety_status.tcs_wheel_mask = slip_mask;
    if (slip_mask != 0U) sat_inc_u32(&safety_status.tcs_activation_count);

    /* No global Traction_SetDemand(): only the slipping corner is reduced. */
}

void Safety_CheckRelayHealth(void)
{
    Safety_UpdateRelayHealthDiag();
    relay_chk_active         = 0U;
    relay_chk_debounce       = 0U;
    relay_chk_recovery_tick  = 0U;
    relay_chk_fault_set_tick = 0U;
}

void Relay_SequencerUpdate(void)
{
    const bool drive_capable =
        (system_state == SYS_STATE_ACTIVE)   ||
        (system_state == SYS_STATE_DEGRADED) ||
        (system_state == SYS_STATE_LIMP_HOME);

    if (!drive_capable) {
        if (relay_seq_state == RELAY_SEQ_TRACTION_ON ||
            relay_seq_state == RELAY_SEQ_COMPLETE) Relay_PowerDown();
        return;
    }

    if (relay_seq_state == RELAY_SEQ_IDLE) Relay_PowerUp();

    const uint32_t now = HAL_GetTick();
    if (relay_seq_state == RELAY_SEQ_TRACTION_ON &&
        (now - relay_seq_timestamp) >= RELAY_TRACTION_SETTLE_MS) {
        HAL_GPIO_WritePin(GPIOC, PIN_RELAY_STEER_PWR,
            Steering_MotorRelayAllowed() ? GPIO_PIN_SET : GPIO_PIN_RESET);
        relay_seq_state = RELAY_SEQ_COMPLETE;
    }

    if (relay_seq_state == RELAY_SEQ_COMPLETE) {
        HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, PIN_RELAY_STEER_PWR,
            Steering_MotorRelayAllowed() ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void Safety_RelayOverrideUpdate(void)
{
    if (Steering_IsAssistLatchedOff()) {
        relay_override_mask &= (uint8_t)~0x04U;
    }
    Safety_RelayOverrideUpdate_Legacy();
    if (Steering_IsAssistLatchedOff()) {
        GPIOC->BSRR = (uint32_t)PIN_RELAY_STEER_PWR << 16U;
    }
}
