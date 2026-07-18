/**
  ****************************************************************************
  * @file    safety_system_patched.c
  * @brief   Field-validated safety-policy corrections layered over production.
  *
  * The production safety_system.c is included once and only the audited entry
  * points below are replaced.  This keeps unrelated battery, thermal, obstacle,
  * relay and EPS behaviour byte-for-byte while fixing three reproduced faults:
  *
  *  - CAN RX is drained immediately before evaluating the 250 ms watchdog, so
  *    a heartbeat already waiting in FDCAN FIFO cannot be reported as TIMEOUT
  *    merely because the cooperative loop was delayed by I2C/OneWire work.
  *  - Wheel-speed availability faults remain local diagnostics; wheel slip is
  *    TCS activity, never a global SENSOR_FAULT/DEGRADED transition.
  *  - TCS reduces only the slipping wheel(s).  It never calls the global
  *    Traction_SetDemand() fallback that removed current from every wheel.
  ****************************************************************************
  */

#ifdef HOST_TEST
#include "standby_mode_sync_policy.c"
#endif

/* During the legacy non-wheel plausibility pass, mask only wheel inputs.  The
 * public wrapper runs a dedicated fail-operational wheel diagnostic afterwards.
 * All ABS/TCS and telemetry calls outside that narrow window still see the real
 * wheel values through the forwarding functions defined below. */
static bool pr_mask_wheel_inputs = false;

#define Wheel_GetSpeed_FL        PR_Wheel_GetSpeed_FL
#define Wheel_GetSpeed_FR        PR_Wheel_GetSpeed_FR
#define Wheel_GetSpeed_RL        PR_Wheel_GetSpeed_RL
#define Wheel_GetSpeed_RR        PR_Wheel_GetSpeed_RR
#define Wheel_IsStale            PR_Wheel_IsStale
#define Wheel_GetGpioLevel       PR_Wheel_GetGpioLevel

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

#undef Wheel_GetGpioLevel
#undef Wheel_IsStale
#undef Wheel_GetSpeed_RR
#undef Wheel_GetSpeed_RL
#undef Wheel_GetSpeed_FR
#undef Wheel_GetSpeed_FL

/* Real sensor-manager entry points.  sensor_manager.h was included above while
 * the forwarding macros were active, so declare the unmapped names explicitly. */
extern float    Wheel_GetSpeed_FL(void);
extern float    Wheel_GetSpeed_FR(void);
extern float    Wheel_GetSpeed_RL(void);
extern float    Wheel_GetSpeed_RR(void);
extern bool     Wheel_IsStale(uint8_t idx);
extern uint8_t  Wheel_GetGpioLevel(uint8_t idx);
extern uint32_t Wheel_GetPulseCount(uint8_t idx);

/* Forwarders used by every legacy function.  Only Safety_CheckSensors() sets
 * pr_mask_wheel_inputs, preventing wheel mismatch from contributing to the
 * global non-wheel fault_count while preserving real inputs everywhere else. */
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

/* ------------------------------------------------------------------------- */
/* CAN watchdog: consume queued RX before judging heartbeat age.              */
/* ------------------------------------------------------------------------- */
void Safety_CheckCANTimeout(void)
{
#ifndef HOST_TEST
    /* CAN_ProcessMessages() is non-blocking and drains FIFO0.  Calling it here
     * closes the reproduced ordering bug:
     *
     *   slow I2C/OneWire work -> heartbeat waits in FIFO -> watchdog ran first
     *   -> false CAN_TIMEOUT despite a physically healthy bus.
     *
     * A truly silent/bus-off link still has no frame to consume and therefore
     * reaches the unchanged legacy timeout policy. */
    CAN_ProcessMessages();
    CAN_TxPump();
#endif
    Safety_CheckCANTimeout_Legacy();
}

/* ------------------------------------------------------------------------- */
/* Wheel sensors: local availability diagnostics, no global degradation.      */
/* ------------------------------------------------------------------------- */
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

    /* This helper follows the MODE ACTUALLY APPLIED by the current traction
     * implementation.  The separately requested conversion of 4x2 to rear
     * propulsion must change motor_control and this helper together; it is not
     * silently faked inside diagnostics. */
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
    for (uint8_t i = 0U; i < NUM_WHEELS; ++i) {
        spd[i] = PR_WheelSpeed(i);
    }

    bool any_moving = false;
    for (uint8_t i = 0U; i < NUM_WHEELS; ++i) {
        if (isfinite(spd[i]) && spd[i] > 1.0f) {
            any_moving = true;
            break;
        }
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
                /* Hand movement / bench rotation is information, not a fault. */
                wheel_diag[i] = WHEEL_DIAG_MANUAL_MOVEMENT;
                wheel_mismatch_since[i] = 0U;
            } else if (!driven) {
                /* A non-driven wheel may legitimately remain still. */
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

                    /* Local availability warning only.  ABS/TCS exclude this
                     * channel through ServiceMode_GetFault(), but vehicle state,
                     * pedal and the other wheels remain fully available. */
                    ServiceMode_SetFault(mod, MODULE_FAULT_WARNING);
                } else {
                    wheel_diag[i] = WHEEL_DIAG_MISMATCH;
                }
            }
            continue;
        }

        /* A wheel that is faster than the others but still emits valid pulses is
         * SLIP.  It is deliberately left fault-free for per-wheel TCS below. */
        wheel_diag[i] = WHEEL_DIAG_OK;
        wheel_mismatch_since[i] = 0U;

        const ModuleFault_t fault = ServiceMode_GetFault(mod);
        if (fault != MODULE_FAULT_NONE &&
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
    /* Execute all productive temperature/current/pedal checks unchanged, but
     * present a stationary, coherent wheel snapshot to that legacy aggregate so
     * wheel availability can never increment its global fault_count. */
    pr_mask_wheel_inputs = true;
    Safety_CheckSensors_Legacy();
    pr_mask_wheel_inputs = false;

    /* Then run the wheel policy independently: local module diagnostics only. */
    PR_UpdateWheelDiagnostics();
}

/* ------------------------------------------------------------------------- */
/* Per-wheel virtual differential TCS: no global all-wheel current cut.       */
/* ------------------------------------------------------------------------- */
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

    /* With two references choose the slower/gripping wheel; with three use the
     * median so one additional spinning wheel cannot inflate the baseline. */
    return (n == 2U) ? v[0] : v[n / 2U];
}

void TCS_Update(void)
{
    if (!ServiceMode_IsEnabled(MODULE_TCS) || !Safety_PowertrainEngaged()) {
        safety_status.tcs_active = false;
        safety_status.tcs_wheel_mask = 0U;
        for (uint8_t i = 0U; i < NUM_WHEELS; ++i) {
            tcs_reduction[i] = 0.0f;
        }
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

        /* ABS runs first.  The lower scale wins, independently per wheel. */
        const float tcs_scale = 1.0f - tcs_reduction[i];
        if (tcs_scale < safety_status.wheel_scale[i]) {
            safety_status.wheel_scale[i] = tcs_scale;
        }
    }

    safety_status.tcs_active = slip_mask != 0U;
    safety_status.tcs_wheel_mask = slip_mask;
    if (slip_mask != 0U) {
        sat_inc_u32(&safety_status.tcs_activation_count);
    }

    /* Intentionally NO global Traction_SetDemand() here.  One slipping wheel
     * loses torque only on that corner; every healthy gripping wheel keeps its
     * requested current within the existing pedal/current/thermal limits. */
}

/* ------------------------------------------------------------------------- */
/* Existing relay/EPS corrections.                                           */
/* ------------------------------------------------------------------------- */
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
            relay_seq_state == RELAY_SEQ_COMPLETE) {
            Relay_PowerDown();
        }
        return;
    }

    if (relay_seq_state == RELAY_SEQ_IDLE) {
        Relay_PowerUp();
    }

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
