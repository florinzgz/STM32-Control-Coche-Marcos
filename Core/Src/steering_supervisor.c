/**
  ****************************************************************************
  * @file    steering_supervisor.c
  * @brief   EPS assist supervisor decision logic + real-EPS driver.
  *
  * See steering_supervisor.h for the policy contract.  The decision helpers
  * are pure; SteeringSupervisor_Apply() drives the idempotent steering_eps.c
  * isolation API and never touches the traction chain.
  ****************************************************************************
  */

#include "steering_supervisor.h"
#include "steering_z.h"   /* SteeringZStatus_t values only */

#include <stdlib.h>       /* labs */

/* ======================================================================
 *  Pure decision helpers
 * ====================================================================== */

EpsFaultReason_t SteeringSupervisor_Ch5ToEpsFault(Ina226DiagReason_t reason)
{
    switch (reason) {
        /* No usable current sensor: the MUX cannot select the channel, the
         * chip does not ACK, or its measurement registers cannot be read.
         * All three mean "CH5 absent" for policy purposes.                  */
        case INA226_CH_MUX_SELECT_FAIL:
        case INA226_CH_MISSING:
        case INA226_CH_READ_FAIL:
            return EPS_FAULT_CH5_MISSING;

        /* Telemetry not refreshed within the freshness window. */
        case INA226_CH_STALE:
            return EPS_FAULT_CH5_STALE;

        /* Configuration invalid: config write/readback mismatch, or the chip
         * ACKs but reports the wrong manufacturer/die identity.             */
        case INA226_CH_CONFIG_LOST:
        case INA226_CH_WRONG_ID:
            return EPS_FAULT_CH5_CONFIG;

        /* VIN+/VIN- reversed — the current sign is inverted, so an assist
         * command would be measured/acted upon with the wrong polarity.     */
        case INA226_CH_POLARITY_REVERSED:
            return EPS_FAULT_DIRECTION_POLARITY;

        /* Benign: healthy, or present-but-idle (no shunt drop while no
         * current is expected) — not an isolation cause.                    */
        case INA226_CH_OK:
        case INA226_CH_PRESENT_NO_SHUNT:
        case INA226_CH_UNKNOWN:
        default:
            return EPS_FAULT_NONE;
    }
}

EpsFaultReason_t SteeringSupervisor_ParamsPolicy(bool flash_present,
                                                 bool flash_valid)
{
    /* A persisted slot that exists but is structurally corrupt / bad CRC must
     * NOT silently fall back to defaults: isolate the assist so the operator
     * re-provisions the parameters.                                          */
    if (flash_present && !flash_valid) {
        return EPS_FAULT_PARAMETERS_INVALID;
    }
    /* Fresh device (no slot) → compiled defaults are authorised, no fault. */
    return EPS_FAULT_NONE;
}

EpsFaultReason_t SteeringSupervisor_CalPolicy(bool flash_present,
                                              bool flash_corrupt,
                                              bool cal_valid,
                                              bool centering_finished,
                                              bool centering_recovered)
{
    /* A calibration slot that is present but corrupt is an existing-flash
     * corruption — isolate rather than run on an unknown centre.            */
    if (flash_present && flash_corrupt) {
        return EPS_FAULT_CALIBRATION_INVALID;
    }
    /* Otherwise the calibration is only "invalid" once the homing FSM has had
     * its chance and finished WITHOUT recovering a valid centre.            */
    if (centering_finished && !centering_recovered && !cal_valid) {
        return EPS_FAULT_CALIBRATION_INVALID;
    }
    return EPS_FAULT_NONE;
}

EpsFaultReason_t SteeringSupervisor_ZPolicy(bool z_required,
                                            bool center_known,
                                            int z_status)
{
    if (!z_required) {
        return EPS_FAULT_NONE;   /* optional Z: diagnostic only */
    }
    /* Z only becomes meaningful once PB5 confirms the centre. */
    if (!center_known) {
        return EPS_FAULT_NONE;
    }
    switch (z_status) {
        case STEERING_Z_NOT_SEEN:      /* mandatory but no Z pulse captured  */
        case STEERING_Z_OUT_OF_WINDOW: /* Z disagrees with PB5 (incoherent)  */
        case STEERING_Z_MECH_OFFSET:   /* Z strongly disagrees               */
            return EPS_FAULT_ENCODER_Z;
        case STEERING_Z_OK:
        case STEERING_Z_NOT_CALIBRATED:
        default:
            return EPS_FAULT_NONE;
    }
}

/* ======================================================================
 *  Overcurrent state machine
 * ====================================================================== */

void SteeringSupervisor_OcInit(OcFsm_t *f, int32_t limit_ma, uint32_t confirm_ms)
{
    if (f == NULL) return;
    f->state             = OC_STATE_NORMAL;
    f->limit_ma          = limit_ma;
    f->confirm_ms        = confirm_ms;
    f->isolate_sample_id = 0U;
    f->isolate_tick_ms   = 0U;
}

OcAction_t SteeringSupervisor_OcStep(OcFsm_t *f, int32_t current_ma,
                                     uint32_t sample_id, bool sample_valid,
                                     uint32_t now_ms)
{
    if (f == NULL) return OC_ACTION_NONE;

    const bool over = (labs((long)current_ma) > (long)f->limit_ma);

    switch (f->state) {
        case OC_STATE_NORMAL:
            /* Only a VALID sample above the ceiling triggers isolation. */
            if (sample_valid && over) {
                f->state             = OC_STATE_CONFIRM;
                f->isolate_sample_id = sample_id;
                f->isolate_tick_ms   = now_ms;
                return OC_ACTION_ISOLATE;
            }
            return OC_ACTION_NONE;

        case OC_STATE_CONFIRM: {
            /* Non-blocking wait for a genuinely NEW valid CH5 sample. */
            const bool fresh = sample_valid && (sample_id != f->isolate_sample_id);
            if (fresh) {
                if (over) {
                    /* Current did not disappear once the motor was isolated:
                     * this is a non-isolable electrical hazard.             */
                    f->state = OC_STATE_HAZARD;
                    return OC_ACTION_ESCALATE_HAZARD;
                }
                /* Current fell — the isolation cleared it; stay mechanical. */
                f->state = OC_STATE_MECHANICAL;
                return OC_ACTION_KEEP_MECHANICAL;
            }
            /* No fresh confirmation within the window → conservatively treat
             * the danger as unconfirmed-cleared, i.e. a hazard.            */
            if ((uint32_t)(now_ms - f->isolate_tick_ms) >= f->confirm_ms) {
                f->state = OC_STATE_HAZARD;
                return OC_ACTION_ESCALATE_HAZARD;
            }
            return OC_ACTION_NONE;
        }

        case OC_STATE_MECHANICAL:
        case OC_STATE_HAZARD:
        default:
            return OC_ACTION_NONE;   /* terminal — latched */
    }
}

/* ======================================================================
 *  Stateful supervisor
 * ====================================================================== */

static OcFsm_t          s_oc;
static bool             s_want_safe;
static EpsFaultReason_t s_last_cause;
static uint8_t          s_ch5_debounce;

void SteeringSupervisor_Init(void)
{
    SteeringSupervisor_OcInit(&s_oc, STEERING_OC_LIMIT_MA, STEERING_OC_CONFIRM_MS);
    s_want_safe    = false;
    s_last_cause   = EPS_FAULT_NONE;
    s_ch5_debounce = 0U;
}

/* Isolate on the first non-NONE cause; keep the first cause latched by
 * steering_eps.c itself (Steering_DisableAssistFault records only the first). */
static void supervisor_isolate(EpsFaultReason_t cause)
{
    if (cause != EPS_FAULT_NONE) {
        Steering_DisableAssistFault(cause);
        if (s_last_cause == EPS_FAULT_NONE) {
            s_last_cause = cause;
        }
    }
}

void SteeringSupervisor_Apply(const SteeringSupervisorInputs *in)
{
    if (in == NULL) return;

    /* --- (4) parameter store: fresh defaults vs corrupt flash ------------ */
    supervisor_isolate(SteeringSupervisor_ParamsPolicy(in->params_flash_present,
                                                       in->params_flash_valid));

    /* --- (5) persistent invalid calibration ------------------------------ */
    supervisor_isolate(SteeringSupervisor_CalPolicy(in->cal_flash_present,
                                                    in->cal_flash_corrupt,
                                                    in->cal_valid,
                                                    in->centering_finished,
                                                    in->centering_recovered));

    /* --- (3) encoder-Z policy ------------------------------------------- */
    supervisor_isolate(SteeringSupervisor_ZPolicy(in->z_required,
                                                  in->z_center_known,
                                                  in->z_status));

    /* --- (1) INA226 CH5 diagnosis --------------------------------------- *
     * Debounced so a single transient I2C hiccup never isolates the assist. */
    {
        EpsFaultReason_t ch5_cause =
            SteeringSupervisor_Ch5ToEpsFault(in->ch5_reason);
        if (ch5_cause != EPS_FAULT_NONE) {
            if (s_ch5_debounce < 0xFFU) s_ch5_debounce++;
            if (s_ch5_debounce >= STEERING_CH5_FAULT_DEBOUNCE) {
                supervisor_isolate(ch5_cause);
            }
        } else {
            s_ch5_debounce = 0U;
        }
    }

    /* --- (2) overcurrent state machine ---------------------------------- */
    {
        OcAction_t act = SteeringSupervisor_OcStep(&s_oc, in->ch5_current_ma,
                                                   in->ch5_sample_id,
                                                   in->ch5_sample_valid,
                                                   in->now_ms);
        switch (act) {
            case OC_ACTION_ISOLATE:
                /* Isolate PA6/PA7/PC4/PC12 via the shared idempotent path. */
                Steering_DisableAssistFault(EPS_FAULT_OVERCURRENT);
                if (s_last_cause == EPS_FAULT_NONE) {
                    s_last_cause = EPS_FAULT_OVERCURRENT;
                }
                break;
            case OC_ACTION_KEEP_MECHANICAL:
                /* Current disappeared once isolated — remain mechanical-only.
                 * The assist is already latched off; nothing more to do.    */
                break;
            case OC_ACTION_ESCALATE_HAZARD:
                /* Danger persists after isolation: non-isolable hazard. */
                Steering_DeclareElectricalHazard(EPS_FAULT_OVERCURRENT);
                s_want_safe = true;
                if (s_last_cause == EPS_FAULT_NONE) {
                    s_last_cause = EPS_FAULT_OVERCURRENT;
                }
                break;
            case OC_ACTION_NONE:
            default:
                break;
        }
    }
}

bool SteeringSupervisor_WantsSafe(void)      { return s_want_safe; }
EpsFaultReason_t SteeringSupervisor_LastCause(void) { return s_last_cause; }
OcState_t SteeringSupervisor_OcState(void)   { return s_oc.state; }
