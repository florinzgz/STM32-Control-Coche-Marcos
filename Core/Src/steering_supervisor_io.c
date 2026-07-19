/**
  ****************************************************************************
  * @file    steering_supervisor_io.c
  * @brief   Production data-gathering glue for the EPS assist supervisor.
  *
  * Reads the REAL steering detectors (INA226 CH5 diagnosis, EPS parameter
  * store, steering calibration store, encoder Z, centering FSM) once per
  * control cycle, hands the snapshot to the pure supervisor decision logic in
  * steering_supervisor.c, and — only on a proven, persistent electrical
  * hazard — escalates to the safety subsystem (SAFE/ERROR).  It never touches
  * traction, gear, the traction relays or the pedal: an isolable assist loss
  * leaves purely mechanical steering with full traction.
  *
  * This translation unit is deliberately thin so the decision logic stays
  * host-testable while the hardware/global dependencies live here.
  ****************************************************************************
  */

#include "steering_supervisor.h"
#include "sensor_manager.h"
#include "eps_params.h"
#include "steering_cal_store.h"
#include "steering_centering.h"
#include "steering_z.h"
#include "motor_control.h"
#include "safety_system.h"

#include <stddef.h>
#include <string.h>

#include "stm32g4xx_hal.h"

/**
 * @brief  Run the EPS assist supervisor for one control cycle.
 *
 *         Called from the 100 Hz main task after the steering motor writer
 *         (centering FSM or EPS loop) has run.  Builds the detector snapshot
 *         and drives the idempotent EPS isolation API through
 *         SteeringSupervisor_Apply().
 */
void SteeringSupervisor_Service(void)
{
    const Ina226ChannelDiag *ch5 = Sensor_GetChannel5Diag();
    const uint32_t now = HAL_GetTick();

    SteeringSupervisorInputs in;
    memset(&in, 0, sizeof(in));
    in.now_ms = now;

    /* --- INA226 CH5 steering current sensor --------------------------- */
    if (ch5 != NULL) {
        in.ch5_reason       = ch5->fault_reason;
        in.ch5_current_ma   = ch5->signed_current_ma;
        /* Real acquisition identity: the sensor layer increments
         * sample_sequence EXACTLY ONCE per new, valid CH5 acquisition (20 Hz),
         * so the 100 Hz supervisor sees the SAME value across the 5 cycles that
         * re-read one sample and only observes a change when a genuinely new
         * sample lands.  This is what the overcurrent confirm step waits on —
         * NOT a value derived from the supervisor's own tick.               */
        in.ch5_sample_id    = ch5->sample_sequence;
        /* Real acquisition-ATTEMPT identity: the sensor layer increments
         * probe_sequence EXACTLY ONCE per real CH5 probe (valid OR invalid),
         * so the 100 Hz supervisor advances the isolable-fault debounce only
         * once per genuine 20 Hz acquisition, never five times for the same
         * re-read snapshot.                                                  */
        in.ch5_probe_id     = ch5->probe_sequence;
        in.ch5_sample_valid = ch5->i2c_ack && ch5->shunt_read_ok &&
                              ch5->bus_read_ok;
    } else {
        /* Sensor_GetChannel5Diag() normally returns a permanent static object.
         * NULL therefore means the diagnostic source itself is unavailable,
         * not a single transient I2C acquisition.  There is no probe_sequence
         * that could advance the normal three-acquisition debounce; treating
         * this as UNKNOWN would count one fault and then remain fail-open
         * forever.  Isolate the assistance immediately through the shared,
         * idempotent path.  Mechanical steering and traction remain available. */
        Steering_DisableAssistFault(EPS_FAULT_CH5_MISSING);
        in.ch5_reason       = INA226_CH_MISSING;
        in.ch5_sample_valid = false;
    }

    /* --- EPS parameter store (fresh defaults vs corrupt flash) -------- */
    in.params_flash_valid   = EPS_Params_IsFlashValid();
    in.params_flash_present = in.params_flash_valid ||
                              EPS_Params_IsFlashCorrupt();

    /* --- Steering calibration store ---------------------------------- */
    in.cal_flash_corrupt = SteeringCal_IsFlashCorrupt();
    in.cal_flash_present = SteeringCal_IsRestoredValid() || in.cal_flash_corrupt;
    in.cal_valid         = Steering_IsCalibrated();

    {
        const CenteringState_t cs = SteeringCentering_GetState();
        in.centering_finished  = (cs == CENTERING_DONE) || (cs == CENTERING_FAULT);
        in.centering_recovered = Steering_IsCalibrated();
    }

    /* --- Encoder Z policy -------------------------------------------- */
    in.z_required     = (STEERING_Z_REQUIRED != 0);
    {
        const SteeringZStatus_t zs = SteeringZ_GetStatus();
        in.z_status       = (int)zs;
        in.z_center_known = (zs != STEERING_Z_NOT_CALIBRATED);
    }

    SteeringSupervisor_Apply(&in);

    /* --- Non-isolable electrical hazard → hand to the safety subsystem *
     * for SAFE/ERROR escalation.  Only reached after the motor has been
     * isolated (PA6/PA7=0, PC4 LOW, PC12 OFF) and the overcurrent proved
     * to persist on a fresh CH5 sample.                                  */
    if (SteeringSupervisor_WantsSafe()) {
        Safety_SetError(SAFETY_ERROR_OVERCURRENT);
        Safety_SetState(SYS_STATE_SAFE);
    }
}
