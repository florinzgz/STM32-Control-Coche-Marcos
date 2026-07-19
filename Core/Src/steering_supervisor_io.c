/**
  ****************************************************************************
  * @file    steering_supervisor_io.c
  * @brief   Production data-gathering glue for the EPS assist supervisor.
  *
  * Reads the real steering detectors once per control cycle and hands a
  * coherent snapshot to the pure supervisor.  A CH5 current sample is eligible
  * for the ACTIVE overcurrent FSM only while the EPS output is physically
  * commanded in that same cycle.  This prevents a retained 20 Hz INA226 sample
  * from opening PC12 after homing has already stopped at PB5 or an end stop.
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
#include "main.h"

#include <stddef.h>
#include <string.h>

#include "stm32g4xx_hal.h"

extern TIM_HandleTypeDef htim3;

static bool steering_homing_is_active(CenteringState_t cs)
{
    return cs == CENTERING_WAIT_RAIL ||
           cs == CENTERING_SWEEP_LEFT ||
           cs == CENTERING_SWEEP_RIGHT;
}

static bool steering_output_is_actively_driven(void)
{
    const uint32_t ch1 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    const uint32_t ch2 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
    const bool exactly_one_pwm = (ch1 > 0U) != (ch2 > 0U);
    const bool bridge_enabled =
        HAL_GPIO_ReadPin(GPIOC, PIN_EN_STEER) == GPIO_PIN_SET;
    const bool rail_enabled =
        HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR) == GPIO_PIN_SET;

    return exactly_one_pwm && bridge_enabled && rail_enabled;
}

void SteeringSupervisor_Service(void)
{
    const Ina226ChannelDiag *ch5 = Sensor_GetChannel5Diag();
    const uint32_t now = HAL_GetTick();
    const CenteringState_t centering = SteeringCentering_GetState();

    SteeringSupervisorInputs in;
    memset(&in, 0, sizeof(in));
    in.now_ms = now;

    /* --- INA226 CH5 steering-current sensor --------------------------- */
    if (ch5 != NULL) {
        const bool raw_sample_valid = ch5->i2c_ack && ch5->shunt_read_ok &&
                                      ch5->bus_read_ok;
        const bool eps_drive_active = steering_output_is_actively_driven();
        const bool homing_active = steering_homing_is_active(centering);

        in.ch5_reason       = ch5->fault_reason;
        in.ch5_current_ma   = ch5->signed_current_ma;
        in.ch5_sample_id    = ch5->sample_sequence;
        in.ch5_probe_id     = ch5->probe_sequence;

        /* The generic EPS overcurrent FSM is for normal assistance.  Homing has
         * its own encoder+current end-stop guard and must be allowed to remove
         * torque, wait dead-time and reverse LEFT->RIGHT before any latched EPS
         * isolation occurs.  Outside homing, a current sample is actionable only
         * while one TIM3 PWM channel, PC4 and PC12 are all active now.  A sample
         * left over from the previous 50 ms INA acquisition can therefore never
         * open PC12 after the output has already been neutralised. */
        in.ch5_sample_valid = raw_sample_valid && eps_drive_active &&
                              !homing_active;
    } else {
        /* A missing diagnostic source is a persistent loss of current
         * supervision, not one transient read.  Isolate assistance immediately;
         * mechanical steering and traction remain available. */
        Steering_DisableAssistFault(EPS_FAULT_CH5_MISSING);
        in.ch5_reason       = INA226_CH_MISSING;
        in.ch5_sample_valid = false;
    }

    /* --- EPS parameter store ------------------------------------------ */
    in.params_flash_valid   = EPS_Params_IsFlashValid();
    in.params_flash_present = in.params_flash_valid ||
                              EPS_Params_IsFlashCorrupt();

    /* --- Steering calibration store ---------------------------------- */
    in.cal_flash_corrupt = SteeringCal_IsFlashCorrupt();
    in.cal_flash_present = SteeringCal_IsRestoredValid() || in.cal_flash_corrupt;
    in.cal_valid         = Steering_IsCalibrated();
    in.centering_finished = (centering == CENTERING_DONE) ||
                            (centering == CENTERING_FAULT);
    in.centering_recovered = Steering_IsCalibrated();

    /* --- Encoder Z policy -------------------------------------------- */
    in.z_required = (STEERING_Z_REQUIRED != 0);
    {
        const SteeringZStatus_t zs = SteeringZ_GetStatus();
        in.z_status       = (int)zs;
        in.z_center_known = (zs != STEERING_Z_NOT_CALIBRATED);
    }

    SteeringSupervisor_Apply(&in);

    if (SteeringSupervisor_WantsSafe()) {
        Safety_SetError(SAFETY_ERROR_OVERCURRENT);
        Safety_SetState(SYS_STATE_SAFE);
    }
}
