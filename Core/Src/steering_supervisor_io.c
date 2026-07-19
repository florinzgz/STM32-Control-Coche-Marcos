/**
  ****************************************************************************
  * @file    steering_supervisor_io.c
  * @brief   Production data-gathering glue for the EPS assist supervisor.
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

#ifndef HOST_TEST
extern TIM_HandleTypeDef htim3;
#endif

static bool steering_homing_is_active(CenteringState_t cs)
{
    return cs == CENTERING_WAIT_RAIL ||
           cs == CENTERING_SWEEP_LEFT ||
           cs == CENTERING_SWEEP_RIGHT;
}

static bool steering_output_is_actively_driven(const Ina226ChannelDiag *ch5)
{
#ifdef HOST_TEST
    /* The host integration test has no TIM3/GPIO register model for this TU.
     * current_expected is its existing explicit command seam. */
    return ch5 != NULL && ch5->current_expected;
#else
    (void)ch5;
    const uint32_t ch1 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    const uint32_t ch2 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
    const bool exactly_one_pwm = (ch1 > 0U) != (ch2 > 0U);
    const bool bridge_enabled =
        HAL_GPIO_ReadPin(GPIOC, PIN_EN_STEER) == GPIO_PIN_SET;
    const bool rail_enabled =
        HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR) == GPIO_PIN_SET;
    return exactly_one_pwm && bridge_enabled && rail_enabled;
#endif
}

void SteeringSupervisor_Service(void)
{
    const Ina226ChannelDiag *ch5 = Sensor_GetChannel5Diag();
    const uint32_t now = HAL_GetTick();
    const CenteringState_t centering = SteeringCentering_GetState();

    SteeringSupervisorInputs in;
    memset(&in, 0, sizeof(in));
    in.now_ms = now;

    if (ch5 != NULL) {
        const bool raw_sample_valid = ch5->i2c_ack && ch5->shunt_read_ok &&
                                      ch5->bus_read_ok;
        const bool eps_drive_active = steering_output_is_actively_driven(ch5);
        const bool homing_active = steering_homing_is_active(centering);
        Ina226ChannelFaultReason reason = ch5->fault_reason;

        /* Reversed polarity is meaningful only while current is intentionally
         * commanded.  Do not let a retained direction classification from the
         * last homing sample isolate PC12 after PB5 has already stopped PWM. */
        if (reason == INA226_CH_POLARITY_REVERSED &&
            (!eps_drive_active || homing_active)) {
            reason = INA226_CH_OK;
        }

        in.ch5_reason       = reason;
        in.ch5_current_ma   = ch5->signed_current_ma;
        in.ch5_sample_id    = ch5->sample_sequence;
        in.ch5_probe_id     = ch5->probe_sequence;

        /* Normal EPS overcurrent is actionable only with a live output command.
         * Homing owns its end-stop current policy.  Once the assist is latched
         * off, fresh CH5 acquisitions become eligible again so the supervisor
         * can prove that current disappeared or escalate a genuinely persistent
         * electrical hazard.  Its sample-sequence guard rejects the retained
         * acquisition that originally caused the isolation. */
        in.ch5_sample_valid = raw_sample_valid && !homing_active &&
            (eps_drive_active || Steering_IsAssistLatchedOff());
    } else {
        Steering_DisableAssistFault(EPS_FAULT_CH5_MISSING);
        in.ch5_reason       = INA226_CH_MISSING;
        in.ch5_sample_valid = false;
    }

    in.params_flash_valid   = EPS_Params_IsFlashValid();
    in.params_flash_present = in.params_flash_valid ||
                              EPS_Params_IsFlashCorrupt();

    in.cal_flash_corrupt = SteeringCal_IsFlashCorrupt();
    in.cal_flash_present = SteeringCal_IsRestoredValid() || in.cal_flash_corrupt;
    in.cal_valid         = Steering_IsCalibrated();
    in.centering_finished = (centering == CENTERING_DONE) ||
                            (centering == CENTERING_FAULT);
    in.centering_recovered = Steering_IsCalibrated();

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
