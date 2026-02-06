/**
  ****************************************************************************
  * @file    safety_system.c
  * @brief   Safety: ABS, TCS, overcurrent, overtemp, CAN timeout, fail-safe
  ****************************************************************************
  */

#include "safety_system.h"
#include "main.h"
#include "sensor_manager.h"
#include "motor_control.h"

/* ---- Thresholds ---- */
#define ABS_SLIP_THRESHOLD   20   /* % wheel slip to trigger ABS */
#define TCS_SLIP_THRESHOLD   15   /* % wheel slip to trigger TCS */
#define MAX_CURRENT_A        25.0f
#define MAX_TEMP_C           90.0f
#define CAN_TIMEOUT_MS       250

/* ---- Module state ---- */
SafetyStatus_t safety_status = {0};
Safety_Error_t safety_error  = SAFETY_ERROR_NONE;

static uint32_t last_can_rx_time = 0;
static uint8_t  emergency_stopped = 0;

/* ================================================================== */

void Safety_Init(void)
{
    safety_status.abs_active = false;
    safety_status.tcs_active = false;
    safety_status.abs_wheel_mask = 0;
    safety_status.tcs_wheel_mask = 0;
    safety_status.abs_activation_count = 0;
    safety_status.tcs_activation_count = 0;
    safety_error     = SAFETY_ERROR_NONE;
    emergency_stopped = 0;
    last_can_rx_time = HAL_GetTick();
}

/* ---- ABS --------------------------------------------------------- */

void ABS_Update(void)
{
    float spd[4];
    spd[0] = Wheel_GetSpeed_FL();
    spd[1] = Wheel_GetSpeed_FR();
    spd[2] = Wheel_GetSpeed_RL();
    spd[3] = Wheel_GetSpeed_RR();

    float avg = (spd[0] + spd[1] + spd[2] + spd[3]) / 4.0f;
    if (avg < 2.0f) {          /* Below 2 km/h – ABS meaningless */
        safety_status.abs_active = false;
        safety_status.abs_wheel_mask = 0;
        return;
    }

    uint8_t mask = 0;
    for (uint8_t i = 0; i < 4; i++) {
        float slip = ((avg - spd[i]) * 100.0f) / avg;
        if (slip > (float)ABS_SLIP_THRESHOLD) {
            mask |= (1U << i);
        }
    }

    if (mask) {
        safety_status.abs_active = true;
        safety_status.abs_wheel_mask = mask;
        safety_status.abs_activation_count++;
        Traction_SetDemand(0);  /* Cut throttle during ABS */
    } else {
        safety_status.abs_active = false;
        safety_status.abs_wheel_mask = 0;
    }
}

bool ABS_IsActive(void)  { return safety_status.abs_active; }
void ABS_Reset(void)     { safety_status.abs_active = false; safety_status.abs_wheel_mask = 0; }

/* ---- TCS --------------------------------------------------------- */

void TCS_Update(void)
{
    float spd[4];
    spd[0] = Wheel_GetSpeed_FL();
    spd[1] = Wheel_GetSpeed_FR();
    spd[2] = Wheel_GetSpeed_RL();
    spd[3] = Wheel_GetSpeed_RR();

    float avg = (spd[0] + spd[1] + spd[2] + spd[3]) / 4.0f;
    if (avg < 1.0f) {
        safety_status.tcs_active = false;
        safety_status.tcs_wheel_mask = 0;
        return;
    }

    uint8_t mask = 0;
    for (uint8_t i = 0; i < 4; i++) {
        float slip = ((spd[i] - avg) * 100.0f) / avg;
        if (slip > (float)TCS_SLIP_THRESHOLD) {
            mask |= (1U << i);
        }
    }

    if (mask) {
        safety_status.tcs_active = true;
        safety_status.tcs_wheel_mask = mask;
        safety_status.tcs_activation_count++;
        Traction_SetDemand(Pedal_GetPercent() / 2.0f);
    } else {
        safety_status.tcs_active = false;
        safety_status.tcs_wheel_mask = 0;
    }
}

bool TCS_IsActive(void) { return safety_status.tcs_active; }
void TCS_Reset(void)    { safety_status.tcs_active = false; safety_status.tcs_wheel_mask = 0; }

/* ---- Overcurrent ------------------------------------------------- */

void Safety_CheckCurrent(void)
{
    for (uint8_t i = 0; i < NUM_INA226; i++) {
        float amps = Current_GetAmps(i);
        if (amps > MAX_CURRENT_A) {
            Safety_SetError(SAFETY_ERROR_OVERCURRENT);
            Safety_EmergencyStop();
            return;
        }
    }
}

/* ---- Overtemperature ---------------------------------------------- */

void Safety_CheckTemperature(void)
{
    for (uint8_t i = 0; i < NUM_DS18B20; i++) {
        float t = Temperature_Get(i);
        if (t > MAX_TEMP_C) {
            Safety_SetError(SAFETY_ERROR_OVERTEMP);
            Safety_EmergencyStop();
            return;
        }
    }
}

/* ---- CAN Heartbeat Timeout --------------------------------------- */

void Safety_CheckCANTimeout(void)
{
    if ((HAL_GetTick() - last_can_rx_time) > CAN_TIMEOUT_MS) {
        Safety_SetError(SAFETY_ERROR_CAN_TIMEOUT);
        Safety_EmergencyStop();
    }
}

/* Called by CAN RX handler to refresh watchdog */
void Safety_UpdateCANRxTime(void)
{
    last_can_rx_time = HAL_GetTick();
}

/* ---- Sensor plausibility (stub for future extension) ------------- */

void Safety_CheckSensors(void)
{
    /* TODO: Verify sensor readings are within plausible range */
}

/* ---- Emergency actions ------------------------------------------- */

void Safety_EmergencyStop(void)
{
    emergency_stopped = 1;
    Traction_EmergencyStop();
}

void Safety_FailSafe(void)
{
    Safety_EmergencyStop();
    /* Center steering */
    Steering_SetAngle(0.0f);
}

void Safety_PowerDown(void)
{
    Safety_EmergencyStop();
    /* De-energize relays */
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_MAIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_DIR,  GPIO_PIN_RESET);
}

/* ---- Error tracking ---------------------------------------------- */

void Safety_SetError(Safety_Error_t error)   { safety_error = error; }
void Safety_ClearError(Safety_Error_t error) { if (safety_error == error) safety_error = SAFETY_ERROR_NONE; }
Safety_Error_t Safety_GetError(void)         { return safety_error; }
bool Safety_IsError(void)                    { return (safety_error != SAFETY_ERROR_NONE); }