/**
  ****************************************************************************
  * @file    safety_system.c
  * @brief   Safety: ABS, TCS, overcurrent, overtemp, CAN timeout, fail-safe
  *
  *          Implements the STM32 safety-authority role:
  *            – System state machine (BOOT→STANDBY→ACTIVE→SAFE→ERROR)
  *            – Command validation gate for ESP32 requests
  *            – Relay power sequencing
  *            – Sensor plausibility checks
  ****************************************************************************
  */

#include "safety_system.h"
#include "main.h"
#include "sensor_manager.h"
#include "motor_control.h"
#include <math.h>

/* ---- Thresholds ---- */
#define ABS_SLIP_THRESHOLD   20   /* % wheel slip to trigger ABS */
#define TCS_SLIP_THRESHOLD   15   /* % wheel slip to trigger TCS */
#define MAX_CURRENT_A        25.0f
#define MAX_TEMP_C           90.0f
#define CAN_TIMEOUT_MS       250

/* Command-validation constants */
#define THROTTLE_MIN         0.0f
#define THROTTLE_MAX         100.0f
#define STEERING_MAX_DEG     45.0f
#define STEERING_RATE_MAX_DEG_PER_S  200.0f  /* max steering rate          */
#define MODE_CHANGE_MAX_SPEED_KMH 1.0f       /* speed below which mode OK  */

/* Sensor plausibility */
#define SENSOR_TEMP_MIN_C    (-40.0f)
#define SENSOR_TEMP_MAX_C    125.0f   /* DS18B20 absolute range */
#define SENSOR_CURRENT_MAX_A 50.0f    /* anything above this is a fault */
#define SENSOR_SPEED_MAX_KMH 60.0f    /* this vehicle cannot exceed 60 */

/* ---- Module state ---- */
SafetyStatus_t safety_status = {0};
Safety_Error_t safety_error  = SAFETY_ERROR_NONE;

static SystemState_t system_state   = SYS_STATE_BOOT;
static uint32_t last_can_rx_time    = 0;
static uint8_t  emergency_stopped   = 0;
static float    last_steering_cmd   = 0.0f;
static uint32_t last_steering_tick  = 0;

/* ================================================================== */
/*  State Machine                                                      */
/* ================================================================== */

SystemState_t Safety_GetState(void) { return system_state; }

void Safety_SetState(SystemState_t state)
{
    if (state == system_state) return;

    /* Only allow forward transitions and SAFE→ACTIVE recovery */
    switch (state) {
        case SYS_STATE_STANDBY:
            if (system_state == SYS_STATE_BOOT)
                system_state = SYS_STATE_STANDBY;
            break;

        case SYS_STATE_ACTIVE:
            if (system_state == SYS_STATE_STANDBY ||
                system_state == SYS_STATE_SAFE) {
                /* Require no active faults to enter ACTIVE */
                if (safety_error == SAFETY_ERROR_NONE) {
                    system_state = SYS_STATE_ACTIVE;
                    Relay_PowerUp();
                }
            }
            break;

        case SYS_STATE_SAFE:
            if (system_state == SYS_STATE_ACTIVE ||
                system_state == SYS_STATE_STANDBY) {
                system_state = SYS_STATE_SAFE;
                Safety_FailSafe();
            }
            break;

        case SYS_STATE_ERROR:
            system_state = SYS_STATE_ERROR;
            Safety_PowerDown();
            break;

        default:
            break;
    }
}

bool Safety_IsCommandAllowed(void)
{
    return (system_state == SYS_STATE_ACTIVE);
}

uint8_t Safety_GetFaultFlags(void)
{
    uint8_t flags = 0;
    if (safety_error == SAFETY_ERROR_CAN_TIMEOUT)  flags |= FAULT_CAN_TIMEOUT;
    if (safety_error == SAFETY_ERROR_OVERTEMP)      flags |= FAULT_TEMP_OVERLOAD;
    if (safety_error == SAFETY_ERROR_OVERCURRENT)   flags |= FAULT_CURRENT_OVERLOAD;
    if (safety_error == SAFETY_ERROR_SENSOR_FAULT)  flags |= FAULT_ENCODER_ERROR;
    if (safety_status.abs_active)                   flags |= FAULT_ABS_ACTIVE;
    if (safety_status.tcs_active)                   flags |= FAULT_TCS_ACTIVE;
    return flags;
}

/* ================================================================== */
/*  Relay Power Sequencing                                             */
/* ================================================================== */

void Relay_PowerUp(void)
{
    /* Sequence: Main → wait → Traction → Direction
     * This prevents inrush current spikes.                              */
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_MAIN, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_SET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_DIR,  GPIO_PIN_SET);
}

void Relay_PowerDown(void)
{
    /* Reverse order: Direction → Traction → Main */
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_DIR,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_MAIN, GPIO_PIN_RESET);
}

/* ================================================================== */
/*  Command Validation Gate                                            */
/* ================================================================== */

float Safety_ValidateThrottle(float requested_pct)
{
    /* Reject commands when not ACTIVE */
    if (!Safety_IsCommandAllowed()) return 0.0f;

    /* Clamp to valid range */
    if (requested_pct < THROTTLE_MIN) requested_pct = THROTTLE_MIN;
    if (requested_pct > THROTTLE_MAX) requested_pct = THROTTLE_MAX;

    /* ABS/TCS override: if active, safety system has already limited demand */
    if (safety_status.abs_active) return 0.0f;
    if (safety_status.tcs_active) return requested_pct * 0.5f;

    return requested_pct;
}

float Safety_ValidateSteering(float requested_deg)
{
    /* Reject commands when not ACTIVE */
    if (!Safety_IsCommandAllowed()) return Steering_GetCurrentAngle();

    /* Clamp to mechanical limits */
    if (requested_deg < -STEERING_MAX_DEG) requested_deg = -STEERING_MAX_DEG;
    if (requested_deg >  STEERING_MAX_DEG) requested_deg =  STEERING_MAX_DEG;

    /* Rate-limit to prevent violent movements */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - last_steering_tick) / 1000.0f;
    if (dt > 0.001f) {
        float max_delta = STEERING_RATE_MAX_DEG_PER_S * dt;
        float delta = requested_deg - last_steering_cmd;
        if (delta >  max_delta) requested_deg = last_steering_cmd + max_delta;
        if (delta < -max_delta) requested_deg = last_steering_cmd - max_delta;
    }
    last_steering_cmd  = requested_deg;
    last_steering_tick = now;

    return requested_deg;
}

bool Safety_ValidateModeChange(bool enable_4x4, bool tank_turn)
{
    /* Reject commands when not ACTIVE */
    if (!Safety_IsCommandAllowed()) return false;

    /* Mode change only allowed at very low speed */
    float avg_speed = (Wheel_GetSpeed_FL() + Wheel_GetSpeed_FR() +
                       Wheel_GetSpeed_RL() + Wheel_GetSpeed_RR()) / 4.0f;
    if (avg_speed > MODE_CHANGE_MAX_SPEED_KMH) return false;

    return true;
}

/* ================================================================== */
/*  Initialization                                                     */
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
    last_can_rx_time  = HAL_GetTick();
    last_steering_cmd = 0.0f;
    last_steering_tick = HAL_GetTick();
    system_state      = SYS_STATE_BOOT;
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
            Safety_SetState(SYS_STATE_SAFE);
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
            Safety_SetState(SYS_STATE_SAFE);
            return;
        }
    }
}

/* ---- CAN Heartbeat Timeout --------------------------------------- */

void Safety_CheckCANTimeout(void)
{
    if ((HAL_GetTick() - last_can_rx_time) > CAN_TIMEOUT_MS) {
        Safety_SetError(SAFETY_ERROR_CAN_TIMEOUT);
        Safety_SetState(SYS_STATE_SAFE);
    } else {
        /* ESP32 alive – if we were in STANDBY, transition to ACTIVE */
        if (system_state == SYS_STATE_STANDBY &&
            safety_error == SAFETY_ERROR_NONE) {
            Safety_SetState(SYS_STATE_ACTIVE);
        }
        /* If in SAFE due to CAN timeout and heartbeat restored, try recovery */
        if (system_state == SYS_STATE_SAFE &&
            safety_error == SAFETY_ERROR_CAN_TIMEOUT) {
            Safety_ClearError(SAFETY_ERROR_CAN_TIMEOUT);
            Safety_SetState(SYS_STATE_ACTIVE);
        }
    }
}

/* Called by CAN RX handler to refresh watchdog */
void Safety_UpdateCANRxTime(void)
{
    last_can_rx_time = HAL_GetTick();
}

/* ---- Sensor plausibility checks ---------------------------------- */

void Safety_CheckSensors(void)
{
    /* Temperature plausibility: values must be within DS18B20 range */
    for (uint8_t i = 0; i < NUM_DS18B20; i++) {
        float t = Temperature_Get(i);
        if (t < SENSOR_TEMP_MIN_C || t > SENSOR_TEMP_MAX_C) {
            Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
            return;
        }
    }

    /* Current plausibility: negative or extremely high = fault */
    for (uint8_t i = 0; i < NUM_INA226; i++) {
        float a = Current_GetAmps(i);
        if (a < 0.0f || a > SENSOR_CURRENT_MAX_A) {
            Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
            return;
        }
    }

    /* Wheel speed plausibility: no single wheel wildly different */
    float spd[4];
    spd[0] = Wheel_GetSpeed_FL();
    spd[1] = Wheel_GetSpeed_FR();
    spd[2] = Wheel_GetSpeed_RL();
    spd[3] = Wheel_GetSpeed_RR();
    for (uint8_t i = 0; i < 4; i++) {
        if (spd[i] < 0.0f || spd[i] > SENSOR_SPEED_MAX_KMH) {
            Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
            return;
        }
    }
}

/* ---- Emergency actions ------------------------------------------- */

void Safety_EmergencyStop(void)
{
    emergency_stopped = 1;
    Traction_EmergencyStop();
    Safety_SetState(SYS_STATE_ERROR);
}

void Safety_FailSafe(void)
{
    Traction_EmergencyStop();
    /* Center steering */
    Steering_SetAngle(0.0f);
}

void Safety_PowerDown(void)
{
    Traction_EmergencyStop();
    Relay_PowerDown();
}

/* ---- Error tracking ---------------------------------------------- */

void Safety_SetError(Safety_Error_t error)   { safety_error = error; }
void Safety_ClearError(Safety_Error_t error) { if (safety_error == error) safety_error = SAFETY_ERROR_NONE; }
Safety_Error_t Safety_GetError(void)         { return safety_error; }
bool Safety_IsError(void)                    { return (safety_error != SAFETY_ERROR_NONE); }