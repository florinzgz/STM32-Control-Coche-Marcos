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
#include "service_mode.h"

/* ---- Thresholds (from base firmware) ---- */
#define ABS_SLIP_THRESHOLD   15   /* abs_system.cpp: slipThreshold = 15.0f */
#define TCS_SLIP_THRESHOLD   15   /* tcs_system.cpp: slipThreshold = 15.0f */
#define MAX_CURRENT_A        25.0f
#define CAN_TIMEOUT_MS       250

/* Command-validation constants */
#define THROTTLE_MIN         0.0f
#define THROTTLE_MAX         100.0f
#define STEERING_RATE_MAX_DEG_PER_S  200.0f  /* max steering rate          */
#define STEERING_RATE_MIN_DT_S       0.001f /* ignore dt below 1 ms       */
#define MODE_CHANGE_MAX_SPEED_KMH 1.0f       /* speed below which mode OK  */

/* Relay power sequencing delays (milliseconds) */
#define RELAY_MAIN_SETTLE_MS     50   /* inrush current settling time      */
#define RELAY_TRACTION_SETTLE_MS 20   /* contactor arc suppression delay   */
#define SENSOR_TEMP_MIN_C    (-40.0f)
#define SENSOR_TEMP_MAX_C    125.0f   /* DS18B20 absolute range */
#define SENSOR_CURRENT_MAX_A 50.0f    /* anything above this is a fault */
#define SENSOR_SPEED_MAX_KMH 60.0f    /* this vehicle cannot exceed 60 */

/* ---- Module state ---- */
SafetyStatus_t safety_status = {0};
Safety_Error_t safety_error  = SAFETY_ERROR_NONE;

static SystemState_t system_state       = SYS_STATE_BOOT;
static volatile uint32_t last_can_rx_time = 0;  /* Written from ISR (FDCAN RxFifo0Callback) */
static uint8_t  emergency_stopped       = 0;
static float    last_steering_cmd   = 0.0f;
static uint32_t last_steering_tick  = 0;

/* Consecutive-error counter for DEGRADED → SAFE escalation.
 * Traced to base firmware relays.cpp: consecutiveErrors.
 * Only modified from the main-loop safety checks (never from ISR).    */
static uint8_t  consecutive_errors      = 0;
static uint32_t last_error_tick         = 0;

/* Recovery debounce: require RECOVERY_HOLD_MS of clean operation
 * before transitioning DEGRADED → ACTIVE.  Prevents rapid state
 * oscillation when a sensor value fluctuates near a threshold.
 * Traced to limp_mode.cpp: STATE_HYSTERESIS_MS = 500.                 */
#define RECOVERY_HOLD_MS  500
static uint32_t recovery_clean_since    = 0;
static uint8_t  recovery_pending        = 0;  /* 1 = waiting for debounce */

/* ================================================================== */
/*  State Machine                                                      */
/* ================================================================== */

SystemState_t Safety_GetState(void) { return system_state; }

void Safety_SetState(SystemState_t state)
{
    if (state == system_state) return;

    /* Only allow forward transitions and recovery transitions:
     *   SAFE→ACTIVE, DEGRADED→ACTIVE                                   */
    switch (state) {
        case SYS_STATE_STANDBY:
            if (system_state == SYS_STATE_BOOT)
                system_state = SYS_STATE_STANDBY;
            break;

        case SYS_STATE_ACTIVE:
            if (system_state == SYS_STATE_STANDBY ||
                system_state == SYS_STATE_SAFE    ||
                system_state == SYS_STATE_DEGRADED) {
                /* Require no active faults to enter ACTIVE */
                if (safety_error == SAFETY_ERROR_NONE) {
                    system_state = SYS_STATE_ACTIVE;
                    consecutive_errors = 0;
                    Relay_PowerUp();
                }
            }
            break;

        /* DEGRADED: limp / reduced-power mode.  Vehicle can still
         * "drive home".  Traced to base firmware limp_mode.cpp.
         * Unlike SAFE, relays stay ON and commands are accepted
         * (with power/speed limits applied by Safety_ValidateThrottle). */
        case SYS_STATE_DEGRADED:
            if (system_state == SYS_STATE_ACTIVE ||
                system_state == SYS_STATE_STANDBY) {
                system_state = SYS_STATE_DEGRADED;
                /* Do NOT call Safety_FailSafe() — keep relays on.
                 * Traction demand is limited via Safety_ValidateThrottle(). */
            }
            break;

        case SYS_STATE_SAFE:
            if (system_state == SYS_STATE_ACTIVE  ||
                system_state == SYS_STATE_STANDBY  ||
                system_state == SYS_STATE_DEGRADED) {
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
    return (system_state == SYS_STATE_ACTIVE ||
            system_state == SYS_STATE_DEGRADED);
}

bool Safety_IsDegraded(void)
{
    return (system_state == SYS_STATE_DEGRADED);
}

/* Return power-limit multiplier for the current state.
 * ACTIVE  → 1.0 (100 %)
 * DEGRADED → DEGRADED_POWER_LIMIT_PCT / 100 (40 % — traced to
 *            limp_mode.cpp POWER_LIMP)
 * Others  → 0.0 (commands rejected upstream)                          */
float Safety_GetPowerLimitFactor(void)
{
    if (system_state == SYS_STATE_ACTIVE)   return 1.0f;
    if (system_state == SYS_STATE_DEGRADED) return DEGRADED_POWER_LIMIT_PCT / 100.0f;
    return 0.0f;
}

uint8_t Safety_GetFaultFlags(void)
{
    uint8_t flags = 0;
    if (safety_error == SAFETY_ERROR_CAN_TIMEOUT)  flags |= FAULT_CAN_TIMEOUT;
    if (safety_error == SAFETY_ERROR_OVERTEMP)      flags |= FAULT_TEMP_OVERLOAD;
    if (safety_error == SAFETY_ERROR_OVERCURRENT)   flags |= FAULT_CURRENT_OVERLOAD;
    if (safety_error == SAFETY_ERROR_CENTERING)     flags |= FAULT_CENTERING;

    /* Use service mode per-module fault tracking for more granular
     * fault flags (encoder vs wheel speed differentiation) */
    if (ServiceMode_GetFault(MODULE_STEER_ENCODER) != MODULE_FAULT_NONE)
        flags |= FAULT_ENCODER_ERROR;
    for (uint8_t i = 0; i < 4; i++) {
        if (ServiceMode_GetFault((ModuleID_t)(MODULE_WHEEL_SPEED_FL + i)) != MODULE_FAULT_NONE) {
            flags |= FAULT_WHEEL_SENSOR;
            break;
        }
    }
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
     * Delays prevent inrush current spikes on the power bus.            */
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_MAIN, GPIO_PIN_SET);
    HAL_Delay(RELAY_MAIN_SETTLE_MS);
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_SET);
    HAL_Delay(RELAY_TRACTION_SETTLE_MS);
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
    /* Reject commands when not ACTIVE or DEGRADED */
    if (!Safety_IsCommandAllowed()) return 0.0f;

    /* Clamp to valid range */
    if (requested_pct < THROTTLE_MIN) requested_pct = THROTTLE_MIN;
    if (requested_pct > THROTTLE_MAX) requested_pct = THROTTLE_MAX;

    /* ABS/TCS override: if active, safety system has already limited demand */
    if (safety_status.abs_active) return 0.0f;
    if (safety_status.tcs_active) return requested_pct * 0.5f;

    /* Degraded-mode power limit (limp_mode.cpp: POWER_LIMP = 0.4) */
    float limit = Safety_GetPowerLimitFactor();
    return requested_pct * limit;
}

float Safety_ValidateSteering(float requested_deg)
{
    /* Reject commands when not ACTIVE or DEGRADED */
    if (!Safety_IsCommandAllowed()) return Steering_GetCurrentAngle();

    /* Clamp to mechanical limits */
    if (requested_deg < -MAX_STEER_DEG) requested_deg = -MAX_STEER_DEG;
    if (requested_deg >  MAX_STEER_DEG) requested_deg =  MAX_STEER_DEG;

    /* Rate-limit to prevent violent movements */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - last_steering_tick) / 1000.0f;
    if (dt > STEERING_RATE_MIN_DT_S) {
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
    /* Reject commands when not ACTIVE or DEGRADED */
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
    consecutive_errors = 0;
    last_error_tick    = 0;
    recovery_clean_since = 0;
    recovery_pending     = 0;
    system_state      = SYS_STATE_BOOT;
}

/* ---- ABS --------------------------------------------------------- */

void ABS_Update(void)
{
    /* Skip if ABS module is disabled (service mode) */
    if (!ServiceMode_IsEnabled(MODULE_ABS)) {
        safety_status.abs_active = false;
        safety_status.abs_wheel_mask = 0;
        return;
    }

    float spd[4];
    spd[0] = Wheel_GetSpeed_FL();
    spd[1] = Wheel_GetSpeed_FR();
    spd[2] = Wheel_GetSpeed_RL();
    spd[3] = Wheel_GetSpeed_RR();

    float avg = (spd[0] + spd[1] + spd[2] + spd[3]) / 4.0f;
    if (avg < 10.0f) {         /* abs_system.cpp: minSpeedKmh = 10.0f */
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
    /* Skip if TCS module is disabled (service mode) */
    if (!ServiceMode_IsEnabled(MODULE_TCS)) {
        safety_status.tcs_active = false;
        safety_status.tcs_wheel_mask = 0;
        return;
    }

    float spd[4];
    spd[0] = Wheel_GetSpeed_FL();
    spd[1] = Wheel_GetSpeed_FR();
    spd[2] = Wheel_GetSpeed_RL();
    spd[3] = Wheel_GetSpeed_RR();

    float avg = (spd[0] + spd[1] + spd[2] + spd[3]) / 4.0f;
    if (avg < 3.0f) {          /* tcs_system.cpp: minSpeedKmh = 3.0f */
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
        /* Skip disabled current sensors (service mode).
         * Traced to base firmware car_sensors.cpp:
         *   if (!cfg.currentSensorsEnabled) { return 0.0f; }         */
        ModuleID_t mod = (ModuleID_t)(MODULE_CURRENT_SENSOR_0 + i);
        if (!ServiceMode_IsEnabled(mod)) continue;

        float amps = Current_GetAmps(i);
        if (amps > MAX_CURRENT_A) {
            ServiceMode_SetFault(mod, MODULE_FAULT_ERROR);
            Safety_SetError(SAFETY_ERROR_OVERCURRENT);
            /* Count consecutive errors — escalate to SAFE only after
             * CONSECUTIVE_ERROR_THRESHOLD (traced to relays.cpp:
             * consecutiveErrors >= 3).  Single overcurrent events
             * enter DEGRADED to allow "drive home".                   */
            if (consecutive_errors < 255) consecutive_errors++;
            last_error_tick = HAL_GetTick();
            if (consecutive_errors >= CONSECUTIVE_ERROR_THRESHOLD) {
                Safety_SetState(SYS_STATE_SAFE);
            } else {
                Safety_SetState(SYS_STATE_DEGRADED);
            }
            return;
        } else {
            ServiceMode_ClearFault(mod);
        }
    }
    /* No overcurrent — decay consecutive error counter after 1 s of
     * clean operation (traced to relays.cpp: lastErrorMs > 1000).     */
    if ((HAL_GetTick() - last_error_tick) > 1000 && consecutive_errors > 0) {
        consecutive_errors = 0;
    }
    /* Clear overcurrent error once current is back to normal,
     * allowing DEGRADED → ACTIVE recovery.                            */
    if (safety_error == SAFETY_ERROR_OVERCURRENT &&
        system_state == SYS_STATE_DEGRADED &&
        consecutive_errors == 0) {
        Safety_ClearError(SAFETY_ERROR_OVERCURRENT);
    }
}

/* ---- Overtemperature ---------------------------------------------- */

/* Temperature warning threshold — enter DEGRADED, not SAFE.
 * Traced to limp_mode.cpp: Thresholds::TEMP_WARNING = 80.0f            */
#define TEMP_WARNING_C    80.0f
/* Temperature critical threshold — enter SAFE (actuators off).
 * Traced to limp_mode.cpp: Thresholds::TEMP_CRITICAL = 90.0f           */
#define TEMP_CRITICAL_C   90.0f

void Safety_CheckTemperature(void)
{
    for (uint8_t i = 0; i < NUM_DS18B20; i++) {
        /* Skip disabled temperature sensors (service mode).
         * Traced to base firmware car_sensors.cpp:
         *   if (!cfg.tempSensorsEnabled) { return 0.0f; }
         * and temperature.cpp: sensorOk[] per-sensor tracking         */
        ModuleID_t mod = (ModuleID_t)(MODULE_TEMP_SENSOR_0 + i);
        if (!ServiceMode_IsEnabled(mod)) continue;

        float t = Temperature_Get(i);
        if (t > TEMP_CRITICAL_C) {
            ServiceMode_SetFault(mod, MODULE_FAULT_ERROR);
            Safety_SetError(SAFETY_ERROR_OVERTEMP);
            Safety_SetState(SYS_STATE_SAFE);
            return;
        }
        if (t > TEMP_WARNING_C) {
            ServiceMode_SetFault(mod, MODULE_FAULT_WARNING);
            Safety_SetError(SAFETY_ERROR_OVERTEMP);
            Safety_SetState(SYS_STATE_DEGRADED);
            return;
        }
        ServiceMode_ClearFault(mod);
    }
    /* All temperatures OK — clear overtemp error if it was set,
     * allowing DEGRADED → ACTIVE recovery via Safety_CheckCANTimeout.
     * Apply 5 °C hysteresis below TEMP_WARNING_C to prevent
     * oscillation when a motor temp hovers near the threshold.        */
    if (safety_error == SAFETY_ERROR_OVERTEMP &&
        system_state == SYS_STATE_DEGRADED) {
        bool all_below_hysteresis = true;
        for (uint8_t i = 0; i < NUM_DS18B20; i++) {
            ModuleID_t mod = (ModuleID_t)(MODULE_TEMP_SENSOR_0 + i);
            if (!ServiceMode_IsEnabled(mod)) continue;
            if (Temperature_Get(i) > (TEMP_WARNING_C - 5.0f)) {
                all_below_hysteresis = false;
                break;
            }
        }
        if (all_below_hysteresis) {
            Safety_ClearError(SAFETY_ERROR_OVERTEMP);
        }
    }
}

/* ---- CAN Heartbeat Timeout --------------------------------------- */

void Safety_CheckCANTimeout(void)
{
    if ((HAL_GetTick() - last_can_rx_time) > CAN_TIMEOUT_MS) {
        ServiceMode_SetFault(MODULE_CAN_TIMEOUT, MODULE_FAULT_ERROR);
        Safety_SetError(SAFETY_ERROR_CAN_TIMEOUT);
        Safety_SetState(SYS_STATE_SAFE);
    } else {
        ServiceMode_ClearFault(MODULE_CAN_TIMEOUT);
        /* ESP32 alive – if we were in STANDBY, transition to ACTIVE
         * only when steering centering has completed successfully.       */
        if (system_state == SYS_STATE_STANDBY &&
            safety_error == SAFETY_ERROR_NONE &&
            Steering_IsCalibrated()) {
            Safety_SetState(SYS_STATE_ACTIVE);
        }
        /* If in SAFE due to CAN timeout and heartbeat restored, try recovery */
        if (system_state == SYS_STATE_SAFE &&
            safety_error == SAFETY_ERROR_CAN_TIMEOUT) {
            Safety_ClearError(SAFETY_ERROR_CAN_TIMEOUT);
            Safety_SetState(SYS_STATE_ACTIVE);
        }
        /* DEGRADED recovery: if fault has been cleared while in DEGRADED,
         * attempt to return to ACTIVE after a debounce period.
         * Traced to limp_mode.cpp: STATE_HYSTERESIS_MS = 500.
         * The debounce prevents rapid state oscillation when sensor
         * values fluctuate near thresholds.                              */
        if (system_state == SYS_STATE_DEGRADED &&
            safety_error == SAFETY_ERROR_NONE) {
            if (!recovery_pending) {
                recovery_pending   = 1;
                recovery_clean_since = HAL_GetTick();
            } else if ((HAL_GetTick() - recovery_clean_since) >= RECOVERY_HOLD_MS) {
                recovery_pending = 0;
                Safety_SetState(SYS_STATE_ACTIVE);
            }
        } else {
            recovery_pending = 0;  /* Reset debounce if fault reappears */
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
    /* Temperature plausibility: values must be within DS18B20 range.
     * Non-critical sensor fault → DEGRADED (not SAFE) to allow
     * "drive home".  Traced to base firmware system.cpp selfTest:
     * temperature sensors are OPTIONAL and use MODE_DEGRADED.           */
    uint8_t fault_count = 0;
    for (uint8_t i = 0; i < NUM_DS18B20; i++) {
        ModuleID_t mod = (ModuleID_t)(MODULE_TEMP_SENSOR_0 + i);
        if (!ServiceMode_IsEnabled(mod)) continue;
        float t = Temperature_Get(i);
        if (t < SENSOR_TEMP_MIN_C || t > SENSOR_TEMP_MAX_C) {
            ServiceMode_SetFault(mod, MODULE_FAULT_ERROR);
            fault_count++;
        }
    }

    /* Current plausibility: negative or extremely high = fault.
     * Traced to base firmware system.cpp selfTest: current sensors
     * are OPTIONAL and use MODE_DEGRADED.                               */
    for (uint8_t i = 0; i < NUM_INA226; i++) {
        ModuleID_t mod = (ModuleID_t)(MODULE_CURRENT_SENSOR_0 + i);
        if (!ServiceMode_IsEnabled(mod)) continue;
        float a = Current_GetAmps(i);
        if (a < 0.0f || a > SENSOR_CURRENT_MAX_A) {
            ServiceMode_SetFault(mod, MODULE_FAULT_ERROR);
            fault_count++;
        }
    }

    /* Wheel speed plausibility: no single wheel wildly out of range.
     * Traced to base firmware system.cpp selfTest: wheel sensors are
     * OPTIONAL and use MODE_DEGRADED.                                   */
    float spd[4];
    spd[0] = Wheel_GetSpeed_FL();
    spd[1] = Wheel_GetSpeed_FR();
    spd[2] = Wheel_GetSpeed_RL();
    spd[3] = Wheel_GetSpeed_RR();
    for (uint8_t i = 0; i < 4; i++) {
        ModuleID_t mod = (ModuleID_t)(MODULE_WHEEL_SPEED_FL + i);
        if (!ServiceMode_IsEnabled(mod)) continue;
        if (spd[i] < 0.0f || spd[i] > SENSOR_SPEED_MAX_KMH) {
            ServiceMode_SetFault(mod, MODULE_FAULT_ERROR);
            fault_count++;
        }
    }

    /* If any enabled sensor has a plausibility fault, enter DEGRADED */
    if (fault_count > 0) {
        Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
        Safety_SetState(SYS_STATE_DEGRADED);
        return;
    }

    /* All sensor checks passed — if currently DEGRADED due to a sensor
     * fault, clear the error so CAN timeout handler can recover to ACTIVE. */
    if (system_state == SYS_STATE_DEGRADED &&
        safety_error == SAFETY_ERROR_SENSOR_FAULT) {
        Safety_ClearError(SAFETY_ERROR_SENSOR_FAULT);
    }
}

/* ---- Steering encoder health ----------------------------------------- */

/**
 * @brief  Check the steering encoder for faults.
 *
 * Delegates the actual detection to Encoder_CheckHealth() in
 * motor_control.c (which monitors range, jumps and frozen values).
 * If a fault is detected, raise SAFETY_ERROR_SENSOR_FAULT and
 * transition to DEGRADED state.  Steering is neutralised (no PID
 * without encoder feedback), but traction remains operational at
 * reduced power so the vehicle can "drive home".
 *
 * Traced to base firmware limp_mode.cpp: steering not centered →
 * LimpState::LIMP (40 % power, 50 % speed).
 */
void Safety_CheckEncoder(void)
{
    /* Skip if steering encoder module is disabled (service mode).
     * The user has acknowledged the fault and wants to drive without
     * encoder-based steering assist. */
    if (!ServiceMode_IsEnabled(MODULE_STEER_ENCODER)) return;

    Encoder_CheckHealth();

    if (Encoder_HasFault()) {
        ServiceMode_SetFault(MODULE_STEER_ENCODER, MODULE_FAULT_ERROR);
        /* Only set the error once to avoid overwriting a different
         * existing fault code.                                      */
        if (safety_error == SAFETY_ERROR_NONE) {
            Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
        }
        /* Neutralise steering (no PID without encoder) but keep
         * traction alive in DEGRADED mode for "drive home".         */
        Steering_Neutralize();
        Safety_SetState(SYS_STATE_DEGRADED);
    } else {
        ServiceMode_ClearFault(MODULE_STEER_ENCODER);
    }
}

/* ---- Emergency actions ------------------------------------------- */

void Safety_EmergencyStop(void)
{
    emergency_stopped = 1;
    Traction_EmergencyStop();
    /* Transition to ERROR which calls Safety_PowerDown → Relay_PowerDown.
     * Safety_PowerDown is safe to call after Traction_EmergencyStop
     * (actuators are already inhibited; relays are de-energised).       */
    system_state = SYS_STATE_ERROR;
    Relay_PowerDown();
}

void Safety_FailSafe(void)
{
    Traction_EmergencyStop();

    /* When the encoder is healthy, command the steering motor toward
     * centre (0°) so the vehicle tracks straight under inertia.
     * When the encoder is faulted, we have no reliable position
     * feedback — driving the motor blind could make things worse.
     * Instead, neutralise the motor (cut PWM + disable H-bridge)
     * and let mechanical return springs / friction slow the rack.    */
    if (Encoder_HasFault()) {
        Steering_Neutralize();
    } else {
        Steering_SetAngle(0.0f);
    }
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