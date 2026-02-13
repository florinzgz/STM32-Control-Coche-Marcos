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
#include "boot_validation.h"

/* ---- Thresholds (from base firmware) ---- */
#define ABS_SLIP_THRESHOLD   15   /* abs_system.cpp: slipThreshold = 15.0f */
#define TCS_SLIP_THRESHOLD   15   /* tcs_system.cpp: slipThreshold = 15.0f */
#define MAX_CURRENT_A        25.0f
#define CAN_TIMEOUT_MS       250

/* Battery undervoltage thresholds (24 V system).
 *
 * Rationale for chosen voltages:
 *   24 V nominal → six 18650 cells in 6S configuration (25.2 V full, 18.0 V empty).
 *   20.0 V warning ≈ 3.33 V/cell — cells are nearly depleted, reduce load to
 *       prolong remaining capacity and prevent deep-discharge damage.
 *   18.0 V critical ≈ 3.00 V/cell — absolute minimum safe cell voltage.
 *       Below this, cell chemistry degrades irreversibly and motor controller
 *       behaviour becomes unpredictable (brown-out risk).
 *
 * Hysteresis (0.5 V) prevents oscillation when voltage sags under load
 * then recovers during coast.  This is a common pattern in automotive BMS.
 *
 * Recovery from SAFE is intentionally blocked (non-auto-recovery):
 *   A critically depleted battery cannot reliably power actuators.
 *   Operator must recharge and reset the system to clear the fault.
 *   This follows the fail-safe philosophy used for CAN timeout and
 *   emergency stop in the existing safety architecture.                  */
#define BATTERY_UV_WARNING_V    20.0f   /* Enter DEGRADED below this      */
#define BATTERY_UV_CRITICAL_V   18.0f   /* Enter SAFE below this          */
#define BATTERY_UV_HYST_V       0.5f    /* Hysteresis band for recovery   */

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
#define SENSOR_SPEED_MAX_KMH 25.0f    /* RS775 20000RPM / 1:75 gear → ~266 wheel RPM
                                        * × 1.1m circumf → ~17.6 km/h max.
                                        * 25 km/h gives ~40 % plausibility margin. */

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

/* ---- Non-blocking relay sequencer state machine ---- */
typedef enum {
    RELAY_SEQ_IDLE = 0,     /* All relays off, no sequence in progress   */
    RELAY_SEQ_MAIN_ON,      /* Main relay energised, waiting settle      */
    RELAY_SEQ_TRACTION_ON,  /* Traction relay energised, waiting settle  */
    RELAY_SEQ_COMPLETE       /* All relays on, sequence finished          */
} RelaySeqState_t;

static RelaySeqState_t relay_seq_state     = RELAY_SEQ_IDLE;
static uint32_t        relay_seq_timestamp = 0;

/* Recovery debounce: require RECOVERY_HOLD_MS of clean operation
 * before transitioning DEGRADED → ACTIVE.  Prevents rapid state
 * oscillation when a sensor value fluctuates near a threshold.
 * Traced to limp_mode.cpp: STATE_HYSTERESIS_MS = 500.                 */
#define RECOVERY_HOLD_MS  500
static uint32_t recovery_clean_since    = 0;
static uint8_t  recovery_pending        = 0;  /* 1 = waiting for debounce */

/* Per-wheel TCS reduction accumulator (persistent across calls).
 * Mirrors tcs_system.cpp WheelTCSState::powerReduction.
 * Declared here (module state section) so Safety_Init can reset them. */
static float tcs_reduction[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint32_t tcs_last_tick = 0;

/* ---- Obstacle CAN receiver state -------------------------------- */

/* Obstacle distance thresholds for the STM32 backstop limiter.
 * These are simplified 3-tier thresholds — the ESP32 runs the full
 * 5-zone logic with linear interpolation and child reaction detection.
 * The STM32 backstop is a defence-in-depth layer that operates
 * independently of the ESP32's obstacle safety logic.
 *
 * Traced to OBSTACLE_SYSTEM_STRATEGY.md Part 3, proposed values.    */
#define OBSTACLE_EMERGENCY_MM       200     /* < 200 mm → scale = 0.0, SAFE   */
#define OBSTACLE_CRITICAL_MM        500     /* 200–500 mm → scale = 0.3       */
#define OBSTACLE_WARNING_MM         1000    /* 500–1000 mm → scale = 0.7      */
/* > 1000 mm → scale = 1.0 (no reduction)                                     */

/* Recovery hysteresis: after emergency stop, require distance > 500 mm
 * for > 1 second before allowing auto-recovery.  Prevents oscillation
 * when obstacle is near the 200 mm boundary.                          */
#define OBSTACLE_RECOVERY_MM        500
#define OBSTACLE_RECOVERY_MS        1000

/* CAN timeout for obstacle messages (0x208).  If no message is
 * received within this window, the STM32 assumes obstacle sensor
 * failure and applies fail-safe (obstacle_scale = 0.0, SAFE state).
 * This is longer than the heartbeat timeout (250 ms) because obstacle
 * messages are sent at 66 ms (15 Hz) and we allow 7+ missed frames
 * before declaring timeout.                                           */
#define OBSTACLE_CAN_TIMEOUT_MS     500

static uint32_t obstacle_last_rx_tick   = 0;   /* Last 0x208 reception time   */
static uint16_t obstacle_distance_mm    = 0xFFFF; /* Last reported distance   */
static uint8_t  obstacle_zone           = 0;   /* Last reported zone (0–5)    */
static uint8_t  obstacle_sensor_healthy = 0;   /* Sensor health from ESP32    */
static uint8_t  obstacle_last_counter   = 0;   /* Rolling counter for stale   */
static uint8_t  obstacle_stale_count    = 0;   /* Consecutive stale frames    */
static uint8_t  obstacle_data_valid     = 0;   /* 1 = at least one msg rcvd   */
static uint32_t obstacle_recovery_tick  = 0;   /* Recovery debounce start     */
static uint8_t  obstacle_in_emergency   = 0;   /* Currently in emergency stop */

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
    /* Initiate non-blocking power-up sequence.
     * The actual relay transitions are driven by Relay_SequencerUpdate()
     * called from the 10 ms safety loop.
     * Re-entry safe: if already sequencing or complete, do nothing.     */
    if (relay_seq_state != RELAY_SEQ_IDLE) {
        return;  /* Sequence already in progress or complete */
    }

    /* Step 1: Energise main relay and record timestamp */
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_MAIN, GPIO_PIN_SET);
    relay_seq_state     = RELAY_SEQ_MAIN_ON;
    relay_seq_timestamp = HAL_GetTick();
}

void Relay_SequencerUpdate(void)
{
    /* Non-blocking relay sequencer — call from the 10 ms safety loop.
     * Progresses through the power-up sequence using timestamps:
     *   MAIN_ON  →  (50 ms)  →  TRACTION_ON  →  (20 ms)  →  COMPLETE
     * IDLE and COMPLETE are no-ops.                                     */
    uint32_t now = HAL_GetTick();

    switch (relay_seq_state) {
        case RELAY_SEQ_MAIN_ON:
            if ((now - relay_seq_timestamp) >= RELAY_MAIN_SETTLE_MS) {
                HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_SET);
                relay_seq_state     = RELAY_SEQ_TRACTION_ON;
                relay_seq_timestamp = now;
            }
            break;

        case RELAY_SEQ_TRACTION_ON:
            if ((now - relay_seq_timestamp) >= RELAY_TRACTION_SETTLE_MS) {
                HAL_GPIO_WritePin(GPIOC, PIN_RELAY_DIR, GPIO_PIN_SET);
                relay_seq_state = RELAY_SEQ_COMPLETE;
            }
            break;

        case RELAY_SEQ_IDLE:
        case RELAY_SEQ_COMPLETE:
        default:
            break;
    }
}

void Relay_PowerDown(void)
{
    /* Reverse order: Direction → Traction → Main.
     * Cancels any in-progress power-up sequence immediately.            */
    relay_seq_state = RELAY_SEQ_IDLE;
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

    /* Per-wheel ABS/TCS modulation is now handled in Traction_Update()
     * via safety_status.wheel_scale[].  No global override here.
     * Aligned with base firmware: abs_system.cpp modulateBrake() and
     * tcs_system.cpp modulatePower() operate per-wheel, not globally.
     *
     * Global fallback (all 4 wheels slipping) is handled directly in
     * ABS_Update / TCS_Update via Traction_SetDemand().               */

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
    safety_status.obstacle_scale = 1.0f;
    for (uint8_t i = 0; i < 4; i++) {
        safety_status.wheel_scale[i] = 1.0f;
        tcs_reduction[i] = 0.0f;
        abs_pulse_timer[i] = HAL_GetTick();
        abs_pulse_phase[i] = 0U;
    }
    tcs_last_tick    = HAL_GetTick();
    safety_error     = SAFETY_ERROR_NONE;
    emergency_stopped = 0;
    last_can_rx_time  = HAL_GetTick();
    last_steering_cmd = 0.0f;
    last_steering_tick = HAL_GetTick();
    consecutive_errors = 0;
    last_error_tick    = 0;
    recovery_clean_since = 0;
    recovery_pending     = 0;
    obstacle_last_rx_tick   = 0;
    obstacle_distance_mm    = 0xFFFF;
    obstacle_zone           = 0;
    obstacle_sensor_healthy = 0;
    obstacle_last_counter   = 0;
    obstacle_stale_count    = 0;
    obstacle_data_valid     = 0;
    obstacle_recovery_tick  = 0;
    obstacle_in_emergency   = 0;
    system_state      = SYS_STATE_BOOT;
}

/* ---- ABS --------------------------------------------------------- */

/* ---- ABS Pulse Modulation ----------------------------------------
 *
 * Previous behaviour: wheel_scale = 0.0 (full torque cut) whenever
 * slip exceeded the threshold.  This was too aggressive — a 100 %
 * cut removes all motor torque instantly, preventing the tyre from
 * recovering traction smoothly.  On real vehicles ABS works by
 * rapidly cycling brake pressure (pulse modulation) so the wheel
 * alternates between a reduced-torque phase and a recovery phase,
 * maintaining directional control and shortening stopping distance.
 *
 * New behaviour (aligned with reference firmware abs_system.cpp):
 *   pressureReduction = 0.30 → 30 % reduction during the ON phase.
 *   Pulse period       = 80 ms  (square-wave cycle).
 *   ON ratio           = 60 %   → 48 ms reduced, 32 ms recovery.
 *
 * During the ON phase wheel_scale = 0.70 (1.0 − 0.30), allowing
 * the motor to provide 70 % torque.  During the OFF phase
 * wheel_scale = 1.0 (full torque) so the wheel can spin back up.
 * The rapid cycling improves grip recovery compared to a sustained
 * full cut while still limiting lock-up torque.
 *
 * 30 % reduction was chosen because:
 *   1. It matches the reference firmware abs_system.cpp value.
 *   2. It is aggressive enough to break the lock-up cycle.
 *   3. It is mild enough to preserve steering authority.
 *
 * Implementation uses HAL_GetTick() for non-blocking timing.
 * Per-wheel state (abs_pulse_timer[], abs_pulse_phase[]) ensures
 * independent modulation on each corner.
 * ----------------------------------------------------------------- */

#define ABS_BASE_REDUCTION      0.30f   /* 30 % reduction (ref firmware)  */
#define ABS_PULSE_PERIOD_MS     80      /* total pulse cycle in ms        */
#define ABS_PULSE_ON_RATIO      0.6f    /* 60 % of period = reduced phase */

/* Per-wheel pulse state (non-blocking, timestamp-based) */
static uint32_t abs_pulse_timer[4];     /* HAL_GetTick() at phase start   */
static uint8_t  abs_pulse_phase[4];     /* 1 = ON (reduced), 0 = OFF      */

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
        /* Do NOT reset wheel_scale here — TCS_Update may have active
         * reductions that must not be overwritten.  ABS-specific
         * scales are restored in the per-wheel loop below.            */
        return;
    }

    uint32_t now = HAL_GetTick();
    uint32_t on_duration  = (uint32_t)(ABS_PULSE_PERIOD_MS * ABS_PULSE_ON_RATIO);
    uint32_t off_duration = (uint32_t)(ABS_PULSE_PERIOD_MS) - on_duration;

    uint8_t mask = 0;
    for (uint8_t i = 0; i < 4; i++) {
        float slip = ((avg - spd[i]) * 100.0f) / avg;
        if (slip > (float)ABS_SLIP_THRESHOLD) {
            mask |= (1U << i);

            /* Advance pulse state machine (non-blocking square-wave).
             * ON phase  → wheel_scale = 1.0 − ABS_BASE_REDUCTION (70 %)
             * OFF phase → wheel_scale = 1.0 (full torque recovery)      */
            uint32_t elapsed = now - abs_pulse_timer[i];
            if (abs_pulse_phase[i]) {
                /* Currently in ON (reduced) phase */
                if (elapsed >= on_duration) {
                    abs_pulse_phase[i] = 0U;
                    abs_pulse_timer[i] = now;
                }
            } else {
                /* Currently in OFF (recovery) phase */
                if (elapsed >= off_duration) {
                    abs_pulse_phase[i] = 1U;
                    abs_pulse_timer[i] = now;
                }
            }

            if (abs_pulse_phase[i]) {
                safety_status.wheel_scale[i] = 1.0f - ABS_BASE_REDUCTION;
            } else {
                safety_status.wheel_scale[i] = 1.0f;
            }
        } else {
            /* Wheel not locking — restore full power and reset pulse.
             * TCS_Update runs after ABS_Update and may further reduce
             * this value if the wheel is spinning (TCS takes the min). */
            safety_status.wheel_scale[i] = 1.0f;
            abs_pulse_timer[i] = now;
            abs_pulse_phase[i] = 0U;
        }
    }

    if (mask) {
        safety_status.abs_active = true;
        safety_status.abs_wheel_mask = mask;
        safety_status.abs_activation_count++;
        /* Global fallback: if ALL wheels lock, apply global throttle
         * cut as a last-resort safety measure (vehicle is on ice or
         * sensors are unreliable).                                    */
        if (mask == 0x0F) {
            Traction_SetDemand(0);
        }
        /* Otherwise: per-wheel scale is applied in Traction_Update(). */
    } else {
        safety_status.abs_active = false;
        safety_status.abs_wheel_mask = 0;
    }
}

bool ABS_IsActive(void)  { return safety_status.abs_active; }
void ABS_Reset(void)
{
    safety_status.abs_active = false;
    safety_status.abs_wheel_mask = 0;
    for (uint8_t i = 0; i < 4; i++) {
        safety_status.wheel_scale[i] = 1.0f;
        abs_pulse_timer[i] = HAL_GetTick();
        abs_pulse_phase[i] = 0U;
    }
}

/* ---- TCS --------------------------------------------------------- */

/* Per-wheel TCS power reduction, aligned with base firmware
 * tcs_system.cpp:
 *   aggressiveReduction = 40.0f  → initial 40 % cut (scale = 0.6)
 *   smoothReduction     =  5.0f  → +5 % per cycle while still slipping
 *   max reduction       = 80.0f  → floor scale = 0.2
 *   recoveryRatePerSec  = 25.0f  → 25 %/s recovery when slip clears
 *
 * These are NOT new thresholds — they are taken directly from the
 * base firmware TCSSystem::Config defaults.                          */
#define TCS_INITIAL_REDUCTION   0.40f   /* 40 % power cut on activation */
#define TCS_SMOOTH_REDUCTION    0.05f   /* 5 % additional per cycle     */
#define TCS_MAX_REDUCTION       0.80f   /* Maximum 80 % power cut       */
#define TCS_RECOVERY_RATE_PER_S 0.25f   /* 25 %/s recovery rate         */

void TCS_Update(void)
{
    /* Skip if TCS module is disabled (service mode) */
    if (!ServiceMode_IsEnabled(MODULE_TCS)) {
        safety_status.tcs_active = false;
        safety_status.tcs_wheel_mask = 0;
        for (uint8_t i = 0; i < 4; i++)
            tcs_reduction[i] = 0.0f;
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
        for (uint8_t i = 0; i < 4; i++)
            tcs_reduction[i] = 0.0f;
        return;
    }

    /* Delta time for recovery ramp (tcs_system.cpp uses millis delta) */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - tcs_last_tick) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;  /* Guard, same as base */
    tcs_last_tick = now;

    uint8_t mask = 0;
    for (uint8_t i = 0; i < 4; i++) {
        float slip = ((spd[i] - avg) * 100.0f) / avg;
        if (slip > (float)TCS_SLIP_THRESHOLD) {
            mask |= (1U << i);

            if (tcs_reduction[i] < 0.01f) {
                /* First activation — aggressive initial cut.
                 * tcs_system.cpp: aggressiveReduction = 40.0f          */
                tcs_reduction[i] = TCS_INITIAL_REDUCTION;
            } else {
                /* Already active — smooth progressive reduction.
                 * tcs_system.cpp: smoothReduction = 5.0f               */
                tcs_reduction[i] += TCS_SMOOTH_REDUCTION;
            }
            /* Clamp to maximum reduction (tcs_system.cpp: 80 %).       */
            if (tcs_reduction[i] > TCS_MAX_REDUCTION)
                tcs_reduction[i] = TCS_MAX_REDUCTION;
        } else {
            /* Slip under control — gradually recover power.
             * tcs_system.cpp: recoveryRatePerSec = 25.0f               */
            if (tcs_reduction[i] > 0.0f) {
                tcs_reduction[i] -= TCS_RECOVERY_RATE_PER_S * dt;
                if (tcs_reduction[i] < 0.0f)
                    tcs_reduction[i] = 0.0f;
            }
        }

        /* Compute per-wheel scale.  ABS_Update runs first and may have
         * already set wheel_scale[i] < 1.0.  Take the minimum of ABS
         * and TCS so the most restrictive intervention wins.           */
        float tcs_scale = 1.0f - tcs_reduction[i];
        if (tcs_scale < safety_status.wheel_scale[i])
            safety_status.wheel_scale[i] = tcs_scale;
    }

    if (mask) {
        safety_status.tcs_active = true;
        safety_status.tcs_wheel_mask = mask;
        safety_status.tcs_activation_count++;
        /* Global fallback: if ALL wheels spin, apply global limit as
         * last-resort safety (all traction lost).                     */
        if (mask == 0x0F) {
            Traction_SetDemand(Pedal_GetPercent() * (1.0f - TCS_MAX_REDUCTION));
        }
        /* Otherwise: per-wheel scale is applied in Traction_Update(). */
    } else {
        safety_status.tcs_active = false;
        safety_status.tcs_wheel_mask = 0;
    }
}

bool TCS_IsActive(void) { return safety_status.tcs_active; }
void TCS_Reset(void)    { safety_status.tcs_active = false; safety_status.tcs_wheel_mask = 0; for (uint8_t i = 0; i < 4; i++) tcs_reduction[i] = 0.0f; }

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
         * only when steering centering has completed successfully
         * AND the boot validation checklist has passed.                 */
        if (system_state == SYS_STATE_STANDBY &&
            safety_error == SAFETY_ERROR_NONE &&
            Steering_IsCalibrated() &&
            BootValidation_IsPassed()) {
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

/* ---- Battery undervoltage protection ----------------------------- */

/**
 * @brief  Check battery bus voltage and enforce undervoltage protection.
 *
 * Reads the INA226 on channel INA226_CHANNEL_BATTERY (24 V bus).
 * Voltage_GetBus() returns the last value sampled by Current_ReadAll()
 * in the 50 ms tier.  This function runs in the 100 ms tier.
 *
 * Warning (< 20.0 V):
 *   - Transition to SYS_STATE_DEGRADED
 *   - Power limited via DEGRADED_POWER_LIMIT_PCT (40 %)
 *   - Dynamic braking, park hold, and traction demand are reduced
 *     through the existing degraded-mode power limiter.
 *   - Recovery: voltage must exceed 20.5 V (0.5 V hysteresis).
 *
 * Critical (< 18.0 V or sensor failure):
 *   - Transition to SYS_STATE_SAFE
 *   - All traction outputs disabled (Safety_FailSafe)
 *   - Dynamic braking and park hold disabled (SAFE state inhibits)
 *   - Steering centering preserved if encoder is healthy
 *   - NO auto-recovery: operator must recharge and reset
 *
 * Sensor failure (0.0 V reading indicates I2C / multiplexer fault):
 *   - Treated as CRITICAL (fail-safe default)
 *
 * Works alongside overcurrent / overtemperature / CAN timeout:
 *   - Does not interfere with existing fault escalation
 *   - Only sets error if no higher-priority fault is active
 */
void Safety_CheckBatteryVoltage(void)
{
    float voltage = Voltage_GetBus(INA226_CHANNEL_BATTERY);

    /* Sensor failure: 0.0 V means TCA9548A channel select failed or
     * INA226 returned zero — treat as critical (fail-safe).           */
    if (voltage <= 0.0f) {
        Safety_SetError(SAFETY_ERROR_BATTERY_UV_CRITICAL);
        Safety_SetState(SYS_STATE_SAFE);
        return;
    }

    /* Critical undervoltage — SAFE state, no auto-recovery */
    if (voltage < BATTERY_UV_CRITICAL_V) {
        Safety_SetError(SAFETY_ERROR_BATTERY_UV_CRITICAL);
        Safety_SetState(SYS_STATE_SAFE);
        return;
    }

    /* Warning undervoltage — DEGRADED state with power limiting */
    if (voltage < BATTERY_UV_WARNING_V) {
        Safety_SetError(SAFETY_ERROR_BATTERY_UV_WARNING);
        Safety_SetState(SYS_STATE_DEGRADED);
        return;
    }

    /* Voltage OK — attempt recovery from DEGRADED if hysteresis met.
     * Recovery from DEGRADED requires voltage > WARNING + HYSTERESIS
     * (20.5 V) to prevent oscillation under load transients.
     *
     * Recovery from SAFE is intentionally NOT attempted here:
     * a critically depleted battery cannot reliably power actuators.
     * The operator must recharge and reset the system.                 */
    if (system_state == SYS_STATE_DEGRADED &&
        safety_error == SAFETY_ERROR_BATTERY_UV_WARNING &&
        voltage > (BATTERY_UV_WARNING_V + BATTERY_UV_HYST_V)) {
        Safety_ClearError(SAFETY_ERROR_BATTERY_UV_WARNING);
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

/* ================================================================== */
/*  Obstacle Safety (CAN-received from ESP32)                          */
/* ================================================================== */

/**
 * @brief  Process a CAN obstacle distance message (0x208) from ESP32.
 *
 * Called from CAN_ProcessMessages() when a frame with ID 0x208 arrives.
 * Stores the received values and updates the reception timestamp.
 * Stale-data detection compares the rolling counter — if the counter
 * has not changed for 3 consecutive receptions, the data is considered
 * frozen (ESP32 obstacle module may have crashed while the CAN driver
 * continues sending cached values).
 *
 * Payload (DLC ≥ 5):
 *   Byte 0-1: minimum distance (mm, uint16 little-endian)
 *   Byte 2:   zone level (0 = no obstacle, 1–5 = proximity zones)
 *   Byte 3:   sensor health (0 = unhealthy, 1 = healthy)
 *   Byte 4:   rolling counter (uint8, 0–255, must increment)
 */
void Obstacle_ProcessCAN(const uint8_t *data, uint8_t len)
{
    if (len < 5) return;

    /* Skip if obstacle detection module is disabled (service mode) */
    if (!ServiceMode_IsEnabled(MODULE_OBSTACLE_DETECT)) return;

    uint16_t dist = (uint16_t)(data[0] | (data[1] << 8));
    uint8_t  zone = data[2];
    uint8_t  health = data[3];
    uint8_t  counter = data[4];

    /* Stale-data detection: counter must change between frames.
     * If the counter is identical for 3 consecutive receptions,
     * treat the data as frozen / stale.                               */
    if (obstacle_data_valid && counter == obstacle_last_counter) {
        if (obstacle_stale_count < 255) obstacle_stale_count++;
    } else {
        obstacle_stale_count = 0;
    }
    obstacle_last_counter = counter;

    /* Zone plausibility: clamp to valid range */
    if (zone > 5) zone = 0;

    obstacle_distance_mm    = dist;
    obstacle_zone           = zone;
    obstacle_sensor_healthy = health;
    obstacle_last_rx_tick   = HAL_GetTick();
    obstacle_data_valid     = 1;
}

/**
 * @brief  Periodic obstacle safety check — called every 10 ms.
 *
 * Computes obstacle_scale from the last CAN-received distance and
 * applies timeout / stale-data detection.  This is the STM32's
 * independent backstop limiter — it does NOT replicate the ESP32's
 * full 5-zone logic.  The 3-tier mapping provides defence-in-depth.
 *
 * Safety actions:
 *   - Distance < 200 mm → obstacle_scale = 0.0, SAFE state
 *   - Distance 200–500 mm → obstacle_scale = 0.3
 *   - Distance 500–1000 mm → obstacle_scale = 0.7
 *   - Distance > 1000 mm → obstacle_scale = 1.0
 *   - CAN timeout (> 500 ms) → obstacle_scale = 0.0, SAFE state
 *   - Sensor unhealthy → obstacle_scale = 0.0, SAFE state
 *   - Stale data (counter frozen ≥ 3) → obstacle_scale = 0.0, SAFE
 *
 * Recovery from emergency:
 *   - Distance must exceed 500 mm for > 1 second
 *   - Sensor must be healthy
 *   - CAN messages must be arriving
 */
void Obstacle_Update(void)
{
    /* Skip if obstacle detection module is disabled (service mode) */
    if (!ServiceMode_IsEnabled(MODULE_OBSTACLE_DETECT)) {
        safety_status.obstacle_scale = 1.0f;
        return;
    }

    uint32_t now = HAL_GetTick();

    /* ---- Check for CAN timeout ---- */
    if (obstacle_data_valid &&
        (now - obstacle_last_rx_tick) > OBSTACLE_CAN_TIMEOUT_MS) {
        /* No obstacle data for > 500 ms — assume sensor failure */
        safety_status.obstacle_scale = 0.0f;
        obstacle_in_emergency = 1;
        ServiceMode_SetFault(MODULE_OBSTACLE_DETECT, MODULE_FAULT_ERROR);
        Safety_SetError(SAFETY_ERROR_OBSTACLE);
        Safety_SetState(SYS_STATE_SAFE);
        return;
    }

    /* ---- No data ever received — not yet initialised ---- */
    if (!obstacle_data_valid) {
        /* Before first obstacle message, allow normal operation.
         * The ESP32 may not have booted its obstacle module yet.
         * The heartbeat timeout (250 ms) provides protection.         */
        safety_status.obstacle_scale = 1.0f;
        return;
    }

    /* ---- Stale-data detection ---- */
    if (obstacle_stale_count >= 3) {
        /* Counter has not changed for 3+ frames — data is frozen */
        safety_status.obstacle_scale = 0.0f;
        obstacle_in_emergency = 1;
        ServiceMode_SetFault(MODULE_OBSTACLE_DETECT, MODULE_FAULT_ERROR);
        Safety_SetError(SAFETY_ERROR_OBSTACLE);
        Safety_SetState(SYS_STATE_SAFE);
        return;
    }

    /* ---- Sensor health check ---- */
    if (!obstacle_sensor_healthy) {
        /* ESP32 reports sensor failure — fail-safe */
        safety_status.obstacle_scale = 0.0f;
        obstacle_in_emergency = 1;
        ServiceMode_SetFault(MODULE_OBSTACLE_DETECT, MODULE_FAULT_ERROR);
        Safety_SetError(SAFETY_ERROR_OBSTACLE);
        Safety_SetState(SYS_STATE_SAFE);
        return;
    }

    /* ---- 3-tier distance → scale mapping ---- */
    float scale;
    if (obstacle_distance_mm < OBSTACLE_EMERGENCY_MM) {
        /* Zone 5 equivalent: full stop + SAFE state */
        scale = 0.0f;
        obstacle_in_emergency = 1;
        Safety_SetError(SAFETY_ERROR_OBSTACLE);
        Safety_SetState(SYS_STATE_SAFE);
    } else if (obstacle_distance_mm < OBSTACLE_CRITICAL_MM) {
        /* Zone 4 equivalent: heavy braking */
        scale = 0.3f;
        obstacle_in_emergency = 0;
        obstacle_recovery_tick = 0;
    } else if (obstacle_distance_mm < OBSTACLE_WARNING_MM) {
        /* Zone 3 equivalent: moderate reduction */
        scale = 0.7f;
        obstacle_in_emergency = 0;
        obstacle_recovery_tick = 0;
    } else {
        /* Zone 1-2 / no obstacle: full power */
        scale = 1.0f;
        obstacle_in_emergency = 0;
        obstacle_recovery_tick = 0;
    }

    /* ---- Emergency recovery with hysteresis ---- */
    if (obstacle_in_emergency &&
        obstacle_distance_mm >= OBSTACLE_RECOVERY_MM &&
        obstacle_sensor_healthy &&
        obstacle_stale_count == 0) {
        /* Distance has cleared — start recovery debounce */
        if (obstacle_recovery_tick == 0) {
            obstacle_recovery_tick = now;
        } else if ((now - obstacle_recovery_tick) >= OBSTACLE_RECOVERY_MS) {
            /* Sustained clearance — recover from emergency */
            obstacle_in_emergency = 0;
            obstacle_recovery_tick = 0;
            ServiceMode_ClearFault(MODULE_OBSTACLE_DETECT);
            Safety_ClearError(SAFETY_ERROR_OBSTACLE);
            /* Scale will be recomputed on next iteration */
            scale = 1.0f;
        }
    } else if (obstacle_in_emergency) {
        /* Still in danger zone — reset recovery timer */
        obstacle_recovery_tick = 0;
    }

    /* ---- Clear fault when operating normally ---- */
    if (!obstacle_in_emergency && scale >= 1.0f) {
        ServiceMode_ClearFault(MODULE_OBSTACLE_DETECT);
        if (safety_error == SAFETY_ERROR_OBSTACLE) {
            Safety_ClearError(SAFETY_ERROR_OBSTACLE);
        }
    }

    safety_status.obstacle_scale = scale;
}

/**
 * @brief  Get the current obstacle torque scale factor.
 * @return 0.0–1.0 (0.0 = full stop, 1.0 = no obstacle reduction)
 */
float Obstacle_GetScale(void)
{
    return safety_status.obstacle_scale;
}