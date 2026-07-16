/**
  ****************************************************************************
  * @file    steering_centering.c
  * @brief   Automatic steering centering — implementation
  *
  * Centering sequence:
  *   1. Begin sweeping LEFT at low PWM (~10 %).
  *   2. While sweeping, monitor:
  *      a) Inductive center sensor pulse → center found.
  *      b) Encoder stall (count unchanged for STALL_TIMEOUT_MS while
  *         motor is commanded) → end-of-travel reached.
  *      c) Total timeout (TOTAL_TIMEOUT_MS) → abort.
  *      d) Encoder range exceeded (MAX_CENTERING_COUNTS) → abort.
  *   3. If end-of-travel stall detected during LEFT sweep, reverse to
  *      sweep RIGHT.
  *   4. If end-of-travel stall detected during RIGHT sweep without
  *      center found → FAULT.
  *   5. When center pulse detected:
  *      a) Immediately stop the motor (PWM = 0, disable H-bridge).
  *      b) Zero the encoder counter at this position.
  *      c) Call Steering_SetCalibrated() to unlock normal operation.
  *      d) Transition to CENTERING_DONE.
  *   6. On any fault:
  *      a) Neutralise the steering motor.
  *      b) Latch SAFETY_ERROR_CENTERING.
  *      c) Transition system to SAFE state.
  *
  * Safety limits:
  *   - CENTERING_PWM           ~425  (10 % of 4249 full scale)
  *   - STALL_TIMEOUT_MS        300   (encoder frozen while driving)
  *   - TOTAL_TIMEOUT_MS        10000 (absolute deadline)
  *   - MAX_CENTERING_COUNTS    6000  (guard against runaway travel)
  *
  * Failure conditions:
  *   - Center pulse not detected after sweeping both directions.
  *   - Total timeout exceeded.
  *   - Encoder range exceeded.
  *   - Encoder fault (detected by Encoder_CheckHealth).
  ****************************************************************************
  */

#include "steering_centering.h"
#include "steering_centering_diag.h"
#include "steering_eps.h"
#include "motor_control.h"
#include "sensor_manager.h"
#include "safety_system.h"
#include "service_mode.h"
#include "steering_cal_store.h"
#include "steering_z.h"
#include "encoder_reader.h"
#include "main.h"
#include <math.h>

/* ---- Centering constants ----
 *
 * CENTERING_PWM: ~10 % of full PWM scale (4249 at 20 kHz center-aligned).
 * Intentionally LOW to avoid forcing the rack.  Positioning only, not power.
 *
 * STALL_TIMEOUT_MS: If the encoder does not change for this duration
 * while the motor is being driven, we infer end-of-travel (no
 * mechanical endstop assumption — purely encoder-based).
 *
 * TOTAL_TIMEOUT_MS: If centering is not complete within this time,
 * something is wrong.  Abort and enter SAFE state.
 *
 * MAX_CENTERING_COUNTS: Maximum encoder travel in one direction before
 * we conclude the center sensor is not going to be found (guard).      */

#define CENTERING_PWM              425U    /* ~10 % of 4249             */
#define STALL_TIMEOUT_MS           300U    /* End-of-travel stall (ms)  */
#define TOTAL_TIMEOUT_MS           10000U  /* Absolute deadline (ms)    */
#define MAX_CENTERING_COUNTS       6000    /* Safety range limit        */

/* Steering BTS7960 power rail settle after energising PIN_RELAY_STEER_PWR.
 * Matches RELAY_TRACTION_SETTLE_MS used by Relay_SequencerUpdate
 * (safety_system.c) for arc suppression on SRD-style relays.
 * Reason: SteeringCentering_Step() runs in BOOT/STANDBY, but the main
 * relay sequencer only energises PIN_RELAY_STEER_PWR in SYS_STATE_ACTIVE,
 * so the BTS7960 steering rail would otherwise be unpowered during
 * the homing sweep.  We drive PIN_RELAY_STEER_PWR locally and wait this
 * settle window before emitting any PWM to the BTS7960.                */
#define STEERING_RAIL_SETTLE_MS    50U     /* DIR relay settle (ms)     */

/* ---- Module state ---- */
static CenteringState_t centering_state = CENTERING_IDLE;
static uint32_t centering_start_tick    = 0;   /* Tick when centering began   */
static int32_t  stall_prev_count        = 0;   /* Last encoder reading        */
static uint32_t stall_last_change_tick  = 0;   /* Tick when encoder last moved*/
static int32_t  sweep_origin_count      = 0;   /* Encoder at sweep start      */
static uint32_t rail_settle_tick        = 0;   /* Tick when DIR relay energised*/

/* ---- Diagnostics state (Problem 1 instrumentation, read-only) ---- */
static SteeringCenteringDiag s_diag = { 0 };   /* Latest classified snapshot  */
static bool s_center_already_active = false;   /* PB5 active when sweep began */
static bool s_restored_from_flash   = false;   /* Homing skipped via flash    */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* ---- Private helpers ---- */

/**
 * @brief  Abort centering: isolate the steering assist, leave mechanical
 *         steering.  Does NOT degrade the global vehicle state.
 *
 * A failed centre search is an ISOLABLE ASSIST fault: centering is only
 * required to enable the electric assist, never to move the car with
 * mechanical steering.  We therefore disconnect the motor (PA6=PA7=0,
 * PC4=LOW, PC12=OFF, owner=NONE, EPS mechanical-only) via the EPS authority
 * and latch CENTERING_FAULT so the sweep never restarts.  The vehicle stays
 * ACTIVE with full traction; the failure is reported locally (0x316 / HMI),
 * NOT as DEGRADED / LIMP_HOME / SAFE / ERROR.
 *
 * @param  reason  EPS isolation cause for this abort.
 */
static void Centering_Abort(EpsFaultReason_t reason)
{
    centering_state = CENTERING_FAULT;
    Steering_DisableAssistFault(reason);
}

/**
 * @brief  Complete centering: stop motor, zero encoder, mark calibrated.
 *         Persists the calibration to flash for fast startup next boot.
 */
static void Centering_Complete(void)
{
    Steering_Neutralize();

    /* ---- Fase 3: PB5 + Z dual reference ----
     * PB5 has just confirmed the physical center.  BEFORE zeroing TIM2,
     * capture the current count (= count at center) and the last Z pulse
     * position (captured during the sweep, same frame) so we can compute
     * the Z↔center offset.  Z is a SECONDARY reference only — it never
     * centers on its own and never blocks completion.                  */
    int32_t center_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    bool    z_seen       = (Encoder_Z_GetPulseCount() > 0U);
    SteeringZ_OnCenterConfirmed(center_count,
                                Encoder_Z_GetLastPosition(),
                                z_seen);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    Steering_SetCalibrated();
    SteeringCenter_ClearFlag();
    centering_state = CENTERING_DONE;

    /* Persist calibration to flash.
     * Conditions: centering just succeeded, vehicle is in BOOT/STANDBY
     * (speed must be 0), and no safety errors at this point.
     * Failure to save is non-critical — next boot will just re-sweep.
     * The Z offset/validity is persisted alongside the PB5 center so the
     * secondary reference survives a power cycle (Fase 4).             */
    if (Safety_GetError() == SAFETY_ERROR_NONE) {
        int32_t center = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        (void)SteeringCal_SaveWithZ(center,
                                    SteeringZ_GetOffset(),
                                    SteeringZ_IsValid(),
                                    (int32_t)STEERING_Z_WINDOW_COUNTS);
    }
}

/**
 * @brief  Check if the encoder has stalled (end-of-travel inference).
 * @retval true if the encoder count has not changed for STALL_TIMEOUT_MS.
 */
static bool Centering_IsStalled(void)
{
    int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint32_t now  = HAL_GetTick();

    if (count != stall_prev_count) {
        stall_prev_count      = count;
        stall_last_change_tick = now;
        return false;
    }

    return ((now - stall_last_change_tick) >= STALL_TIMEOUT_MS);
}

/**
 * @brief  Check if the encoder has exceeded the safe centering range.
 * @retval true if |current_count - sweep_origin| > MAX_CENTERING_COUNTS.
 */
static bool Centering_RangeExceeded(void)
{
    int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t delta = count - sweep_origin_count;
    if (delta < 0) delta = -delta;
    return (delta > MAX_CENTERING_COUNTS);
}

/* ==================================================================
 *  Public API
 * ================================================================== */

void SteeringCentering_Init(void)
{
    centering_state = CENTERING_IDLE;
    s_center_already_active = false;
    s_restored_from_flash   = false;
    SteeringZ_Init();
    SteeringCenter_ClearFlag();
}

void SteeringCentering_Step(void)
{
    /* Once centering is done or faulted, do nothing further. */
    if (centering_state == CENTERING_DONE ||
        centering_state == CENTERING_FAULT) {
        return;
    }

    /* If the encoder is already faulted, abort immediately. */
    if (Encoder_HasFault()) {
        Centering_Abort(EPS_FAULT_ENCODER_AB);
        return;
    }

    uint32_t now = HAL_GetTick();

    switch (centering_state) {

    /* ---- IDLE: first call energises steering power rail ----
     *
     * The BTS7960 steering H-bridge gets its 12 V from PIN_RELAY_STEER_PWR
     * (PC12).  In BOOT/STANDBY the main relay sequencer does NOT
     * energise this relay (its hard gate requires SYS_STATE_ACTIVE,
     * see safety_system.c:Relay_SequencerUpdate).  We therefore drive
     * the DIR relay locally here, BEFORE emitting any PWM, and wait
     * STEERING_RAIL_SETTLE_MS for the relay contacts to settle.
     *
     * Idempotent w.r.t. Relay_PowerUp/Relay_SequencerUpdate: when the
     * system later transitions to ACTIVE, the sequencer re-writes
     * PIN_RELAY_STEER_PWR to SET (same value) — no glitch.  On any path that
     * powers the vehicle down (Safety_FailSafe / Safety_PowerDown /
     * Safety_EmergencyStop / Relay_PowerDown), the atomic BSRR reset
     * in safety_system.c forces this relay back OFF, so steering can
     * never remain energised in SAFE/ERROR.                            */
    case CENTERING_IDLE:
        /* Never re-close the steering-motor relay (PC12) once the EPS assist
         * has been isolated (MECHANICAL_ONLY / ELECTRICAL_HAZARD).  The latch
         * holds for the whole power cycle, so homing must not re-energise the
         * motor branch even if it is somehow re-entered.  Steering stays
         * purely mechanical. */
        if (Steering_IsMechanicalOnly()) {
            Centering_Abort(EPS_FAULT_UNKNOWN);
            break;
        }
        SteeringCenter_ClearFlag();
        HAL_GPIO_WritePin(GPIOC, PIN_RELAY_STEER_PWR, GPIO_PIN_SET);
        rail_settle_tick = now;
        centering_state  = CENTERING_WAIT_RAIL;
        break;

    /* ---- WAIT_RAIL: DIR relay energised, hold off PWM until settle ---- */
    case CENTERING_WAIT_RAIL:
        if ((now - rail_settle_tick) >= STEERING_RAIL_SETTLE_MS) {
            centering_start_tick   = now;
            stall_prev_count       = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
            stall_last_change_tick = now;
            sweep_origin_count     = stall_prev_count;

            /* Latch whether PB5 is already asserted at the instant the sweep
             * begins.  The inductive centre sensor is active-LOW (falling
             * edge → screw in front of the LJ12A3), so a LOW level here means
             * the rack is reported "at centre" before we have moved — a
             * suspicious physical condition surfaced by the classifier as
             * STEER_DIAG_CENTER_SENSOR_ALREADY_ACTIVE.  Read-only. */
            s_center_already_active =
                (HAL_GPIO_ReadPin(GPIOB, PIN_STEER_CENTER) == GPIO_PIN_RESET);

            /* Begin sweeping LEFT (direction = reverse = true → negative) */
            Motor_SetPWM_Steering(CENTERING_PWM, true);
            centering_state = CENTERING_SWEEP_LEFT;
        }
        break;

    /* ---- SWEEP LEFT ---- */
    case CENTERING_SWEEP_LEFT:
        /* 1. Center detected? */
        if (SteeringCenter_Detected()) {
            Centering_Complete();
            return;
        }

        /* 2. Total timeout? */
        if ((now - centering_start_tick) >= TOTAL_TIMEOUT_MS) {
            Centering_Abort(EPS_FAULT_CENTER_NOT_FOUND);
            return;
        }

        /* 3. Range exceeded? */
        if (Centering_RangeExceeded()) {
            Centering_Abort(EPS_FAULT_CENTER_NOT_FOUND);
            return;
        }

        /* 4. End-of-travel stall? → reverse to sweep right */
        if (Centering_IsStalled()) {
            sweep_origin_count     = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
            stall_prev_count       = sweep_origin_count;
            stall_last_change_tick = now;
            Motor_SetPWM_Steering(CENTERING_PWM, false);
            centering_state = CENTERING_SWEEP_RIGHT;
        }
        break;

    /* ---- SWEEP RIGHT ---- */
    case CENTERING_SWEEP_RIGHT:
        /* 1. Center detected? */
        if (SteeringCenter_Detected()) {
            Centering_Complete();
            return;
        }

        /* 2. Total timeout? */
        if ((now - centering_start_tick) >= TOTAL_TIMEOUT_MS) {
            Centering_Abort(EPS_FAULT_CENTER_NOT_FOUND);
            return;
        }

        /* 3. Range exceeded? */
        if (Centering_RangeExceeded()) {
            Centering_Abort(EPS_FAULT_CENTER_NOT_FOUND);
            return;
        }

        /* 4. End-of-travel stall? → center was never found */
        if (Centering_IsStalled()) {
            Centering_Abort(EPS_FAULT_CENTER_NOT_FOUND);
        }
        break;

    default:
        break;
    }
}

bool SteeringCentering_IsComplete(void)
{
    return (centering_state == CENTERING_DONE);
}

bool SteeringCentering_HasFault(void)
{
    return (centering_state == CENTERING_FAULT);
}

CenteringState_t SteeringCentering_GetState(void)
{
    return centering_state;
}

SteeringMotorOwner_t SteeringCentering_DecideOwner(CenteringState_t state,
                                                   bool in_homing_state)
{
    SteeringMotorOwner_t owner;

    /* An isolated assist fault takes absolute priority: nobody drives the
     * motor and steering is purely mechanical.                            */
    if (Steering_IsMechanicalOnly()) {
        owner = STEER_OWNER_NONE;
    } else if (!in_homing_state) {
        /* Centering is the exclusive writer of the steering motor only while
         * it is actively homing AND the system is still in a homing-capable
         * state.  In DONE/FAULT — or once the system has left BOOT/STANDBY
         * (e.g. SAFE/ERROR) — the EPS loop owns the motor and neutralises it
         * as needed.                                                        */
        owner = STEER_OWNER_EPS;
    } else {
        switch (state) {
        case CENTERING_IDLE:
        case CENTERING_WAIT_RAIL:
        case CENTERING_SWEEP_LEFT:
        case CENTERING_SWEEP_RIGHT:
            owner = STEER_OWNER_CENTERING;
            break;

        case CENTERING_DONE:
        case CENTERING_FAULT:
        default:
            owner = STEER_OWNER_EPS;
            break;
        }
    }

    /* SINGLE OWNER AUTHORITY: commit the decision to the EPS authority
     * (steering_eps.c::s_owner) — the one and only stored owner — and return
     * exactly what it holds.  Keeping the decision here (and consuming it
     * BEFORE SteeringCentering_Step() runs) still guarantees the two
     * subsystems never both write the motor in the same cycle, while the main
     * loop, the 0x316 diagnostic and the CAN telemetry now all read one
     * identical owner.  While a MECHANICAL_ONLY / ELECTRICAL_HAZARD isolation
     * is latched the authority forces STEER_OWNER_NONE.                     */
    Steering_EpsSetOwner(owner);
    return Steering_EpsGetOwner();
}


void SteeringCentering_MarkRestoredFromFlash(int32_t stored_center)
{
    /* Apply the stored center value: set the TIM2 counter so that
     * the encoder reference frame matches the original calibration.
     * TIM2 is a 32-bit counter; casting int32_t → uint32_t preserves
     * the bit pattern (two's complement), which is correct for
     * quadrature mode where the counter can wrap in both directions.
     * In practice, centering zeroes the counter, so the stored value
     * is typically 0.                                                */
    __HAL_TIM_SET_COUNTER(&htim2, (uint32_t)stored_center);
    Steering_SetCalibrated();
    /* Restore the secondary Z reference from flash (Fase 5).  Z is
     * auxiliary: an absent/invalid Z calibration never blocks the
     * PB5-authorised restore — it is surfaced as diagnostic only.      */
    SteeringZ_LoadFromFlash(SteeringCal_GetStoredZOffset(),
                            SteeringCal_IsStoredZValid());
    centering_state = CENTERING_DONE;
    s_restored_from_flash = true;
}

/* ==================================================================
 *  Homing diagnostics (Problem 1 integration) — read-only
 * ================================================================== */

/**
 * @brief  Map the safety SystemState_t onto the classifier's frozen
 *         STEER_DIAG_SS_* numeric codes (kept independent so the pure
 *         classifier never pulls in HAL/safety types).
 */
static uint8_t Diag_MapSystemState(SystemState_t ss)
{
    switch (ss) {
    case SYS_STATE_BOOT:      return STEER_DIAG_SS_BOOT;
    case SYS_STATE_STANDBY:   return STEER_DIAG_SS_STANDBY;
    case SYS_STATE_ACTIVE:    return STEER_DIAG_SS_ACTIVE;
    case SYS_STATE_DEGRADED:  return STEER_DIAG_SS_DEGRADED;
    case SYS_STATE_SAFE:      return STEER_DIAG_SS_SAFE;
    case SYS_STATE_ERROR:     return STEER_DIAG_SS_ERROR;
    case SYS_STATE_LIMP_HOME: return STEER_DIAG_SS_LIMP_HOME;
    default:                  return STEER_DIAG_SS_BOOT;
    }
}

void SteeringCentering_UpdateDiag(void)
{
    uint32_t now = HAL_GetTick();

    SystemState_t sys = Safety_GetState();
    bool in_homing = (sys == SYS_STATE_BOOT) || (sys == SYS_STATE_STANDBY);
    bool sweeping = (centering_state == CENTERING_SWEEP_LEFT) ||
                    (centering_state == CENTERING_SWEEP_RIGHT);

    int32_t enc = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

    s_diag.fsm_state    = centering_state;
    s_diag.motor_owner  = SteeringCentering_DecideOwner(centering_state,
                                                        in_homing);
    s_diag.system_state = Diag_MapSystemState(sys);

    /* PB5 is active-LOW; raw level LOW ⇒ centre screw detected. */
    s_diag.center_sensor_raw =
        (HAL_GPIO_ReadPin(GPIOB, PIN_STEER_CENTER) == GPIO_PIN_RESET);
    s_diag.center_sensor_debounced      = SteeringCenter_Detected();
    s_diag.center_sensor_already_active = s_center_already_active;

    s_diag.encoder_count = enc;
    s_diag.encoder_delta = enc - sweep_origin_count;

    /* PC12 steering-rail relay and PC4 EN_STEER are GPIO outputs; reading
     * the pin returns the commanded (ODR) level. */
    s_diag.relay_steer_commanded =
        (HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR) == GPIO_PIN_SET);
    s_diag.power_ready        = Safety_IsPowerReady();
    s_diag.enable_commanded   =
        (HAL_GPIO_ReadPin(GPIOC, PIN_EN_STEER) == GPIO_PIN_SET);

    s_diag.pwm_requested   = sweeping ? CENTERING_PWM : 0U;
    s_diag.pwm_applied_ch1 =
        (uint16_t)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    s_diag.pwm_applied_ch2 =
        (uint16_t)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);

    s_diag.elapsed_ms =
        (centering_start_tick != 0U) ? (now - centering_start_tick) : 0U;
    s_diag.last_encoder_change_ms = now - stall_last_change_tick;

    s_diag.encoder_fault      = Encoder_HasFault();
    s_diag.restored_from_flash = s_restored_from_flash;
    s_diag.module_disabled     = !ServiceMode_IsEnabled(MODULE_STEER_CENTER);
    s_diag.fault_latched       =
        (Safety_GetError() == SAFETY_ERROR_CENTERING) ||
        (centering_state == CENTERING_FAULT);

    /* Never invent the cause — derive it from the captured numbers only. */
    s_diag.abort_reason = SteeringCentering_ClassifyDiag(&s_diag);
}

const SteeringCenteringDiag *SteeringCentering_GetDiag(void)
{
    return &s_diag;
}
