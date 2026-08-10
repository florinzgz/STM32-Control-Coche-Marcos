/**
  ****************************************************************************
  * @file    steering_centering_patched.c
  * @brief   Effective steering-homing policy for the installed RS390 gearbox.
  *
  * The productive FSM remains in steering_centering.c.  This adapter provides:
  *   - a full-authority search command (100 % PWM) to overcome the installed
  *     gearbox static friction;
  *   - a fast encoder+CH5 end-stop guard;
  *   - mandatory zero-PWM dead-time before LEFT->RIGHT reversal;
  *   - stable active-low PB5 level acceptance, including boot over the target;
  *   - a fresh physical PB5 search on every power cycle;
  *   - immediate CENTERING->NONE->EPS owner transfer at completion.
  *
  * Full PWM does not mean sustained force.  Two fresh hard-current samples, or
  * two fresh lower-current samples after confirmed encoder standstill, remove
  * torque first.  The normal encoder stall/range/timeout protections remain
  * active as independent fallbacks.
  ****************************************************************************
  */

#include "steering_centering.h"
#include "steering_eps.h"
#include "steering_supervisor.h"
#include "sensor_manager.h"
#include "safety_system.h"
#include "motor_control.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <limits.h>

/* HOMING_PWM_COUNTS/HOMING_PWM_REDUCED_PERCENT/HOMING_PWM_REDUCED_COUNTS now
 * live in steering_centering.h (Bloque A / V2 audit, PR #445) so
 * steering_service_store.h can _Static_assert against the true safety floor
 * instead of a duplicated magic number.  Values/semantics unchanged.       */
#define CENTER_RAW_CONFIRM_CYCLES        3U
#define REVERSE_DEADTIME_MS              100U
#define ENDSTOP_NO_MOTION_MS             100U
#define ENDSTOP_SAMPLE_MAX_AGE_MS        250U
#define ENDSTOP_ENCODER_DELTA_COUNTS     2
#define ENDSTOP_CONFIRM_SAMPLES           2U
/* The hard threshold does not require encoder standstill, but still requires
 * two genuinely fresh CH5 acquisitions to reject a single PWM/noise spike.
 * The lower threshold additionally requires confirmed no-motion. */
#define ENDSTOP_HARD_CURRENT_MA          20000U
#define ENDSTOP_STALL_CURRENT_MA          8000U

/* ---- Bloque 2 (P1 audit fix) -------------------------------------------
 * "Homing a 100% PWM sin guarda efectiva": endstop_pressure_detected() below
 * can only observe end-of-travel pressure while the CH5 current reading is
 * trustworthy.  INA226_CH_PRESENT_NO_SHUNT still ACKs and returns a "valid"
 * shunt read, but the measured current is structurally ~0 A, so the guard
 * silently never fires — full 100% authority could then push against a
 * mechanical stop for up to TOTAL_TIMEOUT_MS (10 s) with only the encoder
 * stall/range fallbacks left, both of which a jittering encoder can defeat.
 *
 * Fix (minimal, does not redesign the FSM):
 *   (a) HOMING_SWEEP_TIMEOUT_MS - a wall-clock ceiling per sweep leg,
 *       independent of encoder motion and of CH5, mirroring the existing
 *       end-stop guard's reversal/abort behaviour on expiry.
 *   (b) HOMING_PWM_REDUCED_PERCENT - full 4249-count (100%) authority is
 *       only applied while current_guard_armed() confirms CH5 is genuinely
 *       healthy; otherwise homing falls back to this safer percentage.
 *   (c) current_guard_armed() is also latched into the 0x316 telemetry
 *       (SteeringCenteringDiag.current_guard_armed) so the "guard inert"
 *       state is observable instead of silent.
 *   (d) INA226_CH_PRESENT_NO_SHUNT keeps classifying as EPS_FAULT_NONE in
 *       SteeringSupervisor_Ch5ToEpsFault() (steering_supervisor.c) — the
 *       manual/mechanical steering path must keep working; only the homing
 *       PWM policy and its diagnostics change here.                       */
#define HOMING_SWEEP_TIMEOUT_MS         1000U

_Static_assert(CENTER_RAW_CONFIRM_CYCLES >= 2U,
               "PB5 level fallback must debounce EMI");
_Static_assert(ENDSTOP_CONFIRM_SAMPLES >= 2U,
               "end-stop pressure must reject an isolated current spike");
_Static_assert(HOMING_PWM_REDUCED_PERCENT >= 40U &&
               HOMING_PWM_REDUCED_PERCENT <= 100U,
               "reduced homing PWM must stay a safe fraction of full authority");
_Static_assert(HOMING_SWEEP_TIMEOUT_MS >= 600U && HOMING_SWEEP_TIMEOUT_MS <= 1000U,
               "max full-authority sweep-leg guard must stay in the mandated "
               "600-1000ms band");

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

static void Homing_SetPwm(uint16_t requested_pwm, bool reverse);
void Homing_BaseInit(void);
void Homing_BaseStep(void);
void Homing_BaseUpdateDiag(void);
void Homing_BaseMarkRestored(int32_t stored_center);

static bool HomingPowerReady(void)
{
    CenteringState_t state = SteeringCentering_GetState();
    bool sweeping = state == CENTERING_SWEEP_LEFT ||
                    state == CENTERING_SWEEP_RIGHT;
    return sweeping &&
        HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR) == GPIO_PIN_SET;
}

#define Safety_IsPowerReady                     HomingPowerReady
#define Motor_SetPWM_Steering                   Homing_SetPwm
#define SteeringCentering_Init                  Homing_BaseInit
#define SteeringCentering_Step                  Homing_BaseStep
#define SteeringCentering_UpdateDiag            Homing_BaseUpdateDiag
#define SteeringCentering_MarkRestoredFromFlash Homing_BaseMarkRestored
#include "steering_centering.c"
#undef SteeringCentering_MarkRestoredFromFlash
#undef SteeringCentering_UpdateDiag
#undef SteeringCentering_Step
#undef SteeringCentering_Init
#undef Motor_SetPWM_Steering
#undef Safety_IsPowerReady

/**
 * @brief  Bloque 2 (P1 audit fix): is the CH5 current end-stop guard armed?
 *
 *         "Armed" means endstop_pressure_detected() can actually observe a
 *         genuine end-of-travel current rise: CH5 ACKs, the shunt read is
 *         valid, the sample is fresh (same freshness window the guard
 *         itself requires) AND the channel classifies as fully healthy
 *         (INA226_CH_OK).  The classifier check is the important part —
 *         INA226_CH_PRESENT_NO_SHUNT also ACKs and reports shunt_read_ok,
 *         yet the shunt drop stays ~0 V so current always reads ~0 A and the
 *         guard can never cross ENDSTOP_STALL_CURRENT_MA /
 *         ENDSTOP_HARD_CURRENT_MA.  Pure read-only query; drives nothing. */
static bool current_guard_armed(void)
{
    const Ina226ChannelDiag *diag = Sensor_GetChannel5Diag();
    return diag != NULL && diag->i2c_ack && diag->shunt_read_ok &&
           diag->sample_age_ms <= ENDSTOP_SAMPLE_MAX_AGE_MS &&
           diag->fault_reason == INA226_CH_OK;
}

static void Homing_SetPwm(uint16_t requested_pwm, bool reverse)
{
    /* Full 100% authority is reserved for a confirmed-healthy CH5, because
     * only then can the fast current guard above stop it quickly.  Whenever
     * that guard is not armed, fall back to HOMING_PWM_REDUCED_COUNTS so the
     * new HOMING_SWEEP_TIMEOUT_MS wall-clock guard (see
     * handle_sweep_leg_timeout()) is the sole, but sufficient, backstop. */
    const uint16_t applied = requested_pwm == 0U ? 0U :
        (current_guard_armed() ? HOMING_PWM_COUNTS : HOMING_PWM_REDUCED_COUNTS);
    Motor_SetPWM_Steering(applied, reverse);
}

static uint8_t s_center_raw_cycles;

/* The host HAL GPIO objects are translation-unit local by design.  This weak,
 * test-only hook lets the integration test inject one shared PB5 level without
 * changing the production read path.  When no host override is linked, the
 * exact real GPIO read below remains the fallback. */
#ifdef HOST_TEST
extern bool SteeringCentering_TestPb5Active(void) __attribute__((weak));

/* Marker used only by the shared test source.  The repository's legacy host
 * suite also links that test against steering_centering.c directly; this symbol
 * lets it skip wrapper-specific cases while the PR workflow still executes
 * them against this effective production translation unit. */
bool SteeringCentering_EffectiveWrapperPresent(void) { return true; }
#endif

static bool read_pb5_active(void)
{
#ifdef HOST_TEST
    if (SteeringCentering_TestPb5Active != NULL) {
        return SteeringCentering_TestPb5Active();
    }
#endif
    return HAL_GPIO_ReadPin(GPIOB, PIN_STEER_CENTER) == GPIO_PIN_RESET;
}

static bool center_raw_stable(void)
{
    /* PB5 is the mandatory physical centre reference for every homing run.
     * Service mode may suppress optional diagnostics, but it must never make
     * the homing FSM blind to an active centre sensor (including boot-over-PB5,
     * where no new EXTI edge is guaranteed). */
    const bool active = read_pb5_active();

    if (!active) {
        s_center_raw_cycles = 0U;
        return false;
    }

    if (s_center_raw_cycles < CENTER_RAW_CONFIRM_CYCLES) {
        s_center_raw_cycles++;
    }
    return s_center_raw_cycles >= CENTER_RAW_CONFIRM_CYCLES;
}

static bool is_sweep(CenteringState_t state)
{
    return state == CENTERING_SWEEP_LEFT || state == CENTERING_SWEEP_RIGHT;
}

static uint32_t current_magnitude_ma(int32_t current_ma)
{
    return current_ma < 0
        ? (uint32_t)(-(int64_t)current_ma)
        : (uint32_t)current_ma;
}

static int32_t abs_count_delta(int32_t a, int32_t b)
{
    int64_t d = (int64_t)a - (int64_t)b;
    if (d < 0) d = -d;
    if (d > INT32_MAX) return INT32_MAX;
    return (int32_t)d;
}

typedef struct {
    bool initialized;
    CenteringState_t state;
    int32_t reference_count;
    uint32_t last_motion_ms;
    uint32_t last_sample_sequence;
    uint8_t hard_current_samples;
    uint8_t stall_current_samples;
    bool reverse_pending;
    uint32_t reverse_started_ms;
    /* Bloque 2: wall-clock start of the CURRENT sweep leg.  Unlike
     * last_motion_ms, this is set ONLY once per new leg (reset_observation())
     * and is never refreshed by encoder motion, so a jittering encoder
     * (small counts changing <300ms apart while physically stalled against
     * the mechanical stop) cannot keep resetting it and defeat the guard. */
    uint32_t leg_start_ms;
} HomingGuard;

static HomingGuard s_guard;

static void reset_pressure_samples(void)
{
    s_guard.hard_current_samples = 0U;
    s_guard.stall_current_samples = 0U;
}

static void reset_observation(CenteringState_t state)
{
    s_guard.initialized = true;
    s_guard.state = state;
    s_guard.reference_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    s_guard.last_motion_ms = HAL_GetTick();
    s_guard.last_sample_sequence = 0U;
    s_guard.leg_start_ms = s_guard.last_motion_ms;
    reset_pressure_samples();
}

static void reset_guard(void)
{
    s_guard.initialized = false;
    s_guard.state = CENTERING_IDLE;
    s_guard.reference_count = 0;
    s_guard.last_motion_ms = 0U;
    s_guard.last_sample_sequence = 0U;
    reset_pressure_samples();
    s_guard.reverse_pending = false;
    s_guard.reverse_started_ms = 0U;
    s_guard.leg_start_ms = 0U;
}

static bool endstop_pressure_detected(void)
{
    const CenteringState_t state = centering_state;
    if (!is_sweep(state)) {
        s_guard.initialized = false;
        reset_pressure_samples();
        return false;
    }

    const uint32_t now = HAL_GetTick();
    const int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

    if (!s_guard.initialized || s_guard.state != state) {
        reset_observation(state);
        return false;
    }

    if (abs_count_delta(count, s_guard.reference_count) >=
        ENDSTOP_ENCODER_DELTA_COUNTS) {
        s_guard.reference_count = count;
        s_guard.last_motion_ms = now;
        /* Movement invalidates only the low-current stall hypothesis.  Do not
         * erase hard-current evidence: two fresh >=20 A samples must cut even
         * when vibration/noise keeps changing the encoder count. */
        s_guard.stall_current_samples = 0U;
    }

    const Ina226ChannelDiag *diag = Sensor_GetChannel5Diag();
    if (diag == NULL || !diag->i2c_ack || !diag->shunt_read_ok ||
        diag->sample_age_ms > ENDSTOP_SAMPLE_MAX_AGE_MS) {
        reset_pressure_samples();
        return false;
    }
    if (diag->sample_sequence == s_guard.last_sample_sequence) {
        return false;
    }

    s_guard.last_sample_sequence = diag->sample_sequence;
    const uint32_t current = current_magnitude_ma(diag->signed_current_ma);
    const bool no_motion =
        (uint32_t)(now - s_guard.last_motion_ms) >= ENDSTOP_NO_MOTION_MS;

    if (current >= ENDSTOP_HARD_CURRENT_MA) {
        s_guard.stall_current_samples = 0U;
        if (s_guard.hard_current_samples < ENDSTOP_CONFIRM_SAMPLES) {
            s_guard.hard_current_samples++;
        }
        if (s_guard.hard_current_samples >= ENDSTOP_CONFIRM_SAMPLES) {
            reset_pressure_samples();
            return true;
        }
        return false;
    }

    /* Any fresh sample below the hard threshold breaks hard-current
     * consecutiveness. */
    s_guard.hard_current_samples = 0U;

    if (no_motion && current >= ENDSTOP_STALL_CURRENT_MA) {
        if (s_guard.stall_current_samples < ENDSTOP_CONFIRM_SAMPLES) {
            s_guard.stall_current_samples++;
        }
        if (s_guard.stall_current_samples >= ENDSTOP_CONFIRM_SAMPLES) {
            reset_pressure_samples();
            return true;
        }
        return false;
    }

    s_guard.stall_current_samples = 0U;
    return false;
}

static void begin_right_reverse(void)
{
    /* Torque must disappear before the direction changes. */
    Homing_SetPwm(0U, false);
    reset_pressure_samples();
    s_guard.reverse_pending = true;
    s_guard.reverse_started_ms = HAL_GetTick();
}

static bool service_reverse_deadtime(void)
{
    if (!s_guard.reverse_pending) return false;

    Homing_SetPwm(0U, false);
    if ((uint32_t)(HAL_GetTick() - s_guard.reverse_started_ms) <
        REVERSE_DEADTIME_MS) {
        return true;
    }

    const int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    sweep_origin_count = count;
    stall_prev_count = count;
    stall_last_change_tick = HAL_GetTick();
    centering_state = CENTERING_SWEEP_RIGHT;
    s_guard.reverse_pending = false;
    reset_observation(CENTERING_SWEEP_RIGHT);
    Homing_SetPwm(CENTERING_PWM, false);
    return true;
}

static bool handle_endstop_pressure(void)
{
    if (!endstop_pressure_detected()) return false;

    Homing_SetPwm(0U, false);
    if (centering_state == CENTERING_SWEEP_LEFT) {
        begin_right_reverse();
    } else {
        Centering_Abort(EPS_FAULT_CENTER_NOT_FOUND);
    }
    return true;
}

/**
 * @brief  Bloque 2 (P1 audit fix): wall-clock ceiling per sweep leg.
 *
 *         endstop_pressure_detected() above is the fast, preferred guard but
 *         depends on CH5 being armed (see current_guard_armed()); the
 *         encoder stall/range fallbacks inside Homing_BaseStep() depend on
 *         the encoder actually being quiet.  A jittering encoder (counts
 *         changing <300ms apart while the rack is physically against the
 *         stop) defeats both at once, leaving only TOTAL_TIMEOUT_MS (10 s)
 *         at full authority.  This guard is independent of both signals: it
 *         only looks at HAL_GetTick() since the CURRENT leg started
 *         (leg_start_ms, set once per leg and never refreshed by encoder
 *         motion), so jitter cannot extend it.  On expiry it mirrors
 *         handle_endstop_pressure()'s own behaviour exactly: zero PWM, then
 *         reverse if this was the left leg or abort if it was the right. */
static bool handle_sweep_leg_timeout(void)
{
    const CenteringState_t state = centering_state;
    if (!is_sweep(state) || !s_guard.initialized || s_guard.state != state) {
        return false;
    }
    if ((uint32_t)(HAL_GetTick() - s_guard.leg_start_ms) < HOMING_SWEEP_TIMEOUT_MS) {
        return false;
    }

    Homing_SetPwm(0U, false);
    if (state == CENTERING_SWEEP_LEFT) {
        begin_right_reverse();
    } else {
        Centering_Abort(EPS_FAULT_CENTER_NOT_FOUND);
    }
    return true;
}

static void transfer_completed_homing_to_eps(void)
{
    if (centering_state != CENTERING_DONE || Encoder_HasFault() ||
        !Steering_IsCalibrated() || Steering_IsAssistLatchedOff()) {
        return;
    }

    Steering_EpsSetOwner(STEER_OWNER_NONE);
    Steering_EpsSetOwner(STEER_OWNER_EPS);
}

void SteeringCentering_Init(void)
{
    reset_guard();
    s_center_raw_cycles = 0U;
    Homing_BaseInit();
}

void SteeringCentering_MarkRestoredFromFlash(int32_t stored_center)
{
    /* An incremental encoder cannot observe movement while powered off.
     * Stored data remains diagnostic only; every boot must confirm PB5. */
    (void)stored_center;
}

void SteeringCentering_Step(void)
{
    if (centering_state != CENTERING_DONE &&
        centering_state != CENTERING_FAULT && !Encoder_HasFault() &&
        center_raw_stable()) {
        Centering_Complete();
        transfer_completed_homing_to_eps();
        reset_guard();
        s_center_raw_cycles = 0U;
        return;
    }

    if (service_reverse_deadtime()) return;
    if (handle_endstop_pressure()) return;
    if (handle_sweep_leg_timeout()) return;

    const CenteringState_t before = centering_state;
    Homing_BaseStep();

    /* The base encoder-stall fallback also detects the left stop.  It normally
     * writes right PWM immediately; override that transition with the same
     * zero-output dead-time used by the CH5 fast path. */
    if (before == CENTERING_SWEEP_LEFT &&
        centering_state == CENTERING_SWEEP_RIGHT) {
        begin_right_reverse();
        return;
    }

    if (before != CENTERING_DONE && centering_state == CENTERING_DONE) {
        transfer_completed_homing_to_eps();
    }

    if (centering_state == CENTERING_DONE ||
        centering_state == CENTERING_FAULT) {
        reset_guard();
        s_center_raw_cycles = 0U;
    }
}

void SteeringCentering_UpdateDiag(void)
{
    Homing_BaseUpdateDiag();
    s_diag.current_guard_armed = current_guard_armed();
    if (s_guard.reverse_pending) {
        s_diag.pwm_requested = 0U;
    } else if (is_sweep(centering_state)) {
        s_diag.pwm_requested =
            s_diag.current_guard_armed ? HOMING_PWM_COUNTS : HOMING_PWM_REDUCED_COUNTS;
    }
}
