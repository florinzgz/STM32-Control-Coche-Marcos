/**
  ****************************************************************************
  * @file    steering_centering_patched.c
  * @brief   Effective steering-homing policy for the RS390 + 1:50 gearbox.
  *
  * The production centering FSM (steering_centering.c) remains the source of
  * truth for centre detection, encoder/range checks, ownership and isolation.
  * This narrow wrapper adapts the physical homing attempt to the installed
  * actuator without weakening any real EPS safety protection:
  *
  *   - a non-zero homing command is applied at 60 % PWM (2550 / 4249) so the
  *     RS390 and 1:50 reduction can overcome static friction and backlash;
  *   - each sweep receives a 900 ms gearbox take-up grace before the original
  *     300 ms encoder-stall timer advances, giving an effective ~1.2 s start
  *     window per direction instead of interpreting a slow start as an
  *     immediate end stop;
  *   - PB5 is accepted both as the normal debounced falling-edge event and as
  *     a stable active-LOW level for three 100 Hz cycles.  This prevents a unit
  *     already over the centre metal—or an edge missed during boot—from first
  *     sweeping away and travelling almost the whole steering range;
  *   - PB5 not seeing metal starts/continues the search and does NOT itself
  *     disconnect PC12.  PC12 stays authorised while CENTERING owns the motor;
  *   - genuine encoder faults, range/total timeout, both-direction failure,
  *     CH5 faults and overcurrent still isolate the EPS exactly as before;
  *   - the diagnostic reports the effective requested PWM (2550), not the
  *     historical 425-count request inside the base FSM.
  *
  * A failed physical-centre search remains an ISOLABLE ASSIST fault:
  * PA6=PA7=0, PC4=LOW, PC12=OFF, owner=NONE, EPS mechanical-only, while the
  * vehicle remains ACTIVE with full traction.
  ****************************************************************************
  */

#include "steering_centering.h"
#include "safety_system.h"
#include "motor_control.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* TIM3 is configured with ARR=4249.  60 % rounds to 2550 timer counts. */
#define PR434_HOMING_PWM_PERCENT          60U
#define PR434_HOMING_PWM_COUNTS           2550U
#define PR434_GEARBOX_TAKEUP_GRACE_MS     900U
#define PR434_CENTER_RAW_CONFIRM_CYCLES   3U

_Static_assert(PR434_HOMING_PWM_PERCENT == 60U,
               "RS390 homing profile must remain at the requested 60 percent");
_Static_assert(PR434_HOMING_PWM_COUNTS < 4250U,
               "Steering homing PWM must fit the TIM3 period");
_Static_assert(PR434_CENTER_RAW_CONFIRM_CYCLES >= 2U,
               "PB5 level fallback must reject a one-cycle EMI transient");

/* Prototypes consumed by the included production FSM after macro routing. */
static uint32_t PR434_HomingTick(void);
static void PR434_HomingMotorSetPwm(uint16_t requested_pwm, bool reverse);
void PR434_SteeringCentering_Init_Base(void);
void PR434_SteeringCentering_Step_Base(void);
void PR434_SteeringCentering_UpdateDiag_Base(void);

static bool PR434_SteeringHomingPowerReady(void)
{
    const CenteringState_t state = SteeringCentering_GetState();
    const bool sweeping = (state == CENTERING_SWEEP_LEFT) ||
                          (state == CENTERING_SWEEP_RIGHT);

    /* WAIT_RAIL intentionally remains not-ready.  Once the FSM enters an
     * active sweep, PC12 HIGH is the real local power evidence. */
    return sweeping &&
        (HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR) == GPIO_PIN_SET);
}

/* Route only this translation unit through the installed-actuator policy. */
#define Safety_IsPowerReady            PR434_SteeringHomingPowerReady
#define Motor_SetPWM_Steering          PR434_HomingMotorSetPwm
#define HAL_GetTick                    PR434_HomingTick
#define SteeringCentering_Init         PR434_SteeringCentering_Init_Base
#define SteeringCentering_Step         PR434_SteeringCentering_Step_Base
#define SteeringCentering_UpdateDiag   PR434_SteeringCentering_UpdateDiag_Base
#include "steering_centering.c"
#undef SteeringCentering_UpdateDiag
#undef SteeringCentering_Step
#undef SteeringCentering_Init
#undef HAL_GetTick
#undef Motor_SetPWM_Steering
#undef Safety_IsPowerReady

/* -------------------------------------------------------------------------
 * 60 % physical homing command
 * ------------------------------------------------------------------------- */
static void PR434_HomingMotorSetPwm(uint16_t requested_pwm, bool reverse)
{
    /* Preserve an explicit zero command.  Every non-zero centering request is
     * the controlled search command and is resolved to exactly 60 %. */
    const uint16_t applied_pwm =
        (requested_pwm == 0U) ? 0U : (uint16_t)PR434_HOMING_PWM_COUNTS;
    Motor_SetPWM_Steering(applied_pwm, reverse);
}

/* -------------------------------------------------------------------------
 * Gearbox take-up clock
 * -------------------------------------------------------------------------
 * The base FSM has a proven 300 ms no-encoder-change stall detector.  Rather
 * than removing or globally enlarging it, freeze only the centering FSM's
 * private time for the first 900 real milliseconds of each LEFT/RIGHT sweep.
 * The result is an effective 1.2 s start window per direction.  Rail settle
 * remains 50 real ms; the absolute search deadline is extended by at most
 * 1.8 s total.  All other firmware continues using the real HAL tick.
 */
typedef struct {
    bool             initialized;
    CenteringState_t observed_state;
    uint32_t         sweep_entry_real_ms;
    uint32_t         completed_grace_ms;
} PR434_HomingClock_t;

static PR434_HomingClock_t s_pr434_homing_clock;
static uint8_t s_pr434_center_raw_cycles;

static bool PR434_IsSweep(CenteringState_t state)
{
    return state == CENTERING_SWEEP_LEFT ||
           state == CENTERING_SWEEP_RIGHT;
}

static uint32_t PR434_MinU32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

static void PR434_ResetHomingClock(void)
{
    s_pr434_homing_clock.initialized = false;
    s_pr434_homing_clock.observed_state = CENTERING_IDLE;
    s_pr434_homing_clock.sweep_entry_real_ms = 0U;
    s_pr434_homing_clock.completed_grace_ms = 0U;
}

static uint32_t PR434_HomingTick(void)
{
    const uint32_t real_now = HAL_GetTick();
    const CenteringState_t state = centering_state;

    if (!s_pr434_homing_clock.initialized) {
        s_pr434_homing_clock.initialized = true;
        s_pr434_homing_clock.observed_state = state;
        s_pr434_homing_clock.sweep_entry_real_ms = real_now;
    } else if (state != s_pr434_homing_clock.observed_state) {
        /* Finalise the grace consumed by the sweep we just left. */
        if (PR434_IsSweep(s_pr434_homing_clock.observed_state)) {
            const uint32_t elapsed =
                real_now - s_pr434_homing_clock.sweep_entry_real_ms;
            s_pr434_homing_clock.completed_grace_ms +=
                PR434_MinU32(elapsed, PR434_GEARBOX_TAKEUP_GRACE_MS);
        }

        s_pr434_homing_clock.observed_state = state;
        s_pr434_homing_clock.sweep_entry_real_ms = real_now;
    }

    uint32_t active_grace_ms = 0U;
    if (PR434_IsSweep(state)) {
        const uint32_t elapsed =
            real_now - s_pr434_homing_clock.sweep_entry_real_ms;
        active_grace_ms =
            PR434_MinU32(elapsed, PR434_GEARBOX_TAKEUP_GRACE_MS);
    }

    return real_now - s_pr434_homing_clock.completed_grace_ms -
           active_grace_ms;
}

/* Stable level fallback for an active-LOW LJ12A3 center sensor.  The normal
 * EXTI/debounced event remains authoritative and is still checked inside the
 * base FSM.  This second path only closes the boot race where PB5 was already
 * LOW before the falling-edge interrupt was armed or its flag was cleared. */
static bool PR434_CenterRawStable(void)
{
    const bool active = ServiceMode_IsEnabled(MODULE_STEER_CENTER) &&
        (HAL_GPIO_ReadPin(GPIOB, PIN_STEER_CENTER) == GPIO_PIN_RESET);

    if (!active) {
        s_pr434_center_raw_cycles = 0U;
        return false;
    }

    if (s_pr434_center_raw_cycles < PR434_CENTER_RAW_CONFIRM_CYCLES) {
        s_pr434_center_raw_cycles++;
    }
    return s_pr434_center_raw_cycles >= PR434_CENTER_RAW_CONFIRM_CYCLES;
}

/* Public init wrapper: reset the private timing and PB5 level adaptation on
 * every new centering session, then run the unchanged production init. */
void SteeringCentering_Init(void)
{
    PR434_ResetHomingClock();
    s_pr434_center_raw_cycles = 0U;
    PR434_SteeringCentering_Init_Base();
}

/* Public step wrapper: complete immediately after a stable PB5 active level,
 * even when no new falling edge can occur because the sensor was already over
 * metal at startup.  Otherwise delegate the full two-direction base FSM. */
void SteeringCentering_Step(void)
{
    if (centering_state != CENTERING_DONE &&
        centering_state != CENTERING_FAULT &&
        PR434_CenterRawStable()) {
        Centering_Complete();
        s_pr434_center_raw_cycles = 0U;
        return;
    }

    PR434_SteeringCentering_Step_Base();

    if (centering_state == CENTERING_DONE ||
        centering_state == CENTERING_FAULT) {
        s_pr434_center_raw_cycles = 0U;
    }
}

/* Public diagnostic wrapper: publish the command that reaches TIM3. */
void SteeringCentering_UpdateDiag(void)
{
    PR434_SteeringCentering_UpdateDiag_Base();

    if (PR434_IsSweep(centering_state)) {
        s_diag.pwm_requested = (uint16_t)PR434_HOMING_PWM_COUNTS;
    }
}
