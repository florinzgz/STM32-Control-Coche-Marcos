/**
  ****************************************************************************
  * @file    sensor_manager_patched.c
  * @brief   Demand-aware CH5 diagnosis + polarity-independent wheel edges.
  *
  * The productive sensor_manager.c remains the single implementation of ADC,
  * I2C, temperature and wheel-speed calculation.  This wrapper corrects three
  * hardware-integration details without duplicating that large module:
  *
  *  1. CH5 INA226 classification is qualified by real steering PWM demand.
  *  2. CH5 current sign is normalised against the commanded steering direction,
  *     so reverse/left homing current is not mistaken for reversed VIN+/VIN-.
  *  3. LJ12A3 wheel inputs are captured on BOTH electrical edges.  Two valid
  *     transitions (metal-entry + metal-exit, regardless of optocoupler
  *     inversion) form one physical target pulse, preserving 6 pulses/rev.
  *
  * Physical finding that motivated (3): a complete hand rotation showed
  * VALID=0 and REJECTED=0.  Rising-only EXTI cannot diagnose an inverted or
  * polarity-mismatched interface.  Both-edge acquisition makes the firmware
  * polarity independent and exposes raw-transition counters so “no signal at
  * the STM32 pin” remains distinguishable from “filtered EMI”.
  ****************************************************************************
  */

#include "sensor_manager.h"
#include "ina226_channel_diag.h"
#include "main.h"
#include <stddef.h>

extern TIM_HandleTypeDef htim3;

/* Convert measured CH5 current into the sign expected by the classifier:
 * positive means current follows the commanded motor direction; negative means
 * current flows against it and may indicate VIN+/VIN- reversal.  The physical
 * shunt sign naturally changes between TIM3 CH1 and CH2, so treating every
 * negative sample as a wiring fault incorrectly isolated EPS during the
 * deterministic left/reverse homing sweep. */
static int32_t PR429_NormalizeSteeringCurrent(int32_t measured_ma,
                                              bool reverse_command)
{
    if (!reverse_command) return measured_ma;

    /* Negating INT32_MIN in a signed 32-bit domain is undefined.  Saturate to
     * the largest positive value; the overcurrent supervisor will still treat
     * this impossible magnitude conservatively. */
    if (measured_ma == INT32_MIN) return INT32_MAX;
    return -measured_ma;
}

static Ina226DiagReason_t PR429_ClassifyInaWithDemand(
    const Ina226ChannelDiag *snapshot)
{
    if (snapshot == NULL) {
        return INA226_CH_UNKNOWN;
    }

    Ina226ChannelDiag qualified = *snapshot;
    const bool rpwm_active =
        (__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1) > 0U);
    const bool lpwm_active =
        (__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2) > 0U);

    /* Exactly one PWM channel must be active for a valid DRIVE command.  At
     * idle—or in the impossible/ambiguous both-active case—do not make a shunt
     * polarity conclusion from current sign. */
    const bool direction_known = (rpwm_active != lpwm_active);
    qualified.current_expected = qualified.channel_powered && direction_known;

    if (qualified.current_expected) {
        qualified.signed_current_ma =
            PR429_NormalizeSteeringCurrent(qualified.signed_current_ma,
                                            lpwm_active);
    }

    return Ina226_ClassifyChannel(&qualified);
}

/* Rename only the entry points replaced below.  All private state from
 * sensor_manager.c stays in this same translation unit, so the corrected ISR
 * updates the exact counters consumed by Wheel_UpdateSpeeds(). */
#define Ina226_ClassifyChannel PR429_ClassifyInaWithDemand
#define Sensor_Init            Sensor_Init_Legacy
#define Wheel_FL_IRQHandler    Wheel_FL_IRQHandler_Legacy
#define Wheel_FR_IRQHandler    Wheel_FR_IRQHandler_Legacy
#define Wheel_RL_IRQHandler    Wheel_RL_IRQHandler_Legacy
#define Wheel_RR_IRQHandler    Wheel_RR_IRQHandler_Legacy
#include "sensor_manager.c"
#undef Wheel_RR_IRQHandler
#undef Wheel_RL_IRQHandler
#undef Wheel_FR_IRQHandler
#undef Wheel_FL_IRQHandler
#undef Sensor_Init
#undef Ina226_ClassifyChannel

/* Raw electrical transition telemetry.  These counters are deliberately
 * separate from wheel_pulse[]: they prove whether EXTI sees any pin activity
 * before pairing and speed calculation. */
static volatile uint32_t pr_wheel_raw_transition_count[NUM_WHEELS] = {0};
static volatile uint32_t pr_wheel_last_raw_tick[NUM_WHEELS] = {0};
static volatile uint8_t  pr_wheel_half_cycle[NUM_WHEELS] = {0};
static volatile uint32_t pr_wheel_raw_window_start[NUM_WHEELS] = {0};
static volatile uint16_t pr_wheel_raw_window_count[NUM_WHEELS] = {0};

/* Two electrical transitions per metal target (entry + exit).  The existing
 * WHEEL_MAX_FREQ_HZ ceiling is expressed in completed target pulses, therefore
 * the raw-transition ceiling is exactly doubled. */
#define PR_WHEEL_MAX_RAW_EDGES_HZ ((uint16_t)(WHEEL_MAX_FREQ_HZ * 2U))

static inline void PR429_WheelTransitionIRQ(uint8_t idx)
{
    if (idx >= NUM_WHEELS) return;

    /* High-resolution EMI rejection.  Count every rejected transition so a
     * motor-noise problem remains visible instead of becoming a false speed. */
    const uint32_t cyc_now = DWT->CYCCNT;
    if ((cyc_now - wheel_last_edge_cyc[idx]) < sensor_debounce_cycles) {
        if (sensor_dbg_filtered_count[idx] < UINT32_MAX) {
            sensor_dbg_filtered_count[idx]++;
        }
        return;
    }
    wheel_last_edge_cyc[idx] = cyc_now;

    const uint32_t now = HAL_GetTick();
    if (pr_wheel_raw_transition_count[idx] < UINT32_MAX) {
        pr_wheel_raw_transition_count[idx]++;
    }
    pr_wheel_last_raw_tick[idx] = now;

    /* Bound CPU use under sustained EMI.  This is a raw-edge ceiling; normal
     * operation at 25 km/h is ~76 edges/s, far below the 400 edges/s limit. */
    if ((now - pr_wheel_raw_window_start[idx]) >= WHEEL_FLOOD_WINDOW_MS) {
        pr_wheel_raw_window_start[idx] = now;
        pr_wheel_raw_window_count[idx] = 0U;
    }
    if (pr_wheel_raw_window_count[idx] >= PR_WHEEL_MAX_RAW_EDGES_HZ) {
        if (sensor_dbg_filtered_count[idx] < UINT32_MAX) {
            sensor_dbg_filtered_count[idx]++;
        }
        return;
    }
    pr_wheel_raw_window_count[idx]++;

    /* Pair entry+exit.  This is independent of whether an EL817 or another
     * interface inverts the active-low LJ12A3 signal. */
    pr_wheel_half_cycle[idx] ^= 1U;
    if (pr_wheel_half_cycle[idx] != 0U) {
        return;
    }

    /* The millisecond debounce applies between completed physical targets,
     * not between the two edges of one target. */
    if ((now - wheel_last_pulse_tick[idx]) < WHEEL_MIN_PULSE_INTERVAL_MS) {
        return;
    }

    if ((now - wheel_flood_window_start[idx]) >= WHEEL_FLOOD_WINDOW_MS) {
        wheel_flood_window_start[idx] = now;
        wheel_flood_count[idx] = 0U;
    }
    if (wheel_flood_count[idx] >= WHEEL_MAX_FREQ_HZ) {
        return;
    }

    if (wheel_pulse[idx] < UINT32_MAX) {
        wheel_pulse[idx]++;
    }
    wheel_flood_count[idx]++;
    wheel_prev_pulse_tick[idx] = wheel_last_pulse_tick[idx];
    wheel_last_pulse_tick[idx] = now;
}

void Sensor_Init(void)
{
    Sensor_Init_Legacy();

    for (uint8_t i = 0; i < NUM_WHEELS; ++i) {
        pr_wheel_raw_transition_count[i] = 0U;
        pr_wheel_last_raw_tick[i] = 0U;
        pr_wheel_half_cycle[i] = 0U;
        pr_wheel_raw_window_start[i] = HAL_GetTick();
        pr_wheel_raw_window_count[i] = 0U;
    }

    /* LJ12A3 outputs are open-collector and may be inverted by the installed
     * optocoupler/interface.  Capture both edges on every channel; the paired
     * ISR above preserves six completed pulses per revolution. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = PIN_WHEEL_FL | PIN_WHEEL_FR;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = PIN_WHEEL_RL | PIN_WHEEL_RR;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Drop any transition latched while the GPIO mode was being changed. */
    __HAL_GPIO_EXTI_CLEAR_IT(PIN_WHEEL_FL | PIN_WHEEL_FR |
                             PIN_WHEEL_RL | PIN_WHEEL_RR);
}

void Wheel_FL_IRQHandler(void) { PR429_WheelTransitionIRQ(0U); }
void Wheel_FR_IRQHandler(void) { PR429_WheelTransitionIRQ(1U); }
void Wheel_RL_IRQHandler(void) { PR429_WheelTransitionIRQ(2U); }
void Wheel_RR_IRQHandler(void) { PR429_WheelTransitionIRQ(3U); }

uint32_t Wheel_GetRawTransitionCount(uint8_t idx)
{
    if (idx >= NUM_WHEELS) return 0U;
    return pr_wheel_raw_transition_count[idx];
}

uint32_t Wheel_GetLastRawTransitionAgeMs(uint8_t idx)
{
    if (idx >= NUM_WHEELS) return UINT32_MAX;
    if (pr_wheel_raw_transition_count[idx] == 0U) return UINT32_MAX;
    return HAL_GetTick() - pr_wheel_last_raw_tick[idx];
}
