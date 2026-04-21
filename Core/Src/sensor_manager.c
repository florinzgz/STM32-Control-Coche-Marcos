/**
  ****************************************************************************
  * @file    sensor_manager.c
  * @brief   Sensor acquisition: wheel speed, DS18B20, INA226, pedal (ADC)
  *
  * Hardware managed by this module:
  *   - 4× LJ12A3 inductive wheel speed sensors (EXTI interrupts)
  *   - 5× DS18B20 temperature sensors (OneWire on PB0)
  *   - 6× INA226 current/voltage sensors (I2C via TCA9548A multiplexer)
  *   - 1× Hall-effect pedal: internal ADC1 on PA3 (via voltage divider)
  *         Plausibility: dual-sample consistency, EMA filter,
  *                       range validation, rate-of-change detection
  *
  * I2C Bus Recovery Mechanism
  * Protects against SDA held low by slave device
  * Based on NXP AN10216
  ****************************************************************************
  */

#include "sensor_manager.h"
#include "safety_system.h"
#include "main.h"

/* =========================================================================
 *  Wheel Speed Sensors – EXTI pulse counting
 *
 *  ARCHITECTURE NOTE — Timer-based alternative:
 *  The current EXTI+counter approach is simple and adequate for the
 *  low-frequency LJ12A3 sensors (6 pulses/rev, max ~38 Hz at 25 km/h).
 *  For higher-resolution sensors or faster vehicles, consider using
 *  TIMx in Input Capture mode (one TIM channel per wheel).  Benefits:
 *    - Hardware edge timestamping (no ISR jitter)
 *    - Built-in digital filtering (TIMx_CCMR1.ICxF)
 *    - No ISR at all for pulse counting (DMA or period measurement)
 *    - Precise instantaneous frequency measurement
 *  The STM32G474 has enough timers, but the 4 wheel pins (PA0, PA1,
 *  PA2, PB15) are not all on the same timer, so 3-4 timers would be
 *  required.  The current EXTI approach is retained because:
 *    (a) interrupt rate is very low (~38 Hz worst-case per wheel)
 *    (b) existing wiring matches EXTI lines, not timer channels
 *    (c) software debounce + stale detection provides adequate safety
 * ========================================================================= */

/* Pulse counters incremented in EXTI ISR */
static volatile uint32_t wheel_pulse[NUM_WHEELS] = {0};
static uint32_t wheel_pulse_prev[NUM_WHEELS]     = {0};
static uint32_t wheel_last_tick[NUM_WHEELS]       = {0};
static float    wheel_speed_kmh[NUM_WHEELS]       = {0};
static float    wheel_rpm[NUM_WHEELS]             = {0};

/* Debounce and flood detection state (ISR context) */
static volatile uint32_t wheel_last_pulse_tick[NUM_WHEELS] = {0};
static volatile uint32_t wheel_prev_pulse_tick[NUM_WHEELS] = {0};
static volatile uint32_t wheel_flood_count[NUM_WHEELS]     = {0};
static volatile uint32_t wheel_flood_window_start[NUM_WHEELS] = {0};

/* Precomputed flood ceiling: max accepted pulses per 1-second window */
#define WHEEL_FLOOD_WINDOW_MS    1000U

/* ISR-safe debounce + flood filter.
 * Called from EXTI vectors — must be minimal and non-blocking.
 *
 * 1. Debounce: reject pulses arriving faster than
 *    WHEEL_MIN_PULSE_INTERVAL_MS (contact bounce rejection).
 * 2. Flood detection: count pulses within a 1-second sliding window.
 *    If the count exceeds WHEEL_MAX_FREQ_HZ, further pulses are
 *    silently dropped until the window rolls over.  This caps CPU
 *    load even under sensor noise / wiring fault conditions.
 *
 * Performance: worst-case ISR body is ~20 instructions (read tick,
 * two comparisons, one increment, one store).  At max accepted rate
 * of 200 Hz, CPU overhead is < 0.01% at 170 MHz.                     */
static inline void Wheel_IRQDebounced(uint8_t idx)
{
    uint32_t now = HAL_GetTick();

    /* 1. Time-based debounce — reject contact bounce */
    if ((now - wheel_last_pulse_tick[idx]) < WHEEL_MIN_PULSE_INTERVAL_MS)
        return;

    /* 2. Flood detection — cap pulse rate per 1-second window */
    if ((now - wheel_flood_window_start[idx]) >= WHEEL_FLOOD_WINDOW_MS) {
        /* New window: reset counter */
        wheel_flood_window_start[idx] = now;
        wheel_flood_count[idx] = 0;
    }
    if (wheel_flood_count[idx] >= WHEEL_MAX_FREQ_HZ)
        return;  /* Silently drop — sensor saturated */

    /* Accept pulse */
    wheel_pulse[idx]++;
    wheel_flood_count[idx]++;
    wheel_prev_pulse_tick[idx] = wheel_last_pulse_tick[idx];
    wheel_last_pulse_tick[idx] = now;
}

void Wheel_FL_IRQHandler(void) { Wheel_IRQDebounced(0); }
void Wheel_FR_IRQHandler(void) { Wheel_IRQDebounced(1); }
void Wheel_RL_IRQHandler(void) { Wheel_IRQDebounced(2); }
void Wheel_RR_IRQHandler(void) { Wheel_IRQDebounced(3); }

/* =========================================================================
 *  Steering Center Inductive Sensor – EXTI pulse detection
 *
 *  An LJ12A3-type inductive proximity sensor detects a physical screw
 *  at the mechanical center of the steering rack.  A single rising edge
 *  on PIN_STEER_CENTER (PB5 / EXTI5) indicates the rack is at center.
 * ========================================================================= */

static volatile uint8_t steer_center_flag = 0;

void SteeringCenter_IRQHandler(void) { steer_center_flag = 1; }

bool SteeringCenter_Detected(void) { return (steer_center_flag != 0); }

void SteeringCenter_ClearFlag(void) { steer_center_flag = 0; }

/**
 * @brief  Compute speed for one wheel from accumulated pulses.
 * @param  idx   Wheel index 0-3 (FL,FR,RL,RR).
 *
 * Called from Wheel_UpdateSpeeds() — NOT from individual getters.
 *
 * Concurrency: volatile ISR counters (wheel_pulse, wheel_last_pulse_tick,
 * wheel_prev_pulse_tick) are snapshot-copied under a brief __disable_irq /
 * __enable_irq critical section (~5 instructions) to prevent the ISR from
 * updating the tick between the pulse read and the tick read.
 *
 * Counter overflow: uint32_t subtraction is modular (C99 §6.2.5 ¶9).
 * `pulses − prev` gives the correct delta even when the counter wraps
 * (≈49.7 days at 200 Hz max).  No explicit overflow guard needed.
 *
 * Hybrid speed computation:
 *   delta ≥ 2 → standard count-based (good accuracy at medium-high speed)
 *   delta == 1 → period-based using inter-pulse interval (avoids 10 ms
 *                quantisation at low speed)
 *   delta == 0 → upper-bound estimate from time-since-last-pulse with
 *                monotonic decay (smooth approach to 0 instead of abrupt
 *                0/nonzero oscillation)
 *
 * Stale detection: if no new pulse within WHEEL_STALE_TIMEOUT_MS, speed
 * is forced to zero (sensor disconnect / vehicle stopped).
 *
 * Plausibility ceiling: output is clamped to WHEEL_SPEED_CLAMP_KMH.
 * Prevents NaN, negative, or wildly impossible values from reaching
 * traction control, ABS, or CAN telemetry.
 */
static void Wheel_ComputeSpeed(uint8_t idx)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - wheel_last_tick[idx];
    if (dt == 0) return;

    /* ---- Atomic snapshot ----
     * Briefly disable interrupts to copy all ISR-shared state in one
     * indivisible read.  Critical section is ~5 instructions at
     * 170 MHz ≈ 30 ns — negligible jitter on any peripheral.           */
    __disable_irq();
    uint32_t pulses    = wheel_pulse[idx];
    uint32_t last_pt   = wheel_last_pulse_tick[idx];
    uint32_t prev_pt   = wheel_prev_pulse_tick[idx];
    __enable_irq();

    /* Overflow-safe delta (unsigned modular subtraction) */
    uint32_t delta = pulses - wheel_pulse_prev[idx];

    /* Stale detection: no new pulse within timeout → force zero.
     * This catches sensor disconnect and vehicle-stopped conditions.   */
    if ((now - last_pt) >= WHEEL_STALE_TIMEOUT_MS) {
        wheel_speed_kmh[idx] = 0.0f;
        wheel_rpm[idx]       = 0.0f;
        wheel_pulse_prev[idx] = pulses;
        wheel_last_tick[idx]  = now;
        return;
    }

    /* ---- Hybrid speed computation ----                                */
    float speed_kmh;
    float rpm;
    const float dist_per_pulse = WHEEL_CIRCUMF_M / (float)WHEEL_PULSES_REV;

    if (delta >= 2U) {
        /* Count-based: accurate at medium-high speed */
        float revolutions = (float)delta / (float)WHEEL_PULSES_REV;
        float dist_m      = revolutions * WHEEL_CIRCUMF_M;
        float speed_ms    = dist_m * 1000.0f / (float)dt;
        speed_kmh = speed_ms * 3.6f;
        rpm       = revolutions * 60000.0f / (float)dt;

    } else if (delta == 1U) {
        /* Period-based: use interval between last two accepted pulses.
         * More precise than "1 pulse / 10 ms" at low speeds.           */
        uint32_t period_ms = last_pt - prev_pt;
        if (period_ms > 0U && period_ms < WHEEL_STALE_TIMEOUT_MS) {
            float speed_ms = dist_per_pulse * 1000.0f / (float)period_ms;
            speed_kmh = speed_ms * 3.6f;
            rpm       = 60000.0f / ((float)period_ms * (float)WHEEL_PULSES_REV);
        } else {
            /* First pulse ever or period unreliable — fall back to count */
            float speed_ms = dist_per_pulse * 1000.0f / (float)dt;
            speed_kmh = speed_ms * 3.6f;
            rpm       = 60000.0f / ((float)dt * (float)WHEEL_PULSES_REV);
        }

    } else {
        /* delta == 0: no new pulse in this window.
         * Upper-bound estimate: speed ≤ 1 pulse distance / time since
         * the last pulse.  Ensures smooth monotonic decay toward 0
         * instead of abrupt 0-nonzero oscillation at low speeds.       */
        uint32_t since_last = now - last_pt;
        if (since_last > 0U) {
            float speed_ms = dist_per_pulse * 1000.0f / (float)since_last;
            speed_kmh = speed_ms * 3.6f;
            rpm       = 60000.0f / ((float)since_last * (float)WHEEL_PULSES_REV);
            /* Monotonic decay: never exceed previous reading */
            if (speed_kmh > wheel_speed_kmh[idx])
                speed_kmh = wheel_speed_kmh[idx];
            if (rpm > wheel_rpm[idx])
                rpm = wheel_rpm[idx];
        } else {
            speed_kmh = wheel_speed_kmh[idx];
            rpm       = wheel_rpm[idx];
        }
    }

    /* ---- Plausibility ceiling ----
     * Clamp to WHEEL_SPEED_CLAMP_KMH.  Also reject NaN (x != x).     */
    if (speed_kmh < 0.0f || speed_kmh != speed_kmh)
        speed_kmh = 0.0f;
    if (speed_kmh > WHEEL_SPEED_CLAMP_KMH)
        speed_kmh = WHEEL_SPEED_CLAMP_KMH;

    if (rpm < 0.0f || rpm != rpm)
        rpm = 0.0f;
    {
        float rpm_ceil = WHEEL_SPEED_CLAMP_KMH / 3.6f / WHEEL_CIRCUMF_M * 60.0f;
        if (rpm > rpm_ceil) rpm = rpm_ceil;
    }

    wheel_speed_kmh[idx] = speed_kmh;
    wheel_rpm[idx]       = rpm;
    wheel_pulse_prev[idx] = pulses;
    wheel_last_tick[idx]  = now;
}

/**
 * @brief  Batch-update all 4 wheel speeds.
 *
 * Must be called exactly ONCE per control cycle (10 ms tier recommended).
 * All subsequent Wheel_GetSpeed_*() calls return the cached values,
 * eliminating redundant computation and ensuring all consumers in the
 * same cycle see identical speed values (consistency guarantee).
 */
void Wheel_UpdateSpeeds(void)
{
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        Wheel_ComputeSpeed(i);
    }
}

/* Getters: return cached speed from last Wheel_UpdateSpeeds() call.
 * No computation, no side-effects on shared state.                    */
float Wheel_GetSpeed_FL(void) { return wheel_speed_kmh[0]; }
float Wheel_GetSpeed_FR(void) { return wheel_speed_kmh[1]; }
float Wheel_GetSpeed_RL(void) { return wheel_speed_kmh[2]; }
float Wheel_GetSpeed_RR(void) { return wheel_speed_kmh[3]; }
float Wheel_GetRPM_FL(void)   { return wheel_rpm[0]; }

/**
 * @brief  Check if a wheel sensor is stale (no pulse within timeout).
 * @param  idx  Wheel index 0-3.
 * @return true if stale or idx out of range.
 */
bool Wheel_IsStale(uint8_t idx)
{
    if (idx >= NUM_WHEELS) return true;
    return ((HAL_GetTick() - wheel_last_pulse_tick[idx]) >= WHEEL_STALE_TIMEOUT_MS);
}

/* =========================================================================
 *  Pedal – Internal ADC dual-sample + software plausibility
 *
 *  The 5V pedal signal (Hall-effect SS1324LUA-T, 0.3V–4.8V) is scaled to
 *  0–3.3V via a voltage divider (10 kΩ + 6.8 kΩ → ratio 0.4048).
 *  Plausibility is ensured entirely in software:
 *
 *  1. Dual-sample consistency: two consecutive ADC reads must agree
 *     within ±PEDAL_SAMPLE_TOLERANCE counts.  Disagreement indicates
 *     electrical noise or ADC transient fault.
 *
 *  2. Range validation: ADC value must stay within
 *     [PEDAL_ADC_FAULT_LO .. PEDAL_ADC_FAULT_HI].
 *     Values outside this band indicate an open/short circuit.
 *
 *  3. Rate-of-change limiting: pedal percentage cannot jump more than
 *     PEDAL_MAX_RATE_PCT per update cycle (50 ms).  Exceeding this
 *     indicates a sensor or wiring fault (not physically possible).
 *
 *  4. EMA filter: Exponential Moving Average with α = 0.3 to smooth
 *     noise without adding significant lag (~2 cycles settling).
 *
 *  Advantages over previous ADS1115 I2C implementation:
 *   - Conversion: ~1 µs per sample vs 8 ms + I2C overhead
 *   - No HAL_Delay() blocking call in the control loop
 *   - No I2C bus contention with INA226/TCA9548A
 *   - Deterministic timing (no bus arbitration or recovery)
 *   - Simpler hardware (no external ADC component)
 * ========================================================================= */

static uint16_t pedal_raw_adc   = 0;     /* Primary ADC raw (12-bit)          */
static uint16_t pedal_raw_adc2  = 0;     /* Second sample for consistency     */
static float    pedal_pct       = 0.0f;  /* Control output: 0–100% (EMA)     */
static float    pedal_pct_raw   = 0.0f;  /* Instantaneous 0–100% (pre-EMA)   */
static float    pedal_pct_prev  = 0.0f;  /* Previous cycle % for rate check   */
static float    pedal_ema       = 0.0f;  /* EMA filtered percentage           */
static bool     pedal_plausible = true;  /* Software plausibility result      */
static bool     pedal_channels_contradict = false; /* Dual samples disagree   */
static bool     pedal_ema_primed = false; /* EMA initialized after first read */

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;

/* ADC calibration (voltage divider 10kΩ + 6.8kΩ, 12-bit, 3.3V ref)
 * Divider ratio: 6.8/(10+6.8) = 0.4048
 * Pedal 0.3V released → 0.3 × 0.4048 = 0.121V → 0.121/3.3 × 4095 ≈ 150
 * Pedal 4.8V pressed  → 4.8 × 0.4048 = 1.943V → 1.943/3.3 × 4095 ≈ 2413 */
#define PEDAL_ADC_MIN   150U     /* ~0.3V (pedal released), after divider */
#define PEDAL_ADC_MAX   2413U    /* ~4.8V (pedal fully pressed), after divider */

/* Fault detection thresholds — values outside this band indicate
 * open/short circuit (wider margin than calibrated range).
 * Below FAULT_LO → wire open or sensor unpowered.
 * Above FAULT_HI → wire shorted to supply.                          */
#define PEDAL_ADC_FAULT_LO   30U     /* ~0.024V — well below 0.3V rest */
#define PEDAL_ADC_FAULT_HI   2800U   /* ~2.25V — above 4.8V full scale */

/* Dual-sample consistency tolerance (ADC counts).
 * Two consecutive reads of the same DC signal should agree within
 * a few LSBs.  12-bit ADC noise is typically ±2 LSB; allow ±30
 * counts (~24 mV) to handle minor noise without false alarms.        */
#define PEDAL_SAMPLE_TOLERANCE  30U

/* Rate-of-change limit: maximum % change per 50 ms update cycle.
 * Physical pedal travel time (0–100%) is ~200 ms minimum → max
 * rate ≈ 25 %/50ms.  Set threshold at 35 %/50ms for margin.         */
#define PEDAL_MAX_RATE_PCT  35.0f

/* EMA filter coefficient (0 < α ≤ 1).
 * α = 0.3 gives ~3 cycle settling time (150 ms at 20 Hz).
 * Good balance between noise rejection and pedal responsiveness.     */
#define PEDAL_EMA_ALPHA  0.3f

/* Cross-validation tolerance: 5% of full pedal range
 * (kept for API compatibility with safety_system.c checks)           */
#define PEDAL_PLAUSIBILITY_PCT  5.0f

/**
 * @brief  Read pedal via internal ADC, two consecutive samples (~2 µs total).
 */
static void Pedal_ReadDualSample(void)
{
    /* First sample */
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK) {
        pedal_raw_adc = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    /* Second sample (immediate re-read for consistency check) */
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK) {
        pedal_raw_adc2 = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
}

/**
 * @brief  Map raw ADC count to 0–100% using calibrated range.
 */
static float Pedal_RawToPercent(uint16_t raw)
{
    if (raw <= PEDAL_ADC_MIN) return 0.0f;
    if (raw >= PEDAL_ADC_MAX) return 100.0f;
    return (float)(raw - PEDAL_ADC_MIN) * 100.0f
         / (float)(PEDAL_ADC_MAX - PEDAL_ADC_MIN);
}

void Pedal_Update(void)
{
    /* 1. Read ADC twice (~2 µs total — no blocking delay) */
    Pedal_ReadDualSample();

    /* 2. Dual-sample consistency check */
    uint16_t diff_raw;
    if (pedal_raw_adc >= pedal_raw_adc2)
        diff_raw = pedal_raw_adc - pedal_raw_adc2;
    else
        diff_raw = pedal_raw_adc2 - pedal_raw_adc;

    if (diff_raw > PEDAL_SAMPLE_TOLERANCE) {
        /* Consecutive samples disagree — noise or transient fault */
        pedal_channels_contradict = true;
        pedal_plausible = false;
        return;  /* Skip update — keep previous valid values */
    }
    pedal_channels_contradict = false;

    /* 3. Use average of both samples for best accuracy */
    uint16_t avg_raw = (uint16_t)(((uint32_t)pedal_raw_adc + pedal_raw_adc2) / 2U);

    /* 4. Range validation — detect open/short circuit */
    if (avg_raw < PEDAL_ADC_FAULT_LO || avg_raw > PEDAL_ADC_FAULT_HI) {
        pedal_plausible = false;
        return;
    }

    /* 5. Convert to percentage */
    pedal_pct_raw = Pedal_RawToPercent(avg_raw);

    /* 6. EMA filter */
    if (!pedal_ema_primed) {
        pedal_ema = pedal_pct_raw;
        pedal_ema_primed = true;
    } else {
        pedal_ema = PEDAL_EMA_ALPHA * pedal_pct_raw
                  + (1.0f - PEDAL_EMA_ALPHA) * pedal_ema;
    }

    /* 7. Rate-of-change check (after EMA to catch raw jumps too) */
    float rate = pedal_pct_raw - pedal_pct_prev;
    if (rate < 0.0f) rate = -rate;

    if (rate > PEDAL_MAX_RATE_PCT) {
        pedal_plausible = false;
        /* Don't update output or history — keep last safe baseline */
        return;
    }

    /* 8. All checks passed — update output and history */
    pedal_plausible = true;
    pedal_pct = pedal_ema;
    pedal_pct_prev = pedal_pct_raw;
}

float Pedal_GetValue(void)          { return (float)pedal_raw_adc; }
float Pedal_GetPercent(void)        { return pedal_pct; }
bool  Pedal_IsPlausible(void)       { return pedal_plausible; }
bool  Pedal_IsContradictory(void)   { return pedal_channels_contradict; }
float Pedal_GetRawPercent(void)    { return pedal_pct_raw; }

/* =========================================================================
 *  INA226 Current Sensors via TCA9548A I2C multiplexer
 * ========================================================================= */

static float current_amps[NUM_INA226]  = {0};
static float voltage_bus[NUM_INA226]   = {0};

extern I2C_HandleTypeDef hi2c1;

/* ---- I2C failure tracking ---- */
#define I2C_FAIL_THRESHOLD       3   /* consecutive failures before recovery */
#define I2C_RECOVERY_MAX_ATTEMPTS 2  /* max recovery tries before safe-state */

static uint8_t i2c_fail_count       = 0;
static uint8_t i2c_recovery_attempts = 0;

/**
 * @brief  I2C bus recovery via manual SCL clock cycling.
 *
 * Procedure (per NXP AN10216):
 *   1. Deinitialize I2C peripheral
 *   2. Reconfigure SCL (PB6) as GPIO push-pull output
 *   3. Toggle SCL 16 times while monitoring SDA
 *   4. Generate STOP condition (SDA low→high while SCL high)
 *   5. Reinitialize I2C peripheral
 *   6. Reset failure counter
 *
 * Execution time: ~16 × 10 µs = ~160 µs (well under 5 ms limit).
 */
static void I2C_BusRecovery(void)
{
    /* Step 1: Deinitialize I2C peripheral */
    HAL_I2C_DeInit(&hi2c1);

    /* Step 2: Configure SCL (PB6) as GPIO output, SDA (PB7) as input */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = PIN_I2C_SCL;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin   = PIN_I2C_SDA;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Step 3: Toggle SCL 16 times to release stuck slave */
    for (uint8_t i = 0; i < 16; i++) {
        HAL_GPIO_WritePin(GPIOB, PIN_I2C_SCL, GPIO_PIN_RESET);
        /* ~5 µs low — short busy-wait (170 MHz, ~850 cycles) */
        for (volatile uint32_t d = 0; d < 210; d++) { __NOP(); }

        HAL_GPIO_WritePin(GPIOB, PIN_I2C_SCL, GPIO_PIN_SET);
        /* ~5 µs high */
        for (volatile uint32_t d = 0; d < 210; d++) { __NOP(); }

        /* If SDA released, slave is unstuck */
        if (HAL_GPIO_ReadPin(GPIOB, PIN_I2C_SDA) == GPIO_PIN_SET) {
            break;
        }
    }

    /* Step 4: Generate STOP condition (SDA low→high while SCL high) */
    gpio.Pin  = PIN_I2C_SDA;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, PIN_I2C_SDA, GPIO_PIN_RESET);
    for (volatile uint32_t d = 0; d < 210; d++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, PIN_I2C_SCL, GPIO_PIN_SET);
    for (volatile uint32_t d = 0; d < 210; d++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, PIN_I2C_SDA, GPIO_PIN_SET);
    for (volatile uint32_t d = 0; d < 210; d++) { __NOP(); }

    /* Step 5: Reinitialize I2C peripheral */
    HAL_I2C_Init(&hi2c1);

    /* Step 6: Reset failure counter */
    i2c_fail_count = 0;
}

#define INA226_REG_CONFIG          0x00
#define INA226_REG_SHUNT_VOLTAGE   0x01
#define INA226_REG_BUS_VOLTAGE     0x02
#define INA226_SHUNT_LSB_UV        2.5f   /* 2.5 µV per LSB */
#define INA226_BUS_LSB_MV          1.25f  /* 1.25 mV per LSB */

/* INA226 configuration register value (explicit — do not rely on defaults).
 * Bits [14:12] = 100  (reserved, always 4)
 * Bits [11:9]  = 001  (AVG = 4 samples — noise rejection for motor loads)
 * Bits [8:6]   = 100  (VBUSCT = 1.1 ms)
 * Bits [5:3]   = 100  (VSHCT  = 1.1 ms)
 * Bits [2:0]   = 111  (continuous shunt + bus voltage)
 * Result: 0x4327 → averaging smooths ADC noise from PWM switching.
 * Total conversion time per measurement: 4 × (1.1 + 1.1) = 8.8 ms,
 * well within the 50 ms read cycle.
 *
 * ---- MEASUREMENT LATENCY WARNING ----
 * Per-sensor conversion latency: 8.8 ms (4-sample avg × 2.2 ms).
 * I2C read overhead per sensor: ~0.3 ms (mux select + 2 register reads).
 * Full 6-sensor cycle: ~6 × (8.8 + 0.3) ≈ 54.6 ms effective latency.
 * The 50 ms read cycle guarantees fresh data each call.
 *
 * IMPORTANT: INA226 measurements are NOT suitable for fast overcurrent
 * protection (<10 ms events).  Hardware protection (fuses, BTS7960
 * internal current limiting) must handle sub-millisecond transients.
 * The INA226 provides monitoring and slow-fault detection only.           */
#define INA226_CONFIG_VALUE        0x4327

/* ---- EMA filter for current readings ----
 * α = 0.2 balances noise rejection with response time.
 * Settling time: ~5 cycles (250 ms at 50 ms update rate).
 * Applied to current_amps[] after each read; raw value available via
 * Current_GetAmpsRaw() for diagnostics.                                   */
#define CURRENT_EMA_ALPHA          0.2f

static float current_amps_raw[NUM_INA226] = {0};
static bool  current_ema_primed           = false;

/**
 * @brief  Select a channel on the TCA9548A multiplexer.
 */
static HAL_StatusTypeDef TCA9548A_SelectChannel(uint8_t channel)
{
    if (channel > 7) return HAL_ERROR;
    uint8_t data = (uint8_t)(1U << channel);
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, (I2C_ADDR_TCA9548A << 1), &data, 1, 50);
    if (status != HAL_OK) {
        i2c_fail_count++;
    }
    return status;
}

/**
 * @brief  Read a 16-bit register from INA226 on the currently selected channel.
 * @retval Register value, or 0 on I2C failure (failure counted).
 */
static int16_t INA226_ReadReg(uint8_t reg)
{
    uint8_t buf[2] = {0};
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, (I2C_ADDR_INA226 << 1), reg,
                                                 I2C_MEMADD_SIZE_8BIT, buf, 2, 50);
    if (status != HAL_OK) {
        i2c_fail_count++;
        return 0;
    }
    return (int16_t)((buf[0] << 8) | buf[1]);
}

/**
 * @brief  Write a 16-bit register on INA226 (currently selected mux channel).
 * @retval HAL status.
 */
static HAL_StatusTypeDef INA226_WriteReg(uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)(value & 0xFF);
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, (I2C_ADDR_INA226 << 1), reg,
                                                  I2C_MEMADD_SIZE_8BIT, buf, 2, 50);
    if (status != HAL_OK) {
        i2c_fail_count++;
    }
    return status;
}

/**
 * @brief  Check that the TCA9548A multiplexer is present on the I2C bus.
 * @retval true if ACK received from 0x70, false otherwise.
 *
 * Called during Sensor_Init().  If the mux is absent, no INA226 channel
 * can be reached — all current/voltage readings will be zero.
 * The caller must enter SAFE state on failure.
 */
#define TCA9548A_PRESENCE_TRIALS      2    /* I2C address probe retries   */
#define TCA9548A_PRESENCE_TIMEOUT_MS  50   /* Timeout per trial (ms)      */

static bool TCA9548A_IsPresent(void)
{
    return (HAL_I2C_IsDeviceReady(&hi2c1, (I2C_ADDR_TCA9548A << 1),
                                  TCA9548A_PRESENCE_TRIALS,
                                  TCA9548A_PRESENCE_TIMEOUT_MS) == HAL_OK);
}

/**
 * @brief  Configure all INA226 sensors via TCA9548A multiplexer.
 *
 * Writes the configuration register (0x00) on each channel to ensure
 * a known state (averaging, conversion time, continuous mode) rather
 * than relying on power-on defaults.  Called once during Sensor_Init().
 */
static void INA226_ConfigureAll(void)
{
    for (uint8_t i = 0; i < NUM_INA226; i++) {
        if (TCA9548A_SelectChannel(i) != HAL_OK) continue;
        INA226_WriteReg(INA226_REG_CONFIG, INA226_CONFIG_VALUE);
    }
}

void Current_ReadAll(void)
{
    /* Reset per-cycle failure counter */
    i2c_fail_count = 0;

    for (uint8_t i = 0; i < NUM_INA226; i++) {
        if (TCA9548A_SelectChannel(i) != HAL_OK) {
            current_amps_raw[i] = 0.0f;
            current_amps[i]     = 0.0f;
            voltage_bus[i]      = 0.0f;
            continue;
        }

        /* Shunt voltage → current:  I = V_shunt / R_shunt
         * Use correct shunt resistance per channel:
         *   Channel 4 (battery): 0.75 mΩ (100A/75mV sensor)
         *   All others:          1.5 mΩ  (50A/75mV sensors)            */
        int16_t shunt_raw = INA226_ReadReg(INA226_REG_SHUNT_VOLTAGE);
        float shunt_uv    = (float)shunt_raw * INA226_SHUNT_LSB_UV;
        float shunt_mohm  = (i == INA226_CHANNEL_BATTERY)
                          ? (float)INA226_SHUNT_MOHM_BATTERY
                          : (float)INA226_SHUNT_MOHM_MOTOR;
        float raw_amps    = shunt_uv / shunt_mohm / 1000.0f;  /* µV/mΩ→mA→A */

        /* Battery channel: clamp negative current to 0.
         * The battery bus cannot sink current in this topology (no regen
         * charging path).  Negative readings on ch4 are noise artefacts
         * from INA226 offset; clamping prevents false fault triggers.
         * Motor channels: allow bidirectional (regen braking / back-EMF). */
        if (i == INA226_CHANNEL_BATTERY && raw_amps < 0.0f) {
            raw_amps = 0.0f;
        }

        current_amps_raw[i] = raw_amps;

        /* EMA filter: smooth transient spikes from PWM switching noise.
         * filtered = α × new + (1 − α) × old                           */
        if (!current_ema_primed) {
            current_amps[i] = raw_amps;     /* Prime on first cycle */
        } else {
            current_amps[i] = CURRENT_EMA_ALPHA * raw_amps
                            + (1.0f - CURRENT_EMA_ALPHA) * current_amps[i];
        }

        /* Bus voltage */
        int16_t bus_raw   = INA226_ReadReg(INA226_REG_BUS_VOLTAGE);
        voltage_bus[i]    = (float)bus_raw * INA226_BUS_LSB_MV / 1000.0f;  /* Convert mV to V */
    }

    current_ema_primed = true;

    /* I2C failure detection and recovery */
    if (i2c_fail_count >= I2C_FAIL_THRESHOLD) {
        if (i2c_recovery_attempts < I2C_RECOVERY_MAX_ATTEMPTS) {
            I2C_BusRecovery();
            i2c_recovery_attempts++;
        } else {
            /* Recovery exhausted — enter safe state */
            Safety_SetError(SAFETY_ERROR_I2C_FAILURE);
            Safety_SetState(SYS_STATE_SAFE);
        }
    } else {
        /* Successful cycle — reset recovery attempt counter */
        i2c_recovery_attempts = 0;
    }
}

float Current_GetAmps(uint8_t index) {
    if (index >= NUM_INA226) return 0.0f;
    return current_amps[index];
}

float Current_GetAmpsRaw(uint8_t index) {
    if (index >= NUM_INA226) return 0.0f;
    return current_amps_raw[index];
}

float Voltage_GetBus(uint8_t index) {
    if (index >= NUM_INA226) return 0.0f;
    return voltage_bus[index];
}

/* =========================================================================
 *  DS18B20 Temperature Sensors (OneWire bit-bang on PB0)
 * ========================================================================= */

static float temperatures[NUM_DS18B20] = {0};

/* DS18B20 64-bit ROM addresses discovered by Search ROM */
static uint8_t  ds18b20_rom[NUM_DS18B20][8];
static uint8_t  ds18b20_count = 0;

/* ----------------------------------------------------------------------
 * Sensor topology & staleness tracking (diagnostics — NOT on CAN)
 *
 * These are purely internal flags exposed via dedicated getters.  They
 * do NOT affect the existing safety system, the CAN protocol, or the
 * sensor-map frame (0x206).  They allow higher-level code (e.g. UI,
 * future diagnostic frame) to detect:
 *   - Missing sensors         (topology_invalid)
 *   - Frozen / dead sensors   (temp_stale[i])
 * without changing any existing public API.
 * -------------------------------------------------------------------- */

/* Minimum discovered sensor count required for a physically valid
 * topology.  If fewer than NUM_DS18B20 sensors are found the saved
 * physIdx→role map can no longer be guaranteed to be correct (a sensor
 * may have disappeared and shifted the discovery order).             */
#define DS18B20_MIN_REQUIRED    NUM_DS18B20

/* A sensor whose reading has not changed for this many milliseconds is
 * declared "stale".  DS18B20 at 1/16 °C resolution normally fluctuates
 * at least one LSB within a few seconds of ambient noise; a truly
 * frozen value is a strong indicator of a disconnected or dead sensor
 * (or a stuck 1-Wire bus).                                            */
#define DS18B20_STALE_TIMEOUT_MS 2000U

static bool     topology_invalid = true;                    /* until first scan */
static uint32_t temp_last_change_tick[NUM_DS18B20] = {0};
static float    temp_prev_value[NUM_DS18B20] = {0};
static bool     temp_stale[NUM_DS18B20] = {false};

/* Hot-plug rescan interval (ms) — re-enumerate OneWire bus periodically */
#define OW_RESCAN_INTERVAL_MS  10000   /* 10 seconds between rescans */

/*
 * OneWire bit-bang helpers (simplified – production code should use a
 * dedicated OneWire library or DMA-based UART trick).
 */

static void OW_SetOutput(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = PIN_ONEWIRE;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void OW_SetInput(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = PIN_ONEWIRE;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void OW_DelayUs(uint16_t us)
{
    /* Simple busy-wait delay – for microsecond precision on 170 MHz Cortex-M4
     * each NOP ~6 ns, so ~167 NOPs per µs.  Good enough for OneWire timing. */
    uint32_t loops = (uint32_t)us * 42;  /* Approximate: 170MHz / 4 cycles ≈ 42 */
    while (loops--) { __NOP(); }
}

static uint8_t OW_Reset(void)
{
    OW_SetOutput();
    HAL_GPIO_WritePin(GPIOB, PIN_ONEWIRE, GPIO_PIN_RESET);
    OW_DelayUs(480);
    OW_SetInput();
    OW_DelayUs(70);
    uint8_t presence = (HAL_GPIO_ReadPin(GPIOB, PIN_ONEWIRE) == GPIO_PIN_RESET) ? 1 : 0;
    OW_DelayUs(410);
    return presence;
}

static void OW_WriteBit(uint8_t bit)
{
    OW_SetOutput();
    HAL_GPIO_WritePin(GPIOB, PIN_ONEWIRE, GPIO_PIN_RESET);
    if (bit) {
        OW_DelayUs(6);
        OW_SetInput();
        OW_DelayUs(64);
    } else {
        OW_DelayUs(60);
        OW_SetInput();
        OW_DelayUs(10);
    }
}

static uint8_t OW_ReadBit(void)
{
    OW_SetOutput();
    HAL_GPIO_WritePin(GPIOB, PIN_ONEWIRE, GPIO_PIN_RESET);
    OW_DelayUs(6);
    OW_SetInput();
    OW_DelayUs(9);
    uint8_t bit = (HAL_GPIO_ReadPin(GPIOB, PIN_ONEWIRE) == GPIO_PIN_SET) ? 1 : 0;
    OW_DelayUs(55);
    return bit;
}

static void OW_WriteByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        OW_WriteBit(byte & 0x01);
        byte >>= 1;
    }
}

static uint8_t OW_ReadByte(void)
{
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        byte >>= 1;
        if (OW_ReadBit()) byte |= 0x80;
    }
    return byte;
}

/* -------------------------------------------------------------------------
 *  DS18B20 ROM Search algorithm (per Maxim/Dallas AN187)
 *
 *  Discovers up to NUM_DS18B20 unique 64-bit ROM codes on the bus.
 *  Must be called once at init (from Sensor_Init) before first read.
 * ------------------------------------------------------------------------- */

#define OW_SEARCH_ERROR  (-1)

/**
 * @brief  CRC-8/MAXIM (poly 0x31, reflected as 0x8C, init 0x00).
 *         Used to verify DS18B20 ROM codes and scratchpad data.
 */
static uint8_t OW_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            byte >>= 1;
        }
    }
    return crc;
}

/**
 * @brief  Perform one pass of the OneWire Search ROM algorithm.
 *
 * @param  rom          8-byte buffer to store discovered ROM code.
 * @param  last_discrepancy  Set to 0 for first call; updated each pass.
 * @retval Next discrepancy bit position (1-64), 0 when search completes
 *         successfully, or OW_SEARCH_ERROR on error/no devices.
 */
static int OW_SearchROM_Next(uint8_t rom[8], int last_discrepancy)
{
    if (!OW_Reset()) return OW_SEARCH_ERROR; /* No presence pulse → no devices */

    OW_WriteByte(0xF0);                    /* Search ROM command */

    int discrepancy_marker = 0;

    for (int bit_idx = 1; bit_idx <= 64; bit_idx++) {
        uint8_t id_bit      = OW_ReadBit();
        uint8_t id_bit_comp = OW_ReadBit();

        if (id_bit && id_bit_comp) {
            /* No devices responding – abort */
            return OW_SEARCH_ERROR;
        }

        uint8_t direction;
        if (id_bit != id_bit_comp) {
            /* All remaining devices agree on this bit */
            direction = id_bit;
        } else {
            /* Discrepancy: devices with 0 and 1 both present */
            if (bit_idx == last_discrepancy) {
                direction = 1;             /* Take the 1-branch this time */
            } else if (bit_idx > last_discrepancy) {
                direction = 0;             /* Default: take 0-branch first */
            } else {
                /* Reproduce the path from the previous search */
                uint8_t byte_idx = (uint8_t)((bit_idx - 1) / 8);
                uint8_t bit_mask = (uint8_t)(1U << ((bit_idx - 1) % 8));
                direction = (rom[byte_idx] & bit_mask) ? 1 : 0;
            }
            if (direction == 0) {
                discrepancy_marker = bit_idx;
            }
        }

        /* Write chosen direction back to bus */
        OW_WriteBit(direction);

        /* Store bit in ROM buffer */
        {
            uint8_t byte_idx = (uint8_t)((bit_idx - 1) / 8);
            uint8_t bit_mask = (uint8_t)(1U << ((bit_idx - 1) % 8));
            if (direction) {
                rom[byte_idx] |= bit_mask;
            } else {
                rom[byte_idx] &= (uint8_t)~bit_mask;
            }
        }
    }

    return discrepancy_marker;
}

/**
 * @brief  Enumerate all DS18B20 devices on the bus.
 *         Populates ds18b20_rom[] and sets ds18b20_count.
 */
static void OW_SearchAll(void)
{
    uint8_t rom[8] = {0};
    int last_discrepancy = 0;

    uint8_t prev_count = ds18b20_count;
    ds18b20_count = 0;

    do {
        int next = OW_SearchROM_Next(rom, last_discrepancy);
        if (next == OW_SEARCH_ERROR) break;   /* Error or no device */

        /* Validate CRC of discovered ROM */
        if (OW_CRC8(rom, 7) != rom[7]) break; /* Bad CRC – stop */

        /* Accept only DS18B20 family code (0x28) */
        if (rom[0] == 0x28 && ds18b20_count < NUM_DS18B20) {
            for (uint8_t j = 0; j < 8; j++) {
                ds18b20_rom[ds18b20_count][j] = rom[j];
            }
            ds18b20_count++;
        }

        last_discrepancy = next;
    } while (last_discrepancy != 0);

    /* Hot-plug removal: clear stale temperature data for sensors
     * that are no longer present after re-enumeration.              */
    for (uint8_t i = ds18b20_count; i < prev_count && i < NUM_DS18B20; i++) {
        temperatures[i] = 0.0f;
        /* Reset stale tracking for slots that disappeared so that a
         * later re-discovery starts from a clean baseline rather than
         * inheriting a frozen timestamp from a dead sensor.          */
        temp_stale[i]            = false;
        temp_prev_value[i]       = 0.0f;
        temp_last_change_tick[i] = HAL_GetTick();
    }

    /* -----------------------------------------------------------------
     * TASK 1 — sensor topology validity.
     *
     * The persisted physIdx→role map is bound to the exact 1-Wire
     * discovery order in effect when the user performed the mapping.
     * If a sensor later disappears, Search ROM returns fewer entries
     * AND the surviving sensors may have shifted positions — so the
     * stored map can no longer be trusted.
     *
     * We expose this as a read-only flag (`topology_invalid`).  No CAN
     * frame is altered; higher layers (UI / future diagnostic frame)
     * may query it via Temperature_IsTopologyValid().
     * --------------------------------------------------------------- */
    topology_invalid = (ds18b20_count < DS18B20_MIN_REQUIRED);
}

/**
 * @brief  Read scratchpad from one specific DS18B20 using Match ROM (0x55).
 * @param  idx  Index into ds18b20_rom[].
 * @retval Temperature in °C, or 0.0f on failure.
 */
static float OW_ReadTemperature(uint8_t idx)
{
    if (idx >= ds18b20_count) return 0.0f;

    if (!OW_Reset()) return 0.0f;

    OW_WriteByte(0x55);                      /* Match ROM */
    for (uint8_t i = 0; i < 8; i++) {
        OW_WriteByte(ds18b20_rom[idx][i]);
    }
    OW_WriteByte(0xBE);                      /* Read Scratchpad */

    uint8_t scratch[9];
    for (uint8_t i = 0; i < 9; i++) {
        scratch[i] = OW_ReadByte();
    }

    /* CRC check on scratchpad */
    if (OW_CRC8(scratch, 8) != scratch[8]) {
        Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
        return 0.0f;
    }

    int16_t raw = (int16_t)((scratch[1] << 8) | scratch[0]);
    float temp = (float)raw / 16.0f;

    /* DS18B20 operating range: −55 °C to +125 °C.
     * Values outside this range indicate corrupted data.            */
    if (temp < -55.0f || temp > 125.0f) {
        Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
        return 0.0f;
    }

    return temp;
}

void Temperature_StartConversion(void)
{
    if (!OW_Reset()) return;
    OW_WriteByte(0xCC);  /* Skip ROM – all sensors convert simultaneously */
    OW_WriteByte(0x44);  /* Start conversion */
}

void Temperature_ReadAll(void)
{
    if (ds18b20_count == 0) {
        /* No sensors discovered yet – fall back to Skip ROM single read
         * so temperatures[0] still provides a value during early boot.  */
        if (!OW_Reset()) return;
        OW_WriteByte(0xCC);  /* Skip ROM */
        OW_WriteByte(0xBE);  /* Read Scratchpad */

        uint8_t lsb = OW_ReadByte();
        uint8_t msb = OW_ReadByte();
        int16_t raw = (int16_t)((msb << 8) | lsb);
        float temp = (float)raw / 16.0f;

        /* DS18B20 operating range: −55 °C to +125 °C.
         * Values outside this range indicate corrupted data.
         * Must match the validation in OW_ReadTemperature().        */
        if (temp < -55.0f || temp > 125.0f) {
            Safety_SetError(SAFETY_ERROR_SENSOR_FAULT);
            temperatures[0] = 0.0f;
        } else {
            temperatures[0] = temp;
        }
        return;
    }

    /* Read each discovered sensor individually via Match ROM */
    for (uint8_t i = 0; i < ds18b20_count; i++) {
        temperatures[i] = OW_ReadTemperature(i);
    }

    /* -----------------------------------------------------------------
     * TASK 2 — stale detection.
     *
     * After every read-cycle, per-sensor: if the raw value is the
     * exact same bit-pattern as last time, do not refresh the
     * "last change" timestamp.  Once the stale window elapses with no
     * change, the sensor is marked stale.  A real DS18B20 at 1/16 °C
     * resolution practically always moves at least one LSB within the
     * 2 s window in any normal vehicle environment; a stuck value
     * therefore reliably signals a dead or disconnected sensor (or a
     * CRC-pegged failure: OW_ReadTemperature() returns 0.0f on CRC
     * error, and a real sensor rarely reports exactly 0.0f for 2 s).
     *
     * This is informational only.  The Safety_SetError() calls inside
     * OW_ReadTemperature() remain the authoritative safety path.
     * --------------------------------------------------------------- */
    {
        uint32_t now = HAL_GetTick();
        for (uint8_t i = 0; i < NUM_DS18B20; i++) {
            if (i >= ds18b20_count) {
                /* Not discovered — not applicable. Keep flag false so
                 * UI does not confuse "missing" with "stale"; the
                 * topology flag covers the missing case.              */
                temp_stale[i] = false;
                continue;
            }
            if (temperatures[i] != temp_prev_value[i]) {
                temp_prev_value[i]       = temperatures[i];
                temp_last_change_tick[i] = now;
                temp_stale[i]            = false;
            } else if ((uint32_t)(now - temp_last_change_tick[i])
                       >= DS18B20_STALE_TIMEOUT_MS) {
                temp_stale[i] = true;
            }
        }
    }
}

float Temperature_Get(uint8_t index)
{
    if (index >= NUM_DS18B20) return 0.0f;
    return temperatures[index];
}

uint8_t Temperature_GetCount(void)
{
    return ds18b20_count;
}

/* -----------------------------------------------------------------------
 * DS18B20 diagnostic accessors (TASK 1 / 2 / 4)
 *
 * These getters are the ONLY way external code queries topology /
 * staleness state.  They do not allocate, do not block, and must not
 * mutate any shared state — they are safe to call from any tier of the
 * main cooperative loop.  No CAN frame format is altered.
 * -----------------------------------------------------------------------*/

/**
 * @brief  True when the discovered sensor count matches expectations.
 *         False if any sensor is missing after the latest Search ROM.
 *         When false, the persisted physIdx→role mapping is potentially
 *         misaligned with physical positions.
 */
bool Temperature_IsTopologyValid(void)
{
    return !topology_invalid;
}

/**
 * @brief  True if sensor @p idx has produced the same value for at
 *         least DS18B20_STALE_TIMEOUT_MS — a strong indicator of a
 *         frozen, dead, or disconnected sensor.
 *         Returns false for out-of-range indices or sensors that have
 *         not yet been discovered.
 */
bool Temperature_IsStale(uint8_t idx)
{
    if (idx >= NUM_DS18B20) return false;
    return temp_stale[idx];
}

/**
 * @brief  Packed snapshot of DS18B20 diagnostic state.
 *
 * Layout (uint16_t):
 *   bits  0..2  — discovered sensor count (0..7, masked)
 *   bit   3     — topology invalid (1 = missing sensors)
 *   bits  8..12 — stale bitmask, one bit per physIdx (bit 8 = physIdx 0)
 *
 * Intended for internal logging / future diagnostic frame.  Fits in a
 * single CAN byte pair if needed, but this function does not transmit
 * anything by itself.
 */
uint16_t Temperature_GetDiagnosticFlags(void)
{
    uint16_t flags = (uint16_t)(ds18b20_count & 0x07U);
    if (topology_invalid) flags |= (uint16_t)(1U << 3);
    for (uint8_t i = 0; i < NUM_DS18B20 && i < 5U; i++) {
        if (temp_stale[i]) flags |= (uint16_t)(1U << (8U + i));
    }
    return flags;
}

/* ---- DS18B20 hot-plug detection ----
 * Periodically re-enumerates the OneWire bus to detect sensors
 * added or removed at runtime.  Called from the 1000 ms tier
 * in main.c; actual rescan executes once every OW_RESCAN_INTERVAL_MS. */

static uint32_t ow_rescan_tick = 0;

void Temperature_PeriodicRescan(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - ow_rescan_tick) < OW_RESCAN_INTERVAL_MS) return;
    ow_rescan_tick = now;

    OW_SearchAll();
}

/* =========================================================================
 *  Initialization
 * ========================================================================= */

void Sensor_Init(void)
{
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        wheel_pulse[i]              = 0;
        wheel_pulse_prev[i]         = 0;
        wheel_last_tick[i]          = HAL_GetTick();
        wheel_speed_kmh[i]          = 0.0f;
        wheel_rpm[i]                = 0.0f;
        wheel_last_pulse_tick[i]    = 0;
        wheel_prev_pulse_tick[i]    = 0;
        wheel_flood_count[i]        = 0;
        wheel_flood_window_start[i] = 0;
    }

    pedal_raw_adc  = 0;
    pedal_raw_adc2 = 0;
    pedal_pct      = 0.0f;
    pedal_pct_raw  = 0.0f;
    pedal_pct_prev = 0.0f;
    pedal_ema      = 0.0f;
    pedal_plausible        = true;
    pedal_channels_contradict = false;
    pedal_ema_primed       = false;

    for (uint8_t i = 0; i < NUM_INA226; i++) {
        current_amps[i]     = 0.0f;
        current_amps_raw[i] = 0.0f;
        voltage_bus[i]      = 0.0f;
    }

    i2c_fail_count        = 0;
    i2c_recovery_attempts = 0;
    current_ema_primed    = false;

    /* TCA9548A multiplexer presence check (CRITICAL).
     * If the mux is absent, no INA226 channel is reachable and all
     * current/voltage readings will be zero — enter SAFE state.       */
    if (!TCA9548A_IsPresent()) {
        Safety_SetError(SAFETY_ERROR_I2C_FAILURE);
        Safety_SetState(SYS_STATE_SAFE);
        /* Continue init so that DS18B20 and other sensors are still set up;
         * INA226_ConfigureAll will silently fail on each channel.         */
    }

    /* Explicitly configure all INA226 sensors to a known state
     * (averaging, conversion time, continuous mode) rather than
     * relying on power-on defaults — safety-critical requirement.     */
    INA226_ConfigureAll();

    for (uint8_t i = 0; i < NUM_DS18B20; i++) {
        temperatures[i] = 0.0f;
    }

    /* Discover all DS18B20 sensors on the OneWire bus.
     * This populates ds18b20_rom[] with their 64-bit addresses. */
    OW_SearchAll();
}