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
 * ========================================================================= */

/* Pulse counters incremented in EXTI ISR */
static volatile uint32_t wheel_pulse[NUM_WHEELS] = {0};
static uint32_t wheel_pulse_prev[NUM_WHEELS]     = {0};
static uint32_t wheel_last_tick[NUM_WHEELS]       = {0};
static float    wheel_speed_kmh[NUM_WHEELS]       = {0};
static float    wheel_rpm[NUM_WHEELS]             = {0};

/* Software debounce: minimum interval between accepted pulses.
 * At 60 km/h with 1.1 m circumference and 6 pulses/rev:
 *   freq = (60/3.6)/1.1 × 6 ≈ 91 Hz → period ≈ 11 ms.
 * A 1 ms blanking window (HAL_GetTick resolution) rejects contact
 * bounce without affecting valid pulses at any realistic speed.       */
static volatile uint32_t wheel_last_pulse_tick[NUM_WHEELS] = {0};

static inline void Wheel_IRQDebounced(uint8_t idx)
{
    uint32_t now = HAL_GetTick();
    /* HAL_GetTick() has 1 ms resolution; accept pulse only if at least
     * 1 ms has elapsed since the last accepted pulse.  This provides
     * effective debounce for mechanical contact bounce.               */
    if ((now - wheel_last_pulse_tick[idx]) >= 1U) {
        wheel_pulse[idx]++;
        wheel_last_pulse_tick[idx] = now;
    }
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
 * Called periodically (e.g. every 50 ms) from the main loop.
 * speed_kmh = (pulses_delta / PULSES_PER_REV) * CIRCUMFERENCE * (1000/dt_ms) * 3.6
 */
static void Wheel_ComputeSpeed(uint8_t idx)
{
    uint32_t now    = HAL_GetTick();
    uint32_t dt     = now - wheel_last_tick[idx];
    if (dt == 0) return;

    uint32_t pulses = wheel_pulse[idx];
    uint32_t delta  = pulses - wheel_pulse_prev[idx];

    float revolutions = (float)delta / (float)WHEEL_PULSES_REV;
    float dist_m      = revolutions * WHEEL_CIRCUMF_M;
    float speed_ms    = dist_m * 1000.0f / (float)dt;

    wheel_speed_kmh[idx] = speed_ms * 3.6f;
    wheel_rpm[idx]       = revolutions * 60000.0f / (float)dt;

    wheel_pulse_prev[idx] = pulses;
    wheel_last_tick[idx]  = now;
}

float Wheel_GetSpeed_FL(void) { Wheel_ComputeSpeed(0); return wheel_speed_kmh[0]; }
float Wheel_GetSpeed_FR(void) { Wheel_ComputeSpeed(1); return wheel_speed_kmh[1]; }
float Wheel_GetSpeed_RL(void) { Wheel_ComputeSpeed(2); return wheel_speed_kmh[2]; }
float Wheel_GetSpeed_RR(void) { Wheel_ComputeSpeed(3); return wheel_speed_kmh[3]; }
float Wheel_GetRPM_FL(void)   { return wheel_rpm[0]; }

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
static bool TCA9548A_IsPresent(void)
{
    return (HAL_I2C_IsDeviceReady(&hi2c1, (I2C_ADDR_TCA9548A << 1), 2, 50) == HAL_OK);
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
    }
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
        wheel_pulse[i]           = 0;
        wheel_pulse_prev[i]      = 0;
        wheel_last_tick[i]       = HAL_GetTick();
        wheel_speed_kmh[i]       = 0.0f;
        wheel_rpm[i]             = 0.0f;
        wheel_last_pulse_tick[i] = 0;
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