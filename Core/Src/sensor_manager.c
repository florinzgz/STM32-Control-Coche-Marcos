/**
  ****************************************************************************
  * @file    sensor_manager.c
  * @brief   Sensor acquisition: wheel speed, DS18B20, INA226, pedal (dual-ch)
  *
  * Hardware managed by this module:
  *   - 4× LJ12A3 inductive wheel speed sensors (EXTI interrupts)
  *   - 5× DS18B20 temperature sensors (OneWire on PB0)
  *   - 6× INA226 current/voltage sensors (I2C via TCA9548A multiplexer)
  *   - 1× Hall-effect pedal: dual-channel redundant reading
  *         Primary:      internal ADC1 on PA3 (via voltage divider)
  *         Plausibility: ADS1115 16-bit I2C ADC (full 5V range)
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
 *  Pedal – Dual-channel redundant reading (automotive-grade architecture)
 *
 *  PRIMARY channel: Internal ADC1 on PA3 (fast, ~1 µs conversion)
 *    The 5V pedal signal is scaled to 0–3.3V via a voltage divider
 *    (10 kΩ series + 6.8 kΩ to GND → Vout/Vin = 6.8/(10+6.8) = 0.4048).
 *    With pedal range 0.3V–4.8V → divider output 0.121V–1.943V.
 *    ADC 12-bit (3.3V): 0.121V → ~150 counts, 1.943V → ~2413 counts.
 *
 *  PLAUSIBILITY channel: ADS1115 16-bit I2C ADC (slow, ~8 ms)
 *    Reads the unscaled 5V signal on A0 for cross-validation.
 *    Used to verify the primary channel is not stuck or shorted.
 *
 *  Cross-validation: both channels must agree within ±5% pedal range.
 *  If they diverge for >200 ms, a pedal plausibility fault is raised.
 *  If ADS1115 I2C fails, the primary channel continues operating but
 *  a sensor degraded warning is set (single-channel mode).
 * ========================================================================= */

static uint16_t pedal_raw_adc  = 0;     /* Primary: internal ADC raw (12-bit) */
static uint16_t pedal_raw_ads  = 0;     /* Plausibility: ADS1115 raw (16-bit) */
static float    pedal_pct      = 0.0f;  /* Control output: 0–100% from primary */
static float    pedal_pct_ads  = 0.0f;  /* Plausibility: 0–100% from ADS1115  */
static bool     pedal_plausible = true; /* Cross-validation result             */
static bool     pedal_ads_ever_read = false; /* First successful ADS1115 read? */
static uint32_t pedal_ads_last_ok_tick = 0;  /* Last successful ADS1115 read   */
static uint32_t pedal_diverge_start    = 0;  /* When channels started diverging */

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;

/* ADS1115 register addresses */
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01

/* ADS1115 config: single-shot, AIN0, ±6.144V, 128 SPS = 0xC183 */
#define ADS1115_CONFIG_PEDAL    0xC183U

/* Primary ADC calibration (voltage divider 10kΩ + 6.8kΩ, 12-bit, 3.3V ref)
 * Divider ratio: 6.8/(10+6.8) = 0.4048
 * Pedal 0.3V released → 0.3 × 0.4048 = 0.121V → 0.121/3.3 × 4095 ≈ 150
 * Pedal 4.8V pressed  → 4.8 × 0.4048 = 1.943V → 1.943/3.3 × 4095 ≈ 2413 */
#define PEDAL_ADC_MIN   150U     /* ~0.3V (pedal released), after divider */
#define PEDAL_ADC_MAX   2413U    /* ~4.8V (pedal fully pressed), after divider */

/* ADS1115 calibration (PGA ±6.144V, LSB = 187.5 µV)
 * Pedal 0.3V → 0.3/0.0001875 = 1600 counts
 * Pedal 4.8V → 4.8/0.0001875 = 25600 counts */
#define PEDAL_ADS_MIN   1600U
#define PEDAL_ADS_MAX   25600U

/* Cross-validation tolerance: 5% of full pedal range */
#define PEDAL_PLAUSIBILITY_PCT  5.0f
/* Time before divergence becomes a fault (ms) */
#define PEDAL_DIVERGE_TIMEOUT_MS 200U
/* ADS1115 stale data timeout (ms) — if I2C fails this long, warn */
#define PEDAL_ADS_STALE_TIMEOUT_MS 500U

/**
 * @brief  Read primary pedal channel via internal ADC (fast, ~1 µs).
 */
static void Pedal_ReadADC(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK) {
        pedal_raw_adc = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    /* Map calibrated range to 0–100% */
    if (pedal_raw_adc <= PEDAL_ADC_MIN) {
        pedal_pct = 0.0f;
    } else if (pedal_raw_adc >= PEDAL_ADC_MAX) {
        pedal_pct = 100.0f;
    } else {
        pedal_pct = (float)(pedal_raw_adc - PEDAL_ADC_MIN) * 100.0f
                  / (float)(PEDAL_ADC_MAX - PEDAL_ADC_MIN);
    }
}

/**
 * @brief  Read plausibility channel via ADS1115 I2C ADC (slow, ~8 ms).
 * @retval true if I2C read succeeded, false on I2C failure.
 */
static bool Pedal_ReadADS1115(void)
{
    /* Start single-shot conversion */
    uint8_t cfg[3];
    cfg[0] = ADS1115_REG_CONFIG;
    cfg[1] = (uint8_t)(ADS1115_CONFIG_PEDAL >> 8);
    cfg[2] = (uint8_t)(ADS1115_CONFIG_PEDAL & 0xFF);

    if (HAL_I2C_Master_Transmit(&hi2c1, (I2C_ADDR_ADS1115 << 1),
                                 cfg, 3, 10) != HAL_OK) {
        return false;
    }

    /* Wait for conversion (128 SPS → ~8 ms) */
    HAL_Delay(8);

    /* Read conversion result */
    uint8_t reg = ADS1115_REG_CONVERSION;
    uint8_t result[2] = {0};
    if (HAL_I2C_Master_Transmit(&hi2c1, (I2C_ADDR_ADS1115 << 1),
                                 &reg, 1, 5) != HAL_OK) {
        return false;
    }
    if (HAL_I2C_Master_Receive(&hi2c1, (I2C_ADDR_ADS1115 << 1),
                                result, 2, 5) != HAL_OK) {
        return false;
    }

    int16_t raw_signed = (int16_t)((result[0] << 8) | result[1]);
    uint16_t raw = (raw_signed < 0) ? 0U : (uint16_t)raw_signed;
    pedal_raw_ads = raw;

    /* Map calibrated range to 0–100% */
    if (raw <= PEDAL_ADS_MIN) {
        pedal_pct_ads = 0.0f;
    } else if (raw >= PEDAL_ADS_MAX) {
        pedal_pct_ads = 100.0f;
    } else {
        pedal_pct_ads = (float)(raw - PEDAL_ADS_MIN) * 100.0f
                      / (float)(PEDAL_ADS_MAX - PEDAL_ADS_MIN);
    }

    pedal_ads_last_ok_tick = HAL_GetTick();
    pedal_ads_ever_read = true;
    return true;
}

void Pedal_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* 1. Always read primary channel first (fast, ~1 µs) */
    Pedal_ReadADC();

    /* 2. Read plausibility channel (slow, ~8 ms — still fits in 50 ms slot) */
    bool ads_ok = Pedal_ReadADS1115();

    /* 3. Cross-validate both channels */
    if (ads_ok) {
        float diff = pedal_pct - pedal_pct_ads;
        if (diff < 0.0f) diff = -diff;

        if (diff > PEDAL_PLAUSIBILITY_PCT) {
            /* Channels disagree — start / continue divergence timer */
            if (pedal_diverge_start == 0) {
                pedal_diverge_start = now;
            }
            if ((now - pedal_diverge_start) >= PEDAL_DIVERGE_TIMEOUT_MS) {
                /* Sustained divergence → plausibility fault */
                pedal_plausible = false;
            }
        } else {
            /* Channels agree — reset divergence timer */
            pedal_diverge_start = 0;
            pedal_plausible = true;
        }
    } else {
        /* ADS1115 I2C failed — check stale timeout (only after first
         * successful read, to avoid false alarm during boot) */
        if (pedal_ads_ever_read &&
            (now - pedal_ads_last_ok_tick) >= PEDAL_ADS_STALE_TIMEOUT_MS) {
            /* ADS1115 has been offline too long — flag as degraded but
             * continue using primary ADC (single-channel fallback).
             * Reset divergence timer so recovery starts fresh. */
            pedal_plausible = false;
            pedal_diverge_start = 0;
        }
    }
}

float Pedal_GetValue(void)       { return (float)pedal_raw_adc; }
float Pedal_GetPercent(void)     { return pedal_pct; }
bool  Pedal_IsPlausible(void)    { return pedal_plausible; }
float Pedal_GetADSPercent(void)  { return pedal_pct_ads; }

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

#define INA226_REG_SHUNT_VOLTAGE   0x01
#define INA226_REG_BUS_VOLTAGE     0x02
#define INA226_SHUNT_LSB_UV        2.5f   /* 2.5 µV per LSB */
#define INA226_BUS_LSB_MV          1.25f  /* 1.25 mV per LSB */

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

void Current_ReadAll(void)
{
    /* Reset per-cycle failure counter */
    i2c_fail_count = 0;

    for (uint8_t i = 0; i < NUM_INA226; i++) {
        if (TCA9548A_SelectChannel(i) != HAL_OK) {
            current_amps[i] = 0.0f;
            voltage_bus[i]  = 0.0f;
            continue;
        }

        /* Shunt voltage → current:  I = V_shunt / R_shunt
         * Use correct shunt resistance per channel:
         *   Channel 4 (battery): 0.5 mΩ (100A sensor)
         *   All others:          1.0 mΩ (50A sensors)             */
        int16_t shunt_raw = INA226_ReadReg(INA226_REG_SHUNT_VOLTAGE);
        float shunt_uv    = (float)shunt_raw * INA226_SHUNT_LSB_UV;
        float shunt_mohm  = (i == INA226_CHANNEL_BATTERY)
                          ? (float)INA226_SHUNT_MOHM_BATTERY
                          : (float)INA226_SHUNT_MOHM_MOTOR;
        current_amps[i]   = shunt_uv / shunt_mohm;  /* µV / mΩ = mA */
        current_amps[i]  /= 1000.0f;  /* Convert mA to A */

        /* Bus voltage */
        int16_t bus_raw   = INA226_ReadReg(INA226_REG_BUS_VOLTAGE);
        voltage_bus[i]    = (float)bus_raw * INA226_BUS_LSB_MV / 1000.0f;  /* Convert mV to V */
    }

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
    if (OW_CRC8(scratch, 8) != scratch[8]) return 0.0f;

    int16_t raw = (int16_t)((scratch[1] << 8) | scratch[0]);
    return (float)raw / 16.0f;
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
        temperatures[0] = (float)raw / 16.0f;
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

    pedal_raw_adc = 0;
    pedal_raw_ads = 0;
    pedal_pct     = 0.0f;
    pedal_pct_ads = 0.0f;
    pedal_plausible        = true;
    pedal_ads_ever_read    = false;
    pedal_ads_last_ok_tick = 0;
    pedal_diverge_start    = 0;

    for (uint8_t i = 0; i < NUM_INA226; i++) {
        current_amps[i] = 0.0f;
        voltage_bus[i]  = 0.0f;
    }

    i2c_fail_count        = 0;
    i2c_recovery_attempts = 0;

    for (uint8_t i = 0; i < NUM_DS18B20; i++) {
        temperatures[i] = 0.0f;
    }

    /* Discover all DS18B20 sensors on the OneWire bus.
     * This populates ds18b20_rom[] with their 64-bit addresses. */
    OW_SearchAll();
}