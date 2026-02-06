/**
  ****************************************************************************
  * @file    sensor_manager.c
  * @brief   Sensor acquisition: wheel speed, DS18B20, INA226, pedal ADC
  *
  * Hardware managed by this module:
  *   - 4× LJ12A3 inductive wheel speed sensors (EXTI interrupts)
  *   - 5× DS18B20 temperature sensors (OneWire on PB0)
  *   - 6× INA226 current/voltage sensors (I2C via TCA9548A multiplexer)
  *   - 1× Hall-effect pedal (ADC1)
  ****************************************************************************
  */

#include "sensor_manager.h"
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

void Wheel_FL_IRQHandler(void) { wheel_pulse[0]++; }
void Wheel_FR_IRQHandler(void) { wheel_pulse[1]++; }
void Wheel_RL_IRQHandler(void) { wheel_pulse[2]++; }
void Wheel_RR_IRQHandler(void) { wheel_pulse[3]++; }

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
 *  Pedal – ADC single conversion
 * ========================================================================= */

static uint16_t pedal_raw   = 0;
static float    pedal_pct   = 0.0f;

extern ADC_HandleTypeDef hadc1;

void Pedal_Update(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        pedal_raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    pedal_pct = (float)pedal_raw * 100.0f / 4095.0f;
}

float Pedal_GetValue(void)   { return (float)pedal_raw; }
float Pedal_GetPercent(void) { return pedal_pct; }

/* =========================================================================
 *  INA226 Current Sensors via TCA9548A I2C multiplexer
 * ========================================================================= */

static float current_amps[NUM_INA226]  = {0};
static float voltage_bus[NUM_INA226]   = {0};

extern I2C_HandleTypeDef hi2c1;

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
    return HAL_I2C_Master_Transmit(&hi2c1, (I2C_ADDR_TCA9548A << 1), &data, 1, 50);
}

/**
 * @brief  Read a 16-bit register from INA226 on the currently selected channel.
 */
static int16_t INA226_ReadReg(uint8_t reg)
{
    uint8_t buf[2] = {0};
    HAL_I2C_Mem_Read(&hi2c1, (I2C_ADDR_INA226 << 1), reg, I2C_MEMADD_SIZE_8BIT,
                     buf, 2, 50);
    return (int16_t)((buf[0] << 8) | buf[1]);
}

void Current_ReadAll(void)
{
    for (uint8_t i = 0; i < NUM_INA226; i++) {
        if (TCA9548A_SelectChannel(i) != HAL_OK) {
            current_amps[i] = 0.0f;
            voltage_bus[i]  = 0.0f;
            continue;
        }

        /* Shunt voltage → current:  I = V_shunt / R_shunt */
        int16_t shunt_raw = INA226_ReadReg(INA226_REG_SHUNT_VOLTAGE);
        float shunt_uv    = (float)shunt_raw * INA226_SHUNT_LSB_UV;
        current_amps[i]   = shunt_uv / (float)(INA226_SHUNT_MOHM);  /* µV / mΩ = mA → ÷1000 later if needed */
        current_amps[i]  /= 1000.0f;  /* Convert mA to A */

        /* Bus voltage */
        int16_t bus_raw   = INA226_ReadReg(INA226_REG_BUS_VOLTAGE);
        voltage_bus[i]    = (float)bus_raw * INA226_BUS_LSB_MV / 1000.0f;  /* Convert mV to V */
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

/**
 * @brief  CRC-8/MAXIM (poly 0x31, init 0x00, reflect I/O).
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
 * @retval  Next discrepancy marker (0 = search complete, no more devices).
 */
static int OW_SearchROM_Next(uint8_t rom[8], int last_discrepancy)
{
    if (!OW_Reset()) return -1;            /* No presence pulse → no devices */

    OW_WriteByte(0xF0);                    /* Search ROM command */

    int discrepancy_marker = 0;

    for (int bit_idx = 1; bit_idx <= 64; bit_idx++) {
        uint8_t id_bit      = OW_ReadBit();
        uint8_t id_bit_comp = OW_ReadBit();

        if (id_bit && id_bit_comp) {
            /* No devices responding – abort */
            return -1;
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
        if (next < 0) break;                  /* Error or no device */

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
        wheel_pulse[i]      = 0;
        wheel_pulse_prev[i] = 0;
        wheel_last_tick[i]  = HAL_GetTick();
        wheel_speed_kmh[i]  = 0.0f;
        wheel_rpm[i]        = 0.0f;
    }

    pedal_raw = 0;
    pedal_pct = 0.0f;

    for (uint8_t i = 0; i < NUM_INA226; i++) {
        current_amps[i] = 0.0f;
        voltage_bus[i]  = 0.0f;
    }

    for (uint8_t i = 0; i < NUM_DS18B20; i++) {
        temperatures[i] = 0.0f;
    }

    /* Discover all DS18B20 sensors on the OneWire bus.
     * This populates ds18b20_rom[] with their 64-bit addresses. */
    OW_SearchAll();
}