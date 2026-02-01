/**
  ******************************************************************************
  * @file    sensor_manager.c
  * @brief   Sensor management implementation
  * @author  florinzgz
  * @date    2026-02-01
  ******************************************************************************
  * @attention
  *
  * Gestión de sensores:
  * - 4 sensores de velocidad de rueda (EXTI interrupts)
  * - 5 sensores de temperatura DS18B20 (OneWire)
  * - 6 sensores de corriente INA226 (I2C via TCA9548A multiplexer)
  * - 1 pedal analógico (ADC)
  * - 1 selector de marcha (3 GPIO)
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sensor_manager.h"
#include <string.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* INA226 Register addresses */
#define INA226_REG_CONFIG       0x00
#define INA226_REG_SHUNT_V      0x01
#define INA226_REG_BUS_V        0x02
#define INA226_REG_POWER        0x03
#define INA226_REG_CURRENT      0x04
#define INA226_REG_CALIBRATION  0x05

/* INA226 Configuration */
#define INA226_CONFIG_VALUE     0x4527  /* AVG=16, VBUS CT=1.1ms, VSH CT=1.1ms, Continuous */
#define INA226_CAL_VALUE        2048    /* Calibration for 1mΩ shunt, 20A max */

/* OneWire commands */
#define ONEWIRE_CMD_SKIP_ROM        0xCC
#define ONEWIRE_CMD_MATCH_ROM       0x55
#define ONEWIRE_CMD_CONVERT_T       0x44
#define ONEWIRE_CMD_READ_SCRATCHPAD 0xBE

/* DS18B20 timing (microseconds) */
#define ONEWIRE_RESET_DELAY_US      480
#define ONEWIRE_PRESENCE_DELAY_US   70
#define ONEWIRE_WRITE_1_DELAY_US    10
#define ONEWIRE_WRITE_0_DELAY_US    60
#define ONEWIRE_READ_DELAY_US       15
#define ONEWIRE_SLOT_DELAY_US       65

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

WheelSpeed_t wheel_speed;
Temperature_t temperature;
Current_t current;
Pedal_t pedal;

/* Wheel pulse counters and timing */
static volatile uint32_t wheel_pulse_count[4] = {0};
static volatile uint32_t wheel_last_pulse_time[4] = {0};

/* Private function prototypes -----------------------------------------------*/
static void OneWire_WriteBit(uint8_t bit);
static uint8_t OneWire_ReadBit(void);
static void OneWire_DelayUs(uint32_t us);
static HAL_StatusTypeDef INA226_WriteReg(uint8_t i2c_addr, uint8_t reg, uint16_t value);
static uint16_t INA226_ReadReg(uint8_t i2c_addr, uint8_t reg);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief Initialize all sensors
  * @retval HAL status
  */
HAL_StatusTypeDef Sensors_Init(void)
{
  uint8_t i;
  
  /* Initialize wheel speed structure */
  memset(&wheel_speed, 0, sizeof(WheelSpeed_t));
  memset(&temperature, 0, sizeof(Temperature_t));
  memset(&current, 0, sizeof(Current_t));
  memset(&pedal, 0, sizeof(Pedal_t));
  
  /* Initialize ADC */
  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    return HAL_ERROR;
  }
  
  /* Initialize INA226 current sensors */
  uint8_t ina226_addrs[] = {INA226_ADDR_FL, INA226_ADDR_FR, INA226_ADDR_RL, 
                            INA226_ADDR_RR, INA226_ADDR_STEER, INA226_ADDR_BATT};
  
  for (i = 0; i < 6; i++) {
    /* Select multiplexer channel */
    if (TCA9548A_SelectChannel(i) != HAL_OK) {
      return HAL_ERROR;
    }
    
    /* Initialize INA226 */
    if (INA226_Init(ina226_addrs[i]) != HAL_OK) {
      return HAL_ERROR;
    }
  }
  
  return HAL_OK;
}

/**
  * @brief Read all wheel speeds
  * @param speeds: Pointer to WheelSpeed_t structure
  * @retval None
  */
void Sensors_ReadWheelSpeeds(WheelSpeed_t *speeds)
{
  uint32_t current_time = HAL_GetTick();
  uint32_t delta_time;
  uint8_t i;
  
  for (i = 0; i < 4; i++) {
    /* Calculate time since last pulse */
    delta_time = current_time - wheel_last_pulse_time[i];
    
    /* If no pulse for >500ms, speed is zero */
    if (delta_time > 500) {
      speeds->pulse_count[i] = 0;
      speeds->last_pulse_time[i] = current_time;
      
      switch (i) {
        case 0: speeds->speed_FL = 0; break;
        case 1: speeds->speed_FR = 0; break;
        case 2: speeds->speed_RL = 0; break;
        case 3: speeds->speed_RR = 0; break;
      }
    } else {
      /* Calculate speed from pulse frequency */
      uint32_t pulse_count_diff = wheel_pulse_count[i] - speeds->pulse_count[i];
      speeds->pulse_count[i] = wheel_pulse_count[i];
      speeds->last_pulse_time[i] = wheel_last_pulse_time[i];
      
      if (pulse_count_diff > 0) {
        uint16_t speed = Sensors_CalculateWheelSpeed(pulse_count_diff, delta_time);
        
        switch (i) {
          case 0: speeds->speed_FL = speed; break;
          case 1: speeds->speed_FR = speed; break;
          case 2: speeds->speed_RL = speed; break;
          case 3: speeds->speed_RR = speed; break;
        }
      }
    }
  }
  
  /* Calculate average speed */
  speeds->speed_avg = (speeds->speed_FL + speeds->speed_FR + 
                      speeds->speed_RL + speeds->speed_RR) / 4;
}

/**
  * @brief Read all temperatures (DS18B20)
  * @param temps: Pointer to Temperature_t structure
  * @retval HAL status
  */
HAL_StatusTypeDef Sensors_ReadTemperatures(Temperature_t *temps)
{
  /* Note: DS18B20 requires 750ms conversion time */
  /* This implementation uses simplified reading for demonstration */
  
  /* Start temperature conversion on all sensors */
  if (!OneWire_Reset()) {
    return HAL_ERROR;
  }
  
  OneWire_WriteByte(ONEWIRE_CMD_SKIP_ROM);
  OneWire_WriteByte(ONEWIRE_CMD_CONVERT_T);
  
  /* Wait for conversion (750ms for 12-bit resolution) */
  HAL_Delay(750);
  
  /* Read each sensor individually */
  temps->temp_FL = DS18B20_ReadTemperature(DS18B20_ROM_FL);
  temps->temp_FR = DS18B20_ReadTemperature(DS18B20_ROM_FR);
  temps->temp_RL = DS18B20_ReadTemperature(DS18B20_ROM_RL);
  temps->temp_RR = DS18B20_ReadTemperature(DS18B20_ROM_RR);
  temps->temp_ambient = DS18B20_ReadTemperature(DS18B20_ROM_AMB);
  
  /* Find maximum temperature */
  temps->temp_max = temps->temp_FL;
  if (temps->temp_FR > temps->temp_max) temps->temp_max = temps->temp_FR;
  if (temps->temp_RL > temps->temp_max) temps->temp_max = temps->temp_RL;
  if (temps->temp_RR > temps->temp_max) temps->temp_max = temps->temp_RR;
  
  return HAL_OK;
}

/**
  * @brief Read all currents (INA226)
  * @param currents: Pointer to Current_t structure
  * @retval HAL status
  */
HAL_StatusTypeDef Sensors_ReadCurrents(Current_t *currents)
{
  /* Read motor FL */
  if (TCA9548A_SelectChannel(0) == HAL_OK) {
    currents->current_FL = INA226_ReadCurrent(INA226_ADDR_FL);
  }
  
  /* Read motor FR */
  if (TCA9548A_SelectChannel(1) == HAL_OK) {
    currents->current_FR = INA226_ReadCurrent(INA226_ADDR_FR);
  }
  
  /* Read motor RL */
  if (TCA9548A_SelectChannel(2) == HAL_OK) {
    currents->current_RL = INA226_ReadCurrent(INA226_ADDR_RL);
  }
  
  /* Read motor RR */
  if (TCA9548A_SelectChannel(3) == HAL_OK) {
    currents->current_RR = INA226_ReadCurrent(INA226_ADDR_RR);
  }
  
  /* Read steering motor */
  if (TCA9548A_SelectChannel(4) == HAL_OK) {
    currents->current_STEER = INA226_ReadCurrent(INA226_ADDR_STEER);
  }
  
  /* Read battery */
  if (TCA9548A_SelectChannel(5) == HAL_OK) {
    currents->current_BATT = INA226_ReadCurrent(INA226_ADDR_BATT);
    currents->voltage_BATT = INA226_ReadVoltage(INA226_ADDR_BATT);
  }
  
  return HAL_OK;
}

/**
  * @brief Read pedal analog value
  * @param pedal: Pointer to Pedal_t structure
  * @retval HAL status
  */
HAL_StatusTypeDef Sensors_ReadPedal(Pedal_t *pedal)
{
  /* Start ADC conversion */
  HAL_ADC_Start(&hadc1);
  
  /* Wait for conversion */
  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
    return HAL_ERROR;
  }
  
  /* Read ADC value */
  pedal->adc_raw = HAL_ADC_GetValue(&hadc1);
  
  /* Convert to voltage */
  pedal->voltage = (float)pedal->adc_raw * ADC_VREF / (float)ADC_RESOLUTION;
  
  /* Convert to throttle percentage (0-100%) */
  pedal->throttle_pct = (uint8_t)((pedal->adc_raw * 100) / ADC_RESOLUTION);
  
  return HAL_OK;
}

/**
  * @brief Read shifter position
  * @retval ShifterState_t (FORWARD, NEUTRAL, REVERSE, ERROR)
  */
ShifterState_t Sensors_ReadShifter(void)
{
  uint8_t fwd = HAL_GPIO_ReadPin(SHIFTER_FWD_GPIO_Port, SHIFTER_FWD_Pin);
  uint8_t neu = HAL_GPIO_ReadPin(SHIFTER_NEU_GPIO_Port, SHIFTER_NEU_Pin);
  uint8_t rev = HAL_GPIO_ReadPin(SHIFTER_REV_GPIO_Port, SHIFTER_REV_Pin);
  
  /* Check for exactly one active signal (active HIGH) */
  uint8_t active_count = (fwd == GPIO_PIN_SET ? 1 : 0) + 
                         (neu == GPIO_PIN_SET ? 1 : 0) + 
                         (rev == GPIO_PIN_SET ? 1 : 0);
  
  if (active_count != 1) {
    return SHIFTER_ERROR;
  }
  
  if (fwd == GPIO_PIN_SET) return SHIFTER_FORWARD;
  if (neu == GPIO_PIN_SET) return SHIFTER_NEUTRAL;
  if (rev == GPIO_PIN_SET) return SHIFTER_REVERSE;
  
  return SHIFTER_ERROR;
}

/**
  * @brief Wheel sensor interrupt handler (EXTI)
  * @param wheel_index: Wheel index (0=FL, 1=FR, 2=RL, 3=RR)
  * @retval None
  */
void Sensors_WheelPulseCallback(uint8_t wheel_index)
{
  if (wheel_index < 4) {
    wheel_pulse_count[wheel_index]++;
    wheel_last_pulse_time[wheel_index] = HAL_GetTick();
  }
}

/**
  * @brief Calculate wheel speed from pulse frequency
  * @param pulse_count: Number of pulses
  * @param time_ms: Time period (ms)
  * @retval Speed in mm/s
  */
uint16_t Sensors_CalculateWheelSpeed(uint32_t pulse_count, uint32_t time_ms)
{
  float frequency_hz;
  float speed_mm_s;
  
  if (time_ms == 0) return 0;
  
  /* Calculate pulse frequency (Hz) */
  frequency_hz = (float)pulse_count * 1000.0f / (float)time_ms;
  
  /* Calculate linear speed (mm/s) */
  /* speed = (frequency / pulses_per_rev) * circumference */
  speed_mm_s = (frequency_hz / WHEEL_PPR) * WHEEL_CIRCUMFERENCE_MM;
  
  return (uint16_t)speed_mm_s;
}

/* OneWire functions ---------------------------------------------------------*/

/**
  * @brief OneWire reset pulse
  * @retval 1 if device present, 0 if no device
  */
uint8_t OneWire_Reset(void)
{
  uint8_t presence;
  
  /* Configure as output */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = TEMP_ONEWIRE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TEMP_ONEWIRE_GPIO_Port, &GPIO_InitStruct);
  
  /* Pull low for 480us */
  HAL_GPIO_WritePin(TEMP_ONEWIRE_GPIO_Port, TEMP_ONEWIRE_Pin, GPIO_PIN_RESET);
  OneWire_DelayUs(ONEWIRE_RESET_DELAY_US);
  
  /* Release and wait 70us */
  HAL_GPIO_WritePin(TEMP_ONEWIRE_GPIO_Port, TEMP_ONEWIRE_Pin, GPIO_PIN_SET);
  OneWire_DelayUs(ONEWIRE_PRESENCE_DELAY_US);
  
  /* Read presence pulse */
  presence = !HAL_GPIO_ReadPin(TEMP_ONEWIRE_GPIO_Port, TEMP_ONEWIRE_Pin);
  
  /* Wait for presence pulse to finish */
  OneWire_DelayUs(ONEWIRE_RESET_DELAY_US - ONEWIRE_PRESENCE_DELAY_US);
  
  return presence;
}

/**
  * @brief Write byte to OneWire bus
  * @param byte: Byte to write
  * @retval None
  */
void OneWire_WriteByte(uint8_t byte)
{
  uint8_t i;
  
  for (i = 0; i < 8; i++) {
    OneWire_WriteBit((byte >> i) & 0x01);
  }
}

/**
  * @brief Read byte from OneWire bus
  * @retval Read byte
  */
uint8_t OneWire_ReadByte(void)
{
  uint8_t byte = 0;
  uint8_t i;
  
  for (i = 0; i < 8; i++) {
    byte |= (OneWire_ReadBit() << i);
  }
  
  return byte;
}

/**
  * @brief Read temperature from DS18B20 with ROM addressing
  * @param rom_address: 64-bit ROM address
  * @retval Temperature in °C
  */
float DS18B20_ReadTemperature(uint64_t rom_address)
{
  uint8_t scratchpad[9];
  int16_t temp_raw;
  float temperature;
  uint8_t i;
  
  /* Reset and select device */
  if (!OneWire_Reset()) {
    return -127.0f; // Error value
  }
  
  /* Match ROM */
  OneWire_WriteByte(ONEWIRE_CMD_MATCH_ROM);
  for (i = 0; i < 8; i++) {
    OneWire_WriteByte((uint8_t)((rom_address >> (i * 8)) & 0xFF));
  }
  
  /* Read scratchpad */
  OneWire_WriteByte(ONEWIRE_CMD_READ_SCRATCHPAD);
  for (i = 0; i < 9; i++) {
    scratchpad[i] = OneWire_ReadByte();
  }
  
  /* Convert temperature */
  temp_raw = (scratchpad[1] << 8) | scratchpad[0];
  temperature = (float)temp_raw / 16.0f;
  
  return temperature;
}

/* I2C functions -------------------------------------------------------------*/

/**
  * @brief Select TCA9548A multiplexer channel
  * @param channel: Channel number (0-7)
  * @retval HAL status
  */
HAL_StatusTypeDef TCA9548A_SelectChannel(uint8_t channel)
{
  uint8_t control_byte;
  
  if (channel > 7) return HAL_ERROR;
  
  /* Set channel bit */
  control_byte = (1 << channel);
  
  /* Write to TCA9548A */
  return HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDR << 1, &control_byte, 1, 100);
}

/**
  * @brief Initialize INA226 sensor
  * @param i2c_addr: I2C address (7-bit)
  * @retval HAL status
  */
HAL_StatusTypeDef INA226_Init(uint8_t i2c_addr)
{
  /* Configure INA226 */
  if (INA226_WriteReg(i2c_addr, INA226_REG_CONFIG, INA226_CONFIG_VALUE) != HAL_OK) {
    return HAL_ERROR;
  }
  
  /* Set calibration */
  if (INA226_WriteReg(i2c_addr, INA226_REG_CALIBRATION, INA226_CAL_VALUE) != HAL_OK) {
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

/**
  * @brief Read current from INA226
  * @param i2c_addr: I2C address (7-bit)
  * @retval Current in Amperes
  */
float INA226_ReadCurrent(uint8_t i2c_addr)
{
  int16_t current_raw;
  float current;
  
  current_raw = (int16_t)INA226_ReadReg(i2c_addr, INA226_REG_CURRENT);
  
  /* Convert to Amperes (1 LSB = 1mA with current calibration) */
  current = (float)current_raw / 1000.0f;
  
  return current;
}

/**
  * @brief Read voltage from INA226
  * @param i2c_addr: I2C address (7-bit)
  * @retval Voltage in Volts
  */
float INA226_ReadVoltage(uint8_t i2c_addr)
{
  uint16_t voltage_raw;
  float voltage;
  
  voltage_raw = INA226_ReadReg(i2c_addr, INA226_REG_BUS_V);
  
  /* Convert to Volts (1 LSB = 1.25mV) */
  voltage = (float)voltage_raw * 0.00125f;
  
  return voltage;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Write bit to OneWire bus
  * @param bit: Bit value (0 or 1)
  * @retval None
  */
static void OneWire_WriteBit(uint8_t bit)
{
  HAL_GPIO_WritePin(TEMP_ONEWIRE_GPIO_Port, TEMP_ONEWIRE_Pin, GPIO_PIN_RESET);
  
  if (bit) {
    OneWire_DelayUs(ONEWIRE_WRITE_1_DELAY_US);
    HAL_GPIO_WritePin(TEMP_ONEWIRE_GPIO_Port, TEMP_ONEWIRE_Pin, GPIO_PIN_SET);
    OneWire_DelayUs(ONEWIRE_SLOT_DELAY_US - ONEWIRE_WRITE_1_DELAY_US);
  } else {
    OneWire_DelayUs(ONEWIRE_WRITE_0_DELAY_US);
    HAL_GPIO_WritePin(TEMP_ONEWIRE_GPIO_Port, TEMP_ONEWIRE_Pin, GPIO_PIN_SET);
    OneWire_DelayUs(ONEWIRE_SLOT_DELAY_US - ONEWIRE_WRITE_0_DELAY_US);
  }
}

/**
  * @brief Read bit from OneWire bus
  * @retval Bit value (0 or 1)
  */
static uint8_t OneWire_ReadBit(void)
{
  uint8_t bit;
  
  HAL_GPIO_WritePin(TEMP_ONEWIRE_GPIO_Port, TEMP_ONEWIRE_Pin, GPIO_PIN_RESET);
  OneWire_DelayUs(ONEWIRE_READ_DELAY_US);
  HAL_GPIO_WritePin(TEMP_ONEWIRE_GPIO_Port, TEMP_ONEWIRE_Pin, GPIO_PIN_SET);
  OneWire_DelayUs(ONEWIRE_READ_DELAY_US);
  
  bit = HAL_GPIO_ReadPin(TEMP_ONEWIRE_GPIO_Port, TEMP_ONEWIRE_Pin);
  
  OneWire_DelayUs(ONEWIRE_SLOT_DELAY_US - 2 * ONEWIRE_READ_DELAY_US);
  
  return bit;
}

/**
  * @brief Microsecond delay for OneWire timing
  * @param us: Delay in microseconds
  * @retval None
  */
static void OneWire_DelayUs(uint32_t us)
{
  /* Simple busy-wait delay (assumes 170 MHz CPU clock) */
  uint32_t cycles = us * 170 / 4;
  while (cycles--) {
    __NOP();
  }
}

/**
  * @brief Write register to INA226
  * @param i2c_addr: I2C address (7-bit)
  * @param reg: Register address
  * @param value: 16-bit value to write
  * @retval HAL status
  */
static HAL_StatusTypeDef INA226_WriteReg(uint8_t i2c_addr, uint8_t reg, uint16_t value)
{
  uint8_t data[3];
  
  data[0] = reg;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value & 0xFF);
  
  return HAL_I2C_Master_Transmit(&hi2c1, i2c_addr << 1, data, 3, 100);
}

/**
  * @brief Read register from INA226
  * @param i2c_addr: I2C address (7-bit)
  * @param reg: Register address
  * @retval 16-bit register value
  */
static uint16_t INA226_ReadReg(uint8_t i2c_addr, uint8_t reg)
{
  uint8_t data[2];
  
  /* Write register address */
  HAL_I2C_Master_Transmit(&hi2c1, i2c_addr << 1, &reg, 1, 100);
  
  /* Read 2 bytes */
  HAL_I2C_Master_Receive(&hi2c1, i2c_addr << 1, data, 2, 100);
  
  return (uint16_t)((data[0] << 8) | data[1]);
}
