#include <stm32f4xx_hal.h>

// Global variables
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1, htim2, htim8;
ADC_HandleTypeDef hadc1;
IWDG_HandleTypeDef hiwdg;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);

void CAN_SendHeartbeat(void);
void Pedal_Update(void);
void Current_ReadAll(void);
void ABS_Check(void);
void TCS_Check(void);
void Steering_ControlLoop(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_FDCAN1_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM8_Init();
    MX_ADC1_Init();
    MX_IWDG_Init();

    while (1) {
        HAL_Delay(100);
        CAN_SendHeartbeat();

        HAL_Delay(50);
        Pedal_Update();
        Current_ReadAll();

        HAL_Delay(10);
        ABS_Check();
        TCS_Check();
        Steering_ControlLoop();
    }
}

static void MX_FDCAN1_Init(void) {
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.BitRate = 500000;
    hfdcan1.Init.Prescaler = 17;
    HAL_FDCAN_Init(&hfdcan1);
}

static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.Timing = 0x10909CEC;
    HAL_I2C_Init(&hi2c1);
}

static void MX_TIM1_Init(void) {
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 8499;
    htim1.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&htim1);
}

static void MX_TIM2_Init(void) {
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    HAL_TIM_Encoder_Init(&htim2);
}

static void MX_TIM8_Init(void) {
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 8499;
    HAL_TIM_PWM_Init(&htim8);
}

static void MX_ADC1_Init(void) {
    hadc1.Instance = ADC1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    HAL_ADC_Init(&hadc1);
}

static void MX_IWDG_Init(void) {
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Reload = 4095;
    HAL_IWDG_Init(&hiwdg);
}

void Error_Handler(void) {
    while (1) { }
}