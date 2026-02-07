/**
  ****************************************************************************
  * @file    main.c
  * @brief   STM32G474RE vehicle control – main entry point
  *
  *  Peripherals initialised:
  *    FDCAN1  – CAN bus @ 500 kbps (ESP32-S3 link)
  *    I2C1    – INA226 / TCA9548A sensors
  *    TIM1    – PWM for 4 traction motors (20 kHz)
  *    TIM2    – Quadrature encoder (steering)
  *    TIM8    – PWM for steering motor (20 kHz)
  *    ADC1    – Pedal analogue input
  *    IWDG    – Independent watchdog (500 ms)
  ****************************************************************************
  */

#include "main.h"
#include "motor_control.h"
#include "can_handler.h"
#include "sensor_manager.h"
#include "safety_system.h"

/* ---- HAL handle instances ---- */
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef   hi2c1;
TIM_HandleTypeDef   htim1, htim2, htim8;
ADC_HandleTypeDef   hadc1;
IWDG_HandleTypeDef  hiwdg;

/* ---- Private prototypes ---- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);

/* ================================================================== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* Peripheral initialisation */
    MX_GPIO_Init();
    MX_FDCAN1_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM8_Init();
    MX_ADC1_Init();
    MX_IWDG_Init();

    /* Module initialisation */
    Motor_Init();
    Traction_Init();
    Steering_Init();
    Sensor_Init();
    Safety_Init();
    CAN_Init();

    /* Transition: BOOT → STANDBY (peripherals ready, waiting for ESP32) */
    Safety_SetState(SYS_STATE_STANDBY);

    /* Timing counters */
    uint32_t tick_10ms   = 0;
    uint32_t tick_50ms   = 0;
    uint32_t tick_100ms  = 0;
    uint32_t tick_1000ms = 0;

    /* ---- Main control loop ---- */
    while (1) {
        uint32_t now = HAL_GetTick();

        /* ---- 10 ms tasks (100 Hz): safety + steering PID ---- */
        if ((now - tick_10ms) >= 10) {
            tick_10ms = now;

            ABS_Update();
            TCS_Update();
            Safety_CheckCurrent();
            Safety_CheckTemperature();
            Safety_CheckCANTimeout();
            Safety_CheckSensors();
            Steering_ControlLoop();
            Traction_Update();
        }

        /* ---- 50 ms tasks (20 Hz): sensors + pedal ---- */
        if ((now - tick_50ms) >= 50) {
            tick_50ms = now;

            Pedal_Update();
            Current_ReadAll();
            Temperature_StartConversion();
            Temperature_ReadAll();

            /* Feed pedal demand into traction only when STM32 is in
             * ACTIVE state — the safety authority decides whether
             * actuator commands are permitted.                          */
            if (Safety_IsCommandAllowed()) {
                float validated = Safety_ValidateThrottle(Pedal_GetPercent());
                Traction_SetDemand(validated);
            } else {
                Traction_SetDemand(0.0f);
            }
        }

        /* ---- 100 ms tasks (10 Hz): CAN heartbeat + status ---- */
        if ((now - tick_100ms) >= 100) {
            tick_100ms = now;

            CAN_SendHeartbeat();
            CAN_SendStatusSpeed(
                (uint16_t)(Wheel_GetSpeed_FL() * 10),
                (uint16_t)(Wheel_GetSpeed_FR() * 10),
                (uint16_t)(Wheel_GetSpeed_RL() * 10),
                (uint16_t)(Wheel_GetSpeed_RR() * 10));
            CAN_SendStatusCurrent(
                (uint16_t)(Current_GetAmps(0) * 100),
                (uint16_t)(Current_GetAmps(1) * 100),
                (uint16_t)(Current_GetAmps(2) * 100),
                (uint16_t)(Current_GetAmps(3) * 100));
            CAN_SendStatusSafety(
                ABS_IsActive(), TCS_IsActive(),
                (uint8_t)Safety_GetError());
            CAN_SendStatusSteering(
                (int16_t)(Steering_GetCurrentAngle() * 10),
                Steering_IsCalibrated());
        }

        /* ---- 1000 ms tasks (1 Hz): temperatures ---- */
        if ((now - tick_1000ms) >= 1000) {
            tick_1000ms = now;

            CAN_SendStatusTemp(
                (int8_t)Temperature_Get(0),
                (int8_t)Temperature_Get(1),
                (int8_t)Temperature_Get(2),
                (int8_t)Temperature_Get(3),
                (int8_t)Temperature_Get(4));
        }

        /* Process incoming CAN commands from ESP32 */
        CAN_ProcessMessages();

        /* Kick the watchdog */
        HAL_IWDG_Refresh(&hiwdg);
    }
}

/* ================================================================== */
/*  Peripheral Init (STM32CubeMX-generated stubs)                     */
/* ================================================================== */

void SystemClock_Config(void)
{
    /*
     * Clock tree (from .ioc):
     *   HSI 16 MHz → PLL: /4 (PLLM) × 85 (PLLN) = 340 MHz VCO
     *                      /2 (PLLR) = 170 MHz SYSCLK
     *   AHB  = 170 MHz (no prescaler)
     *   APB1 = 170 MHz (no prescaler)
     *   APB2 = 170 MHz (no prescaler)
     */
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the main internal regulator output voltage */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK) {
        Error_Handler();
    }

    /* Initialise HSI and PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV4;   /* 16 / 4 = 4 MHz */
    RCC_OscInitStruct.PLL.PLLN       = 85;               /* 4 × 85 = 340 MHz VCO */
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;    /* 340 / 2 = 170 MHz */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* Select PLL as system clock source and configure bus prescalers */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    /* 170 MHz requires 8 flash wait states (see RM0440 Table 9) */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* Direction outputs (GPIOC) */
    gpio.Pin   = PIN_DIR_FL | PIN_DIR_FR | PIN_DIR_RL | PIN_DIR_RR | PIN_DIR_STEER;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* Enable outputs (GPIOC) */
    gpio.Pin = PIN_EN_FL | PIN_EN_FR | PIN_EN_RL | PIN_EN_RR | PIN_EN_STEER;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* Relay outputs (GPIOC) */
    gpio.Pin = PIN_RELAY_MAIN | PIN_RELAY_TRAC | PIN_RELAY_DIR;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* Wheel speed EXTI inputs */
    gpio.Pin  = PIN_WHEEL_FL | PIN_WHEEL_FR | PIN_WHEEL_RL;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin  = PIN_WHEEL_RR;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* EXTI IRQs */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void MX_FDCAN1_Init(void)
{
    hfdcan1.Instance                  = FDCAN1;
    hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.NominalPrescaler     = 17;
    hfdcan1.Init.NominalSyncJumpWidth = 1;
    hfdcan1.Init.NominalTimeSeg1      = 14;
    hfdcan1.Init.NominalTimeSeg2      = 5;
    hfdcan1.Init.AutoRetransmission   = ENABLE;
    hfdcan1.Init.TransmitPause        = DISABLE;
    hfdcan1.Init.ProtocolException    = DISABLE;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x10909CEC;  /* 400 kHz Fast Mode @ 170 MHz */
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM1_Init(void)
{
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 8499;   /* 170 MHz / (0+1) / (8499+1) = 20 kHz */
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_4);
}

static void MX_TIM2_Init(void)
{
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 65535;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef enc = {0};
    enc.EncoderMode  = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter    = 0;
    enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter    = 0;
    if (HAL_TIM_Encoder_Init(&htim2, &enc) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM8_Init(void)
{
    htim8.Instance               = TIM8;
    htim8.Init.Prescaler         = 0;
    htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim8.Init.Period            = 8499;
    htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
        Error_Handler();
    }

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_3);
}

static void MX_ADC1_Init(void)
{
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = ADC_CHANNEL_4;    /* PA3 = ADC1_IN4 */
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &ch);
}

static void MX_IWDG_Init(void)
{
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Reload    = 4095;           /* ~500 ms at 32 kHz / 32 */
    hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    /* Safe the hardware: drive all GPIOC outputs LOW (relays off, motors disabled).
     * Uses direct register access because HAL may be in an inconsistent state.    */
    GPIOC->BSRR = (uint32_t)(PIN_EN_FL | PIN_EN_FR | PIN_EN_RL | PIN_EN_RR | PIN_EN_STEER
                  | PIN_RELAY_MAIN | PIN_RELAY_TRAC | PIN_RELAY_DIR) << 16U;
    while (1) { }
}