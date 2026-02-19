/**
  ****************************************************************************
  * @file    main.c
  * @brief   STM32G474RE vehicle control – main entry point
  *
  *  Peripherals initialised:
  *    ADC1    – Pedal accelerator primary channel (PA3 via voltage divider)
  *    FDCAN1  – CAN bus @ 500 kbps (ESP32-S3 link)
  *    I2C1    – INA226 / TCA9548A sensors + ADS1115 pedal plausibility
  *    TIM1    – PWM for 4 traction motors (20 kHz)
  *    TIM2    – Quadrature encoder (steering)
  *    TIM8    – PWM for steering motor (20 kHz)
  *    IWDG    – Independent watchdog (500 ms)
  ****************************************************************************
  */

#include "main.h"
#include "motor_control.h"
#include "can_handler.h"
#include "sensor_manager.h"
#include "safety_system.h"
#include "steering_centering.h"
#include "service_mode.h"
#include "boot_validation.h"
#include "encoder_reader.h"
#include "math_safety.h"
#include <math.h>

/* ---- HAL handle instances ---- */
ADC_HandleTypeDef   hadc1;
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef   hi2c1;
TIM_HandleTypeDef   htim1, htim2, htim8;
IWDG_HandleTypeDef  hiwdg;

/* ---- Reset cause (read once at boot, before IWDG clears flags) ---- */
static uint8_t reset_cause = 0;
#define RESET_CAUSE_POWERON   (1U << 0)
#define RESET_CAUSE_SOFTWARE  (1U << 1)
#define RESET_CAUSE_IWDG      (1U << 2)
#define RESET_CAUSE_WWDG      (1U << 3)
#define RESET_CAUSE_BROWNOUT   (1U << 4)
#define RESET_CAUSE_PIN        (1U << 5)

/**
 * @brief  Read RCC_CSR reset flags and clear them.
 *         Must be called before MX_IWDG_Init() (IWDG start clears some flags).
 */
static void Boot_ReadResetCause(void)
{
    uint32_t csr = RCC->CSR;

    reset_cause = 0;
    /* LPWRSTF and BORRSTF are both mapped to BROWNOUT intentionally:
     * both indicate power-supply issues and require the same diagnostic
     * response.  Separate flags would not change the recovery action.   */
    if (csr & RCC_CSR_LPWRRSTF)  reset_cause |= RESET_CAUSE_BROWNOUT;
    if (csr & RCC_CSR_WWDGRSTF)  reset_cause |= RESET_CAUSE_WWDG;
    if (csr & RCC_CSR_IWDGRSTF)  reset_cause |= RESET_CAUSE_IWDG;
    if (csr & RCC_CSR_SFTRSTF)   reset_cause |= RESET_CAUSE_SOFTWARE;
    if (csr & RCC_CSR_BORRSTF)   reset_cause |= RESET_CAUSE_BROWNOUT;
    if (csr & RCC_CSR_PINRSTF)   reset_cause |= RESET_CAUSE_PIN;

    /* If only PIN reset flag is set → power-on reset */
    if (reset_cause == RESET_CAUSE_PIN)
        reset_cause = RESET_CAUSE_POWERON;

    /* Clear all reset flags */
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

uint8_t Boot_GetResetCause(void) { return reset_cause; }

/* ---- Peripheral init status flags ---- */
bool fdcan_init_ok = false;
bool i2c_init_ok   = false;

/* ---- Private prototypes ---- */
void SystemClock_Config(void);
static void MX_ADC1_Init(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_IWDG_Init(void);

/* ================================================================== */

int main(void)
{
    HAL_Init();

    /* Read reset cause before IWDG start (which clears some flags) */
    Boot_ReadResetCause();

    SystemClock_Config();

    /* Peripheral initialisation */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_FDCAN1_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM8_Init();
    MX_IWDG_Init();

    /* Module initialisation */
    Motor_Init();
    Traction_Init();
    Steering_Init();
    Sensor_Init();
    Safety_Init();
    ServiceMode_Init();
    CAN_Init();
    SteeringCentering_Init();

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
            CAN_CheckBusOff();
            Safety_CheckSensors();
            Safety_CheckEncoder();

            /* Boot validation checklist — run during STANDBY to
             * evaluate sensor plausibility before allowing ACTIVE.
             * Non-blocking; results queried by Safety_CheckCANTimeout(). */
            if (Safety_GetState() == SYS_STATE_STANDBY) {
                BootValidation_Run();
            }

            /* Obstacle safety — STM32 primary safety controller.
             * CAN obstacle data from ESP32 is advisory only.
             * Local state machine with plausibility validation,
             * stuck-sensor detection, speed-dependent thresholds,
             * and temporal hysteresis.  CAN loss → scale 1.0
             * (LIMP_HOME speed cap provides safety net).
             * Reverse escape is allowed when forward is blocked.   */
            Obstacle_Update();

            /* Non-blocking relay sequencer — progresses the power-up
             * sequence (Main → Traction → Direction) using timestamps
             * instead of blocking HAL_Delay calls.                     */
            Relay_SequencerUpdate();

            /* Run automatic centering during BOOT / STANDBY.
             * Once complete, Steering_ControlLoop() takes over. */
            if (!SteeringCentering_IsComplete() &&
                !SteeringCentering_HasFault()) {
                SystemState_t st = Safety_GetState();
                if (st == SYS_STATE_BOOT || st == SYS_STATE_STANDBY) {
                    SteeringCentering_Step();
                }
            }

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

            /* Feed pedal demand into traction.
             *
             * Three modes of operation:
             * 1. ACTIVE/DEGRADED: CAN commands accepted, pedal validated
             *    through Safety_ValidateThrottle() pipeline.
             * 2. LIMP_HOME: Local pedal only, strong clamp (20% torque),
             *    CAN throttle commands ignored.  Vehicle remains mobile
             *    at walking speed without CAN/ESP32.
             * 3. All other states: throttle suppressed.
             *
             * In Park or Neutral gear, throttle is always suppressed.    */
            if (Safety_IsCommandAllowed()) {
                GearPosition_t gear = Traction_GetGear();
                if (gear == GEAR_PARK || gear == GEAR_NEUTRAL) {
                    Traction_SetDemand(0.0f);
                } else {
                    float validated = Safety_ValidateThrottle(Pedal_GetPercent());
                    Traction_SetDemand(validated);
                }
            } else if (Safety_IsLimpHome()) {
                /* LIMP_HOME: local pedal drives traction directly.
                 * Ignore all CAN throttle commands.
                 * Apply LIMP_HOME torque limit (20%) as hard clamp.
                 * The traction pipeline applies additional speed cap
                 * and ramp limiting via Safety_GetTractionCapFactor(). */
                GearPosition_t gear = Traction_GetGear();
                if (gear == GEAR_PARK || gear == GEAR_NEUTRAL) {
                    Traction_SetDemand(0.0f);
                } else {
                    float pedal = Pedal_GetPercent();
                    /* Hard clamp: 20% max torque in LIMP_HOME.
                     * pedal is 0–100%, factor is 0.20 → max demand = 20%. */
                    float clamped = pedal * LIMP_HOME_TORQUE_LIMIT_FACTOR;
                    if (clamped < 0.0f)  clamped = 0.0f;
                    if (clamped > 100.0f * LIMP_HOME_TORQUE_LIMIT_FACTOR)
                        clamped = 100.0f * LIMP_HOME_TORQUE_LIMIT_FACTOR;
                    Traction_SetDemand(clamped);
                }
            } else {
                Traction_SetDemand(0.0f);
            }
        }

        /* ---- 100 ms tasks (10 Hz): CAN heartbeat + status ---- */
        if ((now - tick_100ms) >= 100) {
            tick_100ms = now;

            Safety_CheckBatteryVoltage();

            CAN_SendHeartbeat();
            CAN_SendStatusSpeed(
                float_to_u16_clamped(Wheel_GetSpeed_FL() * 10),
                float_to_u16_clamped(Wheel_GetSpeed_FR() * 10),
                float_to_u16_clamped(Wheel_GetSpeed_RL() * 10),
                float_to_u16_clamped(Wheel_GetSpeed_RR() * 10));
            CAN_SendStatusCurrent(
                float_to_u16_clamped(Current_GetAmps(0) * 100),
                float_to_u16_clamped(Current_GetAmps(1) * 100),
                float_to_u16_clamped(Current_GetAmps(2) * 100),
                float_to_u16_clamped(Current_GetAmps(3) * 100));
            CAN_SendStatusSafety(
                ABS_IsActive(), TCS_IsActive(),
                (uint8_t)Safety_GetError());
            CAN_SendStatusSteering(
                (int16_t)(Steering_GetCurrentAngle() * 10),
                Steering_IsCalibrated());
            CAN_SendStatusTraction();
            CAN_SendStatusBattery();
        }

        /* ---- 1000 ms tasks (1 Hz): temperatures + service status ---- */
        if ((now - tick_1000ms) >= 1000) {
            tick_1000ms = now;

            CAN_SendStatusTemp(
                (int8_t)Temperature_Get(0),
                (int8_t)Temperature_Get(1),
                (int8_t)Temperature_Get(2),
                (int8_t)Temperature_Get(3),
                (int8_t)Temperature_Get(4));
            CAN_SendStatusTempMap();

            /* Service mode: send module fault/enable/disable bitmasks
             * to ESP32 for the diagnostic/service menu.               */
            CAN_SendServiceStatus();

            /* Encoder diagnostic: raw count + delta for hardware validation.
             * Diagnostic only — not used by any control path.             */
            Encoder_SendDiagnostic();
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

    /* Steering center inductive sensor (PB5 / EXTI5) - same
     * configuration as the wheel speed sensors (rising-edge trigger). */
    gpio.Pin  = PIN_STEER_CENTER;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* EXTI IRQs */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);   /* PB5 center sensor */
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
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
        fdcan_init_ok = false;
        return;  /* Non-fatal: system continues without CAN */
    }
    fdcan_init_ok = true;
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
        i2c_init_ok = false;
        return;  /* Non-fatal: sensors unavailable, system continues */
    }
    i2c_init_ok = true;
}

static void MX_TIM1_Init(void)
{
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period            = 4249;   /* Center-aligned: 170 MHz / (2 × 4250) = 20 kHz */
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
    oc.OCPreload  = TIM_OCPRELOAD_ENABLE;  /* Buffer CCR — update at period boundary
                                            * only, preventing mid-cycle duty changes
                                            * that cause asymmetric pulses in
                                            * center-aligned mode.                   */
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
    htim2.Init.Period            = 0xFFFFFFFF; /* TIM2 is 32-bit — use full range to
                                               * prevent counter wrap at ±350° travel
                                               * (±4667 counts at 4800 CPR).          */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef enc = {0};
    enc.EncoderMode  = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter    = 6;  /* Digital filter: 6 × fDTS rejects noise from
                            * NPN open-collector outputs (E6B2-CWZ6C).
                            * At 170 MHz ≈ 35 ns per sample → ~210 ns
                            * glitch rejection.  Sufficient for 1200 PPR
                            * at typical steering rates.                */
    enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter    = 6;  /* Same digital filter as IC1 for symmetry   */
    if (HAL_TIM_Encoder_Init(&htim2, &enc) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM8_Init(void)
{
    htim8.Instance               = TIM8;
    htim8.Init.Prescaler         = 0;
    htim8.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
    htim8.Init.Period            = 4249;   /* Center-aligned: 170 MHz / (2 × 4250) = 20 kHz */
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
    oc.OCPreload  = TIM_OCPRELOAD_ENABLE;  /* Buffer CCR — same as TIM1 */
    HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_3);
}

static void MX_ADC1_Init(void)
{
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4; /* 170/4 = 42.5 MHz */
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;  /* Single channel */
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;  /* Single-shot per Pedal_Update() */
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode      = DISABLE;
    hadc1.Init.GainCompensation      = 0;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /* Calibrate ADC for single-ended mode (must be done before first conversion) */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    /* Configure channel: PA3 = ADC1_IN4, single-ended */
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = ADC_CHANNEL_4;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5; /* ~1.1 µs at 42.5 MHz */
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
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