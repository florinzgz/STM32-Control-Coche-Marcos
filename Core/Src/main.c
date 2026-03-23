/**
  ****************************************************************************
  * @file    main.c
  * @brief   STM32G474RE vehicle control – main entry point
  *
  *  Peripherals initialised:
  *    ADC1    – Pedal accelerator primary channel (PA3 via voltage divider)
  *    FDCAN1  – CAN bus @ 500 kbps (ESP32-S3 link)
  *    I2C1    – INA226 / TCA9548A sensors
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
#include "steering_cal_store.h"
#include "error_log.h"
#include <math.h>

/* ---- Compile-time CAN self-test option ----
 * Set to 1 to enable FDCAN1 internal loopback mode.  In loopback the
 * transceiver is bypassed: transmitted frames are routed directly back
 * to the Rx FIFO, allowing self-test without external bus hardware.
 * Set to 0 (default) for normal external bus operation.               */
#ifndef CAN_LOOPBACK_TEST
#define CAN_LOOPBACK_TEST  0
#endif

/* ---- HAL handle instances ---- */
ADC_HandleTypeDef   hadc1;
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef   hi2c1;
TIM_HandleTypeDef   htim1, htim2, htim3, htim8;
IWDG_HandleTypeDef  hiwdg;

/* ---- LIMP_HOME degraded-pedal arming constants ---- */
#define LIMP_PEDAL_REST_PCT   3.0f   /* Pedal % below which "at rest"  */
#define LIMP_PEDAL_ARM_MS     300U   /* Continuous rest time to arm     */

/* ---- Power-On Movement Prevention constants ----
 * Prevents unintended torque after any reboot (power loss, watchdog,
 * brownout, MCU reset) while accelerator pedal is pressed.
 * The inhibit clears only when the driver releases the pedal
 * (< 3 %) continuously for 400 ms.  Independent of LIMP_HOME latch
 * and pedal plausibility logic.  Applies in STANDBY, LIMP_HOME,
 * DEGRADED, and ACTIVE — SAFE already forces zero torque.           */
#define STARTUP_PEDAL_REST_PCT   3.0f   /* Pedal % below which "at rest"  */
#define STARTUP_PEDAL_CLEAR_MS   400U   /* Continuous rest time to clear   */

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

/* ---- Power-On Movement Prevention latch ----
 * Starts true; cleared only after pedal held below rest threshold
 * for STARTUP_PEDAL_CLEAR_MS.  Re-activates on every MCU reset
 * (the variable reinitialises to true because it is not in NVM).   */
static bool     startup_inhibit           = true;
static uint32_t startup_pedal_rest_since  = 0;

bool Startup_IsInhibited(void) { return startup_inhibit; }

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
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_IWDG_Init(void);

/* ================================================================== */

int main(void)
{
    HAL_Init();

    /* Read reset cause before IWDG start (which clears some flags) */
    Boot_ReadResetCause();

    SystemClock_Config();

    /* Lower SysTick priority so it cannot preempt time-critical FDCAN
     * reception (priority 1) or motor timer interrupts (priority 2).
     * HAL_Init() sets SysTick to priority 0 (highest); move it to 4
     * so the interrupt hierarchy is:
     *   0 = Cortex faults (HardFault, MemManage, BusFault, UsageFault)
     *   1 = FDCAN1_IT0  (CAN reception — most time-critical peripheral)
     *   2 = TIM1/TIM2/TIM8/EXTI  (motor PWM + encoder + wheel sensors)
     *   3 = I2C1  (sensor polling — tolerates latency)
     *   4 = SysTick  (1 ms tick — only needs to run between ISRs)
     * SysTick still preempts the main loop, so HAL_Delay() and
     * HAL_GetTick() work correctly in non-ISR context.                */
    HAL_NVIC_SetPriority(SysTick_IRQn, 4, 0);

    /* Peripheral initialisation */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_FDCAN1_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
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
    ErrorLog_Init();
    ErrorLog_SetResetCause(reset_cause);

    /* ---- Persistent steering calibration ----
     * Attempt to restore the last known center position from flash.
     * If the stored value is valid AND the center sensor currently
     * confirms the steering is at center, skip the centering sweep.
     * Otherwise, the normal centering sequence runs as usual.
     * Safety: flash alone NEVER authorises ACTIVE — the physical
     * center sensor must agree.                                      */
    SteeringCal_Init();
    if (SteeringCal_ValidateAtBoot()) {
        SteeringCentering_MarkRestoredFromFlash(
            SteeringCal_GetStoredCenter());
    }

    /* Transition: BOOT → STANDBY (peripherals ready, waiting for ESP32) */
    Safety_SetState(SYS_STATE_STANDBY);

    /* Timing counters */
    uint32_t tick_10ms   = 0;
    uint32_t tick_50ms   = 0;
    uint32_t tick_100ms  = 0;
    uint32_t tick_200ms  = 0;    /* LED heartbeat */
    uint32_t tick_500ms  = 0;    /* CAN test transmit */
    uint32_t tick_1000ms = 0;

    /* ---- LIMP_HOME degraded-pedal arming latch ----
     * Prevents unintended creep torque from ADC offset/noise when
     * entering LIMP_HOME with degraded (single-channel) pedal input.
     * Torque is only allowed after the driver confirms intent by
     * releasing the pedal (< 3 %) for at least 300 ms continuously.
     * Any noise spike above the threshold resets the timer.
     * The latch is reset on every entry to or exit from LIMP_HOME.   */
    bool     limp_home_pedal_armed = false;
    uint32_t limp_pedal_rest_since = 0;
    SystemState_t prev_state_for_arm = SYS_STATE_BOOT;

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
            Safety_CheckSteeringTimeout();
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

            /* ---- Power-On Movement Prevention latch update ----
             * While startup_inhibit is active, monitor pedal %.
             * Clear only when pedal is held below rest threshold
             * for STARTUP_PEDAL_CLEAR_MS continuously.
             * Any reading above the threshold resets the timer.
             * Once cleared, never re-activates until next MCU reset. */
            if (startup_inhibit) {
                if (Pedal_GetPercent() < STARTUP_PEDAL_REST_PCT) {
                    if (startup_pedal_rest_since == 0)
                        startup_pedal_rest_since = now;
                    else if ((now - startup_pedal_rest_since) >= STARTUP_PEDAL_CLEAR_MS)
                        startup_inhibit = false;
                } else {
                    startup_pedal_rest_since = 0;
                }
            }

            /* ---- LIMP_HOME degraded-pedal arming latch update ----
             * Reset on any transition into or out of LIMP_HOME.
             * While in LIMP_HOME: arm when pedal held below rest
             * threshold for LIMP_PEDAL_ARM_MS.  Any reading above
             * the threshold resets the timer (noise rejection).      */
            {
                SystemState_t cur_st = Safety_GetState();
                if (cur_st != prev_state_for_arm) {
                    if (cur_st == SYS_STATE_LIMP_HOME ||
                        prev_state_for_arm == SYS_STATE_LIMP_HOME) {
                        limp_home_pedal_armed = false;
                        limp_pedal_rest_since = 0;
                    }
                    prev_state_for_arm = cur_st;
                }
                if (cur_st == SYS_STATE_LIMP_HOME && !limp_home_pedal_armed) {
                    if (Pedal_GetPercent() < LIMP_PEDAL_REST_PCT) {
                        if (limp_pedal_rest_since == 0)
                            limp_pedal_rest_since = now;
                        else if ((now - limp_pedal_rest_since) >= LIMP_PEDAL_ARM_MS)
                            limp_home_pedal_armed = true;
                    } else {
                        limp_pedal_rest_since = 0;
                    }
                }
            }

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
             * Power-On Movement Prevention: while startup_inhibit is
             * active, force zero demand regardless of state.  SAFE
             * already forces zero torque independently.
             *
             * In Park or Neutral gear, throttle is always suppressed.    */
            if (startup_inhibit) {
                Traction_SetDemand(0.0f);
            } else if (Safety_IsCommandAllowed()) {
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
                 * and ramp limiting via Safety_GetTractionCapFactor().
                 *
                 * Safety invariant: contradictory pedal samples
                 * (dual ADC reads disagreeing) → zero demand.
                 * Pedal implausible (not contradictory) → primary
                 * ADC still used with LIMP_HOME torque clamp.          */
                GearPosition_t gear = Traction_GetGear();
                if (gear == GEAR_PARK || gear == GEAR_NEUTRAL ||
                    Pedal_IsContradictory()) {
                    Traction_SetDemand(0.0f);
                } else if (!Pedal_IsPlausible() && !limp_home_pedal_armed) {
                    /* Degraded pedal (plausibility fault): suppress torque until
                     * driver confirms intent by releasing pedal to rest.
                     * Prevents ADC offset/noise from causing creep torque
                     * when cross-validation is unavailable.               */
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
            Safety_CheckBatteryOvervoltage();

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

            /* Error log header: send entry count to ESP32 engineering menu */
            CAN_SendErrorLogHeader();

            /* LED/lights status: send current relay state to ESP32
             * for HMI synchronisation.                                */
            CAN_SendStatusLights();

            /* Encoder diagnostic: raw count + delta for hardware validation.
             * Diagnostic only — not used by any control path.             */
            Encoder_SendDiagnostic();

            /* DS18B20 hot-plug detection: re-enumerates the OneWire bus
             * every OW_RESCAN_INTERVAL_MS to detect sensors added or
             * removed at runtime (guarded internally by timestamp).       */
            Temperature_PeriodicRescan();
        }

        /* ---- 200 ms LED heartbeat (5 Hz): visible firmware alive indicator ---- */
        if ((now - tick_200ms) >= 200) {
            tick_200ms = now;
            HAL_GPIO_TogglePin(GPIOA, PIN_LD2);
        }

        /* ---- 500 ms CAN test transmit: send a test frame (ID 0x123) ---- */
        if ((now - tick_500ms) >= 500) {
            tick_500ms = now;
            CAN_TestTransmit();
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

    /* Configure FDCAN kernel clock source.
     * RCC_CCIPR.FDCANSEL resets to 00 = HSE, which is NOT enabled
     * (this project uses HSI + PLL).  Select PCLK1 (170 MHz) so the
     * FDCAN bit-timing registers produce the intended 500 kbps:
     *   170 MHz / (10 × (1 + 29 + 4)) = 500 kbps
     * Sample point: 88.2 %, SJW: 4 TQ                                */
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* GPIO enable outputs (GPIOC).
     * EN_FL (PC5) and EN_RR (PC13) remain as GPIO outputs.
     * PC6/PC7 (TIM8_CH1/CH2 = RPWM_RL/LPWM_RL) and PC8/PC9
     * (TIM8_CH3/CH4 = RPWM_RR/LPWM_RR) are timer AF outputs;
     * they must NOT be initialised here as GPIO.
     * Wire the corresponding BTS7960 R_EN/L_EN pins to 3.3 V.         */
    gpio.Pin   = PIN_EN_FL | PIN_EN_RR;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* Relay outputs (GPIOC) */
    gpio.Pin = PIN_RELAY_MAIN | PIN_RELAY_TRAC | PIN_RELAY_DIR;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* Nucleo-64 user LED LD2 (PA5) — heartbeat indicator */
    gpio.Pin   = PIN_LD2;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* LED power relays (PB10 front, PB11 rear) — both start OFF (safe default) */
    gpio.Pin = PIN_RELAY_LED | PIN_RELAY_LED_REAR;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, PIN_RELAY_LED | PIN_RELAY_LED_REAR, GPIO_PIN_RESET);

    /* OneWire bus (PB0) — DS18B20 temperature sensors.
     * Initialise as open-drain HIGH (idle) so the bus is in a known
     * state before Sensor_Init() performs the first OW_Reset().       */
    gpio.Pin   = PIN_ONEWIRE;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, PIN_ONEWIRE, GPIO_PIN_SET);

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

    /* Encoder Z-index pulse (PB4).  The Z channel is NOT used for
     * control (see motor_control.c rationale), but the pin must be
     * initialised as input with pull-up to match the .ioc config and
     * prevent the NPN open-collector output from floating.            */
    gpio.Pin  = PIN_ENC_Z;
    gpio.Mode = GPIO_MODE_INPUT;
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
#if defined(CAN_LOOPBACK_TEST) && CAN_LOOPBACK_TEST
    hfdcan1.Init.Mode                 = FDCAN_MODE_INTERNAL_LOOPBACK;
#else
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
#endif
    hfdcan1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    /* CAN 500 kbps bit timing — optimised for high-noise motor environment.
     *
     * Clock source: PCLK1 = 170 MHz (HSI+PLL, no external crystal).
     *   TQ = 170 MHz / 10 = 17 MHz → 58.82 ns per time quantum.
     *   Bit time = 1 (sync) + 29 (seg1) + 4 (seg2) = 34 TQ.
     *   Baud rate = 17 MHz / 34 = 500 kbps.
     *
     * Sample point = (1 + 29) / 34 = 88.2 %
     *   CiA 301 recommends 87.5 % for 500 kbps; 88.2 % is within
     *   the ±2 % tolerance window and provides excellent noise margin
     *   by sampling late in the bit period.
     *
     * SJW = 4 TQ → ±11.8 % oscillator tolerance compensation.
     *   The HSI RC oscillator has ±1 % accuracy; SJW = 4 gives >10×
     *   margin, ensuring reliable synchronisation even with temperature
     *   drift.  Previous SJW = 1 only allowed ±2.94 % tolerance.       */
    hfdcan1.Init.NominalPrescaler     = 10;
    hfdcan1.Init.NominalSyncJumpWidth = 4;
    hfdcan1.Init.NominalTimeSeg1      = 29;
    hfdcan1.Init.NominalTimeSeg2      = 4;
    hfdcan1.Init.AutoRetransmission   = ENABLE;
    hfdcan1.Init.TransmitPause        = ENABLE;  /* Pause between TX frames:
                                                   * inserts ≥2 TQ idle between
                                                   * consecutive transmissions,
                                                   * improving bus fairness and
                                                   * reducing error frames under
                                                   * high bus load.              */
    hfdcan1.Init.ProtocolException    = DISABLE;

    /* ---- Message RAM configuration ----
     * In HAL driver v1.2.4+, the STM32G4 FDCAN Message RAM layout is
     * configured automatically by HAL_FDCAN_Init() based on
     * StdFiltersNbr, ExtFiltersNbr, and TxFifoQueueMode.
     * The per-element fields (RxFifo0ElmtsNbr, TxFifoQueueElmtsNbr,
     * MessageRAMOffset, etc.) were removed from FDCAN_InitTypeDef. */
    hfdcan1.Init.StdFiltersNbr        = 28;  /* Up to 28 standard-ID filters  */
    hfdcan1.Init.ExtFiltersNbr        = 0;
    hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

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

    /* Digital noise filter: reject glitches shorter than DNF × tI2CCLK
     * (≈ 12 ns at 170 MHz for DNF=2).  Negligible impact on SCL frequency
     * but improves I2C reliability in the high-EMI environment near motor
     * PWM drivers and BTS7960 switching transients.
     * Based on NXP AN10216 bus robustness recommendations.
     * Range 0–15; 0 = disabled, 2 = rejects pulses < 2 I2C clock cycles. */
#define I2C_DIGITAL_NOISE_FILTER  2U
    HAL_I2CEx_ConfigDigitalFilter(&hi2c1, I2C_DIGITAL_NOISE_FILTER);

    /* Ensure analog filter is enabled (default ON, but be explicit for
     * documentation and defence against accidental CubeMX regeneration). */
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);

    i2c_init_ok = true;
}

static void MX_TIM1_Init(void)
{
    /* TIM1 drives FL motor (CH1=RPWM_FL/PA8, CH2=LPWM_FL/PA9) and
     * FR motor (CH3=RPWM_FR/PA10, CH4=LPWM_FR/PA11).
     * RPWM and LPWM of each motor are on the SAME timer so both CCR
     * preload registers transfer at the same UEV — overlap = 0.
     * TIM1 is an advanced timer: BREAK2 is armed to the Cortex-M4
     * LOCKUP signal so any true CPU lockup instantly clears MOE,
     * forcing all four outputs LOW without software intervention.    */
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
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);  /* RPWM_FL — PA8  */
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2);  /* LPWM_FL — PA9  */
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_3);  /* RPWM_FR — PA10 */
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_4);  /* LPWM_FR — PA11 */

    /* Buffer CCR — update at period boundary only, preventing mid-cycle
     * duty changes that cause asymmetric pulses in center-aligned mode. */
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_4);

    /* ---- BREAK2: Cortex-M4 LOCKUP → hardware disables all TIM1 outputs ----
     *
     * When the CPU enters LOCKUP (unhandled fault escalation), the LOCKUP
     * signal asserts HIGH and is routed to TIM1 BKIN2.  This clears MOE
     * automatically, forcing all CH1-4 outputs to their idle state (LOW,
     * because OCPolarity=HIGH and OCIdleState=RESET).
     *
     * OSSR=1 / OSSI=1: outputs are driven LOW when MOE=0, both in RUN
     *   mode and in IDLE mode — no floating state possible.
     * AutomaticOutput=DISABLE: MOE is NOT re-enabled by hardware after
     *   break; software must explicitly call __HAL_TIM_MOE_ENABLE()
     *   (done inside HAL_TIM_PWM_Start for each channel in Motor_Init).
     * BreakState=DISABLE: the external BKIN1 pin is not used.
     *
     * Combined with the software path in fault handlers (option D = A+C):
     *   A = LOCKUP → BKIN2 → MOE cleared (hardware, instantaneous)
     *   C = HardFault/BusFault/UsageFault handlers clear MOE + CCRs
     */
    TIM_BreakDeadTimeConfigTypeDef bdtr = {0};
    bdtr.OffStateRunMode  = TIM_OSSR_ENABLE;
    bdtr.OffStateIDLEMode = TIM_OSSI_ENABLE;
    bdtr.LockLevel        = TIM_LOCKLEVEL_OFF;
    bdtr.DeadTime         = 0;
    bdtr.BreakState       = TIM_BREAK_DISABLE;
    bdtr.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
    bdtr.BreakFilter      = 0;
    bdtr.Break2State      = TIM_BREAK2_ENABLE;
    bdtr.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    bdtr.Break2Filter     = 0;
    bdtr.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdtr) != HAL_OK) {
        Error_Handler();
    }

    /* Route Cortex-M4 LOCKUP signal to TIM1/TIM8 BREAK via SYSCFG.
     * This is a one-time write that locks the CLL bit in CFGR2. */
    __HAL_SYSCFG_BREAK_LOCKUP_LOCK();
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

static void MX_TIM3_Init(void)
{
    /* TIM3 drives STEER motor (CH1=RPWM_STEER/PA6, CH2=LPWM_STEER/PA7).
     * Same 20 kHz center-aligned configuration as TIM1 and TIM8.
     * TIM3 is a general-purpose timer: it has no BREAK input.
     * Hardware protection for STEER is via the software path only —
     * fault handlers zero CCR1/CCR2 via direct register access.
     * TIM3 is on APB1; with APB1 prescaler = 1 its clock = 170 MHz. */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
    htim3.Init.Period            = 4249;   /* 170 MHz / (2 × 4250) = 20 kHz */
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.RepetitionCounter = 0;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1);  /* RPWM_STEER — PA6 */
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_2);  /* LPWM_STEER — PA7 */

    /* Buffer CCR — same rationale as TIM1 */
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_2);
}

static void MX_TIM8_Init(void)
{
    /* TIM8 drives RL motor (CH1=RPWM_RL/PC6, CH2=LPWM_RL/PC7) and
     * RR motor (CH3=RPWM_RR/PC8, CH4=LPWM_RR/PC9).
     * Same same-timer guarantee and BREAK2/LOCKUP protection as TIM1. */
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
    HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_1);  /* RPWM_RL — PC6 */
    HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_2);  /* LPWM_RL — PC7 */
    HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_3);  /* RPWM_RR — PC8 */
    HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_4);  /* LPWM_RR — PC9 */

    /* Buffer CCR — same rationale as TIM1 */
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim8, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim8, TIM_CHANNEL_2);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim8, TIM_CHANNEL_3);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim8, TIM_CHANNEL_4);

    /* BREAK2: Cortex-M4 LOCKUP → hardware disables all TIM8 outputs.
     * Identical configuration and rationale as MX_TIM1_Init().
     * The __HAL_SYSCFG_BREAK_LOCKUP_LOCK() call in MX_TIM1_Init()
     * already covers both TIM1 and TIM8 (SYSCFG CLL bit is global). */
    TIM_BreakDeadTimeConfigTypeDef bdtr = {0};
    bdtr.OffStateRunMode  = TIM_OSSR_ENABLE;
    bdtr.OffStateIDLEMode = TIM_OSSI_ENABLE;
    bdtr.LockLevel        = TIM_LOCKLEVEL_OFF;
    bdtr.DeadTime         = 0;
    bdtr.BreakState       = TIM_BREAK_DISABLE;
    bdtr.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
    bdtr.BreakFilter      = 0;
    bdtr.Break2State      = TIM_BREAK2_ENABLE;
    bdtr.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    bdtr.Break2Filter     = 0;
    bdtr.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &bdtr) != HAL_OK) {
        Error_Handler();
    }
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
    /* Hardware oversampling: 16 conversions averaged by right-shifting 4 bits.
     * Result remains 12-bit (0–4095), fully transparent to Pedal_Update().
     * Provides ~12 dB noise reduction on the pedal ADC input — critical in
     * the high-EMI environment near BTS7960 motor drivers and PWM wiring.
     * Conversion time: 16 × (247.5 + 12.5) / 42.5 MHz ≈ 98 µs, negligible
     * compared to the 50 ms Pedal_Update() cycle.                          */
    hadc1.Init.OversamplingMode      = ENABLE;
    hadc1.Init.Oversampling.Ratio                = ADC_OVERSAMPLING_RATIO_16;
    hadc1.Init.Oversampling.RightBitShift        = ADC_RIGHTBITSHIFT_4;
    hadc1.Init.Oversampling.TriggeredMode        = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
    hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
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
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5; /* ~5.8 µs at 42.5 MHz — extended
                                                       * from 47.5 cycles (1.1 µs) for
                                                       * better noise rejection on the
                                                       * pedal input in motor-EMI
                                                       * environment.                   */
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
    /* Safe the hardware: clear MOE on advanced timers, zero TIM3 STEER CCRs,
     * then drive GPIO outputs LOW (relays off, motor EN pins off).
     * Uses direct register access because HAL may be in an inconsistent state.
     * PC6-PC9 are TIM8 AF outputs; PA6-PA7 are TIM3 AF outputs — not GPIO.
     * Only PC5 (EN_FL) and PC13 (EN_RR) remain as GPIO enable outputs.       */
    TIM1->BDTR &= ~TIM_BDTR_MOE;   /* Disable all TIM1 PWM outputs (FL, FR)   */
    TIM8->BDTR &= ~TIM_BDTR_MOE;   /* Disable all TIM8 PWM outputs (RL, RR)   */
    TIM3->CCR1  = 0U;               /* RPWM_STEER → 0 (TIM3 has no BREAK)      */
    TIM3->CCR2  = 0U;               /* LPWM_STEER → 0                          */
    GPIOC->BSRR = (uint32_t)(PIN_EN_FL | PIN_EN_RR
                  | PIN_RELAY_MAIN | PIN_RELAY_TRAC | PIN_RELAY_DIR) << 16U;
    /* LED power relays on GPIOB — also force OFF (both front and rear) */
    GPIOB->BSRR = (uint32_t)(PIN_RELAY_LED | PIN_RELAY_LED_REAR) << 16U;
    while (1) { }
}