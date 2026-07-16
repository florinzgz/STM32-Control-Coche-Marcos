#pragma once
/**
 * @file  stm32g4xx_hal.h
 * @brief Comprehensive HAL stub for CBMC / Infer static analysis.
 *
 * Provides every type, constant, macro, and function stub required to compile
 * all 19+ production C files in Core/Src/ with a host GCC (no ARM toolchain).
 * All function bodies are trivial no-ops that return HAL_OK / 0 / safe defaults.
 *
 * This file intentionally merges content that the real HAL splits across dozens
 * of headers (stm32g4xx.h, stm32g4xx_hal_*.h, stm32g4xx_hal_conf.h, CMSIS).
 */

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <sys/stat.h>   /* S_IFCHR for syscalls.c */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  CMSIS-compatible core types and macros                                   */
/* ========================================================================= */

#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I  volatile const
#endif
#ifndef __O
#define __O  volatile
#endif

typedef enum {
    NonMaskableInt_IRQn   = -14,
    HardFault_IRQn        = -13,
    MemoryManagement_IRQn = -12,
    BusFault_IRQn         = -11,
    UsageFault_IRQn       = -10,
    SVCall_IRQn           = -5,
    DebugMonitor_IRQn     = -4,
    PendSV_IRQn           = -2,
    SysTick_IRQn          = -1,
    /* STM32G4 peripheral IRQs */
    EXTI0_IRQn            = 6,
    EXTI1_IRQn            = 7,
    EXTI2_IRQn            = 8,
    EXTI3_IRQn            = 9,
    EXTI4_IRQn            = 10,
    EXTI9_5_IRQn          = 23,
    EXTI15_10_IRQn        = 40,
    FDCAN1_IT0_IRQn       = 21,
    FDCAN1_IT1_IRQn       = 22,
    TIM1_UP_TIM16_IRQn    = 25,
    TIM2_IRQn             = 28,
    TIM3_IRQn             = 29,
    TIM8_UP_IRQn          = 44,
    I2C1_EV_IRQn          = 31,
    I2C1_ER_IRQn          = 32,
    ADC1_2_IRQn           = 18,
    FLASH_IRQn            = 4,
} IRQn_Type;

/* SCB stub for system_stm32g4xx.c: SCB->VTOR */
typedef struct {
    __I  uint32_t CPUID;
    __IO uint32_t ICSR;
    __IO uint32_t VTOR;
    __IO uint32_t AIRCR;
    __IO uint32_t SCR;
    __IO uint32_t CCR;
    __IO uint8_t  SHP[12];
    __IO uint32_t SHCSR;
    __IO uint32_t CFSR;
    __IO uint32_t HFSR;
    __IO uint32_t DFSR;
    __IO uint32_t MMFAR;
    __IO uint32_t BFAR;
    __IO uint32_t AFSR;
} SCB_Type;

/* FPU stub for system_stm32g4xx.c: FPU->FPCCR */
typedef struct {
    uint32_t RESERVED0[1];
    __IO uint32_t FPCCR;
    __IO uint32_t FPCAR;
    __IO uint32_t FPDSCR;
    __I  uint32_t MVFR0;
    __I  uint32_t MVFR1;
    __I  uint32_t MVFR2;
} FPU_Type;

/* DWT stub for DWT-based microsecond delay (motor_control.c delay_us_dwt) */
typedef struct {
    __IO uint32_t CTRL;
    __IO uint32_t CYCCNT;
    __IO uint32_t CPICNT;
    __IO uint32_t EXCCNT;
    __IO uint32_t SLEEPCNT;
    __IO uint32_t LSUCNT;
    __IO uint32_t FOLDCNT;
    __I  uint32_t PCSR;
    __IO uint32_t COMP0;
    __IO uint32_t MASK0;
    __IO uint32_t FUNCTION0;
} DWT_Type;

#define DWT_CTRL_CYCCNTENA_Msk  (1UL)

/* CoreDebug stub for enabling DWT trace (DEMCR register) */
typedef struct {
    __IO uint32_t DHCSR;
    __O  uint32_t DCRSR;
    __IO uint32_t DCRDR;
    __IO uint32_t DEMCR;
} CoreDebug_Type;

#define CoreDebug_DEMCR_TRCENA_Msk  (1UL << 24)

static SCB_Type       _stub_scb;
static FPU_Type       _stub_fpu;
static DWT_Type       _stub_dwt;
static CoreDebug_Type _stub_coredebug;
#define SCB       (&_stub_scb)
#define FPU       (&_stub_fpu)
#define DWT       (&_stub_dwt)
#define CoreDebug (&_stub_coredebug)

/* VECT_TAB constants used by system_stm32g4xx.c */
#ifndef VECT_TAB_BASE_ADDRESS
#define VECT_TAB_BASE_ADDRESS 0x08000000UL
#endif
#ifndef VECT_TAB_OFFSET
#define VECT_TAB_OFFSET       0x00000000UL
#endif

/* Intrinsics */
static inline void __disable_irq(void) { }
static inline void __enable_irq(void)  { }
static inline void __NOP(void)         { }
static inline void __WFI(void)         { }
static inline void __DSB(void)         { }
static inline void __ISB(void)         { }
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }

/* ========================================================================= */
/*  FunctionalState and basic HAL enums                                      */
/* ========================================================================= */

typedef enum { DISABLE = 0U, ENABLE = !DISABLE } FunctionalState;

typedef enum {
    HAL_OK      = 0x00U,
    HAL_ERROR   = 0x01U,
    HAL_BUSY    = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef enum {
    HAL_UNLOCKED = 0x00U,
    HAL_LOCKED   = 0x01U
} HAL_LockTypeDef;

typedef enum {
    HAL_TICK_FREQ_10HZ   = 100U,
    HAL_TICK_FREQ_100HZ  = 10U,
    HAL_TICK_FREQ_1KHZ   = 1U,
    HAL_TICK_FREQ_DEFAULT = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;

/* ========================================================================= */
/*  Oscillator / Clock values (from stm32g4xx_hal_conf.h)                   */
/* ========================================================================= */

#ifndef HSE_VALUE
#define HSE_VALUE    8000000U
#endif
#ifndef HSE_STARTUP_TIMEOUT
#define HSE_STARTUP_TIMEOUT 100U
#endif
#ifndef HSI_VALUE
#define HSI_VALUE    16000000U
#endif
#ifndef LSI_VALUE
#define LSI_VALUE    32000U
#endif
#ifndef LSE_VALUE
#define LSE_VALUE    32768U
#endif
#ifndef HSI48_VALUE
#define HSI48_VALUE  48000000U
#endif

#define VDD_VALUE               3300U
#define TICK_INT_PRIORITY       0U
#define USE_RTOS                0U
#define PREFETCH_ENABLE         0U
#define INSTRUCTION_CACHE_ENABLE 1U
#define DATA_CACHE_ENABLE       1U

/* assert_param */
#ifndef USE_FULL_ASSERT
#define assert_param(expr) ((void)0U)
#endif

/* Global clock variable (defined in system_stm32g4xx.c) */
extern uint32_t SystemCoreClock;

/* ========================================================================= */
/*  GPIO Types and Constants                                                 */
/* ========================================================================= */

typedef struct {
    __IO uint32_t MODER;
    __IO uint32_t OTYPER;
    __IO uint32_t OSPEEDR;
    __IO uint32_t PUPDR;
    __IO uint32_t IDR;
    __IO uint32_t ODR;
    __IO uint32_t BSRR;
    __IO uint32_t LCKR;
    __IO uint32_t AFR[2];
    __IO uint32_t BRR;
} GPIO_TypeDef;

static GPIO_TypeDef _stub_gpioa, _stub_gpiob, _stub_gpioc;
#define GPIOA (&_stub_gpioa)
#define GPIOB (&_stub_gpiob)
#define GPIOC (&_stub_gpioc)

typedef enum {
    GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET   = 1U
} GPIO_PinState;

/* GPIO pin definitions */
#define GPIO_PIN_0   ((uint16_t)0x0001)
#define GPIO_PIN_1   ((uint16_t)0x0002)
#define GPIO_PIN_2   ((uint16_t)0x0004)
#define GPIO_PIN_3   ((uint16_t)0x0008)
#define GPIO_PIN_4   ((uint16_t)0x0010)
#define GPIO_PIN_5   ((uint16_t)0x0020)
#define GPIO_PIN_6   ((uint16_t)0x0040)
#define GPIO_PIN_7   ((uint16_t)0x0080)
#define GPIO_PIN_8   ((uint16_t)0x0100)
#define GPIO_PIN_9   ((uint16_t)0x0200)
#define GPIO_PIN_10  ((uint16_t)0x0400)
#define GPIO_PIN_11  ((uint16_t)0x0800)
#define GPIO_PIN_12  ((uint16_t)0x1000)
#define GPIO_PIN_13  ((uint16_t)0x2000)
#define GPIO_PIN_14  ((uint16_t)0x4000)
#define GPIO_PIN_15  ((uint16_t)0x8000)
#define GPIO_PIN_All ((uint16_t)0xFFFF)

/* GPIO modes */
#define GPIO_MODE_INPUT              0x00000000U
#define GPIO_MODE_OUTPUT_PP          0x00000001U
#define GPIO_MODE_OUTPUT_OD          0x00000011U
#define GPIO_MODE_AF_PP              0x00000002U
#define GPIO_MODE_AF_OD              0x00000012U
#define GPIO_MODE_ANALOG             0x00000003U
#define GPIO_MODE_IT_RISING          0x10110000U
#define GPIO_MODE_IT_FALLING         0x10210000U
#define GPIO_MODE_IT_RISING_FALLING  0x10310000U
#define GPIO_MODE_EVT_RISING         0x10120000U
#define GPIO_MODE_EVT_FALLING        0x10220000U
#define GPIO_MODE_EVT_RISING_FALLING 0x10320000U

/* GPIO pull */
#define GPIO_NOPULL   0x00000000U
#define GPIO_PULLUP   0x00000001U
#define GPIO_PULLDOWN 0x00000002U

/* GPIO speed */
#define GPIO_SPEED_FREQ_LOW       0x00000000U
#define GPIO_SPEED_FREQ_MEDIUM    0x00000001U
#define GPIO_SPEED_FREQ_HIGH      0x00000002U
#define GPIO_SPEED_FREQ_VERY_HIGH 0x00000003U

/* GPIO alternate functions (used in stm32g4xx_hal_msp.c) */
#define GPIO_AF0_MCO       0x00U
#define GPIO_AF1_TIM2      0x01U
#define GPIO_AF2_TIM3      0x02U
#define GPIO_AF4_TIM8      0x04U
#define GPIO_AF4_I2C1      0x04U
#define GPIO_AF1_TIM15     0x01U
#define GPIO_AF2_TIM1      0x02U
#define GPIO_AF6_TIM1      0x06U
#define GPIO_AF9_FDCAN1    0x09U

typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
    uint32_t Alternate;
} GPIO_InitTypeDef;

/* ========================================================================= */
/*  TIM Types and Constants                                                  */
/* ========================================================================= */

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMCR;
    __IO uint32_t DIER;
    __IO uint32_t SR;
    __IO uint32_t EGR;
    __IO uint32_t CCMR1;
    __IO uint32_t CCMR2;
    __IO uint32_t CCER;
    __IO uint32_t CNT;
    __IO uint32_t PSC;
    __IO uint32_t ARR;
    __IO uint32_t RCR;
    __IO uint32_t CCR1;
    __IO uint32_t CCR2;
    __IO uint32_t CCR3;
    __IO uint32_t CCR4;
    __IO uint32_t BDTR;
    __IO uint32_t CCR5;
    __IO uint32_t CCR6;
    __IO uint32_t CCMR3;
    __IO uint32_t DTR2;
    __IO uint32_t ECR;
    __IO uint32_t TISEL;
    __IO uint32_t AF1;
    __IO uint32_t AF2;
    __IO uint32_t OR;
         uint32_t RESERVED0[220];
    __IO uint32_t DCR;
    __IO uint32_t DMAR;
} TIM_TypeDef;

static TIM_TypeDef _stub_tim1, _stub_tim2, _stub_tim3, _stub_tim8, _stub_tim15;
#define TIM1  (&_stub_tim1)
#define TIM2  (&_stub_tim2)
#define TIM3  (&_stub_tim3)
#define TIM8  (&_stub_tim8)
#define TIM15 (&_stub_tim15)

#define TIM_BDTR_MOE  0x8000U

typedef struct {
    uint32_t Prescaler;
    uint32_t CounterMode;
    uint32_t Period;
    uint32_t ClockDivision;
    uint32_t RepetitionCounter;
    uint32_t AutoReloadPreload;
} TIM_Base_InitTypeDef;

typedef struct {
    TIM_TypeDef          *Instance;
    TIM_Base_InitTypeDef  Init;
    HAL_LockTypeDef       Lock;
    uint32_t              State;
    uint32_t              DMABurstState;
} TIM_HandleTypeDef;

typedef struct {
    uint32_t OCMode;
    uint32_t Pulse;
    uint32_t OCPolarity;
    uint32_t OCNPolarity;
    uint32_t OCFastMode;
    uint32_t OCIdleState;
    uint32_t OCNIdleState;
} TIM_OC_InitTypeDef;

typedef struct {
    uint32_t ICPolarity;
    uint32_t ICSelection;
    uint32_t ICPrescaler;
    uint32_t ICFilter;
} TIM_IC_InitTypeDef;

typedef struct {
    uint32_t EncoderMode;
    uint32_t IC1Polarity;
    uint32_t IC1Selection;
    uint32_t IC1Prescaler;
    uint32_t IC1Filter;
    uint32_t IC2Polarity;
    uint32_t IC2Selection;
    uint32_t IC2Prescaler;
    uint32_t IC2Filter;
} TIM_Encoder_InitTypeDef;

typedef struct {
    uint32_t OffStateRunMode;
    uint32_t OffStateIDLEMode;
    uint32_t LockLevel;
    uint32_t DeadTime;
    uint32_t BreakState;
    uint32_t BreakPolarity;
    uint32_t BreakFilter;
    uint32_t BreakAFMode;
    uint32_t Break2State;
    uint32_t Break2Polarity;
    uint32_t Break2Filter;
    uint32_t Break2AFMode;
    uint32_t AutomaticOutput;
} TIM_BreakDeadTimeConfigTypeDef;

typedef struct {
    uint32_t MasterOutputTrigger;
    uint32_t MasterOutputTrigger2;
    uint32_t MasterSlaveMode;
} TIM_MasterConfigTypeDef;

/* TIM counter modes */
#define TIM_COUNTERMODE_UP              0x00000000U
#define TIM_COUNTERMODE_DOWN            0x00000010U
#define TIM_COUNTERMODE_CENTERALIGNED1  0x00000020U
#define TIM_COUNTERMODE_CENTERALIGNED2  0x00000040U
#define TIM_COUNTERMODE_CENTERALIGNED3  0x00000060U

/* TIM clock division */
#define TIM_CLOCKDIVISION_DIV1 0x00000000U
#define TIM_CLOCKDIVISION_DIV2 0x00000100U
#define TIM_CLOCKDIVISION_DIV4 0x00000200U

/* TIM auto-reload preload */
#define TIM_AUTORELOAD_PRELOAD_DISABLE  0x00000000U
#define TIM_AUTORELOAD_PRELOAD_ENABLE   0x00000080U

/* TIM OC modes */
#define TIM_OCMODE_TIMING        0x00000000U
#define TIM_OCMODE_ACTIVE        0x00000010U
#define TIM_OCMODE_INACTIVE      0x00000020U
#define TIM_OCMODE_TOGGLE        0x00000030U
#define TIM_OCMODE_PWM1          0x00000060U
#define TIM_OCMODE_PWM2          0x00000070U

/* TIM OC polarity */
#define TIM_OCPOLARITY_HIGH   0x00000000U
#define TIM_OCPOLARITY_LOW    0x00000002U

/* TIM OC fast mode */
#define TIM_OCFAST_DISABLE 0x00000000U
#define TIM_OCFAST_ENABLE  0x00000004U

/* TIM encoder mode */
#define TIM_ENCODERMODE_TI1  0x00000001U
#define TIM_ENCODERMODE_TI2  0x00000002U
#define TIM_ENCODERMODE_TI12 0x00000003U

/* TIM IC polarity */
#define TIM_ICPOLARITY_RISING   0x00000000U
#define TIM_ICPOLARITY_FALLING  0x00000002U
#define TIM_ICPOLARITY_BOTHEDGE 0x0000000AU

/* TIM IC selection */
#define TIM_ICSELECTION_DIRECTTI   0x00000001U
#define TIM_ICSELECTION_INDIRECTTI 0x00000002U
#define TIM_ICSELECTION_TRC        0x00000003U

/* TIM IC prescaler */
#define TIM_ICPSC_DIV1 0x00000000U
#define TIM_ICPSC_DIV2 0x00000004U
#define TIM_ICPSC_DIV4 0x00000008U
#define TIM_ICPSC_DIV8 0x0000000CU

/* TIM channels */
#define TIM_CHANNEL_1   0x00000000U
#define TIM_CHANNEL_2   0x00000004U
#define TIM_CHANNEL_3   0x00000008U
#define TIM_CHANNEL_4   0x0000000CU
#define TIM_CHANNEL_5   0x00000010U
#define TIM_CHANNEL_6   0x00000014U
#define TIM_CHANNEL_ALL 0x0000003CU

/* TIM break/deadtime constants */
#define TIM_BREAK_DISABLE           0x00000000U
#define TIM_BREAK_ENABLE            0x00001000U
#define TIM_BREAK2_DISABLE          0x00000000U
#define TIM_BREAK2_ENABLE           0x01000000U
#define TIM_BREAKPOLARITY_LOW       0x00000000U
#define TIM_BREAKPOLARITY_HIGH      0x00002000U
#define TIM_BREAK2POLARITY_LOW      0x00000000U
#define TIM_BREAK2POLARITY_HIGH     0x02000000U
#define TIM_OSSR_DISABLE            0x00000000U
#define TIM_OSSR_ENABLE             0x00000800U
#define TIM_OSSI_DISABLE            0x00000000U
#define TIM_OSSI_ENABLE             0x00000400U
#define TIM_LOCKLEVEL_OFF           0x00000000U
#define TIM_LOCKLEVEL_1             0x00000100U
#define TIM_LOCKLEVEL_2             0x00000200U
#define TIM_LOCKLEVEL_3             0x00000300U
#define TIM_AUTOMATICOUTPUT_DISABLE 0x00000000U
#define TIM_AUTOMATICOUTPUT_ENABLE  0x00004000U

/* TIM master trigger output */
#define TIM_TRGO_RESET  0x00000000U
#define TIM_TRGO_ENABLE 0x00000010U
#define TIM_TRGO_UPDATE 0x00000020U

/* TIM master slave mode */
#define TIM_MASTERSLAVEMODE_ENABLE  0x00000080U
#define TIM_MASTERSLAVEMODE_DISABLE 0x00000000U

/* ---- TIM macros ---- */
#define __HAL_TIM_GET_COUNTER(__HANDLE__)             ((__HANDLE__)->Instance->CNT)
#define __HAL_TIM_SET_COUNTER(__HANDLE__, __COUNTER__) ((__HANDLE__)->Instance->CNT = (__COUNTER__))

#define __HAL_TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__) \
    do { \
        switch (__CHANNEL__) { \
            case TIM_CHANNEL_1: (__HANDLE__)->Instance->CCR1 = (__COMPARE__); break; \
            case TIM_CHANNEL_2: (__HANDLE__)->Instance->CCR2 = (__COMPARE__); break; \
            case TIM_CHANNEL_3: (__HANDLE__)->Instance->CCR3 = (__COMPARE__); break; \
            case TIM_CHANNEL_4: (__HANDLE__)->Instance->CCR4 = (__COMPARE__); break; \
            default: break; \
        } \
    } while(0)

#define __HAL_TIM_GET_COMPARE(__HANDLE__, __CHANNEL__) \
    (((__CHANNEL__) == TIM_CHANNEL_1) ? (__HANDLE__)->Instance->CCR1 : \
     ((__CHANNEL__) == TIM_CHANNEL_2) ? (__HANDLE__)->Instance->CCR2 : \
     ((__CHANNEL__) == TIM_CHANNEL_3) ? (__HANDLE__)->Instance->CCR3 : \
     ((__CHANNEL__) == TIM_CHANNEL_4) ? (__HANDLE__)->Instance->CCR4 : 0U)

#define __HAL_TIM_MOE_ENABLE(__HANDLE__) \
    ((__HANDLE__)->Instance->BDTR |= TIM_BDTR_MOE)

#define __HAL_TIM_MOE_DISABLE(__HANDLE__) \
    ((__HANDLE__)->Instance->BDTR &= ~TIM_BDTR_MOE)

#define __HAL_TIM_ENABLE_OCxPRELOAD(__HANDLE__, __CHANNEL__) ((void)0)

/* ========================================================================= */
/*  ADC Types and Constants                                                  */
/* ========================================================================= */

typedef struct {
    __IO uint32_t ISR;
    __IO uint32_t IER;
    __IO uint32_t CR;
    __IO uint32_t CFGR;
    __IO uint32_t CFGR2;
    __IO uint32_t SMPR1;
    __IO uint32_t SMPR2;
    uint32_t      RESERVED1;
    __IO uint32_t TR1;
    __IO uint32_t TR2;
    __IO uint32_t TR3;
    uint32_t      RESERVED2;
    __IO uint32_t SQR1;
    __IO uint32_t SQR2;
    __IO uint32_t SQR3;
    __IO uint32_t SQR4;
    __IO uint32_t DR;
} ADC_TypeDef;

static ADC_TypeDef _stub_adc1;
#define ADC1 (&_stub_adc1)

typedef struct {
    uint32_t Ratio;
    uint32_t RightBitShift;
    uint32_t TriggeredMode;
    uint32_t OversamplingStopReset;
} ADC_OversamplingTypeDef;

#define ADC_OVERSAMPLING_RATIO_2        0x00000000U
#define ADC_OVERSAMPLING_RATIO_4        0x00000001U
#define ADC_OVERSAMPLING_RATIO_8        0x00000002U
#define ADC_OVERSAMPLING_RATIO_16       0x00000003U
#define ADC_OVERSAMPLING_RATIO_32       0x00000004U
#define ADC_OVERSAMPLING_RATIO_64       0x00000005U
#define ADC_OVERSAMPLING_RATIO_128      0x00000006U
#define ADC_OVERSAMPLING_RATIO_256      0x00000007U
#define ADC_RIGHTBITSHIFT_NONE          0x00000000U
#define ADC_RIGHTBITSHIFT_1             0x00000001U
#define ADC_RIGHTBITSHIFT_2             0x00000002U
#define ADC_RIGHTBITSHIFT_3             0x00000003U
#define ADC_RIGHTBITSHIFT_4             0x00000004U
#define ADC_TRIGGEREDMODE_SINGLE_TRIGGER  0x00000000U
#define ADC_TRIGGEREDMODE_MULTI_TRIGGER   0x00000001U
#define ADC_REGOVERSAMPLING_CONTINUED_MODE  0x00000000U
#define ADC_REGOVERSAMPLING_RESUMED_MODE    0x00000001U

typedef struct {
    uint32_t ClockPrescaler;
    uint32_t Resolution;
    uint32_t DataAlign;
    uint32_t ScanConvMode;
    uint32_t EOCSelection;
    FunctionalState LowPowerAutoWait;
    FunctionalState ContinuousConvMode;
    uint32_t NbrOfConversion;
    FunctionalState DiscontinuousConvMode;
    uint32_t NbrOfDiscConversion;
    uint32_t ExternalTrigConv;
    uint32_t ExternalTrigConvEdge;
    FunctionalState DMAContinuousRequests;
    uint32_t Overrun;
    FunctionalState OversamplingMode;
    ADC_OversamplingTypeDef Oversampling;
    uint32_t GainCompensation;
} ADC_InitTypeDef;

typedef struct {
    ADC_TypeDef     *Instance;
    ADC_InitTypeDef  Init;
    HAL_LockTypeDef  Lock;
    uint32_t         State;
    uint32_t         ErrorCode;
} ADC_HandleTypeDef;

typedef struct {
    uint32_t Channel;
    uint32_t Rank;
    uint32_t SamplingTime;
    uint32_t SingleDiff;
    uint32_t OffsetNumber;
    uint32_t Offset;
    uint32_t OffsetSign;
    FunctionalState OffsetSaturation;
} ADC_ChannelConfTypeDef;

/* ADC clock */
#define ADC_CLOCK_SYNC_PCLK_DIV1 0x00000001U
#define ADC_CLOCK_SYNC_PCLK_DIV2 0x00000002U
#define ADC_CLOCK_SYNC_PCLK_DIV4 0x00000003U
#define ADC_CLOCK_ASYNC_DIV1     0x00000000U

/* ADC resolution */
#define ADC_RESOLUTION_12B 0x00000000U
#define ADC_RESOLUTION_10B 0x00000008U
#define ADC_RESOLUTION_8B  0x00000010U
#define ADC_RESOLUTION_6B  0x00000018U

/* ADC data alignment */
#define ADC_DATAALIGN_RIGHT 0x00000000U
#define ADC_DATAALIGN_LEFT  0x00000020U

/* ADC scan mode */
#define ADC_SCAN_DISABLE 0x00000000U
#define ADC_SCAN_ENABLE  0x00000001U

/* ADC end-of-conversion selection */
#define ADC_EOC_SINGLE_CONV 0x00000000U
#define ADC_EOC_SEQ_CONV    0x00000004U

/* ADC external trigger */
#define ADC_SOFTWARE_START            0x00000000U
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0x00000000U

/* ADC overrun mode */
#define ADC_OVR_DATA_PRESERVED  0x00000000U
#define ADC_OVR_DATA_OVERWRITTEN 0x00001000U

/* ADC channels */
#define ADC_CHANNEL_0   0x00000000U
#define ADC_CHANNEL_1   0x00000001U
#define ADC_CHANNEL_2   0x00000002U
#define ADC_CHANNEL_3   0x00000003U
#define ADC_CHANNEL_4   0x00000004U
#define ADC_CHANNEL_5   0x00000005U
#define ADC_CHANNEL_6   0x00000006U
#define ADC_CHANNEL_7   0x00000007U
#define ADC_CHANNEL_8   0x00000008U
#define ADC_CHANNEL_9   0x00000009U
#define ADC_CHANNEL_10  0x0000000AU
#define ADC_CHANNEL_11  0x0000000BU
#define ADC_CHANNEL_12  0x0000000CU

/* ADC rank */
#define ADC_REGULAR_RANK_1  0x00000001U
#define ADC_REGULAR_RANK_2  0x00000002U

/* ADC sampling time */
#define ADC_SAMPLETIME_2CYCLES_5   0x00000000U
#define ADC_SAMPLETIME_6CYCLES_5   0x00000001U
#define ADC_SAMPLETIME_12CYCLES_5  0x00000002U
#define ADC_SAMPLETIME_24CYCLES_5  0x00000003U
#define ADC_SAMPLETIME_47CYCLES_5  0x00000004U
#define ADC_SAMPLETIME_92CYCLES_5  0x00000005U
#define ADC_SAMPLETIME_247CYCLES_5 0x00000006U
#define ADC_SAMPLETIME_640CYCLES_5 0x00000007U

/* ADC single/diff ended */
#define ADC_SINGLE_ENDED     0x00000000U
#define ADC_DIFFERENTIAL_ENDED 0x00000001U

/* ADC offset */
#define ADC_OFFSET_NONE 0x00000000U
#define ADC_OFFSET_1    0x00000001U

/* ========================================================================= */
/*  FDCAN Types and Constants                                                */
/* ========================================================================= */

typedef struct {
    __IO uint32_t CREL;
    __IO uint32_t DBTP;
    __IO uint32_t TEST;
    __IO uint32_t RWD;
    __IO uint32_t CCCR;
    __IO uint32_t NBTP;
    __IO uint32_t TSCC;
    __IO uint32_t TSCV;
    __IO uint32_t TOCC;
    __IO uint32_t TOCV;
    __IO uint32_t ECR;
    __IO uint32_t PSR;
} FDCAN_GlobalTypeDef;

static FDCAN_GlobalTypeDef _stub_fdcan1;
#define FDCAN1 (&_stub_fdcan1)

typedef struct {
    uint32_t ClockDivider;
    uint32_t FrameFormat;
    uint32_t Mode;
    uint32_t AutoRetransmission;
    FunctionalState TransmitPause;
    FunctionalState ProtocolException;
    uint32_t NominalPrescaler;
    uint32_t NominalSyncJumpWidth;
    uint32_t NominalTimeSeg1;
    uint32_t NominalTimeSeg2;
    uint32_t DataPrescaler;
    uint32_t DataSyncJumpWidth;
    uint32_t DataTimeSeg1;
    uint32_t DataTimeSeg2;
    uint32_t MessageRAMOffset;
    uint32_t StdFiltersNbr;
    uint32_t ExtFiltersNbr;
    uint32_t RxFifo0ElmtsNbr;
    uint32_t RxFifo0ElmtSize;
    uint32_t RxFifo1ElmtsNbr;
    uint32_t RxFifo1ElmtSize;
    uint32_t RxBuffersNbr;
    uint32_t RxBufferSize;
    uint32_t TxEventsNbr;
    uint32_t TxBuffersNbr;
    uint32_t TxFifoQueueElmtsNbr;
    uint32_t TxFifoQueueMode;
    uint32_t TxElmtSize;
} FDCAN_InitTypeDef;

typedef struct {
    FDCAN_GlobalTypeDef *Instance;
    FDCAN_InitTypeDef    Init;
    HAL_LockTypeDef      Lock;
    uint32_t             State;
    uint32_t             ErrorCode;
    uint32_t             LatestTxFifoQRequest;
} FDCAN_HandleTypeDef;

typedef struct {
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t RxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t RxTimestamp;
    uint32_t FilterIndex;
    uint32_t IsFilterMatchingFrame;
} FDCAN_RxHeaderTypeDef;

typedef struct {
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t TxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t TxEventFifoControl;
    uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct {
    uint32_t IdType;
    uint32_t FilterIndex;
    uint32_t FilterType;
    uint32_t FilterConfig;
    uint32_t FilterID1;
    uint32_t FilterID2;
} FDCAN_FilterTypeDef;

typedef struct {
    uint32_t LastErrorCode;
    uint32_t DataLastErrorCode;
    uint32_t Activity;
    uint32_t ErrorPassive;
    uint32_t Warning;
    uint32_t BusOff;
    uint32_t RxESIflag;
    uint32_t TDCvalue;
} FDCAN_ProtocolStatusTypeDef;

typedef struct {
    uint32_t TxErrorCnt;     /*!< Specifies the Transmit Error Counter Value */
    uint32_t RxErrorCnt;     /*!< Specifies the Receive Error Counter Value */
    uint32_t RxErrorPassive; /*!< Specifies the Receive Error Passive status */
    uint32_t ErrorLogging;   /*!< Specifies the error logging counter value */
} FDCAN_ErrorCountersTypeDef;

/* FDCAN ID types */
#define FDCAN_STANDARD_ID 0x00000000U
#define FDCAN_EXTENDED_ID 0x40000000U

/* FDCAN frame types */
#define FDCAN_DATA_FRAME   0x00000000U
#define FDCAN_REMOTE_FRAME 0x00100000U

/* FDCAN frame format */
#define FDCAN_FRAME_CLASSIC 0x00000000U
#define FDCAN_FRAME_FD_BRS  0x00100000U
#define FDCAN_FRAME_FD_NO_BRS 0x00200000U

/* FDCAN mode */
#define FDCAN_MODE_NORMAL            0x00000000U
#define FDCAN_MODE_RESTRICTED_OPERATION 0x00000001U
#define FDCAN_MODE_BUS_MONITORING    0x00000002U
#define FDCAN_MODE_INTERNAL_LOOPBACK 0x00000003U
#define FDCAN_MODE_EXTERNAL_LOOPBACK 0x00000004U

/* FDCAN clock divider */
#define FDCAN_CLOCK_DIV1  0x00000000U
#define FDCAN_CLOCK_DIV2  0x00010000U

/* FDCAN data size */
#define FDCAN_DATA_BYTES_8  0x00000004U
#define FDCAN_DATA_BYTES_12 0x00000005U
#define FDCAN_DATA_BYTES_16 0x00000006U
#define FDCAN_DATA_BYTES_20 0x00000007U
#define FDCAN_DATA_BYTES_24 0x00000008U
#define FDCAN_DATA_BYTES_32 0x0000000AU
#define FDCAN_DATA_BYTES_48 0x0000000EU
#define FDCAN_DATA_BYTES_64 0x00000012U

/* FDCAN DLC */
#define FDCAN_DLC_BYTES_0  0x00000000U
#define FDCAN_DLC_BYTES_1  0x00010000U
#define FDCAN_DLC_BYTES_2  0x00020000U
#define FDCAN_DLC_BYTES_3  0x00030000U
#define FDCAN_DLC_BYTES_4  0x00040000U
#define FDCAN_DLC_BYTES_5  0x00050000U
#define FDCAN_DLC_BYTES_6  0x00060000U
#define FDCAN_DLC_BYTES_7  0x00070000U
#define FDCAN_DLC_BYTES_8  0x00080000U
#define FDCAN_DLC_BYTES_12 0x00090000U
#define FDCAN_DLC_BYTES_16 0x000A0000U
#define FDCAN_DLC_BYTES_20 0x000B0000U
#define FDCAN_DLC_BYTES_24 0x000C0000U
#define FDCAN_DLC_BYTES_32 0x000D0000U
#define FDCAN_DLC_BYTES_48 0x000E0000U
#define FDCAN_DLC_BYTES_64 0x000F0000U

/* FDCAN ESI */
#define FDCAN_ESI_ACTIVE  0x00000000U
#define FDCAN_ESI_PASSIVE 0x80000000U

/* FDCAN BRS */
#define FDCAN_BRS_OFF 0x00000000U
#define FDCAN_BRS_ON  0x00100000U

/* FDCAN FD format */
#define FDCAN_CLASSIC_CAN 0x00000000U
#define FDCAN_FD_CAN      0x00200000U

/* FDCAN TX event FIFO control */
#define FDCAN_NO_TX_EVENTS    0x00000000U
#define FDCAN_STORE_TX_EVENTS 0x00800000U

/* FDCAN TX FIFO/queue mode */
#define FDCAN_TX_FIFO_OPERATION  0x00000000U
#define FDCAN_TX_QUEUE_OPERATION 0x01000000U

/* FDCAN filter types */
#define FDCAN_FILTER_RANGE         0x00000000U
#define FDCAN_FILTER_DUAL          0x00000001U
#define FDCAN_FILTER_MASK          0x00000002U
#define FDCAN_FILTER_RANGE_NO_EIDM 0x00000003U

/* FDCAN filter config */
#define FDCAN_FILTER_DISABLE    0x00000000U
#define FDCAN_FILTER_TO_RXFIFO0 0x00000001U
#define FDCAN_FILTER_TO_RXFIFO1 0x00000002U
#define FDCAN_FILTER_REJECT     0x00000003U
#define FDCAN_FILTER_HP         0x00000004U
#define FDCAN_FILTER_TO_RXBUFFER 0x00000005U

/* FDCAN RX FIFOs */
#define FDCAN_RX_FIFO0 0x00000040U
#define FDCAN_RX_FIFO1 0x00000041U

/* FDCAN interrupts */
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE  0x00000001U
#define FDCAN_IT_RX_FIFO0_WATERMARK   0x00000002U
#define FDCAN_IT_RX_FIFO0_FULL        0x00000004U
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST 0x00000008U
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE  0x00000010U
#define FDCAN_IT_RX_FIFO1_WATERMARK   0x00000020U
#define FDCAN_IT_RX_FIFO1_FULL        0x00000040U
#define FDCAN_IT_RX_FIFO1_MESSAGE_LOST 0x00000080U
#define FDCAN_IT_TX_COMPLETE           0x00000100U
#define FDCAN_IT_TX_ABORT_COMPLETE     0x00000200U
#define FDCAN_IT_TX_FIFO_EMPTY         0x00000400U
#define FDCAN_IT_BUS_OFF               0x00000800U
#define FDCAN_IT_ERROR_PASSIVE         0x00001000U
#define FDCAN_IT_ERROR_WARNING         0x00002000U
#define FDCAN_IT_ARB_PROTOCOL_ERROR    0x00004000U
#define FDCAN_IT_DATA_PROTOCOL_ERROR   0x00008000U
#define FDCAN_IT_ERROR_LOGGING_OVERFLOW 0x00010000U

/* FDCAN flags */
#define FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE   0x00000001U
#define FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST  0x00000008U
#define FDCAN_FLAG_RX_FIFO1_NEW_MESSAGE   0x00000010U
#define FDCAN_FLAG_RX_FIFO1_MESSAGE_LOST  0x00000080U
#define FDCAN_FLAG_BUS_OFF                0x00000800U
#define FDCAN_FLAG_ERROR_PASSIVE          0x00001000U
#define FDCAN_FLAG_ERROR_WARNING          0x00002000U

/* FDCAN global filter config */
#define FDCAN_ACCEPT_IN_RX_FIFO0 0x00000000U
#define FDCAN_ACCEPT_IN_RX_FIFO1 0x00000001U
#define FDCAN_REJECT              0x00000002U
#define FDCAN_FILTER_REMOTE       0x00000000U
#define FDCAN_REJECT_REMOTE       0x00000001U

/* FDCAN flag/interrupt macros */
#define __HAL_FDCAN_GET_FLAG(__HANDLE__, __FLAG__)    (0U)
#define __HAL_FDCAN_CLEAR_FLAG(__HANDLE__, __FLAG__)  ((void)0)

/* ========================================================================= */
/*  I2C Types and Constants                                                  */
/* ========================================================================= */

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t OAR1;
    __IO uint32_t OAR2;
    __IO uint32_t TIMINGR;
    __IO uint32_t TIMEOUTR;
    __IO uint32_t ISR;
    __IO uint32_t ICR;
    __IO uint32_t PECR;
    __IO uint32_t RXDR;
    __IO uint32_t TXDR;
} I2C_TypeDef;

static I2C_TypeDef _stub_i2c1;
#define I2C1 (&_stub_i2c1)

typedef struct {
    uint32_t Timing;
    uint32_t OwnAddress1;
    uint32_t AddressingMode;
    uint32_t DualAddressMode;
    uint32_t OwnAddress2;
    uint32_t OwnAddress2Masks;
    uint32_t GeneralCallMode;
    uint32_t NoStretchMode;
} I2C_InitTypeDef;

typedef struct {
    I2C_TypeDef     *Instance;
    I2C_InitTypeDef  Init;
    uint8_t         *pBuffPtr;
    uint16_t         XferSize;
    volatile uint16_t XferCount;
    HAL_LockTypeDef  Lock;
    uint32_t         State;
    uint32_t         Mode;
    uint32_t         ErrorCode;
    uint32_t         AddrEventCount;
    uint32_t         Devaddress;
    uint32_t         Memaddress;
    uint32_t         MemaddSize;
    uint32_t         XferOptions;
    uint32_t         PreviousState;
} I2C_HandleTypeDef;

/* I2C addressing mode */
#define I2C_ADDRESSINGMODE_7BIT  0x00000001U
#define I2C_ADDRESSINGMODE_10BIT 0x00000002U

/* I2C dual address */
#define I2C_DUALADDRESS_DISABLE 0x00000000U
#define I2C_DUALADDRESS_ENABLE  0x00000001U

/* I2C general call */
#define I2C_GENERALCALL_DISABLE 0x00000000U
#define I2C_GENERALCALL_ENABLE  0x00000001U

/* I2C no stretch */
#define I2C_NOSTRETCH_DISABLE 0x00000000U
#define I2C_NOSTRETCH_ENABLE  0x00000001U

/* I2C memory address size */
#define I2C_MEMADD_SIZE_8BIT  0x00000001U
#define I2C_MEMADD_SIZE_16BIT 0x00000010U

/* ========================================================================= */
/*  IWDG Types and Constants                                                 */
/* ========================================================================= */

typedef struct {
    __IO uint32_t KR;
    __IO uint32_t PR;
    __IO uint32_t RLR;
    __IO uint32_t SR;
    __IO uint32_t WINR;
} IWDG_TypeDef;

static IWDG_TypeDef _stub_iwdg;
#define IWDG (&_stub_iwdg)

typedef struct {
    uint32_t Prescaler;
    uint32_t Reload;
    uint32_t Window;
} IWDG_InitTypeDef;

typedef struct {
    IWDG_TypeDef     *Instance;
    IWDG_InitTypeDef  Init;
} IWDG_HandleTypeDef;

#define IWDG_PRESCALER_4   0x00000000U
#define IWDG_PRESCALER_8   0x00000001U
#define IWDG_PRESCALER_16  0x00000002U
#define IWDG_PRESCALER_32  0x00000003U
#define IWDG_PRESCALER_64  0x00000004U
#define IWDG_PRESCALER_128 0x00000005U
#define IWDG_PRESCALER_256 0x00000006U

#define IWDG_WINDOW_DISABLE 0x00000FFFU

/* ========================================================================= */
/*  FLASH Types and Constants                                                */
/* ========================================================================= */

typedef struct {
    uint32_t TypeErase;
    uint32_t Banks;
    uint32_t Page;
    uint32_t NbPages;
} FLASH_EraseInitTypeDef;

#define FLASH_TYPEERASE_PAGES     0x00000000U
#define FLASH_TYPEERASE_MASSERASE 0x00000001U

#define FLASH_BANK_1 0x00000001U
#define FLASH_BANK_2 0x00000002U

#define FLASH_TYPEPROGRAM_DOUBLEWORD 0x00000000U

#define FLASH_PAGE_SIZE 0x800U   /* 2 KB */

#define HAL_FLASH_ERROR_NONE 0x00000000U

#define FLASH_LATENCY_0  0x00000000U
#define FLASH_LATENCY_1  0x00000001U
#define FLASH_LATENCY_2  0x00000002U
#define FLASH_LATENCY_3  0x00000003U
#define FLASH_LATENCY_4  0x00000004U
#define FLASH_LATENCY_5  0x00000005U
#define FLASH_LATENCY_6  0x00000006U
#define FLASH_LATENCY_7  0x00000007U
#define FLASH_LATENCY_8  0x00000008U

/* ========================================================================= */
/*  RCC Types and Constants                                                  */
/* ========================================================================= */

typedef struct {
    __IO uint32_t CR;
    __IO uint32_t ICSCR;
    __IO uint32_t CFGR;
    __IO uint32_t PLLCFGR;
    uint32_t      RESERVED0;
    uint32_t      RESERVED1;
    __IO uint32_t CIER;
    __IO uint32_t CIFR;
    __IO uint32_t CICR;
    uint32_t      RESERVED2;
    __IO uint32_t AHB1RSTR;
    __IO uint32_t AHB2RSTR;
    __IO uint32_t AHB3RSTR;
    uint32_t      RESERVED3;
    __IO uint32_t APB1RSTR1;
    __IO uint32_t APB1RSTR2;
    __IO uint32_t APB2RSTR;
    uint32_t      RESERVED4;
    __IO uint32_t AHB1ENR;
    __IO uint32_t AHB2ENR;
    __IO uint32_t AHB3ENR;
    uint32_t      RESERVED5;
    __IO uint32_t APB1ENR1;
    __IO uint32_t APB1ENR2;
    __IO uint32_t APB2ENR;
    uint32_t      RESERVED6;
    __IO uint32_t AHB1SMENR;
    __IO uint32_t AHB2SMENR;
    __IO uint32_t AHB3SMENR;
    uint32_t      RESERVED7;
    __IO uint32_t APB1SMENR1;
    __IO uint32_t APB1SMENR2;
    __IO uint32_t APB2SMENR;
    uint32_t      RESERVED8;
    __IO uint32_t CCIPR;
    uint32_t      RESERVED9;
    __IO uint32_t BDCR;
    __IO uint32_t CSR;
    __IO uint32_t CRRCR;
    __IO uint32_t CCIPR2;
} RCC_TypeDef;

static RCC_TypeDef _stub_rcc;
#define RCC (&_stub_rcc)

/* RCC register bit positions used in system_stm32g4xx.c */
#define RCC_CFGR_SWS_Pos       3U
#define RCC_CFGR_SWS           (0x3UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_HPRE_Pos      4U
#define RCC_CFGR_HPRE          (0xFUL << RCC_CFGR_HPRE_Pos)
#define RCC_PLLCFGR_PLLSRC     (0x3UL << 0U)
#define RCC_PLLCFGR_PLLM_Pos   4U
#define RCC_PLLCFGR_PLLM       (0xFUL << RCC_PLLCFGR_PLLM_Pos)
#define RCC_PLLCFGR_PLLN_Pos   8U
#define RCC_PLLCFGR_PLLN       (0x7FUL << RCC_PLLCFGR_PLLN_Pos)
#define RCC_PLLCFGR_PLLR_Pos   25U
#define RCC_PLLCFGR_PLLR       (0x3UL << RCC_PLLCFGR_PLLR_Pos)

/* RCC CSR reset flags (used in Boot_ReadResetCause) */
#define RCC_CSR_LPWRRSTF  0x80000000U
#define RCC_CSR_WWDGRSTF  0x40000000U
#define RCC_CSR_IWDGRSTF  0x20000000U
#define RCC_CSR_SFTRSTF   0x10000000U
#define RCC_CSR_BORRSTF   0x02000000U
#define RCC_CSR_PINRSTF   0x04000000U

/* RCC oscillator configuration */
typedef struct {
    uint32_t OscillatorType;
    uint32_t HSEState;
    uint32_t LSEState;
    uint32_t HSIState;
    uint32_t HSICalibrationValue;
    uint32_t LSIState;
    struct {
        uint32_t PLLState;
        uint32_t PLLSource;
        uint32_t PLLM;
        uint32_t PLLN;
        uint32_t PLLP;
        uint32_t PLLQ;
        uint32_t PLLR;
    } PLL;
} RCC_OscInitTypeDef;

/* RCC clock configuration */
typedef struct {
    uint32_t ClockType;
    uint32_t SYSCLKSource;
    uint32_t AHBCLKDivider;
    uint32_t APB1CLKDivider;
    uint32_t APB2CLKDivider;
} RCC_ClkInitTypeDef;

/* RCC peripheral clock configuration */
typedef struct {
    uint32_t PeriphClockSelection;
    uint32_t FdcanClockSelection;
    uint32_t I2c1ClockSelection;
    uint32_t Adc12ClockSelection;
    uint32_t Usart1ClockSelection;
} RCC_PeriphCLKInitTypeDef;

/* RCC oscillator type */
#define RCC_OSCILLATORTYPE_NONE 0x00000000U
#define RCC_OSCILLATORTYPE_HSE  0x00000001U
#define RCC_OSCILLATORTYPE_HSI  0x00000002U
#define RCC_OSCILLATORTYPE_LSE  0x00000004U
#define RCC_OSCILLATORTYPE_LSI  0x00000008U

/* RCC HSI state */
#define RCC_HSI_OFF 0x00000000U
#define RCC_HSI_ON  0x00000100U

/* RCC HSI calibration */
#define RCC_HSICALIBRATION_DEFAULT 64U

/* RCC PLL state */
#define RCC_PLL_NONE 0x00000000U
#define RCC_PLL_OFF  0x00000001U
#define RCC_PLL_ON   0x00000002U

/* RCC PLL source */
#define RCC_PLLSOURCE_NONE 0x00000000U
#define RCC_PLLSOURCE_HSI  0x00000002U
#define RCC_PLLSOURCE_HSE  0x00000003U

/* RCC PLL dividers */
#define RCC_PLLM_DIV1  0x00000001U
#define RCC_PLLM_DIV2  0x00000002U
#define RCC_PLLM_DIV3  0x00000003U
#define RCC_PLLM_DIV4  0x00000004U
#define RCC_PLLM_DIV5  0x00000005U
#define RCC_PLLM_DIV6  0x00000006U
#define RCC_PLLM_DIV7  0x00000007U
#define RCC_PLLM_DIV8  0x00000008U

#define RCC_PLLP_DIV2  0x00000002U
#define RCC_PLLP_DIV7  0x00000007U
#define RCC_PLLP_DIV17 0x00000011U

#define RCC_PLLQ_DIV2 0x00000002U
#define RCC_PLLQ_DIV4 0x00000004U
#define RCC_PLLQ_DIV6 0x00000006U
#define RCC_PLLQ_DIV8 0x00000008U

#define RCC_PLLR_DIV2 0x00000002U
#define RCC_PLLR_DIV4 0x00000004U
#define RCC_PLLR_DIV6 0x00000006U
#define RCC_PLLR_DIV8 0x00000008U

/* RCC clock type */
#define RCC_CLOCKTYPE_SYSCLK 0x00000001U
#define RCC_CLOCKTYPE_HCLK   0x00000002U
#define RCC_CLOCKTYPE_PCLK1  0x00000004U
#define RCC_CLOCKTYPE_PCLK2  0x00000008U

/* RCC sysclk source */
#define RCC_SYSCLKSOURCE_HSI    0x00000000U
#define RCC_SYSCLKSOURCE_HSE    0x00000001U
#define RCC_SYSCLKSOURCE_PLLCLK 0x00000003U

/* RCC clock dividers */
#define RCC_SYSCLK_DIV1   0x00000000U
#define RCC_SYSCLK_DIV2   0x00000080U
#define RCC_HCLK_DIV1     0x00000000U
#define RCC_HCLK_DIV2     0x00000400U

/* RCC peripheral clock selection */
#define RCC_PERIPHCLK_FDCAN   0x00000001U
#define RCC_PERIPHCLK_I2C1    0x00000002U
#define RCC_PERIPHCLK_ADC12   0x00000004U
#define RCC_PERIPHCLK_USART1  0x00000008U

/* RCC FDCAN clock source */
#define RCC_FDCANCLKSOURCE_HSE   0x00000000U
#define RCC_FDCANCLKSOURCE_PLL   0x00000001U
#define RCC_FDCANCLKSOURCE_PCLK1 0x00000002U

/* RCC FDCAN clock source query (reads RCC_CCIPR.FDCANSEL bits [25:24]) */
#define RCC_CCIPR_FDCANSEL_Pos  24U
#define RCC_CCIPR_FDCANSEL_Msk  (0x3UL << RCC_CCIPR_FDCANSEL_Pos)
#define RCC_CCIPR_FDCANSEL      RCC_CCIPR_FDCANSEL_Msk

/* APB1ENR1 — FDCAN clock enable bit (bit 25) */
#define RCC_APB1ENR1_FDCANEN_Pos 25U
#define RCC_APB1ENR1_FDCANEN_Msk (0x1UL << RCC_APB1ENR1_FDCANEN_Pos)
#define RCC_APB1ENR1_FDCANEN     RCC_APB1ENR1_FDCANEN_Msk
#define __HAL_RCC_GET_FDCAN_SOURCE() \
    ((uint32_t)(RCC->CCIPR & RCC_CCIPR_FDCANSEL))

/* FDCAN CCCR register bit — INIT (bit 0) */
#define FDCAN_CCCR_INIT_Pos  0U
#define FDCAN_CCCR_INIT_Msk  (0x1UL << FDCAN_CCCR_INIT_Pos)
#define FDCAN_CCCR_INIT      FDCAN_CCCR_INIT_Msk

/* RCC clock enable macros */
#define __HAL_RCC_GPIOA_CLK_ENABLE()   ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE()   ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE()   ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE()   ((void)0)
#define __HAL_RCC_FDCAN_CLK_ENABLE()   ((void)0)
#define __HAL_RCC_FDCAN_FORCE_RESET()  ((void)0)
#define __HAL_RCC_FDCAN_RELEASE_RESET() ((void)0)
#define __HAL_RCC_FDCAN_CONFIG(source)  ((void)0)
#define __HAL_RCC_TIM1_CLK_ENABLE()    ((void)0)
#define __HAL_RCC_TIM2_CLK_ENABLE()    ((void)0)
#define __HAL_RCC_TIM3_CLK_ENABLE()    ((void)0)
#define __HAL_RCC_TIM8_CLK_ENABLE()    ((void)0)
#define __HAL_RCC_TIM15_CLK_ENABLE()   ((void)0)
#define __HAL_RCC_I2C1_CLK_ENABLE()    ((void)0)
#define __HAL_RCC_ADC12_CLK_ENABLE()   ((void)0)
#define __HAL_RCC_PWR_CLK_ENABLE()     ((void)0)
#define __HAL_RCC_SYSCFG_CLK_ENABLE()  ((void)0)
#define __HAL_RCC_CLEAR_RESET_FLAGS()  ((void)0)

/* ========================================================================= */
/*  PWR / SYSCFG constants                                                   */
/* ========================================================================= */

#define PWR_REGULATOR_VOLTAGE_SCALE1       0x00000001U
#define PWR_REGULATOR_VOLTAGE_SCALE1_BOOST 0x00000002U
#define PWR_REGULATOR_VOLTAGE_SCALE2       0x00000003U

#define __HAL_SYSCFG_BREAK_LOCKUP_LOCK() ((void)0)

/* ========================================================================= */
/*  HAL Core Function Stubs                                                  */
/* ========================================================================= */

static inline HAL_StatusTypeDef HAL_Init(void) { return HAL_OK; }

static inline uint32_t HAL_GetTick(void)
{
#ifdef HOST_TEST_GPIO_MODEL
    /* Opt-in deterministic tick controlled by the host test (declared by the
     * test TU) so relay-sequencer timing can be stepped precisely. */
    extern uint32_t g_stub_hal_tick;
    return g_stub_hal_tick;
#else
    static volatile uint32_t tick = 0;
    return tick++;
#endif
}

static inline void HAL_IncTick(void)    { }
static inline void HAL_Delay(uint32_t Delay) { (void)Delay; }

static inline HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    (void)TickPriority;
    return HAL_OK;
}

/* HAL_MspInit: real definition lives in stm32g4xx_hal_msp.c */
void HAL_MspInit(void);

static inline void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{
    (void)IRQn; (void)PreemptPriority; (void)SubPriority;
}

static inline void HAL_NVIC_EnableIRQ(IRQn_Type IRQn)  { (void)IRQn; }
static inline void HAL_NVIC_DisableIRQ(IRQn_Type IRQn) { (void)IRQn; }
static inline void HAL_NVIC_SystemReset(void)           { for(;;) { } }

static inline uint32_t HAL_RCC_GetSysClockFreq(void) { return 170000000U; }
static inline uint32_t HAL_RCC_GetHCLKFreq(void)     { return 170000000U; }

static inline HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct)
{
    (void)RCC_OscInitStruct;
    return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency)
{
    (void)RCC_ClkInitStruct; (void)FLatency;
    return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *PeriphClkInit)
{
    (void)PeriphClkInit;
    return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling)
{
    (void)VoltageScaling;
    return HAL_OK;
}

/* ========================================================================= */
/*  GPIO Function Stubs                                                      */
/* ========================================================================= */

static inline void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    (void)GPIOx; (void)GPIO_Init;
}

static inline void HAL_GPIO_DeInit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
    (void)GPIOx; (void)GPIO_Pin;
}

static inline void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
#ifdef HOST_TEST_GPIO_MODEL
#ifdef HOST_TEST_GPIO_WRITE_OBSERVER
    /* Opt-in write observer: lets a host test record EVERY commanded pin write
     * (including a transient SET later cleared in the same tick) so it can
     * prove, e.g., zero GPIO_PIN_SET writes ever reach PC12 after isolation.
     * The test TU provides HostGpioWriteObserver(); enabled only when the
     * HOST_TEST_GPIO_WRITE_OBSERVER macro is defined for the whole link. */
    extern void HostGpioWriteObserver(void *port, uint16_t pin, int is_set);
    HostGpioWriteObserver((void *)GPIOx, GPIO_Pin, (PinState != GPIO_PIN_RESET) ? 1 : 0);
#endif
    /* Opt-in functional model: reflect the commanded level in ODR so host
     * integration tests can observe pin state.  Off by default so every
     * other host test keeps the historical no-op behaviour. */
    if (PinState != GPIO_PIN_RESET) {
        GPIOx->ODR |= (uint32_t)GPIO_Pin;
    } else {
        GPIOx->ODR &= ~(uint32_t)GPIO_Pin;
    }
#else
    (void)GPIOx; (void)GPIO_Pin; (void)PinState;
#endif
}

static inline GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
#ifdef HOST_TEST_GPIO_MODEL
    /* Fold any pending atomic BSRR writes (reset-half only, as used by the
     * production relay/steer power-off paths) into ODR before reporting. */
    GPIOx->ODR |=  (GPIOx->BSRR & 0x0000FFFFu);
    GPIOx->ODR &= ~((GPIOx->BSRR >> 16) & 0x0000FFFFu);
    GPIOx->BSRR = 0U;
    return (GPIOx->ODR & (uint32_t)GPIO_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
#else
    (void)GPIOx; (void)GPIO_Pin;
    return GPIO_PIN_RESET;
#endif
}

static inline void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    (void)GPIOx; (void)GPIO_Pin;
}

static inline void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
}

/* ========================================================================= */
/*  ADC Function Stubs                                                       */
/* ========================================================================= */

static inline HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *hadc)
{
    (void)hadc; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *hadc)
{
    (void)hadc; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc)
{
    (void)hadc; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc)
{
    (void)hadc; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout)
{
    (void)hadc; (void)Timeout; return HAL_OK;
}

static inline uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc)
{
    (void)hadc; return 1500U;
}

static inline HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *sConfig)
{
    (void)hadc; (void)sConfig; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc, uint32_t SingleDiff)
{
    (void)hadc; (void)SingleDiff; return HAL_OK;
}

/* ADC MspInit callback — real definition in stm32g4xx_hal_msp.c */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc);

/* ========================================================================= */
/*  FDCAN Function Stubs                                                     */
/* ========================================================================= */

static inline HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_FDCAN_DeInit(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan, FDCAN_FilterTypeDef *sFilterConfig)
{
    (void)hfdcan; (void)sFilterConfig; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan,
    uint32_t NonMatchingStd, uint32_t NonMatchingExt, uint32_t RejectRemoteStd, uint32_t RejectRemoteExt)
{
    (void)hfdcan; (void)NonMatchingStd; (void)NonMatchingExt;
    (void)RejectRemoteStd; (void)RejectRemoteExt;
    return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
    FDCAN_TxHeaderTypeDef *pTxHeader, uint8_t *pTxData)
{
    (void)hfdcan; (void)pTxHeader; (void)pTxData; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan,
    uint32_t RxLocation, FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *pRxData)
{
    (void)hfdcan; (void)RxLocation; (void)pRxHeader; (void)pRxData;
    return HAL_OK;
}

static inline uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo)
{
    (void)hfdcan; (void)RxFifo; return 0U;
}

static inline uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan; return 3U;
}

static inline HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan,
    uint32_t ActiveITs, uint32_t BufferIndexes)
{
    (void)hfdcan; (void)ActiveITs; (void)BufferIndexes; return HAL_OK;
}

static inline void HAL_FDCAN_IRQHandler(FDCAN_HandleTypeDef *hfdcan) { (void)hfdcan; }

static inline HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *hfdcan,
    FDCAN_ProtocolStatusTypeDef *ProtocolStatus)
{
    (void)hfdcan;
    if (ProtocolStatus) { memset(ProtocolStatus, 0, sizeof(*ProtocolStatus)); }
    return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *hfdcan,
    FDCAN_ErrorCountersTypeDef *ErrorCounters)
{
    (void)hfdcan;
    if (ErrorCounters) { memset(ErrorCounters, 0, sizeof(*ErrorCounters)); }
    return HAL_OK;
}

/* FDCAN MspInit callback — real definition in stm32g4xx_hal_msp.c */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan);

/* ========================================================================= */
/*  I2C Function Stubs                                                       */
/* ========================================================================= */

static inline HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)hi2c; (void)DevAddress; (void)pData; (void)Size; (void)Timeout;
    return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)hi2c; (void)DevAddress; (void)pData; (void)Size; (void)Timeout;
    return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
    uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)hi2c; (void)DevAddress; (void)MemAddress; (void)MemAddSize;
    (void)pData; (void)Size; (void)Timeout;
    return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
    uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)hi2c; (void)DevAddress; (void)MemAddress; (void)MemAddSize;
    (void)pData; (void)Size; (void)Timeout;
    return HAL_OK;
}

static inline void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c)  { (void)hi2c; }
static inline void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c)  { (void)hi2c; }

static inline HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
{
    (void)hi2c; (void)DevAddress; (void)Trials; (void)Timeout;
    return HAL_OK;
}

/* I2C MspInit callback — real definition in stm32g4xx_hal_msp.c */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);

/* I2C Extended API stubs */
#define I2C_ANALOGFILTER_ENABLE     0x00000000U
#define I2C_ANALOGFILTER_DISABLE    0x00000001U

static inline HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c,
    uint32_t DigitalFilter)
{
    (void)hi2c; (void)DigitalFilter;
    return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c,
    uint32_t AnalogFilter)
{
    (void)hi2c; (void)AnalogFilter;
    return HAL_OK;
}

/* ========================================================================= */
/*  TIM Function Stubs                                                       */
/* ========================================================================= */

static inline HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim)
{
    (void)htim; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim)
{
    (void)htim; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim)
{
    (void)htim; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim)
{
    (void)htim; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    (void)htim; (void)Channel; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    (void)htim; (void)Channel; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim,
    TIM_OC_InitTypeDef *sConfig, uint32_t Channel)
{
    (void)htim; (void)sConfig; (void)Channel; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim,
    TIM_OC_InitTypeDef *sConfig, uint32_t Channel)
{
    (void)htim; (void)sConfig; (void)Channel; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim,
    TIM_Encoder_InitTypeDef *sConfig)
{
    (void)htim; (void)sConfig; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    (void)htim; (void)Channel; return HAL_OK;
}

static inline void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim) { (void)htim; }

static inline HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *htim,
    TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig)
{
    (void)htim; (void)sBreakDeadTimeConfig; return HAL_OK;
}

/* TIM MspInit/MspPostInit callbacks — real definitions in stm32g4xx_hal_msp.c */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* ========================================================================= */
/*  IWDG Function Stubs                                                      */
/* ========================================================================= */

static inline HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg)
{
    (void)hiwdg; return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg)
{
    (void)hiwdg; return HAL_OK;
}

/* ========================================================================= */
/*  FLASH Function Stubs                                                     */
/* ========================================================================= */

static inline HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
static inline HAL_StatusTypeDef HAL_FLASH_Lock(void)   { return HAL_OK; }

static inline HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
    (void)TypeProgram; (void)Address; (void)Data;
    return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError)
{
    (void)pEraseInit;
    if (PageError) { *PageError = 0xFFFFFFFFU; }
    return HAL_OK;
}

static inline uint32_t HAL_FLASH_GetError(void) { return HAL_FLASH_ERROR_NONE; }

/* ========================================================================= */
/*  stm32g4xx.h compatibility — included by system_stm32g4xx.c              */
/*  (This block provides everything system_stm32g4xx.c expects from          */
/*   stm32g4xx.h without requiring a separate file.)                         */
/* ========================================================================= */

/* AHB prescaler table (defined in system_stm32g4xx.c — declared here) */
extern const uint8_t AHBPrescTable[16];
extern const uint8_t APBPrescTable[8];

void SystemInit(void);
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
