/**
  ****************************************************************************
  * @file    test_steering_outputs.c
  * @brief   Section-M output-production test: proves that an isolable EPS
  *          fault physically drives the four steering-assist actuators to
  *          their safe state — verified on REAL shared registers, not a
  *          neutralise call-counter.
  *
  *              CCR PA6 (TIM3_CH1) = 0
  *              CCR PA7 (TIM3_CH2) = 0
  *              PC4  (EN_STEER)    = LOW
  *              PC12 (STEER relay) = OFF
  *
  *          The fault is injected in the DETECTOR, never by calling
  *          Steering_DisableAssistFault() directly:
  *              Encoder_HasFault() → Safety_CheckEncoder() →
  *              Encoder_CheckHealth() → real isolation path.
  *
  *          The isolation drives the REAL shared production shutdown function
  *          Steering_PhysicalOff() (Core/Src/steering_output.c) — the exact
  *          register writes motor_control.c and steering_eps.c use in the
  *          firmware — so the assertions observe genuine production behaviour,
  *          not a re-implemented wrapper.  Steering_SteerPowerOff() and the
  *          relay command byte come from the REAL safety translation unit.
  *
  *          Compile (from repository root):
  *            gcc -std=c11 -DHOST_TEST -DHOST_TEST_GPIO_MODEL \
  *                -DHOST_TEST_GPIO_WRITE_OBSERVER -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  *                Core/Src/test_steering_outputs.c \
  *                Core/Src/steering_output.c \
  *                Core/Src/safety_system_patched.c \
  *                Core/Src/steering_eps.c \
  *                Core/Src/math_safety.c \
  *                -lm -o /tmp/test_steering_outputs && /tmp/test_steering_outputs
  ****************************************************************************
  */

#ifdef HOST_TEST

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"
#include "safety_system.h"
#include "main.h"
#include "battery_limits_store.h"
#include "sensor_manager.h"
#include "motor_control.h"
#include "service_mode.h"
#include "error_log.h"
#include "relay_health_diag.h"
#include "steering_eps.h"
#include "steering_output.h"
#include "project_config.h"

static int tests_run = 0, tests_failed = 0;
#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

uint32_t g_stub_hal_tick = 0;
uint32_t SystemCoreClock = 170000000U;

/* ---- Test-controlled encoder health (fault injected in the detector). ---- */
static bool s_encoder_fault = false;

/* ---- Shared actuator registers for the steering assist ----
 * TIM3_CH1 = PA6 (RPWM), TIM3_CH2 = PA7 (LPWM), PC4 = EN_STEER.
 * `htim3` is the SAME non-static global the production steering_output.c
 * externs, so __HAL_TIM_*_COMPARE in that real TU act on the CCR1/CCR2 this
 * test inspects (Instance is a shared pointer). */
TIM_HandleTypeDef htim3;

/* ---- Cross-TU PC4 (EN_STEER) observer ----
 * The real production Steering_PhysicalOff() lives in steering_output.c, a
 * DIFFERENT translation unit with its own per-TU GPIO shadow, so its PC4 write
 * is observed here through the write observer rather than a shared register. */
static bool s_pc4_level = false;   /* true = PC4 HIGH */

void HostGpioWriteObserver(void *port, uint16_t pin, int is_set)
{
    (void)port;
    if (pin == PIN_EN_STEER) {
        s_pc4_level = (is_set != 0);
    }
}

/* ==================================================================
 *  Steering_Neutralize() — production coast wrapper.
 *  Mirrors motor_control.c: delegates the physical actuator shutdown to the
 *  REAL shared production function Steering_PhysicalOff() (steering_output.c),
 *  which zeroes both TIM3 compares and drives EN_STEER (PC4) LOW.  No register
 *  writes are re-implemented in the test.
 * ================================================================== */
void Steering_Neutralize(void)
{
    Steering_PhysicalOff();
}

/* Register readers (shared TIM3 via htim3.Instance; PC4 via the observer). */
static uint32_t ccr_pa6(void) { return __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1); }
static uint32_t ccr_pa7(void) { return __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2); }
static bool     pc4_high(void){ return s_pc4_level; }

/* PC12 relay command observed through the safety translation unit. */
static bool pc12_on(void) { return (Safety_GetRelayStatusByte() & (1U << 2)) != 0U; }

/* ==================================================================
 *  Stubs — benign; the only real fault is the injected encoder one.
 * ================================================================== */
bool  Steering_IsCalibrated(void)     { return true; }
float Steering_GetCurrentAngle(void)  { return 0.0f; }
void  Steering_SetAngle(float a)      { (void)a; }

float Wheel_GetSpeed_FL(void) { return 0.0f; }
float Wheel_GetSpeed_FR(void) { return 0.0f; }
float Wheel_GetSpeed_RL(void) { return 0.0f; }
float Wheel_GetSpeed_RR(void) { return 0.0f; }
bool  Wheel_IsStale(uint8_t idx)        { (void)idx; return false; }
uint8_t Wheel_GetGpioLevel(uint8_t idx) { (void)idx; return 0U; }

float Pedal_GetPercent(void)      { return 0.0f; }
bool  Pedal_IsPlausible(void)     { return true; }
bool  Pedal_IsContradictory(void) { return false; }

void  Traction_EmergencyStop(void)          { }
void  Traction_SetDemand(float p)           { (void)p; }
float Traction_GetEffectiveDemandPct(void)  { return 0.0f; }
uint8_t Traction_GetFinalPwmPct(void)       { return 0U; }
GearPosition_t Traction_GetGear(void)       { return (GearPosition_t)0; }
void  Traction_SetGear(GearPosition_t g)    { (void)g; }
const TractionState_t* Traction_GetState(void) { static TractionState_t s; return &s; }

bool  ServiceMode_IsEnabled(ModuleID_t id)   { return (id == MODULE_STEER_ENCODER); }
ModuleFault_t ServiceMode_GetFault(ModuleID_t id) { (void)id; return (ModuleFault_t)0; }
void  ServiceMode_SetFault(ModuleID_t id, ModuleFault_t f) { (void)id; (void)f; }
void  ServiceMode_ClearFault(ModuleID_t id)  { (void)id; }

/* The detector: Encoder_CheckHealth() latches nothing on its own; the fault
 * state is provided by the test through Encoder_HasFault(). */
void  Encoder_CheckHealth(void) { }
bool  Encoder_HasFault(void)    { return s_encoder_fault; }

float   Temperature_Get(uint8_t i)  { (void)i; return 25.0f; }
bool    Temperature_IsValid(uint8_t i) { (void)i; return true; }
float   Voltage_GetBus(uint8_t i)   { return (i == 4U) ? 24.0f : 12.5f; }
float   Current_GetAmps(uint8_t i)  { (void)i; return 0.0f; }
uint16_t Current_GetSampleAgeMs(void) { return 0U; }

uint8_t Sensor_GetInaOkMask(void)       { return 0xFFU; }
uint8_t Sensor_GetInaBusOkMask(void)    { return 0xFFU; }
uint8_t Sensor_GetInaExpectedMask(void) { return 0xFFU; }
bool    Sensor_GetMuxPresent(void)      { return true; }

bool  CAN_IsGlobalSilent(void)  { return false; }
bool  Startup_IsInhibited(void) { return false; }
void  Startup_Rearm(void)       { }
bool  BootValidation_IsPassed(void) { return true; }
bool  BatteryLimitsStore_Validate(const BatteryLimits_t *b) { (void)b; return true; }
RelayDiagReason_t Relay_ClassifyHealth(const RelayHealthDiag *d) { (void)d; return (RelayDiagReason_t)0; }
void  ErrorLog_Record(uint8_t code, uint8_t subsystem, uint8_t st, uint8_t flags) {
    (void)code; (void)subsystem; (void)st; (void)flags;
}

/* ==================================================================
 *  Test
 * ================================================================== */

/* Drive the steering actuators to a non-safe "assist active" state so the
 * subsequent isolation has something real to tear down. */
static void arm_assist_outputs(void)
{
    htim3.Instance = TIM3;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 640U);  /* PA6 driving */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 480U);  /* PA7 driving */
    HAL_GPIO_WritePin(GPIOC, PIN_EN_STEER, GPIO_PIN_SET);      /* PC4 HIGH    */
}

int main(void)
{
    /* ---- Bring-up: healthy STANDBY, relay override drives PC12 ON. ---- */
    g_stub_hal_tick = 0;
    s_encoder_fault = false;

    Safety_Init();
    Steering_EpsInit();
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);

    Relay_PowerDown();
    (void)Safety_GetRelayStatusByte();          /* flush pending BSRR */

    Safety_SetState(SYS_STATE_STANDBY);
    g_stub_hal_tick = 100U;
    Safety_UpdateCANRxTime();

    /* Command PC12 ON (healthy assist rail) via the engineering override so
     * the isolation has a real relay to open. */
    Safety_SetRelayOverride(true, 0x04);
    g_stub_hal_tick += 10U;
    Safety_RelayOverrideUpdate();

    arm_assist_outputs();

    /* Baseline: assist outputs are actively driving and PC12 is ON. */
    CHECK(ccr_pa6() == 640U);
    CHECK(ccr_pa7() == 480U);
    CHECK(pc4_high());
    CHECK(pc12_on());
    CHECK(!Steering_IsMechanicalOnly());

    /* ---- Inject the fault in the DETECTOR and run the real check path. ---- */
    s_encoder_fault = true;
    Encoder_CheckHealth();      /* real detector tick */
    Safety_CheckEncoder();      /* real isolation path (NOT DisableAssistFault) */

    /* ---- Assist must now be physically neutralised on real registers. ---- */
    CHECK(Steering_IsMechanicalOnly());
    CHECK(ccr_pa6() == 0U);     /* CCR PA6 = 0 */
    CHECK(ccr_pa7() == 0U);     /* CCR PA7 = 0 */
    CHECK(!pc4_high());         /* PC4 = LOW   */
    CHECK(!pc12_on());          /* PC12 = OFF  */

    /* Idempotent re-check: a second detector pass keeps everything safe and
     * never re-energises the actuators. */
    Safety_CheckEncoder();
    CHECK(ccr_pa6() == 0U);
    CHECK(ccr_pa7() == 0U);
    CHECK(!pc4_high());
    CHECK(!pc12_on());

    printf("==== test_steering_outputs: %d run, %d failed ====\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#else
int main(void) { return 0; }
#endif /* HOST_TEST */
