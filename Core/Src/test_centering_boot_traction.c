/**
  ****************************************************************************
  * @file    test_centering_boot_traction.c
  * @brief   Section-M integration test proving that a REAL centering homing
  *          failure — driven through the genuine SteeringCentering_Step()
  *          finite-state machine, not a direct Steering_DisableAssistFault()
  *          call — isolates the EPS assist to MECHANICAL_ONLY while NEVER
  *          immobilising the vehicle.
  *
  *          Each scenario runs the actual homing FSM to one of its four
  *          abort causes and then verifies the full acceptance chain:
  *
  *              1. total timeout      (TOTAL_TIMEOUT_MS)
  *              2. encoder range      (MAX_CENTERING_COUNTS)
  *              3. end-of-travel stall (both sweeps stall, no centre)
  *              4. Encoder_HasFault() (A/B detector latched)
  *
  *          After each abort:
  *              EPS MECHANICAL_ONLY · owner NONE · PC12 OFF (and never re-armed)
  *              · BootValidation permits traction · STANDBY → ACTIVE · PC11 ON
  *              · power_ready true · pedal + traction operative.
  *
  *          Links the REAL production translation units:
  *              - Core/Src/steering_centering.c      (homing FSM under test)
  *              - Core/Src/steering_centering_diag.c (diag classifier)
  *              - Core/Src/steering_eps.c            (EPS isolation authority)
  *              - Core/Src/steering_output.c         (shared physical shutdown)
  *              - Core/Src/safety_system_patched.c   (effective safety loop)
  *              - Core/Src/boot_validation.c         (real boot checklist)
  *              - Core/Src/math_safety.c
  *
  *          Compile (from repository root):
  *            gcc -std=c11 -DHOST_TEST -DHOST_TEST_GPIO_MODEL \
  *                -DHOST_TEST_GPIO_WRITE_OBSERVER -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  *                Core/Src/test_centering_boot_traction.c \
  *                Core/Src/steering_centering.c \
  *                Core/Src/steering_centering_diag.c \
  *                Core/Src/steering_eps.c \
  *                Core/Src/steering_output.c \
  *                Core/Src/safety_system_patched.c \
  *                Core/Src/boot_validation.c \
  *                Core/Src/math_safety.c \
  *                -lm -o /tmp/test_centering_boot_traction && \
  *            /tmp/test_centering_boot_traction
  ****************************************************************************
  */

#ifdef HOST_TEST

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "safety_system.h"
#include "main.h"
#include "battery_limits_store.h"
#include "sensor_manager.h"
#include "motor_control.h"
#include "service_mode.h"
#include "boot_validation.h"
#include "error_log.h"
#include "can_handler.h"
#include "math_safety.h"
#include "relay_health_diag.h"
#include "steering_eps.h"
#include "steering_centering.h"
#include "project_config.h"

static int tests_run = 0, tests_failed = 0;
#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ---- Deterministic host tick consumed by the stub HAL_GetTick(). ---- */
uint32_t g_stub_hal_tick = 0;

/* CMSIS / peripheral-init externs referenced by boot_validation.c. */
uint32_t SystemCoreClock = 170000000U;
bool fdcan_init_ok = true;
bool i2c_init_ok   = true;

/* ---- Encoder / centering hardware model. ---- */
static bool     s_encoder_fault = false;   /* Encoder_HasFault() latch    */
static bool     s_center_found  = false;   /* PB5 inductive centre pulse  */

/* TIM2 = quadrature encoder counter; TIM3 = steering PWM (shared with the
 * real steering_output.c which externs htim3). */
static TIM_TypeDef  fake_tim2_regs;
static TIM_TypeDef  fake_tim3_regs;
TIM_HandleTypeDef   htim2;
TIM_HandleTypeDef   htim3;

/* ---- PC12 (steering-motor relay) observer -------------------------------
 * The homing FSM (steering_centering.c) energises PC12 with HAL_GPIO_WritePin
 * in its OWN translation unit, while the authoritative relay state lives in
 * the safety TU.  The write observer gives a single logical view so the test
 * can prove the rail was raised for homing and then never re-armed. */
static int  s_pc12_set_writes = 0;   /* HAL_GPIO_WritePin(PC12, SET) count */
static bool s_pc12_logical_on = false;

void HostGpioWriteObserver(void *port, uint16_t pin, int is_set)
{
    (void)port;
    if (pin == PIN_RELAY_STEER_PWR) {
        s_pc12_logical_on = (is_set != 0);
        if (is_set) {
            s_pc12_set_writes++;
        }
    }
}

/* Authoritative relay command state (safety translation unit's GPIOC). */
static bool pc12_on(void) { return (Safety_GetRelayStatusByte() & (1U << 2)) != 0U; }
static bool pc11_on(void) { return (Safety_GetRelayStatusByte() & (1U << 1)) != 0U; }

/* ==================================================================
 *  Stubs — safety-loop surface (chosen so the REAL boot validation and
 *  promotion logic run cleanly; the only fault is the homing failure).
 * ================================================================== */
void  Steering_Neutralize(void)       { }   /* Centering_Complete coast (unused on abort) */
bool  Steering_IsCalibrated(void)     { return false; } /* uncalibrated: homing is required */
float Steering_GetCurrentAngle(void)  { return 0.0f; }
void  Steering_SetAngle(float a)      { (void)a; }
void  Steering_SetCalibrated(void)    { }

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

/* Only the steering-encoder module is "enabled"; others disabled so the boot
 * temp/current loops skip. */
bool  ServiceMode_IsEnabled(ModuleID_t id)   { return (id == MODULE_STEER_ENCODER); }
ModuleFault_t ServiceMode_GetFault(ModuleID_t id) { (void)id; return (ModuleFault_t)0; }
void  ServiceMode_SetFault(ModuleID_t id, ModuleFault_t f) { (void)id; (void)f; }
void  ServiceMode_ClearFault(ModuleID_t id)  { (void)id; }

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
bool  BatteryLimitsStore_Validate(const BatteryLimits_t *b) { (void)b; return true; }
RelayDiagReason_t Relay_ClassifyHealth(const RelayHealthDiag *d) { (void)d; return (RelayDiagReason_t)0; }
void  ErrorLog_Record(uint8_t code, uint8_t subsystem, uint8_t st, uint8_t flags) {
    (void)code; (void)subsystem; (void)st; (void)flags;
}

/* ---- Centering hardware boundary ---- */
void  Motor_SetPWM_Steering(uint16_t pwm, bool reverse) { (void)pwm; (void)reverse; }
bool  SteeringCenter_Detected(void)  { return s_center_found; }
void  SteeringCenter_ClearFlag(void) { s_center_found = false; }

uint32_t Encoder_Z_GetPulseCount(void)   { return 0U; }
int32_t  Encoder_Z_GetLastPosition(void) { return 0; }
void SteeringZ_Init(void) { }
void SteeringZ_OnCenterConfirmed(int32_t c, int32_t z, bool seen) { (void)c; (void)z; (void)seen; }
int32_t SteeringZ_GetOffset(void) { return 0; }
bool    SteeringZ_IsValid(void)   { return false; }
bool SteeringCal_SaveWithZ(int32_t a, int32_t b, bool c, int32_t d)
{ (void)a; (void)b; (void)c; (void)d; return true; }
int32_t SteeringCal_GetStoredZOffset(void) { return 0; }
bool    SteeringCal_IsStoredZValid(void)   { return false; }
void SteeringZ_LoadFromFlash(int32_t off, bool valid) { (void)off; (void)valid; }

/* ==================================================================
 *  Helpers
 * ================================================================== */

static void set_encoder(int32_t v) { fake_tim2_regs.CNT = (uint32_t)v; }
static int32_t get_encoder(void)   { return (int32_t)fake_tim2_regs.CNT; }

/* Fresh boot into a healthy STANDBY with a valid boot validation and CAN,
 * the EPS healthy (not latched) and the homing FSM back at IDLE. */
static void bring_up_standby_healthy(void)
{
    g_stub_hal_tick   = 0;
    s_encoder_fault   = false;
    s_center_found    = false;
    s_pc12_set_writes = 0;
    s_pc12_logical_on = false;

    htim2.Instance = &fake_tim2_regs;
    htim3.Instance = &fake_tim3_regs;
    set_encoder(0);
    fake_tim3_regs.CCR1 = 0;
    fake_tim3_regs.CCR2 = 0;

    Safety_Init();
    Steering_EpsInit();
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);
    SteeringCentering_Init();

    Relay_PowerDown();
    (void)Safety_GetRelayStatusByte();   /* flush pending BSRR */

    Safety_SetState(SYS_STATE_STANDBY);

    g_stub_hal_tick = 100U;
    Safety_UpdateCANRxTime();

    BootValidation_Run();
}

/* Drive the effective relay sequencer to COMPLETE after an ACTIVE transition. */
static void complete_power_sequence(void)
{
    g_stub_hal_tick += 100U;      /* > RELAY_TRACTION_SETTLE_MS (50 ms) */
    Relay_SequencerUpdate();
}

/* Advance the homing FSM out of IDLE/WAIT_RAIL into the active LEFT sweep. */
static void enter_sweep(void)
{
    SteeringCentering_Step();            /* IDLE  → energise PC12, WAIT_RAIL */
    CHECK(s_pc12_set_writes >= 1);       /* homing raised the steering rail  */
    g_stub_hal_tick += 60U;              /* > STEERING_RAIL_SETTLE_MS (50)   */
    SteeringCentering_Step();            /* WAIT_RAIL → SWEEP_LEFT           */
    CHECK(SteeringCentering_GetState() == CENTERING_SWEEP_LEFT);
}

typedef enum { AB_TIMEOUT, AB_RANGE, AB_STALL, AB_ENCODER } AbortMode_t;

/* Run the REAL SteeringCentering_Step() FSM until it aborts via `mode`. */
static void drive_to_abort(AbortMode_t mode)
{
    enter_sweep();

    switch (mode) {

    case AB_TIMEOUT:
        /* Keep the encoder MOVING (no stall) and INSIDE range while time runs
         * out.  Oscillate ±1 around the sweep origin so neither the stall nor
         * the range guard trips before the absolute deadline. */
        for (int i = 0; i < 200 &&
             SteeringCentering_GetState() != CENTERING_FAULT; i++) {
            set_encoder((i & 1) ? 1 : 0);   /* changes every call: no stall  */
            g_stub_hal_tick += 200U;        /* 200 ms/step → >10 s total     */
            SteeringCentering_Step();
        }
        break;

    case AB_RANGE:
        /* Jump the encoder beyond MAX_CENTERING_COUNTS from the sweep origin
         * (0) — the range guard aborts before the deadline. */
        g_stub_hal_tick += 100U;
        set_encoder(6001);
        SteeringCentering_Step();
        break;

    case AB_STALL:
        /* Freeze the encoder: the LEFT sweep stalls (end-of-travel) and
         * reverses, then the RIGHT sweep stalls too → centre never found. */
        set_encoder(0);
        g_stub_hal_tick += 350U;            /* > STALL_TIMEOUT_MS (300)      */
        SteeringCentering_Step();           /* LEFT stall → reverse to RIGHT */
        CHECK(SteeringCentering_GetState() == CENTERING_SWEEP_RIGHT);
        g_stub_hal_tick += 350U;
        SteeringCentering_Step();           /* RIGHT stall → abort           */
        break;

    case AB_ENCODER:
        /* Encoder A/B detector latches a fault mid-sweep → immediate abort. */
        s_encoder_fault = true;
        SteeringCentering_Step();
        break;
    }

    CHECK(SteeringCentering_HasFault());
    CHECK(SteeringCentering_GetState() == CENTERING_FAULT);
}

/* Full item-4 acceptance for one homing-abort cause. */
static void run_centering_scenario(const char *label, AbortMode_t mode)
{
    (void)label;
    bring_up_standby_healthy();

    /* Baseline: healthy CAN, STANDBY, boot valid, assist NOT yet isolated. */
    CHECK(Safety_GetState() == SYS_STATE_STANDBY);
    CHECK(BootValidation_IsPassed());
    CHECK(!Steering_IsMechanicalOnly());

    /* ---- Run the genuine homing FSM to its abort ---- */
    drive_to_abort(mode);

    /* ---- Isolation invariants ---- */
    CHECK(Steering_IsMechanicalOnly());
    CHECK(Steering_GetEpsState() == EPS_STATE_MECHANICAL_ONLY);
    CHECK(!Steering_IsElectricalHazard());
    CHECK(Steering_EpsGetOwner() == STEER_OWNER_NONE);
    CHECK(!Steering_MotorRelayAllowed());       /* PC12 policy: forbidden     */
    CHECK(!pc12_on());                          /* PC12 OFF (authoritative)   */

    /* PC12 must not reappear: further FSM ticks stay FAULT and never re-arm
     * the steering-motor relay. */
    int writes_before = s_pc12_set_writes;
    SteeringCentering_Step();
    SteeringCentering_Step();
    CHECK(SteeringCentering_GetState() == CENTERING_FAULT);
    CHECK(s_pc12_set_writes == writes_before);  /* no new PC12 SET            */

    /* ---- BootValidation must still permit traction ---- */
    BootValidation_Run();
    CHECK(BootValidation_IsPassed());
    if (mode == AB_ENCODER) {
        /* Encoder health still fails as a *diagnostic* while the overall
         * validation passes (gated out because the assist is mechanical). */
        CHECK((BootValidation_GetStatus()->checks_failed &
               BOOT_CHECK_ENCODER_HEALTHY) != 0U);
    }

    /* ---- STANDBY → ACTIVE via the normal promotion path ---- */
    g_stub_hal_tick += 10U;
    Safety_UpdateCANRxTime();
    Safety_CheckCANTimeout();
    CHECK(Safety_GetState() == SYS_STATE_ACTIVE);

    complete_power_sequence();
    (void)Safety_GetRelayStatusByte();          /* flush pending BSRR         */

    /* ---- Acceptance: traction available, steering purely mechanical ---- */
    CHECK(Safety_IsPowerReady());               /* power_ready true           */
    CHECK(pc11_on());                           /* PC11 (traction) ON         */
    CHECK(!pc12_on());                          /* PC12 (steering motor) OFF  */
    CHECK(Steering_IsMechanicalOnly());         /* assist stays isolated      */
    CHECK(Safety_IsCommandAllowed());           /* pedal + traction operative */
    CHECK(Safety_ValidateThrottle(50.0f) > 0.0f);
    (void)get_encoder;
}

int main(void)
{
    run_centering_scenario("total timeout",  AB_TIMEOUT);
    run_centering_scenario("range exceeded", AB_RANGE);
    run_centering_scenario("end-of-travel stall", AB_STALL);
    run_centering_scenario("encoder A/B fault", AB_ENCODER);

    printf("==== test_centering_boot_traction: %d run, %d failed ====\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#else
int main(void) { return 0; }
#endif /* HOST_TEST */
