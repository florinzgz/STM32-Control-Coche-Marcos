/**
  ****************************************************************************
  * @file    test_eps_boot_traction.c
  * @brief   Section-M integration test proving that an isolable EPS fault at
  *          boot NEVER immobilises the vehicle, and that the engineering
  *          relay override can never pulse the steering-motor relay (PC12).
  *
  *          Links the REAL production translation units that ship in the
  *          firmware:
  *              - Core/Src/safety_system_patched.c  (effective sequencer +
  *                Safety_CheckCANTimeout / Safety_CheckEncoder / override)
  *              - Core/Src/steering_eps.c           (EPS isolation authority)
  *              - Core/Src/boot_validation.c        (real boot checklist)
  *
  *          Part 1 — EPS fault at boot keeps traction:
  *            healthy CAN · STANDBY · steering fault (centering / encoder A/B)
  *            · EPS MECHANICAL_ONLY · BootValidation still valid for traction
  *            · STANDBY → ACTIVE · PC11 ON · PC12 OFF · power_ready · pedal +
  *            traction operative.
  *
  *          Part 2 — override cannot pulse PC12:
  *            STANDBY · Safety_SetRelayOverride(true, 0x04) healthy · inject
  *            EPS fault · 100 updates · every HAL_GPIO_WritePin recorded ·
  *            zero GPIO_PIN_SET on PC12 after the latch · PC12 always OFF ·
  *            no pulses · no flapping.
  *
  *          Compile (from repository root):
  *            gcc -std=c11 -DHOST_TEST -DHOST_TEST_GPIO_MODEL \
  *                -DHOST_TEST_GPIO_WRITE_OBSERVER -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  *                Core/Src/test_eps_boot_traction.c \
  *                Core/Src/safety_system_patched.c \
  *                Core/Src/steering_eps.c \
  *                Core/Src/boot_validation.c \
  *                Core/Src/math_safety.c \
  *                -lm -o /tmp/test_eps_boot_traction && /tmp/test_eps_boot_traction
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

/* ---- Test-controlled steering / encoder health. ---- */
static bool s_calibrated   = true;
static bool s_encoder_fault = false;

/* ---- PC12 write observer (Part 2): record EVERY commanded pin write. ---- */
static int  s_pc12_set_writes = 0;   /* HAL_GPIO_WritePin(PC12, SET) count   */
static bool s_observe_enabled  = false;

void HostGpioWriteObserver(void *port, uint16_t pin, int is_set)
{
    (void)port;
    if (s_observe_enabled && pin == PIN_RELAY_STEER_PWR && is_set) {
        s_pc12_set_writes++;
    }
}

/* Relay command state observed through the safety translation unit's GPIOC
 * (bit1 = PC11 TRAC, bit2 = PC12 STEER). */
static bool pc12_on(void) { return (Safety_GetRelayStatusByte() & (1U << 2)) != 0U; }
static bool pc11_on(void) { return (Safety_GetRelayStatusByte() & (1U << 1)) != 0U; }

/* ==================================================================
 *  Stubs — benign so the safety loop itself raises no fault.  Sensor
 *  values are chosen so the REAL boot validation checklist passes.
 * ================================================================== */
void  Steering_Neutralize(void)       { }               /* PA6=PA7=0, PC4 LOW */
bool  Steering_IsCalibrated(void)     { return s_calibrated; }
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

/* Only the steering-encoder module is "enabled" so Safety_CheckEncoder()
 * runs; all other modules disabled so the boot temp/current loops skip. */
bool  ServiceMode_IsEnabled(ModuleID_t id)   { return (id == MODULE_STEER_ENCODER); }
ModuleFault_t ServiceMode_GetFault(ModuleID_t id) { (void)id; return (ModuleFault_t)0; }
void  ServiceMode_SetFault(ModuleID_t id, ModuleFault_t f) { (void)id; (void)f; }
void  ServiceMode_ClearFault(ModuleID_t id)  { (void)id; }

void  Encoder_CheckHealth(void) { }
bool  Encoder_HasFault(void)    { return s_encoder_fault; }

float   Temperature_Get(uint8_t i)  { (void)i; return 25.0f; }
bool    Temperature_IsValid(uint8_t i) { (void)i; return true; }
/* Battery bus (index 4) must be >= 20 V for the boot battery check. */
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

/* ==================================================================
 *  Helpers
 * ================================================================== */

/* Fresh boot into a healthy STANDBY with a valid boot validation and CAN. */
static void bring_up_standby_healthy(void)
{
    g_stub_hal_tick   = 0;
    s_calibrated      = true;
    s_encoder_fault   = false;
    s_pc12_set_writes = 0;
    s_observe_enabled = false;

    Safety_Init();
    Steering_EpsInit();
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);

    /* Safety_Init() does not reset the module-static relay sequencer state;
     * force a clean IDLE baseline so repeated scenarios in one process start
     * from the same power-down condition a real boot would have. */
    Relay_PowerDown();
    /* Flush the pending atomic BSRR reset the host GPIO model only folds into
     * ODR on the next read, so a later WritePin(SET) is not clobbered. */
    (void)Safety_GetRelayStatusByte();

    Safety_SetState(SYS_STATE_STANDBY);

    /* Fresh heartbeat so CAN is considered alive. */
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

/* Full Part-1 scenario for a given isolable fault injection. */
static void run_boot_traction_scenario(const char *label, bool via_encoder)
{
    (void)label;
    bring_up_standby_healthy();

    /* Baseline: healthy CAN, STANDBY, boot valid, assist available. */
    CHECK(Safety_GetState() == SYS_STATE_STANDBY);
    CHECK(BootValidation_IsPassed());
    CHECK(!Steering_IsMechanicalOnly());

    /* ---- Inject an isolable steering (EPS) fault ---- */
    if (via_encoder) {
        /* Encoder A/B fault flows through the REAL detector path:
         * Safety_CheckEncoder() → Encoder_CheckHealth() → isolation. */
        s_encoder_fault = true;
        Safety_CheckEncoder();
    } else {
        /* Centering fault: the homing FSM aborts and isolates the assist
         * (Centering_Abort → Steering_DisableAssistFault).  Modelled here by
         * the isolation the abort performs. */
        Steering_DisableAssistFault(EPS_FAULT_CENTER_NOT_FOUND);
    }

    /* EPS is now latched MECHANICAL_ONLY (isolable fault, no hazard). */
    CHECK(Steering_IsMechanicalOnly());
    CHECK(Steering_GetEpsState() == EPS_STATE_MECHANICAL_ONLY);
    CHECK(!Steering_EpsIsAvailable());

    /* Re-run boot validation: the isolated steering fault must NOT block it. */
    BootValidation_Run();
    CHECK(BootValidation_IsPassed());
    if (via_encoder) {
        /* The encoder health check still fails as a *diagnostic* even though
         * the overall validation passes. */
        CHECK((BootValidation_GetStatus()->checks_failed &
               BOOT_CHECK_ENCODER_HEALTHY) != 0U);
    }

    /* ---- STANDBY → ACTIVE via the normal promotion path ---- */
    g_stub_hal_tick += 10U;
    Safety_UpdateCANRxTime();
    Safety_CheckCANTimeout();

    CHECK(Safety_GetState() == SYS_STATE_ACTIVE);

    complete_power_sequence();

    /* Acceptance: traction fully available, steering purely mechanical. */
    CHECK(Safety_IsPowerReady());          /* power_ready true            */
    CHECK(pc11_on());                      /* PC11 (traction) ON          */
    CHECK(!pc12_on());                     /* PC12 (steering motor) OFF   */
    CHECK(Steering_IsMechanicalOnly());    /* assist stays isolated       */
    CHECK(Safety_IsCommandAllowed());      /* pedal + traction operative  */
    CHECK(Safety_ValidateThrottle(50.0f) > 0.0f);
}

int main(void)
{
    /* ================= Part 1: EPS fault at boot keeps traction ========= */
    run_boot_traction_scenario("centering fault", false);
    run_boot_traction_scenario("encoder A/B fault", true);

    /* ================= Part 2: override cannot pulse PC12 ================ */
    bring_up_standby_healthy();

    /* Enable the engineering relay override for the steering-motor relay bit
     * (0x04 / PC12) while the assist is still healthy.  This is the "healthy
     * operation" baseline: the override drives PC12 ON. */
    Safety_SetRelayOverride(true, 0x04);
    CHECK(Safety_IsRelayOverrideActive());

    g_stub_hal_tick += 10U;
    Safety_RelayOverrideUpdate();
    CHECK(pc12_on());                      /* healthy: PC12 driven ON      */

    /* Inject an isolable EPS fault → MECHANICAL_ONLY latched. */
    Steering_DisableAssistFault(EPS_FAULT_ENCODER_AB);
    CHECK(Steering_IsMechanicalOnly());

    /* From now on: record EVERY HAL_GPIO_WritePin and demand zero SET writes
     * on PC12.  Run 100 override updates. */
    s_observe_enabled = true;
    s_pc12_set_writes = 0;
    bool pc12_ever_on_after_latch = false;
    for (int i = 0; i < 100; i++) {
        g_stub_hal_tick += 10U;
        Safety_RelayOverrideUpdate();
        if (pc12_on()) pc12_ever_on_after_latch = true;
    }

    CHECK(s_pc12_set_writes == 0);         /* zero GPIO_PIN_SET on PC12    */
    CHECK(!pc12_ever_on_after_latch);      /* PC12 always OFF, no pulse     */

    /* A brand-new override command must also refuse to store the PC12 bit
     * while the EPS is latched — no re-arming a transient pulse. */
    Safety_SetRelayOverride(true, 0x04);
    s_pc12_set_writes = 0;
    for (int i = 0; i < 100; i++) {
        g_stub_hal_tick += 10U;
        Safety_RelayOverrideUpdate();
        if (pc12_on()) pc12_ever_on_after_latch = true;
    }
    CHECK(s_pc12_set_writes == 0);
    CHECK(!pc12_ever_on_after_latch);      /* still no flapping / no pulse  */

    printf("==== test_eps_boot_traction: %d run, %d failed ====\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#else
int main(void) { return 0; }
#endif /* HOST_TEST */
