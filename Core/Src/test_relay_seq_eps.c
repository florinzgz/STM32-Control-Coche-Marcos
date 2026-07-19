/**
  ****************************************************************************
  * @file    test_relay_seq_eps.c
  * @brief   Section-L integration test for the EFFECTIVE (compiled) relay
  *          sequencer.
  *
  *          This test links the REAL production translation units that the
  *          firmware actually builds:
  *
  *              - Core/Src/safety_system_patched.c  (the effective sequencer;
  *                includes safety_system.c and overrides Relay_SequencerUpdate
  *                and Safety_RelayOverrideUpdate)
  *              - Core/Src/steering_eps.c           (the EPS isolation authority)
  *
  *          Only the two hardware side effects of the EPS module
  *          (Steering_Neutralize) and the wider sensor/traction surface are
  *          stubbed.  Steering_SteerPowerOff() is the REAL implementation from
  *          safety_system.c so the PC12 write path is exercised end-to-end.
  *
  *          It reproduces the exact defect the PR targets: after an isolable
  *          EPS fault forces MECHANICAL_ONLY, the effective sequencer must
  *          keep PC12 (steering-motor isolation relay) OFF for the rest of the
  *          power cycle while traction (PC11, power-ready, ACTIVE state) is
  *          fully preserved.
  *
  *          Compile (from repository root):
  *            gcc -std=c11 -DHOST_TEST -DHOST_TEST_GPIO_MODEL -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  *                Core/Src/test_relay_seq_eps.c \
  *                Core/Src/safety_system_patched.c \
  *                Core/Src/steering_eps.c \
  *                -lm -o /tmp/test_relay_seq_eps && /tmp/test_relay_seq_eps
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

/* ---- Test-controlled steering health used by the sequencer gate. ---- */
static bool s_calibrated      = true;
static int  s_neutralize_calls = 0;
static int  s_estop_calls      = 0;

/* Convenience pin-state readers.  The stub GPIO structs are per-translation-
 * unit (declared `static` in the HAL stub), so we must observe the relay
 * command state through Safety_GetRelayStatusByte(), which reads GPIOC from
 * INSIDE the safety translation unit (bit1=PC11 TRAC, bit2=PC12 STEER). */
static bool pc12_on(void) { return (Safety_GetRelayStatusByte() & (1U << 2)) != 0U; }
static bool pc11_on(void) { return (Safety_GetRelayStatusByte() & (1U << 1)) != 0U; }

/* ==================================================================
 *  Stubs for the sensor/traction/service surface that safety_system.c
 *  references but which is irrelevant to the relay-sequencer contract.
 *  All are benign so no fault is ever raised by the safety loop itself.
 * ================================================================== */
void  Steering_Neutralize(void)        { s_neutralize_calls++; }      /* PA6=PA7=0, PC4 LOW */
void  Steering_PhysicalOff(void)       { s_neutralize_calls++; }      /* shared coast (steering_output.c) */
bool  Steering_IsCalibrated(void)      { return s_calibrated; }
float Steering_GetCurrentAngle(void)   { return 0.0f; }
void  Steering_SetAngle(float a)       { (void)a; }

float Wheel_GetSpeed_FL(void) { return 0.0f; }
float Wheel_GetSpeed_FR(void) { return 0.0f; }
float Wheel_GetSpeed_RL(void) { return 0.0f; }
float Wheel_GetSpeed_RR(void) { return 0.0f; }
bool  Wheel_IsStale(uint8_t idx)      { (void)idx; return false; }
uint8_t Wheel_GetGpioLevel(uint8_t idx) { (void)idx; return 0U; }

float Pedal_GetPercent(void)      { return 0.0f; }
bool  Pedal_IsPlausible(void)     { return true; }
bool  Pedal_IsContradictory(void) { return false; }

void  Traction_EmergencyStop(void)          { s_estop_calls++; }
void  Traction_SetDemand(float p)           { (void)p; }
float Traction_GetEffectiveDemandPct(void)  { return 0.0f; }
uint8_t Traction_GetFinalPwmPct(void)       { return 0U; }
GearPosition_t Traction_GetGear(void)       { return (GearPosition_t)0; }
void  Traction_SetGear(GearPosition_t g)    { (void)g; }
const TractionState_t* Traction_GetState(void) { static TractionState_t s; return &s; }

bool  ServiceMode_IsEnabled(ModuleID_t id)              { (void)id; return false; }
ModuleFault_t ServiceMode_GetFault(ModuleID_t id)       { (void)id; return (ModuleFault_t)0; }
void  ServiceMode_SetFault(ModuleID_t id, ModuleFault_t f) { (void)id; (void)f; }
void  ServiceMode_ClearFault(ModuleID_t id)             { (void)id; }

void  Encoder_CheckHealth(void) { }
bool  Encoder_HasFault(void)    { return false; }

float   Temperature_Get(uint8_t i)  { (void)i; return 25.0f; }
bool    Temperature_IsValid(uint8_t i) { (void)i; return true; }
float   Voltage_GetBus(uint8_t i)   { (void)i; return 12.5f; }
float   Current_GetAmps(uint8_t i)  { (void)i; return 0.0f; }
uint16_t Current_GetSampleAgeMs(void) { return 0U; }

uint8_t Sensor_GetInaOkMask(void)       { return 0xFFU; }
uint8_t Sensor_GetInaBusOkMask(void)    { return 0xFFU; }
uint8_t Sensor_GetInaExpectedMask(void) { return 0xFFU; }
bool    Sensor_GetMuxPresent(void)      { return true; }

bool  BootValidation_IsPassed(void) { return true; }bool  CAN_IsGlobalSilent(void)      { return false; }
bool  Startup_IsInhibited(void)     { return false; }
void  Startup_Rearm(void)           { }
bool  BatteryLimitsStore_Validate(const BatteryLimits_t *b) { (void)b; return true; }
RelayDiagReason_t Relay_ClassifyHealth(const RelayHealthDiag *d) { (void)d; return (RelayDiagReason_t)0; }
void  ErrorLog_Record(uint8_t code, uint8_t subsystem, uint8_t st, uint8_t flags) {
    (void)code; (void)subsystem; (void)st; (void)flags;
}

/* ==================================================================
 *  Test helpers
 * ================================================================== */

/* Drive the effective sequencer from BOOT to a fully powered ACTIVE state
 * with both relays energised and traction power-ready. */
static void bring_up_active_powered(void)
{
    g_stub_hal_tick = 0;

    s_calibrated       = true;
    s_neutralize_calls = 0;
    s_estop_calls      = 0;

    Safety_Init();
    Steering_EpsInit();
    /* Advance EPS through a clean self-check so it reports ACTIVE. */
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);
    /* Model the main loop's ownership arbitration: once out of homing the EPS
     * assist loop owns the steering motor (SteeringCentering_DecideOwner →
     * STEER_OWNER_EPS).  The PC12 gate is owner-aware, so this must be set for
     * the assist branch to authorise the relay. */
    Steering_EpsSetOwner(STEER_OWNER_EPS);

    Safety_SetState(SYS_STATE_STANDBY);
    Safety_SetState(SYS_STATE_ACTIVE);      /* calls Relay_PowerUp() → TRACTION_ON */

    /* Let the traction relay settle, then complete the sequence. */
    g_stub_hal_tick = 1000U;
    Relay_SequencerUpdate();                /* TRACTION_ON → COMPLETE */
}

int main(void)
{
    /* ---- Scenario 1: healthy EPS — sequencer energises PC12 ---- */
    bring_up_active_powered();
    CHECK(Safety_GetState() == SYS_STATE_ACTIVE);
    CHECK(Safety_IsPowerReady());
    CHECK(pc11_on());
    CHECK(pc12_on());                       /* assist allowed, relay closed  */
    CHECK(!Steering_IsMechanicalOnly());

    /* ---- Scenario 2: inject an isolable EPS fault (encoder A/B) ---- *
     * This is the exact acceptance contract: motor neutralised, PC12 OFF,
     * owner NONE, EPS MECHANICAL_ONLY, traction untouched.               */
    Steering_DisableAssistFault(EPS_FAULT_ENCODER_AB);

    CHECK(s_neutralize_calls > 0);          /* PA6=0, PA7=0, PC4 LOW delegated */
    CHECK(!pc12_on());                      /* PC12 OFF immediately            */
    CHECK(Steering_EpsGetOwner() == STEER_OWNER_NONE);
    CHECK(Steering_GetEpsState() == EPS_STATE_MECHANICAL_ONLY);
    CHECK(Steering_GetEpsFault() == EPS_FAULT_ENCODER_AB);

    /* Traction must be fully preserved at the instant of isolation. */
    CHECK(Safety_GetState() == SYS_STATE_ACTIVE);
    CHECK(Safety_IsPowerReady());
    CHECK(pc11_on());
    CHECK(s_estop_calls == 0);              /* traction never emergency-stopped */

    /* ---- Scenario 3: run the effective sequencer 200x ---- *
     * PC12 must stay OFF for the rest of the power cycle; PC11 / power-ready /
     * ACTIVE must remain untouched.  The previous Steering_IsCalibrated()-only
     * check re-energised PC12 here — s_calibrated stays true to prove the new
     * gate does NOT rely on calibration alone.  The engineering override tick
     * is intentionally NOT called here so this loop isolates the SEQUENCER. */
    for (int i = 0; i < 200; i++) {
        g_stub_hal_tick += 10U;
        Relay_SequencerUpdate();

        CHECK(!pc12_on());                  /* never reconnects              */
        CHECK(pc11_on());                   /* traction relay stays ON       */
        CHECK(Safety_IsPowerReady());       /* traction power-ready true      */
        CHECK(Safety_GetState() == SYS_STATE_ACTIVE);
        CHECK(Steering_IsMechanicalOnly()); /* EPS stays latched             */
        CHECK(Steering_EpsGetOwner() == STEER_OWNER_NONE);
    }

    /* ---- Scenario 3b: engineering relay-override tick cannot re-close PC12 --
     * The override path is a separate audited write.  Even if it were to try
     * to set the steering-motor relay, a latched EPS isolation must force it
     * back OFF, while the traction relay handling is left to the legacy path. */
    for (int i = 0; i < 50; i++) {
        g_stub_hal_tick += 10U;
        Safety_RelayOverrideUpdate();
        CHECK(!pc12_on());
        CHECK(pc11_on());
    }

    /* ---- Scenario 4: topology "normal isolation" sanity ---- *
     * With PC12 commanded OFF the steering-motor branch is isolated, yet the
     * INA226/CH5 supply (before the relay) is unaffected: reading a healthy
     * 12 V bus with zero branch current is NORMAL and must NOT re-enable the
     * assist.  We assert the firmware never spontaneously clears the latch.  */
    CHECK(Steering_GetEpsState() == EPS_STATE_MECHANICAL_ONLY);
    CHECK(!Steering_EpsIsAvailable());
    CHECK(!pc12_on());

    printf("==== test_relay_seq_eps: %d run, %d failed ====\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#else
int main(void) { return 0; }
#endif /* HOST_TEST */
