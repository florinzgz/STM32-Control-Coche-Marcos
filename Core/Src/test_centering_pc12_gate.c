/**
  ****************************************************************************
  * @file    test_centering_pc12_gate.c
  * @brief   Integration test proving the owner-aware PC12 authorisation gate
  *          lets the REAL centering homing sequence keep the steering-motor
  *          relay (PC12) energised, driven through the EXACT main-loop order:
  *
  *              Relay_SequencerUpdate
  *              Safety_RelayOverrideUpdate
  *              Safety_SteerRelaySupervise      <-- runs BEFORE the FSM step
  *              SteeringCentering_DecideOwner
  *              SteeringCentering_Step
  *              SteeringSupervisor_Service
  *
  *          Because Safety_SteerRelaySupervise() runs BEFORE
  *          SteeringCentering_Step(), the old Steering_IsCalibrated()-only gate
  *          declared a policy violation and tore PC12 down every cycle during
  *          CENTERING_WAIT_RAIL (homing needs PC12 BEFORE it can calibrate).
  *          The owner-aware gate authorises PC12 while the CENTERING owner holds
  *          the motor in BOOT/STANDBY, so:
  *              - PC12 stays ON throughout homing (IDLE / WAIT_RAIL / sweep);
  *              - the sweep PWM begins with PC12 already ON;
  *              - a healthy homing run completes (CENTERING_DONE, calibrated);
  *              - ANY latched fault forces PC12 OFF and prevents every
  *                reconnection for the rest of the power cycle.
  *
  *          The same effective-wrapper test also proves two field invariants:
  *              - PB5 active-low completion still works when its optional
  *                service module is disabled and no EXTI edge is injected;
  *              - two fresh >=20 A CH5 samples cut PWM even while encoder counts
  *                continue changing, followed by the full 100 ms dead-time.
  *
  *          Per-TU host GPIO model: steering_centering.c commands PC12 in its
  *          OWN translation unit, while Safety_SteerRelaySupervise() reads PC12
  *          in the safety TU.  The write observer mirrors the centering FSM's
  *          PC12 command into the safety TU (Safety_TestInjectSteerRelayOn/Off)
  *          so the supervisor decides on the SAME PC12 the homing FSM raised —
  *          faithfully modelling the single shared GPIOC register on hardware.
  *
  *          Compile (from repository root):
  *            gcc -std=c11 -DHOST_TEST -DHOST_TEST_GPIO_MODEL \
  *                -DHOST_TEST_GPIO_WRITE_OBSERVER -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  *                Core/Src/test_centering_pc12_gate.c \
  *                Core/Src/steering_centering_patched.c \
  *                Core/Src/steering_centering_diag.c \
  *                Core/Src/steering_eps.c \
  *                Core/Src/steering_output.c \
  *                Core/Src/steering_supervisor_io.c \
  *                Core/Src/steering_supervisor.c \
  *                Core/Src/safety_system_patched.c \
  *                Core/Src/boot_validation.c \
  *                Core/Src/ina226_channel_diag.c \
  *                Core/Src/math_safety.c \
  *                -lm -o /tmp/test_centering_pc12_gate && \
  *            /tmp/test_centering_pc12_gate
  ****************************************************************************
  */

#ifdef HOST_TEST

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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
#include "steering_centering_diag.h"
#include "steering_supervisor.h"
#include "steering_z.h"
#include "ina226_channel_diag.h"
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

/* ---- Centering / encoder hardware model. ---- */
static bool     s_encoder_fault = false;   /* Encoder_HasFault() latch    */
static bool     s_center_found  = false;   /* PB5 inductive centre pulse  */
static bool     s_pb5_active    = false;   /* shared raw active-low level */
static bool     s_calibrated    = false;   /* Steering_IsCalibrated()     */

/* Strong host override for the weak hook in steering_centering_patched.c.
 * It avoids the HAL stub's intentional per-translation-unit GPIO copies. */
bool SteeringCentering_TestPb5Active(void) { return s_pb5_active; }

/* PWM instrumentation: capture start conditions and the latest command. */
static bool     s_pwm_started        = false;
static bool     s_pc12_on_at_pwm     = false;
static uint16_t s_last_steer_pwm     = 0U;
static bool     s_last_steer_reverse = false;

static TIM_TypeDef  fake_tim2_regs;
static TIM_TypeDef  fake_tim3_regs;
TIM_HandleTypeDef   htim2;
TIM_HandleTypeDef   htim3;

/* ---- PC12 write observer: mirror the centering FSM's PC12 command (raised in
 * its OWN TU) into the safety TU so Safety_SteerRelaySupervise() reads back the
 * SAME PC12 state.  A re-entrancy guard blocks the mirror's own write from
 * recursing back through the observer. */
static int  s_pc12_set_writes = 0;   /* HAL_GPIO_WritePin(PC12, SET) count */
static bool s_mirror_guard    = false;

/* Authoritative PC12 command state as seen by the safety TU (bit2). */
static bool pc12_on(void) { return (Safety_GetRelayStatusByte() & (1U << 2)) != 0U; }
static bool pc11_on(void) { return (Safety_GetRelayStatusByte() & (1U << 1)) != 0U; }

void HostGpioWriteObserver(void *port, uint16_t pin, int is_set)
{
    (void)port;
    if (pin != PIN_RELAY_STEER_PWR) return;
    if (s_mirror_guard) return;      /* ignore the mirror's own re-entrant write */
    if (is_set) s_pc12_set_writes++;
    s_mirror_guard = true;
    if (is_set) Safety_TestInjectSteerRelayOn();
    else        Safety_TestInjectSteerRelayOff();
    s_mirror_guard = false;
}

/* ==================================================================
 *  Stubs — benign so the safety loop raises no fault of its own.
 * ================================================================== */
void  Steering_Neutralize(void)       { }   /* PA6=PA7=0, PC4 LOW (Complete coast) */
bool  Steering_IsCalibrated(void)     { return s_calibrated; }
void  Steering_SetCalibrated(void)    { s_calibrated = true; }
float Steering_GetCurrentAngle(void)  { return 0.0f; }
void  Steering_SetAngle(float a)      { (void)a; }
/* owner!=CENTERING branch of the main loop calls the EPS control loop. */
void  Steering_ControlLoop(void)      { }

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

/* Intentionally leave MODULE_STEER_CENTER disabled.  PB5 homing must not depend
 * on this optional service-mode flag. */
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
void  Motor_SetPWM_Steering(uint16_t pwm, bool reverse)
{
    s_last_steer_pwm = pwm;
    s_last_steer_reverse = reverse;
    /* The homing sweep just commanded PWM: record whether PC12 was already ON
     * (the whole point of the gate fix). */
    if (pwm > 0U && !s_pwm_started) {
        s_pwm_started    = true;
        s_pc12_on_at_pwm = pc12_on();
    }
}
bool  SteeringCenter_Detected(void)  { return s_center_found; }
void  SteeringCenter_ClearFlag(void) { s_center_found = false; }

/* ---- Encoder-Z secondary reference (never blocks homing) ---- */
uint32_t Encoder_Z_GetPulseCount(void)   { return 0U; }
int32_t  Encoder_Z_GetLastPosition(void) { return 0; }
void SteeringZ_Init(void) { }
void SteeringZ_OnCenterConfirmed(int32_t c, int32_t z, bool seen) { (void)c; (void)z; (void)seen; }
int32_t SteeringZ_GetOffset(void) { return 0; }
bool    SteeringZ_IsValid(void)   { return false; }
void SteeringZ_LoadFromFlash(int32_t off, bool valid) { (void)off; (void)valid; }
SteeringZStatus_t SteeringZ_GetStatus(void) { return STEERING_Z_OK; }
bool SteeringCal_SaveWithZ(int32_t a, int32_t b, bool c, int32_t d)
{ (void)a; (void)b; (void)c; (void)d; return true; }
int32_t SteeringCal_GetStoredZOffset(void) { return 0; }
bool    SteeringCal_IsStoredZValid(void)   { return false; }

/* ---- EPS supervisor detector seams (all healthy so it never isolates) ---- */
bool EPS_Params_IsFlashValid(void)     { return true; }
bool EPS_Params_IsFlashCorrupt(void)   { return false; }
bool SteeringCal_IsFlashCorrupt(void)  { return false; }
bool SteeringCal_IsRestoredValid(void) { return true; }

/* Controllable healthy CH5 behind the real Sensor_GetChannel5Diag() seam. */
static Ina226ChannelDiag g_ch5;
const Ina226ChannelDiag *Sensor_GetChannel5Diag(void) { return &g_ch5; }

static void ch5_set_healthy(void)
{
    memset(&g_ch5, 0, sizeof(g_ch5));
    g_ch5.channel            = INA226_CHANNEL_STEER;
    g_ch5.expected_address   = 0x40;
    g_ch5.detected_address   = 0x40;
    g_ch5.mux_select_ok      = true;
    g_ch5.i2c_ack            = true;
    g_ch5.manufacturer_id_ok = true;
    g_ch5.die_id_ok          = true;
    g_ch5.config_write_ok    = true;
    g_ch5.config_readback_ok = true;
    g_ch5.shunt_read_ok      = true;
    g_ch5.bus_read_ok        = true;
    g_ch5.bus_mv             = 12400;
    g_ch5.signed_current_ma  = 0;
    g_ch5.sample_age_ms      = 10;
    g_ch5.sample_sequence    = 1;
    g_ch5.probe_sequence     = 1;
    g_ch5.channel_powered    = true;
    g_ch5.current_expected   = false;
    g_ch5.fault_reason       = INA226_CH_OK;
}

/* ==================================================================
 *  Helpers
 * ================================================================== */
static void set_encoder(int32_t v) { fake_tim2_regs.CNT = (uint32_t)v; }

/* Fresh boot into a healthy STANDBY with the EPS healthy (not latched) and the
 * homing FSM back at IDLE. */
static void bring_up_standby_healthy(void)
{
    g_stub_hal_tick      = 0;
    s_encoder_fault      = false;
    s_center_found       = false;
    s_pb5_active         = false;
    s_calibrated         = false;
    s_pwm_started        = false;
    s_pc12_on_at_pwm     = false;
    s_last_steer_pwm     = 0U;
    s_last_steer_reverse = false;
    s_pc12_set_writes    = 0;
    s_mirror_guard       = false;

    htim2.Instance = &fake_tim2_regs;
    htim3.Instance = &fake_tim3_regs;
    set_encoder(0);
    fake_tim3_regs.CCR1 = 0;
    fake_tim3_regs.CCR2 = 0;

    ch5_set_healthy();

    Safety_Init();
    Steering_EpsInit();
    Steering_EpsSetHealthyState(EPS_STATE_ACTIVE);
    SteeringCentering_Init();
    SteeringSupervisor_Init();

    Relay_PowerDown();
    (void)Safety_GetRelayStatusByte();   /* flush pending BSRR */
    Safety_TestInjectSteerRelayOff();    /* PC12 known OFF in the safety TU  */

    Safety_SetState(SYS_STATE_STANDBY);

    g_stub_hal_tick = 100U;
    Safety_UpdateCANRxTime();

    /* Counters must ignore any relay activity during bring-up. */
    s_pc12_set_writes = 0;
}

/* One 10 ms (100 Hz) main-loop pass in the EXACT production order.  Each new
 * cycle advances the real CH5 acquisition identity so the supervisor treats it
 * as a genuine fresh sample. */
static void main_cycle(void)
{
    g_stub_hal_tick += 10U;
    g_ch5.sample_sequence++;
    g_ch5.probe_sequence++;

    Relay_SequencerUpdate();
    Safety_RelayOverrideUpdate();
    Safety_SteerRelaySupervise();

    const SystemState_t st = Safety_GetState();
    const bool in_homing = (st == SYS_STATE_BOOT || st == SYS_STATE_STANDBY);
    const SteeringMotorOwner_t owner =
        SteeringCentering_DecideOwner(SteeringCentering_GetState(), in_homing);

    if (owner == STEER_OWNER_CENTERING) {
        SteeringCentering_Step();
    } else {
        Steering_ControlLoop();
    }
    SteeringCentering_UpdateDiag();
    SteeringSupervisor_Service();
}

static void advance_to_left_sweep(void)
{
    int guard = 0;
    while (!s_pwm_started && guard++ < 50) main_cycle();
    CHECK(s_pwm_started);
    CHECK(SteeringCentering_GetState() == CENTERING_SWEEP_LEFT);
}

/* ==================================================================
 *  Test 1: healthy homing keeps PC12 ON and completes.
 * ================================================================== */
static void test_homing_keeps_pc12_on_and_completes(void)
{
    bring_up_standby_healthy();

    /* Drive the main order until the sweep PWM begins (past RAIL settle).  Each
     * cycle PC12 must remain ON — Safety_SteerRelaySupervise() must NOT tear the
     * rail down while centering owns the motor in STANDBY. */
    int guard = 0;
    while (!s_pwm_started && guard++ < 50) {
        main_cycle();
        /* Once homing has raised PC12 (IDLE step done) it must stay ON. */
        if (s_pc12_set_writes > 0) {
            CHECK(pc12_on());
        }
    }
    CHECK(s_pwm_started);                 /* the sweep actually started        */
    CHECK(s_pc12_set_writes >= 1);        /* homing raised PC12                 */
    CHECK(s_pc12_on_at_pwm);              /* PWM began with PC12 already ON      */
    CHECK(pc12_on());                     /* still ON entering the sweep         */
    CHECK(SteeringCentering_GetState() == CENTERING_SWEEP_LEFT);
    CHECK(!Steering_IsMechanicalOnly());  /* assist never isolated by homing     */

    /* Present the centre pulse: the next FSM step completes homing. */
    s_center_found = true;
    main_cycle();
    CHECK(SteeringCentering_GetState() == CENTERING_DONE);
    CHECK(Steering_IsCalibrated());       /* calibration unlocked                */
    CHECK(!Steering_IsMechanicalOnly());

    /* A few post-completion cycles: EPS now owns the motor and, being
     * calibrated + available, PC12 stays authorised (no violation tear-down). */
    for (int i = 0; i < 5; i++) {
        main_cycle();
        CHECK(pc12_on());
    }
    CHECK(pc11_on() || Safety_GetState() == SYS_STATE_STANDBY); /* traction untouched */
}

/* ==================================================================
 *  Test 2: a latched fault during homing forces PC12 OFF forever.
 * ================================================================== */
static void test_latched_fault_kills_pc12_forever(void)
{
    bring_up_standby_healthy();

    /* Start the sweep exactly as before (PC12 ON). */
    advance_to_left_sweep();
    CHECK(pc12_on());

    /* Latch an encoder A/B fault: the REAL homing FSM aborts → isolates the
     * assist (MECHANICAL_ONLY) → PC12 OFF. */
    s_encoder_fault = true;
    main_cycle();
    CHECK(SteeringCentering_GetState() == CENTERING_FAULT);
    CHECK(Steering_IsMechanicalOnly());
    CHECK(!Steering_IsElectricalHazard());
    CHECK(!Steering_MotorRelayAllowed());   /* policy now forbids PC12           */
    CHECK(!pc12_on());                       /* PC12 OFF (authoritative)          */

    /* No reconnection: many further cycles must never re-arm PC12. */
    int writes_before = s_pc12_set_writes;
    for (int i = 0; i < 60; i++) {
        main_cycle();
        CHECK(!pc12_on());
    }
    CHECK(s_pc12_set_writes == writes_before);  /* zero new PC12 SET writes       */
    CHECK(Steering_IsMechanicalOnly());          /* still latched                  */
}

/* ==================================================================
 *  Test 3: PB5 level remains mandatory even when service mode disables it.
 * ================================================================== */
static void test_pb5_level_ignores_optional_service_disable(void)
{
    bring_up_standby_healthy();
    CHECK(!ServiceMode_IsEnabled(MODULE_STEER_CENTER));
    advance_to_left_sweep();

    /* No EXTI/event flag: only the raw active-low level is presented. */
    s_center_found = false;
    s_pb5_active = true;

    main_cycle();
    CHECK(SteeringCentering_GetState() == CENTERING_SWEEP_LEFT);
    main_cycle();
    CHECK(SteeringCentering_GetState() == CENTERING_SWEEP_LEFT);
    main_cycle();

    CHECK(SteeringCentering_GetState() == CENTERING_DONE);
    CHECK(Steering_IsCalibrated());
    CHECK(!Steering_IsMechanicalOnly());
}

/* ==================================================================
 *  Test 4: two fresh >=20 A samples cut even while encoder still changes.
 * ================================================================== */
static void test_hard_current_cuts_while_encoder_moves(void)
{
    bring_up_standby_healthy();
    advance_to_left_sweep();

    /* Prime the end-stop observation in the current sweep with a normal sample. */
    set_encoder(5);
    g_ch5.signed_current_ma = 0;
    main_cycle();
    CHECK(SteeringCentering_GetState() == CENTERING_SWEEP_LEFT);

    /* First hard-current acquisition while encoder moves: reject as a lone spike. */
    set_encoder(10);
    g_ch5.signed_current_ma = 21000;
    main_cycle();
    CHECK(SteeringCentering_GetState() == CENTERING_SWEEP_LEFT);
    CHECK(SteeringCentering_GetDiag()->pwm_requested == 4249U);

    /* Second genuinely fresh hard-current acquisition while it STILL moves:
     * torque must be zeroed immediately and reversal dead-time must begin. */
    set_encoder(15);
    main_cycle();
    CHECK(SteeringCentering_GetState() == CENTERING_SWEEP_LEFT);
    CHECK(SteeringCentering_GetDiag()->pwm_requested == 0U);
    CHECK(s_last_steer_pwm == 0U);

    /* 90 ms is still inside the mandatory 100 ms zero-output interval. */
    for (int i = 0; i < 9; ++i) {
        set_encoder(20 + i * 5);
        main_cycle();
        CHECK(SteeringCentering_GetState() == CENTERING_SWEEP_LEFT);
        CHECK(SteeringCentering_GetDiag()->pwm_requested == 0U);
        CHECK(s_last_steer_pwm == 0U);
    }

    /* At/after 100 ms the right sweep may start, but never earlier. */
    set_encoder(70);
    main_cycle();
    CHECK(SteeringCentering_GetState() == CENTERING_SWEEP_RIGHT);
    CHECK(SteeringCentering_GetDiag()->pwm_requested == 4249U);
    CHECK(s_last_steer_pwm == 4249U);
    CHECK(!s_last_steer_reverse);  /* base right-sweep convention */
}

int main(void)
{
    test_homing_keeps_pc12_on_and_completes();
    test_latched_fault_kills_pc12_forever();
    test_pb5_level_ignores_optional_service_disable();
    test_hard_current_cuts_while_encoder_moves();

    printf("==== test_centering_pc12_gate: %d run, %d failed ====\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#else
int main(void) { return 0; }
#endif /* HOST_TEST */
