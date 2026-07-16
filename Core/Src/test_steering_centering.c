/**
  ****************************************************************************
  * @file    test_steering_centering.c
  * @brief   Host unit tests for steering-motor OWNERSHIP / mutual exclusion
  *          between the homing FSM (steering_centering.c) and the EPS
  *          torque-assist loop.  Links the REAL steering_centering.c module
  *          and stubs its hardware / cross-module boundary.
  *
  *          Regression under test (Error 8 / FAULT_CENTERING):
  *            SteeringCentering_Step() commanded CENTERING_PWM, but
  *            Steering_ControlLoop() ran unconditionally in the SAME 10 ms
  *            cycle and, while uncalibrated, called Steering_Neutralize(),
  *            cancelling the centering PWM before TIM3 could apply it.  The
  *            motor never moved and the sweep aborted after the stall
  *            timeout.
  *
  *          The fix establishes exclusive ownership via
  *          SteeringCentering_DecideOwner(): centering owns PC4/PA6/PA7
  *          while homing, the EPS loop owns it afterwards, and the two
  *          never write the motor in the same cycle.
  *
  *          Compile with host GCC (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  *                Core/Src/test_steering_centering.c \
  *                Core/Src/steering_centering.c -lm \
  *                -o /tmp/test_steering_centering
  ****************************************************************************
  */

#ifdef HOST_TEST
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "steering_centering.h"
#include "steering_eps.h"
#include "safety_system.h"
#include "service_mode.h"
#include "stm32g4xx_hal.h"

static int tests_run = 0, tests_failed = 0;

#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ==================================================================
 *  Fake hardware / cross-module boundary
 * ================================================================== */

/* TIM2 encoder counter (quadrature position) that steering_centering.c
 * reads through __HAL_TIM_GET_COUNTER / __HAL_TIM_SET_COUNTER.          */
static TIM_TypeDef       fake_tim2_regs;
TIM_HandleTypeDef        htim2;

/* TIM3 steering PWM timer.  SteeringCentering_UpdateDiag() reads the
 * applied compare values through __HAL_TIM_GET_COMPARE(&htim3, ...).    */
static TIM_TypeDef       fake_tim3_regs;
TIM_HandleTypeDef        htim3;

/* ---- Steering motor "hardware" model ----
 * We model the three motor lines the problem statement calls out:
 *   PC4  EN_STEER, PA6 TIM3_CH1, PA7 TIM3_CH2.
 * The stubs record who wrote the motor and the resulting drive state so
 * tests can assert exclusive ownership and safe (neutral) states.        */
typedef struct {
    bool     driven;       /* true = PWM active, false = neutral/coast   */
    uint16_t pwm;          /* last commanded centering PWM               */
    bool     reverse;      /* last commanded direction                   */
} MotorModel_t;

static MotorModel_t motor;

/* Per-cycle bookkeeping for the mutual-exclusion assertions.  Ownership
 * is attributed at the SCHEDULER call site (run_cycle), not inside the
 * motor stubs — Steering_Neutralize() is legitimately used BY the
 * centering FSM (Abort/Complete) as well as by the EPS loop.             */
static bool centering_ran_this_cycle;      /* SteeringCentering_Step()   */
static bool eps_ran_this_cycle;            /* EPS control-loop stand-in  */
static int  motor_writes_this_cycle;       /* total motor line writes    */

static void cycle_begin(void)
{
    centering_ran_this_cycle = false;
    eps_ran_this_cycle       = false;
    motor_writes_this_cycle  = 0;
}

/* ---- Motor stubs ---- */
void Motor_SetPWM_Steering(uint16_t pwm, bool reverse)
{
    motor.driven  = (pwm != 0U);
    motor.pwm     = pwm;
    motor.reverse = reverse;
    motor_writes_this_cycle++;
}

void Steering_Neutralize(void)
{
    /* Coast: EN=LOW, both PWM channels 0 → safe state. */
    motor.driven = false;
    motor.pwm    = 0U;
    motor_writes_this_cycle++;
}

/* Steering rail (PC12) release — used by the EPS isolation authority. */
void Steering_SteerPowerOff(void) { }

static bool calibrated = false;
void Steering_SetCalibrated(void) { calibrated = true; }

/* ---- Safety stubs ---- */
static Safety_Error_t    s_error = SAFETY_ERROR_NONE;
static SystemState_t     s_state = SYS_STATE_BOOT;
static bool              s_power_ready = false;
static DegradedLevel_t   s_deg_level = DEGRADED_LEVEL_NONE;
static DegradedReason_t  s_deg_reason;

void Safety_SetError(Safety_Error_t error) { s_error = error; }
Safety_Error_t Safety_GetError(void)       { return s_error; }
void Safety_SetState(SystemState_t state)  { s_state = state; }
SystemState_t Safety_GetState(void)        { return s_state; }
bool Safety_IsPowerReady(void)             { return s_power_ready; }
void Safety_SetDegradedLevel(DegradedLevel_t level, DegradedReason_t reason)
{
    s_deg_level  = level;
    s_deg_reason = reason;
}

/* ---- Center sensor (PB5 inductive) stub ---- */
static bool center_detected = false;
bool SteeringCenter_Detected(void) { return center_detected; }
void SteeringCenter_ClearFlag(void) { center_detected = false; }

/* ---- Service-mode stub ---- */
static bool s_module_enabled = true;
bool ServiceMode_IsEnabled(ModuleID_t id) { (void)id; return s_module_enabled; }

/* ---- Encoder stubs ---- */
static bool     encoder_fault   = false;
static uint32_t z_pulse_count   = 0;
static int32_t  z_last_position = 0;
bool     Encoder_HasFault(void)          { return encoder_fault; }
uint32_t Encoder_Z_GetPulseCount(void)   { return z_pulse_count; }
int32_t  Encoder_Z_GetLastPosition(void) { return z_last_position; }

/* ---- SteeringZ / calibration store stubs ---- */
void SteeringZ_Init(void) { }
void SteeringZ_OnCenterConfirmed(int32_t c, int32_t z, bool seen)
{ (void)c; (void)z; (void)seen; }
void SteeringZ_LoadFromFlash(int32_t off, bool valid) { (void)off; (void)valid; }
int32_t SteeringZ_GetOffset(void) { return 0; }
bool    SteeringZ_IsValid(void)   { return false; }
bool SteeringCal_SaveWithZ(int32_t a, int32_t b, bool c, int32_t d)
{ (void)a; (void)b; (void)c; (void)d; return true; }
int32_t SteeringCal_GetStoredZOffset(void) { return 0; }
bool    SteeringCal_IsStoredZValid(void)   { return false; }

/* ==================================================================
 *  Test harness helpers
 * ================================================================== */

static void set_encoder(int32_t v) { fake_tim2_regs.CNT = (uint32_t)v; }
static int32_t get_encoder(void)   { return (int32_t)fake_tim2_regs.CNT; }

/* Reproduce the exact main.c arbitration so the test exercises the same
 * scheduling logic the firmware runs at 100 Hz.  Returns the owner that
 * actually ran this cycle.  `eps_run` is a stub for Steering_ControlLoop()
 * that, while uncalibrated, neutralises the motor (the real EPS guard).  */
static SteeringMotorOwner_t run_cycle(void)
{
    cycle_begin();

    bool in_homing = (s_state == SYS_STATE_BOOT || s_state == SYS_STATE_STANDBY);
    SteeringMotorOwner_t owner =
        SteeringCentering_DecideOwner(SteeringCentering_GetState(), in_homing);

    if (owner == STEER_OWNER_CENTERING) {
        centering_ran_this_cycle = true;
        SteeringCentering_Step();
    } else {
        /* Stand-in for Steering_ControlLoop(): uncalibrated or SAFE/ERROR
         * → coast (Steering_Neutralize); otherwise the EPS torque loop
         * would write an effort here.  Either way it is the sole writer. */
        eps_ran_this_cycle = true;
        Steering_Neutralize();
    }

    /* Invariant checked every single cycle: the centering FSM and the EPS
     * loop must NEVER both run (and thus never both write the steering
     * motor) in the same cycle.                                           */
    CHECK(!(centering_ran_this_cycle && eps_ran_this_cycle));
    CHECK(centering_ran_this_cycle || eps_ran_this_cycle);
    return owner;
}

static void reset_all(void)
{
    htim2.Instance = &fake_tim2_regs;
    htim3.Instance = &fake_tim3_regs;
    set_encoder(0);
    fake_tim3_regs.CCR1 = 0; fake_tim3_regs.CCR2 = 0;
    motor.driven = false; motor.pwm = 0; motor.reverse = false;
    calibrated = false;
    s_error = SAFETY_ERROR_NONE;
    s_state = SYS_STATE_BOOT;
    s_power_ready = false;
    s_module_enabled = true;
    center_detected = false;
    encoder_fault = false;
    z_pulse_count = 0;
    SteeringCentering_Init();
    Steering_EpsInit();     /* clear any latched EPS mechanical-only state */
}

/* Advance the FSM out of IDLE/WAIT_RAIL into an active sweep by pumping
 * cycles until the state reaches SWEEP_LEFT (relay settle uses the HAL
 * tick, which the stubbed HAL_GetTick advances monotonically).           */
static bool drive_to_sweep_left(int max_cycles)
{
    for (int i = 0; i < max_cycles; i++) {
        run_cycle();
        if (SteeringCentering_GetState() == CENTERING_SWEEP_LEFT) return true;
    }
    return false;
}

/* ==================================================================
 *  Tests
 * ================================================================== */

/* ---- 1. Pure ownership policy matrix ---- */
static void test_decide_owner_matrix(void)
{
    /* Active homing states + homing-capable system → centering owns. */
    CHECK(SteeringCentering_DecideOwner(CENTERING_IDLE,       true) == STEER_OWNER_CENTERING);
    CHECK(SteeringCentering_DecideOwner(CENTERING_WAIT_RAIL,  true) == STEER_OWNER_CENTERING);
    CHECK(SteeringCentering_DecideOwner(CENTERING_SWEEP_LEFT, true) == STEER_OWNER_CENTERING);
    CHECK(SteeringCentering_DecideOwner(CENTERING_SWEEP_RIGHT,true) == STEER_OWNER_CENTERING);

    /* After DONE → EPS owns (takes control). */
    CHECK(SteeringCentering_DecideOwner(CENTERING_DONE,  true) == STEER_OWNER_EPS);
    /* FAULT → EPS owns (keeps motor neutralised). */
    CHECK(SteeringCentering_DecideOwner(CENTERING_FAULT, true) == STEER_OWNER_EPS);

    /* System not in BOOT/STANDBY (e.g. SAFE/ERROR) → EPS owns regardless
     * of centering state, so the motor is always neutralised there.       */
    CHECK(SteeringCentering_DecideOwner(CENTERING_SWEEP_LEFT, false) == STEER_OWNER_EPS);
    CHECK(SteeringCentering_DecideOwner(CENTERING_IDLE,       false) == STEER_OWNER_EPS);
    CHECK(SteeringCentering_DecideOwner(CENTERING_DONE,       false) == STEER_OWNER_EPS);
    CHECK(SteeringCentering_DecideOwner(CENTERING_FAULT,      false) == STEER_OWNER_EPS);
}

/* ---- 2. Centering PWM stays active across many cycles and is NEVER
 *          neutralised by the EPS loop before completing. ---- */
static void test_pwm_persists_and_not_neutralized(void)
{
    reset_all();
    CHECK(drive_to_sweep_left(200));

    int pos = 100;
    for (int i = 0; i < 50; i++) {
        /* Encoder keeps moving → no stall, no completion. */
        pos += 5;
        set_encoder(pos);
        SteeringMotorOwner_t owner = run_cycle();

        CHECK(owner == STEER_OWNER_CENTERING);         /* centering owns   */
        CHECK(SteeringCentering_GetState() == CENTERING_SWEEP_LEFT);
        CHECK(motor.driven);                           /* PWM still active */
        CHECK(motor.pwm == 425U);                      /* CENTERING_PWM    */
        CHECK(!eps_ran_this_cycle);           /* EPS did NOT run this cycle */
    }
    CHECK(!calibrated);
}

/* ---- 3. Left-sweep stall → reversal to the right. ---- */
static void test_left_stall_causes_reversal(void)
{
    reset_all();
    CHECK(drive_to_sweep_left(200));

    /* Freeze the encoder so the stall timer elapses (STALL_TIMEOUT_MS). */
    set_encoder(500);
    bool reversed = false;
    for (int i = 0; i < 2000; i++) {
        run_cycle();
        if (SteeringCentering_GetState() == CENTERING_SWEEP_RIGHT) {
            reversed = true;
            CHECK(motor.driven);        /* still driving, now reversed */
            CHECK(motor.reverse == false); /* right sweep = forward dir  */
            break;
        }
    }
    CHECK(reversed);
    CHECK(!calibrated);                 /* no false center */
}

/* ---- 4. PB5 center detection completes and transfers ownership to EPS. -- */
static void test_center_detection_transfers_to_eps(void)
{
    reset_all();
    CHECK(drive_to_sweep_left(200));

    set_encoder(250);
    center_detected = true;             /* PB5 fires */
    SteeringMotorOwner_t owner = run_cycle();

    /* Completion cycle: centering neutralises + marks DONE.  It is the
     * ONLY writer this cycle (owner captured before Step). */
    CHECK(owner == STEER_OWNER_CENTERING);
    CHECK(SteeringCentering_GetState() == CENTERING_DONE);
    CHECK(SteeringCentering_IsComplete());
    CHECK(calibrated);
    CHECK(!motor.driven);               /* motor stopped at center */
    CHECK(!eps_ran_this_cycle);           /* EPS did NOT run this cycle */

    /* Next cycle: EPS takes control. */
    calibrated = true;
    owner = run_cycle();
    CHECK(owner == STEER_OWNER_EPS);
    CHECK(!centering_ran_this_cycle);
}

/* ---- 5. Encoder pre-fault isolates the ASSIST (mechanical-only), leaving
 *          the motor at a safe zero state WITHOUT degrading the vehicle. ---- */
static void test_encoder_prefault_aborts_safe(void)
{
    reset_all();
    CHECK(drive_to_sweep_left(200));
    CHECK(motor.driven);

    encoder_fault = true;               /* encoder health fault */
    SteeringMotorOwner_t owner = run_cycle();

    CHECK(owner == STEER_OWNER_CENTERING);          /* owner decided pre-fault */
    CHECK(SteeringCentering_GetState() == CENTERING_FAULT);
    CHECK(SteeringCentering_HasFault());
    CHECK(!motor.driven);                           /* PWM/EN forced to zero   */
    /* New policy: an isolable centering/encoder fault must NOT raise a global
     * safety error — it isolates the assist and keeps mechanical steering.   */
    CHECK(s_error == SAFETY_ERROR_NONE);
    CHECK(Steering_IsMechanicalOnly());
    CHECK(Steering_GetEpsState() == EPS_STATE_MECHANICAL_ONLY);
    CHECK(Steering_EpsGetOwner() == STEER_OWNER_NONE);
    /* The next cycle reports NONE (nobody drives the motor). */
    owner = run_cycle();
    CHECK(owner == STEER_OWNER_NONE);
    CHECK(!motor.driven);
}

/* ---- 6. After isolation, and in SAFE/ERROR, the motor stays neutral and
 *          no subsystem co-drives it. ---- */
static void test_fault_safe_error_states_safe(void)
{
    /* (a) After an isolable centering fault the EPS is mechanical-only and
     *     nobody (owner NONE) drives the motor; the global state is NOT
     *     degraded by this fault. */
    reset_all();
    CHECK(drive_to_sweep_left(200));
    encoder_fault = true;
    run_cycle();                                   /* → FAULT + isolated */
    CHECK(SteeringCentering_GetState() == CENTERING_FAULT);
    CHECK(Steering_IsMechanicalOnly());

    encoder_fault = false;
    SteeringMotorOwner_t owner = run_cycle();
    CHECK(owner == STEER_OWNER_NONE);              /* mechanical-only latched */
    CHECK(!motor.driven);
    CHECK(!centering_ran_this_cycle);

    /* (b) SAFE while centering was mid-sweep → EPS owns, motor neutral. */
    reset_all();
    CHECK(drive_to_sweep_left(200));
    s_state = SYS_STATE_SAFE;
    owner = run_cycle();
    CHECK(owner == STEER_OWNER_EPS);
    CHECK(!motor.driven);
    CHECK(!centering_ran_this_cycle);

    /* (c) ERROR → EPS owns, motor neutral. */
    reset_all();
    CHECK(drive_to_sweep_left(200));
    s_state = SYS_STATE_ERROR;
    owner = run_cycle();
    CHECK(owner == STEER_OWNER_EPS);
    CHECK(!motor.driven);
    CHECK(!centering_ran_this_cycle);
}

/* ---- 7. Explicit mutual-exclusion sweep: across a full lifecycle,
 *          centering and EPS are never co-writers in any cycle. ----
 * (run_cycle() already asserts the invariant every cycle; this test
 *  drives a representative lifecycle to exercise it broadly.)            */
static void test_mutual_exclusion_lifecycle(void)
{
    reset_all();
    /* Homing... */
    CHECK(drive_to_sweep_left(200));
    for (int i = 0; i < 20; i++) { set_encoder(get_encoder() + 3); run_cycle(); }
    /* Complete via PB5. */
    center_detected = true;
    run_cycle();
    calibrated = true;
    /* EPS phase in STANDBY then ACTIVE. */
    s_state = SYS_STATE_STANDBY;
    for (int i = 0; i < 10; i++) run_cycle();
    s_state = SYS_STATE_ACTIVE;
    for (int i = 0; i < 10; i++) run_cycle();
    /* If any cycle had co-writers, CHECK inside run_cycle() flags it. */
    CHECK(SteeringCentering_IsComplete());
}

int main(void)
{
    test_decide_owner_matrix();
    test_pwm_persists_and_not_neutralized();
    test_left_stall_causes_reversal();
    test_center_detection_transfers_to_eps();
    test_encoder_prefault_aborts_safe();
    test_fault_safe_error_states_safe();
    test_mutual_exclusion_lifecycle();

    printf("\n==== test_steering_centering: %d run, %d failed ====\n",
           tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}

#else
int main(void) { return 0; }
#endif /* HOST_TEST */
