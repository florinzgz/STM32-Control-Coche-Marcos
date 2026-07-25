/**
  ****************************************************************************
  * @file    main.c
  * @brief   STM32G474RE vehicle control – main entry point
  *
  *  Peripherals initialised:
  *    ADC1    – Pedal accelerator primary channel (PB1 via voltage divider)
  *    FDCAN1  – CAN bus @ 500 kbps (ESP32-S3 link, PA11 RX / PA12 TX)
  *    I2C1    – INA226 / TCA9548A sensors
  *    TIM1    – PWM for FL motor + FR motor (20 kHz, PA8-PA10, PC3)
  *    TIM2    – Quadrature encoder (steering)
  *    TIM8    – PWM for RL/RR motors (20 kHz, PC6-PC9)
  *    IWDG    – Independent watchdog (~4.1 s)
  ****************************************************************************
  */

#include "main.h"
#include "motor_control.h"
#include "can_handler.h"
#include "rc_arbiter.h"
#include "sensor_manager.h"
#include "safety_system.h"
#include "steering_centering.h"
#include "steering_supervisor.h"
#include "service_mode.h"
#include "boot_validation.h"
#include "encoder_reader.h"
#include "math_safety.h"
#include "steering_cal_store.h"
#include "sensor_map_store.h"
#include "pedal_cal_store.h"
#include "gear_limits_store.h"
#include "drive_tuning_store.h"
#include "battery_limits_store.h"
#include "error_log.h"
#include "loop_diag.h"
#include <math.h>
#include "build_sanity_checks.h"

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

/* ---- DS18B20 conversion timing ----
 * 12-bit conversion can take up to 750 ms; wait >= 750 ms between
 * Convert T (0x44) and Read Scratchpad (0xBE). */
#define DS18B20_CONV_WAIT_MS     800U

typedef enum {
    TEMP_SM_START_CONVERSION = 0,
    TEMP_SM_WAIT_CONVERSION,
    TEMP_SM_READ_ALL
} TempSmState_t;

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

/* Re-arm the startup inhibit latch.
 * Called by Safety_CheckSensors() when a LIMP_HOME that was entered
 * from STANDBY (sole cause: pedal sensor fault) has recovered.
 * Re-arming returns the calibration gate to a satisfiable state:
 * STANDBY + startup_inhibit == true.  The inhibit will clear again
 * automatically after the pedal is held below STARTUP_PEDAL_REST_PCT
 * for STARTUP_PEDAL_CLEAR_MS, exactly as at power-on.              */
void Startup_Rearm(void)
{
    startup_inhibit          = true;
    startup_pedal_rest_since = 0;
}

/* ---- Peripheral init status flags ---- */
bool fdcan_init_ok = false;
bool i2c_init_ok   = false;

/* ---- Boot phase tracker (readable via SWD even when debugger halts at boot) ----
 * 0 = pre-init (initial C startup)
 * 1 = GPIO + FDCAN done (CAN bus active, ACKing ESP32 frames)
 * 2 = MX peripherals done (I2C, TIM, IWDG)
 * 3 = Module init done (Motor, Safety, etc.)
 * 4 = Post-init LED indication done
 * 5 = Main loop entered
 *
 * If the debugger shows boot_phase < 5, the MCU was halted during boot
 * and all diagnostic variables (can_init_diag, fdcan_init_ok, etc.)
 * may still be at their initial zero/false values — not a real failure. */
volatile uint8_t boot_phase = 0;

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

    /* ================================================================
     * CRITICAL: Start FDCAN BEFORE the boot LED sequence.
     *
     * On a two-node CAN bus (ESP32-S3 + STM32), the ESP32 boots ~600 ms
     * after power-on and immediately starts transmitting heartbeat frames
     * every 100 ms.  Without a second node to ACK those frames, every
     * transmission fails (TEC += 8).  At ~32 failures the ESP32 reaches
     * BUS_OFF (TEC = 256) and must recover — a process that adds seconds
     * of latency and may exhaust the 10-attempt recovery budget.
     *
     * Previously, the 2.3 s boot LED blink ran BEFORE FDCAN init,
     * leaving the ESP32 alone on the bus for the entire duration.
     * Moving FDCAN init + CAN_Init() here ensures the STM32 starts
     * ACKing CAN frames within ~50 ms of power-on, eliminating the
     * ESP32 bus-off entirely.
     *
     * CAN_Init() has no dependency on any other module (Motor, Safety,
     * Sensors) — only on SystemClock_Config() for PCLK1 and GPIO for
     * PA11/PA12 (configured by HAL_FDCAN_MspInit internally).
     * ================================================================ */
    MX_FDCAN1_Init();
    CAN_Init();
    RcArbiter_Init();

    boot_phase = 1;  /* GPIO + FDCAN done, CAN bus active */

    /* ---- Startup LED indication ----
     * Three clearly-visible blinks on LD2 (PA5, soldered on the Nucleo
     * board) confirm the firmware has booted and reached peripheral init.
     *
     * Sequence: 500 ms dark (baseline) → 3 × (300 ms ON / 300 ms OFF).
     * Total ≈ 2.3 s.  IWDG has not started yet, so there is no
     * watchdog constraint on this phase.
     *
     * FDCAN is already running — the STM32 is ACKing ESP32 frames
     * during these blinks.  Messages queue in FIFO0 (depth 3);
     * any overflow is harmless and cleared before the main loop.
     *
     * Pattern interpretation (observe LD2 after reset):
     *   • No blink at all     → MCU not executing user code
     *                           (check BOOT0 jumper or re-flash via ST-LINK)
     *   • 3 blinks then ~2 Hz → firmware crashed during a later MX_Init
     *                           (Error_Handler constant-blink pattern)
     *   • 3 blinks, pause, 1 long blink, then brief flash every ~2 s
     *                         → firmware running normally, CAN init OK
     *   • 3 blinks, pause, 5 rapid blinks, then steady 1 Hz blink
     *                         → firmware running, but CAN init FAILED
     *
     * Main-loop LD2 summary (no external LED needed):
     *   • Brief flash every ~2 s (50 ms ON) → CAN OK, firmware alive
     *   • Steady 1 Hz blink (500 ms ON/OFF) → CAN FAILED              */
    HAL_GPIO_WritePin(PORT_LD2, PIN_LD2, GPIO_PIN_RESET); /* ensure OFF  */
    HAL_Delay(500);  /* dark lead-in so the first ON edge is obvious   */
    for (int k = 0; k < 3; k++) {
        HAL_GPIO_WritePin(PORT_LD2, PIN_LD2, GPIO_PIN_SET);
        HAL_Delay(300);
        HAL_GPIO_WritePin(PORT_LD2, PIN_LD2, GPIO_PIN_RESET);
        HAL_Delay(300);
    }

    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM8_Init();
    MX_IWDG_Init();
    boot_phase = 2;  /* All MX peripherals initialised */

    /* Module initialisation */
    Motor_Init();
    Traction_Init();
    Steering_Init();
    Sensor_Init();
    Safety_Init();
    ServiceMode_Init();
    /* CAN_Init() was moved to early boot (before LED blinks) to ensure
     * the FDCAN starts ACKing ESP32 frames immediately on power-on. */
    SteeringCentering_Init();
    ErrorLog_Init();
    ErrorLog_SetResetCause(reset_cause);
    boot_phase = 3;  /* All modules initialised */

    /* ---- Post-init CAN status LED indication ----
     * After all peripherals are initialised, show CAN init result on LD2:
     *   • 1 long blink (600 ms)  → FDCAN initialised successfully
     *   • 5 blinks (150 ms ON/OFF) → FDCAN init failed (hardware issue)
     * A 500 ms gap before the pattern separates it visually from the
     * 3 boot blinks.  The total extra delay (≤ 2 s) is acceptable
     * because IWDG timeout is ~4 s and CAN heartbeat hasn't started. */
    HAL_Delay(500);  /* visual separator after 3 boot blinks */
    {
        if (fdcan_init_ok) {
            /* CAN OK: one long blink */
            HAL_GPIO_WritePin(PORT_LD2, PIN_LD2, GPIO_PIN_SET);
            HAL_Delay(600);
            HAL_GPIO_WritePin(PORT_LD2, PIN_LD2, GPIO_PIN_RESET);
            HAL_Delay(300);
        } else {
            /* CAN FAILED: 5 rapid blinks */
            for (int k = 0; k < 5; k++) {
                HAL_GPIO_WritePin(PORT_LD2, PIN_LD2, GPIO_PIN_SET);
                HAL_Delay(150);
                HAL_GPIO_WritePin(PORT_LD2, PIN_LD2, GPIO_PIN_RESET);
                HAL_Delay(150);
            }
        }
    }

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

    /* ---- Persistent DS18B20 sensor mapping ----
     * Load the user-assigned physIdx→role mapping from flash page 123.
     * If no valid mapping has been saved yet the identity mapping
     * (index 0=FL, 1=FR, 2=RL, 3=RR, 4=AMB) is used as fall-back.    */
    SensorMapStore_Init();

    /* ---- Persistent pedal endpoint calibration ----
     * Page 124 stores the per-vehicle ADC counts for released (MIN)
     * and fully pressed (MAX) pedal positions.  On CRC / magic /
     * range failure the call is a no-op — Pedal_RawToPercent() keeps
     * its compile-time defaults (50 / 4000).  Boot is never blocked
     * by a missing or corrupt calibration slot.
     *
     * Safety: this only changes the linear interpolation endpoints.
     * It does NOT clear startup_inhibit, change EMA state, alter
     * plausibility or FAULT_LO/HI thresholds, or authorise ACTIVE.   */
    PedalCal_Init();
    if (PedalCal_IsValid()) {
        uint16_t pmin = 0, pmax = 0;
        PedalCal_GetStored(&pmin, &pmax);
        Pedal_ApplyCalibration(pmin, pmax);
    }

    /* ---- Gear power-limit + accel-response slot (page 122) ----
     * Load the persisted D2/D1/R traction power limits AND acceleration
     * response profile, then apply them.  On flash blank / CRC-invalid /
     * out-of-range the call is a no-op: motor_control.c keeps its compile-
     * time defaults (power 100/60/60 %, response 100/70/40 %).  A legacy
     * (power-only) slot is migrated safely: the persisted power limits are
     * applied and the compile-time response defaults are used until the
     * operator next SAVEs (which upgrades the slot to v2).
     * Boot is never blocked by a missing or corrupt slot, and this only
     * scales an already-validated traction demand — it does NOT clear
     * startup_inhibit or authorise ACTIVE.                              */
    GearLimitsStore_Init();
    if (GearLimitsStore_IsValid()) {
        uint8_t gd2 = 0, gd1 = 0, gr = 0;
        GearLimitsStore_GetStored(&gd2, &gd1, &gr);
        (void)Traction_SetGearLimits(gd2, gd1, gr);

        uint8_t rd2 = 0, rd1 = 0, rr = 0;
        GearLimitsStore_GetStoredResponse(&rd2, &rd1, &rr);
        (void)Traction_SetGearResponse(rd2, rd1, rr);
    }

    /* ---- Drive-tuning slot (page 121) ----
     * Load the persisted pedal ramp rates and creep parameters, then apply
     * them.  On flash blank / CRC-invalid / out-of-range the call is a
     * no-op: motor_control.c keeps its compile-time defaults (AccelRamp 50,
     * BrakeRamp 100, ReverseRamp 50, CreepEnable on, CreepPower 8, CreepDelay
     * 0), which reproduce the historic firmware behaviour exactly.  Boot is
     * never blocked, and these values only shape an already-validated
     * traction demand — they do NOT clear startup_inhibit or authorise
     * ACTIVE.                                                              */
    DriveTuningStore_Init();
    if (DriveTuningStore_IsValid()) {
        DriveTuning_t dt;
        DriveTuningStore_GetStored(&dt);
        (void)Traction_SetDriveTuning(&dt);
    }

    /* ---- Battery-limits slot (page 120) ----
     * Load the persisted low-voltage warning / derate / cutoff / recovery
     * thresholds and the optional voltage filter, then apply them.  On flash
     * blank / CRC-invalid / out-of-range the call is a no-op: safety_system.c
     * keeps its compile-time defaults (Limit/Warning 20.0 V, Cutoff 18.0 V,
     * Recovery 18.5 V, Filter 0 ms), which reproduce the historic firmware
     * behaviour exactly.  Only the threshold VALUES are loaded — the safety
     * state machine itself is unchanged.                                   */
    BatteryLimitsStore_Init();
    if (BatteryLimitsStore_IsValid()) {
        BatteryLimits_t bl;
        BatteryLimitsStore_GetStored(&bl);
        (void)Safety_SetBatteryLimits(&bl);
    }

    /* Transition: BOOT → STANDBY (peripherals ready, waiting for ESP32) */
    Safety_SetState(SYS_STATE_STANDBY);
    boot_phase = 4;  /* Post-init complete, about to enter main loop */

    /* ---- Drain CAN FIFO and clear boot-time overflow ----
     * During the boot LED sequence and module init the FDCAN was
     * running (ACKing ESP32 frames) but no one was reading FIFO0.
     * Messages accumulated and may have overflowed (message lost).
     * Drain the FIFO now so the main loop starts with a clean slate,
     * and reset the overflow counter to prevent false DEGRADED_L1
     * escalation from harmless boot-time message loss.                */
    CAN_ProcessMessages();
    can_stats.fifo_overflow_count = 0;

    /* Timing counters */
    uint32_t tick_10ms   = 0;
    uint32_t tick_50ms   = 0;
    uint32_t tick_100ms  = 0;
    uint32_t tick_heartbeat = 0; /* LED heartbeat (brief flash every ~2 s) */
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
    TempSmState_t temp_sm_state = TEMP_SM_START_CONVERSION;
    uint32_t temp_conv_start_ms = 0;

    /* ---- Main control loop ---- */
    boot_phase = 5;  /* Main loop entered — init complete */
    while (1) {
        uint32_t now = HAL_GetTick();

        /* ---- PRIORITY: heartbeat before any blocking task ----
         * CAN_SendHeartbeat() carries its own 100 ms guard (HAL_GetTick()
         * fresh call inside), so calling it every iteration is safe and
         * ensures the frame goes out even when a slow I2C/DS18B20 task has
         * consumed several ms of the previous iteration.                   */
        CAN_SendHeartbeat();
        CAN_TxPump();

        /* ---- 10 ms tasks (100 Hz): safety + steering PID ---- */
        if ((now - tick_10ms) >= 10) {
            tick_10ms = now;

            /* Loop-time diagnostic — capture DWT cycle count at task
             * entry so we can publish the peak 100 Hz task duration in
             * STATUS_SAFETY byte 5.  Pure observational; SystemCoreClock
             * tick rate (170 MHz on STM32G474) gives µs = cycles/170.   */
            uint32_t loop_diag_cyc_start = DWT->CYCCNT;

            /* Batch-update wheel speeds FIRST — all safety/traction
             * consumers in this cycle see identical, consistent values.
             * Must precede ABS/TCS/Safety which read Wheel_GetSpeed_*().*/
            Wheel_UpdateSpeeds();

            /* Start every ABS/TCS arbitration cycle from full per-wheel
             * authority.  ABS and tuned TCS may only lower the current
             * cycle values; no historical reduction can remain latched. */
            Safety_ResetWheelInterventionScales();
            ABS_Update();
            TCS_Update();
            Safety_CheckCurrent();
            Safety_CheckTemperature();
            Safety_CheckCANTimeout();
            Safety_CheckSteeringTimeout();
            CAN_CheckBusOff();
            Safety_CheckSensors();
            Safety_CheckEncoder();
            Safety_CheckRelayHealth();

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

            /* Relay override (engineering diagnostic mode) — applies
             * manual relay GPIO control when enabled.  Runs AFTER the
             * normal sequencer so override GPIOs take precedence.
             * Continuously re-validates safety conditions and auto-
             * disables on any violation.                                */
            Safety_RelayOverrideUpdate();

            /* Steering-motor relay (PC12) policy supervisor — independent,
             * always-run guard.  If any path leaves PC12 commanded ON while
             * the assist is isolated (policy forbids it), force PC12 OFF and
             * surface the violation.  Never touches PC11 / traction.         */
            Safety_SteerRelaySupervise();

            /* Steering motor ownership arbitration.
             *
             * Exactly ONE subsystem may write the steering motor (PC4
             * EN_STEER / PA6 / PA7) per cycle.  We decide the owner from
             * the state captured at the START of this cycle, BEFORE
             * stepping the centering FSM.  This prevents Steering_ControlLoop()
             * from neutralising the centering PWM in the same 10 ms cycle
             * (the root cause of the Error 8 / FAULT_CENTERING regression),
             * and guarantees centering and the EPS loop never both drive
             * the motor simultaneously.
             *
             * While homing owns the motor, SteeringCentering_Step() is the
             * sole writer.  Once centering reaches DONE/FAULT — or the
             * system leaves BOOT/STANDBY (SAFE/ERROR) — the EPS loop takes
             * over and neutralises the motor when uncalibrated. */
            {
                SystemState_t steer_state = Safety_GetState();
                bool in_homing = (steer_state == SYS_STATE_BOOT ||
                                  steer_state == SYS_STATE_STANDBY);
                SteeringMotorOwner_t owner =
                    SteeringCentering_DecideOwner(SteeringCentering_GetState(),
                                                  in_homing);

                if (owner == STEER_OWNER_CENTERING) {
                    SteeringCentering_Step();
                } else {
                    /* STEER_OWNER_EPS or STEER_OWNER_NONE.  Steering_ControlLoop()
                     * self-guards: when the assist is isolated (mechanical-only)
                     * it simply coasts the motor and never re-drives it. */
                    Steering_ControlLoop();
                }

                /* Capture homing telemetry AFTER the motor writer has run so
                 * the recorded PWM/CCR reflect what was actually emitted this
                 * cycle.  Pure instrumentation — drives nothing. */
                SteeringCentering_UpdateDiag();
            }

            /* EPS assist supervisor — connects the real steering detectors
             * (INA226 CH5 current sensor, EPS parameter store, calibration
             * store, encoder Z) to the EPS isolation policy.  Any isolable
             * fault disconnects the assist (mechanical-only) without touching
             * traction; a proven persistent overcurrent escalates to SAFE.
             * Runs AFTER the motor writer so it observes this cycle's PWM.  */
            SteeringSupervisor_Service();
            Traction_Update();

            /* Loop-time diagnostic — record duration of this 100 Hz
             * task block.  DWT cycle counter is monotonic 32-bit; the
             * unsigned subtraction wraps correctly across rollover.
             * Conversion: µs = cycles / cycles_per_us (170 at 170 MHz). */
            {
                uint32_t cyc_per_us = SystemCoreClock / 1000000U;
                if (cyc_per_us == 0U) cyc_per_us = 170U;  /* Safe default */
                uint32_t elapsed_cyc = DWT->CYCCNT - loop_diag_cyc_start;
                LoopDiag_RecordTaskUs(elapsed_cyc / cyc_per_us);
            }
        }

        /* ---- 50 ms tasks (20 Hz): sensors + pedal ---- */
        if ((now - tick_50ms) >= 50) {
            tick_50ms = now;

            Pedal_Update();
            CAN_PedalCalCaptureTick();   /* R-1: cooperative pedalcal FSM */
            Current_ReadAll();
            switch (temp_sm_state) {
                case TEMP_SM_START_CONVERSION:
                    Temperature_StartConversion();
                    temp_conv_start_ms = now;
                    temp_sm_state = TEMP_SM_WAIT_CONVERSION;
                    break;

                case TEMP_SM_WAIT_CONVERSION:
                    if ((now - temp_conv_start_ms) >= DS18B20_CONV_WAIT_MS) {
                        temp_sm_state = TEMP_SM_READ_ALL;
                    }
                    break;

                case TEMP_SM_READ_ALL:
                    Temperature_ReadAll();
                    temp_sm_state = TEMP_SM_START_CONVERSION;
                    break;

                default:
                    temp_sm_state = TEMP_SM_START_CONVERSION;
                    break;
            }

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
             * 2. LIMP_HOME: Local pedal only, clamp (40% torque),
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
                    /* RC override arbiter (Modo Control Remoto Clásico):
                     * if a fresh 0x10A frame says override_active, use RC
                     * throttle; otherwise use the physical pedal.  The
                     * arbiter has a strict 200 ms watchdog → if the ESP32
                     * or the RC link dies, control returns to the pedal
                     * automatically.  Safety_ValidateThrottle stays
                     * DOWNSTREAM so all existing safety gates (state,
                     * step-rate, gear, limits) apply equally to local and
                     * RC demands.                                          */
                    float local_pct = Pedal_GetPercent();
                    float demand_pct = RcArbiter_GetThrottle(local_pct, now);
                    float validated = Safety_ValidateThrottle(demand_pct);
                    Traction_SetDemand(validated);
                }
            } else if (Safety_IsLimpHome()) {
                /* LIMP_HOME: local pedal drives traction directly.
                 * Ignore all CAN throttle commands.
                 * Apply the LIMP_HOME torque limit (40%) as a hard clamp.
                 * This is the SINGLE point where the LIMP_HOME ceiling is
                 * applied (AUDIT G): the traction pipeline no longer
                 * re-scales it (Safety_GetTractionCapFactor()==1.0 in
                 * LIMP_HOME).  The ramp rate and 5 km/h speed cap still
                 * apply downstream, but they do not re-scale the ceiling.
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
                    /* Hard clamp: 40% max torque in LIMP_HOME.
                     * pedal is 0–100%, factor is 0.40 → max demand = 40%. */
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
                (uint8_t)Safety_GetError(),
                LoopDiag_GetAndResetPeak100us());
            CAN_SendStatusSteering(
                (int16_t)(Steering_GetCurrentAngle() * 10),
                Steering_IsCalibrated());
            CAN_SendStatusTraction();
            CAN_SendStatusWheelEffort();   /* 0x20C — real applied per-wheel PWM */
            CAN_SendMotionInhibit();
            CAN_SendStatusBattery();

            /* Pedal position telemetry (0x20B) — real Hall pedal travel for
             * the HMI THROTTLE bar.  Telemetry only: this read does not affect
             * control, PID, safety or pedal logic in any way.                */
            {
                float pedal_tlm = Pedal_GetPercent();
                if (pedal_tlm < 0.0f)   pedal_tlm = 0.0f;
                if (pedal_tlm > 100.0f) pedal_tlm = 100.0f;
                CAN_SendStatusPedal((uint8_t)(pedal_tlm + 0.5f));
            }

            /* Pedal calibration telemetry burst (0x308) — emits only
             * while an explicit QUERY is active (1 s, 10 Hz).  No-op
             * otherwise.  Does not affect backward-compatible nodes. */
            CAN_PedalCalBurstUpdate();

            /* Gear power-limit telemetry burst (0x30D) — emits only while
             * an explicit QUERY is active (1 s, 10 Hz).  No-op otherwise. */
            CAN_GearLimitsBurstUpdate();

            /* PB5 + encoder-Z center diagnostic burst (0x30E) — emits only
             * while an explicit QUERY/CALIBRATE is active (1 s, 10 Hz).
             * No-op otherwise; diagnostic only, never gates control.      */
            CAN_SteeringZBurstUpdate();

            /* EPS parameter + live-state telemetry burst (0x30F) — emits
             * only while an explicit QUERY/SET/SAVE is active (1 s, 10 Hz).
             * No-op otherwise; zero impact on backward-compatible nodes.  */
            CAN_EPS_ParamsBurstUpdate();

            /* Drive-tuning (ramp/creep) telemetry burst (0x310) — emits only
             * while an explicit QUERY is active (1 s, 10 Hz).  No-op otherwise;
             * zero impact on backward-compatible nodes that ignore 0x310.    */
            CAN_DriveTuningBurstUpdate();

            /* Battery voltage-limit telemetry burst (0x311) — emits only while
             * an explicit QUERY is active (1 s, 10 Hz).  No-op otherwise.     */
            CAN_BatteryLimitsBurstUpdate();
        }

        /* ---- 1000 ms tasks (1 Hz): temperatures + service status ---- */
        if ((now - tick_1000ms) >= 1000) {
            tick_1000ms = now;

            /* Observability B: count 1 Hz scheduler-block iterations so the
             * 0x30A meta-frame can prove the diagnostic block runs.  Saturates. */
            if (can_txmeta.tick_1000ms_count < UINT32_MAX)
                can_txmeta.tick_1000ms_count++;

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

            /* Debounce EMI diagnostic counters (0x306 + 0x307) for the
             * ESP32 engineering "DEBOUNCE DEBUG" submenu.  Diagnostic
             * only — does not gate any control or safety path.        */
            CAN_SendDebounceDiag();

            /* I2C topology diagnostic (0x309): TCA9548A presence + per-channel
             * INA226 health, so the HMI Safe Mode screen can tell a missing
             * mux apart from a missing/dead INA226.  Diagnostic only — does
             * not gate any control or safety path.                        */
            CAN_SendI2CDiag();

            /* CAN/0x309 delivery meta-diagnostic (0x30A): call/tick/TX-ok/err
             * and FIFO-full drop counters, so the ESP32 can tell apart "0x309
             * never generated" from "generated but never reached the bus".
             * Diagnostic only — does not gate any control or safety path.   */
            CAN_SendCanMetaDiag();

            /* Boot/reset diagnostic (0x312): uptime + RCC reset-cause bitmask.
             * Lets the HMI confirm whether 8–10 s gaps are IWDG/brownout resets
             * by watching the uptime counter restart and checking the cause byte.
             * Diagnostic only — does not gate any control or safety path.     */
            CAN_SendBootResetDiag();

            /* Per-wheel speed-sensor fault-reason diagnostic (0x313): reason
             * code + GPIO level + fault mask per wheel, so the HMI can name the
             * failing wheel and tell manual movement apart from a real fault.
             * Diagnostic only — does not gate any control or safety path.     */
            CAN_SendWheelSensorDiag();

            /* Steering homing telemetry (0x316): classified reason + FSM/owner
             * + PB5/PC12/PC4/power/PWM/encoder snapshot so the HMI can show the
             * real cause of a stuck centering sweep.  Instrumentation only.   */
            CAN_SendSteeringCenteringDiag();

            /* Relay / current-sense health telemetry (0x317): evidence-graded
             * cause + the numbers behind it so the HMI can show CURRENT SENSE
             * INVALID vs RELAY OPEN SUSPECTED.  Instrumentation only.          */
            CAN_SendRelayHealthDiag();

            /* Steering INA226 (CH5) channel diagnostic (0x318): explicit
             * MISSING vs CONFIG FAIL vs PRESENT-NO-SHUNT vs POLARITY vs STALE,
             * with a signed shunt/current that is never zeroed.               */
            CAN_SendIna226Ch5Diag();

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

            /* CAN frame rate computation — diagnostic metric.
             * Updates can_stats.rx_frames_per_sec for SWD inspection.     */
            CAN_UpdateFrameRate();
        }

        /* ---- LD2 heartbeat with CAN status ----
         * LD2 (PA5, green LED soldered on the Nucleo board) now shows
         * both firmware liveness AND CAN bus status at a glance:
         *
         *   • CAN OK  (fdcan_init_ok): brief flash every ~2 s
         *     (50 ms ON / 1950 ms OFF).  Means firmware + CAN healthy.
         *   • CAN FAIL (!fdcan_init_ok): steady 1 Hz blink
         *     (500 ms ON / 500 ms OFF).  Means CAN init failed.
         *
         * Both are clearly distinct from:
         *   - Error_Handler  → constant ~2 Hz blink (250 ms toggle)
         *   - HardFault      → rapid ~10 Hz blink                       */
        if (fdcan_init_ok) {
            /* CAN OK: brief flash every 2 s */
            uint32_t phase = now - tick_heartbeat;
            if (phase >= 2000) {
                tick_heartbeat = now;
                HAL_GPIO_WritePin(PORT_LD2, PIN_LD2, GPIO_PIN_SET);
            } else if (phase >= 50) {
                HAL_GPIO_WritePin(PORT_LD2, PIN_LD2, GPIO_PIN_RESET);
            }
        } else {
            /* CAN FAIL: steady 1 Hz blink (500 ms ON / 500 ms OFF) */
            uint32_t ld2_phase = now % 1000U;
            HAL_GPIO_WritePin(PORT_LD2, PIN_LD2,
                (ld2_phase < 500U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        /* ---- LED_DIAG (PB14): CAN status indicator ----
         * Visible without a debugger on the external diagnostic LED.
         *   • CAN OK (fdcan_init_ok):  solid ON
         *   • CAN FAIL (!fdcan_init_ok): fast blink ~4 Hz (125 ms ON/OFF)
         * Requires external LED + 330 Ω resistor on PB14 (CN10 pin 28). */
        if (fdcan_init_ok) {
            HAL_GPIO_WritePin(PORT_LED_DIAG, PIN_LED_DIAG, GPIO_PIN_SET);
        } else {
            uint32_t diag_phase = now % 250U;
            HAL_GPIO_WritePin(PORT_LED_DIAG, PIN_LED_DIAG,
                (diag_phase < 125U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        /* ---- 500 ms CAN test transmit: send a test frame (ID 0x123) ---- */
        if ((now - tick_500ms) >= 500) {
            tick_500ms = now;
            CAN_TestTransmit();
        }

        /* Process incoming CAN commands from ESP32 */
        CAN_ProcessMessages();

        /* Drain the software CAN TX queue into the FDCAN hardware FIFO.
         * The 100 ms / 1 s status blocks enqueue bursts far faster than the
         * 3-slot hardware FIFO can drain at the bus rate; pumping every
         * iteration moves the backlog out one frame at a time as slots free,
         * so diagnostic frames (0x309/0x30A/0x30B/0x30C) are never dropped to
         * a momentarily-full FIFO.  Non-blocking — returns as soon as the
         * FIFO is full or the queue is empty.                              */
        CAN_TxPump();

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

    /* C1 HARDENING: Force all motor EN pins LOW before configuring them.
     * STM32G4 ODR resets to 0 on power-on, but this explicit write
     * guarantees EN=LOW even after a warm reset or watchdog reset
     * where ODR may retain its previous value.  This prevents any
     * transient motor activation before the timers are configured.
     *
     * BSRR write is atomic and takes effect in one AHB cycle.
     * GPIO clock was just enabled above, so GPIOC is accessible.       */
    GPIOC->BSRR = (uint32_t)(PIN_EN_FL | PIN_EN_FR | PIN_EN_RL | PIN_EN_RR
                  | PIN_EN_STEER) << 16U;

    /* GPIO enable outputs for all five BTS7960 modules (GPIOC).
     * All motors now have dedicated GPIO EN pins for symmetric
     * coast/brake behaviour.  PC0 (EN_FR), PC1 (EN_RL), PC4 (EN_STEER)
     * were freed direction pins; PC5 (EN_FL) unchanged; EN_RR moved
     * from PC13 → PC2 to avoid conflict with the NUCLEO-G474RE USER
     * button (B1 on PC13).                                            */
    gpio.Pin   = PIN_EN_FL | PIN_EN_FR | PIN_EN_RL | PIN_EN_RR | PIN_EN_STEER;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* Relay outputs (GPIOC): PC11 = traction, PC12 = steering actuator power. */
    gpio.Pin = PIN_RELAY_TRAC | PIN_RELAY_STEER_PWR;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* PC10 — AVAILABLE / not connected to any hardware.
     * Configured as digital input with internal pull-down so the pin
     * sits in a deterministic logic-LOW state instead of floating.
     * Rationale:
     *   - prevents indeterminate readings / spurious EXTI noise
     *   - eliminates leakage current through floating CMOS input
     *   - keeps the pin safe for future reassignment
     * No firmware logic depends on this pin.                            */
    gpio.Pin   = GPIO_PIN_10;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* Nucleo-64 user LED LD2 (PA5) — soldered on the board.
     * Used for boot blinks, CAN status, heartbeat, and fault indication. */
    gpio.Pin   = PIN_LD2;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PORT_LD2, &gpio);

    /* Diagnostic LED (PB14) — external LED on Morpho CN10 pin 28.
     * Freed from TIM15_CH1 (LPWM_FR moved to PC3/TIM1_CH4).
     * Not shared with ST-Link; reliable during debug sessions. */
    gpio.Pin   = PIN_LED_DIAG;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PORT_LED_DIAG, &gpio);
    HAL_GPIO_WritePin(PORT_LED_DIAG, PIN_LED_DIAG, GPIO_PIN_RESET);

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

    /* Wheel speed EXTI inputs.
     * FL/FR remain on port A (PA0/PA1).  RL was moved from PA2 to PB2
     * because the original PA2 / CN10_35 net was shorted to GND (dead
     * channel).  RL keeps EXTI line 2, so EXTI2_IRQHandler and the NVIC
     * setup below are unchanged — only the GPIO source port differs.   */
    gpio.Pin  = PIN_WHEEL_FL | PIN_WHEEL_FR;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* RL (PB2, EXTI2) + RR (PB15, EXTI15) — both on port B, same
     * rising-edge / pull-up configuration inherited from above.        */
    gpio.Pin  = PIN_WHEEL_RL | PIN_WHEEL_RR;
    HAL_GPIO_Init(PORT_WHEEL_RL, &gpio);   /* GPIOB */

    /* Steering center inductive sensor (PB5 / EXTI5).
     * LJ12A3 is NPN open-collector: with GPIO_PULLUP the idle level is
     * HIGH; when the metal screw enters the sensing zone the transistor
     * pulls the line LOW.  FALLING edge = "screw just entered center" —
     * this is the instant at which steering_centering.c stops the motor
     * and zeroes TIM2->CNT, so centering latches on entry rather than
     * slightly past center (which a RISING trigger would have done).
     * Polarity is consistent with SteeringCal_ValidateAtBoot() in
     * steering_cal_store.c, which expects GPIO_PIN_RESET (LOW) at
     * center on boot.                                                 */
    gpio.Pin  = PIN_STEER_CENTER;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Encoder Z-index pulse (PB4 / EXTI4).  The E6B2-CWZ6C Z output is NPN
     * open-collector: with GPIO_PULLUP the idle level is HIGH and the Z
     * pulse pulls it LOW → FALLING edge trigger.
     *
     * The Z channel is used for inter-revolution drift detection only.
     * It does NOT control steering centering (that uses PB5 / LJ12A3).
     * See encoder_reader.c — EncoderZ_IRQHandler() for the ISR logic.   */
    gpio.Pin  = PIN_ENC_Z;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* EXTI IRQs */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);         /* PB4 encoder Z     */
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);       /* PB5 center sensor */
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

    /* Retry loop: HAL_FDCAN_Init() can fail on the first attempt when
     * the peripheral clock gate has not fully stabilised after a
     * force-reset (observed as CCCR reading garbage, e.g. 0x8007aa5).
     * A short delay followed by another attempt lets the bus bridge
     * settle.  Five attempts with 10 ms spacing provide sufficient
     * margin even on STM32G4 revisions with slow clock-gate recovery. */
    #define FDCAN_INIT_MAX_RETRIES       5
    #define FDCAN_CLOCK_SETTLE_DELAY_MS  10U  /* Post-reset bus bridge settle */
    #define FDCAN_CCCR_RESERVED_MASK     0xFFFF0000U  /* Bits 16-31 are reserved */

    /* Pre-loop delay: let the peripheral's initial clock-gate state
     * settle after SystemClock_Config() changed the PLL / PCLK1 source
     * and HAL_RCCEx_PeriphCLKConfig() set FDCANSEL = PCLK1.
     * 2 ms provides margin for bus-bridge pipeline on all G4 revisions. */
    #define FDCAN_INITIAL_SETTLE_DELAY_MS  2U
    HAL_Delay(FDCAN_INITIAL_SETTLE_DELAY_MS);

    can_init_diag.retries = 0U;

    for (int attempt = 0; attempt < FDCAN_INIT_MAX_RETRIES; attempt++) {
        can_init_diag.retries = (uint8_t)attempt;
        if (attempt > 0) {
            HAL_FDCAN_DeInit(&hfdcan1);
            HAL_Delay(FDCAN_CLOCK_SETTLE_DELAY_MS);
        }

        if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
            continue;  /* Retry */
        }

        /* Sanity-check: after successful init CCCR.INIT must be set
         * (peripheral in configuration mode) and reserved upper bits
         * (16-31) must be zero.  If they are not, the peripheral is
         * not responding — register reads return bus-default garbage.
         *
         * IMPORTANT: Triple-read CCCR validation.  When the FDCAN APB
         * clock gate is unstable after force-reset, register reads
         * return stale AHB bus data.  This garbage is random and can
         * occasionally pass the bitmask checks.  Reading CCCR three
         * times and comparing detects this: real register values are
         * deterministic, stale bus data is not.                       */
        // cppcheck-suppress duplicateAssignExpression
        uint32_t cccr1 = hfdcan1.Instance->CCCR;
        // cppcheck-suppress duplicateAssignExpression
        uint32_t cccr2 = hfdcan1.Instance->CCCR;
        uint32_t cccr3 = hfdcan1.Instance->CCCR;
        can_init_diag.cccr_last = cccr1;  /* Capture for SWD debugging */
        if (cccr1 != cccr2 || cccr2 != cccr3) {
            HAL_FDCAN_DeInit(&hfdcan1);
            continue;  /* Inconsistent reads — clock gate unstable */
        }
        if ((cccr1 & FDCAN_CCCR_INIT) == 0U || (cccr1 & FDCAN_CCCR_RESERVED_MASK) != 0U) {
            HAL_FDCAN_DeInit(&hfdcan1);
            continue;  /* Retry */
        }

        fdcan_init_ok = true;
        return;
    }

    fdcan_init_ok = false;  /* All attempts exhausted — CAN disabled */
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x30F0EDFF;  /* 100 kHz Standard Mode @ 170 MHz
                                                 * PRESC=3 → t_PRESC=23.53 ns
                                                 * SCLH=237 → t_SCLH≈5600 ns (min 4000 ns ✓)
                                                 * SCLL=255 → t_SCLL≈6023 ns (min 4700 ns ✓)
                                                 * Actual f_SCL ≈ 86 kHz — within SM spec
                                                 * (I2C Standard Mode = ≤ 100 kHz; 86 kHz is
                                                 * deliberately conservative for automotive EMI).
                                                 * Reduced from 400 kHz Fast Mode. */
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
     * FR motor (CH3=RPWM_FR/PA10, CH4=LPWM_FR/PC3).
     * Both channels of each motor share the SAME timer so all CCR
     * preload registers transfer at the same UEV — overlap = 0.
     * TIM1 is an advanced timer: BREAK2 is armed to the Cortex-M4
     * LOCKUP signal so any true CPU lockup instantly clears MOE,
     * forcing all four outputs LOW without software intervention.
     *
     * PC3 provides TIM1_CH4 via AF2.  This was formerly LPWM_FR on
     * TIM15_CH1/PB14 (cross-timer); moving it to TIM1 eliminates the
     * cross-timer arrangement and gives perfect UEV synchronisation.
     * PB14 is freed for use as a diagnostic LED (LED_DIAG).          */
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
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_4);  /* LPWM_FR — PC3  */

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

    /* Configure channel: PB1 = ADC1_IN12, single-ended
     * (moved from PA3/ADC1_IN4 — original PA3 net shorted to GND). */
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = PEDAL_ADC_CHANNEL;   /* ADC_CHANNEL_12 (PB1) */
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
    /* Reload = 4095 → ~4.1 s timeout.
     * Nominal calculation: LSI / 32 = 1 kHz tick × 4095 counts = 4.095 s.
     * The "~" reflects LSI tolerance: per RM0440 the internal LSI is rated
     * ±5 % across temperature and supply, so the real-world reload range is
     * approximately 3.9–4.3 s.  All time-critical software loops are sized
     * with margin for the worst-case (shortest) bound.                     */
    hiwdg.Init.Reload    = 4095;
    hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    /* EN-first shutdown: BSRR → MOE → CCR.
     *
     * Force all motor EN pins LOW as the VERY FIRST action so every
     * BTS7960 enters Hi-Z immediately — including TIM3/steering which
     * has no BREAK mechanism.  In overcurrent / short-circuit faults
     * Hi-Z (coast) is safer than passive brake because it stops all
     * FET conduction instantly.
     *
     * MOE clear and CCR zeroing follow as defence-in-depth layers.
     * Uses direct register access because HAL may be inconsistent.    */
    GPIOC->BSRR = (uint32_t)(PIN_EN_FL | PIN_EN_FR | PIN_EN_RL | PIN_EN_RR
                  | PIN_EN_STEER
                  | PIN_RELAY_TRAC | PIN_RELAY_STEER_PWR) << 16U;
    /* LED power relays on GPIOB — also force OFF (both front and rear) */
    GPIOB->BSRR = (uint32_t)(PIN_RELAY_LED | PIN_RELAY_LED_REAR) << 16U;
    TIM1->BDTR &= ~TIM_BDTR_MOE;   /* Disable all TIM1 PWM outputs (FL, FR)   */
    TIM8->BDTR &= ~TIM_BDTR_MOE;   /* Disable all TIM8 PWM outputs (RL, RR)   */
    TIM1->CCR1  = 0U;  TIM1->CCR2  = 0U;  /* FL: RPWM, LPWM → 0 */
    TIM1->CCR3  = 0U;  TIM1->CCR4  = 0U;  /* FR: RPWM, LPWM → 0 */
    TIM8->CCR1  = 0U;  TIM8->CCR2  = 0U;  /* RL: RPWM, LPWM → 0 */
    TIM8->CCR3  = 0U;  TIM8->CCR4  = 0U;  /* RR: RPWM, LPWM → 0 */
    TIM3->CCR1  = 0U;               /* RPWM_STEER → 0 (TIM3 has no BREAK)      */
    TIM3->CCR2  = 0U;               /* LPWM_STEER → 0                          */

    /* Slow-blink LD2 (~2 Hz) — distinguishes "firmware crashed" from
     * "firmware not loaded" (LED stays off).  Uses busy-wait because
     * interrupts are disabled.  If IWDG is already running the MCU
     * will reset after ~500 ms; the brief blink is still visible.
     * Delay count assumes SYSCLK = 170 MHz (HSI+PLL, see
     * SystemClock_Config); at that frequency ~4 000 000 volatile
     * iterations ≈ 250 ms.                                            */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    PORT_LD2->MODER = (PORT_LD2->MODER
                       & ~(3U << (PIN_LD2_N * 2)))
                    | (1U << (PIN_LD2_N * 2));           /* PA5 = output */
    while (1) {
        PORT_LD2->ODR ^= PIN_LD2;
        for (volatile uint32_t d = 0; d < 4000000U; d++) { __NOP(); }
    }
}