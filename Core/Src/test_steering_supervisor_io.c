/**
  ****************************************************************************
  * @file    test_steering_supervisor_io.c
  * @brief   Integration host test for the EPS supervisor PRODUCTION GLUE.
  *
  *          Unlike test_steering_supervisor.c (which injects a decision
  *          snapshot directly), this test exercises the REAL production
  *          data-gathering path steering_supervisor_io.c:SteeringSupervisor_Service():
  *          it reads a controllable INA226 CH5 diagnostic through the real
  *          Sensor_GetChannel5Diag() seam, builds the SteeringSupervisorInputs
  *          exactly like firmware, and lets the real steering_supervisor.c +
  *          steering_eps.c drive isolation.  It proves:
  *            - the REAL acquisition sequence (sample_sequence) is transported,
  *              so the 100 Hz service does not reprocess one 20 Hz sample;
  *            - CH5 is treated as always-powered (pre-relay), PC12 OFF is normal;
  *            - missing / stale / config-invalid / polarity CH5 isolate the
  *              assist (MECHANICAL_ONLY) WITHOUT a global SAFE;
  *            - a single transient does not isolate; three consecutive do;
  *            - after isolation: latched mechanical-only, never a hazard from
  *              missing information;
  *            - overcurrent with no fresh post-isolation sample stays
  *              ISOLATION UNCONFIRMED (never auto-SAFE) while uncalibrated;
  *            - (calibrated build) a persistent fresh over-current escalates
  *              to ELECTRICAL_HAZARD + SAFE.
  *
  *          Compile (from repository root):
  *            gcc -std=c11 -DHOST_TEST -DHOST_TEST_GPIO_MODEL -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  *                Core/Src/test_steering_supervisor_io.c \
  *                Core/Src/steering_supervisor_io.c \
  *                Core/Src/steering_supervisor.c \
  *                Core/Src/steering_eps.c \
  *                Core/Src/ina226_channel_diag.c \
  *                -lm -o /tmp/test_steering_supervisor_io && \
  *            /tmp/test_steering_supervisor_io
  ****************************************************************************
  */

#ifdef HOST_TEST

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "steering_supervisor.h"
#include "steering_eps.h"
#include "steering_z.h"
#include "steering_centering.h"
#include "ina226_channel_diag.h"
#include "safety_system.h"

/* ---- Test harness ---------------------------------------------------- */
static int tests_run = 0, tests_failed = 0;
#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ---- Deterministic HAL tick (HOST_TEST_GPIO_MODEL seam) --------------- */
uint32_t g_stub_hal_tick = 0;

/* ---- steering_eps.c physical-shutdown externs (observed) ------------- */
static int phys_off_calls = 0;
static int power_off_calls = 0;
void Steering_PhysicalOff(void)   { phys_off_calls++; }
void Steering_SteerPowerOff(void) { power_off_calls++; }

/* ---- Controllable CH5 diagnostic behind the real sensor seam --------- */
static Ina226ChannelDiag g_ch5;
const Ina226ChannelDiag *Sensor_GetChannel5Diag(void) { return &g_ch5; }

/* ---- EPS parameter store / calibration / centering / Z stubs --------- */
static bool s_params_valid   = true;
static bool s_params_corrupt = false;
static bool s_cal_corrupt    = false;
static bool s_cal_restored   = true;
static bool s_calibrated     = true;
static CenteringState_t s_centering = CENTERING_DONE;
static SteeringZStatus_t s_zstatus  = STEERING_Z_OK;

bool EPS_Params_IsFlashValid(void)     { return s_params_valid; }
bool EPS_Params_IsFlashCorrupt(void)   { return s_params_corrupt; }
bool SteeringCal_IsFlashCorrupt(void)  { return s_cal_corrupt; }
bool SteeringCal_IsRestoredValid(void) { return s_cal_restored; }
bool Steering_IsCalibrated(void)       { return s_calibrated; }
CenteringState_t SteeringCentering_GetState(void) { return s_centering; }
SteeringZStatus_t SteeringZ_GetStatus(void)       { return s_zstatus; }

/* ---- Safety escalation seam (observed) ------------------------------- */
static int  safe_state_calls = 0;
static int  set_error_calls  = 0;
static SystemState_t last_state_req = SYS_STATE_ACTIVE;
static Safety_Error_t last_error_req = SAFETY_ERROR_NONE;
void Safety_SetState(SystemState_t s) { safe_state_calls++; last_state_req = s; }
void Safety_SetError(Safety_Error_t e){ set_error_calls++;  last_error_req = e; }

/* ---- Scenario reset --------------------------------------------------- */
static Ina226ChannelDiag healthy_ch5(void)
{
    Ina226ChannelDiag d;
    memset(&d, 0, sizeof(d));
    d.channel            = INA226_CHANNEL_STEER;
    d.mux_channel        = INA226_CHANNEL_STEER;
    d.expected_address   = 0x40;
    d.detected_address   = 0x40;
    d.mux_select_ok      = true;
    d.i2c_ack            = true;
    d.manufacturer_id_ok = true;
    d.die_id_ok          = true;
    d.config_write_ok    = true;
    d.config_readback_ok = true;
    d.shunt_read_ok      = true;
    d.bus_read_ok        = true;
    d.raw_shunt          = 1200;
    d.shunt_uv           = 2340;
    d.bus_mv             = 12450;
    d.signed_current_ma  = 1820;
    d.sample_age_ms      = 10;
    d.sample_sequence    = 1;
    d.last_valid_tick_ms = 0;
    d.channel_powered    = true;   /* pre-relay: always powered            */
    d.current_expected   = true;
    d.fault_reason       = INA226_CH_OK;
    return d;
}

static void reset_all(void)
{
    Steering_EpsInit();
    SteeringSupervisor_Init();
    phys_off_calls = power_off_calls = 0;
    safe_state_calls = set_error_calls = 0;
    last_state_req = SYS_STATE_ACTIVE;
    last_error_req = SAFETY_ERROR_NONE;
    s_params_valid = true; s_params_corrupt = false;
    s_cal_corrupt = false; s_cal_restored = true; s_calibrated = true;
    s_centering = CENTERING_DONE;
    s_zstatus = STEERING_Z_OK;
    g_stub_hal_tick = 0;
    g_ch5 = healthy_ch5();
}

/* Advance the deterministic tick and run one 100 Hz supervisor cycle. */
static void service_at(uint32_t tick)
{
    g_stub_hal_tick = tick;
    SteeringSupervisor_Service();
}

/* ===================================================================== */

/* Healthy CH5 → assist active, nothing isolated, no SAFE. */
static void test_io_healthy(void)
{
    reset_all();
    service_at(1000);
    CHECK(!Steering_IsMechanicalOnly());
    CHECK(!Steering_IsElectricalHazard());
    CHECK(safe_state_calls == 0);
    CHECK(Steering_GetEpsFault() == EPS_FAULT_NONE);
}

/* PC12 OFF, CH5 still ~12 V and ~0 A → NORMAL (pre-relay), no isolation. */
static void test_io_pc12_off_normal(void)
{
    reset_all();
    g_ch5 = healthy_ch5();
    g_ch5.current_expected  = false;   /* relay open, no PWM commanded      */
    g_ch5.signed_current_ma = 0;       /* residual/zero                     */
    g_ch5.bus_mv            = 12420;   /* still fed from the pre-relay bus   */
    g_ch5.fault_reason      = INA226_CH_OK;
    service_at(1000);
    CHECK(!Steering_IsMechanicalOnly());
    CHECK(!Steering_IsElectricalHazard());
    CHECK(safe_state_calls == 0);
}

/* The 100 Hz service must NOT reprocess one 20 Hz sample: with sample_sequence
 * frozen, five calls behave as one; only a new sequence is a fresh sample.  */
static void test_io_sequence_freshness(void)
{
    reset_all();
    /* Drive an overcurrent so the FSM enters CONFIRM_WAIT on the first cycle. */
    g_ch5 = healthy_ch5();
    g_ch5.sample_sequence   = 5;
    g_ch5.signed_current_ma = STEERING_ACTIVE_OVERCURRENT_MA + 4000;
    service_at(1000);                       /* isolate, capture seq 5        */
    CHECK(Steering_IsMechanicalOnly());
    CHECK(SteeringSupervisor_OcState() == OC_STATE_CONFIRM_WAIT);
    /* Four intermediate 100 Hz cycles re-read the SAME sample (seq 5). */
    g_ch5.signed_current_ma = 100;          /* even a "cleared" re-read...   */
    service_at(1010);
    service_at(1020);
    service_at(1030);
    service_at(1040);
    /* ...must NOT confirm anything: seq unchanged → still CONFIRM_WAIT. */
    CHECK(SteeringSupervisor_OcState() == OC_STATE_CONFIRM_WAIT);
    /* Only a genuinely new acquisition (seq 6) may be evaluated. */
    g_ch5.sample_sequence = 6;
    service_at(1050);
    CHECK(SteeringSupervisor_OcState() == OC_STATE_MECHANICAL_CONFIRMED);
    CHECK(!SteeringSupervisor_WantsSafe());
    CHECK(safe_state_calls == 0);
}

/* CH5 MISSING isolates the assist (after debounce) — mechanical-only, no SAFE. */
static void test_io_ch5_missing_isolates(void)
{
    reset_all();
    g_ch5 = healthy_ch5();
    g_ch5.i2c_ack = false; g_ch5.detected_address = 0;
    g_ch5.shunt_read_ok = false; g_ch5.bus_read_ok = false;
    g_ch5.fault_reason = Ina226_ClassifyChannel(&g_ch5);
    CHECK(g_ch5.fault_reason == INA226_CH_MISSING);
    /* Debounce = 3: first two cycles must NOT isolate. */
    service_at(1000);
    service_at(1010);
    CHECK(!Steering_IsMechanicalOnly());
    /* Third consecutive fault isolates. */
    service_at(1020);
    CHECK(Steering_IsMechanicalOnly());
    CHECK(!Steering_IsElectricalHazard());
    CHECK(phys_off_calls > 0 && power_off_calls > 0);
    CHECK(safe_state_calls == 0);            /* NEVER a global SAFE          */
}

/* A single transient miss (then healthy) never isolates. */
static void test_io_single_transient_no_isolate(void)
{
    reset_all();
    Ina226ChannelDiag miss = healthy_ch5();
    miss.i2c_ack = false; miss.shunt_read_ok = false; miss.bus_read_ok = false;
    miss.fault_reason = Ina226_ClassifyChannel(&miss);
    g_ch5 = miss;  service_at(1000);         /* one transient fault          */
    g_ch5 = healthy_ch5(); g_ch5.sample_sequence = 2; service_at(1010);
    g_ch5 = healthy_ch5(); g_ch5.sample_sequence = 3; service_at(1020);
    g_ch5 = healthy_ch5(); g_ch5.sample_sequence = 4; service_at(1030);
    CHECK(!Steering_IsMechanicalOnly());
    CHECK(safe_state_calls == 0);
}

/* CH5 STALE isolates the assist (mechanical-only, no SAFE). */
static void test_io_ch5_stale_isolates(void)
{
    reset_all();
    g_ch5 = healthy_ch5();
    g_ch5.sample_age_ms = INA226_DIAG_STALE_MS + 50;
    g_ch5.fault_reason = Ina226_ClassifyChannel(&g_ch5);
    CHECK(g_ch5.fault_reason == INA226_CH_STALE);
    service_at(1000); service_at(1010); service_at(1020);
    CHECK(Steering_IsMechanicalOnly());
    CHECK(!Steering_IsElectricalHazard());
    CHECK(safe_state_calls == 0);
}

/* CH5 CONFIG invalid isolates the assist. */
static void test_io_ch5_config_isolates(void)
{
    reset_all();
    g_ch5 = healthy_ch5();
    g_ch5.config_readback_ok = false;
    g_ch5.fault_reason = Ina226_ClassifyChannel(&g_ch5);
    CHECK(g_ch5.fault_reason == INA226_CH_CONFIG_LOST);
    service_at(1000); service_at(1010); service_at(1020);
    CHECK(Steering_IsMechanicalOnly());
    CHECK(safe_state_calls == 0);
}

/* CH5 reversed polarity WITH current expected isolates the assist. */
static void test_io_ch5_polarity_isolates(void)
{
    reset_all();
    g_ch5 = healthy_ch5();
    g_ch5.current_expected  = true;
    g_ch5.signed_current_ma = -(INA226_DIAG_REVERSED_MA + 800);
    g_ch5.shunt_uv          = -2000;
    g_ch5.fault_reason = Ina226_ClassifyChannel(&g_ch5);
    CHECK(g_ch5.fault_reason == INA226_CH_POLARITY_REVERSED);
    service_at(1000); service_at(1010); service_at(1020);
    CHECK(Steering_IsMechanicalOnly());
    CHECK(safe_state_calls == 0);
}

/* Overcurrent, then NO fresh post-isolation sample: must stay isolated and
 * ISOLATION UNCONFIRMED — never a false SAFE from missing information. */
static void test_io_overcurrent_no_fresh_sample_no_safe(void)
{
    reset_all();
    g_ch5 = healthy_ch5();
    g_ch5.sample_sequence   = 10;
    g_ch5.signed_current_ma = STEERING_ACTIVE_OVERCURRENT_MA + 5000;
    service_at(1000);                        /* isolate, capture seq 10      */
    CHECK(Steering_IsMechanicalOnly());
    CHECK(SteeringSupervisor_OcState() == OC_STATE_CONFIRM_WAIT);
    /* No new acquisition (seq frozen) well past the confirm window. */
    service_at(1000 + STEERING_OC_CONFIRM_MS + 100);
    CHECK(SteeringSupervisor_OcState() == OC_STATE_ISOLATED_UNCONFIRMED);
    CHECK(!Steering_IsElectricalHazard());
    CHECK(!SteeringSupervisor_WantsSafe());
    CHECK(safe_state_calls == 0);            /* NEVER auto-SAFE              */
}

/* Overcurrent, then a fresh valid sample shows the current fell → mechanical
 * confirmed, still no SAFE. */
static void test_io_overcurrent_cleared_mechanical(void)
{
    reset_all();
    g_ch5 = healthy_ch5();
    g_ch5.sample_sequence   = 20;
    g_ch5.signed_current_ma = STEERING_ACTIVE_OVERCURRENT_MA + 5000;
    service_at(1000);                        /* isolate                      */
    CHECK(SteeringSupervisor_OcState() == OC_STATE_CONFIRM_WAIT);
    /* Fresh sample, current fell below the residual ceiling. */
    g_ch5.sample_sequence   = 21;
    g_ch5.signed_current_ma = 150;
    service_at(1010);
    CHECK(SteeringSupervisor_OcState() == OC_STATE_MECHANICAL_CONFIRMED);
    CHECK(!Steering_IsElectricalHazard());
    CHECK(!SteeringSupervisor_WantsSafe());
    CHECK(safe_state_calls == 0);
}

#if (STEERING_ISOLATION_THRESHOLD_CALIBRATED != 0)
/* Calibrated build only: a persistent fresh post-isolation over-current is a
 * proven electrical hazard → ELECTRICAL_HAZARD + SAFE through the real glue. */
static void test_io_overcurrent_persist_hazard(void)
{
    reset_all();
    g_ch5 = healthy_ch5();
    g_ch5.sample_sequence   = 30;
    g_ch5.signed_current_ma = STEERING_ACTIVE_OVERCURRENT_MA + 5000;
    service_at(1000);                        /* isolate                      */
    /* Fresh sample STILL over the residual ceiling. */
    g_ch5.sample_sequence   = 31;
    g_ch5.signed_current_ma = STEERING_ACTIVE_OVERCURRENT_MA + 8000;
    service_at(1010);
    CHECK(SteeringSupervisor_OcState() == OC_STATE_HAZARD);
    CHECK(Steering_IsElectricalHazard());
    CHECK(SteeringSupervisor_WantsSafe());
    CHECK(safe_state_calls > 0 && last_state_req == SYS_STATE_SAFE);
    CHECK(set_error_calls > 0 && last_error_req == SAFETY_ERROR_OVERCURRENT);
}
#endif

int main(void)
{
    test_io_healthy();
    test_io_pc12_off_normal();
    test_io_sequence_freshness();
    test_io_ch5_missing_isolates();
    test_io_single_transient_no_isolate();
    test_io_ch5_stale_isolates();
    test_io_ch5_config_isolates();
    test_io_ch5_polarity_isolates();
    test_io_overcurrent_no_fresh_sample_no_safe();
    test_io_overcurrent_cleared_mechanical();
#if (STEERING_ISOLATION_THRESHOLD_CALIBRATED != 0)
    test_io_overcurrent_persist_hazard();
#endif

    printf("==== test_steering_supervisor_io: %d run, %d failed ====\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#endif /* HOST_TEST */
