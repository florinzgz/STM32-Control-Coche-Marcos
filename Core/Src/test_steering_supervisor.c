/**
  ****************************************************************************
  * @file    test_steering_supervisor.c
  * @brief   Host test for the EPS assist supervisor (steering_supervisor.c).
  *
  *          Proves the audit acceptance chain WITHOUT calling
  *          Steering_DisableAssistFault() directly: faults are injected into
  *          the REAL detectors —
  *            - Ina226_ClassifyChannel() (the genuine CH5 classifier), and
  *            - the real EPS parameter / calibration / Z policy inputs —
  *          and the supervisor is left to drive the real, idempotent
  *          steering_eps.c isolation API.
  *
  *          Acceptance for every isolable cause:
  *            EPS MECHANICAL_ONLY · latched · owner NONE · NOT a hazard.
  *          Overcurrent that persists on a fresh CH5 sample must instead reach
  *          ELECTRICAL_HAZARD and request the SAFE/ERROR escalation.
  *
  *          Compile (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 \
  *                Core/Src/test_steering_supervisor.c \
  *                Core/Src/steering_supervisor.c \
  *                Core/Src/steering_eps.c \
  *                Core/Src/ina226_channel_diag.c \
  *                -lm -o /tmp/test_steering_supervisor && \
  *            /tmp/test_steering_supervisor
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
#include "ina226_channel_diag.h"

/* ---- Test harness ---------------------------------------------------- */
static int tests_run = 0, tests_failed = 0;
#define CHECK(expr) do {                                              \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);        \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ---- steering_eps.c physical-shutdown externs (observed) ------------- */
static int phys_off_calls = 0;
static int power_off_calls = 0;
void Steering_PhysicalOff(void)   { phys_off_calls++; }
void Steering_SteerPowerOff(void) { power_off_calls++; }

/* ---- Fresh EPS + supervisor state before each scenario --------------- */
static void reset_all(void)
{
    Steering_EpsInit();
    SteeringSupervisor_Init();
    phys_off_calls = 0;
    power_off_calls = 0;
}

/* ---- Real CH5 classifier helpers ------------------------------------- */
static Ina226ChannelDiag base_ok_ch5(void)
{
    Ina226ChannelDiag d;
    memset(&d, 0, sizeof(d));
    d.channel            = 5;
    d.mux_channel        = 5;
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
    d.raw_shunt          = 1800;
    d.shunt_uv           = 3510;
    d.bus_mv             = 12200;
    d.signed_current_ma  = 2340;
    d.sample_age_ms      = 20;
    d.channel_powered    = true;
    d.current_expected   = true;
    return d;
}

/* Build a supervisor input snapshot with everything healthy. */
static SteeringSupervisorInputs healthy_inputs(uint32_t now)
{
    SteeringSupervisorInputs in;
    memset(&in, 0, sizeof(in));
    in.ch5_reason         = INA226_CH_OK;
    in.ch5_current_ma     = 2340;
    in.ch5_sample_id      = now;
    in.ch5_sample_valid   = true;
    in.params_flash_present = true;
    in.params_flash_valid   = true;
    in.cal_flash_present    = true;
    in.cal_flash_corrupt    = false;
    in.cal_valid            = true;
    in.centering_finished   = true;
    in.centering_recovered  = true;
    in.z_required           = false;
    in.z_center_known       = true;
    in.z_status             = STEERING_Z_OK;
    in.now_ms               = now;
    return in;
}

/* Drive the CH5 debounce past its threshold with a snapshot classified by the
 * REAL Ina226_ClassifyChannel(), returning the classified reason. */
static Ina226DiagReason_t run_ch5_fault(Ina226ChannelDiag d, uint32_t now)
{
    Ina226DiagReason_t reason = Ina226_ClassifyChannel(&d);
    SteeringSupervisorInputs in = healthy_inputs(now);
    in.ch5_reason = reason;
    for (unsigned i = 0; i < STEERING_CH5_FAULT_DEBOUNCE; i++) {
        in.now_ms = now + i;
        SteeringSupervisor_Apply(&in);
    }
    return reason;
}

/* ---- Pure-function unit tests ---------------------------------------- */
static void test_ch5_mapping(void)
{
    CHECK(SteeringSupervisor_Ch5ToEpsFault(INA226_CH_MISSING)        == EPS_FAULT_CH5_MISSING);
    CHECK(SteeringSupervisor_Ch5ToEpsFault(INA226_CH_MUX_SELECT_FAIL)== EPS_FAULT_CH5_MISSING);
    CHECK(SteeringSupervisor_Ch5ToEpsFault(INA226_CH_READ_FAIL)      == EPS_FAULT_CH5_MISSING);
    CHECK(SteeringSupervisor_Ch5ToEpsFault(INA226_CH_STALE)          == EPS_FAULT_CH5_STALE);
    CHECK(SteeringSupervisor_Ch5ToEpsFault(INA226_CH_CONFIG_LOST)    == EPS_FAULT_CH5_CONFIG);
    CHECK(SteeringSupervisor_Ch5ToEpsFault(INA226_CH_WRONG_ID)       == EPS_FAULT_CH5_CONFIG);
    CHECK(SteeringSupervisor_Ch5ToEpsFault(INA226_CH_POLARITY_REVERSED) == EPS_FAULT_DIRECTION_POLARITY);
    CHECK(SteeringSupervisor_Ch5ToEpsFault(INA226_CH_OK)             == EPS_FAULT_NONE);
    CHECK(SteeringSupervisor_Ch5ToEpsFault(INA226_CH_PRESENT_NO_SHUNT) == EPS_FAULT_NONE);
}

static void test_params_policy(void)
{
    /* fresh device — no slot: defaults authorised, no fault. */
    CHECK(SteeringSupervisor_ParamsPolicy(false, false) == EPS_FAULT_NONE);
    /* valid persisted slot: no fault. */
    CHECK(SteeringSupervisor_ParamsPolicy(true, true)  == EPS_FAULT_NONE);
    /* existing but corrupt slot: isolate. */
    CHECK(SteeringSupervisor_ParamsPolicy(true, false) == EPS_FAULT_PARAMETERS_INVALID);
}

static void test_cal_policy(void)
{
    /* fresh, homing not finished yet → no fault. */
    CHECK(SteeringSupervisor_CalPolicy(false, false, false, false, false) == EPS_FAULT_NONE);
    /* corrupt existing calibration → isolate immediately. */
    CHECK(SteeringSupervisor_CalPolicy(true, true, false, false, false) == EPS_FAULT_CALIBRATION_INVALID);
    /* homing finished, recovered a centre → no fault. */
    CHECK(SteeringSupervisor_CalPolicy(false, false, true, true, true) == EPS_FAULT_NONE);
    /* homing finished, could NOT recover → isolate. */
    CHECK(SteeringSupervisor_CalPolicy(false, false, false, true, false) == EPS_FAULT_CALIBRATION_INVALID);
}

static void test_z_policy(void)
{
    /* optional Z: never a fault regardless of status. */
    CHECK(SteeringSupervisor_ZPolicy(false, true, STEERING_Z_NOT_SEEN)     == EPS_FAULT_NONE);
    CHECK(SteeringSupervisor_ZPolicy(false, true, STEERING_Z_MECH_OFFSET)  == EPS_FAULT_NONE);
    /* mandatory Z but centre not confirmed yet: not a fault. */
    CHECK(SteeringSupervisor_ZPolicy(true, false, STEERING_Z_NOT_SEEN)     == EPS_FAULT_NONE);
    /* mandatory Z, centre known, Z missing/incoherent: isolate. */
    CHECK(SteeringSupervisor_ZPolicy(true, true, STEERING_Z_NOT_SEEN)      == EPS_FAULT_ENCODER_Z);
    CHECK(SteeringSupervisor_ZPolicy(true, true, STEERING_Z_OUT_OF_WINDOW) == EPS_FAULT_ENCODER_Z);
    CHECK(SteeringSupervisor_ZPolicy(true, true, STEERING_Z_MECH_OFFSET)   == EPS_FAULT_ENCODER_Z);
    /* mandatory Z, coherent: no fault. */
    CHECK(SteeringSupervisor_ZPolicy(true, true, STEERING_Z_OK)            == EPS_FAULT_NONE);
}

/* ---- Overcurrent FSM unit tests -------------------------------------- */
static void test_oc_fsm_clears(void)
{
    OcFsm_t f;
    SteeringSupervisor_OcInit(&f, 25000, 200U);
    /* Below limit: nothing. */
    CHECK(SteeringSupervisor_OcStep(&f, 5000, 1, true, 0)   == OC_ACTION_NONE);
    /* Above limit: isolate. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 2, true, 10) == OC_ACTION_ISOLATE);
    CHECK(f.state == OC_STATE_CONFIRM);
    /* Same sample id → no confirmation yet. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 2, true, 15) == OC_ACTION_NONE);
    /* Fresh sample, current fell → keep mechanical (isolable). */
    CHECK(SteeringSupervisor_OcStep(&f, 1000, 3, true, 20)  == OC_ACTION_KEEP_MECHANICAL);
    CHECK(f.state == OC_STATE_MECHANICAL);
    /* Terminal — latched. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 4, true, 30) == OC_ACTION_NONE);
}

static void test_oc_fsm_persists(void)
{
    OcFsm_t f;
    SteeringSupervisor_OcInit(&f, 25000, 200U);
    CHECK(SteeringSupervisor_OcStep(&f, -40000, 2, true, 10) == OC_ACTION_ISOLATE); /* signed */
    /* Fresh sample still over limit → hazard escalation. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 3, true, 20) == OC_ACTION_ESCALATE_HAZARD);
    CHECK(f.state == OC_STATE_HAZARD);
}

static void test_oc_fsm_timeout(void)
{
    OcFsm_t f;
    SteeringSupervisor_OcInit(&f, 25000, 200U);
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 2, true, 10) == OC_ACTION_ISOLATE);
    /* No fresh valid sample arrives within the window → hazard. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 2, true, 100) == OC_ACTION_NONE);
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 2, true, 210) == OC_ACTION_ESCALATE_HAZARD);
    CHECK(f.state == OC_STATE_HAZARD);
}

/* ---- Integration: real classifier → real EPS isolation --------------- */
static void assert_isolated_mechanical(EpsFaultReason_t expect)
{
    CHECK(Steering_IsMechanicalOnly());
    CHECK(Steering_IsAssistLatchedOff());
    CHECK(!Steering_IsElectricalHazard());
    CHECK(Steering_GetEpsState() == EPS_STATE_MECHANICAL_ONLY);
    CHECK(Steering_EpsGetOwner() == STEER_OWNER_NONE);
    CHECK(Steering_GetEpsFault() == expect);
    CHECK(SteeringSupervisor_LastCause() == expect);
    CHECK(!SteeringSupervisor_WantsSafe());
    CHECK(phys_off_calls > 0);
    CHECK(power_off_calls > 0);
}

static void test_ch5_missing_isolates(void)
{
    reset_all();
    Ina226ChannelDiag d = base_ok_ch5();
    d.i2c_ack = false; d.detected_address = 0;   /* real detector: no ACK */
    CHECK(run_ch5_fault(d, 1000) == INA226_CH_MISSING);
    assert_isolated_mechanical(EPS_FAULT_CH5_MISSING);
}

static void test_ch5_mux_isolates(void)
{
    reset_all();
    Ina226ChannelDiag d = base_ok_ch5();
    d.mux_select_ok = false;                     /* TCA9548A select failed */
    CHECK(run_ch5_fault(d, 1000) == INA226_CH_MUX_SELECT_FAIL);
    assert_isolated_mechanical(EPS_FAULT_CH5_MISSING);
}

static void test_ch5_stale_isolates(void)
{
    reset_all();
    Ina226ChannelDiag d = base_ok_ch5();
    d.sample_age_ms = INA226_DIAG_STALE_MS + 10;
    CHECK(run_ch5_fault(d, 5000) == INA226_CH_STALE);
    assert_isolated_mechanical(EPS_FAULT_CH5_STALE);
}

static void test_ch5_config_isolates(void)
{
    reset_all();
    Ina226ChannelDiag d = base_ok_ch5();
    d.config_readback_ok = false;                /* config invalid */
    CHECK(run_ch5_fault(d, 1000) == INA226_CH_CONFIG_LOST);
    assert_isolated_mechanical(EPS_FAULT_CH5_CONFIG);
}

static void test_ch5_polarity_isolates(void)
{
    reset_all();
    Ina226ChannelDiag d = base_ok_ch5();
    d.signed_current_ma = -(INA226_DIAG_REVERSED_MA + 500); /* reversed VIN */
    d.shunt_uv = -1500;
    CHECK(run_ch5_fault(d, 1000) == INA226_CH_POLARITY_REVERSED);
    assert_isolated_mechanical(EPS_FAULT_DIRECTION_POLARITY);
}

static void test_ch5_debounce(void)
{
    reset_all();
    Ina226ChannelDiag d = base_ok_ch5();
    d.i2c_ack = false; d.detected_address = 0;
    Ina226DiagReason_t r = Ina226_ClassifyChannel(&d);
    SteeringSupervisorInputs in = healthy_inputs(1000);
    in.ch5_reason = r;
    /* One transient cycle must NOT isolate (debounce = 3). */
    SteeringSupervisor_Apply(&in);
    CHECK(!Steering_IsMechanicalOnly());
    /* A healthy cycle resets the debounce. */
    in.ch5_reason = INA226_CH_OK;
    SteeringSupervisor_Apply(&in);
    /* Two consecutive faults still short of the threshold: no isolation. */
    in.ch5_reason = r;
    SteeringSupervisor_Apply(&in);
    SteeringSupervisor_Apply(&in);
    CHECK(!Steering_IsMechanicalOnly());
    /* Third consecutive fault → isolate. */
    SteeringSupervisor_Apply(&in);
    CHECK(Steering_IsMechanicalOnly());
}

static void test_params_corrupt_isolates(void)
{
    reset_all();
    SteeringSupervisorInputs in = healthy_inputs(1000);
    in.params_flash_present = true;
    in.params_flash_valid   = false;   /* existing corrupt flash */
    SteeringSupervisor_Apply(&in);
    assert_isolated_mechanical(EPS_FAULT_PARAMETERS_INVALID);
}

static void test_params_fresh_ok(void)
{
    reset_all();
    SteeringSupervisorInputs in = healthy_inputs(1000);
    in.params_flash_present = false;   /* fresh device */
    in.params_flash_valid   = false;
    SteeringSupervisor_Apply(&in);
    CHECK(!Steering_IsMechanicalOnly());
    CHECK(Steering_GetEpsFault() == EPS_FAULT_NONE);
}

static void test_cal_corrupt_isolates(void)
{
    reset_all();
    SteeringSupervisorInputs in = healthy_inputs(1000);
    in.cal_flash_present = true;
    in.cal_flash_corrupt = true;
    in.cal_valid         = false;
    SteeringSupervisor_Apply(&in);
    assert_isolated_mechanical(EPS_FAULT_CALIBRATION_INVALID);
}

static void test_cal_unrecoverable_isolates(void)
{
    reset_all();
    SteeringSupervisorInputs in = healthy_inputs(1000);
    in.cal_flash_present   = false;
    in.cal_flash_corrupt   = false;
    in.cal_valid           = false;
    in.centering_finished  = true;    /* homing tried and failed */
    in.centering_recovered = false;
    SteeringSupervisor_Apply(&in);
    assert_isolated_mechanical(EPS_FAULT_CALIBRATION_INVALID);
}

static void test_z_mandatory_isolates(void)
{
    reset_all();
    SteeringSupervisorInputs in = healthy_inputs(1000);
    in.z_required     = true;
    in.z_center_known = true;
    in.z_status       = STEERING_Z_NOT_SEEN;
    SteeringSupervisor_Apply(&in);
    assert_isolated_mechanical(EPS_FAULT_ENCODER_Z);
}

static void test_z_optional_no_fault(void)
{
    reset_all();
    SteeringSupervisorInputs in = healthy_inputs(1000);
    in.z_required     = false;
    in.z_center_known = true;
    in.z_status       = STEERING_Z_NOT_SEEN;
    SteeringSupervisor_Apply(&in);
    CHECK(!Steering_IsMechanicalOnly());
    CHECK(Steering_GetEpsFault() == EPS_FAULT_NONE);
}

/* Overcurrent through the full supervisor: cleared vs persistent. */
static void test_supervisor_overcurrent_clears(void)
{
    reset_all();
    SteeringSupervisorInputs in = healthy_inputs(1000);
    in.ch5_current_ma   = STEERING_OC_LIMIT_MA + 5000;
    in.ch5_sample_id    = 1000;
    SteeringSupervisor_Apply(&in);                 /* isolate */
    CHECK(Steering_IsMechanicalOnly());
    CHECK(SteeringSupervisor_OcState() == OC_STATE_CONFIRM);
    /* Fresh sample, current fell → stay mechanical, NO hazard. */
    in.now_ms         = 1010;
    in.ch5_sample_id  = 1010;
    in.ch5_current_ma = 1000;
    SteeringSupervisor_Apply(&in);
    CHECK(SteeringSupervisor_OcState() == OC_STATE_MECHANICAL);
    CHECK(!Steering_IsElectricalHazard());
    CHECK(!SteeringSupervisor_WantsSafe());
}

static void test_supervisor_overcurrent_hazard(void)
{
    reset_all();
    SteeringSupervisorInputs in = healthy_inputs(1000);
    in.ch5_current_ma = STEERING_OC_LIMIT_MA + 5000;
    in.ch5_sample_id  = 1000;
    SteeringSupervisor_Apply(&in);                 /* isolate */
    CHECK(Steering_IsMechanicalOnly());
    /* Fresh sample still over limit → hazard + SAFE escalation request. */
    in.now_ms         = 1010;
    in.ch5_sample_id  = 1010;
    in.ch5_current_ma = STEERING_OC_LIMIT_MA + 8000;
    SteeringSupervisor_Apply(&in);
    CHECK(SteeringSupervisor_OcState() == OC_STATE_HAZARD);
    CHECK(Steering_IsElectricalHazard());
    CHECK(!Steering_IsMechanicalOnly());           /* hazard, not benign */
    CHECK(SteeringSupervisor_WantsSafe());
    CHECK(Steering_GetEpsFault() == EPS_FAULT_OVERCURRENT);
}

int main(void)
{
    test_ch5_mapping();
    test_params_policy();
    test_cal_policy();
    test_z_policy();
    test_oc_fsm_clears();
    test_oc_fsm_persists();
    test_oc_fsm_timeout();

    test_ch5_missing_isolates();
    test_ch5_mux_isolates();
    test_ch5_stale_isolates();
    test_ch5_config_isolates();
    test_ch5_polarity_isolates();
    test_ch5_debounce();

    test_params_corrupt_isolates();
    test_params_fresh_ok();
    test_cal_corrupt_isolates();
    test_cal_unrecoverable_isolates();
    test_z_mandatory_isolates();
    test_z_optional_no_fault();

    test_supervisor_overcurrent_clears();
    test_supervisor_overcurrent_hazard();

    printf("==== test_steering_supervisor: %d run, %d failed ====\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#endif /* HOST_TEST */
