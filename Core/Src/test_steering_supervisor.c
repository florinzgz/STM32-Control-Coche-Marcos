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
    in.ch5_probe_id       = now;
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
 * REAL Ina226_ClassifyChannel(), returning the classified reason.  Each loop
 * pass is a genuinely NEW real acquisition (ch5_probe_id changes), which is
 * what the per-acquisition debounce requires to advance. */
static Ina226DiagReason_t run_ch5_fault(Ina226ChannelDiag d, uint32_t now)
{
    Ina226DiagReason_t reason = Ina226_ClassifyChannel(&d);
    SteeringSupervisorInputs in = healthy_inputs(now);
    in.ch5_reason = reason;
    for (unsigned i = 0; i < STEERING_CH5_FAULT_DEBOUNCE; i++) {
        in.now_ms       = now + i;
        in.ch5_probe_id = now + i;   /* new real acquisition each pass */
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
    SteeringSupervisor_OcInit(&f, 25000, 1000, true, 200U);
    /* Below active limit: nothing. */
    CHECK(SteeringSupervisor_OcStep(&f, 5000, 1, true, 0)   == OC_ACTION_NONE);
    /* Above active limit: isolate. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 2, true, 10) == OC_ACTION_ISOLATE);
    CHECK(f.state == OC_STATE_CONFIRM_WAIT);
    /* Same sample id → no confirmation yet. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 2, true, 15) == OC_ACTION_NONE);
    /* Fresh sample, current below residual → keep mechanical (isolable). */
    CHECK(SteeringSupervisor_OcStep(&f, 500, 3, true, 20)   == OC_ACTION_KEEP_MECHANICAL);
    CHECK(f.state == OC_STATE_MECHANICAL_CONFIRMED);
    /* Terminal — latched. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 4, true, 30) == OC_ACTION_NONE);
}

static void test_oc_fsm_persists(void)
{
    OcFsm_t f;
    /* Calibrated residual ceiling → a persistent fresh over-current escalates. */
    SteeringSupervisor_OcInit(&f, 25000, 1000, true, 200U);
    CHECK(SteeringSupervisor_OcStep(&f, -40000, 2, true, 10) == OC_ACTION_ISOLATE); /* signed */
    /* Fresh sample still over residual (calibrated) → hazard escalation. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 3, true, 20) == OC_ACTION_ESCALATE_HAZARD);
    CHECK(f.state == OC_STATE_HAZARD);
}

static void test_oc_fsm_timeout(void)
{
    OcFsm_t f;
    SteeringSupervisor_OcInit(&f, 25000, 1000, true, 200U);
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 2, true, 10) == OC_ACTION_ISOLATE);
    /* Within the window, no fresh sample yet → nothing. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 2, true, 100) == OC_ACTION_NONE);
    /* Window elapses WITHOUT a fresh valid sample: missing information is NOT
     * proof of a persistent hazard → ISOLATION UNCONFIRMED, never HAZARD/SAFE. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 2, true, 210) == OC_ACTION_ISOLATION_UNCONFIRMED);
    CHECK(f.state == OC_STATE_ISOLATED_UNCONFIRMED);
    /* Still unconfirmed, still no fresh sample → stays isolated, no SAFE. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 2, true, 400) == OC_ACTION_NONE);
    CHECK(f.state == OC_STATE_ISOLATED_UNCONFIRMED);
    /* A future FRESH valid sample below residual finally confirms mechanical. */
    CHECK(SteeringSupervisor_OcStep(&f, 200, 3, true, 500) == OC_ACTION_KEEP_MECHANICAL);
    CHECK(f.state == OC_STATE_MECHANICAL_CONFIRMED);
}

/* Uncalibrated residual ceiling: a persistent fresh over-current must NEVER
 * auto-escalate to SAFE — it stays isolated + unconfirmed instead. */
static void test_oc_fsm_uncalibrated_never_safe(void)
{
    OcFsm_t f;
    SteeringSupervisor_OcInit(&f, 25000, 1000, false, 200U);
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 2, true, 10) == OC_ACTION_ISOLATE);
    /* Fresh sample STILL over residual, but threshold uncalibrated → no hazard. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 3, true, 20) == OC_ACTION_ISOLATION_UNCONFIRMED);
    CHECK(f.state == OC_STATE_ISOLATED_UNCONFIRMED);
    /* Even repeated fresh over-residual samples never escalate while uncal. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 4, true, 30) == OC_ACTION_ISOLATION_UNCONFIRMED);
    /* But a genuine drop below residual still confirms mechanical isolation. */
    CHECK(SteeringSupervisor_OcStep(&f, 100, 5, true, 40) == OC_ACTION_KEEP_MECHANICAL);
    CHECK(f.state == OC_STATE_MECHANICAL_CONFIRMED);
}

/* The confirm step must ignore a stale re-read (same sample id) and only act on
 * a genuinely new acquisition sequence — the 100 Hz / 20 Hz freshness contract. */
static void test_oc_fsm_sequence_freshness(void)
{
    OcFsm_t f;
    SteeringSupervisor_OcInit(&f, 25000, 1000, true, 10000U);
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 7, true, 1000) == OC_ACTION_ISOLATE);
    /* Four intermediate 100 Hz cycles re-read the SAME 20 Hz sample (id 7):
     * none may confirm anything even though current still looks high. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 7, true, 1010) == OC_ACTION_NONE);
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 7, true, 1020) == OC_ACTION_NONE);
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 7, true, 1030) == OC_ACTION_NONE);
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 7, true, 1040) == OC_ACTION_NONE);
    CHECK(f.state == OC_STATE_CONFIRM_WAIT);
    /* Only the next real acquisition (id 8) may be evaluated. */
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 8, true, 1050) == OC_ACTION_ESCALATE_HAZARD);
    CHECK(f.state == OC_STATE_HAZARD);
}

/* A uint32_t sample-sequence wrap (0xFFFFFFFF → 0) is still "fresh" because the
 * FSM compares for inequality, never magnitude. */
static void test_oc_fsm_sequence_wrap(void)
{
    OcFsm_t f;
    SteeringSupervisor_OcInit(&f, 25000, 1000, true, 200U);
    CHECK(SteeringSupervisor_OcStep(&f, 40000, 0xFFFFFFFFU, true, 10) == OC_ACTION_ISOLATE);
    /* Sequence wraps to 0: different id → treated as a fresh sample. */
    CHECK(SteeringSupervisor_OcStep(&f, 500, 0U, true, 20) == OC_ACTION_KEEP_MECHANICAL);
    CHECK(f.state == OC_STATE_MECHANICAL_CONFIRMED);
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
    in.ch5_probe_id = 1;
    /* One transient acquisition must NOT isolate (debounce = 3). */
    SteeringSupervisor_Apply(&in);
    CHECK(!Steering_IsMechanicalOnly());
    /* A healthy acquisition resets the debounce. */
    in.ch5_reason = INA226_CH_OK;
    in.ch5_probe_id = 2;
    SteeringSupervisor_Apply(&in);
    /* Two consecutive faulty acquisitions still short of the threshold. */
    in.ch5_reason = r;
    in.ch5_probe_id = 3;
    SteeringSupervisor_Apply(&in);
    in.ch5_probe_id = 4;
    SteeringSupervisor_Apply(&in);
    CHECK(!Steering_IsMechanicalOnly());
    /* Third consecutive faulty acquisition → isolate. */
    in.ch5_probe_id = 5;
    SteeringSupervisor_Apply(&in);
    CHECK(Steering_IsMechanicalOnly());
}

/* MANDATORY (audit): the 100 Hz supervisor must debounce CH5 faults by REAL
 * acquisition, not by service call.  A single failed 20 Hz acquisition seen
 * across five 100 Hz cycles (ch5_probe_id frozen) must count as ONE failure;
 * a subsequent healthy acquisition resets; only THREE consecutive failed
 * acquisitions isolate the assist. */
static void test_ch5_debounce_by_acquisition(void)
{
    reset_all();
    Ina226ChannelDiag d = base_ok_ch5();
    d.i2c_ack = false; d.detected_address = 0;
    d.shunt_read_ok = false; d.bus_read_ok = false;
    Ina226DiagReason_t r = Ina226_ClassifyChannel(&d);
    CHECK(r == INA226_CH_MISSING);

    /* ---- One failed acquisition, re-read by five 100 Hz service cycles ---- */
    SteeringSupervisorInputs in = healthy_inputs(1000);
    in.ch5_reason       = r;
    in.ch5_sample_valid = false;
    in.ch5_probe_id     = 100;     /* one real (failed) acquisition */
    for (unsigned i = 0; i < 5; i++) {
        in.now_ms = 1000 + i;      /* five 100 Hz cycles, SAME probe */
        SteeringSupervisor_Apply(&in);
    }
    /* Only ONE failure counted → nowhere near the debounce of 3. */
    CHECK(!Steering_IsMechanicalOnly());

    /* ---- A healthy acquisition resets the debounce ---- */
    in.ch5_reason       = INA226_CH_OK;
    in.ch5_sample_valid = true;
    in.ch5_probe_id     = 101;
    in.now_ms           = 1010;
    SteeringSupervisor_Apply(&in);
    CHECK(!Steering_IsMechanicalOnly());

    /* ---- Only three consecutive FAILED acquisitions isolate ---- */
    in.ch5_reason       = r;
    in.ch5_sample_valid = false;
    /* First failed acquisition (again re-read five times → still one count). */
    in.ch5_probe_id = 102;
    for (unsigned i = 0; i < 5; i++) { in.now_ms = 1020 + i; SteeringSupervisor_Apply(&in); }
    CHECK(!Steering_IsMechanicalOnly());
    /* Second failed acquisition. */
    in.ch5_probe_id = 103;
    for (unsigned i = 0; i < 5; i++) { in.now_ms = 1030 + i; SteeringSupervisor_Apply(&in); }
    CHECK(!Steering_IsMechanicalOnly());
    /* Third consecutive failed acquisition → isolate now. */
    in.ch5_probe_id = 104;
    in.now_ms = 1040;
    SteeringSupervisor_Apply(&in);
    CHECK(Steering_IsMechanicalOnly());
    CHECK(!Steering_IsElectricalHazard());
    CHECK(!SteeringSupervisor_WantsSafe());
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
    CHECK(SteeringSupervisor_OcState() == OC_STATE_CONFIRM_WAIT);
    /* Fresh sample, current fell below residual → stay mechanical, NO hazard. */
    in.now_ms         = 1010;
    in.ch5_sample_id  = 1010;
    in.ch5_current_ma = 200;
    SteeringSupervisor_Apply(&in);
    CHECK(SteeringSupervisor_OcState() == OC_STATE_MECHANICAL_CONFIRMED);
    CHECK(!Steering_IsElectricalHazard());
    CHECK(!SteeringSupervisor_WantsSafe());
}

/* Production default: the post-isolation residual threshold is UNCALIBRATED
 * (STEERING_ISOLATION_THRESHOLD_CALIBRATED == 0).  A fresh sample that still
 * looks over-current must therefore NEVER auto-escalate to ELECTRICAL_HAZARD /
 * SAFE — the assist stays isolated (MECHANICAL_ONLY) and reports the isolation
 * as unconfirmed.  Only a calibrated build may declare a hazard.            */
static void test_supervisor_overcurrent_uncalibrated_no_safe(void)
{
    /* This end-to-end guard only holds while the shipped build is uncalibrated. */
#if (STEERING_ISOLATION_THRESHOLD_CALIBRATED == 0)
    reset_all();
    SteeringSupervisorInputs in = healthy_inputs(1000);
    in.ch5_current_ma = STEERING_OC_LIMIT_MA + 5000;
    in.ch5_sample_id  = 1000;
    SteeringSupervisor_Apply(&in);                 /* isolate */
    CHECK(Steering_IsMechanicalOnly());
    /* Fresh sample STILL over current, but threshold uncalibrated. */
    in.now_ms         = 1010;
    in.ch5_sample_id  = 1010;
    in.ch5_current_ma = STEERING_OC_LIMIT_MA + 8000;
    SteeringSupervisor_Apply(&in);
    CHECK(SteeringSupervisor_OcState() == OC_STATE_ISOLATED_UNCONFIRMED);
    CHECK(!Steering_IsElectricalHazard());         /* NOT a hazard */
    CHECK(Steering_IsMechanicalOnly());            /* still isolated only */
    CHECK(!SteeringSupervisor_WantsSafe());        /* never auto-SAFE */
#endif
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
    test_oc_fsm_uncalibrated_never_safe();
    test_oc_fsm_sequence_freshness();
    test_oc_fsm_sequence_wrap();

    test_ch5_missing_isolates();
    test_ch5_mux_isolates();
    test_ch5_stale_isolates();
    test_ch5_config_isolates();
    test_ch5_polarity_isolates();
    test_ch5_debounce();
    test_ch5_debounce_by_acquisition();

    test_params_corrupt_isolates();
    test_params_fresh_ok();
    test_cal_corrupt_isolates();
    test_cal_unrecoverable_isolates();
    test_z_mandatory_isolates();
    test_z_optional_no_fault();

    test_supervisor_overcurrent_clears();
    test_supervisor_overcurrent_uncalibrated_no_safe();

    printf("==== test_steering_supervisor: %d run, %d failed ====\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}

#endif /* HOST_TEST */
