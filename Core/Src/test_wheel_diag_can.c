/**
  ****************************************************************************
  * @file    test_wheel_diag_can.c
  * @brief   Unit tests for the 0x313 DIAG_WHEEL_SENSOR frame packing and the
  *          WheelDiag_t -> CAN reason-code mapping.
  *
  *          This mirrors Safety_WheelDiagToCanReason() and the byte layout
  *          built by CAN_SendWheelSensorDiag() (Core/Src/can_handler.c) as a
  *          self-contained host replica (same style as test_wheel_fault_gate.c)
  *          so it can run on host GCC without the full HAL / CAN stack.
  *
  *          Compile with host GCC (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE \
  *                -Ianalysis_artifacts/stubs -ICore/Inc -O2 -lm \
  *                Core/Src/test_wheel_diag_can.c -o /tmp/test_wheel_diag_can
  ****************************************************************************
  */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* ---- WheelDiag_t codes mirrored from safety_system.h ---- */
typedef enum {
    WHEEL_DIAG_OK              = 0,
    WHEEL_DIAG_NO_PULSE       = 1,
    WHEEL_DIAG_STUCK_HIGH     = 2,
    WHEEL_DIAG_STUCK_LOW      = 3,
    WHEEL_DIAG_MISMATCH       = 4,
    WHEEL_DIAG_IMPOSSIBLE_RATE = 5,
    WHEEL_DIAG_MANUAL_MOVEMENT = 6,
    WHEEL_DIAG_DISABLED_STATE  = 7
} WheelDiag_t;

#define WHEEL_DIAG_CAN_UNKNOWN   8U
#define NUM_WHEELS               4
#define FAULT_WHEEL_SENSOR       (1U << 4)

/* Flag bits (0x313 byte 7 low nibble). */
#define FLAG_POWERTRAIN   (1U << 0)
#define FLAG_MANUAL       (1U << 1)
#define FLAG_DEBOUNCING   (1U << 2)
#define FLAG_LATCHED      (1U << 3)

/* ---- Replica of Safety_WheelDiagToCanReason() ---- */
static uint8_t diag_to_can_reason(WheelDiag_t d)
{
    if ((unsigned)d <= (unsigned)WHEEL_DIAG_DISABLED_STATE) return (uint8_t)d;
    return WHEEL_DIAG_CAN_UNKNOWN;
}

/* ---- Replica of CAN_SendWheelSensorDiag() packing ----
 * Inputs model the safety diagnostics for one 1 Hz emission:
 *   diag[4]        : per-wheel WheelDiag_t (FL,FR,RL,RR)
 *   gpio[4]        : raw pin level (0/1) per wheel
 *   fault_mask     : Safety_GetWheelFaultMask() (bit0..3)
 *   powertrain     : Safety_IsPowertrainEngaged()
 *   wheel_latched  : Safety_GetFaultFlags() & FAULT_WHEEL_SENSOR
 *   seq            : sequence counter value used for this frame
 * Writes 8 bytes into `out`. */
static void pack_wheel_diag(const WheelDiag_t diag[4], const uint8_t gpio[4],
                            uint8_t fault_mask, bool powertrain,
                            bool wheel_latched, uint8_t seq, uint8_t out[8])
{
    for (int i = 0; i < 8; i++) out[i] = 0;

    uint8_t gpio_mask = 0, manual_seen = 0, debouncing = 0;
    for (uint8_t i = 0; i < NUM_WHEELS && i < 4U; i++) {
        out[i] = diag_to_can_reason(diag[i]);
        if (gpio[i] == 1U) gpio_mask |= (uint8_t)(1U << i);
        if (diag[i] == WHEEL_DIAG_MANUAL_MOVEMENT) manual_seen = 1U;
        if (diag[i] == WHEEL_DIAG_MISMATCH)        debouncing = 1U;
    }
    out[4] = WHEEL_DIAG_OK;      /* STEER/CENTER reserved */
    out[5] = gpio_mask;
    out[6] = fault_mask;

    uint8_t flags = 0;
    if (powertrain)    flags |= FLAG_POWERTRAIN;
    if (manual_seen)   flags |= FLAG_MANUAL;
    if (debouncing)    flags |= FLAG_DEBOUNCING;
    if (wheel_latched) flags |= FLAG_LATCHED;
    flags |= (uint8_t)((seq & 0x0FU) << 4);
    out[7] = flags;
}

/* ---- Replica of Safety_GetWheelLatchedReason() ----
 * While the channel's service-mode fault is latched AND a culprit reason was
 * captured, return the captured culprit; otherwise return the live reason.
 * This models the persistence that keeps the aborting channel identifiable
 * after wheel_diag[i] self-heals back to OK. */
static WheelDiag_t latched_reason(WheelDiag_t live, WheelDiag_t latched,
                                  bool fault_latched)
{
    if (fault_latched && latched != WHEEL_DIAG_OK) return latched;
    return live;
}

/* ---- Replica of Safety_GetSteerDiagReason() ----
 * DISABLED_STATE when the steering-encoder module is off, else OK. */
static uint8_t steer_reason(bool steer_module_enabled)
{
    return steer_module_enabled ? (uint8_t)WHEEL_DIAG_OK
                                : (uint8_t)WHEEL_DIAG_DISABLED_STATE;
}

/* ---- Test framework ---- */
static int tests_run = 0, tests_failed = 0;
#define CHECK(cond, msg) do {                                     \
        tests_run++;                                              \
        if (!(cond)) { tests_failed++;                            \
            printf("  FAIL: %s\n", msg); }                        \
        else { printf("  ok:   %s\n", msg); }                     \
    } while (0)

int main(void)
{
    printf("=== test_wheel_diag_can ===\n");
    uint8_t f[8];

    /* Reason-code mapping is a stable 1:1 for 0-7, UNKNOWN for out-of-range. */
    CHECK(diag_to_can_reason(WHEEL_DIAG_OK) == 0, "reason OK=0");
    CHECK(diag_to_can_reason(WHEEL_DIAG_NO_PULSE) == 1, "reason NO_PULSE=1");
    CHECK(diag_to_can_reason(WHEEL_DIAG_STUCK_HIGH) == 2, "reason STUCK_HIGH=2");
    CHECK(diag_to_can_reason(WHEEL_DIAG_STUCK_LOW) == 3, "reason STUCK_LOW=3");
    CHECK(diag_to_can_reason(WHEEL_DIAG_MISMATCH) == 4, "reason MISMATCH=4");
    CHECK(diag_to_can_reason(WHEEL_DIAG_IMPOSSIBLE_RATE) == 5, "reason IMPOSSIBLE_RATE=5");
    CHECK(diag_to_can_reason(WHEEL_DIAG_MANUAL_MOVEMENT) == 6, "reason MANUAL_MOVEMENT=6");
    CHECK(diag_to_can_reason(WHEEL_DIAG_DISABLED_STATE) == 7, "reason DISABLED_STATE=7");
    CHECK(diag_to_can_reason((WheelDiag_t)42) == WHEEL_DIAG_CAN_UNKNOWN, "out-of-range -> UNKNOWN=8");

    /* Case 1: STANDBY, all wheels stopped -> OK, no fault, no flags. */
    {
        WheelDiag_t d[4] = { WHEEL_DIAG_OK, WHEEL_DIAG_OK, WHEEL_DIAG_OK, WHEEL_DIAG_OK };
        uint8_t g[4] = { 0, 1, 0, 1 };
        pack_wheel_diag(d, g, 0x00, false, false, 0, f);
        CHECK(f[0]==0 && f[1]==0 && f[2]==0 && f[3]==0, "standby: all reasons OK");
        CHECK(f[4]==0, "standby: steer reserved 0");
        CHECK(f[5]==0x0A, "standby: gpio mask reflects pin levels (FR,RR high)");
        CHECK(f[6]==0x00, "standby: no active fault");
        CHECK((f[7] & 0x0F)==0x00, "standby: no flags set");
    }

    /* Case 2: STANDBY, one wheel hand-spun -> MANUAL on that wheel, no fault. */
    {
        WheelDiag_t d[4] = { WHEEL_DIAG_MANUAL_MOVEMENT, WHEEL_DIAG_OK,
                             WHEEL_DIAG_OK, WHEEL_DIAG_OK };
        uint8_t g[4] = { 0, 0, 0, 0 };
        pack_wheel_diag(d, g, 0x00, false, false, 0, f);
        CHECK(f[0]==6, "manual: FL reason MANUAL_MOVEMENT");
        CHECK(f[6]==0x00, "manual: fault mask empty");
        CHECK((f[7] & FLAG_MANUAL)!=0, "manual: manual_movement flag set");
        CHECK((f[7] & FLAG_LATCHED)==0, "manual: not latched");
        CHECK((f[7] & FLAG_POWERTRAIN)==0, "manual: powertrain not engaged");
    }

    /* Case 3: ACTIVE, throttle <3% (no powertrain) behaves like manual. */
    {
        WheelDiag_t d[4] = { WHEEL_DIAG_MANUAL_MOVEMENT, WHEEL_DIAG_OK,
                             WHEEL_DIAG_OK, WHEEL_DIAG_OK };
        uint8_t g[4] = { 1, 0, 0, 0 };
        pack_wheel_diag(d, g, 0x00, false, false, 0, f);
        CHECK(f[6]==0x00 && (f[7]&FLAG_LATCHED)==0, "active low-throttle: no fault/latch");
        CHECK((f[7]&FLAG_MANUAL)!=0, "active low-throttle: manual flag set");
    }

    /* Case 4: ACTIVE under load, FR silent -> NO_PULSE, fault bit1, latched. */
    {
        WheelDiag_t d[4] = { WHEEL_DIAG_OK, WHEEL_DIAG_NO_PULSE,
                             WHEEL_DIAG_OK, WHEEL_DIAG_OK };
        uint8_t g[4] = { 0, 0, 0, 0 };
        pack_wheel_diag(d, g, 0x02, true, true, 3, f);
        CHECK(f[1]==1, "load: FR reason NO_PULSE");
        CHECK(f[6]==0x02, "load: fault mask bit1 (FR)");
        CHECK((f[7]&FLAG_POWERTRAIN)!=0, "load: powertrain engaged flag");
        CHECK((f[7]&FLAG_LATCHED)!=0, "load: wheel_fault latched flag");
        CHECK(((f[7]>>4)&0x0F)==3, "load: sequence counter packed in high nibble");
    }

    /* Case 5: mismatch under load, debounce armed but not latched. */
    {
        WheelDiag_t d[4] = { WHEEL_DIAG_OK, WHEEL_DIAG_OK,
                             WHEEL_DIAG_MISMATCH, WHEEL_DIAG_OK };
        uint8_t g[4] = { 0, 0, 0, 0 };
        pack_wheel_diag(d, g, 0x00, true, false, 0, f);
        CHECK(f[2]==4, "debounce: RL reason MISMATCH");
        CHECK((f[7]&FLAG_DEBOUNCING)!=0, "debounce: debouncing flag set");
        CHECK((f[7]&FLAG_LATCHED)==0, "debounce: not yet latched");
    }

    /* Case 6: RR stuck-low under load. */
    {
        WheelDiag_t d[4] = { WHEEL_DIAG_OK, WHEEL_DIAG_OK,
                             WHEEL_DIAG_OK, WHEEL_DIAG_STUCK_LOW };
        uint8_t g[4] = { 0, 0, 0, 0 };
        pack_wheel_diag(d, g, 0x08, true, true, 0, f);
        CHECK(f[3]==3, "stuck-low: RR reason STUCK_LOW");
        CHECK(f[6]==0x08, "stuck-low: fault mask bit3 (RR)");
    }

    /* Case 7: RL stuck-high under load, pin high shows in gpio mask. */
    {
        WheelDiag_t d[4] = { WHEEL_DIAG_OK, WHEEL_DIAG_OK,
                             WHEEL_DIAG_STUCK_HIGH, WHEEL_DIAG_OK };
        uint8_t g[4] = { 0, 0, 1, 0 };
        pack_wheel_diag(d, g, 0x04, true, true, 0, f);
        CHECK(f[2]==2, "stuck-high: RL reason STUCK_HIGH");
        CHECK((f[5]&0x04)!=0, "stuck-high: RL gpio level high");
        CHECK(f[6]==0x04, "stuck-high: fault mask bit2 (RL)");
    }

    /* Case 8: disabled channel -> DISABLED_STATE, never a fault. */
    {
        WheelDiag_t d[4] = { WHEEL_DIAG_DISABLED_STATE, WHEEL_DIAG_OK,
                             WHEEL_DIAG_OK, WHEEL_DIAG_OK };
        uint8_t g[4] = { 0, 0, 0, 0 };
        pack_wheel_diag(d, g, 0x00, false, false, 0, f);
        CHECK(f[0]==7, "disabled: FL reason DISABLED_STATE");
        CHECK(f[6]==0x00 && (f[7]&FLAG_LATCHED)==0, "disabled: no fault/latch");
    }

    /* Sequence counter wraps within the 4-bit field. */
    {
        WheelDiag_t d[4] = { WHEEL_DIAG_OK, WHEEL_DIAG_OK, WHEEL_DIAG_OK, WHEEL_DIAG_OK };
        uint8_t g[4] = { 0, 0, 0, 0 };
        pack_wheel_diag(d, g, 0x00, false, false, 0x1F, f);
        CHECK(((f[7]>>4)&0x0F)==0x0F, "sequence masked to 4 bits");
    }

    /* Case 9: latched culprit persists after the live reason self-heals.
     * RL aborted with NO_PULSE, then wheel_diag[RL] healed back to OK while the
     * service-mode fault stays latched -> byte2 must still report NO_PULSE (1),
     * not OK, so the HMI can name the aborting channel. */
    {
        CHECK(latched_reason(WHEEL_DIAG_NO_PULSE, WHEEL_DIAG_NO_PULSE, true)
                  == WHEEL_DIAG_NO_PULSE, "latch: culprit captured at latch");
        CHECK(latched_reason(WHEEL_DIAG_OK, WHEEL_DIAG_NO_PULSE, true)
                  == WHEEL_DIAG_NO_PULSE, "latch: culprit persists after heal");
        CHECK(latched_reason(WHEEL_DIAG_OK, WHEEL_DIAG_NO_PULSE, false)
                  == WHEEL_DIAG_OK, "latch: cleared latch reports live OK");
        CHECK(latched_reason(WHEEL_DIAG_MANUAL_MOVEMENT, WHEEL_DIAG_OK, false)
                  == WHEEL_DIAG_MANUAL_MOVEMENT,
              "latch: no fault reports live reason verbatim");
    }

    /* Case 10: STEER/CENTER byte4 reflects the steering-encoder module state. */
    {
        CHECK(steer_reason(false) == (uint8_t)WHEEL_DIAG_DISABLED_STATE,
              "steer: module off -> DISABLED_STATE");
        CHECK(steer_reason(true) == (uint8_t)WHEEL_DIAG_OK,
              "steer: module on -> OK");
    }

    printf("=== %d run, %d failed ===\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
