/**
  ****************************************************************************
  * @file    test_standby_mode_sync_gate.c
  * @brief   Unit tests for the §3 (Option A) STANDBY logical-mode-sync gate
  *          (Safety_IsStandbyModeSyncAllowed) and the CMD_MODE handler branch
  *          that uses it to close the "boot in the wrong mode" window.
  *
  *          The gate lets a CMD_MODE update ONLY the in-RAM 4x4 / tank-turn
  *          flags while the vehicle is still in STANDBY, so the STM32 mode can
  *          track the physical selector BEFORE traction is ever energised.  It
  *          must NOT grant motion or a gear change, and every safety
  *          precondition (state == STANDBY, pedal released, zero PWM, safe
  *          speed) must hold.
  *
  *          This is a self-contained logic replica (same style as
  *          test_wheel_fault_gate.c) that mirrors the decision table in
  *          safety_system.c / can_handler.c, so it runs on host GCC without the
  *          full HAL.  KEEP IN SYNC with the firmware if the gate changes.
  *
  *          Compile with host GCC (from repository root):
  *            gcc -std=c11 -DHOST_TEST -D_GNU_SOURCE -ICore/Inc -O2 -lm \
  *                Core/Src/test_standby_mode_sync_gate.c \
  *                -o /tmp/test_standby_mode_sync_gate
  ****************************************************************************
  */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ---- Constants mirrored from safety_system.c ---- */
#define MODE_CHANGE_MAX_SPEED_KMH        1.0f
#define STANDBY_MODE_SYNC_MAX_PEDAL_PCT  3.0f

/* System states (subset, mirrors SystemState_t). */
typedef enum {
    S_BOOT = 0, S_STANDBY, S_ACTIVE, S_DEGRADED, S_SAFE, S_ERROR, S_LIMP_HOME
} State_t;

/* ACK codes (mirror can::AckResult / STM32 ACK_*). */
typedef enum { A_OK = 0, A_REJECTED = 1, A_INVALID = 2, A_BLOCKED = 3 } Ack_t;

/* sanitize_float replica: NaN/Inf → fallback (mirrors safety_system.c). */
static float sanitize_float(float v, float fallback) {
    if (isnan(v) || isinf(v)) return fallback;
    return v;
}

/* Replica of Safety_IsCommandAllowed(). */
static bool command_allowed(State_t s) {
    return s == S_ACTIVE || s == S_DEGRADED;
}

/* Replica of Safety_IsStandbyModeSyncAllowed(). */
static bool standby_mode_sync_allowed(State_t s, float pedal_pct,
                                       uint8_t final_pwm_pct, float avg_speed) {
    if (s != S_STANDBY) return false;
    if (sanitize_float(pedal_pct, 100.0f) > STANDBY_MODE_SYNC_MAX_PEDAL_PCT) return false;
    if (final_pwm_pct != 0U) return false;
    if (sanitize_float(avg_speed, MODE_CHANGE_MAX_SPEED_KMH + 1.0f) > MODE_CHANGE_MAX_SPEED_KMH) return false;
    return true;
}

/* Model of the CMD_MODE handler's first gate (the part changed in §3).  It
 * writes the applied mode flags ONLY when a command is applied, and reports
 * whether any gear change was permitted (must stay false on the STANDBY path).
 * Returns the ACK the ESP32 would observe. */
static Ack_t cmd_mode_gate(State_t s, float pedal_pct, uint8_t final_pwm_pct,
                           float avg_speed, uint8_t req_flags,
                           uint8_t *applied_flags_io, bool *gear_allowed_out) {
    *gear_allowed_out = false;
    if (!command_allowed(s)) {
        if (standby_mode_sync_allowed(s, pedal_pct, final_pwm_pct, avg_speed)) {
            *applied_flags_io = (uint8_t)(req_flags & 0x03u); /* logical only */
            return A_OK;                                      /* no gear/motion */
        }
        return A_BLOCKED;
    }
    /* ACTIVE/DEGRADED full path modelled minimally (speed gate). */
    if (avg_speed > MODE_CHANGE_MAX_SPEED_KMH) return A_REJECTED;
    *applied_flags_io = (uint8_t)(req_flags & 0x03u);
    *gear_allowed_out = true;   /* gear may also be applied on the ACTIVE path */
    return A_OK;
}

/* ---- Test harness ---- */
static int g_run = 0, g_fail = 0;
#define CHECK(cond) do { g_run++; if (!(cond)) {                              \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while (0)

/* The gate is CLOSED unless the state is exactly STANDBY. */
static void test_state_must_be_standby(void) {
    CHECK(!standby_mode_sync_allowed(S_BOOT,     0.0f, 0, 0.0f));
    CHECK( standby_mode_sync_allowed(S_STANDBY,  0.0f, 0, 0.0f));
    CHECK(!standby_mode_sync_allowed(S_ACTIVE,   0.0f, 0, 0.0f));
    CHECK(!standby_mode_sync_allowed(S_DEGRADED, 0.0f, 0, 0.0f));
    CHECK(!standby_mode_sync_allowed(S_SAFE,     0.0f, 0, 0.0f));
    CHECK(!standby_mode_sync_allowed(S_ERROR,    0.0f, 0, 0.0f));
    CHECK(!standby_mode_sync_allowed(S_LIMP_HOME,0.0f, 0, 0.0f));
}

/* A pressed pedal, non-zero PWM, or unsafe speed each closes the gate. */
static void test_preconditions_block_gate(void) {
    CHECK(!standby_mode_sync_allowed(S_STANDBY, 3.1f, 0, 0.0f));   /* pedal */
    CHECK( standby_mode_sync_allowed(S_STANDBY, 3.0f, 0, 0.0f));   /* at limit ok */
    CHECK(!standby_mode_sync_allowed(S_STANDBY, 0.0f, 1, 0.0f));   /* PWM */
    CHECK(!standby_mode_sync_allowed(S_STANDBY, 0.0f, 0, 1.1f));   /* speed */
    CHECK( standby_mode_sync_allowed(S_STANDBY, 0.0f, 0, 1.0f));   /* at limit ok */
    /* NaN sensor readings are treated as unsafe. */
    CHECK(!standby_mode_sync_allowed(S_STANDBY, NAN, 0, 0.0f));
    CHECK(!standby_mode_sync_allowed(S_STANDBY, 0.0f, 0, NAN));
}

/* STANDBY + safe: CMD_MODE(4x4) applies the LOGICAL mode and ACKs OK, but
 * NEVER permits a gear change (no motion). */
static void test_standby_applies_logical_mode_only(void) {
    uint8_t applied = 0x00;   /* power-on 4x2 */
    bool gear_ok = true;
    Ack_t a = cmd_mode_gate(S_STANDBY, 0.0f, 0, 0.0f, 0x01, &applied, &gear_ok);
    CHECK(a == A_OK);
    CHECK(applied == 0x01);   /* 4x4 applied logically */
    CHECK(gear_ok == false);  /* gear/motion NOT granted in STANDBY */
}

/* STANDBY + unsafe (pedal pressed): CMD_MODE is BLOCKED and the mode is left
 * unchanged (no partial application). */
static void test_standby_unsafe_blocks(void) {
    uint8_t applied = 0x00;
    bool gear_ok = false;
    Ack_t a = cmd_mode_gate(S_STANDBY, 50.0f, 0, 0.0f, 0x01, &applied, &gear_ok);
    CHECK(a == A_BLOCKED);
    CHECK(applied == 0x00);   /* unchanged */
    CHECK(gear_ok == false);
}

/* Tank-turn bit is carried through the logical apply as well. */
static void test_standby_applies_tank_bit(void) {
    uint8_t applied = 0x00;
    bool gear_ok = false;
    Ack_t a = cmd_mode_gate(S_STANDBY, 0.0f, 0, 0.0f, 0x03, &applied, &gear_ok);
    CHECK(a == A_OK);
    CHECK(applied == 0x03);   /* 4x4 + tank turn */
}

/* SAFE/ERROR never sync the mode (they are not STANDBY). */
static void test_safe_state_blocks(void) {
    uint8_t applied = 0x00;
    bool gear_ok = false;
    Ack_t a = cmd_mode_gate(S_SAFE, 0.0f, 0, 0.0f, 0x01, &applied, &gear_ok);
    CHECK(a == A_BLOCKED);
    CHECK(applied == 0x00);
}

/* The boot scenario end-to-end: selector 4x4, STANDBY, safe → the STM32 mode
 * becomes 4x4 BEFORE it ever transitions to ACTIVE (so the vehicle cannot start
 * moving in the default 4x2). */
static void test_boot_window_closed(void) {
    uint8_t applied = 0x00;   /* STM32 power-on default 4x2 */
    bool gear_ok = false;
    /* ESP32 keeps retransmitting CMD_MODE(4x4) while in STANDBY. */
    Ack_t a = cmd_mode_gate(S_STANDBY, 0.0f, 0, 0.0f, 0x01, &applied, &gear_ok);
    CHECK(a == A_OK);
    CHECK(applied == 0x01);
    /* Only now does the driver press the pedal → ACTIVE, already in 4x4. */
    a = cmd_mode_gate(S_ACTIVE, 80.0f, 0, 0.0f, 0x01, &applied, &gear_ok);
    CHECK(a == A_OK);
    CHECK(applied == 0x01);   /* still 4x4 — never a wrong-mode moment */
}

int main(void) {
    test_state_must_be_standby();
    test_preconditions_block_gate();
    test_standby_applies_logical_mode_only();
    test_standby_unsafe_blocks();
    test_standby_applies_tank_bit();
    test_safe_state_blocks();
    test_boot_window_closed();

    printf("\n--- standby_mode_sync_gate tests: %d run, %d failed ---\n",
           g_run, g_fail);
    return g_fail ? 1 : 0;
}
