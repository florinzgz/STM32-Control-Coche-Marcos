#!/usr/bin/env python3
"""Apply the reviewed PR441 completion changes using exact structural anchors."""
from pathlib import Path


def load(path: str) -> str:
    return Path(path).read_text(encoding="utf-8")


def save(path: str, text: str) -> None:
    Path(path).write_text(text, encoding="utf-8")


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one match, found {count}")
    return text.replace(old, new, 1)


# ---------------------------------------------------------------------------
# 1. Calibration rejected-BEGIN safety fix and safe gear-limit persistence.
# ---------------------------------------------------------------------------
path = "Core/Src/can_handler.c"
text = load(path)
text = replace_once(
    text,
    "        if (!ok) pedalcal_service_exit();\n",
    "        /* A rejected BEGIN whose preconditions were never met did not\n"
    "         * touch the powertrain.  Enter HOLD only after service_enter()\n"
    "         * actually acquired the physical lock. */\n"
    "        if (!ok && service_ok) pedalcal_service_exit();\n",
    "rejected BEGIN must not power down traction",
)

old = """static inline bool gearlim_safety_ok(void)
{
    return (Safety_GetState() == SYS_STATE_STANDBY);
}
"""
new = """/* Read-only eligibility for editing a pending gear profile.  Editing RAM is
 * allowed in STANDBY or in a clean, stopped ACTIVE/DEGRADED state, but never
 * while another service lock owns the powertrain.  Flash is gated separately. */
static bool gearlim_edit_allowed(void)
{
    PedalCalConds c;
    pedalcal_build_conds(&c);
    const SystemState_t st = Safety_GetState();
    const bool state_ok = (st == SYS_STATE_STANDBY) ||
                          (st == SYS_STATE_ACTIVE) ||
                          (st == SYS_STATE_DEGRADED);
    return state_ok && !CAN_PedalCalServiceActive() &&
           Safety_GetError() == SAFETY_ERROR_NONE &&
           c.gear_park_or_neutral && !c.wheels_moving &&
           c.pedal_plausible && c.pedal_released &&
           !c.critical_error && !c.safe_state && !c.emergency &&
           !c.can_loss;
}

/* Acquire a physically confirmed movement lock for flash persistence.  True
 * STANDBY already owns the same safe output state; ACTIVE/DEGRADED must enter
 * the confirmed service phase.  The caller must release an acquired lock. */
static bool gearlim_acquire_save_lock(bool *service_owned)
{
    if (service_owned == NULL) return false;
    *service_owned = false;
    if (!gearlim_edit_allowed()) return false;

    if (Safety_GetState() == SYS_STATE_STANDBY) {
        return ((Safety_GetRelayStatusByte() & (1U << 1)) == 0U) &&
               (Traction_GetFinalPwmPct() == 0U);
    }

    if (!pedalcal_service_enter()) return false;
    if (!CAN_PedalCalServiceConfirmed() ||
        !Traction_IsCalibrationLockConfirmed() ||
        ((Safety_GetRelayStatusByte() & (1U << 1)) != 0U) ||
        Traction_GetFinalPwmPct() != 0U) {
        pedalcal_service_exit();
        return false;
    }

    *service_owned = true;
    return true;
}

static void gearlim_release_save_lock(bool service_owned)
{
    if (service_owned) pedalcal_service_exit();
}
"""
text = replace_once(text, old, new, "gear limit safety helper")
text = text.replace("gearlim_safety_ok()", "gearlim_edit_allowed()")

old_save = """    case GEAR_LIMIT_OP_SAVE: {
        if (!gearlim_pending_seeded) {
            /* Nothing staged — nothing to save. */
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        if (!Traction_ValidateGearLimits(gearlim_pending_d2,
                                         gearlim_pending_d1,
                                         gearlim_pending_r) ||
            !Traction_ValidateGearResponse(gearlim_pending_d2_resp,
                                           gearlim_pending_d1_resp,
                                           gearlim_pending_r_resp)) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }
        if (!GearLimitsStore_Save(gearlim_pending_d2,
                                  gearlim_pending_d1,
                                  gearlim_pending_r,
                                  gearlim_pending_d2_resp,
                                  gearlim_pending_d1_resp,
                                  gearlim_pending_r_resp)) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        /* Apply so the next Traction_Update() cycle uses the new limits.
         * Vehicle is in STANDBY (gated above) so there is no live demand. */
        (void)Traction_SetGearLimits(gearlim_pending_d2,
                                     gearlim_pending_d1,
                                     gearlim_pending_r);
        (void)Traction_SetGearResponse(gearlim_pending_d2_resp,
                                       gearlim_pending_d1_resp,
                                       gearlim_pending_r_resp);
        gearlim_pending_seeded = false;   /* clear edit; reseed on next SET */
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
"""
new_save = """    case GEAR_LIMIT_OP_SAVE: {
        if (!gearlim_pending_seeded) {
            /* Nothing staged — nothing to save. */
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        if (!Traction_ValidateGearLimits(gearlim_pending_d2,
                                         gearlim_pending_d1,
                                         gearlim_pending_r) ||
            !Traction_ValidateGearResponse(gearlim_pending_d2_resp,
                                           gearlim_pending_d1_resp,
                                           gearlim_pending_r_resp)) {
            CAN_SendCommandAck(0x10, ACK_INVALID);
            return;
        }

        bool service_owned = false;
        if (!gearlim_acquire_save_lock(&service_owned)) {
            CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
            return;
        }
        const bool saved = GearLimitsStore_Save(gearlim_pending_d2,
                                                 gearlim_pending_d1,
                                                 gearlim_pending_r,
                                                 gearlim_pending_d2_resp,
                                                 gearlim_pending_d1_resp,
                                                 gearlim_pending_r_resp);
        if (!saved) {
            gearlim_release_save_lock(service_owned);
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        /* Apply only while the same physical lock still owns the vehicle. */
        (void)Traction_SetGearLimits(gearlim_pending_d2,
                                     gearlim_pending_d1,
                                     gearlim_pending_r);
        (void)Traction_SetGearResponse(gearlim_pending_d2_resp,
                                       gearlim_pending_d1_resp,
                                       gearlim_pending_r_resp);
        gearlim_pending_seeded = false;
        gearlim_release_save_lock(service_owned);
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
"""
text = replace_once(text, old_save, new_save, "gear limit save path")

old_reset = """    case GEAR_LIMIT_OP_RESET_DEFAULTS: {
        if (!GearLimitsStore_Save(GEAR_LIMIT_D2_DEFAULT_PCT,
                                  GEAR_LIMIT_D1_DEFAULT_PCT,
                                  GEAR_LIMIT_R_DEFAULT_PCT,
                                  GEAR_RESPONSE_D2_DEFAULT_PCT,
                                  GEAR_RESPONSE_D1_DEFAULT_PCT,
                                  GEAR_RESPONSE_R_DEFAULT_PCT)) {
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        (void)Traction_SetGearLimits(GEAR_LIMIT_D2_DEFAULT_PCT,
                                     GEAR_LIMIT_D1_DEFAULT_PCT,
                                     GEAR_LIMIT_R_DEFAULT_PCT);
        (void)Traction_SetGearResponse(GEAR_RESPONSE_D2_DEFAULT_PCT,
                                       GEAR_RESPONSE_D1_DEFAULT_PCT,
                                       GEAR_RESPONSE_R_DEFAULT_PCT);
        gearlim_pending_seeded = false;
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
"""
new_reset = """    case GEAR_LIMIT_OP_RESET_DEFAULTS: {
        bool service_owned = false;
        if (!gearlim_acquire_save_lock(&service_owned)) {
            CAN_SendCommandAck(0x10, ACK_BLOCKED_BY_SAFETY);
            return;
        }
        const bool saved = GearLimitsStore_Save(GEAR_LIMIT_D2_DEFAULT_PCT,
                                                 GEAR_LIMIT_D1_DEFAULT_PCT,
                                                 GEAR_LIMIT_R_DEFAULT_PCT,
                                                 GEAR_RESPONSE_D2_DEFAULT_PCT,
                                                 GEAR_RESPONSE_D1_DEFAULT_PCT,
                                                 GEAR_RESPONSE_R_DEFAULT_PCT);
        if (!saved) {
            gearlim_release_save_lock(service_owned);
            CAN_SendCommandAck(0x10, ACK_REJECTED);
            return;
        }
        (void)Traction_SetGearLimits(GEAR_LIMIT_D2_DEFAULT_PCT,
                                     GEAR_LIMIT_D1_DEFAULT_PCT,
                                     GEAR_LIMIT_R_DEFAULT_PCT);
        (void)Traction_SetGearResponse(GEAR_RESPONSE_D2_DEFAULT_PCT,
                                       GEAR_RESPONSE_D1_DEFAULT_PCT,
                                       GEAR_RESPONSE_R_DEFAULT_PCT);
        gearlim_pending_seeded = false;
        gearlim_release_save_lock(service_owned);
        CAN_SendCommandAck(0x10, ACK_OK);
        return;
    }
"""
text = replace_once(text, old_reset, new_reset, "gear limit reset path")
save(path, text)

# ---------------------------------------------------------------------------
# 2. Reverse limit defaults/bounds and documentation.
# ---------------------------------------------------------------------------
path = "Core/Inc/gear_limits_store.h"
text = load(path)
text = text.replace("Defaults 100 / 60 / 60 %.", "Defaults 100 / 60 / 80 %.")
text = replace_once(text, "#define GEAR_LIMIT_R_MAX_PCT    60U", "#define GEAR_LIMIT_R_MAX_PCT    80U", "R max")
text = replace_once(text, "#define GEAR_LIMIT_R_DEFAULT_PCT   60U", "#define GEAR_LIMIT_R_DEFAULT_PCT   80U", "R default")
text = text.replace("GEAR_POWER_REVERSE_PCT    = 0.60f ->  60 %", "GEAR_POWER_REVERSE_PCT    = 0.80f ->  80 %")
text = text.replace("*   - Safety_GetState() == SYS_STATE_STANDBY\n", "*   - SYS_STATE_STANDBY, or a physically confirmed service lock\n")
save(path, text)

# ---------------------------------------------------------------------------
# 3. Persistence boundary: STANDBY or confirmed service lock only.
# ---------------------------------------------------------------------------
path = "Core/Src/gear_limits_store.c"
text = load(path)
text = text.replace("100 / 60 / 60 limits", "100 / 60 / 80 limits")
anchor = '#include <stddef.h>\n'
insert = '''#include <stddef.h>\n\n/* Optional in host fixtures; productive firmware provides the confirmed\n * service-lock predicate.  PENDING/HOLD never authorize flash writes. */\nextern bool CAN_PedalCalServiceConfirmed(void) __attribute__((weak));\n\nstatic bool glim_write_context_authorized(bool standby_context)\n{\n    const SystemState_t state = Safety_GetState();\n    if (standby_context) return state == SYS_STATE_STANDBY;\n    return CAN_PedalCalServiceConfirmed != 0 &&\n           CAN_PedalCalServiceConfirmed() &&\n           (state == SYS_STATE_ACTIVE || state == SYS_STATE_DEGRADED) &&\n           Safety_GetError() == SAFETY_ERROR_NONE;\n}\n'''
text = replace_once(text, anchor, insert, "gear store service predicate")
old_gate = '''    /* Defense in depth: never persist while actuators may be live.\n     * The CAN dispatcher already blocks any caller path outside\n     * SYS_STATE_STANDBY, but the same gate is re-asserted at the\n     * persistence boundary so no future caller can erase page 122\n     * while the vehicle is in ACTIVE / DEGRADED / LIMP_HOME state.     */\n    if (Safety_GetState() != SYS_STATE_STANDBY)\n        return false;\n'''
new_gate = '''    /* Defense in depth: persist only in true STANDBY or while the\n     * confirmed service lock owns a clean ACTIVE/DEGRADED vehicle. */\n    const bool standby_context = (Safety_GetState() == SYS_STATE_STANDBY);\n    if (!glim_write_context_authorized(standby_context))\n        return false;\n'''
text = replace_once(text, old_gate, new_gate, "gear store entry gate")
slot_anchor = '''    slot.checksum      = glim_crc32(&slot,\n                                    offsetof(glim_flash_slot_t, checksum));\n\n    /* Unlock flash */\n'''
slot_new = '''    slot.checksum      = glim_crc32(&slot,\n                                    offsetof(glim_flash_slot_t, checksum));\n\n    if (!glim_write_context_authorized(standby_context)) return false;\n\n    /* Unlock flash */\n'''
text = replace_once(text, slot_anchor, slot_new, "gear store pre-erase revalidation")
erase_anchor = '''    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {\n        HAL_FLASH_Lock();\n        return false;\n    }\n\n    /* Write the slot (double-word aligned). 16 bytes -> 2 doublewords. */\n'''
erase_new = '''    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {\n        HAL_FLASH_Lock();\n        return false;\n    }\n    if (!glim_write_context_authorized(standby_context)) {\n        HAL_FLASH_Lock();\n        return false;\n    }\n\n    /* Write the slot (double-word aligned). 16 bytes -> 2 doublewords. */\n'''
text = replace_once(text, erase_anchor, erase_new, "gear store post-erase revalidation")
loop_anchor = '''    for (uint32_t i = 0; i < dword_count; i++) {\n        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,\n'''
loop_new = '''    for (uint32_t i = 0; i < dword_count; i++) {\n        if (!glim_write_context_authorized(standby_context)) {\n            HAL_FLASH_Lock();\n            return false;\n        }\n        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,\n'''
text = replace_once(text, loop_anchor, loop_new, "gear store program revalidation")
lock_anchor = '''    HAL_FLASH_Lock();\n\n    /* Update RAM state + rate-limit bookkeeping. */\n'''
lock_new = '''    HAL_FLASH_Lock();\n    if (!glim_write_context_authorized(standby_context)) return false;\n\n    /* Update RAM state + rate-limit bookkeeping. */\n'''
text = replace_once(text, lock_anchor, lock_new, "gear store final revalidation")
save(path, text)

# ---------------------------------------------------------------------------
# 4. Exact rear-axle symmetry in straight 2WD, reduction-only.
# ---------------------------------------------------------------------------
path = "Core/Inc/traction_output_policy.h"
text = load(path)
old_sig = '''static inline bool TractionOutput_Resolve4x2Rear(\n    const uint8_t logical_mode[TRACTION_OUTPUT_WHEEL_COUNT],\n    const int8_t logical_direction[TRACTION_OUTPUT_WHEEL_COUNT],\n    const uint16_t logical_pwm[TRACTION_OUTPUT_WHEEL_COUNT],\n    const float wheel_scale[TRACTION_OUTPUT_WHEEL_COUNT],\n    uint16_t pwm_max,\n    TractionOutputPlan *out)\n'''
new_sig = '''static inline bool TractionOutput_Resolve4x2Rear(\n    const uint8_t logical_mode[TRACTION_OUTPUT_WHEEL_COUNT],\n    const int8_t logical_direction[TRACTION_OUTPUT_WHEEL_COUNT],\n    const uint16_t logical_pwm[TRACTION_OUTPUT_WHEEL_COUNT],\n    float steering_angle_deg,\n    float deadband_deg,\n    const float wheel_scale[TRACTION_OUTPUT_WHEEL_COUNT],\n    uint16_t pwm_max,\n    TractionOutputPlan *out)\n'''
text = replace_once(text, old_sig, new_sig, "4x2 resolver signature")
old_tail = '''    out->pwm[TRACTION_OUTPUT_RR] = TractionOutput_CapPwmByScale(\n        logical_pwm[TRACTION_OUTPUT_FR], wheel_scale[TRACTION_OUTPUT_RR], pwm_max);\n    return true;\n}\n'''
new_tail = '''    out->pwm[TRACTION_OUTPUT_RR] = TractionOutput_CapPwmByScale(\n        logical_pwm[TRACTION_OUTPUT_FR], wheel_scale[TRACTION_OUTPUT_RR], pwm_max);\n\n    /* In straight-line rear drive, neutralise any residual logical Ackermann or\n     * per-wheel history by reducing both driven wheels to the safer minimum.\n     * A real ABS/TCS scale disables equalisation so no limiter is bypassed. */\n    const bool rear_unlimited =\n        isfinite(steering_angle_deg) && isfinite(deadband_deg) &&\n        deadband_deg > 0.0f && fabsf(steering_angle_deg) < deadband_deg &&\n        isfinite(wheel_scale[TRACTION_OUTPUT_RL]) &&\n        isfinite(wheel_scale[TRACTION_OUTPUT_RR]) &&\n        fabsf(wheel_scale[TRACTION_OUTPUT_RL] - 1.0f) <=\n            TRACTION_OUTPUT_UNITY_EPSILON &&\n        fabsf(wheel_scale[TRACTION_OUTPUT_RR] - 1.0f) <=\n            TRACTION_OUTPUT_UNITY_EPSILON;\n    const bool same_direction =\n        out->direction[TRACTION_OUTPUT_RL] != 0 &&\n        out->direction[TRACTION_OUTPUT_RL] ==\n            out->direction[TRACTION_OUTPUT_RR];\n    if (rear_unlimited && same_direction) {\n        const uint16_t equal_pwm =\n            out->pwm[TRACTION_OUTPUT_RL] < out->pwm[TRACTION_OUTPUT_RR]\n            ? out->pwm[TRACTION_OUTPUT_RL]\n            : out->pwm[TRACTION_OUTPUT_RR];\n        out->pwm[TRACTION_OUTPUT_RL] = equal_pwm;\n        out->pwm[TRACTION_OUTPUT_RR] = equal_pwm;\n    }\n    return true;\n}\n'''
text = replace_once(text, old_tail, new_tail, "4x2 straight symmetry")
save(path, text)

path = "Core/Src/motor_control_patched.c"
text = load(path)
old_call = '''        if (!TractionOutput_Resolve4x2Rear(policy_mode, direction, pwm,\n                                           safety_status.wheel_scale,\n                                           PWM_PERIOD, &plan)) {\n'''
new_call = '''        if (!TractionOutput_Resolve4x2Rear(policy_mode, direction, pwm,\n                                           Steering_GetCurrentAngle(),\n                                           (float)ACKERMANN_DEADBAND_DEG,\n                                           safety_status.wheel_scale,\n                                           PWM_PERIOD, &plan)) {\n'''
text = replace_once(text, old_call, new_call, "productive 4x2 call")
save(path, text)

# ---------------------------------------------------------------------------
# 5. Host tests.
# ---------------------------------------------------------------------------
path = "Core/Src/test_gear_limits.c"
text = load(path)
text = text.replace("historic compile-time behaviour (100/60/60)", "requested defaults (100/60/80)")
text = replace_once(text, "ASSERT_TRUE(GEAR_LIMIT_R_DEFAULT_PCT  == 60U);", "ASSERT_TRUE(GEAR_LIMIT_R_DEFAULT_PCT  == 80U);", "gear test R default")
text = text.replace("R max is 60 %, so 100 %", "R max is 80 %, so 100 %")
text = replace_once(text, "ASSERT_FALSE(test_validate(101U, 60U, 60U));", "ASSERT_FALSE(test_validate(101U, 60U, 80U));", "gear test D2 max")
text = text.replace("/* A realistic re-tune (D2 80, D1 40, R 30) must validate. */\n    ASSERT_TRUE(test_validate(80U, 40U, 30U));", "/* Requested reverse power and a lower retune both validate. */\n    ASSERT_TRUE(test_validate(100U, 60U, 80U));\n    ASSERT_TRUE(test_validate(80U, 40U, 30U));")
save(path, text)

path = "Core/Src/test_traction_output_policy.c"
text = load(path)
# Add steering/deadband args to every existing 4x2 call.
text = text.replace("TractionOutput_Resolve4x2Rear(mode, direction, pwm, scales,\n                                         4249U, NULL)", "TractionOutput_Resolve4x2Rear(mode, direction, pwm, 3.0f, 2.0f,\n                                         scales, 4249U, NULL)")
text = text.replace("TractionOutput_Resolve4x2Rear(mode, direction, pwm, scales,\n                                        4249U, &p)", "TractionOutput_Resolve4x2Rear(mode, direction, pwm, 3.0f, 2.0f,\n                                        scales, 4249U, &p)")
text = text.replace("TractionOutput_Resolve4x2Rear(mode, direction, pwm, rear_cut,\n                                        4249U, &p)", "TractionOutput_Resolve4x2Rear(mode, direction, pwm, 0.0f, 2.0f,\n                                        rear_cut, 4249U, &p)")
insert_anchor = '''    CHECK(p.pwm[TRACTION_OUTPUT_RL] == 600U);  /* physical RL scale reapplied */\n    CHECK(p.pwm[TRACTION_OUTPUT_RR] == 1000U);\n\n'''
insert_new = '''    CHECK(p.pwm[TRACTION_OUTPUT_RL] == 600U);  /* physical RL scale reapplied */\n    CHECK(p.pwm[TRACTION_OUTPUT_RR] == 1000U);\n\n    /* Straight, unlimited rear drive removes residual left/right skew without\n     * ever raising the lower wheel. */\n    const float unity[4] = {1.0f, 1.0f, 1.0f, 1.0f};\n    const int8_t same_forward[4] = {1, 1, 0, 0};\n    CHECK(TractionOutput_Resolve4x2Rear(mode, same_forward, pwm, 0.0f, 2.0f,\n                                        unity, 4249U, &p));\n    CHECK(p.pwm[TRACTION_OUTPUT_RL] == 1000U);\n    CHECK(p.pwm[TRACTION_OUTPUT_RR] == 1000U);\n    CHECK(p.direction[TRACTION_OUTPUT_RL] == 1);\n    CHECK(p.direction[TRACTION_OUTPUT_RR] == 1);\n\n    const int8_t same_reverse[4] = {-1, -1, 0, 0};\n    CHECK(TractionOutput_Resolve4x2Rear(mode, same_reverse, pwm, 0.0f, 2.0f,\n                                        unity, 4249U, &p));\n    CHECK(p.pwm[TRACTION_OUTPUT_RL] == 1000U);\n    CHECK(p.pwm[TRACTION_OUTPUT_RR] == 1000U);\n    CHECK(p.direction[TRACTION_OUTPUT_RL] == -1);\n    CHECK(p.direction[TRACTION_OUTPUT_RR] == -1);\n\n    /* At the deadband boundary Ackermann remains authoritative. */\n    CHECK(TractionOutput_Resolve4x2Rear(mode, same_forward, pwm, 2.0f, 2.0f,\n                                        unity, 4249U, &p));\n    CHECK(p.pwm[TRACTION_OUTPUT_RL] == 1200U);\n    CHECK(p.pwm[TRACTION_OUTPUT_RR] == 1000U);\n\n'''
text = replace_once(text, insert_anchor, insert_new, "4x2 symmetry tests")
save(path, text)

print("PR441 completion patch applied")
