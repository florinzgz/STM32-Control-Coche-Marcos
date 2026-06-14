/**
  ****************************************************************************
  * @file    eps_params.c
  * @brief   EPS calibration parameter storage — flash persistence
  *
  * Stores an eps_params_t block in the last flash page of the
  * STM32G474RE (page 127, 4 KB at 0x0807F000).  A 32-bit CRC32
  * checksum and a 32-bit magic word provide validity detection.
  * Double-buffering: two slots (A / B) occupy the same page; the
  * most-recently-written valid slot is loaded at boot.
  *
  * The active copy lives in RAM and is used directly by the EPS
  * control loop.  Runtime changes via EPS_Params_Set() modify the
  * RAM copy immediately; EPS_Params_Save() writes to flash.
  ****************************************************************************
  */

#include "eps_params.h"
#include "stm32g4xx_hal.h"
#ifndef HOST_TEST
#include "safety_system.h"
#endif
#include <string.h>
#include <math.h>

/* ---- Flash layout ----
 * STM32G474RE: 512 KB flash, 128 pages of 4 KB each.
 * We use the last page (page 127) for parameter storage.
 * Page 127 starts at 0x0807F000 (bank 1, page 127 in single-bank
 * mode which is the default for STM32G474RE).
 *
 * Slot A: offset 0x000  (256 bytes)
 * Slot B: offset 0x100  (256 bytes)
 * Remaining space unused.                                          */
#define EPS_FLASH_PAGE       127U
#define EPS_FLASH_BASE       0x0807F000U
#define EPS_SLOT_A_ADDR      EPS_FLASH_BASE
#define EPS_SLOT_B_ADDR      (EPS_FLASH_BASE + 0x100U)

#define EPS_MAGIC            0x45505331U   /* "EPS1" */

/* ---- Flash-wear rate limit (Fase 4 hardening) -----------------------
 * Mirrors PCAL_WRITE_MIN_INTERVAL_MS / STCAL_WRITE_MIN_INTERVAL_MS.
 * EPS_Params_Save() is a public API reserved for future EPS-cal
 * tooling; rate-limiting it here pre-empts CAN-frame-storm wear of
 * page 127 once the API is wired up.  First call after boot is
 * exempt so a future cal flow can persist its first write.          */
#define EPS_WRITE_MIN_INTERVAL_MS  1000U

/* ---- On-flash slot format ---- */
typedef struct {
    uint32_t     magic;       /* Must equal EPS_MAGIC                */
    uint32_t     sequence;    /* Monotonic write counter             */
    eps_params_t params;      /* Calibration data                    */
    uint32_t     checksum;    /* CRC32 of magic + sequence + params  */
} eps_flash_slot_t;

/* ---- Compiled defaults ---- */
static const eps_params_t eps_defaults = {
    .assist_strength  = 0.45f,
    .center_strength  = 0.30f,
    .damping          = 0.10f,
    .friction_comp    = 0.05f,
    .coast_band_pct   = 3.0f,
    .min_drive_pct    = 8.0f,
    .assist_vs_speed  = 18.0f,
    .return_vs_speed  = 35.0f,
    /* Mechanical / output stage — defaults reproduce previous hardcoded values */
    .deadband_deg     = 1.8f,   /* STEERING_DEADBAND_DEG (motor_control.c)  */
    .max_pwm_pct      = 60.0f,  /* Hardcoded ±60 % clamp in Steering_ControlLoop */
    .slew_rate_pct    = 5.883f, /* 250 cts / 4249 cts_max × 100 %           */
    .center_offset_deg = 0.0f,  /* No offset at factory                     */
};

/* ---- RAM state ---- */
static eps_params_t  eps_active;
static uint32_t      eps_sequence = 0;

/* ---- Flash-wear rate-limit + no-op elision bookkeeping (Fase 4) ----
 * eps_persisted is a snapshot of what is committed to flash; it is
 * loaded from flash in EPS_Params_Init() (or seeded from defaults if
 * no slot is valid) and refreshed on every successful Save.  This
 * lets EPS_Params_Save() short-circuit when the active params
 * already match the persisted slot — same pattern as
 * pedal_cal_store.c (PedalCal_Save no-op elision).                  */
static eps_params_t eps_persisted;
static bool         eps_persisted_valid    = false;
static bool         eps_has_written_once   = false;
static uint32_t     eps_last_write_tick    = 0U;

/* ---- CRC32 (software, no HW CRC unit dependency) ---- */
static uint32_t eps_crc32(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1U)
                crc = (crc >> 1) ^ 0xEDB88320U;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/* ---- Validate a flash slot ---- */
static bool eps_slot_valid(const eps_flash_slot_t *slot)
{
    if (slot->magic != EPS_MAGIC) return false;
    /* Checksum covers magic + sequence + params (everything before checksum) */
    uint32_t crc = eps_crc32(slot, offsetof(eps_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

/* ---- Public API ---- */

void EPS_Params_Init(void)
{
    const eps_flash_slot_t *slotA = (const eps_flash_slot_t *)EPS_SLOT_A_ADDR;
    const eps_flash_slot_t *slotB = (const eps_flash_slot_t *)EPS_SLOT_B_ADDR;

    bool a_ok = eps_slot_valid(slotA);
    bool b_ok = eps_slot_valid(slotB);

    if (a_ok && b_ok) {
        /* Both valid — use the one with higher sequence */
        if (slotB->sequence > slotA->sequence) {
            memcpy(&eps_active, &slotB->params, sizeof(eps_params_t));
            eps_sequence = slotB->sequence;
        } else {
            memcpy(&eps_active, &slotA->params, sizeof(eps_params_t));
            eps_sequence = slotA->sequence;
        }
        memcpy(&eps_persisted, &eps_active, sizeof(eps_params_t));
        eps_persisted_valid = true;
    } else if (a_ok) {
        memcpy(&eps_active, &slotA->params, sizeof(eps_params_t));
        eps_sequence = slotA->sequence;
        memcpy(&eps_persisted, &eps_active, sizeof(eps_params_t));
        eps_persisted_valid = true;
    } else if (b_ok) {
        memcpy(&eps_active, &slotB->params, sizeof(eps_params_t));
        eps_sequence = slotB->sequence;
        memcpy(&eps_persisted, &eps_active, sizeof(eps_params_t));
        eps_persisted_valid = true;
    } else {
        /* No valid data — use compiled defaults */
        memcpy(&eps_active, &eps_defaults, sizeof(eps_params_t));
        eps_sequence = 0;
        /* No persisted slot — Save() must not elide; leave invalid. */
        eps_persisted_valid = false;
    }
}

const eps_params_t *EPS_Params_Get(void)
{
    return &eps_active;
}

bool EPS_Params_Set(eps_param_id_t id, float value)
{
    if (id >= EPS_PARAM_COUNT) return false;

    /* Reject NaN and Inf for all parameters — corrupt values would
     * propagate through the EPS control loop into motor torque.     */
    if (isnan(value) || isinf(value)) return false;

    /* Reject zero or negative values for parameters used as divisors
     * in the EPS control loop (motor_control.c Steering_ControlLoop).
     * assist_vs_speed and return_vs_speed appear in denominators:
     *   g(v) = 1 / (1 + v / assist_vs_speed)
     *   h(v) = 0.3 + v / return_vs_speed                            */
    if ((id == EPS_PARAM_ASSIST_VS_SPEED || id == EPS_PARAM_RETURN_VS_SPEED)
        && value <= 0.0f) {
        return false;
    }

    /* Mechanical parameter range guards.
     * deadband_deg: must be > 0 (zero deadband causes hunting)
     * max_pwm_pct:  must be in (0, 100] (0 would disable steering; >100 meaningless)
     * slew_rate_pct: must be > 0 (zero slew would freeze PWM output)
     * center_offset_deg: any finite value is accepted; extremely large offsets
     *   are not explicitly blocked here but will be clamped in the control loop
     *   by the ±MAX_STEER_DEG encoder guard.                         */
    if (id == EPS_PARAM_DEADBAND_DEG && value <= 0.0f)  return false;
    if (id == EPS_PARAM_MAX_PWM_PCT  && (value <= 0.0f || value > 100.0f)) return false;
    if (id == EPS_PARAM_SLEW_RATE_PCT && value <= 0.0f) return false;

    float *fields = (float *)&eps_active;
    fields[id] = value;
    return true;
}

void EPS_Params_ResetDefaults(void)
{
    memcpy(&eps_active, &eps_defaults, sizeof(eps_params_t));
}

const eps_params_t *EPS_Params_GetDefaults(void)
{
    return &eps_defaults;
}

bool EPS_Params_IsFlashValid(void)
{
    return eps_persisted_valid;
}

bool EPS_Params_Save(void)
{
    /* Defense in depth: never erase page 127 while the vehicle is
     * driving.  EPS_Params_Save() currently has no live callers in
     * the firmware (it is a public API reserved for future EPS-cal
     * tooling), so this gate is pure future-proofing: any future
     * caller that forgets to gate to STANDBY will be rejected here
     * instead of corrupting the EPS slot during ACTIVE/DEGRADED
     * driving.  Consistent with PedalCal_Save() (STANDBY only) and
     * SteeringCal_Save() (BOOT/STANDBY).
     *
     * Compiled out in host tests (test_eps_params.c) which exercise
     * the Set/NaN-rejection paths only and have no Safety subsystem
     * linked in.                                                     */
#ifndef HOST_TEST
    if (Safety_GetState() != SYS_STATE_STANDBY)
        return false;
#endif

    /* Flash-wear guard #1 — no-op elision (Fase 4).
     *
     * If the active params already match the slot persisted in flash,
     * the caller's desired state is already on disk; return success
     * without erasing the page.  Consistent with
     * pedal_cal_store.c::PedalCal_Save and
     * sensor_map_store.c::SensorMapStore_Save.                       */
    if (eps_persisted_valid &&
        memcmp(&eps_active, &eps_persisted, sizeof(eps_params_t)) == 0) {
        return true;
    }

    /* Flash-wear guard #2 — minimum write interval (Fase 4).
     *
     * Reject successive writes that arrive closer together than
     * EPS_WRITE_MIN_INTERVAL_MS (1 s).  First call after boot
     * (eps_has_written_once == false) is exempt.  HAL_GetTick()
     * wraps at ~49.7 days; unsigned modular subtraction is safe.
     * Compiled out in host tests which have no HAL_GetTick().      */
#ifndef HOST_TEST
    {
        uint32_t now_tick = HAL_GetTick();
        if (eps_has_written_once &&
            (uint32_t)(now_tick - eps_last_write_tick) < EPS_WRITE_MIN_INTERVAL_MS) {
            return false;
        }
    }
#endif

    /* Read the previous slot from flash before erasing (if valid).
     * After page erase, both slots are lost, so we must rewrite
     * the backup slot to maintain double-buffer safety.             */
    eps_flash_slot_t prev_slot;
    bool has_prev = false;
    uint32_t prev_addr;

    eps_sequence++;

    /* The "other" slot is the one we are NOT writing this time */
    uint32_t target_addr = (eps_sequence & 1U) ? EPS_SLOT_A_ADDR
                                                : EPS_SLOT_B_ADDR;
    prev_addr = (eps_sequence & 1U) ? EPS_SLOT_B_ADDR
                                     : EPS_SLOT_A_ADDR;

    /* Copy the backup slot from flash into RAM before erasing */
    /* Cast via uintptr_t for C standard compliance (uint32_t → pointer) */
    const eps_flash_slot_t *prev_flash = (const eps_flash_slot_t *)(uintptr_t)prev_addr;
    if (eps_slot_valid(prev_flash)) {
        memcpy(&prev_slot, prev_flash, sizeof(eps_flash_slot_t));
        has_prev = true;
    }

    /* Build the new slot in RAM */
    eps_flash_slot_t slot;
    slot.magic    = EPS_MAGIC;
    slot.sequence = eps_sequence;
    memcpy(&slot.params, &eps_active, sizeof(eps_params_t));
    slot.checksum = eps_crc32(&slot, offsetof(eps_flash_slot_t, checksum));

    /* Unlock flash */
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK) { eps_sequence--; return false; }

    /* Erase page 127 */
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = EPS_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        eps_sequence--;
        return false;
    }

    /* Write the new slot (double-word aligned writes).
     * STM32G4 flash requires 64-bit (double-word) writes.          */
    uint32_t slot_size    = sizeof(eps_flash_slot_t);
    uint32_t dword_count  = (slot_size + 7U) / 8U;
    const uint64_t *src   = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   target_addr + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    /* Rewrite the backup slot so the double-buffer survives the
     * page erase.  If a power loss occurs between the erase and
     * this write, the backup is lost but the new slot is valid.    */
    if (has_prev) {
        const uint64_t *prev_src = (const uint64_t *)&prev_slot;
        for (uint32_t i = 0; i < dword_count; i++) {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                       prev_addr + (i * 8U), prev_src[i]);
            if (status != HAL_OK) {
                /* Backup write failed — new slot is still valid */
                break;
            }
        }
    }

    HAL_FLASH_Lock();

    /* Fase 4: refresh no-op / rate-limit bookkeeping on success.
     * Capture the persisted snapshot AFTER the page commit so a
     * future Save() with identical params is correctly elided.    */
    memcpy(&eps_persisted, &eps_active, sizeof(eps_params_t));
    eps_persisted_valid  = true;
    eps_has_written_once = true;
#ifndef HOST_TEST
    eps_last_write_tick  = HAL_GetTick();
#endif
    return true;
}
