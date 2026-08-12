/**
  ****************************************************************************
  * @file    dynbrake_store.c
  * @brief   Persistent dynamic-braking tuning storage — flash persistence
  *
  * Stores the runtime-tunable dynamic-braking parameters (factor, max %,
  * min speed, ramp down/up, enable) in flash page 111 (0x0806F000, 4 KB) of
  * the STM32G474RE.  A 32-bit CRC32 checksum plus a 32-bit magic word
  * ("DYB1") provide integrity, identical to the proven drive_tuning_store.c
  * / gear_limits_store.c implementations.
  *
  * Page 111 is reserved for dynamic-braking tuning only and is separate from
  * every other NVM page (112-127, see STM32G474RETX_FLASH.ld's page-map
  * comment), so each NVM slot can be erased independently.
  *
  * Safety invariants:
  *   - Flash data alone NEVER authorises ACTIVE or clears startup_inhibit.
  *     The parameters only shape an already-validated traction demand.
  *   - On CRC / magic / range failure: silent fallback to defaults
  *     (DynBrakeStore_IsValid() returns false; caller keeps compile-time
  *     values, which are SOFTER than the historic firmware).  Boot is
  *     never blocked.
  *
  * The flash write primitive set (HAL_FLASH_Unlock, HAL_FLASHEx_Erase,
  * HAL_FLASH_Program with FLASH_TYPEPROGRAM_DOUBLEWORD, HAL_FLASH_Lock) is
  * modelled 1:1 on drive_tuning_store.c — no IRQ disables, no RTOS
  * primitives, no watchdog interaction.
  ****************************************************************************
  */

#include "dynbrake_store.h"
#include "stm32g4xx_hal.h"
#include "safety_system.h"
#include <string.h>
#include <stddef.h>

/* ---- Flash layout ----
 * STM32G474RE: 512 KB flash, 128 pages of 4 KB each.
 * Page 111 starts at 0x0806F000 (bank 1).  Single slot at the start.    */
#define DYNB_FLASH_PAGE   111U
#define DYNB_FLASH_BASE   0x0806F000U

/* ---- On-flash format ---- */
#define DYNB_MAGIC        0x44594231U   /* "DYB1" */
#define DYNB_VALID_FLAG   0xA5U

/* ---- Flash-wear rate limit -----------------------------------------
 * Minimum interval between consecutive successful flash writes on page
 * 111.  Mirrors the drive_tuning_store guard so a CAN-frame storm or buggy
 * caller cannot wear the page.                                          */
#define DYNB_WRITE_MIN_INTERVAL_MS  1000U

/* ---- On-flash slot format ----
 * Exactly 16 bytes, double-word aligned (2 doublewords).  Field order is
 * chosen (7 uint8 fields then a uint32) so the natural compiler alignment
 * padding before `checksum` lands the struct on a clean multiple of 8 —
 * identical technique to drive_tuning_store.c's dtun_flash_slot_t — so the
 * doubleword programming loop in DynBrakeStore_Save() never reads past the
 * struct. */
typedef struct {
    uint32_t magic;          /* DYNB_MAGIC                              */
    uint8_t  factor_pct;     /* x100 -> 0.00-1.00                       */
    uint8_t  max_pct;        /* 0-60                                    */
    uint8_t  min_speed_dkmh; /* x10 -> 0.0-10.0 km/h                    */
    uint8_t  ramp_down;      /* %/s                                     */
    uint8_t  ramp_up;        /* %/s                                     */
    uint8_t  enable;         /* 0/1                                     */
    uint8_t  validity_flag;  /* DYNB_VALID_FLAG when committed          */
    uint32_t checksum;       /* CRC32 of all fields before this field   */
} dynb_flash_slot_t;

/* Compile-time guarantee that the on-flash layout is exactly 16 bytes
 * (a multiple of 8, required by the doubleword programming loop below). */
typedef char dynb_size_check_[(sizeof(dynb_flash_slot_t) == 16) ? 1 : -1];

/* ---- RAM state ---- */
static bool             dynb_flash_valid = false;
static DynBrakeTuning_t dynb_stored      = {0};

/* ---- Flash-wear rate-limit bookkeeping ---- */
static bool     dynb_has_written_once = false;
static uint32_t dynb_last_write_tick  = 0U;

/* ---- CRC32 (same polynomial as drive_tuning_store / gear_limits_store) */
static uint32_t dynb_crc32(const void *data, uint32_t len)
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

static bool dynb_slot_integrity_ok(const dynb_flash_slot_t *slot)
{
    if (slot->magic != DYNB_MAGIC) return false;
    if (slot->validity_flag != DYNB_VALID_FLAG) return false;
    uint32_t crc = dynb_crc32(slot, offsetof(dynb_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

/* ==================================================================
 *  Public API
 * ================================================================== */

void DynBrakeStore_GetDefaults(DynBrakeTuning_t *out)
{
    if (!out) return;
    out->factor_pct     = DYNBRAKE_FACTOR_PCT_DEFAULT;
    out->max_pct        = DYNBRAKE_MAX_PCT_DEFAULT;
    out->min_speed_dkmh = DYNBRAKE_MIN_SPEED_DKMH_DEFAULT;
    out->ramp_down      = DYNBRAKE_RAMP_DOWN_DEFAULT;
    out->ramp_up        = DYNBRAKE_RAMP_UP_DEFAULT;
    out->enable         = DYNBRAKE_ENABLE_DEFAULT;
}

bool DynBrakeStore_Validate(const DynBrakeTuning_t *t)
{
    if (!t) return false;
    if (t->factor_pct     < DYNBRAKE_FACTOR_PCT_MIN     || t->factor_pct     > DYNBRAKE_FACTOR_PCT_MAX)     return false;
    if (t->max_pct        < DYNBRAKE_MAX_PCT_MIN        || t->max_pct        > DYNBRAKE_MAX_PCT_MAX)        return false;
    if (t->min_speed_dkmh < DYNBRAKE_MIN_SPEED_DKMH_MIN || t->min_speed_dkmh > DYNBRAKE_MIN_SPEED_DKMH_MAX) return false;
    if (t->ramp_down      < DYNBRAKE_RAMP_DOWN_MIN       || t->ramp_down      > DYNBRAKE_RAMP_DOWN_MAX)      return false;
    if (t->ramp_up        < DYNBRAKE_RAMP_UP_MIN         || t->ramp_up        > DYNBRAKE_RAMP_UP_MAX)        return false;
    if (t->enable         > DYNBRAKE_ENABLE_MAX)                                                             return false;
    return true;
}

void DynBrakeStore_Init(void)
{
    dynb_flash_valid      = false;
    memset(&dynb_stored, 0, sizeof(dynb_stored));
    dynb_has_written_once = false;
    dynb_last_write_tick  = 0U;

    const dynb_flash_slot_t *slot = (const dynb_flash_slot_t *)DYNB_FLASH_BASE;

    if (!dynb_slot_integrity_ok(slot))
        return;  /* silent fallback to defaults */

    DynBrakeTuning_t cand;
    cand.factor_pct     = slot->factor_pct;
    cand.max_pct        = slot->max_pct;
    cand.min_speed_dkmh = slot->min_speed_dkmh;
    cand.ramp_down      = slot->ramp_down;
    cand.ramp_up        = slot->ramp_up;
    cand.enable         = slot->enable;

    /* Range validation — even an integrity-good slot must satisfy the hard
     * ranges before we trust it.  Out-of-range = corrupt = silent fallback. */
    if (!DynBrakeStore_Validate(&cand))
        return;

    dynb_flash_valid = true;
    dynb_stored      = cand;
}

bool DynBrakeStore_IsValid(void)
{
    return dynb_flash_valid;
}

void DynBrakeStore_GetStored(DynBrakeTuning_t *out)
{
    if (out) *out = dynb_stored;
}

bool DynBrakeStore_Save(const DynBrakeTuning_t *t)
{
    if (!t) return false;

    /* Defense in depth: never persist while actuators may be live. */
    if (Safety_GetState() != SYS_STATE_STANDBY)
        return false;

    /* Hard validation gate — never persist out-of-range values. */
    if (!DynBrakeStore_Validate(t))
        return false;

    /* Flash-wear guard #1 — no-op elision (makes RESET idempotent). */
    if (dynb_flash_valid &&
        dynb_stored.factor_pct     == t->factor_pct     &&
        dynb_stored.max_pct        == t->max_pct        &&
        dynb_stored.min_speed_dkmh == t->min_speed_dkmh &&
        dynb_stored.ramp_down      == t->ramp_down      &&
        dynb_stored.ramp_up        == t->ramp_up        &&
        dynb_stored.enable         == t->enable) {
        return true;
    }

    /* Flash-wear guard #2 — minimum write interval. */
    uint32_t now = HAL_GetTick();
    if (dynb_has_written_once &&
        (uint32_t)(now - dynb_last_write_tick) < DYNB_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    /* Build the slot in RAM (8-byte aligned for doubleword programming). */
    _Alignas(8) dynb_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic          = DYNB_MAGIC;
    slot.factor_pct     = t->factor_pct;
    slot.max_pct        = t->max_pct;
    slot.min_speed_dkmh = t->min_speed_dkmh;
    slot.ramp_down      = t->ramp_down;
    slot.ramp_up        = t->ramp_up;
    slot.enable         = t->enable;
    slot.validity_flag  = DYNB_VALID_FLAG;
    slot.checksum       = dynb_crc32(&slot,
                                     offsetof(dynb_flash_slot_t, checksum));

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return false;

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = DYNB_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    uint32_t slot_size   = sizeof(dynb_flash_slot_t);
    uint32_t dword_count = (slot_size + 7U) / 8U;
    const uint64_t *src  = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   DYNB_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    dynb_flash_valid      = true;
    dynb_stored           = *t;
    dynb_has_written_once = true;
    dynb_last_write_tick  = HAL_GetTick();
    return true;
}
