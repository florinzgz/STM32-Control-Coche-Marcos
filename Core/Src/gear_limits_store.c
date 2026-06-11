/**
  ****************************************************************************
  * @file    gear_limits_store.c
  * @brief   Persistent gear/lever power-limit storage — flash persistence
  *
  * Stores the per-gear traction power-limit percentages (D2 / D1 / R) in
  * flash page 122 (0x0807A000, 4 KB) of the STM32G474RE.  A 32-bit CRC32
  * checksum plus a 32-bit magic word provide integrity, identical to the
  * proven pedal_cal_store.c implementation.
  *
  * Page 122 is reserved for gear power limits only and is separate from
  * page 123 (sensor map), 124 (pedal cal), 125 (error log), 126 (steering
  * cal), and 127 (EPS params), so each NVM slot can be erased independently.
  *
  * Safety invariants:
  *   - Flash data alone NEVER authorises ACTIVE or clears startup_inhibit.
  *     The limits only scale an already-validated traction demand.
  *   - On CRC / magic / range failure: silent fallback to defaults
  *     (GearLimitsStore_IsValid() returns false; caller keeps compile-time
  *     100 / 60 / 60 limits).  Boot is never blocked.
  *
  * The flash write primitive set (HAL_FLASH_Unlock, HAL_FLASHEx_Erase,
  * HAL_FLASH_Program with FLASH_TYPEPROGRAM_DOUBLEWORD, HAL_FLASH_Lock)
  * is modelled 1:1 on pedal_cal_store.c — no IRQ disables, no RTOS
  * primitives, no watchdog interaction.
  ****************************************************************************
  */

#include "gear_limits_store.h"
#include "stm32g4xx_hal.h"
#include "safety_system.h"
#include <string.h>
#include <stddef.h>

/* ---- Flash layout ----
 * STM32G474RE: 512 KB flash, 128 pages of 4 KB each.
 * Page 122 starts at 0x0807A000 (bank 1).
 * Single slot at the beginning of the page.                       */
#define GLIM_FLASH_PAGE        122U
#define GLIM_FLASH_BASE        0x0807A000U

#define GLIM_MAGIC             0x474C4D31U   /* "GLM1" */
#define GLIM_VALID_FLAG        0xA5U

/* ---- Flash-wear rate limit ----------------------------------------
 * Minimum interval between consecutive successful flash writes on
 * page 122.  Mirrors the PCAL_WRITE_MIN_INTERVAL_MS guard so a CAN-frame
 * storm or buggy/malicious caller cannot wear the page.              */
#define GLIM_WRITE_MIN_INTERVAL_MS  1000U

/* ---- On-flash slot format ----
 * Exactly 16 bytes, double-word aligned, identical structure pattern
 * to pcal_flash_slot_t.                                                */
typedef struct {
    uint32_t magic;          /* Must equal GLIM_MAGIC                   */
    uint8_t  d2_pct;         /* D2 power limit (percent)                */
    uint8_t  d1_pct;         /* D1 power limit (percent)                */
    uint8_t  r_pct;          /* R  power limit (percent)                */
    uint8_t  validity_flag;  /* GLIM_VALID_FLAG when slot is committed  */
    uint8_t  reserved[4];    /* Padding / future use                    */
    uint32_t checksum;       /* CRC32 of all fields before this field   */
} glim_flash_slot_t;

/* Compile-time guarantee that the on-flash layout is exactly 16 bytes. */
typedef char glim_size_check_[(sizeof(glim_flash_slot_t) == 16) ? 1 : -1];

/* ---- RAM state ---- */
static bool    glim_flash_valid = false;   /* Flash slot passed CRC + range */
static uint8_t glim_stored_d2   = 0;
static uint8_t glim_stored_d1   = 0;
static uint8_t glim_stored_r    = 0;

/* ---- Flash-wear rate-limit bookkeeping ---- */
static bool     glim_has_written_once = false;
static uint32_t glim_last_write_tick  = 0U;

/* ---- CRC32 (same polynomial as pedal_cal_store / eps_params) ---- */
static uint32_t glim_crc32(const void *data, uint32_t len)
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

/* ---- Validate a flash slot (integrity only) ---- */
static bool glim_slot_integrity_ok(const glim_flash_slot_t *slot)
{
    if (slot->magic != GLIM_MAGIC) return false;
    if (slot->validity_flag != GLIM_VALID_FLAG) return false;
    uint32_t crc = glim_crc32(slot, offsetof(glim_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

/* ==================================================================
 *  Public API
 * ================================================================== */

bool GearLimitsStore_Validate(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct)
{
    if (d2_pct < GEAR_LIMIT_D2_MIN_PCT || d2_pct > GEAR_LIMIT_D2_MAX_PCT)
        return false;
    if (d1_pct < GEAR_LIMIT_D1_MIN_PCT || d1_pct > GEAR_LIMIT_D1_MAX_PCT)
        return false;
    if (r_pct  < GEAR_LIMIT_R_MIN_PCT  || r_pct  > GEAR_LIMIT_R_MAX_PCT)
        return false;
    return true;
}

void GearLimitsStore_Init(void)
{
    glim_flash_valid = false;
    glim_stored_d2   = 0;
    glim_stored_d1   = 0;
    glim_stored_r    = 0;

    glim_has_written_once = false;
    glim_last_write_tick  = 0U;

    const glim_flash_slot_t *slot =
        (const glim_flash_slot_t *)GLIM_FLASH_BASE;

    if (!glim_slot_integrity_ok(slot))
        return;  /* silent fallback to defaults */

    /* Range validation — even an integrity-good slot must satisfy the
     * hard limits before we trust it.  Out-of-range values are treated
     * identically to a corrupt slot: silent fallback.                  */
    if (!GearLimitsStore_Validate(slot->d2_pct, slot->d1_pct, slot->r_pct))
        return;

    glim_flash_valid = true;
    glim_stored_d2   = slot->d2_pct;
    glim_stored_d1   = slot->d1_pct;
    glim_stored_r    = slot->r_pct;
}

bool GearLimitsStore_IsValid(void)
{
    return glim_flash_valid;
}

void GearLimitsStore_GetStored(uint8_t *d2_pct, uint8_t *d1_pct, uint8_t *r_pct)
{
    if (d2_pct) *d2_pct = glim_stored_d2;
    if (d1_pct) *d1_pct = glim_stored_d1;
    if (r_pct)  *r_pct  = glim_stored_r;
}

bool GearLimitsStore_Save(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct)
{
    /* Defense in depth: never persist while actuators may be live.
     * The CAN dispatcher already blocks any caller path outside
     * SYS_STATE_STANDBY, but the same gate is re-asserted at the
     * persistence boundary so no future caller can erase page 122
     * while the vehicle is in ACTIVE / DEGRADED / LIMP_HOME state.     */
    if (Safety_GetState() != SYS_STATE_STANDBY)
        return false;

    /* Hard validation gate — never persist out-of-range limits. */
    if (!GearLimitsStore_Validate(d2_pct, d1_pct, r_pct))
        return false;

    /* Flash-wear guard #1 — no-op elision.  If the requested values
     * already match the persisted slot, return success without erasing
     * the page (makes RESET_DEFAULTS idempotent).                      */
    if (glim_flash_valid &&
        glim_stored_d2 == d2_pct &&
        glim_stored_d1 == d1_pct &&
        glim_stored_r  == r_pct) {
        return true;
    }

    /* Flash-wear guard #2 — minimum write interval. */
    uint32_t now = HAL_GetTick();
    if (glim_has_written_once &&
        (uint32_t)(now - glim_last_write_tick) < GLIM_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    /* Build the slot in RAM (8-byte aligned for doubleword programming). */
    _Alignas(8) glim_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic         = GLIM_MAGIC;
    slot.d2_pct        = d2_pct;
    slot.d1_pct        = d1_pct;
    slot.r_pct         = r_pct;
    slot.validity_flag = GLIM_VALID_FLAG;
    slot.checksum      = glim_crc32(&slot,
                                    offsetof(glim_flash_slot_t, checksum));

    /* Unlock flash */
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return false;

    /* Erase page 122 */
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = GLIM_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    /* Write the slot (double-word aligned). 16 bytes -> 2 doublewords. */
    uint32_t slot_size   = sizeof(glim_flash_slot_t);
    uint32_t dword_count = (slot_size + 7U) / 8U;
    const uint64_t *src  = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   GLIM_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    /* Update RAM state + rate-limit bookkeeping. */
    glim_flash_valid      = true;
    glim_stored_d2        = d2_pct;
    glim_stored_d1        = d1_pct;
    glim_stored_r         = r_pct;
    glim_has_written_once = true;
    glim_last_write_tick  = HAL_GetTick();
    return true;
}
