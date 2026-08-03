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
  *     100 / 60 / 80 limits).  Boot is never blocked.
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

/* Optional in host fixtures; productive firmware provides the confirmed
 * service-lock predicate.  PENDING/HOLD never authorize flash writes. */
extern bool CAN_PedalCalServiceConfirmed(void) __attribute__((weak));

static bool glim_write_context_authorized(bool standby_context)
{
    const SystemState_t state = Safety_GetState();
    if (standby_context) return state == SYS_STATE_STANDBY;
    return CAN_PedalCalServiceConfirmed != 0 &&
           CAN_PedalCalServiceConfirmed() &&
           (state == SYS_STATE_ACTIVE || state == SYS_STATE_DEGRADED) &&
           Safety_GetError() == SAFETY_ERROR_NONE;
}

/* ---- Flash layout ----
 * STM32G474RE: 512 KB flash, 128 pages of 4 KB each.
 * Page 122 starts at 0x0807A000 (bank 1).
 * Single slot at the beginning of the page.                       */
#define GLIM_FLASH_PAGE        122U
#define GLIM_FLASH_BASE        0x0807A000U

/* ---- On-flash format versioning ----
 * v1 ("GLM1") stored only the power limits; the response bytes did not
 * exist (they sat in the reserved area = 0).  v2 ("GLM2") adds the accel-
 * response triple.  A valid v1 slot is migrated on read (power kept,
 * response = compile-time defaults) and only rewritten as v2 on SAVE.   */
#define GLIM_MAGIC_V1          0x474C4D31U   /* "GLM1" — power only        */
#define GLIM_MAGIC_V2          0x474C4D32U   /* "GLM2" — power + response  */
#define GLIM_VALID_FLAG        0xA5U

/* ---- Flash-wear rate limit ----------------------------------------
 * Minimum interval between consecutive successful flash writes on
 * page 122.  Mirrors the PCAL_WRITE_MIN_INTERVAL_MS guard so a CAN-frame
 * storm or buggy/malicious caller cannot wear the page.              */
#define GLIM_WRITE_MIN_INTERVAL_MS  1000U

/* ---- On-flash slot format ----
 * Exactly 16 bytes, double-word aligned.  v2 repurposes 3 of the 4 bytes
 * that were "reserved" in v1 to carry the accel-response triple, so the
 * slot stays 16 bytes and page 122's layout footprint is unchanged.
 *
 *   v1 ("GLM1"):  magic d2 d1 r flag  [reserved x4]              crc
 *   v2 ("GLM2"):  magic d2 d1 r flag  d2_resp d1_resp r_resp [r1] crc
 *
 * The CRC covers every field before `checksum`, at the same offset for
 * both versions, so a v1 slot read through this struct still validates
 * (its response bytes read back as 0, which is how v1 is detected).      */
typedef struct {
    uint32_t magic;          /* GLIM_MAGIC_V1 or GLIM_MAGIC_V2          */
    uint8_t  d2_pct;         /* D2 power limit (percent)                */
    uint8_t  d1_pct;         /* D1 power limit (percent)                */
    uint8_t  r_pct;          /* R  power limit (percent)                */
    uint8_t  validity_flag;  /* GLIM_VALID_FLAG when slot is committed  */
    uint8_t  d2_resp;        /* D2 accel response (percent, v2 only)    */
    uint8_t  d1_resp;        /* D1 accel response (percent, v2 only)    */
    uint8_t  r_resp;         /* R  accel response (percent, v2 only)    */
    uint8_t  reserved;       /* Padding / future use                    */
    uint32_t checksum;       /* CRC32 of all fields before this field   */
} glim_flash_slot_t;

/* Compile-time guarantee that the on-flash layout is exactly 16 bytes. */
typedef char glim_size_check_[(sizeof(glim_flash_slot_t) == 16) ? 1 : -1];

/* ---- RAM state ---- */
static bool    glim_flash_valid  = false;  /* Flash slot passed CRC + range */
static bool    glim_legacy_fmt   = false;  /* Loaded a v1 (power-only) slot  */
static uint8_t glim_stored_d2    = 0;
static uint8_t glim_stored_d1    = 0;
static uint8_t glim_stored_r     = 0;
static uint8_t glim_stored_d2_resp = 0;
static uint8_t glim_stored_d1_resp = 0;
static uint8_t glim_stored_r_resp  = 0;

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

/* ---- Validate a flash slot (CRC + flag only) ----
 * Accepts either format magic; the caller distinguishes v1 vs v2 by the
 * `magic` field after this passes.                                      */
static bool glim_slot_integrity_ok(const glim_flash_slot_t *slot)
{
    if (slot->magic != GLIM_MAGIC_V1 && slot->magic != GLIM_MAGIC_V2)
        return false;
    if (slot->validity_flag != GLIM_VALID_FLAG) return false;
    uint32_t crc = glim_crc32(slot, offsetof(glim_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

/* ==================================================================
 *  Public API
 * ================================================================== */

bool GearLimitsStore_ValidateResponse(uint8_t d2_pct, uint8_t d1_pct,
                                      uint8_t r_pct)
{
    if (d2_pct < GEAR_RESPONSE_D2_MIN_PCT || d2_pct > GEAR_RESPONSE_D2_MAX_PCT)
        return false;
    if (d1_pct < GEAR_RESPONSE_D1_MIN_PCT || d1_pct > GEAR_RESPONSE_D1_MAX_PCT)
        return false;
    if (r_pct  < GEAR_RESPONSE_R_MIN_PCT  || r_pct  > GEAR_RESPONSE_R_MAX_PCT)
        return false;
    return true;
}

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
    glim_flash_valid     = false;
    glim_legacy_fmt      = false;
    glim_stored_d2       = 0;
    glim_stored_d1       = 0;
    glim_stored_r        = 0;
    glim_stored_d2_resp  = 0;
    glim_stored_d1_resp  = 0;
    glim_stored_r_resp   = 0;

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

    /* Power limits accepted — common to both formats. */
    glim_flash_valid = true;
    glim_stored_d2   = slot->d2_pct;
    glim_stored_d1   = slot->d1_pct;
    glim_stored_r    = slot->r_pct;

    if (slot->magic == GLIM_MAGIC_V2 &&
        GearLimitsStore_ValidateResponse(slot->d2_resp, slot->d1_resp,
                                         slot->r_resp)) {
        /* Full v2 slot: honour the persisted accel-response profile. */
        glim_legacy_fmt     = false;
        glim_stored_d2_resp = slot->d2_resp;
        glim_stored_d1_resp = slot->d1_resp;
        glim_stored_r_resp  = slot->r_resp;
    } else {
        /* Legacy v1 slot (or a v2 slot whose response bytes are out of
         * range): keep the power limits, apply the compile-time response
         * defaults, and flag the slot as legacy.  The slot is NOT rewritten
         * here — it is upgraded to v2 only on the next explicit SAVE.     */
        glim_legacy_fmt     = true;
        glim_stored_d2_resp = GEAR_RESPONSE_D2_DEFAULT_PCT;
        glim_stored_d1_resp = GEAR_RESPONSE_D1_DEFAULT_PCT;
        glim_stored_r_resp  = GEAR_RESPONSE_R_DEFAULT_PCT;
    }
}

bool GearLimitsStore_IsValid(void)
{
    return glim_flash_valid;
}

bool GearLimitsStore_IsLegacyFormat(void)
{
    return glim_legacy_fmt;
}

void GearLimitsStore_GetStored(uint8_t *d2_pct, uint8_t *d1_pct, uint8_t *r_pct)
{
    if (d2_pct) *d2_pct = glim_stored_d2;
    if (d1_pct) *d1_pct = glim_stored_d1;
    if (r_pct)  *r_pct  = glim_stored_r;
}

void GearLimitsStore_GetStoredResponse(uint8_t *d2_pct, uint8_t *d1_pct,
                                       uint8_t *r_pct)
{
    if (d2_pct) *d2_pct = glim_stored_d2_resp;
    if (d1_pct) *d1_pct = glim_stored_d1_resp;
    if (r_pct)  *r_pct  = glim_stored_r_resp;
}

bool GearLimitsStore_Save(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct,
                          uint8_t d2_resp, uint8_t d1_resp, uint8_t r_resp)
{
    /* Defense in depth: persist only in true STANDBY or while the
     * confirmed service lock owns a clean ACTIVE/DEGRADED vehicle. */
    const bool standby_context = (Safety_GetState() == SYS_STATE_STANDBY);
    if (!glim_write_context_authorized(standby_context))
        return false;

    /* Hard validation gate — never persist out-of-range limits. */
    if (!GearLimitsStore_Validate(d2_pct, d1_pct, r_pct))
        return false;
    if (!GearLimitsStore_ValidateResponse(d2_resp, d1_resp, r_resp))
        return false;

    /* Flash-wear guard #1 — no-op elision.  If the requested values
     * already match the persisted slot AND the slot is already in the
     * current (v2) format, return success without erasing the page
     * (makes RESET_DEFAULTS idempotent).  A legacy slot is always
     * rewritten so the upgrade to v2 is persisted.                     */
    if (glim_flash_valid && !glim_legacy_fmt &&
        glim_stored_d2 == d2_pct &&
        glim_stored_d1 == d1_pct &&
        glim_stored_r  == r_pct &&
        glim_stored_d2_resp == d2_resp &&
        glim_stored_d1_resp == d1_resp &&
        glim_stored_r_resp  == r_resp) {
        return true;
    }

    /* Flash-wear guard #2 — minimum write interval. */
    uint32_t now = HAL_GetTick();
    if (glim_has_written_once &&
        (uint32_t)(now - glim_last_write_tick) < GLIM_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    /* Build the slot in RAM (8-byte aligned for doubleword programming).
     * Always written in the current (v2) format: power + response.       */
    _Alignas(8) glim_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic         = GLIM_MAGIC_V2;
    slot.d2_pct        = d2_pct;
    slot.d1_pct        = d1_pct;
    slot.r_pct         = r_pct;
    slot.validity_flag = GLIM_VALID_FLAG;
    slot.d2_resp       = d2_resp;
    slot.d1_resp       = d1_resp;
    slot.r_resp        = r_resp;
    slot.checksum      = glim_crc32(&slot,
                                    offsetof(glim_flash_slot_t, checksum));

    if (!glim_write_context_authorized(standby_context)) return false;

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
    if (!glim_write_context_authorized(standby_context)) {
        HAL_FLASH_Lock();
        return false;
    }

    /* Write the slot (double-word aligned). 16 bytes -> 2 doublewords. */
    uint32_t slot_size   = sizeof(glim_flash_slot_t);
    uint32_t dword_count = (slot_size + 7U) / 8U;
    const uint64_t *src  = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        if (!glim_write_context_authorized(standby_context)) {
            HAL_FLASH_Lock();
            return false;
        }
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   GLIM_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            /* Page was erased but writing failed: the slot is incomplete.
             * Mark the RAM cache invalid so callers see the true flash state
             * and the next Save() attempt re-erases and rewrites from scratch.*/
            glim_flash_valid = false;
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();
    if (!glim_write_context_authorized(standby_context)) return false;

    /* Update RAM state + rate-limit bookkeeping. */
    glim_flash_valid      = true;
    glim_legacy_fmt       = false;   /* now persisted as v2 */
    glim_stored_d2        = d2_pct;
    glim_stored_d1        = d1_pct;
    glim_stored_r         = r_pct;
    glim_stored_d2_resp   = d2_resp;
    glim_stored_d1_resp   = d1_resp;
    glim_stored_r_resp    = r_resp;
    glim_has_written_once = true;
    glim_last_write_tick  = HAL_GetTick();
    return true;
}
