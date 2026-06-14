/**
  ****************************************************************************
  * @file    battery_limits_store.c
  * @brief   Persistent battery-limit storage — flash persistence
  *
  * Stores the runtime-tunable battery-voltage protection thresholds (low-V
  * warning, derate limit, SAFE cutoff, recovery, voltage filter) in flash
  * page 120 (0x08078000, 4 KB) of the STM32G474RE.  A 32-bit CRC32 checksum
  * plus a 32-bit magic word ("BAT1") provide integrity, identical to the
  * proven gear_limits_store.c implementation.
  *
  * Page 120 is reserved for battery limits only and is separate from page
  * 121 (drive tuning), 122 (gear limits), 123 (sensor map), 124 (pedal cal),
  * 125 (error log), 126 (steering cal) and 127 (EPS params).
  *
  * Safety invariants:
  *   - The safety STATE MACHINE is not modified; only the threshold values
  *     it compares against become runtime variables.  With defaults the
  *     comparisons reproduce the historic #define values exactly.
  *   - Flash data alone NEVER authorises ACTIVE or clears startup_inhibit.
  *   - On CRC / magic / range failure: silent fallback to defaults.  Boot
  *     is never blocked.
  ****************************************************************************
  */

#include "battery_limits_store.h"
#include "stm32g4xx_hal.h"
#include "safety_system.h"
#include <string.h>
#include <stddef.h>

/* ---- Flash layout ---- */
#define BATT_FLASH_PAGE   120U
#define BATT_FLASH_BASE   0x08078000U

/* ---- On-flash format ---- */
#define BATT_MAGIC        0x42415431U   /* "BAT1" */
#define BATT_VALID_FLAG   0xA5U

/* ---- Flash-wear rate limit ---- */
#define BATT_WRITE_MIN_INTERVAL_MS  1000U

/* ---- On-flash slot format ----
 * Exactly 24 bytes, double-word aligned (3 doublewords).               */
typedef struct {
    uint32_t magic;          /* BATT_MAGIC                              */
    uint16_t warning_cv;     /* low-V warning (centivolts)              */
    uint16_t limit_cv;       /* derate threshold (centivolts)           */
    uint16_t cutoff_cv;      /* SAFE cutoff (centivolts)                */
    uint16_t recovery_cv;    /* recovery threshold (centivolts)         */
    uint16_t filter_ms;      /* voltage filter time constant (ms)       */
    uint8_t  validity_flag;  /* BATT_VALID_FLAG when committed          */
    uint8_t  reserved0;      /* padding                                 */
    uint32_t reserved1;      /* padding / future use                    */
    uint32_t checksum;       /* CRC32 of all fields before this field   */
} batt_flash_slot_t;

/* Compile-time guarantee that the on-flash layout is exactly 24 bytes. */
typedef char batt_size_check_[(sizeof(batt_flash_slot_t) == 24) ? 1 : -1];

/* ---- RAM state ---- */
static bool            batt_flash_valid = false;
static BatteryLimits_t batt_stored      = {0};

/* ---- Flash-wear rate-limit bookkeeping ---- */
static bool     batt_has_written_once = false;
static uint32_t batt_last_write_tick  = 0U;

/* ---- CRC32 (same polynomial as gear_limits_store) ---- */
static uint32_t batt_crc32(const void *data, uint32_t len)
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

static bool batt_slot_integrity_ok(const batt_flash_slot_t *slot)
{
    if (slot->magic != BATT_MAGIC) return false;
    if (slot->validity_flag != BATT_VALID_FLAG) return false;
    uint32_t crc = batt_crc32(slot, offsetof(batt_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

/* ==================================================================
 *  Public API
 * ================================================================== */

void BatteryLimitsStore_GetDefaults(BatteryLimits_t *out)
{
    if (!out) return;
    out->warning_cv  = BATT_WARNING_DEFAULT_CV;
    out->limit_cv    = BATT_LIMIT_DEFAULT_CV;
    out->cutoff_cv   = BATT_CUTOFF_DEFAULT_CV;
    out->recovery_cv = BATT_RECOVERY_DEFAULT_CV;
    out->filter_ms   = BATT_FILTER_DEFAULT_MS;
}

bool BatteryLimitsStore_Validate(const BatteryLimits_t *b)
{
    if (!b) return false;

    /* Hard ranges. */
    if (b->warning_cv  < BATT_WARNING_MIN_CV  || b->warning_cv  > BATT_WARNING_MAX_CV)  return false;
    if (b->limit_cv    < BATT_LIMIT_MIN_CV    || b->limit_cv    > BATT_LIMIT_MAX_CV)    return false;
    if (b->cutoff_cv   < BATT_CUTOFF_MIN_CV   || b->cutoff_cv   > BATT_CUTOFF_MAX_CV)   return false;
    if (b->recovery_cv < BATT_RECOVERY_MIN_CV || b->recovery_cv > BATT_RECOVERY_MAX_CV) return false;
#if (BATT_FILTER_MIN_MS > 0U)
    if (b->filter_ms   < BATT_FILTER_MIN_MS)                                             return false;
#endif
    if (b->filter_ms   > BATT_FILTER_MAX_MS)                                             return false;

    /* Coherence rules (FASE 4). */
    if (b->warning_cv  <= b->cutoff_cv)   return false;  /* Warning  > Cutoff   */
    if (b->limit_cv    <= b->cutoff_cv)   return false;  /* Limit    > Cutoff   */
    if (b->recovery_cv <= b->cutoff_cv)   return false;  /* Recovery > Cutoff   */
    if (b->warning_cv  >  BATT_OV_WARNING_CV) return false; /* Warning <= OV    */
    if (b->limit_cv    >  BATT_OV_WARNING_CV) return false; /* Limit   <= OV    */
    return true;
}

void BatteryLimitsStore_Init(void)
{
    batt_flash_valid      = false;
    memset(&batt_stored, 0, sizeof(batt_stored));
    batt_has_written_once = false;
    batt_last_write_tick  = 0U;

    const batt_flash_slot_t *slot = (const batt_flash_slot_t *)BATT_FLASH_BASE;

    if (!batt_slot_integrity_ok(slot))
        return;  /* silent fallback to defaults */

    BatteryLimits_t cand;
    cand.warning_cv  = slot->warning_cv;
    cand.limit_cv    = slot->limit_cv;
    cand.cutoff_cv   = slot->cutoff_cv;
    cand.recovery_cv = slot->recovery_cv;
    cand.filter_ms   = slot->filter_ms;

    if (!BatteryLimitsStore_Validate(&cand))
        return;

    batt_flash_valid = true;
    batt_stored      = cand;
}

bool BatteryLimitsStore_IsValid(void)
{
    return batt_flash_valid;
}

void BatteryLimitsStore_GetStored(BatteryLimits_t *out)
{
    if (out) *out = batt_stored;
}

bool BatteryLimitsStore_Save(const BatteryLimits_t *b)
{
    if (!b) return false;

    /* Defense in depth: never persist while actuators may be live. */
    if (Safety_GetState() != SYS_STATE_STANDBY)
        return false;

    if (!BatteryLimitsStore_Validate(b))
        return false;

    /* Flash-wear guard #1 — no-op elision. */
    if (batt_flash_valid &&
        batt_stored.warning_cv  == b->warning_cv  &&
        batt_stored.limit_cv    == b->limit_cv    &&
        batt_stored.cutoff_cv   == b->cutoff_cv   &&
        batt_stored.recovery_cv == b->recovery_cv &&
        batt_stored.filter_ms   == b->filter_ms) {
        return true;
    }

    /* Flash-wear guard #2 — minimum write interval. */
    uint32_t now = HAL_GetTick();
    if (batt_has_written_once &&
        (uint32_t)(now - batt_last_write_tick) < BATT_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    _Alignas(8) batt_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic         = BATT_MAGIC;
    slot.warning_cv    = b->warning_cv;
    slot.limit_cv      = b->limit_cv;
    slot.cutoff_cv     = b->cutoff_cv;
    slot.recovery_cv   = b->recovery_cv;
    slot.filter_ms     = b->filter_ms;
    slot.validity_flag = BATT_VALID_FLAG;
    slot.checksum      = batt_crc32(&slot,
                                    offsetof(batt_flash_slot_t, checksum));

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return false;

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = BATT_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    uint32_t slot_size   = sizeof(batt_flash_slot_t);
    uint32_t dword_count = (slot_size + 7U) / 8U;
    const uint64_t *src  = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   BATT_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    batt_flash_valid      = true;
    batt_stored           = *b;
    batt_has_written_once = true;
    batt_last_write_tick  = HAL_GetTick();
    return true;
}
