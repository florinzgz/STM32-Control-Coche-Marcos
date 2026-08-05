/**
  ****************************************************************************
  * @file    gear_limits_store.c
  * @brief   Redundant power-loss-tolerant gear profile persistence
  *
  * Pages 122 and 118 alternate.  The old committed page is preserved until
  * the new record is completely programmed, committed and read back.
  ****************************************************************************
  */

#include "gear_limits_store.h"
#include "stm32g4xx_hal.h"
#include "safety_system.h"
#include <stddef.h>
#include <string.h>

#define GLIM_PRIMARY_PAGE            122U
#define GLIM_PRIMARY_BASE            0x0807A000U
#define GLIM_BACKUP_PAGE             118U
#define GLIM_BACKUP_BASE             0x08076000U
#define GLIM_MAGIC_V1                0x474C4D31U
#define GLIM_MAGIC_V2                0x474C4D32U
#define GLIM_MAGIC_V3                0x474C4D33U
#define GLIM_VALID_FLAG              0xA5U
#define GLIM_COMMIT_MARKER           0xC04D17EDU
#define GLIM_WRITE_MIN_INTERVAL_MS   1000U

typedef struct {
    uint32_t magic;
    uint8_t  d2_pct;
    uint8_t  d1_pct;
    uint8_t  r_pct;
    uint8_t  validity_flag;
    uint8_t  d2_resp;
    uint8_t  d1_resp;
    uint8_t  r_resp;
    uint8_t  reserved;
    uint32_t checksum;
} glim_legacy_slot_t;

typedef struct {
    uint32_t magic;
    uint32_t generation;
    uint8_t  d2_pct;
    uint8_t  d1_pct;
    uint8_t  r_pct;
    uint8_t  validity_flag;
    uint8_t  d2_resp;
    uint8_t  d1_resp;
    uint8_t  r_resp;
    uint8_t  reserved;
    uint32_t checksum;
    uint32_t commit_marker;
} glim_journal_slot_t;

typedef char glim_legacy_size_check_[(sizeof(glim_legacy_slot_t) == 16U) ? 1 : -1];
typedef char glim_journal_size_check_[(sizeof(glim_journal_slot_t) == 24U) ? 1 : -1];

static bool     glim_flash_valid;
static bool     glim_legacy_fmt;
static uint8_t  glim_stored_d2;
static uint8_t  glim_stored_d1;
static uint8_t  glim_stored_r;
static uint8_t  glim_stored_d2_resp;
static uint8_t  glim_stored_d1_resp;
static uint8_t  glim_stored_r_resp;
static uint32_t glim_generation;
static uint32_t glim_active_base;
static bool     glim_has_written_once;
static uint32_t glim_last_write_tick;

static bool glim_write_context_authorized(void)
{
    return Safety_GetState() == SYS_STATE_STANDBY &&
           Safety_GetError() == SAFETY_ERROR_NONE;
}

static uint32_t glim_crc32(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0U; i < len; ++i) {
        crc ^= p[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320U) : (crc >> 1);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

static bool glim_generation_newer(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) > 0;
}

bool GearLimitsStore_ValidateResponse(uint8_t d2_pct, uint8_t d1_pct,
                                      uint8_t r_pct)
{
    return d2_pct >= GEAR_RESPONSE_D2_MIN_PCT &&
           d2_pct <= GEAR_RESPONSE_D2_MAX_PCT &&
           d1_pct >= GEAR_RESPONSE_D1_MIN_PCT &&
           d1_pct <= GEAR_RESPONSE_D1_MAX_PCT &&
           r_pct >= GEAR_RESPONSE_R_MIN_PCT &&
           r_pct <= GEAR_RESPONSE_R_MAX_PCT;
}

bool GearLimitsStore_Validate(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct)
{
    return d2_pct >= GEAR_LIMIT_D2_MIN_PCT &&
           d2_pct <= GEAR_LIMIT_D2_MAX_PCT &&
           d1_pct >= GEAR_LIMIT_D1_MIN_PCT &&
           d1_pct <= GEAR_LIMIT_D1_MAX_PCT &&
           r_pct >= GEAR_LIMIT_R_MIN_PCT &&
           r_pct <= GEAR_LIMIT_R_MAX_PCT;
}

static bool glim_legacy_integrity_ok(const glim_legacy_slot_t *slot)
{
    if (slot == NULL ||
        (slot->magic != GLIM_MAGIC_V1 && slot->magic != GLIM_MAGIC_V2) ||
        slot->validity_flag != GLIM_VALID_FLAG) {
        return false;
    }
    return glim_crc32(slot, offsetof(glim_legacy_slot_t, checksum)) ==
           slot->checksum;
}

static bool glim_journal_integrity_ok(const glim_journal_slot_t *slot)
{
    return slot != NULL && slot->magic == GLIM_MAGIC_V3 &&
           slot->validity_flag == GLIM_VALID_FLAG &&
           slot->commit_marker == GLIM_COMMIT_MARKER &&
           glim_crc32(slot, offsetof(glim_journal_slot_t, checksum)) ==
               slot->checksum &&
           GearLimitsStore_Validate(slot->d2_pct, slot->d1_pct, slot->r_pct) &&
           GearLimitsStore_ValidateResponse(slot->d2_resp, slot->d1_resp,
                                             slot->r_resp);
}

typedef struct {
    bool valid;
    bool legacy;
    uint8_t d2;
    uint8_t d1;
    uint8_t r;
    uint8_t d2_resp;
    uint8_t d1_resp;
    uint8_t r_resp;
    uint32_t generation;
    uint32_t base;
} glim_candidate_t;

static glim_candidate_t glim_read_candidate(uint32_t base)
{
    glim_candidate_t out = {0};
    const glim_journal_slot_t *journal =
        (const glim_journal_slot_t *)(uintptr_t)base;
    if (glim_journal_integrity_ok(journal)) {
        out.valid = true;
        out.d2 = journal->d2_pct;
        out.d1 = journal->d1_pct;
        out.r = journal->r_pct;
        out.d2_resp = journal->d2_resp;
        out.d1_resp = journal->d1_resp;
        out.r_resp = journal->r_resp;
        out.generation = journal->generation;
        out.base = base;
        return out;
    }

    const glim_legacy_slot_t *legacy =
        (const glim_legacy_slot_t *)(uintptr_t)base;
    if (!glim_legacy_integrity_ok(legacy) ||
        !GearLimitsStore_Validate(legacy->d2_pct, legacy->d1_pct,
                                  legacy->r_pct)) {
        return out;
    }

    out.valid = true;
    out.legacy = true;
    out.d2 = legacy->d2_pct;
    out.d1 = legacy->d1_pct;
    out.r = legacy->r_pct;
    if (legacy->magic == GLIM_MAGIC_V2 &&
        GearLimitsStore_ValidateResponse(legacy->d2_resp, legacy->d1_resp,
                                         legacy->r_resp)) {
        out.d2_resp = legacy->d2_resp;
        out.d1_resp = legacy->d1_resp;
        out.r_resp = legacy->r_resp;
    } else {
        out.d2_resp = GEAR_RESPONSE_D2_DEFAULT_PCT;
        out.d1_resp = GEAR_RESPONSE_D1_DEFAULT_PCT;
        out.r_resp = GEAR_RESPONSE_R_DEFAULT_PCT;
    }
    out.generation = 0U;
    out.base = base;
    return out;
}

void GearLimitsStore_Init(void)
{
    glim_flash_valid = false;
    glim_legacy_fmt = false;
    glim_stored_d2 = 0U;
    glim_stored_d1 = 0U;
    glim_stored_r = 0U;
    glim_stored_d2_resp = 0U;
    glim_stored_d1_resp = 0U;
    glim_stored_r_resp = 0U;
    glim_generation = 0U;
    glim_active_base = 0U;
    glim_has_written_once = false;
    glim_last_write_tick = 0U;

    const glim_candidate_t primary = glim_read_candidate(GLIM_PRIMARY_BASE);
    const glim_candidate_t backup = glim_read_candidate(GLIM_BACKUP_BASE);
    const glim_candidate_t *best = NULL;
    if (primary.valid) best = &primary;
    if (backup.valid &&
        (best == NULL || glim_generation_newer(backup.generation,
                                                best->generation))) {
        best = &backup;
    }
    if (best == NULL) return;

    glim_flash_valid = true;
    glim_legacy_fmt = best->legacy;
    glim_stored_d2 = best->d2;
    glim_stored_d1 = best->d1;
    glim_stored_r = best->r;
    glim_stored_d2_resp = best->d2_resp;
    glim_stored_d1_resp = best->d1_resp;
    glim_stored_r_resp = best->r_resp;
    glim_generation = best->generation;
    glim_active_base = best->base;
}

bool GearLimitsStore_IsValid(void) { return glim_flash_valid; }
bool GearLimitsStore_IsLegacyFormat(void) { return glim_legacy_fmt; }

void GearLimitsStore_GetStored(uint8_t *d2_pct, uint8_t *d1_pct,
                               uint8_t *r_pct)
{
    if (d2_pct != NULL) *d2_pct = glim_stored_d2;
    if (d1_pct != NULL) *d1_pct = glim_stored_d1;
    if (r_pct != NULL) *r_pct = glim_stored_r;
}

void GearLimitsStore_GetStoredResponse(uint8_t *d2_pct, uint8_t *d1_pct,
                                       uint8_t *r_pct)
{
    if (d2_pct != NULL) *d2_pct = glim_stored_d2_resp;
    if (d1_pct != NULL) *d1_pct = glim_stored_d1_resp;
    if (r_pct != NULL) *r_pct = glim_stored_r_resp;
}

static bool glim_program_page(uint32_t page, uint32_t base,
                              const glim_journal_slot_t *slot)
{
    if (HAL_FLASH_Unlock() != HAL_OK) return false;

    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = page;
    erase.NbPages = 1U;
    uint32_t page_error = 0U;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &page_error);
    if (status != HAL_OK || page_error != 0xFFFFFFFFU) {
        (void)HAL_FLASH_Lock();
        return false;
    }

    const uint64_t *src = (const uint64_t *)slot;
    for (uint32_t i = 0U; i < 3U; ++i) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   base + (i * 8U), src[i]);
        if (status != HAL_OK) {
            (void)HAL_FLASH_Lock();
            return false;
        }
    }
    if (HAL_FLASH_Lock() != HAL_OK) return false;

    const glim_journal_slot_t *readback =
        (const glim_journal_slot_t *)(uintptr_t)base;
    return memcmp(readback, slot, sizeof(*slot)) == 0 &&
           glim_journal_integrity_ok(readback);
}

bool GearLimitsStore_Save(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct,
                          uint8_t d2_resp, uint8_t d1_resp, uint8_t r_resp)
{
    if (!glim_write_context_authorized() ||
        !GearLimitsStore_Validate(d2_pct, d1_pct, r_pct) ||
        !GearLimitsStore_ValidateResponse(d2_resp, d1_resp, r_resp)) {
        return false;
    }

    if (glim_flash_valid && !glim_legacy_fmt &&
        glim_stored_d2 == d2_pct && glim_stored_d1 == d1_pct &&
        glim_stored_r == r_pct && glim_stored_d2_resp == d2_resp &&
        glim_stored_d1_resp == d1_resp && glim_stored_r_resp == r_resp) {
        return true;
    }

    const uint32_t now = HAL_GetTick();
    if (glim_has_written_once &&
        (uint32_t)(now - glim_last_write_tick) <
            GLIM_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    _Alignas(8) glim_journal_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic = GLIM_MAGIC_V3;
    slot.generation = glim_flash_valid ? glim_generation + 1U : 1U;
    slot.d2_pct = d2_pct;
    slot.d1_pct = d1_pct;
    slot.r_pct = r_pct;
    slot.validity_flag = GLIM_VALID_FLAG;
    slot.d2_resp = d2_resp;
    slot.d1_resp = d1_resp;
    slot.r_resp = r_resp;
    slot.checksum = glim_crc32(&slot,
                               offsetof(glim_journal_slot_t, checksum));
    slot.commit_marker = GLIM_COMMIT_MARKER;

    if (!glim_write_context_authorized()) return false;

    const bool target_backup = glim_active_base == GLIM_PRIMARY_BASE;
    const uint32_t target_page = target_backup ? GLIM_BACKUP_PAGE
                                                : GLIM_PRIMARY_PAGE;
    const uint32_t target_base = target_backup ? GLIM_BACKUP_BASE
                                                : GLIM_PRIMARY_BASE;
    if (!glim_program_page(target_page, target_base, &slot)) return false;

    const glim_journal_slot_t *readback =
        (const glim_journal_slot_t *)(uintptr_t)target_base;
    glim_flash_valid = true;
    glim_legacy_fmt = false;
    glim_stored_d2 = readback->d2_pct;
    glim_stored_d1 = readback->d1_pct;
    glim_stored_r = readback->r_pct;
    glim_stored_d2_resp = readback->d2_resp;
    glim_stored_d1_resp = readback->d1_resp;
    glim_stored_r_resp = readback->r_resp;
    glim_generation = readback->generation;
    glim_active_base = target_base;
    glim_has_written_once = true;
    glim_last_write_tick = HAL_GetTick();
    return true;
}
