/**
  ****************************************************************************
  * @file    pedal_cal_store.c
  * @brief   Power-loss-tolerant pedal calibration persistence
  *
  * Two independent flash pages are alternated.  The previously committed page
  * is never erased until the replacement record has been fully programmed,
  * committed, locked and read back.  A reset during erase/program therefore
  * leaves at least one valid calibration record available at the next boot.
  ****************************************************************************
  */

#include "pedal_cal_store.h"
#include "stm32g4xx_hal.h"
#include "safety_system.h"
#include <stddef.h>
#include <string.h>

extern bool CAN_PedalCalServiceConfirmed(void) __attribute__((weak));

static bool pcal_service_lock_confirmed(void)
{
    return CAN_PedalCalServiceConfirmed != 0 &&
           CAN_PedalCalServiceConfirmed();
}

static bool pcal_write_context_authorized(bool standby_context)
{
    const SystemState_t state = Safety_GetState();
    if (standby_context) {
        return state == SYS_STATE_STANDBY &&
               Safety_GetError() == SAFETY_ERROR_NONE;
    }
    return pcal_service_lock_confirmed() &&
           (state == SYS_STATE_ACTIVE || state == SYS_STATE_DEGRADED) &&
           Safety_GetError() == SAFETY_ERROR_NONE;
}

#define PCAL_PRIMARY_PAGE       124U
#define PCAL_PRIMARY_BASE       0x0807C000U
#define PCAL_BACKUP_PAGE        119U
#define PCAL_BACKUP_BASE        0x08077000U
#define PCAL_LEGACY_MAGIC       0x50434C31U  /* PCL1 */
#define PCAL_JOURNAL_MAGIC      0x50434C32U  /* PCL2 */
#define PCAL_VALID_FLAG         0xA5U
#define PCAL_COMMIT_MARKER      0xC04D17EDU
#define PCAL_WRITE_MIN_INTERVAL_MS 1000U

typedef struct {
    uint32_t magic;
    uint16_t adc_min;
    uint16_t adc_max;
    uint8_t  validity_flag;
    uint8_t  reserved[3];
    uint32_t checksum;
} pcal_legacy_slot_t;

typedef struct {
    uint32_t magic;
    uint32_t generation;
    uint16_t adc_min;
    uint16_t adc_max;
    uint32_t reserved;
    uint32_t checksum;
    uint32_t commit_marker;
} pcal_journal_slot_t;

typedef char pcal_legacy_size_check_[(sizeof(pcal_legacy_slot_t) == 16U) ? 1 : -1];
typedef char pcal_journal_size_check_[(sizeof(pcal_journal_slot_t) == 24U) ? 1 : -1];

static bool     pcal_flash_valid;
static uint16_t pcal_stored_min;
static uint16_t pcal_stored_max;
static uint32_t pcal_generation;
static uint32_t pcal_active_base;
static bool     pcal_has_written_once;
static uint32_t pcal_last_write_tick;

static uint32_t pcal_crc32(const void *data, uint32_t len)
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

static bool pcal_generation_newer(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) > 0;
}

static bool pcal_legacy_ok(const pcal_legacy_slot_t *slot)
{
    return slot != NULL && slot->magic == PCAL_LEGACY_MAGIC &&
           slot->validity_flag == PCAL_VALID_FLAG &&
           pcal_crc32(slot, offsetof(pcal_legacy_slot_t, checksum)) ==
               slot->checksum &&
           PedalCal_Validate(slot->adc_min, slot->adc_max);
}

static bool pcal_journal_ok(const pcal_journal_slot_t *slot)
{
    return slot != NULL && slot->magic == PCAL_JOURNAL_MAGIC &&
           slot->commit_marker == PCAL_COMMIT_MARKER &&
           pcal_crc32(slot, offsetof(pcal_journal_slot_t, checksum)) ==
               slot->checksum &&
           PedalCal_Validate(slot->adc_min, slot->adc_max);
}

typedef struct {
    bool valid;
    uint16_t adc_min;
    uint16_t adc_max;
    uint32_t generation;
    uint32_t base;
} pcal_candidate_t;

static pcal_candidate_t pcal_read_candidate(uint32_t base)
{
    pcal_candidate_t out = {0};
    const pcal_journal_slot_t *journal =
        (const pcal_journal_slot_t *)(uintptr_t)base;
    if (pcal_journal_ok(journal)) {
        out.valid = true;
        out.adc_min = journal->adc_min;
        out.adc_max = journal->adc_max;
        out.generation = journal->generation;
        out.base = base;
        return out;
    }

    const pcal_legacy_slot_t *legacy =
        (const pcal_legacy_slot_t *)(uintptr_t)base;
    if (pcal_legacy_ok(legacy)) {
        out.valid = true;
        out.adc_min = legacy->adc_min;
        out.adc_max = legacy->adc_max;
        out.generation = 0U;
        out.base = base;
    }
    return out;
}

bool PedalCal_Validate(uint16_t adc_min, uint16_t adc_max)
{
#if PEDAL_CAL_MIN_LIMIT > 0U
    if (adc_min < PEDAL_CAL_MIN_LIMIT) return false;
#endif
    if (adc_max > PEDAL_CAL_MAX_LIMIT) return false;
    if (adc_max <= adc_min) return false;
    return (uint32_t)(adc_max - adc_min) >= PEDAL_CAL_RANGE_MIN;
}

void PedalCal_Init(void)
{
    pcal_flash_valid = false;
    pcal_stored_min = 0U;
    pcal_stored_max = 0U;
    pcal_generation = 0U;
    pcal_active_base = 0U;
    pcal_has_written_once = false;
    pcal_last_write_tick = 0U;

    const pcal_candidate_t primary = pcal_read_candidate(PCAL_PRIMARY_BASE);
    const pcal_candidate_t backup = pcal_read_candidate(PCAL_BACKUP_BASE);
    const pcal_candidate_t *best = NULL;
    if (primary.valid) best = &primary;
    if (backup.valid &&
        (best == NULL || pcal_generation_newer(backup.generation,
                                                best->generation))) {
        best = &backup;
    }
    if (best == NULL) return;

    pcal_flash_valid = true;
    pcal_stored_min = best->adc_min;
    pcal_stored_max = best->adc_max;
    pcal_generation = best->generation;
    pcal_active_base = best->base;
}

bool PedalCal_IsValid(void)
{
    return pcal_flash_valid;
}

void PedalCal_GetStored(uint16_t *adc_min, uint16_t *adc_max)
{
    if (adc_min != NULL) *adc_min = pcal_stored_min;
    if (adc_max != NULL) *adc_max = pcal_stored_max;
}

static bool pcal_program_page(uint32_t page, uint32_t base,
                              const pcal_journal_slot_t *slot)
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

    const pcal_journal_slot_t *readback =
        (const pcal_journal_slot_t *)(uintptr_t)base;
    return memcmp(readback, slot, sizeof(*slot)) == 0 &&
           pcal_journal_ok(readback);
}

bool PedalCal_Save(uint16_t adc_min, uint16_t adc_max)
{
    const bool standby_context = Safety_GetState() == SYS_STATE_STANDBY;
    if (!pcal_write_context_authorized(standby_context) ||
        !PedalCal_Validate(adc_min, adc_max)) {
        return false;
    }

    if (pcal_flash_valid && pcal_stored_min == adc_min &&
        pcal_stored_max == adc_max) {
        return true;
    }

    const uint32_t now = HAL_GetTick();
    if (pcal_has_written_once &&
        (uint32_t)(now - pcal_last_write_tick) <
            PCAL_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    _Alignas(8) pcal_journal_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic = PCAL_JOURNAL_MAGIC;
    slot.generation = pcal_flash_valid ? pcal_generation + 1U : 1U;
    slot.adc_min = adc_min;
    slot.adc_max = adc_max;
    slot.checksum = pcal_crc32(&slot,
                               offsetof(pcal_journal_slot_t, checksum));
    slot.commit_marker = PCAL_COMMIT_MARKER;

    /* Last cancelable gate.  Once the inactive page is erased the old active
     * page remains valid, and this transaction is completed without further
     * authorization exits. */
    if (!pcal_write_context_authorized(standby_context)) return false;

    const bool target_backup = pcal_active_base == PCAL_PRIMARY_BASE;
    const uint32_t target_page = target_backup ? PCAL_BACKUP_PAGE
                                                : PCAL_PRIMARY_PAGE;
    const uint32_t target_base = target_backup ? PCAL_BACKUP_BASE
                                                : PCAL_PRIMARY_BASE;
    if (!pcal_program_page(target_page, target_base, &slot)) return false;

    const pcal_journal_slot_t *readback =
        (const pcal_journal_slot_t *)(uintptr_t)target_base;
    pcal_flash_valid = true;
    pcal_stored_min = readback->adc_min;
    pcal_stored_max = readback->adc_max;
    pcal_generation = readback->generation;
    pcal_active_base = target_base;
    pcal_has_written_once = true;
    pcal_last_write_tick = HAL_GetTick();
    return true;
}
