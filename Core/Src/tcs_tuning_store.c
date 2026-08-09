/**
  ****************************************************************************
  * @file    tcs_tuning_store.c
  * @brief   Persistent TCS (anti-slip) tuning storage — flash persistence
  ****************************************************************************
  */

#include "tcs_tuning_store.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stddef.h>

/* ---- Flash layout ---- */
#define TCS_TUNE_FLASH_PAGE   112U
#define TCS_TUNE_FLASH_BASE   0x08070000U

/* ---- On-flash format ---- */
#define TCS_TUNE_MAGIC        0x54435331U   /* "TCS1" */
#define TCS_TUNE_VALID_FLAG   0xA5U

/* ---- Flash-wear rate limit ---- */
#define TCS_TUNE_WRITE_MIN_INTERVAL_MS  1000U

typedef struct {
    uint32_t magic;
    float    min_reference_kmh;
    float    slip_threshold_pct;
    float    initial_reduction;
    float    reduction_rate_per_s;
    float    recovery_rate_per_s;
    float    max_reduction;
    uint8_t  validity_flag;
    uint8_t  reserved0;
    uint16_t reserved1;
    uint32_t checksum;
} tcs_tune_flash_slot_t;

typedef char tcs_tune_size_check_[(sizeof(tcs_tune_flash_slot_t) == 36) ? 1 : -1];

static bool         tcs_tune_flash_valid = false;
static TcsTuning_t  tcs_tune_stored      = {0};
static TcsTuning_t  tcs_tune_staged      = {0};
static bool         tcs_tune_has_written_once = false;
static uint32_t     tcs_tune_last_write_tick  = 0U;

static uint32_t tcs_tune_crc32(const void *data, uint32_t len)
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

static bool tcs_tune_slot_integrity_ok(const tcs_tune_flash_slot_t *slot)
{
    if (slot->magic != TCS_TUNE_MAGIC) return false;
    if (slot->validity_flag != TCS_TUNE_VALID_FLAG) return false;
    uint32_t crc = tcs_tune_crc32(slot, offsetof(tcs_tune_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

void TcsTuningStore_GetDefaults(TcsTuning_t *out)
{
    if (!out) return;
    out->min_reference_kmh    = DRIVE_TCS_MIN_REFERENCE_KMH;
    out->slip_threshold_pct   = DRIVE_TCS_SLIP_THRESHOLD_PCT;
    out->initial_reduction    = DRIVE_TCS_INITIAL_REDUCTION;
    out->reduction_rate_per_s = DRIVE_TCS_REDUCTION_RATE_PER_S;
    out->recovery_rate_per_s  = DRIVE_TCS_RECOVERY_RATE_PER_S;
    out->max_reduction        = DRIVE_TCS_MAX_REDUCTION;
}

bool TcsTuningStore_Validate(const TcsTuning_t *t)
{
    return TcsTuning_ValidateValues(t);
}

void TcsTuningStore_Init(void)
{
    tcs_tune_flash_valid      = false;
    memset(&tcs_tune_stored, 0, sizeof(tcs_tune_stored));
    tcs_tune_has_written_once = false;
    tcs_tune_last_write_tick  = 0U;

    const tcs_tune_flash_slot_t *slot =
        (const tcs_tune_flash_slot_t *)TCS_TUNE_FLASH_BASE;

    TcsTuningStore_GetDefaults(&tcs_tune_stored);

    if (tcs_tune_slot_integrity_ok(slot)) {
        TcsTuning_t cand;
        cand.min_reference_kmh    = slot->min_reference_kmh;
        cand.slip_threshold_pct   = slot->slip_threshold_pct;
        cand.initial_reduction    = slot->initial_reduction;
        cand.reduction_rate_per_s = slot->reduction_rate_per_s;
        cand.recovery_rate_per_s  = slot->recovery_rate_per_s;
        cand.max_reduction        = slot->max_reduction;

        if (TcsTuningStore_Validate(&cand)) {
            tcs_tune_flash_valid = true;
            tcs_tune_stored      = cand;
        }
    }

    /* Effective runtime values start at whatever is stored (or defaults). */
    tcs_tune_staged = tcs_tune_stored;
}

bool TcsTuningStore_IsValid(void)
{
    return tcs_tune_flash_valid;
}

float TcsTuningStore_GetMinReferenceKmh(void)    { return tcs_tune_staged.min_reference_kmh; }
float TcsTuningStore_GetSlipThresholdPct(void)   { return tcs_tune_staged.slip_threshold_pct; }
float TcsTuningStore_GetInitialReduction(void)   { return tcs_tune_staged.initial_reduction; }
float TcsTuningStore_GetReductionRatePerS(void)  { return tcs_tune_staged.reduction_rate_per_s; }
float TcsTuningStore_GetRecoveryRatePerS(void)   { return tcs_tune_staged.recovery_rate_per_s; }
float TcsTuningStore_GetMaxReduction(void)       { return tcs_tune_staged.max_reduction; }

bool TcsTuningStore_Stage(const TcsTuning_t *t)
{
    if (!t) return false;
    if (!TcsTuningStore_Validate(t)) return false;
    tcs_tune_staged = *t;
    return true;
}

void TcsTuningStore_GetStaged(TcsTuning_t *out)
{
    if (out) *out = tcs_tune_staged;
}

void TcsTuningStore_Revert(void)
{
    tcs_tune_staged = tcs_tune_stored;
}

void TcsTuningStore_ResetToDefaults(void)
{
    TcsTuningStore_GetDefaults(&tcs_tune_staged);
}

bool TcsTuningStore_Save(void)
{
    const TcsTuning_t *t = &tcs_tune_staged;

    if (!TcsTuningStore_Validate(t))
        return false;

    if (tcs_tune_flash_valid &&
        tcs_tune_stored.min_reference_kmh    == t->min_reference_kmh &&
        tcs_tune_stored.slip_threshold_pct   == t->slip_threshold_pct &&
        tcs_tune_stored.initial_reduction    == t->initial_reduction &&
        tcs_tune_stored.reduction_rate_per_s == t->reduction_rate_per_s &&
        tcs_tune_stored.recovery_rate_per_s  == t->recovery_rate_per_s &&
        tcs_tune_stored.max_reduction        == t->max_reduction) {
        return true;
    }

    uint32_t now = HAL_GetTick();
    if (tcs_tune_has_written_once &&
        (uint32_t)(now - tcs_tune_last_write_tick) < TCS_TUNE_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    _Alignas(8) tcs_tune_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic                 = TCS_TUNE_MAGIC;
    slot.min_reference_kmh     = t->min_reference_kmh;
    slot.slip_threshold_pct    = t->slip_threshold_pct;
    slot.initial_reduction     = t->initial_reduction;
    slot.reduction_rate_per_s  = t->reduction_rate_per_s;
    slot.recovery_rate_per_s   = t->recovery_rate_per_s;
    slot.max_reduction         = t->max_reduction;
    slot.validity_flag         = TCS_TUNE_VALID_FLAG;
    slot.checksum              = tcs_tune_crc32(&slot,
                                     offsetof(tcs_tune_flash_slot_t, checksum));

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return false;

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = TCS_TUNE_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    uint32_t slot_size   = sizeof(tcs_tune_flash_slot_t);
    uint32_t dword_count = (slot_size + 7U) / 8U;
    const uint64_t *src  = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   TCS_TUNE_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    tcs_tune_flash_valid      = true;
    tcs_tune_stored           = *t;
    tcs_tune_has_written_once = true;
    tcs_tune_last_write_tick  = HAL_GetTick();
    return true;
}
