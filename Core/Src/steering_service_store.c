/**
  ****************************************************************************
  * @file    steering_service_store.c
  * @brief   Persistent steering diagnostic-homing parameters — flash storage
  ****************************************************************************
  */

#include "steering_service_store.h"
#include "stm32g4xx_hal.h"
#include "safety_system.h"
#include <string.h>
#include <stddef.h>

/* ---- Flash layout ---- */
#define STSVC_FLASH_PAGE   115U
#define STSVC_FLASH_BASE   0x08073000U

/* ---- On-flash format ---- */
#define STSVC_MAGIC        0x53545331U   /* "STS1" */
#define STSVC_VALID_FLAG   0xA5U

/* ---- Flash-wear rate limit ---- */
#define STSVC_WRITE_MIN_INTERVAL_MS  1000U

typedef struct {
    uint32_t magic;
    uint16_t search_pwm_counts;
    uint16_t sweep_time_guard_ms;
    uint16_t stall_timeout_ms;
    uint16_t total_timeout_ms;
    uint16_t max_centering_counts;
    uint16_t stop_current_ma;
    uint8_t  validity_flag;
    uint8_t  reserved0;
    uint16_t reserved1;
    uint32_t checksum;
} stsvc_flash_slot_t;

typedef char stsvc_size_check_[(sizeof(stsvc_flash_slot_t) == 24) ? 1 : -1];

static bool                    stsvc_flash_valid = false;
static SteeringServiceParams_t stsvc_stored      = {0};
static SteeringServiceParams_t stsvc_staged      = {0};
static bool                    stsvc_has_written_once = false;
static uint32_t                stsvc_last_write_tick  = 0U;

static uint32_t stsvc_crc32(const void *data, uint32_t len)
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

static bool stsvc_slot_integrity_ok(const stsvc_flash_slot_t *slot)
{
    if (slot->magic != STSVC_MAGIC) return false;
    if (slot->validity_flag != STSVC_VALID_FLAG) return false;
    uint32_t crc = stsvc_crc32(slot, offsetof(stsvc_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

void SteeringServiceStore_GetDefaults(SteeringServiceParams_t *out)
{
    if (!out) return;
    out->search_pwm_counts    = 425U;
    out->sweep_time_guard_ms  = 2000U;
    out->stall_timeout_ms     = 300U;
    out->total_timeout_ms     = 10000U;
    out->max_centering_counts = 6000U;
    out->stop_current_ma      = 1000U;
}

bool SteeringServiceStore_Validate(const SteeringServiceParams_t *p)
{
    return SteeringService_ValidateValues(p);
}

void SteeringServiceStore_Init(void)
{
    stsvc_flash_valid      = false;
    memset(&stsvc_stored, 0, sizeof(stsvc_stored));
    stsvc_has_written_once = false;
    stsvc_last_write_tick  = 0U;

    const stsvc_flash_slot_t *slot =
        (const stsvc_flash_slot_t *)STSVC_FLASH_BASE;

    SteeringServiceStore_GetDefaults(&stsvc_stored);

    if (stsvc_slot_integrity_ok(slot)) {
        SteeringServiceParams_t cand;
        cand.search_pwm_counts    = slot->search_pwm_counts;
        cand.sweep_time_guard_ms  = slot->sweep_time_guard_ms;
        cand.stall_timeout_ms     = slot->stall_timeout_ms;
        cand.total_timeout_ms     = slot->total_timeout_ms;
        cand.max_centering_counts = slot->max_centering_counts;
        cand.stop_current_ma      = slot->stop_current_ma;

        if (SteeringServiceStore_Validate(&cand)) {
            stsvc_flash_valid = true;
            stsvc_stored      = cand;
        }
    }

    stsvc_staged = stsvc_stored;
}

bool SteeringServiceStore_IsValid(void)
{
    return stsvc_flash_valid;
}

void SteeringServiceStore_GetEffective(SteeringServiceParams_t *out)
{
    if (out) *out = stsvc_staged;
}

bool SteeringServiceStore_Stage(const SteeringServiceParams_t *p)
{
    if (!p) return false;
    if (!SteeringServiceStore_Validate(p)) return false;
    stsvc_staged = *p;
    return true;
}

void SteeringServiceStore_GetStaged(SteeringServiceParams_t *out)
{
    if (out) *out = stsvc_staged;
}

void SteeringServiceStore_Revert(void)
{
    stsvc_staged = stsvc_stored;
}

void SteeringServiceStore_ResetToDefaults(void)
{
    SteeringServiceStore_GetDefaults(&stsvc_staged);
}

bool SteeringServiceStore_Save(void)
{
    const SteeringServiceParams_t *p = &stsvc_staged;

    /* Defense in depth: never persist while actuators may be live (flash
     * erase/program blocks the CPU for tens of ms). Consistent with every
     * other flash-backed store. No production caller exists yet (Block A),
     * so this is pure future-proofing against a caller that forgets to
     * gate to STANDBY. Compiled out in host tests, which stub
     * Safety_GetState() to always report STANDBY. */
    if (Safety_GetState() != SYS_STATE_STANDBY)
        return false;

    if (!SteeringServiceStore_Validate(p))
        return false;

    if (stsvc_flash_valid &&
        memcmp(&stsvc_stored, p, sizeof(*p)) == 0) {
        return true;
    }

    uint32_t now = HAL_GetTick();
    if (stsvc_has_written_once &&
        (uint32_t)(now - stsvc_last_write_tick) < STSVC_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    _Alignas(8) stsvc_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic                = STSVC_MAGIC;
    slot.search_pwm_counts    = p->search_pwm_counts;
    slot.sweep_time_guard_ms  = p->sweep_time_guard_ms;
    slot.stall_timeout_ms     = p->stall_timeout_ms;
    slot.total_timeout_ms     = p->total_timeout_ms;
    slot.max_centering_counts = p->max_centering_counts;
    slot.stop_current_ma      = p->stop_current_ma;
    slot.validity_flag        = STSVC_VALID_FLAG;
    slot.checksum             = stsvc_crc32(&slot,
                                    offsetof(stsvc_flash_slot_t, checksum));

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return false;

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = STSVC_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    uint32_t slot_size   = sizeof(stsvc_flash_slot_t);
    uint32_t dword_count = (slot_size + 7U) / 8U;
    const uint64_t *src  = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   STSVC_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    stsvc_flash_valid      = true;
    stsvc_stored           = *p;
    stsvc_has_written_once = true;
    stsvc_last_write_tick  = HAL_GetTick();
    return true;
}
