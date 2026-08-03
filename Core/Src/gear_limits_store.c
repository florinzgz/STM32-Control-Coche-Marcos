/**
  ****************************************************************************
  * @file    gear_limits_store.c
  * @brief   Persistent gear power-limit and accel-response storage
  *
  * Page 122 contains one 16-byte CRC-protected slot.  Reads fail closed to
  * compile-time defaults.  Writes are STANDBY-only and become visible in RAM
  * only after the complete programmed slot is read back and validated.
  ****************************************************************************
  */

#include "gear_limits_store.h"
#include "stm32g4xx_hal.h"
#include "safety_system.h"
#include <stddef.h>
#include <string.h>

#define GLIM_FLASH_PAGE             122U
#define GLIM_FLASH_BASE             0x0807A000U
#define GLIM_MAGIC_V1               0x474C4D31U
#define GLIM_MAGIC_V2               0x474C4D32U
#define GLIM_VALID_FLAG             0xA5U
#define GLIM_WRITE_MIN_INTERVAL_MS  1000U

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
} glim_flash_slot_t;

typedef char glim_size_check_[(sizeof(glim_flash_slot_t) == 16U) ? 1 : -1];

static bool    glim_flash_valid;
static bool    glim_legacy_fmt;
static uint8_t glim_stored_d2;
static uint8_t glim_stored_d1;
static uint8_t glim_stored_r;
static uint8_t glim_stored_d2_resp;
static uint8_t glim_stored_d1_resp;
static uint8_t glim_stored_r_resp;
static bool    glim_has_written_once;
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

static bool glim_slot_integrity_ok(const glim_flash_slot_t *slot)
{
    if (slot == NULL) return false;
    if (slot->magic != GLIM_MAGIC_V1 && slot->magic != GLIM_MAGIC_V2)
        return false;
    if (slot->validity_flag != GLIM_VALID_FLAG) return false;
    return glim_crc32(slot, offsetof(glim_flash_slot_t, checksum)) ==
           slot->checksum;
}

bool GearLimitsStore_ValidateResponse(uint8_t d2_pct, uint8_t d1_pct,
                                      uint8_t r_pct)
{
    return d2_pct >= GEAR_RESPONSE_D2_MIN_PCT &&
           d2_pct <= GEAR_RESPONSE_D2_MAX_PCT &&
           d1_pct >= GEAR_RESPONSE_D1_MIN_PCT &&
           d1_pct <= GEAR_RESPONSE_D1_MAX_PCT &&
           r_pct  >= GEAR_RESPONSE_R_MIN_PCT  &&
           r_pct  <= GEAR_RESPONSE_R_MAX_PCT;
}

bool GearLimitsStore_Validate(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct)
{
    return d2_pct >= GEAR_LIMIT_D2_MIN_PCT &&
           d2_pct <= GEAR_LIMIT_D2_MAX_PCT &&
           d1_pct >= GEAR_LIMIT_D1_MIN_PCT &&
           d1_pct <= GEAR_LIMIT_D1_MAX_PCT &&
           r_pct  >= GEAR_LIMIT_R_MIN_PCT  &&
           r_pct  <= GEAR_LIMIT_R_MAX_PCT;
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
    glim_has_written_once = false;
    glim_last_write_tick = 0U;

    const glim_flash_slot_t *slot =
        (const glim_flash_slot_t *)GLIM_FLASH_BASE;
    if (!glim_slot_integrity_ok(slot)) return;
    if (!GearLimitsStore_Validate(slot->d2_pct, slot->d1_pct, slot->r_pct))
        return;

    glim_stored_d2 = slot->d2_pct;
    glim_stored_d1 = slot->d1_pct;
    glim_stored_r = slot->r_pct;

    if (slot->magic == GLIM_MAGIC_V2 &&
        GearLimitsStore_ValidateResponse(slot->d2_resp, slot->d1_resp,
                                         slot->r_resp)) {
        glim_legacy_fmt = false;
        glim_stored_d2_resp = slot->d2_resp;
        glim_stored_d1_resp = slot->d1_resp;
        glim_stored_r_resp = slot->r_resp;
    } else {
        glim_legacy_fmt = true;
        glim_stored_d2_resp = GEAR_RESPONSE_D2_DEFAULT_PCT;
        glim_stored_d1_resp = GEAR_RESPONSE_D1_DEFAULT_PCT;
        glim_stored_r_resp = GEAR_RESPONSE_R_DEFAULT_PCT;
    }

    glim_flash_valid = true;
}

bool GearLimitsStore_IsValid(void)
{
    return glim_flash_valid;
}

bool GearLimitsStore_IsLegacyFormat(void)
{
    return glim_legacy_fmt;
}

void GearLimitsStore_GetStored(uint8_t *d2_pct, uint8_t *d1_pct,
                               uint8_t *r_pct)
{
    if (d2_pct != NULL) *d2_pct = glim_stored_d2;
    if (d1_pct != NULL) *d1_pct = glim_stored_d1;
    if (r_pct  != NULL) *r_pct  = glim_stored_r;
}

void GearLimitsStore_GetStoredResponse(uint8_t *d2_pct, uint8_t *d1_pct,
                                       uint8_t *r_pct)
{
    if (d2_pct != NULL) *d2_pct = glim_stored_d2_resp;
    if (d1_pct != NULL) *d1_pct = glim_stored_d1_resp;
    if (r_pct  != NULL) *r_pct  = glim_stored_r_resp;
}

bool GearLimitsStore_Save(uint8_t d2_pct, uint8_t d1_pct, uint8_t r_pct,
                          uint8_t d2_resp, uint8_t d1_resp, uint8_t r_resp)
{
    if (!glim_write_context_authorized()) return false;
    if (!GearLimitsStore_Validate(d2_pct, d1_pct, r_pct)) return false;
    if (!GearLimitsStore_ValidateResponse(d2_resp, d1_resp, r_resp))
        return false;

    if (glim_flash_valid && !glim_legacy_fmt &&
        glim_stored_d2 == d2_pct &&
        glim_stored_d1 == d1_pct &&
        glim_stored_r == r_pct &&
        glim_stored_d2_resp == d2_resp &&
        glim_stored_d1_resp == d1_resp &&
        glim_stored_r_resp == r_resp) {
        return true;
    }

    const uint32_t now = HAL_GetTick();
    if (glim_has_written_once &&
        (uint32_t)(now - glim_last_write_tick) < GLIM_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    _Alignas(8) glim_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic = GLIM_MAGIC_V2;
    slot.d2_pct = d2_pct;
    slot.d1_pct = d1_pct;
    slot.r_pct = r_pct;
    slot.validity_flag = GLIM_VALID_FLAG;
    slot.d2_resp = d2_resp;
    slot.d1_resp = d1_resp;
    slot.r_resp = r_resp;
    slot.checksum = glim_crc32(&slot,
                               offsetof(glim_flash_slot_t, checksum));

    /* Last cancelable gate.  Once erase succeeds, finish the destructive
     * transaction and decide success only from program/readback integrity. */
    if (!glim_write_context_authorized()) return false;
    if (HAL_FLASH_Unlock() != HAL_OK) return false;

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = GLIM_FLASH_PAGE;
    erase.NbPages = 1U;

    uint32_t page_err = 0U;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        glim_flash_valid = false;
        (void)HAL_FLASH_Lock();
        return false;
    }

    /* The previous persisted slot no longer exists from this point onward. */
    glim_flash_valid = false;
    glim_legacy_fmt = false;

    const uint64_t *src = (const uint64_t *)&slot;
    const uint32_t dword_count =
        (uint32_t)((sizeof(glim_flash_slot_t) + 7U) / 8U);
    for (uint32_t i = 0U; i < dword_count; ++i) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   GLIM_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            (void)HAL_FLASH_Lock();
            return false;
        }
    }

    if (HAL_FLASH_Lock() != HAL_OK) return false;

    const glim_flash_slot_t *readback =
        (const glim_flash_slot_t *)GLIM_FLASH_BASE;
    if (memcmp(readback, &slot, sizeof(slot)) != 0 ||
        readback->magic != GLIM_MAGIC_V2 ||
        !glim_slot_integrity_ok(readback) ||
        !GearLimitsStore_Validate(readback->d2_pct, readback->d1_pct,
                                  readback->r_pct) ||
        !GearLimitsStore_ValidateResponse(readback->d2_resp,
                                          readback->d1_resp,
                                          readback->r_resp)) {
        return false;
    }

    glim_stored_d2 = readback->d2_pct;
    glim_stored_d1 = readback->d1_pct;
    glim_stored_r = readback->r_pct;
    glim_stored_d2_resp = readback->d2_resp;
    glim_stored_d1_resp = readback->d1_resp;
    glim_stored_r_resp = readback->r_resp;
    glim_legacy_fmt = false;
    glim_flash_valid = true;
    glim_has_written_once = true;
    glim_last_write_tick = HAL_GetTick();
    return true;
}
