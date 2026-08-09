/**
  ****************************************************************************
  * @file    shunt_store.c
  * @brief   Persistent per-channel INA226 shunt resistance — flash persistence
  ****************************************************************************
  */

#include "shunt_store.h"
#include "project_config.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stddef.h>

/* ---- Flash layout ---- */
#define SHUNT_FLASH_PAGE   114U
#define SHUNT_FLASH_BASE   0x08072000U

/* ---- On-flash format ---- */
#define SHUNT_MAGIC        0x53484E31U   /* "SHN1" */
#define SHUNT_VALID_FLAG   0xA5U

/* ---- Flash-wear rate limit ---- */
#define SHUNT_WRITE_MIN_INTERVAL_MS  1000U

typedef struct {
    uint32_t magic;
    float    mohm[SHUNT_STORE_NUM_CHANNELS];
    uint8_t  validity_flag;
    uint8_t  reserved0;
    uint16_t reserved1;
    uint32_t checksum;
} shunt_flash_slot_t;

typedef char shunt_size_check_[(sizeof(shunt_flash_slot_t) == 36) ? 1 : -1];

static bool       shunt_flash_valid = false;
static ShuntCal_t shunt_stored      = {0};
static ShuntCal_t shunt_staged      = {0};
static bool       shunt_has_written_once = false;
static uint32_t   shunt_last_write_tick  = 0U;

static uint32_t shunt_crc32(const void *data, uint32_t len)
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

static bool shunt_slot_integrity_ok(const shunt_flash_slot_t *slot)
{
    if (slot->magic != SHUNT_MAGIC) return false;
    if (slot->validity_flag != SHUNT_VALID_FLAG) return false;
    uint32_t crc = shunt_crc32(slot, offsetof(shunt_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

void ShuntStore_GetDefaults(ShuntCal_t *out)
{
    if (!out) return;
    for (uint32_t i = 0; i < SHUNT_STORE_NUM_CHANNELS; i++) {
        out->mohm[i] = (i == INA226_CHANNEL_BATTERY)
                     ? (float)INA226_SHUNT_MOHM_BATTERY
                     : (float)INA226_SHUNT_MOHM_MOTOR;
    }
}

bool ShuntStore_Validate(const ShuntCal_t *s)
{
    return ShuntCal_ValidateValues(s);
}

void ShuntStore_Init(void)
{
    shunt_flash_valid      = false;
    memset(&shunt_stored, 0, sizeof(shunt_stored));
    shunt_has_written_once = false;
    shunt_last_write_tick  = 0U;

    const shunt_flash_slot_t *slot =
        (const shunt_flash_slot_t *)SHUNT_FLASH_BASE;

    ShuntStore_GetDefaults(&shunt_stored);

    if (shunt_slot_integrity_ok(slot)) {
        ShuntCal_t cand;
        memcpy(cand.mohm, slot->mohm, sizeof(cand.mohm));

        if (ShuntStore_Validate(&cand)) {
            shunt_flash_valid = true;
            shunt_stored      = cand;
        }
    }

    shunt_staged = shunt_stored;
}

bool ShuntStore_IsValid(void)
{
    return shunt_flash_valid;
}

float ShuntStore_GetEffectiveMohm(uint8_t channel)
{
    if (channel >= SHUNT_STORE_NUM_CHANNELS) return (float)INA226_SHUNT_MOHM_MOTOR;
    return shunt_staged.mohm[channel];
}

bool ShuntStore_Stage(const ShuntCal_t *s)
{
    if (!s) return false;
    if (!ShuntStore_Validate(s)) return false;
    shunt_staged = *s;
    return true;
}

void ShuntStore_GetStaged(ShuntCal_t *out)
{
    if (out) *out = shunt_staged;
}

void ShuntStore_Revert(void)
{
    shunt_staged = shunt_stored;
}

void ShuntStore_ResetToDefaults(void)
{
    ShuntStore_GetDefaults(&shunt_staged);
}

bool ShuntStore_Save(void)
{
    const ShuntCal_t *s = &shunt_staged;

    if (!ShuntStore_Validate(s))
        return false;

    if (shunt_flash_valid &&
        memcmp(shunt_stored.mohm, s->mohm, sizeof(s->mohm)) == 0) {
        return true;
    }

    uint32_t now = HAL_GetTick();
    if (shunt_has_written_once &&
        (uint32_t)(now - shunt_last_write_tick) < SHUNT_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    _Alignas(8) shunt_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic         = SHUNT_MAGIC;
    memcpy(slot.mohm, s->mohm, sizeof(slot.mohm));
    slot.validity_flag = SHUNT_VALID_FLAG;
    slot.checksum      = shunt_crc32(&slot,
                             offsetof(shunt_flash_slot_t, checksum));

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return false;

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = SHUNT_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    uint32_t slot_size   = sizeof(shunt_flash_slot_t);
    uint32_t dword_count = (slot_size + 7U) / 8U;
    const uint64_t *src  = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   SHUNT_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    shunt_flash_valid      = true;
    shunt_stored           = *s;
    shunt_has_written_once = true;
    shunt_last_write_tick  = HAL_GetTick();
    return true;
}
