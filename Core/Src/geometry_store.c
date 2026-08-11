/**
  ****************************************************************************
  * @file    geometry_store.c
  * @brief   Persistent Ackermann geometry storage — flash persistence
  ****************************************************************************
  */

#include "geometry_store.h"
#include "motor_control.h"
#include "stm32g4xx_hal.h"
#include "safety_system.h"
#include <string.h>
#include <stddef.h>

/* ---- Flash layout ---- */
#define GEOMETRY_FLASH_PAGE   113U
#define GEOMETRY_FLASH_BASE   0x08071000U

/* ---- On-flash format ---- */
#define GEOMETRY_MAGIC        0x47454F31U   /* "GEO1" */
#define GEOMETRY_VALID_FLAG   0xA5U

/* ---- Flash-wear rate limit ---- */
#define GEOMETRY_WRITE_MIN_INTERVAL_MS  1000U

typedef struct {
    uint32_t magic;
    float    wheelbase_m;
    float    track_width_m;
    uint8_t  validity_flag;
    uint8_t  reserved0;
    uint16_t reserved1;
    uint32_t checksum;
} geometry_flash_slot_t;

typedef char geometry_size_check_[(sizeof(geometry_flash_slot_t) == 20) ? 1 : -1];

static bool        geometry_flash_valid = false;
static Geometry_t  geometry_stored      = {0};
static Geometry_t  geometry_staged      = {0};
static bool        geometry_has_written_once = false;
static uint32_t    geometry_last_write_tick  = 0U;

static uint32_t geometry_crc32(const void *data, uint32_t len)
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

static bool geometry_slot_integrity_ok(const geometry_flash_slot_t *slot)
{
    if (slot->magic != GEOMETRY_MAGIC) return false;
    if (slot->validity_flag != GEOMETRY_VALID_FLAG) return false;
    uint32_t crc = geometry_crc32(slot, offsetof(geometry_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

static void geometry_apply(const Geometry_t *g)
{
    Ackermann_SetGeometry(g->wheelbase_m, g->track_width_m, MAX_STEER_DEG);
}

void GeometryStore_GetDefaults(Geometry_t *out)
{
    if (!out) return;
    out->wheelbase_m   = WHEELBASE_M;
    out->track_width_m = TRACK_WIDTH_M;
}

bool GeometryStore_Validate(const Geometry_t *g)
{
    return Geometry_ValidateValues(g);
}

void GeometryStore_Init(void)
{
    geometry_flash_valid      = false;
    memset(&geometry_stored, 0, sizeof(geometry_stored));
    geometry_has_written_once = false;
    geometry_last_write_tick  = 0U;

    const geometry_flash_slot_t *slot =
        (const geometry_flash_slot_t *)GEOMETRY_FLASH_BASE;

    GeometryStore_GetDefaults(&geometry_stored);

    if (geometry_slot_integrity_ok(slot)) {
        Geometry_t cand;
        cand.wheelbase_m   = slot->wheelbase_m;
        cand.track_width_m = slot->track_width_m;

        if (GeometryStore_Validate(&cand)) {
            geometry_flash_valid = true;
            geometry_stored      = cand;
        }
    }

    geometry_staged = geometry_stored;
    geometry_apply(&geometry_staged);
}

bool GeometryStore_IsValid(void)
{
    return geometry_flash_valid;
}

void GeometryStore_GetEffective(Geometry_t *out)
{
    if (out) *out = geometry_staged;
}

bool GeometryStore_Stage(const Geometry_t *g)
{
    if (!g) return false;
    if (!GeometryStore_Validate(g)) return false;
    geometry_staged = *g;
    geometry_apply(&geometry_staged);
    return true;
}

void GeometryStore_GetStaged(Geometry_t *out)
{
    if (out) *out = geometry_staged;
}

void GeometryStore_Revert(void)
{
    geometry_staged = geometry_stored;
    geometry_apply(&geometry_staged);
}

void GeometryStore_ResetToDefaults(void)
{
    GeometryStore_GetDefaults(&geometry_staged);
    geometry_apply(&geometry_staged);
}

bool GeometryStore_Save(void)
{
    const Geometry_t *g = &geometry_staged;

    /* Defense in depth: never persist while actuators may be live (flash
     * erase/program blocks the CPU for tens of ms). Consistent with every
     * other flash-backed store. No production caller exists yet (Block A),
     * so this is pure future-proofing against a caller that forgets to
     * gate to STANDBY. Compiled out in host tests, which stub
     * Safety_GetState() to always report STANDBY. */
    if (Safety_GetState() != SYS_STATE_STANDBY)
        return false;

    if (!GeometryStore_Validate(g))
        return false;

    if (geometry_flash_valid &&
        geometry_stored.wheelbase_m   == g->wheelbase_m &&
        geometry_stored.track_width_m == g->track_width_m) {
        return true;
    }

    uint32_t now = HAL_GetTick();
    if (geometry_has_written_once &&
        (uint32_t)(now - geometry_last_write_tick) < GEOMETRY_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    _Alignas(8) geometry_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic          = GEOMETRY_MAGIC;
    slot.wheelbase_m    = g->wheelbase_m;
    slot.track_width_m  = g->track_width_m;
    slot.validity_flag  = GEOMETRY_VALID_FLAG;
    slot.checksum       = geometry_crc32(&slot,
                               offsetof(geometry_flash_slot_t, checksum));

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return false;

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = GEOMETRY_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    uint32_t slot_size   = sizeof(geometry_flash_slot_t);
    uint32_t dword_count = (slot_size + 7U) / 8U;
    const uint64_t *src  = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   GEOMETRY_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    geometry_flash_valid      = true;
    geometry_stored           = *g;
    geometry_has_written_once = true;
    geometry_last_write_tick  = HAL_GetTick();
    return true;
}
