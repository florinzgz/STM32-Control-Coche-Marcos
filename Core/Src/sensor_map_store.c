/**
  ****************************************************************************
  * @file    sensor_map_store.c
  * @brief   Persistent DS18B20 temperature sensor mapping storage
  *
  * Stores a 5-byte physIdx→role array in the third-to-last flash page
  * of the STM32G474RE (page 125, 4 KB at 0x0807D000).
  * A 32-bit CRC32 checksum and a 32-bit magic word provide integrity.
  *
  * Flash page allocation:
  *   Page 127 (0x0807F000) — EPS parameters  (eps_params.c)
  *   Page 126 (0x0807E000) — Steering cal     (steering_cal_store.c)
  *   Page 125 (0x0807D000) — Sensor map       (this file)
  *
  * Writing one page never erases either of the other two.
  ****************************************************************************
  */

#include "sensor_map_store.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stddef.h>

/* ---- Flash layout -------------------------------------------------------- */
#define SMAP_FLASH_PAGE     125U
#define SMAP_FLASH_BASE     0x0807D000U

#define SMAP_MAGIC          0x534D4150U   /* "SMAP" */
#define SMAP_VALID_FLAG     0xA5U

/* ---- Write rate limit ---------------------------------------------------
 * Minimum interval between consecutive successful flash writes.  Protects
 * the flash page against wear from accidental rapid-fire saves (for
 * example a CAN frame storm / UI bug) while still allowing legitimate
 * user-initiated changes to go through after a short cool-down.
 * HAL_GetTick() is the 1 ms system tick (see stm32g4xx_it.c).
 * ------------------------------------------------------------------------ */
#define SMAP_WRITE_MIN_INTERVAL_MS  1000U

/* ---- On-flash slot format (16 bytes = 2 × 64-bit doublewords) ----------- */
typedef struct {
    uint32_t magic;                       /* Must equal SMAP_MAGIC          */
    uint8_t  tempMap[SMAP_NUM_SENSORS];   /* physIdx → role                 */
    uint8_t  valid_flag;                  /* SMAP_VALID_FLAG when valid      */
    uint8_t  reserved[2];                 /* Padding                         */
    uint32_t checksum;                    /* CRC32 of all fields before this */
} smap_flash_slot_t;

/* Compile-time check: slot must be an exact multiple of 8 bytes */
_Static_assert(sizeof(smap_flash_slot_t) % 8 == 0,
               "smap_flash_slot_t size must be a multiple of 8 bytes");

/* ---- RAM state ----------------------------------------------------------- */
static bool     smap_flash_valid      = false;
static uint8_t  smap_active[SMAP_NUM_SENSORS] = {0, 1, 2, 3, 4};  /* identity */
static uint32_t smap_last_write_tick  = 0;    /* HAL_GetTick of last successful flash write */
static bool     smap_has_written_once = false;

/* ---- CRC32 (same polynomial as steering_cal_store.c) --------------------- */
static uint32_t smap_crc32(const void *data, uint32_t len)
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

/* ---- Validate a flash slot ----------------------------------------------- */
static bool smap_slot_valid(const smap_flash_slot_t *slot)
{
    if (slot->magic != SMAP_MAGIC)           return false;
    if (slot->valid_flag != SMAP_VALID_FLAG) return false;
    uint32_t crc = smap_crc32(slot, offsetof(smap_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

/* ========================================================================== */
/*  Public API                                                                 */
/* ========================================================================== */

void SensorMapStore_Init(void)
{
    smap_flash_valid = false;

    const smap_flash_slot_t *slot =
        (const smap_flash_slot_t *)SMAP_FLASH_BASE;

    if (smap_slot_valid(slot)) {
        smap_flash_valid = true;
        memcpy(smap_active, slot->tempMap, SMAP_NUM_SENSORS);
    }
    /* else: smap_active keeps the identity mapping set at declaration */
}

bool SensorMapStore_Save(const uint8_t map[SMAP_NUM_SENSORS])
{
    /* ----------------------------------------------------------------------
     * Flash-wear guard #1 — no-op elision.
     *
     * If the new map is byte-for-byte identical to the currently-active
     * mapping AND a valid slot has already been written to flash (so we
     * know the flash contents match `smap_active`), skip the erase/write
     * entirely.  The result is still success because the caller's
     * requested state is already in flash.
     *
     * Without this guard, the ESP32 auto-resync-on-STM32-reboot would
     * re-erase and re-write the page on every boot cycle even when the
     * user never changed anything.
     * --------------------------------------------------------------------*/
    if (smap_flash_valid &&
        memcmp(smap_active, map, SMAP_NUM_SENSORS) == 0) {
        return true;
    }

    /* ----------------------------------------------------------------------
     * Flash-wear guard #2 — minimum write interval.
     *
     * Reject successive writes that arrive closer together than
     * SMAP_WRITE_MIN_INTERVAL_MS (1 s).  The engineering menu is strictly
     * user-driven, so a legitimate save can never arrive faster than a
     * human tap.  Anything faster must be a bug, injected traffic, or a
     * CAN storm — none of which should be allowed to wear out the page.
     *
     * Guard is disabled on the very first call (smap_has_written_once
     * starts false) so that the first save after boot always runs.
     * HAL_GetTick() is monotonic and overflows only after ~49.7 days.
     * Using subtraction with uint32_t wrap-around makes the comparison
     * correct even across the overflow boundary.
     * --------------------------------------------------------------------*/
    uint32_t now = HAL_GetTick();
    if (smap_has_written_once &&
        (uint32_t)(now - smap_last_write_tick) < SMAP_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    /* Build the slot in RAM */
    smap_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic      = SMAP_MAGIC;
    slot.valid_flag = SMAP_VALID_FLAG;
    memcpy(slot.tempMap, map, SMAP_NUM_SENSORS);
    slot.checksum   = smap_crc32(&slot, offsetof(smap_flash_slot_t, checksum));

    /* Unlock flash */
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return false;

    /* Erase page 125 */
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = SMAP_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    /* Write slot (64-bit doubleword-aligned writes, as required by STM32G4) */
    uint32_t       dword_count = sizeof(smap_flash_slot_t) / 8U;
    const uint64_t *src        = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   SMAP_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    /* Update RAM state + rate-limit bookkeeping */
    smap_flash_valid      = true;
    smap_has_written_once = true;
    smap_last_write_tick  = HAL_GetTick();
    memcpy(smap_active, map, SMAP_NUM_SENSORS);
    return true;
}

const uint8_t *SensorMapStore_GetMap(void)
{
    return smap_active;
}

bool SensorMapStore_IsValid(void)
{
    return smap_flash_valid;
}
