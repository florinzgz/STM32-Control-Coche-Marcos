/**
  ****************************************************************************
  * @file    drive_tuning_store.c
  * @brief   Persistent drive-tuning storage — flash persistence
  *
  * Stores the runtime-tunable traction "feel" parameters (accel/brake/
  * reverse ramp rates, creep enable/power/delay) in flash page 121
  * (0x08079000, 4 KB) of the STM32G474RE.  A 32-bit CRC32 checksum plus a
  * 32-bit magic word ("DTN1") provide integrity, identical to the proven
  * gear_limits_store.c / pedal_cal_store.c implementations.
  *
  * Page 121 is reserved for drive tuning only and is separate from page 120
  * (battery limits), 122 (gear limits), 123 (sensor map), 124 (pedal cal),
  * 125 (error log), 126 (steering cal) and 127 (EPS params), so each NVM
  * slot can be erased independently.
  *
  * Safety invariants:
  *   - Flash data alone NEVER authorises ACTIVE or clears startup_inhibit.
  *     The parameters only shape an already-validated traction demand.
  *   - On CRC / magic / range failure: silent fallback to defaults
  *     (DriveTuningStore_IsValid() returns false; caller keeps compile-time
  *     values that reproduce the historic firmware behaviour).  Boot is
  *     never blocked.
  *
  * The flash write primitive set (HAL_FLASH_Unlock, HAL_FLASHEx_Erase,
  * HAL_FLASH_Program with FLASH_TYPEPROGRAM_DOUBLEWORD, HAL_FLASH_Lock) is
  * modelled 1:1 on gear_limits_store.c — no IRQ disables, no RTOS
  * primitives, no watchdog interaction.
  ****************************************************************************
  */

#include "drive_tuning_store.h"
#include "stm32g4xx_hal.h"
#include "safety_system.h"
#include <string.h>
#include <stddef.h>

/* ---- Flash layout ----
 * STM32G474RE: 512 KB flash, 128 pages of 4 KB each.
 * Page 121 starts at 0x08079000 (bank 1).  Single slot at the start.   */
#define DTUN_FLASH_PAGE   121U
#define DTUN_FLASH_BASE   0x08079000U

/* ---- On-flash format ---- */
#define DTUN_MAGIC        0x44544E31U   /* "DTN1" */
#define DTUN_VALID_FLAG   0xA5U

/* ---- Flash-wear rate limit -----------------------------------------
 * Minimum interval between consecutive successful flash writes on page
 * 121.  Mirrors the gear_limits_store guard so a CAN-frame storm or buggy
 * caller cannot wear the page.                                          */
#define DTUN_WRITE_MIN_INTERVAL_MS  1000U

/* ---- On-flash slot format ----
 * Exactly 16 bytes, double-word aligned (2 doublewords).               */
typedef struct {
    uint32_t magic;          /* DTUN_MAGIC                              */
    uint8_t  accel_ramp;     /* %/s ramp-up                             */
    uint8_t  brake_ramp;     /* %/s ramp-down                           */
    uint8_t  reverse_ramp;   /* %/s ramp-up in reverse                  */
    uint8_t  creep_enable;   /* 0/1                                     */
    uint8_t  creep_power;    /* % dead-zone floor                       */
    uint8_t  validity_flag;  /* DTUN_VALID_FLAG when committed          */
    uint16_t creep_delay;    /* ms                                      */
    uint32_t checksum;       /* CRC32 of all fields before this field   */
} dtun_flash_slot_t;

/* Compile-time guarantee that the on-flash layout is exactly 16 bytes. */
typedef char dtun_size_check_[(sizeof(dtun_flash_slot_t) == 16) ? 1 : -1];

/* ---- RAM state ---- */
static bool          dtun_flash_valid = false;
static DriveTuning_t dtun_stored      = {0};

/* ---- Flash-wear rate-limit bookkeeping ---- */
static bool     dtun_has_written_once = false;
static uint32_t dtun_last_write_tick  = 0U;

/* ---- CRC32 (same polynomial as gear_limits_store / pedal_cal_store) - */
static uint32_t dtun_crc32(const void *data, uint32_t len)
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

static bool dtun_slot_integrity_ok(const dtun_flash_slot_t *slot)
{
    if (slot->magic != DTUN_MAGIC) return false;
    if (slot->validity_flag != DTUN_VALID_FLAG) return false;
    uint32_t crc = dtun_crc32(slot, offsetof(dtun_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

/* ==================================================================
 *  Public API
 * ================================================================== */

void DriveTuningStore_GetDefaults(DriveTuning_t *out)
{
    if (!out) return;
    out->accel_ramp   = DRIVE_ACCEL_RAMP_DEFAULT;
    out->brake_ramp   = DRIVE_BRAKE_RAMP_DEFAULT;
    out->reverse_ramp = DRIVE_REVERSE_RAMP_DEFAULT;
    out->creep_enable = DRIVE_CREEP_ENABLE_DEFAULT;
    out->creep_power  = DRIVE_CREEP_POWER_DEFAULT;
    out->creep_delay  = DRIVE_CREEP_DELAY_DEFAULT;
}

bool DriveTuningStore_Validate(const DriveTuning_t *t)
{
    if (!t) return false;
    if (t->accel_ramp   < DRIVE_ACCEL_RAMP_MIN   || t->accel_ramp   > DRIVE_ACCEL_RAMP_MAX)   return false;
    if (t->brake_ramp   < DRIVE_BRAKE_RAMP_MIN   || t->brake_ramp   > DRIVE_BRAKE_RAMP_MAX)   return false;
    if (t->reverse_ramp < DRIVE_REVERSE_RAMP_MIN || t->reverse_ramp > DRIVE_REVERSE_RAMP_MAX) return false;
    if (t->creep_enable > 1U)                                                                 return false;
#if (DRIVE_CREEP_POWER_MIN > 0U)
    if (t->creep_power  < DRIVE_CREEP_POWER_MIN)                                               return false;
#endif
    if (t->creep_power  > DRIVE_CREEP_POWER_MAX)                                               return false;
#if (DRIVE_CREEP_DELAY_MIN > 0U)
    if (t->creep_delay  < DRIVE_CREEP_DELAY_MIN)                                               return false;
#endif
    if (t->creep_delay  > DRIVE_CREEP_DELAY_MAX)                                               return false;
    return true;
}

void DriveTuningStore_Init(void)
{
    dtun_flash_valid      = false;
    memset(&dtun_stored, 0, sizeof(dtun_stored));
    dtun_has_written_once = false;
    dtun_last_write_tick  = 0U;

    const dtun_flash_slot_t *slot = (const dtun_flash_slot_t *)DTUN_FLASH_BASE;

    if (!dtun_slot_integrity_ok(slot))
        return;  /* silent fallback to defaults */

    DriveTuning_t cand;
    cand.accel_ramp   = slot->accel_ramp;
    cand.brake_ramp   = slot->brake_ramp;
    cand.reverse_ramp = slot->reverse_ramp;
    cand.creep_enable = slot->creep_enable;
    cand.creep_power  = slot->creep_power;
    cand.creep_delay  = slot->creep_delay;

    /* Range validation — even an integrity-good slot must satisfy the hard
     * ranges before we trust it.  Out-of-range = corrupt = silent fallback. */
    if (!DriveTuningStore_Validate(&cand))
        return;

    dtun_flash_valid = true;
    dtun_stored      = cand;
}

bool DriveTuningStore_IsValid(void)
{
    return dtun_flash_valid;
}

void DriveTuningStore_GetStored(DriveTuning_t *out)
{
    if (out) *out = dtun_stored;
}

bool DriveTuningStore_Save(const DriveTuning_t *t)
{
    if (!t) return false;

    /* Defense in depth: never persist while actuators may be live. */
    if (Safety_GetState() != SYS_STATE_STANDBY)
        return false;

    /* Hard validation gate — never persist out-of-range values. */
    if (!DriveTuningStore_Validate(t))
        return false;

    /* Flash-wear guard #1 — no-op elision (makes RESET idempotent). */
    if (dtun_flash_valid &&
        dtun_stored.accel_ramp   == t->accel_ramp   &&
        dtun_stored.brake_ramp   == t->brake_ramp   &&
        dtun_stored.reverse_ramp == t->reverse_ramp &&
        dtun_stored.creep_enable == t->creep_enable &&
        dtun_stored.creep_power  == t->creep_power  &&
        dtun_stored.creep_delay  == t->creep_delay) {
        return true;
    }

    /* Flash-wear guard #2 — minimum write interval. */
    uint32_t now = HAL_GetTick();
    if (dtun_has_written_once &&
        (uint32_t)(now - dtun_last_write_tick) < DTUN_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    /* Build the slot in RAM (8-byte aligned for doubleword programming). */
    _Alignas(8) dtun_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic         = DTUN_MAGIC;
    slot.accel_ramp    = t->accel_ramp;
    slot.brake_ramp    = t->brake_ramp;
    slot.reverse_ramp  = t->reverse_ramp;
    slot.creep_enable  = t->creep_enable;
    slot.creep_power   = t->creep_power;
    slot.validity_flag = DTUN_VALID_FLAG;
    slot.creep_delay   = t->creep_delay;
    slot.checksum      = dtun_crc32(&slot,
                                    offsetof(dtun_flash_slot_t, checksum));

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return false;

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = DTUN_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    uint32_t slot_size   = sizeof(dtun_flash_slot_t);
    uint32_t dword_count = (slot_size + 7U) / 8U;
    const uint64_t *src  = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   DTUN_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    dtun_flash_valid      = true;
    dtun_stored           = *t;
    dtun_has_written_once = true;
    dtun_last_write_tick  = HAL_GetTick();
    return true;
}
