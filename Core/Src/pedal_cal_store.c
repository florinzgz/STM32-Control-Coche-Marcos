/**
  ****************************************************************************
  * @file    pedal_cal_store.c
  * @brief   Persistent pedal calibration storage — flash persistence
  *
  * Stores the calibrated pedal ADC endpoints (min / max) in flash page
  * 124 (0x0807C000, 4 KB) of the STM32G474RE.  A 32-bit CRC32 checksum
  * plus a 32-bit magic word provide integrity, identical to the
  * proven steering_cal_store.c implementation.
  *
  * Page 124 is reserved for pedal calibration only and is separate
  * from page 123 (sensor map), 125 (error log), 126 (steering
  * calibration), and 127 (EPS parameters), so each NVM slot can be
  * erased independently.
  *
  * Safety invariants:
  *   - Flash data alone NEVER authorises ACTIVE or clears
  *     startup_inhibit.  Calibration only changes the raw→% mapping.
  *   - On CRC / magic / range failure: silent fallback to defaults
  *     (PedalCal_IsValid() returns false; caller keeps compile-time
  *     150 / 2413 endpoints).  Boot is never blocked.
  *   - Plausibility, FAULT_LO/HI thresholds, EMA, rate-of-change,
  *     and dual-sample consistency checks in Pedal_Update() are
  *     unchanged and apply equally to default and calibrated maps.
  *
  * The flash write primitive set (HAL_FLASH_Unlock, HAL_FLASHEx_Erase,
  * HAL_FLASH_Program with FLASH_TYPEPROGRAM_DOUBLEWORD, HAL_FLASH_Lock)
  * is modelled 1:1 on steering_cal_store.c — no IRQ disables, no RTOS
  * primitives, no watchdog interaction.
  ****************************************************************************
  */

#include "pedal_cal_store.h"
#include "stm32g4xx_hal.h"
#include "safety_system.h"
#include <string.h>
#include <stddef.h>

/* ---- Flash layout ----
 * STM32G474RE: 512 KB flash, 128 pages of 4 KB each.
 * Page 124 starts at 0x0807C000 (bank 1).
 * Single slot at the beginning of the page.                       */
#define PCAL_FLASH_PAGE        124U
#define PCAL_FLASH_BASE        0x0807C000U

#define PCAL_MAGIC             0x50434C31U   /* "PCL1" */
#define PCAL_VALID_FLAG        0xA5U

/* ---- On-flash slot format ----
 * Exactly 16 bytes, double-word aligned, identical structure pattern
 * to stcal_flash_slot_t.                                              */
typedef struct {
    uint32_t magic;          /* Must equal PCAL_MAGIC                   */
    uint16_t adc_min;        /* Calibrated ADC count at pedal released  */
    uint16_t adc_max;        /* Calibrated ADC count at pedal pressed   */
    uint8_t  validity_flag;  /* PCAL_VALID_FLAG when slot is committed  */
    uint8_t  reserved[3];    /* Padding / future use                    */
    uint32_t checksum;       /* CRC32 of all fields before this field   */
} pcal_flash_slot_t;

/* Compile-time guarantee that the on-flash layout is exactly 16 bytes
 * — the struct is double-word aligned so HAL_FLASH_Program can
 * write it in two 8-byte doublewords.                                  */
typedef char pcal_size_check_[(sizeof(pcal_flash_slot_t) == 16) ? 1 : -1];

/* ---- RAM state ---- */
static bool     pcal_flash_valid   = false;   /* Flash slot passed CRC + range */
static uint16_t pcal_stored_min    = 0;
static uint16_t pcal_stored_max    = 0;

/* ---- CRC32 (same polynomial as steering_cal_store / eps_params) ---- */
static uint32_t pcal_crc32(const void *data, uint32_t len)
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

/* ---- Validate a flash slot (integrity only) ---- */
static bool pcal_slot_integrity_ok(const pcal_flash_slot_t *slot)
{
    if (slot->magic != PCAL_MAGIC) return false;
    if (slot->validity_flag != PCAL_VALID_FLAG) return false;
    uint32_t crc = pcal_crc32(slot, offsetof(pcal_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

/* ==================================================================
 *  Public API
 * ================================================================== */

bool PedalCal_Validate(uint16_t adc_min, uint16_t adc_max)
{
    if (adc_min < PEDAL_CAL_MIN_LIMIT) return false;
    if (adc_max > PEDAL_CAL_MAX_LIMIT) return false;
    if (adc_max <= adc_min)            return false;
    if ((uint32_t)(adc_max - adc_min) < PEDAL_CAL_RANGE_MIN) return false;
    return true;
}

void PedalCal_Init(void)
{
    pcal_flash_valid = false;
    pcal_stored_min  = 0;
    pcal_stored_max  = 0;

    const pcal_flash_slot_t *slot =
        (const pcal_flash_slot_t *)PCAL_FLASH_BASE;

    if (!pcal_slot_integrity_ok(slot))
        return;  /* silent fallback to defaults */

    /* Range validation — even an integrity-good slot must satisfy
     * the hard limits before we trust it.  Out-of-range values are
     * treated identically to a corrupt slot: silent fallback.        */
    if (!PedalCal_Validate(slot->adc_min, slot->adc_max))
        return;

    pcal_flash_valid = true;
    pcal_stored_min  = slot->adc_min;
    pcal_stored_max  = slot->adc_max;
}

bool PedalCal_IsValid(void)
{
    return pcal_flash_valid;
}

void PedalCal_GetStored(uint16_t *adc_min, uint16_t *adc_max)
{
    if (adc_min) *adc_min = pcal_stored_min;
    if (adc_max) *adc_max = pcal_stored_max;
}

bool PedalCal_Save(uint16_t adc_min, uint16_t adc_max)
{
    /* Defense in depth: never persist while actuators may be live.
     * The CAN dispatcher (pedalcal_safety_ok in can_handler.c) already
     * blocks any caller path outside SYS_STATE_STANDBY, but the same
     * gate is re-asserted at the persistence boundary so that any
     * future caller (service-mode shortcut, host test fixture, etc.)
     * cannot accidentally erase page 124 while the vehicle is in
     * ACTIVE / DEGRADED / LIMP_HOME state.                            */
    if (Safety_GetState() != SYS_STATE_STANDBY)
        return false;

    /* Hard validation gate — never persist out-of-range endpoints. */
    if (!PedalCal_Validate(adc_min, adc_max))
        return false;

    /* Build the slot in RAM */
    pcal_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic         = PCAL_MAGIC;
    slot.adc_min       = adc_min;
    slot.adc_max       = adc_max;
    slot.validity_flag = PCAL_VALID_FLAG;
    slot.checksum      = pcal_crc32(&slot,
                                    offsetof(pcal_flash_slot_t, checksum));

    /* Unlock flash */
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return false;

    /* Erase page 124 */
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = PCAL_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    /* Write the slot (double-word aligned).
     * STM32G4 flash requires 64-bit (double-word) writes.
     * Slot is exactly 16 bytes → 2 doublewords.                      */
    uint32_t slot_size   = sizeof(pcal_flash_slot_t);
    uint32_t dword_count = (slot_size + 7U) / 8U;
    const uint64_t *src  = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   PCAL_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    /* Update RAM state */
    pcal_flash_valid = true;
    pcal_stored_min  = adc_min;
    pcal_stored_max  = adc_max;
    return true;
}
