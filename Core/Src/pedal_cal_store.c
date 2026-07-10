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
  *     50 / 4000 endpoints).  Boot is never blocked.
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

/* ---- Flash-wear rate limit ---------------------------------------
 * Minimum interval between consecutive successful flash writes on
 * page 124.  Protects the page from wear caused by a CAN-frame
 * storm or a buggy/malicious caller spamming PEDAL_CAL_OP_SAVE /
 * PEDAL_CAL_OP_RESET_DEFAULTS.  Mirrors the SMAP_WRITE_MIN_INTERVAL_MS
 * guard in sensor_map_store.c (single source of truth pattern).
 * 1 s is far slower than any human-driven calibration cadence yet
 * still permits the legitimate SAVE-then-RESET test flow.          */
#define PCAL_WRITE_MIN_INTERVAL_MS  1000U

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

/* ---- Flash-wear rate-limit bookkeeping ----
 * pcal_has_written_once stays false until the first successful flash
 * write, so the very first PedalCal_Save() call after boot is never
 * gated by the timestamp (matches sensor_map_store.c semantics).
 * pcal_last_write_tick uses HAL_GetTick() and is compared with
 * unsigned modular subtraction so wrap-around at ~49.7 days is safe. */
static bool     pcal_has_written_once = false;
static uint32_t pcal_last_write_tick  = 0U;

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
#if PEDAL_CAL_MIN_LIMIT > 0U
    if (adc_min < PEDAL_CAL_MIN_LIMIT) return false;
#endif
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

    /* Reset the rate-limit bookkeeping on every init so a re-init
     * never starts inside a cool-down window collapsed across reboot
     * — the first PedalCal_Save() call will always be allowed.
     * Matches sensor_map_store.c semantics.                          */
    pcal_has_written_once = false;
    pcal_last_write_tick  = 0U;

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

    /* ----------------------------------------------------------------------
     * Flash-wear guard #1 — no-op elision.
     *
     * If the requested endpoints exactly match the slot currently held in
     * flash (validated at boot or by the last successful Save), the caller's
     * desired state is already persisted.  Return success without erasing
     * the page.  This is what makes PEDAL_CAL_OP_RESET_DEFAULTS idempotent:
     * after the first RESET_DEFAULTS, subsequent RESET_DEFAULTS frames
     * cannot wear page 124 because the slot already contains the defaults.
     *
     * Consistent with sensor_map_store.c::SensorMapStore_Save.
     * --------------------------------------------------------------------*/
    if (pcal_flash_valid &&
        pcal_stored_min == adc_min &&
        pcal_stored_max == adc_max) {
        return true;
    }

    /* ----------------------------------------------------------------------
     * Flash-wear guard #2 — minimum write interval.
     *
     * Reject successive writes that arrive closer together than
     * PCAL_WRITE_MIN_INTERVAL_MS (1 s).  Pedal calibration is strictly
     * user-driven (UI button press); writes faster than 1 Hz can only
     * come from a bug, replay, or injected traffic.  The first call
     * (pcal_has_written_once == false) is exempt so a freshly booted
     * unit can always persist its first calibration.
     * HAL_GetTick() is monotonic and wraps every ~49.7 days; the
     * unsigned modular subtraction below is correct across the wrap.
     * --------------------------------------------------------------------*/
    uint32_t now = HAL_GetTick();
    if (pcal_has_written_once &&
        (uint32_t)(now - pcal_last_write_tick) < PCAL_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    /* Build the slot in RAM.
     * Aligned to 8 bytes so the (uint64_t *)&slot cast below performs
     * naturally-aligned doubleword loads (required by ARMv7-M LDRD and
     * safe for HAL_FLASH_Program's FLASH_TYPEPROGRAM_DOUBLEWORD).
     * _Alignas is the C11 standard keyword (same as _Static_assert
     * used elsewhere in this project) — no compiler-specific syntax. */
    _Alignas(8) pcal_flash_slot_t slot;
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

    /* Update RAM state + rate-limit bookkeeping (matches the pattern
     * in sensor_map_store.c::SensorMapStore_Save).  Refresh the
     * timestamp to HAL_GetTick() at completion rather than reusing
     * `now`, so the cool-down window starts from the moment the page
     * is actually committed (page erase + program ≈ 25 ms).         */
    pcal_flash_valid      = true;
    pcal_stored_min       = adc_min;
    pcal_stored_max       = adc_max;
    pcal_has_written_once = true;
    pcal_last_write_tick  = HAL_GetTick();
    return true;
}
