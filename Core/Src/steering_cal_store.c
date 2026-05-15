/**
  ****************************************************************************
  * @file    steering_cal_store.c
  * @brief   Persistent steering calibration storage — flash persistence
  *
  * Stores the encoder-count-at-center in the second-to-last flash page
  * of the STM32G474RE (page 126, 4 KB at 0x0807E000).
  * A 32-bit CRC32 checksum and a 32-bit magic word provide integrity.
  *
  * Page 126 is separate from the EPS-parameter page (127) so that
  * writing one never erases the other.
  *
  * Safety invariant:
  *   Flash data alone NEVER authorises ACTIVE.  The physical center
  *   sensor must confirm plausibility before the stored calibration
  *   is accepted (enforced by SteeringCal_ValidateAtBoot).
  ****************************************************************************
  */

#include "steering_cal_store.h"
#include "stm32g4xx_hal.h"
#include "main.h"
#include "sensor_manager.h"
#include "safety_system.h"
#include <string.h>
#include <stddef.h>

/* ---- Flash layout ----
 * STM32G474RE: 512 KB flash, 128 pages of 4 KB each.
 * Page 126 starts at 0x0807E000 (bank 1).
 * Single slot at the beginning of the page.                       */
#define STCAL_FLASH_PAGE       126U
#define STCAL_FLASH_BASE       0x0807E000U

#define STCAL_MAGIC            0x53544331U   /* "STC1" */
#define STCAL_VALID_FLAG       0xA5U

/* ---- Flash-wear rate limit (Fase 4 hardening) -----------------------
 * Mirrors PCAL_WRITE_MIN_INTERVAL_MS in pedal_cal_store.c and
 * SMAP_WRITE_MIN_INTERVAL_MS in sensor_map_store.c (single project
 * pattern).  Steering recentre + save is a human-driven operation in
 * BOOT/STANDBY, so 1 s is far above any legitimate cadence yet still
 * permits an immediate retry after a failed save.  The very first
 * save after boot is exempt (stcal_has_written_once == false) so a
 * fresh device can always persist its first calibration.            */
#define STCAL_WRITE_MIN_INTERVAL_MS  1000U

/* ---- On-flash slot format ---- */
typedef struct {
    uint32_t magic;                     /* Must equal STCAL_MAGIC          */
    int32_t  encoder_count_at_center;   /* TIM2 count at calibrated center */
    uint8_t  validity_flag;             /* STCAL_VALID_FLAG when valid      */
    uint8_t  reserved[3];              /* Padding / future use             */
    uint32_t checksum;                  /* CRC32 of all fields before this */
} stcal_flash_slot_t;

/* ---- RAM state ---- */
static bool    stcal_flash_valid   = false;   /* Flash slot passed CRC   */
static bool    stcal_boot_valid    = false;   /* Boot validation passed  */
static int32_t stcal_stored_center = 0;

/* ---- Flash-wear rate-limit bookkeeping (Fase 4) ----
 * Same scheme as pedal_cal_store.c: first call after boot is exempt
 * (stcal_has_written_once == false) so a fresh device always
 * accepts its first persistence; subsequent calls within
 * STCAL_WRITE_MIN_INTERVAL_MS are rejected.  HAL_GetTick() is
 * monotonic with ~49.7 day wrap; unsigned modular subtraction is
 * safe across the wrap.                                              */
static bool     stcal_has_written_once = false;
static uint32_t stcal_last_write_tick  = 0U;

/* ---- CRC32 (same polynomial as eps_params.c, software) ---- */
static uint32_t stcal_crc32(const void *data, uint32_t len)
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

/* ---- Validate a flash slot ---- */
static bool stcal_slot_valid(const stcal_flash_slot_t *slot)
{
    if (slot->magic != STCAL_MAGIC) return false;
    if (slot->validity_flag != STCAL_VALID_FLAG) return false;
    uint32_t crc = stcal_crc32(slot, offsetof(stcal_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

/* ==================================================================
 *  Public API
 * ================================================================== */

void SteeringCal_Init(void)
{
    stcal_flash_valid   = false;
    stcal_boot_valid    = false;
    stcal_stored_center = 0;

    const stcal_flash_slot_t *slot =
        (const stcal_flash_slot_t *)STCAL_FLASH_BASE;

    if (stcal_slot_valid(slot)) {
        stcal_flash_valid   = true;
        stcal_stored_center = slot->encoder_count_at_center;
    }
}

bool SteeringCal_ValidateAtBoot(void)
{
    stcal_boot_valid = false;

    /* 1. Flash must contain a valid slot */
    if (!stcal_flash_valid)
        return false;

    /* 2. Current encoder position must be close to stored center.
     *    After MX_TIM2_Init the counter starts at 0 (or whatever
     *    value TIM2->CNT holds after reset — hardware resets to 0).
     *    Because centering zeroes the counter at center, the stored
     *    value should be ~0.  At boot, the counter is 0 if the
     *    steering hasn't moved.  If it HAS moved, the quadrature
     *    encoder won't track movement while powered off, so the
     *    counter will still read 0 but the stored center is also 0.
     *    The real discriminator is the center sensor: if the wheels
     *    were turned while power was off, the screw is no longer in
     *    front of the sensor.                                         */
    int32_t current = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t diff    = current - stcal_stored_center;
    if (diff < 0) diff = -diff;

    if (diff > STEERING_CAL_TOLERANCE_COUNTS)
        return false;

    /* 3. Physical center sensor must agree.
     *    The LJ12A3 inductive sensor with NPN open-collector and
     *    internal pull-up reads LOW when the screw is detected
     *    (metal in proximity).  At boot, if steering is still at
     *    center the sensor pin should be LOW = GPIO_PIN_RESET.
     *
     *    This is the critical safety gate: flash alone NEVER
     *    authorises ACTIVE.                                           */
    if (HAL_GPIO_ReadPin(GPIOB, PIN_STEER_CENTER) != GPIO_PIN_RESET)
        return false;

    stcal_boot_valid = true;
    return true;
}

bool SteeringCal_IsRestoredValid(void)
{
    return stcal_boot_valid;
}

int32_t SteeringCal_GetStoredCenter(void)
{
    return stcal_stored_center;
}

bool SteeringCal_Save(int32_t encoder_count_at_center)
{
    /* Defense in depth: never erase page 126 while the vehicle is
     * driving.  The only legitimate caller (SteeringCentering_Step)
     * runs in BOOT / STANDBY, so this guard rejects ACTIVE /
     * DEGRADED / SAFE / LIMP_HOME and is permissive everywhere the
     * centering routine can legitimately complete.                  */
    SystemState_t st = Safety_GetState();
    if (st != SYS_STATE_BOOT && st != SYS_STATE_STANDBY)
        return false;

    /* Flash-wear guard #1 — no-op elision.
     *
     * If the slot already in flash matches the requested center,
     * the caller's desired state is already persisted; return
     * success without erasing the page.  Mirrors
     * pedal_cal_store.c::PedalCal_Save and
     * sensor_map_store.c::SensorMapStore_Save.                       */
    if (stcal_flash_valid && stcal_stored_center == encoder_count_at_center) {
        return true;
    }

    /* Flash-wear guard #2 — minimum write interval.
     *
     * Reject successive writes that arrive closer together than
     * STCAL_WRITE_MIN_INTERVAL_MS (1 s).  The first call after
     * boot (stcal_has_written_once == false) is exempt so a
     * freshly booted unit always accepts its first calibration.    */
    uint32_t now_tick = HAL_GetTick();
    if (stcal_has_written_once &&
        (uint32_t)(now_tick - stcal_last_write_tick) < STCAL_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    /* Build the slot in RAM */
    stcal_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic                   = STCAL_MAGIC;
    slot.encoder_count_at_center = encoder_count_at_center;
    slot.validity_flag           = STCAL_VALID_FLAG;
    slot.checksum = stcal_crc32(&slot,
                                offsetof(stcal_flash_slot_t, checksum));

    /* Unlock flash */
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return false;

    /* Erase page 126 */
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = STCAL_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    /* Write the slot (double-word aligned).
     * STM32G4 flash requires 64-bit (double-word) writes.           */
    uint32_t slot_size   = sizeof(stcal_flash_slot_t);
    uint32_t dword_count = (slot_size + 7U) / 8U;
    const uint64_t *src  = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   STCAL_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    /* Update RAM state.
     * Refresh the rate-limit timestamp with HAL_GetTick() at
     * completion (not the `now_tick` captured before the erase) so
     * the cool-down window starts from the moment the page is
     * actually committed (page erase + program ≈ 25 ms).            */
    stcal_flash_valid     = true;
    stcal_stored_center   = encoder_count_at_center;
    stcal_has_written_once = true;
    stcal_last_write_tick = HAL_GetTick();
    return true;
}
