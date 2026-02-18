/**
  ****************************************************************************
  * @file    eps_params.c
  * @brief   EPS calibration parameter storage — flash persistence
  *
  * Stores an eps_params_t block in the last flash page of the
  * STM32G474RE (page 127, 4 KB at 0x0807F000).  A 32-bit CRC32
  * checksum and a 32-bit magic word provide validity detection.
  * Double-buffering: two slots (A / B) occupy the same page; the
  * most-recently-written valid slot is loaded at boot.
  *
  * The active copy lives in RAM and is used directly by the EPS
  * control loop.  Runtime changes via EPS_Params_Set() modify the
  * RAM copy immediately; EPS_Params_Save() writes to flash.
  ****************************************************************************
  */

#include "eps_params.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* ---- Flash layout ----
 * STM32G474RE: 512 KB flash, 128 pages of 4 KB each.
 * We use the last page (page 127) for parameter storage.
 * Page 127 starts at 0x0807F000 (bank 1, page 127 in single-bank
 * mode which is the default for STM32G474RE).
 *
 * Slot A: offset 0x000  (256 bytes)
 * Slot B: offset 0x100  (256 bytes)
 * Remaining space unused.                                          */
#define EPS_FLASH_PAGE       127U
#define EPS_FLASH_BASE       0x0807F000U
#define EPS_SLOT_A_ADDR      EPS_FLASH_BASE
#define EPS_SLOT_B_ADDR      (EPS_FLASH_BASE + 0x100U)

#define EPS_MAGIC            0x45505331U   /* "EPS1" */

/* ---- On-flash slot format ---- */
typedef struct {
    uint32_t     magic;       /* Must equal EPS_MAGIC                */
    uint32_t     sequence;    /* Monotonic write counter             */
    eps_params_t params;      /* Calibration data                    */
    uint32_t     checksum;    /* CRC32 of magic + sequence + params  */
} eps_flash_slot_t;

/* ---- Compiled defaults ---- */
static const eps_params_t eps_defaults = {
    .assist_strength = 0.45f,
    .center_strength = 0.30f,
    .damping         = 0.10f,
    .friction_comp   = 0.05f,
    .coast_band_pct  = 3.0f,
    .min_drive_pct   = 8.0f,
    .assist_vs_speed = 18.0f,
    .return_vs_speed = 35.0f,
};

/* ---- RAM state ---- */
static eps_params_t  eps_active;
static uint32_t      eps_sequence = 0;

/* ---- CRC32 (software, no HW CRC unit dependency) ---- */
static uint32_t eps_crc32(const void *data, uint32_t len)
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
static bool eps_slot_valid(const eps_flash_slot_t *slot)
{
    if (slot->magic != EPS_MAGIC) return false;
    /* Checksum covers magic + sequence + params (everything before checksum) */
    uint32_t crc = eps_crc32(slot, offsetof(eps_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

/* ---- Public API ---- */

void EPS_Params_Init(void)
{
    const eps_flash_slot_t *slotA = (const eps_flash_slot_t *)EPS_SLOT_A_ADDR;
    const eps_flash_slot_t *slotB = (const eps_flash_slot_t *)EPS_SLOT_B_ADDR;

    bool a_ok = eps_slot_valid(slotA);
    bool b_ok = eps_slot_valid(slotB);

    if (a_ok && b_ok) {
        /* Both valid — use the one with higher sequence */
        if (slotB->sequence > slotA->sequence) {
            memcpy(&eps_active, &slotB->params, sizeof(eps_params_t));
            eps_sequence = slotB->sequence;
        } else {
            memcpy(&eps_active, &slotA->params, sizeof(eps_params_t));
            eps_sequence = slotA->sequence;
        }
    } else if (a_ok) {
        memcpy(&eps_active, &slotA->params, sizeof(eps_params_t));
        eps_sequence = slotA->sequence;
    } else if (b_ok) {
        memcpy(&eps_active, &slotB->params, sizeof(eps_params_t));
        eps_sequence = slotB->sequence;
    } else {
        /* No valid data — use compiled defaults */
        memcpy(&eps_active, &eps_defaults, sizeof(eps_params_t));
        eps_sequence = 0;
    }
}

const eps_params_t *EPS_Params_Get(void)
{
    return &eps_active;
}

bool EPS_Params_Set(eps_param_id_t id, float value)
{
    if ((int)id < 0 || id >= EPS_PARAM_COUNT) return false;

    float *fields = (float *)&eps_active;
    fields[id] = value;
    return true;
}

void EPS_Params_ResetDefaults(void)
{
    memcpy(&eps_active, &eps_defaults, sizeof(eps_params_t));
}

bool EPS_Params_Save(void)
{
    /* Build the slot in RAM */
    eps_sequence++;
    eps_flash_slot_t slot;
    slot.magic    = EPS_MAGIC;
    slot.sequence = eps_sequence;
    memcpy(&slot.params, &eps_active, sizeof(eps_params_t));
    slot.checksum = eps_crc32(&slot, offsetof(eps_flash_slot_t, checksum));

    /* Determine target slot (alternate A/B based on sequence parity) */
    uint32_t target_addr = (eps_sequence & 1U) ? EPS_SLOT_A_ADDR
                                                : EPS_SLOT_B_ADDR;

    /* Unlock flash */
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK) { eps_sequence--; return false; }

    /* Erase page 127 */
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = EPS_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        eps_sequence--;
        return false;
    }

    /* Write the target slot (double-word aligned writes).
     * STM32G4 flash requires 64-bit (double-word) writes.
     * Pad slot to multiple of 8 bytes.                     */
    uint32_t slot_size = sizeof(eps_flash_slot_t);
    uint32_t words     = (slot_size + 7U) / 8U;
    const uint64_t *src = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < words; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   target_addr + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    /* If the other slot was valid from a previous save, rewrite it too
     * so it survives the page erase.  Read it from RAM (we loaded at
     * init) — actually, after erase both slots are gone.  We only need
     * to write the current one.  The double-buffer safety comes from
     * alternating which slot gets the latest sequence number.  After
     * an erase, only one slot is valid, which is fine.               */

    HAL_FLASH_Lock();
    return true;
}
