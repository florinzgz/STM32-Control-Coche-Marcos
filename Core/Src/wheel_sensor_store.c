/**
  ****************************************************************************
  * @file    wheel_sensor_store.c
  * @brief   Persistent GLOBAL wheel-sensor geometry/timing — flash storage
  ****************************************************************************
  */

#include "wheel_sensor_store.h"
#include "project_config.h"
#include "vehicle_physics.h"
#include "safety_system.h"     /* WHEEL_FAULT_DEBOUNCE_MS default */
#include "sensor_manager.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stddef.h>

/* ---- Flash layout ---- */
#define WSNSTORE_FLASH_PAGE   116U
#define WSNSTORE_FLASH_BASE   0x08074000U

/* ---- On-flash format ---- */
#define WSNSTORE_MAGIC        0x57534E31U   /* "WSN1" */
#define WSNSTORE_VALID_FLAG   0xA5U

/* ---- Flash-wear rate limit ---- */
#define WSNSTORE_WRITE_MIN_INTERVAL_MS  1000U

typedef struct {
    uint32_t magic;
    uint16_t pulses_per_rev;
    float    circumference_mm;
    uint16_t debounce_us;
    uint16_t mismatch_debounce_ms;
    uint8_t  validity_flag;
    uint8_t  reserved0;
    uint16_t reserved1;
    uint32_t checksum;
} wsnstore_flash_slot_t;

typedef char wsnstore_size_check_[(sizeof(wsnstore_flash_slot_t) == 24) ? 1 : -1];

static bool                wsn_flash_valid = false;
static WheelSensorParams_t wsn_stored      = {0};
static WheelSensorParams_t wsn_staged      = {0};
static bool                wsn_has_written_once = false;
static uint32_t            wsn_last_write_tick  = 0U;

static uint32_t wsn_crc32(const void *data, uint32_t len)
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

static bool wsn_slot_integrity_ok(const wsnstore_flash_slot_t *slot)
{
    if (slot->magic != WSNSTORE_MAGIC) return false;
    if (slot->validity_flag != WSNSTORE_VALID_FLAG) return false;
    uint32_t crc = wsn_crc32(slot, offsetof(wsnstore_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

/* Push the RAM-staged debounce window into sensor_manager.c's ISR-facing
 * cached cycle count.  Cheap (one multiply), safe to call from Init/Stage/
 * Save/Revert/ResetToDefaults -- never from the ISR itself. */
static void wsn_apply(const WheelSensorParams_t *w)
{
    Sensor_SetDebounceUs(w->debounce_us);
}

void WheelSensorStore_GetDefaults(WheelSensorParams_t *out)
{
    if (!out) return;
    out->pulses_per_rev      = (uint16_t)WHEEL_PULSES_REV;
    out->circumference_mm    = (float)WHEEL_CIRCUM_MM;
    out->debounce_us         = (uint16_t)SENSOR_DEBOUNCE_US;
    out->mismatch_debounce_ms = (uint16_t)WHEEL_FAULT_DEBOUNCE_MS;
}

bool WheelSensorStore_Validate(const WheelSensorParams_t *w)
{
    return WheelSensor_ValidateValues(w);
}

void WheelSensorStore_Init(void)
{
    wsn_flash_valid      = false;
    memset(&wsn_stored, 0, sizeof(wsn_stored));
    wsn_has_written_once = false;
    wsn_last_write_tick  = 0U;

    const wsnstore_flash_slot_t *slot =
        (const wsnstore_flash_slot_t *)WSNSTORE_FLASH_BASE;

    WheelSensorStore_GetDefaults(&wsn_stored);

    if (wsn_slot_integrity_ok(slot)) {
        WheelSensorParams_t cand;
        cand.pulses_per_rev      = slot->pulses_per_rev;
        cand.circumference_mm    = slot->circumference_mm;
        cand.debounce_us         = slot->debounce_us;
        cand.mismatch_debounce_ms = slot->mismatch_debounce_ms;

        if (WheelSensorStore_Validate(&cand)) {
            wsn_flash_valid = true;
            wsn_stored      = cand;
        }
    }

    wsn_staged = wsn_stored;
    wsn_apply(&wsn_staged);
}

bool WheelSensorStore_IsValid(void)
{
    return wsn_flash_valid;
}

void WheelSensorStore_GetEffective(WheelSensorParams_t *out)
{
    if (out) *out = wsn_staged;
}

uint16_t WheelSensorStore_GetEffectivePulsesPerRev(void)
{
    return wsn_staged.pulses_per_rev;
}

float WheelSensorStore_GetEffectiveCircumferenceM(void)
{
    return wsn_staged.circumference_mm / 1000.0f;
}

uint16_t WheelSensorStore_GetEffectiveDebounceUs(void)
{
    return wsn_staged.debounce_us;
}

uint16_t WheelSensorStore_GetEffectiveMismatchDebounceMs(void)
{
    return wsn_staged.mismatch_debounce_ms;
}

bool WheelSensorStore_Stage(const WheelSensorParams_t *w)
{
    if (!w) return false;
    if (!WheelSensorStore_Validate(w)) return false;
    wsn_staged = *w;
    wsn_apply(&wsn_staged);
    return true;
}

void WheelSensorStore_GetStaged(WheelSensorParams_t *out)
{
    if (out) *out = wsn_staged;
}

void WheelSensorStore_Revert(void)
{
    wsn_staged = wsn_stored;
    wsn_apply(&wsn_staged);
}

void WheelSensorStore_ResetToDefaults(void)
{
    WheelSensorStore_GetDefaults(&wsn_staged);
    wsn_apply(&wsn_staged);
}

bool WheelSensorStore_Save(void)
{
    const WheelSensorParams_t *w = &wsn_staged;

    /* Defense in depth: never persist while actuators may be live (flash
     * erase/program blocks the CPU for tens of ms). Consistent with every
     * other flash-backed store. No production caller exists yet (Block A),
     * so this is pure future-proofing against a caller that forgets to
     * gate to STANDBY. Compiled out in host tests, which stub
     * Safety_GetState() to always report STANDBY. */
    if (Safety_GetState() != SYS_STATE_STANDBY)
        return false;

    if (!WheelSensorStore_Validate(w))
        return false;

    if (wsn_flash_valid &&
        memcmp(&wsn_stored, w, sizeof(*w)) == 0) {
        return true;
    }

    uint32_t now = HAL_GetTick();
    if (wsn_has_written_once &&
        (uint32_t)(now - wsn_last_write_tick) < WSNSTORE_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    _Alignas(8) wsnstore_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic                = WSNSTORE_MAGIC;
    slot.pulses_per_rev       = w->pulses_per_rev;
    slot.circumference_mm     = w->circumference_mm;
    slot.debounce_us          = w->debounce_us;
    slot.mismatch_debounce_ms = w->mismatch_debounce_ms;
    slot.validity_flag        = WSNSTORE_VALID_FLAG;
    slot.checksum             = wsn_crc32(&slot,
                                    offsetof(wsnstore_flash_slot_t, checksum));

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return false;

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = WSNSTORE_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    uint32_t slot_size   = sizeof(wsnstore_flash_slot_t);
    uint32_t dword_count = (slot_size + 7U) / 8U;
    const uint64_t *src  = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   WSNSTORE_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    wsn_flash_valid      = true;
    wsn_stored           = *w;
    wsn_has_written_once = true;
    wsn_last_write_tick  = HAL_GetTick();
    return true;
}
