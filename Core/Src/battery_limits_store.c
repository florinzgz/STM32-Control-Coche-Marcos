/**
  ****************************************************************************
  * @file    battery_limits_store.c
  * @brief   Persistent battery-limit storage — flash persistence
  ****************************************************************************
  */

#include "battery_limits_store.h"
#include "stm32g4xx_hal.h"
#include "safety_system.h"
#include "motor_control.h"
#include "sensor_manager.h"
#include <string.h>
#include <stddef.h>
#include <math.h>

/* ---- Flash layout ---- */
#define BATT_FLASH_PAGE   120U
#define BATT_FLASH_BASE   0x08078000U

/* ---- On-flash format ---- */
#define BATT_MAGIC        0x42415431U   /* "BAT1" */
#define BATT_VALID_FLAG   0xA5U

/* ---- Flash-wear rate limit ---- */
#define BATT_WRITE_MIN_INTERVAL_MS  1000U

/* A service save outside STANDBY is accepted only when the vehicle is proven
 * stationary and no traction command can reach the hardware.  This closes the
 * deadlock where a low-voltage warning itself keeps the state in DEGRADED and
 * therefore made it impossible to correct the thresholds from the HMI. */
#define BATT_SERVICE_PEDAL_MAX_PCT  3.0f
#define BATT_SERVICE_SPEED_MAX_KMH  0.5f

typedef struct {
    uint32_t magic;
    uint16_t warning_cv;
    uint16_t limit_cv;
    uint16_t cutoff_cv;
    uint16_t recovery_cv;
    uint16_t filter_ms;
    uint8_t  validity_flag;
    uint8_t  reserved0;
    uint32_t reserved1;
    uint32_t checksum;
} batt_flash_slot_t;

typedef char batt_size_check_[(sizeof(batt_flash_slot_t) == 24) ? 1 : -1];

static bool            batt_flash_valid = false;
static BatteryLimits_t batt_stored      = {0};
static bool             batt_has_written_once = false;
static uint32_t         batt_last_write_tick  = 0U;

static uint32_t batt_crc32(const void *data, uint32_t len)
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

static bool batt_slot_integrity_ok(const batt_flash_slot_t *slot)
{
    if (slot->magic != BATT_MAGIC) return false;
    if (slot->validity_flag != BATT_VALID_FLAG) return false;
    uint32_t crc = batt_crc32(slot, offsetof(batt_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

bool BatteryLimitsStore_ServiceWriteAllowed(void)
{
    const SystemState_t state = Safety_GetState();

    if (state == SYS_STATE_STANDBY) {
        return true;
    }

    /* Never write while the safety authority is in a no-motion terminal state
     * or while LIMP_HOME is controlling a degraded pedal path. */
    if (state != SYS_STATE_ACTIVE && state != SYS_STATE_DEGRADED) {
        return false;
    }

    /* In DEGRADED, only the battery-UV warning is allowed to use this escape
     * path.  A sensor/current/thermal/CAN degradation must not be bypassed. */
    if (state == SYS_STATE_DEGRADED &&
        Safety_GetError() != SAFETY_ERROR_BATTERY_UV_WARNING) {
        return false;
    }

    const GearPosition_t gear = Traction_GetGear();
    if (gear != GEAR_PARK && gear != GEAR_NEUTRAL) {
        return false;
    }

    const float pedal = Pedal_GetPercent();
    if (!isfinite(pedal) || pedal >= BATT_SERVICE_PEDAL_MAX_PCT) {
        return false;
    }

    /* motor_control_patched publishes the resolved real shadow duty, including
     * Neutral's ramp.  The separate active-brake override can intentionally put
     * LPWM on all bridges while normal drive telemetry remains zero, so it must
     * also be disabled before flash erase/program operations are allowed. */
    if (Motor_GetBrakeActiveOverride() != 0U ||
        Traction_GetFinalPwmPct() != 0U) {
        return false;
    }

    const float speeds[4] = {
        Wheel_GetSpeed_FL(), Wheel_GetSpeed_FR(),
        Wheel_GetSpeed_RL(), Wheel_GetSpeed_RR()
    };
    for (uint8_t i = 0U; i < 4U; ++i) {
        if (!isfinite(speeds[i]) ||
            fabsf(speeds[i]) >= BATT_SERVICE_SPEED_MAX_KMH) {
            return false;
        }
    }

    return true;
}

void BatteryLimitsStore_GetDefaults(BatteryLimits_t *out)
{
    if (!out) return;
    out->warning_cv  = BATT_WARNING_DEFAULT_CV;
    out->limit_cv    = BATT_LIMIT_DEFAULT_CV;
    out->cutoff_cv   = BATT_CUTOFF_DEFAULT_CV;
    out->recovery_cv = BATT_RECOVERY_DEFAULT_CV;
    out->filter_ms   = BATT_FILTER_DEFAULT_MS;
}

bool BatteryLimitsStore_Validate(const BatteryLimits_t *b)
{
    return BatteryLimits_ValidateValues(b);
}

void BatteryLimitsStore_Init(void)
{
    batt_flash_valid      = false;
    memset(&batt_stored, 0, sizeof(batt_stored));
    batt_has_written_once = false;
    batt_last_write_tick  = 0U;

    const batt_flash_slot_t *slot = (const batt_flash_slot_t *)BATT_FLASH_BASE;

    if (!batt_slot_integrity_ok(slot))
        return;

    BatteryLimits_t cand;
    cand.warning_cv  = slot->warning_cv;
    cand.limit_cv    = slot->limit_cv;
    cand.cutoff_cv   = slot->cutoff_cv;
    cand.recovery_cv = slot->recovery_cv;
    cand.filter_ms   = slot->filter_ms;

    if (!BatteryLimitsStore_Validate(&cand))
        return;

    batt_flash_valid = true;
    batt_stored      = cand;
}

bool BatteryLimitsStore_IsValid(void)
{
    return batt_flash_valid;
}

void BatteryLimitsStore_GetStored(BatteryLimits_t *out)
{
    if (out) *out = batt_stored;
}

bool BatteryLimitsStore_Save(const BatteryLimits_t *b)
{
    if (!b) return false;

    if (!BatteryLimitsStore_ServiceWriteAllowed())
        return false;

    if (!BatteryLimitsStore_Validate(b))
        return false;

    if (batt_flash_valid &&
        batt_stored.warning_cv  == b->warning_cv  &&
        batt_stored.limit_cv    == b->limit_cv    &&
        batt_stored.cutoff_cv   == b->cutoff_cv   &&
        batt_stored.recovery_cv == b->recovery_cv &&
        batt_stored.filter_ms   == b->filter_ms) {
        return true;
    }

    uint32_t now = HAL_GetTick();
    if (batt_has_written_once &&
        (uint32_t)(now - batt_last_write_tick) < BATT_WRITE_MIN_INTERVAL_MS) {
        return false;
    }

    _Alignas(8) batt_flash_slot_t slot;
    memset(&slot, 0, sizeof(slot));
    slot.magic         = BATT_MAGIC;
    slot.warning_cv    = b->warning_cv;
    slot.limit_cv      = b->limit_cv;
    slot.cutoff_cv     = b->cutoff_cv;
    slot.recovery_cv   = b->recovery_cv;
    slot.filter_ms     = b->filter_ms;
    slot.validity_flag = BATT_VALID_FLAG;
    slot.checksum      = batt_crc32(&slot,
                                    offsetof(batt_flash_slot_t, checksum));

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
        return false;

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = BATT_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    uint32_t slot_size   = sizeof(batt_flash_slot_t);
    uint32_t dword_count = (slot_size + 7U) / 8U;
    const uint64_t *src  = (const uint64_t *)&slot;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   BATT_FLASH_BASE + (i * 8U), src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    batt_flash_valid      = true;
    batt_stored           = *b;
    batt_has_written_once = true;
    batt_last_write_tick  = HAL_GetTick();
    return true;
}
