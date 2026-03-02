/**
  ****************************************************************************
  * @file    error_log.c
  * @brief   Persistent error log — flash NVM ring buffer implementation
  *
  * Stores error log entries in flash page 125 (0x0807C000, 4 KB).
  * Uses a ring buffer with a flash header for integrity checking.
  *
  * Write strategy:
  *   - Full page erase + rewrite on every ErrorLog_Record() call.
  *   - Error events are infrequent (seconds to minutes apart),
  *     so flash endurance (10K cycles) is not a concern.
  *   - Write time: ~5 ms for erase + ~2 ms for writes = ~7 ms total.
  *     This fits within the 10 ms main loop tick budget because
  *     error recording only happens during fault transitions, not
  *     every cycle.
  *
  * Safety:
  *   - If power is lost during write, the header CRC will be invalid
  *     on next boot and the log will be reformatted (entries lost).
  *   - This is acceptable: the error that caused the power loss is
  *     the one we care about, and it will be re-recorded on boot.
  ****************************************************************************
  */

#include "error_log.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stddef.h>

/* ---- Flash layout ----
 * STM32G474RE: 512 KB flash, 128 pages of 4 KB each.
 * Page 125 starts at 0x0807C000 (bank 1).
 * Page 126 = steering calibration, page 127 = EPS params.          */
#define ERRLOG_FLASH_PAGE      125U
#define ERRLOG_FLASH_BASE      0x0807C000U

#define ERRLOG_MAGIC           0x4552524CU   /* "ERRL" */

/* ---- On-flash header (16 bytes, 8-byte aligned) ---- */
typedef struct {
    uint32_t magic;            /* Must equal ERRLOG_MAGIC                */
    uint16_t entry_count;      /* Number of valid entries (0..MAX)       */
    uint16_t write_index;      /* Next write position (ring index)       */
    uint32_t total_events;     /* Lifetime event counter (monotonic)     */
    uint32_t checksum;         /* CRC32 of all fields before this       */
} errlog_flash_header_t;

_Static_assert(sizeof(errlog_flash_header_t) == 16,
               "errlog_flash_header_t must be 16 bytes");

/* ---- RAM state ---- */
static errlog_flash_header_t log_header;
static error_log_entry_t     log_entries[ERROR_LOG_MAX_ENTRIES];
static uint8_t               log_reset_cause = 0;
static bool                  log_initialized = false;

/* ---- CRC32 (same polynomial as steering_cal_store.c / eps_params.c) ---- */
static uint32_t errlog_crc32(const void *data, uint32_t len)
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

/* ---- Validate flash header ---- */
static bool errlog_header_valid(const errlog_flash_header_t *hdr)
{
    if (hdr->magic != ERRLOG_MAGIC) return false;
    if (hdr->entry_count > ERROR_LOG_MAX_ENTRIES) return false;
    if (hdr->write_index >= ERROR_LOG_MAX_ENTRIES) return false;
    uint32_t crc = errlog_crc32(hdr, offsetof(errlog_flash_header_t, checksum));
    return (crc == hdr->checksum);
}

/* ---- Write the entire log (header + entries) to flash ---- */
static bool errlog_write_flash(void)
{
    /* Update header checksum */
    log_header.checksum = errlog_crc32(&log_header,
                                        offsetof(errlog_flash_header_t, checksum));

    /* Unlock flash */
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK) return false;

    /* Erase page 125 */
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = ERRLOG_FLASH_PAGE;
    erase.NbPages   = 1;

    uint32_t page_err = 0;
    status = HAL_FLASHEx_Erase(&erase, &page_err);
    if (status != HAL_OK || page_err != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    /* Write header (16 bytes = 2 double-words) */
    const uint64_t *hdr_src = (const uint64_t *)&log_header;
    for (uint32_t i = 0; i < (sizeof(errlog_flash_header_t) / 8U); i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   ERRLOG_FLASH_BASE + (i * 8U), hdr_src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    /* Write entries (entry_count × 16 bytes, starting after header) */
    uint32_t entries_offset = sizeof(errlog_flash_header_t);
    uint32_t entries_bytes  = (uint32_t)log_header.entry_count * sizeof(error_log_entry_t);
    uint32_t dword_count    = (entries_bytes + 7U) / 8U;
    const uint64_t *ent_src = (const uint64_t *)log_entries;

    for (uint32_t i = 0; i < dword_count; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   ERRLOG_FLASH_BASE + entries_offset + (i * 8U),
                                   ent_src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();
    return true;
}

/* ---- Public API ---- */

void ErrorLog_Init(void)
{
    const errlog_flash_header_t *flash_hdr =
        (const errlog_flash_header_t *)ERRLOG_FLASH_BASE;

    if (errlog_header_valid(flash_hdr)) {
        /* Load header */
        memcpy(&log_header, flash_hdr, sizeof(errlog_flash_header_t));

        /* Load entries */
        const error_log_entry_t *flash_entries =
            (const error_log_entry_t *)(ERRLOG_FLASH_BASE + sizeof(errlog_flash_header_t));
        uint32_t copy_bytes = (uint32_t)log_header.entry_count * sizeof(error_log_entry_t);
        if (copy_bytes > sizeof(log_entries)) {
            copy_bytes = sizeof(log_entries);
        }
        memcpy(log_entries, flash_entries, copy_bytes);
    } else {
        /* Invalid or first use — format the log */
        memset(&log_header, 0, sizeof(log_header));
        log_header.magic = ERRLOG_MAGIC;
        log_header.entry_count  = 0;
        log_header.write_index  = 0;
        log_header.total_events = 0;
        memset(log_entries, 0, sizeof(log_entries));
        /* Don't write to flash here — wait for first error or explicit clear */
    }

    log_initialized = true;
}

void ErrorLog_Record(uint8_t error_code, uint8_t subsystem,
                     uint8_t system_state, uint8_t fault_flags)
{
    if (!log_initialized) return;

    /* Build entry */
    error_log_entry_t entry;
    entry.timestamp_ms  = HAL_GetTick();
    entry.error_code    = error_code;
    entry.subsystem     = subsystem;
    entry.system_state  = system_state;
    entry.fault_flags   = fault_flags;
    entry.reset_cause   = log_reset_cause;
    entry.i2c_fail_count = 0;  /* Caller can set if relevant */
    entry.reserved[0]   = 0;
    entry.reserved[1]   = 0;
    entry.uptime_sec    = HAL_GetTick() / 1000U;

    /* Write to ring buffer */
    log_entries[log_header.write_index] = entry;
    log_header.write_index = (log_header.write_index + 1) % ERROR_LOG_MAX_ENTRIES;
    if (log_header.entry_count < ERROR_LOG_MAX_ENTRIES) {
        log_header.entry_count++;
    }
    log_header.total_events++;

    /* Auto-save to flash */
    errlog_write_flash();
}

uint16_t ErrorLog_GetCount(void)
{
    return log_header.entry_count;
}

bool ErrorLog_GetEntry(uint16_t index, error_log_entry_t *out)
{
    if (!out || index >= log_header.entry_count) return false;

    /* Calculate actual ring buffer position.
     * If buffer is full (entry_count == MAX), oldest entry is at write_index.
     * If buffer is not full, oldest entry is at index 0.                    */
    uint16_t actual;
    if (log_header.entry_count >= ERROR_LOG_MAX_ENTRIES) {
        actual = (log_header.write_index + index) % ERROR_LOG_MAX_ENTRIES;
    } else {
        actual = index;
    }

    *out = log_entries[actual];
    return true;
}

bool ErrorLog_Clear(void)
{
    log_header.magic       = ERRLOG_MAGIC;
    log_header.entry_count = 0;
    log_header.write_index = 0;
    /* Preserve total_events as lifetime counter */
    memset(log_entries, 0, sizeof(log_entries));

    return errlog_write_flash();
}

const error_log_entry_t *ErrorLog_GetEntries(void)
{
    return log_entries;
}

void ErrorLog_SetResetCause(uint8_t cause)
{
    log_reset_cause = cause;
}
