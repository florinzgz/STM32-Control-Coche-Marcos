/**
  ****************************************************************************
  * @file    error_log.c
  * @brief   Persistent error log — flash NVM ring buffer implementation
  *
  * Stores error log entries in flash page 125 (0x0807D000, 4 KB).
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
  * Safety (power-loss):
  *   - Entries are programmed FIRST, header LAST (commit-pointer
  *     pattern).  Until the header magic+CRC are committed to flash,
  *     errlog_header_valid() rejects the slot on the next boot and
  *     the log is silently reformatted.  This prevents partial writes
  *     from being read back as a valid header pointing at 0xFF-filled
  *     "ghost" entries.
  *   - Losing the in-progress record is acceptable: the underlying
  *     fault will be re-asserted and re-recorded after boot.
  ****************************************************************************
  */

#include "error_log.h"
#include "stm32g4xx_hal.h"
#ifndef HOST_TEST
#include "safety_system.h"
#endif
#include <string.h>
#include <stddef.h>

/* ---- Flash layout ----
 * STM32G474RE: 512 KB flash, 128 pages of 4 KB each (single-bank).
 * Page 125 starts at 0x0807D000 (bank 1).
 * Page 126 = steering calibration, page 127 = EPS params.          */
#define ERRLOG_FLASH_PAGE      125U
#define ERRLOG_FLASH_BASE      0x0807D000U

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

/* ---- Flash-wear guard ---------------------------------------------------
 * Minimum interval between consecutive flash writes.  RAM ring buffer
 * is always updated; only the persistence call is deferred.  After the
 * cool-down window passes the next ErrorLog_Record() call flushes the
 * full RAM image (header + all accumulated entries) to flash, so no
 * entry is lost as long as the system keeps running.  A power loss
 * inside the cool-down loses the un-flushed entries — that is the
 * same trade-off documented in the file header (the underlying fault
 * is re-asserted and re-recorded after the reboot).
 *
 * Sized below the 100 Hz main-loop tick (10 ms) × ~10, so a fault
 * flap of order kHz cannot wear the page.                                */
#define ERRLOG_WRITE_MIN_INTERVAL_MS  100U
static uint32_t log_last_flash_tick   = 0;
static bool     log_has_flushed_once  = false;

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

/* ---- Write the entire log (entries first, header last) to flash ----
 *
 * Power-loss safety:
 *   The header is written LAST so it acts as a commit pointer.  After
 *   erase, every byte in page 125 reads as 0xFF; if power is lost
 *   while writing the entries the header bytes remain 0xFF, the magic
 *   word does not match ERRLOG_MAGIC on the next boot, and
 *   errlog_header_valid() rejects the slot — the log is silently
 *   reformatted (entries lost) instead of being read back as a mix of
 *   real entries and 0xFF garbage.
 *
 *   Writing the header first (the previous order) was unsafe: a
 *   partial entry write would still leave a *valid* header on flash
 *   whose entry_count was larger than the number of entries actually
 *   committed, causing ErrorLog_GetEntry() to return 0xFF-filled
 *   pseudo-entries on the next boot.                                  */
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

    /* 1) Write entries FIRST (entry_count × 16 bytes, immediately after
     *    the reserved header slot).                                    */
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

    /* 2) Write header LAST (16 bytes = 2 double-words).  This is the
     *    commit point: until the header magic + CRC are on flash the
     *    slot is invisible to errlog_header_valid() on the next boot. */
    const uint64_t *hdr_src = (const uint64_t *)&log_header;
    for (uint32_t i = 0; i < (sizeof(errlog_flash_header_t) / 8U); i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   ERRLOG_FLASH_BASE + (i * 8U), hdr_src[i]);
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
    /* Reset the rate-limit bookkeeping on every init so a re-init
     * (or a fresh boot after a reformat) starts with the cool-down
     * window collapsed — the first ErrorLog_Record() call will
     * always reach flash, which is mandatory after a reformat to
     * commit a valid header.                                      */
    log_last_flash_tick  = 0;
    log_has_flushed_once = false;

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
    if (log_header.total_events < UINT32_MAX) {
        log_header.total_events++;
    }

    /* Auto-save to flash, rate-limited to protect page 125 from wear.
     * The RAM ring buffer was already updated above, so an elided
     * write is recovered on the next call past the cool-down window
     * (which re-flushes the complete RAM image including this entry).
     *
     * Wraparound notes:
     *   - The very first call (log_has_flushed_once == false) is
     *     forced through the gate, so the initial value of
     *     log_last_flash_tick is irrelevant on the first call.
     *   - For all subsequent calls the unsigned 32-bit subtraction
     *     (now - log_last_flash_tick) is intentionally modular: it
     *     stays correct across the HAL_GetTick() 32-bit wrap (~49.7
     *     days) as long as the cool-down interval is far below 2^31
     *     ms, which it trivially is at 100 ms.                        */
    uint32_t now = HAL_GetTick();
    if (!log_has_flushed_once ||
        (uint32_t)(now - log_last_flash_tick) >= ERRLOG_WRITE_MIN_INTERVAL_MS) {
        if (errlog_write_flash()) {
            log_last_flash_tick  = now;
            log_has_flushed_once = true;
        }
    }
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
    /* ----------------------------------------------------------------------
     * Safety guard — STANDBY-only.
     *
     * Clearing the error log triggers a full page erase (~22 ms CPU
     * stall) and a re-program of the header.  Doing that while the
     * vehicle is driving would lose two 100 Hz control iterations
     * and is never a legitimate user action — the engineering UI
     * always asks the user to stop the vehicle before issuing a
     * factory-reset / clear-log command.
     *
     * This gate is pure defense-in-depth: a stray, replayed or
     * spoofed factory-reset 0x10/0xFE frame cannot stall the control
     * loop mid-drive.  Refusing both the RAM clear and the flash
     * write keeps the operation atomic: either everything happens
     * (in STANDBY) or nothing does (otherwise) — preserving evidence
     * in the in-RAM ring buffer for the technician.
     *
     * Consistent with the other NVM-write entry points.  The CAN
     * dispatcher path is updated to send ACK_OK on success and
     * ACK_REJECTED on this guard or on a flash error.
     *
     * Compiled out in host tests (test_error_log.c) which exercise
     * a self-contained simulator and link the real error_log.c
     * without the Safety subsystem.                                 */
#ifndef HOST_TEST
    if (Safety_GetState() != SYS_STATE_STANDBY)
        return false;
#endif

    log_header.magic       = ERRLOG_MAGIC;
    log_header.entry_count = 0;
    log_header.write_index = 0;
    /* Preserve total_events as lifetime counter */
    memset(log_entries, 0, sizeof(log_entries));

    /* User-initiated clear bypasses the Record() rate-limit (it is
     * the whole purpose of the call), but we still record the
     * resulting flush timestamp so a subsequent ErrorLog_Record()
     * respects the cool-down window relative to *this* write.       */
    bool ok = errlog_write_flash();
    if (ok) {
        log_last_flash_tick  = HAL_GetTick();
        log_has_flushed_once = true;
    }
    return ok;
}

const error_log_entry_t *ErrorLog_GetEntries(void)
{
    return log_entries;
}

void ErrorLog_SetResetCause(uint8_t cause)
{
    log_reset_cause = cause;
}

uint32_t ErrorLog_GetTotalEvents(void)
{
    return log_header.total_events;
}
