/**
  ****************************************************************************
  * @file    error_log.h
  * @brief   Persistent error log — flash NVM ring buffer
  *
  * Stores safety errors, faults, and system events in a non-volatile ring
  * buffer on flash page 125 (0x0807C000, 4 KB).  Survives power cycles,
  * watchdog resets, and brownouts.  Enables post-mortem diagnosis.
  *
  * Flash layout:
  *   Page 125 (0x0807C000, 4 KB) — dedicated to error log.
  *   Header:  16 bytes (magic, entry count, write index, CRC32)
  *   Entries: up to ERROR_LOG_MAX_ENTRIES × 16 bytes each
  *
  * API:
  *   ErrorLog_Init()        — load from flash (if valid), or format
  *   ErrorLog_Record()      — append an error entry (auto-saves)
  *   ErrorLog_GetCount()    — number of recorded entries
  *   ErrorLog_GetEntry()    — read entry by index (0 = oldest)
  *   ErrorLog_Clear()       — erase all entries
  *   ErrorLog_GetEntries()  — pointer to internal entry array (read-only)
  *
  * Thread safety:
  *   All functions are called from the main loop only (no ISR access).
  *   No mutex needed on bare-metal single-threaded STM32.
  ****************************************************************************
  */

#ifndef ERROR_LOG_H
#define ERROR_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Maximum number of error log entries.
 * 4096 bytes page - 16 bytes header = 4080 bytes for entries.
 * Each entry is 16 bytes → 255 entries max.
 * Use 250 for alignment headroom.                              */
#define ERROR_LOG_MAX_ENTRIES   250

/* ---- Error log entry ---- */
typedef struct {
    uint32_t timestamp_ms;     /* HAL_GetTick() at time of error              */
    uint8_t  error_code;       /* Safety_Error_t enum value                   */
    uint8_t  subsystem;        /* 0=GLOBAL, 1=MOTOR, 2=SENSOR, 3=CAN         */
    uint8_t  system_state;     /* SystemState_t at time of error              */
    uint8_t  fault_flags;      /* Heartbeat fault flags at time of error      */
    uint8_t  reset_cause;      /* MCU reset cause flags (boot only)           */
    uint8_t  i2c_fail_count;   /* I2C consecutive failure count               */
    uint8_t  reserved[2];      /* Padding for 4-byte alignment                */
    uint32_t uptime_sec;       /* Seconds since last MCU reset                */
} error_log_entry_t;

/* Static assert: entry must be exactly 16 bytes for flash alignment */
_Static_assert(sizeof(error_log_entry_t) == 16,
               "error_log_entry_t must be 16 bytes");

/* ---- Public API ---- */

/**
 * @brief  Initialise the error log module.
 *         Reads flash and validates header CRC.
 *         If invalid or first use, formats the log (zero entries).
 */
void ErrorLog_Init(void);

/**
 * @brief  Record a new error entry.
 *         Appends to the ring buffer and auto-saves to flash.
 *         If the buffer is full, the oldest entry is overwritten.
 *
 * @param  error_code   Safety error code (Safety_Error_t)
 * @param  subsystem    Diagnostic subsystem (0-3)
 * @param  system_state Current system state (SystemState_t)
 * @param  fault_flags  Current heartbeat fault flags
 */
void ErrorLog_Record(uint8_t error_code, uint8_t subsystem,
                     uint8_t system_state, uint8_t fault_flags);

/**
 * @brief  Get the number of recorded entries (0 to ERROR_LOG_MAX_ENTRIES).
 */
uint16_t ErrorLog_GetCount(void);

/**
 * @brief  Read an entry by index (0 = oldest still in buffer).
 * @param  index   Entry index (0 to GetCount()-1)
 * @param  out     Output entry buffer
 * @retval true if valid index, false if out of range
 */
bool ErrorLog_GetEntry(uint16_t index, error_log_entry_t *out);

/**
 * @brief  Clear all log entries and reformat flash.
 * @retval true on success, false on flash error
 */
bool ErrorLog_Clear(void);

/**
 * @brief  Get read-only pointer to the internal entry array.
 *         Entries are in ring-buffer order (use write_index to find oldest).
 * @retval Pointer to entry array (never NULL)
 */
const error_log_entry_t *ErrorLog_GetEntries(void);

/**
 * @brief  Set the reset cause byte for future log entries.
 *         Called once at boot from main.c after reading RCC_CSR.
 * @param  cause  Reset cause bitmask
 */
void ErrorLog_SetResetCause(uint8_t cause);

/**
 * @brief  Get the lifetime total event count.
 *         This counter is monotonic and never reset (survives ErrorLog_Clear).
 * @return Total number of errors ever recorded
 */
uint32_t ErrorLog_GetTotalEvents(void);

#ifdef __cplusplus
}
#endif

#endif /* ERROR_LOG_H */
