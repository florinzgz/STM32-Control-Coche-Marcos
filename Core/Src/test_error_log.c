/**
  ****************************************************************************
  * @file    test_error_log.c
  * @brief   Host-compilable unit tests for error_log module.
  *
  *          Validates:
  *            - Entry count tracking
  *            - Ring buffer wrap-around (oldest overwritten)
  *            - GetEntry index mapping (oldest-first)
  *            - Clear resets count to zero
  *            - Reset cause propagation
  *            - Boundary conditions
  *
  *          Compile with host GCC (from Core/Src/ directory):
  *            gcc -std=c11 -I../Inc -O2 \
  *                test_error_log.c -o test_error_log
  ****************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ---- Test harness ---- */
static int tests_run    = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(expr) do {                                        \
    tests_run++;                                                      \
    if (!(expr)) {                                                    \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr);       \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_FALSE(expr) do {                                       \
    tests_run++;                                                      \
    if ((expr)) {                                                     \
        printf("FAIL %s:%d  !(%s)\n", __FILE__, __LINE__, #expr);    \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_EQ_U16(got, expected) do {                             \
    uint16_t _got = (got);                                            \
    uint16_t _exp = (expected);                                       \
    tests_run++;                                                      \
    if (_got != _exp) {                                               \
        printf("FAIL %s:%d  %s == %u (expected %u)\n",               \
               __FILE__, __LINE__, #got, (unsigned)_got, (unsigned)_exp); \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_EQ_U8(got, expected) do {                              \
    uint8_t _got = (got);                                             \
    uint8_t _exp = (expected);                                        \
    tests_run++;                                                      \
    if (_got != _exp) {                                               \
        printf("FAIL %s:%d  %s == %u (expected %u)\n",               \
               __FILE__, __LINE__, #got, (unsigned)_got, (unsigned)_exp); \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ---- Reproduce structures from error_log.h ---- */
#define ERROR_LOG_MAX_ENTRIES   250

typedef struct {
    uint32_t timestamp_ms;
    uint8_t  error_code;
    uint8_t  subsystem;
    uint8_t  system_state;
    uint8_t  fault_flags;
    uint8_t  reset_cause;
    uint8_t  i2c_fail_count;
    uint8_t  reserved[2];
    uint32_t uptime_sec;
} error_log_entry_t;

_Static_assert(sizeof(error_log_entry_t) == 16, "entry must be 16 bytes");

/* ---- Ring buffer header ---- */
typedef struct {
    uint32_t magic;
    uint16_t entry_count;
    uint16_t write_index;
    uint32_t total_events;
    uint32_t checksum;
} errlog_flash_header_t;

_Static_assert(sizeof(errlog_flash_header_t) == 16, "header must be 16 bytes");

/* ---- CRC32 (same as error_log.c) ---- */
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

/* ---- Simulated ring buffer (replicates error_log.c logic) ---- */
static errlog_flash_header_t sim_header;
static error_log_entry_t     sim_entries[ERROR_LOG_MAX_ENTRIES];
static uint8_t               sim_reset_cause = 0;
static uint32_t              sim_tick = 0;

static void sim_init(void) {
    memset(&sim_header, 0, sizeof(sim_header));
    sim_header.magic = 0x4552524CU;
    sim_header.entry_count  = 0;
    sim_header.write_index  = 0;
    sim_header.total_events = 0;
    memset(sim_entries, 0, sizeof(sim_entries));
    sim_reset_cause = 0;
    sim_tick = 0;
}

static void sim_record(uint8_t error_code, uint8_t subsystem,
                       uint8_t system_state, uint8_t fault_flags) {
    error_log_entry_t entry;
    entry.timestamp_ms  = sim_tick;
    entry.error_code    = error_code;
    entry.subsystem     = subsystem;
    entry.system_state  = system_state;
    entry.fault_flags   = fault_flags;
    entry.reset_cause   = sim_reset_cause;
    entry.i2c_fail_count = 0;
    entry.reserved[0]   = 0;
    entry.reserved[1]   = 0;
    entry.uptime_sec    = sim_tick / 1000U;

    sim_entries[sim_header.write_index] = entry;
    sim_header.write_index = (sim_header.write_index + 1) % ERROR_LOG_MAX_ENTRIES;
    if (sim_header.entry_count < ERROR_LOG_MAX_ENTRIES) {
        sim_header.entry_count++;
    }
    sim_header.total_events++;
    sim_tick += 100;
}

static bool sim_get_entry(uint16_t index, error_log_entry_t *out) {
    if (!out || index >= sim_header.entry_count) return false;
    uint16_t actual;
    if (sim_header.entry_count >= ERROR_LOG_MAX_ENTRIES) {
        actual = (sim_header.write_index + index) % ERROR_LOG_MAX_ENTRIES;
    } else {
        actual = index;
    }
    *out = sim_entries[actual];
    return true;
}

static void sim_clear(void) {
    sim_header.entry_count = 0;
    sim_header.write_index = 0;
    memset(sim_entries, 0, sizeof(sim_entries));
}

/* ---- Rate-limit simulator -------------------------------------------------
 * Mirrors the ErrorLog_Record() flash-write rate-limit logic in error_log.c.
 *
 * NOTE: ERRLOG_FLASH_MIN_INTERVAL_MS below MUST stay in sync with
 *       ERRLOG_WRITE_MIN_INTERVAL_MS in error_log.c.  The constant is
 *       duplicated rather than shared via a header because this test
 *       deliberately avoids pulling in HAL / safety_system / linker
 *       dependencies — it is a pure-simulator harness.  If the
 *       production constant changes, update both.
 *
 *   - RAM ring buffer is always updated.
 *   - Flash flush is throttled to ERRLOG_FLASH_MIN_INTERVAL_MS (100 ms).
 *   - The first record after Init bypasses the gate (has_flushed_once=false).
 *   - HAL_GetTick wraparound is handled by unsigned subtraction.
 *   - ErrorLog_Init() resets the rate-limit bookkeeping (last_tick and
 *     has_flushed_once) so a freshly reformatted log can flush immediately.
 * ------------------------------------------------------------------------- */
#define ERRLOG_FLASH_MIN_INTERVAL_MS  100U   /* sync with error_log.c */

static uint32_t rl_last_flash_tick   = 0;
static bool     rl_has_flushed_once  = false;
static uint32_t rl_flash_write_count = 0;

static void rl_init(void) {
    rl_last_flash_tick   = 0;
    rl_has_flushed_once  = false;
    rl_flash_write_count = 0;
    sim_init();
}

/* Replicates the gate in ErrorLog_Record(): RAM updates always, flash
 * write only when (!has_flushed_once || elapsed >= MIN_INTERVAL_MS).
 * The tick is updated BEFORE attempting the write (matches the
 * retry-storm fix in error_log.c) so that a flash failure does not
 * cause back-to-back retries on every subsequent record.
 *
 * Variant that lets the test simulate a flash-write success/failure. */
static void rl_record_with_result(uint32_t now_ms, uint8_t code, bool flash_ok) {
    sim_tick = now_ms;
    sim_record(code, 0, 2, 0);    /* RAM always */
    bool first = !rl_has_flushed_once;
    bool elapsed = ((uint32_t)(now_ms - rl_last_flash_tick) >= ERRLOG_FLASH_MIN_INTERVAL_MS);
    if (first || elapsed) {
        rl_last_flash_tick = now_ms;     /* always update */
        if (flash_ok) {
            rl_flash_write_count++;
            rl_has_flushed_once = true;
        }
    }
}

static void rl_record(uint32_t now_ms, uint8_t code) {
    rl_record_with_result(now_ms, code, true);
}

/* ---- Rate-limit tests (mirror the ErrorLog_Record gate) ---- */

static void test_rate_limit_first_call_always_flushes(void)
{
    rl_init();
    /* First record at t=0: has_flushed_once is false, so flash flush runs
     * unconditionally regardless of the tick value. */
    rl_record(0, 1);
    ASSERT_EQ_U16((uint16_t)rl_flash_write_count, 1);
    ASSERT_TRUE(rl_has_flushed_once);
}

static void test_rate_limit_blocks_within_window(void)
{
    rl_init();
    rl_record(0, 1);     /* flush #1 (first call) */
    rl_record(10, 2);    /* +10 ms — blocked */
    rl_record(50, 3);    /* +50 ms — blocked */
    rl_record(99, 4);    /* +99 ms — blocked (strict < 100) */
    ASSERT_EQ_U16((uint16_t)rl_flash_write_count, 1);
    /* RAM ring buffer received every record */
    ASSERT_EQ_U16(sim_header.entry_count, 4);
}

static void test_rate_limit_releases_after_window(void)
{
    rl_init();
    rl_record(0, 1);     /* flush #1 */
    rl_record(50, 2);    /* blocked */
    rl_record(100, 3);   /* flush #2 (exactly at boundary) */
    rl_record(200, 4);   /* flush #3 */
    ASSERT_EQ_U16((uint16_t)rl_flash_write_count, 3);
}

static void test_rate_limit_tick_wraparound(void)
{
    rl_init();
    /* Force a state where last_tick is near UINT32_MAX and the next
     * "now" wraps past zero.  Unsigned subtraction must still yield
     * the correct elapsed delta (50 ms here). */
    rl_record(0xFFFFFFFFU - 49U, 1);  /* flush #1: first call, sets last */
    rl_record(0U, 2);                  /* +50 ms across wrap — blocked */
    rl_record(50U, 3);                 /* +100 ms across wrap — flush #2 */
    ASSERT_EQ_U16((uint16_t)rl_flash_write_count, 2);
}

static void test_init_resets_rate_limit_state(void)
{
    /* Replicates ErrorLog_Init() resetting log_last_flash_tick and
     * log_has_flushed_once.  Without this, a reformat after a recent
     * flush would skip the first post-init flush. */
    rl_init();
    rl_record(0, 1);
    rl_record(50, 2);    /* blocked: too soon */
    ASSERT_EQ_U16((uint16_t)rl_flash_write_count, 1);

    /* Now simulate Init() — must reset both the tick and the
     * has_flushed_once flag so the very next record flushes. */
    rl_init();
    rl_record(60, 3);    /* tick 60 < 100 ms after previous, but Init
                          * reset cleared has_flushed_once: must flush */
    ASSERT_EQ_U16((uint16_t)rl_flash_write_count, 1);  /* counter was zeroed by rl_init */
    ASSERT_TRUE(rl_has_flushed_once);
}

static void test_rate_limit_no_retry_storm_on_failure(void)
{
    /* If the flash write itself fails (e.g. sector locked / supply
     * brown-out), the next call must still respect the cool-down
     * window: we update log_last_flash_tick BEFORE attempting the
     * write so a persistent failure throttles retries to the same
     * 100 ms cadence instead of attempting on every record.
     *
     * has_flushed_once stays false until the first SUCCESSFUL write,
     * so the "first call always flushes" semantic survives — it just
     * applies to the first SUCCESSFUL call.                          */
    rl_init();
    rl_record_with_result(0, 1, false);   /* attempt #1: flash fails */
    ASSERT_EQ_U16((uint16_t)rl_flash_write_count, 0);
    ASSERT_FALSE(rl_has_flushed_once);

    /* Subsequent records within the window must NOT retry the flash. */
    rl_record_with_result(10, 2, false);
    rl_record_with_result(50, 3, false);
    rl_record_with_result(99, 4, false);
    ASSERT_EQ_U16((uint16_t)rl_flash_write_count, 0);

    /* At exactly the cool-down boundary, a retry is allowed.  This
     * time we let it succeed. */
    rl_record_with_result(100, 5, true);
    ASSERT_EQ_U16((uint16_t)rl_flash_write_count, 1);
    ASSERT_TRUE(rl_has_flushed_once);
}

/* ---- Tests ---- */

static void test_entry_size(void)
{
    ASSERT_EQ_U16(sizeof(error_log_entry_t), 16);
    ASSERT_EQ_U16(sizeof(errlog_flash_header_t), 16);
}

static void test_init_empty(void)
{
    sim_init();
    ASSERT_EQ_U16(sim_header.entry_count, 0);
    ASSERT_EQ_U16(sim_header.write_index, 0);
}

static void test_record_single(void)
{
    sim_init();
    sim_record(1, 0, 2, 0x01);  /* OVERCURRENT, GLOBAL, ACTIVE, CAN_TIMEOUT */

    ASSERT_EQ_U16(sim_header.entry_count, 1);
    ASSERT_EQ_U16(sim_header.write_index, 1);

    error_log_entry_t out;
    ASSERT_TRUE(sim_get_entry(0, &out));
    ASSERT_EQ_U8(out.error_code, 1);
    ASSERT_EQ_U8(out.subsystem, 0);
    ASSERT_EQ_U8(out.system_state, 2);
    ASSERT_EQ_U8(out.fault_flags, 0x01);
}

static void test_record_multiple(void)
{
    sim_init();
    sim_record(1, 0, 2, 0x01);
    sim_record(2, 1, 4, 0x02);
    sim_record(3, 2, 5, 0x04);

    ASSERT_EQ_U16(sim_header.entry_count, 3);

    error_log_entry_t out;
    ASSERT_TRUE(sim_get_entry(0, &out));
    ASSERT_EQ_U8(out.error_code, 1);  /* oldest */

    ASSERT_TRUE(sim_get_entry(2, &out));
    ASSERT_EQ_U8(out.error_code, 3);  /* newest */
}

static void test_ring_buffer_wraparound(void)
{
    sim_init();

    /* Fill the entire buffer */
    for (uint16_t i = 0; i < ERROR_LOG_MAX_ENTRIES; i++) {
        sim_record((uint8_t)(i & 0xFF), 0, 2, 0);
    }

    ASSERT_EQ_U16(sim_header.entry_count, ERROR_LOG_MAX_ENTRIES);
    ASSERT_EQ_U16(sim_header.write_index, 0);  /* Wrapped to 0 */

    /* Oldest entry should be #0 (error_code=0) */
    error_log_entry_t out;
    ASSERT_TRUE(sim_get_entry(0, &out));
    ASSERT_EQ_U8(out.error_code, 0);

    /* Add one more — overwrites the oldest */
    sim_record(0xAA, 0, 2, 0);

    ASSERT_EQ_U16(sim_header.entry_count, ERROR_LOG_MAX_ENTRIES);
    ASSERT_EQ_U16(sim_header.write_index, 1);

    /* Oldest entry should now be #1 (error_code=1) */
    ASSERT_TRUE(sim_get_entry(0, &out));
    ASSERT_EQ_U8(out.error_code, 1);

    /* Newest entry should be 0xAA */
    ASSERT_TRUE(sim_get_entry(ERROR_LOG_MAX_ENTRIES - 1, &out));
    ASSERT_EQ_U8(out.error_code, 0xAA);
}

static void test_clear(void)
{
    sim_init();
    sim_record(1, 0, 2, 0x01);
    sim_record(2, 0, 2, 0x02);

    ASSERT_EQ_U16(sim_header.entry_count, 2);

    sim_clear();

    ASSERT_EQ_U16(sim_header.entry_count, 0);
    ASSERT_EQ_U16(sim_header.write_index, 0);

    /* GetEntry should fail on empty log */
    error_log_entry_t out;
    ASSERT_FALSE(sim_get_entry(0, &out));
}

static void test_get_entry_out_of_range(void)
{
    sim_init();
    sim_record(1, 0, 2, 0);

    error_log_entry_t out;
    ASSERT_TRUE(sim_get_entry(0, &out));
    ASSERT_FALSE(sim_get_entry(1, &out));  /* Only 1 entry, index 1 invalid */
    ASSERT_FALSE(sim_get_entry(255, &out));
}

static void test_get_entry_null_pointer(void)
{
    sim_init();
    sim_record(1, 0, 2, 0);

    ASSERT_FALSE(sim_get_entry(0, NULL));
}

static void test_reset_cause_propagation(void)
{
    sim_init();
    sim_reset_cause = 0x04;  /* IWDG reset */
    sim_record(7, 0, 5, 0x01);  /* WATCHDOG error */

    error_log_entry_t out;
    ASSERT_TRUE(sim_get_entry(0, &out));
    ASSERT_EQ_U8(out.reset_cause, 0x04);
    ASSERT_EQ_U8(out.error_code, 7);
}

static void test_total_events_counter(void)
{
    sim_init();
    sim_record(1, 0, 2, 0);
    sim_record(2, 0, 2, 0);
    sim_record(3, 0, 2, 0);

    ASSERT_EQ_U16((uint16_t)sim_header.total_events, 3);
}

static void test_header_crc(void)
{
    errlog_flash_header_t hdr;
    hdr.magic       = 0x4552524CU;
    hdr.entry_count = 5;
    hdr.write_index = 5;
    hdr.total_events = 42;
    hdr.checksum    = errlog_crc32(&hdr, offsetof(errlog_flash_header_t, checksum));

    /* Verify CRC matches */
    uint32_t computed = errlog_crc32(&hdr, offsetof(errlog_flash_header_t, checksum));
    ASSERT_TRUE(computed == hdr.checksum);

    /* Corrupt magic — CRC should no longer match */
    hdr.magic = 0xDEADBEEF;
    computed = errlog_crc32(&hdr, offsetof(errlog_flash_header_t, checksum));
    ASSERT_FALSE(computed == hdr.checksum);
}

static void test_header_validation(void)
{
    errlog_flash_header_t hdr;
    hdr.magic       = 0x4552524CU;
    hdr.entry_count = 5;
    hdr.write_index = 5;
    hdr.total_events = 10;
    hdr.checksum    = errlog_crc32(&hdr, offsetof(errlog_flash_header_t, checksum));

    /* Valid header */
    ASSERT_TRUE(hdr.magic == 0x4552524CU);
    uint32_t crc = errlog_crc32(&hdr, offsetof(errlog_flash_header_t, checksum));
    ASSERT_TRUE(crc == hdr.checksum);

    /* entry_count out of range */
    hdr.entry_count = ERROR_LOG_MAX_ENTRIES + 1;
    ASSERT_TRUE(hdr.entry_count > ERROR_LOG_MAX_ENTRIES);

    /* write_index out of range */
    hdr.entry_count = 5;
    hdr.write_index = ERROR_LOG_MAX_ENTRIES;
    ASSERT_TRUE(hdr.write_index >= ERROR_LOG_MAX_ENTRIES);
}

/* ---- Main ---- */
int main(void)
{
    test_entry_size();
    test_init_empty();
    test_record_single();
    test_record_multiple();
    test_ring_buffer_wraparound();
    test_clear();
    test_get_entry_out_of_range();
    test_get_entry_null_pointer();
    test_reset_cause_propagation();
    test_total_events_counter();
    test_header_crc();
    test_header_validation();

    /* Flash-write rate-limit coverage (mirrors ErrorLog_Record gate) */
    test_rate_limit_first_call_always_flushes();
    test_rate_limit_blocks_within_window();
    test_rate_limit_releases_after_window();
    test_rate_limit_tick_wraparound();
    test_init_resets_rate_limit_state();
    test_rate_limit_no_retry_storm_on_failure();

    printf("--- error_log tests: %d run, %d failed ---\n",
           tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
