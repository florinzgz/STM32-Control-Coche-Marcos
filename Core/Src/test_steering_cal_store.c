/**
  ****************************************************************************
  * @file    test_steering_cal_store.c
  * @brief   Minimal host-compilable unit tests for steering_cal_store module.
  *
  *          Tests the CRC32 and slot-validation logic that can run on the
  *          host without HAL or real flash.  The flash-write and GPIO
  *          paths are hardware-dependent and cannot be tested on host.
  *
  *          Compile with host GCC:
  *            gcc -std=c11 -I../Inc -O2 \
  *                test_steering_cal_store.c -o test_steering_cal_store
  *
  *          Edge cases tested:
  *            - Valid slot construction and CRC round-trip
  *            - Bad magic rejection
  *            - Bad CRC rejection
  *            - Bad validity flag rejection
  *            - Tolerance window boundary checks
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

#define ASSERT_EQ_I32(got, expected) do {                             \
    int32_t _got = (got);                                             \
    int32_t _exp = (expected);                                        \
    tests_run++;                                                      \
    if (_got != _exp) {                                               \
        printf("FAIL %s:%d  %s == %d (expected %d)\n",               \
               __FILE__, __LINE__, #got, (int)_got, (int)_exp);      \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ---- Reproduce the CRC32 and slot format from steering_cal_store.c ---- */

#define STCAL_MAGIC       0x53544331U
#define STCAL_VALID_FLAG  0xA5U

#define STEERING_CAL_TOLERANCE_COUNTS 100

typedef struct {
    uint32_t magic;
    int32_t  encoder_count_at_center;
    uint8_t  validity_flag;
    uint8_t  reserved[3];
    uint32_t checksum;
} stcal_flash_slot_t;

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

static bool stcal_slot_valid(const stcal_flash_slot_t *slot)
{
    if (slot->magic != STCAL_MAGIC) return false;
    if (slot->validity_flag != STCAL_VALID_FLAG) return false;
    uint32_t crc = stcal_crc32(slot, offsetof(stcal_flash_slot_t, checksum));
    return (crc == slot->checksum);
}

static void build_valid_slot(stcal_flash_slot_t *slot, int32_t center)
{
    memset(slot, 0, sizeof(*slot));
    slot->magic                   = STCAL_MAGIC;
    slot->encoder_count_at_center = center;
    slot->validity_flag           = STCAL_VALID_FLAG;
    slot->checksum = stcal_crc32(slot, offsetof(stcal_flash_slot_t, checksum));
}

/* ---- Tests ---- */

static void test_valid_slot_roundtrip(void)
{
    stcal_flash_slot_t slot;
    build_valid_slot(&slot, 0);
    ASSERT_TRUE(stcal_slot_valid(&slot));

    build_valid_slot(&slot, 42);
    ASSERT_TRUE(stcal_slot_valid(&slot));

    build_valid_slot(&slot, -1000);
    ASSERT_TRUE(stcal_slot_valid(&slot));
}

static void test_bad_magic_rejected(void)
{
    stcal_flash_slot_t slot;
    build_valid_slot(&slot, 0);
    slot.magic = 0xDEADBEEF;
    ASSERT_FALSE(stcal_slot_valid(&slot));
}

static void test_bad_crc_rejected(void)
{
    stcal_flash_slot_t slot;
    build_valid_slot(&slot, 0);
    slot.checksum ^= 1U;
    ASSERT_FALSE(stcal_slot_valid(&slot));
}

static void test_bad_validity_flag_rejected(void)
{
    stcal_flash_slot_t slot;
    build_valid_slot(&slot, 0);
    /* Corrupt validity flag after CRC was computed → CRC will also
     * mismatch, but the flag check fires first.                     */
    slot.validity_flag = 0x00;
    ASSERT_FALSE(stcal_slot_valid(&slot));
}

static void test_erased_flash_rejected(void)
{
    /* Erased flash is all 0xFF */
    stcal_flash_slot_t slot;
    memset(&slot, 0xFF, sizeof(slot));
    ASSERT_FALSE(stcal_slot_valid(&slot));
}

static void test_zero_flash_rejected(void)
{
    stcal_flash_slot_t slot;
    memset(&slot, 0x00, sizeof(slot));
    ASSERT_FALSE(stcal_slot_valid(&slot));
}

static void test_tolerance_window(void)
{
    int32_t stored_center = 0;
    int32_t current;
    int32_t diff;

    /* Exactly at tolerance → should pass */
    current = STEERING_CAL_TOLERANCE_COUNTS;
    diff = current - stored_center;
    if (diff < 0) diff = -diff;
    ASSERT_TRUE(diff <= STEERING_CAL_TOLERANCE_COUNTS);

    /* One beyond tolerance → should fail */
    current = STEERING_CAL_TOLERANCE_COUNTS + 1;
    diff = current - stored_center;
    if (diff < 0) diff = -diff;
    ASSERT_FALSE(diff <= STEERING_CAL_TOLERANCE_COUNTS);

    /* Negative direction exactly at tolerance */
    current = -STEERING_CAL_TOLERANCE_COUNTS;
    diff = current - stored_center;
    if (diff < 0) diff = -diff;
    ASSERT_TRUE(diff <= STEERING_CAL_TOLERANCE_COUNTS);

    /* Negative direction one beyond */
    current = -(STEERING_CAL_TOLERANCE_COUNTS + 1);
    diff = current - stored_center;
    if (diff < 0) diff = -diff;
    ASSERT_FALSE(diff <= STEERING_CAL_TOLERANCE_COUNTS);

    /* Exact match */
    current = 0;
    diff = current - stored_center;
    if (diff < 0) diff = -diff;
    ASSERT_TRUE(diff <= STEERING_CAL_TOLERANCE_COUNTS);
}

static void test_nonzero_stored_center(void)
{
    stcal_flash_slot_t slot;
    build_valid_slot(&slot, 500);
    ASSERT_TRUE(stcal_slot_valid(&slot));
    ASSERT_EQ_I32(slot.encoder_count_at_center, 500);
}

/* ---- Main ---- */

int main(void)
{
    test_valid_slot_roundtrip();
    test_bad_magic_rejected();
    test_bad_crc_rejected();
    test_bad_validity_flag_rejected();
    test_erased_flash_rejected();
    test_zero_flash_rejected();
    test_tolerance_window();
    test_nonzero_stored_center();

    printf("\n%d tests run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}
