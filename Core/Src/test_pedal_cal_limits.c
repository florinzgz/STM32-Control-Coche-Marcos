#ifdef HOST_TEST
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pedal_cal_store.h"

static int tests_run = 0;
static int tests_failed = 0;

#define CHECK_TRUE(expr) do { \
    tests_run++; \
    if (!(expr)) { \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #expr); \
        tests_failed++; \
    } \
} while (0)

#define CHECK_FALSE(expr) CHECK_TRUE(!(expr))

static bool pedalcal_validate_mirror(uint16_t adc_min, uint16_t adc_max)
{
    if (adc_min < PEDAL_CAL_MIN_LIMIT) return false;
    if (adc_max > PEDAL_CAL_MAX_LIMIT) return false;
    if (adc_max <= adc_min)            return false;
    if ((uint32_t)(adc_max - adc_min) < PEDAL_CAL_RANGE_MIN) return false;
    return true;
}

int main(void)
{
    CHECK_TRUE(pedalcal_validate_mirror(0U, 4000U));
    CHECK_FALSE(pedalcal_validate_mirror(0U, 799U));
    CHECK_TRUE(pedalcal_validate_mirror(0U, 4094U));
    CHECK_FALSE(pedalcal_validate_mirror(0U, 4095U));
    CHECK_TRUE(pedalcal_validate_mirror(50U, 4000U));
    CHECK_TRUE(pedalcal_validate_mirror(PEDAL_CAL_DEFAULT_MIN, PEDAL_CAL_DEFAULT_MAX));

    printf("=== pedal_cal_limits: %d run, %d failed ===\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
#endif
