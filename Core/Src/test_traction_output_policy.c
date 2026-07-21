#ifdef HOST_TEST
#include <stdio.h>
#include "traction_output_policy.h"

static int run_count;
static int fail_count;

#define CHECK(expr) do { \
    ++run_count; \
    if (!(expr)) { \
        ++fail_count; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
    } \
} while (0)

int main(void)
{
    const uint16_t unequal[4] = {1200U, 1000U, 900U, 1100U};
    const int8_t same_dir[4] = {1, 1, 1, 1};
    const int8_t mixed_dir[4] = {1, 1, -1, 1};

    CHECK(TractionOutput_MinPwm(unequal) == 900U);
    CHECK(TractionOutput_SameNonzeroDirection(same_dir));
    CHECK(!TractionOutput_SameNonzeroDirection(mixed_dir));
    CHECK(TractionOutput_CapPwmByScale(1000U, 0.0f, 4249U) == 0U);
    CHECK(TractionOutput_CapPwmByScale(1000U, 0.5f, 4249U) == 500U);
    CHECK(TractionOutput_CapPwmByScale(5000U, 1.0f, 4249U) == 4249U);

    printf("test_traction_output_policy: %d run, %d failed\n",
           run_count, fail_count);
    return fail_count == 0 ? 0 : 1;
}
#endif
