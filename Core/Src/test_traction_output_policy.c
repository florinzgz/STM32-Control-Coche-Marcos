#ifdef HOST_TEST
#include <stdio.h>
#include <math.h>
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

static void check_all_coast(const TractionOutputPlan *p)
{
    for (uint8_t i = 0U; i < 4U; ++i) {
        CHECK(p->mode[i] == TRACTION_OUTPUT_MODE_COAST);
        CHECK(p->direction[i] == 0);
        CHECK(p->pwm[i] == 0U);
    }
}

static void test_helpers(void)
{
    const uint16_t unequal[4] = {1200U, 1000U, 900U, 1100U};
    const int8_t same_dir[4] = {1, 1, 1, 1};
    const int8_t mixed_dir[4] = {1, 1, -1, 1};
    const float unity[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float limited[4] = {1.0f, 1.0f, 0.8f, 1.0f};

    CHECK(TractionOutput_MinPwm(unequal) == 900U);
    CHECK(TractionOutput_SameNonzeroDirection(same_dir));
    CHECK(!TractionOutput_SameNonzeroDirection(mixed_dir));
    CHECK(TractionOutput_CapPwmByScale(1000U, 0.0f, 4249U) == 0U);
    CHECK(TractionOutput_CapPwmByScale(1000U, 0.5f, 4249U) == 500U);
    CHECK(TractionOutput_CapPwmByScale(5000U, 1.0f, 4249U) == 4249U);

    CHECK(TractionOutput_StraightUnlimited(0.0f, 2.0f, unity));
    CHECK(TractionOutput_StraightUnlimited(1.999f, 2.0f, unity));
    CHECK(!TractionOutput_StraightUnlimited(2.0f, 2.0f, unity));
    CHECK(!TractionOutput_StraightUnlimited(NAN, 2.0f, unity));
    CHECK(!TractionOutput_StraightUnlimited(INFINITY, 2.0f, unity));
    CHECK(!TractionOutput_StraightUnlimited(0.0f, NAN, unity));
    CHECK(!TractionOutput_StraightUnlimited(0.0f, 0.0f, unity));
    CHECK(!TractionOutput_StraightUnlimited(0.0f, 2.0f, limited));
    limited[2] = NAN;
    CHECK(!TractionOutput_StraightUnlimited(0.0f, 2.0f, limited));
}

static void test_physical_4x4_plan(void)
{
    const uint8_t drive[4] = {
        TRACTION_OUTPUT_MODE_DRIVE, TRACTION_OUTPUT_MODE_DRIVE,
        TRACTION_OUTPUT_MODE_DRIVE, TRACTION_OUTPUT_MODE_DRIVE
    };
    const int8_t forward[4] = {1, 1, 1, 1};
    const uint16_t unequal[4] = {1200U, 1000U, 900U, 1100U};
    const float unity[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    TractionOutputPlan p;

    TractionOutput_Resolve4x4(drive, forward, unequal, 0.0f, 2.0f,
                              unity, &p);
    for (uint8_t i = 0U; i < 4U; ++i) {
        CHECK(p.mode[i] == TRACTION_OUTPUT_MODE_DRIVE);
        CHECK(p.direction[i] == 1);
        CHECK(p.pwm[i] == 900U);  /* minimum, never maximum */
    }

    /* Outside the deadband, Ackermann/per-wheel results pass through. */
    TractionOutput_Resolve4x4(drive, forward, unequal, 2.0f, 2.0f,
                              unity, &p);
    for (uint8_t i = 0U; i < 4U; ++i) CHECK(p.pwm[i] == unequal[i]);

    /* Any individual limiter disables symmetry and preserves the resolved cap. */
    const float limited[4] = {1.0f, 1.0f, 0.8f, 1.0f};
    TractionOutput_Resolve4x4(drive, forward, unequal, 0.0f, 2.0f,
                              limited, &p);
    for (uint8_t i = 0U; i < 4U; ++i) CHECK(p.pwm[i] == unequal[i]);

    /* Non-finite steering data fails closed to no equalisation. */
    TractionOutput_Resolve4x4(drive, forward, unequal, NAN, 2.0f,
                              unity, &p);
    for (uint8_t i = 0U; i < 4U; ++i) CHECK(p.pwm[i] == unequal[i]);
}

static void test_physical_rear_4x2_plan(void)
{
    uint8_t mode[4] = {
        TRACTION_OUTPUT_MODE_DRIVE, TRACTION_OUTPUT_MODE_DRIVE,
        TRACTION_OUTPUT_MODE_COAST, TRACTION_OUTPUT_MODE_COAST
    };
    const int8_t direction[4] = {1, -1, 0, 0};
    const uint16_t pwm[4] = {1200U, 1000U, 0U, 0U};
    const float scales[4] = {1.0f, 1.0f, 0.5f, 1.0f};
    TractionOutputPlan p;

    CHECK(!TractionOutput_Resolve4x2Rear(mode, direction, pwm, scales,
                                         4249U, NULL));

    CHECK(TractionOutput_Resolve4x2Rear(mode, direction, pwm, scales,
                                        4249U, &p));
    CHECK(p.mode[TRACTION_OUTPUT_FL] == TRACTION_OUTPUT_MODE_COAST);
    CHECK(p.mode[TRACTION_OUTPUT_FR] == TRACTION_OUTPUT_MODE_COAST);
    CHECK(p.pwm[TRACTION_OUTPUT_FL] == 0U);
    CHECK(p.pwm[TRACTION_OUTPUT_FR] == 0U);
    CHECK(p.mode[TRACTION_OUTPUT_RL] == TRACTION_OUTPUT_MODE_DRIVE);
    CHECK(p.mode[TRACTION_OUTPUT_RR] == TRACTION_OUTPUT_MODE_DRIVE);
    CHECK(p.direction[TRACTION_OUTPUT_RL] == 1);
    CHECK(p.direction[TRACTION_OUTPUT_RR] == -1);
    CHECK(p.pwm[TRACTION_OUTPUT_RL] == 600U);  /* physical RL scale reapplied */
    CHECK(p.pwm[TRACTION_OUTPUT_RR] == 1000U);

    /* A physical rear cutoff must never be bypassed by the logical FL result. */
    const float rear_cut[4] = {1.0f, 1.0f, 0.0f, 1.0f};
    CHECK(TractionOutput_Resolve4x2Rear(mode, direction, pwm, rear_cut,
                                        4249U, &p));
    CHECK(p.pwm[TRACTION_OUTPUT_RL] == 0U);
    CHECK(p.pwm[TRACTION_OUTPUT_RR] == 1000U);

    /* Brake is routed to the rear axle only. */
    mode[0] = mode[1] = TRACTION_OUTPUT_MODE_BRAKE;
    CHECK(TractionOutput_Resolve4x2Rear(mode, direction, pwm, scales,
                                        4249U, &p));
    CHECK(p.mode[TRACTION_OUTPUT_FL] == TRACTION_OUTPUT_MODE_COAST);
    CHECK(p.mode[TRACTION_OUTPUT_FR] == TRACTION_OUTPUT_MODE_COAST);
    CHECK(p.mode[TRACTION_OUTPUT_RL] == TRACTION_OUTPUT_MODE_BRAKE);
    CHECK(p.mode[TRACTION_OUTPUT_RR] == TRACTION_OUTPUT_MODE_BRAKE);

    /* An asymmetric or unknown base decision fails closed to all-coast. */
    mode[0] = TRACTION_OUTPUT_MODE_DRIVE;
    mode[1] = TRACTION_OUTPUT_MODE_BRAKE;
    CHECK(!TractionOutput_Resolve4x2Rear(mode, direction, pwm, scales,
                                         4249U, &p));
    check_all_coast(&p);

    mode[0] = mode[1] = 99U;
    CHECK(!TractionOutput_Resolve4x2Rear(mode, direction, pwm, scales,
                                         4249U, &p));
    check_all_coast(&p);
}

int main(void)
{
    test_helpers();
    test_physical_4x4_plan();
    test_physical_rear_4x2_plan();

    printf("test_traction_output_policy: %d run, %d failed\n",
           run_count, fail_count);
    return fail_count == 0 ? 0 : 1;
}
#endif
