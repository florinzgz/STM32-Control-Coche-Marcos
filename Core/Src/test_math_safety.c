/**
  ****************************************************************************
  * @file    test_math_safety.c
  * @brief   Minimal unit tests for math_safety module.
  *
  *          Compile with host GCC:
  *            gcc -std=c11 -I../Inc -O2 -lm \
  *                math_safety.c test_math_safety.c -o test_math_safety
  *
  *          Edge cases tested:
  *            NaN, Inf, negative, subnormal, bounds inversion,
  *            overflow, exact limits.
  ****************************************************************************
  */

#include "math_safety.h"
#include <stdio.h>
#include <math.h>
#include <float.h>

static int tests_run   = 0;
static int tests_failed = 0;

#define ASSERT_EQ_U16(fn, expected) do {                              \
    uint16_t _got = (fn);                                             \
    uint16_t _exp = (expected);                                       \
    tests_run++;                                                      \
    if (_got != _exp) {                                               \
        printf("FAIL %s:%d  %s == %u (expected %u)\n",               \
               __FILE__, __LINE__, #fn, (unsigned)_got, (unsigned)_exp); \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_EQ_U8(fn, expected) do {                               \
    uint8_t _got = (fn);                                              \
    uint8_t _exp = (expected);                                        \
    tests_run++;                                                      \
    if (_got != _exp) {                                               \
        printf("FAIL %s:%d  %s == %u (expected %u)\n",               \
               __FILE__, __LINE__, #fn, (unsigned)_got, (unsigned)_exp); \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

#define ASSERT_EQ_F(fn, expected) do {                                \
    float _got = (fn);                                                \
    float _exp = (expected);                                          \
    tests_run++;                                                      \
    if (_got != _exp) {                                               \
        printf("FAIL %s:%d  %s == %f (expected %f)\n",               \
               __FILE__, __LINE__, #fn, (double)_got, (double)_exp);  \
        tests_failed++;                                               \
    }                                                                 \
} while (0)

/* ---- sanitize_float -------------------------------------------------- */

static void test_sanitize_float(void)
{
    ASSERT_EQ_F(sanitize_float(1.0f, 0.0f),            1.0f);
    ASSERT_EQ_F(sanitize_float(NAN,  42.0f),           42.0f);
    ASSERT_EQ_F(sanitize_float(INFINITY, 7.0f),         7.0f);
    ASSERT_EQ_F(sanitize_float(-INFINITY, 7.0f),        7.0f);
    ASSERT_EQ_F(sanitize_float(-3.0f, 0.0f),           -3.0f);
    ASSERT_EQ_F(sanitize_float(FLT_MIN, 0.0f),      FLT_MIN);  /* subnormal-ish */
}

/* ---- float_to_u16_clamped -------------------------------------------- */

static void test_float_to_u16_clamped(void)
{
    ASSERT_EQ_U16(float_to_u16_clamped(0.0f),            0U);
    ASSERT_EQ_U16(float_to_u16_clamped(100.0f),        100U);
    ASSERT_EQ_U16(float_to_u16_clamped(65535.0f),    65535U);
    ASSERT_EQ_U16(float_to_u16_clamped(65536.0f),    65535U);   /* overflow */
    ASSERT_EQ_U16(float_to_u16_clamped(1.0e9f),      65535U);
    ASSERT_EQ_U16(float_to_u16_clamped(-1.0f),           0U);   /* negative */
    ASSERT_EQ_U16(float_to_u16_clamped(NAN),             0U);
    ASSERT_EQ_U16(float_to_u16_clamped(INFINITY),        0U);
    ASSERT_EQ_U16(float_to_u16_clamped(-INFINITY),       0U);
    ASSERT_EQ_U16(float_to_u16_clamped(FLT_MIN),        0U);   /* subnormal → truncated to 0 */
}

/* ---- float_scaled_to_u16 --------------------------------------------- */

static void test_float_scaled_to_u16(void)
{
    ASSERT_EQ_U16(float_scaled_to_u16(10.0f, 100.0f),  1000U);  /* normal */
    ASSERT_EQ_U16(float_scaled_to_u16(0.0f, 100.0f),      0U);
    ASSERT_EQ_U16(float_scaled_to_u16(1.0e9f, 1.0f),  65535U);  /* overflow */
    ASSERT_EQ_U16(float_scaled_to_u16(NAN, 1.0f),         0U);
    ASSERT_EQ_U16(float_scaled_to_u16(1.0f, NAN),         0U);
    ASSERT_EQ_U16(float_scaled_to_u16(INFINITY, 1.0f),    0U);
    ASSERT_EQ_U16(float_scaled_to_u16(1.0f, INFINITY),    0U);

    /* Negative scale: product negative → clamped to 0 */
    ASSERT_EQ_U16(float_scaled_to_u16(10.0f, -1.0f),      0U);

    /* Negative v with negative scale: product positive → valid */
    ASSERT_EQ_U16(float_scaled_to_u16(-5.0f, -20.0f),   100U);

    /* Negative v with positive scale: product negative → 0 */
    ASSERT_EQ_U16(float_scaled_to_u16(-5.0f, 20.0f),      0U);

    /* Subnormal inputs */
    ASSERT_EQ_U16(float_scaled_to_u16(FLT_MIN, 1.0f),    0U);
}

/* ---- float_to_u8_clamped --------------------------------------------- */

static void test_float_to_u8_clamped(void)
{
    ASSERT_EQ_U8(float_to_u8_clamped(100.0f, 200.0f),  100U);
    ASSERT_EQ_U8(float_to_u8_clamped(0.0f, 255.0f),      0U);
    ASSERT_EQ_U8(float_to_u8_clamped(255.0f, 255.0f),  255U);  /* exact limit */
    ASSERT_EQ_U8(float_to_u8_clamped(300.0f, 255.0f),  255U);  /* overflow */
    ASSERT_EQ_U8(float_to_u8_clamped(300.0f, 500.0f),  255U);  /* max clamped to 255 */
    ASSERT_EQ_U8(float_to_u8_clamped(-5.0f, 100.0f),     0U);  /* negative v */
    ASSERT_EQ_U8(float_to_u8_clamped(50.0f, -10.0f),     0U);  /* negative max → max=0 */

    /* NaN / Inf in v */
    ASSERT_EQ_U8(float_to_u8_clamped(NAN, 100.0f),       0U);
    ASSERT_EQ_U8(float_to_u8_clamped(INFINITY, 100.0f),  0U);  /* sanitized to 0, then clamped */

    /* NaN / Inf in max → max sanitized to 0, v clamped to 0 */
    ASSERT_EQ_U8(float_to_u8_clamped(50.0f, NAN),        0U);
    ASSERT_EQ_U8(float_to_u8_clamped(50.0f, INFINITY),   0U);

    /* Subnormal v */
    ASSERT_EQ_U8(float_to_u8_clamped(FLT_MIN, 255.0f),  0U);
}

/* ---- clampf ---------------------------------------------------------- */

static void test_clampf(void)
{
    ASSERT_EQ_F(clampf(5.0f, 0.0f, 10.0f),    5.0f);   /* in range */
    ASSERT_EQ_F(clampf(-1.0f, 0.0f, 10.0f),   0.0f);   /* below */
    ASSERT_EQ_F(clampf(20.0f, 0.0f, 10.0f),  10.0f);   /* above */
    ASSERT_EQ_F(clampf(0.0f, 0.0f, 0.0f),     0.0f);   /* degenerate */

    /* NaN / Inf in v → returns lo */
    ASSERT_EQ_F(clampf(NAN, 1.0f, 10.0f),     1.0f);
    ASSERT_EQ_F(clampf(INFINITY, 1.0f, 10.0f), 1.0f);
    ASSERT_EQ_F(clampf(-INFINITY, 1.0f, 10.0f), 1.0f);

    /* NaN / Inf in bounds → sanitized to 0 */
    ASSERT_EQ_F(clampf(5.0f, NAN, 10.0f),     5.0f);   /* lo=0, 5 in [0,10] */
    ASSERT_EQ_F(clampf(5.0f, 0.0f, NAN),      0.0f);   /* hi=0, clamp to [0,0] → 0 */

    /* Inverted bounds → swapped */
    ASSERT_EQ_F(clampf(5.0f, 10.0f, 2.0f),    5.0f);   /* swapped to [2,10] */
    ASSERT_EQ_F(clampf(1.0f, 10.0f, 2.0f),    2.0f);   /* below swapped lo */
    ASSERT_EQ_F(clampf(15.0f, 10.0f, 2.0f),  10.0f);   /* above swapped hi */

    /* Negative bounds */
    ASSERT_EQ_F(clampf(-5.0f, -10.0f, -1.0f), -5.0f);
    ASSERT_EQ_F(clampf(-20.0f, -10.0f, -1.0f), -10.0f);

    /* Subnormal value */
    ASSERT_EQ_F(clampf(FLT_MIN, 0.0f, 1.0f), FLT_MIN);
}

/* ---- main ------------------------------------------------------------ */

int main(void)
{
    test_sanitize_float();
    test_float_to_u16_clamped();
    test_float_scaled_to_u16();
    test_float_to_u8_clamped();
    test_clampf();

    printf("\n--- math_safety tests: %d run, %d failed ---\n",
           tests_run, tests_failed);

    return tests_failed ? 1 : 0;
}
