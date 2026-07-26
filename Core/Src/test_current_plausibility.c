#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "current_plausibility.h"

static int failed;
#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failed++; \
    } \
} while (0)

int main(void)
{
    CHECK(!CurrentPlausibility_IsFault(false, -0.5f, 50.0f));
    CHECK(!CurrentPlausibility_IsFault(false, -25.0f, 50.0f));
    CHECK(!CurrentPlausibility_IsFault(false,  50.0f, 50.0f));
    CHECK( CurrentPlausibility_IsFault(false, -50.1f, 50.0f));
    CHECK( CurrentPlausibility_IsFault(false,  50.1f, 50.0f));

    CHECK(!CurrentPlausibility_IsFault(true, -0.9f, 100.0f));
    CHECK( CurrentPlausibility_IsFault(true, -1.1f, 100.0f));
    CHECK( CurrentPlausibility_IsFault(true, 100.1f, 100.0f));

    CHECK(CurrentPlausibility_IsFault(false, NAN, 50.0f));
    CHECK(CurrentPlausibility_IsFault(false, INFINITY, 50.0f));
    CHECK(CurrentPlausibility_IsFault(false, 1.0f, 0.0f));

    if (failed != 0) return 1;
    puts("current plausibility tests: PASS");
    return 0;
}
