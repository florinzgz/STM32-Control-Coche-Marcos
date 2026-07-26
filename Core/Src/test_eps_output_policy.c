#include <math.h>
#include <stdio.h>
#include "eps_output_policy.h"

static int failed;
#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failed++; \
    } \
} while (0)
#define NEAR(a,b) (fabsf((a) - (b)) < 0.001f)

int main(void)
{
    EpsOutputDecision_t d;

    d = EpsOutput_Resolve(0.5f, 3.0f, 8.0f);
    CHECK(d.coast && NEAR(d.pwm_pct, 0.0f));

    d = EpsOutput_Resolve(2.99f, 3.0f, 8.0f);
    CHECK(d.coast && NEAR(d.pwm_pct, 0.0f));

    d = EpsOutput_Resolve(3.0f, 3.0f, 8.0f);
    CHECK(!d.coast && NEAR(d.pwm_pct, 8.0f));

    d = EpsOutput_Resolve(-5.0f, 3.0f, 8.0f);
    CHECK(!d.coast && NEAR(d.pwm_pct, -8.0f));

    d = EpsOutput_Resolve(12.0f, 3.0f, 8.0f);
    CHECK(!d.coast && NEAR(d.pwm_pct, 12.0f));

    d = EpsOutput_Resolve(NAN, 3.0f, 8.0f);
    CHECK(d.coast && NEAR(d.pwm_pct, 0.0f));

    if (failed != 0) return 1;
    puts("EPS output policy tests: PASS");
    return 0;
}
