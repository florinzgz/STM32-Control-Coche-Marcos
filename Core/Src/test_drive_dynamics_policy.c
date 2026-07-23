#include "drive_dynamics_policy.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static int nearf(float a, float b, float eps)
{
    return fabsf(a - b) <= eps;
}

int main(void)
{
    /* 6 PPR quantisation: no TCS below 6 km/h and no trip at exactly 25 %. */
    assert(!DriveDynamics_TcsIsSlipping(5.99f, 20.0f));
    assert(!DriveDynamics_TcsIsSlipping(8.0f, 10.0f));
    assert( DriveDynamics_TcsIsSlipping(8.0f, 10.1f));

    /* Initial intervention remains decisive, later growth is time-based. */
    float reduction = DriveDynamics_TcsReductionNext(0.0f, true, 0.01f);
    assert(nearf(reduction, 0.40f, 0.0001f));
    for (int i = 0; i < 10; ++i) {
        reduction = DriveDynamics_TcsReductionNext(reduction, true, 0.01f);
    }
    assert(nearf(reduction, 0.50f, 0.0002f));
    for (int i = 0; i < 100; ++i) {
        reduction = DriveDynamics_TcsReductionNext(reduction, true, 0.01f);
    }
    assert(nearf(reduction, 0.80f, 0.0001f));

    /* Recovery remains smooth: 25 percentage points per second. */
    reduction = DriveDynamics_TcsReductionNext(0.80f, false, 1.0f);
    assert(nearf(reduction, 0.55f, 0.0001f));

    /* Dynamic braking is equivalent to factor 0.2 and capped at 30 %. */
    assert(nearf(DriveDynamics_DynbrakeLimitedPct(50.0f), 20.0f, 0.0001f));
    assert(nearf(DriveDynamics_DynbrakeLimitedPct(60.0f), 24.0f, 0.0001f));
    assert(nearf(DriveDynamics_DynbrakeLimitedPct(100.0f), 30.0f, 0.0001f));

    /* EPS intent starts early and reaches full authority by 4 road-wheel °/s. */
    assert(nearf(DriveDynamics_EpsLambda(0.5f), 0.0f, 0.0001f));
    assert(DriveDynamics_EpsLambda(1.0f) > 0.0f);
    assert(DriveDynamics_EpsLambda(2.0f) > DriveDynamics_EpsLambda(1.0f));
    assert(nearf(DriveDynamics_EpsLambda(4.0f), 1.0f, 0.0001f));

    puts("drive dynamics policy: PASS");
    return 0;
}
