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
    assert(!DriveDynamics_TcsIsSlipping(5.99f, 20.0f,
        DRIVE_TCS_MIN_REFERENCE_KMH, DRIVE_TCS_SLIP_THRESHOLD_PCT));
    assert(!DriveDynamics_TcsIsSlipping(8.0f, 10.0f,
        DRIVE_TCS_MIN_REFERENCE_KMH, DRIVE_TCS_SLIP_THRESHOLD_PCT));
    assert( DriveDynamics_TcsIsSlipping(8.0f, 10.1f,
        DRIVE_TCS_MIN_REFERENCE_KMH, DRIVE_TCS_SLIP_THRESHOLD_PCT));
    assert(!DriveDynamics_TcsIsSlipping(NAN, 10.0f,
        DRIVE_TCS_MIN_REFERENCE_KMH, DRIVE_TCS_SLIP_THRESHOLD_PCT));
    assert(!DriveDynamics_TcsIsSlipping(8.0f, INFINITY,
        DRIVE_TCS_MIN_REFERENCE_KMH, DRIVE_TCS_SLIP_THRESHOLD_PCT));
    assert(!DriveDynamics_TcsIsSlipping(8.0f, -1.0f,
        DRIVE_TCS_MIN_REFERENCE_KMH, DRIVE_TCS_SLIP_THRESHOLD_PCT));

    /* Initial intervention remains decisive, later growth is time-based. */
    float reduction = DriveDynamics_TcsReductionNext(0.0f, true, 0.01f,
        DRIVE_TCS_INITIAL_REDUCTION, DRIVE_TCS_REDUCTION_RATE_PER_S,
        DRIVE_TCS_RECOVERY_RATE_PER_S, DRIVE_TCS_MAX_REDUCTION);
    assert(nearf(reduction, 0.40f, 0.0001f));
    for (int i = 0; i < 10; ++i) {
        reduction = DriveDynamics_TcsReductionNext(reduction, true, 0.01f,
        DRIVE_TCS_INITIAL_REDUCTION, DRIVE_TCS_REDUCTION_RATE_PER_S,
        DRIVE_TCS_RECOVERY_RATE_PER_S, DRIVE_TCS_MAX_REDUCTION);
    }
    assert(nearf(reduction, 0.50f, 0.0002f));
    for (int i = 0; i < 100; ++i) {
        reduction = DriveDynamics_TcsReductionNext(reduction, true, 0.01f,
        DRIVE_TCS_INITIAL_REDUCTION, DRIVE_TCS_REDUCTION_RATE_PER_S,
        DRIVE_TCS_RECOVERY_RATE_PER_S, DRIVE_TCS_MAX_REDUCTION);
    }
    assert(nearf(reduction, 0.80f, 0.0001f));

    /* Recovery remains smooth: 25 percentage points per second. */
    reduction = DriveDynamics_TcsReductionNext(0.80f, false, 1.0f,
        DRIVE_TCS_INITIAL_REDUCTION, DRIVE_TCS_REDUCTION_RATE_PER_S,
        DRIVE_TCS_RECOVERY_RATE_PER_S, DRIVE_TCS_MAX_REDUCTION);
    assert(nearf(reduction, 0.55f, 0.0001f));

    /* Invalid dt is forced to one 100 Hz control period, never a large jump. */
    reduction = DriveDynamics_TcsReductionNext(0.40f, true, NAN,
        DRIVE_TCS_INITIAL_REDUCTION, DRIVE_TCS_REDUCTION_RATE_PER_S,
        DRIVE_TCS_RECOVERY_RATE_PER_S, DRIVE_TCS_MAX_REDUCTION);
    assert(nearf(reduction, 0.41f, 0.0001f));
    reduction = DriveDynamics_TcsReductionNext(0.40f, true, 2.0f,
        DRIVE_TCS_INITIAL_REDUCTION, DRIVE_TCS_REDUCTION_RATE_PER_S,
        DRIVE_TCS_RECOVERY_RATE_PER_S, DRIVE_TCS_MAX_REDUCTION);
    assert(nearf(reduction, 0.41f, 0.0001f));

    /* BLOQUE 1 (dynbrake_store): motor_control.c's own runtime-tunable
     * factor_pct is now the single source of truth for the brake-effort
     * factor, so this ratio is neutral (1.0) — only the 30 % absolute
     * field-safety backstop still applies here. */
    assert(nearf(DriveDynamics_DynbrakeLimitedPct(20.0f), 20.0f, 0.0001f));
    assert(nearf(DriveDynamics_DynbrakeLimitedPct(30.0f), 30.0f, 0.0001f));
    assert(nearf(DriveDynamics_DynbrakeLimitedPct(60.0f), 30.0f, 0.0001f));
    assert(nearf(DriveDynamics_DynbrakeLimitedPct(100.0f), 30.0f, 0.0001f));
    assert(nearf(DriveDynamics_DynbrakeLimitedPct(NAN), 0.0f, 0.0001f));

    /* EPS intent starts early and reaches full authority by 4 road-wheel °/s. */
    assert(nearf(DriveDynamics_EpsLambda(0.5f), 0.0f, 0.0001f));
    assert(DriveDynamics_EpsLambda(1.0f) > 0.0f);
    assert(DriveDynamics_EpsLambda(2.0f) > DriveDynamics_EpsLambda(1.0f));
    assert(nearf(DriveDynamics_EpsLambda(4.0f), 1.0f, 0.0001f));
    assert(nearf(DriveDynamics_EpsLambda(NAN), 0.0f, 0.0001f));

    puts("drive dynamics policy: PASS");
    return 0;
}
