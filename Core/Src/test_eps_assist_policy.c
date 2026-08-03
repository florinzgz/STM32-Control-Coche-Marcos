#include <math.h>
#include <stdio.h>
#include "eps_assist_policy.h"
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
    EpsAssistState_t state = {0};
    EpsAssistDecision_t d;

    /* At parking speed, a large centring request must never brake the driver. */
    d = EpsAssist_Resolve(&state, 0U, 0.0f, 0.0f,
                          0.0f, -12.0f, 3.0f);
    CHECK(d.coast && !d.driver_intent && NEAR(d.pwm_pct, 0.0f));

    /* Slow positive motion becomes positive assist, never opposite return. */
    d = EpsAssist_Resolve(&state, 10U, 0.6f, 0.0f,
                          0.1f, -12.0f, 3.0f);
    CHECK(!d.coast && d.driver_intent && NEAR(d.pwm_pct, 1.25f));

    /* Encoder quantisation gaps keep intent alive briefly, avoiding chatter. */
    d = EpsAssist_Resolve(&state, 100U, 0.0f, 0.0f,
                          0.0f, -12.0f, 3.0f);
    CHECK(!d.coast && d.driver_intent && NEAR(d.pwm_pct, 1.25f));

    d = EpsAssist_Resolve(&state, 131U, 0.0f, 0.0f,
                          0.0f, -12.0f, 3.0f);
    CHECK(d.coast && !d.driver_intent && NEAR(d.pwm_pct, 0.0f));

    /* Negative wheel motion produces only negative assistance. */
    d = EpsAssist_Resolve(&state, 200U, -0.8f, 0.0f,
                          0.2f, 10.0f, 3.0f);
    CHECK(!d.coast && d.driver_intent && NEAR(d.pwm_pct, -1.25f));

    /* Low-speed assistance is bounded even with a corruptly large request. */
    d = EpsAssist_Resolve(&state, 210U, -5.0f, 2.0f,
                          -80.0f, 0.0f, 3.0f);
    CHECK(!d.coast && NEAR(d.pwm_pct, -20.0f));

    /* At road speed and without driver intent, gentle return remains allowed. */
    EpsAssist_Reset(&state);
    d = EpsAssist_Resolve(&state, 500U, 0.0f, 12.0f,
                          0.0f, -5.0f, 3.0f);
    CHECK(!d.coast && !d.driver_intent && NEAR(d.pwm_pct, -5.0f));

    CHECK(NEAR(EpsAssist_EffectiveCoastBand(3.0f, true), 1.0f));
    CHECK(NEAR(EpsAssist_EffectiveCoastBand(3.0f, false), 3.0f));
    /* Speed-gated breakaway cap: active only at low speed (≤ 8.0 km/h). */
    CHECK(NEAR(EpsAssist_EffectiveMinDrive(8.0f, true, 7.9f), 4.0f));  /* 7.9 km/h → cap */
    CHECK(NEAR(EpsAssist_EffectiveMinDrive(8.0f, true, 8.0f), 4.0f));  /* 8.0 km/h → cap */
    CHECK(NEAR(EpsAssist_EffectiveMinDrive(8.0f, true, 8.1f), 8.0f));  /* 8.1 km/h → no cap */
    CHECK(NEAR(EpsAssist_EffectiveMinDrive(3.0f, true, 0.0f), 3.0f));
    CHECK(NEAR(EpsAssist_EffectiveMinDrive(8.0f, false, 0.0f), 8.0f));
    /* Speed-gated slew rate cap: active only at low speed (≤ 8.0 km/h). */
    CHECK(NEAR(EpsAssist_EffectiveSlewRate(5.883f, true, 7.9f), 1.5f));  /* 7.9 km/h → cap */
    CHECK(NEAR(EpsAssist_EffectiveSlewRate(5.883f, true, 8.0f), 1.5f));  /* 8.0 km/h → cap */
    CHECK(NEAR(EpsAssist_EffectiveSlewRate(5.883f, true, 8.1f), 5.883f)); /* 8.1 km/h → no cap */
    CHECK(NEAR(EpsAssist_EffectiveSlewRate(1.0f, true, 0.0f), 1.0f));

    /* Damping may reduce assistance, but never reverse its sign. */
    CHECK(NEAR(EpsAssist_ApplyDamping(4.5f, 0.1f, 10.0f), 3.5f));
    CHECK(NEAR(EpsAssist_ApplyDamping(-4.5f, 0.1f, -10.0f), -3.5f));
    CHECK(NEAR(EpsAssist_ApplyDamping(0.2f, 1.0f, 2.0f), 0.0f));

    /* One opposite raw encoder tick is jitter, not a confirmed reversal. */
    uint8_t reversal_cycles = 0U;
    CHECK(!EpsAssist_UpdateRawReversal(&reversal_cycles, -20.0f, 2.0f));
    CHECK(reversal_cycles == 1U);
    CHECK(EpsAssist_UpdateRawReversal(&reversal_cycles, -20.0f, 1.0f));
    CHECK(reversal_cycles == EPS_RAW_REVERSAL_CONFIRM_CYCLES);
    CHECK(!EpsAssist_UpdateRawReversal(&reversal_cycles, 2.0f, 1.0f));
    CHECK(reversal_cycles == 0U);

    /* A real latch direction flip requires a zero-torque transition cycle. */
    EpsAssistState_t flip_state = { true, -1, 100U };
    CHECK(EpsAssist_DirectionChanged(1, &flip_state, true));
    CHECK(!EpsAssist_DirectionChanged(-1, &flip_state, true));
    CHECK(!EpsAssist_DirectionChanged(1, &flip_state, false));

    /* The complete driver-intent path resolves to a bounded 4 % breakaway,
     * not the historical 8 % step and never to the opposite direction.
     * The cap applies only at low speed (0 km/h here is below 8.0 km/h). */
    EpsOutputDecision_t output = EpsOutput_Resolve(
        1.25f, EpsAssist_EffectiveCoastBand(3.0f, true),
        EpsAssist_EffectiveMinDrive(8.0f, true, 0.0f));
    CHECK(!output.coast && NEAR(output.pwm_pct, 4.0f));

    d = EpsAssist_Resolve(&state, 600U, NAN, 0.0f,
                          0.0f, 0.0f, 3.0f);
    CHECK(d.coast && !d.driver_intent);

    if (failed != 0) return 1;
    puts("EPS assist policy tests: PASS");
    return 0;
}
