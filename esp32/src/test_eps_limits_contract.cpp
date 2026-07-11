// =============================================================================
// Host cross-contract test for the EPS parameter limits (Item 6).
//
// Verifies that the authoritative HMI editor contract (esp32/src/eps_limits.h,
// eps::LIMITS) matches, per parameter, the STM32 server-side enforcement
// (Core/Src/eps_params.c eps_limits[]) — so a raw CAN EPS_PARAM_OP_SET_PARAM
// frame can never set a value wider than the HMI-safe range.  The STM32 values
// are mirrored below; the STM32 side independently checks its real table
// against the same numbers in Core/Src/test_eps_params.c
// (test_limits_match_hmi_contract).
//
// Compile (from repo root):
//   g++ -std=c++17 -Wall -Wextra -Iesp32/src
//       esp32/src/test_eps_limits_contract.cpp -o /tmp/test_eps_limits_contract
//   /tmp/test_eps_limits_contract
// =============================================================================

#include "eps_limits.h"

#include <cstdio>

static int g_run    = 0;
static int g_failed = 0;

#define ASSERT(cond) do {                                                   \
    g_run++;                                                                \
    if (!(cond)) {                                                          \
        std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);         \
        g_failed++;                                                         \
    }                                                                       \
} while (0)

// Mirror of Core/Src/eps_params.c eps_limits[] (EPS_MIN_POS == eps::MIN_POS).
// MUST stay identical to eps::LIMITS — this test exists to prove it.
struct Pair { float min; float max; };
static const Pair kStm32Limits[eps::LIMIT_COUNT] = {
    /* [0]  ASSIST_STRENGTH   */ { 0.0f,        1.0f   },
    /* [1]  CENTER_STRENGTH   */ { 0.0f,        1.0f   },
    /* [2]  DAMPING           */ { 0.0f,        1.0f   },
    /* [3]  FRICTION_COMP     */ { 0.0f,        0.5f   },
    /* [4]  COAST_BAND_PCT    */ { 0.0f,       20.0f   },
    /* [5]  MIN_DRIVE_PCT     */ { 1.0f,       50.0f   },
    /* [6]  ASSIST_VS_SPEED   */ { eps::MIN_POS, 100.0f },
    /* [7]  RETURN_VS_SPEED   */ { eps::MIN_POS, 100.0f },
    /* [8]  DEADBAND_DEG      */ { 0.1f,        10.0f  },
    /* [9]  MAX_PWM_PCT       */ { 5.0f,       100.0f  },
    /* [10] SLEW_RATE_PCT     */ { 0.1f,        20.0f  },
    /* [11] CENTER_OFFSET_DEG */ { -10.0f,      10.0f  },
};

int main() {
    // 1. HMI editor contract must EQUAL the STM32 server-side limits, per param.
    for (int i = 0; i < eps::LIMIT_COUNT; ++i) {
        ASSERT(eps::LIMITS[i].min == kStm32Limits[i].min);
        ASSERT(eps::LIMITS[i].max == kStm32Limits[i].max);
    }

    // 2. The HMI must never offer a value the STM32 server would reject:
    //    every HMI [min, max] must lie within the STM32 [min, max]
    //    (here equal, so this holds with margin zero).
    for (int i = 0; i < eps::LIMIT_COUNT; ++i) {
        ASSERT(eps::LIMITS[i].min >= kStm32Limits[i].min);
        ASSERT(eps::LIMITS[i].max <= kStm32Limits[i].max);
    }

    // 3. Every contract range must be well-formed (min <= max) and the
    //    strict-positive divisors must keep a positive lower bound.
    for (int i = 0; i < eps::LIMIT_COUNT; ++i) {
        ASSERT(eps::LIMITS[i].min <= eps::LIMITS[i].max);
    }
    ASSERT(eps::LIMITS[6].min > 0.0f);   // ASSIST_VS_SPEED divisor > 0
    ASSERT(eps::LIMITS[7].min > 0.0f);   // RETURN_VS_SPEED divisor > 0

    // 4. Guard against a silent parameter-count drift.
    ASSERT(eps::LIMIT_COUNT == 12);

    std::printf("\n--- eps_limits_contract tests: %d run, %d failed ---\n",
                g_run, g_failed);
    return g_failed ? 1 : 0;
}
