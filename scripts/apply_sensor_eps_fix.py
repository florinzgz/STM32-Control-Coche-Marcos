#!/usr/bin/env python3
"""One-shot deterministic patch for PR #438.

The script is intentionally removed by itself after a successful run.  It starts
from the partially-applied current-plausibility fix already present on the PR
branch and completes the EPS, HMI and workflow changes.
"""

from pathlib import Path


def replace_exact(path: str, old: str, new: str, count: int = 1) -> None:
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    found = text.count(old)
    if found != count:
        raise SystemExit(
            f"{path}: expected {count} occurrence(s), found {found}: {old[:100]!r}"
        )
    p.write_text(text.replace(old, new, count), encoding="utf-8")


# ---------------------------------------------------------------------------
# EPS output shaping: coast before minimum-drive compensation.
# ---------------------------------------------------------------------------
Path("Core/Inc/eps_output_policy.h").write_text(
    """#ifndef EPS_OUTPUT_POLICY_H
#define EPS_OUTPUT_POLICY_H

#include <stdbool.h>
#include <math.h>

typedef struct {
    float pwm_pct;
    bool  coast;
} EpsOutputDecision_t;

/* Resolve a requested EPS output without touching hardware.
 * Requests inside the coast band physically release the H-bridge.  Minimum
 * drive compensation is applied only after genuine driver/return intent has
 * crossed that band, preventing a tiny request from becoming a permanent
 * ~8 percent steering effort near centre. */
static inline EpsOutputDecision_t
EpsOutput_Resolve(float pwm_pct, float coast_band_pct, float min_drive_pct)
{
    EpsOutputDecision_t out = { 0.0f, true };

    if (isnan(pwm_pct) || isinf(pwm_pct) ||
        isnan(coast_band_pct) || isinf(coast_band_pct) ||
        isnan(min_drive_pct) || isinf(min_drive_pct) ||
        coast_band_pct < 0.0f || min_drive_pct < 0.0f) {
        return out;
    }

    const float abs_pct = fabsf(pwm_pct);
    if (abs_pct <= 0.01f || abs_pct < coast_band_pct) {
        return out;
    }

    out.pwm_pct = pwm_pct;
    if (abs_pct < min_drive_pct) {
        out.pwm_pct = (pwm_pct > 0.0f) ? min_drive_pct : -min_drive_pct;
    }
    out.coast = false;
    return out;
}

#endif /* EPS_OUTPUT_POLICY_H */
""",
    encoding="utf-8",
)

Path("Core/Src/test_eps_output_policy.c").write_text(
    r'''#include <math.h>
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
''',
    encoding="utf-8",
)

motor = "Core/Src/motor_control.c"
replace_exact(
    motor,
    '#include "eps_params.h"\n',
    '#include "eps_params.h"\n#include "eps_output_policy.h"\n',
)
replace_exact(
    motor,
    """    /* ---- Guard: encoder fault → disable motor ---- */
""",
    """    /* ---- Guard: steering power path not ready → coast ----
     * The EPS may only write PC4/PA6/PA7 after the relay sequence has
     * completed and the steering-motor rail command (PC12) is ON. */
    if (!Safety_IsPowerReady() ||
        HAL_GPIO_ReadPin(GPIOC, PIN_RELAY_STEER_PWR) != GPIO_PIN_SET) {
        Steering_Neutralize();
        return;
    }

    /* ---- Guard: encoder fault → disable motor ---- */
""",
)
replace_exact(
    motor,
    """    /* ---- Dead-zone compensation: jump to min_drive_pct ---- */
    float abs_pct = fabsf(pwm_pct);
    if (abs_pct > 0.01f && abs_pct < p->min_drive_pct) {
        pwm_pct = (pwm_pct > 0.0f) ? p->min_drive_pct : -p->min_drive_pct;
        abs_pct = p->min_drive_pct;
    }

    /* ---- Coast band: below coast_band_pct → motor off (EN=LOW) ---- */
    if (abs_pct < p->coast_band_pct) {
        Steering_Neutralize();
        return;
    }
""",
    """    /* ---- Coast first; minimum-drive only for genuine intent ----
     * Previously a tiny request was raised to min_drive_pct before this
     * decision, keeping the bridge enabled and fighting the driver. */
    EpsOutputDecision_t output = EpsOutput_Resolve(
        pwm_pct, p->coast_band_pct, p->min_drive_pct);
    if (output.coast) {
        Steering_Neutralize();
        return;
    }
    pwm_pct = output.pwm_pct;
""",
)


# ---------------------------------------------------------------------------
# HMI: render the real degradation level exported by CAN 0x315.
# ---------------------------------------------------------------------------
Path("esp32/src/degraded_banner.h").write_text(
    """#ifndef DEGRADED_BANNER_H
#define DEGRADED_BANNER_H

#include <cstdint>

namespace degraded_banner {

inline const char* text(uint8_t level)
{
    switch (level) {
        case 1: return "DEGRADED L1 70%";
        case 2: return "DEGRADED L2 50%";
        case 3: return "DEGRADED L3 40%";
        default: return "DEGRADED";
    }
}

} // namespace degraded_banner

#endif /* DEGRADED_BANNER_H */
""",
    encoding="utf-8",
)

Path("esp32/src/test_degraded_banner.cpp").write_text(
    r'''#include <cstring>
#include <iostream>
#include "degraded_banner.h"

static int failed;
#define CHECK_EQ(got, expected) do { \
    if (std::strcmp((got), (expected)) != 0) { \
        std::cerr << "FAIL " << __FILE__ << ':' << __LINE__ \
                  << " got='" << (got) << "' expected='" << (expected) << "'\n"; \
        failed++; \
    } \
} while (0)

int main()
{
    CHECK_EQ(degraded_banner::text(0), "DEGRADED");
    CHECK_EQ(degraded_banner::text(1), "DEGRADED L1 70%");
    CHECK_EQ(degraded_banner::text(2), "DEGRADED L2 50%");
    CHECK_EQ(degraded_banner::text(3), "DEGRADED L3 40%");
    CHECK_EQ(degraded_banner::text(9), "DEGRADED");
    if (failed != 0) return 1;
    std::cout << "degraded banner tests: PASS\n";
    return 0;
}
''',
    encoding="utf-8",
)

replace_exact(
    "esp32/src/screens/drive_screen.h",
    """    can::SystemState curSystemState_  = can::SystemState::ACTIVE;
    can::SystemState prevSystemState_ = can::SystemState::ACTIVE;
""",
    """    can::SystemState curSystemState_  = can::SystemState::ACTIVE;
    can::SystemState prevSystemState_ = can::SystemState::ACTIVE;
    uint8_t          curDegradedLevel_ = 0;  // 0x315: 0=unknown, 1..3
""",
)

screen = "esp32/src/screens/drive_screen.cpp"
replace_exact(
    screen,
    '#include "drive_screen.h"\n',
    '#include "drive_screen.h"\n#include "degraded_banner.h"\n',
)
replace_exact(
    screen,
    """    curSystemState_  = can::SystemState::ACTIVE;
    prevSystemState_ = can::SystemState::ACTIVE;
    curFaultFlags_   = 0;
""",
    """    curSystemState_   = can::SystemState::ACTIVE;
    prevSystemState_  = can::SystemState::ACTIVE;
    curDegradedLevel_ = 0;
    curFaultFlags_    = 0;
""",
)
replace_exact(
    screen,
    """    // System state for degraded/limp overlay (HMI_STATE_MODEL §2.4)
    curSystemState_ = data.heartbeat().systemState;

    // Safety error code for LIMP HOME banner reason — identifies the cause
""",
    """    // System state for degraded/limp overlay (HMI_STATE_MODEL §2.4)
    curSystemState_ = data.heartbeat().systemState;

    // Real degradation level comes from 0x315 at 10 Hz.  When that frame is
    // unavailable, render a neutral DEGRADED label instead of inventing 40 %.
    {
        const auto& mi = data.motionInhibit();
        const bool miFresh = mi.valid &&
            ((frameTimeMs - mi.timestampMs) <= ui::cfg::TELEMETRY_FAST_STALE_MS);
        curDegradedLevel_ = miFresh ? mi.degradedLevel : 0;
    }

    // Safety error code for LIMP HOME banner reason — identifies the cause
""",
)
replace_exact(
    screen,
    """    tiles_.updateHash(DTILE_DEGRADED, ui::tileHashVal(
        (uint32_t)curDegradedVisible_ | ((uint32_t)curLimpErrorCode_ << 8)));
""",
    """    tiles_.updateHash(DTILE_DEGRADED, ui::tileHashVal(
        (uint32_t)curDegradedVisible_ |
        ((uint32_t)curLimpErrorCode_ << 8) |
        ((uint32_t)curDegradedLevel_ << 16)));
""",
)
replace_exact(
    screen,
    '        bannerText = "DEGRADED  40%";\n',
    '        bannerText = degraded_banner::text(curDegradedLevel_);\n',
)


# ---------------------------------------------------------------------------
# Restore manual workflow execution and register the new tests.
# ---------------------------------------------------------------------------
workflow_path = Path(".github/workflows/firmware-validation.yml")
lines = workflow_path.read_text(encoding="utf-8").splitlines()

if "  workflow_dispatch:" not in lines:
    on_index = lines.index("on:")
    lines.insert(on_index + 1, "  workflow_dispatch:")

motor_test = "              Core/Src/test_motor_control.c " + "\\"
current_test = "              Core/Src/test_current_plausibility.c " + "\\"
eps_test = "              Core/Src/test_eps_output_policy.c " + "\\"
if current_test not in lines:
    test_index = lines.index(motor_test)
    lines[test_index + 1 : test_index + 1] = [current_test, eps_test]

degraded_echo = '            echo "-- test_degraded_banner.cpp"'
if degraded_echo not in lines:
    anchor = lines.index("            /tmp/test_eps_limits_contract")
    block = [
        "",
        degraded_echo,
        "            g++ -std=c++17 -Wall -Wextra -Werror -Iesp32/src " + "\\",
        "              esp32/src/test_degraded_banner.cpp " + "\\",
        "              -o /tmp/test_degraded_banner",
        "            /tmp/test_degraded_banner",
    ]
    lines[anchor + 1 : anchor + 1] = block

workflow_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# Remove all one-shot diagnostic/bootstrap artefacts from the final PR diff.
for obsolete in (
    ".bootstrap-error.txt",
    ".github/workflows/bootstrap-sensor-eps-fix.yml",
    ".github/workflows/bootstrap-pr-apply-v2.yml",
    "scripts/apply_sensor_eps_fix.py",
):
    p = Path(obsolete)
    if p.exists():
        p.unlink()
