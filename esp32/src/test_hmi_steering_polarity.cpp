// Host tests for the presentation-only steering polarity shared by all HMI screens.
#include "ui/ui_common.h"
#include <cstdio>

static int run_count = 0;
static int fail_count = 0;
#define CHECK(expr) do { ++run_count; if (!(expr)) { \
    std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); ++fail_count; \
} } while (0)

int main() {
    CHECK(ui::STEER_DISPLAY_SIGN == -1);
    CHECK(ui::clampRoadWheelTenths(100) == -100);
    CHECK(ui::clampRoadWheelTenths(-100) == 100);
    CHECK(ui::clampRoadWheelTenths(0) == 0);
    CHECK(ui::clampRoadWheelTenths(700) == -540);
    CHECK(ui::clampRoadWheelTenths(-700) == 540);
    CHECK(ui::steeringWheelDeg(ui::clampRoadWheelTenths(100)) < 0);
    CHECK(ui::steeringWheelDeg(ui::clampRoadWheelTenths(-100)) > 0);
    std::printf("HMI steering polarity: %d run, %d failed\n", run_count, fail_count);
    return fail_count == 0 ? 0 : 1;
}
