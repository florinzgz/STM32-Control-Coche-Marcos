#include <cstdio>
#include <cstring>

#include "traction_limit_diag_view.h"
#include "../../Core/Inc/traction_limit_frame.h"

static int failed;
#define CHECK(expr) do { \
    if (!(expr)) { \
        std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        ++failed; \
    } \
} while (0)

int main() {
    uint8_t raw[TRACTION_LIMIT_FRAME_DLC] = {0};
    TractionLimitFrame_Pack(0.30f, 0.60f, 42.6f, 5U, raw);

    traction_limit_diag_view::TractionLimitView view{};
    CHECK(traction_limit_diag_view::decode(raw, sizeof(raw), view));
    CHECK(view.obstacleScalePct == 30U);
    CHECK(view.tractionCapPct == 60U);
    CHECK(view.brakeReleasePct == 43U);
    CHECK(view.obstacleState == 5U);
    CHECK(std::strcmp(traction_limit_diag_view::obstacleStateText(5U),
                      "SENSOR") == 0);

    CHECK(!traction_limit_diag_view::decode(raw, 3U, view));
    CHECK(traction_limit_diag_view::freshness(false, 5000U, 0U) ==
          traction_limit_diag_view::Freshness::NEVER_RECEIVED);
    CHECK(traction_limit_diag_view::freshness(true, 3999U, 1000U) ==
          traction_limit_diag_view::Freshness::VALID);
    CHECK(traction_limit_diag_view::freshness(true, 4001U, 1000U) ==
          traction_limit_diag_view::Freshness::STALE);

    if (failed != 0) return 1;
    std::puts("Traction limit diagnostic decode tests: PASS");
    return 0;
}
