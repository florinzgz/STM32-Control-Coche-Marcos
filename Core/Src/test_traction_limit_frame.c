#include "traction_limit_frame.h"

#include <math.h>
#include <stdio.h>

static int failed;
#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failed++; \
    } \
} while (0)

int main(void)
{
    uint8_t bytes[TRACTION_LIMIT_FRAME_DLC] = {0};
    TractionLimitFrame_Pack(0.30f, 0.60f, 42.6f, 5U, bytes);
    CHECK(bytes[0] == 30U);
    CHECK(bytes[1] == 60U);
    CHECK(bytes[2] == 43U);
    CHECK(bytes[3] == 5U);

    TractionLimitFrame decoded = {0};
    TractionLimitFrame_Unpack(bytes, &decoded);
    CHECK(decoded.obstacle_scale_pct == 30U);
    CHECK(decoded.traction_cap_pct == 60U);
    CHECK(decoded.brake_release_pct == 43U);
    CHECK(decoded.obstacle_state == 5U);

    TractionLimitFrame_Pack(2.0f, -1.0f, NAN, 2U, bytes);
    CHECK(bytes[0] == 100U);
    CHECK(bytes[1] == 0U);
    CHECK(bytes[2] == 0U);
    CHECK(bytes[3] == 2U);

    if (failed != 0) return 1;
    puts("Traction limit frame tests: PASS");
    return 0;
}
