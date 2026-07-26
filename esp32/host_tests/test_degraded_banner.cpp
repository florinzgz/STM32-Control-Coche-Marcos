#include <cstring>
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
