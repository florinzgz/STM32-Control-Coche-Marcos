// test_boot_diag.cpp — host unit test for the pure boot-diagnostics formatter.
//
// Build & run (from repo root):
//   g++ -std=c++17 -Iesp32/src esp32/src/test_boot_diag.cpp -o /tmp/t_bd && /tmp/t_bd
#include "boot_diag.h"

#include <cstring>
#include <cstdio>
#include <string>

static int g_run = 0;
static int g_failed = 0;

#define CHECK(cond, msg)                                            \
    do {                                                            \
        ++g_run;                                                    \
        if (!(cond)) {                                              \
            ++g_failed;                                             \
            printf("  FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
        } else {                                                    \
            printf("  ok:   %s\n", msg);                            \
        }                                                           \
    } while (0)

using boot_diag::ResetClass;

int main() {
    printf("=== boot_diag tests ===\n");

    // --- name() covers every enumerator ---------------------------------
    CHECK(std::strcmp(boot_diag::name(ResetClass::POWER_ON), "PowerOn") == 0, "name PowerOn");
    CHECK(std::strcmp(boot_diag::name(ResetClass::SOFTWARE), "Software") == 0, "name Software");
    CHECK(std::strcmp(boot_diag::name(ResetClass::PANIC), "Panic") == 0, "name Panic");
    CHECK(std::strcmp(boot_diag::name(ResetClass::WATCHDOG), "Watchdog") == 0, "name Watchdog");
    CHECK(std::strcmp(boot_diag::name(ResetClass::BROWNOUT), "Brownout") == 0, "name Brownout");
    CHECK(std::strcmp(boot_diag::name(ResetClass::DEEPSLEEP), "DeepSleep") == 0, "name DeepSleep");
    CHECK(std::strcmp(boot_diag::name(ResetClass::EXTERNAL_PIN), "External") == 0, "name External");
    CHECK(std::strcmp(boot_diag::name(ResetClass::UNKNOWN), "Unknown") == 0, "name Unknown");

    // --- isAbnormal(): only panic/watchdog/brownout are abnormal --------
    CHECK(!boot_diag::isAbnormal(ResetClass::POWER_ON),  "PowerOn is normal");
    CHECK(!boot_diag::isAbnormal(ResetClass::SOFTWARE),  "Software is normal");
    CHECK(!boot_diag::isAbnormal(ResetClass::DEEPSLEEP), "DeepSleep is normal");
    CHECK(!boot_diag::isAbnormal(ResetClass::EXTERNAL_PIN),  "External is normal");
    CHECK(!boot_diag::isAbnormal(ResetClass::UNKNOWN),   "Unknown is normal");
    CHECK(boot_diag::isAbnormal(ResetClass::PANIC),    "Panic is abnormal");
    CHECK(boot_diag::isAbnormal(ResetClass::WATCHDOG), "Watchdog is abnormal");
    CHECK(boot_diag::isAbnormal(ResetClass::BROWNOUT), "Brownout is abnormal");

    // --- format(): normal power-on boot ---------------------------------
    {
        char buf[256];
        boot_diag::HeapStats h{320000u, 250000u, 350000u, 110000u};
        int w = boot_diag::format(buf, sizeof(buf), ResetClass::POWER_ON, h, 1500u, 900u);
        CHECK(w > 0, "format returns positive length");
        CHECK((int)std::strlen(buf) == w, "returned length matches strlen");
        std::string s(buf);
        CHECK(s.find("reset=PowerOn") != std::string::npos, "contains reset=PowerOn");
        CHECK(s.find("abnormal=0") != std::string::npos, "power-on abnormal=0");
        CHECK(s.find("heap_free=320000") != std::string::npos, "contains heap_free");
        CHECK(s.find("heap_min=250000") != std::string::npos, "contains heap_min");
        CHECK(s.find("heap_size=350000") != std::string::npos, "contains heap_size");
        CHECK(s.find("heap_maxblk=110000") != std::string::npos, "contains heap_maxblk");
        CHECK(s.find("stack_loop=1500") != std::string::npos, "contains stack_loop");
        CHECK(s.find("stack_render=900") != std::string::npos, "contains stack_render");
    }

    // --- format(): abnormal brownout boot -------------------------------
    {
        char buf[256];
        boot_diag::HeapStats h{10000u, 8000u, 350000u, 4096u};
        boot_diag::format(buf, sizeof(buf), ResetClass::BROWNOUT, h, 100u, 0u);
        std::string s(buf);
        CHECK(s.find("reset=Brownout") != std::string::npos, "brownout name in output");
        CHECK(s.find("abnormal=1") != std::string::npos, "brownout abnormal=1");
        CHECK(s.find("stack_render=0") != std::string::npos, "render stack 0 sentinel");
    }

    // --- format(): guards against null / zero-length buffer -------------
    {
        boot_diag::HeapStats h{0u, 0u, 0u, 0u};
        CHECK(boot_diag::format(nullptr, 16, ResetClass::PANIC, h, 0, 0) == 0, "null buf -> 0");
        char tiny[1];
        CHECK(boot_diag::format(tiny, 0, ResetClass::PANIC, h, 0, 0) == 0, "zero len -> 0");
    }

    // --- format(): truncation is bounded (never exceeds buffer) ---------
    {
        char small[16];
        boot_diag::HeapStats h{4294967295u, 4294967295u, 4294967295u, 4294967295u};
        int w = boot_diag::format(small, sizeof(small), ResetClass::WATCHDOG, h, 4294967295u, 4294967295u);
        CHECK(w < (int)sizeof(small), "truncated length < buffer size");
        CHECK(std::strlen(small) < sizeof(small), "truncated string fits buffer");
    }

    printf("=== %d run, %d failed ===\n", g_run, g_failed);
    return g_failed == 0 ? 0 : 1;
}
