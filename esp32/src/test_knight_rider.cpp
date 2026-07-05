/**
 ****************************************************************************
 * @file    test_knight_rider.cpp
 * @brief   Host-compilable unit tests for the KNIGHT_RIDER ("coche
 *          fantástico") decorative LED scanner algorithm.
 *
 *          The firmware helper renderKnightRider() in led_controller.cpp is
 *          static and depends on the FastLED library (CRGB / fill_solid),
 *          which is not available on the host.  This test mirrors the exact
 *          pure geometry + tail contract of that helper against a minimal RGB
 *          buffer so the visual behaviour can be verified without hardware.
 *
 *          Any change to the scanner contract in led_controller.cpp
 *          (position bounce, tail profile, red-only colour) MUST be mirrored
 *          here and vice-versa.
 *
 *          Compile and run on host (from esp32/src/):
 *            g++ -std=c++17 test_knight_rider.cpp \
 *                -o /tmp/test_knight_rider && /tmp/test_knight_rider
 *
 *          Requirements covered (from the KITT effect spec):
 *            1. Black background — buffer cleared every frame, no stale pixels
 *            2. Red bounce point moves left↔right and rebounds at both ends
 *            3. Position never leaves the range 0..count-1 (clamped)
 *            4. Symmetric red tail: centre 255, ±1 120, ±2 50, ±3 15
 *            5. Pure red only — green and blue channels are always 0 (no HSV)
 *            6. The whole bar is never permanently lit
 ****************************************************************************
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

// --- Minimal RGB pixel mirroring FastLED's CRGB memory layout ---
struct RGB { uint8_t r = 0, g = 0, b = 0; };

// --- Exact mirror of led_controller.cpp::renderKnightRider() ---
static void renderKnightRider(RGB* leds, int count, uint16_t step) {
    if (count <= 0) return;

    for (int i = 0; i < count; ++i) leds[i] = RGB{0, 0, 0};  // fill_solid black

    if (count == 1) { leds[0] = RGB{255, 0, 0}; return; }

    const int period = 2 * (count - 1);
    const int phase  = step % period;
    const int pos    = (phase < count) ? phase : (period - phase);

    static const uint8_t kTail[] = {255, 120, 50, 15};
    const int kTailLen = static_cast<int>(sizeof(kTail) / sizeof(kTail[0]));

    for (int off = 0; off < kTailLen; ++off) {
        const RGB col{kTail[off], 0, 0};
        if (off == 0) {
            leds[pos] = col;
        } else {
            const int lo = pos - off;
            const int hi = pos + off;
            if (lo >= 0 && lo < count) leds[lo] = col;
            if (hi >= 0 && hi < count) leds[hi] = col;
        }
    }
}

// Recompute the expected centre position independently of the helper.
static int expectedPos(int count, uint16_t step) {
    const int period = 2 * (count - 1);
    const int phase  = step % period;
    return (phase < count) ? phase : (period - phase);
}

static int g_failures = 0;
#define CHECK(cond, msg)                                             \
    do {                                                            \
        if (!(cond)) { printf("  FAIL: %s\n", msg); ++g_failures; } \
    } while (0)

int main() {
    printf("== KNIGHT_RIDER (KITT) scanner tests ==\n");

    const int counts[] = {70, 72, 50, 2};  // front, rear, centre-only, degenerate

    for (int count : counts) {
        std::vector<RGB> leds(count);
        const int period = 2 * (count - 1);

        bool sawLeftEnd = false, sawRightEnd = false;
        int  minPos = count, maxPos = -1;

        // Sweep a full bounce period plus extra to prove wrap/rebound.
        for (int step = 0; step < period * 2 + 5; ++step) {
            renderKnightRider(leds.data(), count, static_cast<uint16_t>(step));

            const int pos = expectedPos(count, static_cast<uint16_t>(step));
            CHECK(pos >= 0 && pos < count, "position stays within range");
            if (pos < minPos) minPos = pos;
            if (pos > maxPos) maxPos = pos;
            if (pos == 0)         sawLeftEnd  = true;
            if (pos == count - 1) sawRightEnd = true;

            // Centre pixel must be full red.
            CHECK(leds[pos].r == 255 && leds[pos].g == 0 && leds[pos].b == 0,
                  "centre pixel is pure full red");

            // Green/blue are ALWAYS zero — no rainbow anywhere.
            int litCount = 0;
            for (int i = 0; i < count; ++i) {
                CHECK(leds[i].g == 0 && leds[i].b == 0,
                      "no green/blue channel (no rainbow)");
                if (leds[i].r != 0) ++litCount;
            }
            // At most centre + 3 tail LEDs each side = 7 lit; never the whole bar.
            CHECK(litCount <= 7, "only scanner + tail lit, never whole bar");

            // Verify symmetric tail values around the centre.
            const uint8_t tail[] = {255, 120, 50, 15};
            for (int off = 1; off < 4; ++off) {
                if (pos - off >= 0)
                    CHECK(leds[pos - off].r == tail[off], "left tail brightness");
                if (pos + off < count)
                    CHECK(leds[pos + off].r == tail[off], "right tail brightness");
            }
        }

        CHECK(sawLeftEnd,  "scanner reaches left end (pos 0)");
        CHECK(sawRightEnd, "scanner reaches right end (pos count-1)");
        CHECK(minPos == 0 && maxPos == count - 1, "full travel to both extremes");

        printf("  count=%d: travel %d..%d, rebound OK\n", count, minPos, maxPos);
    }

    if (g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("%d CHECK(s) FAILED\n", g_failures);
    return 1;
}
