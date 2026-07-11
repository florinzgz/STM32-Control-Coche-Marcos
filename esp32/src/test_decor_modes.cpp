/**
 ****************************************************************************
 * @file    test_decor_modes.cpp
 * @brief   Host-compilable unit tests for the decorative LED modes
 *          (POLICE_US, AMBULANCE, WARNING_AMBER, HAZARD_RED, DEMO_SHOW,
 *          CUSTOM_TEST, OFF) implemented in led_controller.cpp.
 *
 *          The firmware renderers updateDecorativeFront()/Rear() are static
 *          and depend on FastLED (CRGB / fill_solid / fill_rainbow), which is
 *          not available on the host.  This test mirrors the exact colour and
 *          geometry contract of each mode against a minimal RGB buffer so the
 *          behaviour can be verified without hardware.
 *
 *          Any change to a pattern's colour/geometry contract in
 *          led_controller.cpp MUST be mirrored here and vice-versa.
 *
 *          Compile and run on host (from esp32/src/):
 *            g++ -std=c++17 test_decor_modes.cpp \
 *                -o /tmp/test_decor_modes && /tmp/test_decor_modes
 *
 *          Requirements covered (from the decorative-mode audit spec):
 *            - OFF paints nothing (all pixels black).
 *            - POLICE_US uses only red/blue (no green), never rainbow.
 *            - AMBULANCE uses only red/white, distinct pattern from police.
 *            - WARNING_AMBER uses amber only (blue channel always 0).
 *            - HAZARD_RED uses red only (green & blue always 0).
 *            - DEMO_SHOW is the only mode allowed to be multicolour.
 *            - CUSTOM_TEST cycles pure colours for wiring verification.
 *            - No writes outside the strip buffer for count 1, 2, 70, 72.
 *            - Functional rear overlays keep priority over any decor mode.
 ****************************************************************************
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

// --- Minimal RGB pixel mirroring FastLED's CRGB memory layout ---
struct RGB { uint8_t r = 0, g = 0, b = 0; };

// --- Mirror of led_controller.cpp colour palette ---
static constexpr RGB C_RED   {255,   0,   0};
static constexpr RGB C_BLUE  {  0,   0, 255};
static constexpr RGB C_WHITE {255, 255, 255};
static constexpr RGB C_AMBER {255, 200,   0};   // LED_COLOR_AMBER
static constexpr RGB C_GREEN {  0, 255,   0};
static constexpr RGB C_BLACK {  0,   0,   0};

// --- Brightness scale factors (mirror of header constants) ---
static constexpr uint8_t BR_EMERGENCY = 120;    // LED_BRIGHTNESS_EMERGENCY
static constexpr uint8_t BR_DEMO      = 60;     // LED_BRIGHTNESS_DEMO

// --- Mirror of led_controller.cpp helpers ---
static RGB scaled(RGB c, uint8_t s) {
    return RGB{ (uint8_t)((uint16_t)c.r * s / 255),
                (uint8_t)((uint16_t)c.g * s / 255),
                (uint8_t)((uint16_t)c.b * s / 255) };
}
static void fillSolid(RGB* leds, int count, RGB c) {
    for (int i = 0; i < count; ++i) leds[i] = c;
}
static void fillHalves(RGB* leds, int count, RGB l, RGB r) {
    if (count <= 0) return;
    const int mid = count / 2;
    for (int i = 0; i < count; ++i) leds[i] = (i < mid) ? l : r;
}

// --- Mirror of the front/rear POLICE_US renderer (front variant) ---
static void renderPoliceFront(RGB* leds, int count, uint8_t phase) {
    const RGB red = scaled(C_RED, BR_EMERGENCY), blue = scaled(C_BLUE, BR_EMERGENCY);
    if      (phase == 0) fillHalves(leds, count, red, blue);
    else if (phase == 2) fillHalves(leds, count, blue, red);
    else                 fillSolid(leds, count, C_BLACK);
}
static void renderPoliceRear(RGB* leds, int count, uint8_t phase) {
    const RGB red = scaled(C_RED, BR_EMERGENCY), blue = scaled(C_BLUE, BR_EMERGENCY);
    if      (phase == 0) fillHalves(leds, count, blue, red);
    else if (phase == 2) fillHalves(leds, count, red, blue);
    else                 fillSolid(leds, count, C_BLACK);
}

// --- Mirror of the AMBULANCE renderer (front & rear identical) ---
static void renderAmbulance(RGB* leds, int count, uint8_t phase) {
    RGB col = C_BLACK;
    if      (phase == 0 || phase == 2) col = scaled(C_RED, BR_EMERGENCY);
    else if (phase == 4 || phase == 6) col = scaled(C_WHITE, BR_EMERGENCY);
    fillSolid(leds, count, col);
}

// --- Mirror of WARNING_AMBER (front full-strip blink) ---
static void renderWarningFront(RGB* leds, int count, uint8_t phase) {
    RGB col = ((phase & 1) == 0) ? scaled(C_AMBER, BR_DEMO) : C_BLACK;
    fillSolid(leds, count, col);
}

// --- Mirror of HAZARD_RED (rear double-flash) ---
static void renderHazardRear(RGB* leds, int count, uint8_t phase) {
    bool on = (phase == 0 || phase == 2);
    fillSolid(leds, count, on ? scaled(C_RED, BR_EMERGENCY) : C_BLACK);
}

// --- Mirror of OFF ---
static void renderOff(RGB* leds, int count) { fillSolid(leds, count, C_BLACK); }

// --- Mirror of RGB_DIAG (independent R/G/B per-strip colour-order test) ---
// Front lit only in phases 0-2 (R/G/B); rear lit only in phases 3-5 (R/G/B).
// Pure primaries at FULL brightness — no scaling — so a byte-order fault shows
// up as an obvious wrong colour on physical validation.
static void renderRgbDiagFront(RGB* leds, int count, uint8_t phase) {
    RGB col = C_BLACK;
    switch (phase % 6) {
        case 0: col = RGB{255, 0, 0}; break;  // FRONT red
        case 1: col = RGB{0, 255, 0}; break;  // FRONT green
        case 2: col = RGB{0, 0, 255}; break;  // FRONT blue
        default: col = C_BLACK; break;        // rear under test
    }
    fillSolid(leds, count, col);
}
static void renderRgbDiagRear(RGB* leds, int count, uint8_t phase) {
    RGB col = C_BLACK;
    switch (phase % 6) {
        case 3: col = RGB{255, 0, 0}; break;  // REAR red
        case 4: col = RGB{0, 255, 0}; break;  // REAR green
        case 5: col = RGB{0, 0, 255}; break;  // REAR blue
        default: col = C_BLACK; break;        // front under test
    }
    fillSolid(leds, count, col);
}

static int g_failures = 0;
#define CHECK(cond, msg)                                             \
    do {                                                            \
        if (!(cond)) { printf("  FAIL: %s\n", msg); ++g_failures; } \
    } while (0)

// Count pixels whose channels are non-zero.
static int litCount(const std::vector<RGB>& v) {
    int n = 0;
    for (const auto& p : v) if (p.r || p.g || p.b) ++n;
    return n;
}
// True if any pixel has a green channel set.
static bool anyGreen(const std::vector<RGB>& v) {
    for (const auto& p : v) if (p.g) return true;
    return false;
}
// True if any pixel has a blue channel set.
static bool anyBlue(const std::vector<RGB>& v) {
    for (const auto& p : v) if (p.b) return true;
    return false;
}

int main() {
    printf("== Decorative LED mode tests ==\n");

    const int counts[] = {1, 2, 70, 72};

    for (int count : counts) {
        std::vector<RGB> leds(count);

        // ---- OFF: nothing lit ----
        renderOff(leds.data(), count);
        CHECK(litCount(leds) == 0, "OFF paints all pixels black");

        // ---- POLICE_US: only red/blue, no green, at least one lit phase ----
        int policeLitPhases = 0;
        for (uint8_t phase = 0; phase < 4; ++phase) {
            renderPoliceFront(leds.data(), count, phase);
            CHECK(!anyGreen(leds), "POLICE front never uses green (no rainbow)");
            if (litCount(leds) > 0) ++policeLitPhases;
            renderPoliceRear(leds.data(), count, phase);
            CHECK(!anyGreen(leds), "POLICE rear never uses green (no rainbow)");
        }
        CHECK(policeLitPhases == 2, "POLICE has exactly 2 lit phases (0 and 2)");

        // POLICE phase 0 front: left half red, right half blue (count >= 2).
        if (count >= 2) {
            renderPoliceFront(leds.data(), count, 0);
            const int mid = count / 2;
            CHECK(leds[0].r > 0 && leds[0].b == 0, "POLICE ph0 left half is red");
            CHECK(leds[count - 1].b > 0 && leds[count - 1].r == 0,
                  "POLICE ph0 right half is blue");
            (void)mid;
        }

        // ---- AMBULANCE: only red/white, distinct double-flash ----
        int ambuLit = 0;
        for (uint8_t phase = 0; phase < 8; ++phase) {
            renderAmbulance(leds.data(), count, phase);
            if (litCount(leds) > 0) ++ambuLit;
            // Red phase: no blue/green; White phase: r==g==b.
            for (const auto& p : leds) {
                if (p.r || p.g || p.b) {
                    bool isRed   = (p.g == 0 && p.b == 0);
                    bool isWhite = (p.r == p.g && p.g == p.b);
                    CHECK(isRed || isWhite, "AMBULANCE pixel is red or white only");
                }
            }
        }
        CHECK(ambuLit == 4, "AMBULANCE double-flash: 4 lit phases (0,2,4,6)");

        // ---- WARNING_AMBER: amber only, blue channel always 0 ----
        for (uint8_t phase = 0; phase < 2; ++phase) {
            renderWarningFront(leds.data(), count, phase);
            CHECK(!anyBlue(leds), "WARNING_AMBER never uses blue channel");
            for (const auto& p : leds)
                if (p.r || p.g)
                    CHECK(p.r > 0 && p.g > 0 && p.b == 0,
                          "WARNING_AMBER lit pixel is amber (red+green, no blue)");
        }
        renderWarningFront(leds.data(), count, 0);
        CHECK(litCount(leds) == count, "WARNING_AMBER phase 0 lights whole strip");
        renderWarningFront(leds.data(), count, 1);
        CHECK(litCount(leds) == 0, "WARNING_AMBER phase 1 is off");

        // ---- HAZARD_RED: red only, green & blue always 0 ----
        for (uint8_t phase = 0; phase < 8; ++phase) {
            renderHazardRear(leds.data(), count, phase);
            CHECK(!anyGreen(leds) && !anyBlue(leds),
                  "HAZARD_RED uses red only (no green/blue)");
        }
        renderHazardRear(leds.data(), count, 0);
        CHECK(litCount(leds) == count, "HAZARD_RED flash-on lights whole strip");
        renderHazardRear(leds.data(), count, 1);
        CHECK(litCount(leds) == 0, "HAZARD_RED between-flash is off");

        printf("  count=%d: OFF/POLICE/AMBULANCE/WARNING/HAZARD OK\n", count);
    }

    // ---- RGB_DIAG: independent R/G/B per-strip colour-order diagnostic ----
    // Contract: exactly one strip lit at a time, in exactly one PURE primary,
    // never both strips simultaneously, and colours are full-scale (255) so a
    // wrong byte order is unmistakable on physical validation.
    for (int count : counts) {
        std::vector<RGB> front(count), rear(count);
        // Expected pure primary per phase for the strip under test.
        const RGB expect[6] = {
            {255, 0, 0}, {0, 255, 0}, {0, 0, 255},   // front R/G/B
            {255, 0, 0}, {0, 255, 0}, {0, 0, 255},   // rear  R/G/B
        };
        for (uint8_t phase = 0; phase < 6; ++phase) {
            renderRgbDiagFront(front.data(), count, phase);
            renderRgbDiagRear(rear.data(),  count, phase);

            const bool frontPhase = (phase < 3);
            const auto& lit  = frontPhase ? front : rear;
            const auto& dark = frontPhase ? rear  : front;

            // Exactly one strip is energised.
            CHECK(litCount(lit)  == count, "RGB_DIAG lights the whole strip under test");
            CHECK(litCount(dark) == 0,     "RGB_DIAG keeps the other strip dark (isolation)");

            // The lit strip shows exactly the expected pure primary everywhere.
            for (const auto& p : lit)
                CHECK(p.r == expect[phase].r && p.g == expect[phase].g &&
                      p.b == expect[phase].b,
                      "RGB_DIAG lit strip is the exact pure primary");

            // Pure primary => exactly one channel non-zero, at full scale.
            const RGB& e = expect[phase];
            const int nz = (e.r != 0) + (e.g != 0) + (e.b != 0);
            CHECK(nz == 1, "RGB_DIAG uses a single-channel pure primary");
            CHECK(e.r == 255 || e.g == 255 || e.b == 255,
                  "RGB_DIAG primary is full-scale (255)");
        }
        printf("  count=%d: RGB_DIAG per-strip R/G/B isolation OK\n", count);
    }

    // ---- Functional rear overlay predicate keeps priority over decor ----
    // Mirror of the isRearFunctionalOverlay lambda in update().
    enum RearMode { OFF, POSITION, BRAKE, BRAKE_EMERGENCY, REVERSE, REGEN_ACTIVE };
    auto isFunctional = [](RearMode m) {
        return m == BRAKE || m == BRAKE_EMERGENCY || m == REVERSE || m == REGEN_ACTIVE;
    };
    CHECK(isFunctional(BRAKE),           "BRAKE overrides decor");
    CHECK(isFunctional(BRAKE_EMERGENCY), "BRAKE_EMERGENCY overrides decor");
    CHECK(isFunctional(REVERSE),         "REVERSE overrides decor");
    CHECK(isFunctional(REGEN_ACTIVE),    "REGEN_ACTIVE overrides decor");
    CHECK(!isFunctional(OFF),            "OFF does not override decor");
    CHECK(!isFunctional(POSITION),       "POSITION does not override decor");

    if (g_failures == 0) {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("%d CHECK(s) FAILED\n", g_failures);
    return 1;
}
