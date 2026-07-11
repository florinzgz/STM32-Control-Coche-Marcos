// =============================================================================
// ESP32-S3 HMI — LED Controller Implementation (WS2812B via FastLED)
//
// Two independent strips driven by separate GPIOs:
//   Front (GPIO 47, 70 LEDs): KITT scanner, throttle-reactive chase, alerts,
//                              turn signals (10 LEDs each side)
//   Rear  (GPIO 48, 72 LEDs): tail, brake, reverse, turn signals, regen
//                              (10 LEDs each side)
//
// Front LED zone mapping:
//   [0–9]    Left  turn-signal indicator (10 LEDs)
//   [10–59]  Centre: KITT / throttle / alert effects (50 LEDs)
//   [60–69]  Right turn-signal indicator (10 LEDs)
//
// Rear LED zone mapping:
//   [0–9]    Left  turn-signal indicator (sequential sweep, non-destructive overlay)
//   [10–61]  Centre: position / brake / reverse / regen (52 LEDs)
//   [62–71]  Right turn-signal indicator (sequential sweep, non-destructive overlay)
//
// Both front and rear turn-signal overlays are non-destructive:
//   - AMBER is written ONLY during blink-ON
//   - During blink-OFF, no write → base effect remains visible
//   - Rear uses sequential sweep (European-style, outward from centre)
//
// Architecture ported from FULL-FIRMWARE-Coche-Marcos (florinzgz):
//   src/lighting/led_controller.cpp + include/led_controller.h
//
// FastLED.show() is called once per update() invocation (~20 Hz).
// Non-blocking: no delay() calls, all state tracked via millis().
// =============================================================================

#include "led_controller.h"
#include <FastLED.h>

namespace led_ctrl {

// ---- LED arrays (one per strip) ----
static CRGB ledsFront[NUM_LEDS_FRONT];
static CRGB ledsRear[NUM_LEDS_REAR];
static bool initialized = false;
static bool enabled     = true;

// ---- Current state ----
static FrontMode  currentFrontMode  = FrontMode::OFF;
static RearMode   currentRearMode   = RearMode::OFF;
static TurnSignal currentTurnSignal = TurnSignal::OFF;

// ---- DEMO turn-signal override ----
// While DecorMode::DEMO_SHOW cycles through its indicator/hazard steps, it
// requests a turn signal here so the amber overlays can be verified visually.
// The REAL turn signal (from steering / SAFE-mode hazard) always keeps
// priority: this override is only applied when currentTurnSignal is OFF.
static TurnSignal demoTurnOverride = TurnSignal::OFF;

// ---- Animation counters ----
static uint32_t lastUpdateMs   = 0;
static uint16_t animationStep  = 0;
static bool     blinkState     = false;
static uint32_t lastBlinkMs    = 0;
static constexpr uint32_t UPDATE_RATE_MS = 50;  // 20 Hz

// ---- KITT scanner state ----
static int8_t scannerPos       = 0;
static int8_t scannerDirection = 1;
static constexpr uint8_t KITT_TAIL_LENGTH = 4;

// ---- Chase speed per throttle band ----
static constexpr uint8_t CHASE_SPEED_LOW  = 8;
static constexpr uint8_t CHASE_SPEED_MED  = 5;
static constexpr uint8_t CHASE_SPEED_HIGH = 3;
static constexpr uint8_t RAINBOW_SPEED    = 3;

// ---- Emergency flash (non-blocking) ----
static bool     emergencyActive    = false;
static uint8_t  emergencyCount     = 0;
static uint8_t  emergencyCurrent   = 0;
static uint32_t emergencyLastMs    = 0;
static bool     emergencyPhase     = false;

// =========================================================================
// Explicit decorative / signal colour palette (documented)
//
// The rear WS2812B strip is configured GRB in init() and the front strip is
// configured RGB (see FastLED.addLeds<> — the front controllers use RGB byte
// order on this hardware). FastLED translates these logical RGB values to the
// correct per-strip byte order, so on hardware red=red, green=green, blue=blue
// on BOTH strips.
// Define every decorative colour explicitly here — no ambiguous inline
// colours in the pattern renderers.
// =========================================================================
static constexpr CRGB LED_COLOR_RED_EMERGENCY = CRGB(255,   0,   0);  // emergency red
static constexpr CRGB LED_COLOR_BLUE_POLICE   = CRGB(  0,   0, 255);  // police blue
static constexpr CRGB LED_COLOR_WHITE         = CRGB(255, 255, 255);  // pure white

// ---- Amber colour for indicators and amber decorative modes ----
// Real hardware showed the previous value CRGB(255,100,0) as too ORANGE.
// Raising the green channel pushes the hue toward a clean amber/yellow.
// Verified options: A(255,180,0) B(255,200,0) C(255,220,0); (255,80/100/120,0)
// are too orange. B is the chosen default — adjust the green channel here if a
// different amber is preferred on the installed strip.
static constexpr CRGB LED_COLOR_AMBER = CRGB(255, 200, 0);

// Backwards-compatible alias used throughout the indicator/decor renderers.
static constexpr CRGB AMBER = LED_COLOR_AMBER;

// =========================================================================
// Decorative mode state
// =========================================================================

static DecorMode currentDecorMode = DecorMode::NORMAL;

// Returns the turn signal that the overlays should render.  The real turn
// signal (steering / SAFE-mode hazard) always wins; the DEMO override is only
// honoured while DEMO_SHOW is active AND no real turn signal is requested.
static TurnSignal effectiveTurnSignal() {
    if (currentTurnSignal != TurnSignal::OFF) return currentTurnSignal;
    if (currentDecorMode == DecorMode::DEMO_SHOW) return demoTurnOverride;
    return TurnSignal::OFF;
}

// Shared phase counter for all decorative patterns (non-blocking timing)
static uint32_t decorLastMs   = 0;
static uint8_t  decorPhase    = 0;    // generic phase index within pattern
// POLICE_US: 4-phase left/right block wig-wag (red↔blue swap, documented).
// The left HALF of the strip and the right HALF of the strip flash opposite
// colours, then swap, giving the classic US police "wig-wag" look.  No amber,
// no rainbow — only LED_COLOR_RED_EMERGENCY and LED_COLOR_BLUE_POLICE.
//   Phase 0: left half RED   / right half BLUE
//   Phase 1: brief blackout (crisp strobe edge)
//   Phase 2: left half BLUE  / right half RED   (colours inverted)
//   Phase 3: brief blackout
static constexpr uint16_t DECOR_POLICE_STEP_MS  = 110;  // ms per step (~440 ms cycle)
static constexpr uint8_t  DECOR_POLICE_PHASES   = 4;    // total phases in cycle

// HAZARD_RED: double-flash rear (phases 0-3), long pause (phases 4-7)
static constexpr uint16_t DECOR_HAZARD_STEP_MS = 80;
static constexpr uint8_t  DECOR_HAZARD_PHASES  = 8;

// WARNING_AMBER: slow blink 400 ms on / 400 ms off
static constexpr uint16_t DECOR_WARN_STEP_MS = 400;

// AMBULANCE: red/white DOUBLE-FLASH — clearly distinct from POLICE_US.
// Two quick red flashes, pause, two quick white flashes, pause, repeat.
// Whole strip flashes (no left/right split) so it never looks like police.
//   Phase 0: RED   ON    Phase 4: WHITE ON
//   Phase 1: OFF         Phase 5: OFF
//   Phase 2: RED   ON    Phase 6: WHITE ON
//   Phase 3: OFF (pause) Phase 7: OFF (pause)
static constexpr uint16_t DECOR_AMBU_STEP_MS = 110;  // ms per step
static constexpr uint8_t  DECOR_AMBU_PHASES  = 8;    // total phases in cycle

// DEMO_SHOW: rainbow speed (advance hue by 1 per step)
static constexpr uint16_t DECOR_DEMO_STEP_MS = 60;

// DEMO_SHOW real cycle: rotate through every pattern (KITT, KNIGHT_RIDER,
// POLICE_US, AMBULANCE, WARNING_AMBER, HAZARD_RED, solid test colours) and
// then the amber indicator overlays (left, right, hazard) so an operator can
// visually confirm the turn signals work on top of every decorative mode.
static constexpr uint16_t DECOR_DEMO_SUBSTEP_MS = 3000;  // time per demo step
static constexpr uint8_t  DECOR_DEMO_STEP_COUNT = 10;    // number of demo steps
static uint8_t  demoStep    = 0;
static uint32_t demoStepMs  = 0;

// CUSTOM_TEST: cycle through 9 segments every 2 s
static constexpr uint16_t DECOR_TEST_STEP_MS = 2000;
static constexpr uint8_t  DECOR_TEST_PHASES  = 9;

// RGB_DIAG: independent R/G/B-per-strip colour-order diagnostic.
// Isolates exactly ONE strip in exactly ONE pure primary at a time so the
// installer can physically confirm that a commanded RED renders RED (and not
// green/blue) on EACH strip separately — the definitive per-strip byte-order
// check.  6 phases, 2.5 s each:
//   0 FRONT red   1 FRONT green   2 FRONT blue
//   3 REAR  red   4 REAR  green   5 REAR  blue
static constexpr uint16_t DECOR_RGB_DIAG_STEP_MS = 2500;
static constexpr uint8_t  DECOR_RGB_DIAG_PHASES  = 6;

// Scaled brightness helpers (apply scale factor to a CRGB without FastLED API)
static inline CRGB scaledBrightness(CRGB c, uint8_t scale) {
    return CRGB(
        (uint16_t)c.r * scale / 255,
        (uint16_t)c.g * scale / 255,
        (uint16_t)c.b * scale / 255
    );
}

// Fill the left half of a strip with leftCol and the right half with rightCol.
// The split point is the geometric midpoint of the strip, so both halves are
// large, clearly-visible blocks (used by the POLICE_US wig-wag pattern).
static inline void fillHalves(CRGB* leds, int count, CRGB leftCol, CRGB rightCol) {
    if (count <= 0) return;
    const int mid = count / 2;
    for (int i = 0; i < count; ++i)
        leds[i] = (i < mid) ? leftCol : rightCol;
}

// =====================================================================
// Helpers
// =====================================================================

static CRGB blendColor(CRGB c1, CRGB c2, uint8_t amount) {
    return CRGB((c1.r * (255 - amount) + c2.r * amount) / 255,
                (c1.g * (255 - amount) + c2.g * amount) / 255,
                (c1.b * (255 - amount) + c2.b * amount) / 255);
}

static CRGB getThrottleColor(float pct) {
    if (pct < 25.0f)
        return blendColor(CRGB::Red,    CRGB::Orange, static_cast<uint8_t>(pct * 255 / 25));
    if (pct < 50.0f)
        return blendColor(CRGB::Orange, CRGB::Yellow, static_cast<uint8_t>((pct - 25) * 255 / 25));
    if (pct < 75.0f)
        return blendColor(CRGB::Yellow, CRGB::Green,  static_cast<uint8_t>((pct - 50) * 255 / 25));
    return blendColor(CRGB::Green, CRGB::Blue, static_cast<uint8_t>((pct - 75) * 255 / 25));
}

// ---- KITT scanner effect (red or white bounce) ----
static void updateKITTScanner(CRGB* leds, int count, CRGB color) {
    for (int i = 0; i < count; ++i)
        leds[i].fadeToBlackBy(FADE_RATE_KITT);

    if (scannerPos >= 0 && scannerPos < count)
        leds[scannerPos] = color;

    for (int i = 1; i <= KITT_TAIL_LENGTH; ++i) {
        int pos = scannerPos - (i * scannerDirection);
        if (pos >= 0 && pos < count) {
            uint8_t bright = 255 - (i * 255 / KITT_TAIL_LENGTH);
            leds[pos] = color;
            leds[pos].fadeToBlackBy(255 - bright);
        }
    }

    scannerPos += scannerDirection;
    if (scannerPos >= count || scannerPos < 0) {
        scannerDirection = -scannerDirection;
        scannerPos += scannerDirection * 2;
        if (scannerPos < 0) scannerPos = 0;
        if (scannerPos >= count) scannerPos = count - 1;
    }
}

// ---- Chase effect (sequential LED chase) ----
static void updateChase(CRGB* leds, int count, CRGB color, uint8_t speed) {
    if (count <= 0 || speed == 0) return;
    int pos = (animationStep / speed) % count;

    for (int i = 0; i < count; ++i)
        leds[i].fadeToBlackBy(FADE_RATE_CHASE);

    for (int i = 0; i < 5 && pos - i >= 0; ++i) {
        leds[pos - i] = color;
        leds[pos - i].fadeToBlackBy(i * 50);
    }
}

// ---- Rainbow sweep ----
static void updateRainbow(CRGB* leds, int count, uint8_t speed) {
    if (count <= 0) return;
    uint8_t hue      = (animationStep * speed) & 0xFF;
    uint8_t deltaHue = static_cast<uint8_t>(256 / count);
    fill_rainbow(leds, count, hue, deltaHue);
}

// ---- Flash between two colours ----
static void updateFlash(CRGB* leds, int count, CRGB c1, CRGB c2) {
    fill_solid(leds, count, blinkState ? c1 : c2);
}

// ---- Knight Rider / KITT scanner ("coche fantástico") ----
//
// Pure red left↔right scanner with a fading red tail on a black background.
// Used by the decorative DecorMode::KNIGHT_RIDER.  Timing is driven by the
// caller's monotonically-increasing `step` (animationStep, one per ~50 ms
// update), so the effect advances smoothly without any delay() and without
// its own timer state.
//
// Distinct from updateKITTScanner() (which keeps mutable scannerPos state and
// is used for the NORMAL/idle and REVERSE front base): this variant is fully
// stateless and derives the bounce position from `step`, so it can drive the
// front and rear strips independently regardless of their differing lengths.
//
//   Background: black (buffer cleared every frame — no stale pixels).
//   Position  : triangular bounce  0 → count-1 → 0 …
//   Tail      : centre 255, ±1 → 120, ±2 → 50, ±3 → 15 (red only, no HSV).
static void renderKnightRider(CRGB* leds, int count, uint16_t step) {
    if (count <= 0) return;

    // Clear background to black so nothing from previous frames lingers.
    fill_solid(leds, count, CRGB::Black);

    if (count == 1) {
        leds[0] = CRGB(255, 0, 0);
        return;
    }

    // Triangular bounce: pos sweeps 0..count-1 and back, always in range.
    const int period = 2 * (count - 1);
    const int phase  = step % period;
    const int pos    = (phase < count) ? phase : (period - phase);

    // Symmetric red tail brightness profile.
    static constexpr uint8_t kTail[] = {255, 120, 50, 15};
    constexpr int kTailLen = static_cast<int>(sizeof(kTail) / sizeof(kTail[0]));

    for (int off = 0; off < kTailLen; ++off) {
        const CRGB col(kTail[off], 0, 0);  // pure red, never rainbow
        if (off == 0) {
            leds[pos] = col;               // pos is guaranteed 0..count-1
        } else {
            const int lo = pos - off;
            const int hi = pos + off;
            if (lo >= 0 && lo < count) leds[lo] = col;
            if (hi >= 0 && hi < count) leds[hi] = col;
        }
    }
}

// =====================================================================
// Decorative mode — front and rear strip renderers
//
// Called from update() ONLY when currentDecorMode != DecorMode::NORMAL.
// They paint the full strip (including indicator zones); the turn-signal
// overlays (updateFrontTurnSignals / updateRearTurnSignals) run AFTER
// and have absolute priority on the indicator zones.
//
// No delay() anywhere in this section.
// =====================================================================

static void updateDecorativeFront(DecorMode mode) {
    const uint8_t br = LED_BRIGHTNESS_EMERGENCY;
    const uint8_t brDemo = LED_BRIGHTNESS_DEMO;

    switch (mode) {
        case DecorMode::OFF:
            fill_solid(ledsFront, NUM_LEDS_FRONT, CRGB::Black);
            break;

        case DecorMode::POLICE_US: {
            // US wig-wag: left/right HALVES flash opposite red/blue, then swap.
            //   Phase 0: left RED  / right BLUE
            //   Phase 1: blackout   Phase 3: blackout
            //   Phase 2: left BLUE / right RED  (inverted)
            const CRGB red  = scaledBrightness(LED_COLOR_RED_EMERGENCY, br);
            const CRGB blue = scaledBrightness(LED_COLOR_BLUE_POLICE,   br);
            if (decorPhase == 0)
                fillHalves(ledsFront, NUM_LEDS_FRONT, red, blue);
            else if (decorPhase == 2)
                fillHalves(ledsFront, NUM_LEDS_FRONT, blue, red);
            else
                fill_solid(ledsFront, NUM_LEDS_FRONT, CRGB::Black);
            break;
        }

        case DecorMode::AMBULANCE: {
            // Red/white double-flash (whole strip) — distinct from police.
            // Flash on even phases 0,2 (red) and 4,6 (white); off otherwise.
            CRGB col = CRGB::Black;
            if (decorPhase == 0 || decorPhase == 2)
                col = scaledBrightness(LED_COLOR_RED_EMERGENCY, br);
            else if (decorPhase == 4 || decorPhase == 6)
                col = scaledBrightness(LED_COLOR_WHITE, br);
            fill_solid(ledsFront, NUM_LEDS_FRONT, col);
            break;
        }

        case DecorMode::WARNING_AMBER: {
            // Slow amber blink: ON on even phases, OFF on odd
            CRGB col = ((decorPhase & 1) == 0)
                ? scaledBrightness(AMBER, brDemo)
                : CRGB::Black;
            fill_solid(ledsFront, NUM_LEDS_FRONT, col);
            break;
        }

        case DecorMode::HAZARD_RED:
            // Front is dim red or off when hazard rear is flashing
            fill_solid(ledsFront, NUM_LEDS_FRONT, scaledBrightness(LED_COLOR_RED_EMERGENCY, 20));
            break;

        case DecorMode::KNIGHT_RIDER:
            // "Coche fantástico" — red scanner bounce with tail, black background
            renderKnightRider(ledsFront, NUM_LEDS_FRONT, animationStep);
            break;

        case DecorMode::DEMO_SHOW: {
            // Slow rainbow sweep across the full front strip
            uint8_t hue      = static_cast<uint8_t>(decorPhase * 4);
            uint8_t deltaHue = static_cast<uint8_t>(256 / NUM_LEDS_FRONT);
            fill_rainbow(ledsFront, NUM_LEDS_FRONT, hue, deltaHue);
            // Apply demo brightness globally
            for (int i = 0; i < NUM_LEDS_FRONT; ++i)
                ledsFront[i] = scaledBrightness(ledsFront[i], brDemo);
            break;
        }

        case DecorMode::CUSTOM_TEST: {
            // Cycle through segments to verify wiring
            fill_solid(ledsFront, NUM_LEDS_FRONT, CRGB::Black);
            switch (decorPhase % DECOR_TEST_PHASES) {
                case 0: // Front-left zone — white
                    fill_solid(&ledsFront[FRONT_IND_LEFT_START], FRONT_IND_LEFT_COUNT,
                               scaledBrightness(LED_COLOR_WHITE, br));
                    break;
                case 1: // Front-right zone — white
                    fill_solid(&ledsFront[FRONT_IND_RIGHT_START], FRONT_IND_RIGHT_COUNT,
                               scaledBrightness(LED_COLOR_WHITE, br));
                    break;
                case 2: // All front — red
                    fill_solid(ledsFront, NUM_LEDS_FRONT, scaledBrightness(LED_COLOR_RED_EMERGENCY, br));
                    break;
                case 3: // All front — blue
                    fill_solid(ledsFront, NUM_LEDS_FRONT, scaledBrightness(LED_COLOR_BLUE_POLICE, br));
                    break;
                case 4: // All front — white
                    fill_solid(ledsFront, NUM_LEDS_FRONT, scaledBrightness(LED_COLOR_WHITE, br));
                    break;
                case 5: // All front — amber
                    fill_solid(ledsFront, NUM_LEDS_FRONT, scaledBrightness(AMBER, br));
                    break;
                case 6: // All front — green (full-strip, for colour-order /
                        // first-LED / dead-pixel diagnosis on every LED)
                    fill_solid(ledsFront, NUM_LEDS_FRONT, scaledBrightness(CRGB::Green, br));
                    break;
                case 7: // All front — rainbow stripe
                    fill_rainbow(ledsFront, NUM_LEDS_FRONT, 0, 10);
                    break;
                case 8: // All off
                    break;
            }
            break;
        }

        case DecorMode::RGB_DIAG: {
            // Front strip is lit ONLY during its own phases (0-2); it stays
            // black while the rear strip is under test (phases 3-5) so the two
            // strips are never energised together and can be judged in
            // isolation.  Pure primaries at full brightness — no scaling, no
            // blending — so any deviation is a genuine byte-order/wiring fault.
            fill_solid(ledsFront, NUM_LEDS_FRONT, CRGB::Black);
            switch (decorPhase % DECOR_RGB_DIAG_PHASES) {
                case 0: fill_solid(ledsFront, NUM_LEDS_FRONT, CRGB(255, 0, 0)); break; // FRONT red
                case 1: fill_solid(ledsFront, NUM_LEDS_FRONT, CRGB(0, 255, 0)); break; // FRONT green
                case 2: fill_solid(ledsFront, NUM_LEDS_FRONT, CRGB(0, 0, 255)); break; // FRONT blue
                default: break;  // phases 3-5: rear under test, front dark
            }
            break;
        }

        default:
            fill_solid(ledsFront, NUM_LEDS_FRONT, CRGB::Black);
            break;
    }
}

static void updateDecorativeRear(DecorMode mode) {
    const uint8_t br = LED_BRIGHTNESS_EMERGENCY;
    const uint8_t brDemo = LED_BRIGHTNESS_DEMO;

    switch (mode) {
        case DecorMode::OFF:
            fill_solid(ledsRear, NUM_LEDS_REAR, CRGB::Black);
            break;

        case DecorMode::POLICE_US: {
            // Rear wig-wag mirrors the front but swaps which half starts red,
            // so front and rear are visually offset (classic cross-flash).
            //   Phase 0: left BLUE / right RED
            //   Phase 2: left RED  / right BLUE
            const CRGB red  = scaledBrightness(LED_COLOR_RED_EMERGENCY, br);
            const CRGB blue = scaledBrightness(LED_COLOR_BLUE_POLICE,   br);
            if (decorPhase == 0)
                fillHalves(ledsRear, NUM_LEDS_REAR, blue, red);
            else if (decorPhase == 2)
                fillHalves(ledsRear, NUM_LEDS_REAR, red, blue);
            else
                fill_solid(ledsRear, NUM_LEDS_REAR, CRGB::Black);
            break;
        }

        case DecorMode::AMBULANCE: {
            // Rear red/white double-flash — same timing as front.
            CRGB col = CRGB::Black;
            if (decorPhase == 0 || decorPhase == 2)
                col = scaledBrightness(LED_COLOR_RED_EMERGENCY, br);
            else if (decorPhase == 4 || decorPhase == 6)
                col = scaledBrightness(LED_COLOR_WHITE, br);
            fill_solid(ledsRear, NUM_LEDS_REAR, col);
            break;
        }

        case DecorMode::WARNING_AMBER: {
            // Left/right amber alternate (left on even, right on odd)
            fill_solid(ledsRear, NUM_LEDS_REAR, CRGB::Black);
            if ((decorPhase & 1) == 0)
                fill_solid(&ledsRear[REAR_IND_LEFT_START], REAR_IND_LEFT_COUNT,
                           scaledBrightness(AMBER, brDemo));
            else
                fill_solid(&ledsRear[REAR_IND_RIGHT_START], REAR_IND_RIGHT_COUNT,
                           scaledBrightness(AMBER, brDemo));
            break;
        }

        case DecorMode::HAZARD_RED: {
            // Double flash rear red: ON at phases 0 and 2, OFF elsewhere
            bool on = (decorPhase == 0 || decorPhase == 2);
            CRGB col = on ? scaledBrightness(LED_COLOR_RED_EMERGENCY, br) : CRGB::Black;
            fill_solid(ledsRear, NUM_LEDS_REAR, col);
            break;
        }

        case DecorMode::KNIGHT_RIDER:
            // "Coche fantástico" — red scanner bounce with tail, black background
            renderKnightRider(ledsRear, NUM_LEDS_REAR, animationStep);
            break;

        case DecorMode::DEMO_SHOW: {
            // Offset rainbow by half the hue cycle for a mirror effect
            uint8_t hue      = static_cast<uint8_t>(decorPhase * 4 + 128);
            uint8_t deltaHue = static_cast<uint8_t>(256 / NUM_LEDS_REAR);
            fill_rainbow(ledsRear, NUM_LEDS_REAR, hue, deltaHue);
            for (int i = 0; i < NUM_LEDS_REAR; ++i)
                ledsRear[i] = scaledBrightness(ledsRear[i], brDemo);
            break;
        }

        case DecorMode::CUSTOM_TEST: {
            fill_solid(ledsRear, NUM_LEDS_REAR, CRGB::Black);
            switch (decorPhase % DECOR_TEST_PHASES) {
                case 0: // Rear-left zone — white
                    fill_solid(&ledsRear[REAR_IND_LEFT_START], REAR_IND_LEFT_COUNT,
                               scaledBrightness(LED_COLOR_WHITE, br));
                    break;
                case 1: // Rear-right zone — white
                    fill_solid(&ledsRear[REAR_IND_RIGHT_START], REAR_IND_RIGHT_COUNT,
                               scaledBrightness(LED_COLOR_WHITE, br));
                    break;
                case 2: // All rear — red
                    fill_solid(ledsRear, NUM_LEDS_REAR, scaledBrightness(LED_COLOR_RED_EMERGENCY, br));
                    break;
                case 3: // All rear — blue
                    fill_solid(ledsRear, NUM_LEDS_REAR, scaledBrightness(LED_COLOR_BLUE_POLICE, br));
                    break;
                case 4: // All rear — white
                    fill_solid(ledsRear, NUM_LEDS_REAR, scaledBrightness(LED_COLOR_WHITE, br));
                    break;
                case 5: // All rear — amber
                    fill_solid(ledsRear, NUM_LEDS_REAR, scaledBrightness(AMBER, br));
                    break;
                case 6: // All rear — green (full-strip, for colour-order /
                        // first-LED / dead-pixel diagnosis on every LED)
                    fill_solid(ledsRear, NUM_LEDS_REAR, scaledBrightness(CRGB::Green, br));
                    break;
                case 7: // All rear — rainbow stripe
                    fill_rainbow(ledsRear, NUM_LEDS_REAR, 128, 10);
                    break;
                case 8: // All off
                    break;
            }
            break;
        }

        case DecorMode::RGB_DIAG: {
            // Rear strip is lit ONLY during its own phases (3-5); it stays
            // black while the front strip is under test (phases 0-2).  Pure
            // primaries at full brightness — no scaling, no blending.
            fill_solid(ledsRear, NUM_LEDS_REAR, CRGB::Black);
            switch (decorPhase % DECOR_RGB_DIAG_PHASES) {
                case 3: fill_solid(ledsRear, NUM_LEDS_REAR, CRGB(255, 0, 0)); break; // REAR red
                case 4: fill_solid(ledsRear, NUM_LEDS_REAR, CRGB(0, 255, 0)); break; // REAR green
                case 5: fill_solid(ledsRear, NUM_LEDS_REAR, CRGB(0, 0, 255)); break; // REAR blue
                default: break;  // phases 0-2: front under test, rear dark
            }
            break;
        }

        default:
            fill_solid(ledsRear, NUM_LEDS_REAR, CRGB::Black);
            break;
    }
}
// Advance the shared decorative phase counter.
// Each mode has its own step period and wrap count.
static void advanceDecorPhase(uint32_t now, DecorMode mode) {
    uint16_t stepMs  = DECOR_POLICE_STEP_MS;
    uint8_t  wrapAt  = DECOR_POLICE_PHASES;

    switch (mode) {
        case DecorMode::POLICE_US:
            stepMs = DECOR_POLICE_STEP_MS;
            wrapAt = DECOR_POLICE_PHASES;
            break;
        case DecorMode::HAZARD_RED:
            stepMs = DECOR_HAZARD_STEP_MS;
            wrapAt = DECOR_HAZARD_PHASES;
            break;
        case DecorMode::WARNING_AMBER:
            stepMs = DECOR_WARN_STEP_MS;
            wrapAt = 2;
            break;
        case DecorMode::AMBULANCE:
            stepMs = DECOR_AMBU_STEP_MS;
            wrapAt = DECOR_AMBU_PHASES;
            break;
        case DecorMode::DEMO_SHOW:
            stepMs = DECOR_DEMO_STEP_MS;
            wrapAt = 64;   // arbitrary wrap — drives hue evolution
            break;
        case DecorMode::CUSTOM_TEST:
            stepMs = DECOR_TEST_STEP_MS;
            wrapAt = DECOR_TEST_PHASES;
            break;
        case DecorMode::RGB_DIAG:
            stepMs = DECOR_RGB_DIAG_STEP_MS;
            wrapAt = DECOR_RGB_DIAG_PHASES;
            break;
        default:
            return;  // NORMAL / OFF: no phase counter needed
    }

    if (now - decorLastMs >= stepMs) {
        decorLastMs = now;
        decorPhase  = (decorPhase + 1) % wrapAt;
    }
}

// =====================================================================
// DEMO_SHOW — full pattern walkthrough
//
// Instead of a single rainbow sweep, DEMO_SHOW rotates through every
// decorative pattern and then exercises the amber indicator overlays so
// the turn signals can be verified on top of a live decorative base.
//
// Steps (DECOR_DEMO_SUBSTEP_MS each):
//   0 NORMAL / KITT red   1 KNIGHT_RIDER      2 POLICE_US
//   3 AMBULANCE           4 WARNING_AMBER     5 HAZARD_RED
//   6 CUSTOM_TEST (solid colours)
//   7 left indicator      8 right indicator   9 hazard
//
// For the indicator steps a KNIGHT_RIDER base is rendered and the amber
// overlay is requested via demoTurnOverride; effectiveTurnSignal() then
// drives the same overlay path used in normal operation.  The real turn
// signal always keeps priority (see effectiveTurnSignal()).
//
// No delay(); timing via millis()/now only.
// =====================================================================
static void updateDemoShow(uint32_t now) {
    // Advance to the next demo step on the sub-step timer, resetting the
    // per-pattern phase so each pattern starts cleanly (no stale phase).
    if (now - demoStepMs >= DECOR_DEMO_SUBSTEP_MS) {
        demoStepMs = now;
        demoStep   = (demoStep + 1) % DECOR_DEMO_STEP_COUNT;
        decorPhase = 0;
        decorLastMs = now;
    }

    // Default: no forced indicator this step.
    demoTurnOverride = TurnSignal::OFF;

    // Step 0 shows the NORMAL base: KITT red scanner front, dim red rear —
    // exactly what the driver sees in day-to-day NORMAL mode.
    if (demoStep == 0) {
        renderKnightRider(ledsFront, NUM_LEDS_FRONT, animationStep);
        const CRGB tail = scaledBrightness(LED_COLOR_RED_EMERGENCY, BRIGHTNESS_POSITION);
        fill_solid(ledsRear, NUM_LEDS_REAR, tail);
        return;
    }

    // Map the remaining steps to an effective decorative pattern plus an
    // optional forced indicator overlay.
    DecorMode eff = DecorMode::KNIGHT_RIDER;
    switch (demoStep) {
        case 1: eff = DecorMode::KNIGHT_RIDER;  break;
        case 2: eff = DecorMode::POLICE_US;     break;
        case 3: eff = DecorMode::AMBULANCE;     break;
        case 4: eff = DecorMode::WARNING_AMBER; break;
        case 5: eff = DecorMode::HAZARD_RED;    break;
        case 6: eff = DecorMode::CUSTOM_TEST;   break;
        case 7: eff = DecorMode::KNIGHT_RIDER;  demoTurnOverride = TurnSignal::LEFT;   break;
        case 8: eff = DecorMode::KNIGHT_RIDER;  demoTurnOverride = TurnSignal::RIGHT;  break;
        case 9: eff = DecorMode::KNIGHT_RIDER;  demoTurnOverride = TurnSignal::HAZARD; break;
        default: eff = DecorMode::KNIGHT_RIDER; break;
    }

    advanceDecorPhase(now, eff);
    updateDecorativeFront(eff);
    updateDecorativeRear(eff);
}

// =====================================================================
// Front LED pattern dispatch
//
// ZONE NOTE (Task 4 — KITT zone isolation):
//   Effects render across the full 70-LED strip to provide full-bar KITT
//   when no turn signal is active.  Side zones [0–9] and [60–69] are
//   conditionally overridden by updateFrontTurnSignals() (called next).
//   The overlay writes ONLY during blink-ON, so KITT remains the visual
//   base during blink-OFF — no explicit zone clipping is needed here.
// =====================================================================

static void updateFrontLEDs() {
    switch (currentFrontMode) {
        case FrontMode::OFF:
            fill_solid(ledsFront, NUM_LEDS_FRONT, CRGB::Black);
            break;
        case FrontMode::KITT_IDLE:
            updateKITTScanner(ledsFront, NUM_LEDS_FRONT, CRGB::Red);
            break;
        case FrontMode::ACCEL_LOW:
            updateChase(ledsFront, NUM_LEDS_FRONT, getThrottleColor(12.5f), CHASE_SPEED_LOW);
            break;
        case FrontMode::ACCEL_MED:
            updateChase(ledsFront, NUM_LEDS_FRONT, getThrottleColor(37.5f), CHASE_SPEED_MED);
            break;
        case FrontMode::ACCEL_HIGH:
            updateChase(ledsFront, NUM_LEDS_FRONT, getThrottleColor(62.5f), CHASE_SPEED_HIGH);
            break;
        case FrontMode::ACCEL_MAX:
            updateRainbow(ledsFront, NUM_LEDS_FRONT, RAINBOW_SPEED);
            break;
        case FrontMode::REVERSE:
            updateKITTScanner(ledsFront, NUM_LEDS_FRONT, CRGB::White);
            break;
        case FrontMode::ABS_ALERT:
            updateFlash(ledsFront, NUM_LEDS_FRONT, CRGB::Red, CRGB::White);
            break;
        case FrontMode::TCS_ALERT:
            updateFlash(ledsFront, NUM_LEDS_FRONT, CRGB::Orange, CRGB::Black);
            break;
    }
}

// =====================================================================
// Rear base layer (ALL 72 LEDs)
//
// Paints the entire rear strip with the current rear mode (position,
// brake, reverse, regen, etc.).  This provides a visible base on the
// side zones [0–9] and [62–71] so that the turn-signal overlay can be
// non-destructive — matching the front strip's KITT-under-blink design.
//
// Without this full-strip base, side zones would be black whenever no
// turn signal is active, wasting 20 LEDs that could show tail/brake.
// =====================================================================

static void updateRearBase() {
    CRGB col = CRGB::Black;
    uint8_t bright = BRIGHTNESS_FULL;

    switch (currentRearMode) {
        case RearMode::OFF:
            break;
        case RearMode::POSITION:
            col = CRGB::Red;
            bright = BRIGHTNESS_POSITION;
            break;
        case RearMode::BRAKE:
            col = CRGB::Red;
            break;
        case RearMode::BRAKE_EMERGENCY:
            col = blinkState ? CRGB::Red : CRGB::Black;
            break;
        case RearMode::REVERSE:
            col = CRGB::White;
            break;
        case RearMode::REGEN_ACTIVE: {
            // Blue pulsing — simple sine approximation via animationStep
            uint8_t phase = static_cast<uint8_t>((animationStep * 5) & 0xFF);
            bright = 128 + (phase < 128 ? phase : 255 - phase);
            col = CRGB::Blue;
            break;
        }
    }

    for (int i = 0; i < NUM_LEDS_REAR; ++i) {
        ledsRear[i] = col;
        ledsRear[i].fadeToBlackBy(255 - bright);
    }
}

// =====================================================================
// Front turn-signal zones (LEDs 0–9, 60–69)
//
// OVERLAY PRIORITY:
//   1. updateFrontLEDs()        — renders KITT / base effect on full strip
//   2. updateFrontTurnSignals() — overlays turn signals on side zones ONLY
//
// ZONE CONTRACT (Task 4 — KITT zone isolation):
//   KITT / base effects render across the full 70-LED strip.  When no turn
//   signal is active the full-bar KITT ("coche fantástico") is visible.
//   When a turn signal is active, this overlay writes AMBER over the side
//   zones during blink-ON only.  During blink-OFF the overlay does NOT
//   write, so the underlying KITT fade remains visible — eliminating the
//   hard BLACK cut that previously caused flicker.
//
// SAFE MODE (Task 3):
//   TurnSignal::HAZARD (set by main.cpp on SAFE/ERROR state) forces both
//   hazardOverride = true, activating both sides regardless of steering.
// =====================================================================

static void updateFrontTurnSignals() {
    // ---- Persistent turn-indicator state (Task 1 — hysteresis hardening) ----
    // These static flags provide frame-stable tracking of which indicators
    // are logically active.  The underlying currentTurnSignal is already
    // debounced by main.cpp (15°/10° hysteresis + 100 ms persistence filter),
    // so these flags serve as an explicit, documented record of active state
    // that persists across frames and prevents any residual visual toggling.
    static bool turnLeftActive  = false;
    static bool turnRightActive = false;

    // ---- SAFE MODE hazard override (Task 3) ----
    // When TurnSignal::HAZARD is set (SAFE/ERROR state from main.cpp),
    // both sides are forced ON regardless of steering angle input.
    // This is independent of the steering hysteresis — it's an absolute override.
    const bool hazardOverride = (effectiveTurnSignal() == TurnSignal::HAZARD);
    turnLeftActive  = hazardOverride || (effectiveTurnSignal() == TurnSignal::LEFT);
    turnRightActive = hazardOverride || (effectiveTurnSignal() == TurnSignal::RIGHT);

    // No turn signals active — KITT / base effect fills the full strip unmodified
    if (!turnLeftActive && !turnRightActive) return;

    // ---- Overlay: AMBER only during blink-ON phase (Task 2) ----
    // When blinkState == true  → write AMBER to active indicator zones.
    // When blinkState == false → do NOT write anything.
    //   This allows the underlying KITT / base effect to remain visible
    //   during the OFF phase, eliminating the visual glitch caused by
    //   writing BLACK (which would create a hard cut-to-black flicker).
    if (blinkState) {
        if (turnLeftActive) {
            fill_solid(&ledsFront[FRONT_IND_LEFT_START], FRONT_IND_LEFT_COUNT, AMBER);
        }
        if (turnRightActive) {
            fill_solid(&ledsFront[FRONT_IND_RIGHT_START], FRONT_IND_RIGHT_COUNT, AMBER);
        }
    }
}

// =====================================================================
// Rear turn-signal zones (LEDs 0–9, 62–71)
//
// OVERLAY PRIORITY:
//   1. updateRearBase()          — renders tail/brake/reverse on ALL 72 LEDs
//   2. updateRearTurnSignals()   — overlays turn signals on side zones ONLY
//
// ZONE CONTRACT (matching front strip design):
//   The rear base paints all 72 LEDs with the current mode (position,
//   brake, etc.).  When a turn signal is active, this overlay writes
//   AMBER over the side zones during blink-ON only.  During blink-OFF
//   the overlay does NOT write, so the underlying base (dim red, brake
//   red, white reverse, etc.) remains visible — no hard black cut.
//
// SEQUENTIAL ANIMATION:
//   During blink-ON, LEDs fill progressively outward from centre:
//     Left  [0–9]:   LED 9 → 8 → … → 0    (centre-adjacent first)
//     Right [62–71]: LED 62 → 63 → … → 71 (centre-adjacent first)
//   The 500 ms blink-ON window is divided into 10 equal steps (~50 ms
//   each).  Each step lights one additional LED.
//
// SAFE MODE:
//   TurnSignal::HAZARD forces both sides — same as front.
// =====================================================================

/// Sequential sweep helper: fills LEDs outward from centre.
/// Determines sweep direction from zone position relative to centre.
static void sweepFill(int zoneStart, int zoneCount, uint8_t step, CRGB color) {
    if (zoneStart < REAR_CENTRE_START) {
        // Zone is physically left of centre → sweep outward = high index to low
        for (int i = 0; i <= step && i < zoneCount; ++i)
            ledsRear[zoneStart + zoneCount - 1 - i] = color;
    } else {
        // Zone is physically right of centre → sweep outward = low index to high
        for (int i = 0; i <= step && i < zoneCount; ++i)
            ledsRear[zoneStart + i] = color;
    }
}

static void updateRearTurnSignals() {
    const TurnSignal ts = effectiveTurnSignal();
    bool leftOn  = (ts == TurnSignal::LEFT  || ts == TurnSignal::HAZARD);
    bool rightOn = (ts == TurnSignal::RIGHT || ts == TurnSignal::HAZARD);

    // No turn signals active — base effect fills the full strip unmodified
    if (!leftOn && !rightOn) return;

    // Only overlay during blink-ON (non-destructive, like front)
    if (!blinkState) return;

    // Sequential sweep: divide the 500 ms blink-ON window into N steps
    // (one per LED in the zone).  Each step lights one additional LED
    // outward from centre.
    uint32_t elapsed = millis() - lastBlinkMs;
    // Clamp elapsed to blink period to avoid overflow after period end
    if (elapsed > TURN_SIGNAL_BLINK_MS) elapsed = TURN_SIGNAL_BLINK_MS;

    if (leftOn) {
        uint8_t step = static_cast<uint8_t>(elapsed * REAR_IND_LEFT_COUNT / TURN_SIGNAL_BLINK_MS);
        if (step >= REAR_IND_LEFT_COUNT) step = REAR_IND_LEFT_COUNT - 1;
        sweepFill(REAR_IND_LEFT_START, REAR_IND_LEFT_COUNT, step, AMBER);
    }

    if (rightOn) {
        uint8_t step = static_cast<uint8_t>(elapsed * REAR_IND_RIGHT_COUNT / TURN_SIGNAL_BLINK_MS);
        if (step >= REAR_IND_RIGHT_COUNT) step = REAR_IND_RIGHT_COUNT - 1;
        sweepFill(REAR_IND_RIGHT_START, REAR_IND_RIGHT_COUNT, step, AMBER);
    }
}

// =====================================================================
// Public API
// =====================================================================

void init() {
    // NOTE: the two strips use DIFFERENT byte orders on this hardware.
    // The rear strip is genuine WS2812B (GRB). The front strip's controllers
    // expect RGB order — configuring it as GRB swapped its red/green channels,
    // so the red KITT scan and POLICE_US red flashes appeared GREEN. Sending
    // RGB to the front makes logical red=red, green=green, blue=blue.
    FastLED.addLeds<WS2812B, LED_FRONT_PIN, RGB>(ledsFront, NUM_LEDS_FRONT);
    FastLED.addLeds<WS2812B, LED_REAR_PIN,  GRB>(ledsRear,  NUM_LEDS_REAR);
    FastLED.setBrightness(200);
    FastLED.clear(true);
    lastUpdateMs = millis();
    initialized  = true;
}

void update() {
    if (!initialized || !enabled) return;

    uint32_t now = millis();

    // ---- Emergency flash has highest priority ----
    if (emergencyActive) {
        if (now - emergencyLastMs >= EMERGENCY_FLASH_MS) {
            emergencyLastMs = now;
            emergencyPhase  = !emergencyPhase;

            if (emergencyPhase) {
                fill_solid(ledsFront, NUM_LEDS_FRONT, CRGB::Red);
                fill_solid(ledsRear,  NUM_LEDS_REAR,  CRGB::Red);
            } else {
                fill_solid(ledsFront, NUM_LEDS_FRONT, CRGB::Black);
                fill_solid(ledsRear,  NUM_LEDS_REAR,  CRGB::Black);
                ++emergencyCurrent;
                if (emergencyCurrent >= emergencyCount) {
                    emergencyActive  = false;
                    emergencyCurrent = 0;
                }
            }
            FastLED.show();
        }
        return;
    }

    // ---- Rate-limited normal update ----
    if (now - lastUpdateMs < UPDATE_RATE_MS) return;
    lastUpdateMs = now;

    // Keep this advancing before the decor branch: REGEN_ACTIVE pulsing uses
    // animationStep even when rendered as a functional rear overlay.
    animationStep++;  // natural uint16_t wrap at 65536

    // Turn-signal blink timer (500 ms half-period)
    if (now - lastBlinkMs >= TURN_SIGNAL_BLINK_MS) {
        blinkState  = !blinkState;
        lastBlinkMs = now;
    }

    // ---- Base layer: normal KITT/throttle OR decorative mode ----
    // Decorative modes (POLICE, AMBULANCE, etc.) replace the normal base.
    // Turn-signal overlays always run AFTER and have unconditional priority
    // on the indicator zones — this is the key safety requirement.
    //
    // SAFETY LAYER: after any decorative rear render, safety-critical rear
    // signals (BRAKE, BRAKE_EMERGENCY, REVERSE, REGEN_ACTIVE) are always
    // composited on top so they can NEVER be hidden by a decorative mode.
    const auto isRearFunctionalOverlay = [](RearMode mode) {
        switch (mode) {
            case RearMode::BRAKE:
            case RearMode::BRAKE_EMERGENCY:
            case RearMode::REVERSE:
            case RearMode::REGEN_ACTIVE:
                return true;
            case RearMode::OFF:
            case RearMode::POSITION:
                return false;
        }
        return false;
    };

    if (currentDecorMode == DecorMode::NORMAL) {
        updateFrontLEDs();
        updateRearBase();
    } else if (currentDecorMode == DecorMode::DEMO_SHOW) {
        // DEMO walks through every pattern + indicator overlay (real demo).
        updateDemoShow(now);
        // Overlay safety-critical rear states — priority beats the demo too
        if (isRearFunctionalOverlay(currentRearMode)) {
            updateRearBase();
        }
    } else {
        advanceDecorPhase(now, currentDecorMode);
        updateDecorativeFront(currentDecorMode);
        updateDecorativeRear(currentDecorMode);
        // Overlay safety-critical rear states — priority beats ALL decor modes
        if (isRearFunctionalOverlay(currentRearMode)) {
            updateRearBase();
        }
    }

    // Turn-signal overlays — ALWAYS run regardless of decor mode ----
    // These write AMBER over the indicator zones (side 10 LEDs each) on
    // blink-ON only.  No decorative mode may skip or suppress these calls.
    updateFrontTurnSignals();
    updateRearTurnSignals();

    FastLED.show();
}

void setFrontMode(FrontMode mode) {
    if (currentFrontMode != mode) {
        currentFrontMode = mode;
        animationStep    = 0;
        scannerPos       = 0;
        scannerDirection = 1;
        // Clear the front buffer so the KITT/chase fade renderers never drag
        // stale pixels (old rainbow/alert colours) from the previous mode.
        if (initialized) fill_solid(ledsFront, NUM_LEDS_FRONT, CRGB::Black);
    }
}

void setFrontFromThrottle(float throttlePercent) {
    if (throttlePercent < 1.0f)        setFrontMode(FrontMode::KITT_IDLE);
    else if (throttlePercent < 25.0f)  setFrontMode(FrontMode::ACCEL_LOW);
    else if (throttlePercent < 50.0f)  setFrontMode(FrontMode::ACCEL_MED);
    else if (throttlePercent < 75.0f)  setFrontMode(FrontMode::ACCEL_HIGH);
    else                                setFrontMode(FrontMode::ACCEL_MAX);
}

void setRearMode(RearMode mode) {
    currentRearMode = mode;
}

TurnSignal getTurnSignal() {
    return currentTurnSignal;
}

void setTurnSignal(TurnSignal signal) {
    currentTurnSignal = signal;
}

void setEnabled(bool en) {
    enabled = en;
    if (!enabled) {
        FastLED.clear(true);
    }
}

void startEmergencyFlash(uint8_t count) {
    emergencyActive  = true;
    emergencyCount   = count;
    emergencyCurrent = 0;
    emergencyPhase   = false;
    emergencyLastMs  = millis();
}

bool isEmergencyFlashActive() {
    return emergencyActive;
}

void setDecorMode(DecorMode mode) {
    // Defensive range check: reject out-of-range values (e.g. from corrupt NVS cast)
    if (static_cast<uint8_t>(mode) >= DECOR_MODE_COUNT) {
        mode = DecorMode::NORMAL;
        Serial.printf("[LED] setDecorMode: invalid value clamped to NORMAL\n");
    }
    if (currentDecorMode != mode) {
        currentDecorMode = mode;
        decorPhase   = 0;
        decorLastMs  = millis();
        demoStep     = 0;
        demoStepMs   = millis();
        demoTurnOverride = TurnSignal::OFF;
        // Wipe both strips on every mode change so no stale rainbow/demo/alert
        // pixels linger under the new pattern (fade-based renderers otherwise
        // drag old colours for several frames).
        if (initialized) {
            fill_solid(ledsFront, NUM_LEDS_FRONT, CRGB::Black);
            fill_solid(ledsRear,  NUM_LEDS_REAR,  CRGB::Black);
        }
        Serial.printf("[LED] DecorMode set to %u\n", static_cast<uint8_t>(mode));
    }
}

DecorMode getDecorMode() {
    return currentDecorMode;
}

const char* getRgbDiagLabel() {
    if (currentDecorMode != DecorMode::RGB_DIAG) return "";
    switch (decorPhase % DECOR_RGB_DIAG_PHASES) {
        case 0: return "FRONT RED";
        case 1: return "FRONT GREEN";
        case 2: return "FRONT BLUE";
        case 3: return "REAR RED";
        case 4: return "REAR GREEN";
        case 5: return "REAR BLUE";
        default: return "";
    }
}

} // namespace led_ctrl
