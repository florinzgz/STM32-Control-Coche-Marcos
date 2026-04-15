// =============================================================================
// ESP32-S3 HMI — LED Controller Implementation (WS2812B via FastLED)
//
// Two independent strips driven by separate GPIOs:
//   Front (GPIO 47, 28 LEDs): KITT scanner, throttle-reactive chase, alerts
//   Rear  (GPIO 48, 16 LEDs): tail, brake, reverse, turn signals, regen
//
// Rear LED zone mapping:
//   [0–2]   Left turn-signal indicator
//   [3–12]  Centre: position / brake / reverse / regen
//   [13–15] Right turn-signal indicator
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

// ---- Amber colour for indicators ----
static constexpr CRGB AMBER = CRGB(255, 100, 0);

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

// =====================================================================
// Front LED pattern dispatch
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
// Rear centre zone (LEDs 3–12)
// =====================================================================

static void updateRearCentre() {
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

    for (int i = REAR_CENTRE_START; i < REAR_CENTRE_START + REAR_CENTRE_COUNT; ++i) {
        ledsRear[i] = col;
        ledsRear[i].fadeToBlackBy(255 - bright);
    }
}

// =====================================================================
// Rear turn-signal zones (LEDs 0–2, 13–15)
// =====================================================================

static void updateTurnSignals() {
    bool leftOn  = (currentTurnSignal == TurnSignal::LEFT  || currentTurnSignal == TurnSignal::HAZARD);
    bool rightOn = (currentTurnSignal == TurnSignal::RIGHT || currentTurnSignal == TurnSignal::HAZARD);

    // Left indicator (LEDs 0-2)
    if (leftOn) {
        if (currentRearMode == RearMode::REVERSE && blinkState) {
            // Sequential chase in reverse
            int pos = (animationStep / 10) % REAR_IND_LEFT_COUNT;
            for (int i = 0; i < REAR_IND_LEFT_COUNT; ++i)
                ledsRear[REAR_IND_LEFT_START + i] = (i == pos) ? AMBER : CRGB::Black;
        } else {
            fill_solid(&ledsRear[REAR_IND_LEFT_START], REAR_IND_LEFT_COUNT,
                       blinkState ? AMBER : CRGB::Black);
        }
    } else {
        fill_solid(&ledsRear[REAR_IND_LEFT_START], REAR_IND_LEFT_COUNT, CRGB::Black);
    }

    // Right indicator (LEDs 13-15)
    if (rightOn) {
        if (currentRearMode == RearMode::REVERSE && blinkState) {
            int pos = (animationStep / 10) % REAR_IND_RIGHT_COUNT;
            for (int i = 0; i < REAR_IND_RIGHT_COUNT; ++i)
                ledsRear[REAR_IND_RIGHT_START + i] = (i == pos) ? AMBER : CRGB::Black;
        } else {
            fill_solid(&ledsRear[REAR_IND_RIGHT_START], REAR_IND_RIGHT_COUNT,
                       blinkState ? AMBER : CRGB::Black);
        }
    } else {
        fill_solid(&ledsRear[REAR_IND_RIGHT_START], REAR_IND_RIGHT_COUNT, CRGB::Black);
    }
}

// =====================================================================
// Public API
// =====================================================================

void init() {
    FastLED.addLeds<WS2812B, LED_FRONT_PIN, GRB>(ledsFront, NUM_LEDS_FRONT);
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

    animationStep++;  // natural uint16_t wrap at 65536

    // Turn-signal blink timer (500 ms half-period)
    if (now - lastBlinkMs >= TURN_SIGNAL_BLINK_MS) {
        blinkState  = !blinkState;
        lastBlinkMs = now;
    }

    updateFrontLEDs();
    updateRearCentre();
    updateTurnSignals();

    FastLED.show();
}

void setFrontMode(FrontMode mode) {
    if (currentFrontMode != mode) {
        currentFrontMode = mode;
        animationStep    = 0;
        scannerPos       = 0;
        scannerDirection = 1;
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

} // namespace led_ctrl
