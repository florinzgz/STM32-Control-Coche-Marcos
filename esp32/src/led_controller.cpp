// =============================================================================
// ESP32-S3 HMI — LED Controller Implementation (WS2812B via FastLED)
//
// Pattern mapping:
//   ACTIVE:    Front = white position lights, Rear = red tail lights
//   Braking:   Rear = bright red (full intensity)
//   Reverse:   Rear = white backup lights
//   SAFE/ERROR: All LEDs flash amber at 2 Hz (250 ms on/off)
//   LIMP_HOME: Front = dim yellow, Rear = dim red
//   OFF/STANDBY: All LEDs dark
//
// FastLED.show() is called once per update() invocation (~20 Hz).
// Non-blocking: no delay() calls, pattern state tracked via millis().
// =============================================================================

#include "led_controller.h"
#include <FastLED.h>

namespace led_ctrl {

static CRGB leds[NUM_LEDS_TOTAL];
static bool initialized = false;

// Flash state for emergency patterns
static bool flashPhase = false;
static unsigned long lastFlashMs = 0;
static constexpr unsigned long FLASH_INTERVAL_MS = 250;

void init() {
    FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS_TOTAL);
    FastLED.setBrightness(128);  // 50% default brightness
    FastLED.clear(true);
    initialized = true;
}

void update(uint8_t systemState, bool braking, bool reverse, bool enabled) {
    if (!initialized) return;

    // If LED relay is OFF or system is in BOOT (0) / STANDBY (1), keep all dark
    if (!enabled || systemState <= static_cast<uint8_t>(1)) {
        FastLED.clear(true);
        return;
    }

    unsigned long now = millis();

    // Flash timer for emergency patterns
    if (now - lastFlashMs >= FLASH_INTERVAL_MS) {
        lastFlashMs = now;
        flashPhase = !flashPhase;
    }

    // SAFE (4) or ERROR (5) → amber emergency flash
    if (systemState == 4 || systemState == 5) {
        CRGB col = flashPhase ? CRGB(255, 140, 0) : CRGB::Black;
        fill_solid(leds, NUM_LEDS_TOTAL, col);
        FastLED.show();
        return;
    }

    // LIMP_HOME (6) → dim warning
    if (systemState == 6) {
        fill_solid(leds, NUM_LEDS_FRONT, CRGB(60, 50, 0));           // Front: dim yellow
        fill_solid(leds + NUM_LEDS_FRONT, NUM_LEDS_REAR, CRGB(60, 0, 0)); // Rear: dim red
        FastLED.show();
        return;
    }

    // ACTIVE (2) or DEGRADED (3) — normal driving patterns

    // Front: white position lights
    fill_solid(leds, NUM_LEDS_FRONT, CRGB(180, 180, 180));

    // Rear: depends on driving state
    if (reverse) {
        // Reverse: white backup lights
        fill_solid(leds + NUM_LEDS_FRONT, NUM_LEDS_REAR, CRGB(200, 200, 200));
    } else if (braking) {
        // Braking: bright red
        fill_solid(leds + NUM_LEDS_FRONT, NUM_LEDS_REAR, CRGB(255, 0, 0));
    } else {
        // Normal tail: dim red
        fill_solid(leds + NUM_LEDS_FRONT, NUM_LEDS_REAR, CRGB(80, 0, 0));
    }

    FastLED.show();
}

} // namespace led_ctrl
