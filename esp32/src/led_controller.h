// =============================================================================
// ESP32-S3 HMI — LED Controller (WS2812B)
//
// Drives two independent addressable WS2812B LED strips:
//   Front strip (GPIO 47): 70 LEDs — KITT scanner, throttle-reactive effects,
//                           turn signals (10 LEDs each side)
//   Rear  strip (GPIO 48): 72 LEDs — tail, brake, reverse, turn signals
//                           (10 LEDs each side)
//
// Front LED zones (70 LEDs):
//   [0–9]    Left  turn-signal indicator (amber blink 500 ms, 10 LEDs)
//   [10–59]  Centre KITT scanner / throttle-reactive / alert effects (50 LEDs)
//   [60–69]  Right turn-signal indicator (amber blink 500 ms, 10 LEDs)
//
// Rear LED zones (72 LEDs):
//   [0–9]    Left  turn-signal indicator (amber blink 500 ms, 10 LEDs)
//   [10–61]  Centre tail / brake / reverse / regen lights (52 LEDs)
//   [62–71]  Right turn-signal indicator (amber blink 500 ms, 10 LEDs)
//
// Reference: docs/PIN_USAGE_INVENTORY.md §5.6
//            github.com/florinzgz/FULL-FIRMWARE-Coche-Marcos
// =============================================================================

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <cstdint>
#include "can_ids.h"

namespace led_ctrl {

// ---- Strip hardware configuration ----
inline constexpr int LED_FRONT_PIN   = 47;   // GPIO 47 — front strip data
inline constexpr int LED_REAR_PIN    = 48;   // GPIO 48 — rear strip data
inline constexpr int NUM_LEDS_FRONT  = 70;
inline constexpr int NUM_LEDS_REAR   = 72;

// ---- Per-strip FastLED colour order (AUDIT B) ----
//
// The wire byte order (RGB vs GRB) is a per-strip HARDWARE property and MUST
// be determined PHYSICALLY with the RGB_DIAG DecorMode (per-strip
// RED/GREEN/BLUE/WHITE/OFF sweep), NOT assumed from a code comment.  History
// has flip-flopped between RGB and GRB for the front strip, so both orders are
// now INDEPENDENTLY overridable at build time WITHOUT touching the other strip:
//
//   ; platformio.ini  build_flags
//   -D FRONT_LED_COLOR_ORDER=RGB   ; only if RGB_DIAG proves FRONT RED→green with GRB
//   -D REAR_LED_COLOR_ORDER=GRB
//
// Procedure: run RGB_DIAG, order "FRONT RED".  If the strip physically shows
// GREEN, the configured order is wrong for that strip — flip ONLY that strip's
// macro.  Defaults below match the current in-tree known-good build (both GRB);
// change via build_flags only after a physical RGB_DIAG confirmation.
#ifndef FRONT_LED_COLOR_ORDER
#define FRONT_LED_COLOR_ORDER GRB
#endif
#ifndef REAR_LED_COLOR_ORDER
#define REAR_LED_COLOR_ORDER GRB
#endif

// ---- Front and Rear installation direction ----
//
// ⚠️ PHYSICAL INSTALLATION CONTRACT (ACTUALIZADO MAY 2026):
// Las dos tiras deben conectarse con DATA IN por el lado DERECHO mirando el coche de frente
// (o, desde atrás hacia adelante, el lado izquierdo del coche). LED_STRIP_REVERSED=1.
// No se ha alterado ninguna otra lógica ni hardware, solo el sentido de conexión física.
//
//  #define LED_STRIP_REVERSED 1
#ifndef LED_STRIP_REVERSED
#define LED_STRIP_REVERSED 1
#endif

#if LED_STRIP_REVERSED
inline constexpr int FRONT_IND_LEFT_START  = 60;   // Swapped: physical right = logical left
inline constexpr int FRONT_IND_LEFT_COUNT  = 10;
inline constexpr int FRONT_CENTRE_START    = 10;   // Centre unchanged
inline constexpr int FRONT_CENTRE_COUNT    = 50;
inline constexpr int FRONT_IND_RIGHT_START = 0;    // Swapped: physical left = logical right
inline constexpr int FRONT_IND_RIGHT_COUNT = 10;

inline constexpr int REAR_IND_LEFT_START  = 62;   // Swapped: physical right = logical left
inline constexpr int REAR_IND_LEFT_COUNT  = 10;
inline constexpr int REAR_CENTRE_START    = 10;   // Centre unchanged
inline constexpr int REAR_CENTRE_COUNT    = 52;
inline constexpr int REAR_IND_RIGHT_START = 0;    // Swapped: physical left = logical right
inline constexpr int REAR_IND_RIGHT_COUNT = 10;
#else
inline constexpr int FRONT_IND_LEFT_START  = 0;    // LEDs 0-9:   left turn
inline constexpr int FRONT_IND_LEFT_COUNT  = 10;
inline constexpr int FRONT_CENTRE_START    = 10;   // LEDs 10-59: centre
inline constexpr int FRONT_CENTRE_COUNT    = 50;
inline constexpr int FRONT_IND_RIGHT_START = 60;   // LEDs 60-69: right turn
inline constexpr int FRONT_IND_RIGHT_COUNT = 10;

inline constexpr int REAR_IND_LEFT_START  = 0;    // LEDs 0-9:   left turn
inline constexpr int REAR_IND_LEFT_COUNT  = 10;
inline constexpr int REAR_CENTRE_START    = 10;   // LEDs 10-61: centre
inline constexpr int REAR_CENTRE_COUNT    = 52;
inline constexpr int REAR_IND_RIGHT_START = 62;   // LEDs 62-71: right turn
inline constexpr int REAR_IND_RIGHT_COUNT = 10;
#endif

// ---- Animation / brightness constants ----
inline constexpr uint8_t  BRIGHTNESS_POSITION  = 51;   // 20 % for tail lights
inline constexpr uint8_t  BRIGHTNESS_FULL      = 255;
inline constexpr uint8_t  FADE_RATE_KITT       = 60;
inline constexpr uint8_t  FADE_RATE_CHASE      = 30;
inline constexpr uint16_t EMERGENCY_FLASH_MS   = 100;  // emergency toggle
inline constexpr uint16_t TURN_SIGNAL_BLINK_MS = 500;  // indicator toggle

// ---- Front LED modes ----
enum class FrontMode : uint8_t {
    OFF          = 0,
    KITT_IDLE    = 1,   // Red KITT scanner (idle / no throttle)
    ACCEL_LOW    = 2,   // Throttle  1-25 % chase (red→orange)
    ACCEL_MED    = 3,   // Throttle 25-50 % chase (orange→yellow)
    ACCEL_HIGH   = 4,   // Throttle 50-75 % chase (yellow→green)
    ACCEL_MAX    = 5,   // Throttle 75-100 % rainbow
    REVERSE      = 6,   // White KITT scanner (reverse gear)
    ABS_ALERT    = 7,   // Red/white flash (ABS active)
    TCS_ALERT    = 8    // Orange/black flash (TCS active)
};

// ---- Rear LED modes ----
enum class RearMode : uint8_t {
    OFF              = 0,
    POSITION         = 1,   // Dim red tail lights (20 %)
    BRAKE            = 2,   // Bright red (100 %)
    BRAKE_EMERGENCY  = 3,   // Red flashing (emergency braking)
    REVERSE          = 4,   // White backup lights
    REGEN_ACTIVE     = 5    // Blue pulse (regenerative braking)
};

// ---- Turn signal state ----
enum class TurnSignal : uint8_t {
    OFF     = 0,
    LEFT    = 1,
    RIGHT   = 2,
    HAZARD  = 3
};

/// Initialize FastLED for both strips
void init();

/// Master update — call from main loop at ~20 Hz.
/// Handles all animation timers internally.
void update();

/// Set front LED mode directly
void setFrontMode(FrontMode mode);

/// Set front LED mode from throttle percentage (0-100)
void setFrontFromThrottle(float throttlePercent);

/// Set rear LED mode
void setRearMode(RearMode mode);

/// Set turn signal state
void setTurnSignal(TurnSignal signal);

/// Get current turn signal state (read-only, for UI display)
TurnSignal getTurnSignal();

/// Enable / disable entire LED system
void setEnabled(bool en);

/// Trigger non-blocking emergency flash (all LEDs red, count cycles)
void startEmergencyFlash(uint8_t count);

/// Returns true while emergency flash is in progress
bool isEmergencyFlashActive();

// =========================================================================
// Decorative LED modes — Engineering menu / hidden menu
//
// These modes overlay the centre zones of front and rear strips with
// decorative patterns (police, ambulance, warning, demo, etc.).
// They are ALWAYS rendered BEFORE the turn-signal overlays so that amber
// indicators retain full priority on all modes.
//
// PRIORITY (highest to lowest):
//   1. Emergency flash (startEmergencyFlash)
//   2. Turn-signal overlay (amber — unchanged by any decor mode)
//   3. Decorative mode (centre + side zones, EXCEPT where turn signal active)
//   4. Normal KITT / throttle-reactive base (when DecorMode::NORMAL)
//
// SAFETY NOTE:
//   These modes are purely decorative for private/demo use.
//   They do NOT affect relays, CAN, RC, audio, traction or steering.
//   No delay() calls.  All timing via millis() in update().
// =========================================================================

/// Brightness scale factors for decorative modes (0-255)
inline constexpr uint8_t LED_BRIGHTNESS_NORMAL    = 80;   // Normal operation
inline constexpr uint8_t LED_BRIGHTNESS_EMERGENCY = 120;  // Police / Ambulance
inline constexpr uint8_t LED_BRIGHTNESS_DEMO      = 60;   // Demo / Warning

/// Decorative mode enum (stored in NVS via config_store)
enum class DecorMode : uint8_t {
    NORMAL        = 0,  // Default: existing KITT / throttle-reactive behaviour
    OFF           = 1,  // All LEDs off
    POLICE_US     = 2,  // US-style red/blue alternating wig-wag
    AMBULANCE     = 3,  // Red/white alternating
    WARNING_AMBER = 4,  // Amber blink (precaution / slow)
    HAZARD_RED    = 5,  // Rear red double-flash
    DEMO_SHOW     = 6,  // Full walkthrough: cycles every pattern + indicators
    CUSTOM_TEST   = 7,  // Segment diagnostic (cycles through zones)
    KNIGHT_RIDER  = 8,  // "Coche fantástico" — red scanner bounce with tail
    RGB_DIAG      = 9   // Per-strip RED/GREEN/BLUE/WHITE/OFF colour diagnostic
};

/// Number of valid DecorMode values
inline constexpr uint8_t DECOR_MODE_COUNT = 10;

// ---- RGB_DIAG colour-order diagnostic timing --------------------------------
// Mirror of the render-side constants in led_controller.cpp.  RGB_DIAG walks a
// per-strip RED → GREEN → BLUE → WHITE → OFF sequence: 10 phases at 2.5 s each
// (front phases 0-4, rear phases 5-9).  The full sequence therefore lasts
// 10 × 2500 ms = 25 000 ms and EVERY phase (both strips, including the OFF
// phases 4 and 9) must be reachable before the Engineering TEST auto-restores.
inline constexpr uint16_t DECOR_RGB_DIAG_STEP_MS = 2500;
inline constexpr uint8_t  DECOR_RGB_DIAG_PHASES  = 10;
inline constexpr uint32_t DECOR_RGB_DIAG_SEQUENCE_MS =
    static_cast<uint32_t>(DECOR_RGB_DIAG_STEP_MS) * DECOR_RGB_DIAG_PHASES;

// ---- Engineering LED-mode TEST auto-restore duration ------------------------
// The Engineering screen runs a decorative mode for a bounded window then
// auto-restores the previous mode.  A normal decor test only needs ~10 s, but
// RGB_DIAG must run its COMPLETE 25 s colour-order sequence (otherwise the rear
// strip and the OFF phases are never diagnosed), so it gets a longer window
// with margin.  Pure/host-testable — no hardware access.
inline constexpr uint32_t DECOR_TEST_DEFAULT_MS  = 10000;  // normal decor test
inline constexpr uint32_t DECOR_TEST_RGB_DIAG_MS = 30000;  // ≥ 25 s + margin

/// Auto-restore duration (ms) for the Engineering LED-mode TEST of @p mode.
/// RGB_DIAG returns a window long enough to render its full colour sequence;
/// every other mode returns the default 10 s window.
constexpr uint32_t decorTestDurationMs(DecorMode mode) {
    return (mode == DecorMode::RGB_DIAG) ? DECOR_TEST_RGB_DIAG_MS
                                         : DECOR_TEST_DEFAULT_MS;
}

/// Set the decorative LED mode.  Takes effect on next update() call.
void setDecorMode(DecorMode mode);

/// Get the currently active decorative LED mode.
DecorMode getDecorMode();

/// Human-readable label of the primary/strip currently commanded by the
/// RGB_DIAG mode (e.g. "FRONT RED", "REAR BLUE").  Lets the HMI display the
/// COMMANDED colour so it can be compared against what the strip physically
/// shows — the only reliable way to confirm the WS2812B byte order per strip.
/// Returns an empty string ("") when RGB_DIAG is not the active mode.
const char* getRgbDiagLabel();

} // namespace led_ctrl

#endif // LED_CONTROLLER_H

