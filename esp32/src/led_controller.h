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

} // namespace led_ctrl

#endif // LED_CONTROLLER_H

