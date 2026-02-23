// =============================================================================
// ESP32-S3 HMI — LED Controller (WS2812B)
//
// Drives two independent addressable WS2812B LED strips:
//   Front strip (GPIO 47): 28 LEDs — KITT scanner, throttle-reactive effects
//   Rear  strip (GPIO 48): 16 LEDs — tail, brake, reverse, turn signals
//
// Rear LED zones (16 LEDs):
//   [0–2]   Left turn-signal indicator  (amber blink 500 ms)
//   [3–12]  Centre tail / brake / reverse / regen lights
//   [13–15] Right turn-signal indicator (amber blink 500 ms)
//
// Architecture ported from FULL-FIRMWARE-Coche-Marcos (florinzgz):
//   src/lighting/led_controller.cpp + include/led_controller.h
// Adapted for CAN-based VehicleData model (no direct sensor reads).
//
// Turn signals are managed locally on the ESP32 (no STM32 CAN involvement).
// Power relay on STM32 (PB10) must be ON for LEDs to light.
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
inline constexpr int NUM_LEDS_FRONT  = 28;
inline constexpr int NUM_LEDS_REAR   = 16;

// ---- Rear strip zone boundaries ----
inline constexpr int REAR_IND_LEFT_START  = 0;    // LEDs 0-2: left turn
inline constexpr int REAR_IND_LEFT_COUNT  = 3;
inline constexpr int REAR_CENTRE_START    = 3;    // LEDs 3-12: centre
inline constexpr int REAR_CENTRE_COUNT    = 10;
inline constexpr int REAR_IND_RIGHT_START = 13;   // LEDs 13-15: right turn
inline constexpr int REAR_IND_RIGHT_COUNT = 3;

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

/// Enable / disable entire LED system
void setEnabled(bool en);

/// Trigger non-blocking emergency flash (all LEDs red, count cycles)
void startEmergencyFlash(uint8_t count);

/// Returns true while emergency flash is in progress
bool isEmergencyFlashActive();

} // namespace led_ctrl

#endif // LED_CONTROLLER_H
