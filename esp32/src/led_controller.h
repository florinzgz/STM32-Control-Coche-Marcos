// =============================================================================
// ESP32-S3 HMI — LED Controller (WS2812B)
//
// Drives two independent addressable WS2812B LED strips:
//   Front strip (GPIO 47): 28 LEDs — KITT scanner, throttle-reactive effects,
//                           turn signals (5 LEDs each side)
//   Rear  strip (GPIO 48): 16 LEDs — tail, brake, reverse, turn signals
//
// Front LED zones (28 LEDs):
//   [0–4]   Left turn-signal indicator  (amber blink 500 ms, 5 LEDs)
//   [5–22]  Centre KITT scanner / throttle-reactive / alert effects
//   [23–27] Right turn-signal indicator (amber blink 500 ms, 5 LEDs)
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
// ---- FUTURE: RELAY-BASED VISUAL DIAGNOSTICS ----
// The motor relay command state (MAIN/TRACTION/DIRECTION) is now available
// in VehicleData::heartbeat().relayStatus (CAN 0x001 byte 5).
// This can be used to:
//   - Flash LED strips during relay power-up sequence as visual feedback
//   - Show relay failure state via LED color changes
//   - Provide non-UI relay diagnostics for installations without a display
// Currently NOT implemented — relay state is display-only (DriveScreen +
// EngineeringScreen).  LED controller remains purely lighting-focused.
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

// ---- Front strip zone boundaries ----
//
// ⚠ PHYSICAL INSTALLATION CONTRACT:
//
// DATA IN of the front WS2812B strip must start from the LEFT side of
// the vehicle (standing in front of the vehicle, looking backward).
//
//   FRONT VIEW (looking at the front of the vehicle):
//
//     DATA IN →  [0] [1] [2] [3] [4] [5] ... [22] [23] [24] [25] [26] [27]
//                ├──── LEFT ────┤  ├──── CENTRE ────┤  ├───── RIGHT ─────┤
//                5 LEDs              18 LEDs              5 LEDs
//
// LED_STRIP_REVERSED effect on front strip:
//   0 (default): LED[0-4]=LEFT,  LED[23-27]=RIGHT  (normal)
//   1 (swapped): LED[0-4]=RIGHT, LED[23-27]=LEFT   (reversed data cable)
//
// ---- Rear strip zone boundaries ----
//
// ⚠ PHYSICAL INSTALLATION CONTRACT:
//
// DATA IN of the rear WS2812B strip must start from the LEFT side of the
// vehicle (standing behind the vehicle, looking forward toward the front).
//
//   REAR VIEW (looking at the back of the vehicle):
//
//     DATA IN →  [0] [1] [2] [3] ... [12] [13] [14] [15]
//                ├─ LEFT ─┤  ├──── CENTRE ────┤  ├─ RIGHT ─┤
//                3 LEDs        10 LEDs             3 LEDs
//
// If the installer routes the data cable from the RIGHT side instead,
// define LED_STRIP_REVERSED=1 at compile time to swap LEFT/RIGHT zones
// without editing hardware.  The swap is compile-time only — zero runtime
// cost.  Centre zone indices are unchanged in both orientations.
//
// LED_STRIP_REVERSED effect:
//   0 (default): LED[0-2]=LEFT,  LED[13-15]=RIGHT  (normal)
//   1 (swapped): LED[0-2]=RIGHT, LED[13-15]=LEFT   (reversed data cable)
#ifndef LED_STRIP_REVERSED
#define LED_STRIP_REVERSED 0
#endif

#if LED_STRIP_REVERSED
inline constexpr int FRONT_IND_LEFT_START  = 23;   // Swapped: physical right = logical left
inline constexpr int FRONT_IND_LEFT_COUNT  = 5;
inline constexpr int FRONT_CENTRE_START    = 5;    // Centre unchanged
inline constexpr int FRONT_CENTRE_COUNT    = 18;
inline constexpr int FRONT_IND_RIGHT_START = 0;    // Swapped: physical left = logical right
inline constexpr int FRONT_IND_RIGHT_COUNT = 5;

inline constexpr int REAR_IND_LEFT_START  = 13;   // Swapped: physical right = logical left
inline constexpr int REAR_IND_LEFT_COUNT  = 3;
inline constexpr int REAR_CENTRE_START    = 3;    // Centre unchanged
inline constexpr int REAR_CENTRE_COUNT    = 10;
inline constexpr int REAR_IND_RIGHT_START = 0;    // Swapped: physical left = logical right
inline constexpr int REAR_IND_RIGHT_COUNT = 3;
#else
inline constexpr int FRONT_IND_LEFT_START  = 0;    // LEDs 0-4: left turn
inline constexpr int FRONT_IND_LEFT_COUNT  = 5;
inline constexpr int FRONT_CENTRE_START    = 5;    // LEDs 5-22: centre
inline constexpr int FRONT_CENTRE_COUNT    = 18;
inline constexpr int FRONT_IND_RIGHT_START = 23;   // LEDs 23-27: right turn
inline constexpr int FRONT_IND_RIGHT_COUNT = 5;

inline constexpr int REAR_IND_LEFT_START  = 0;    // LEDs 0-2: left turn
inline constexpr int REAR_IND_LEFT_COUNT  = 3;
inline constexpr int REAR_CENTRE_START    = 3;    // LEDs 3-12: centre
inline constexpr int REAR_CENTRE_COUNT    = 10;
inline constexpr int REAR_IND_RIGHT_START = 13;   // LEDs 13-15: right turn
inline constexpr int REAR_IND_RIGHT_COUNT = 3;
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
