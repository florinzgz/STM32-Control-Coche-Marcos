// =============================================================================
// ESP32-S3 HMI — LED Controller (WS2812B)
//
// Drives addressable WS2812B LED strips: 28 front + 16 rear = 44 LEDs.
// Patterns change automatically based on vehicle state (via VehicleData).
// Power relay on STM32 (PB10) must be ON for LEDs to light.
//
// GPIO 38: WS2812B data output (chosen for no alternate-function conflicts)
//
// Reference: FIRMWARE_MIGRATION_AUDIT.md Step 6
// =============================================================================

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <cstdint>
#include "can_ids.h"

namespace led_ctrl {

/// LED strip configuration
inline constexpr int LED_DATA_PIN    = 38;
inline constexpr int NUM_LEDS_FRONT  = 28;
inline constexpr int NUM_LEDS_REAR   = 16;
inline constexpr int NUM_LEDS_TOTAL  = NUM_LEDS_FRONT + NUM_LEDS_REAR;

/// Initialize FastLED and LED array
void init();

/// Update LED patterns based on vehicle state.
/// Call from main loop at ~20 Hz.
///   @param systemState  Current SystemState from heartbeat
///   @param braking      true if throttle is decreasing (rear brake lights)
///   @param reverse      true if in reverse gear (rear white lights)
///   @param enabled      true if LED relay is ON (STM32 confirmed)
void update(uint8_t systemState, bool braking, bool reverse, bool enabled);

} // namespace led_ctrl

#endif // LED_CONTROLLER_H
