#ifndef DISPLAY_BACKLIGHT_H
#define DISPLAY_BACKLIGHT_H

#include <cstdint>

namespace display_backlight {

inline constexpr uint8_t BRIGHTNESS_MIN = 26;   // ~10% to avoid near-off display
inline constexpr uint8_t BRIGHTNESS_MAX = 255;  // full-scale PWM duty
inline constexpr uint8_t BRIGHTNESS_STEP = 13;  // ~5% step per touch

void init(uint8_t requestedBrightness);
uint8_t apply(uint8_t requestedBrightness);
uint8_t current();
uint8_t toPercent(uint8_t brightness);

} // namespace display_backlight

#endif // DISPLAY_BACKLIGHT_H
