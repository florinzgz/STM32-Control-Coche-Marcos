#include "display_backlight.h"
#include <Arduino.h>
#include "User_Setup.h"

namespace display_backlight {

namespace {

constexpr uint8_t PWM_CHANNEL = 7;
constexpr uint32_t PWM_FREQ_HZ = 20000;   // high enough to avoid visible flicker
constexpr uint8_t PWM_BITS = 8;

uint8_t currentBrightness_ = 100;
bool initialized_ = false;

uint8_t clampBrightness(uint8_t v) {
    if (v < BRIGHTNESS_MIN) return BRIGHTNESS_MIN;
    if (v >= BRIGHTNESS_MAX) return BRIGHTNESS_MAX;
    return v;
}

} // namespace

void init(uint8_t requestedBrightness) {
    currentBrightness_ = clampBrightness(requestedBrightness);

#if defined(TFT_BL)
    ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(TFT_BL, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, currentBrightness_);
    initialized_ = true;
#else
    initialized_ = false;
#endif
}

uint8_t apply(uint8_t requestedBrightness) {
    currentBrightness_ = clampBrightness(requestedBrightness);
#if defined(TFT_BL)
    if (!initialized_) {
        init(currentBrightness_);
    } else {
        ledcWrite(PWM_CHANNEL, currentBrightness_);
    }
#endif
    return currentBrightness_;
}

uint8_t current() {
    return currentBrightness_;
}

uint8_t toPercent(uint8_t brightness) {
    return static_cast<uint8_t>((static_cast<uint16_t>(brightness) * 100U + 127U) / 255U);
}

} // namespace display_backlight
