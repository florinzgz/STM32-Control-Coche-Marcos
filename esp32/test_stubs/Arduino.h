/**
 * @file  Arduino.h  (test stub)
 * @brief Minimal stub replacing the real Arduino.h for host-side unit tests.
 *
 * All functions and globals are defined with C++17 inline linkage so they
 * can be safely included in multiple translation units without ODR violations.
 *
 * Tests control time by writing to g_test_millis.
 * Tests inspect GPIO state via g_gpio_state[] and g_gpio_write_count[].
 */

#pragma once
#include <cstdint>
#include <cstdio>

/* ---- Arduino constants -------------------------------------------------- */
#define HIGH 1
#define LOW  0
#define INPUT    0
#define OUTPUT   1
#define INPUT_PULLUP 0x05
#define INPUT_PULLDOWN 0x09
#define SERIAL_8N1 0x06

/* ---- Controllable test clock ------------------------------------------- */
inline unsigned long g_test_millis      = 0;
inline int           g_gpio_state[64]   = {};   /* 0 = LOW by default        */
inline unsigned int  g_gpio_write_count[64] = {};

/* ---- Arduino API stubs -------------------------------------------------- */
static inline unsigned long millis() { return g_test_millis; }

static inline void pinMode(int, int) {}

static inline void digitalWrite(int pin, int val) {
    if (pin >= 0 && pin < 64) {
        g_gpio_state[pin] = val;
        g_gpio_write_count[pin]++;
    }
}

static inline int digitalRead(int pin) {
    return (pin >= 0 && pin < 64) ? g_gpio_state[pin] : 0;
}

static inline void delay(unsigned long) {}
static inline void delayMicroseconds(unsigned long) {}

/* ---- Minimal Serial stub ----------------------------------------------- */
struct FakeSerial_ {
    void begin(unsigned long) {}
    void println(const char*)  {}
    void println(int)          {}
    template<typename T> void println(T) {}
    template<typename... Args> void printf(Args...) {}
};
inline FakeSerial_ Serial;

/* ---- HardwareSerial stub (used by audio_manager, obstacle_sensor) ----- */
struct HardwareSerial {
    explicit HardwareSerial(int) {}
    void begin(unsigned long, int, int, int) {}
    size_t write(const uint8_t*, size_t n) { return n; }
    int available() { return 0; }
    int read() { return -1; }
};
