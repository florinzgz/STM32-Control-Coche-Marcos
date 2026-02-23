// =============================================================================
// TFT_eSPI User Setup -- ESP32-S3 + ST7796 (480x320, landscape)
//
// Board: ESP32-S3-DevKitC-1 (ESP32-S3-WROOM-1-N16R8)
//        16 MB Quad Flash + 8 MB Octal PSRAM (qio_opi)
//
// IMPORTANT — ESP32-S3-WROOM-1-N16R8 pin restrictions:
//   GPIO 15      → flash/PSRAM internal (MUST NOT use)
//   GPIO 26-37   → flash (26-32) and Octal PSRAM (33-37) internal (MUST NOT use)
//   GPIO 18-20   → USB-Serial/JTAG (avoid)
//   GPIO 0, 3    → strapping pins (avoid)
//   GPIO 1, 2    → JTAG (avoid)
//   GPIO 12      → input-limited, unreliable as MISO
//
// This file is force-included via platformio.ini:
//   -include $PROJECT_DIR/include/User_Setup.h
// ensuring TFT_eSPI (and all libraries) pick up these defines
// instead of the library's default User_Setup.h.
// =============================================================================

#ifndef USER_SETUP_H
#define USER_SETUP_H

// USER_SETUP_LOADED must be the FIRST #define in this file.
// TFT_eSPI checks it to confirm a custom setup has been loaded.
#define USER_SETUP_LOADED 1

// --- Driver ---
#define ST7796_DRIVER

// --- Panel resolution (physical, before rotation) ---
#define TFT_WIDTH  320
#define TFT_HEIGHT 480

// --- SPI port selection ---
// Use HSPI (SPI3_HOST) to avoid potential conflict with SPI2 (FSPI) which
// shares default pin-mux entries with the Octal PSRAM controller on N16R8.
// TFT_eSPI will create its own SPIClass on SPI3, independent of the global
// Arduino SPI object.
#define USE_HSPI_PORT

// --- SPI pin assignment ---
// All pins verified safe for ESP32-S3-WROOM-1-N16R8:
//   - Not used by flash/PSRAM (GPIO 15, 26-37)
//   - Not used by USB (GPIO 18-20) or JTAG (GPIO 1-2)
//   - Not strapping pins (GPIO 0, 3)
//   - Bidirectional, support output and input
//   - No conflict with other project peripherals (CAN 4/5, obstacle 6/7,
//     I2C 8/9, LED 38, power 40/41, DFPlayer 43/44)
#define TFT_MISO 13    // GPIO 13 — safe bidirectional; shared with XPT2046 T_DO
#define TFT_MOSI 11    // GPIO 11 — safe bidirectional
#define TFT_SCLK 14    // GPIO 14 — safe bidirectional
#define TFT_CS   10    // GPIO 10 — safe bidirectional (replaces GPIO 15 which was
                       //            reserved for flash/PSRAM and caused the crash)
#define TFT_DC   17    // GPIO 17 — safe bidirectional; DC is low-speed
#define TFT_RST  16    // GPIO 16 — safe bidirectional; RST is low-speed

// --- Backlight ---
#define TFT_BL            45    // GPIO 45 — valid output; strapping pin (VDD_SPI)
                                //           but safe for output after boot
#define TFT_BACKLIGHT_ON  HIGH

// --- SPI frequencies ---
#define SPI_FREQUENCY       40000000   // 40 MHz (display writes)
#define SPI_READ_FREQUENCY  20000000   // 20 MHz (display reads)
#define SPI_TOUCH_FREQUENCY  2500000   //  2.5 MHz (XPT2046 touch)

// --- Touch (XPT2046) ---
#define TOUCH_CS 21    // GPIO 21 — safe bidirectional; no conflict

// --- Compiled fonts ---
#define LOAD_GLCD    // Adafruit GLCD font (5×7 px)
#define LOAD_FONT2   // Smooth 8 pt
#define LOAD_FONT4   // Smooth 26 pt
#define LOAD_FONT6   // Digit 48 px (0-9 only)
#define LOAD_FONT7   // 7-segment 48 px (0-9 only)
#define LOAD_FONT8   // 75 px (0-9 only)
#define LOAD_GFXFF   // FreeFonts — Adafruit GFX proportional fonts
#define SMOOTH_FONT  // VLW smooth font support

// --- Default rotation (0=portrait, 1=landscape) ---
#define TFT_ROTATION 1

// --- XPT2046 touch calibration ---
// Default values. Run TFT_eSPI/Touch_calibrate example to obtain
// values specific to your panel, then replace these.
#define TOUCH_CALIBRATION  { 256, 3643, 182, 3672, 1 }

#endif // USER_SETUP_H
