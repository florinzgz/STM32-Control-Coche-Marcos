// =============================================================================
// TFT_eSPI User Setup — ESP32-S3 + ST7796 (480×320, landscape)
//
// Board: ESP32-S3-DevKitC-1 (ESP32-S3-WROOM-1-N16R8)
//        16 MB Quad Flash + 8 MB Octal PSRAM (qio_opi)
//
// Pin assignment uses the Espressif-recommended SPI-capable GPIOs (34-37)
// which are fast, safe, and free from internal function conflicts on the
// ESP32-S3.
//
// IMPORTANT — pins that MUST NOT be used for SPI / TFT:
//   GPIO 15      → reserved for flash/PSRAM (caused the original crash)
//   GPIO 16, 17  → internal RAM/flash lines on some modules
//   GPIO 18-20   → USB-Serial/JTAG
//   GPIO 0       → boot strapping pin
//   GPIO 1, 2    → JTAG
//   GPIO 12      → input-limited, unreliable as MISO
//   GPIO 26-32   → ADC-only, not usable for SPI
//   GPIO 46      → output-only, no special functions
//
// This file is force-included via platformio.ini:
//   -include $PROJECT_DIR/include/User_Setup.h
// ensuring TFT_eSPI picks up these defines instead of its own defaults.
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
// Use HSPI (SPI3_HOST) to keep the TFT bus fully independent of the default
// Arduino SPI object (FSPI / SPI2_HOST).  TFT_eSPI creates its own SPIClass
// on SPI3, avoiding any bus-level contention.
#define USE_HSPI_PORT

// --- SPI pin assignment (Espressif-recommended high-speed SPI GPIOs) ---
//
// Pin   GPIO  Why it is safe
// ────  ────  ──────────────────────────────────────────────────────────────
// MOSI   35   Bidirectional, SPI-capable, no internal function conflict
// MISO   37   Bidirectional, SPI-capable, shared with XPT2046 T_DO
// SCLK   36   Bidirectional, SPI-capable, recommended clock pin
// CS     34   Bidirectional, SPI-capable (replaces GPIO 15 — root cause)
// DC     33   Input-capable; DC is an *output from ESP32 to TFT* but on
//              the TFT_eSPI side it is driven via direct GPIO register
//              writes (GPIO.out_w1ts / w1tc), which work on GPIO 33
// RST    38   Bidirectional, output-capable (LED data moved to GPIO 48)
// BL     45   Valid output after boot (VDD_SPI strapping, safe post-boot)
// T_CS   21   Bidirectional, no conflict with any project peripheral
//
// None of these collide with:
//   CAN 4/5 · obstacle 6/7 · I2C 8/9 · power 40/41 · DFPlayer 43/44
//   LED strip (relocated to GPIO 48)
//
#define TFT_MOSI 35
#define TFT_MISO 37
#define TFT_SCLK 36
#define TFT_CS   34
#define TFT_DC   33
#define TFT_RST  38

// --- Backlight ---
#define TFT_BL            45
#define TFT_BACKLIGHT_ON  HIGH

// --- SPI frequencies ---
#define SPI_FREQUENCY       40000000   // 40 MHz (display writes)
#define SPI_READ_FREQUENCY  20000000   // 20 MHz (display reads)
#define SPI_TOUCH_FREQUENCY  2500000   //  2.5 MHz (XPT2046 touch)

// --- Touch (XPT2046) ---
#define TOUCH_CS 21

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
// Default values.  Run TFT_eSPI/Touch_calibrate example to obtain
// values specific to your panel, then replace these.
#define TOUCH_CALIBRATION  { 256, 3643, 182, 3672, 1 }

#endif // USER_SETUP_H
