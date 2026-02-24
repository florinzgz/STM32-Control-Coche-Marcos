// =============================================================================
// TFT_eSPI User Setup — ESP32-S3 + ST7796 (480×320, landscape)
//
// Board: ESP32-S3-DevKitC-1 (ESP32-S3-WROOM-1-N16R8)
//        16 MB Quad Flash + 8 MB Octal PSRAM (qio_opi)
//
// GPIO map for the ESP32-S3-DevKitC-1:
//   Available : GPIO 0–21 and GPIO 33–48
//   Non-existent (classic ESP32 only): GPIO 34, 35, 36, 37
//   Reserved (Flash/PSRAM internal bus): GPIO 26, 27, 28, 29, 30, 31, 32
//
// IMPORTANT — pins that MUST NOT be used for SPI / TFT:
//   GPIO 26–32   → connected to internal Flash/PSRAM bus — boot corruption risk
//   GPIO 34–37   → do NOT exist on the ESP32-S3 (were input-only on classic ESP32)
//   GPIO 0       → boot strapping pin
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

// --- SPI pin assignment ---
//
// Pin   GPIO  Why it is safe
// ────  ────  ──────────────────────────────────────────────────────────────
// MOSI   13   GPIO 1–21 range — bidirectional, SPI-capable, no conflict
// MISO   12   GPIO 1–21 range — bidirectional, SPI-capable, shared with T_DO
// SCLK   14   GPIO 1–21 range — bidirectional, SPI-capable, clock output
// CS     10   GPIO 1–21 range — bidirectional, SPI-capable, no conflict
// DC     33   GPIO 33–48 range — output-capable, no conflict
// RST    38   GPIO 33–48 range — output-capable, no conflict
// BL     45   GPIO 33–48 range — valid output after boot (VDD_SPI strapping)
// T_CS   21   GPIO 1–21 range — bidirectional, no conflict
//
// GPIO 34/35/36/37 are NOT used: they do not exist on the ESP32-S3.
// GPIO 26–32 are NOT used: reserved for internal Flash/PSRAM bus.
//
// None of these collide with:
//   CAN 4/5 · obstacle 6/7 · I2C 8/9 · power 40/41 · DFPlayer 43/44
//   LED strip 47/48
//
#define TFT_MOSI 13
#define TFT_MISO 12
#define TFT_SCLK 14
#define TFT_CS   10
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
