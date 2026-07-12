// =============================================================================
// TFT_eSPI User Setup — ESP32-S3 + ST7796 (480×320, landscape)
//
// Board: ESP32-S3-DevKitC-1 (ESP32-S3-WROOM-1-N16R8)
//        16 MB Quad Flash + 8 MB Octal PSRAM (qio_opi)
//
// GPIO map for the ESP32-S3-WROOM-1-N16R8 module:
//   Available for external use: GPIO 0–21 and GPIO 38–48
//     (GPIO 0, GPIO 3 are strapping pins — safe as GPIO but require care at boot)
//     (GPIO 45, GPIO 46 are strapping pins — avoid for outputs that must be
//     HIGH at boot; GPIO 45 controls VDD_SPI voltage selection)
//   Reserved — QSPI Flash  (internal, not on module pins): GPIO 26–32
//   Reserved — Octal PSRAM (internal, not on module pins): GPIO 33–37
//   Do not exist in ESP32-S3: GPIO 22–25
//
// IMPORTANT — pins that MUST NOT be used for SPI / TFT on this module:
//   GPIO 26–32   → QSPI Flash bus — boot corruption risk
//   GPIO 33–37   → Octal PSRAM bus (N16R8) — memory corruption / crash risk
//   GPIO 0       → strapping pin (boot mode)
//   GPIO 3       → strapping pin (eFuse download)
//
// Reference: Espressif ESP32-S3 Hardware Design Guidelines
//            ESP32-S3-WROOM-1 datasheet (module pin table)
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
// DC     39   GPIO 38–48 range — output-capable, outside Flash/PSRAM range
// RST    38   GPIO 38–48 range — output-capable, outside Flash/PSRAM range
// BL     42   GPIO 38–48 range — output-capable, no strapping restrictions
// T_CS   21   GPIO 1–21 range — bidirectional, no conflict
//
// GPIO 26–32 are NOT used: QSPI Flash interface (boot failure if used).
// GPIO 33–37 are NOT used: Octal PSRAM interface on N16R8 (crash if used).
// GPIO 22–25 do NOT exist in the ESP32-S3 SoC.
//
// None of these collide with:
//   CAN 4/5 · obstacle 18 · I2C 8/9 · power 40/41 · DFPlayer 43/44
//   LED strip 47/48
//
#define TFT_MOSI 13
#define TFT_MISO 12
#define TFT_SCLK 14
#define TFT_CS   10
#define TFT_DC   39
#define TFT_RST  38

// --- Backlight ---
#define TFT_BL            42
#define TFT_BACKLIGHT_ON  HIGH

// --- SPI frequencies ---
// Audit note (Problem 2, §2.5): the white-screen fault is being investigated
// as possible SPI signal-integrity loss.  The recommended SAFE initial value
// is 20 MHz, BUT per the audit policy this value is NOT lowered here until the
// failure is demonstrated with real telemetry (display_supervisor.h captures
// render/status/heap evidence, and the recovery banner reports SPI_TIMEOUT vs
// TFT_STATUS_LOST vs TFT_RESET_PROBABLE).  To bench 20 MHz vs 40 MHz, change
// only the line below and compare display_recovery_count over a run:
//   #define SPI_FREQUENCY  20000000   // 20 MHz — safe fallback candidate
// Physical mitigations (do NOT rely on software): ~10 kΩ pull-up on TFT_RST,
// 100 nF at VCC/GND, 10–47 µF near the TFT, short SPI/RST leads, firm ground,
// and separation from motor/PWM/relay/BTS7960 noise sources.
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
// Format: { x_offset, x_range, y_offset, y_range, flags } as produced by
// TFT_eSPI::calibrateTouch().  parameters[1] and [3] are pre-computed
// RANGES (raw_max - raw_min), NOT raw maximum values.  parameters[4] is
// a 3-bit flags bitmask: bit0=rotate(swap X/Y), bit1=invert_x, bit2=invert_y
// (so flags=0x07 means rotate + invert both axes — the common combination
// for this 480x320 ST7796 panel mounted in landscape rotation 1).
//
// These values are the boot-time fallback used when no valid calibration
// is found in NVS (first boot, after factory reset, or if NVS is corrupt).
// They are an averaged approximation of multiple real captures from the
// production panel — accurate enough for the user to reliably hit the
// wizard buttons (EMPEZAR / GUARDAR) so a precise per-unit calibration
// can be captured and persisted to NVS.  Once saved, NVS takes precedence.
#define TOUCH_CALIBRATION  { 320, 3470, 230, 3280, 7 }

#endif // USER_SETUP_H
