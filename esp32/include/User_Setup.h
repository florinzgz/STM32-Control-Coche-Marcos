// =============================================================================
// TFT_eSPI User Setup -- ESP32-S3 + ST7796 (480x320, landscape)
//
// Este archivo debe estar en esp32/include/ para que PlatformIO lo
// encuentre antes que el User_Setup.h interno de la libreria TFT_eSPI.
//
// Pines definidos segun platformio.ini (ESP32-S3-DevKitC-1, SPI estandar)
// =============================================================================

// USER_SETUP_LOADED debe ser el PRIMER #define del archivo.
// TFT_eSPI lo comprueba para confirmar que el setup ha sido cargado.
#define USER_SETUP_LOADED 1

// --- Driver ---
#define ST7796_DRIVER

// --- Resolución del panel (físico, antes de rotación) ---
#define TFT_WIDTH  320
#define TFT_HEIGHT 480

// --- Pines SPI ---
#define TFT_MISO 12     // Compartido con T_DO del XPT2046 (necesario para touch)
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC   16
#define TFT_RST  17

// --- Retroiluminación ---
#define TFT_BL            42
#define TFT_BACKLIGHT_ON  HIGH

// --- Frecuencias SPI ---
#define SPI_FREQUENCY       40000000   // 40 MHz
#define SPI_READ_FREQUENCY  20000000   // 20 MHz (lectura panel)
#define SPI_TOUCH_FREQUENCY  2500000   //  2.5 MHz (XPT2046)

// --- Touch (XPT2046) ---
#define TOUCH_CS 21

// --- Fuentes compiladas ---
#define LOAD_GLCD    // Fuente Adafruit GLCD (5×7 px)
#define LOAD_FONT2   // Fuente smooth 8 pt
#define LOAD_FONT4   // Fuente smooth 26 pt
#define LOAD_FONT6   // Fuente dígitos 48 px (solo dígitos 0-9)
#define LOAD_FONT7   // Fuente 7-segmentos 48 px (solo dígitos 0-9)
#define LOAD_FONT8   // Fuente 75 px (solo dígitos 0-9)
#define LOAD_GFXFF   // FreeFonts — fuentes proporcionales de Adafruit GFX
#define SMOOTH_FONT  // Activa soporte VLW smooth fonts

// --- Rotación por defecto (0=portrait, 1=landscape) ---
#define TFT_ROTATION 1

// --- Calibración del touch XPT2046 ---
// Valores por defecto. Ejecutar el ejemplo TFT_eSPI/Touch_calibrate
// para obtener valores específicos de tu pantalla y sustituirlos aquí.
#define TOUCH_CALIBRATION  { 256, 3643, 182, 3672, 1 }
