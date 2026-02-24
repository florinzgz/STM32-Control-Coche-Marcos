# PINES DE LA PANTALLA — Listado Detallado

**Proyecto:** Control Coche Marcos  
**MCU HMI:** ESP32-S3-DevKitC-1 (N16R8, 16 MB Flash + 8 MB PSRAM)  
**Pantalla:** TFT LCD 480×320 — Driver ST7796 — Interface SPI  
**Touch:** XPT2046 (resistivo), bus SPI compartido con el display  
**Fuente de verdad del firmware:** `esp32/include/User_Setup.h`

---

## 1. TABLA COMPLETA DE CONEXIONES — DISPLAY Y TOUCH

| Pin en la Pantalla | Señal / Función                        | GPIO ESP32-S3 | Notas |
|--------------------|----------------------------------------|---------------|-------|
| **VCC**            | Alimentación 3.3 V                     | 3.3 V (pin regulador) | Conectar a la salida 3.3 V del DevKitC-1 |
| **GND**            | Tierra común                           | GND           | GND compartido con ESP32 |
| **CS**             | Chip Select del display (activo bajo)  | **GPIO 34**   | `TFT_CS` en User_Setup.h |
| **RESET**          | Reset hardware del display (activo bajo) | **GPIO 38** | `TFT_RST` en User_Setup.h |
| **DC / RS**        | Selección Dato / Comando               | **GPIO 33**   | `TFT_DC`; HIGH = dato, LOW = comando |
| **SDI / MOSI**     | Datos SPI del ESP32 al display         | **GPIO 35**   | `TFT_MOSI`; SPI HSPI (SPI3_HOST) |
| **SCK / SCLK**     | Reloj SPI                              | **GPIO 36**   | `TFT_SCLK`; SPI HSPI (SPI3_HOST) |
| **SDO / MISO**     | Datos SPI del display al ESP32         | **GPIO 37**   | `TFT_MISO`; compartido con T_DO del XPT2046 |
| **LED / BL**       | Retroiluminación (backlight)           | **GPIO 45**   | `TFT_BL`; HIGH = encendido, LOW = apagado |
| **T_CS**           | Chip Select del touch (activo bajo)    | **GPIO 21**   | `TOUCH_CS` en User_Setup.h |
| **T_DIN**          | Datos SPI al controlador touch         | **GPIO 35**   | Compartido con SDI / MOSI del display |
| **T_CLK**          | Reloj SPI del touch                    | **GPIO 36**   | Compartido con SCK del display |
| **T_DO**           | Datos SPI del controlador touch        | **GPIO 37**   | Compartido con SDO / MISO del display |
| **T_IRQ**          | Interrupción del touch                 | **No conectar** | El firmware usa polling; no se usa interrupción |

---

## 2. RESUMEN RÁPIDO POR GPIO

| GPIO ESP32-S3 | Pin en la Pantalla | Descripción |
|---------------|--------------------|-------------|
| GPIO 33       | DC / RS            | Data/Command — selecciona modo del bus SPI |
| GPIO 34       | CS                 | Chip Select display — selecciona el display en el bus |
| GPIO 35       | SDI (MOSI) + T_DIN | Línea de datos SPI (ESP32 → display y touch) |
| GPIO 36       | SCK + T_CLK        | Reloj SPI compartido display y touch |
| GPIO 37       | SDO (MISO) + T_DO  | Línea de datos SPI (display y touch → ESP32) |
| GPIO 38       | RESET              | Reset hardware del controlador ST7796 |
| GPIO 21       | T_CS               | Chip Select touch — selecciona el XPT2046 en el bus |
| GPIO 45       | LED / BL           | Control de retroiluminación |
| 3.3 V         | VCC                | Alimentación del display y del touch |
| GND           | GND                | Masa común |

---

## 3. DIAGRAMA DE CONEXIÓN FÍSICA

```
ESP32-S3 DevKitC-1                       Display TFT ST7796 + Touch XPT2046
┌─────────────────────┐                  ┌───────────────────────────┐
│                     │                  │                           │
│  3.3 V  ────────────┼──────────────────┼──► VCC                    │
│  GND    ────────────┼──────────────────┼──► GND                    │
│                     │                  │                           │
│  GPIO 34 (CS)  ─────┼──────────────────┼──► CS    (display)        │
│  GPIO 38 (RST) ─────┼──────────────────┼──► RESET (display)        │
│  GPIO 33 (DC)  ─────┼──────────────────┼──► DC/RS (display)        │
│  GPIO 35 (MOSI)─────┼──────────────────┼──► SDI   (display)        │
│  GPIO 36 (SCK) ─────┼──────────────────┼──► SCK   (display)        │
│  GPIO 45 (BL)  ─────┼──────────────────┼──► LED   (retroilum.)     │
│  GPIO 37 (MISO)◄────┼──────────────────┼─── SDO   (display)        │
│                     │                  │                           │
│  GPIO 21 (T_CS)─────┼──────────────────┼──► T_CS  (touch)          │
│  GPIO 35 (MOSI)─────┼──────────────────┼──► T_DIN (touch, compart.)│
│  GPIO 36 (SCK) ─────┼──────────────────┼──► T_CLK (touch, compart.)│
│  GPIO 37 (MISO)◄────┼──────────────────┼─── T_DO  (touch, compart.)│
│                     │                  │  T_IRQ — no conectar      │
└─────────────────────┘                  └───────────────────────────┘

Nota: GPIO 35, 36 y 37 son compartidos entre display y touch.
      Se selecciona el periférico activo mediante CS (GPIO 34) o T_CS (GPIO 21).
```

---

## 4. CONFIGURACIÓN SPI

| Parámetro            | Valor          | Notas |
|----------------------|----------------|-------|
| Bus SPI utilizado    | HSPI (SPI3_HOST) | Independiente del Arduino SPI por defecto (FSPI/SPI2) |
| Frecuencia display   | 40 MHz         | Escrituras normales al ST7796 |
| Frecuencia lectura   | 20 MHz         | Lecturas del framebuffer del display |
| Frecuencia touch     | 2.5 MHz        | XPT2046 requiere frecuencia reducida para estabilidad |
| Modo SPI             | Modo 0         | CPOL = 0, CPHA = 0 |
| Rotación             | 1 (landscape)  | 480 × 320 px en horizontal |

---

## 5. DEFINICIONES EN EL FIRMWARE (`esp32/include/User_Setup.h`)

```cpp
#define ST7796_DRIVER          // Driver del display

#define TFT_WIDTH   320        // Resolución física (antes de rotación)
#define TFT_HEIGHT  480

#define USE_HSPI_PORT          // SPI3_HOST (HSPI), independiente del SPI por defecto

#define TFT_MOSI 35            // GPIO 35 — datos SPI al display
#define TFT_MISO 37            // GPIO 37 — datos SPI del display / T_DO del touch
#define TFT_SCLK 36            // GPIO 36 — reloj SPI
#define TFT_CS   34            // GPIO 34 — Chip Select display
#define TFT_DC   33            // GPIO 33 — Data / Command
#define TFT_RST  38            // GPIO 38 — Reset display

#define TFT_BL           45   // GPIO 45 — backlight
#define TFT_BACKLIGHT_ON HIGH  // HIGH = retroiluminación encendida

#define SPI_FREQUENCY      40000000   // 40 MHz (escritura)
#define SPI_READ_FREQUENCY 20000000   // 20 MHz (lectura)
#define SPI_TOUCH_FREQUENCY 2500000   //  2.5 MHz (touch XPT2046)

#define TOUCH_CS 21            // GPIO 21 — Chip Select touch
```

---

## 6. PINES QUE **NO** SE DEBEN USAR PARA EL DISPLAY

Los GPIOs que aparecen en documentación antigua son **incorrectos y peligrosos**:

| GPIO antiguo | Razón por la que NO se debe usar |
|---|---|
| GPIO 12 | Input-limited, no fiable como MISO |
| GPIO 13 | Sin conflicto propio, pero ya reemplazado por GPIO 35 |
| GPIO 14 | Sin conflicto propio, pero ya reemplazado por GPIO 36 |
| GPIO 15 | **Reservado para flash/PSRAM — causó el crash original** |
| GPIO 16 | Líneas internas de RAM/Flash en algunos módulos |
| GPIO 17 | Líneas internas de RAM/Flash en algunos módulos |
| GPIO 42 | Reemplazado por GPIO 45 para el backlight |

---

## 7. VERIFICACIÓN CON MULTÍMETRO

| Punto de medición          | Valor esperado | Fallo si… |
|----------------------------|----------------|-----------|
| VCC del display → GND      | 3.30 V ± 0.1 V | < 3.0 V o > 3.5 V |
| GPIO 45 → GND (encendido)  | 3.3 V (HIGH)   | 0 V → backlight apagado |
| GPIO 34 → GND (inactivo)   | 3.3 V (HIGH)   | CS debe estar en HIGH cuando no se usa |
| GPIO 21 → GND (inactivo)   | 3.3 V (HIGH)   | T_CS debe estar en HIGH cuando no se usa |

---

## 8. SOLUCIÓN DE PROBLEMAS

| Síntoma                 | Causa probable                          | Solución |
|-------------------------|-----------------------------------------|----------|
| Pantalla completamente negra | Backlight apagado o sin alimentación | Verificar VCC = 3.3 V y GPIO 45 = HIGH |
| Pantalla blanca / sin imagen | Reset incorrecto o CS incorrecto      | Verificar GPIO 38 (RST) y GPIO 34 (CS) |
| Imagen corrupta / ruido  | Interferencia en el bus SPI            | Cables SPI < 20 cm; capacitor 100 nF en VCC del display |
| Touch no responde        | T_CS o MISO no conectados              | Verificar GPIO 21 → T_CS y GPIO 37 → T_DO |
| Crash al iniciar el SPI  | GPIO 15 / 16 / 17 en uso              | **Usar únicamente los GPIOs de este documento** |

---

## REFERENCIAS

- `esp32/include/User_Setup.h` — Definiciones de pines del firmware (fuente de verdad)
- `docs/ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md` — Guía detallada con CAN Bus incluido
- `docs/DIAGRAMA_PINES_VISUAL.md` — Diagramas visuales ASCII
- `docs/CONEXIONES_RAPIDAS_ESP32.md` — Referencia rápida en tablas
- [ST7796 Datasheet](https://www.displayfuture.com/Display/datasheet/controller/ST7796s.pdf)
- [XPT2046 Datasheet](https://ldm-systems.ru/f/doc/catalog/HY-TFT-2,8/XPT2046.pdf)

---

_Última actualización: 2026-02-24_  
_Fuente de pines verificada contra: `esp32/include/User_Setup.h`_
