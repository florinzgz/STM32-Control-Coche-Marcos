# RESUMEN — Pines que NO se pueden usar en la ESP32-S3

**Proyecto:** Control Coche Marcos
**Placa:** ESP32-S3-DevKitC-1 (módulo WROOM-1-N16R8, 16 MB Flash + 8 MB PSRAM)
**Fuente:** `docs/PIN_USAGE_INVENTORY.md`, `docs/PINES_PANTALLA.md`
**Fecha:** 2026-02-26

---

## 1. Pines que NO existen en la ESP32-S3

Estos GPIO **no existen** en el chip ESP32-S3 (sí existen en el ESP32 original, pero NO en la variante S3):

| GPIO | Estado |
|------|--------|
| GPIO22 | **No existe** en ESP32-S3 |
| GPIO23 | **No existe** en ESP32-S3 |
| GPIO24 | **No existe** en ESP32-S3 |
| GPIO25 | **No existe** en ESP32-S3 |

> **Total: 4 pines inexistentes**

---

## 2. Pines reservados para QSPI Flash (NO usar)

Conectados internamente al bus QSPI Flash. **No accesibles** en el módulo — usar cualquiera de ellos provoca un crash inmediato:

| GPIO | Función interna |
|------|----------------|
| GPIO26 | Bus QSPI Flash — CLK |
| GPIO27 | Bus QSPI Flash — DATA0 |
| GPIO28 | Bus QSPI Flash — DATA1 |
| GPIO29 | Bus QSPI Flash — DATA2 |
| GPIO30 | Bus QSPI Flash — DATA3 |
| GPIO31 | Bus QSPI Flash — CS |
| GPIO32 | Bus QSPI Flash — adicional |

> **Total: 7 pines reservados para Flash**

---

## 3. Pines reservados para Octal PSRAM (NO usar)

Conectados internamente al bus Octal PSRAM en el módulo WROOM-1-N16R8. Estos GPIO **sí existen** en el chip ESP32-S3, pero en el módulo N16R8 están conectados al bus de PSRAM y **no tienen salida a pines físicos**:

| GPIO | Función interna |
|------|----------------|
| GPIO33 | Bus Octal PSRAM (N16R8) |
| GPIO34 | Bus Octal PSRAM (N16R8) |
| GPIO35 | Bus Octal PSRAM (N16R8) |
| GPIO36 | Bus Octal PSRAM (N16R8) |
| GPIO37 | Bus Octal PSRAM (N16R8) |

> **Total: 5 pines reservados para PSRAM**
>
> Fuente: *Espressif ESP32-S3 Hardware Design Guidelines*

---

## 4. Pines de boot strapping (usar con precaución ⚠️)

Estos pines tienen funciones especiales durante el arranque (boot). Se pueden usar como GPIO general, pero **con precaución** — un nivel eléctrico incorrecto durante el boot puede impedir que la ESP32-S3 arranque:

| GPIO | Restricción |
|------|-------------|
| GPIO0 | **Boot strapping** — nivel durante boot selecciona modo (no cargar con pull-down fuerte) |
| GPIO3 | **Boot strapping** — JTAG signal source |
| GPIO45 | **Boot strapping** — VDD_SPI voltage select (si se pone HIGH al boot, selecciona 1.8V y puede causar reinicios aleatorios) |
| GPIO46 | **Boot strapping** — solo funciona como **input** (no se puede usar como output) |

> **Total: 4 pines con restricciones de boot**

---

## 5. Pines ya asignados por el proyecto (NO usar para otra función)

Estos pines están en uso activo por el firmware y el hardware del proyecto:

| GPIO | Módulo | Función |
|------|--------|---------|
| GPIO4 | CAN bus | TWAI TX → TJA1051 TXD |
| GPIO5 | CAN bus | TWAI RX → TJA1051 RXD |
| GPIO10 | Display TFT | SPI CS (chip select display) |
| GPIO12 | Display TFT | SPI MISO (datos desde display/touch) |
| GPIO13 | Display TFT | SPI MOSI (datos al display) |
| GPIO14 | Display TFT | SPI SCLK (reloj) |
| GPIO15 | Interruptor tracción | Selector 2WD/4WD (DPDT rocker switch) |
| GPIO16 | Display TFT | BL (Backlight) |
| GPIO19 | DFPlayer Mini | UART1 TX (comandos audio) — Phase 5 |
| GPIO20 | DFPlayer Mini | UART1 RX (respuestas audio) — Phase 5 |
| GPIO21 | Panel táctil | TOUCH_CS (chip select touch) |
| GPIO38 | Display TFT | RST (Reset display) |
| GPIO39 | Display TFT | DC (Data/Command) |
| GPIO43 | LEDs WS2812B | Tira trasera 16 LEDs — Phase 3 |
| GPIO44 | Sensor obstáculos | TOFSense UART2 RX — Phase 3 |
| GPIO47 | LEDs WS2812B | Tira frontal 28 LEDs — Phase 3 |

> **Total: 16 pines asignados por el proyecto**

---

## 6. Tabla resumen completa

| Categoría | GPIOs afectados | Cantidad |
|-----------|----------------|----------|
| **No existen** en ESP32-S3 | GPIO22–25 | 4 |
| **Reservados Flash** (QSPI, crash si se usan) | GPIO26–32 | 7 |
| **Reservados PSRAM** (Octal, N16R8) | GPIO33–37 | 5 |
| **Boot strapping** (usar con precaución) | GPIO0, GPIO3, GPIO45, GPIO46 | 4 |
| **Asignados al proyecto** (ya en uso) | GPIO4, 5, 10, 12–16, 19–21, 38, 39, 43, 44, 47 | 16 |
| **TOTAL NO DISPONIBLES** | — | **36** |
| **Pines LIBRES sin restricciones** | GPIO1, 2, 6, 7, 8, 9, 11, 17, 18, 40, 41, 42, 48 | **13** |

---

## 7. Pines LIBRES (disponibles para expansión)

| GPIO | Funciones disponibles |
|------|----------------------|
| GPIO1 | ADC1_CH1, GPIO |
| GPIO2 | ADC1_CH2, GPIO |
| GPIO6 | GPIO, SPI |
| GPIO7 | GPIO, SPI |
| GPIO8 | GPIO, SPI |
| GPIO9 | GPIO |
| GPIO11 | GPIO, SPI |
| GPIO17 | GPIO |
| GPIO18 | GPIO |
| GPIO40 | GPIO |
| GPIO41 | GPIO |
| GPIO42 | GPIO |
| GPIO48 | GPIO |

> **Total: 13 pines libres sin restricciones** para uso inmediato.

---

## Referencias

- `docs/PIN_USAGE_INVENTORY.md` — Inventario completo de pines (secciones 4, 5 y 6)
- `docs/PINES_PANTALLA.md` — Listado de pines del display (sección 6: pines que NO se deben usar)
- `docs/DIAGRAMA_PINES_VISUAL.md` — Diagramas visuales de conexiones
- `docs/CONEXIONES_RAPIDAS_ESP32.md` — Referencia rápida de conexiones
- [ESP32-S3 Datasheet (Espressif)](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)

---

_Última actualización: 2026-02-26_
