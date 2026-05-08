# Índice de Documentación - Conexiones ESP32-S3

Índice consolidado de los documentos de montaje del **ESP32-S3 HMI**.

> Fuente verificada:
> `esp32/include/User_Setup.h`, `esp32/platformio.ini`, `esp32/src/main.cpp`,
> `esp32/src/audio_manager.h`, `esp32/src/relay_audio.h`, `esp32/src/shifter_input.h`,
> `esp32/src/power_manager.h`, `esp32/src/sensors/obstacle_sensor.h`

---

## 📄 Documentos disponibles

### 1. `ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md` ⭐ DOCUMENTO PRINCIPAL
Guía detallada del lado ESP32:

- ✅ Pantalla ST7796 + touch XPT2046
- ✅ CAN lado ESP32 con **TJA1051T/3** (GPIO4/GPIO5)
- ✅ Audio DFPlayer + relé GPIO11
- ✅ Sensor actual **TF-Mini Plus** (GPIO18, 115200 bps)
- ✅ Verificaciones y troubleshooting

**Usar para:** montaje completo del ESP32-S3.

---

### 2. `CONEXIONES_RAPIDAS_ESP32.md` ⚡ REFERENCIA RÁPIDA
Resumen compacto de:

- ✅ Display + touch
- ✅ CAN con **TJA1051T/3**
- ✅ TF-Mini Plus
- ✅ Relé audio GPIO11
- ✅ MCP23017 en GPIO8/GPIO9
- ✅ GPIO40/GPIO41 (ignición / power hold)

**Usar para:** consulta rápida en banco o taller.

---

### 3. `PINES_PANTALLA.md`
Listado detallado del display y touch.

**Usar para:** cableado fino de pantalla.

---

### 4. `TFMINI_PLUS_WIRING_GUIDE.md` 🎯 SENSOR ACTUAL
Guía del sensor de obstáculos **actual** en firmware:

- ✅ TF-Mini Plus
- ✅ GPIO18 / UART1 RX
- ✅ 115200 bps
- ✅ **Sin divisor de tensión**

**Usar para:** montaje del sensor de obstáculos actual.

---

### 5. `TOFSENSE_M_WIRING_GUIDE.md` / `CONEXION_TOF_SENSE_M_LIDAR.md` ⚠ HISTÓRICO / OPCIONAL
Documentación mantenida como referencia del sensor anterior / alternativo:

- TOFSense-M
- GPIO18
- 921600 bps
- divisor o level shifter

> **No es el sensor activo del firmware actual.** El firmware actual usa **TF-Mini Plus**.

---

### 6. `LLAVE_CONTACTO_ENCENDIDO_APAGADO.md`
Circuito de llave de contacto y apagado ordenado.

- ✅ GPIO40 = `IGNITION_SENSE`
- ✅ GPIO41 = `POWER_HOLD`
- ✅ Entrada vía **módulo NPN optoacoplador / EL817**
- ✅ Pull-up externo 10 kΩ recomendado en GPIO40

> La solución actual **no** es un divisor resistivo directo 12V→3.3V; es una entrada
> optoacoplada de lógica invertida.

---

### 7. `EL817_WIRING_REFERENCE.md`
Referencia del módulo optoacoplador NPN:

- ✅ llave de contacto a GPIO40
- ✅ lógica invertida LOW=ON
- ✅ pinout del módulo

---

### 8. `DIAGRAMA_PINES_VISUAL.md`
Diagramas ASCII y vista general de cableado.

---

## 🎯 ¿Qué documento usar?

### Para montar desde cero
1. `ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md`
2. `PINES_PANTALLA.md`
3. `TFMINI_PLUS_WIRING_GUIDE.md`
4. `LLAVE_CONTACTO_ENCENDIDO_APAGADO.md`
5. `CONEXIONES_RAPIDAS_ESP32.md`

### Para verificación rápida
- `CONEXIONES_RAPIDAS_ESP32.md`

### Para la llave / alimentación
- `LLAVE_CONTACTO_ENCENDIDO_APAGADO.md`
- `EL817_WIRING_REFERENCE.md`

### Para el sensor antiguo / alternativo
- `TOFSENSE_M_WIRING_GUIDE.md`
- `CONEXION_TOF_SENSE_M_LIDAR.md`

---

## 📌 Resumen de GPIO ESP32-S3

| GPIO | Uso |
|------|-----|
| 4 | CAN TX |
| 5 | CAN RX |
| 8 | MCP23017 SDA |
| 9 | MCP23017 SCL |
| 10 | TFT CS |
| 11 | Relé audio |
| 12 | TFT MISO + Touch T_DO |
| 13 | TFT MOSI + Touch T_DIN |
| 14 | TFT SCK + Touch T_CLK |
| 18 | TF-Mini Plus TX → UART1 RX |
| 21 | TOUCH_CS |
| 38 | TFT_RST |
| 39 | TFT_DC |
| 40 | IGNITION_SENSE |
| 41 | POWER_HOLD |
| 42 | TFT_BL |
| 43 | DFPlayer RX |
| 44 | DFPlayer TX |
| 47 | WS2812B frontal |
| 48 | WS2812B trasera |

---

## ⚠️ Reglas rápidas

- ESP32 CAN: **TJA1051T/3**
- Sensor actual: **TF-Mini Plus**, no TOFSense-M
- GPIO18: UART1 RX del sensor de obstáculos
- GPIO40: entrada optoacoplada invertida (`LOW = llave ON`)
- GPIO41: uso interno firmware, no cablear a ciegas
- MCP23017: GPIO8/GPIO9, dirección I2C `0x20`
