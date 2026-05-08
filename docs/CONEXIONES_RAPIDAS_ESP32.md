# REFERENCIA RÁPIDA - CONEXIONES ESP32-S3
## Pantalla, CAN, audio, sensor obstáculos, palanca e ignición

> Fuente verificada contra firmware:
> `esp32/include/User_Setup.h`, `esp32/platformio.ini`, `esp32/src/main.cpp`,
> `esp32/src/audio_manager.h`, `esp32/src/relay_audio.h`, `esp32/src/shifter_input.h`,
> `esp32/src/power_manager.h`, `esp32/src/sensors/obstacle_sensor.h`

---

## 1. PANTALLA TFT ST7796 + TOUCH XPT2046

### Display

| Módulo | → | ESP32-S3 GPIO | Función |
|--------|---|---------------|---------|
| VCC | → | 3.3V | Alimentación |
| GND | → | GND | Tierra |
| CS | → | GPIO 10 | Chip Select display |
| RESET | → | GPIO 38 | Reset |
| DC/RS | → | GPIO 39 | Data / Command |
| SDI (MOSI) | → | GPIO 13 | Datos SPI |
| SCK | → | GPIO 14 | Reloj SPI |
| SDO (MISO) | → | GPIO 12 | Datos SPI |
| LED / BL | → | GPIO 42 | Backlight |

### Touch

| Módulo | → | ESP32-S3 GPIO | Función |
|--------|---|---------------|---------|
| T_CS | → | GPIO 21 | Touch chip select |
| T_DIN | → | GPIO 13 | Compartido con MOSI |
| T_CLK | → | GPIO 14 | Compartido con SCK |
| T_DO | → | GPIO 12 | Compartido con MISO |
| T_IRQ | → | No conectar | No usado por el firmware |

---

## 2. CAN BUS (ESP32-S3)

### Transceiver usado en el lado ESP32

- **TJA1051T/3**
- Alimentación **3.3 V**
- Bus clásico CAN 2.0A a **500 kbps**

> El lado STM32 usa `TJA1051T/3` con `PA11/PA12`. En el lado ESP32 también se usa
> `TJA1051T/3` con `GPIO4/GPIO5`, alimentado a 3.3V.

### Conexiones ESP32 ↔ TJA1051T/3

| ESP32-S3 | → | TJA1051T/3 | Función |
|----------|---|------------|---------|
| GPIO 4 | → | TXD (pin 1) | CAN TX |
| GPIO 5 | → | RXD (pin 4) | CAN RX |
| 3.3V | → | VCC (pin 3) | Alimentación |
| 3.3V | → | VIO (pin 5) | Nivel lógico |
| GND | → | GND (pin 2) | Tierra |
| GND | → | S (pin 8) | Modo normal |
| CANH | ↔ | CANH (pin 7) | Bus CAN alto |
| CANL | ↔ | CANL (pin 6) | Bus CAN bajo |

### Bus físico

| Señal | Notas |
|-------|-------|
| CANH | Par trenzado |
| CANL | Par trenzado |
| 120 Ω | Una resistencia en cada extremo del bus |

---

## 3. SENSOR DE OBSTÁCULOS (TF-Mini Plus)

| TF-Mini Plus | → | ESP32-S3 / Fuente | Función |
|--------------|---|-------------------|---------|
| VCC (rojo) | → | 5V | Alimentación |
| GND (negro) | → | GND | Tierra |
| TX (verde) | → | GPIO 18 | UART1 RX |

- **Baudrate:** 115200 bps
- **Conexión:** directa, **sin divisor**
- **Nivel lógico:** 3.3 V TTL nativo

> El `TOFSense-M` sigue documentado en archivos históricos/alternativos, pero el firmware
> activo usa **TF-Mini Plus en GPIO18 a 115200 bps**.

---

## 4. AUDIO

### DFPlayer Mini

| DFPlayer | → | ESP32-S3 | Función |
|----------|---|----------|---------|
| RX | ← | GPIO 43 | UART2 TX del ESP32 |
| TX | → | GPIO 44 | UART2 RX del ESP32 |
| VCC | → | 5V | Alimentación |
| GND | → | GND | Tierra |

### Relé de audio

| Señal | → | ESP32-S3 | Notas |
|-------|---|----------|-------|
| IN relé audio | ← | GPIO 11 | **Active LOW**: LOW = relé ON |

---

## 5. PALANCA DE CAMBIOS (MCP23017)

| Señal | → | ESP32-S3 | Función |
|-------|---|----------|---------|
| SDA | ↔ | GPIO 8 | I2C datos |
| SCL | ↔ | GPIO 9 | I2C reloj |
| VDD | → | 3.3V | Alimentación |
| GND | → | GND | Tierra |
| A0 / A1 / A2 | → | GND | Dirección I2C = 0x20 |

**Extras recomendados:**
- Pull-up SDA 4.7 kΩ a 3.3 V
- Pull-up SCL 4.7 kΩ a 3.3 V
- RESET pull-up 10 kΩ a 3.3 V
- 10 µF + 100 nF entre VDD y GND del MCP23017

---

## 6. IGNICIÓN Y RETENCIÓN DE ALIMENTACIÓN

| Señal | → | ESP32-S3 | Función |
|-------|---|----------|---------|
| IGNITION_SENSE | → | GPIO 40 | Entrada, lógica invertida |
| POWER_HOLD | ← | GPIO 41 | Salida interna firmware |

### GPIO 40 (llave)

- Se conecta a través de **módulo NPN optoacoplador 4ch / EL817**
- **LOW = llave ON**
- **HIGH = llave OFF**
- `INPUT_PULLUP` en firmware
- Pull-up externo **10 kΩ a 3.3 V** recomendado

> **GPIO 41 no se conecta al módulo de retardo ni a ningún componente externo** salvo si se
> documenta expresamente en una revisión futura. El firmware lo usa como señal interna.

---

## 7. RESUMEN RÁPIDO DE GPIO USADOS

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
| 43 | DFPlayer RX ← ESP32 TX |
| 44 | DFPlayer TX → ESP32 RX |
| 47 | WS2812B frontal |
| 48 | WS2812B trasera |

---

## 8. VERIFICACIÓN RÁPIDA

### Con multímetro

1. 3.3V ESP32 → GND: **3.30V ±0.1V**
2. TFT VCC → GND: **3.30V ±0.1V**
3. TJA1051T/3 VCC → GND: **3.30V ±0.1V**
4. TF-Mini Plus VCC → GND: **5.00V ±0.2V**
5. CANH → GND (idle): **~2.5V**
6. CANL → GND (idle): **~2.5V**
7. CANH ↔ CANL: **~60 Ω** con las dos terminaciones montadas

### En monitor serie

1. `[HMI] ESP32 HMI CAN bring-up booted`
2. `[TFT] Display initialized`
3. `[CAN] Initialized at 500 kbps`
4. `[OBSTACLE] TF-Mini Plus init (UART1, 115200 bps...)`

---

## 9. DIAGRAMA SIMPLE

```
┌─────────────┐
│  ESP32-S3   │
│             │
│ GPIO12 ◄────┼── Display MISO + Touch DO
│ GPIO13 ─────┼── Display MOSI + Touch DIN
│ GPIO14 ─────┼── Display SCK  + Touch CLK
│ GPIO10 ─────┼── Display CS
│ GPIO39 ─────┼── Display DC
│ GPIO38 ─────┼── Display RESET
│ GPIO21 ─────┼── Touch CS
│ GPIO42 ─────┼── Backlight
│             │
│ GPIO4  ─────┼── TJA1051T/3 TXD
│ GPIO5  ◄────┼── TJA1051T/3 RXD
│ 3.3V   ─────┼── TJA1051T/3 VCC + VIO
│             │
│ GPIO18 ◄────┼── TF-Mini Plus TX
│ GPIO11 ─────┼── Relé audio IN (active LOW)
│ GPIO8  ─────┼── MCP23017 SDA
│ GPIO9  ─────┼── MCP23017 SCL
│ GPIO40 ◄────┼── IGNITION_SENSE
│ GPIO41 ─────┼── POWER_HOLD
│ GPIO43 ─────┼── DFPlayer RX
│ GPIO44 ◄────┼── DFPlayer TX
│ GPIO47 ─────┼── WS2812B frontal
│ GPIO48 ─────┼── WS2812B trasera
└─────────────┘
```

---

Ver también:
- `ESP32_S3_DISPLAY_Y_CAN_CONEXIONES.md`
- `TFMINI_PLUS_WIRING_GUIDE.md`
- `EL817_WIRING_REFERENCE.md`
- `LLAVE_CONTACTO_ENCENDIDO_APAGADO.md`
