# ESP32-S3 — Display, CAN y periféricos HMI

> Documento principal de montaje del lado ESP32-S3.
>
> Fuente verificada contra firmware real:
> `esp32/include/User_Setup.h`, `esp32/platformio.ini`, `esp32/src/main.cpp`,
> `esp32/src/audio_manager.h`, `esp32/src/relay_audio.h`, `esp32/src/power_manager.h`,
> `esp32/src/shifter_input.h`, `esp32/src/sensors/obstacle_sensor.h`

---

## 1. Resumen de pines usados

| GPIO | Uso |
|------|-----|
| 4 | CAN TX |
| 5 | CAN RX |
| 8 | MCP23017 SDA |
| 9 | MCP23017 SCL |
| 10 | TFT_CS |
| 11 | Relé audio |
| 12 | TFT_MISO + Touch T_DO |
| 13 | TFT_MOSI + Touch T_DIN |
| 14 | TFT_SCLK + Touch T_CLK |
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

## 2. Pantalla TFT ST7796 + Touch XPT2046

### Display

| Señal display | GPIO ESP32 | Notas |
|---------------|------------|-------|
| CS | 10 | Chip select |
| SDO / MISO | 12 | Compartido con touch T_DO |
| SDI / MOSI | 13 | Compartido con touch T_DIN |
| SCK | 14 | Compartido con touch T_CLK |
| RESET | 38 | Reset |
| DC / RS | 39 | Data/Command |
| BL / LED | 42 | Backlight |
| VCC | 3.3V | Alimentación |
| GND | GND | Tierra |

### Touch

| Señal touch | GPIO ESP32 | Notas |
|-------------|------------|-------|
| T_CS | 21 | Chip select touch |
| T_DIN | 13 | Compartido con MOSI |
| T_CLK | 14 | Compartido con SCK |
| T_DO | 12 | Compartido con MISO |
| T_IRQ | No usar | El firmware no lo usa |

### Parámetros de firmware

- Driver: `ST7796_DRIVER`
- Puerto SPI: `USE_HSPI_PORT`
- `SPI_FREQUENCY = 40 MHz`
- `SPI_READ_FREQUENCY = 20 MHz`
- `SPI_TOUCH_FREQUENCY = 2.5 MHz`
- Rotación: `TFT_ROTATION = 1`

---

## 3. CAN Bus (lado ESP32)

### Transceiver correcto

- **ESP32-S3:** `SN65HVD230`
- **STM32:** `TJA1051T/3`

> El lado ESP32 **no** usa TJA1051 en la configuración actual del firmware.
> La referencia correcta es `SN65HVD230` alimentado a **3.3 V**.

### Conexiones ESP32 ↔ SN65HVD230

| ESP32 | SN65HVD230 | Función |
|-------|------------|---------|
| GPIO 4 | TXD | CAN TX |
| GPIO 5 | RXD | CAN RX |
| 3.3V | VCC | Alimentación |
| GND | GND | Tierra |
| GND | RS / slope control | Modo alta velocidad |
| CANH | CANH | Bus CAN alto |
| CANL | CANL | Bus CAN bajo |

### Reglas del bus

- Bitrate: **500 kbps**
- Terminación: **120 Ω** en cada extremo del bus
- CANH/CANL en **par trenzado**
- No conectar CANH/CANL directamente al ESP32

---

## 4. Sensor de obstáculos (actual)

### TF-Mini Plus

| Señal sensor | GPIO / alimentación | Notas |
|--------------|---------------------|-------|
| VCC | 5V | Alimentación |
| GND | GND | Tierra |
| TX | GPIO 18 | UART1 RX |

- UART: **115200 bps**
- Conexión: **directa**
- Nivel lógico: **3.3 V TTL**
- **No usar divisor de tensión**

> El `TOFSense-M` queda como sensor alternativo/histórico. El firmware activo usa
> `TF-Mini Plus`.

---

## 5. Audio

### DFPlayer Mini

| Señal | GPIO ESP32 | Notas |
|-------|------------|-------|
| ESP32 TX → DFPlayer RX | 43 | UART2 |
| DFPlayer TX → ESP32 RX | 44 | UART2 |
| VCC | 5V | Alimentación |
| GND | GND | Tierra |

### Relé de audio

| Señal | GPIO ESP32 | Notas |
|-------|------------|-------|
| IN relé audio | 11 | **Active LOW** |

---

## 6. Palanca de cambios (MCP23017)

| Señal | GPIO ESP32 | Notas |
|-------|------------|-------|
| SDA | 8 | I2C |
| SCL | 9 | I2C |
| A0 / A1 / A2 | GND | Dirección 0x20 |
| VDD | 3.3V | Alimentación |
| GND | GND | Tierra |

**Extras recomendados:**
- 4.7 kΩ pull-up en SDA
- 4.7 kΩ pull-up en SCL
- 10 kΩ pull-up en RESET
- 10 µF + 100 nF entre VDD y GND

---

## 7. Ignición y apagado ordenado

| Señal | GPIO ESP32 | Notas |
|-------|------------|-------|
| IGNITION_SENSE | 40 | Entrada optoacoplada, lógica invertida |
| POWER_HOLD | 41 | Salida interna del firmware |

### GPIO 40

- Entrada por **módulo NPN optoacoplador / EL817**
- `LOW = llave ON`
- `HIGH = llave OFF`
- `INPUT_PULLUP` en firmware
- 10 kΩ externo a 3.3 V recomendado

> `GPIO 41` no debe cablearse como sustituto de la llave. Su uso actual es interno
> del firmware; conectarlo externamente puede interferir con la secuencia de apagado
> ordenado y con la lógica de `POWER_HOLD`.

---

## 8. WS2812B

| Señal | GPIO ESP32 | Notas |
|-------|------------|-------|
| Datos tira frontal | 47 | 28 LEDs |
| Datos tira trasera | 48 | 16 LEDs |

**Extras recomendados por tira:**
- 330 Ω en serie con DIN
- 1000 µF entre 5V y GND cerca de la tira

---

## 9. Componentes a montar en el lado ESP32

| Cantidad | Componente | Notas |
|----------|------------|-------|
| 1 | ESP32-S3 DevKitC-1 N16R8 | MCU HMI |
| 1 | Display ST7796 + XPT2046 | 480×320 |
| 1 | SN65HVD230 | Transceiver CAN ESP32 |
| 1 | TF-Mini Plus | Sensor de obstáculos actual |
| 1 | DFPlayer Mini | Audio |
| 1 | Relé audio | GPIO11, active LOW |
| 1 | MCP23017 | Palanca de cambios |
| 2 | Resistencias 4.7 kΩ | Pull-up I2C |
| 1 | Resistencia 10 kΩ | Pull-up RESET MCP23017 |
| 1 | Resistencia 10 kΩ | Pull-up recomendado GPIO40 |
| 2 | Resistencias 330 Ω | Datos WS2812B |
| 2 | Capacitores 1000 µF | Tiras LED |
| 1 | Capacitor 100 nF | VCC TF-Mini Plus |
| 1 | Capacitor 100 nF | VCC SN65HVD230 |
| 1 | Capacitor 10 µF | VCC SN65HVD230 / rail ESP32 CAN |
| 1 | Capacitor 10 µF | VDD MCP23017 |
| 1 | Capacitor 100 nF | VDD MCP23017 |

---

## 10. Verificación rápida

### Multímetro

| Punto | Valor esperado |
|-------|----------------|
| 3.3V ESP32 → GND | 3.30V ±0.1V |
| VCC SN65HVD230 → GND | 3.30V ±0.1V |
| VCC TFT → GND | 3.30V ±0.1V |
| VCC TF-Mini Plus → GND | 5.00V ±0.2V |
| CANH ↔ CANL (sin alimentar) | ~60 Ω |

### Serial

- `[HMI] ESP32 HMI CAN bring-up booted`
- `[TFT] Display initialized`
- `[CAN] Initialized at 500 kbps`
- `[OBSTACLE] TF-Mini Plus init ...`

---

## 11. Diagrama rápido

```
ESP32-S3
├── GPIO4 / GPIO5 ── SN65HVD230 ── CANH/CANL ── TJA1051T/3 ── STM32 PA12/PA11
├── GPIO10/12/13/14/21/38/39/42 ── ST7796 + XPT2046
├── GPIO18 ── TF-Mini Plus TX
├── GPIO43 / GPIO44 ── DFPlayer Mini
├── GPIO11 ── Relé audio
├── GPIO8 / GPIO9 ── MCP23017
├── GPIO40 ── IGNITION_SENSE (EL817, LOW=ON)
├── GPIO41 ── POWER_HOLD
├── GPIO47 ── WS2812B frontal
└── GPIO48 ── WS2812B trasera
```

---

Ver también:
- `CONEXIONES_RAPIDAS_ESP32.md`
- `TFMINI_PLUS_WIRING_GUIDE.md`
- `EL817_WIRING_REFERENCE.md`
- `LLAVE_CONTACTO_ENCENDIDO_APAGADO.md`
