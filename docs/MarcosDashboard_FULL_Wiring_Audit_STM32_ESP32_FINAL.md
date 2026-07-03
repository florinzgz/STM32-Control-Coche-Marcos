# MarcosDashboard — AUDITORÍA TOTAL DE CABLEADO
## STM32 NUCLEO-G474RE (D-1686) + ESP32-S3 HMI

**Versión:** FINAL 2026-06-16  
**Proyecto:** STM32-Control-Coche-Marcos / MarcosDashboard  
**Fuentes verificadas:** `Core/Inc/project_config.h`, `Core/Src/main.c`, `Core/Src/motor_control.c`, `Core/Src/safety_system.c`, `esp32/src/main.cpp`, `esp32/include/User_Setup.h`, `esp32/src/relay_audio.h`, `esp32/src/traction_switch.h`, `esp32/src/remote_control.h`, `esp32/src/led_controller.h`, `esp32/src/power_manager.h`, `docs/STM32_BREAKOUT_BOARD_WIRING.md`, `docs/POWER_DISTRIBUTION.md`

> ⛔ **REGLA:** Nada está inventado. Cada pin tiene fuente exacta. Si algo no está confirmado al 100 % se indica como **REQUIERE VERIFICACIÓN**.

---

## LEYENDA DE ESTADOS

| Símbolo | Significado |
|---------|-------------|
| ✅ VERDE | Conexión directa segura (3.3 V, sin adaptación) |
| 🟠 NARANJA | Requiere adaptación/protección antes de conectar |
| 🔴 ROJO | Peligro — no conectar directamente |
| 🔵 AZUL | Va a ESP32-S3, NO a STM32 |
| 🟣 MORADO | Alimentación / potencia |
| ⚪ GRIS | Opcional / no instalado todavía |

---

## ÍNDICE

1. [Resumen ejecutivo](#1-resumen-ejecutivo)
2. [TABLA MAESTRA STM32 + D-1686](#2-tabla-maestra-stm32--d-1686)
3. [TABLA MAESTRA ESP32-S3](#3-tabla-maestra-esp32-s3)
4. [Alimentación general](#4-alimentación-general)
5. [Relés y ULN2803A](#5-relés-y-uln2803a)
6. [Relés y contactores de potencia](#6-relés-y-contactores-de-potencia)
7. [Motores BTS7960](#7-motores-bts7960)
8. [Sensores STM32](#8-sensores-stm32)
9. [CAN Bus STM32 ↔ ESP32-S3](#9-can-bus-stm32--esp32-s3)
10. [Componentes exclusivos ESP32-S3](#10-componentes-exclusivos-esp32-s3)
11. [Lista NO conectar nunca](#11-lista-no-conectar-nunca)
12. [Checklist final antes de encender](#12-checklist-final-antes-de-encender)
13. [Errores / conflictos detectados](#13-errores--conflictos-detectados)

---

## 1. RESUMEN EJECUTIVO

### Qué va a STM32 (por breakout D-1686)
- Alimentación E5V desde buck 5V
- 5× BTS7960 (FL, FR, RL, RR, STEER): RPWM, LPWM, EN
- 4× relés de potencia (tracción 24V, dirección 12V, LED frontal, LED trasero) vía ULN2803A
- CAN FDCAN1 TX/RX → transceptor TJA1051 → ESP32-S3
- I2C (PB8/PB9) → TCA9548A → 6× INA226
- Encoder E6B2-CWZ6C A/B/Z → por 6N137 optoacopladores
- Sensor centro dirección PB5 (LJ12A3) → por PC817 opto
- 4× sensores rueda FL/FR/RL/RR (LJ12A3) → por PC817 opto
- Pedal Hall ADC PB1 → divisor 10kΩ+6.8kΩ
- DS18B20 OneWire PB0 → 5 sensores en bus único

### Qué va a ESP32-S3 (NO a STM32)
- Pantalla ST7796 480×320 (SPI HSPI)
- Touch XPT2046 (SPI compartido)
- Backlight TFT GPIO42 PWM
- MCP23017 palanca de cambios (I2C GPIO8/9)
- Interruptor 4x2/4x4 GPIO15
- DFPlayer Mini (UART2 GPIO43/44)
- Relé audio GPIO11 → ULN2803A → módulo relé 5V
- Sensor obstáculos TFMini Plus (UART1 GPIO18)
- Mando RC FlySky FS-iA6B iBUS (UART0 GPIO16)
- WS2812B datos: frontal GPIO47, trasero GPIO48
- Llave contacto GPIO40 (entrada, opto), GPIO41 (salida hold)
- CAN TWAI: TX=GPIO4, RX=GPIO5 → transceptor TJA1051

### Qué requiere adaptación/protección
| Señal | Razón | Adaptación |
|-------|-------|-----------|
| Encoder 5V A/B/Z | STM32 GPIO = 3.3V máx | 6N137 opto por señal |
| LJ12A3 sensores rueda | Salida NPN 6–36V | PC817 opto |
| LJ12A3 centro dirección | Salida NPN 6–36V | PC817 opto |
| Pedal Hall 5V | ADC STM32 = 3.3V máx | Divisor 10kΩ + 6.8kΩ |
| Relés desde STM32 | GPIO 3.3V no conduce bobina 12V | ULN2803A |

---

## 2. TABLA MAESTRA STM32 + D-1686

> **Columnas D-1686:** CN10_N = columna izquierda D-1686 · CN7_N = columna derecha D-1686

### 2.1 Alimentación

| Función | Pin STM32 | Macro firmware | Borne D-1686 | Cable va a | Tensión | Protección | Estado | Fuente |
|---------|-----------|----------------|--------------|-----------|---------|------------|--------|--------|
| 🟣 E5V entrada | E5V | — | **CN7_6** | Buck +5V positivo | 5V entrada | 100µF 35V + 100nF en borne | ✅ | `STM32_BREAKOUT_BOARD_WIRING.md §6.1` |
| 🟣 GND principal | GND | — | **CN7_8** | Buck GND / estrella | 0V | — | ✅ | `STM32_BREAKOUT_BOARD_WIRING.md §6.2` |
| 🟣 GND alternativo | GND | — | **CN10_9** | GND estrella | 0V | — | ✅ | `STM32_BREAKOUT_BOARD_WIRING.md §6.2` |
| 🟣 AGND analógico | AGND | — | **CN10_32** | GND estrella | 0V | Unir al GND digital | ✅ | `STM32_BREAKOUT_BOARD_WIRING.md §6.2` |
| 🟣 3.3V salida | VDD | — | **CN7_5** | TCA9548A VCC, INA226 VCC, BTS7960 VCC lógico | 3.3V salida | Máx 300mA total | ✅ | `STM32_BREAKOUT_BOARD_WIRING.md §6.1` |
| 🟣 +5V salida | +5V | — | **CN7_18** | TJA1051 VCC, optos 5V side, encoder | 5V salida | Máx 500mA; solo con USB/VIN | ✅ | `STM32_BREAKOUT_BOARD_WIRING.md §6.1` |
| 🔴 VIN NO USAR | VIN | — | **CN7_24** | ❌ NO CONECTAR con buck 5V | 7–12V esperados | — | ❌ | `STM32_BREAKOUT_BOARD_WIRING.md §6.4` |
| 🔴 AVDD NO USAR | AVDD | — | **CN10_7** | ❌ NUNCA CONECTAR | Ref interna ADC | — | ❌ | `STM32_BREAKOUT_BOARD_WIRING.md §6.4` |
| 🔴 U5V NO USAR | U5V | — | **CN10_8** | ❌ NUNCA USAR como entrada | 5V salida USB | — | ❌ | `STM32_BREAKOUT_BOARD_WIRING.md §6.4` |

> **JP5 Nucleo:** debe estar en posición **E5V** (pines 2-3) antes de alimentar por E5V. Si JP5 está en USB con buck conectado → conflicto destructivo.

### 2.2 CAN Bus (FDCAN1)

| Función | Pin STM32 | Macro firmware | Borne D-1686 | Cable va a | Tensión | Protección | Estado | Fuente |
|---------|-----------|----------------|--------------|-----------|---------|------------|--------|--------|
| CAN_TX | PA12 | `PIN_CAN_TX` / AF9 | **CN10_12** | TJA1051T/3 pin TXD | 3.3V | TJA1051 VCC=3.3V | ✅ | `project_config.h:358` |
| CAN_RX | PA11 | `PIN_CAN_RX` / AF9 | **CN10_14** | TJA1051T/3 pin RXD | 3.3V | ⚠️ PA11 ≠ LPWM_FR | ✅ | `project_config.h:359` |

**TJA1051T/3:** VCC=3.3V, VIO=3.3V (o puentear VIO=VCC), GND=GND común, CAN_H/CAN_L → bus.  
**Terminación 120Ω** en extremos físicos del bus: uno en transceptor STM32, otro en transceptor ESP32.  
**STM32 TX → TXD transceptor** (directo, NO cruzado).  
**STM32 RX ← RXD transceptor** (directo, NO cruzado).

### 2.3 I2C (TCA9548A + INA226)

| Función | Pin STM32 | Macro firmware | Borne D-1686 | Cable va a | Tensión | Protección | Estado | Fuente |
|---------|-----------|----------------|--------------|-----------|---------|------------|--------|--------|
| I2C1_SCL | PB8 | `PIN_I2C_SCL` / AF4 | **CN10_3** | TCA9548A SCL | 3.3V OD | Pull-up 4.7kΩ a 3.3V externo OBLIGATORIO | ✅ | `project_config.h:306` |
| I2C1_SDA | PB9 | `PIN_I2C_SDA` / AF4 | **CN10_5** | TCA9548A SDA | 3.3V OD | Pull-up 4.7kΩ a 3.3V externo OBLIGATORIO | ✅ | `project_config.h:307` |

**TCA9548A:** addr=0x70, A0=A1=A2=GND, !RESET=3.3V (siempre HIGH), VCC=3.3V.  
**INA226:** addr=0x40 en cada canal del mux, VCC=3.3V.  
**Firmware usa GPIO_NOPULL** → pull-ups externos son mandatorios o se dispara Error Code 11 (SAFETY_ERROR_I2C_FAILURE).

| Canal INA226 | Mide | Posición circuito | Shunt (mΩ) | Estado con relés |
|-------------|------|-------------------|-----------|-----------------|
| CH0 | Motor FL | Post RELAY_TRAC | 1.5 | Sin tensión si TRAC off |
| CH1 | Motor FR | Post RELAY_TRAC | 1.5 | Sin tensión si TRAC off |
| CH2 | Motor RL | Post RELAY_TRAC | 1.5 | Sin tensión si TRAC off |
| CH3 | Motor RR | Post RELAY_TRAC | 1.5 | Sin tensión si TRAC off |
| CH4 | Batería 24V | Antes de relés | 0.75 | SIEMPRE alimentado |
| CH5 | Motor Dirección | Post RELAY_STEER | 1.5 | Sin tensión si STEER off |

### 2.4 Relés STM32

| Función | Pin STM32 | Macro firmware | Borne D-1686 | Cable va a | Tensión | Protección | Estado | Fuente |
|---------|-----------|----------------|--------------|-----------|---------|------------|--------|--------|
| RELAY_TRAC | PC11 | `PIN_RELAY_TRAC` | **CN7_2** | ULN2803A entrada 4B | 3.3V → ULN | Via ULN canal 4; NUNCA directo | ✅ | `project_config.h:218` |
| RELAY_STEER_PWR | PC12 | `PIN_RELAY_STEER_PWR` | **CN7_3** | ULN2803A entrada 5B | 3.3V → ULN | Via ULN canal 5 | ✅ | `project_config.h:225` |
| RELAY_LED frontal | PB10 | `PIN_RELAY_LED` | **CN10_25** | ULN2803A entrada 1B | 3.3V → ULN | Via ULN canal 1 | ✅ | `project_config.h:235` |
| RELAY_LED trasero | PB11 | `PIN_RELAY_LED_REAR` | **CN10_18** | ULN2803A entrada 2B | 3.3V → ULN | Via ULN canal 2 | ✅ | `project_config.h:236` |

**Temporizador relés:** PC11 se activa 50ms antes de PC12 (`RELAY_TRACTION_SETTLE_MS=50`). `safety_system.c:93-140`.

### 2.5 BTS7960 — Tracción (FL, FR, RL, RR) y Dirección

| Motor | Señal | Pin STM32 | Macro | Borne D-1686 | Timer | Tensión |
|-------|-------|-----------|-------|--------------|-------|---------|
| FL | RPWM | PA8 | `PIN_PWM_FL` / TIM1_CH1 AF6 | **CN10_23** | TIM1 | 3.3V 20kHz |
| FL | LPWM | PA9 | `PIN_LPWM_FL` / TIM1_CH2 AF6 | **CN10_21** | TIM1 | 3.3V 20kHz |
| FL | EN | PC5 | `PIN_EN_FL` | **CN10_6** | GPIO | 3.3V |
| FR | RPWM | PA10 | `PIN_PWM_FR` / TIM1_CH3 AF6 | **CN10_33** | TIM1 | 3.3V 20kHz |
| FR | LPWM | PC3 | `PIN_LPWM_FR` / TIM1_CH4 AF2 | **CN7_37** | TIM1 | 3.3V 20kHz ⚠️ |
| FR | EN | PC0 | `PIN_EN_FR` | **CN7_38** | GPIO | 3.3V |
| RL | RPWM | PC6 | `PIN_PWM_RL` / TIM8_CH1 AF4 | **CN10_4** | TIM8 | 3.3V 20kHz |
| RL | LPWM | PC7 | `PIN_LPWM_RL` / TIM8_CH2 AF4 | **CN10_19** | TIM8 | 3.3V 20kHz |
| RL | EN | PC1 | `PIN_EN_RL` | **CN7_36** | GPIO | 3.3V |
| RR | RPWM | PC8 | `PIN_PWM_RR` / TIM8_CH3 AF4 | **CN10_2** | TIM8 | 3.3V 20kHz |
| RR | LPWM | PC9 | `PIN_LPWM_RR` / TIM8_CH4 AF4 | **CN10_1** | TIM8 | 3.3V 20kHz |
| RR | EN | PC2 | `PIN_EN_RR` | **CN7_35** | GPIO | 3.3V ⚠️ |
| STEER | RPWM | PA6 | `PIN_PWM_STEER` / TIM3_CH1 AF2 | **CN10_13** | TIM3 | 3.3V 20kHz |
| STEER | LPWM | PA7 | `PIN_LPWM_STEER` / TIM3_CH2 AF2 | **CN10_15** | TIM3 | 3.3V 20kHz |
| STEER | EN | PC4 | `PIN_EN_STEER` | **CN10_34** | GPIO | 3.3V |

> ⚠️ **PC3 (LPWM_FR):** fue reasignado desde PA11. PA11 ahora es CAN_RX. NO confundir.  
> ⚠️ **PC2 (EN_RR):** fue reasignado desde PC13. PC13 es el botón USER B1. NO usar PC13.  
> ⚠️ **BTS7960 VCC lógico:** SIEMPRE 3.3V (CN7_5 o CN7_16). Nunca 5V — umbral V_IH(min) 5V = 3.5V > señales STM32.  
> **R_EN y L_EN** del IBT-2: unirlos físicamente y conectar ambos con un solo cable al pin EN correspondiente.

### 2.6 Sensores STM32

| Función | Pin STM32 | Macro | Borne D-1686 | Señal entrada | Tensión pín | Adaptación | Estado | Fuente |
|---------|-----------|-------|--------------|--------------|-------------|-----------|--------|--------|
| PEDAL ADC | PB1 | `PIN_PEDAL` ADC1_IN12 | **CN10_24** | Hall sensor 5V | 0–3.3V | 🟠 Divisor 10kΩ+6.8kΩ OBLIGATORIO | ✅ | `project_config.h` (movido desde PA3/CN10_37 por corto a GND) |
| ENC_A | PA15 | `PIN_ENC_A` TIM2_CH1 AF1 | **CN7_17** | 6N137 salida | 3.3V | 🟠 6N137 por encoder A | ✅ | `project_config.h:292` |
| ENC_B | PB3 | `PIN_ENC_B` TIM2_CH2 AF1 | **CN10_31** | 6N137 salida | 3.3V | 🟠 6N137 por encoder B | ✅ | `project_config.h:293` |
| ENC_Z índice | PB4 | `PIN_ENC_Z` EXTI4 | **CN10_27** | 6N137 salida | 3.3V | 🟠 6N137 por encoder Z | ✅ | `project_config.h:294` |
| STEER_CENTER | PB5 | `PIN_STEER_CENTER` EXTI5 | **CN10_29** | PC817 salida | 3.3V | 🟠 PC817 opto OBLIGATORIO | ✅ | `project_config.h:301` |
| WHEEL_FL | PA0 | `PIN_WHEEL_FL` EXTI0 | **CN7_28** | PC817 salida | 3.3V | 🟠 PC817 opto | ✅ | `project_config.h:241` |
| WHEEL_FR | PA1 | `PIN_WHEEL_FR` EXTI1 | **CN7_30** | PC817 salida | 3.3V | 🟠 PC817 opto | ✅ | `project_config.h:242` |
| WHEEL_RL | PA2 | `PIN_WHEEL_RL` EXTI2 | **CN10_35** | PC817 salida | 3.3V | 🟠 PC817 opto | ✅ | `project_config.h:243` |
| WHEEL_RR | PB15 | `PIN_WHEEL_RR` EXTI15 | **CN10_26** | PC817 salida | 3.3V | 🟠 PC817 opto | ✅ | `project_config.h:244` |
| ONEWIRE DS18B20 | PB0 | `PIN_ONEWIRE` GPIO OD | **CN7_34** | Bus DAT DS18B20 | 3.3V OD | Pull-up único 4.7kΩ a 3.3V | ✅ | `project_config.h:341` |
| LED_DIAG externo | PB14 | — | **CN10_28** | LED externo | 3.3V | Resistencia 330Ω en serie | ✅ | `project_config.h:325` |

### 2.7 Pines NO conectar en la breakout

| Borne | GPIO | Razón |
|-------|------|-------|
| CN7_23 | PC13 | Botón USER B1 de la Nucleo — NO como salida |
| CN10_7 | AVDD | Referencia analógica interna — NUNCA conectar |
| CN10_8 | U5V | Salida USB ST-Link — NUNCA como entrada |
| CN7_24 | VIN | Solo 7–12V; 5V no activa el regulador |
| CN7_29/31 | PF0/PF1 | Cristal principal — NO conectar |
| CN7_25/27 | PC14/PC15 | Cristal RTC — NO conectar |

---

## 3. TABLA MAESTRA ESP32-S3

> **Nota:** Ubicación física en el breakout ESP32-S3 "Placa de Expansión LARVIN" como se ve en la foto adjunta.  
> Para pines específicos: **CONFIRMADO POR FIRMWARE — ubicación en breakout LARVIN requiere identificación visual en serigrafía.**

### 3.1 Pantalla ST7796 + Touch XPT2046

| Función | GPIO ESP32-S3 | Archivo/macro | Cable va a | Tensión | Estado | Fuente |
|---------|--------------|---------------|-----------|---------|--------|--------|
| TFT_MOSI | GPIO13 | `User_Setup.h:71` | ST7796 SDA/MOSI | 3.3V | ✅ | `User_Setup.h:71` |
| TFT_MISO | GPIO12 | `User_Setup.h:72` | ST7796 SDO/MISO | 3.3V | ✅ | `User_Setup.h:72` |
| TFT_SCLK | GPIO14 | `User_Setup.h:73` | ST7796 SCK | 3.3V | ✅ | `User_Setup.h:73` |
| TFT_CS | GPIO10 | `User_Setup.h:74` | ST7796 CS | 3.3V | ✅ | `User_Setup.h:74` |
| TFT_DC | GPIO39 | `User_Setup.h:75` | ST7796 DC/RS | 3.3V | ✅ | `User_Setup.h:75` |
| TFT_RST | GPIO38 | `User_Setup.h:76` | ST7796 RST | 3.3V | ✅ | `User_Setup.h:76` |
| TFT_BL | GPIO42 | `display_backlight.cpp:9` | Backlight PWM | 3.3V PWM 20kHz | ✅ | `display_backlight.h` |
| T_CS | GPIO21 | `User_Setup.h` `TOUCH_CS` | XPT2046 CS | 3.3V | ✅ | `User_Setup.h:88` |
| T_IRQ | — | — | XPT2046 IRQ | 3.3V | ⚪ No configurado en firmware | `User_Setup.h` |

> **Display:** ST7796 (480×320, landscape rotation 1, SPI HSPI 40MHz). NO ILI9488.  
> **SPI compartido** entre display y touch (mismo MOSI/MISO/SCLK, CS distintos).  
> **Alimentación pantalla:** 3.3V (NO 5V). GND = GND común.  
> **Brillo:** LEDC canal 7, 8-bit, persiste en `config_store::brightness`.

### 3.2 CAN Bus ESP32-S3 (TWAI)

| Función | GPIO ESP32-S3 | Archivo/macro | Cable va a | Tensión | Estado | Fuente |
|---------|--------------|---------------|-----------|---------|--------|--------|
| CAN_TX TWAI | GPIO4 | `CAN_TX_PIN = 4` | TJA1051T/3 TXD | 3.3V | ✅ | `esp32/src/main.cpp:103` |
| CAN_RX TWAI | GPIO5 | `CAN_RX_PIN = 5` | TJA1051T/3 RXD | 3.3V | ✅ | `esp32/src/main.cpp:104` |

**Bitrate:** 500 kbps. Timing: 80MHz/10/16 = 500 kbps, sample point 87.5%.  
**Transceptor ESP32:** TJA1051T/3 VCC=3.3V, VIO=3.3V, GND=GND común.  
**Terminación 120Ω** en este extremo del bus.

### 3.3 I2C ESP32-S3 (MCP23017 palanca + llave contacto)

| Función | GPIO ESP32-S3 | Archivo | Cable va a | Tensión | Estado | Fuente |
|---------|--------------|---------|-----------|---------|--------|--------|
| I2C SCL | GPIO8 | `esp32/src/main.cpp:731` | MCP23017 SCL | 3.3V OD | ✅ | `main.cpp:731` |
| I2C SDA | GPIO9 | `esp32/src/main.cpp:731` | MCP23017 SDA | 3.3V OD | ✅ | `main.cpp:731` |

**MCP23017:** I2C addr = 0x20 (A0=A1=A2=GND), VDD=3.3V, VSS=GND. Pull-ups 4.7kΩ en SCL/SDA.  
Palanca conecta físicamente a pines GPIO del MCP23017 (P/R/N/D — verificar `shifter_input.h` para mapping de pines interno).  
**La STM32 NO lee la palanca directamente** — recibe marcha por CAN 0x102 byte1.

### 3.4 Interruptor tracción 4x2/4x4

| Función | GPIO ESP32-S3 | Archivo | Lógica | Estado | Fuente |
|---------|--------------|---------|--------|--------|--------|
| Switch 4x2/4x4 | GPIO15 | `traction_switch.h:66` | LOW=4x4 (GND) / HIGH=4x2 (pull-up interno) | ✅ | `traction_switch.h:66` |

> NO va a STM32. ESP32 lo envía por CAN CMD_MODE (0x102).  
> Pull-up interno en GPIO15. Interruptor conecta GPIO15 a GND para 4x4.

### 3.5 Audio (DFPlayer Mini + Relé + PAM)

| Función | GPIO ESP32-S3 | Archivo/macro | Cable va a | Tensión | Estado | Fuente |
|---------|--------------|---------------|-----------|---------|--------|--------|
| DFPlayer TX (ESP→DF) | GPIO43 | `main.cpp:728` UART2 | DFPlayer RX | 3.3V | ✅ | `main.cpp:728` |
| DFPlayer RX (DF→ESP) | GPIO44 | `main.cpp:728` UART2 | DFPlayer TX | 3.3V | ✅ | `main.cpp:728` |
| Relé audio | GPIO11 | `relay_audio.h:PIN_AUDIO_RELAY=11` | ULN2803A canal 3B | 3.3V → ULN | ✅ | `relay_audio.h` |

**ULN2803A canal 3:** entrada 3B = GPIO11 ESP32, salida 3C → Songle IN3 → relé audio 5V.  
Relé audio en OFF = radio. Relé audio en ON = DFPlayer → altavoz.  
DFPlayer VCC = 5V (desde buck). GND = GND común.  
PAM/amplificador: alimentación según módulo (5V típico). GND = GND común.

### 3.6 Sensor obstáculos (TFMini Plus)

| Función | GPIO ESP32-S3 | Archivo | Protocolo | Estado | Fuente |
|---------|--------------|---------|-----------|--------|--------|
| TFMini RX | GPIO18 | `main.cpp:706` UART1 | UART 3.3V TTL | ✅ | `main.cpp:706-707` |
| TFMini TX | — | — | No necesario (solo lectura) | ⚪ | — |

**Conexión directa GPIO18 ← TFMini TX** (3.3V TTL, sin adaptador).  
Envía distancia por CAN 0x208 al STM32. Obligatorio para modo emergencia.

### 3.7 Mando RC (FlySky FS-iA6B iBUS)

| Función | GPIO ESP32-S3 | Archivo/macro | Protocolo | Estado | Fuente |
|---------|--------------|---------------|-----------|--------|--------|
| RC iBUS RX | GPIO16 | `remote_control.h:rxPin=16` UART0 | iBUS 115200 8N1 | ✅ | `remote_control.h:66` |

Receptor FS-iA6B: cable señal iBUS → GPIO16. Solo RX necesario (TX del ESP32 no se usa).  
Alimentación receptor: 5V (desde buck). GND = GND común.

### 3.8 WS2812B datos (LED strips)

| Función | GPIO ESP32-S3 | Archivo/macro | Cable va a | Tensión | Estado | Fuente |
|---------|--------------|---------------|-----------|---------|--------|--------|
| LED frontal DATA | GPIO47 | `led_controller.h:LED_FRONT_PIN=47` | Tira frontal 70 LEDs DATA | 3.3V data | ✅ | `led_controller.h:33` |
| LED trasero DATA | GPIO48 | `led_controller.h:LED_REAR_PIN=48` | Tira trasera 72 LEDs DATA | 3.3V data | ✅ | `led_controller.h:34` |

> **Alimentación 5V** de las tiras la cortan los relés PB10 (frontal) y PB11 (trasero) del STM32.  
> El ESP32 solo controla los **datos**; no alimenta la tira directamente.  
> Resistencia 300–500Ω en serie en el cable DATA antes del primer LED.

### 3.9 Llave de contacto / encendido

| Función | GPIO ESP32-S3 | Archivo/macro | Tipo | Estado | Fuente |
|---------|--------------|---------------|------|--------|--------|
| Ignition sense | GPIO40 | `power_manager.h:PIN_IGNITION_SENSE=40` | Entrada, activo LOW (opto NPN) | ✅ | `power_manager.h:45` |
| Power hold | GPIO41 | `power_manager.h:PIN_POWER_HOLD=41` | Salida HIGH = mantener encendido | ✅ | `power_manager.h:46` |

GPIO40: conectado a colector de opto NPN. 10kΩ pull-up a 3.3V recomendado.  
GPIO41: HIGH = ordena al circuito de alimentación que mantenga energía.

---

## 4. ALIMENTACIÓN GENERAL

### STM32

```
BUCK +5V ──[Polyfuse 2A]──[SS14 Schottky>]──► E5V (CN7_6 breakout)
                                               │
                                      [100µF/35V] ║ [100nF cerámico]
                                               │
BUCK GND ──────────────────────────────────► GND (CN7_8)  también CN10_9, CN10_32

JP5 Nucleo: posición E5V (pines 2-3) — OBLIGATORIO

Salida 3.3V Nucleo (CN7_5/CN7_16): → TCA9548A, INA226, BTS7960 VCC lógico (máx 300mA total)
Salida +5V Nucleo (CN7_18): → TJA1051, optos (solo con USB/VIN activo — verificar disponibilidad)
```

> **NOTA:** La salida +5V de la Nucleo (CN7_18) solo está activa cuando hay USB o VIN conectado. Con alimentación exclusiva por E5V, esta salida puede no estar disponible. Verificar con multímetro. Si no aparece, alimentar encoder, optos y TJA1051 desde el propio buck 5V directamente.

### ESP32-S3

```
BUCK +5V ──[SS14 Schottky>]──► VIN ESP32-S3 DevKitC-1
                                │
                       [100µF/35V] ║ [100nF cerámico]
                                │
BUCK GND ────────────────────► GND ESP32-S3

3.3V interno ESP32-S3 → MCP23017, touch XPT2046, TJA1051 ESP32
5V externo → DFPlayer, amplificador PAM, receptor RC FS-iA6B
```

---

## 5. RELÉS Y ULN2803A

```
STM32 CN7_2  (PC11) ──► ULN2803A 4B ──► 4C ──► Songle/relé IN → RELAY_TRAC (24V trac)
STM32 CN7_3  (PC12) ──► ULN2803A 5B ──► 5C ──► relé 12V IN   → RELAY_STEER (12V dir)
STM32 CN10_25(PB10) ──► ULN2803A 1B ──► 1C ──► Songle 4CH IN1 → RELAY_LED frontal
STM32 CN10_18(PB11) ──► ULN2803A 2B ──► 2C ──► Songle 4CH IN2 → RELAY_LED trasero
ESP32 GPIO11        ──► ULN2803A 3B ──► 3C ──► Songle 4CH IN3 → RELAY_AUDIO

ULN2803A GND (pin 9) → GND común
ULN2803A COM (pin 10) → SIN CONECTAR (módulos Songle tienen pull-up interno a +5V)

Módulo Songle 4CH (5V, IN activo LOW via ULN):
  VCC  → +5V
  GND  → GND común
  IN1–4 reciben señal del colector ULN

Lógica:
  STM32 GPIO HIGH (3.3V) → ULN activo → colector hunde a GND → IN relé = ~0V → RELÉ ON
  STM32 GPIO LOW  (0V)   → ULN inactivo → IN relé = ~5V pull-up → RELÉ OFF
```

**Prueba con multímetro (antes de conectar STM32):**
1. Alimentar solo ULN y módulo relé (5V y GND).
2. Medir INx del módulo: debe haber ~5V (pull-up, relé OFF).
3. NO debe haber continuidad desde INx hacia el rail de 12V/24V.

---

## 6. RELÉS Y CONTACTORES DE POTENCIA

```
RELAY TRACCIÓN (24V):
  Bobina (85/86 o bornes pequeños): 12V
    → Módulo relé 12V canal 1 COM/NO
    → Diodo flyback en paralelo con bobina:
       banda (cátodo) → borne positivo bobina (rail 12V)
       sin banda (ánodo) → borne negativo bobina (GND)
    → Verificar si módulo ya trae diodo (medir en modo diodo con relé desconectado)
    → Fusible bobina: REQUIERE VERIFICACIÓN (típico 1A slow-blow)
  Contactos de potencia (30/87 o bornes grandes):
    COM → batería 24V positivo (tras fusible 50A)
    NO  → rail 24V a BTS7960 B+ (tracción) y INA226 CH0-3

RELAY DIRECCIÓN (12V):
  Bobina: 12V
    → Módulo relé 12V canal 2
    → Diodo flyback igual que tracción
    → Fusible bobina: REQUIERE VERIFICACIÓN
  Contactos de potencia:
    COM → batería 12V positivo (tras fusible 20A)
    NO  → rail 12V a BTS7960 STEER B+ y INA226 CH5

RELAY LED FRONTAL (5V WS2812B):
  Contactos: COM → +5V, NO → VCC tira frontal 70 LEDs
  Condensador 1000µF/10V en paralelo con VCC de la tira (ante el primer LED)

RELAY LED TRASERO (5V WS2812B):
  Contactos: COM → +5V, NO → VCC tira trasera 72 LEDs
  Condensador 1000µF/10V en paralelo con VCC de la tira

⚠️ El diodo flyback va en la BOBINA (bornes pequeños 85/86), NO en los contactos de potencia.
⚠️ Si el módulo relé ya tiene diodo SMD integrado, NO añadir externo salvo misma polaridad.
```

---

## 7. MOTORES BTS7960

Para cada BTS7960 (IBT-2):

```
BTS7960 VCC lógico  → 3.3V (CN7_16 breakout Nucleo)     ← CRÍTICO: NO 5V
BTS7960 GND lógico  → GND común
BTS7960 RPWM        → señal STM32 3.3V 20kHz
BTS7960 LPWM        → señal STM32 3.3V 20kHz
BTS7960 R_EN ──┐
BTS7960 L_EN ──┘ → señal EN STM32 3.3V (unir físicamente R_EN y L_EN)
BTS7960 B+          → rail 24V (tracción) o 12V (dirección), post-relé, post-INA226
BTS7960 B-          → GND potencia (punto estrella)
BTS7960 M+ / M-     → terminales del motor

Condensador 470µF/35V entre B+ y GND (en cada BTS7960, lo más cerca del chip)
Condensador 100nF/50V en paralelo con el electrolítico (mismo punto)
```

---

## 8. SENSORES STM32

### Pedal Hall
```
Sensor Hall VCC → +5V
Sensor Hall GND → GND común
Señal Hall ──[10kΩ]──┬──► PB1 (CN10_24)
                      │
                    [6.8kΩ]
                      │
                     GND
Tensión PB1: 5V × 6.8/16.8 = 2.02V ≤ 3.3V ✅
Calibración: SERVICE_CMD 0xF5 (menú ingeniería ESP32) → flash página 124
```

### Encoder E6B2-CWZ6C (A/B/Z → 6N137)
```
Encoder VCC → +5V
Encoder GND → GND común
Por cada señal A, B, Z:
  [220Ω] ── ánodo 6N137 ← señal encoder 5V
  cátodo 6N137 → GND
  salida 6N137 (pin 6) ──[1kΩ pull-up a 3.3V]──► GPIO STM32
  VCC 6N137 (pin 8) → 3.3V (salida-lado)
  GND 6N137 (pin 5) → GND (salida-lado)
  Enable 6N137 (pin 7) → GND (siempre habilitado)
```

### Sensor centro dirección PB5 (LJ12A3, NPN, 6–36V)
```
LJ12A3 VCC → 12V o 24V (fuente separada, NO al STM32)
LJ12A3 GND → GND común
LJ12A3 señal (NPN activo LOW) → PC817 ánodo LED con [470Ω]
PC817 cátodo LED → GND
PC817 colector → PB5 (CN10_29) + [10kΩ pull-up a 3.3V]
PC817 emisor → GND STM32

Lógica PB5: PULLUP interno. HIGH = sin metal. LOW = metal detectado (centrado).
PB5 es PRIMARIO para centrado. PB4 (Z encoder) es SECUNDARIO.
```

### Sensores rueda FL/FR/RL/RR (mismo esquema PC817)
```
Mismo circuito que centro dirección.
6 pernos/imanes por vuelta (WHEEL_PULSES_REV=6).
EXTI en flanco ascendente (PULLUP).
```

### DS18B20 OneWire
```
PB0 (CN7_34) ───────── DAT bus común (todos los DS18B20)
3.3V ──[4.7kΩ ÚNICO]── mismo bus DAT
3.3V ─────────────────── VCC todos los DS18B20 (NO parasite power)
GND ──────────────────── GND todos los DS18B20

SOLO UNA resistencia 4.7kΩ en todo el bus.
Si tus módulos adaptadores llevan 472 (=4.7kΩ): retira todos salvo uno,
  o quita todos los integrados y pon un único pull-up externo.
```

---

## 9. CAN BUS STM32 ↔ ESP32-S3

```
STM32 (FDCAN1):        TJA1051 STM32:    BUS CAN:    TJA1051 ESP32:    ESP32 (TWAI):
  PA12 TX (CN10_12) → TXD               CAN_H ←→── TXD             ← GPIO4 TX
  PA11 RX (CN10_14) ← RXD               CAN_L ←→── RXD             → GPIO5 RX
  3.3V (CN7_16) → VCC/VIO                              3.3V → VCC/VIO
  GND → GND                              GND───────── GND
                       120Ω entre H/L                  120Ω entre H/L
```

**Bitrate:** 500 kbps. STM32: 170MHz/10/34. ESP32: 80MHz/10/16.  
**GND común obligatorio** entre STM32 y ESP32 en el bus CAN.

| ID CAN | Dirección | Descripción |
|--------|-----------|-------------|
| 0x001 | STM32→ESP32 | Heartbeat estado sistema |
| 0x102 | ESP32→STM32 | CMD_MODE (marcha, modo tracción) |
| 0x103 | STM32→ESP32 | ACK comandos |
| 0x204 | STM32→ESP32 | Ángulo dirección |
| 0x208 | ESP32→STM32 | Distancia obstáculo (TFMini) |
| 0x309 | STM32→ESP32 | Diagnóstico I2C/INA226 |

---

## 10. COMPONENTES EXCLUSIVOS ESP32-S3

> ❌ Los siguientes NO deben cablearse a la breakout STM32:

| Componente | Va a ESP32 | NO al STM32 |
|-----------|-----------|------------|
| Pantalla ST7796 | GPIO10/12/13/14/38/39 | ❌ |
| Touch XPT2046 | GPIO21 | ❌ |
| Backlight TFT | GPIO42 | ❌ |
| MCP23017 palanca | GPIO8/9 (I2C) | ❌ |
| Switch 4x2/4x4 | GPIO15 | ❌ |
| DFPlayer Mini | GPIO43/44 (UART2) | ❌ |
| Relé audio | GPIO11 → ULN | ❌ |
| TFMini obstáculos | GPIO18 (UART1) | ❌ |
| Mando RC FS-iA6B | GPIO16 (UART0) | ❌ |
| WS2812B datos | GPIO47 (frontal) / GPIO48 (trasero) | ❌ |
| Ignition key | GPIO40 (sense) / GPIO41 (hold) | ❌ |

---

## 11. LISTA NO CONECTAR NUNCA

1. 🔴 **NO** meter 12V/24V en cualquier GPIO STM32 (Vmax = 3.6V → destrucción inmediata)
2. 🔴 **NO** conectar IN del módulo relé 12V directamente a STM32 — siempre ULN de por medio
3. 🔴 **NO** alimentar por VIN (CN7_24) con buck 5V — VIN espera 7–12V
4. 🔴 **NO** conectar nada a AVDD (CN10_7) ni a U5V (CN10_8)
5. 🔴 **NO** usar PA11 para BTS7960 — es CAN_RX; usarlo como PWM destruye el CAN
6. 🔴 **NO** usar PC13 (CN7_23) como salida — es botón USER B1 de la Nucleo
7. 🔴 **NO** poner VCC lógico BTS7960 a 5V — usar 3.3V
8. 🔴 **NO** conectar señales encoder 5V directas a GPIO STM32 sin 6N137
9. 🔴 **NO** conectar LJ12A3 (6–36V) directamente a GPIO STM32 sin opto
10. 🔴 **NO** conectar pedal Hall directamente al ADC sin divisor resistivo
11. 🔴 **NO** poner electrolítico al revés (flange/banda a positivo = exploración)
12. 🔴 **NO** poner diodo flyback al revés en bobina del relé
13. 🔴 **NO** tener múltiples pull-ups 4.7kΩ en el bus OneWire (solo uno)
14. 🔴 **NO** alimentar E5V y VIN simultáneamente
15. 🔴 **NO** alimentar ESP32-S3 directamente por 3.3V salvo regulador externo exacto
16. 🔴 **NO** conectar pantalla ST7796 a 5V — lógica 3.3V
17. 🔴 **NO** usar GPIO26–32 del ESP32-S3 para nada externo — bus QSPI Flash interno
18. 🔴 **NO** compartir retorno de corriente de motores con el bus I2C fuera del punto estrella

---

## 12. CHECKLIST FINAL ANTES DE ENCENDER

### Paso 1 — Mediciones previas (sin corriente en el sistema)
- [ ] Medir salida buck 5V: debe ser +5.0V ±0.1V entre + y GND
- [ ] Medir VIN ESP32-S3 − GND: debe ser +5.0V cuando buck activo
- [ ] Verificar JP5 Nucleo en posición **E5V**
- [ ] Medir continuidad E5V(CN7_6) ↔ GND(CN7_8): debe ser >1MΩ (sin corto)
- [ ] Medir continuidad VIN ESP32 ↔ GND ESP32: debe ser >1MΩ
- [ ] Verificar polaridad de TODOS los condensadores electrolíticos
- [ ] Medir continuidad GND buck ↔ CN7_8 ↔ CN10_9 ↔ CN10_32: debe ser ~0Ω

### Paso 2 — Verificar adaptaciones antes de conectar breakout
- [ ] Verificar salida optos 6N137 (encoder): ≤3.3V con señal encoder activa
- [ ] Verificar salida optos PC817 (ruedas/centro): ≤3.3V con sensor activo
- [ ] Verificar divisor pedal: tensión en PB1 con pedal a fondo ≤3.3V
- [ ] Verificar que IN1..IN5 del módulo relé (sin STM32) muestran ~5V (pull-up activo)
- [ ] Verificar que NO hay continuidad desde IN módulo relé hacia rail 12V/24V

### Paso 3 — Verificar ULN y relés
- [ ] Alimentar ULN+módulo relé. Medir INx = ~5V, relé OFF
- [ ] Conectar STM32. Verificar relés responden al firmware
- [ ] Medir diodos flyback (bobinas): modo diodo, banda → positivo bobina
- [ ] Verificar que módulo relé no trae diodo al revés (medir antes de montar)

### Paso 4 — Primer encendido (lógica solamente)
- [ ] Sin BTS7960 conectados a potencia (B+ desconectado de batería)
- [ ] Sin contactores de potencia (24V/12V desconectados)
- [ ] Encender buck → LED PWR de la D-1686 debe encenderse (rojo)
- [ ] Verificar +3.3V en CN7_5 (VDD): señal de que regulador Nucleo activo
- [ ] Verificar CAN funciona: STM32 y ESP32 deben aparecer en heartbeat
- [ ] Verificar I2C: pantalla ESP32 debe mostrar datos de INA226 (o error I2C si no hay INA)

### Paso 5 — Sensores
- [ ] Conectar INA226 + TCA9548A al I2C. Verificar no hay Error Code 11
- [ ] Conectar DS18B20. Verificar lecturas de temperatura en HMI
- [ ] Conectar pedal. Calibrar desde menú (SERVICE_CMD 0xF5)
- [ ] Conectar encoder. Verificar ángulo en HMI
- [ ] Conectar sensores rueda. Verificar velocidad en HMI

### Paso 6 — Relés y motores
- [ ] Conectar relés de potencia (bobinas 12V). Verificar activación por firmware
- [ ] Conectar BTS7960 a potencia. Verificar enable antes de dar PWM
- [ ] Prueba de motores a <10% potencia en zona segura

### Paso 7 — Periféricos ESP32
- [ ] Pantalla ST7796: imagen correcta, sin artefactos
- [ ] Touch: responde al tacto, calibración guardada
- [ ] DFPlayer: reproduce sonido de arranque
- [ ] Mando RC: canales responden en HMI
- [ ] WS2812B: LEDs encienden con colores correctos

---

## 13. ERRORES / CONFLICTOS DETECTADOS

| Punto | Estado anterior | Estado real | Corrección | Fuente | Impacto si mal cableado |
|-------|----------------|-------------|-----------|--------|------------------------|
| LPWM_FR | Documentación antigua: PA11 | **PC3 (CN7_37)** — reasignado | Cablear LPWM_FR a CN7_37, NO a CN10_14 | `project_config.h:99,66` | PA11 es CAN_RX; conectar motor aquí destruye el CAN |
| EN_RR | Documentación antigua: PC13 | **PC2 (CN7_35)** — reasignado | Cablear EN_RR a CN7_35, NO a CN7_23 | `project_config.h:141,132` | PC13 es botón USER; activar EN_RR pulsa el botón |
| Display | Algunas referencias antiguas: ILI9488 | **ST7796** | Driver correcto: `#define ST7796_DRIVER` | `User_Setup.h:38` | Driver incorrecto = pantalla en blanco o artefactos |
| +5V Nucleo (CN7_18) | Asumida disponible | Solo activa con USB/VIN | Verificar con multímetro con E5V; usar buck directo si no aparece | `STM32_BREAKOUT_BOARD_WIRING.md §6.1` | Encoder y optos sin alimentación |
| VCC lógico BTS7960 | Algunos manuales dicen 5V | **3.3V obligatorio** | Cablear VCC lógico BTS7960 a CN7_16 (+3V3) | `project_config.h:70-74`, `HARDWARE_SPEC.md §§BTS7960` | Motor errático o no responde a señales 3.3V del STM32 |
| WS2812B datos | Asumido desde STM32 | **Desde ESP32 GPIO47/48** | Cablear datos WS2812B a ESP32, NO a STM32 | `led_controller.h:33-34` | STM32 no tiene controlador NeoPixel |

---

## PUNTOS QUE REQUIEREN MEDICIÓN CON MULTÍMETRO

1. **+5V en CN7_18 (Nucleo):** medir con multímetro con E5V conectado y USB desconectado.
2. **JP5 posición correcta:** medir que E5V llega al STM32 (CN7_6) y no hay conflicto con USB.
3. **Diodos flyback de relés:** verificar en modo diodo que el módulo relé tiene diodo interno y en qué polaridad.
4. **Salida optos:** con sensor activo, medir tensión en pin GPIO STM32 antes de conectar.
5. **Divisor pedal:** con pedal a fondo, medir que tensión en CN10_24 ≤ 3.3V.
6. **GND común:** continuidad entre todos los GND del sistema (buck, STM32, ESP32, relés, BTS7960).
7. **B+ BTS7960 con relé abierto:** verificar que INA226 CH0-3 no miden tensión con RELAY_TRAC OFF.
8. **IN módulos relé:** sin STM32 conectado, verificar que no hay tensión de 12V/24V en INx.
