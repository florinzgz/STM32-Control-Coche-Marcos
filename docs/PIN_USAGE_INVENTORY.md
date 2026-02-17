# INVENTARIO COMPLETO DE PINES — STM32G474RE y ESP32-S3

> **Fecha:** 2026-02-17  
> **Propósito:** Documentar el uso exacto de pines, cuántos se usan por módulo, y cuántos quedan libres  
> **Fuentes:** `Core/Inc/main.h`, `esp32/platformio.ini`, `esp32/src/main.cpp`, `docs/CONEXIONES_COMPLETAS.md`

---

## ÍNDICE

1. [STM32G474RE — Resumen](#1-stm32g474re--resumen)
2. [STM32G474RE — Detalle por módulo](#2-stm32g474re--detalle-por-módulo)
3. [STM32G474RE — Pines libres](#3-stm32g474re--pines-libres)
4. [ESP32-S3 — Resumen](#4-esp32-s3--resumen)
5. [ESP32-S3 — Detalle por módulo](#5-esp32-s3--detalle-por-módulo)
6. [ESP32-S3 — Pines libres](#6-esp32-s3--pines-libres)
7. [Conexión del pedal con sensor Hall y ADS1115 — Detalle completo](#7-conexión-del-pedal-con-sensor-hall-y-ads1115--detalle-completo)

---

## 1. STM32G474RE — Resumen

| Dato | Valor |
|------|-------|
| **MCU** | STM32G474RE |
| **Package** | LQFP64 (64 pines físicos) |
| **Pines totales del chip** | 64 |
| **Pines de alimentación/tierra/reset/cristal** | 13 (VBAT, VDDA, VSSA, 3×VDD, 4×VSS, NRST, PH0, PH1) |
| **Pines GPIO disponibles** | 47 (PA0–PA15, PB0–PB15, PC0–PC13, PD2) |
| **Pines usados por el proyecto** | 32 |
| **Pines reservados (SWD debug)** | 2 (PA13/SWDIO, PA14/SWCLK) |
| **Pines libres para expansión** | **13** |

### Distribución de pines usados por categoría

| Categoría | Pines usados | Porcentaje |
|-----------|-------------|------------|
| PWM motores (TIM1 + TIM8) | 5 | 10.6% |
| Dirección motores (GPIO out) | 5 | 10.6% |
| Habilitación motores (GPIO out) | 5 | 10.6% |
| Relés (GPIO out) | 3 | 6.4% |
| Sensores velocidad rueda (EXTI) | 4 | 8.5% |
| Encoder dirección (TIM2 + EXTI) | 3 | 6.4% |
| Sensor centrado dirección (EXTI) | 1 | 2.1% |
| Pedal acelerador (ADC) | 1 | 2.1% |
| Bus I2C (INA226/TCA9548A/ADS1115) | 2 | 4.3% |
| Bus OneWire (DS18B20) | 1 | 2.1% |
| Bus CAN (FDCAN1) | 2 | 4.3% |
| **TOTAL USADOS** | **32** | **68.1%** |
| Reservados SWD | 2 | 4.3% |
| **LIBRES** | **13** | **27.7%** |

---

## 2. STM32G474RE — Detalle por módulo

### 2.1 Motores de tracción — 4× BTS7960 (12 pines)

Cada motor de tracción necesita 3 señales: PWM (velocidad), DIR (dirección de giro) y EN (habilitación).

| Motor | Pin PWM | Timer | Pin DIR | Pin EN | Total pines |
|-------|---------|-------|---------|--------|-------------|
| FL (Delantero Izq.) | **PA8** | TIM1_CH1 | **PC0** | **PC5** | 3 |
| FR (Delantero Der.) | **PA9** | TIM1_CH2 | **PC1** | **PC6** | 3 |
| RL (Trasero Izq.) | **PA10** | TIM1_CH3 | **PC2** | **PC7** | 3 |
| RR (Trasero Der.) | **PA11** | TIM1_CH4 | **PC3** | **PC13** | 3 |
| | | | | **Subtotal:** | **12 pines** |

- PWM a 20 kHz, resolución 13 bits (ARR = 8499)
- DIR: HIGH = adelante, LOW = atrás
- EN: HIGH = motor habilitado

### 2.2 Motor de dirección — 1× BTS7960 (3 pines)

| Motor | Pin PWM | Timer | Pin DIR | Pin EN | Total pines |
|-------|---------|-------|---------|--------|-------------|
| STEER (Dirección) | **PC8** | TIM8_CH3 | **PC4** | **PC9** | 3 |
| | | | | **Subtotal:** | **3 pines** |

### 2.3 Relés de potencia (3 pines)

| Relé | Pin | Función |
|------|-----|---------|
| RELAY_MAIN | **PC10** | Alimentación general |
| RELAY_TRAC | **PC11** | Alimentación motores 24V |
| RELAY_DIR | **PC12** | Alimentación motor dirección 12V |
| | **Subtotal:** | **3 pines** |

- Controlados vía optoacopladores (HY-M158)
- Activo HIGH (3.3V del STM32 activa el optoacoplador)

### 2.4 Sensores de velocidad de rueda — 4× inductivos LJ12A3 (4 pines)

| Sensor | Pin | Interrupción | Función |
|--------|-----|-------------|---------|
| Rueda FL | **PA0** | EXTI0 | 6 pulsos/vuelta |
| Rueda FR | **PA1** | EXTI1 | 6 pulsos/vuelta |
| Rueda RL | **PA2** | EXTI2 | 6 pulsos/vuelta |
| Rueda RR | **PB15** | EXTI15 | 6 pulsos/vuelta |
| | **Subtotal:** | | **4 pines** |

- Tipo: NPN NO (open-collector)
- Pull-up interno a 3.3V activado
- Detección por flanco de subida

### 2.5 Encoder de dirección — E6B2-CWZ6C (3 pines)

| Señal | Pin | Periférico | Función |
|-------|-----|-----------|---------|
| Canal A | **PA15** | TIM2_CH1 | Cuadratura (requiere adaptador 5V→3.3V) |
| Canal B | **PB3** | TIM2_CH2 | Cuadratura (requiere adaptador 5V→3.3V) |
| Índice Z | **PB4** | EXTI4 | 1 pulso por vuelta |
| | **Subtotal:** | | **3 pines** |

- Resolución: 1200 PPR × 4 (cuadratura) = 4800 cuentas/vuelta = 0.075°/cuenta

### 2.6 Sensor de centrado de dirección — LJ12A3 inductivo (1 pin)

| Señal | Pin | Interrupción | Función |
|-------|-----|-------------|---------|
| Centro dirección | **PB5** | EXTI5 | Detecta tornillo mecánico en el centro |
| | **Subtotal:** | | **1 pin** |

### 2.7 Pedal acelerador — Canal primario ADC (1 pin)

| Señal | Pin | Periférico | Función |
|-------|-----|-----------|---------|
| Posición pedal | **PA3** | ADC1_IN4 | Señal dividida 0.12V–1.94V (vía divisor 10kΩ/6.8kΩ) |
| | **Subtotal:** | | **1 pin** |

- Sensor Hall SS1324LUA-T (5V, salida 0.3V–4.8V)
- Divisor de tensión: R1=10kΩ, R2=6.8kΩ → ratio 0.4048
- El canal de plausibilidad (ADS1115) usa el bus I2C, no pin adicional

### 2.8 Bus I2C — TCA9548A + 6× INA226 + ADS1115 (2 pines)

| Señal | Pin | Periférico | Función |
|-------|-----|-----------|---------|
| SCL | **PB6** | I2C1_SCL | Reloj I2C 400 kHz |
| SDA | **PB7** | I2C1_SDA | Datos I2C 400 kHz |
| | **Subtotal:** | | **2 pines** |

Dispositivos en el bus I2C (solo 2 pines para todos):

| Dispositivo | Dirección I2C | Función |
|-------------|---------------|---------|
| TCA9548A | 0x70 | Multiplexor 8 canales (para 6× INA226) |
| INA226 ×6 | 0x40 (detrás del mux) | Sensores de corriente (4 motores + batería + dirección) |
| ADS1115 | 0x48 | ADC 16-bit pedal plausibilidad (directo, sin mux) |

> **Nota:** Los 6 sensores INA226 y el multiplexor TCA9548A comparten los mismos 2 pines I2C. El ADS1115 también está en el mismo bus pero con dirección diferente (0x48). Esto permite 9 dispositivos con solo 2 pines GPIO.

### 2.9 Bus OneWire — 5× DS18B20 (1 pin)

| Señal | Pin | Periférico | Función |
|-------|-----|-----------|---------|
| OneWire bus | **PB0** | GPIO (bit-bang) | 5 sensores de temperatura en un solo cable |
| | **Subtotal:** | | **1 pin** |

Sensores en el bus (un solo pin):

| Sensor | Ubicación |
|--------|-----------|
| DS18B20 #0 | Motor FL |
| DS18B20 #1 | Motor FR |
| DS18B20 #2 | Motor RL |
| DS18B20 #3 | Motor RR |
| DS18B20 #4 | Motor Dirección / Ambiente |

### 2.10 Bus CAN — FDCAN1 (2 pines)

| Señal | Pin | Periférico | Función |
|-------|-----|-----------|---------|
| CAN RX | **PB8** | FDCAN1_RX (AF9) | Recepción CAN a 500 kbps |
| CAN TX | **PB9** | FDCAN1_TX (AF9) | Transmisión CAN a 500 kbps |
| | **Subtotal:** | | **2 pines** |

- Transceiver externo TJA1051T/3 requerido
- Terminación 120Ω en ambos extremos del bus

### 2.11 Debug SWD — Reservados (2 pines)

| Señal | Pin | Función |
|-------|-----|---------|
| SWDIO | **PA13** | Datos debug ST-Link |
| SWCLK | **PA14** | Reloj debug ST-Link |
| | **Subtotal:** | **2 pines** (reservados, no disponibles) |

### Tabla resumen STM32 — Todos los pines usados

| # | Pin | Puerto | Módulo | Función |
|---|-----|--------|--------|---------|
| 1 | PA0 | GPIOA | Sensor rueda FL | EXTI0, velocidad rueda |
| 2 | PA1 | GPIOA | Sensor rueda FR | EXTI1, velocidad rueda |
| 3 | PA2 | GPIOA | Sensor rueda RL | EXTI2, velocidad rueda |
| 4 | PA3 | GPIOA | Pedal acelerador | ADC1_IN4, canal primario |
| 5 | PA8 | GPIOA | Motor FL | TIM1_CH1, PWM 20 kHz |
| 6 | PA9 | GPIOA | Motor FR | TIM1_CH2, PWM 20 kHz |
| 7 | PA10 | GPIOA | Motor RL | TIM1_CH3, PWM 20 kHz |
| 8 | PA11 | GPIOA | Motor RR | TIM1_CH4, PWM 20 kHz |
| 9 | PA15 | GPIOA | Encoder dirección | TIM2_CH1, canal A cuadratura |
| 10 | PB0 | GPIOB | Temperatura (DS18B20) | OneWire bus, 5 sensores |
| 11 | PB3 | GPIOB | Encoder dirección | TIM2_CH2, canal B cuadratura |
| 12 | PB4 | GPIOB | Encoder dirección | EXTI4, índice Z |
| 13 | PB5 | GPIOB | Centrado dirección | EXTI5, sensor inductivo |
| 14 | PB6 | GPIOB | I2C (corriente/pedal) | I2C1_SCL, 400 kHz |
| 15 | PB7 | GPIOB | I2C (corriente/pedal) | I2C1_SDA, 400 kHz |
| 16 | PB8 | GPIOB | CAN bus | FDCAN1_RX, 500 kbps |
| 17 | PB9 | GPIOB | CAN bus | FDCAN1_TX, 500 kbps |
| 18 | PB15 | GPIOB | Sensor rueda RR | EXTI15, velocidad rueda |
| 19 | PC0 | GPIOC | Motor FL | DIR (dirección de giro) |
| 20 | PC1 | GPIOC | Motor FR | DIR (dirección de giro) |
| 21 | PC2 | GPIOC | Motor RL | DIR (dirección de giro) |
| 22 | PC3 | GPIOC | Motor RR | DIR (dirección de giro) |
| 23 | PC4 | GPIOC | Motor dirección | DIR (dirección de giro) |
| 24 | PC5 | GPIOC | Motor FL | EN (habilitación) |
| 25 | PC6 | GPIOC | Motor FR | EN (habilitación) |
| 26 | PC7 | GPIOC | Motor RL | EN (habilitación) |
| 27 | PC8 | GPIOC | Motor dirección | TIM8_CH3, PWM 20 kHz |
| 28 | PC9 | GPIOC | Motor dirección | EN (habilitación) |
| 29 | PC10 | GPIOC | Relé MAIN | Alimentación general |
| 30 | PC11 | GPIOC | Relé TRAC | Alimentación motores |
| 31 | PC12 | GPIOC | Relé DIR | Alimentación dirección |
| 32 | PC13 | GPIOC | Motor RR | EN (habilitación) |

---

## 3. STM32G474RE — Pines libres

Los siguientes pines GPIO del STM32G474RE **NO están usados** y están disponibles para expansión futura:

| # | Pin | Puerto | Funciones alternativas disponibles | Estado |
|---|-----|--------|--------------------------------------|--------|
| 1 | PA4 | GPIOA | DAC1_OUT1, SPI1_NSS, ADC2_IN17 | **LIBRE** |
| 2 | PA5 | GPIOA | DAC1_OUT2, SPI1_SCK, ADC2_IN13 | **LIBRE** |
| 3 | PA6 | GPIOA | SPI1_MISO, TIM3_CH1, ADC2_IN3 | **LIBRE** |
| 4 | PA7 | GPIOA | SPI1_MOSI, TIM3_CH2, ADC2_IN4 | **LIBRE** |
| 5 | PA12 | GPIOA | USB_DP, FDCAN1_TX (alt), TIM1_ETR | **LIBRE** |
| 6 | PB1 | GPIOB | ADC3_IN1, TIM3_CH4 | **LIBRE** |
| 7 | PB2 | GPIOB | GPIO general | **LIBRE** |
| 8 | PB10 | GPIOB | USART3_TX, I2C2_SCL, TIM2_CH3 | **LIBRE** |
| 9 | PB11 | GPIOB | USART3_RX, I2C2_SDA, TIM2_CH4 | **LIBRE** |
| 10 | PB12 | GPIOB | SPI2_NSS, I2S2_WS | **LIBRE** |
| 11 | PB13 | GPIOB | SPI2_SCK, I2S2_CK | **LIBRE** |
| 12 | PB14 | GPIOB | SPI2_MISO, TIM15_CH1 | **LIBRE** |
| 13 | PD2 | GPIOD | GPIO general, TIM3_ETR | **LIBRE** |

Además, los 2 pines SWD están reservados pero podrían reutilizarse si no se necesita debugging:

| # | Pin | Puerto | Función reservada | Nota |
|---|-----|--------|-------------------|------|
| 14 | PA13 | GPIOA | SWDIO (debug) | ⚠️ NO recomendado liberar |
| 15 | PA14 | GPIOA | SWCLK (debug) | ⚠️ NO recomendado liberar |

> **Resumen: 13 pines libres para uso inmediato** (15 si se sacrifica debug SWD, no recomendado).

### Posibles usos de los pines libres

| Posible expansión | Pines sugeridos | Nº pines |
|-------------------|-----------------|----------|
| UART debug serie | PB10 (TX) + PB11 (RX) | 2 |
| SPI adicional (sensor, SD card) | PA5 (SCK) + PA6 (MISO) + PA7 (MOSI) + PA4 (CS) | 4 |
| ADC adicional (sensor batería, otro sensor) | PB1, PA4, PA5 | 1-3 |
| Segundo bus I2C | PB10 (SCL) + PB11 (SDA) | 2 |
| LEDs de estado / buzzer | PB2, PB12, PB13, PB14 | 1-4 |
| Sensores de obstáculos (ultrasónicos) | PA6, PA7, PB12, PB13, PB14 | 2-5 |
| USB | PA12 (DP) | 1 |

---

## 4. ESP32-S3 — Resumen

| Dato | Valor |
|------|-------|
| **MCU** | ESP32-S3 (Xtensa LX7 dual-core @ 240 MHz) |
| **Placa** | ESP32-S3-DevKitC-1 |
| **Pines GPIO totales del chip** | 45 (GPIO0–GPIO21, GPIO26–GPIO48) |
| **Pines disponibles en DevKitC** | ~36 (algunos reservados por flash/PSRAM) |
| **Pines usados por el proyecto** | 14 |
| **Pines reservados (Flash/PSRAM interno)** | ~9 (GPIO26–GPIO32, GPIO33–GPIO37 en módulos con Octal PSRAM) |
| **Pines libres para expansión** | **~15** |

### Distribución de pines usados por categoría

| Categoría | Pines usados | Porcentaje |
|-----------|-------------|------------|
| Display TFT SPI (ST7796) | 6 | 16.7% |
| Touch panel SPI | 1 | 2.8% |
| CAN bus (TWAI vía TJA1051) | 2 | 5.6% |
| Sensor de obstáculos TOFSense (UART2) | 1 | 2.8% |
| DFPlayer Mini audio (UART1) | 2 | 5.6% |
| LEDs WS2812B (front + rear) | 2 | 5.6% |
| **TOTAL USADOS** | **14** | **38.9%** |
| Reservados Flash/PSRAM | ~9 | 25.0% |
| **LIBRES** | **~15** | **~41.7%** |

---

## 5. ESP32-S3 — Detalle por módulo

### 5.1 Display TFT — ST7796 480×320 vía SPI (6 pines)

| Señal | GPIO | Función | Velocidad |
|-------|------|---------|-----------|
| MOSI (SDA) | **GPIO13** | Datos SPI al display | 40 MHz |
| SCLK (SCL) | **GPIO14** | Reloj SPI | 40 MHz |
| CS | **GPIO15** | Chip Select display | — |
| DC (A0) | **GPIO16** | Data/Command select | — |
| RST | **GPIO17** | Reset hardware del display | — |
| BL | **GPIO42** | Backlight (HIGH = encendido) | — |
| | | **Subtotal:** | **6 pines** |

- Driver: ST7796, resolución 480×320 en modo landscape (rotación 1)
- Biblioteca: TFT_eSPI v2.5.43
- MISO: no conectado (−1), solo escritura

### 5.2 Panel táctil — SPI (1 pin)

| Señal | GPIO | Función | Velocidad |
|-------|------|---------|-----------|
| TOUCH_CS | **GPIO21** | Chip Select del panel táctil | 2.5 MHz |
| | | **Subtotal:** | **1 pin** |

- Comparte MOSI (GPIO13) y SCLK (GPIO14) con el display
- Pero solo TOUCH_CS cuenta como pin adicional

### 5.3 CAN bus — TWAI vía TJA1051 (2 pines)

| Señal | GPIO | Función | Velocidad |
|-------|------|---------|-----------|
| CAN TX | **GPIO4** | Transmisión CAN (TWAI_TX) | 500 kbps |
| CAN RX | **GPIO5** | Recepción CAN (TWAI_RX) | 500 kbps |
| | | **Subtotal:** | **2 pines** |

- Transceiver externo TJA1051T/3 obligatorio
- Biblioteca: ESP32-TWAI-CAN v1.0.1
- Terminación 120Ω en el lado ESP32

### 5.4 Sensor de obstáculos — TOFSense-M S LiDAR vía UART2 (1 pin)

| Señal | GPIO | Función | Velocidad |
|-------|------|---------|-----------|
| UART2 RX | **GPIO44** | Recepción datos TOFSense (solo RX) | 921600 baud |
| | | **Subtotal:** | **1 pin** |

- Sensor: TOFSense-M S LiDAR (sensor de distancia 0–5000 mm)
- Protocolo: UART unidireccional (solo sensor TX → ESP32 RX)
- TX del ESP32 no se usa (PIN_TOFSENSE_TX = −1)
- Tramas de 8 bytes cada 66 ms (15 Hz)
- Procesamiento: Parser UART + matriz 8×8 + lógica 5 zonas
- Datos enviados al STM32 vía CAN ID 0x208 (distancia, zona, salud, contador)
- Referencia: `docs/OBSTACLE_SYSTEM_ARCHITECTURE.md`

### 5.5 DFPlayer Mini — Audio MP3 vía UART1 (2 pines)

| Señal | GPIO | Función | Velocidad |
|-------|------|---------|-----------|
| UART1 TX | **GPIO19** | Comandos al DFPlayer (ESP32 → DFPlayer RX) | 9600 baud |
| UART1 RX | **GPIO20** | Respuestas del DFPlayer (DFPlayer TX → ESP32) | 9600 baud |
| | | **Subtotal:** | **2 pines** |

- Módulo: DFPlayer Mini (reproductor MP3 con DAC + amplificador 3W)
- Tarjeta SD: FAT32, 68 archivos MP3 (0001.mp3–0068.mp3)
- Altavoz: 3W 8Ω conectado a las salidas DAC_R/DAC_L del DFPlayer
- Biblioteca: DFRobotDFPlayerMini (inicialización lazy, no bloqueante)
- Estado: **Planificado Phase 5** — hardware no conectado todavía
- Referencia: `docs/PENDING_FEATURES_SCHEDULE.md` §3

> **Nota sobre GPIO 19/20:** En el ESP32-S3, estos son los pines nativos USB D−/D+. Con `ARDUINO_USB_CDC_ON_BOOT=1` (configurado en `platformio.ini`), el USB CDC serial usa el periférico USB nativo internamente, liberando GPIO 19 y GPIO 20 para uso general como UART1.

### 5.6 LEDs WS2812B — Iluminación front/rear (2 pines)

| Señal | GPIO | Función | Protocolo |
|-------|------|---------|-----------|
| LED Front Data | **GPIO47** | Tira frontal 28× WS2812B | FastLED / RMT 800 kHz |
| LED Rear Data | **GPIO43** | Tira trasera 16× WS2812B | FastLED / RMT 800 kHz |
| | | **Subtotal:** | **2 pines** |

- LEDs: WS2812B (RGB, orden GRB, 800 kHz)
- Frontal: 28 LEDs — efecto Knight Rider, colores reactivos al acelerador
- Trasero: 16 LEDs — luces de posición, freno, intermitentes
- Periférico: RMT del ESP32-S3 (hardware, sin bloquear CPU ni interrupciones)
- Tiempo de transferencia: 44 LEDs × 30 µs = 1.32 ms
- Biblioteca: FastLED (uso del periférico RMT, no bit-bang)
- Estado: **Planificado Phase 3** — hardware no conectado todavía
- Referencia: `docs/ESP32_HMI_ARCHITECTURE_REBUILD.md` §3, Addendum D.2

### Tabla resumen ESP32-S3 — Todos los pines usados

| # | GPIO | Módulo | Función |
|---|------|--------|---------|
| 1 | GPIO4 | CAN bus | TWAI TX → TJA1051 TXD |
| 2 | GPIO5 | CAN bus | TWAI RX → TJA1051 RXD |
| 3 | GPIO13 | Display TFT | SPI MOSI (datos al display) |
| 4 | GPIO14 | Display TFT | SPI SCLK (reloj) |
| 5 | GPIO15 | Display TFT | SPI CS (chip select display) |
| 6 | GPIO16 | Display TFT | DC (Data/Command) |
| 7 | GPIO17 | Display TFT | RST (Reset display) |
| 8 | GPIO19 | DFPlayer Mini | UART1 TX (comandos audio) — Phase 5 |
| 9 | GPIO20 | DFPlayer Mini | UART1 RX (respuestas audio) — Phase 5 |
| 10 | GPIO21 | Panel táctil | TOUCH_CS (chip select touch) |
| 11 | GPIO42 | Display TFT | BL (Backlight) |
| 12 | GPIO43 | LEDs WS2812B | Tira trasera 16 LEDs — Phase 3 |
| 13 | GPIO44 | Sensor obstáculos | TOFSense UART2 RX — Phase 3 |
| 14 | GPIO47 | LEDs WS2812B | Tira frontal 28 LEDs — Phase 3 |

---

## 6. ESP32-S3 — Pines libres

Los siguientes GPIO del ESP32-S3-DevKitC-1 **NO están usados** y están disponibles:

| # | GPIO | Funciones disponibles | Estado |
|---|------|----------------------|--------|
| 1 | GPIO0 | ADC1_CH0, GPIO, touch | **LIBRE** (⚠️ boot strapping) |
| 2 | GPIO1 | ADC1_CH1, GPIO | **LIBRE** |
| 3 | GPIO2 | ADC1_CH2, GPIO | **LIBRE** |
| 4 | GPIO3 | ADC1_CH3, GPIO | **LIBRE** (⚠️ boot strapping) |
| 5 | GPIO6 | GPIO, SPI | **LIBRE** |
| 6 | GPIO7 | GPIO, SPI | **LIBRE** |
| 7 | GPIO8 | GPIO, SPI | **LIBRE** |
| 8 | GPIO9 | GPIO | **LIBRE** |
| 9 | GPIO10 | GPIO, SPI | **LIBRE** |
| 10 | GPIO11 | GPIO, SPI | **LIBRE** |
| 11 | GPIO12 | GPIO, SPI | **LIBRE** |
| 12 | GPIO18 | GPIO | **LIBRE** |
| 13 | GPIO38 | GPIO | **LIBRE** |
| 14 | GPIO39 | GPIO | **LIBRE** |
| 15 | GPIO40 | GPIO | **LIBRE** |
| 16 | GPIO41 | GPIO | **LIBRE** |
| 17 | GPIO45 | GPIO | **LIBRE** (⚠️ boot strapping) |
| 18 | GPIO46 | GPIO | **LIBRE** (⚠️ boot strapping, solo input) |
| 19 | GPIO48 | GPIO | **LIBRE** |

> **Nota:** GPIO26–GPIO37 están reservados para Flash/PSRAM interno en el módulo ESP32-S3-WROOM y **NO están accesibles** en la placa DevKitC-1. GPIO19/GPIO20 están asignados al DFPlayer (UART1). GPIO43/GPIO44 están asignados a LEDs traseros y sensor de obstáculos. GPIO47 está asignado a LEDs frontales.

> **Resumen: ~15 pines libres de forma segura** (evitando pines de boot strapping).

### Posibles usos de los pines libres

| Posible expansión | GPIOs sugeridos | Nº pines |
|-------------------|-----------------|----------|
| Botones físicos (menú, modo, emergencia) | GPIO6, GPIO7, GPIO8 | 3 |
| LEDs de estado (CAN ok, error, etc.) | GPIO9, GPIO10 | 2 |
| SD Card (SPI) | GPIO6 (MISO), GPIO7 (MOSI), GPIO8 (SCK), GPIO9 (CS) | 4 |
| Buzzer / alarma sonora | GPIO38 | 1 |
| ADC adicional (batería, sensor luz) | GPIO1, GPIO2, GPIO3 | 1-3 |
| I2C adicional (sensor ambiental) | GPIO38 (SCL), GPIO39 (SDA) | 2 |

---

## 7. Conexión del pedal con sensor Hall y ADS1115 — Detalle completo

### 7.1 Componentes del sistema de pedal

| Componente | Modelo/Valor | Función |
|------------|-------------|---------|
| **Sensor Hall** | SS1324LUA-T | Sensor lineal magnético, alimentado a 5V |
| **ADC externo** | ADS1115 (módulo breakout) | ADC 16-bit I2C, canal de plausibilidad |
| **Resistencia R1** | 10 kΩ (1% precisión) | Parte superior del divisor de tensión |
| **Resistencia R2** | 6.8 kΩ (1% precisión) | Parte inferior del divisor de tensión |
| **Pull-ups I2C** | 2× 4.7 kΩ | En PB6 y PB7 (si no están en el módulo ADS1115) |

### 7.2 Cómo funciona el sensor Hall SS1324LUA-T

El sensor Hall SS1324LUA-T es un **sensor magnético lineal de efecto Hall**. Se monta de forma que un imán unido al mecanismo del pedal se mueve acercándose o alejándose del sensor conforme se pisa el pedal:

```
    Pedal suelto (reposo)              Pedal pisado (a fondo)
    ┌──────────────┐                   ┌──────────────┐
    │ Imán   [N S] │ ←── lejos         │ Imán   [N S] │ ←── cerca
    │              │                   │    ↓↓↓↓↓↓    │
    │  SS1324LUA-T │                   │  SS1324LUA-T │
    │  Salida: 0.3V│                   │  Salida: 4.8V│
    └──────────────┘                   └──────────────┘
```

- **Alimentación:** 5V (pin VCC) + GND
- **Salida:** Voltaje proporcional al campo magnético: 0.3V (sin campo) a 4.8V (campo máximo)
- **Sin contacto mecánico:** No hay desgaste, ideal para pedal de acelerador

### 7.3 Arquitectura de doble canal redundante

La señal del pedal se lee por **dos canales independientes** simultáneamente, como en la industria automotriz real:

```
                            ┌────────────────────────────────┐
                            │     SENSOR HALL SS1324LUA-T    │
                            │     (5V, salida 0.3V–4.8V)     │
                            │                                │
                            │  VCC (Pin 1) ────► 5V fuente   │
                            │  GND (Pin 2) ────► GND común   │
                            │  Señal (Pin 3) ──┬────────────►│ salida analógica
                            └──────────────────┼─────────────┘
                                               │
                    ┌──────────────────────────┤
                    │                          │
     ═══════════════════════        ═══════════════════════════
     ║ CANAL 1 — PRIMARIO  ║        ║ CANAL 2 — PLAUSIBILIDAD ║
     ║ (rápido, ~1 µs)     ║        ║ (preciso, ~8 ms)         ║
     ═══════════════════════        ═══════════════════════════
                    │                          │
                    ▼                          ▼
        ┌──────────────────┐        ┌──────────────────────┐
        │ DIVISOR RESISTIVO│        │      ADS1115         │
        │                  │        │   (I2C, 16-bit)      │
        │ Señal 5V ──┐    │        │                      │
        │            R1    │        │  VDD ──► 5V fuente   │
        │         10 kΩ    │        │  GND ──► GND común   │
        │            │     │        │  A0  ◄── Señal 5V    │
        │       NODO ├─────┼──► PA3 │  ADDR──► GND (=0x48) │
        │            │     │        │  SCL ──► PB6 (I2C)   │
        │           R2     │        │  SDA ──► PB7 (I2C)   │
        │         6.8 kΩ   │        │                      │
        │            │     │        └──────────────────────┘
        │           GND    │
        └──────────────────┘
                    │
                    ▼
        ┌──────────────────────┐
        │     STM32G474RE      │
        │                      │
        │  PA3: ADC1_IN4       │ ◄── 0.12V–1.94V (dividida)
        │  PB6: I2C1_SCL      │ ◄──► ADS1115 + TCA9548A/INA226
        │  PB7: I2C1_SDA      │ ◄──► ADS1115 + TCA9548A/INA226
        └──────────────────────┘
```

### 7.4 Canal 1 — Primario (ADC interno, PA3)

**Ruta de la señal:** Sensor → Divisor de tensión → PA3 (ADC1_IN4)

| Parámetro | Valor |
|-----------|-------|
| Pin STM32 | **PA3** |
| Periférico | ADC1, canal IN4 |
| Resolución | 12 bits (0–4095) |
| Latencia de lectura | ~1 µs |
| Rango de entrada en PA3 | 0.12V – 1.94V |
| Cálculo ADC mínimo | 0.121V / 3.3V × 4095 = **~150 counts** |
| Cálculo ADC máximo | 1.943V / 3.3V × 4095 = **~2413 counts** |

**Divisor de tensión — cálculo:**
```
Vout = Vin × R2 / (R1 + R2)
     = Vin × 6.8k / (10k + 6.8k)
     = Vin × 0.4048

Pedal suelto:  0.3V × 0.4048 = 0.121V  →  ADC ≈ 150
Pedal pisado:  4.8V × 0.4048 = 1.943V  →  ADC ≈ 2413
Máximo absoluto: 5.0V × 0.4048 = 2.024V  (bien debajo de 3.3V) ✅
```

> ⚠️ **IMPORTANTE:** Sin el divisor de tensión, la señal de 5V del sensor dañaría el pin PA3 (máximo absoluto: 3.6V).

### 7.5 Canal 2 — Plausibilidad (ADS1115, I2C)

**Ruta de la señal:** Sensor → ADS1115 entrada A0 → lectura I2C → STM32

| Parámetro | Valor |
|-----------|-------|
| Módulo | ADS1115 (breakout board) |
| Bus | I2C1 (PB6=SCL, PB7=SDA, 400 kHz) |
| Dirección I2C | **0x48** (pin ADDR conectado a GND) |
| Resolución | 16 bits (0–65535, con signo: ±32767) |
| PGA (ganancia) | ±4.096V (rango más adecuado para señal de 0–5V) |
| Sample Rate | 128 SPS |
| Latencia de lectura | ~8 ms |
| Rango de entrada en A0 | 0.3V – 4.8V (señal directa del sensor, sin dividir) |
| Cálculo ADS mínimo | 0.3V / 4.096V × 32767 = **~2400** (firmware usa 1600) |
| Cálculo ADS máximo | 4.8V / 4.096V × 32767 = **~38400** (firmware usa 25600) |

**Conexiones físicas del ADS1115:**

| Pin ADS1115 | Conectar a | Cable |
|-------------|-----------|-------|
| VDD | 5V fuente | Alimentación |
| GND | GND común | Masa |
| A0 | Pin 3 del sensor (señal) | Señal analógica directa 5V |
| ADDR | GND | Fija dirección I2C = 0x48 |
| SCL | PB6 (STM32) | Bus I2C compartido |
| SDA | PB7 (STM32) | Bus I2C compartido |
| A1, A2, A3 | Sin conectar | No usados |
| ALRT | Sin conectar | No usado |

> **Nota:** El ADS1115 alimentado a 5V es compatible con niveles I2C de 3.3V porque su umbral VIH = 0.7 × VDD ≈ 3.5V, ligeramente por encima de 3.3V. En la práctica funciona de forma fiable. Los módulos breakout comerciales suelen incluir adaptador de nivel I2C integrado.

### 7.6 Validación cruzada — Firmware

Cada 50 ms, el firmware ejecuta `Pedal_Update()` que:

1. **Lee el canal primario** (ADC interno, ~1 µs):
   - `HAL_ADC_Start()` → `HAL_ADC_PollForConversion()` → `HAL_ADC_GetValue()`
   - Convierte el valor ADC (150–2413) a porcentaje (0–100%)

2. **Lee el canal de plausibilidad** (ADS1115, ~8 ms):
   - Envía configuración por I2C (`I2C_Transmit` a 0x48)
   - Espera conversión (~8 ms)
   - Lee resultado 16 bits por I2C
   - Convierte el valor ADS (1600–25600) a porcentaje (0–100%)

3. **Compara ambos canales:**

| Condición | Resultado |
|-----------|-----------|
| Ambos coinciden ±5% | ✅ `pedal_plausible = true` — acelerador funciona normal |
| Divergencia > 5% durante > 200 ms | ❌ `pedal_plausible = false` — acelerador cortado a 0% |
| I2C del ADS1115 no responde > 500 ms | ⚠️ Modo degradado — solo canal primario (ADC) |

### 7.7 Lista de cables — Pedal completo

| # | De | A | Función | Tipo cable |
|---|-----|---|---------|-----------|
| 1 | Sensor Pin 1 (VCC) | 5V fuente | Alimentación sensor Hall | 26 AWG |
| 2 | Sensor Pin 2 (GND) | GND común | Masa sensor | 26 AWG |
| 3 | Sensor Pin 3 (Señal) | R1 (10 kΩ) entrada | Señal 5V al divisor | 26 AWG |
| 4 | Nodo R1/R2 | PA3 (STM32) | Señal dividida ~0–2V | 26 AWG, corto |
| 5 | R2 otro extremo | GND | Referencia del divisor | 26 AWG |
| 6 | Sensor Pin 3 (Señal) | ADS1115 A0 | Señal 5V directa al ADS1115 | 26 AWG |
| 7 | ADS1115 VDD | 5V fuente | Alimentación ADS1115 | 26 AWG |
| 8 | ADS1115 GND | GND común | Masa ADS1115 | 26 AWG |
| 9 | ADS1115 ADDR | GND | Fija dirección = 0x48 | 26 AWG |
| 10 | ADS1115 SCL | PB6 (STM32) | Bus I2C reloj | 28 AWG |
| 11 | ADS1115 SDA | PB7 (STM32) | Bus I2C datos | 28 AWG |

> **Total: 11 cables** para el sistema completo del pedal (5 para divisor + 6 para ADS1115).

### 7.8 Pines GPIO del STM32 usados por el pedal

| Pin | Función | ¿Exclusivo del pedal? |
|-----|---------|----------------------|
| **PA3** | ADC1_IN4 (canal primario) | ✅ Sí, exclusivo |
| **PB6** | I2C1_SCL (ADS1115 + TCA9548A + INA226) | ❌ Compartido con sensores de corriente |
| **PB7** | I2C1_SDA (ADS1115 + TCA9548A + INA226) | ❌ Compartido con sensores de corriente |

> **Resultado: Solo 1 pin GPIO adicional** (PA3) es exclusivo del pedal. El ADS1115 reutiliza el bus I2C existente (PB6/PB7) que ya se usa para los sensores de corriente INA226 y el multiplexor TCA9548A. Esto es posible porque el ADS1115 tiene una dirección I2C diferente (0x48) de los demás dispositivos del bus (0x70, 0x40).

---

## Resumen final

### ¿Hay suficientes pines?

| MCU | Pines necesarios | Pines disponibles | Pines usados | Pines libres | ¿Suficiente? |
|-----|-----------------|-------------------|-------------|-------------|--------------|
| **STM32G474RE** | 32 | 47 GPIO | 32 (68.1%) | 13 libres + 2 SWD | ✅ **SÍ** — sobran 13 pines |
| **ESP32-S3** | 14 | ~36 accesibles | 14 (38.9%) | ~15 libres | ✅ **SÍ** — sobran ~15 pines |

Ambos microcontroladores tienen **margen amplio** para expansiones futuras (botones físicos, LEDs de estado, SD card, buzzer, etc.).

> **Nota sobre phases:** De los 14 pines del ESP32-S3, 9 están actualmente en uso (display, touch, CAN). Los otros 5 pines (GPIO19/20 para DFPlayer, GPIO43/47 para LEDs WS2812B, GPIO44 para sensor TOFSense) están asignados y documentados pero el hardware aún no está conectado — se implementarán en Phase 3 (obstáculos/LEDs) y Phase 5 (audio).

---

**Documento creado:** 2026-02-17  
**Versión:** 1.1  
**Autor:** Sistema de Control Coche Marcos  
**Referencias:** `Core/Inc/main.h`, `esp32/platformio.ini`, `esp32/src/main.cpp`, `docs/CONEXIONES_COMPLETAS.md`, `docs/PEDAL_SENSOR_FINAL_SUMMARY.md`, `docs/OBSTACLE_SYSTEM_ARCHITECTURE.md`, `docs/ESP32_HMI_ARCHITECTURE_REBUILD.md` (Appendix D.6), `docs/PENDING_FEATURES_SCHEDULE.md`
