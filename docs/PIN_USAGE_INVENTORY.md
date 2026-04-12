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
7. [Conexión del pedal con sensor Hall — Detalle completo](#7-conexión-del-pedal-con-sensor-hall--detalle-completo)

---

## 1. STM32G474RE — Resumen

| Dato | Valor |
|------|-------|
| **MCU** | STM32G474RE |
| **Package** | LQFP64 (64 pines físicos) |
| **Pines totales del chip** | 64 |
| **Pines de alimentación/tierra/reset/cristal** | 13 (VBAT, VDDA, VSSA, 3×VDD, 4×VSS, NRST, PH0, PH1) |
| **Pines GPIO disponibles** | 47 (PA0–PA15, PB0–PB15, PC0–PC13, PD2) |
| **Pines usados por el proyecto** | 34 |
| **Pines reservados (SWD debug)** | 2 (PA13/SWDIO, PA14/SWCLK) |
| **Pines libres para expansión** | **11** |

### Distribución de pines usados por categoría

| Categoría | Pines usados | Porcentaje |
|-----------|-------------|------------|
| PWM motores RPWM/LPWM (TIM1 + TIM3 + TIM8) | 10 | 21.3% |
| Habilitación motores (GPIO out) | 5 (PC0, PC1, PC4, PC5, PC13) | 10.6% |
| Relés potencia (GPIO out) | 3 | 6.4% |
| Relés LED (GPIO out) | 2 (PB10, PB11) | 4.3% |
| Sensores velocidad rueda (EXTI) | 4 | 8.5% |
| Encoder dirección (TIM2 + EXTI) | 3 | 6.4% |
| Sensor centrado dirección (EXTI) | 1 | 2.1% |
| Pedal acelerador (ADC) | 1 | 2.1% |
| Bus I2C (INA226/TCA9548A) | 2 | 4.3% |
| Bus OneWire (DS18B20) | 1 | 2.1% |
| Bus CAN (FDCAN1) | 2 | 4.3% |
| **TOTAL USADOS** | **34** | **72.3%** |
| Reservados SWD | 2 | 4.3% |
| Pin DIR liberado (PC2) | 1 | 2.1% |
| **LIBRES** | **10** | **21.3%** |

---

## 2. STM32G474RE — Detalle por módulo

### 2.1 Motores de tracción — 4× BTS7960 (8 pines PWM + 4 pines EN = 12 pines)

Cada motor de tracción necesita 2 señales PWM (RPWM + LPWM) y 1 pin EN (habilitación).

| Motor | Pin RPWM | Timer | Pin LPWM | Timer | Pin EN | Total pines |
|-------|----------|-------|----------|-------|--------|-------------|
| FL (Delantero Izq.) | **PA8** | TIM1_CH1 | **PA9** | TIM1_CH2 | **PC5** (GPIO) | 3 |
| FR (Delantero Der.) | **PA10** | TIM1_CH3 | **PC3** | TIM1_CH4 (AF2) | **PC0** (GPIO) | 3 |
| RL (Trasero Izq.) | **PC6** | TIM8_CH1 | **PC7** | TIM8_CH2 | **PC1** (GPIO) | 3 |
| RR (Trasero Der.) | **PC8** | TIM8_CH3 | **PC9** | TIM8_CH4 | **PC13** (GPIO) | 3 |
| | | | | | **Subtotal:** | **8 PWM + 4 EN = 12 pines** |

- PWM a 20 kHz, center-aligned, resolución ~12 bits (ARR = 4249)
- Solo un canal (RPWM o LPWM) activo por motor a la vez
- TIM1 y TIM8: BREAK2 armado a Cortex LOCKUP (hardware PWM kill en fallo CPU)

### 2.2 Motor de dirección — 1× BTS7960 (2 pines PWM + 1 pin EN = 3 pines)

| Motor | Pin RPWM | Timer | Pin LPWM | Timer | Pin EN | Total pines |
|-------|----------|-------|----------|-------|--------|-------------|
| STEER (Dirección) | **PA6** | TIM3_CH1 | **PA7** | TIM3_CH2 | **PC4** (GPIO) | 3 |
| | | | | | **Subtotal:** | **3 pines** |

> PC9 fue reasignado a TIM8_CH4 (LPWM_RR). PC4 se usa ahora como EN_STEER (GPIO output).
> TIM3 no tiene entrada BREAK; los fault handlers escriben CCR1=0, CCR2=0 por software.

### 2.3 Relés de potencia (5 pines)

| Relé | Pin | Función |
|------|-----|---------|
| RELAY_MAIN | **PC10** | Alimentación general |
| RELAY_TRAC | **PC11** | Alimentación motores 24V |
| RELAY_DIR | **PC12** | Alimentación motor dirección 12V |
| RELAY_LED (front) | **PB10** | Alimentación 5V tira LED frontal WS2812B |
| RELAY_LED_REAR | **PB11** | Alimentación 5V tira LED trasera WS2812B |
| | **Subtotal:** | **5 pines** |

- PC10–PC12: controlados vía optoacopladores (HY-M158), activo HIGH
- PB10/PB11: controlados vía CAN (ID 0x120); el ESP32 genera la señal WS2812B, el STM32 controla el relé de alimentación

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
- La plausibilidad se realiza por software (dual-sample ADC, no requiere hardware adicional)

### 2.8 Bus I2C — TCA9548A + 6× INA226 (2 pines)

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

> **Nota:** Los 6 sensores INA226 y el multiplexor TCA9548A comparten los mismos 2 pines I2C (7 dispositivos físicos, 2 direcciones visibles en el bus: 0x70 para TCA9548A y 0x40 para INA226 vía mux).

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
| CAN RX | **PA11** | FDCAN1_RX (AF9) | Recepción CAN a 500 kbps |
| CAN TX | **PA12** | FDCAN1_TX (AF9) | Transmisión CAN a 500 kbps |
| | **Subtotal:** | | **2 pines** |

- Transceiver externo SN65HVD230 requerido
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
| 5 | PA6 | GPIOA | Motor STEER | TIM3_CH1, RPWM_STEER 20 kHz |
| 6 | PA7 | GPIOA | Motor STEER | TIM3_CH2, LPWM_STEER 20 kHz |
| 7 | PA8 | GPIOA | Motor FL | TIM1_CH1, RPWM_FL 20 kHz |
| 8 | PA9 | GPIOA | Motor FL | TIM1_CH2, LPWM_FL 20 kHz |
| 9 | PA10 | GPIOA | Motor FR | TIM1_CH3, RPWM_FR 20 kHz |
| 10 | PC3 | GPIOC | Motor FR | TIM1_CH4, LPWM_FR 20 kHz |
| 11 | PA15 | GPIOA | Encoder dirección | TIM2_CH1, canal A cuadratura |
| 12 | PB0 | GPIOB | Temperatura (DS18B20) | OneWire bus, 5 sensores |
| 13 | PB3 | GPIOB | Encoder dirección | TIM2_CH2, canal B cuadratura |
| 14 | PB4 | GPIOB | Encoder dirección | EXTI4, índice Z |
| 15 | PB5 | GPIOB | Centrado dirección | EXTI5, sensor inductivo |
| 16 | PB6 | GPIOB | I2C (corriente/pedal) | I2C1_SCL, 400 kHz |
| 17 | PB7 | GPIOB | I2C (corriente/pedal) | I2C1_SDA, 400 kHz |
| 18 | PA11 | GPIOA | CAN bus | FDCAN1_RX, 500 kbps |
| 19 | PA12 | GPIOA | CAN bus | FDCAN1_TX, 500 kbps |
| 20 | PB10 | GPIOB | Relé LED frontal | Alimentación 5V tira WS2812B frontal |
| 21 | PB11 | GPIOB | Relé LED trasero | Alimentación 5V tira WS2812B trasera |
| 22 | PB15 | GPIOB | Sensor rueda RR | EXTI15, velocidad rueda |
| 23 | PC5 | GPIOC | Motor FL | EN (habilitación), GPIO active HIGH |
| 24 | PC6 | GPIOC | Motor RL | TIM8_CH1, RPWM_RL 20 kHz |
| 25 | PC7 | GPIOC | Motor RL | TIM8_CH2, LPWM_RL 20 kHz |
| 26 | PC8 | GPIOC | Motor RR | TIM8_CH3, RPWM_RR 20 kHz |
| 27 | PC9 | GPIOC | Motor RR | TIM8_CH4, LPWM_RR 20 kHz |
| 28 | PC10 | GPIOC | Relé MAIN | Alimentación general |
| 29 | PC11 | GPIOC | Relé TRAC | Alimentación motores |
| 30 | PC12 | GPIOC | Relé DIR | Alimentación dirección |
| 31 | PC13 | GPIOC | Motor RR | EN (habilitación), GPIO active HIGH |
| 32 | PC0 | GPIOC | Motor FR | EN (habilitación), GPIO active HIGH |
| 33 | PC1 | GPIOC | Motor RL | EN (habilitación), GPIO active HIGH |
| 34 | PC4 | GPIOC | Motor STEER | EN (habilitación), GPIO active HIGH |

---

## 3. STM32G474RE — Pines libres

Los siguientes pines GPIO del STM32G474RE están disponibles para expansión o han sido reasignados recientemente:

| # | Pin | Puerto | Funciones alternativas disponibles | Estado |
|---|-----|--------|--------------------------------------|--------|
| 1 | PA4 | GPIOA | DAC1_OUT1, SPI1_NSS, ADC2_IN17 | **LIBRE** |
| 2 | PA5 | GPIOA | DAC1_OUT2, SPI1_SCK, ADC2_IN13 | **LIBRE** |
| 3 | PB1 | GPIOB | ADC3_IN1, TIM3_CH4 | **LIBRE** |
| 4 | PB2 | GPIOB | GPIO general | **LIBRE** |
| 5 | PB12 | GPIOB | SPI2_NSS, I2S2_WS | **LIBRE** |
| 6 | PB13 | GPIOB | SPI2_SCK, I2S2_CK | **LIBRE** |
| 7 | PB14 | GPIOB | GPIO_Output | **LED_DIAG** |
| 8 | PC0 | GPIOC | GPIO, ADC1_IN6 | **EN_FR** — habilitación motor FR (GPIO) |
| 9 | PC1 | GPIOC | GPIO, ADC1_IN7 | **EN_RL** — habilitación motor RL (GPIO) |
| 10 | PC2 | GPIOC | GPIO, ADC1_IN8 | **LIBRE** (antes DIR_RL, liberado) |
| 11 | PC3 | GPIOC | TIM1_CH4 (AF2) | **LPWM_FR** — PWM motor FR 20 kHz |
| 12 | PC4 | GPIOC | GPIO, ADC2_IN5 | **EN_STEER** — habilitación motor dirección (GPIO) |
| 13 | PD2 | GPIOD | GPIO general, TIM3_ETR | **LIBRE** |

Además, los 2 pines SWD están reservados pero podrían reutilizarse si no se necesita debugging:

| # | Pin | Puerto | Función reservada | Nota |
|---|-----|--------|-------------------|------|
| 14 | PA13 | GPIOA | SWDIO (debug) | ⚠️ NO recomendado liberar |
| 15 | PA14 | GPIOA | SWCLK (debug) | ⚠️ NO recomendado liberar |

> **Resumen: 8 pines libres para uso inmediato** (10 si se sacrifica debug SWD, no recomendado).

### Posibles usos de los pines libres

| Posible expansión | Pines sugeridos | Nº pines |
|-------------------|-----------------|----------|
| UART debug serie | PB12 (TX) + PB13 (RX) | 2 |
| SPI adicional (sensor, SD card) | PA5 (SCK) + PC2 (MISO) + PA4 (CS) | 3 |
| ADC adicional (sensor batería, otro sensor) | PB1, PA4, PA5 | 1-3 |
| LEDs de estado / buzzer | PB2, PB12, PB13 | 1-3 |
| Sensores adicionales | PC2, PB12, PB13 | 2-3 |

---

## 4. ESP32-S3 — Resumen

| Dato | Valor |
|------|-------|
| **MCU** | ESP32-S3 (Xtensa LX7 dual-core @ 240 MHz) |
| **Placa** | ESP32-S3-DevKitC-1 |
| **Pines GPIO totales del chip** | 45 (GPIO0–GPIO21, GPIO26–GPIO48; GPIO22–25 no existen en ESP32-S3) |
| **Pines disponibles en DevKitC** | ~36 (algunos reservados por flash/PSRAM) |
| **Pines usados por el proyecto** | 21 |
| **Pines reservados — QSPI Flash** | 7 (GPIO26–GPIO32) |
| **Pines reservados — Octal PSRAM** | 5 (GPIO33–GPIO37, N16R8) |
| **Pines no existentes en ESP32-S3** | 4 (GPIO22–GPIO25) |
| **Pines libres para expansión** | **~11** |

### Distribución de pines usados por categoría

| Categoría | Pines usados | Porcentaje |
|-----------|-------------|------------|
| Display TFT SPI (ST7796) | 6 | 16.7% |
| Touch panel SPI | 1 | 2.8% |
| Display backlight | 1 | 2.8% |
| CAN bus (TWAI vía SN65HVD230) | 2 | 5.6% |
| Sensor de obstáculos TOFSense-M | 1 (UART RX) | 2.8% |
| DFPlayer Mini audio (UART2) | 2 | 5.6% |
| Relé audio | 1 | 2.8% |
| LEDs WS2812B (front + rear) | 2 | 5.6% |
| Palanca de cambios I2C (MCP23017) | 2 | 5.6% |
| Interruptor tracción 2WD/4WD | 1 | 2.8% |
| Ignition sense + Power hold | 2 | 5.6% |
| **TOTAL USADOS** | **21** | **58.3%** |
| Reservados Flash/PSRAM | 12 (GPIO26–37) | — |
| No existen en ESP32-S3 | 4 (GPIO22–25) | — |
| **LIBRES** | **~11** | **~30.6%** |

---

## 5. ESP32-S3 — Detalle por módulo

### 5.1 Display TFT — ST7796 480×320 vía SPI (7 pines)

| Señal | GPIO | Función | Velocidad |
|-------|------|---------|-----------|
| MISO (SDO) | **GPIO12** | Datos SPI desde display/touch | 20 MHz |
| MOSI (SDA) | **GPIO13** | Datos SPI al display | 40 MHz |
| SCLK (SCL) | **GPIO14** | Reloj SPI | 40 MHz |
| CS | **GPIO10** | Chip Select display | — |
| DC (A0) | **GPIO39** | Data/Command select | — |
| RST | **GPIO38** | Reset hardware del display | — |
| BL | **GPIO42** | Backlight (HIGH = encendido) | — |
| | | **Subtotal:** | **7 pines** |

- Driver: ST7796, resolución 480×320 en modo landscape (rotación 1)
- Biblioteca: TFT_eSPI v2.5.43
- MISO (GPIO 12): compartido con T_DO del XPT2046 (necesario para touch)

### 5.2 Panel táctil — SPI (1 pin)

| Señal | GPIO | Función | Velocidad |
|-------|------|---------|-----------|
| TOUCH_CS | **GPIO21** | Chip Select del panel táctil | 2.5 MHz |
| | | **Subtotal:** | **1 pin** |

- Comparte MOSI (GPIO13), SCLK (GPIO14) y MISO (GPIO12) con el display
- TOUCH_CS cuenta como pin adicional; MISO (T_DO) se comparte con el bus SPI

### 5.3 CAN bus — TWAI vía SN65HVD230 (2 pines)

| Señal | GPIO | Función | Velocidad |
|-------|------|---------|-----------|
| CAN TX | **GPIO4** | Transmisión CAN (TWAI_TX) | 500 kbps |
| CAN RX | **GPIO5** | Recepción CAN (TWAI_RX) | 500 kbps |
| | | **Subtotal:** | **2 pines** |

- Transceiver externo SN65HVD230 obligatorio
- Biblioteca: ESP32-TWAI-CAN v1.0.1
- Terminación 120Ω en el lado ESP32

### 5.4 Sensor de obstáculos — TOFSense-M LiDAR (Nooploop) (1 pin)

| Señal | GPIO | Función | Velocidad |
|-------|------|---------|-----------|
| UART1 RX | **GPIO18** | Datos del sensor (sensor TX → ESP32 RX) | 921600 bps |
| | | **Subtotal:** | **1 pin** |

- Sensor: TOFSense-M S (Nooploop), LiDAR 8×8 Time-of-Flight, rango 20–4000 mm
- Protocolo: NLink_TOFSense_M_Frame0 (400 bytes/trama, ~10 Hz)
- Comunicación: UART 921600 bps, 8N1, recepción unidireccional
- Datos enviados al STM32 vía CAN ID 0x208 (distancia, zona, salud, contador)

### 5.5 DFPlayer Mini — Audio MP3 vía UART2 (2 pines)

| Señal | GPIO | Función | Velocidad |
|-------|------|---------|-----------|
| UART2 TX | **GPIO43** | Comandos al DFPlayer (ESP32 → DFPlayer RX) | 9600 baud |
| UART2 RX | **GPIO44** | Respuestas del DFPlayer (DFPlayer TX → ESP32) | 9600 baud |
| | | **Subtotal:** | **2 pines** |

- Módulo: DFPlayer Mini (reproductor MP3 con DAC + amplificador 3W)
- Tarjeta SD: FAT32, 68 archivos MP3 (0001.mp3–0068.mp3)
- Altavoz: 3W 8Ω conectado a las salidas DAC_R/DAC_L del DFPlayer
- Biblioteca: DFRobotDFPlayerMini (inicialización lazy, no bloqueante)

### 5.5a Relé de audio (1 pin)

| Señal | GPIO | Función | Lógica |
|-------|------|---------|--------|
| Relay Coil | **GPIO11** | Conmuta altavoz entre radio y DFPlayer | Active LOW |
| | | **Subtotal:** | **1 pin** |

- LOW = DFPlayer conectado al altavoz; HIGH = radio conectada
- Tiempo de estabilización del relé: 20 ms

### 5.5b Palanca de cambios — MCP23017 vía I2C (2 pines)

| Señal | GPIO | Función | Velocidad |
|-------|------|---------|-----------|
| SDA | **GPIO8** | I2C datos | Default (100 kHz) |
| SCL | **GPIO9** | I2C reloj | Default (100 kHz) |
| | | **Subtotal:** | **2 pines** |

- IC: MCP23017 (expansor I/O 16-bit I2C) @ dirección 0x20
- Puerto A lee 5 posiciones: P (0x00), R (0x01), N (0x02), D1 (0x03), D2 (0x04)
- Marcha enviada por CAN (CMD_MODE 0x102, byte 1)

### 5.5c Interruptor de tracción 2WD/4WD (1 pin)

| Señal | GPIO | Función | Lógica |
|-------|------|---------|--------|
| Switch | **GPIO15** | Entrada con pull-up interno | LOW = 4WD, HIGH = 2WD |
| | | **Subtotal:** | **1 pin** |

- Tipo: DPDT rocker switch (usado como SPDT)
- Debounce: 50 ms × 3 lecturas estables
- Gate de velocidad: rechaza cambio si >0.5 km/h
- Modo enviado por CAN (CMD_MODE 0x102, byte 0, bit 0)

### 5.5d Control de encendido (2 pines)

| Señal | GPIO | Función | Lógica |
|-------|------|---------|--------|
| Ignition Sense | **GPIO40** | Llave de contacto (input, PULLDOWN) | HIGH = contacto ON |
| Power Hold | **GPIO41** | Mantiene alimentación (output) | HIGH = mantener encendido |
| | | **Subtotal:** | **2 pines** |

- Secuencia: llave ON → Power Hold → arranque sistema → llave OFF → shutdown delay (3 s) → Power Hold OFF

### 5.6 LEDs WS2812B — Iluminación front/rear (2 pines)

| Señal | GPIO | Función | Protocolo |
|-------|------|---------|-----------|
| LED Front Data | **GPIO47** | Tira frontal 28× WS2812B | FastLED / RMT 800 kHz |
| LED Rear Data | **GPIO48** | Tira trasera 16× WS2812B | FastLED / RMT 800 kHz |
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
| 1 | GPIO4 | CAN bus | TWAI TX → SN65HVD230 D |
| 2 | GPIO5 | CAN bus | TWAI RX → SN65HVD230 R |
| 3 | GPIO8 | Palanca de cambios | I2C SDA (MCP23017 @ 0x20) |
| 4 | GPIO9 | Palanca de cambios | I2C SCL (MCP23017 @ 0x20) |
| 5 | GPIO10 | Display TFT | SPI CS (chip select display) |
| 6 | GPIO11 | Relé audio | Conmuta altavoz radio/DFPlayer (active LOW) |
| 7 | GPIO12 | Display TFT | SPI MISO (datos desde display/touch) |
| 8 | GPIO13 | Display TFT | SPI MOSI (datos al display) |
| 9 | GPIO14 | Display TFT | SPI SCLK (reloj) |
| 10 | GPIO15 | Tracción | Interruptor 2WD/4WD (pull-up, LOW=4WD) |
| 11 | GPIO18 | Sensor obstáculos | TOFSense-M UART1 RX (921600 bps) |
| 12 | GPIO21 | Panel táctil | TOUCH_CS (chip select touch) |
| 13 | GPIO38 | Display TFT | RST (Reset display) |
| 14 | GPIO39 | Display TFT | DC (Data/Command) |
| 15 | GPIO40 | Encendido | Ignition Key Sense (input, PULLDOWN) |
| 16 | GPIO41 | Encendido | Power Hold Output (active HIGH) |
| 17 | GPIO42 | Display TFT | BL (Backlight) |
| 18 | GPIO43 | DFPlayer Mini | UART2 TX (comandos audio) |
| 19 | GPIO44 | DFPlayer Mini | UART2 RX (respuestas audio) |
| 20 | GPIO47 | LEDs WS2812B | Tira frontal 28 LEDs |
| 21 | GPIO48 | LEDs WS2812B | Tira trasera 16 LEDs |

---

## 6. ESP32-S3 — Pines libres

Los siguientes GPIO del ESP32-S3-DevKitC-1 **NO están usados** y están disponibles:

| # | GPIO | Funciones disponibles | Estado |
|---|------|----------------------|--------|
| 1 | GPIO0 | ADC1_CH0, GPIO, touch | **LIBRE** (⚠️ boot strapping) |
| 2 | GPIO1 | ADC1_CH1, GPIO | **LIBRE** |
| 3 | GPIO2 | ADC1_CH2, GPIO | **LIBRE** |
| 4 | GPIO3 | ADC1_CH3, GPIO | **LIBRE** (⚠️ boot strapping) |
| 5 | GPIO6 | GPIO | **LIBRE** |
| 6 | GPIO7 | GPIO | **LIBRE** |
| 7 | GPIO16 | GPIO | **LIBRE** |
| 8 | GPIO17 | GPIO | **LIBRE** |
| 9 | GPIO19 | GPIO (USB D−) | **LIBRE** (si no se usa USB nativo) |
| 10 | GPIO20 | GPIO (USB D+) | **LIBRE** (si no se usa USB nativo) |
| 11 | GPIO46 | GPIO | **LIBRE** (⚠️ boot strapping, solo input) |

> **Nota:** GPIO26–GPIO32 están reservados para el bus QSPI Flash. GPIO33–GPIO37 están reservados para el bus Octal PSRAM en el módulo N16R8. GPIO22–25 no existen en el chip ESP32-S3. GPIO45 es un strapping pin (VDD_SPI) y se ha evitado. Los pines GPIO4–15, GPIO18 y GPIO38–48 están asignados según tabla anterior.

> **Resumen: ~11 pines libres de forma segura** (evitando pines de boot strapping).

### Posibles usos de los pines libres

| Posible expansión | GPIOs sugeridos | Nº pines |
|-------------------|-----------------|----------|
| Botones físicos (menú, modo, emergencia) | GPIO6, GPIO7, GPIO16 | 3 |
| LEDs de estado (CAN ok, error, etc.) | GPIO6, GPIO7 | 2 |
| SD Card (SPI) | GPIO16 (MOSI), GPIO17 (SCK), GPIO19 (MISO), GPIO20 (CS) | 4 |
| Buzzer / alarma sonora | GPIO16 | 1 |
| ADC adicional (batería, sensor luz) | GPIO1, GPIO2, GPIO3 | 1-3 |

---

## 7. Conexión del pedal con sensor Hall — Detalle completo

### 7.1 Componentes del sistema de pedal

| Componente | Modelo/Valor | Función |
|------------|-------------|---------|
| **Sensor Hall** | SS1324LUA-T | Sensor lineal magnético, alimentado a 5V |
| **Resistencia R1** | 10 kΩ (1% precisión) | Parte superior del divisor de tensión |
| **Resistencia R2** | 6.8 kΩ (1% precisión) | Parte inferior del divisor de tensión |

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

La señal del pedal se lee por **dos muestras ADC consecutivas** con validación por software, como en la industria automotriz real:

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
     ║ (rápido, ~1 µs)     ║        ║ (software, ~2 µs)        ║
     ═══════════════════════        ═══════════════════════════
                    │                          │
                    ▼                          ▼
        ┌──────────────────┐        ┌──────────────────────┐
        │ DIVISOR RESISTIVO│        │ PLAUSIBILIDAD        │
        │                  │        │ SOFTWARE             │
        │ Señal 5V ──┐    │        │                      │
        │            R1    │        │ · Dual-sample ADC    │
        │         10 kΩ    │        │ · Filtro EMA α=0.3   │
        │            │     │        │ · Rango [30..2800]   │
        │       NODO ├─────┼──► PA3 │ · Tasa máx 35%/50ms  │
        │            │     │        └──────────────────────┘
        │           R2     │
        │         6.8 kΩ   │
        │            │     │
        │           GND    │
        └──────────────────┘
                    │
                    ▼
        ┌──────────────────────┐
        │     STM32G474RE      │
        │                      │
        │  PA3: ADC1_IN4       │ ◄── 0.12V–1.94V (dividida)
        │  PB6: I2C1_SCL      │ ◄──► TCA9548A/INA226
        │  PB7: I2C1_SDA      │ ◄──► TCA9548A/INA226
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

### 7.5 Plausibilidad por Software (sin ADS1115)

> **CAMBIO:** El ADS1115 ha sido eliminado. La plausibilidad se realiza por software.

**Método:** Dos lecturas ADC consecutivas (~2 µs total) con 4 capas de validación:

| Verificación | Parámetro | Acción si falla |
|-------------|-----------|-----------------|
| Consistencia dual-sample | Diferencia > 30 counts | `pedal_channels_contradict = true` |
| Rango válido | < 30 o > 2800 counts | `pedal_plausible = false` |
| Tasa de cambio | > 35% por ciclo 50 ms | `pedal_plausible = false` |
| Filtro EMA | α=0.3 (~150 ms settling) | Suaviza ruido |

### 7.6 Validación cruzada — Firmware

Cada 50 ms, el firmware ejecuta `Pedal_Update()` que:

1. **Lee primera muestra ADC** (ADC interno, ~1 µs):
   - `HAL_ADC_Start()` → `HAL_ADC_PollForConversion()` → `HAL_ADC_GetValue()`
   - Convierte el valor ADC (150–2413) a porcentaje (0–100%)

2. **Lee segunda muestra ADC** (~1 µs):
   - Segunda lectura consecutiva del mismo canal PA3
   - Verifica consistencia con la primera muestra

3. **Valida por software:**

| Condición | Resultado |
|-----------|-----------|
| Ambas muestras coinciden (diferencia < 30 counts) | ✅ `pedal_plausible = true` — acelerador funciona normal |
| Divergencia entre muestras > 30 counts | ❌ `pedal_channels_contradict = true` — señal sospechosa |
| Valor fuera de rango [30..2800] | ❌ `pedal_plausible = false` — acelerador cortado a 0% |
| Tasa de cambio > 35% por ciclo 50 ms | ❌ `pedal_plausible = false` — acelerador cortado a 0% |

### 7.7 Lista de cables — Pedal completo

| # | De | A | Función | Tipo cable |
|---|-----|---|---------|-----------|
| 1 | Sensor Pin 1 (VCC) | 5V fuente | Alimentación sensor Hall | 26 AWG |
| 2 | Sensor Pin 2 (GND) | GND común | Masa sensor | 26 AWG |
| 3 | Sensor Pin 3 (Señal) | R1 (10 kΩ) entrada | Señal 5V al divisor | 26 AWG |
| 4 | Nodo R1/R2 | PA3 (STM32) | Señal dividida ~0–2V | 26 AWG, corto |
| 5 | R2 otro extremo | GND | Referencia del divisor | 26 AWG |

> **Total: 5 cables** para el sistema completo del pedal (sensor + divisor resistivo).

### 7.8 Pines GPIO del STM32 usados por el pedal

| Pin | Función | ¿Exclusivo del pedal? |
|-----|---------|----------------------|
| **PA3** | ADC1_IN4 (canal primario + plausibilidad dual-sample) | ✅ Sí, exclusivo |
| **PB6** | I2C1_SCL (TCA9548A + INA226) | ❌ Compartido con sensores de corriente |
| **PB7** | I2C1_SDA (TCA9548A + INA226) | ❌ Compartido con sensores de corriente |

> **Resultado: Solo 1 pin GPIO adicional** (PA3) es exclusivo del pedal. La plausibilidad se realiza por software (dual-sample ADC) sin necesidad de hardware externo adicional. El bus I2C (PB6/PB7) se usa exclusivamente para los sensores de corriente INA226 y el multiplexor TCA9548A.

---

## Resumen final

### ¿Hay suficientes pines?

| MCU | Pines necesarios | Pines disponibles | Pines usados | Pines libres | ¿Suficiente? |
|-----|-----------------|-------------------|-------------|-------------|--------------|
| **STM32G474RE** | 31 | 47 GPIO | 31 (66.0%) | 14 libres + 2 SWD | ✅ **SÍ** — sobran 14 pines (incluye 5 DIR liberados) |
| **ESP32-S3** | 21 | ~36 accesibles | 21 (58.3%) | ~11 libres | ✅ **SÍ** — sobran ~11 pines |

Ambos microcontroladores tienen **margen** para expansiones futuras (botones físicos, SD card, buzzer, etc.).

---

**Documento actualizado:** 2026-02-27
**Versión:** 2.0
**Autor:** Sistema de Control Coche Marcos
**Referencias:** `Core/Inc/main.h`, `esp32/platformio.ini`, `esp32/src/main.cpp`, `esp32/include/can_ids.h`, `esp32/src/traction_switch.cpp`, `esp32/src/shifter_input.cpp`, `esp32/src/audio_manager.cpp`, `esp32/src/led_controller.cpp`, `esp32/src/power_manager.cpp`, `esp32/src/relay_audio.cpp`
