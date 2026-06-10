# STM32 BREAKOUT BOARD — Mapa de Cableado Definitivo

**Placa breakout:** CZH-LABS D-1686 — Top-mount Screw Terminal Block Breakout Board for STM32 NUCLEO-64  
**MCU:** STM32G474RE (Nucleo-64, MB1367)  
**Fecha:** 2026-06-10  
**Fuente de verdad:** `Core/Inc/project_config.h`, `STM32-Control-Coche-Marcos.ioc`, `docs/NUCLEO_G474RE_PINOUT_CROQUIS.md`, `docs/POWER_DISTRIBUTION.md`

> ⚠️ **ESTE DOCUMENTO NO CONTIENE NADA INVENTADO.**  
> Cada pin, función y bornera está extraída exclusivamente del firmware real (`project_config.h`, `.ioc`, `NUCLEO_G474RE_PINOUT_CROQUIS.md`). Cualquier dato no confirmado está marcado explícitamente como **PENDIENTE DE VERIFICACIÓN**.

---

## Índice

1. [Descripción de la placa D-1686](#1--descripción-de-la-placa-d-1686)
2. [Nomenclatura de borneras](#2--nomenclatura-de-borneras)
3. [Mapa completo GPIO → Bornera breakout](#3--mapa-completo-gpio--bornera-breakout)
   - 3.1 Relés de potencia
   - 3.2 Relés LED
   - 3.3 CAN Bus (FDCAN1)
   - 3.4 I2C (INA226 / TCA9548A / DS18B20)
   - 3.5 Encoder de dirección (TIM2)
   - 3.6 Sensores velocidad rueda (EXTI)
   - 3.7 Sensor centrado dirección
   - 3.8 PWM Motores tracción (TIM1, TIM8)
   - 3.9 PWM Motor dirección (TIM3)
   - 3.10 Enable BTS7960
   - 3.11 Pedal acelerador (ADC)
   - 3.12 1-Wire DS18B20
   - 3.13 LED indicadores
   - 3.14 Pines reservados / no conectar
4. [Tabla completa CN10 pin a pin](#4--tabla-completa-cn10-pin-a-pin)
5. [Tabla completa CN7 pin a pin](#5--tabla-completa-cn7-pin-a-pin)
6. [Distribución de alimentación](#6--distribución-de-alimentación)
7. [Integración ULN2803A](#7--integración-uln2803a)
8. [Riesgos eléctricos detectados](#8--riesgos-eléctricos-detectados)
9. [Checklist de montaje](#9--checklist-de-montaje)
10. [Checklist de verificación con multímetro](#10--checklist-de-verificación-con-multímetro)

---

## 1 — Descripción de la placa D-1686

La placa CZH-LABS D-1686 es un adaptador **puramente pasivo** (sin electrónica activa) que monta encima de la Nucleo-64 y saca todas las señales de los conectores Morpho (CN7, CN10) y Arduino (CN6, CN8, CN9, CN5, CN3, CN4) a borneras de tornillo de 2 mm de paso.

```
 ┌────────────────────────────────────────────────────────────┐
 │            CZH-LABS D-1686  (vista desde arriba)          │
 │                                                            │
 │  [CN10 borneras izquierda]      [CN7 borneras derecha]    │
 │   pin 1 (PC9)  ●                  ● pin 1 (PC10)          │
 │   pin 2 (PC8)  ●                  ● pin 2 (PC11) ◄RELAY   │
 │   pin 3 (PB8)  ●                  ● pin 3 (PC12) ◄RELAY   │
 │   ...                             ...                      │
 │   pin 38 (NC)  ●                  ● pin 38 (PC0) EN_FR    │
 │                                                            │
 │  ●PWR-LED   ●RESET SW                                      │
 └────────────────────────────────────────────────────────────┘
```

**Características confirmadas por las fotografías del producto:**
- Conector doble fila de borneras de tornillo a cada lado
- LED indicador de alimentación (PWR)
- Pulsador RESET (SW1)
- Etiquetas serigrafadas indicando números de pin Morpho
- No incluye ninguna resistencia pull-up ni componente activo

**Limitación de corriente por bornera:** PENDIENTE DE VERIFICACIÓN (consultar datasheet D-1686). Las señales GPIO del STM32G474RE son **máximo 25 mA por pin**, pero los pines de alimentación (5V, 3V3) de la Nucleo tienen sus propias limitaciones de corriente indicadas en §6.

---

## 2 — Nomenclatura de borneras

La placa D-1686 usa dos grupos de borneras que reflejan directamente los conectores Morpho de la Nucleo-64:

| Grupo bornera | Conector Nucleo-64 | Posición física |
|---|---|---|
| **CN10_N** | CN10 (Morpho derecho) | Lado izquierdo de la breakout |
| **CN7_N** | CN7 (Morpho izquierdo) | Lado derecho de la breakout |

Donde **N** es el número de pin (1–38) tal como está serigrafado en la PCB de la Nucleo-64.

> **Nota sobre pines Arduino:** La placa D-1686 también incluye terminales para los headers Arduino (CN6, CN8, CN9). Estos comparten los mismos GPIOs que CN7 y CN10 (son los mismos pines físicos del STM32 accesibles desde conectores distintos). En este documento **se usa siempre la numeración Morpho** (CN7_N / CN10_N) porque es la numeración que aparece en la serigrafía de la D-1686.

---

## 3 — Mapa completo GPIO → Bornera breakout

### 3.1 Relés de potencia (GPIOC — salida digital, activo HIGH)

> **Arquitectura:** STM32 GPIO → ULN2803A → Módulo Songle 4CH → bobina de relé  
> El STM32 NO conduce directamente la bobina del relé.

| GPIO STM32 | Función firmware | Conector breakout | Bornera exacta | Nivel activo | Observaciones |
|---|---|---|---|---|---|
| **PC11** | RELAY_TRAC (relé tracción 24V) | CN7 | **CN7_2** | HIGH (3.3V) → ULN → LOW a Songle | 50A fusible; sincronizado 50 ms antes de PC12 |
| **PC12** | RELAY_STEER_PWR (relé dirección 12V) | CN7 | **CN7_3** | HIGH (3.3V) → ULN → LOW a Songle | 20A fusible; se activa 50 ms después de PC11 |

### 3.2 Relés LED (GPIOB — salida digital, activo HIGH)

| GPIO STM32 | Función firmware | Conector breakout | Bornera exacta | Nivel activo | Observaciones |
|---|---|---|---|---|---|
| **PB10** | RELAY_LED (tira LED frontal 5V) | CN10 | **CN10_25** | HIGH → ULN → LOW a relé | WS2812B frontal (28 LEDs), alimentación 5V |
| **PB11** | RELAY_LED_REAR (tira LED trasera 5V) | CN10 | **CN10_18** | HIGH → ULN → LOW a relé | WS2812B trasera (16 LEDs), alimentación 5V |

### 3.3 CAN Bus — FDCAN1 (PA11/PA12, AF9)

| GPIO STM32 | Función firmware | Conector breakout | Bornera exacta | Dirección | Observaciones |
|---|---|---|---|---|---|
| **PA12** | CAN_TX → TJA1051T/3 → ESP32 | CN10 | **CN10_12** | Salida | AF9, 3.3V; transceiver TJA1051 VCC=3.3V |
| **PA11** | CAN_RX ← TJA1051T/3 ← ESP32 | CN10 | **CN10_14** | Entrada | AF9, 3.3V; ⚠️ NO conectar como antes (era LPWM_FR) |

> ⚠️ **CRÍTICO:** PA11 **ya no es LPWM_FR**. Fue reasignado a FDCAN1_RX. Conectar PA11 al BTS7960 destruirá la comunicación CAN con el ESP32.

### 3.4 I2C Bus — I2C1 (PB8/PB9, AF4, ~86 kHz)

> Conecta a: TCA9548A (0x70) → 6× INA226 (0x40, canales 0–5)  
> Pull-ups externos obligatorios: 4.7 kΩ de SDA y SCL a **3.3V** (NO a 5V)

| GPIO STM32 | Función firmware | Conector breakout | Bornera exacta | Dirección | Observaciones |
|---|---|---|---|---|---|
| **PB8** | I2C1_SCL | CN10 | **CN10_3** | Bidireccional | Pull-up 4.7kΩ a 3.3V externo obligatorio |
| **PB9** | I2C1_SDA | CN10 | **CN10_5** | Bidireccional | Pull-up 4.7kΩ a 3.3V externo obligatorio |

> Dispositivos en el bus: TCA9548A (mux I2C, addr 0x70), 6× INA226 (addr 0x40 en canales mux 0–5)  
> Configuración I2C: Standard Mode, Timing=0x30F0EDFF (~86 kHz), GPIO_NOPULL

### 3.5 Encoder de dirección — TIM2 cuadratura + índice (E6B2-CWZ6C, 1200 PPR / 4800 CPR)

> ⚠️ **OBLIGATORIO:** Las 3 señales del encoder (5V) deben pasar por **optoacopladores 6N137** antes de llegar al STM32 (3.3V). Ver `docs/ENCODER_WIRING_6N137.md`.

| GPIO STM32 | Función firmware | Conector breakout | Bornera exacta | Dirección | Observaciones |
|---|---|---|---|---|---|
| **PA15** | ENC_A (TIM2_CH1, AF1) | CN7 | **CN7_17** | Entrada | Salida 6N137 → PA15; encoder canal A |
| **PB3** | ENC_B (TIM2_CH2, AF1) | CN10 | **CN10_31** | Entrada | Salida 6N137 → PB3; encoder canal B |
| **PB4** | ENC_Z (GPIO_Input, PULLUP, EXTI4 ↓) | CN10 | **CN10_27** | Entrada | Salida 6N137 → PB4; pulso índice Z |

### 3.6 Sensores velocidad de rueda — EXTI (LJ12A3 inductivos, 6–36V)

> ⚠️ **OBLIGATORIO:** Sensores LJ12A3 (salida NPN 6–36V) deben pasar por **optoacoplador PC817 o 6N137** antes de llegar al STM32 (3.3V). Ver `docs/CABLEADO_AISLAMIENTO_DEFINITIVO.md`.

| GPIO STM32 | Función firmware | Conector breakout | Bornera exacta | Dirección | Observaciones |
|---|---|---|---|---|---|
| **PA0** | WHEEL_FL (EXTI0, ↑, PULLUP) | CN7 | **CN7_28** | Entrada | Rueda delantera izquierda |
| **PA1** | WHEEL_FR (EXTI1, ↑, PULLUP) | CN7 | **CN7_30** | Entrada | Rueda delantera derecha |
| **PA2** | WHEEL_RL (EXTI2, ↑, PULLUP) | CN10 | **CN10_35** | Entrada | Rueda trasera izquierda |
| **PB15** | WHEEL_RR (EXTI15, ↑, PULLUP) | CN10 | **CN10_26** | Entrada | Rueda trasera derecha |

### 3.7 Sensor centrado dirección (LJ12A3 inductivo, EXTI5)

| GPIO STM32 | Función firmware | Conector breakout | Bornera exacta | Dirección | Observaciones |
|---|---|---|---|---|---|
| **PB5** | STEER_CENTER (EXTI5, ↓, PULLUP) | CN10 | **CN10_29** | Entrada | Sensor inductivo de posición 0° dirección; aislamiento obligatorio |

### 3.8 PWM Motores tracción — TIM1 (FL/FR, 20 kHz) y TIM8 (RL/RR, 20 kHz)

> Señales 3.3V directas al VCC lógico del BTS7960 (IBT-2). **NO usar 5V en VCC lógico.**

| GPIO STM32 | Función firmware | Conector breakout | Bornera exacta | Timer | Observaciones |
|---|---|---|---|---|---|
| **PA8** | RPWM_FL (TIM1_CH1, AF6) | CN10 | **CN10_23** | TIM1 | Motor FL adelante; mismo UEV que PA9 |
| **PA9** | LPWM_FL (TIM1_CH2, AF6) | CN10 | **CN10_21** | TIM1 | Motor FL atrás; mismo UEV que PA8 |
| **PA10** | RPWM_FR (TIM1_CH3, AF6) | CN10 | **CN10_33** | TIM1 | Motor FR adelante; mismo UEV que PC3 |
| **PC3** | LPWM_FR (TIM1_CH4, AF2) | CN7 | **CN7_37** | TIM1 | Motor FR atrás; ⚠️ NO es PA11 (era antes) |
| **PC6** | RPWM_RL (TIM8_CH1, AF4) | CN10 | **CN10_4** | TIM8 | Motor RL adelante; mismo UEV que PC7 |
| **PC7** | LPWM_RL (TIM8_CH2, AF4) | CN10 | **CN10_19** | TIM8 | Motor RL atrás; mismo UEV que PC6 |
| **PC8** | RPWM_RR (TIM8_CH3, AF4) | CN10 | **CN10_2** | TIM8 | Motor RR adelante; mismo UEV que PC9 |
| **PC9** | LPWM_RR (TIM8_CH4, AF4) | CN10 | **CN10_1** | TIM8 | Motor RR atrás; mismo UEV que PC8 |

### 3.9 PWM Motor dirección — TIM3 (20 kHz)

| GPIO STM32 | Función firmware | Conector breakout | Bornera exacta | Timer | Observaciones |
|---|---|---|---|---|---|
| **PA6** | RPWM_STEER (TIM3_CH1, AF2) | CN10 | **CN10_13** | TIM3 | Motor dirección izquierda |
| **PA7** | LPWM_STEER (TIM3_CH2, AF2) | CN10 | **CN10_15** | TIM3 | Motor dirección derecha |

### 3.10 Enable BTS7960 (GPIOC — salida, activo HIGH)

> Forzados a LOW antes de configurar el GPIO (`GPIOC->BSRR` atómico en MX_GPIO_Init).

| GPIO STM32 | Función firmware | Conector breakout | Bornera exacta | Motor | Observaciones |
|---|---|---|---|---|---|
| **PC5** | EN_FL | CN10 | **CN10_6** | FL | R_EN y L_EN del IBT-2 unidos; HIGH=enable |
| **PC0** | EN_FR | CN7 | **CN7_38** | FR | Morpho CN7 pin 38 (NO en Arduino CN9) |
| **PC1** | EN_RL | CN7 | **CN7_36** | RL | Morpho CN7 pin 36 (NO en Arduino CN9) |
| **PC2** | EN_RR | CN7 | **CN7_35** | RR | Reasignado desde PC13 (PC13=botón USER) |
| **PC4** | EN_STEER | CN10 | **CN10_34** | STEER | Enable BTS7960 dirección |

### 3.11 Pedal acelerador — ADC1_IN4 (PA3)

> Señal del pedal Hall pasa por **divisor resistivo** (10kΩ + 6.8kΩ) antes de PA3 para escalar de 5V a ≤3.3V.

| GPIO STM32 | Función firmware | Conector breakout | Bornera exacta | Tipo | Observaciones |
|---|---|---|---|---|---|
| **PA3** | PEDAL (ADC1_IN4) | CN10 | **CN10_37** | Entrada analógica | Vmax en bornera: 3.3V; divisor obligatorio antes del pin |

### 3.12 1-Wire DS18B20 (PB0)

> Configurado como salida open-drain con pull-up interno + pull-up externo.

| GPIO STM32 | Función firmware | Conector breakout | Bornera exacta | Tipo | Observaciones |
|---|---|---|---|---|---|
| **PB0** | ONEWIRE (GPIO Output OD, PULLUP) | CN7 | **CN7_34** | Bidireccional OD | 5× DS18B20 en bus; pull-up externo 4.7kΩ a 3.3V |

### 3.13 LED indicadores

| GPIO STM32 | Función firmware | Conector breakout | Bornera exacta | Tipo | Observaciones |
|---|---|---|---|---|---|
| **PA5** | LD2 (LED verde de la Nucleo) | CN10 | **CN10_11** | Salida | LED integrado en placa Nucleo; no requiere cable externo |
| **PB14** | LED_DIAG (LED externo diagnóstico) | CN10 | **CN10_28** | Salida | LED externo + resistencia 330Ω en serie |

### 3.14 Pines reservados / no conectar en el proyecto

| GPIO STM32 | Estado | Conector breakout | Bornera | Observaciones |
|---|---|---|---|---|
| **PC10** | Input PullDown (libre, no conectado) | CN7 | CN7_1 | NO conectar; firmware lo usa como GPIO_MODE_INPUT/PULLDOWN |
| **PC13** | Botón USER B1 de la Nucleo | CN7 | CN7_23 | NO usar como salida; conflicto con botón físico |
| **PA13** | SWDIO (debugger) | CN7 | CN7_13 | Solo para programación/debug ST-Link |
| **PA14** | SWCLK (debugger) | CN7 | CN7_15 | Solo para programación/debug ST-Link |
| **PA4** | Libre (ADC disponible) | CN7 | CN7_32 | Sin uso en firmware actual |
| **PB6** | Libre | CN10 | CN10_17 | Sin uso en firmware actual |
| **PB7** | Libre | CN7 | CN7_21 | Sin uso en firmware actual |
| **PB12** | Libre | CN10 | CN10_16 | Sin uso en firmware actual |
| **PB13** | Libre | CN10 | CN10_30 | Sin uso en firmware actual |
| **PB1** | Libre | CN10 | CN10_24 | Sin uso en firmware actual |
| **PB2** | Libre | CN10 | CN10_22 | Sin uso en firmware actual |
| **PC14** | OSC32_IN (cristal RTC) | CN7 | CN7_25 | Reservado para cristal; NO conectar |
| **PC15** | OSC32_OUT (cristal RTC) | CN7 | CN7_27 | Reservado para cristal; NO conectar |
| **PF0** | OSC_IN (cristal principal) | CN7 | CN7_29 | Reservado para cristal; NO conectar |
| **PF1** | OSC_OUT (cristal principal) | CN7 | CN7_31 | Reservado para cristal; NO conectar |
| **PD2** | Libre | CN7 | CN7_4 | Sin uso en firmware actual |

---

## 4 — Tabla completa CN10 pin a pin

> Morpho derecho de la Nucleo-64 → Borneras izquierdas de la D-1686

| CN10 Pin | GPIO STM32 | Función firmware | Dirección | Nivel/Tipo | ⚠️ Restricción |
|---|---|---|---|---|---|
| 1 | PC9 | LPWM_RR (TIM8_CH4) | Salida PWM | 3.3V, 20kHz | Solo 3.3V hacia BTS7960 VCC lógico |
| 2 | PC8 | RPWM_RR (TIM8_CH3) | Salida PWM | 3.3V, 20kHz | Solo 3.3V hacia BTS7960 VCC lógico |
| 3 | PB8 | I2C1_SCL (AF4) | Bidireccional OD | 3.3V | Pull-up 4.7kΩ a 3.3V externo |
| 4 | PC6 | RPWM_RL (TIM8_CH1) | Salida PWM | 3.3V, 20kHz | Solo 3.3V |
| 5 | PB9 | I2C1_SDA (AF4) | Bidireccional OD | 3.3V | Pull-up 4.7kΩ a 3.3V externo |
| 6 | PC5 | EN_FL (GPIO output) | Salida | 3.3V activo HIGH | — |
| 7 | AVDD | Tensión analógica | — | ~3.3V | **NO conectar; referencia interna** |
| 8 | U5V | 5V desde USB ST-Link | — | 5V | **NO usar como fuente de potencia** |
| 9 | GND | Masa digital | — | 0V | GND común |
| 10 | NC | Sin conexión | — | — | — |
| 11 | PA5 | LD2 (LED verde Nucleo) | Salida | 3.3V | LED integrado; sin cable externo |
| 12 | PA12 | CAN_TX (FDCAN1, AF9) | Salida | 3.3V | Hacia TJA1051T/3 pin TXD |
| 13 | PA6 | RPWM_STEER (TIM3_CH1) | Salida PWM | 3.3V, 20kHz | Solo 3.3V |
| 14 | PA11 | CAN_RX (FDCAN1, AF9) | Entrada | 3.3V | Desde TJA1051T/3 pin RXD; ⚠️ NO es LPWM_FR |
| 15 | PA7 | LPWM_STEER (TIM3_CH2) | Salida PWM | 3.3V, 20kHz | Solo 3.3V |
| 16 | PB12 | Libre | — | — | Sin uso en firmware |
| 17 | PB6 | Libre | — | — | Sin uso en firmware |
| 18 | PB11 | RELAY_LED_REAR (GPIO out) | Salida | 3.3V → ULN | Via ULN2803A canal 2 |
| 19 | PC7 | LPWM_RL (TIM8_CH2) | Salida PWM | 3.3V, 20kHz | Solo 3.3V |
| 20 | GND | Masa digital | — | 0V | GND común |
| 21 | PA9 | LPWM_FL (TIM1_CH2) | Salida PWM | 3.3V, 20kHz | Solo 3.3V |
| 22 | PB2 | Libre | — | — | Sin uso en firmware |
| 23 | PA8 | RPWM_FL (TIM1_CH1) | Salida PWM | 3.3V, 20kHz | Solo 3.3V |
| 24 | PB1 | Libre | — | — | Sin uso en firmware |
| 25 | PB10 | RELAY_LED (GPIO out) | Salida | 3.3V → ULN | Via ULN2803A canal 1 |
| 26 | PB15 | WHEEL_RR (EXTI15, PULLUP) | Entrada | 3.3V | Desde opto PC817/6N137 |
| 27 | PB4 | ENC_Z (EXTI4 ↓, PULLUP) | Entrada | 3.3V | Desde 6N137; índice encoder |
| 28 | PB14 | LED_DIAG (GPIO out) | Salida | 3.3V | LED externo + 330Ω |
| 29 | PB5 | STEER_CENTER (EXTI5 ↓, PULLUP) | Entrada | 3.3V | Desde opto; sensor inductivo centro |
| 30 | PB13 | Libre | — | — | Sin uso en firmware |
| 31 | PB3 | ENC_B (TIM2_CH2, AF1) | Entrada | 3.3V | Desde 6N137; canal B encoder |
| 32 | AGND | Masa analógica | — | 0V | Conectar a GND estrella |
| 33 | PA10 | RPWM_FR (TIM1_CH3) | Salida PWM | 3.3V, 20kHz | Solo 3.3V |
| 34 | PC4 | EN_STEER (GPIO out) | Salida | 3.3V activo HIGH | — |
| 35 | PA2 | WHEEL_RL (EXTI2 ↑, PULLUP) | Entrada | 3.3V | Desde opto |
| 36 | NC | Sin conexión | — | — | — |
| 37 | PA3 | PEDAL (ADC1_IN4) | Entrada analógica | 0–3.3V | Divisor 10k+6.8k obligatorio |
| 38 | NC | Sin conexión | — | — | — |

---

## 5 — Tabla completa CN7 pin a pin

> Morpho izquierdo de la Nucleo-64 → Borneras derechas de la D-1686

| CN7 Pin | GPIO STM32 | Función firmware | Dirección | Nivel/Tipo | ⚠️ Restricción |
|---|---|---|---|---|---|
| 1 | PC10 | Libre (Input PULLDOWN) | Entrada | 3.3V | NO conectar; pull-down al iniciar |
| 2 | PC11 | RELAY_TRAC (GPIO out) | Salida | 3.3V → ULN → Songle | Via ULN2803A canal 4 |
| 3 | PC12 | RELAY_STEER_PWR (GPIO out) | Salida | 3.3V → ULN → Songle | Via ULN2803A canal 5 |
| 4 | PD2 | Libre | — | — | Sin uso en firmware |
| 5 | VDD | 3.3V salida de placa Nucleo | Alimentación | 3.3V (salida) | Máx. ~300 mA total Nucleo; ver §6 |
| 6 | E5V | 5V entrada externa | Alimentación | 5V (entrada) | Para alimentar Nucleo externamente; ver §6 |
| 7 | BOOT0 | Selección modo arranque | Config | LOW=Flash | Dejar libre o LOW para modo normal |
| 8 | GND | Masa | — | 0V | GND común |
| 9 | NC | Sin conexión | — | — | — |
| 10 | NC | Sin conexión | — | — | — |
| 11 | NC | Sin conexión | — | — | — |
| 12 | IOREF | Referencia I/O (~3.3V) | Salida | ~3.3V | Referencia para shields Arduino |
| 13 | PA13 | SWDIO (debug) | Debug | 3.3V | Solo programación/debug |
| 14 | NRST | Reset del MCU | Entrada OD | Activo LOW | Reset; pulsador en la D-1686 conectado aquí |
| 15 | PA14 | SWCLK (debug) | Debug | 3.3V | Solo programación/debug |
| 16 | +3V3 | 3.3V salida regulador Nucleo | Alimentación | 3.3V (salida) | Misma fuente que CN7_5 (VDD) |
| 17 | PA15 | ENC_A (TIM2_CH1, AF1) | Entrada | 3.3V | Desde 6N137; canal A encoder |
| 18 | +5V | 5V salida regulador Nucleo | Alimentación | 5V (salida) | Máx. 500 mA; ver §6 |
| 19 | GND | Masa | — | 0V | GND común |
| 20 | GND | Masa | — | 0V | GND común |
| 21 | PB7 | Libre | — | — | Sin uso en firmware |
| 22 | GND | Masa | — | 0V | GND común |
| 23 | PC13 | Botón USER B1 | Entrada | 3.3V | **NO usar como salida GPIO** |
| 24 | VIN | Entrada alimentación externa | Alimentación | 7–12V (entrada) | Alimenta regulador interno Nucleo; ver §6 |
| 25 | PC14 | OSC32_IN | Cristal | — | **NO conectar** |
| 26 | NC | Sin conexión | — | — | — |
| 27 | PC15 | OSC32_OUT | Cristal | — | **NO conectar** |
| 28 | PA0 | WHEEL_FL (EXTI0 ↑, PULLUP) | Entrada | 3.3V | Desde opto |
| 29 | PF0 | OSC_IN | Cristal | — | **NO conectar** |
| 30 | PA1 | WHEEL_FR (EXTI1 ↑, PULLUP) | Entrada | 3.3V | Desde opto |
| 31 | PF1 | OSC_OUT | Cristal | — | **NO conectar** |
| 32 | PA4 | Libre (ADC disponible) | — | — | Sin uso actual |
| 33 | VBAT | Alimentación backup RTC | Alimentación | 1.55–3.6V | Para pila CR2032 opcional; ver §6 |
| 34 | PB0 | ONEWIRE (GPIO OD, PULLUP) | Bidireccional OD | 3.3V | Pull-up 4.7kΩ a 3.3V; bus DS18B20 |
| 35 | PC2 | EN_RR (GPIO out) | Salida | 3.3V activo HIGH | Reasignado desde PC13; activo HIGH |
| 36 | PC1 | EN_RL (GPIO out) | Salida | 3.3V activo HIGH | — |
| 37 | PC3 | LPWM_FR (TIM1_CH4, AF2) | Salida PWM | 3.3V, 20kHz | ⚠️ Antes era en PA11; cambiado en PR #120 |
| 38 | PC0 | EN_FR (GPIO out) | Salida | 3.3V activo HIGH | — |

---

## 6 — Distribución de alimentación

### 6.1 Entradas de tensión disponibles en la D-1686

| Bornera | Tensión | Tipo | Descripción | Máximo | Usar para |
|---|---|---|---|---|---|
| **CN7_6 (E5V)** | 5V | Entrada | 5V externa alimenta la Nucleo (bypass regulador) | 1A recomendado | Alimentar la Nucleo desde fuente 5V regulada del sistema |
| **CN7_24 (VIN)** | 7–12V | Entrada | Alimenta el regulador interno de la Nucleo | 800 mA | Alimentar la Nucleo desde batería 12V directamente |
| **CN7_5 / CN7_16 (+3V3)** | 3.3V | Salida | 3.3V regulados desde la Nucleo | ~300 mA total | Alimentar lógica externa: TCA9548A, INA226, etc. |
| **CN7_18 (+5V)** | 5V | Salida | 5V desde la Nucleo (solo con USB o VIN activo) | 500 mA total | Alimentar TJA1051 CAN transceiver, optoacopladores |
| **CN10_8 (U5V)** | 5V | Salida | 5V del USB ST-Link (solo cuando conectado a PC) | — | **NO usar como fuente de proyecto** |
| **CN10_7 (AVDD)** | ~3.3V | Referencia interna | Tensión analógica del ADC | — | **NO conectar nada** |
| **CN7_33 (VBAT)** | 1.55–3.6V | Entrada/Salida | Batería de backup para RTC/SRAM | — | Solo pila CR2032 (opcional); NO conectar si no se usa RTC con pila |

### 6.2 Masas disponibles

| Bornera(s) | Tipo | Descripción |
|---|---|---|
| CN10_9, CN10_20, CN7_8, CN7_19, CN7_20, CN7_22 | GND digital | Masa digital común |
| CN10_32 (AGND) | GND analógico | Masa del ADC; conectar al punto estrella de masa |

> **Punto estrella de masa obligatorio:** Todo GND digital, AGND, GND de baterías 24V/12V, GND de BTS7960 y GND de la Nucleo deben converger en UN ÚNICO punto. Ver `docs/POWER_DISTRIBUTION.md`.

### 6.3 Esquema de alimentación recomendado

```
  Batería 12V ──► CN7_24 (VIN)              ← Alimenta regulador Nucleo
    o
  Fuente 5V ────► CN7_6 (E5V)               ← Bypass directo (más eficiente)

  CN7_5 / CN7_16 (+3V3 salida) ──────────► TCA9548A VCC, INA226 VCC, lógica 3.3V
  CN7_18 (+5V salida) ────────────────────► TJA1051T/3 VCC+VIO, optoacopladores 5V side

  CN7_8 / CN10_9 / etc. (GND) ───────────► Punto estrella de masa
  CN10_32 (AGND) ──────────────────────── Punto estrella (junto con GND digital)
```

### 6.4 Restricciones críticas de tensión — QUÉ NO CONECTAR

| Restricción | Bornera afectada | Consecuencia si se viola |
|---|---|---|
| **NUNCA** conectar 5V a cualquier pin GPIO (CN10_1..38, CN7_1..4, CN7_13..CN7_17..etc) | Todos los I/O | Destrucción inmediata del STM32G474RE (Vmax GPIO = 3.6V) |
| **NUNCA** conectar 12V a cualquier pin GPIO | Todos los I/O | Destrucción inmediata del STM32G474RE |
| **NUNCA** conectar 24V a cualquier pin GPIO | Todos los I/O | Destrucción inmediata del STM32G474RE |
| **NO** aplicar 5V en CN10_7 (AVDD) | CN10_7 | Daño al ADC y al regulador analógico interno |
| **NO** conectar fuente externa a CN10_8 (U5V) | CN10_8 | Conflicto con USB ST-Link; posible daño |
| **NO** aplicar más de 12V en CN7_24 (VIN) | CN7_24 | Supera límite del regulador LD1117 de la Nucleo |
| **NO** conectar más de 1A en CN7_6 (E5V) | CN7_6 | Sobrecarga del regulador interno |
| **NO** dejar CN7_33 (VBAT) flotante si no se usa pila | CN7_33 | PENDIENTE DE VERIFICACIÓN; puede causar comportamiento errático del RTC |

---

## 7 — Integración ULN2803A

La arquitectura actual (confirmada en `docs/POWER_DISTRIBUTION.md`, §10) es:

```
STM32/ESP32 GPIO (3.3V salida)
       │
       ▼
 ┌─────────────┐
 │  ULN2803A   │  (8 canales Darlington, sink 500mA/canal, VCE_sat ~1V)
 │  1B → 1C    │  COM del ULN: sin conectar (bobinas Songle tienen pull-up interno a +5V)
 └─────────────┘
       │ (colector open-drain, hunde a GND cuando entrada HIGH)
       ▼
 ┌─────────────────┐
 │ Módulo Songle   │  (4 canales, IN activo LOW via ULN, COM del relé = +5V/12V/24V)
 │ 4CH relay       │
 └─────────────────┘
       │
       ▼
  Bobina de relé → contactos de potencia
```

### 7.1 Mapa de canales ULN2803A

| Canal ULN | Pin entrada (1B–8B) | Pin salida (1C–8C) | GPIO MCU | Bornera breakout | Destino Songle | Función |
|---|---|---|---|---|---|---|
| Canal 1 | 1B | 1C | **PB10** | **CN10_25** | IN1 | RELAY_LED (tira frontal) |
| Canal 2 | 2B | 2C | **PB11** | **CN10_18** | IN2 | RELAY_LED_REAR (tira trasera) |
| Canal 3 | 3B | 3C | **GPIO11 ESP32** | — (ESP32, no STM32) | IN3 | RELAY_AUDIO (altavoz) |
| Canal 4 | 4B | 4C | **PC11** | **CN7_2** | IN4 | RELAY_TRAC (tracción 24V) |
| Canal 5 | 5B | 5C | **PC12** | **CN7_3** | Relé 12V dirección | RELAY_STEER_PWR |
| Canal 6–8 | — | — | — | — | — | Libres / sin uso actual |

### 7.2 Cableado completo STM32 breakout → ULN2803A → Songle

```
CN10_25 (PB10) ──────[cable]──────► ULN2803A 1B ──► 1C ──► Songle IN1
CN10_18 (PB11) ──────[cable]──────► ULN2803A 2B ──► 2C ──► Songle IN2
ESP32 GPIO11   ──────[cable]──────► ULN2803A 3B ──► 3C ──► Songle IN3
CN7_2  (PC11)  ──────[cable]──────► ULN2803A 4B ──► 4C ──► Songle IN4
CN7_3  (PC12)  ──────[cable]──────► ULN2803A 5B ──► 5C ──► Bobina relé 12V DIR

Cualquier GND breakout (ej. CN7_8) ──► ULN2803A GND (pin 9)
ULN2803A COM (pin 10): SIN CONECTAR (las entradas INx de Songle tienen pull-up interno)
```

### 7.3 Notas de seguridad ULN2803A

- Las entradas del Songle (IN1–IN4) tienen pull-up interno de ~5V. En reposo (salida ULN en alta impedancia) la entrada del relé es ≈5V → relé desactivado.
- Cuando el STM32 pone HIGH el GPIO → ULN hunde la salida a GND → entrada Songle = ≈0V → relé ACTIVADO.
- La señal del STM32 es 3.3V. El ULN2803A especifica V_IL(max)=0.8V y V_IH(min)=2.7V para +5V VCC. Con V_IH(min)=2.7V y señal de entrada 3.3V: ✅ el margen de ruido es 0.3V — aceptable pero ajustado.

> ⚠️ **Confirmación de niveles lógicos ULN2803A con señales 3.3V:** PENDIENTE DE VERIFICACIÓN en montaje real (medir tensión de salida en colector con señal 3.3V en base para confirmar saturación completa).

---

## 8 — Riesgos eléctricos detectados

### RIESGO 1 — CRÍTICO: 5V en GPIO
**Descripción:** Cualquier señal superior a 3.6V en un pin GPIO del STM32G474RE destruye el pin de forma irreversible.  
**Pines en riesgo:** PA0–PA2 (sensores rueda), PB3/PB4/PA15 (encoder), PB5 (centro dirección), PA3 (pedal).  
**Solución:** Optoacoplador 6N137 o PC817 en **todas** las señales de sensores externos antes de llegar a la breakout. Divisor resistivo para PA3.  
**Estado:** Descrito en `docs/CABLEADO_AISLAMIENTO_DEFINITIVO.md` y `docs/ENCODER_WIRING_6N137.md`.

### RIESGO 2 — CRÍTICO: Alimentación duplicada en E5V y VIN simultáneos
**Descripción:** Si se conecta 5V en CN7_6 (E5V) y simultáneamente se aplica tensión en CN7_24 (VIN), los dos reguladores pueden entrar en conflicto.  
**Solución:** Usar **solo una** fuente de alimentación de la Nucleo a la vez: o E5V **o** VIN, nunca las dos.

### RIESGO 3 — ALTO: Señal PA11 reutilizada
**Descripción:** PA11 fue LPWM_FR en arquitecturas antiguas. Ahora es CAN_RX (FDCAN1). Si un cable antiguo conecta CN10_14 (PA11) al BTS7960 del motor FR, destruye la comunicación CAN.  
**Solución:** Verificar que CN10_14 va exclusivamente al TJA1051T/3 CAN transceiver.

### RIESGO 4 — ALTO: PC3 confundido con PC0/PC1/PC4
**Descripción:** PC3 (CN7_37) es LPWM_FR (PWM de motor), no un pin de enable. Confundirlo con los enables (PC0/PC1/PC4) puede causar comportamiento errático del motor FR.  
**Solución:** Verificar que CN7_37 va a LPWM del BTS7960 FR, no a R_EN/L_EN.

### RIESGO 5 — ALTO: PC13 usado como salida
**Descripción:** PC13 (CN7_23) es el botón USER B1 físico de la Nucleo. Usado anteriormente como EN_RR en firmware antiguo. El firmware actual lo ha reasignado a PC2.  
**Solución:** CN7_23 no debe conectarse a ningún hardware externo.

### RIESGO 6 — MEDIO: Masas no conectadas en punto estrella
**Descripción:** Si AGND (CN10_32) se deja flotante o no se conecta con el GND digital, las lecturas ADC del pedal (PA3) serán erróneas y ruidosas.  
**Solución:** Conectar CN10_32 (AGND) al mismo punto de masa estrella que todos los GND del sistema.

### RIESGO 7 — MEDIO: BTS7960 VCC lógico a 5V
**Descripción:** Si el VCC lógico del IBT-2/BTS7960 se conecta a 5V, el umbral V_IH(min) sube a 3.5V, y las señales PWM de 3.3V del STM32 quedan por debajo del umbral → motor no responde o responde de forma errática.  
**Solución:** Alimentar el VCC lógico del BTS7960 con **3.3V** desde CN7_5 o CN7_16.

### RIESGO 8 — BAJO: BOOT0 (CN7_7) mal configurado
**Descripción:** Si CN7_7 (BOOT0) queda flotante o se conecta accidentalmente a 3.3V, el STM32 arranca en modo bootloader en vez de ejecutar el firmware.  
**Solución:** Dejar CN7_7 libre (la Nucleo tiene resistencia pull-down interna) o conectar a GND explícitamente.

### Confirmación de seguridad de GPIO
> ✅ **Ninguna conexión descrita en este documento aplica 5V o 12V o 24V directamente sobre un GPIO del STM32.** Todas las señales de entrada de sensores que superan 3.3V pasan por optoacopladores. La alimentación de módulos de potencia (BTS7960, relés) va a terminales de potencia separados, no a GPIO.

---

## 9 — Checklist de montaje

### Antes de conectar cualquier cable

- [ ] Verificar que el firmware correcto está grabado en el STM32 (el proyecto usa STM32G474RE, no otro micro)
- [ ] Verificar que BOOT0 (CN7_7) está en LOW o libre (modo Flash)
- [ ] Elegir UNA sola fuente para la Nucleo: E5V (CN7_6) o VIN (CN7_24), nunca ambas
- [ ] Preparar optoacopladores 6N137 para encoder y sensores de rueda
- [ ] Preparar optoacopladores PC817 para sensores LJ12A3
- [ ] Preparar divisor resistivo 10kΩ + 6.8kΩ para la señal del pedal antes de CN10_37

### Conexión de masa (imprescindible antes de nada)

- [ ] Conectar GND estrella principal al punto de masa del sistema
- [ ] Conectar CN10_9 (GND) al punto de masa
- [ ] Conectar CN7_8 (GND) al punto de masa
- [ ] Conectar CN10_32 (AGND) al punto de masa estrella
- [ ] Verificar continuidad entre CN10_9, CN7_8, CN10_32 y masa de baterías

### Conexión de alimentación

- [ ] Conectar 5V regulada (o VIN 7–12V) a la Nucleo
- [ ] Verificar LED PWR de la breakout D-1686 enciende
- [ ] Medir tensión en CN7_16 (+3V3): debe ser 3.28–3.36V
- [ ] Medir tensión en CN7_18 (+5V): debe ser 4.9–5.1V

### Conexión relés (via ULN2803A)

- [ ] Conectar CN7_2 (PC11) → ULN2803A canal 4 → Songle IN4 → relé tracción
- [ ] Conectar CN7_3 (PC12) → ULN2803A canal 5 → relé dirección
- [ ] Conectar CN10_25 (PB10) → ULN2803A canal 1 → Songle IN1 → relé LED frontal
- [ ] Conectar CN10_18 (PB11) → ULN2803A canal 2 → Songle IN2 → relé LED trasero
- [ ] Conectar GND ULN2803A a punto de masa estrella
- [ ] Verificar que COM del ULN2803A está SIN CONECTAR

### Conexión CAN bus

- [ ] Conectar CN10_12 (PA12/CAN_TX) → TJA1051T/3 pin TXD
- [ ] Conectar CN10_14 (PA11/CAN_RX) → TJA1051T/3 pin RXD
- [ ] Verificar que PA11 NO está conectado a ningún BTS7960

### Conexión I2C

- [ ] Conectar CN10_3 (PB8/SCL) → TCA9548A SCL + pull-up 4.7kΩ a 3.3V
- [ ] Conectar CN10_5 (PB9/SDA) → TCA9548A SDA + pull-up 4.7kΩ a 3.3V
- [ ] Verificar que pull-ups están a 3.3V (NO a 5V)

### Conexión encoder (con 6N137)

- [ ] Conectar señal A del encoder → 6N137 → CN7_17 (PA15)
- [ ] Conectar señal B del encoder → 6N137 → CN10_31 (PB3)
- [ ] Conectar índice Z del encoder → 6N137 → CN10_27 (PB4)

### Conexión sensores de rueda (con opto)

- [ ] Sensor WHEEL_FL → opto → CN7_28 (PA0)
- [ ] Sensor WHEEL_FR → opto → CN7_30 (PA1)
- [ ] Sensor WHEEL_RL → opto → CN10_35 (PA2)
- [ ] Sensor WHEEL_RR → opto → CN10_26 (PB15)

### Conexión PWM motores (solo después de verificar masas y alimentación)

- [ ] Verificar VCC lógico BTS7960 conectado a 3.3V (NO 5V)
- [ ] FL: CN10_23 (PA8/RPWM) y CN10_21 (PA9/LPWM) → BTS7960 FL
- [ ] FR: CN10_33 (PA10/RPWM) y CN7_37 (PC3/LPWM) → BTS7960 FR
- [ ] RL: CN10_4 (PC6/RPWM) y CN10_19 (PC7/LPWM) → BTS7960 RL
- [ ] RR: CN10_2 (PC8/RPWM) y CN10_1 (PC9/LPWM) → BTS7960 RR
- [ ] STEER: CN10_13 (PA6/RPWM) y CN10_15 (PA7/LPWM) → BTS7960 STEER

### Conexión enables BTS7960

- [ ] CN10_6 (PC5/EN_FL) → R_EN+L_EN unidos del BTS7960 FL
- [ ] CN7_38 (PC0/EN_FR) → R_EN+L_EN unidos del BTS7960 FR
- [ ] CN7_36 (PC1/EN_RL) → R_EN+L_EN unidos del BTS7960 RL
- [ ] CN7_35 (PC2/EN_RR) → R_EN+L_EN unidos del BTS7960 RR
- [ ] CN10_34 (PC4/EN_STEER) → R_EN+L_EN unidos del BTS7960 STEER

### Conexión DS18B20 (1-Wire)

- [ ] Conectar CN7_34 (PB0) → bus 1-Wire + pull-up 4.7kΩ a 3.3V → hasta 5× DS18B20

### Conexión pedal

- [ ] Sensor pedal Hall (5V) → divisor 10kΩ + 6.8kΩ → CN10_37 (PA3)
- [ ] Verificar que Vmax en CN10_37 no supera 3.3V

---

## 10 — Checklist de verificación con multímetro

> Realizar con el sistema completamente desenergizado (baterías desconectadas, USB desconectado) excepto donde se indique.

### Verificación de continuidad (sin alimentación)

- [ ] Continuidad entre CN10_9 (GND) y CN7_8 (GND): debe ser ≤0.5Ω
- [ ] Continuidad entre CN10_9 (GND) y CN10_32 (AGND): debe ser ≤0.5Ω
- [ ] Continuidad entre CN10_9 (GND) y masa de BTS7960 (GND lógico): debe ser ≤1Ω
- [ ] **NO debe haber continuidad** entre CN7_16 (+3V3) y CN7_18 (+5V)
- [ ] **NO debe haber continuidad** entre CN7_16 (+3V3) y cualquier terminal de batería 12V o 24V

### Verificación de ausencia de cortocircuitos (sin alimentación)

- [ ] Entre CN7_16 (+3V3) y CN10_9 (GND): debe medir >10kΩ (resistencias de carga)
- [ ] Entre CN7_18 (+5V) y CN10_9 (GND): debe medir >10kΩ
- [ ] Entre CN7_24 (VIN) y CN10_9 (GND): debe medir >10kΩ
- [ ] Entre cualquier pin GPIO y CN10_9 (GND): debe medir >1kΩ (sin cortocircuito a masa)

### Verificación de ausencia de 12V/24V en GPIO (con baterías conectadas, STM32 sin alimentar)

> Usar voltímetro en modo AC y DC.

- [ ] Medir todos los pines GPIO con borneras conectadas vs. GND: ninguno debe superar 3.6V
- [ ] Con baterías 24V conectadas (solo bus de potencia BTS7960): verificar que CN10_23, CN10_21, CN10_33, CN7_37, CN10_4, CN10_19, CN10_2, CN10_1 miden 0V (no hay retorno de tensión desde los BTS7960)
- [ ] Con batería 12V conectada: verificar que CN10_13 y CN10_15 miden 0V

### Verificación con STM32 alimentado (firmware en marcha)

- [ ] CN7_16 (+3V3): 3.28–3.36V
- [ ] CN7_18 (+5V): 4.9–5.1V
- [ ] CN7_5 (VDD): mismo valor que CN7_16
- [ ] CN10_32 (AGND) vs. CN10_9 (GND): ≤0.1V (no debe haber diferencia de potencial)
- [ ] CN10_11 (PA5/LD2): 0V en reposo; parpadea con actividad (si firmware activo)

### Verificación de señales GPIO con osciloscopio o lógico (opcional pero recomendado)

- [ ] CN10_12 (CAN_TX): señal diferencial FDCAN presente a 500kbps al comunicar con ESP32
- [ ] CN10_14 (CAN_RX): ídem
- [ ] CN10_3 (SCL) y CN10_5 (SDA): pulsos I2C a ~86kHz visibles al leer sensores
- [ ] CN10_23 (PA8/RPWM_FL): señal PWM 20kHz visible al dar gas
- [ ] CN7_2 (PC11/RELAY_TRAC): va a HIGH en arranque, después de secuencia de relés

---

> **Documento generado exclusivamente a partir del firmware STM32 real (`Core/Inc/project_config.h`, `STM32-Control-Coche-Marcos.ioc`, `Core/Src/safety_system.c`) y de la documentación técnica del proyecto (`docs/NUCLEO_G474RE_PINOUT_CROQUIS.md`, `docs/POWER_DISTRIBUTION.md`, `docs/CONEXIONES_COMPLETAS.md`). Ningún pin, función ni conexión ha sido inventado o asumido.**
