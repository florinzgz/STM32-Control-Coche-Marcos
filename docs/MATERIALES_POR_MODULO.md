# MATERIALES POR MÓDULO Y CONEXIÓN

**Documento:** Guía completa de materiales necesarios por módulo y por tipo de conexión  
**Proyecto:** STM32G474RE — Control de vehículo eléctrico  
**Fecha:** 2026-03-24  
**Referencia firmware:** `Core/Inc/project_config.h`, `Core/Inc/main.h`, `Core/Src/main.c`  
**Documentos relacionados:** `CONEXIONES_COMPLETAS.md`, `HARDWARE_WIRING_MANUAL.md`

---

## Índice

1. [STM32G474RE Nucleo-64 (MCU)](#1-stm32g474re-nucleo-64-mcu)
2. [Motores de tracción — 4× BTS7960 + Motor 24V](#2-motores-de-tracción--4-bts7960--motor-24v)
3. [Motor de dirección — 1× BTS7960 + Motor 12V](#3-motor-de-dirección--1-bts7960--motor-12v)
4. [Bus CAN — STM32 ↔ ESP32](#4-bus-can--stm32--esp32)
5. [Sensores de corriente — 6× INA226 vía TCA9548A](#5-sensores-de-corriente--6-ina226-vía-tca9548a)
6. [Sensores de velocidad de rueda — 4× LJ12A3](#6-sensores-de-velocidad-de-rueda--4-lj12a3)
7. [Encoder de dirección — E6B2-CWZ6C](#7-encoder-de-dirección--e6b2-cwz6c)
8. [Sensor de centrado — LJ12A3 Inductivo](#8-sensor-de-centrado--lj12a3-inductivo)
9. [Pedal acelerador — Hall SS1324LUA-T](#9-pedal-acelerador--hall-ss1324lua-t)
10. [Sensores de temperatura — 5× DS18B20 (OneWire)](#10-sensores-de-temperatura--5-ds18b20-onewire)
11. [Relés de potencia — 5 módulos](#11-relés-de-potencia--5-módulos)
12. [ESP32-S3 DevKitC-1 (HMI)](#12-esp32-s3-devkitc-1-hmi)
13. [Display TFT ST7796 480×320](#13-display-tft-st7796-480320)
14. [Sensor de obstáculos TOFSense-M S (ESP32)](#14-sensor-de-obstáculos-tofsense-m-s-esp32)
15. [Tiras LED WS2812B (frontal y trasera)](#15-tiras-led-ws2812b-frontal-y-trasera)
16. [Palanca de cambios — MCP23017 (ESP32)](#16-palanca-de-cambios--mcp23017-esp32)
17. [Audio — DFPlayer Mini (ESP32)](#17-audio--dfplayer-mini-esp32)
18. [Distribución de alimentación y masas](#18-distribución-de-alimentación-y-masas)
19. [Resumen total de materiales (BOM completa)](#19-resumen-total-de-materiales-bom-completa)

---

## 1. STM32G474RE Nucleo-64 (MCU)

**Función:** Controlador de tiempo real (PWM motores, lectura sensores, CAN, seguridad, watchdog).

### Materiales para el módulo

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | STM32G474RE Nucleo-64 | Board Nucleo-64 (NUCLEO-G474RE) | Incluye programador ST-LINK integrado y regulador 3.3V |
| 1 | Cable USB Mini-B o Micro-B | Para programación y depuración serie | Solo para desarrollo; no para operación en vehículo |
| 2 | Condensador cerámico 100 nF / 16V | X7R / X5R, entre pin 3.3V y GND cerca del Nucleo | Desacoplo rápido del VDD; reduce reinicios por pico PWM |
| 1 | Condensador electrolítico 10 µF / 10V | Entre pin 3.3V y GND | Desacoplo lento (bulk); estabiliza demanda de corriente transitoria |

### Conexiones al exterior

| Señal | Pin STM32 | Destino | Cable mínimo |
|-------|-----------|---------|--------------|
| PA8 RPWM_FL | PA8 | BTS7960 FL RPWM | 24 AWG |
| PA9 LPWM_FL | PA9 | BTS7960 FL LPWM | 24 AWG |
| PA10 RPWM_FR | PA10 | BTS7960 FR RPWM | 24 AWG |
| PC3 LPWM_FR | PC3 | BTS7960 FR LPWM | 24 AWG |
| PC6 RPWM_RL | PC6 | BTS7960 RL RPWM | 24 AWG |
| PC7 LPWM_RL | PC7 | BTS7960 RL LPWM | 24 AWG |
| PC8 RPWM_RR | PC8 | BTS7960 RR RPWM | 24 AWG |
| PC9 LPWM_RR | PC9 | BTS7960 RR LPWM | 24 AWG |
| PA6 RPWM_STEER | PA6 | BTS7960 STEER RPWM | 24 AWG |
| PA7 LPWM_STEER | PA7 | BTS7960 STEER LPWM | 24 AWG |
| PC5 EN_FL | PC5 | BTS7960 FL R_EN + L_EN | 24 AWG |
| PC0 EN_FR | PC0 | BTS7960 FR R_EN + L_EN | 24 AWG |
| PC1 EN_RL | PC1 | BTS7960 RL R_EN + L_EN | 24 AWG |
| PC13 EN_RR | PC13 | BTS7960 RR R_EN + L_EN | 24 AWG |
| PC4 EN_STEER | PC4 | BTS7960 STEER R_EN + L_EN | 24 AWG |
| PC10/11/12 RELAY | PC10, PC11, PC12 | Módulos relé (opto) | 24 AWG |
| PB10/11 LED RELAY | PB10, PB11 | Módulos relé LED | 24 AWG |
| PA11/PA12 CAN | PA11, PA12 | Transceiver TJA1051T/3 | 24 AWG |
| PB6/PB7 I2C | PB6, PB7 | TCA9548A SCL/SDA | 24 AWG |
| PB0 OneWire | PB0 | Bus DS18B20 | 24 AWG |
| PA3 ADC Pedal | PA3 | Divisor resistivo pedal | 28 AWG apantallado |
| PA0–PA2 / PB15 Wheel | PA0, PA1, PA2, PB15 | Sensores inductivos rueda | 24 AWG |
| PA15 / PB3 Encoder | PA15, PB3 | 6N137 optoacopladores | 28 AWG apantallado |
| PB4 ENC_Z | PB4 | 6N137 optoacoplador índice Z | 28 AWG apantallado |
| PB5 STEER_CENTER | PB5 | Sensor inductivo centrado | 24 AWG |
| PB14 LED_DIAG | PB14 | LED diagnóstico (Morpho CN10 pin 28) | 24 AWG |

---

## 2. Motores de tracción — 4× BTS7960 + Motor 24V

**Función:** Tracción de las 4 ruedas con PWM 20 kHz bidireccional (RPWM/LPWM). Drivers BTS7960 (IBT-2) alimentados a 24V.

### ⚡ Condensadores de protección obligatorios (por cada BTS7960 de tracción)

> **Instalar ANTES de encender el sistema.** Sin estos condensadores los transitorios de 20 kHz pueden resetear el STM32 o dañar sensores.

| Qty | Componente | Valor / Especificación | Ubicación | Función |
|-----|-----------|----------------------|-----------|---------|
| 4 | Condensador electrolítico bulk | **470 µF / 35 V** (temperatura 105°C, baja ESR) | Entre **B+** y **GND** de cada BTS7960 (tracción), lo más cerca del driver | Reserva de energía; absorbe el inrush al arrancar el motor y las caídas bruscas de corriente |
| 4 | Condensador cerámico bypass potencia | **100 nF / 50 V** X7R (0805 o mayor) | Entre **B+** y **GND**, soldado directamente junto al IC BTS7960 | Filtra armónicos del PWM 20 kHz en el bus de alimentación de potencia |
| 4 | Condensador cerámico bypass lógica | **100 nF / 50 V** X7R (0805 o mayor) | Entre **VCC lógico** (pin VCC del módulo) y **GND lógico** | Desacoplo local del buffer 74HC244; evita glitches lógicos a 20 kHz |
| 4 | Condensador cerámico snubber motor | **100 nF / 50 V** X7R (0805 o mayor) | Entre terminales **M+** y **M-** del motor (fijar al conector del motor, NO en el BTS7960) | Suprime picos de contra-EMF del motor DC brushed; filtra ruido RF que se propaga por los cables del motor |

### Materiales para el cableado de señal (por cada BTS7960 de tracción)

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 2 | Cable señal (RPWM + LPWM) | 24 AWG, ~30–50 cm | Del STM32 al BTS7960 — NO debe correr paralelo a cables de potencia |
| 1 | Cable EN | 24 AWG | Desde pin EN del STM32 (PC0/PC1/PC4/PC5/PC13) al R_EN + L_EN del driver |

### Materiales para el cableado de potencia (por cada BTS7960 de tracción)

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | Fusible slow-blow | **30 A** (ajustar al motor real) | En el cable positivo 24V antes de la resistencia shunt INA226 |
| 1 | Portafusible | Para fusible de 30A (tamaño midi/maxi o blade) | |
| 1 | Resistencia shunt INA226 | **1.5 mΩ** ±1%, 5W, footprint 2512 | En el cable positivo B+, ANTES del BTS7960 (entre relé TRAC y B+ del driver); P_max=3.75W @ 50A → 5W da margen 1.33× |
| 1 | Cable de potencia B+ | **2.5 mm²** (14 AWG) mínimo | 24V desde relé de tracción hasta B+ del BTS7960 |
| 1 | Cable de potencia B- | **2.5 mm²** (14 AWG) mínimo | GND de potencia hasta B- del BTS7960 |
| 2 | Cable al motor | **1.5 mm²** (16 AWG) mínimo | M+ y M- del BTS7960 hasta los bornes del motor DC |
| 1 | TVS diodo B+ (opcional) | **SMBJ30A** (30V standoff) | En B+ del BTS7960 para suprimir transitorios de 24V; recomendado si hay inductancias largas |
| 1 | Diodo Schottky volante libre motor (opcional) | **SB560** o similar (5A, 60V mín.) | Entre M+ y M- del motor (en paralelo con el motor); el BTS7960 ya tiene diodos internos, este es protección extra |

### Materiales para la lógica del BTS7960 (por cada módulo)

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | Cable VCC lógico | 24 AWG | 3.3V del STM32 al pin VCC del módulo IBT-2 (**NO 5V**) |
| 1 | Cable GND lógico | 24 AWG | GND común al pin GND lógico del módulo |

> **⚠️ VCC lógico = 3.3V obligatorio.** El buffer 74HC244 del módulo IBT-2 tiene V_IH(min) = 0.7 × VCC. Con 5V → V_IH(min) = 3.5V (las señales 3.3V del STM32 están por debajo del umbral). Con 3.3V → V_IH(min) = 2.31V ✅.

---

## 3. Motor de dirección — 1× BTS7960 + Motor 12V

**Función:** Control de la dirección con PWM 20 kHz (TIM3, PA6/PA7). Motor DC 12V, alimentado vía relé DIR + shunt INA226 #5.

### ⚡ Condensadores de protección obligatorios (BTS7960 de dirección)

> **Especialmente crítico:** El encoder E6B2-CWZ6C está próximo al motor de dirección. Sin el condensador snubber en los terminales del motor, la contra-EMF puede corromper los pulsos del encoder → `SAFETY_ERROR_CENTERING`.

| Qty | Componente | Valor / Especificación | Ubicación | Función |
|-----|-----------|----------------------|-----------|---------|
| 1 | Condensador electrolítico bulk | **470 µF / 25 V** (105°C, baja ESR) | Entre **B+** (12V) y **GND** del BTS7960 STEER | Reserva de energía; absorbe inrush y oscilaciones al girar la dirección |
| 1 | Condensador cerámico bypass potencia | **100 nF / 50 V** X7R | Entre **B+** y **GND**, junto al IC BTS7960 STEER | Filtra PWM 20 kHz en el bus de 12V del motor de dirección |
| 1 | Condensador cerámico bypass lógica | **100 nF / 50 V** X7R | Entre **VCC lógico** y **GND lógico** del BTS7960 STEER | Desacoplo local del 74HC244 |
| 1 | Condensador cerámico snubber motor | **100 nF / 50 V** X7R | Entre terminales **M+** y **M-** del motor de dirección (fijar al conector del motor) | Suprime picos de contra-EMF; protege el encoder de dirección adyacente |

### Materiales para la conexión de señal

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 2 | Cable señal (RPWM + LPWM) | 24 AWG, ~30–50 cm | PA6 → RPWM, PA7 → LPWM del BTS7960 STEER |
| — | Puente 3.3V → R_EN + L_EN | 24 AWG, 5 cm | R_EN y L_EN del BTS7960 STEER siempre a 3.3V (no hay GPIO de enable) |
| 1 | Cable VCC lógico | 24 AWG | 3.3V al VCC del módulo |
| 1 | Cable GND lógico | 24 AWG | GND común |

### Materiales para la conexión de potencia

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | Fusible slow-blow | **15 A** (ajustar al motor de dirección real) | En el cable positivo 12V antes del shunt INA226 |
| 1 | Portafusible | Para fusible de 15A | |
| 1 | Resistencia shunt INA226 | **1.5 mΩ** ±1%, 5W, footprint 2512 | En el cable positivo B+, ANTES del BTS7960 STEER; P_max=3.75W @ 50A → 5W da margen 1.33× |
| 1 | Cable de potencia B+ | **1.5 mm²** (16 AWG) mínimo | 12V desde relé DIR hasta B+ del BTS7960 STEER |
| 1 | Cable de potencia B- | **1.5 mm²** (16 AWG) mínimo | GND de potencia |
| 2 | Cable al motor | **1.0 mm²** (18 AWG) mínimo | M+ y M- hasta los bornes del motor de dirección |

---

## 4. Bus CAN — STM32 ↔ ESP32

**Función:** Comunicación bidireccional a 500 kbps entre el STM32 (controlador) y el ESP32 (HMI). Protocolo FDCAN.

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 2 | Transceiver CAN | **TJA1051T/3** (3.3V compatible) | Uno por nodo (STM32 y ESP32) |
| 2 | Resistencia de terminación CAN | **120 Ω / ¼ W** | Una en cada extremo del bus (CANH↔CANL) |
| 2 | Condensador desacoplo VCC transceiver | **100 nF / 10V** X7R | Junto al pin VCC de cada TJA1051 |
| 1 | Condensador bulk VCC transceiver ESP32 | **10 µF / 10V** | En paralelo con el 100nF del lado ESP32 |
| 2 | Diodo TVS bus CAN | **PESD2CAN** (línea diferencial) | Uno por nodo, entre CANH y CANL; protección ESD del bus |
| 1 | Cable par trenzado apantallado | **22 AWG**, longitud ≤ 5 m | Par trenzado CANH + CANL; apantallar el cable |
| — | Cable drenaje (shield) | — | Conectar a GND solo en el lado STM32; deja el extremo ESP32 flotante |

---

## 5. Sensores de corriente — 6× INA226 vía TCA9548A

**Función:** Medir corriente (y tensión) de los 4 motores de tracción, motor de dirección, y batería. Bus I2C a 400 kHz.

### Materiales para el multiplexor TCA9548A

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | Módulo TCA9548A | Breakout board con pull-ups incluidos (Adafruit 2717 o compatible) | Dirección 0x70 (A0/A1/A2 a GND) |
| 2 | Resistencia pull-up I2C | **4.7 kΩ / ¼W** | En PB6 (SCL) y PB7 (SDA), a 3.3V; verificar si el módulo ya las incluye |
| 2 | Cable I2C | 24 AWG, ~10–20 cm | PB6 → SCL y PB7 → SDA del TCA9548A |

### Materiales por cada INA226 (×6 sensores)

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 6 | Módulo INA226 | Breakout board (Adafruit 4226 o compatible) | Todos a dirección 0x40 (A0/A1 a GND); se diferencian por el canal del TCA9548A |
| 5 | Resistencia shunt motores | **1.5 mΩ** ±1%, 5W, footprint 2512 | CH0–CH3 (motores tracción) y CH5 (motor dirección); shunt en el cable positivo ANTES del BTS7960; P_max=3.75W @ 50A → 5W da margen 1.33× |
| 1 | Resistencia shunt batería | **0.75 mΩ** ±1%, 10W, footprint 2512 | CH4 (batería 24V); shunt ANTES del relé MAIN (entre borne + batería y COM del relé); P_max=7.5W @ 100A → 10W da margen 1.33× |
| 6 | Cable I2C corto | 24 AWG | Canal CHx del TCA9548A a SDA/SCL de cada INA226 |

> **Posición del shunt de batería (CH4):** ANTES del relé MAIN, para que la lectura de tensión de batería esté disponible incluso con el relé abierto (sin alimentación a los motores). Si se coloca después del relé, el firmware detectará 0V como fallo crítico.

---

## 6. Sensores de velocidad de rueda — 4× LJ12A3

**Función:** Detectar la velocidad de cada rueda (6 pulsos/vuelta) para ABS, TCS y odometría.

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 4 | Sensor inductivo LJ12A3-4-Z/BX | NPN, NO, 6–36V, M12 | Uno por rueda; detecta los 6 tornillos/imanes del cubo |
| 4 | Optoacoplador PC817 | o similar (4–6 µs) | Para aislar la señal del sensor (operado a ≥5V) del pin 3.3V del STM32; la velocidad de PC817 es aceptable para señales de rueda (≤ 1 kHz aprox.) |
| 4 | Resistencia serie optoacoplador | **820 Ω / ¼W** | En serie con el LED del PC817; I_LED = (Vsensor – 1.2V) / R |
| 4 | Resistencia pull-up STM32 | Interna STM32 (~40 kΩ) | Activar pull-up interno en PA0, PA1, PA2, PB15; sin componente externo si el PC817 ya tiene pull-up en colector |
| 4 | Cable | 24 AWG, ≥ 50 cm | Señal del sensor al optoacoplador; cable de alimentación del sensor |

> **Nota:** Los sensores LJ12A3 alimentados a voltajes > 3.3V (habitualmente 5V–24V) **requieren adaptación de nivel** antes del GPIO del STM32. Usar PC817 para este propósito. El pull-up interno del STM32 es suficiente en el lado de salida del PC817.

---

## 7. Encoder de dirección — E6B2-CWZ6C

**Función:** Medir la posición angular de la dirección (cuadratura ×4, 4800 CPR). Interfaz TIM2.

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | Encoder E6B2-CWZ6C | 1200 PPR, 5V, **push-pull (voltage output)**, cable de 1–2 m | Omron o compatible |
| 3 | Optoacoplador 6N137 | DIP-8 o módulo breakout doble; uno por canal (A, B, Z) | Aislamiento galvánico 2500 V + conversión 5V→3.3V. Máx 10 Mbps — margen ×500 a 20 kHz |
| 3 | Resistencia serie LED | **330 Ω** / ¼W | R_IN para I_F ≈ 10.6 mA con encoder a 5V: (5V−1.5V)/330Ω |
| 3 | Resistencia pull-up salida | **4.7 kΩ** / ¼W | Pull-up Vo del 6N137 a +3.3V (lógico STM32) |
| 1 | Condensador electrolítico 10 µF | 10V | Bulk en línea 5V del encoder |
| 3 | Condensador cerámico 100 nF | X7R / 10V | Desacoplo VCC lado lógico de cada 6N137 (junto al CI) |
| 1 | Ferrita SMD | 100 Ω @ 100 MHz | En serie con +5V del encoder (p.ej. BLM18AG121SN1) — recomendable en entorno con motores |
| 1 | Cable apantallado | 28 AWG apantallado, ≥ 30 cm | Del 6N137 al STM32 (PA15, PB3, PB4); separado de los cables de potencia |
| 1 | Fuente 5V | Existente en el sistema | Alimentación del encoder y lado LED de los 6N137 |

> **⚠️ Crítico:** La señal del encoder es de 5V. Conectar 5V directamente a PA15/PB3 (pines 3.3V) destruye el STM32. Los **3× 6N137** proporcionan aislamiento galvánico (2500 V) y conversión 5V→3.3V simultáneamente, protegiéndolo también de picos inductivos del motor de dirección adyacente.
>
> **ℹ️ Señal invertida:** La salida del 6N137 es lógicamente invertida. Al invertirse A y B a la vez, la cuadratura se preserva; solo el sentido del conteo cambia. Si es necesario, intercambiar A↔B en el conector STM32.
>
> **Ver esquema completo en:** `docs/ENCODER_WIRING_6N137.md`

---

## 8. Sensor de centrado — LJ12A3 Inductivo

**Función:** Detectar la posición central de la cremallera de dirección para calibración automática al arranque (PB5, EXTI5).

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | Sensor inductivo LJ12A3 | NPN, NO, M12, cable 3 hilos | Detecta un tornillo metálico en la cremallera |
| 1 | Optoacoplador PC817 | o similar | Para aislar si el sensor opera a voltaje > 3.3V |
| 1 | Resistencia serie PC817 | **820 Ω / ¼W** | Limitar corriente del LED del PC817 |
| 1 | Cable | 24 AWG | Señal al PB5 del STM32 (pull-up interno activado) |

---

## 9. Pedal acelerador — Hall SS1324LUA-T

**Función:** Medir la posición del pedal de acelerador (ADC1_IN4, PA3). El firmware realiza doble muestreo + filtro EMA para plausibilidad.

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | Sensor Hall SS1324LUA-T | Lineal, 5V supply, salida 0.3–4.8V | Conectar VCC a 5V |
| 1 | Resistencia divisor R1 | **10 kΩ / ¼W, 1%** | En serie entre salida del sensor y PA3 |
| 1 | Resistencia divisor R2 | **6.8 kΩ / ¼W, 1%** | Entre el nodo medio y GND; escala 5V → 2.0V máximo en PA3 |
| 1 | Condensador filtro ADC | **100 nF / 16V** X7R | Entre PA3 y GND (junto al STM32); filtro paso bajo RC (fc ≈ 393 Hz con R = R1‖R2 = 4.05 kΩ) |
| 1 | Cable apantallado | 28 AWG apantallado | Del divisor al PA3 del STM32; alejado de cables de motor y PWM |

> **⚠️ Usar resistencias de 1% de tolerancia.** Las de 5% introducen hasta ±3% de error de calibración del pedal.
>
> **⚠️ El divisor debe estar físicamente cerca del pin PA3** (no en el pedal) para minimizar la captación de ruido PWM.

---

## 10. Sensores de temperatura — 5× DS18B20 (OneWire)

**Función:** Monitorizar temperatura de los 4 motores y del ambiente/dirección. Bus OneWire en PB0.

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 5 | DS18B20 | Versión cable (waterproof, TO-92); 3.3V compatible | Uno por motor DC (FL, FR, RL, RR) + 1 ambiente/dirección |
| 1 | Resistencia pull-up OneWire | **4.7 kΩ / ¼W** | Entre PB0 y 3.3V; **obligatoria** para el protocolo OneWire |
| — | Cable OneWire | 24 AWG, hasta varios metros | Todos los DS18B20 en paralelo en el mismo bus (PB0); longitud total ≤ 20 m recomendado |
| 5 | Cinta térmica / adhesivo | Cinta Kapton o brida de nylon | Para fijar el DS18B20 al cuerpo del motor |

---

## 11. Relés de potencia — 5 módulos

**Función:** Conmutar la alimentación de los circuitos de potencia (MAIN 24V, TRAC 24V, DIR 12V, LED 5V frontal, LED 5V trasero).

### Materiales por módulo de relé (×5)

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | Módulo 4-ch opto relé | SRD-12VDC-SL-C, 12V, trigger 3.3V, contacto 10A/30VDC | Módulo intermedio para MAIN (PC10), TRAC (PC11), DIR (PC12). CH4 disponible |
| 3 | Relé de potencia (bobina 12V) | Alta corriente: ≥50A (MAIN/TRAC), ≥20A (DIR) | Relés de potencia controlados por contactos del módulo intermedio |
| 2 | Módulo relé con optoacoplador | SRD-05VDC o similar, trigger 3.3V, contacto 10A | Para LED_F (PB10), LED_R (PB11) |
| 5 | Diodo flyback bobina | **1N4007** (1A, 1000V) | En paralelo con la bobina del relé, cátodo al polo positivo; verificar si el módulo ya lo incluye |
| 5 | Condensador snubber contacto | **100 nF / 250V** (polipropileno) | Entre COM y NO del relé; amortigua el arco al conmutar cargas inductivas |
| 5 | Resistencia snubber | **100 Ω / 0.5W** | En serie con el condensador snubber de contacto (red RC snubber) |
| 5 | Resistencia pull-down gate MOSFET | **10 kΩ / ¼W** | Entre gate y GND del MOSFET driver; mantiene el relé apagado durante el reset del STM32 |
| 1 | Condensador bus 24V (junto a relés) | **1000 µF / 35V** (electrolítico) | En el bus 24V cerca de los relés; absorbe el inrush al cerrar el relé de tracción |
| 1 | Condensador bus 12V (junto a relé DIR) | **470 µF / 25V** (electrolítico) | En el bus 12V cerca del relé DIR; absorbe inrush del motor de dirección |

> **⚠️ El diodo flyback es crítico.** Sin él, al abrir el relé se genera un pico de –100V que destruye el transistor del módulo driver.

---

## 12. ESP32-S3 DevKitC-1 (HMI)

**Función:** Interfaz hombre-máquina (pantalla, audio, gestión de energía, palanca de cambios, obstáculos).

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | ESP32-S3 DevKitC-1 | Variante N16R8 (16 MB Flash, 8 MB PSRAM) | GPIO disponibles: 0–21, 38–48 (evitar 26–37, 0, 3, 45, 46, 19, 20) |
| 2 | Condensador desacoplo 3.3V | **100 nF / 10V** X7R | Junto al pin 3.3V del módulo |
| 1 | Cable USB | USB-C | Para programación con PlatformIO |

---

## 13. Display TFT ST7796 480×320

**Función:** Interfaz gráfica del HMI (velocidad, temperatura, estado del sistema). Conectado al ESP32 vía SPI.

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | Display TFT ST7796 480×320 | Con XPT2046 touch, SPI, 3.3V | Pines: CS=GPIO10, RST=GPIO38, DC=GPIO39, MOSI=GPIO13, SCLK=GPIO14, MISO=GPIO12, BL=GPIO42, TOUCH_CS=GPIO21 |
| — | Cables Dupont | 28 AWG, M-F, varios colores | Del ESP32 al display |

---

## 14. Sensor de obstáculos TOFSense-M S (ESP32)

**Función:** LiDAR ToF 8×8 para detección de obstáculos. Conectado a ESP32 UART1 (GPIO18 RX).

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | TOFSense-M S (Nooploop) | LiDAR 8×8, UART 921600 bps, 5V, conector GH1.25 | Rango 20–4000 mm; requiere 5V en VCC |
| 1 | Resistencia divisor R1 | **1 kΩ / ¼W** | En serie entre TX del sensor y GPIO18 del ESP32; nivel 3.5–3.6V → 2.9V |
| 1 | Resistencia divisor R2 | **4.7 kΩ / ¼W** | Entre el nodo medio y GND; escala la señal UART al nivel 3.3V del ESP32 |
| 1 | Condensador desacoplo VCC sensor | **100 nF / 10V** X7R | Entre VCC (5V) y GND del sensor, junto al conector GH1.25 |
| 1 | Cable con conector GH1.25 | 4 pines, ≤ 50 cm | VCC (5V), GND, RX sensor (no conectar), TX sensor → divisor → GPIO18 |

---

## 15. Tiras LED WS2812B (frontal y trasera)

**Función:** Iluminación del vehículo, controladas por el ESP32. Alimentación conmutada por relés PB10/PB11 del STM32.

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | Tira LED WS2812B frontal | 28 LEDs, 5V, ~1.68A máximo (28×60mA) | Datos desde GPIO47 del ESP32 |
| 1 | Tira LED WS2812B trasera | 16 LEDs, 5V, ~0.96A máximo (16×60mA) | Datos desde GPIO48 del ESP32 |
| 2 | Condensador bulk tira LED | **1000 µF / 10V** electrolítico | Uno por tira, en el conector de 5V/GND de la tira; absorbe inrush al encender todos los LEDs |
| 2 | Resistencia serie datos WS2812B | **330 Ω / ¼W** | Entre GPIO47/GPIO48 del ESP32 y el DIN de la tira; reduce reflexiones y protege el GPIO |
| 1 | Fuente 5V | ≥ 3A (frontal) + 2A (trasero) = ≥ 5A total | Fuente dedicada para las tiras LED; separada de la lógica del ESP32 |

---

## 16. Palanca de cambios — MCP23017 (ESP32)

**Función:** Leer la posición de la palanca (P/R/N/D1/D2) y enviar el modo al STM32 vía CAN.

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | MCP23017 | Expansor I2C 16 GPIO, 3.3V, dirección 0x20 | Conectado a I2C del ESP32 (GPIO8 SDA, GPIO9 SCL) |
| 2 | Resistencia pull-up I2C | **4.7 kΩ / ¼W** | En GPIO8 y GPIO9 del ESP32 a 3.3V |
| 1 | Resistencia pull-up RESET | **10 kΩ / ¼W** | Entre pin RESET del MCP23017 y 3.3V; mantiene fuera de reset |
| 1 | Condensador bulk VDD | **10 µF / 16V electrolítico** | Entre VDD (pin 9) y GND (pin 10) del MCP23017, en paralelo con el 100 nF cerámico. Evita hundimientos del rail 3.3V por picos WiFi/BT del ESP32-S3 que causan lecturas I²C erróneas |
| 1 | Condensador bypass VDD | **100 nF cerámico** | En paralelo con el 10 µF, lo más cerca posible del pin VDD (pin 9) del MCP23017 |
| 5 | Conector/cable palanca | 1 común + 4 señal | GPA0=P, GPA1=D2, GPA2=D1, GPA3=R (active LOW con pull-up interno). N es implícito (sin contacto físico). Cable común a GND. |

---

## 17. Audio — DFPlayer Mini (ESP32)

**Función:** Reproducir sonidos del sistema (alertas, señales acústicas).

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | DFPlayer Mini | UART 9600 bps, 5V, entrada microSD | Conectado a UART2 del ESP32 (GPIO43 TX, GPIO44 RX) |
| 1 | Resistencia serie TX | **1 kΩ / ¼W** | Entre GPIO43 (ESP32 TX) y RX del DFPlayer; protege el GPIO |
| 1 | Altavoz | 4–8 Ω, ≥ 2W | Conectado a SPK1 y SPK2 del DFPlayer |
| 1 | Relé de audio (opcional) | Activo LOW, GPIO11 ESP32 | Corta el altavoz cuando no hay audio; evita ruido de fondo |
| 1 | Tarjeta microSD | ≥ 1GB, clase 10 | Con los archivos MP3/WAV de sonidos del sistema |

---

## 18. Distribución de alimentación y masas

### Cables de alimentación principal

| Qty | Componente | Especificación | Notas |
|-----|-----------|---------------|-------|
| 1 | Batería 24V | ≥ 20Ah, ≥ 30A descarga continua | Batería principal del vehículo |
| 1 | Fusible principal | **60A** slow-blow (tipo maxi) | En el positivo de batería antes del relé MAIN |
| 1 | Portafusible tipo maxi | Para 60A | |
| 1 | MOSFET de protección antipolarity (P-MOSFET) | P-channel, ≥ 40V, ≥ 80A (p. ej. SQP100P06) | O diodo ideal; protege toda la electrónica si la batería se conecta al revés |
| 1 | Fuente 12V regulada | ≥ 5A | Para el motor de dirección y otros accesorios 12V |
| 1 | Fuente 5V regulada | ≥ 5A (total LED + sensores + ESP32) | Para lógica 5V, tiras LED, sensores 5V, DFPlayer |
| 1 | Regulador 3.3V | Incluido en el Nucleo-64 y ESP32 | Solo usar los reguladores de las placas; no añadir reguladores externos salvo necesidad específica |

### Bus de masas (GND)

| Elemento | Descripción |
|---------|-------------|
| Punto GND central | Barra de masa central (topología estrella) donde convergen todos los retornos |
| Retorno de potencia motores | Cable ≥ 4 mm² desde la batería negativa hasta el punto GND central |
| Retorno de potencia BTS7960 | Cable ≥ 2.5 mm² desde el B- de cada BTS7960 hasta la barra GND central |
| Retorno de señal (STM32, ESP32) | Cable 24 AWG; **NO compartir con los cables de retorno de potencia** |
| Punto de referencia único | Todos los módulos (STM32, ESP32, BTS7960, sensores) deben compartir exactamente un punto GND |

> **⚠️ Topología de masa en estrella obligatoria.** No daisy-chain (en cadena). Los lazos de tierra inducen corrientes de ruido que corrompen las señales CAN, I2C y ADC.

---

## 19. Resumen total de materiales (BOM completa)

### Componentes electrónicos y módulos

| Qty | Referencia | Componente | Especificación |
|-----|-----------|-----------|---------------|
| 1 | U1 | STM32G474RE Nucleo-64 | NUCLEO-G474RE |
| 1 | U2 | ESP32-S3 DevKitC-1 | Variante N16R8 |
| 5 | DRV1–DRV5 | Módulo driver BTS7960 (IBT-2) | 4 tracción + 1 dirección |
| 4 | M1–M4 | Motor DC 24V tracción | Brushed DC, 4 ruedas |
| 1 | M5 | Motor DC 12V dirección | Brushed DC, dirección |
| 1 | ENC1 | Encoder E6B2-CWZ6C | 1200 PPR, 5V, push-pull |
| 1 | PEDAL | Sensor Hall SS1324LUA-T | 5V supply, 0.3–4.8V output |
| 1 | MOD_RELAY | Módulo 4-ch opto relé SRD-12VDC-SL-C | 12V, 4 canales, trigger 3.3V (etapa 1 relés potencia) |
| 3 | REL1–REL3 | Relé de potencia (bobina 12V) | ≥50A MAIN, ≥50A TRAC, ≥20A DIR (etapa 2) |
| 2 | REL4–REL5 | Módulo relé SRD-05VDC o similar | 5V coil, LED_F (PB10), LED_R (PB11) |
| 6 | INA1–INA6 | Módulo INA226 | Breakout board (Adafruit 4226) |
| 1 | MUX1 | Módulo TCA9548A | Multiplexor I2C 8-canal |
| 5 | TEMP1–TEMP5 | DS18B20 | Waterproof, OneWire |
| 5 | WS1–WS5 | Sensor inductivo LJ12A3 | NPN, NO (4× velocidad rueda + 1× centrado) |
| 2 | CAN1–CAN2 | Transceiver CAN TJA1051T/3 | 3.3V compatible |
| 1 | DISP1 | Display TFT ST7796 480×320 | Con XPT2046 touch |
| 1 | TOF1 | TOFSense-M S (Nooploop) | LiDAR 8×8, 5V, UART 921600 bps |
| 1 | MCP1 | MCP23017 | Expansor I2C para palanca de cambios |
| 1 | DFP1 | DFPlayer Mini | Reproductor MP3/WAV |
| 1 | LED_F | Tira WS2812B frontal | 28 LEDs, 5V |
| 1 | LED_R | Tira WS2812B trasera | 16 LEDs, 5V |

### Componentes pasivos — Condensadores

| Qty | Referencia | Valor | Voltaje | Tipo | Ubicación |
|-----|-----------|-------|---------|------|-----------|
| 4 | C_BULK_TR | **470 µF** | 35V | Electrolítico 105°C | B+ de cada BTS7960 de tracción |
| 1 | C_BULK_DIR | **470 µF** | 25V | Electrolítico 105°C | B+ del BTS7960 de dirección |
| 5 | C_BP_POT | **100 nF** | 50V | Cerámico X7R 0805 | B+ junto al IC de cada BTS7960 |
| 5 | C_BP_LOG | **100 nF** | 50V | Cerámico X7R 0805 | VCC lógico de cada BTS7960 |
| 5 | C_SNB_MOT | **100 nF** | 50V | Cerámico X7R 0805 | Terminales M+/M- de cada motor |
| 2 | C_MCU_C | **100 nF** | 16V | Cerámico X7R 0805 | Alim. 3.3V del STM32 Nucleo |
| 1 | C_MCU_BULK | **10 µF** | 10V | Electrolítico | Alim. 3.3V del STM32 Nucleo |
| 5 | C_REL_SNB | **100 nF** | 250V | Polipropileno | Contactos COM–NO de cada relé |
| 2 | C_CAN | **100 nF** | 10V | Cerámico X7R | VCC de cada transceiver CAN |
| 1 | C_CAN_ESP | **10 µF** | 10V | Electrolítico | VCC del transceiver CAN ESP32 |
| 1 | C_ADC | **100 nF** | 16V | Cerámico X7R | PA3 (pedal ADC) a GND |
| 1 | C_24V_REL | **1000 µF** | 35V | Electrolítico | Bus 24V junto a relés |
| 1 | C_12V_REL | **470 µF** | 25V | Electrolítico | Bus 12V junto a relé DIR |
| 2 | C_LED | **1000 µF** | 10V | Electrolítico | Conector 5V de cada tira LED |
| 1 | C_TOF_VCC | **100 nF** | 10V | Cerámico X7R | VCC del TOFSense-M S |
| 1 | C_MCP_BULK | **10 µF** | 16V | Electrolítico | VDD (pin 9) del MCP23017, en paralelo con C_MCP_BP |
| 1 | C_MCP_BP | **100 nF** | 16V | Cerámico X7R | VDD (pin 9) del MCP23017, en paralelo con C_MCP_BULK |

### Componentes pasivos — Resistencias

| Qty | Referencia | Valor | Potencia | Uso |
|-----|-----------|-------|---------|-----|
| 1 | R_PED_R1 | **10 kΩ** ±1% | ¼W | Divisor pedal (R1) |
| 1 | R_PED_R2 | **6.8 kΩ** ±1% | ¼W | Divisor pedal (R2) |
| 2 | R_I2C | **4.7 kΩ** | ¼W | Pull-up I2C (PB6, PB7) |
| 2 | R_I2C_ESP | **4.7 kΩ** | ¼W | Pull-up I2C ESP32 (GPIO8, GPIO9 para MCP23017) |
| 1 | R_OW | **4.7 kΩ** | ¼W | Pull-up OneWire (PB0) |
| 3 | R_6N137_IF | **330 Ω** | ¼W | Resistencia serie LED del 6N137 encoder (canales A, B, Z) |
| 3 | R_6N137_PU | **4.7 kΩ** | ¼W | Pull-up salida Vo del 6N137 a +3.3V (canales A, B, Z) |
| 4 | R_PC817_IF | **820 Ω** | ¼W | Serie LED del PC817 (sensores velocidad + centrado) |
| 5 | R_MOSFET_PD | **10 kΩ** | ¼W | Pull-down gate MOSFET relé |
| 2 | R_CAN | **120 Ω** | ¼W | Terminación CAN (extremos del bus) |
| 5 | R_REL_SNB | **100 Ω** | ½W | Snubber RC de contactos de relé |
| 1 | R_TOF_R1 | **1 kΩ** | ¼W | Divisor UART TOFSense (R1 serie) |
| 1 | R_TOF_R2 | **4.7 kΩ** | ¼W | Divisor UART TOFSense (R2 a GND) |
| 1 | R_DFP | **1 kΩ** | ¼W | Serie TX DFPlayer |
| 2 | R_LED_DATA | **330 Ω** | ¼W | Serie datos WS2812B (GPIO47, GPIO48) |
| 1 | R_MCP_RST | **10 kΩ** | ¼W | Pull-up RESET del MCP23017 |
| 1 | R_MCP_SDA | Incluido en R_I2C_ESP | — | |

### Componentes de protección

| Qty | Referencia | Componente | Especificación |
|-----|-----------|-----------|---------------|
| 5 | D_FLY1–D_FLY5 | Diodo flyback relés | 1N4007 (1A, 1000V) |
| 5 | D_SCH1–D_SCH5 | Diodo Schottky volante motor (opcional) | SB560 (5A, 60V) |
| 2 | TVS_CAN | TVS diodo bus CAN | PESD2CAN |
| 5 | TVS_B+1–5 | TVS diodo B+ BTS7960 (opcional) | SMBJ30A (30V) |
| 1 | Q_ANTI | MOSFET protección antipolarity | P-channel ≥ 40V, ≥ 80A |
| 3 | U_6N137_ENC | Optoacoplador encoder | 6N137 (canales A, B, Z — aislamiento galvánico + 5V→3.3V) |
| 5 | OPT_WS1–5 | Optoacoplador sensores | PC817 |

### Resistencias shunt

| Qty | Referencia | Valor | Potencia | Canal INA226 | Motor |
|-----|-----------|-------|---------|--------------|-------|
| 5 | SH_M1–M5 | **1.5 mΩ** ±1% | 5W, 2512 | CH0–CH3, CH5 | FL, FR, RL, RR, STEER |
| 1 | SH_BAT | **0.75 mΩ** ±1% | 10W, 2512 | CH4 | Batería 24V |

### Fusibles

| Qty | Tipo | Valor | Ubicación |
|-----|------|-------|-----------|
| 1 | Maxi slow-blow | **60A** | Positivo batería → relé MAIN |
| 4 | Midi/blade slow-blow | **30A** | Positivo 24V → shunt INA226 → BTS7960 (uno por motor de tracción) |
| 1 | Blade slow-blow | **15A** | Positivo 12V → shunt INA226 → BTS7960 STEER |
| 1 | Blade slow-blow | **5A** | Línea 5V para electrónica y LEDs |

### Cables

| Tipo | Sección / AWG | Uso |
|------|--------------|-----|
| Potencia | **4 mm²** (12 AWG) | Batería → relé MAIN; positivo y negativo principales |
| Potencia motores | **2.5 mm²** (14 AWG) | De relés a shunts INA226 y a BTS7960 B+ |
| Potencia motor STEER | **1.5 mm²** (16 AWG) | 12V → BTS7960 STEER |
| Motor | **1.5 mm²** (16 AWG) | M+/M- de cada BTS7960 al motor |
| Señal general | **24 AWG** | PWM, GPIO, I2C, alimentación lógica |
| Señal sensible | **28 AWG apantallado** | Encoder A/B/Z, pedal ADC, UART TOFSense |
| CAN bus | **22 AWG par trenzado apantallado** | CANH + CANL, ≤ 5 m |

---

*Este documento está sincronizado con el firmware `Core/Inc/project_config.h` y las guías de conexión `docs/CONEXIONES_COMPLETAS.md` y `docs/HARDWARE_WIRING_MANUAL.md`. Ante cualquier discrepancia, el firmware y `project_config.h` son la fuente de verdad.*
