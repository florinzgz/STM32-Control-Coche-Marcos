# LISTADO COMPLETO DE PINES — ESP32-S3 y STM32G474RE

> ⚠ **ACTUALIZACIÓN relés (2026-04-23, CAN rev 1.3 compatible):** El hardware real solo tiene **un relé de 24 V (tracción, PC11)** y **un relé de 12 V (dirección, PC12)**. **NO existe un relé MAIN / Power-Hold** independiente. El pin **PC10** queda **DISPONIBLE / libre** (configurado como `GPIO_Input` + `Pull-down`, sin asignación en firmware). Ver `CAN_CONTRACT_FINAL.md` y `PROJECT_CHANGELOG.md`.

**Proyecto:** Control Coche Marcos  
**Fecha:** 2026-02-28  
**Fuente:** Firmware (`main.h`, `main.c`, `platformio.ini`, `User_Setup.h`) y documentación técnica del proyecto

---

## PARTE 1 — ESP32-S3-WROOM-1 (N16R8)

> HMI: Pantalla, audio, LEDs, CAN, sensores auxiliares  
> Alimentación: 3.3 V (interno) / 5 V (USB o regulador)

### 1.1 Pantalla TFT ST7796 (SPI — 480×320)

| GPIO | Señal | Conecta a | Dirección | Componentes externos |
|------|-------|-----------|-----------|----------------------|
| 10 | TFT_CS | Display pin CS | Salida | — |
| 12 | TFT_MISO | Display pin SDO + Touch T_DO (compartido) | Entrada | — |
| 13 | TFT_MOSI | Display pin SDI + Touch T_DIN (compartido) | Salida | — |
| 14 | TFT_SCLK | Display pin SCK + Touch T_CLK (compartido) | Salida | — |
| 38 | TFT_RST | Display pin RESET | Salida | — |
| 39 | TFT_DC | Display pin DC/RS (Data/Command) | Salida | — |
| 42 | TFT_BL | Display pin LED (retroiluminación) | Salida (PWM) | — |
| 21 | TOUCH_CS | Touch panel pin T_CS (XPT2046) | Salida | — |

**Componentes del módulo display:**
- Condensador cerámico 100 nF entre VCC y GND del display (desacoplo)

**Alimentación del display:** 3.3 V y GND directos desde ESP32-S3

---

### 1.2 CAN Bus (Transceiver TJA1051T/3)

| GPIO | Señal | Conecta a | Dirección | Componentes externos |
|------|-------|-----------|-----------|----------------------|
| 4 | CAN_TX | TJA1051T/3 pin 1 (TXD) | Salida | — |
| 5 | CAN_RX | TJA1051T/3 pin 4 (RXD) | Entrada | — |

**Componentes necesarios:**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| Resistencia de terminación | 120 Ω ¼W | Entre CANH y CANL del TJA1051T/3 | Terminación de bus CAN |
| Condensador de desacoplo | 100 nF cerámico | Entre VCC (pin 3) y GND (pin 2) del TJA1051T/3 | Filtrado de ruido |

**Conexiones del TJA1051T/3 (lado ESP32):**
- Pin 1 (TXD) ← GPIO4
- Pin 2 (GND) → GND
- Pin 3 (VCC) → **5 V** (4.5–5.5 V obligatorio)
- Pin 4 (RXD) → GPIO5
- Pin 5 (VIO) → **3.3 V** ⚠️ OBLIGATORIO — sin VIO, RXD = 5 V → destruye ESP32-S3
- Pin 6 (CANL) → Bus CAN bajo
- Pin 7 (CANH) → Bus CAN alto
- Pin 8 (S) → GND (modo normal)

---

### 1.3 Tiras LED WS2812B (FastLED)

| GPIO | Señal | Conecta a | Dirección | Componentes externos |
|------|-------|-----------|-----------|----------------------|
| 47 | LED_FRONT | Datos tira frontal (28 LEDs WS2812B) | Salida | Resistencia 330 Ω en serie en la línea de datos |
| 48 | LED_REAR | Datos tira trasera (16 LEDs WS2812B) | Salida | Resistencia 330 Ω en serie en la línea de datos |

**Componentes necesarios por tira:**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| Resistencia en serie (datos) | 330 Ω | Entre GPIO y DIN del primer LED | Protección contra reflejos de señal |
| Condensador bulk | 1000 µF / 6.3 V electrolítico | Entre VCC y GND de la tira | Estabilización alimentación |
| Condensador de desacoplo | 100 nF cerámico | Entre VCC y GND de cada LED (integrado en PCB WS2812B) | Ya incluido en módulo |

**Alimentación de tiras:** 5 V externa (no desde USB del ESP32)

---

### 1.4 Audio — DFPlayer Mini (UART2 — 9600 baud)

| GPIO | Señal | Conecta a | Dirección | Componentes externos |
|------|-------|-----------|-----------|----------------------|
| 43 | DFPLAYER_TX | DFPlayer pin RX | Salida | Resistencia 1 kΩ en serie (recomendado) |
| 44 | DFPLAYER_RX | DFPlayer pin TX | Entrada | — |
| 11 | AUDIO_RELAY | Relé de audio optoacoplado (activo LOW) | Salida | — |

**Componentes necesarios:**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| Resistencia en serie TX | 1 kΩ | Entre GPIO 43 y DFPlayer RX | Limitación de corriente (5 V tolerante) |
| Relé optoacoplado 2 canales | 5 V | GPIO 11 → IN del módulo relé | Aislamiento galvánico audio |

**Alimentación DFPlayer:** 5 V y GND

---

### 1.5 Sensor de Obstáculos — TOFSense-M (UART1 — 921600 baud)

| GPIO | Señal | Conecta a | Dirección | Componentes externos |
|------|-------|-----------|-----------|----------------------|
| 18 | OBSTACLE_RX | TOFSense-M pin TX (pin 4) | Entrada | Divisor de tensión si sensor sale 5 V (ver abajo) |

**Conexiones del sensor TOFSense-M:**
- Pin 1 (VCC) → 5 V
- Pin 2 (GND) → GND
- Pin 3 (RX) → No conectar (solo recepción)
- Pin 4 (TX) → GPIO 18

**Componentes necesarios si la salida del sensor es 5 V:**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| R1 (divisor) | 10 kΩ | Entre sensor TX y nodo medio | Divisor 5 V → 3.3 V |
| R2 (divisor) | 20 kΩ | Entre nodo medio y GND | Divisor 5 V → 3.3 V |

> Verificar nivel de salida del sensor antes de conectar. Si ya es 3.3 V, no hace falta divisor.

---

### 1.6 Shifter / Palanca de Cambios — MCP23017 (I2C — 0x20)

| GPIO | Señal | Conecta a | Dirección | Componentes externos |
|------|-------|-----------|-----------|----------------------|
| 8 | I2C_SDA | MCP23017 pin SDA | Bidireccional | Resistencia pull-up 4.7 kΩ a 3.3 V |
| 9 | I2C_SCL | MCP23017 pin SCL | Bidireccional | Resistencia pull-up 4.7 kΩ a 3.3 V |

**Conexiones del MCP23017:**
- VDD → 3.3 V
- VSS → GND
- A0, A1, A2 → GND (dirección 0x20)
- RESET → 3.3 V (activo, no en reset)
- Puerto A: Entradas del shifter (D1, D2, Reverse, Neutral, Park) — activo LOW con pull-ups internos habilitados

**Componentes necesarios:**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| Pull-up I2C SDA | 4.7 kΩ | De GPIO 8 a 3.3 V | Pull-up bus I2C (si no está en módulo) |
| Pull-up I2C SCL | 4.7 kΩ | De GPIO 9 a 3.3 V | Pull-up bus I2C (si no está en módulo) |
| Condensador de desacoplo | 100 nF cerámico | Entre VDD y VSS del MCP23017 | Filtrado |

---

### 1.7 Gestión de Energía

| GPIO | Señal | Conecta a | Dirección | Componentes externos |
|------|-------|-----------|-----------|----------------------|
| 40 | IGNITION_SENSE | Señal de llave de contacto | Entrada (pull-down) | Divisor de tensión si señal > 3.3 V |
| 41 | POWER_HOLD | Control de mantenimiento de alimentación | Salida (activo HIGH) | — |

**Componentes necesarios:**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| R1 (divisor ignición) | 10 kΩ | Entre señal de llave y GPIO 40 | Adaptar nivel si > 3.3 V |
| R2 (divisor ignición) | 10 kΩ | Entre GPIO 40 y GND | Pull-down + divisor |

---

### 1.8 Selector de Tracción (Interruptor DPDT)

| GPIO | Señal | Conecta a | Dirección | Componentes externos |
|------|-------|-----------|-----------|----------------------|
| 15 | TRACTION_SWITCH | Interruptor 2WD/4WD | Entrada (pull-up interno) | — |

**Lógica:**
- LOW → 4WD (interruptor conecta a GND)
- HIGH → 2WD (pull-up interno, contacto abierto)
- Debounce: 50 ms, 3 lecturas estables consecutivas

---

### 1.9 Resumen Visual — Todos los GPIO del ESP32-S3

| GPIO | Función | Protocolo | Dirección |
|------|---------|-----------|-----------|
| 4 | CAN TX | TWAI | Salida |
| 5 | CAN RX | TWAI | Entrada |
| 8 | I2C SDA (Shifter) | I2C | Bidireccional |
| 9 | I2C SCL (Shifter) | I2C | Bidireccional |
| 10 | Display CS | SPI | Salida |
| 11 | Relé Audio | GPIO | Salida (activo LOW) |
| 12 | Display MISO / Touch DO | SPI | Entrada |
| 13 | Display MOSI / Touch DIN | SPI | Salida |
| 14 | Display SCLK / Touch CLK | SPI | Salida |
| 15 | Selector tracción 2WD/4WD | GPIO | Entrada (pull-up) |
| 18 | Obstáculo sensor RX | UART1 | Entrada |
| 21 | Touch CS | SPI | Salida |
| 38 | Display RST | GPIO | Salida |
| 39 | Display DC | GPIO | Salida |
| 40 | Ignición sense | GPIO | Entrada (pull-down) |
| 41 | Power hold | GPIO | Salida |
| 42 | Display backlight | GPIO/PWM | Salida |
| 43 | DFPlayer TX | UART2 | Salida |
| 44 | DFPlayer RX | UART2 | Entrada |
| 47 | LED frontal (WS2812B) | GPIO | Salida |
| 48 | LED trasero (WS2812B) | GPIO | Salida |

**GPIO usados:** 21 de ~25 seguros disponibles

### 1.10 GPIO Prohibidos / Reservados — NO USAR

| GPIO | Motivo |
|------|--------|
| 0, 3 | Pines de strapping (modo boot) |
| 19, 20 | USB D−/D+ |
| 22–25 | No existen en ESP32-S3 |
| 26–32 | Bus QSPI Flash interno — **corrupción de boot** |
| 33–37 | Bus Octal PSRAM (módulo N16R8) — **crash** |
| 45, 46 | Pines de strapping (selección VDD_SPI) |

---
---

## PARTE 2 — STM32G474RE (LQFP64)

> Control de motores, sensores, relés, CAN  
> Alimentación: 3.3 V (VDD) / VDDA analógico  
> Clock: 170 MHz (HSE 8 MHz + PLL)

### 2.1 PWM Motores de Tracción — TIM1 (20 kHz, centro-alineado)

| Pin LQFP | GPIO | Timer | Señal | Conecta a | Componentes externos |
|----------|------|-------|-------|-----------|----------------------|
| 41 | PA8 | TIM1_CH1 | RPWM_FL | BTS7960 FL → pin RPWM | Optoacoplador HY-M158 + R 330 Ω en serie (LED del opto) |
| 42 | PA9 | TIM1_CH2 | LPWM_FL | BTS7960 FL → pin LPWM | Optoacoplador HY-M158 + R 330 Ω en serie |
| 43 | PA10 | TIM1_CH3 | RPWM_FR | BTS7960 FR → pin RPWM | Optoacoplador HY-M158 + R 330 Ω en serie |

> **Nota:** PA11 se reasignó a FDCAN1_RX (CAN bus). LPWM_FR ahora usa TIM1_CH4 en PC3 (AF2).
> PB14 queda libre y se configura como GPIO_Output para LED diagnóstico (LED_DIAG).

**Configuración TIM1:** Prescaler = 0, Period = 4249, Center-Aligned, BREAK2 armado a Cortex LOCKUP

---

### 2.2 PWM Motores de Tracción — TIM8 (20 kHz, centro-alineado)

| Pin LQFP | GPIO | Timer | Señal | Conecta a | Componentes externos |
|----------|------|-------|-------|-----------|----------------------|
| 37 | PC6 | TIM8_CH1 | RPWM_RL | BTS7960 RL → pin RPWM | Optoacoplador HY-M158 + R 330 Ω en serie |
| 38 | PC7 | TIM8_CH2 | LPWM_RL | BTS7960 RL → pin LPWM | Optoacoplador HY-M158 + R 330 Ω en serie |
| 39 | PC8 | TIM8_CH3 | RPWM_RR | BTS7960 RR → pin RPWM | Optoacoplador HY-M158 + R 330 Ω en serie |
| 40 | PC9 | TIM8_CH4 | LPWM_RR | BTS7960 RR → pin LPWM | Optoacoplador HY-M158 + R 330 Ω en serie |

**Configuración TIM8:** Igual que TIM1. BREAK2 armado a Cortex LOCKUP.

---

### 2.3 PWM Motor de Dirección — TIM3 (20 kHz, centro-alineado)

| Pin LQFP | GPIO | Timer | Señal | Conecta a | Componentes externos |
|----------|------|-------|-------|-----------|----------------------|
| 22 | PA6 | TIM3_CH1 | RPWM_STEER | BTS7960 STEER → pin RPWM | Optoacoplador HY-M158 + R 330 Ω en serie |
| 23 | PA7 | TIM3_CH2 | LPWM_STEER | BTS7960 STEER → pin LPWM | Optoacoplador HY-M158 + R 330 Ω en serie |

**Nota:** TIM3 no tiene BREAK hardware; protección por software (fault handler pone CCR a 0).

---

### 2.4 Cada BTS7960 — Componentes adicionales por driver

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| Optoacoplador HY-M158 | 2500 V RMS aislamiento | Entre GPIO STM32 y entrada PWM del BTS7960 | Aislamiento galvánico |
| Resistencia LED del opto | 330 Ω ¼W | En serie con LED interno del optoacoplador | Limitación de corriente |
| Diodo snubber | 1N4148 | Antiparalelo en motor | Protección contra picos inductivos |
| Condensador snubber | 100 nF / 50 V cerámico | En paralelo con motor | Absorción de picos |
| Resistencia snubber | 100 Ω ¼W | En serie con condensador snubber | Amortiguación |

---

### 2.5 Enable de Motores (GPIO Salida)

| Pin LQFP | GPIO | Señal | Conecta a | Componentes externos |
|----------|------|-------|-----------|----------------------|
| 35 | PC5 | EN_FL | BTS7960 FL → R_EN + L_EN (unidos) | — |
| 2 | PC13 | EN_RR | BTS7960 RR → R_EN + L_EN (unidos) | — |

**Motores FR, RL, STEER:** R_EN y L_EN conectados directamente a 3.3 V (siempre habilitados; los pines PC6, PC7, PC9 fueron reasignados a TIM8).

---

### 2.6 Relés de Potencia (GPIO Salida — Activo HIGH)

| Pin LQFP | GPIO | Señal | Conecta a | Componentes externos |
|----------|------|-------|-----------|----------------------|
| 52 | PC11 | RELAY_TRAC | Relé tracción (alimentación motores, 40 A) | Módulo 2-ch opto relé (etapa 1) → relé potencia bobina 12V (etapa 2) + Diodo 1N4007 |
| 53 | PC12 | RELAY_DIR | Relé dirección (alimentación dirección, 15 A) | Módulo 2-ch opto relé (etapa 1) → relé potencia bobina 12V (etapa 2) + Diodo 1N4007 |

> **PC10 está DISPONIBLE (INPUT_PULLDOWN, sin conexión)** — No se usa (anteriormente era RELAY_MAIN, simplificado a 2 relés)

**Arquitectura de dos etapas por cada relé de potencia:**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| Módulo 2-ch opto relé | Optoacoplador con relé | Entre GPIO STM32 y bobina del relé de potencia | Aislamiento galvánico + conmutación 12V para bobina |
| Diodo volante (flyback) | 1N4007 | Antiparalelo a la bobina del **relé de potencia** (etapa 2) | Protección contra picos inductivos al cortar la bobina |
| Fusible | 50 A / 20 A | En serie con contacto del relé | Protección de sobrecorriente |

---

### 2.7 Relés de LEDs (GPIO Salida — Activo HIGH)

| Pin LQFP | GPIO | Señal | Conecta a | Componentes externos |
|----------|------|-------|-----------|----------------------|
| 29 | PB10 | RELAY_LED | Relé alimentación tira LED frontal WS2812B | Diodo 1N4007 (volante) |
| 30 | PB11 | RELAY_LED_REAR | Relé alimentación tira LED trasera WS2812B | Diodo 1N4007 (volante) |

---

### 2.8 Encoder de Dirección — TIM2 (Cuadratura, E6B2-CWZ6C, 1200 PPR)

| Pin LQFP | GPIO | Timer/EXTI | Señal | Conecta a | Componentes externos |
|----------|------|------------|-------|-----------|----------------------|
| 50 | PA15 | TIM2_CH1 | ENC_A | 6N137 #1 Vo → Encoder A (cable negro) | 6N137 (aislamiento galvánico + 5V→3.3V) |
| 55 | PB3 | TIM2_CH2 | ENC_B | 6N137 #2 Vo → Encoder B (cable blanco) | 6N137 (aislamiento galvánico + 5V→3.3V) |
| 56 | PB4 | EXTI4 | ENC_Z | 6N137 #3 Vo → Encoder Z (cable naranja) | 6N137 (aislamiento galvánico + 5V→3.3V) |

**Componentes necesarios (3× 6N137):**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| 6N137 × 3 | DIP-8 o módulo breakout | Entre encoder y STM32 | Aislamiento galvánico 2500 V + conversión 5V push-pull → 3.3V (canales A, B, Z) |
| R_IN × 3 | 330 Ω / ¼W | Serie con LED de cada 6N137 | Limita I_F ≈ 10.6 mA con encoder a 5V |
| R_PU × 3 | 4.7 kΩ / ¼W | Vo de cada 6N137 a +3.3V | Pull-up salida open-collector del 6N137 |
| C (desacoplo) × 3 | 100 nF cerámico | Junto a cada 6N137, VCC–GND (lado 3.3V) | Estabilidad lado lógico |
| C (bulk encoder) | 10 µF electrolítico | Línea +5V del encoder | Filtro bulk |

> ⚠️ El encoder E6B2-CWZ6C opera a 5 V. Conectar 5 V directamente al STM32 puede dañar los pines. Usar **3× 6N137** como barrera galvánica y level-shifter (no divisores resistivos ni TXS0108E — no proporcionan aislamiento galvánico).
>
> ℹ️ La salida del 6N137 es **invertida** respecto a la entrada del encoder. Al invertirse A y B simultáneamente, la cuadratura se preserva. El sentido del conteo cambia: si es incorrecto, intercambiar A↔B en el conector STM32. Ver `docs/ENCODER_WIRING_6N137.md`.

**Alimentación encoder:**
- Cable marrón → +5 V (NO conectar a 3.3 V)
- Cable azul → GND (masa común)

---

### 2.9 Sensores de Velocidad de Rueda (EXTI — LJ12A3 Inductivo)

| Pin LQFP | GPIO | EXTI | Señal | Conecta a | Componentes externos |
|----------|------|------|-------|-----------|----------------------|
| 10 | PA0 | EXTI0 | WHEEL_FL | Sensor inductivo rueda delantera izquierda | Pull-up interno activado (3.3 V) |
| 11 | PA1 | EXTI1 | WHEEL_FR | Sensor inductivo rueda delantera derecha | Pull-up interno activado |
| 12 | PA2 | EXTI2 | WHEEL_RL | Sensor inductivo rueda trasera izquierda | Pull-up interno activado |
| 36 | PB15 | EXTI15 | WHEEL_RR | Sensor inductivo rueda trasera derecha | Pull-up interno activado |

**Configuración:** Flanco de subida (rising edge), pull-up interno, prioridad NVIC 2, debounce 1 ms en firmware.

**Cada sensor LJ12A3 necesita:**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| Alimentación | 5 V–24 V DC | Cable marrón (+) del sensor | Alimentación del sensor |
| GND | — | Cable azul/negro (−) del sensor → GND común | Referencia |

> Nota: Los sensores LJ12A3 con alimentación 5 V suelen tener salida NPN (open-collector), compatible con pull-up interno del STM32 a 3.3 V. Si la alimentación es > 5 V, verificar que la salida no supere 3.3 V.

---

### 2.10 Sensor de Centrado de Dirección (EXTI — LJ12A3 Inductivo)

| Pin LQFP | GPIO | EXTI | Señal | Conecta a | Componentes externos |
|----------|------|------|-------|-----------|----------------------|
| 57 | PB5 | EXTI5 | STEER_CENTER | Sensor inductivo posición central dirección | Pull-up interno activado |

**Función:** Detecta el punto central mecánico de la dirección para calibrar la posición cero del encoder al arrancar.

**Mismos componentes y notas que los sensores de velocidad (sección 2.9).**

---

### 2.11 Pedal Acelerador — Doble Canal Redundante

#### Canal Primario: ADC interno (PA3)

| Pin LQFP | GPIO | Periférico | Señal | Conecta a | Componentes externos |
|----------|------|------------|-------|-----------|----------------------|
| 13 | PA3 | ADC1_IN4 | PEDAL (primario) | Sensor Hall SS1324LUA-T (a través de divisor) | Divisor resistivo 5 V → 3.3 V |

**Componentes del divisor:**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| R1 | 10 kΩ (1% precisión) | Entre señal pedal (5 V) y nodo PA3 | Divisor de tensión |
| R2 | 6.8 kΩ (1% precisión) | Entre nodo PA3 y GND | Divisor de tensión |

**Cálculo:** Ratio = 6.8/(10+6.8) = 0.4048 → 5.0 V × 0.4048 = 2.02 V (máximo absoluto en PA3, bien por debajo de 3.3 V)

#### Plausibilidad por Software (sin ADS1115)

> **CAMBIO:** El ADS1115 ha sido eliminado. La plausibilidad se realiza por software usando el ADC interno del STM32 con doble lectura consecutiva, filtro EMA, validación de rango y límite de tasa de cambio.

---

### 2.12 Bus I2C — TCA9548A + 6× INA226

| Pin LQFP | GPIO | Periférico | Señal | Conecta a | Componentes externos |
|----------|------|------------|-------|-----------|----------------------|
| 58 | PB6 | I2C1_SCL | SCL | Bus I2C compartido (TCA9548A, INA226) | Resistencia pull-up 4.7 kΩ a 3.3 V |
| 59 | PB7 | I2C1_SDA | SDA | Bus I2C compartido | Resistencia pull-up 4.7 kΩ a 3.3 V |

**Configuración I2C:** 400 kHz (Fast Mode), Open-Drain, AF4

**Componentes necesarios:**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| Pull-up SCL | 4.7 kΩ | De PB6 a 3.3 V | Pull-up bus I2C |
| Pull-up SDA | 4.7 kΩ | De PB7 a 3.3 V | Pull-up bus I2C |
| Condensador de desacoplo (TCA9548A) | 100 nF cerámico | Entre VCC y GND | Filtrado |

**Dispositivos en el bus:**

| Dispositivo | Dirección I2C | Función |
|------------|---------------|---------|
| TCA9548A | 0x70 | Multiplexor I2C (8 canales) |
| INA226 ×6 | 0x40 (detrás del mux, canales 0–5) | Medición de corriente/tensión |

**Cada INA226 necesita:**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| Resistencia shunt | 1.5 mΩ (motores) / 0.75 mΩ (batería) | En serie con cable de potencia (+) | Medición de corriente |
| Condensador de desacoplo | 100 nF cerámico | Entre VCC y GND del INA226 | Filtrado |

---

### 2.13 Sensores de Temperatura — 5× DS18B20 (OneWire)

| Pin LQFP | GPIO | Señal | Conecta a | Componentes externos |
|----------|------|-------|-----------|----------------------|
| 18 | PB0 | ONEWIRE | Bus OneWire (5 sensores DS18B20 en paralelo) | Resistencia pull-up 4.7 kΩ a 3.3 V (OBLIGATORIA) |

**Componentes necesarios:**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| Pull-up OneWire | 4.7 kΩ | De PB0 a 3.3 V | Pull-up del bus OneWire |

**Sensores:**
1. DS18B20 Motor FL
2. DS18B20 Motor FR
3. DS18B20 Motor RL
4. DS18B20 Motor RR
5. DS18B20 Ambiente / Dirección

**Conexión por sensor:** Data (amarillo) → PB0 | VCC (rojo) → 3.3 V | GND (negro) → GND

---

### 2.14 CAN Bus — FDCAN1 (500 kbps)

| Pin LQFP | GPIO | Periférico | Señal | Conecta a | Componentes externos |
|----------|------|------------|-------|-----------|----------------------|
| 44 | PA11 | FDCAN1_RX | CAN RX | TJA1051T/3 #1 pin 4 (RXD) | — |
| 45 | PA12 | FDCAN1_TX | CAN TX | TJA1051T/3 #1 pin 1 (TXD) | — |

**Componentes del TJA1051T/3 lado STM32:**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| Resistencia de terminación | 120 Ω ¼W | Entre CANH (pin 7) y CANL (pin 6) | Terminación de bus CAN |
| Condensador de desacoplo | 100 nF cerámico | Entre VCC (pin 3) y GND (pin 2) | Filtrado de ruido |
| Protección ESD | TPD2E001 (opcional) | En CANH y CANL | Protección contra descargas |

**Conexiones del TJA1051T/3 #1 (lado STM32):**
- Pin 1 (TXD) ← PA12
- Pin 2 (GND) → GND
- Pin 3 (VCC) → **5 V** (4.5–5.5 V obligatorio)
- Pin 4 (RXD) → PA11
- Pin 5 (VIO) → **3.3 V** (nivel lógico del STM32)
- Pin 6 (CANL) → Bus CAN bajo
- Pin 7 (CANH) → Bus CAN alto
- Pin 8 (S) → GND (modo normal)

---

### 2.15 Alimentación y Reloj

| Pin LQFP | Función | Conecta a | Componentes externos |
|----------|---------|-----------|----------------------|
| 1 | VBAT | 3.3 V (backup) | Condensador 100 nF cerámico |
| 5 | PH0/OSC_IN | Cristal 8 MHz (entrada) | Cristal + 2× condensadores de carga (típico 20 pF) |
| 6 | PH1/OSC_OUT | Cristal 8 MHz (salida) | Cristal + condensador de carga |
| 7 | NRST | Reset (activo bajo) | Condensador 100 nF a GND + resistencia pull-up 10 kΩ a 3.3 V (opcional) |
| 8 | VSSA | GND analógico | Conectar a plano de GND |
| 9 | VDDA | 3.3 V analógico | Condensador tantalio 10 µF/6.3 V + cerámico 100 nF |
| 19 | VSS | GND digital | Conectar a plano de GND |
| 20 | VDD | 3.3 V digital | Condensador cerámico 100 nF + 4.7 µF |
| 32 | VSS | GND digital | Conectar a plano de GND |
| 33 | VDD | 3.3 V digital | Condensador cerámico 100 nF + 4.7 µF |
| 48 | VSS | GND digital | Conectar a plano de GND |
| 49 | VDD | 3.3 V digital | Condensador cerámico 100 nF + 4.7 µF |
| 64 | VSS | GND digital | Conectar a plano de GND |

**Condensadores de alimentación (resumen):**

| Componente | Valor | Ubicación | Propósito |
|-----------|-------|-----------|-----------|
| Bulk capacitor | 1000 µF / 25 V electrolítico | Entrada de alimentación principal | Estabilización general |
| Bypass caps | 100 nF cerámico × 4 | Uno junto a cada pin VDD | Desacoplo de alta frecuencia |
| Tantalio VDDA | 10 µF / 6.3 V | Junto al pin VDDA (pin 9) | Estabilidad ADC analógico |
| Condensadores cristal | 20 pF × 2 | Entre cada pin del cristal y GND | Carga del oscilador |

---

### 2.16 Debug / Programación (SWD)

| Pin LQFP | GPIO | Señal | Conecta a | Componentes externos |
|----------|------|-------|-----------|----------------------|
| 46 | PA13 | SWDIO | Programador ST-Link (datos) | — |
| 49 | PA14 | SWCLK | Programador ST-Link (reloj) | — |

---

### 2.17 Pines Liberados / Sin Uso

| Pin LQFP | GPIO | Estado original | Estado actual |
|----------|------|----------------|---------------|
| 15 | PC0 | DIR_FL | Libre — dejar desconectado o GPIO LOW |
| 16 | PC1 | DIR_FR | Libre — dejar desconectado o GPIO LOW |
| 17 | PC2 | DIR_RL | Libre — dejar desconectado o GPIO LOW |
| 18 | PC3 | LPWM_FR | TIM1_CH4 (AF2) — PWM motor FR reverso |
| 34 | PC4 | DIR_STEER | Libre — dejar desconectado o GPIO LOW |

> Estos pines fueron liberados por la migración a la arquitectura RPWM/LPWM (PR #120).

---

### 2.18 Resumen Visual — Todos los GPIO del STM32G474RE

| GPIO | Función | Periférico | Dirección |
|------|---------|------------|-----------|
| PA0 | Velocidad rueda FL | EXTI0 | Entrada (pull-up) |
| PA1 | Velocidad rueda FR | EXTI1 | Entrada (pull-up) |
| PA2 | Velocidad rueda RL | EXTI2 | Entrada (pull-up) |
| PA3 | Pedal acelerador | ADC1_IN4 | Entrada analógica |
| PA6 | RPWM dirección | TIM3_CH1 | Salida PWM |
| PA7 | LPWM dirección | TIM3_CH2 | Salida PWM |
| PA8 | RPWM motor FL | TIM1_CH1 | Salida PWM |
| PA9 | LPWM motor FL | TIM1_CH2 | Salida PWM |
| PA10 | RPWM motor FR | TIM1_CH3 | Salida PWM |
| PA11 | CAN RX | FDCAN1_RX | Entrada |
| PA12 | CAN TX | FDCAN1_TX | Salida |
| PA13 | SWDIO | SWD | Debug |
| PA14 | SWCLK | SWD | Debug |
| PA15 | Encoder canal A | TIM2_CH1 | Entrada |
| PB0 | Bus OneWire (5× DS18B20) | GPIO | Bidireccional |
| PB3 | Encoder canal B | TIM2_CH2 | Entrada |
| PB4 | Encoder índice Z | EXTI4 | Entrada |
| PB5 | Sensor centrado dirección | EXTI5 | Entrada (pull-up) |
| PB6 | I2C SCL | I2C1_SCL | Bidireccional (OD) |
| PB7 | I2C SDA | I2C1_SDA | Bidireccional (OD) |
| PB10 | Relé LED frontal | GPIO | Salida |
| PB11 | Relé LED trasero | GPIO | Salida |
| PB14 | LED_DIAG | GPIO_Output | Salida digital |
| PB15 | Velocidad rueda RR | EXTI15 | Entrada (pull-up) |
| PC5 | Enable motor FL | GPIO | Salida |
| PC6 | RPWM motor RL | TIM8_CH1 | Salida PWM |
| PC7 | LPWM motor RL | TIM8_CH2 | Salida PWM |
| PC8 | RPWM motor RR | TIM8_CH3 | Salida PWM |
| PC9 | LPWM motor RR | TIM8_CH4 | Salida PWM |
| PC10 | **DISPONIBLE** | — | **Sin usar** (anteriormente RELAY_MAIN) |
| PC11 | Relé tracción | GPIO | Salida |
| PC12 | Relé dirección | GPIO | Salida |
| PC13 | Enable motor RR | GPIO | Salida |

**GPIO usados:** 33 de 51 disponibles

---
---

## PARTE 3 — RESUMEN DE COMPONENTES PASIVOS NECESARIOS

### Resistencias

| Cantidad | Valor | Tolerancia | Ubicación |
|----------|-------|------------|-----------|
| 10 | 330 Ω ¼W | 5% | LED de optoacopladores HY-M158 (PWM señales BTS7960). Relés de potencia ya NO necesitan — usan módulo 4-ch SRD-12VDC-SL-C |
| 2 | 4.7 kΩ | 5% | Pull-up I2C (PB6, PB7 del STM32) |
| 2 | 4.7 kΩ | 5% | Pull-up I2C (GPIO 8, 9 del ESP32) — si no están en módulo |
| 1 | 4.7 kΩ | 5% | Pull-up OneWire (PB0) |
| 2 | 120 Ω ¼W | 5% | Terminación CAN Bus (cada extremo) |
| 1 | 10 kΩ | 1% | Divisor pedal R1 |
| 1 | 6.8 kΩ | 1% | Divisor pedal R2 |
| 3 | 330 Ω | 5% | R_IN serie LED 6N137 encoder (A, B, Z) |
| 3 | 4.7 kΩ | 5% | Pull-up salida Vo 6N137 encoder a +3.3V (A, B, Z) |
| 5 | 100 Ω ¼W | 5% | Snubber motores |
| 2 | 330 Ω | 5% | Línea datos WS2812B (frontal, trasera) |
| 1 | 1 kΩ | 5% | Serie TX DFPlayer |

### Condensadores

| Cantidad | Valor | Tipo | Ubicación |
|----------|-------|------|-----------|
| 1 | 1000 µF / 25 V | Electrolítico | Entrada alimentación principal STM32 |
| 2 | 1000 µF / 6.3 V | Electrolítico | Alimentación tiras LED (una por tira) |
| 4 | 100 nF | Cerámico | Junto a cada pin VDD del STM32 |
| 1 | 10 µF / 6.3 V | Tantalio | Pin VDDA (analógico) del STM32 |
| 2 | 20 pF | Cerámico | Condensadores de carga del cristal 8 MHz |
| 5 | 100 nF / 50 V | Cerámico | Snubber motores |
| ~8 | 100 nF | Cerámico | Desacoplo de módulos (TCA9548A, INA226, TJA1051T/3, etc.) |

### Diodos

| Cantidad | Modelo | Ubicación |
|----------|--------|-----------|
| 5 | 1N4148 | Snubber antiparalelo en los 5 motores |
| 5 | 1N4007 | Diodo volante (flyback) en bobinas de relés (MAIN, TRAC, DIR, LED_F, LED_R) |

### Optoacopladores

| Cantidad | Modelo | Ubicación |
|----------|--------|-----------|
| 10 | HY-M158 | Aislamiento señales PWM (10 canales) |
| 1 | Módulo 4-ch opto relé SRD-12VDC-SL-C | Etapa intermedia relés potencia (MAIN, TRAC, DIR) |

### Otros

| Cantidad | Componente | Ubicación |
|----------|-----------|-----------|
| 1 | Cristal 8 MHz | Oscilador externo STM32 (PH0/PH1) |
| 2 | TJA1051T/3 | Transceiver CAN (uno por MCU) — VCC=5V, VIO=3.3V |
| 1 | TCA9548A | Multiplexor I2C |
| 6 | INA226 | Sensores de corriente/tensión |
| 1 | MCP23017 | I/O expander para shifter |

---

_Documento generado a partir del firmware y documentación del proyecto Control Coche Marcos._  
_Versión: 1.0_
