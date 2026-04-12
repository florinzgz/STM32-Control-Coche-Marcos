# CABLEADO DE AISLAMIENTO DEFINITIVO — Plan Práctico de Implementación

**Sistema de Control Vehicular — STM32G474RE**
**Documento base:** `AISLAMIENTO_GALVANICO_6N137.md`
**Estado: Plan listo para montar — Firmware SIN modificar**

---

## Índice

1. [Recursos disponibles — inventario completo](#1-recursos-disponibles--inventario-completo)
2. [Principios de asignación revisados](#2-principios-de-asignación-revisados)
3. [Por qué el PC817 NO es válido para el encoder A/B](#3-por-qué-el-pc817-no-es-válido-para-el-encoder-ab)
4. [Por qué el PC817 SÍ es válido para los demás sensores](#4-por-qué-el-pc817-sí-es-válido-para-los-demás-sensores)
5. [Asignación de canales por módulo](#5-asignación-de-canales-por-módulo)
6. [Tabla de cableado definitiva](#6-tabla-de-cableado-definitiva)
7. [Circuitos de conexión prácticos](#7-circuitos-de-conexión-prácticos)
8. [Notas de cableado por señal](#8-notas-de-cableado-por-señal)
9. [Aisladores digitales para el bus CAN](#9-aisladores-digitales-para-el-bus-can)
10. [Verificación de compatibilidad con el firmware](#10-verificación-de-compatibilidad-con-el-firmware)
11. [Canales sobrantes — uso recomendado](#11-canales-sobrantes--uso-recomendado)

---

## 1. Recursos disponibles — inventario completo

| Componente | Cantidad | Canales totales | Velocidad máxima |
|------------|----------|-----------------|-----------------|
| Módulos dobles **6N137** | 5 | 10 canales | 10 Mbps / 120 ns propagación |
| Placas de 8 canales **PC817** | 2 | 16 canales | ~80 kHz / 4–6 µs propagación |
| **Aisladores digitales entre placas** | según disponibilidad | ≥ 2 canales | ≥ 10 Mbps (CAN 500 kbps) |

### Características clave del PC817 (relevantes para esta aplicación)

| Parámetro | Valor típico | Valor máximo |
|-----------|-------------|-------------|
| Tiempo de subida (t_r) | 4 µs | 18 µs |
| Tiempo de bajada (t_f) | 3 µs | 18 µs |
| Retardo de propagación (t_PLH) | 4 µs | 18 µs |
| Frecuencia máxima útil | ~80 kHz | — |
| Corriente LED mínima | 1 mA | — |
| Corriente LED típica (10 mA) | I_F = 10 mA | — |
| Salida | Open-collector NPN | — |
| Tensión de aislamiento | 5 000 V rms | — |

### Características clave del 6N137 (para comparación)

| Parámetro | Valor típico | Valor máximo |
|-----------|-------------|-------------|
| Retardo de propagación (t_PLH / t_PHL) | 75 ns | 120 ns |
| Velocidad máxima | 10 Mbps | — |
| Salida | Open-collector, activa baja | — |
| Tensión de aislamiento | 2 500 V rms | — |

---

## 2. Principios de asignación revisados

La reorganización respecto al plan anterior se basa en un único criterio técnico determinante:
**la compatibilidad con el filtro digital hardware de TIM2**.

| Tipo de señal | Aislador asignado | Criterio |
|---------------|-------------------|----------|
| Encoder cuadratura A/B (hasta 20 kHz) | **6N137** | El filtro digital de TIM2 rechaza flancos más lentos que 282 ns. El PC817 (4–6 µs) quedaría bloqueado. |
| Todos los sensores inductivos externos (EXTI) | **PC817** | Frecuencia máxima < 200 Hz (periodo >> 6 ms). El retardo de 4–6 µs del PC817 es completamente negligible. |
| Canal índice encoder Z | **PC817** | Frecuencia < 50 Hz en uso normal. No activo en el firmware actual. |
| Bus CAN (STM32 ↔ ESP32) | **Aislador digital** | Protocolo diferencial de alta velocidad (500 kbps). Requiere chip dedicado con aislamiento galvánico completo. |

### Lo que cambia respecto al plan anterior

| Señal | Plan anterior | Plan actual | Motivo del cambio |
|-------|--------------|-------------|-------------------|
| ENC_A / ENC_B | 6N137 | **6N137 (sin cambio)** | Obligatorio — no compatible con PC817 |
| ENC_Z | 6N137 | **PC817** | < 50 Hz — PC817 suficiente; libera canales 6N137 |
| WHEEL_FL/FR/RL/RR | 6N137 | **PC817** | < 200 Hz — PC817 suficiente; placas PC817 son el recurso adecuado |
| STEER_CENTER | 6N137 | **PC817** | Pulso único — PC817 más que suficiente |
| CAN TX/RX | Sin aislar | **Aislador digital** | Nueva categoría de hardware disponible |

---

## 3. Por qué el PC817 NO es válido para el encoder A/B

Este es el punto técnico central que determina toda la arquitectura.

### El filtro digital del TIM2 rechazaría los flancos del PC817

El firmware configura `IC1Filter = 6` en `MX_TIM2_Init()`. Según RM0440 Tabla 228:

```
Filtro 0110 = f_DTS / 8, con N = 6 muestras requeridas para validar un flanco
Con f_DTS = 170 MHz (sin prescaler de clock division):
  Umbral de rechazo = N × (8 / f_DTS) = 6 × 8 / 170 MHz ≈ 282 ns

  → Cualquier flanco más lento que 282 ns es DESCARTADO por el hardware
```

**Retardo del PC817: 4 000–18 000 ns >> 282 ns → el TIM2 rechazaría cada flanco del encoder.**

Resultado práctico: con PC817 en la línea de ENC_A o ENC_B, el contador TIM2 permanecería en cero independientemente del movimiento del encoder. La dirección quedaría sin control de posición.

**Retardo del 6N137: 75–120 ns < 282 ns → el TIM2 acepta el flanco correctamente. ✅**

### Distorsión de ciclo de trabajo en modo cuadratura

A 20 kHz (peor caso teórico del encoder a 1000 RPM):

```
Periodo de señal = 50 µs

PC817:  retardo = 4–6 µs → distorsión = 8–12 % del periodo
         → El TIM2 vería pulsos con duty cycle incorrecto → conteos erróneos

6N137:  retardo = 0.12 µs → distorsión = 0.24 % del periodo
         → Completamente transparente para TIM2 ✅
```

**Conclusión: Los módulos 6N137 son EXCLUSIVOS de ENC_A y ENC_B. No hay alternativa.**

---

## 4. Por qué el PC817 SÍ es válido para los demás sensores

### Sensores inductivos de velocidad de rueda (EXTI)

```
Frecuencia máxima (100 km/h, 6 tornillos/rev, circunferencia 1.1 m):
  revs/s = (100/3.6) / 1.1 = 25.25 r/s
  f = 25.25 × 6 = 151 Hz
  Periodo mínimo = 1 / 151 = 6.62 ms = 6 620 µs

Retardo PC817 = 4–6 µs
Porcentaje de retardo = 6 µs / 6 620 µs = 0.09 %  → despreciable ✅

Debounce software existente en Wheel_IRQDebounced(): 1 ms
  1 ms = 1 000 µs >> 6 µs → el debounce no queda afectado ✅
```

No hay impacto sobre ABS, TCS ni odometría.

### Sensor de centrado de dirección (EXTI5)

Este sensor genera un único pulso en el arranque durante el barrido de calibración.
La velocidad de barrido es muy baja (~10 % del ciclo PWM = movimiento lento). El pulso
tiene anchura de varios cientos de milisegundos. El PC817 introduce 4–6 µs → absolutamente
negligible. ✅

### Canal índice encoder Z (EXTI4 — actualmente inactivo)

```
Velocidad máxima práctica del eje: 300 RPM
Frecuencia Z = 300 / 60 = 5 Hz
Periodo = 200 ms = 200 000 µs >> 6 µs  ✅

Anchura mínima del pulso Z a 300 RPM, encoder 1200 PPR:
  Resolución angular por pulso A/B = 360° / 4800 = 0.075°
  El pulso Z dura 1 conteo, que a 300 RPM tarda:
  t = (0.075° / 360°) × (60 / 300) = 41.7 µs >> 6 µs  ✅
```

El PC817 es válido para Z. Cuando se active en firmware, la señal será correctamente
detectada por EXTI4. Ver Sección 10 para el único cambio de firmware necesario.

---

## 5. Asignación de canales por módulo

### Módulos 6N137 (5 dobles = 10 canales)

| Módulo | Canal A | Canal B | Estado |
|--------|---------|---------|--------|
| **6N137-M1** | **ENC_A** (PA15 / TIM2_CH1) | **ENC_B** (PB3 / TIM2_CH2) | ⚡ EN USO — obligatorio |
| 6N137-M2 | *(libre)* | *(libre)* | ⬜ Reservado |
| 6N137-M3 | *(libre)* | *(libre)* | ⬜ Reservado |
| 6N137-M4 | *(libre)* | *(libre)* | ⬜ Reservado |
| 6N137-M5 | *(libre)* | *(libre)* | ⬜ Reservado |

Los 8 canales sobrantes (M2–M5) se reservan para futuras señales de alta velocidad. Ver
Sección 11 para usos potenciales. **No conectar por ahora.**

---

### PC817 Placa-A (8 canales) — señales activas del vehículo

| Canal | Señal | Pin STM32 | Periférico | Sensor origen |
|-------|-------|-----------|------------|---------------|
| **A1** | WHEEL_FL | PA0 | EXTI0 | LJ12A3 cubo rueda FL |
| **A2** | WHEEL_FR | PA1 | EXTI1 | LJ12A3 cubo rueda FR |
| **A3** | WHEEL_RL | PA2 | EXTI2 | LJ12A3 cubo rueda RL |
| **A4** | WHEEL_RR | PB15 | EXTI15 | LJ12A3 cubo rueda RR |
| **A5** | STEER_CENTER | PB5 | EXTI5 | LJ12A3 cremallera dirección |
| **A6** | ENC_Z | PB4 | EXTI4 *(inactivo)* | E6B2-CWZ6C canal Z |
| A7 | *(libre)* | — | — | Reservado |
| A8 | *(libre)* | — | — | Reservado |

---

### PC817 Placa-B (8 canales) — expansión y reserva

| Canal | Señal | Estado |
|-------|-------|--------|
| B1–B8 | *(todos libres)* | ⬜ Reservado para futuros sensores inductivos o señales externas |

Usos previstos para Placa-B: sensores de freno de mano, finales de carrera mecánicos,
sensores de presión de rueda, entradas de seguridad externas, señales de carrocería.

---

### Aisladores digitales — Bus CAN (STM32 ↔ ESP32)

| Canal | Señal | Dirección | Pin STM32 | Velocidad requerida |
|-------|-------|-----------|-----------|---------------------|
| **ISO-1** | CAN_TX | STM32 → ESP32 | PA12 (FDCAN1_TX AF9) | ≥ 5 Mbps |
| **ISO-2** | CAN_RX | ESP32 → STM32 | PA11 (FDCAN1_RX AF9) | ≥ 5 Mbps |

Ver Sección 9 para detalles del circuito y chips recomendados.

---

## 6. Tabla de cableado definitiva

Tabla completa de todas las señales aisladas, lista para ejecutar en taller.

| # | Señal | Origen físico | Pin STM32 | Tipo aislador | Módulo concreto | Lado ruidoso | Lado lógico | Motivo técnico |
|---|-------|--------------|-----------|---------------|-----------------|-------------|-------------|----------------|
| 1 | **ENC_A** | E6B2-CWZ6C salida A (NPN OC, 5–24 V) | PA15 / TIM2_CH1 | **6N137** | M1, canal A | VCC_encoder dominio | 3.3 V STM32 | Cuadratura 20 kHz; TIM2 IC filter 282 ns < PC817 4 µs → PC817 sería rechazado |
| 2 | **ENC_B** | E6B2-CWZ6C salida B (NPN OC, 5–24 V) | PB3 / TIM2_CH2 | **6N137** | M1, canal B | VCC_encoder dominio | 3.3 V STM32 | ídem canal A |
| 3 | **WHEEL_FL** | LJ12A3 cubo rueda FL (NPN OC, 6–36 V) | PA0 / EXTI0 | **PC817** | Placa-A, canal A1 | 12–24 V sensor + GND chasis | 3.3 V STM32 | 151 Hz máx; periodo 6.6 ms >> 6 µs PC817; bucle de masa en cubo rueda |
| 4 | **WHEEL_FR** | LJ12A3 cubo rueda FR (NPN OC, 6–36 V) | PA1 / EXTI1 | **PC817** | Placa-A, canal A2 | 12–24 V sensor + GND chasis | 3.3 V STM32 | ídem WHEEL_FL |
| 5 | **WHEEL_RL** | LJ12A3 cubo rueda RL (NPN OC, 6–36 V) | PA2 / EXTI2 | **PC817** | Placa-A, canal A3 | 12–24 V sensor + GND chasis | 3.3 V STM32 | ídem WHEEL_FL |
| 6 | **WHEEL_RR** | LJ12A3 cubo rueda RR (NPN OC, 6–36 V) | PB15 / EXTI15 | **PC817** | Placa-A, canal A4 | 12–24 V sensor + GND chasis | 3.3 V STM32 | ídem WHEEL_FL |
| 7 | **STEER_CENTER** | LJ12A3 cremallera dirección (NPN OC, 6–36 V) | PB5 / EXTI5 | **PC817** | Placa-A, canal A5 | 12–24 V sensor, cerca motor dirección | 3.3 V STM32 | Pulso único en arranque; falsa detección → cero encoder incorrecto; picos inductivos relé DIR |
| 8 | **ENC_Z** | E6B2-CWZ6C salida Z (NPN OC, 5–24 V) | PB4 / EXTI4 *(†)* | **PC817** | Placa-A, canal A6 | VCC_encoder dominio | 3.3 V STM32 | < 50 Hz; pulso 41.7 µs >> 6 µs PC817; EXTI no activo en firmware actual |
| 9 | **CAN_TX** | STM32G474RE PA12 (FDCAN1_TX) | PA12 | **Aislador digital** | ISO canal 1 | 3.3 V STM32 dominio | VCC_transceiver aislado | Aislamiento galvánico entre GND STM32 y GND ESP32 |
| 10 | **CAN_RX** | Transceiver CAN lado ESP32 | PA11 | **Aislador digital** | ISO canal 2 | VCC_transceiver aislado | 3.3 V STM32 dominio | Ídem CAN_TX |

*(†) ENC_Z: no activo en el firmware actual. Requiere configurar EXTI4 y añadir ISR cuando
se decida activar. Ver Sección 10.5.*

---

## 7. Circuitos de conexión prácticos

### 7.1 Circuito 6N137 para ENC_A y ENC_B

El E6B2-CWZ6C tiene salida NPN de colector abierto con pull-up interno al VCC del encoder.
La tensión de alimentación del encoder puede ser 5 V o 12 V (verificar etiqueta del encoder).

```
  LADO RUIDOSO (VCC_encoder, 5–12 V)         BARRERA         LADO LÓGICO (3.3 V STM32)
  ───────────────────────────────────         6N137           ─────────────────────────

  VCC_encoder ─── R_entrada ────────── Pin 2 (Anode)
                  (ver tabla de R)                │
                                                  │           Pin 6 (Vo) ──┬── STM32 PA15 / PB3
  Encoder (A o B)                        Dentro  │ 6N137                  │
  salida NPN OC ─────────────────────── Pin 3 (Cathode)                 [4.7 kΩ]
                                                  │                        │
  GND_encoder ────────────────────────── Pin 5 (GND_izq)    Pin 5 (GND_der) ┴── GND_STM32
                                                  │
                                         Pin 7 (ENABLE) ────── VCC_3.3V (mantener HIGH)
                                         Pin 8 (Vcc) ─────── VCC_encoder (5–12 V)

  NOTA CRÍTICA: GND_encoder y GND_STM32 son ELÉCTRICAMENTE SEPARADOS.
                ¡NO conectarlos entre sí en ningún punto del circuito!
```

**Tabla de resistencia de entrada R según tensión del encoder:**

| VCC_encoder | I_F objetivo (10 mA) | R_entrada |
|-------------|---------------------|-----------|
| 5 V | 10 mA | 330–390 Ω (usar 330 Ω) |
| 12 V | 10 mA | 1 kΩ (usar 1 kΩ) |
| 24 V | 10 mA | 2.2 kΩ (usar 2.2 kΩ) |

**Pull-up en el lado lógico:**
- El firmware configura PA15 y PB3 como `GPIO_NOPULL` (ver `stm32g4xx_hal_msp.c` línea 158).
- **Obligatorio**: añadir resistencia de pull-up **externa** de 4.7 kΩ entre el Pin 6 de cada
  6N137 y la línea de 3.3 V.
- Sin este pull-up externo, la línea flotaría en el nivel alto → el TIM2 no funcionaría.

---

### 7.2 Circuito PC817 para sensores LJ12A3 (ruedas + centrado + ENC_Z)

Las placas de 8 canales PC817 incluyen resistencias serie en la entrada. Verificar el valor
de la resistencia de la placa antes de conectar. Configuración típica de módulo industrial:

```
  LADO RUIDOSO (VCC_sensor, 6–36 V)         BARRERA         LADO LÓGICO (3.3 V STM32)
  ────────────────────────────────           PC817           ─────────────────────────

  VCC_sensor ─── R_placa ──────────── Ánodo LED PC817
                 (resistencia        │
                 de la placa)        │           Colector ──── VCC_3.3V (pull-up de la placa
                                     │                         o pull-up interno STM32)
  Sensor LJ12A3                      │
  salida NPN OC ─────────────────── Cátodo LED
                                     │
  GND_sensor ─────────────────────── GND_primario   GND_secundario ──── GND_STM32

  NOTA: GND_sensor y GND_STM32 son ELÉCTRICAMENTE SEPARADOS.
```

**Verificación de la resistencia de entrada de la placa PC817 según tensión del sensor:**

| VCC_sensor | R_placa típica | I_F resultante | ¿OK? |
|------------|---------------|----------------|------|
| 12 V | 1 kΩ | (12 – 1.5) / 1000 = 10.5 mA | ✅ |
| 12 V | 510 Ω | (12 – 1.5) / 510 = 20.6 mA | ✅ (dentro de If_max = 50 mA) |
| 24 V | 1 kΩ | (24 – 1.5) / 1000 = 22.5 mA | ✅ |
| 24 V | 510 Ω | (24 – 1.5) / 510 = 44.1 mA | ⚠️ Límite — preferir 2.2 kΩ a 24 V |
| 36 V | 1 kΩ | (36 – 1.5) / 1000 = 34.5 mA | ✅ |
| 36 V | 510 Ω | 68.6 mA | ❌ Excede If_max — añadir R serie |

**Si VCC_sensor = 24 V y la placa incluye R = 510 Ω:** añadir resistencia serie de 1 kΩ
en el cable del sensor antes de la entrada de la placa.

**Pull-up en el lado lógico (salida PC817 → STM32 EXTI):**

El firmware configura los pines EXTI con `GPIO_PULLUP` (pull-up interno STM32 ≈ 40 kΩ).
Esto es suficiente para el colector del PC817:
- PC817 transistor ON → colector a GND → línea LOW → EXTI detecta LOW
- PC817 transistor OFF → línea HIGH (vía 40 kΩ interno) → EXTI detecta HIGH ✅

Si la placa PC817 incluye pull-up externo en la salida (común en módulos industriales),
deshabilitar el `GPIO_PULLUP` del firmware o usar la configuración `GPIO_NOPULL` y dejar
actuar el pull-up de la placa. **Ambas opciones son válidas sin cambios de firmware
funcionales.**

---

### 7.3 Polaridad de señales — verificación para todos los optoacopladores

Tanto el PC817 como el 6N137 preservan la **polaridad lógica original** cuando el sensor
es NPN de colector abierto. Análisis:

```
Estado "inactivo" (sensor no detecta):
  - Sensor NPN OC: transistor OFF → salida = HIGH (vía pull-up del sensor o VCC_sensor)
  - LED del optoacoplador: sin corriente → LED OFF
  - Colector del optoacoplador: no conducción → salida = HIGH (pull-up secundario)
  → STM32 ve: HIGH  ✓

Estado "activo" (sensor detecta):
  - Sensor NPN OC: transistor ON → salida = LOW (tira a GND)
  - Corriente fluye: VCC_sensor → R → LED ánodo → LED → cátodo → GND_sensor
  - LED del optoacoplador: conduciendo → transistor secundario ON
  - Colector del optoacoplador: conducción → salida = LOW
  → STM32 ve: LOW  ✓

Transición LOW→HIGH (flanco ascendente):
  Sensor deja de detectar → LED se apaga → transistor secundario OFF → salida sube a HIGH
  → STM32 EXTI ve flanco RISING ✓ (concuerda con GPIO_MODE_IT_RISING del firmware)
```

**Resultado: La polaridad se conserva. El firmware no necesita cambiar la configuración
de flanco de EXTI.** ✅

---

## 8. Notas de cableado por señal

### 8.1 ENC_A y ENC_B (6N137-M1)

- Alimentar el lado primario del encoder a la misma VCC que usa el encoder (5 V o 12 V).
- Añadir condensador de desacoplo de 100 nF entre VCC_encoder y GND_encoder, cerca del módulo.
- El cable del encoder al módulo 6N137 debe ser lo más corto posible (< 30 cm) o usar par trenzado apantallado si el recorrido es largo.
- El GND del encoder NO debe conectarse al GND del STM32. Conectar solo al GND_primario del 6N137.
- Los resistores de 4.7 kΩ de pull-up del lado secundario deben situarse físicamente cerca de los pines PA15 y PB3 del STM32.

### 8.2 WHEEL_FL / FR / RL / RR (PC817 Placa-A, A1–A4)

- Los 4 sensores LJ12A3 comparten el mismo dominio de tensión (alimentados todos a 12 V o todos a 24 V).
- Los GND de los sensores de rueda se unen a la masa del chasis/bastidor del vehículo.
- Esta masa de chasis puede diferir de la GND de la placa STM32 (especialmente en bobinados de motor). El optoacoplador rompe este bucle.
- Los cables de los sensores de rueda deben discurrir alejados (> 10 cm) de los cables de potencia de los motores de tracción.
- Si no es posible separar, usar cable apantallado con la pantalla conectada a GND_chasis en un solo extremo.

### 8.3 STEER_CENTER (PC817 Placa-A, A5)

- Este sensor detecta un tornillo físico en la cremallera durante el barrido de calibración al arranque.
- Solo genera un pulso en la secuencia de inicio (`SteeringCentering_Step()` en el firmware).
- El aislamiento previene que un transitorio inductivo del relé RELAY_DIR (PC12) o del BTS7960
  de dirección se propague por el cable del sensor y dispare EXTI5 erróneamente durante la calibración.

### 8.4 ENC_Z (PC817 Placa-A, A6) — SEÑAL INACTIVA EN FIRMWARE ACTUAL

- El canal Z está físicamente conectado a PB4 y en el plan de cableado pasa por el PC817.
- **En el firmware actual, PB4 no está inicializado como EXTI ni tiene ISR asociada.**
  Si se conecta el PC817 con salida a PB4 ahora, no ocurrirá nada (señal ignorada).
- Cuando se active en el firmware (ver Sección 10.5), el cableado ya estará listo.
- El PC817 es adecuado (Z pulsa a < 50 Hz en uso normal de dirección).

### 8.5 Alimentación de los módulos

- Los módulos PC817 necesitan VCC_sensor (6–36 V) en el lado ruidoso y VCC_lógico (3.3 V) en el lado STM32.
- Los módulos 6N137 necesitan VCC_encoder (5–12 V) en el lado ruidoso y VCC_lógico (3.3 V) en el lado STM32.
- Añadir condensador de desacoplo de 100 nF en cada línea de alimentación de módulo, cerca de los terminales.

---

## 9. Aisladores digitales para el bus CAN

### 9.1 Situación actual

El bus CAN conecta el STM32G474RE (PA11/PA12) con el ESP32-S3 a través del transceiver
TJA1051T/3. El transceiver ya rechaza el ruido de modo común en el bus diferencial (±25 V),
pero ambas placas (STM32 y ESP32) comparten la referencia de masa del sistema. En un
vehículo con motores DC de 24 V, los transitorios en la masa del chasis pueden acoplar
ruido en el CAN RX del STM32 o provocar reinicio del MCU si la diferencia de potencial
de masa supera la tolerancia del GPIO.

### 9.2 Especificación del aislador digital requerido

| Parámetro | Requisito | Razón |
|-----------|-----------|-------|
| Velocidad de señal | ≥ 5 Mbps | CAN a 500 kbps necesita 10× margen de seguridad |
| Canales | 2 (uno TX, uno RX) | O dirección lógica (1+1 unidireccionales) |
| Tensión de aislamiento | ≥ 1 000 V rms | Separación galvánica entre GND STM32 y GND ESP32 |
| Retardo de propagación | ≤ 100 ns | Margen amplio: bit CAN = 2 µs >> 100 ns |
| Alimentación lado aislado | Requiere VCC aislada | Necesita DC-DC aislado para alimentar TJA1051T/3 |

### 9.3 Chips recomendados

#### Opción A — Chip de aislamiento digital + DC-DC separado

```
STM32 PA12 (TX) ──► ADuM1201 canal A ──► TXD del TJA1051T/3 (lado aislado)
STM32 PA11 (RX) ◄── ADuM1201 canal B ◄── RXD del TJA1051T/3 (lado aislado)

Alimentación del lado aislado:
  VCC_3.3V → SN6501 (driver de bobina) → transformador 1:1 → rectificado → 3.3V_aislada
  O: módulo DC-DC aislado de 3.3V/200mA (p. ej., RECOM R-7803 o TMR1-xxxx)
```

| Chip | Descripción | Velocidad | Canales |
|------|-------------|-----------|---------|
| **ADuM1201** (Analog Devices) | Isolador digital dual | 150 Mbps | 2 |
| **Si8621** (Silicon Labs) | Isolador digital dual | 150 Mbps | 2 |
| **HCPL-0630** (Broadcom) | Optoacoplador alta vel. | 10 Mbps | 2 |

#### Opción B — Transceiver CAN integrado con aislamiento (más simple)

Sustituye el TJA1051T/3 + aislador por un chip único. No requiere DC-DC externo porque
el chip incluye convertidor interno.

| Chip | Descripción | Velocidad | Notas |
|------|-------------|-----------|-------|
| **ISO1050** (Texas Instruments) | CAN transceiver + isolador | 1 Mbps | Recomendado para nueva instalación |
| **ADUM1191** (Analog Devices) | CAN transceiver + isolador | 1 Mbps | Compatible con CAN 2.0A |

**Recomendación:** Si se dispone del chip como "aislador digital entre placas", usar la
**Opción A** (ADuM1201 o Si8621) junto con un pequeño DC-DC aislado de 3.3 V. Si se
puede adquirir componentes, la **Opción B** (ISO1050) es más limpia y compacta.

### 9.4 Esquema de conexión con ADuM1201 + DC-DC

```
Lado STM32 (GND_STM32)          Barrera galvánica         Lado CAN (GND_CAN)
───────────────────────          ─────────────────          ──────────────────

STM32 PA12 ──────────────► ADuM1201 [A→B] ──────────────► TXD (TJA1051T/3)
STM32 PA11 ◄────────────── ADuM1201 [B→A] ◄────────────── RXD (TJA1051T/3)

3.3V_STM32 ──► ADuM1201 Vdd1     ADuM1201 Vdd2 ◄── 3.3V_aislada (del DC-DC)

5V_rail    ──► DC-DC aislado ──► 5V_aislada   ──► TJA1051T/3 VCC  (⚠️ VCC = 4.5–5.5 V; 3.3 V NO funciona)

CANH/CANL ────────────────────────────────────────── Bus CAN físico (ESP32 end)
```

---

## 10. Verificación de compatibilidad con el firmware

### 10.1 ENC_A y ENC_B — sin cambio de firmware

El firmware en `MX_TIM2_Init()` configura:
- `IC1Filter = 6` (282 ns umbral de rechazo) — compatible con 6N137 (120 ns) ✅
- `IC1Polarity = TIM_ICPOLARITY_RISING` — el 6N137 preserva la polaridad (NPN OC doble inversión) ✅
- `GPIO_NOPULL` en PA15 y PB3 — el pull-up externo de 4.7 kΩ en el secundario del 6N137 cumple esta función ✅

**Ningún cambio de firmware necesario.** Solo hardware: añadir pull-up 4.7 kΩ externo.

### 10.2 Sensores de rueda — sin cambio de firmware

El firmware en `MX_GPIO_Init()` configura:
- `GPIO_MODE_IT_RISING` para PA0/PA1/PA2/PB15 — la polaridad se conserva a través del PC817 ✅
- `GPIO_PULLUP` — el pull-up interno de 40 kΩ es suficiente para el colector del PC817 ✅

El debounce de 1 ms en `Wheel_IRQDebounced()` no queda afectado por el retardo de 4–6 µs
del PC817. **Ningún cambio de firmware necesario.**

### 10.3 Sensor de centrado de dirección — sin cambio de firmware

El firmware configura PB5 con `GPIO_MODE_IT_RISING` y `GPIO_PULLUP`. La lógica de
`SteeringCenter_IRQHandler()` → `steer_center_flag = 1` es compatible con la señal
proveniente del PC817. **Ningún cambio de firmware necesario.**

### 10.4 Bus CAN — sin cambio de firmware

La interfaz software de FDCAN1 (PA11/PA12) no cambia. El aislador digital y el transceiver
(TJA1051T/3 o ISO1050) son transparentes para el firmware. El protocolo CAN, los filtros
de RX, el heartbeat y los temporizadores de timeout funcionan exactamente igual.
**Ningún cambio de firmware necesario.**

### 10.5 ENC_Z — cableado listo, firmware pendiente (futuro)

**Estado actual:** PB4 está definido en `main.h` como `PIN_ENC_Z` pero **no está
inicializado en `MX_GPIO_Init()`** y **no existe ninguna ISR para EXTI4** en
`stm32g4xx_it.c`. El canal Z no genera ningún efecto en el firmware actual.

**El cableado a través del PC817 puede instalarse ahora** sin ninguna consecuencia.
Cuando se decida activar el canal Z, serán necesarias tres modificaciones de firmware:

1. En `MX_GPIO_Init()` (main.c): configurar PB4 como `GPIO_MODE_IT_RISING` + `GPIO_PULLUP`.
2. En `stm32g4xx_it.c`: añadir handler `EXTI4_IRQHandler()` que llame a la función de gestión Z.
3. Habilitar EXTI4_IRQn en el NVIC con `HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0)` y
   `HAL_NVIC_EnableIRQ(EXTI4_IRQn)`.

Estas modificaciones son localizadas y no afectan al resto del firmware.

### 10.6 Resumen de compatibilidad firmware

| Señal | Aislador | Cambio de firmware | Resultado |
|-------|----------|--------------------|-----------|
| ENC_A | 6N137 | ❌ Ninguno | ✅ Compatible sin cambios |
| ENC_B | 6N137 | ❌ Ninguno | ✅ Compatible sin cambios |
| WHEEL_FL/FR/RL/RR | PC817 | ❌ Ninguno | ✅ Compatible sin cambios |
| STEER_CENTER | PC817 | ❌ Ninguno | ✅ Compatible sin cambios |
| ENC_Z | PC817 | ⚠️ Requerido para activar | Cableado ok; activación futura |
| CAN TX/RX | Aislador digital | ❌ Ninguno | ✅ Compatible sin cambios |

---

## 11. Canales sobrantes — uso recomendado

### 6N137 M2–M5 (8 canales libres)

Estos módulos se reservan para señales de alta velocidad que puedan añadirse en el futuro.
**No conectar ahora.** Posibles usos:

| Uso potencial | Señal | Velocidad | Motivo |
|---------------|-------|-----------|--------|
| Aislamiento PWM tracción (salida) | PA8–PA11 (TIM1) | 20 kHz | Barrera bidireccional entre STM32 y BTS7960. Requiere invertir ciclo de trabajo en firmware porque la salida del 6N137 es open-collector activo bajo. **(Cambio de firmware necesario)** |
| Aislamiento PWM dirección (salida) | PC8 (TIM8_CH3) | 20 kHz | ídem |
| Encoder secundario o encoder de velocidad | Futuro | Alta vel. | Si se añade encoder en las ruedas o en el eje del motor |
| SPI display o comunicación adicional | Futuro | Alta vel. | Si se añade periférico externo de alta velocidad |

> **Nota sobre aislamiento de salidas PWM con 6N137:**
> El 6N137 de salida INVIERTE la señal (LED ON → salida LOW, LED OFF → salida HIGH via pull-up).
> Para preservar el ciclo de trabajo, el firmware debería invertir el duty cycle configurando
> `TIM_OCPOLARITY_LOW` en lugar de `TIM_OCPOLARITY_HIGH`. Esta es la única razón por la
> que los PWM de salida no están en el plan actual. Si se decide hacerlo, es un cambio
> localizado en `MX_TIM1_Init()` y `MX_TIM8_Init()`.

### PC817 Placa-A A7–A8 (2 canales libres)

Reservados para señales externas adicionales del entorno del motor de dirección o cremallera.

### PC817 Placa-B B1–B8 (8 canales libres)

Disponibles para expansión del sistema. Candidatos prioritarios:
- Sensor inductivo de freno de mano
- Sensor de presión de neumáticos (señal digital de módulo TPMS)
- Contacto de puerta o carrocería
- Señales de seguridad externas (parada de emergencia, seta de emergencia)
- Señales de expansión desde el ESP32 al STM32 si se añaden entradas adicionales

---

## 12. Lista de comprobación antes del montaje

- [ ] Medir VCC_encoder con multímetro — determinar valor de R_entrada para 6N137
- [ ] Medir VCC_sensores LJ12A3 — verificar R de la placa PC817 es adecuada para esa tensión
- [ ] Confirmar que GND_encoder NO está conectado a GND_STM32 en ningún otro punto del sistema
- [ ] Confirmar que GND_sensor (LJ12A3) NO está conectado a GND_STM32 (puede estar a GND_chasis)
- [ ] Preparar 2× resistencias 4.7 kΩ para pull-up secundario de 6N137 (PA15 y PB3)
- [ ] Preparar condensadores de desacoplo 100 nF para cada módulo de aislamiento
- [ ] Verificar el valor de la resistencia serie de la placa PC817 antes de conectar sensores de 24 V
- [ ] Para el CAN: decidir entre Opción A (ADuM1201 + DC-DC) u Opción B (ISO1050)
- [ ] Verificar que el cable de ENC_Z está conectado al canal A6 de Placa-A pero que el firmware no lo activará hasta que se decida hacerlo

---

> **Estado del firmware:** NINGÚN archivo de código fuente ha sido modificado.
> Este documento es el plan de cableado definitivo listo para ejecutar en taller.
> La implementación física no requiere cambios de firmware para las señales activas.

---

**Fecha:** 2026-02-25
**Basado en:**
- Plan teórico: `docs/AISLAMIENTO_GALVANICO_6N137.md`
- Firmware auditado: `main.c`, `stm32g4xx_hal_msp.c`, `stm32g4xx_it.c`,
  `sensor_manager.c`, `encoder_reader.c`, `motor_control.c`
- Hardware documentado: `docs/HARDWARE_WIRING_MANUAL.md`, `docs/PINOUT_DEFINITIVO.md`
