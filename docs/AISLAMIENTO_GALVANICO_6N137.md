# PLAN DE AISLAMIENTO GALVÁNICO — 5 Módulos Dobles 6N137

**Sistema de Control Vehicular — STM32G474RE**
**Estado: Plan aprobado — Firmware SIN modificar**

---

## Índice

1. [Contexto y recursos disponibles](#1-contexto-y-recursos-disponibles)
2. [Auditoría de señales del sistema](#2-auditoría-de-señales-del-sistema)
3. [Asignación definitiva de módulos](#3-asignación-definitiva-de-módulos)
4. [Verificación de timings y compatibilidad 6N137](#4-verificación-de-timings-y-compatibilidad-6n137)
5. [Señales NO aisladas — justificación técnica](#5-señales-no-aisladas--justificación-técnica)
6. [Circuito de conexión para cada señal](#6-circuito-de-conexión-para-cada-señal)
7. [Riesgos evitados por señal aislada](#7-riesgos-evitados-por-señal-aislada)
8. [Tabla resumen de aislamiento](#8-tabla-resumen-de-aislamiento)

---

## 1. Contexto y recursos disponibles

### Recursos de aislamiento

| Recurso | Cantidad | Canales totales |
|---------|----------|-----------------|
| Módulos dobles 6N137 | **5** | **10 canales** |

### Características del 6N137 (relevantes para este diseño)

| Parámetro | Valor | Fuente |
|-----------|-------|--------|
| Velocidad máxima | 10 Mbps | Datasheet HCPL-2601/6N137 |
| Propagación típica | 75 ns | t_PLH / t_PHL |
| Propagación máxima | 120 ns | worst-case a 25 °C |
| Tensión de aislamiento | 2500 V rms | VDE 0884 |
| Corriente LED mínima (lógica 1) | 1.4–2 mA | I_F typ |
| Salida | Open-collector TTL | Necesita pull-up externo |
| Pin ENABLE (pin 7) | Debe ir a HIGH | Conectar a VCC del lado lógico |

### Premisas de asignación

- **2 módulos** reservados para el encoder E6B2-CWZ6C (canales A y B obligatorios; canal Z incluido si caben en los 4 canales disponibles).
- **3 módulos** restantes destinados a sensores inductivos y señales externas expuestas a ruido.

---

## 2. Auditoría de señales del sistema

### 2.1 Señales candidatas a aislamiento

A continuación se analiza **cada señal del sistema** que cruza un dominio de potencia, sale del entorno de la placa STM32, o es susceptible a ruido eléctrico.

---

#### 2.1.1 Encoder de dirección E6B2-CWZ6C

| Señal | Pin STM32 | Periférico | Frecuencia máxima estimada | Tensión sensor |
|-------|-----------|------------|----------------------------|----------------|
| ENC_A | PA15 | TIM2_CH1 (cuadratura) | ~20 kHz | 5–24 V NPN OC |
| ENC_B | PB3 | TIM2_CH2 (cuadratura) | ~20 kHz | 5–24 V NPN OC |
| ENC_Z | PB4 | EXTI4 | 1–50 Hz (1 pulso/rev) | 5–24 V NPN OC |

**Entorno físico:** El encoder está acoplado mecánicamente al motor de dirección (12 V,
BTS7960). Sus cables discurren junto a los cables de potencia del motor. El BTS7960
conmuta a 20 kHz con corrientes de hasta 10 A, generando EMI significativa.

**Criticidad:** ALTA. El TIM2 en modo cuadratura alimenta el bucle PID de dirección en
tiempo real (10 ms). Errores de conteo debidos a ruido generan posiciones fantasma que
provocan oscilaciones del PID e inestabilidad de la dirección.

**Filtro digital existente:** `IC1Filter = 6` en `MX_TIM2_Init()` → rechazo de glitches
de ~282 ns (ver Sección 4). Este filtro trabaja *a posteriori* de la señal de entrada;
el aislamiento actúa *a priori*, eliminando la fuente de ruido antes de que llegue al MCU.

**Veredicto: AISLAR** — máxima prioridad.

---

#### 2.1.2 Sensores inductivos de velocidad de rueda (LJ12A3)

| Señal | Pin STM32 | Periférico | Frecuencia máxima | Tensión sensor |
|-------|-----------|------------|-------------------|----------------|
| WHEEL_FL | PA0 | EXTI0 | ~150 Hz @ 100 km/h | 6–36 V NPN OC |
| WHEEL_FR | PA1 | EXTI1 | ~150 Hz | 6–36 V NPN OC |
| WHEEL_RL | PA2 | EXTI2 | ~150 Hz | 6–36 V NPN OC |
| WHEEL_RR | PB15 | EXTI15 | ~150 Hz | 6–36 V NPN OC |

**Cálculo de frecuencia máxima:**
```
60 km/h: revs/s = (60/3.6) / 1.1 m = 15.15 r/s → 15.15 × 6 = 91 Hz
100 km/h: revs/s = 27.78 / 1.1 = 25.25 r/s → 25.25 × 6 = 151 Hz (< 200 Hz)
```

**Entorno físico:** Los sensores están montados en los cubos de las ruedas, en el
entorno más ruidoso del vehículo: cables de 24 V de los motores de tracción, corrientes
de hasta 20 A pico, conmutación BTS7960 a 20 kHz. Los cables del sensor discurren
paralelos a los cables de potencia en el chasis.

**Criticidad:** ALTA. Las interrupciones EXTI falsas inflan la velocidad de rueda
computada. El resultado es intervención ABS/TCS incorrecta (puede frenar una rueda que
no patina o ignorar una que sí patina). Pulsos ficticios acumulados en `wheel_pulse[]`
(variable volatile en ISR) no tienen mecanismo de discriminación de origen.

**Debounce existente:** 1 ms de ventana en `Wheel_IRQDebounced()` — sólo elimina
rebotes mecánicos, no protege contra ráfagas de ruido EMI a alta frecuencia que superen
el umbral de 1 ms.

**Veredicto: AISLAR** — los 4 sensores.

---

#### 2.1.3 Sensor inductivo de centrado de dirección (LJ12A3)

| Señal | Pin STM32 | Periférico | Frecuencia | Tensión sensor |
|-------|-----------|------------|------------|----------------|
| STEER_CENTER | PB5 | EXTI5 | 1 pulso en arranque | 6–36 V NPN OC |

**Entorno físico:** Sensor inductivo montado junto a la cremallera de dirección, cerca
del motor de dirección (12 V). Cable expuesto al entorno de potencia.

**Criticidad:** MEDIA-ALTA. Una detección falsa durante el ciclo de calibración de
arranque posiciona erróneamente el "centro" de la dirección → el vehículo conduce torcido
de forma permanente hasta el próximo arranque. El firmware utiliza este pulso para poner
el contador del encoder TIM2 a cero (`SteeringCentering_Step()` en
`steering_centering.c`).

**Veredicto: AISLAR** — encaja en el canal sobrante del 5.º módulo.

---

#### 2.1.4 Bus CAN (FDCAN1 — PB8/PB9)

| Señal | Pin STM32 | Periférico | Bitrate |
|-------|-----------|------------|---------|
| CAN_RX | PB8 | FDCAN1_RX | 500 kbps |
| CAN_TX | PB9 | FDCAN1_TX | 500 kbps |

**Análisis:** El bus CAN ya usa el transceiver TJA1051T/3 con muy buen rechazo en modo
común (±25 V). Es un protocolo diferencial con detección de errores hardware. La
comunicación es de corto alcance (dentro del mismo vehículo).

**Compatibilidad 6N137 a 500 kbps:** Técnicamente viable (6N137 soporta 10 Mbps;
500 kbps = periodo de bit 2 µs >> 120 ns de retardo). Pero el bus CAN necesita
aislamiento *galvánico completo* (incluyendo alimentación del transceiver), lo que exige
un convertidor DC-DC aislado además del optoacoplador — complejidad que excede los
módulos disponibles. La solución correcta sería un chip integrado ISO1050 o ADuM1191.

**Veredicto: NO AISLAR** con los módulos 6N137 disponibles. El TJA1051T/3 ya ofrece
protección suficiente en la mayoría de entornos industriales de automoción. Si la longitud
del bus CAN o el entorno eléctrico lo requirieran en el futuro, usar ISO1050.

---

#### 2.1.5 Bus I2C (INA226 + TCA9548A — PB6/PB7)

| Señal | Pin STM32 | Periférico | Velocidad |
|-------|-----------|------------|-----------|
| I2C_SCL | PB6 | I2C1_SCL | 400 kHz |
| I2C_SDA | PB7 | I2C1_SDA | 400 kHz |

**Análisis:** El protocolo I2C es open-drain bidireccional. El 6N137 es unidireccional.
Aislar I2C con 6N137 requiere 4 optoacopladores (2 por línea, en configuración push-pull
invertida) más lógica adicional para reproducir la función open-drain — complejidad
incompatible con los módulos dobles genéricos. La solución correcta son chips dedicados
como ADUM1250 o ISO1540.

El bus I2C está físicamente en la placa STM32, con los INA226 y TCA9548A relativamente
cerca. El shunt de los INA226 está en el lado de alta potencia, pero el propio INA226
hace la conversión analógica y expone solo señales I2C de 3.3 V al STM32.

**Veredicto: NO AISLAR** con los módulos 6N137. Si se observara corrupción I2C frecuente,
considerar ADUM1250/ISO1540 en una iteración posterior.

---

#### 2.1.6 Bus OneWire — DS18B20 (PB0)

| Señal | Pin STM32 | Protocolo | Velocidad |
|-------|-----------|-----------|-----------|
| ONEWIRE | PB0 | 1-Wire bit-bang | ~16 kbps |

**Análisis:** OneWire es bidireccional y requiere lógica de master/slave en un único
cable. El 6N137 no puede aislar un protocolo bidireccional directamente. Los DS18B20
operan a 3.3 V (mismo dominio que el STM32). Sus cables van hacia los motores pero el
sensor no está en un entorno de alta tensión. La criticidad de los datos de temperatura
es baja (usados solo para protección térmica a 1 Hz).

**Veredicto: NO AISLAR** con los módulos disponibles.

---

#### 2.1.7 Pedal acelerador — ADC analógico (PA3)

| Señal | Pin STM32 | Periférico | Rango |
|-------|-----------|------------|-------|
| PEDAL | PA3 | ADC1_IN4 | 0–3.3 V analógico |

**Análisis:** Señal analógica continua. Los optoacopladores digitales (6N137) no pueden
aislar señales analógicas. La aislamiento analógico requiere convertidores sigma-delta
aislados (AMC1200, ISO124) o transformadores de señal de audio. El pedal ya tiene
doble canal redundante (ADC primario + ADS1115 I2C de plausibilidad) que detecta
desacuerdos del 5 % y registra el fallo `pedal_plausible`. Está físicamente alejado de
los motores (en el habitáculo del conductor).

**Veredicto: NO AISLAR** con los módulos 6N137.

---

#### 2.1.8 Señales PWM/DIR/EN hacia BTS7960 (múltiples pines GPIOA/GPIOC)

| Señal | Pines | Cantidad | Frecuencia |
|-------|-------|----------|------------|
| PWM tración | PA8–PA11 | 4 | 20 kHz |
| PWM dirección | PC8 | 1 | 20 kHz |
| DIR (dirección) | PC0–PC4 | 5 | DC / conmutación lenta |
| EN (habilitación) | PC5–PC7, PC9, PC13 | 5 | DC / conmutación lenta |

**Análisis:** Estas señales van **desde** el STM32 **hacia** el entorno ruidoso (BTS7960).
El riesgo es la inyección inversa de ruido, que el BTS7960 atenúa con su propia
circuitería de entrada (umbral Schmitt interno). El 6N137 puede manejar 20 kHz (muy por
debajo de sus 10 Mbps), pero 15 canales necesarios (5 motores × 3 señales) superan con
creces los 10 canales disponibles, incluso sin asignar ninguno al encoder ni a los
sensores. No es viable con el presupuesto actual.

**Veredicto: NO AISLAR** con los módulos disponibles. El BTS7960 tiene protección interna
suficiente. Si en el futuro se detecta latch-up o disparo falso del driver, considerar
añadir módulos dedicados de salida.

---

#### 2.1.9 Salidas de control de relés (PC10/PC11/PC12)

| Señal | Pin | Función |
|-------|-----|---------|
| RELAY_MAIN | PC10 | Relé principal |
| RELAY_TRAC | PC11 | Relé tracción 24 V |
| RELAY_DIR | PC12 | Relé dirección 12 V |

**Análisis:** Son salidas del STM32 hacia transistores de conmutación. El circuito driver
del relé ya incluye (o debe incluir) diodo flyback y transistor NPN, lo que protege el
GPIO de la bobina inductiva. No hay camino de retorno de ruido hacia el MCU en condiciones
normales de diseño.

**Veredicto: NO AISLAR** — protección adecuada con el circuito driver estándar.

---

## 3. Asignación definitiva de módulos

### Módulo 1 — Encoder cuadratura (A + B)

| Canal | Señal | STM32 Pin | Lado ruidoso (origen) | Lado lógico (destino) |
|-------|-------|-----------|----------------------|----------------------|
| Ch-1 | ENC_A | PA15 / TIM2_CH1 | E6B2-CWZ6C (5–24 V NPN OC, cerca motor 12 V) | STM32 3.3 V lógico |
| Ch-2 | ENC_B | PB3 / TIM2_CH2 | E6B2-CWZ6C (5–24 V NPN OC, cerca motor 12 V) | STM32 3.3 V lógico |

**Motivo:** Los canales A y B son la entrada de cuadratura crítica para el control de
dirección en lazo cerrado. Cualquier pulso falso altera el contador TIM2 y produce un
error de posición que el PID amplifica en forma de corrección de motor.

---

### Módulo 2 — Encoder índice Z + canal libre

| Canal | Señal | STM32 Pin | Lado ruidoso (origen) | Lado lógico (destino) |
|-------|-------|-----------|----------------------|----------------------|
| Ch-1 | ENC_Z | PB4 / EXTI4 | E6B2-CWZ6C (5–24 V NPN OC) | STM32 3.3 V lógico |
| Ch-2 | *(libre)* | — | — | — |

**Motivo:** El canal Z marca una vuelta completa del encoder (pulso de índice). Aunque el
firmware actual lo usa solo como referencia de diagnóstico, en implementaciones futuras
puede emplearse para recalibración periódica del contador. Su aislamiento es viable y no
tiene riesgo de timing (1–50 Hz). El canal libre del módulo 2 puede reservarse para una
futura señal externa o un segundo sensor inductivo de referencia.

---

### Módulo 3 — Sensores velocidad rueda FL + FR

| Canal | Señal | STM32 Pin | Lado ruidoso (origen) | Lado lógico (destino) |
|-------|-------|-----------|----------------------|----------------------|
| Ch-1 | WHEEL_FL | PA0 / EXTI0 | LJ12A3 (6–36 V NPN OC, cubo rueda FL, cerca cable potencia 24 V) | STM32 3.3 V lógico |
| Ch-2 | WHEEL_FR | PA1 / EXTI1 | LJ12A3 (6–36 V NPN OC, cubo rueda FR, cerca cable potencia 24 V) | STM32 3.3 V lógico |

**Motivo:** Los cubos de las ruedas delanteras son el punto de mayor concentración de
ruido electromagnético del vehículo (cables de 24 V de los BTS7960, corrientes de hasta
20 A pico, motor de tracción en cada rueda). Las ISR EXTI son la primera línea de
adquisición de velocidad para ABS y TCS.

---

### Módulo 4 — Sensores velocidad rueda RL + RR

| Canal | Señal | STM32 Pin | Lado ruidoso (origen) | Lado lógico (destino) |
|-------|-------|-----------|----------------------|----------------------|
| Ch-1 | WHEEL_RL | PA2 / EXTI2 | LJ12A3 (6–36 V NPN OC, cubo rueda RL) | STM32 3.3 V lógico |
| Ch-2 | WHEEL_RR | PB15 / EXTI15 | LJ12A3 (6–36 V NPN OC, cubo rueda RR) | STM32 3.3 V lógico |

**Motivo:** Idéntico al Módulo 3. Las ruedas traseras están directamente impulsadas por
los motores de tracción; sus cables de potencia discurren paralelos a los cables de
sensor.

---

### Módulo 5 — Sensor centrado dirección + canal libre

| Canal | Señal | STM32 Pin | Lado ruidoso (origen) | Lado lógico (destino) |
|-------|-------|-----------|----------------------|----------------------|
| Ch-1 | STEER_CENTER | PB5 / EXTI5 | LJ12A3 (6–36 V NPN OC, cremallera dirección, cerca motor 12 V) | STM32 3.3 V lógico |
| Ch-2 | *(libre — reservado)* | — | — | — |

**Motivo:** Una detección falsa del sensor de centro durante el barrido de calibración
inicial posiciona incorrectamente el cero del encoder de dirección. El canal libre del
módulo 5 puede asignarse en el futuro a un sensor inductivo adicional (p. ej., final de
carrera, sensor de presencia, sensor de freno de mano).

---

## 4. Verificación de timings y compatibilidad 6N137

### 4.1 Encoder A/B con TIM2 en modo cuadratura

**Frecuencia de señal:**

El encoder E6B2-CWZ6C tiene 1200 PPR. La velocidad máxima práctica del eje del encoder
en la aplicación de dirección es conservadoramente ~300 RPM (la cremallera está
reducida mecánicamente):

```
f_max = PPR × RPM_max / 60 = 1200 × 300 / 60 = 6 000 Hz por canal
Periodo mínimo de pulso = 1 / 6000 Hz = 166.7 µs
```

Incluso en el peor caso teórico con el encoder girando a 1000 RPM:

```
f_max = 1200 × 1000 / 60 = 20 000 Hz = 20 kHz
Periodo mínimo = 50 µs
```

**Propagación del 6N137:**

```
t_prop_max = 120 ns  (datasheet, worst-case)
```

**Comparación con el periodo de señal:**

```
Porcentaje de retardo respecto al periodo mínimo:
  120 ns / 50 µs = 0.24 %   ← completamente despreciable
```

**Compatibilidad con el filtro digital TIM2 (IC1Filter = 6):**

Según RM0440 Tabla 228, filtro `0110` corresponde a `f_DTS / 8` con N = 6 muestras.
Con f_DTS = 170 MHz y sin prescaler de clock division:

```
Tiempo de rechazo de glitch = N × (8 / f_DTS) = 6 × 8 / 170 MHz ≈ 282 ns
```

La propagación del 6N137 (120 ns max) es inferior al umbral de rechazo (282 ns), por
lo que un flanco real del encoder **atraviesa el optoacoplador en menos tiempo del que el
filtro hardware rechazaría**. No se generarán falsas transiciones. ✅

**Conclusión:** El 6N137 es **plenamente compatible** con la configuración de TIM2 en
modo cuadratura tal y como está configurado en el firmware. No es necesario modificar
`IC1Filter` ni ningún otro parámetro de TIM2.

---

### 4.2 Sensores de rueda con EXTI y debounce por software

**Frecuencia máxima de señal:**

```
100 km/h, 6 tornillos/rev, circunferencia 1.1 m:
  revs/s = (100/3.6) / 1.1 = 25.25 r/s
  f = 25.25 × 6 = 151 Hz
  Periodo = 6.62 ms
```

**Propagación del 6N137 vs. debounce software:**

```
t_prop_max = 120 ns   << 6.62 ms   → completamente invisible para las ISR
t_prop_max = 120 ns   << 1 ms      (ventana de debounce en Wheel_IRQDebounced)
```

El retardo de propagación es cuatro órdenes de magnitud menor que el periodo de señal a
máxima velocidad. **Sin impacto en ABS, TCS ni odometría.** ✅

---

### 4.3 Sensor de centrado y ENC_Z con EXTI

Señales de muy baja frecuencia (1 pulso por evento / 1–50 Hz). El retardo de 120 ns es
absolutamente negligible frente a los tiempos de evento (milisegundos a segundos). ✅

---

### 4.4 Impacto en el IWDG y el arranque

El 6N137 no interviene en ninguna señal del watchdog IWDG (alimentado por LSI interno de
32 kHz, completamente interno al MCU), ni en la línea NRST, ni en la programación SWD
(SWDIO/SWCLK en PA13/PA14). El aislamiento propuesto **no afecta al arranque, al
bootloader ni a la programación** del STM32. ✅

---

## 5. Señales NO aisladas — justificación técnica

| Señal | Razón técnica | Alternativa futura si fuera necesario |
|-------|--------------|---------------------------------------|
| **CAN Bus (PB8/PB9)** | Protocolo diferencial con transceiver TJA1051T/3. El 6N137 por sí solo no aporta aislamiento galvánico completo (necesita también convertidor DC-DC aislado). La solución adecuada es un chip integrado (ISO1050, ADuM1191). | ISO1050 o ADuM1191 |
| **I2C Bus (PB6/PB7)** | Protocolo open-drain bidireccional. El 6N137 es unidireccional. Aislar I2C requiere chips dedicados (ADUM1250, ISO1540). | ADUM1250 o ISO1540 |
| **OneWire (PB0)** | Protocolo bidireccional de un solo cable. Incompatible con optoacopladores estándar. Los DS18B20 operan a 3.3 V en zona de baja tensión. | Si hubiera ruido, mover DS18B20 fuera del bus OneWire a I2C (MCP9808). |
| **Pedal ADC (PA3)** | Señal analógica continua; los optoacopladores digitales no pueden aislarla. El circuito ya tiene protección: divisor de tensión + ADS1115 plausibilidad. | AMC1200, ISO124 (sigma-delta aislados). |
| **PWM/DIR/EN → BTS7960** | 15 canales necesarios (5 motores × 3 señales) superan los canales disponibles. El riesgo es inyección inversa desde BTS7960, ya mitigada por el Schmitt interno del driver. | Módulos adicionales en iteración futura; prioridad si se detectan latch-ups. |
| **Relés (PC10/11/12)** | Señales de salida hacia drivers con diodo flyback. No existe camino de retorno de ruido al MCU con un diseño correcto del driver. | No necesario en condiciones normales. |

---

## 6. Circuito de conexión para cada señal

### 6.1 Encoder E6B2-CWZ6C (canales A, B, Z)

El E6B2-CWZ6C dispone de salida NPN de colector abierto con resistencia de pull-up
interna al encoder (5 V o 12 V según alimentación del encoder).

```
Lado ruidoso (encoder / 5–24 V)         Barrera 6N137       Lado lógico (STM32 / 3.3 V)
                                         ┌─────────┐
         ┌── R_IN ──────────── Pin 2 (A) │         │ Pin 6 (Vo) ──┬── STM32 PA15/PB3/PB4
Encoder  │  (330–470 Ω)                  │  6N137  │              │
NPN OC ──┤                               │         │          [4.7 kΩ]
         │                   Pin 3 (K) ──┤(Cathode)│              │
         └── GND_encoder ────────────────┤  GND    │ Pin 5 (GND) ─┴── GND_STM32
                                         │         │
         VCC_encoder ───────── Pin 8 (Vcc→ interno)
                                         │         │ Pin 7 (EN) ─── VCC_3.3V (HIGH)
                                         └─────────┘
                                          Pin 1 = NC (no conectar)
```

**Notas:**
- `R_IN = 330–470 Ω` para obtener I_F = 10–15 mA con VCC encoder de 5 V.
  Con 12 V: usar R_IN = 680–820 Ω.
- El pin 7 (ENABLE) del 6N137 debe ir a HIGH (3.3 V o 5 V del lado lógico).
- Pull-up de 4.7 kΩ a 3.3 V en la salida (pin 6) hacia el STM32.
- GND del lado encoder y GND del lado lógico son **eléctricamente separados** (objetivo
  del aislamiento). No deben conectarse entre sí en este punto.

---

### 6.2 Sensores inductivos LJ12A3 (ruedas + centrado)

Misma topología que el encoder — NPN de colector abierto, misma resistencia de entrada.

```
Lado ruidoso (sensor / 6–36 V)          Barrera 6N137       Lado lógico (STM32 / 3.3 V)
                                         ┌─────────┐
VCC_sensor (6–36V) ──── Pull-up [2.2 kΩ]─ Pin 2 (A)│         │ Pin 6 (Vo) ──┬── STM32 GPIO
                                         │  6N137  │              │
LJ12A3                                   │         │          [4.7 kΩ]
NPN OC ──── R_IN ───────────── Pin 2 (A)──(ya incluido en pull-up)
            (la corriente la fija el pull-up)
         │
         └── GND_sensor ─────── Pin 3 (K)│         │ Pin 5 (GND)─── GND_STM32
                                         │         │ Pin 7 (EN) ─── VCC_3.3V
                                         └─────────┘
```

**Alternativa simplificada (pull-up en el primario):**

```
VCC_sensor ── [2.2 kΩ] ── Pin 2 (6N137 anode)
                                 │
LJ12A3 NPN OC ─────────── Pin 2 (se suma al pull-up)
LJ12A3 GND ────────────── Pin 3 (cathode)
```

Con VCC_sensor = 12 V y R = 2.2 kΩ: I_F ≈ (12 - 1.5) / 2200 ≈ 4.8 mA → dentro del
rango operativo del 6N137 (máximo 15 mA continuo). ✅

---

## 7. Riesgos evitados por señal aislada

### Módulo 1+2 — Encoder E6B2-CWZ6C (A, B, Z)

| Riesgo | Mecanismo | Consecuencia evitada |
|--------|-----------|---------------------|
| **Bucle de masa** | Diferencia de potencial entre GND del encoder (alimentado a 5/12 V) y GND STM32 → corriente circula por el cable de señal | Contador TIM2 se desvía → posición de dirección falsa → PID corrige en dirección incorrecta |
| **Ruido de conmutación PWM 20 kHz** | El motor de dirección es controlado por BTS7960 a 20 kHz; el encoder comparte entorno electromagnético | Glitches de ~100 ns en las líneas A/B → conteos extra en TIM2 → dirección oscilante |
| **Transitorios inductivos** | Conmutación del relé RELAY_DIR o del BTS7960 genera picos de +/- cientos de voltios en cables próximos | Daño permanente a GPIO del STM32 (tolerancia abs. máx: 4.0 V) |
| **Acoplamiento capacitivo** | Cables de señal paralelos a cables de potencia de 12 V del motor de dirección | Pulsos espúrios en flancos de conmutación del motor |
| **Recalibración espuria del Z** | Pulso falso en ENC_Z dispararía EXTI4, posiblemente usada en futuras implementaciones de re-homing | Error de posición absoluta acumulado → deriva de la dirección |

---

### Módulo 3+4 — Sensores de velocidad de rueda

| Riesgo | Mecanismo | Consecuencia evitada |
|--------|-----------|---------------------|
| **EMI de cables de potencia 24 V** | Cables de motor y sensor discurren paralelos en el chasis | Pulsos EXTI falsos → velocidad de rueda sobreestimada → ABS interviene sin patinaje real → freno incorrecto |
| **Bucle de masa en chasis** | Las ruedas tocan el suelo; la masa del sensor puede no coincidir exactamente con la masa del STM32 | Corrientes de modo común en el cable del sensor → EXTI activado espontáneamente |
| **Ráfagas de ruido EMI** | El debounce software de 1 ms no filtra una ráfaga de 5 pulsos en 200 µs (cada uno válido individualmente) | Conteo excesivo de pulsos → Wheel_pulse[] desbordado → velocidad calculada erróneamente → TCS limita tracción sin razón |
| **Picos de corriente de frenada regenerativa** | Si se implementa regen en el futuro, la di/dt en los cables de motor es más alta que en tracción normal | Ruido adicional en las líneas cercanas a las ruedas |

---

### Módulo 5 — Sensor de centrado de dirección

| Riesgo | Mecanismo | Consecuencia evitada |
|--------|-----------|---------------------|
| **Falsa detección de centro al arranque** | Pulso EMI durante el barrido de calibración → `SteeringCentering_Step()` detiene el motor y pone TIM2 a 0 en posición incorrecta | Dirección calibrada con offset → vehículo conduce torcido de forma permanente hasta el próximo arranque |
| **Detección de centro durante la conducción** | Si la señal de interrupción EXTI5 se dispara falsamente en marcha, el firmware (según implementación) podría reintentar la calibración | Fallo de seguridad potencial: motor de dirección se mueve inesperadamente al centro mientras el vehículo está en movimiento |
| **Daño al GPIO** | Sensor LJ12A3 operando a 12/24 V con cable cerca de bobinas de relé | Sobretensión transitoria → latch-up o daño permanente del GPIO PB5 |

---

## 8. Tabla resumen de aislamiento

### Señales AISLADAS (10 canales / 10 asignados)

| Módulo | Canal | Señal | Pin STM32 | Periférico HW | Lado ruidoso | Lado lógico | Motivo principal |
|--------|-------|-------|-----------|---------------|-------------|-------------|-----------------|
| **M1** | Ch-1 | ENC_A | PA15 | TIM2_CH1 cuadratura | E6B2 NPN 5–24 V | STM32 3.3 V | PID dirección, bucles de masa, EMI 20 kHz |
| **M1** | Ch-2 | ENC_B | PB3 | TIM2_CH2 cuadratura | E6B2 NPN 5–24 V | STM32 3.3 V | PID dirección, bucles de masa, EMI 20 kHz |
| **M2** | Ch-1 | ENC_Z | PB4 | EXTI4 | E6B2 NPN 5–24 V | STM32 3.3 V | Referencia de índice, picos inductivos |
| **M2** | Ch-2 | *(libre)* | — | — | — | — | Reservado para expansión |
| **M3** | Ch-1 | WHEEL_FL | PA0 | EXTI0 | LJ12A3 NPN 6–36 V, cubo rueda | STM32 3.3 V | ABS/TCS, EMI 24 V tracción, bucles de masa |
| **M3** | Ch-2 | WHEEL_FR | PA1 | EXTI1 | LJ12A3 NPN 6–36 V, cubo rueda | STM32 3.3 V | ABS/TCS, EMI 24 V tracción, bucles de masa |
| **M4** | Ch-1 | WHEEL_RL | PA2 | EXTI2 | LJ12A3 NPN 6–36 V, cubo rueda | STM32 3.3 V | ABS/TCS, EMI 24 V tracción, bucles de masa |
| **M4** | Ch-2 | WHEEL_RR | PB15 | EXTI15 | LJ12A3 NPN 6–36 V, cubo rueda | STM32 3.3 V | ABS/TCS, EMI 24 V tracción, bucles de masa |
| **M5** | Ch-1 | STEER_CENTER | PB5 | EXTI5 | LJ12A3 NPN 6–36 V, cremallera | STM32 3.3 V | Calibración arranque, picos inductivos |
| **M5** | Ch-2 | *(libre)* | — | — | — | — | Reservado para expansión |

### Señales NO AISLADAS (con justificación)

| Señal | Pins | Razón de exclusión | Solución alternativa si se requiere |
|-------|------|--------------------|-------------------------------------|
| CAN TX/RX | PB8/PB9 | Diferencial TJA1051T/3; 6N137 solo no basta (necesita DC-DC aislado) | ISO1050, ADuM1191 |
| I2C SCL/SDA | PB6/PB7 | Protocolo bidireccional open-drain, incompatible con 6N137 | ADUM1250, ISO1540 |
| OneWire | PB0 | Protocolo bidireccional, tensión 3.3 V, baja criticidad | — |
| Pedal ADC | PA3 | Señal analógica; 6N137 es digital | AMC1200, ISO124 |
| PWM/DIR/EN | PA8–PA11, PC0–PC9, PC13 | 15 canales necesarios; excede presupuesto; BTS7960 tiene protección interna | Módulos adicionales, iteración futura |
| Relés | PC10–PC12 | Salidas con driver de transistor + flyback; sin camino de retorno de ruido | — |

---

### Garantía de no-rotura de timings ni periféricos

| Verificación | Resultado |
|-------------|-----------|
| TIM2 cuadratura + 6N137 (t_prop 120 ns vs. periodo 50 µs mínimo) | ✅ Compatible — retardo < 0.24 % del periodo |
| Filtro digital TIM2 (282 ns) vs. propagación 6N137 (120 ns) | ✅ Sin falsos flancos |
| EXTI sensores rueda (periodo mínimo 6.6 ms) vs. propagación 120 ns | ✅ Sin impacto |
| EXTI centro dirección (pulso único, baja frecuencia) | ✅ Sin impacto |
| IWDG, SWD, arranque, bootloader | ✅ No afectados (señales internas al MCU) |
| CAN 500 kbps (no aislado) | ✅ TJA1051T/3 existente |
| I2C 400 kHz (no aislado) | ✅ Sin cambios |

---

> **Estado del firmware:** NINGÚN archivo de código fuente ha sido modificado.
> Este documento es exclusivamente el plan de aislamiento hardware.
> La implementación física (soldadura, cableado) no requiere cambios de firmware.

---

**Fecha:** 2026-02-25
**Basado en auditoría de:** `main.h`, `main.c`, `sensor_manager.c`, `encoder_reader.c`,
`motor_control.c`, `HARDWARE_WIRING_MANUAL.md`, `PINOUT_DEFINITIVO.md`, `HARDWARE.md`
