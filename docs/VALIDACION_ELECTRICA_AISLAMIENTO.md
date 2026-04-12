# VALIDACIÓN ELÉCTRICA DEL PLAN DE AISLAMIENTO

**Sistema de Control Vehicular — STM32G474RE**
**Documento base:** `CABLEADO_AISLAMIENTO_DEFINITIVO.md`
**Estado: Validación antes del montaje — Firmware SIN modificar**

---

## Índice

1. [Topología CAN — diagrama lógico y nodos](#1-topología-can--diagrama-lógico-y-nodos)
2. [Canal Z del encoder con PC817 — análisis de pulso](#2-canal-z-del-encoder-con-pc817--análisis-de-pulso)
3. [Pull-ups y niveles lógicos por señal](#3-pull-ups-y-niveles-lógicos-por-señal)
4. [Dominios de masa y alimentación](#4-dominios-de-masa-y-alimentación)
5. [Resumen: puntos de acción antes del montaje](#5-resumen-puntos-de-acción-antes-del-montaje)

---

## 1. Topología CAN — diagrama lógico y nodos

### 1.1 Arquitectura CAN actual (sin aislamiento)

El documento `ESP32_STM32_CAN_CONNECTION.md` define la topología existente: dos nodos,
dos transceivers TJA1051T/3, bus diferencial compartido.

```
Dominio STM32                 Bus CAN físico          Dominio ESP32
──────────────                ──────────────          ─────────────

STM32G474RE                                           ESP32-S3
  PA12 (FDCAN1_TX) ──► TJA1051T/3_A                   TJA1051T/3_B ◄── GPIO4 (TX)
  PA11 (FDCAN1_RX) ◄── TJA1051T/3_A ──CANH/CANL──── TJA1051T/3_B ──► GPIO5 (RX)

  GND_STM32 ──────────► GND_A                         GND_B ◄── GND_ESP32
  5V_sistema ─────────► VCC_A                         VCC_B ◄── 5V_sistema

  [120 Ω entre CANH y CANL                    120 Ω entre CANH y CANL]
  (extremo A del bus)                         (extremo B del bus)
```

Esta topología es correcta para CAN 2.0A: **2 nodos = 2 transceivers = válido**.
No existe "doble transceiver" problemático: cada transceiver pertenece a un nodo
lógico diferente. El bus tiene exactamente 2 terminaciones de 120 Ω (una en cada extremo).

---

### 1.2 Quién es el nodo principal y cuál es el esclavo

En CAN 2.0A **no existe un nodo maestro físico**: todos los nodos compiten por el bus
con igual derecho de arbitraje (prioridad inversa al ID). Sin embargo, en este sistema
existe una asimetría funcional importante:

| Rol funcional | STM32G474RE | ESP32-S3 |
|---------------|------------|----------|
| **Autoridad de seguridad** | ✅ Sí — valida todos los comandos, controla actuadores | ❌ No |
| **Generador de comandos** | ❌ No | ✅ Sí — envía throttle, steering, mode |
| **Prioridad CAN (ID más bajo = mayor prioridad)** | 0x001 (HEARTBEAT_STM32) | 0x011 (HEARTBEAT_ESP32) |
| **Filtros RX** | Solo acepta IDs ESP32 válidos | Acepta estado STM32 |
| **Timeout de seguridad** | Si no recibe ESP32 en 250 ms → LIMP_HOME | Muestra alerta |

El **STM32 es la autoridad de seguridad y tiene la menor dirección CAN** (0x001).
El **ESP32 es el nodo de control de usuario** con direcciones más altas (0x011, 0x100–0x120).
Ambos son **pares en el bus físico**: no hay gateway, solo un bus lineal.

---

### 1.3 Topología CAN con aislamiento propuesto

El aislador digital se coloca **entre el STM32 y su TJA1051T/3**, no entre los dos
transceivers. El número de transceivers en el bus permanece en 2 (uno por nodo físico).

```
Dominio GND_STM32                  │ Barrera │    Dominio GND_CAN (= GND_ESP32)
──────────────────────────────────  │         │    ──────────────────────────────────

STM32G474RE (3.3 V)                │         │
  PA12 (FDCAN1_TX) ──► [Aislador]──►│~~~~~~~~~│──► TXD TJA1051T/3_A ──►┐
  PA11 (FDCAN1_RX) ◄── [Aislador]◄──│~~~~~~~~~│◄── RXD TJA1051T/3_A   │   CANH ═══ CANH ─── TJA1051T/3_B ─── ESP32
                                   │         │         VCC ◄── 5V_iso  │   CANL ═══ CANL ─── TJA1051T/3_B
  GND_STM32                        │  DC-DC  │         GND ─► GND_CAN  │
  3.3V_STM32 ──► Aislador VDD_1   │ aislado │         VCC ◄── 5V_iso ◄┘
                                   │ (5V_iso)│                           GND_CAN ──── GND_ESP32
                                   │         │   [120 Ω en este extremo]      [120 Ω en extremo ESP32]
```

#### Resumen de la topología propuesta

| Elemento | Posición | Dominio de masa |
|----------|---------|-----------------|
| STM32 FDCAN1 (lógica) | Entre STM32 y barrera | GND_STM32 |
| Aislador digital (ej. ADuM1201) | Barrera galvánica | Cruza GND_STM32 / GND_CAN |
| TJA1051T/3_A (nodo STM32) | Entre barrera y bus | GND_CAN |
| DC-DC aislado (≥200 mA) | Alimenta TJA1051T/3_A | GND_STM32 → 5V_iso en GND_CAN |
| Bus CANH/CANL | Cable diferencial | GND_CAN |
| TJA1051T/3_B (nodo ESP32) | Bus → ESP32 | GND_ESP32 (= GND_CAN) |
| Resistencias 120 Ω | Extremos físicos del bus | GND_CAN |

#### ¿Hay problema de doble transceiver?

**No.** El bus tiene exactamente 2 nodos lógicos (STM32 y ESP32), cada uno con su propio
transceiver. El aislador no añade un tercer transceiver: está entre el FDCAN1 del STM32
y su TJA1051T/3 dedicado. Las terminaciones (120 Ω) se mantienen en los extremos físicos
del cable, independientemente de la barrera galvánica.

#### ¿Puede haber un gateway en el futuro?

Si en el futuro se añadieran más nodos CAN (p. ej., un VCU externo, un BMS, un display
de carrocería), el bus lineal se ampliaría añadiendo más nodos con sus propios
transceivers. No se necesita gateway para los 2 nodos actuales.

---

### 1.4 Impacto del aislador digital en el timing CAN

```
Bit period @ 500 kbps = 2 µs = 2 000 ns

Retardo del aislador digital (ej. ADuM1201): 10–50 ns typical
Retardo del aislador digital (ej. Si8621):   9–15 ns typical

Porcentaje del bit period:
  50 ns / 2 000 ns = 2.5 %  → dentro del margen de sampling point

Configuración FDCAN1 (MX_FDCAN1_Init):
  Prescaler = 17 → f_tq = 170 MHz / 17 = 10 MHz → t_tq = 100 ns
  NominalTimeSeg1 = 14 → Sample point = (1+14)/(1+14+5) = 75 %
  NominalTimeSeg2 = 5  → 5 × 100 ns = 500 ns de margen tras sample
  SyncJumpWidth = 1 → tolerancia ±100 ns

Un aislador digital de 50 ns desplaza el sampling point en 0.5 TQ.
El margen del TQ es 100 ns. 50 ns < 100 ns → completamente dentro del margen. ✅
```

El timing CAN no se ve afectado por ningún aislador digital disponible en el mercado.

---

## 2. Canal Z del encoder con PC817 — análisis de pulso

### 2.1 Estado actual del firmware

El canal Z está **inactivo en el firmware actual**. Confirmado desde el código:

- `main.h` línea 62: `#define PIN_ENC_Z GPIO_PIN_4` — definido pero no usado en GPIO Init
- `MX_GPIO_Init()` en `main.c`: **no inicializa PB4** (ni como EXTI ni como GPIO de entrada)
- `stm32g4xx_it.c`: **no existe `EXTI4_IRQHandler`**
- `steering_centering.c`: el centrado usa `SteeringCenter_Detected()` (PB5/EXTI5, sensor
  inductivo LJ12A3), **no usa el canal Z del encoder en absoluto**

El cableado del PC817 en el canal A6 de la Placa-A puede instalarse ahora sin consecuencias.

---

### 2.2 Análisis de anchura de pulso con PC817

La preocupación es si el retardo de propagación del PC817 (4–6 µs) distorsiona
significativamente la anchura del pulso Z.

**Cálculo de anchura mínima del pulso Z:**

El pulso Z dura exactamente **1 cuenta de encoder** (equivale a una ventana de un solo
bit del contador TIM2 en cuadratura). La anchura temporal de ese pulso depende de la
velocidad angular:

```
Anchura_Z = (1 cuenta / 4800 cuentas/rev) × (60 s / RPM)
           = 60 / (4800 × RPM)  segundos
```

| RPM del eje encoder | Anchura pulso Z | Retardo PC817 | Porcentaje distorsión |
|--------------------|-----------------|----------------|----------------------|
| 300 RPM (máx. práctico) | 41.7 µs | 4–6 µs | 10–14 % |
| 100 RPM | 125 µs | 4–6 µs | 3.2–4.8 % |
| 10 RPM | 1 250 µs | 4–6 µs | 0.32–0.48 % |
| 1 RPM | 12 500 µs | 4–6 µs | 0.032–0.048 % |

**Conclusión de la anchura:** El PC817 introduce una distorsión de timing en el peor caso
del 14 % a 300 RPM. Dado que el canal Z se usa únicamente para marcar una posición de
referencia (un pulso por vuelta), **no hay problema de frecuencia** (≤ 5 Hz a 300 RPM).
La distorsión del pulso no es un problema porque el software solo necesita detectar el
**flanco de subida** (RISING), no la anchura exacta del pulso.

---

### 2.3 ¿Necesita el firmware una ventana de aceptación A/B para el canal Z?

Esta es la pregunta funcional clave: ¿debe el firmware verificar que el flanco EXTI4 del
canal Z llega mientras el contador TIM2 está en una posición esperada?

#### Análisis: causas de disparo falso de EXTI4

Con el PC817 instalado, un disparo falso de EXTI4 podría ocurrir por:
- Ruido en el cable del encoder que supere el umbral del PC817 (mucho menos probable con
  el optoacoplador que sin él, que es precisamente el objetivo del aislamiento)
- Transitorio inductivo del relé RELAY_DIR que acopla al cable del encoder

**Sin aislamiento**: cualquier transitorio de más de ~1 V en el cable llega directamente
al GPIO PB4.
**Con PC817**: el transitorio tendría que vencer la barrera del optoacoplador (V_LED_min
≈ 1.5 V, corriente mínima ≈ 1 mA) durante al menos ~5 µs. Esto es una protección
significativa; los transitorios de corta duración no atraviesan el PC817.

#### ¿La ventana A/B es obligatoria por el PC817?

**No.** El PC817 no exige por sí mismo una ventana de aceptación:
- El retardo de propagación (4–6 µs) es constante y predecible
- A la velocidad del barrido de calibración (~10 % del ciclo PWM del motor de dirección),
  el encoder avanza muy lentamente. En 6 µs de retardo, el avance de posición es:

```
Velocidad barrido estimada: RPM_encoder ≈ 10 % de la relación de transmisión
Si el motor gira a 100 RPM en la salida del BTS7960 y la reductora es 1:20:
  RPM_encoder ≈ 100 / 20 = 5 RPM

Posición avanzada en 6 µs:
  = (5 RPM / 60 s/min) × (1 rev / 4800 cuentas)^-1 × 6e-6 s
  = (5/60) × 4800 × 6e-6  = 0.0024 cuentas  → completamente negligible
```

El pulso Z llega al STM32 exactamente 4–6 µs después del evento mecánico. En ese tiempo,
el contador TIM2 no habrá avanzado ni un solo conteo. **No se pierde referencia.**

#### ¿Es recomendable una ventana A/B de todas formas?

Sí, como **defensa en profundidad** contra ruido imprevisto (no porque el PC817 lo exija):

| Sin ventana A/B | Con ventana A/B |
|----------------|----------------|
| Cualquier flanco EXTI4 reinicia el contador a 0 | El flanco EXTI4 solo se acepta si TIM2 está en [posición_Z - δ, posición_Z + δ] |
| Un pulso falso inesperado provoca deriva permanente de la posición | Un pulso falso fuera de la ventana es descartado silenciosamente |
| Seguridad dependiente solo del PC817 | Doble capa: PC817 + validación software |

**Recomendación funcional (para cuando se active Z en firmware):**
Implementar una ventana de aceptación de ±10 cuentas alrededor de la posición Z esperada.
La posición Z se aprende en el primer arranque y se persiste (junto al valor de centrado).

> **⚠️ Este cambio no es urgente ni está relacionado con el PC817.** Es una mejora de
> robustez general recomendable para cualquier uso de encoder Z, con cualquier tipo de
> aislamiento. No modifica el plan de cableado. Se documenta aquí para el diseño futuro
> del firmware Z.

---

## 3. Pull-ups y niveles lógicos por señal

### 3.1 Principios de funcionamiento de los optoacopladores (salida)

**6N137** — salida open-collector activa baja:
- LED ON (señal en primario activa) → transistor Q2 conduce → Vo = GND (LOW)
- LED OFF (señal en primario inactiva) → transistor Q2 corta → Vo = HIGH (via pull-up externo)

**PC817** — salida open-collector NPN:
- LED ON (señal en primario activa) → transistor conduce → colector = GND (LOW)
- LED OFF (señal en primario inactiva) → transistor corta → colector = HIGH (via pull-up)

En ambos casos: **señal primaria HIGH → salida secundaria HIGH** (si hay pull-up externo).
El optoacoplador conserva la polaridad de la señal digital. ✅

---

### 3.2 Circuito primario (lado ruidoso): cómo los sensores accionan el LED

Todos los sensores son **NPN de colector abierto (NPN OC)**. El circuito primario es:

```
VCC_sensor ──[R_entrada]──[Ánodo LED]──[Cátodo LED]──[Sensor NPN OC colector]── GND_sensor
                                                                  ↑
                                               Emisor conectado a GND_sensor
```

- Sensor inactivo (no detecta): transistor del sensor OFF → sin corriente → LED OFF → salida HIGH
- Sensor activo (detecta): transistor del sensor ON, jala a GND → corriente fluye → LED ON → salida LOW

Transición: inactivo → activo = flanco **FALLING** en la salida del optoacoplador
Transición: activo → inactivo = flanco **RISING** en la salida del optoacoplador

**El firmware configura `GPIO_MODE_IT_RISING`** para todos los sensores de velocidad de
rueda y el sensor de centrado. Esto significa que la ISR se activa cuando el sensor
**deja de detectar** (flanco ascendente). Para sensores NPN OC de paso, esto ocurre
cuando el tornillo/diente del engranaje deja de estar frente al sensor, marcando el
límite posterior del pulso. Este es el comportamiento habitual con sensores inductivos
de proximidad: el flanco ascendente marca el instante de salida del objeto detectado.
La dirección convencional es consistente con el firmware existente. ✅

---

### 3.3 Tabla completa de pull-ups y niveles lógicos

| # | Señal | Pin STM32 | Aislador | Pull-up lado primario | Pull-up lado secundario | Nivel reposo (sensor OFF) | Nivel activo (sensor ON) | Polaridad detectada por firmware |
|---|-------|-----------|----------|-----------------------|------------------------|--------------------------|--------------------------|----------------------------------|
| 1 | **ENC_A** | PA15 / TIM2_CH1 | 6N137 | Pull-up interno del encoder (5–12 V a VCC_encoder, ~1 kΩ) | **4.7 kΩ externo a 3.3 V** (OBLIGATORIO — `GPIO_NOPULL` en MSP) | HIGH (3.3 V) | LOW (≈0 V) | Flanco RISING y FALLING (TIM2 cuadratura — ambos flancos) |
| 2 | **ENC_B** | PB3 / TIM2_CH2 | 6N137 | Pull-up interno del encoder (5–12 V a VCC_encoder, ~1 kΩ) | **4.7 kΩ externo a 3.3 V** (OBLIGATORIO — `GPIO_NOPULL` en MSP) | HIGH (3.3 V) | LOW (≈0 V) | Flanco RISING y FALLING (TIM2 cuadratura — ambos flancos) |
| 3 | **WHEEL_FL** | PA0 / EXTI0 | PC817 | Pull-up en VCC_sensor (resistencia en placa PC817, ~1–2.2 kΩ) | ~40 kΩ interno STM32 (`GPIO_PULLUP`) | HIGH (3.3 V) | LOW (≈0 V) | `IT_RISING` → disparo cuando sensor deja de detectar |
| 4 | **WHEEL_FR** | PA1 / EXTI1 | PC817 | Pull-up en VCC_sensor (resistencia en placa PC817) | ~40 kΩ interno STM32 (`GPIO_PULLUP`) | HIGH (3.3 V) | LOW (≈0 V) | `IT_RISING` → disparo cuando sensor deja de detectar |
| 5 | **WHEEL_RL** | PA2 / EXTI2 | PC817 | Pull-up en VCC_sensor (resistencia en placa PC817) | ~40 kΩ interno STM32 (`GPIO_PULLUP`) | HIGH (3.3 V) | LOW (≈0 V) | `IT_RISING` → disparo cuando sensor deja de detectar |
| 6 | **WHEEL_RR** | PB15 / EXTI15 | PC817 | Pull-up en VCC_sensor (resistencia en placa PC817) | ~40 kΩ interno STM32 (`GPIO_PULLUP`) | HIGH (3.3 V) | LOW (≈0 V) | `IT_RISING` → disparo cuando sensor deja de detectar |
| 7 | **STEER_CENTER** | PB5 / EXTI5 | PC817 | Pull-up en VCC_sensor (resistencia en placa PC817) | ~40 kΩ interno STM32 (`GPIO_PULLUP`) | HIGH (3.3 V) | LOW (≈0 V) | `IT_RISING` → disparo cuando sensor deja de detectar (= el tornillo abandona la zona de detección) |
| 8 | **ENC_Z** *(inactivo)* | PB4 / EXTI4 | PC817 | Pull-up en VCC_encoder (resistencia en placa PC817) | 40 kΩ interno (cuando se configure `GPIO_PULLUP`) | HIGH (3.3 V) | LOW (≈0 V) | `IT_RISING` (cuando se active en firmware) |

**Notas críticas:**

> ⚠️ **ENC_A y ENC_B: pull-up externo OBLIGATORIO.**
> El firmware configura PA15 y PB3 con `GPIO_NOPULL` (ver `stm32g4xx_hal_msp.c` línea 157
> y 163). Sin pull-up externo, la salida open-collector del 6N137 flota en estado
> de alta impedancia cuando el LED está apagado. El TIM2 leerá un nivel indeterminado,
> causando conteos espúrios.
>
> **Valor recomendado:** 4.7 kΩ entre el pin Vo del 6N137 (pin 6) y la línea de 3.3 V.
> Corriente en estado HIGH: I = (3.3 V - 0) / 4700 Ω = 0.70 mA → dentro de los límites
> del 6N137 (I_CE max = 8 mA a Vcc = 5 V). ✅

> ℹ️ **EXTI con GPIO_PULLUP (PC817):**
> El pull-up interno del STM32 (~40 kΩ) es suficiente para la salida open-collector del
> PC817. Cuando el PC817 conduce (LED ON, sensor activo), el colector se satura a
> V_CE_sat ≈ 0.1–0.2 V a I_C ≈ 80 µA → bien por debajo de VIL_max = 0.99 V. ✅

---

### 3.4 Verificación del nivel lógico del 6N137 con pull-up de 4.7 kΩ

```
Estado HIGH (LED OFF, TIM2 debe leer HIGH):
  Vo abierto → pull-up 4.7 kΩ → 3.3 V
  VIH_min STM32 = 0.7 × VDD = 0.7 × 3.3 = 2.31 V
  Nivel leído: 3.3 V >> 2.31 V → HIGH válido ✅

Estado LOW (LED ON, TIM2 debe leer LOW):
  Q2 interno conduce: I_C = (3.3 V - V_CE_sat) / 4700 Ω = (3.3 - 0.1) / 4700 ≈ 0.68 mA
  V_CE_sat del 6N137 a 0.68 mA ≈ 0.05–0.1 V
  VIL_max STM32 = 0.3 × VDD = 0.3 × 3.3 = 0.99 V
  Nivel leído: 0.1 V << 0.99 V → LOW válido ✅
```

---

### 3.5 Verificación del nivel lógico del PC817 con pull-up interno de 40 kΩ

```
Estado HIGH (LED OFF, EXTI en reposo, sensor inactivo):
  Colector abierto → pull-up interno 40 kΩ → 3.3 V
  VIH_min STM32 = 2.31 V
  Nivel leído: 3.3 V >> 2.31 V → HIGH válido ✅

Estado LOW (LED ON, sensor activo, antes del RISING que activa la ISR):
  I_C = (3.3 V - V_CE_sat) / 40 000 Ω ≈ (3.3 - 0.1) / 40 000 ≈ 80 µA
  V_CE_sat del PC817 a 80 µA ≈ 0.05–0.1 V
  VIL_max STM32 = 0.99 V
  Nivel leído: 0.1 V << 0.99 V → LOW válido ✅
```

---

## 4. Dominios de masa y alimentación

### 4.1 Mapa de dominios de masa del sistema

El sistema tiene **6 dominios de masa diferenciables**. La barrera galvánica del
aislamiento separa dominios que no deben tener bucles de corriente entre ellos.

```
┌──────────────────────────────────────────────────────────────────────────────────┐
│                         MAPA DE DOMINIOS DE MASA                                  │
├─────────────────┬────────────────────────────────────────────────────────────────┤
│ Dominio         │ Componentes incluidos                                           │
├─────────────────┼────────────────────────────────────────────────────────────────┤
│ GND_STM32       │ STM32G474RE, condensadores bypass (100 nF en VDD/VDDA),        │
│                 │ regulador 3.3 V, SWD connector, pads de prueba lógicos         │
├─────────────────┼────────────────────────────────────────────────────────────────┤
│ GND_encoder     │ E6B2-CWZ6C: circuito interno del encoder, blindaje del cable   │
│                 │ (si se usa cable apantallado), alimentación 5–12 V del encoder  │
│                 │ Puede conectarse a GND_chasis (depende de la fuente de encoder) │
├─────────────────┼────────────────────────────────────────────────────────────────┤
│ GND_sensores    │ LJ12A3 × 5 (WHEEL_FL/FR/RL/RR + STEER_CENTER)                 │
│                 │ Cuerpo metálico del sensor = GND_chasis (montados en chasis)   │
│                 │ Este dominio ES GND_chasis en la práctica                       │
├─────────────────┼────────────────────────────────────────────────────────────────┤
│ GND_chasis      │ Bastidor metálico del vehículo, cubos de ruedas, cajas de      │
│                 │ motor, terminal negativo de la batería 24 V, GND retorno de    │
│                 │ los motores de tracción y dirección                             │
├─────────────────┼────────────────────────────────────────────────────────────────┤
│ GND_CAN         │ TJA1051T/3_A (lado STM32 del bus, si se aísla el CAN),         │
│                 │ alimentado por DC-DC aislado. Conectar al GND_chasis en        │
│                 │ un punto único para referencia CAN de modo común.              │
├─────────────────┼────────────────────────────────────────────────────────────────┤
│ GND_ESP32       │ ESP32-S3, TJA1051T/3_B, LEDs WS2812B (alimentados por STM32   │
│                 │ relés PB10/PB11), display TFT, DFPlayer Mini.                  │
│                 │ En la arquitectura propuesta: GND_ESP32 = GND_CAN              │
└─────────────────┴────────────────────────────────────────────────────────────────┘
```

---

### 4.2 Dónde se unen los dominios de masa (y dónde NO)

#### Uniones CORRECTAS (punto estrella)

```
GND_STM32
    │
    └─── Punto de estrella único (barra de cobre en placa de control)
              │
              ├─── GND_chasis (a través de un solo cable corto de sección suficiente)
              │    (Este es el único puente entre el dominio lógico y el dominio de potencia)
              │
              └─── (Ninguna otra conexión directa a GND_chasis desde GND_STM32)

GND_CAN = GND_ESP32
    │
    └─── Conectar a GND_chasis en un punto diferente del punto de estrella
         (el GND de referencia del bus CAN debe ir a chasis para estabilidad de modo común)
```

#### Uniones PROHIBIDAS

| Conexión prohibida | Motivo |
|-------------------|--------|
| GND_encoder ↔ GND_STM32 (en el mismo lado) | Anula la barrera galvánica del 6N137 |
| GND_sensores ↔ GND_STM32 (en el mismo lado) | Anula la barrera galvánica del PC817 |
| GND_encoder y GND_sensores conectados a GND_STM32 en **dos puntos distintos** | Crea bucle de masa que puede conducir la corriente de ruido |
| GND_CAN ↔ GND_STM32 (mismo potencial) | Anula la barrera del aislador digital CAN |

---

### 4.3 Diagrama de separación de dominios

```
DOMINIOS AISLADOS (barrera mediante optoacoplador/aislador)
═══════════════════════════════════════════════════════════

  LADO RUIDOSO                    BARRERA              LADO LÓGICO PROTEGIDO
  ─────────────────                ───────               ──────────────────────

  GND_encoder   ─────────────► [6N137 M1] ──────────────► GND_STM32
  (encoder 5–12V)               (A y B)                   (STM32 3.3V)

  GND_sensores  ─────────────► [PC817     ──────────────► GND_STM32
  (= GND_chasis)                Placa-A]                  (STM32 3.3V)
  LJ12A3 × 5                   (6 canales)

  GND_CAN       ─────────────► [Aislador  ──────────────► GND_STM32
  (= GND_ESP32)                 digital]                  (STM32 3.3V)
  TJA1051T/3_A                 (CAN TX/RX)

DOMINIO UNIFICADO (sin barrera — compartido por diseño)
═══════════════════════════════════════════════════════════

  GND_CAN = GND_ESP32 = GND_chasis_CAN (todos al mismo potencial de referencia CAN)
  GND_STM32 ─── [punto de estrella] ─── GND_chasis (UN SOLO PUNTO de conexión)
```

---

### 4.4 Fuentes de alimentación que requieren DC-DC aislado

| Componente | Requiere DC-DC aislado | Motivo | Tensión salida |
|------------|------------------------|--------|----------------|
| **TJA1051T/3_A** (lado STM32, si se aísla CAN) | **SÍ** | Necesita 5 V en GND_CAN, que es galvánicamente diferente de 5V_STM32 | 5 V / ≥ 200 mA |
| 6N137 M1 (lado primario) | **No** | Se alimenta de VCC_encoder (ya existente, misma fuente que el encoder) | VCC_encoder (5–12 V) |
| PC817 Placa-A (lado primario) | **No** | Se alimenta de VCC_sensor (misma fuente que los LJ12A3) | VCC_sensor (6–36 V) |
| 6N137 M1 (lado secundario) | **No** | Se alimenta de 3.3 V del STM32 | 3.3 V (GND_STM32) |
| PC817 Placa-A (lado secundario) | **No** | Se alimenta de 3.3 V del STM32 | 3.3 V (GND_STM32) |

#### Especificación del DC-DC aislado para CAN

| Parámetro | Requisito | Opciones comerciales |
|-----------|-----------|---------------------|
| Tensión entrada | 3.3 V o 5 V (GND_STM32) | — |
| Tensión salida | 5 V (para TJA1051T/3_A) | — |
| Corriente salida | ≥ 200 mA (TJA1051T/3 consume < 70 mA) | — |
| Aislamiento | ≥ 1 000 V rms | RECOM R-78E5.0-1.0 (no aislado — NO válido), RECOM RxxP5.0S, Murata MEE1S0505SC |
| Footprint | Pequeño (módulo SIP-7 o DIP) | RECOM RPM5.0-1.0 (5V/1W), Tracopower TME 0505S (5V/1W) |

**Alternativa (más simple):** Si se utiliza el chip **ISO1050** de Texas Instruments, este
integra el transceiver CAN **más** el aislamiento galvánico en un solo encapsulado, eliminando
la necesidad del DC-DC separado (el ISO1050 incluye su propio convertidor interno).

---

### 4.5 Diagrama de alimentación por dominio

```
Batería 24 V ─── Terminal negativo ─────────────────────── GND_chasis
                │
                ├──► Relé MAIN → BTS7960 × 5 → Motores (GND_potencia = GND_chasis)
                │
                ├──► DC-DC 24V → 5V (para lógica 5V general) ─── GND_sistema
                │        │
                │        ├──► AMS1117-3.3 → 3.3V ─────────────── GND_STM32
                │        │
                │        ├──► TJA1051T/3_B (ESP32) ──────────── GND_ESP32
                │        │
                │        └──► DC-DC aislado → 5V_iso ─────────── GND_CAN
                │                                                 (≠ GND_STM32)
                │
                └──► VCC_encoder (5V o 12V según encoder) ─────── GND_encoder
                     (alimenta E6B2-CWZ6C y primario de 6N137)

VCC_sensor (del 12V o 24V del vehículo):
  → Alimenta primario de PC817 Placa-A (a través de resistencias de la placa)
  → GND_sensor = GND_chasis (sensores montados en chasis)
```

---

### 4.6 Reglas de cableado para evitar bucles de masa no intencionados

1. **Una sola conexión GND_STM32 → GND_chasis.** Si hay más de una, existe un bucle de
   masa. Usar barra de cobre y punto de estrella único.

2. **El blindaje del cable del encoder** (si se usa cable apantallado) debe conectarse
   a GND_encoder (primario) en UN solo extremo (el del encoder), NO en el extremo del
   módulo 6N137. Conectarlo en ambos extremos forma un bucle.

3. **Los cables de señal de los sensores LJ12A3** deben discurrir alejados (> 10 cm)
   de los cables de potencia de los motores. Si se usan en el mismo canalón, usar
   cables apantallados con pantalla conectada a GND_chasis en un solo extremo.

4. **El GND_CAN** debe estar a GND_chasis (único punto). Conectar la carcasa del
   conector CAN a GND_chasis. Los TJA1051T/3 tienen sus GND a GND_chasis a través del
   retorno de sus fuentes de alimentación.

5. **No conectar la pantalla del bus CAN** a GND_STM32 directamente. Si se usa cable
   apantallado para CAN, conectar la pantalla a GND_chasis solo en un extremo.

---

## 5. Resumen: puntos de acción antes del montaje

### Acciones de hardware OBLIGATORIAS

| # | Acción | Señal afectada | Sin esto... |
|---|--------|---------------|-------------|
| **H1** | Añadir 2 × resistencias 4.7 kΩ entre pin 6 del 6N137 y 3.3 V (externas a la placa STM32) | ENC_A (PA15), ENC_B (PB3) | La entrada del TIM2 flota → conteos aleatorios → pérdida de control de dirección |
| **H2** | Confirmar valor de R_entrada del 6N137 según VCC del encoder (330 Ω a 5V, 1 kΩ a 12V) | ENC_A, ENC_B | I_F < mínimo → LED débil → transiciones lentas o sin detección |
| **H3** | Confirmar resistencia de la placa PC817 es adecuada para VCC_sensor real (ver Tabla 7.2 del plan de cableado) | WHEEL_FL/FR/RL/RR, STEER_CENTER, ENC_Z | I_F excesivo si R demasiado pequeña (destrucción del LED), sin señal si R demasiado grande |
| **H4** | Instalar DC-DC aislado (5V, ≥200 mA, ≥1000V aislamiento) si se decide aislar el CAN | CAN TX/RX | Sin alimentación aislada no hay barrera real para el transceiver |
| **H5** | Conectar GND_encoder a GND_primario del 6N137 ÚNICAMENTE (no a GND_STM32 en ningún punto) | ENC_A, ENC_B | La barrera galvánica queda cortocircuitada |
| **H6** | Conectar GND_sensor a GND_primario del PC817 ÚNICAMENTE (no a GND_STM32) | WHEEL × 4, STEER_CENTER | La barrera galvánica queda cortocircuitada |
| **H7** | Verificar con óhmetro (sistema desenergizado) que GND_STM32 y GND_encoder están aislados entre sí antes de encender | ENC_A, ENC_B | Riesgo de daño si hay cortocircuito no detectado |
| **H8** | Verificar con óhmetro que GND_STM32 y GND_sensor están aislados entre sí antes de encender | WHEEL × 4, STEER_CENTER | Ídem |

### Acciones de verificación FUNCIONAL antes de primer arranque

| # | Verificación | Método | Resultado esperado |
|---|-------------|--------|-------------------|
| **V1** | Nivel en reposo PA15 y PB3 | Multímetro en PA15/PB3 con encoder parado | 3.3 V (HIGH) vía pull-up de 4.7 kΩ |
| **V2** | Nivel activo PA15 y PB3 | Girar el encoder lentamente con osciloscopio | Onda cuadrada 0–3.3 V, duty ≈ 50 % |
| **V3** | Nivel en reposo PA0/PA1/PA2/PB15/PB5 | Multímetro con sensores alejados del objeto | 3.3 V (HIGH) |
| **V4** | Nivel activo PA0 | Acercar objeto metálico al sensor WHEEL_FL | Pulso LOW en PA0, EXTI0 dispara Wheel_FL_IRQHandler |
| **V5** | Nivel activo PB5 | Acercar tornillo al sensor STEER_CENTER | Pulso LOW en PB5, `steer_center_flag = 1` |
| **V6** | Bucle CAN sin aislamiento | Conectar STM32 + ESP32 sin aislar primero | Heartbeat recibido a 10 Hz, sin errores CAN |
| **V7** | Aislamiento CAN (si aplica) | Repetir V6 con aislador instalado | Mismo resultado — aislador es transparente al protocolo |
| **V8** | Verificar ≥ 60 Ω entre CANH y CANL antes de encender | Óhmetro entre CANH y CANL del bus | 60 Ω = dos terminaciones 120 Ω en paralelo ✅ |

### Notas sobre firmware (para iteración futura — no bloquea el montaje)

| # | Nota | Señal | Impacto si no se hace |
|---|------|-------|----------------------|
| **F1** | Activar ENC_Z cuando se decida usar: init PB4 como EXTI4 + ISR | ENC_Z (PB4) | El canal Z está cableado pero inactivo. Sin impacto actual. |
| **F2** | Implementar ventana A/B de ±10 cuentas para validar el pulso Z | ENC_Z | Sin la ventana, un pulso falso raro puede resetear el contador. Mitigado por PC817. |

---

> **Estado del firmware:** NINGÚN archivo de código fuente ha sido modificado.
> Esta validación es exclusivamente del hardware. El firmware actual es compatible
> con el plan de aislamiento sin cambios.

---

**Fecha:** 2026-02-25
**Basado en auditoría de:**
- `Core/Src/main.c` — MX_GPIO_Init(), MX_TIM2_Init(), MX_FDCAN1_Init()
- `Core/Src/stm32g4xx_hal_msp.c` — HAL_TIM_Encoder_MspInit() (GPIO_NOPULL en PA15/PB3)
- `Core/Src/stm32g4xx_it.c` — ausencia de EXTI4_IRQHandler (Z inactivo)
- `Core/Inc/main.h` — PIN_ENC_Z = PB4 (definido, no usado)
- `Core/Src/steering_centering.c` — usa STEER_CENTER (PB5), no Z
- `docs/ESP32_STM32_CAN_CONNECTION.md` — topología CAN, 2 × TJA1051T/3
- `docs/CABLEADO_AISLAMIENTO_DEFINITIVO.md` — plan de cableado base
