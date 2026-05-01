# REFERENCIA DE CABLEADO — Módulos EL817 de 4 Canales (×2)
## Sistema de Control Vehicular — STM32G474RE + ESP32-S3
### Sustituye al módulo HY-M158 PC817 de 8 canales para los sensores y la llave

> **Versión:** 1.0 — 2026-05-01
> **Fuente de verdad:** `project_config.h`, `main.c` (MX_GPIO_Init), `sensor_manager.c`
> **Hardware:** 2× placa EL817 de 4 canales (EL817 C446, equivalente funcional al PC817)
>
> Este documento **reemplaza** a `docs/PC817_WIRING_REFERENCE.md` para la
> interconexión de los sensores de rueda, el sensor de centro de dirección y
> la llave de contacto.
> El documento de la HY-M158 (PC817 ×8) se conserva como referencia histórica
> pero **ya no describe el hardware físico montado**.

---

## RESUMEN RÁPIDO

| Placa | Canales en uso | Señales |
|-------|----------------|---------|
| **Board 1** (Ruedas) | 4 de 4 | FR → PA1 · FL → PA0 · RR → PB15 · RL → PA2 |
| **Board 2** (Volante + Llave) | 2 de 4 | Centro volante → PB5 · Llave → ESP32 GPIO 40 |
| — | 2 libres | CH3 y CH4 de Board 2 sin usar (reserva) |

**Componentes adicionales obligatorios:** 6× TVS **P6KE18CA** (uno por canal activo).
**Componentes NO necesarios:** resistencias de entrada (ya integradas 2,8 kΩ por canal),
resistencias de pull-up de salida (ya integradas 2,7 kΩ + 470 Ω por canal).

---

## 1. DESCRIPCIÓN DE LA PLACA EL817 4 CANALES

### Identificación física (imagen de referencia)

```
Cara frontal (componentes visibles):

  Entradas (superior):  1+  1-  2+  2-  3+  3-  4+  4-
                        ○   ○   ○   ○   ○   ○   ○   ○
                       ┌──────────────────────────────────┐
                       │  [101][272] [101][272] ×4        │  ← R serie entrada: 100Ω + 2,7kΩ
                       │       ↓           ↓              │
                       │  [EL817] [EL817] [EL817] [EL817] │  ← 4× EL817 C446
                       │       ↓           ↓              │
                       │  [471][272] [471][272] ×4        │  ← R salida: 470Ω + 2,7kΩ
                       └──────────────────────────────────┘
                        ○   ○   ○   ○   ○   ○
  Salidas (inferior):  O4  O3  O2  O1   G   V
```

### Componentes integrados por canal

| Lado | Componente | Valor SMD | Función |
|------|-----------|-----------|---------|
| **Entrada** | R1 serie | **101** = 100 Ω | Limitación inicial de corriente |
| **Entrada** | R2 serie | **272** = 2,7 kΩ | Limitación principal de corriente LED |
| **Salida** | R3 pull-up | **272** = 2,7 kΩ | Pull-up del colector a V |
| **Salida** | R4 serie out | **471** = 470 Ω | Protección de salida |

### Descripción de terminales

| Terminal | Lado | Descripción |
|----------|------|-------------|
| `n+` | Entrada (12 V) | Ánodo — señal positiva de 12 V |
| `n-` | Entrada (12 V) | Cátodo — GND del dominio de 12 V |
| `On` | Salida (3,3 V) | Colector del fototransistor (señal lógica) |
| `G` | Salida (3,3 V) | GND del dominio lógico (STM32 / ESP32) |
| `V` | Salida (3,3 V) | Alimentación del lado lógico (+3,3 V) |

---

## 2. CÁLCULO ELÉCTRICO

### Corriente de LED de entrada (12 V nominal)

```
I_LED = (V_in − V_F_LED) / (R1 + R2)
      = (12 V − 1,2 V) / (100 Ω + 2700 Ω)
      = 10,8 V / 2800 Ω
      ≈ 3,86 mA   ✅  (dentro del rango 1–20 mA del EL817)
```

Con CTR mín. 50 % (EL817): I_C ≈ 1,9 mA en la salida.
La resistencia de pull-up interna de 2,7 kΩ a 3,3 V proporciona:
```
I_pull-up = 3,3 V / (2700 Ω + 470 Ω) = 3,3 / 3170 ≈ 1,04 mA
```
Con I_C > 1 mA garantizado por el CTR → el transistor satura → V_out ≈ 0,2 V (LOW) ✅

### Tiempo de flanco (carga capacitiva de cable)

```
R_eff = 2700 Ω + 470 Ω = 3170 Ω  (pull-up + serie salida)
τ = R_eff × C_cable = 3170 Ω × 1 nF ≈ 3,2 µs
t_to_VIH ≈ 3,9 µs con cable ≤ 5 m   ✅ (más rápido que la 4,7 kΩ externa anterior)
```

---

## 3. ASIGNACIÓN DE CANALES

### Board 1 — Sensores de rueda (4 canales, 4 en uso)

> **Cableado de entrada:** `n+` ← +12 V del sensor LJ12A3 (terminal `INn` → hilo marrón del sensor).
> `n-` ← GND_vehicle (cable azul del sensor + salida NPN negro — ver topología LJ12A3 más abajo).
>
> **Cableado de salida:** `On` → GPIO STM32. `G` → GND_logic. `V` → +3,3 V.

| Canal | Señal | `n+` → | `n-` → | `On` → | Flanco firmware |
|-------|-------|---------|---------|--------|----------------|
| **CH1** | Sensor rueda **FR** | +12 V (lado sensor) | GND_vehicle | STM32 **PA1** (EXTI1) | RISING |
| **CH2** | Sensor rueda **FL** | +12 V (lado sensor) | GND_vehicle | STM32 **PA0** (EXTI0) | RISING |
| **CH3** | Sensor rueda **RR** | +12 V (lado sensor) | GND_vehicle | STM32 **PB15** (EXTI15) | RISING |
| **CH4** | Sensor rueda **RL** | +12 V (lado sensor) | GND_vehicle | STM32 **PA2** (EXTI2) | RISING |

### Board 2 — Sensor de volante + Llave de contacto (4 canales, 2 en uso)

| Canal | Señal | `n+` → | `n-` → | `On` → | Flanco firmware |
|-------|-------|---------|---------|--------|----------------|
| **CH1** | Sensor centro **volante** (LJ12A3) | +12 V (lado sensor) | GND_vehicle | STM32 **PB5** (EXTI5) | FALLING |
| **CH2** | **Llave de contacto** (ACC) | +12 V cuando llave ON | GND_vehicle | ESP32 **GPIO 40** | nivel (LOW = ON) |
| CH3 | *libre — reserva* | — | — | — | — |
| CH4 | *libre — reserva* | — | — | — | — |

> **Por qué RISING para ruedas y FALLING para volante:**
> El EL817 invierte la señal: sensor activo (NPN conduce) → LED ON → transistor ON → salida LOW.
>
> - **Ruedas:** el firmware cuenta flancos de subida (LOW→HIGH) = momento en que el diente
>   sale del campo del sensor. Válido para cálculo de velocidad. (`GPIO_MODE_IT_RISING`)
>
> - **Centro volante:** el firmware detecta el flanco de bajada (HIGH→LOW) = momento en que
>   el tornillo **entra** en el campo del sensor = posición de centro exacta.
>   (`GPIO_MODE_IT_FALLING` — detecta el inicio del pulso, no el final, para mayor precisión)

---

## 4. TOPOLOGÍA COMPLETA DEL LADO DE 12 V — Sensor LJ12A3

El sensor LJ12A3-4-Z/BX es NPN de colector abierto:
- **MARRÓN** = +12 V (alimentación del sensor)
- **AZUL** = GND_vehicle (masa del sensor)
- **NEGRO** = Salida NPN (colector abierto, se cierra a GND cuando detecta metal)

### Cómo excitar el LED del EL817 con un LJ12A3 NPN:

```
════════ LADO 12 V (entrada — terminales n+ / n- del módulo EL817) ════════

  +12V ────────────────────────────────────────► MARRÓN (VCC sensor LJ12A3)
  +12V ───────────────────────────────────────── n+  del EL817
                                                  │
                                           [100Ω on-board]
                                                  │
                                           [2,7kΩ on-board]
                                                  │
                                                  ▼  ← LED conduce cuando NEGRO=GND
                                              EL817 LED
                                                  │
                                                  └──► n-  del EL817 ──► NEGRO (salida NPN LJ12A3)
                                                                                    │
                                                                               (transistor NPN)
  GND_vehicle ─────────────────────────────────────────────────────────────► AZUL (GND LJ12A3)
               (GND_vehicle = n- del EL817 a través del NEGRO del sensor)

  [P6KE18CA entre n+ y n-]  ← OBLIGATORIO (TVS bidireccional, 1 por canal activo)

════════ LADO 3,3 V (salida — terminales On / G / V del módulo EL817) ════════

  +3,3V ─────────────────────────────────────────────── V  del EL817
                                                 [2,7kΩ on-board]   ← pull-up integrado
                                                          │
                                                   EL817 colector
                                                          │
                                                  [470Ω on-board]
                                                          │
  STM32 GPIO ───────────────────────────────────── On del EL817

  GND_logic ─────────────────────────────────────── G  del EL817
```

**Secuencia de funcionamiento (metal detectado):**
1. Diente/tornillo entra en campo → NPN LJ12A3 satura → NEGRO → GND
2. Corriente: +12V → 100Ω → 2,7kΩ → LED EL817 → n- → NEGRO → GND_vehicle ≈ **3,86 mA**
3. EL817 salida: fototransistor satura → `On` ≈ 0,2 V (**LOW**)
4. STM32/ESP32 lee **LOW** ✅

**Sin metal:**
1. NPN LJ12A3 en corte → NEGRO = alta impedancia
2. Sin corriente → LED apagado → fototransistor en corte
3. `On` = +3,3 V vía pull-up 2,7 kΩ on-board (**HIGH**) ✅

---

## 5. CIRCUITO DE LLAVE DE CONTACTO (Board 2, CH2)

```
════════ LADO 12 V (terminal n+ / n- del EL817 Board 2, CH2) ════════

  Terminal ACC del contacto (+12 V cuando llave en ON) ────► n+  EL817 CH2
                                                               │
                                                        [100Ω + 2,7kΩ on-board]
                                                               │
                                                           EL817 LED
                                                               │
                                                               └──► n-  ──► GND_vehicle

  [P6KE18CA entre n+ y n-]  ← OBLIGATORIO

════════ LADO 3,3 V (salida — On2 / G / V del módulo EL817) ════════

  +3,3V ──[2,7kΩ on-board]──► colector EL817 ──[470Ω on-board]──► O2 ──► ESP32 GPIO 40

  G  del EL817 ──► GND_logic (ESP32 + STM32 GND compartido)
```

**Tabla de niveles:**

| Posición llave | LED EL817 | O2 (ESP32 GPIO 40) | Interpretación firmware |
|----------------|-----------|---------------------|------------------------|
| **ON** (+12 V) | Encendido | **LOW** (≈ 0,2 V) | Llave encendida ✅ |
| **OFF** (0 V) | Apagado | **HIGH** (3,3 V) | Llave apagada |

```cpp
// power_manager.cpp — lectura de la llave (sin cambios respecto a versión anterior)
bool raw = (digitalRead(PIN_IGNITION_SENSE) == LOW);  // LOW = llave ON
```

---

## 6. COMPONENTES A AÑADIR (OBLIGATORIOS Y OPCIONALES)

### Tabla completa

| Cant. | Componente | Valor | Función | Dónde va |
|-------|-----------|-------|---------|----------|
| **6** | **TVS P6KE18CA** | Bidireccional 600 W, DO-15 | Protección automotriz contra transitorios (+40 V load dump / negativos) | Entre `n+` y `n-` de cada canal activo (cara trasera o en regleta de entradas) |
| ~~4~~ | ~~Pull-up 4,7 kΩ~~ | — | **NO necesario** — pull-up 2,7 kΩ ya integrado on-board | — |
| ~~1~~ | ~~Pull-up 10 kΩ~~ | — | **NO necesario** — pull-up 2,7 kΩ ya integrado on-board | — |
| ~~6~~ | ~~R serie entrada~~ | — | **NO necesario** — 100 Ω + 2,7 kΩ ya integrados on-board | — |
| 2 *(opc.)* | Condensador | **100 nF cerámico** | Desacoplo del rail V (+3,3 V) de cada placa | Entre `V` y `G` de cada Board (lo más cerca posible del conector) |

> **Los condensadores de 100 nF son opcionales pero recomendados** para filtrar el rail de
> alimentación del lado lógico ante transitorios de la red 12 V que puedan colarse por el
> lado del sensor.

### Detalle del TVS P6KE18CA

| Parámetro | Valor |
|-----------|-------|
| Tipo | Bidireccional (CA) |
| Tensión de clamping a 5 A | 29,2 V |
| Potencia pico (10/1000 µs) | 600 W |
| Corriente de fuga a 12 V | < 1 µA |
| Encapsulado | DO-15 / DO-201 |
| **Polaridad** | **No importa — bidireccional** |

**Por canal activo (6 en total: Board 1 CH1–CH4 + Board 2 CH1–CH2):**

```
  n+  ───────────────────────────────► (+12 V del sensor)
   │
[P6KE18CA]   ← soldado entre n+ y n-
   │            (cara trasera de la placa o en el bloque de terminales)
  n-  ───────────────────────────────► GND_vehicle
```

**Sin TVS:**
```
  I_pico = (40 V − 1,2 V) / (100 + 2700) Ω ≈ 13,9 mA con pico de 40 V
  V_LED_abs_max = 3 V  → dañaría el LED del EL817 sin TVS
```

**Con TVS P6KE18CA:**
```
  V_clamping = 29,2 V  → I_LED = (29,2 − 1,2) / 2800 ≈ 10 mA  ← dentro de spec ✅
  Energía absorbida por TVS, no por el LED ✅
```

---

## 7. AISLAMIENTO GALVÁNICO Y SEPARACIÓN DE MASAS

Las placas EL817 de 4 canales **aislan galvánicamente** los dominios de 12 V y 3,3 V.
A diferencia del módulo HY-M158, **no hay jumpers que quitar** — las placas EL817 de este
tipo no conectan los dos planos de GND por defecto.

**Verificar antes de montar** (con polímetro en modo continuidad):
- Entre `n-` (GND_vehicle) y `G` (GND_logic): **debe estar abierto** (sin pitido) ✅
- Si hay continuidad: el módulo tiene un puente interno → no aisla → sustituir o cortar pista

```
GND_vehicle (12 V)  ─── n- de cada canal ───┐
                                             │ ← BARRERA GALVÁNICA (EL817)
GND_logic   (3,3 V) ─── G del módulo   ────┤
                         └── STM32 GND ─────┤
                         └── ESP32 GND ──────┘
                         (mismo nodo — obligatorio para bus CAN)
```

---

## 8. ESQUEMA DE CABLEADO COMPLETO — Board 1 (4 ruedas)

```
                        ┌─────────────────────────────────┐
                        │    BOARD 1 — EL817 4 CANALES    │
                        │                                  │
  Sensor FR  +12V ──►── 1+ │  [EL817 CH1] │ O1 ──► STM32 PA1
  Sensor FR  GND  ──►── 1- │              │ G  ──► GND_logic
                            │  [EL817 CH2] │ O2 ──► STM32 PA0
  Sensor FL  +12V ──►── 2+ │              │
  Sensor FL  GND  ──►── 2- │  [EL817 CH3] │ O3 ──► STM32 PB15
                            │              │ V  ──► +3,3 V
  Sensor RR  +12V ──►── 3+ │  [EL817 CH4] │ O4 ──► STM32 PA2
  Sensor RR  GND  ──►── 3- │              │
                            │              │
  Sensor RL  +12V ──►── 4+ │              │
  Sensor RL  GND  ──►── 4- │              │
                        └─────────────────────────────────┘

  [P6KE18CA × 4] — uno entre n+ y n- de cada canal (lado 12 V)
  [100 nF] — entre V y G (lado 3,3 V) — opcional
```

---

## 9. ESQUEMA DE CABLEADO COMPLETO — Board 2 (volante + llave)

```
                        ┌─────────────────────────────────┐
                        │    BOARD 2 — EL817 4 CANALES    │
                        │                                  │
  Sensor volante +12V ─ 1+ │  [EL817 CH1] │ O1 ──► STM32 PB5
  Sensor volante GND  ─ 1- │              │ G  ──► GND_logic
                            │  [EL817 CH2] │ O2 ──► ESP32 GPIO 40
  Llave contacto +12V ─ 2+ │              │ V  ──► +3,3 V
  Llave contacto GND  ─ 2- │              │
                            │  [EL817 CH3] │ O3 ──► libre (sin conectar)
  (libre)              ─ 3+ │              │
  (libre)              ─ 3- │  [EL817 CH4] │ O4 ──► libre (sin conectar)
                            │              │
  (libre)              ─ 4+ │              │
  (libre)              ─ 4- │              │
                        └─────────────────────────────────┘

  [P6KE18CA × 2] — uno entre n+ y n- de CH1 y CH2 (lado 12 V)
  [100 nF] — entre V y G (lado 3,3 V) — opcional
```

---

## 10. LISTA DE COMPRA FINAL

| Cant. | Referencia | Descripción | Destino |
|-------|-----------|-------------|---------|
| **6** | **P6KE18CA** | TVS bidireccional 600 W DO-15 | 4× Board 1 (ruedas) + 2× Board 2 (volante + llave) |
| 2 *(opc.)* | Condensador 100 nF cerámico X7R | Desacoplo rail 3,3 V | 1× Board 1 V-G · 1× Board 2 V-G |

**NO comprar:**
- ❌ Resistencias de entrada (ya integradas: 100 Ω + 2,7 kΩ por canal)
- ❌ Resistencias pull-up de salida (ya integradas: 2,7 kΩ + 470 Ω por canal)
- ❌ Diodo 1N4148 (reemplazado por el TVS P6KE18CA)

---

## 11. VERIFICACIÓN DE FIRMWARE

El firmware **no requiere modificaciones**. Los pines y modos EXTI ya configurados son
plenamente compatibles con la salida de colector abierto del EL817:

| Señal | Pin STM32 | Modo EXTI firmware | Compatible con EL817 |
|-------|-----------|-------------------|----------------------|
| Rueda FR | PA1 | `GPIO_MODE_IT_RISING` + `GPIO_PULLUP` | ✅ |
| Rueda FL | PA0 | `GPIO_MODE_IT_RISING` + `GPIO_PULLUP` | ✅ |
| Rueda RR | PB15 | `GPIO_MODE_IT_RISING` + `GPIO_PULLUP` | ✅ |
| Rueda RL | PA2 | `GPIO_MODE_IT_RISING` + `GPIO_PULLUP` | ✅ |
| Centro volante | PB5 | `GPIO_MODE_IT_FALLING` + `GPIO_PULLUP` | ✅ |
| Llave contacto | ESP32 GPIO 40 | `INPUT_PULLUP`, LOW = ON | ✅ |

> El pull-up interno del STM32 (~40 kΩ) puede permanecer habilitado sin problema;
> el pull-up integrado de 2,7 kΩ de la placa EL817 domina y garantiza flancos rápidos.

---

## 12. DIFERENCIAS CLAVE RESPECTO AL MÓDULO HY-M158 ANTERIOR

| Aspecto | HY-M158 (PC817 ×8) | EL817 4CH ×2 (nuevo) |
|---------|--------------------|-----------------------|
| Canales | 1 módulo × 8 canales | 2 módulos × 4 canales |
| Señales conectadas | 5 (4 ruedas + llave) | **6 (4 ruedas + volante + llave)** |
| R entrada integrada | 3 kΩ (SMD 302) | **2,8 kΩ** (100 Ω + 2,7 kΩ) |
| R pull-up salida | ❌ No integrada | ✅ **2,7 kΩ on-board** |
| R serie salida | ❌ No integrada | ✅ **470 Ω on-board** |
| Pull-up externo | **Sí, obligatorio** (4,7 kΩ / 10 kΩ) | **No necesario** |
| Jumpers a quitar | Sí (8 jumpers rojos) | **No (no tiene)** |
| Polaridad entrada | Implícita (INn / G) | **Explícita (n+ / n-)** |
| Sensor volante (PB5) | No conectado | **✅ Conectado CH1 Board 2** |
| TVS P6KE18CA | 5× (4 ruedas + llave) | **6× (4 ruedas + volante + llave)** |
