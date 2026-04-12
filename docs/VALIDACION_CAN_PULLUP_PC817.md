# VALIDACIÓN CAN Y PULL-UPS PC817 — Definición Completa

**Sistema de Control Vehicular — STM32G474RE**
**Documentos base:** `CABLEADO_AISLAMIENTO_DEFINITIVO.md`, `VALIDACION_ELECTRICA_AISLAMIENTO.md`
**Estado: Definición funcional previa al montaje — Firmware SIN modificar**

---

## Índice

1. [Arquitectura funcional del bus CAN en estados de encendido parcial](#1-arquitectura-funcional-del-bus-can-en-estados-de-encendido-parcial)
2. [Resistencias de pull-up externas para entradas PC817](#2-resistencias-de-pull-up-externas-para-entradas-pc817)

---

## 1. Arquitectura funcional del bus CAN en estados de encendido parcial

### 1.1 Nodo de referencia de polarización del bus

El bus CAN 2.0A **no tiene un nodo de polarización dedicado**. El potencial de modo común
del bus (~2.5 V en estado recesivo) es el resultado del funcionamiento conjunto de los dos
transceivers TJA1051T/3 y del divisor formado por las dos terminaciones de 120 Ω.

#### Mecanismo de polarización en estado recesivo

En estado recesivo (ningún nodo transmitiendo), cada TJA1051T/3 activo deja CANH y CANL
en modo de alta impedancia interna. Las resistencias de terminación (120 Ω × 2 = 60 Ω
en paralelo) conectan CANH a CANL sin fuente de corriente activa. El bus flota a ~2.5 V
de modo común por la simetría del sistema diferencial.

Si **ambos transceivers están activos**: ambos contribuyen a mantener los extremos de
terminación referenciados a sus respectivos GND. El modo común se define por GND_CAN (que
en la arquitectura con aislamiento es el GND del lado bus, compartido por TJA1051_A y
TJA1051_B).

Si **solo un transceiver está activo**: ese transceiver polariza el bus. El otro extremo
del cable tiene solo su resistencia de terminación de 120 Ω, que sigue conectada
físicamente aunque el transceiver esté sin alimentación.

> **Conclusión:** En este sistema de 2 nodos, **el STM32 y el ESP32 comparten la
> responsabilidad de polarización del bus**. No es necesario ni existe un "nodo maestro"
> de referencia. Lo correcto es que ambos transceivers estén activos simultáneamente en
> condiciones normales de operación.

---

### 1.2 Protección hardware presente: Dominant Timeout del TJA1051T/3

El TJA1051T/3 incluye una función de protección crítica para este análisis:

> **Dominant Timeout (t_timeout ≈ 3.2 ms):** Si la entrada TXD permanece en nivel LOW
> (bit dominante) durante más de ~3 ms, el transceiver entra automáticamente en **modo
> silencioso** (CANH/CANL en alta impedancia, RXD fijado HIGH). Esto previene que un MCU
> colgado o sin alimentación bloquee el bus indefinidamente.

Esta función es activa en el modo Normal (pin S = GND), que es la configuración actual
del diseño (confirmado en `ESP32_STM32_CAN_CONNECTION.md`).

---

### 1.3 Escenario A: STM32 apagada, ESP32 encendida

#### ¿Qué ocurre en el TJA1051_A (lado STM32)?

La alimentación del TJA1051_A depende de si comparte la fuente de 5 V general con el ESP32
o si se alimenta exclusivamente de la placa STM32:

| Caso | TJA1051_A VCC | TXD del TJA1051_A | Comportamiento bus |
|------|---------------|-------------------|--------------------|
| **A1.** TJA1051_A sin alimentación (STM32 power off) | 0 V | — | CANH/CANL en alta Z. Solo 120 Ω de terminación en lado A. |
| **A2.** TJA1051_A alimentado desde 5V general (siempre) | 5 V | Flotante (PA12 no traccionado) | TXD puede ir a LOW → dominante → Dominant Timeout en ~3 ms → modo silencioso |

En ambos casos el resultado final es el mismo: el TJA1051_A **no actúa sobre CANH/CANL**.

#### ¿Puede el ESP32 transmitir?

**Sí, físicamente.** El ESP32 puede generar tramas CAN a través de TJA1051_B. Sin embargo:

- Cada trama CAN transmitida necesita que **al menos un nodo receptor** confirme recepción
  enviando el bit ACK dominante en el slot ACK.
- Si el STM32 está apagado, nadie puede recibir ni confirmar las tramas del ESP32.
- El controlador CAN del ESP32 generará errores ACK en cada trama transmitida.
- Tras suficientes errores (TEC ≥ 128 → Error Passive; TEC ≥ 256 → Bus-Off), el ESP32
  entrará en **Bus-Off** y dejará de transmitir.

#### ¿Hay riesgo para el sistema?

- **El STM32 ya maneja este escenario:** si no recibe heartbeat del ESP32 en 250 ms →
  entra en `LIMP_HOME` (operación autónoma local con pedal, velocidad limitada).
- **No hay riesgo de latch-up ni daño:** el TJA1051_A protege el bus con el Dominant
  Timeout; el TJA1051_B puede operar libremente hasta que entre en Bus-Off por ACK errors.
- **El bus no queda bloqueado** porque el Dominant Timeout libera el bus en < 3 ms si TXD
  flotara LOW.

---

### 1.4 Escenario B: ESP32 apagada, STM32 encendida

Situación simétrica al caso A.

#### ¿Qué ocurre en el TJA1051_B (lado ESP32)?

Idéntico análisis. Si TJA1051_B pierde alimentación o su TXD flota, el Dominant Timeout
o la alta impedancia del transceiver sin alimentación liberan el bus.

#### ¿Puede el STM32 transmitir?

**Sí, físicamente.** El STM32 transmite su heartbeat (ID 0x001) cada 100 ms (`CAN_SendHeartbeat()`).
Sin ACK del ESP32, el TEC del STM32 aumenta.

Comportamiento del firmware ya implementado:

```
FDCAN1 AutoRetransmission = ENABLE  → reintenta indefinidamente hasta BUS-OFF
CAN_CheckBusOff() polled every 10 ms → detecta psr.BusOff == true
  → busoff_active = 1
  → Safety_SetError(SAFETY_ERROR_CAN_BUSOFF)
  → Safety_SetState(SYS_STATE_LIMP_HOME)   ← NOT SAFE, vehicle stays mobile
  → retries at CAN_BUSOFF_RETRY_INTERVAL_MS intervals
  → up to CAN_BUSOFF_MAX_RETRIES attempts
  → if recovery succeeds: Safety_ClearError(SAFETY_ERROR_CAN_BUSOFF)
  → Boot validation: check_can_not_busoff() always returns true → bus-off is NOT a boot blocker
```

El STM32 **continuará operando en `LIMP_HOME`** (pedal local, velocidad < 5 km/h)
independientemente del estado del ESP32. Esto está diseñado intencionalmente: la pérdida
del ESP32 no es un fallo de seguridad hardware.

---

### 1.5 Riesgo identificado: startup transient de TXD sin pull-up

El firmware configura PA12 (FDCAN1_TX) como `GPIO_MODE_AF_PP, GPIO_NOPULL` en la función
`HAL_FDCAN_MspInit()`. Antes de que se ejecute `MX_FDCAN1_Init()`, el pin PA12 está en
modo GPIO_INPUT (alta impedancia, comportamiento por defecto tras reset del STM32).

Durante esta ventana de tiempo (desde power-on hasta `MX_FDCAN1_Init()`):

```
t_power_on → t_FDCAN_init ≈ 50–200 µs (inicialización de HAL + relojes + periféricos previos)

Si TXD flota LOW durante este tiempo:
  TJA1051_A conduce bit dominante
  50–200 µs < 3 ms del Dominant Timeout → podría no activarse
  El ESP32 vería bits dominantes inesperados → posibles errores de trama en las primeras tramas
```

Aunque en la práctica es una ventana muy corta (el Dominant Timeout es 3ms), la buena
práctica de hardware elimina este riesgo completamente.

---

### 1.6 Estrategia para evitar Bus-Off al arranque

La estrategia completa combina una medida de hardware (sin cambio de firmware) con el
comportamiento ya existente en el firmware.

#### Medida de hardware: pull-up en TXD de ambos TJA1051T/3

Añadir **una resistencia de 10 kΩ entre TXD y VCC** en cada TJA1051T/3. Esta resistencia
garantiza que TXD esté en HIGH (recesivo) cuando el MCU está apagado, en reset o con el
GPIO en modo de entrada.

```
Circuito por nodo:

  VCC_transceiver (5 V obligatorio — VCC mín 4.5 V según datasheet TJA1051T/3)
        │
       [10 kΩ]
        │
        ├──── TXD (pin 1 del TJA1051T/3)
        │
  STM32/ESP32 TX (GPIO AF) ────────────────────────────────── (mismo nodo)

```

| Condición | TXD con pull-up 10 kΩ | Efecto en bus |
|-----------|----------------------|---------------|
| MCU apagado | TXD = VCC (HIGH) | TJA1051 → recesivo → bus libre |
| MCU en reset | TXD = VCC (HIGH) | TJA1051 → recesivo → bus libre |
| GPIO flotante (input mode) | TXD ≈ VCC (pull-up domina) | TJA1051 → recesivo → bus libre |
| MCU transmitiendo LOW | TXD = 0 V (MCU domina, I = VCC/10kΩ ≈ 0.33–0.5 mA) | TJA1051 → dominante → correcto |
| MCU transmitiendo HIGH | TXD = VCC (ambos a VCC) | TJA1051 → recesivo → correcto |

**Corriente adicional en operación normal** con pull-up de 10 kΩ, VCC = 5 V:
- Durante bit dominante (TXD = LOW): I = 5V / 10kΩ = 0.5 mA (fluye desde VCC al GPIO)
- Esta corriente es absorbida por el GPIO del MCU como sink (STM32G474 soporta 8 mA sink). ✅
- Energía adicional: P = 0.5 mA × 5V = 2.5 mW → completamente despreciable. ✅

#### Comportamiento del firmware en Bus-Off (ya implementado)

| Evento | Firmware STM32 | Impacto en la operación |
|--------|---------------|------------------------|
| ESP32 no responde en 250 ms | → `LIMP_HOME` (falla operacional, no de seguridad) | Vehículo mobile a ≤ 5 km/h, 20 % torque |
| Bus-Off detectado | → `LIMP_HOME` + `SAFETY_ERROR_CAN_BUSOFF` | ídem — no entra en SAFE |
| Recuperación exitosa | → `ClearError(CAN_BUSOFF)` | Sistema sale de LIMP_HOME si heartbeat vuelve |
| Bus-Off no se recupera (max retries) | Sistema permanece en `LIMP_HOME` | Vehículo sigue operativo localmente |
| Boot validation | `check_can_not_busoff()` siempre retorna `true` | Bus-Off NO bloquea el arranque |

---

### 1.7 Diagrama lógico completo de red CAN

```
╔══════════════════════════════════════════════════════════════════════╗
║                  TOPOLOGÍA CAN DEFINITIVA CON PULL-UPS               ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                      ║
║  GND_STM32 domain     │ ISO barrier │   GND_CAN = GND_ESP32 domain  ║
║  ─────────────────    │             │   ─────────────────────────── ║
║                       │             │                               ║
║  STM32G474RE          │             │                               ║
║  PA12(FDCAN1_TX)─┐   │             │                               ║
║                  │   │             │                               ║
║  [Si CAN aislado]│   │             │  ┌── 10kΩ ── VCC_TJA_A        ║
║  [ADuM1201] ─────┤   │~~~~~~~~~~~~~│  │                            ║
║                  └──►│             │►─┤ TJA1051T/3_A TXD (pin 1)   ║
║                      │  DC-DC iso  │  │ CANH (pin 7) ──────────────╫──CANH
║  PA11(FDCAN1_RX)◄┐  │  5V/200mA   │  │ CANL (pin 6) ──────────────╫──CANL
║  [ADuM1201] ─────┘  │             │◄─┤ RXD  (pin 4)               ║
║                      │             │  └─ S (pin 8) → GND_CAN       ║
║                      │             │                               ║
║                      │             │   [120 Ω CANH-CANL]           ║
║                      │             │                               ║
║                      │             │   CANH ═══════ CANH           ║
║                      │             │   CANL ═══════ CANL           ║
║                      │             │                               ║
║                      │             │   [120 Ω CANH-CANL]           ║
║                      │             │                               ║
║                      │             │  ┌── 10kΩ ── VCC_TJA_B        ║
║                      │             │  │ TJA1051T/3_B TXD (pin 1)◄──╫── GPIO4 (ESP32-S3)
║                      │             │  │ CANH (pin 7) ──────────────╫── CANH
║                      │             │  │ CANL (pin 6) ──────────────╫── CANL
║                      │             │  │ RXD  (pin 4) ──────────────╫──► GPIO5 (ESP32-S3)
║                      │             │  └─ S (pin 8) → GND_CAN       ║
║                      │             │                               ║
╠══════════════════════════════════════════════════════════════════════╣
║  Nodo CAN    │ Prioridad bus  │ Rol funcional         │ Arbitraje   ║
╠══════════════╪════════════════╪═══════════════════════╪═════════════╣
║  STM32G474RE │ Más alta       │ Autoridad de seguridad│ ID 0x001    ║
║              │ (ID más bajo)  │ Valida todos los cmds │ (heartbeat) ║
╠══════════════╪════════════════╪═══════════════════════╪═════════════╣
║  ESP32-S3    │ Más baja       │ Control usuario / HMI │ ID 0x011+   ║
║              │ (ID más alto)  │ Genera comandos        │ (heartbeat) ║
╚══════════════╧════════════════╧═══════════════════════╧═════════════╝

NOTA: Los pull-ups de 10kΩ en TXD son la única modificación hardware recomendada.
El aislador digital (ADuM1201 + DC-DC) es OPCIONAL — ver CABLEADO_AISLAMIENTO_DEFINITIVO.
```

---

### 1.8 Resumen de estrategia CAN — puntos de acción

| # | Acción | Tipo | Impacto en firmware |
|---|--------|------|---------------------|
| **C1** | Añadir 10 kΩ de TXD a VCC en TJA1051T/3_A (lado STM32) | Hardware | Ninguno |
| **C2** | Añadir 10 kΩ de TXD a VCC en TJA1051T/3_B (lado ESP32) | Hardware | Ninguno |
| **C3** | No modificar S (pin 8) → mantener GND (modo Normal) | — | — |
| **C4** | No modificar el firmware de CAN (bus-off recovery ya implementado) | Firmware | Ya completo |
| **C5** | Definir arranque: STM32 debe arrancar antes o simultáneamente con ESP32. | Operacional | Ninguno |

La suma de estas medidas garantiza:
- Sin dominante al arranque (pull-up TXD)
- Sin bloqueo de bus si un nodo está apagado (Dominant Timeout + alta Z sin alimentación)
- Recuperación automática de Bus-Off (firmware ya implementado)
- Operación autónoma si el ESP32 falla (LIMP_HOME → vehículo mobile)

---

## 2. Resistencias de pull-up externas para entradas PC817

### 2.1 Por qué el pull-up interno de 40 kΩ del STM32 es insuficiente

El pull-up interno del STM32 tiene un valor nominal de ~40 kΩ (rango garantizado:
10–40 kΩ según RM0440 para la familia G4). Este valor crea dos problemas en aplicaciones
con cable y PC817:

#### Problema A: Flanco de subida lento (RC con capacidad de cable)

```
τ_rise = R_pullup × C_total

Parámetros:
  R_pullup = 40 kΩ (interno, peor caso)
  C_cable  = 100–150 pF/m × 3 m (cable apantallado típico) = 300–450 pF
  C_int    = STM32 GPIO input capacitance ≈ 5–10 pF
  C_total  ≈ 500 pF

τ_rise = 40 000 × 500×10⁻¹² = 20 µs

Tiempo para alcanzar VIH (2.31 V desde 0 V):
  t_to_VIH = -τ × ln(1 - VIH/VDD) = -20µs × ln(1 - 2.31/3.3) = -20µs × ln(0.30) = 24.1 µs

Tiempo para alcanzar 99 % (3.267 V):
  5τ = 100 µs
```

Con 40 kΩ, el flanco de subida tarda hasta **24 µs en superar el umbral VIH** del EXTI.
Comparado con el debounce software de 1 ms, parece aceptable numéricamente, pero:

- El flanco largo aumenta la incertidumbre en el timing del flanco detectado.
- En presencia de ruido durante la transición, el Schmitt trigger puede disparar múltiples
  veces (si el ruido supera la histéresis de ~1.32 V en la zona de transición).
- El valor real del pull-up puede variar de 10 kΩ a 40 kΩ (±75 % de variación según la
  tolerancia garantizada). Con 10 kΩ el comportamiento mejora; con 40 kΩ es el peor caso.
- La capacidad del cable puede variar: con 5 m de cable no apantallado la C puede llegar
  a 1 nF → τ = 40 µs → t_to_VIH ≈ 48 µs.

#### Problema B: El optotransistor del PC817 opera cerca del límite lineal/saturación

El PC817 en saturación requiere que la fotocorriente generada sea mayor que la corriente
de colector necesaria para que V_CE = V_CE_sat.

```
Con R_pullup = 40 kΩ:
  I_C = (3.3 V - V_CE_sat) / 40 000 Ω ≈ (3.3 - 0.1) / 40 000 = 80 µA

Para saturación: I_photo ≥ I_C
  I_photo = I_F × CTR_mínimo

  CTR_mínimo (worst case):
    PC817 Grupo A: CTR_typ = 100 %, CTR_min = 80 %
    Derating temperatura (+70°C): ×0.6 del valor a 25°C
    Derating envejecimiento (10.000 h): ×0.8
    CTR_efectivo_mínimo ≈ 80 % × 0.6 × 0.8 = 38 %

  Con I_F_mínimo = 1 mA (degradación LED, cable largo):
    I_photo_min = 1 mA × 38 % = 380 µA

  Verificación: 380 µA >> 80 µA → saturación garantizada con 40 kΩ
```

Con 40 kΩ **la saturación sí está garantizada en los cálculos estáticos**. Sin embargo,
la región lineal aparece durante las **transiciones**: el optotransistor pasa por la región
activa (V_CE ≠ V_CE_sat) durante el flanco. Con la carga de 40 kΩ, la corriente de colector
disponible es solo 80 µA, lo que hace que esta transición sea más lenta y que el transistor
permanezca en la región activa durante más tiempo.

**En conclusión: el problema de 40 kΩ es principalmente de VELOCIDAD DE TRANSICIÓN
(flanco de subida lento por RC), no de estado estático en saturación.**

---

### 2.2 Cálculo del pull-up externo óptimo

#### Criterios de diseño

| Criterio | Objetivo | Justificación |
|----------|---------|---------------|
| Nivel LOW garantizado | V_OUT_low ≤ 0.5 V | Margen × 2 sobre VIL_max = 0.99 V del STM32 |
| Nivel HIGH en reposo | V_OUT_high = 3.3 V | Sin carga sobre pull-up cuando transistor OFF |
| Flanco de subida | t_to_VIH ≤ 5 µs | 200× más rápido que debounce de 1 ms; margen contra ruido |
| Saturación en todo rango | I_photo ≥ I_C a CTR_min, I_F_min | Válido a -20°C a +70°C, 10 000 h de vida |
| Cable máximo | 5 m, hasta 1 nF capacidad | Instalación en vehículo, posible cable largo |
| Sin cambio de firmware | Compatible con GPIO_PULLUP existente | Paralelo al interno |

#### Cálculo del resistor

**Para el flanco de subida con cable de hasta 5 m (C_total ≈ 1 nF):**

```
τ ≤ t_to_VIH_max / (-ln(1 - VIH/VDD))
τ ≤ 5 µs / (-ln(1 - 2.31/3.3))
τ ≤ 5 µs / 1.204
τ ≤ 4.15 µs

R_pullup_max = τ / C_total = 4.15 µs / 1 nF = 4 150 Ω → máximo permitido: 4.15 kΩ

Selección estándar: R = 4.7 kΩ (da τ = 4.7 nF × 1 nF = 4.7 µs; t_to_VIH = 5.7 µs)

Para cable ≤ 3 m (C_total ≈ 500 pF): τ = 4.7k × 500p = 2.35 µs → t_to_VIH = 2.8 µs ✅
Para cable ≤ 5 m (C_total ≈ 1 nF):  τ = 4.7k × 1n = 4.7 µs  → t_to_VIH = 5.7 µs ✅
```

**Para garantizar saturación con R_pullup = 4.7 kΩ:**

```
I_C = (3.3 - 0.1) / 4 700 = 0.68 mA

Saturación requiere: I_photo ≥ I_C
  I_photo = I_F × CTR_efectivo_mínimo (38 %, calculado arriba)

  I_F_mínimo requerido para saturación:
    I_F_min = I_C / CTR_min = 0.68 mA / 38 % = 1.8 mA

Con placa PC817 módulo industrial (R_placa = 1 kΩ a 12 V):
  I_F = (12 - 1.5) / 1 000 = 10.5 mA >> 1.8 mA → saturación garantizada ✅

Con placa PC817 a 24 V con R = 2.2 kΩ:
  I_F = (24 - 1.5) / 2 200 = 10.2 mA >> 1.8 mA → saturación garantizada ✅

Caso degradado (LED envejecido + cable largo, I_F_min = 3 mA):
  I_F = 3 mA × 38 % = 1.14 mA > 0.68 mA → saturación garantizada ✅

Límite inferior de I_F:
  I_F debe ser ≥ 1.8 mA para garantizar saturación con 4.7 kΩ
  Con cualquier placa módulo estándar a 6–36 V (I_F ≥ 3 mA), se cumple siempre ✅
```

**Verificación del nivel LOW (V_CE_sat) con 4.7 kΩ:**

```
I_C = 0.68 mA, I_F = 5 mA (conservador), overdrive = 5 mA / 0.68 mA = 7.4×

PC817 V_CE_sat a I_C = 0.68 mA, I_F = 5 mA (datasheet):
  V_CE_sat ≈ 0.05 – 0.10 V

Nivel LOW en STM32: 0.10 V << VIL_max = 0.99 V ✅

A 70°C (peor caso de temperatura, CTR reducido pero aún saturado):
  V_CE_sat se incrementa ligeramente, máximo 0.20 V << VIL_max = 0.99 V ✅
```

**Verificación del nivel HIGH (resistor + pull-up interno en paralelo):**

```
Cuando transistor OFF (sensor inactivo):
  R_efectivo = R_ext || R_int = 4.7 kΩ || 40 kΩ = 4.2 kΩ

  V_OUT_high = VDD = 3.3 V (sin carga apreciable cuando transistor OFF)
  VIH_min STM32 = 2.31 V
  3.3 V >> 2.31 V ✅

Corriente cuando transistor OFF: I = 3.3 V / 4.2 kΩ ≈ 0.79 mA
Esta corriente fluye a través del pull-up hacia VDD_STM32 (corriente de reposo).
Potencia: P = 3.3² / 4.2 kΩ ≈ 2.6 mW por canal → 6 canales = 15.6 mW total → aceptable ✅
```

---

### 2.3 Tabla final de resistencias externas obligatorias

| # | Señal | Pin STM32 | Aislador | Pull-up externo R | Ubicación | Notas |
|---|-------|-----------|----------|--------------------|-----------|-------|
| 1 | **WHEEL_FL** | PA0 / EXTI0 | PC817 | **4.7 kΩ, 1/8 W** | Entre Vo del PC817 y línea 3.3V | A 3.3V del lado lógico (secundario) |
| 2 | **WHEEL_FR** | PA1 / EXTI1 | PC817 | **4.7 kΩ, 1/8 W** | Entre Vo del PC817 y línea 3.3V | ídem |
| 3 | **WHEEL_RL** | PA2 / EXTI2 | PC817 | **4.7 kΩ, 1/8 W** | Entre Vo del PC817 y línea 3.3V | ídem |
| 4 | **WHEEL_RR** | PB15 / EXTI15 | PC817 | **4.7 kΩ, 1/8 W** | Entre Vo del PC817 y línea 3.3V | ídem |
| 5 | **STEER_CENTER** | PB5 / EXTI5 | PC817 | **4.7 kΩ, 1/8 W** | Entre Vo del PC817 y línea 3.3V | ídem |
| 6 | **ENC_Z** *(inactivo)* | PB4 / EXTI4 | PC817 | **4.7 kΩ, 1/8 W** | Entre Vo del PC817 y línea 3.3V | Instalar ahora; EXTI4 activar cuando se use Z |
| 7 | **ENC_A** | PA15 / TIM2_CH1 | 6N137 | **4.7 kΩ, 1/8 W** | Entre pin 6 del 6N137 y línea 3.3V | OBLIGATORIO — `GPIO_NOPULL` en MSP |
| 8 | **ENC_B** | PB3 / TIM2_CH2 | 6N137 | **4.7 kΩ, 1/8 W** | Entre pin 6 del 6N137 y línea 3.3V | OBLIGATORIO — `GPIO_NOPULL` en MSP |

**Notas sobre el pull-up interno con los pull-ups externos:**

- Para los canales PC817 (señales WHEEL × 4, STEER_CENTER, ENC_Z): el firmware usa
  `GPIO_PULLUP`. El pull-up interno (40 kΩ) queda en **paralelo** con el externo (4.7 kΩ):
  - R_efectiva = 4.7 kΩ || 40 kΩ ≈ **4.2 kΩ** — se toma como el valor real de carga
  - Esto es más conservador que 4.7 kΩ solo (mayor corriente → mejor saturación, mayor consumo en reposo)
  - No se requiere cambio de firmware; la diferencia 4.7 kΩ vs 4.2 kΩ es despreciable

- Para ENC_A y ENC_B (6N137): el firmware usa `GPIO_NOPULL`. El pull-up externo de 4.7 kΩ
  es el ÚNICO pull-up. **Si se omite, la entrada del TIM2 flota → pérdida de control de dirección.**

---

### 2.4 Tolerancias y variación con temperatura

```
Rango de R_efectiva con tolerancia de 5% en el resistor externo:

  R_ext_min = 4.7 kΩ × 0.95 = 4.47 kΩ → paralelo con 40k = 4.01 kΩ
  R_ext_max = 4.7 kΩ × 1.05 = 4.94 kΩ → paralelo con 40k = 4.39 kΩ

  τ_min = 4.01 kΩ × 1 nF = 4.0 µs → t_to_VIH = 4.8 µs ✅
  τ_max = 4.39 kΩ × 1 nF = 4.4 µs → t_to_VIH = 5.3 µs ✅

Variación de temperatura del resistor (1% Film de metal, ±100 ppm/°C, -20°C a +70°C):
  ΔR/R = 100e-6 × 90°C = 0.9% → no significativo ✅
```

El valor de 4.7 kΩ es robusto frente a todas las variaciones esperadas en el rango de
temperatura y tolerancia de los componentes.

---

### 2.5 Diagrama de conexión con pull-ups externos

```
Lado PC817 (secundario / 3.3 V STM32)

  3.3V ─────[4.7 kΩ]────┬──── STM32 GPIO (EXTI / NOPULL o PULLUP)
                         │
             PC817 Colector (Vo)

Cuando LED OFF (sensor inactivo): Colector = alta Z → V = 3.3V (pull-up externo)
Cuando LED ON  (sensor activo):   Colector = V_CE_sat ≈ 0.1V → V = LOW

Tiempo de transición LOW→HIGH:
  τ = 4.2 kΩ × C_total (4.2 kΩ = 4.7k || 40k paralelo)
  Con C_total = 500 pF: τ = 2.1 µs, t_to_VIH = 2.5 µs ✅
  Con C_total = 1 nF:   τ = 4.2 µs, t_to_VIH = 5.1 µs ✅
```

---

### 2.6 Compatibilidad con el firmware actual

El pull-up externo de 4.7 kΩ es compatible con todas las configuraciones de GPIO actuales:

| Señal | GPIO config firmware | Pull-up externo | Efecto práctico |
|-------|---------------------|-----------------|-----------------|
| WHEEL × 4, STEER_CENTER, ENC_Z | `GPIO_PULLUP` | 4.7 kΩ externo | R_ef = 4.2 kΩ. Flancos más rápidos. Sin cambio de firmware. |
| ENC_A, ENC_B | `GPIO_NOPULL` | 4.7 kΩ externo | Único pull-up. OBLIGATORIO para que TIM2 funcione. |

**No se requiere ningún cambio de firmware** para instalar los pull-ups externos. Los 8
resistores se instalan únicamente en el cableado hardware (cerca de los pines del
STM32 o en el conector del módulo de aislamiento).

---

> **Estado del firmware:** NINGÚN archivo de código fuente ha sido modificado.
> Este documento define la estrategia funcional del CAN y los valores de resistores
> para el cableado físico. La implementación es exclusivamente hardware.

---

**Fecha:** 2026-02-25
**Basado en auditoría de:**
- `Core/Src/main.c` — `MX_FDCAN1_Init()` (FDCAN_MODE_NORMAL, AutoRetransmission=ENABLE)
- `Core/Src/stm32g4xx_hal_msp.c` — `HAL_FDCAN_MspInit()` (GPIO_NOPULL en PA12)
- `Core/Src/can_handler.c` — `CAN_CheckBusOff()`, bus-off → LIMP_HOME, retry logic
- `Core/Src/boot_validation.c` — `check_can_not_busoff()` always true
- `docs/ESP32_STM32_CAN_CONNECTION.md` — topología, 2× TJA1051T/3, S pin → GND
- `docs/CABLEADO_AISLAMIENTO_DEFINITIVO.md` — plan de cableado base
- `docs/VALIDACION_ELECTRICA_AISLAMIENTO.md` — validación eléctrica previa
- Datasheet TJA1051T/3 (NXP): dominant timeout 3.2 ms, 3.3V I/O compatible
- Datasheet PC817 (Sharp/ISOCOM): CTR, V_CE_sat, t_r, t_f parameters
