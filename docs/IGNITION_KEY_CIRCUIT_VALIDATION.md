# 🔧 Validación Eléctrica y Optimización — Circuito Llave de Contacto + Relé de Retención

**Documento técnico de revisión, corrección y mejora del circuito de entrada de llave de contacto 12 V.**

> **Versión:** 1.0  
> **Fecha:** 2026-04-14  
> **Fuentes:** `power_manager.h`, `power_manager.cpp`, `LLAVE_CONTACTO_ENCENDIDO_APAGADO.md`, `SISTEMA_ALIMENTACION_COMPLETO.md`

---

## Índice

1. [Diseño Original (Bajo Revisión)](#1-diseño-original-bajo-revisión)
2. [Validación Funcional del Circuito Original](#2-validación-funcional-del-circuito-original)
3. [Problemas Críticos Identificados](#3-problemas-críticos-identificados)
4. [Diseño Corregido](#4-diseño-corregido)
5. [Lista de Componentes](#5-lista-de-componentes)
6. [Flujo de Señal Paso a Paso](#6-flujo-de-señal-paso-a-paso)
7. [Justificación de Cada Corrección](#7-justificación-de-cada-corrección)
8. [Mejoras de Grado Automotriz (Opcionales)](#8-mejoras-de-grado-automotriz-opcionales)

---

## 1. Diseño Original (Bajo Revisión)

### 1.1 Topología original

```
BAT +12V
   │
   └───(ROJO)───► LLAVE CONTACTO ───(NARANJA)───┬────────────────┐
                   (2 terminales)                │                │
                                                 │                │
                                          [R1 = 33 kΩ]           │
                                                 │                │
                                                 ├──► GPIO 40     │
                                                 │    (ESP32)     │
                                          [R2 = 10 kΩ]           │
                                                 │                │
                                                GND       [D1 = 1N4148] ← EN SERIE
                                                                  │
                                                          Bobina Relé (+)
                                                                  │
                                                              Bobina Relé
                                                                  │
                                                                 GND
```

### 1.2 Descripción de la llave física

La llave de contacto es un **interruptor SPST** (Single Pole, Single Throw) con exactamente **2 terminales / conectores**:

| Terminal | Conexión | Cable |
|----------|----------|-------|
| **Terminal 1 (entrada)** | Batería 12 V (+) | Cable rojo (≥ 0.5 mm²) |
| **Terminal 2 (salida)** | Punto de bifurcación: divisor GPIO + bobina relé | Cable naranja (≥ 0.5 mm²) |

Cuando la llave está en posición **ON**, los dos terminales se cortocircuitan internamente. En posición **OFF**, quedan en circuito abierto.

---

## 2. Validación Funcional del Circuito Original

### 2.1 ¿Funciona eléctricamente?

**Sí, parcialmente.** El circuito original consigue:

- ✅ Detectar la posición de la llave vía GPIO 40 (2.79 V con divisor 33k/10k)
- ✅ Activar el relé de retención cuando la llave está ON
- ✅ El divisor mantiene el voltaje GPIO dentro de límites seguros (< 3.3 V)

**Pero tiene fallos que lo hacen inseguro para uso continuo.**

### 2.2 Modos de fallo identificados

| Modo de fallo | Probabilidad | Severidad | Causa raíz |
|----------------|-------------|-----------|------------|
| Daño ESP32 por pico inductivo | **Alta** | **Crítica** | D1 en serie no protege contra kickback |
| Desgaste prematuro de la llave | Media | Alta | Bobina accionada directamente desde contactos de llave |
| Mis-trigger por ruido EMI | Media | Media | Sin filtro RC en línea GPIO |
| Fallo GPIO por transitorio automotriz | Baja-Media | Crítica | Sin protección TVS/zener |
| Arco eléctrico en contactos de llave | Media | Alta | Corriente inductiva cortada por contacto mecánico |

---

## 3. Problemas Críticos Identificados

### 3.1 ❌ CRÍTICO: Diodo flyback en SERIE (no protege)

**Problema:** En el diseño original, D1 (1N4148) está en **serie** con la bobina del relé, actuando como rectificador simple. Esto **NO proporciona protección contra el kickback inductivo**.

**Consecuencia:** Cuando la llave se abre (OFF), la bobina del relé genera un pico de tensión inverso (V_spike = -L × dI/dt) que puede alcanzar **-60 V a -200 V**. Esta tensión:

- Se propaga al cable naranja compartido con el divisor de tensión
- Llega a GPIO 40 como un pulso negativo destructivo
- Puede destruir el pin GPIO del ESP32 (máx absoluto: -0.3 V a +3.6 V)

**Corrección necesaria:** El diodo flyback debe estar en **paralelo** con la bobina del relé (cátodo a +bobina, ánodo a -bobina/GND).

### 3.2 ❌ IMPORTANTE: Bobina del relé accionada directamente desde la llave

**Problema:** La bobina del relé (típicamente 70-150 mA para relé de 12 V) se alimenta directamente desde los contactos mecánicos de la llave de contacto.

**Consecuencias:**

1. **Desgaste del contacto:** La corriente inductiva de la bobina causa arco eléctrico al abrir/cerrar la llave, erosionando los contactos progresivamente.
2. **Caída de tensión:** La resistencia de contacto de la llave (10-50 mΩ típica, degradándose con el uso) reduce la tensión disponible para la bobina.
3. **Bounce amplificado:** El rebote mecánico de la llave causa ciclos ON/OFF rápidos en la bobina, generando múltiples picos inductivos.
4. **Interferencia con GPIO:** La corriente de bobina (70-150 mA) compartida con el divisor causa fluctuaciones de tensión en GPIO 40.

**Corrección necesaria:** Usar un transistor NPN o MOSFET lógico como driver de la bobina. La llave solo debe conmutar la corriente del divisor de tensión (≈ 0.28 mA).

### 3.3 ⚠️ MEJORABLE: Divisor de tensión sin filtrado

**Estado actual:** R1=33 kΩ / R2=10 kΩ → 2.79 V (✅ dentro de margen)

**Problemas:**

- Sin condensador de filtrado → susceptible a ruido EMI automotriz
- Sin protección contra transitorios (load dump hasta 40 V en 12 V nominal)
- Corriente de divisor: 0.28 mA → adecuada, pero se puede optimizar

---

## 4. Diseño Corregido

### 4.1 Esquema corregido completo

```
BAT +12V
   │
   └───(ROJO)───► LLAVE CONTACTO ───(NARANJA)──┬────────────────────────────────────┐
                   (2 terminales)               │                                     │
                                                │                                     │
                                  ┌─────────────┤                                     │
                                  │             │                                     │
                            ══════════════      │                                     │
                            ║ ETAPA GPIO ║      │                                     │
                            ══════════════      │                                     │
                                  │             │                                     │
                           [R1 = 47 kΩ]         │                                     │
                                  │             │                                     │
                                  ├─[R3 = 10 kΩ]─┐                                   │
                                  │               │                                   │
                                  │          [C1 = 100 nF]                            │
                                  │               │                                   │
                                  ├───► GPIO 40   │                                   │
                                  │    (ESP32)    │                                   │
                           [R2 = 10 kΩ]          GND                                  │
                                  │                                                   │
                                 GND                                                  │
                                                                                      │
                                                                              ════════════════
                                                                              ║ ETAPA RELÉ  ║
                                                                              ════════════════
                                                                                      │
                                                                               [R4 = 10 kΩ]
                                                                                      │
                                                                               Base ──┤
                                                                                      │
                                                                         ┌──── Q1 (BC547 NPN)
                                                                         │     │
                                                              12V ───────┤     │
                                                                  │      │  Emisor
                                                            ┌─────┤      │     │
                                                            │     │      │    GND
                                                      [D2 flyback]│      │
                                                       1N4007│    │      │
                                                            │  Bobina(+)─┘
                                                            │     │
                                                            │  Bobina(−)
                                                            │     │
                                                            └─────┘
                                                                  │
                                                                 GND
```

### 4.2 Esquema simplificado limpio

```
                     ETAPA A: Detección GPIO                    ETAPA B: Control Relé
                     ═══════════════════════                    ═══════════════════════

  BAT 12V (+)                                                  BAT 12V (+)
     │                                                            │
     │                                                            │
  ┌──┴──┐                                                        │
  │LLAVE│ (2 terminales, SPST)                                   │
  │CONT.│                                                        │
  └──┬──┘                                                        │
     │                                                            │
     │ (cable naranja)                                            │
     │                                                            │
     ├──────────────────────────────────────── → ────────────────┤
     │                                                            │
     │                                                     [R4 = 10 kΩ]
  [R1 = 47 kΩ]                                                   │
     │                                              ┌────── Base Q1
     │                                              │      (BC547)
     ├──[R3 = 10 kΩ]──┬──► ESP32 GPIO 40           │   Colector ←── Bobina(−) relé
     │                 │                            │      │
  [R2 = 10 kΩ]   [C1 = 100 nF]                    │   Emisor
     │                 │                            │      │
    GND              GND                           │     GND
                                                    │
                                                    │     Bobina(+) ←── BAT 12V
                                                    │        │  ▲
                                                    │        │  │
                                                    │     ┌──┴──┴──┐
                                                    │     │  D2    │ 1N4007
                                                    │     │flyback │ (cátodo arriba,
                                                    │     │(EN     │  ánodo abajo)
                                                    │     │PARALELO│
                                                    │     └────────┘
                                                    │
                                              (señal ON/OFF
                                               desde cable
                                               naranja)
```

### 4.3 Diagrama de cableado de la llave (vista de taller)

```
  ┌─────────────────────────────────────────────────────────────────────┐
  │                                                                     │
  │   BATERÍA 12V (+)                                                   │
  │        │                                                            │
  │        │  Cable ROJO (≥ 0.5 mm²)                                   │
  │        ▼                                                            │
  │   ┌──────────┐                                                      │
  │   │  LLAVE   │  Terminal 1 (entrada): viene de BAT 12V (+)         │
  │   │ CONTACTO │  Terminal 2 (salida):  sale cable naranja           │
  │   │  (SPST)  │  ⚡ Solo 2 cables/conectores en la llave            │
  │   └────┬─────┘                                                      │
  │        │                                                            │
  │        │  Cable NARANJA (≥ 0.5 mm², señal + activación)            │
  │        │                                                            │
  │        ├──────────────── RAMAL A ──────────────────┐               │
  │        │   (señal GPIO)                            │               │
  │        │                                           │               │
  │        │                                    RAMAL B                │
  │        │                                  (control relé)           │
  │        ▼                                           │               │
  │   ┌─────────┐                                      │               │
  │   │R1 47 kΩ │                               ┌──────┴──────┐       │
  │   └────┬────┘                               │ R4 = 10 kΩ  │       │
  │        │                                    └──────┬──────┘       │
  │        ├──[R3 10kΩ]──[C1 100nF]── GND             │               │
  │        │                                      Base Q1              │
  │        │                                    (BC547 NPN)            │
  │        ├──────► ESP32 GPIO 40                     │               │
  │        │                                     Colector ◄── Bobina(−)│
  │   ┌────┴────┐                                     │               │
  │   │R2 10 kΩ │                                Emisor               │
  │   └────┬────┘                                     │               │
  │        │                                         GND              │
  │       GND                                                          │
  │                          Bobina(+) ◄── BAT 12V (+)                │
  │                               │  ▲                                 │
  │                               │  │                                 │
  │                           ┌───┴──┴───┐                             │
  │                           │ D2 1N4007│  ← FLYBACK EN PARALELO     │
  │                           │(cát.→bob+│    con bobina               │
  │                           │ áno.→bob−│                             │
  │                           └──────────┘                             │
  │                                                                     │
  └─────────────────────────────────────────────────────────────────────┘

  Nota: la señal GPIO 41 (POWER_HOLD) del ESP32 también acciona la bobina
  del relé de retención por un SEGUNDO camino (vía otro transistor NPN),
  en paralelo con el camino de la llave. Ambos caminos se combinan con
  diodos OR (1N4148) para evitar retroalimentación. Ver sección 3.3 de
  LLAVE_CONTACTO_ENCENDIDO_APAGADO.md para el esquema completo del
  circuito de retención.
```

### 4.4 Nota sobre la integración con GPIO 41 (POWER_HOLD)

El circuito completo del relé de retención tiene **dos caminos de activación** en paralelo:

```
  Camino 1: Llave ON → R4 → Q1 → Bobina relé       (activación inicial)
  Camino 2: ESP32 GPIO 41 → R_base → Q2 → Bobina    (retención durante apagado)

  Ambos caminos se combinan con diodos OR (1N4148):

       Llave ON ─── R4 ─── Q1 colector ──┐
                                           ├─── D_OR1 (1N4148) ──► Bobina(+)
  GPIO 41 ─── R_base ─── Q2 colector ─── D_OR2 (1N4148) ──┘

  D2 (1N4007) flyback EN PARALELO con bobina (cátodo a bobina+, ánodo a bobina−/GND)
```

---

## 5. Lista de Componentes

### 5.1 Etapa de detección GPIO (Etapa A)

| Ref. | Componente | Valor | Especificación | Función |
|------|------------|-------|----------------|---------|
| R1 | Resistencia | **47 kΩ** | ¼ W, 1%, metal film | Resistencia superior del divisor optimizado |
| R2 | Resistencia | **10 kΩ** | ¼ W, 1%, metal film | Resistencia inferior del divisor (a GND) |
| R3 | Resistencia | **10 kΩ** | ¼ W, 5% | Resistencia en serie anti-transitorio (limita corriente pico al GPIO) |
| C1 | Condensador cerámico | **100 nF** (0.1 µF) | 50 V, X7R | Filtro RC anti-ruido (τ = R3 × C1 = 1 ms) |
| — | (Opcional) Diodo zener | **3.3 V** | BZX55C3V3 o similar, 500 mW | Clamp de protección (ver §8) |

**Cálculos del divisor optimizado (R1=47k, R2=10k):**

```
V_GPIO = 12 V × 10k / (47k + 10k) = 12 × 10/57 = 2.105 V  ✅

Margen de seguridad:
  - V_GPIO nominal:   2.10 V (< 3.3 V máx. absoluto, margen = +1.2 V)
  - V_GPIO con 14.4V: 2.53 V (< 3.3 V, margen = +0.77 V)  ← carga de alternador
  - V_GPIO con 16V:   2.81 V (< 3.3 V, margen = +0.49 V)  ← load dump transitorio

Umbral HIGH del ESP32-S3: 0.75 × 3.3V = 2.475 V
  → Con 12V nominal: 2.10 V < 2.475 V ⚠️ BAJO

Alternativa recomendada: mantener R1=33 kΩ, R2=10 kΩ (2.79 V ✅ > umbral HIGH)
```

> **⚠️ DECISIÓN DE DISEÑO:** Los valores R1=47k/R2=10k optimizan corriente pero están al límite del umbral HIGH. **Se recomienda mantener R1=33 kΩ / R2=10 kΩ** (2.79 V, diseño original) para garantizar detección fiable. El filtro RC (R3+C1) y la protección zener son las mejoras principales.

### 5.2 Etapa de control de relé (Etapa B)

| Ref. | Componente | Valor | Especificación | Función |
|------|------------|-------|----------------|---------|
| Q1 | Transistor NPN | **BC547B** | V_CE ≥ 45 V, I_C ≥ 100 mA, hFE ≥ 200 | Driver de bobina del relé |
| R4 | Resistencia de base | **10 kΩ** | ¼ W | Limita corriente de base: I_b = (12 V − 0.7 V) / 10 kΩ ≈ 1.13 mA |
| D2 | Diodo flyback | **1N4007** | 1000 V, 1 A | Protección contra kickback inductivo (en **paralelo** con bobina) |
| K1 | Relé de retención | **12 V DC** | Bobina ~100 mA, contactos ≥ 5 A | Mantiene alimentación 12 V → regulador 5 V |

**Cálculos del driver del transistor:**

```
Corriente de bobina del relé (típica): I_C = 80-120 mA
Corriente de base necesaria: I_B = I_C / hFE = 120 mA / 200 = 0.6 mA
Corriente de base real: I_B = (V_llave − V_BE) / R4 = (12 − 0.7) / 10k = 1.13 mA
Factor de saturación: I_B_real / I_B_min = 1.13 / 0.6 = 1.88× ✅ (saturado)
V_CE_sat típico: 0.2 V → Tensión en bobina: 12 − 0.2 = 11.8 V ✅
```

### 5.3 Componentes compartidos / existentes

| Ref. | Componente | Valor | Estado |
|------|------------|-------|--------|
| Llave contacto | SPST, ≥ 12 V / 1 A | **Existente** (2 terminales) |
| Cable rojo | ≥ 0.5 mm² | **Existente** (BAT → llave) |
| Cable naranja | ≥ 0.5 mm² | **Existente** (llave → circuito) |
| Regulador 5 V | LM7805 o buck 12 V→5 V | **Existente** (alimentación ESP32/STM32) |

---

## 6. Flujo de Señal Paso a Paso

### 6.1 Secuencia de encendido (llave ON)

```
Paso 1: Usuario gira llave → contactos internos cierran → 12V aparece en cable naranja

Paso 2: RAMAL A (GPIO)
         12V → R1 (33kΩ) → nodo → R2 (10kΩ) → GND
                                  ↓
                             R3 (10kΩ) → C1 (100nF) ─ GND   [filtro se carga en ~1ms]
                                  ↓
                            GPIO 40 = 2.79 V (HIGH)
                                  ↓
                         ESP32 detecta llave ON (tras debounce 50ms en firmware)

Paso 3: RAMAL B (Relé)
         12V → R4 (10kΩ) → Base Q1 → I_B = 1.13 mA → Q1 saturado
                                           ↓
                                    Colector Q1 → sink → bobina relé energizada
                                           ↓
                                    Relé cierra contactos → 12V → regulador 5V → ESP32/STM32

Paso 4: ESP32 arranca → GPIO 41 HIGH → segundo camino mantiene relé (retención)
```

### 6.2 Secuencia de apagado (llave OFF)

```
Paso 1: Usuario gira llave OFF → contactos abren → 12V desaparece de cable naranja

Paso 2: RAMAL A (GPIO)
         Sin 12V → divisor descarga → C1 descarga vía R2+R3 (τ ≈ 1ms)
         → GPIO 40 = LOW (tras debounce 50ms firmware)
         → ESP32: estado SHUTTING_DOWN

Paso 3: RAMAL B (Relé)
         Sin 12V → Q1 se apaga → corriente de bobina se corta
         → D2 (1N4007) conduce corriente inductiva de bobina (flyback)
         → Energía disipada en D2 + resistencia de bobina (~1-5 ms)
         → NO hay pico de tensión destructivo ✅

         PERO: GPIO 41 sigue HIGH → Q2 mantiene relé energizado
         → Relé permanece cerrado → sistema sigue alimentado

Paso 4: ESP32 reproduce audio despedida (3 s) → GPIO 41 LOW → Q2 se apaga
         → D2 conduce flyback → relé se abre → alimentación cortada
```

### 6.3 Respuesta ante key bounce (rebote de llave)

```
  Sin filtro RC:                           Con filtro RC (R3=10kΩ, C1=100nF):
  
  Llave ──┐ ┌─┐ ┌─┐ ┌──── 12V            Llave ──┐ ┌─┐ ┌─┐ ┌──── 12V
           │ │ │ │ │ │                              │ │ │ │ │ │
           │ │ │ │ │ │                              │ │ │ │ │ │
  GPIO  ───┘ └─┘ └─┘ └──── GND            GPIO  ───────────/──── (rampa suave)
         múltiples flancos                         sin flancos espurios
         → posibles glitches                       τ = 1 ms absorbe bounce
                                                   + debounce firmware 50 ms
```

---

## 7. Justificación de Cada Corrección

### 7.1 Flyback en PARALELO (no en serie)

| Aspecto | Diseño original | Diseño corregido |
|---------|----------------|------------------|
| **Colocación D2** | En serie con bobina | En **paralelo** con bobina |
| **Pico inductivo** | Se propaga al circuito (−60V a −200V) | Clamped a +0.7 V sobre Vbobina |
| **Riesgo GPIO** | Destrucción de pin ESP32 | Eliminado |
| **Componente** | 1N4148 (100V, 200mA) | **1N4007** (1000V, 1A) — más robusto |

**Funcionamiento:** Al cortar la corriente de bobina, la energía almacenada en la inductancia (E = ½LI²) se disipa a través de D2 como corriente circulante (bobina → D2 → bobina). La tensión en los terminales de la bobina se limita a V_supply + V_forward(D2) ≈ 12.7 V.

### 7.2 Transistor driver (no accionamiento directo)

| Aspecto | Diseño original | Diseño corregido |
|---------|----------------|------------------|
| **Corriente por llave** | ~70-150 mA (bobina) + 0.28 mA (divisor) | Solo 0.28 mA (divisor) + 1.13 mA (base Q1) ≈ **1.4 mA** |
| **Arco en contactos** | Sí (corriente inductiva) | No (corriente resistiva mínima) |
| **Vida útil llave** | ~10k-50k ciclos (con arco) | ~100k-500k ciclos (sin arco) |
| **Caída de tensión** | Variable (resistencia de contacto) | Despreciable (1.4 mA × 50 mΩ = 70 µV) |

### 7.3 Filtro RC en línea GPIO

| Aspecto | Diseño original | Diseño corregido |
|---------|----------------|------------------|
| **Filtro** | Ninguno | R3=10kΩ + C1=100nF (τ=1ms) |
| **Ruido EMI** | Susceptible | Atenuado (filtro paso bajo, f_c = 159 Hz) |
| **Key bounce** | Pasa al GPIO | Suavizado por constante RC |
| **Impedancia GPIO** | Solo divisor (7.67 kΩ eq.) | +10kΩ serie (protección ESD mejorada) |

### 7.4 Resistencia serie R3 como protección adicional

R3 (10 kΩ) en serie con GPIO 40 limita la corriente de fallo a:

```
I_max = V_max / R3 = 16V / 10kΩ = 1.6 mA  (< 12 mA máx. absoluto ESP32 por pin)
```

Incluso si el divisor falla (R2 abierto), R3 protege el pin GPIO.

---

## 8. Mejoras de Grado Automotriz (Opcionales)

### 8.1 Protección TVS / Zener

Añadir un diodo zener de 3.3 V (BZX55C3V3) entre GPIO 40 y GND:

```
  R1 ──── R3 ──┬──► GPIO 40
               │
          [ZD1 3.3V] ← cátodo a GPIO, ánodo a GND
               │
              GND
```

**Beneficio:** Clamp absoluto a 3.3 V. Protege contra:
- Load dump automotriz (hasta 40 V en el bus 12 V)
- Transitorios ISO 7637 (±100 V picos)

**Alternativa:** TVS bidireccional (SMBJ3.3A) para protección contra transitorios negativos también.

### 8.2 Condensador de desacoplo en entrada del divisor

```
  Cable naranja ──┬── [C2 = 10 nF / 50V] ── GND
                  │
                  └── R1 ── (resto del divisor)
```

**Beneficio:** Filtra ruido de alta frecuencia antes del divisor. C2 en paralelo con la entrada absorbe picos rápidos (< 1 µs).

### 8.3 Optoacoplador para aislamiento galvánico completo

Para entornos con ruido severo (motores DC con escobillas, alternador):

```
  Cable naranja ── R5 (4.7kΩ) ── LED opto ── GND
                                      │
                              ┌───────┴────────┐
                              │ PC817 / 6N137  │
                              │ Fototransistor │
                              └───────┬────────┘
                                      │
                                 3.3V ── R6 (10kΩ) ── GPIO 40
```

**Beneficio:** Aislamiento completo entre el circuito de 12 V y el ESP32. Elimina ruido de modo común. Consistente con la arquitectura de los relés de potencia del STM32 (que ya usan optoacopladores HY-M158).

**Coste:** +1 optoacoplador, +2 resistencias. Retardo típico: < 10 µs (despreciable para detección de llave).

### 8.4 Tabla resumen de mejoras opcionales

| Mejora | Componentes | Coste est. | Beneficio | Recomendación |
|--------|------------|------------|-----------|---------------|
| Zener 3.3V | ZD1 + (ya tiene R3) | ~0.05 € | Protección transitorios | **Recomendado** |
| C2 desacoplo | 10 nF / 50V | ~0.02 € | Filtro HF adicional | Opcional |
| Optoacoplador | PC817 + 2R | ~0.30 € | Aislamiento total | Para entorno ruidoso |
| TVS bidireccional | SMBJ3.3A | ~0.15 € | Protección ±100V | Producción seria |

---

## Resumen de Cambios: Original → Corregido

```
  ORIGINAL (funcional pero frágil)         CORREGIDO (robusto y seguro)
  ════════════════════════════════         ═══════════════════════════════

  D1 en SERIE con bobina                  D2 en PARALELO con bobina ✅
  → Sin protección flyback                → Flyback absorbido por D2

  Bobina directa desde llave              Q1 (NPN) como driver de bobina ✅
  → Arco, desgaste, 150 mA por llave     → Llave solo conduce 1.4 mA

  Sin filtro en GPIO                      R3 + C1 (filtro RC, τ=1ms) ✅
  → Susceptible a EMI y bounce            → Ruido filtrado, bounce suavizado

  Sin protección de sobretensión          R3 limita corriente de fallo ✅
  → GPIO expuesto a transitorios          → Opción: añadir zener 3.3 V

  Divisor 33k/10k = 2.79 V               Divisor 33k/10k = 2.79 V (sin cambio)
  → Adecuado                              → Se mantiene (margen > umbral HIGH)
```

---

> **Referencia firmware:** `esp32/src/power_manager.h` (GPIO 40/41), `esp32/src/power_manager.cpp` (debounce 50ms, state machine)  
> **Referencia docs:** `docs/LLAVE_CONTACTO_ENCENDIDO_APAGADO.md` (circuito original), `Documentos/SISTEMA_ALIMENTACION_COMPLETO.md` (alimentación general)
