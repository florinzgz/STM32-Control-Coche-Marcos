# INVENTARIO DE COMPONENTES FÍSICOS — Proyecto Marcos

> **Documento de inventario real del material disponible en mano.**  
> Fecha creación: 2026-04-28  
> Versión: 1.2 (corregido R3 kit 1/4W, añadido análisis kit ALLECIN condensadores)  
> Fuente: fotografías del material real + verificación contra firmware  
> Referencia firmware: `Core/Inc/project_config.h`, `Documentos/SISTEMA_ALIMENTACION_COMPLETO.md`

---

## Índice

1. [Resumen rápido de stock](#1-resumen-rápido-de-stock)
2. [Condensadores cerámicos](#2-condensadores-cerámicos)
3. [Condensadores electrolíticos](#3-condensadores-electrolíticos)
4. [Resistencias](#4-resistencias)
5. [Diodos](#5-diodos)
6. [Análisis kit ALLECIN 240PCS electrolíticos](#6-análisis-kit-allecin-240pcs-electrolíticos)
7. [Mapa de montaje — dónde va cada condensador](#7-mapa-de-montaje--dónde-va-cada-condensador)
8. [Mapa de montaje — dónde va cada resistencia](#8-mapa-de-montaje--dónde-va-cada-resistencia)
9. [Qué falta comprar](#9-qué-falta-comprar)

---

## 1. Resumen rápido de stock

| # | Componente | Valor / Tipo | Cant. | Estado |
|---|-----------|-------------|-------|--------|
| C1 | Cerámico monolítico 50V | **100 nF** (código 104) | 100 uds | ✅ EN MANO |
| C2 | Cerámico monolítico 50V | **1 µF** (código 105) | 100 uds | ✅ EN MANO |
| C3 | Electrolítico 35V −40+105°C | **2200 µF** (16×25 mm) | ~5 uds | ✅ EN MANO |
| R1 | Resistencia metal film **3W** 1% | **120 Ω** | 20 uds | ✅ EN MANO |
| R2 | Resistencia metal film **1/8W** ±5% | Kit **400 uds / 40 valores** (AUKENIEN) | 400 uds | ✅ EN MANO |
| R3 | Resistencia metal film **1/4W** 1% | Kit **600 uds / 30 valores** (10Ω → 1MΩ) | 600 uds | ✅ EN MANO |
| D1 | TVS bidireccional DO-15 | **P6KE18CA** (18 V) | 20 uds | ✅ EN MANO |
| D2 | TVS bidireccional DO-15 | **P6KE24CA** (24 V) | 20 uds | ✅ EN MANO |
| D3 | Diodo rectificador DO-27 | **1N5408** (3 A / 1000 V) | 50 uds | ✅ EN MANO |

---

## 2. Condensadores cerámicos

### 2.1 — 100 nF / 50 V (código **104**)

| Campo | Dato |
|-------|------|
| Referencia bolsa | `dushi-50V-104-Bao` |
| Fabricante | Shenzhen Yunke Electronics Co., Ltd. |
| Código cerámico | **104** (10 × 10⁴ pF = 100 nF) |
| Tensión nominal | 50 V |
| Tipo | Monolítico cerámico (disco amarillo) |
| Cantidad | **100 piezas** |
| Temperatura | Estándar (verificar marcado) |

**Uso en el proyecto:** bypass/desacoplo rápido (respuesta < 1 ns), snubber anti-EMI motores, snubber RC relés.

---

### 2.2 — 1 µF / 50 V (código **105**)

| Campo | Dato |
|-------|------|
| Referencia bolsa | `dushi-50V-105-Bao` |
| Fabricante | Shenzhen Yunke Electronics Co., Ltd. |
| Código cerámico | **105** (10 × 10⁵ pF = 1 µF) |
| Tensión nominal | 50 V |
| Tipo | Monolítico cerámico (disco amarillo) |
| Cantidad | **100 piezas** |

**Uso en el proyecto:** bypass intermedio en líneas de señal, filtrado I2C, acoplamiento adicional CAN.

---

## 3. Condensadores electrolíticos

### 3.1 — 2200 µF / 35 V (16×25 mm)

| Campo | Dato |
|-------|------|
| Referencia bolsa | `GH-302_5 / 13293635` |
| Valor | **2200 µF** |
| Tensión nominal | **35 V** |
| Tamaño físico | **16 mm diámetro × 25 mm alto** |
| Temperatura | −40 a +105 °C |
| Marca | Nenox (o equivalente 105°C) |
| Cantidad aprox. | **~5 piezas** |
| ⚠️ Polaridad | PIN LARGO (+), PIN CORTO (−) / Franja blanca = (−) |

**Uso en el proyecto:** bulk de alta capacidad en rail de potencia 24 V / 5 V.  
**Nota:** 2200 µF > 470 µF requerido → válido para todas las posiciones BTS7960.  
**Nota:** 35 V > 5 V rail lógico → válido también para posiciones LED WS2812B (diseño pide 1000 µF).

---

## 4. Resistencias

### 4.1 — Kit 3W Metal Film 1% — 120 Ω (20 piezas)

| Campo | Dato |
|-------|------|
| Referencia | `3w jin shu mo-20PCS-120R` |
| Potencia | **3 W** |
| Tolerancia | **1%** |
| Valor de esta bolsa | **120 Ω** |
| Rango disponible del fabricante | 7Ω, 10Ω, 47Ω, 68Ω, 100Ω, 220Ω, 360Ω, 470Ω, 1kΩ, 2.2kΩ, 10kΩ, 22kΩ, 47kΩ, 4.7kΩ, 100kΩ, 1MΩ |
| Cantidad | **20 piezas (120 Ω)** |
| Color cuerpo | Azul con franjas de colores |

**Uso en el proyecto:** Las de 120 Ω a 3W sirven como **resistencias de terminación CAN** (se necesitan 2×120 Ω, una en cada extremo del bus). Las demás podrían usarse como divisores de potencia o limitadores en aplicaciones de alta corriente.

---

### 4.2 — Kit 1/8W Resistor — AUKENIEN 400 piezas / 40 valores

| Campo | Dato |
|-------|------|
| Marca | **AUKENIEN** |
| Potencia | **1/8 W (0.125 W)** |
| Tolerancia | **±5%** |
| Total piezas | **400 (10 piezas × 40 valores)** |
| Tipo | Metal film de 4 bandas |

**Valores incluidos (10 piezas de cada uno):**

| Columna 1 | Columna 2 | Columna 3 |
|-----------|-----------|-----------|
| 0 Ω | 150 Ω | 22 kΩ |
| 1 Ω | 200 Ω | 33 kΩ |
| 2 Ω | 220 Ω | 47 kΩ |
| 2.2 Ω | 330 Ω | 100 kΩ |
| 3 Ω | 470 Ω | 120 kΩ |
| 3.3 Ω | 680 Ω | 150 kΩ |
| 4.7 Ω | 1 kΩ | 200 kΩ |
| 10 Ω | 1.5 kΩ | 220 kΩ |
| 22 Ω | 2 kΩ | 330 kΩ |
| 33 Ω | 2.2 kΩ | 470 kΩ |
| 47 Ω | 3.3 kΩ | 1 MΩ |
| 68 Ω | 4.7 kΩ | 2.2 MΩ |
| 100 Ω | 10 kΩ | |
| 120 Ω | 20 kΩ | |

**Valores críticos para el proyecto disponibles en este kit:**

| Resistencia | Valor | Función en proyecto | Qty necesaria |
|-------------|-------|---------------------|--------------|
| Pull-up encoder 6N137 | **4.7 kΩ** | PA15, PB3, PB4 (×3) | 3 |
| Pull-up I2C (PB6/PB7) | **4.7 kΩ** | SCL + SDA × 2 buses | 4 |
| Divisor pedal Hall R2 | **10 kΩ** | PA3 (ADC pedal) | 1 |
| Divisor llave R1 | **33 kΩ** | GPIO40 ESP32 (12V→3.3V) | 1 |
| Divisor llave R2 | **10 kΩ** | GPIO40 ESP32 (12V→3.3V) | 1 |
| Base BC547 retención | **1 kΩ** | GPIO41 ESP32 → transistor | 1 |
| Entrada 6N137 encoder | **330 Ω** | R_IN encoder (×3) | 3 |
| Datos WS2812B serie | **330 Ω** | Protección LED DIN (×2) | 2 |
| Snubber RC relé | **100 Ω** | Paralelo contactos (×3) | 3 |

### 4.2 — Kit 1/4W Metal Film 1% — AUKENIEN estilo, 600 piezas / 30 valores (10Ω → 1MΩ)

| Campo | Dato |
|-------|------|
| Referencia imagen | `fbdbbc5f-b78b-4984-9e67-0e78b8a4a9bd` |
| Potencia | **1/4 W (0.25 W)** |
| Tolerancia | **1%** |
| Total piezas | **600 (20 piezas × 30 valores)** |
| Tipo | Metal film de 5 bandas (4ª banda multiplicador, 5ª tolerancia) |
| Presentación | Cajas compartimentadas, cada valor en su compartimento |

**Valores incluidos (20 piezas de cada uno):**

| Col A | Col B | Col C | Col D | Col E |
|-------|-------|-------|-------|-------|
| 10 Ω | 150 Ω | 2 kΩ | 20 kΩ | 220 kΩ |
| 22 Ω | 200 Ω | 2.2 kΩ | 22 kΩ | 300 kΩ |
| 47 Ω | 220 Ω | 3.3 kΩ | 33 kΩ | 470 kΩ |
| 100 Ω | 270 Ω | 4.7 kΩ | 47 kΩ | 680 kΩ |
| 120 Ω | 330 Ω | 5.1 kΩ | 51 kΩ | 1 MΩ |
| 150 Ω | 470 Ω | 6.8 kΩ | 68 kΩ | |
| | 510 Ω | 10 kΩ | 100 kΩ | |
| | 680 Ω | | | |
| | 1 kΩ | | | |
| | 1.5 kΩ | | | |

**Ventaja frente al kit 1/8W:** doble potencia (0.25 W vs 0.125 W). Para snubbers de relé y cualquier posición con cierta disipación, usar estas en lugar de las 1/8W.

**Valores críticos disponibles en este kit (complementan al 1/8W):**

| Resistencia | Valor | Función en proyecto | Qty necesaria | ¿En 1/8W también? |
|-------------|-------|---------------------|--------------|-------------------|
| Snubber RC relé | **100 Ω / 0.25W** | Paralelo contactos MAIN/TRAC/DIR | 3 | ⚠️ En 1/8W es 0.125W — **USAR ESTE 1/4W** |
| Serie datos WS2812B | **330 Ω / 0.25W** | Protección DIN LED (×2) | 2 | ✅ suficiente en 1/8W también |
| Pull-up I2C / encoder | **4.7 kΩ** | SCL/SDA + salida 6N137 | 7 | ✅ En ambos kits |
| Divisor pedal Hall | **10 kΩ** | PA3 ADC pedal | 1 | ✅ En ambos kits |
| Divisor llave R1 | **33 kΩ** | GPIO40 ESP32 | 1 | ✅ En 1/8W (33kΩ existe) |
| Base BC547 | **1 kΩ** | GPIO41 ESP32 | 1 | ✅ En ambos kits |

---

## 5. Diodos

### 5.1 — P6KE18CA — TVS bidireccional 18 V

| Campo | Dato |
|-------|------|
| Referencia | P6KE18CA |
| Tipo | TVS bidireccional (CA = bidireccional) |
| Tensión standoff | 15.3 V |
| Tensión de clamp | ~29.2 V (a 1 A) |
| Potencia pico | 600 W (10/1000 µs) |
| Encapsulado | DO-15 |
| Cantidad | **20 piezas** |

**Uso:** Protección ESD/sobretensión en salidas de los optoacopladores PC817 (5 canales × 4 =posibles 20 nodos protegidos). También válido en pines CAN.

---

### 5.2 — P6KE24CA — TVS bidireccional 24 V

| Campo | Dato |
|-------|------|
| Referencia | P6KE24CA |
| Tipo | TVS bidireccional |
| Tensión standoff | 20.5 V |
| Tensión de clamp | ~38.9 V (a 1 A) |
| Potencia pico | 600 W (10/1000 µs) |
| Encapsulado | DO-15 |
| Cantidad | **20 piezas** |

**Uso:** Protección en líneas de 24 V (baterías de tracción), protección de contactos relés MAIN/TRAC/DIR.

---

### 5.3 — 1N5408 — Diodo rectificador 3A/1000V

| Campo | Dato |
|-------|------|
| Referencia | 1N5408 |
| Corriente continua | 3 A |
| Tensión inversa | 1000 V |
| Encapsulado | DO-27 |
| Cantidad | **50 piezas** |

**Uso:** Flyback antiparalelo con bobinas de relés (MAIN, TRAC, DIR, LED, LED_REAR, audio).  
Sustituye perfectamente al 1N4007 (3A > 1A, 1000V = igual) — **más seguro, mismo coste**.  
Usar: **5 diodos** para los 5 módulos de relé + **1** para relé retención = **6 en total**, sobran 44.

---

---

## 6. Análisis kit ALLECIN 240PCS electrolíticos

> **Estado:** Kit a comprar — imagen referencia `913b0f6f-de4c-49f0-b911-53bc022b7e3e`  
> **Marca:** ALLECIN  
> **Descripción:** 24 Values Kit Surtido de Condensadores Electrolíticos 0.1µF → 1000µF, tensiones 10V/16V/25V/50V

### 6.1 — Contenido completo del kit (24 valores × 10 piezas = 240 total)

| Valor | Tensión | Tamaño | Qty |
|-------|---------|--------|-----|
| 0.1 µF | 50 V | 4×7 | 10 |
| 0.22 µF | 50 V | 5×11 | 10 |
| 0.47 µF | 50 V | 5×11 | 10 |
| 1 µF | 50 V | 5×11 | 10 |
| 2.2 µF | 50 V | 4×7 | 10 |
| 3.3 µF | 50 V | 4×7 | 10 |
| 4.7 µF | 50 V | 4×7 | 10 |
| 10 µF | 25 V | 4×7 | 10 |
| 10 µF | 50 V | 5×11 | 10 |
| 22 µF | 16 V | 4×7 | 10 |
| 22 µF | 25 V | 4×7 | 10 |
| 33 µF | 16 V | 4×7 | 10 |
| **47 µF** | **10 V** | 4×7 | **10** |
| **47 µF** | **25 V** | 5×11 | **10** |
| 47 µF | 50 V | 6×11 | 10 |
| 100 µF | 16 V | 5×11 | 10 |
| 100 µF | 25 V | 5×11 | 10 |
| 220 µF | 10 V | 5×11 | 10 |
| 220 µF | 25 V | 6×11 | 10 |
| 330 µF | 25 V | 8×12 | 10 |
| 470 µF | 10 V | 6×11 | 10 |
| 470 µF | 16 V | 8×12 | 10 |
| 680 µF | 16 V | 8×12 | 10 |
| **1000 µF** | **16 V** | 10×16 | **10** |

### 6.2 — ¿Te sirve para el proyecto? Análisis posición a posición

| Posición | Valor necesario | ¿En el kit? | Resultado | Notas |
|----------|----------------|-------------|-----------|-------|
| Bulk 5V LEDs frontal (PB10) | 1000 µF / ≥6V | 1000µF / **16V** ✅ | ✅ **CUBIERTO** | 16V > 5V, correcto |
| Bulk 5V LEDs trasero (PB11) | 1000 µF / ≥6V | 1000µF / **16V** ✅ | ✅ **CUBIERTO** | 16V > 5V, correcto |
| Bulk salida LM2596 (5V) | 47 µF / ≥6V | 47µF / **10V** ✅ | ✅ **CUBIERTO** | 10V > 5V, perfecto |
| Bulk 3.3V STM32 Nucleo | 10 µF / ≥4V | 10µF / **25V** ✅ | ✅ **CUBIERTO** | 25V > 3.3V, correcto |
| Bulk CAN lado ESP32 | 10 µF / ≥6V | 10µF / **25V** ✅ | ✅ **CUBIERTO** | 25V > 5V, correcto |
| Bulk bus 12V dirección | 470 µF / ≥14V | 470µF / **16V** ✅ | ✅ **CUBIERTO** | 16V > 12V, justo pero OK |
| **Bulk bus 24V relés** | **1000 µF / ≥26V** | 1000µF / **16V** ❌ | ❌ **NO VÁLIDO** | 16V < 24V → **PELIGRO** |

### 6.3 — Veredicto

**✅ SÍ TE SIRVE para 6 de las 7 posiciones faltantes.**

**❌ LA ÚNICA POSICIÓN NO CUBIERTA:** Bulk bus 24V junto a relés (necesita ≥26V).

**Solución para la posición 24V:** Usa uno de los **2200 µF / 35V que ya tienes en mano** para esa posición. Si los 5 están ocupados en los BTS7960, puedes comprar uno suelto extra o verificar si tienes alguno de sobra.

### 6.4 — Posiciones cubierta vs. no cubierta — Resumen visual

```
Con el kit ALLECIN:

  Rail 5V  ──[1000µF/16V]──► RELAY_LED frontal (PB10)  ✅ kit
  Rail 5V  ──[1000µF/16V]──► RELAY_LED trasero (PB11)  ✅ kit
  Rail 5V  ──[47µF/10V]───► Salida LM2596 (bulk)        ✅ kit
  Rail 3.3V──[10µF/25V]───► STM32 Nucleo bulk           ✅ kit
  Rail 5V  ──[10µF/25V]───► SN65HVD230 CAN (ESP32)     ✅ kit
  Rail 12V ──[470µF/16V]──► BTS7960 STEER bulk          ✅ kit

  Rail 24V ──[1000µF/16V]─► Bulk relés MAIN/TRAC        ❌ NO USAR
               └── Usar en cambio: [2200µF/35V] ya en mano ✅
```

---

## 7. Mapa de montaje — dónde va cada condensador

### ✅ Condensadores 100 nF / 50 V (código 104) — TIENES 100, NECESITAS ~22

```
┌─────────────────────────────────────────────────────────────────────┐
│  MÓDULO                  │ Cant │ Punto de soldadura                │
├─────────────────────────────────────────────────────────────────────┤
│  BTS7960 FL (motor FL)   │  3   │ VCC→GND (×1) + M+→GND (×1)       │
│                          │      │ + M−→GND (×1)                    │
│  BTS7960 FR (motor FR)   │  3   │ igual que FL                      │
│  BTS7960 RL (motor RL)   │  3   │ igual que FL                      │
│  BTS7960 RR (motor RR)   │  3   │ igual que FL                      │
│  BTS7960 STEER (dirección│  3   │ igual que FL                      │
├─────────────────────────────────────────────────────────────────────┤
│  RELAY_MAIN (PC10)       │  1   │ Paralelo contactos NO (+ R=100Ω) │
│  RELAY_TRAC (PC11)       │  1   │ Paralelo contactos NO (+ R=100Ω) │
│  RELAY_DIR  (PC12)       │  1   │ Paralelo contactos NO (+ R=100Ω) │
├─────────────────────────────────────────────────────────────────────┤
│  SN65HVD230 CAN STM32    │  1   │ Pin VCC → GND (junto al IC)      │
│  SN65HVD230 CAN ESP32    │  1   │ Pin VCC → GND (junto al IC)      │
├─────────────────────────────────────────────────────────────────────┤
│  Nucleo STM32 (3.3V)     │  2   │ Pin 3V3 → GND (×2, cerca Nucleo) │
├─────────────────────────────────────────────────────────────────────┤
│  TOTAL USADOS            │ 22   │ Quedan 78 de reserva ✅           │
└─────────────────────────────────────────────────────────────────────┘
```

**⚠️ Orientación:** No polarizados — se puede soldar en cualquier sentido.  
**⚠️ Posición:** Lo más cerca posible del componente (< 5 mm del pin de alimentación).

---

### ✅ Condensadores 1 µF / 50 V (código 105) — TIENES 100, NECESITAS ~6

```
┌─────────────────────────────────────────────────────────────────────┐
│  MÓDULO                  │ Cant │ Punto de soldadura                │
├─────────────────────────────────────────────────────────────────────┤
│  TCA9548A I2C mux        │  1   │ Pin VCC → GND                    │
│  INA226 sensor ×6        │  1   │ Pin VCC → GND (uno compartido)   │
│  SN65HVD230 CAN STM32    │  1   │ VCC→GND adicional (bypass medio) │
│  SN65HVD230 CAN ESP32    │  1   │ VCC→GND adicional (bypass medio) │
│  DFPlayer Mini (ESP32)   │  1   │ VCC→GND (5V pin)                 │
│  ESP32-S3 DevKitC        │  1   │ 3.3V → GND (bulk adicional)      │
├─────────────────────────────────────────────────────────────────────┤
│  TOTAL USADOS            │  6   │ Quedan 94 de reserva ✅           │
└─────────────────────────────────────────────────────────────────────┘
```

**⚠️ Orientación:** No polarizados — se puede soldar en cualquier sentido.

---

### ✅ Condensadores 2200 µF / 35 V — TIENES ~5, NECESITAS 5

```
┌─────────────────────────────────────────────────────────────────────┐
│  MÓDULO                  │ Cant │ Punto de soldadura                │
├─────────────────────────────────────────────────────────────────────┤
│  BTS7960 FL              │  1   │ B+(24V) → GND  ← PIN LARGO al +  │
│  BTS7960 FR              │  1   │ B+(24V) → GND  ← PIN LARGO al +  │
│  BTS7960 RL              │  1   │ B+(24V) → GND  ← PIN LARGO al +  │
│  BTS7960 RR              │  1   │ B+(24V) → GND  ← PIN LARGO al +  │
│  BTS7960 STEER           │  1   │ B+(12V) → GND  ← PIN LARGO al +  │
├─────────────────────────────────────────────────────────────────────┤
│  TOTAL USADOS            │  5   │ JUSTO — si necesitas más: comprar │
└─────────────────────────────────────────────────────────────────────┘
```

**⚠️ POLARIDAD OBLIGATORIA:**
- **PIN LARGO** = ánodo (+) → conectar al rail de potencia (24V o 12V)
- **PIN CORTO** = cátodo (−) → conectar a GND
- **Franja blanca** impresa en el cuerpo = indica el (−)
- ⛔ Si se invierte: el condensador puede explotar

**Nota:** 2200 µF > 470 µF especificados en el diseño → **MEJOR**, mayor absorción de inrush.  
**Nota:** 35 V ≥ 24 V de pico en tracción → **OK para tracción y dirección**.

---

## 8. Mapa de montaje — dónde va cada resistencia

### 8.1 Resistencias de 3W (120 Ω) — uso en terminación CAN

```
Kit 3W 120Ω:
  ├── 2 piezas → Terminación bus CAN (120Ω en cada extremo del bus diferencial)
  │              CANH ──[120Ω]── CANL  (una en el nodo STM32, otra en el nodo ESP32)
  │
  └── 18 piezas → RESERVA (posibles aplicaciones de potencia futuras)
```

### 8.2 Resistencias de 1/8W — uso en señales

```
Kit AUKENIEN 1/8W:
  ├── 330Ω × 3 → Entrada 6N137 encoder (R_IN: PA15, PB3, PB4)
  ├── 330Ω × 2 → Datos WS2812B serie (protección DIN antes del primer LED)
  ├── 100Ω × 3 → Snubber RC relés (serie con 100nF en contactos MAIN/TRAC/DIR)
  ├── 4.7kΩ × 3 → Pull-up salida 6N137 (a 3.3V: PA15, PB3, PB4)
  ├── 4.7kΩ × 4 → Pull-up bus I2C (PB6 SCL × 2 + PB7 SDA × 2)
  ├── 10kΩ × 1 → Divisor pedal Hall R2 (PA3, mitad inferior del divisor)
  ├── 10kΩ × 1 → Divisor llave contacto R2 (GPIO40 ESP32)
  ├── 33kΩ × 1 → Divisor llave contacto R1 (GPIO40 ESP32)
  └── 1kΩ × 1  → Base transistor BC547 (relé de retención, GPIO41 ESP32)
```

---

## 9. Qué falta comprar

### 9.1 Condensadores electrolíticos — ESTADO CON KIT ALLECIN

> Si compras el kit ALLECIN 240PCS, las posiciones de abajo quedan cubiertas:

| Posición | Valor | Para qué | Qty | Con ALLECIN | Sin ALLECIN |
|----------|-------|----------|-----|-------------|-------------|
| Bulk 5V LEDs WS2812B frontal | 1000 µF / 16V | Rail 5V → RELAY_LED (PB10) | 1 | ✅ En kit | ❌ Falta |
| Bulk 5V LEDs WS2812B trasero | 1000 µF / 16V | Rail 5V → RELAY_LED_REAR (PB11) | 1 | ✅ En kit | ❌ Falta |
| Bulk salida LM2596 | 47 µF / 10V | Salida regulador 5V | 1 | ✅ En kit | ❌ Falta |
| Bulk 3.3V STM32 | 10 µF / 25V | Rail 3.3V Nucleo | 1 | ✅ En kit | ❌ Falta |
| Bulk CAN ESP32 | 10 µF / 25V | SN65HVD230 lado ESP32 | 1 | ✅ En kit | ❌ Falta |
| Bulk bus 12V dirección | 470 µF / 16V | BTS7960 STEER B+(12V) | 1 | ✅ En kit | ❌ Falta |
| **Bulk bus 24V relés** | **≥1000 µF / 35V** | Rail 24V junto a RELAY_MAIN/TRAC | 1 | ❌ **No cubre (16V insuficiente)** | ❌ Falta |

> **Para el bulk 24V:** Usar uno de los **2200µF/35V ya en mano** (si sobra uno de los 5 para BTS7960). Si no, comprar 1 pieza suelta de 1000µF/35V o 2200µF/35V.

### 9.2 Componentes — FALTAN (no en inventario actual)

| Componente | Especificación | Para qué | Qty |
|-----------|---------------|----------|-----|
| Transistor NPN | BC547 o 2N2222 | Relé de retención (GPIO41 ESP32 → bobina) | 1 |
| Diodo | 1N4148 (×2 en OR) | Activación bobina relé retención | 2 |
| Ferrita/bobina | BLM18AG601SN1D o 100µH | Filtro bus CAN (5V sucio → limpio) | 1 |
| Optoacoplador | **PC817** | 5 canales: 4× rueda + 1× centro dirección | 5 |
| Optoacoplador | **6N137** | Encoder E6B2-CWZ6C (×3 canales: A, B, Z) | 3 |

### 9.3 Estado COMPLETAMENTE CUBIERTO por inventario actual

| Función | Componente disponible | ✅ |
|---------|----------------------|---|
| Terminación CAN (2×120Ω) | Kit 3W 120Ω | ✅ |
| Flyback todos los relés (×6) | 1N5408 ×50 | ✅ |
| TVS protección PC817 (×5 canales) | P6KE18CA ×20 | ✅ |
| TVS protección 24V rail | P6KE24CA ×20 | ✅ |
| Bypass 100nF todos módulos | Cerámica 104 ×100 | ✅ |
| Bypass 1µF ICs sensores | Cerámica 105 ×100 | ✅ |
| Bulk 2200µF BTS7960 ×5 | Electrolítico 2200µF/35V ×5 | ✅ |
| Pull-up 4.7kΩ I2C + encoder | Kit 1/4W (4.7kΩ ×20) o Kit 1/8W (4.7kΩ ×10) | ✅ |
| Divisor pedal + llave | Kit 1/4W (10kΩ, 33kΩ) o Kit 1/8W | ✅ |
| Base BC547 retención (1kΩ) | Kit 1/4W (1kΩ ×20) o Kit 1/8W (1kΩ ×10) | ✅ |
| Serie WS2812B datos (330Ω) | Kit 1/4W (330Ω ×20) — **mejor que 1/8W aquí** | ✅ |
| Snubber RC relés (100Ω) | Kit 1/4W (100Ω ×20) — **1/4W mejor que 1/8W** | ✅ |

---

## Apéndice — Historial de sesiones

| Fecha | Qué se añadió al inventario |
|-------|-----------------------------|
| 2026-04-28 | C1 (100nF/50V ×100), C2 (1µF/50V ×100), C3 (2200µF/35V ×~5) |
| 2026-04-28 | R1 (3W 120Ω ×20), R2 (1/8W kit AUKENIEN 400pcs/40valores) |
| 2026-04-28 | R3 (1/4W 1% kit Metal Film 600pcs/30valores, 10Ω→1MΩ) — corregido CN1 que era este kit |
| 2026-04-28 | Análisis kit ALLECIN 240PCS electrolíticos: cubre 6/7 posiciones, no válido para bulk 24V |
| Anterior | D1 P6KE18CA ×20, D2 P6KE24CA ×20, D3 1N5408 ×50 |
| Anterior | R1 kit 3W metal film (bolsa 120Ω confirmada) |

---

*Este documento se actualiza en cada sesión cuando se añaden o consumen materiales del inventario.*
