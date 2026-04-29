# INVENTARIO DE COMPONENTES FÍSICOS — Proyecto Marcos

> **Documento de inventario real del material disponible en mano.**  
> Fecha creación: 2026-04-28  
> Versión: 1.6 (1N4148 ×150 RECIBIDOS → EN MANO; kit ALLECIN 240PCS electrolíticos RECIBIDO → EN MANO)  
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
10. [Transistores](#10-transistores)
11. [Módulos optoacopladores (PC817 / 6N137)](#11-módulos-optoacopladores-pc817--6n137)

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
| D4 | Diodo Zener DO-35 | **Kit 1N4728–1N4737** (3.3 V–13 V, 10 valores) | ~200 uds | ✅ EN MANO |
| D5 | Diodo rectificador DO-41 | **1N4007** (1 A / 1000 V) | 10 uds | ✅ EN MANO |
| D6 | Diodo conmutación rápida DO-35 | **1N4148** (200 mA / 100 V, trr 4 ns) | 150 uds | ✅ EN MANO |
| C4 | Condensador film CBB22 | **100 nF / 250 V** (código 104J, paso P10) | 20 uds | ✅ EN MANO |
| T1 | Transistor **NPN** TO-92 | **2N2222** (40 V / 600 mA) | 10 uds | ✅ EN MANO |
| T2 | Transistor **NPN** TO-92 | **BC337-40** (45 V / 800 mA) | 3 uds | ✅ EN MANO |
| T3 | Transistor **NPN** TO-92 | **C1815** (50 V / 150 mA) | 3 uds | ✅ EN MANO |
| T4 | Transistor **PNP** TO-92 | **A1015** (50 V / 150 mA) | 4 uds | ✅ EN MANO |
| M1 | **Módulo PC817 8 canales** | Optoacoplador 8-ch en placa | 2 uds (= 16 ch) | ✅ EN MANO |
| M2 | **Módulo 6N137 (5 V lado MCU)** | Optoacoplador alta velocidad en placa | 5 uds | ✅ EN MANO |
| M3 | **Módulo 6N137 (12 V lado sensor)** | Optoacoplador alta velocidad en placa | 5 uds | ✅ EN MANO |
| C5 | Kit electrolíticos ALLECIN 240PCS | **24 valores** 0.1µF–1000µF, 10V/16V/25V/50V | 240 uds | ✅ EN MANO |
| F1 | Ferrita / bobina CAN | BLM18AG601SN1D / 100 µH | 0 uds | ❌ **FALTA — pedir** |

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

### 2.3 — CBB22 100 nF / 250 V (código **104J**, paso P10) — 20 piezas

| Campo | Dato |
|-------|------|
| Referencia bolsa | `EU-HBC-Cap-CBB22-250V104J-P10-20PCS` (B11) |
| Código | **104J** → 100 nF, tolerancia ±5% (J) |
| Tensión nominal | **250 V AC** (≈350 V DC pico) |
| Tipo | Film polipropileno metalizado (CBB22) — no polarizado |
| Paso pines | **P10 = 10 mm** |
| Cantidad | **20 piezas** |

**¿Qué diferencia hay con los cerámicos 104 de C1?**
- C1 (cerámica 50V) → bypass rápido de señal, junto a ICs en rail de 3.3V/5V
- CBB22 (film 250V) → aguanta alta tensión AC; va en circuitos conectados a red 230V o en snubber de relés con bobinas de 24V donde hay picos de tensión altos

**Uso en el proyecto:**
- **Snubber RC de relés** → los contactos de RELAY_MAIN/TRAC/DIR generan arcos → 100nF en serie con 100Ω paralelo a los contactos. La tensión de trabajo es 24V, pero los picos de conmutación pueden llegar a 100-200V; la CBB22 a 250V aguanta esto mejor que la cerámica 50V
- **Recomendado:** usar las CBB22 preferentemente en las 3 posiciones de snubber de relé, y guardar las cerámicas 50V para los bypass de ICs
- Sobran 17 de reserva

---

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

### 5.4 — Kit Zener 1N4728–1N4737 — 10 valores / ~200 piezas

| Campo | Dato |
|-------|------|
| Referencia | "10 Value Zener Diode Assorted Kit" |
| Valores incluidos | 1N4728 (3.3V), 1N4729 (3.6V), 1N4730 (3.9V), 1N4731 (4.3V), 1N4732 (4.7V), 1N4733 (5.1V), 1N4734 (5.6V), 1N4735 (6.2V), 1N4736 (6.8V), 1N4737 (7.5V) |
| Tensión Zener | 3.3 V → 7.5 V (modo inverso) |
| Corriente Zener max | 500 mW / Vz (típico 75–150 mA) |
| Encapsulado | DO-35 (similar a 1N4148) |
| Cantidad estimada | ~20 piezas × 10 valores = **~200 piezas** |

#### ¿Para qué sirven en este proyecto?

| Uso | En modo directo (Vf ≈ 0.65 V) | Resultado |
|-----|-------------------------------|-----------|
| **Flyback bobina relé** (en antiparalelo) | ✅ actúa como diodo normal | **Funciona perfectamente** — sobran 1N5408 pero estos también valen |
| **OR lógico retención** (sustituto 1N4148) | ✅ para señales DC lentas | **Sí sirve** — más lento que 1N4148 pero irrelevante en control de relé |
| Protección ESD entrada PC817 | ❌ no es el componente adecuado | Usar P6KE18CA ya disponible |

#### ¿Sirven para el PC817?

**No se necesita ningún diodo en paralelo con el PC817.** El PC817 tiene un LED en su entrada (no una bobina inductiva), por lo que no genera picos de back-EMF. Lo único que necesita la entrada del PC817 es una **resistencia en serie** para limitar la corriente al LED (~5–20 mA). Los diodos Zener son innecesarios en esa posición.

**Conclusión:** Los Zener **no se soldan junto al PC817** como protección ESD. Sí pueden usarse en **polarización directa** para el OR de activación del relé de retención. **Decisión 2026-04-28:** se pide caja de **1N4148 ×150** porque es el componente "correcto" para protección antiparalelo del LED de PC817 y 6N137 (más rápido que el Zener, más versátil, sin riesgo de elegir Vz incorrecta). Los Zener quedan como reserva polivalente.

---

### 5.5 — 1N4007 — Diodo rectificador 1A/1000V

| Campo | Dato |
|-------|------|
| Referencia bolsa | `120000505899990295` |
| Fabricante | Shenzhen HuiJiong Electronic Technology Co., Ltd. |
| Item | "10/50/100PCS 1N4007 4007 1A 1000V DO-41 High quality Rectifier Diode IN4007 1n4007" |
| Corriente continua | 1 A |
| Tensión inversa | 1000 V |
| Encapsulado | DO-41 (axial pequeño, cuerpo negro con banda blanca) |
| Cantidad | **10 piezas** |

**Uso en el proyecto:**
- **Redundante con 1N5408** (que ya tienes ×50, 3A/1000V, mismas características eléctricas pero más robusto). El 1N5408 sustituye perfectamente al 1N4007 en TODAS las posiciones.
- Quedan como **reserva** para flyback de relés/bobinas de bajo consumo (≤500 mA) si se acaban los 1N5408.
- También válidos como diodo OR de retención (mejor que el Zener, peor que el 1N4148 — Vf ≈ 0.9 V, trr ≈ 30 µs).

**No los uses para:** protección antiparalelo del LED de PC817 / 6N137 → muy lentos (trr 30 µs) frente al 1N4148 (trr 4 ns). Para ese uso esperar al 1N4148 pedido.

---

### 5.6 — 1N4148 — Diodo conmutación rápida 200mA/100V ✅ EN MANO

| Campo | Dato |
|-------|------|
| Referencia | 1N4148 (caja de 150 piezas) |
| Marca | **ALLECIN** |
| Estado | ✅ **EN MANO** (recibidos 2026-04-29) |
| Corriente continua | 200 mA |
| Tensión inversa | 100 V |
| Vf típica | 0.7 V @ 10 mA |
| Tiempo recuperación inversa (trr) | **4 ns** (rapidísimo) |
| Encapsulado | DO-35 (cristal pequeño, banda negra = cátodo) |
| Cantidad | **150 piezas** |

**¿Para qué se pide si los Zener servirían?**
- Es el diodo más universal y versátil del taller. A precio de 150 unidades (<3 €) la inversión por pieza es ridícula.
- **Polaridad uniforme:** todos los 1N4148 son intercambiables; no hay que pensar en "qué Vz elegir" como con el kit Zener (donde solo Vz≥6.2V vale).
- Libera al kit Zener para su función real (regulación / referencias).
- **Velocidad 4 ns** vs 1 µs típico de un Zener: relevante para 6N137 (encoder a alta frecuencia) — un Zener lento podría dejar pasar parte del transitorio inverso antes de empezar a conducir.

**Usos previstos en este proyecto:**

| Aplicación | Cantidad estimada |
|-----------|-------------------|
| Protección antiparalelo LED PC817 (5 canales LJ12A3) | 5 |
| Protección antiparalelo LED 6N137 (3 canales encoder A/B/Z) | 3 |
| Diodos OR retención relé MAIN/TRAC/DIR | 6–9 |
| Flyback de relés pequeños de señal (si se añaden, bobina <100 mA) | reserva |
| Clamps protección entradas digitales STM32 (3.3V/GND) | reserva |
| Repuesto general / prototipado | resto |

**Total uso inmediato estimado: 14–17 unidades. Sobran ~135 de reserva.**

#### Cómo se solda — IMPORTANTE

⚠️ **NO se solda entre VCC y GND del lado 3.3V.** Eso lo cortocircuitaría y quemaría el diodo o la fuente. Se solda en **antiparalelo al LED del optoacoplador**, en el **lado de ENTRADA** (lado del sensor a 12-24 V).

**Polaridad (regla mnemotécnica):**
- **Banda negra = cátodo (K)** → al **POSITIVO de la entrada (IN+)**
- Lado **sin banda = ánodo (A)** → al **NEGATIVO/GND de la entrada (IN−)**

Esto es **al revés** que el LED interno del optoacoplador → por eso se llama "antiparalelo".

**Verificación con multímetro antes de soldar (modo diodo):**
- Punta roja al ánodo (lado liso) + punta negra al cátodo (banda) → debe leer **~0.6–0.7 V**
- Al revés → debe leer **OL** (abierto)

**Precauciones de soldadura:**
- Soldador a **320–340 °C**, máximo 3 s por pin
- Dejar **3–4 mm de patilla** entre el cuerpo del diodo y el punto de soldadura (la patilla disipa calor y protege el cristal interno)
- No recortar la patilla demasiado corta antes de soldar — riesgo de microfracturas

#### Cuándo NO usar 1N4148

| Aplicación | Componente correcto |
|-----------|--------------------|
| Corrientes >200 mA continuas | 1N5408 (ya en mano ×50) |
| Picos de energía grandes / TVS | P6KE18CA / P6KE24CA (ya en mano ×40) |
| Velocidades >100 MHz / RF | Schottky BAT (no necesario en este proyecto) |

---

> **Estado:** ✅ **EN MANO** — Kit recibido 2026-04-29 (ALLECIN 240PCS)  
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

### 9.1 Condensadores electrolíticos — ESTADO CON KIT ALLECIN ✅ EN MANO

> El kit ALLECIN 240PCS ha llegado (2026-04-29). Las posiciones de abajo quedan cubiertas:

| Posición | Valor | Para qué | Qty | Estado |
|----------|-------|----------|-----|--------|
| Bulk 5V LEDs WS2812B frontal | 1000 µF / 16V | Rail 5V → RELAY_LED (PB10) | 1 | ✅ EN MANO (kit) |
| Bulk 5V LEDs WS2812B trasero | 1000 µF / 16V | Rail 5V → RELAY_LED_REAR (PB11) | 1 | ✅ EN MANO (kit) |
| Bulk salida LM2596 | 47 µF / 10V | Salida regulador 5V | 1 | ✅ EN MANO (kit) |
| Bulk 3.3V STM32 | 10 µF / 25V | Rail 3.3V Nucleo | 1 | ✅ EN MANO (kit) |
| Bulk CAN ESP32 | 10 µF / 25V | SN65HVD230 lado ESP32 | 1 | ✅ EN MANO (kit) |
| Bulk bus 12V dirección | 470 µF / 16V | BTS7960 STEER B+(12V) | 1 | ✅ EN MANO (kit) |
| **Bulk bus 24V relés** | **≥1000 µF / 35V** | Rail 24V junto a RELAY_MAIN/TRAC | 1 | ⚠️ **Usar 2200µF/35V en mano** |

> **Para el bulk 24V:** Usar uno de los **2200µF/35V ya en mano** (si sobra uno de los 5 para BTS7960). Si no, comprar 1 pieza suelta de 1000µF/35V o 2200µF/35V.

### 9.2 Componentes — FALTAN (no en inventario actual)

| Componente | Especificación | Para qué | Qty |
|-----------|---------------|----------|-----|
| Ferrita/bobina | BLM18AG601SN1D o 100µH | Filtro bus CAN (5V sucio → limpio) | 1 |

> **Optoacopladores PC817 y 6N137 ya NO hacen falta comprar:**
> - PC817 → **2 módulos de 8 canales en mano = 16 canales** (solo se usan 5) ✅
> - 6N137 encoder → **10 módulos en mano** (5×5V + 5×12V) ✅ sobra ampliamente
>
> **Transistor NPN ya en mano:**
> - Transistor para relé retención → **2N2222 ×10 en mano** ✅
>
> **1N4148 ×150 — EN MANO (recibidos 2026-04-29):** sustituye a la solución provisional de Zener en antiparalelo. Más rápido, más versátil, polaridad uniforme. Ya se puede soldar en todas las posiciones documentadas.
>
> **Pendientes de pedir:** ferrita CAN (1 unidad).

### 9.3 Estado COMPLETAMENTE CUBIERTO por inventario actual

| Función | Componente disponible | ✅ |
|---------|----------------------|---|
| Terminación CAN (2×120Ω) | Kit 3W 120Ω | ✅ |
| Flyback todos los relés (×6) | 1N5408 ×50 (o Zener en directa) | ✅ |
| TVS protección PC817 (×5 canales) | P6KE18CA ×20 | ✅ |
| TVS protección 24V rail | P6KE24CA ×20 | ✅ |
| Bypass 100nF todos módulos | Cerámica 104 ×100 | ✅ |
| Snubber RC relés (100nF alta tensión) | CBB22 250V/100nF ×20 — **preferir sobre cerámica 50V** | ✅ |
| Bypass 1µF ICs sensores | Cerámica 105 ×100 | ✅ |
| Bulk 2200µF BTS7960 ×5 | Electrolítico 2200µF/35V ×5 | ✅ |
| Pull-up 4.7kΩ I2C + encoder | Kit 1/4W (4.7kΩ ×20) o Kit 1/8W (4.7kΩ ×10) | ✅ |
| Divisor pedal + llave | Kit 1/4W (10kΩ, 33kΩ) o Kit 1/8W | ✅ |
| Base transistor retención (1kΩ) | Kit 1/4W (1kΩ ×20) o Kit 1/8W (1kΩ ×10) | ✅ |
| Serie WS2812B datos (330Ω) | Kit 1/4W (330Ω ×20) — **mejor que 1/8W aquí** | ✅ |
| Snubber RC relés (100Ω) | Kit 1/4W (100Ω ×20) — **1/4W mejor que 1/8W** | ✅ |
| **Transistor relé retención** | **2N2222 ×10** en mano | ✅ |
| **OR diodos retención** | **1N4148 ×150 EN MANO** o Zener 1N4728–1N4737 directo | ✅ |
| **Aislamiento sensores LJ12A3 (×5 ch)** | **Módulos PC817 8-ch ×2** (16 canales) | ✅ |
| **Aislamiento encoder A/B/Z (×3 ch)** | **Módulos 6N137 5V ×5** | ✅ |
| **Protección polaridad inversa LED PC817** | **1N4148 ×150 EN MANO ×5** | ✅ |
| **Protección polaridad inversa LED 6N137** | **1N4148 ×150 EN MANO ×3** antiparalelo en cada canal A/B/Z | ✅ |

---

## 10. Transistores

### 10.1 — 2N2222 — NPN 40V/600mA (×10)

| Campo | Dato |
|-------|------|
| Referencia | 2N2222 |
| Tipo | NPN BJT |
| Vceo (tensión colector-emisor) | 40 V |
| Ic max (corriente colector) | 600 mA |
| hFE típico | 100–300 |
| Encapsulado | TO-92 |
| Cantidad | **10 piezas** |

**Uso en el proyecto:**
- **Driver relé retención** (GPIO41 ESP32 → R_base=1kΩ → base, colector → bobina relé → 5V): 1 pieza
- Pueden usarse como driver para cualquier relé de 5V en el circuito
- Sobran 9 en reserva

---

### 10.2 — BC337-40 — NPN 45V/800mA (×3)

| Campo | Dato |
|-------|------|
| Referencia | BC337-40 (sufijo -40 indica hFE grupo 3: 250–630) |
| Tipo | NPN BJT |
| Vceo | 45 V |
| Ic max | 800 mA |
| hFE | 250–630 (grupo "40") |
| Encapsulado | TO-92 |
| Cantidad | **3 piezas** |

**Uso en el proyecto:** Alternativa al 2N2222 con mayor ganancia y corriente. Idóneo para driver de relés con bobinas de mayor consumo.

---

### 10.3 — C1815 — NPN 50V/150mA (×3)

| Campo | Dato |
|-------|------|
| Referencia | C1815 GR (hFE grupo GR: 200–400) |
| Tipo | NPN BJT |
| Vceo | 50 V |
| Ic max | 150 mA |
| hFE | 200–400 (grupo GR) |
| Encapsulado | TO-92 |
| Cantidad | **3 piezas** |

**Uso en el proyecto:** Señales de control de baja corriente, buffer de lógica, activación de optoacopladores. No recomendado para drivers de relé (150 mA < bobinas típicas 360 mA a 5V).

---

### 10.4 — A1015 — PNP 50V/150mA (×4)

| Campo | Dato |
|-------|------|
| Referencia | A1015 GR (complementario del C1815) |
| Tipo | **PNP** BJT |
| Vceo | 50 V |
| Ic max | 150 mA |
| hFE | 200–400 (grupo GR) |
| Encapsulado | TO-92 |
| Cantidad | **4 piezas** |
| Pinout | **EBC** (Emisor–Base–Colector, de izquierda a derecha con texto al frente) |

**⚠️ ATENCIÓN — Es PNP:** El transistor conduce cuando la base está a tensión INFERIOR al emisor (lógica invertida respecto a los NPN). Para conmutar desde un GPIO de 3.3V (nivel bajo activo), se usa en conmutación de alto lado (high-side switch).

**Uso en el proyecto:** Conmutación de alto lado en líneas de 5V o 12V donde se necesita activación a nivel bajo. No es el transistor principal para los relés (esos usan NPN a bajo lado).

---

### 10.5 — Tabla resumen de disponibilidad

| Transistor | Tipo | Qty | Driver relé 5V | Driver PC817 | Señal lógica | Observación |
|-----------|------|-----|---------------|-------------|-------------|-------------|
| **2N2222** | NPN | 10 | ✅ **Primario** | ✅ | ✅ | Usar este para relé retención |
| **BC337-40** | NPN | 3 | ✅ (mejor Ic) | ✅ | ✅ | Reserva o relés de mayor bobina |
| **C1815** | NPN | 3 | ⚠️ Ic=150mA (límite) | ✅ | ✅ | Para señales, no relés de potencia |
| **A1015** | PNP | 4 | ⚠️ high-side only | ⚠️ invertido | ✅ | Lógica PNP, diferente conexión |

---

## 11. Módulos optoacopladores (PC817 / 6N137)

### 11.1 — Módulo PC817 8 canales (×2 unidades)

| Campo | Dato |
|-------|------|
| Tipo | Placa con 8× PC817 ya montados |
| Canales totales | 8 ch/módulo × 2 módulos = **16 canales** |
| Aislamiento | 5 000 V (PC817 estándar) |
| Velocidad | Baja (BW ≈ 80 kHz) — ideal para LJ12A3 inductivos |
| Cantidad | **2 módulos** |

**Uso en el proyecto (5 canales necesarios):**
- 4× sensores LJ12A3 velocidad rueda → PA0, PA1, PA2, PB15
- 1× sensor LJ12A3 centro dirección → PB5
- Sobran 11 canales (más de un módulo entero de reserva)

**Recomendación de reparto:**
- Módulo #1: usar 5 canales para los sensores LJ12A3 (4 ruedas + centro dirección)
- Módulo #2: 100 % reserva (futuras señales digitales aisladas: pulsador externo, fin de carrera, etc.)

---

### 11.2 — ¿Sirve el Zener kit (1N4728–1N4737) para el PC817? **SÍ, igual que un 1N4148**

#### Por qué es necesario un diodo de protección en la entrada del PC817

El diodo LED interno del PC817 tiene:
- **Vf típica:** 1.2 V (en directa, conducción)
- **Vr máxima:** **6 V** ← si llega más voltaje en inverso, **se destruye**

Los sensores **LJ12A3** son inductivos PNP a 12-24 V. Si por error de cableado, ruido inductivo o pulso de retorno la señal se invierte (la línea cae por debajo de GND), el LED del PC817 recibe tensión inversa y se quema.

**Solución estándar:** un diodo en **antiparalelo** al LED del PC817 que se polariza al revés que el LED. Cuando entra una tensión positiva (normal) el LED conduce y el diodo de protección está en inverso (no afecta). Cuando entra tensión inversa, el diodo de protección conduce en directa (Vf ≈ 0.7 V) y "cortocircuita" el LED protegiéndolo.

#### Esquema de conexión — Zener en antiparalelo

```
                            R_serie 820Ω-2.2kΩ
   Señal sensor LJ12A3 ──────[ R ]──────┬─────► PIN 1 (Anodo LED PC817)
   (12-24 V PNP)                         │
                                         │
                              ╔══════════╪══════════╗
                              ║          │   K  A   ║
                              ║   LED    ↓   ←──┐   ║   ← Zener en antiparalelo
                              ║   ────►  │      │   ║      cátodo al ánodo del LED
                              ║   PC817  │      │   ║      ánodo  al cátodo del LED
                              ╚══════════╪══════╪═══╝
                                         │      │
   GND sensor ───────────────────────────┴──────┴─────► PIN 2 (Cátodo LED PC817)
```

**¿Cómo identificar la polaridad del Zener?**
- **Banda negra** = cátodo (K)
- Lado **sin banda** = ánodo (A)

**Conexión correcta:**
- **Cátodo del Zener (banda)** → conectar al **PIN 1 del PC817 (lado positivo de la entrada)** = mismo nodo que el ánodo del LED
- **Ánodo del Zener (sin banda)** → conectar al **PIN 2 del PC817 (lado GND de la entrada)** = mismo nodo que el cátodo del LED

> En condiciones normales (señal positiva), el Zener queda polarizado en INVERSO. Como las tensiones de trabajo (5-24 V según el divisor R_serie) son **menores que la Vz mínima del kit (3.3 V)**, **¡cuidado!**: el Zener 1N4728 (3.3 V) podría empezar a conducir en inverso si la tensión en el LED supera 3.3 V → **NO USAR el 1N4728**.
>
> **Usa cualquiera de los siguientes valores que tienes en mano (Vz ≥ 6.2 V):**
> - **1N4735 (6.2 V)** ✅ recomendado — Vz alta, no interfiere
> - **1N4736 (6.8 V)** ✅
> - **1N4737 (7.5 V)** ✅
>
> Estos NO conducirán en inverso porque la caída en el LED del PC817 es ~1.2 V (siempre muy por debajo de 6.2 V). Solo conducirán en **directa** cuando haya inversión de polaridad accidental, exactamente como un 1N4148.

#### ¿Y si el módulo PC817 8 canales ya trae diodo de protección integrado?

La mayoría de módulos comerciales chinos de PC817 8-ch **YA incluyen** el diodo antiparalelo en la placa (suele ser un 1N4148 SMD junto a cada canal). **Verifica visualmente la placa antes de añadir Zener externos:**

| Caso | Acción |
|------|--------|
| Módulo trae diodo SMD junto al PC817 | ✅ **NO necesitas añadir nada** — los Zener quedan de reserva |
| Módulo SIN diodo de protección | ✅ Añade 1N4735/4736/4737 en antiparalelo a la entrada de cada canal usado (5 unidades) |

#### ¿Dónde se sueldan en el módulo si hace falta?

En las regletas de **entrada (lado IN)** del módulo:
- Cada canal tiene 2 pines (IN+ e IN−)
- El Zener se suelda **directamente entre IN+ e IN−**, con la **banda (cátodo) hacia IN+**

```
   Borna IN+ canal X ●──────┬───── (al PC817 anodo LED)
                            │
                            ▼ banda (cátodo)
                          Zener 1N4735 (6.2 V)
                            │ (ánodo, sin banda)
                            │
   Borna IN− canal X ●──────┴───── (al PC817 cátodo LED, GND sensor)
```

**Conclusión final:**
- ✅ **SÍ te sirven los Zener que tienes**, usados en antiparalelo igual que un 1N4148
- ✅ Usa **1N4735 (6.2 V)** o superior — **no** uses los de Vz baja (1N4728 3.3 V)
- ⚠️ Antes de soldar, comprueba si tu módulo 8-ch ya lleva diodo de protección integrado; en ese caso no hace falta añadir nada

---

### 11.3 — Módulos 6N137 (×10 unidades: 5×5V + 5×12V)

| Campo | Dato |
|-------|------|
| Tipo | Placas con 6N137 ya montados |
| Velocidad | Alta (10 Mbit/s) — apropiado para encoder E6B2-CWZ6C |
| Aislamiento | 2 500 V típico |
| Variantes en mano | **5 módulos lado 5 V** + **5 módulos lado 12 V** |
| Cantidad total | **10 módulos** |

**Uso en el proyecto (3 canales necesarios):**
- 1× canal A encoder → PA15 (TIM2_CH1)
- 1× canal B encoder → PB3 (TIM2_CH2)
- 1× canal Z encoder → PB4 (EXTI4)

**¿5V o 12V?** El encoder Omron E6B2-CWZ6C es alimentado a **5 V** (open collector NPN), por lo tanto la entrada del 6N137 ve picos a 5 V.
- **Usar los 5 módulos de 5 V** → coinciden exactamente con la tensión de salida del encoder
- Los 5 módulos de 12 V quedan como **reserva** o para futuros sensores con salida 12 V

**Distribución recomendada:**
- 3 módulos de 5 V → encoder A/B/Z (lado MCU)
- 2 módulos de 5 V → reserva
- 5 módulos de 12 V → reserva total para futuros canales digitales aislados a 12 V

---

### 11.3.1 — ⚠️ **RECORDATORIO IMPORTANTE — Protección antiparalelo al montar 6N137**

> **Cuando montes los módulos 6N137 (encoder A/B/Z) acuérdate de soldar un 1N4148 en antiparalelo al LED de entrada en cada canal**, **igual que en el PC817**.

**Por qué:**
- El LED interno del 6N137 tiene **Vr_max ≈ 5 V**. Si la línea de salida del encoder (open-collector NPN, alimentado a 5 V) sufre un transitorio inverso (ruido EMI del motor, pulso de retorno, mal cableado), el LED se quema.
- Es exactamente el mismo razonamiento que con el PC817 — solo cambia el LED y el encapsulado del optoacoplador.
- Velocidad importa más aquí: el encoder E6B2-CWZ6C @ 4800 cpr puede generar pulsos a varios kHz. El 1N4148 (trr = 4 ns) protege sin afectar la señal; un Zener (trr ≈ 1 µs) ya empieza a notarse en el flanco.

**Esquema de conexión (uno por cada canal A/B/Z, 3 en total):**

```
   Salida encoder (5V open collector NPN) ───[R serie]──┬─── IN+ módulo 6N137
                                                         │
                                                         ▼ banda (cátodo)
                                                      1N4148
                                                         │ (ánodo, sin banda)
                                                         │
   GND encoder ─────────────────────────────────────────┴─── IN− módulo 6N137
```

**Polaridad (igual que en PC817):**
- **Banda negra (cátodo) → IN+** (lado positivo, mismo nodo que el ánodo del LED interno)
- Lado **sin banda (ánodo) → IN−** (GND, mismo nodo que el cátodo del LED interno)

**Antes de soldar:**
1. Verifica si el módulo 6N137 ya trae diodo de protección integrado (algunos los traen como SMD junto al optoacoplador). Si lo trae → **no hace falta añadir nada**.
2. Si no lo trae → soldar 1N4148 en cada uno de los 3 canales (A, B, Z) entre IN+ e IN−.
3. Multímetro modo diodo: punta roja al lado liso, negra a la banda → debe leer ~0.65 V. Al revés OL.

**Cantidad necesaria al montar 6N137:** **3 unidades** (una por canal A/B/Z) si los módulos no traen protección integrada.

> **Estado pedido 1N4148:** EN PEDIDO 2026-04-28 (caja de 150 uds). Hasta que lleguen, si quieres avanzar el montaje del encoder puedes usar **1N4735/4736/4737** (Vz ≥ 6.2 V) del kit Zener — funcionan igual aunque más lentos. Para encoder a frecuencia alta, mejor esperar al 1N4148.

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
| 2026-04-28 | **v1.3:** D4 kit Zener 1N4728–1N4737 ×~200; C4 CBB22 250V/100nF ×20; T1 2N2222×10, T2 BC337-40×3, T3 C1815×3, T4 A1015×4; eliminados de "falta comprar": transistor NPN y 1N4148 |
| 2026-04-28 | **v1.4:** M1 módulos PC817 8ch ×2 (16 canales totales); M2 módulos 6N137 5V ×5; M3 módulos 6N137 12V ×5; F1 ferrita CAN confirmada NO disponible (única pieza pendiente). Documentado cableado Zener antiparalelo en PC817 (usar Vz ≥ 6.2 V: 1N4735/4736/4737) |
| 2026-04-28 | **v1.5:** D5 1N4007 ×10 en mano (DO-41, 1A/1000V, redundante con 1N5408 — queda como reserva); D6 1N4148 ×150 EN PEDIDO (caja completa, decisión: protección antiparalelo PC817+6N137 con componente correcto y polivalente, sustituye solución provisional con Zener). Añadido §11.3.1 con recordatorio explícito de soldar 1N4148 antiparalelo al montar los módulos 6N137 (Vr_max LED ≈ 5V, mismo principio que PC817). |

---

*Este documento se actualiza en cada sesión cuando se añaden o consumen materiales del inventario.*
