# REFERENCIA COMPLETA DE CABLEADO PC817
## Sistema de Control Vehicular — STM32G474RE + ESP32-S3
### Fuente de verdad: `project_config.h` y `power_manager.h`

> **Este documento es la referencia consolidada para conectar el módulo PC817
> de 8 canales. Todos los valores han sido verificados en el código fuente.**

---

## RESUMEN DE CORRECCIONES (rev. 2026-04-28)

| # | Corrección | Impacto |
|---|-----------|---------|
| C1 | Topología de entrada LJ12A3 corregida — el cable NEGRO (señal NPN) va a IN-, no a GND | **Crítico** — error de cableado que impediría funcionar |
| C2 | Descripción de aislamiento GND corregida — STM32 y ESP32 **comparten GND lógico** | Importante |
| C3 | Diodo 1N4148 antiparalelo añadido como protección automotriz | Recomendado |
| C4 | Canal A6 (señal Z) aclarado: solo referencia de centro de dirección, no encoder de alta velocidad | Claridad |
| C5 | Cálculos de resistencias confirmados con justificación explícita | Confirmación |

---

## RESUMEN RÁPIDO

| Canal | Señal | Destino | Pull-up salida |
|-------|-------|---------|----------------|
| A1 | Sensor rueda FL | STM32 **PA0** (EXTI0) | **4.7 kΩ** a 3.3V |
| A2 | Sensor rueda FR | STM32 **PA1** (EXTI1) | **4.7 kΩ** a 3.3V |
| A3 | Sensor rueda RL | STM32 **PA2** (EXTI2) | **4.7 kΩ** a 3.3V |
| A4 | Sensor rueda RR | STM32 **PB15** (EXTI15) | **4.7 kΩ** a 3.3V |
| A5 | Sensor centro dirección | STM32 **PB5** (EXTI5) | **4.7 kΩ** a 3.3V |
| A6 | Señal Z dirección *(solo referencia de centro)* | STM32 **PB4** (EXTI4) | **4.7 kΩ** a 3.3V |
| A7 | **Llave de contacto** | ESP32 **GPIO 40** | **10 kΩ** a 3.3V |

**Resistencia de entrada (lado 12V) TODOS los canales: 1 kΩ ¼W**

---

## COMPONENTES A AÑADIR AL MÓDULO PC817

> ⚠️ El módulo PC817 de 8 canales **NO incluye pull-up de salida**.
> Verificado por medición (comentario en `power_manager.h`).
> Deben soldarse externamente.

### Por cada canal (×7 canales usados):

| Componente | Valor | Dónde va |
|-----------|-------|----------|
| **Resistencia entrada** | **1 kΩ ¼W** | En serie con **IN+** (ánodo LED), lado 12V |
| **Pull-up salida** (A1–A5, A6) | **4.7 kΩ 1/8W** | Entre **colector (OUT)** y **+3.3V**, lado lógico |
| **Pull-up salida** (A7 — llave) | **10 kΩ 1/8W** | Entre **colector (OUT)** y **+3.3V**, lado ESP32 |

### Por qué 1 kΩ en la entrada (no 330 Ω):
```
I_LED = (12V − 1.2V) / 1kΩ = 10.8 mA  ← dentro del límite continuo (20 mA) ✅
I_LED = (12V − 1.2V) / 330Ω = 32.7 mA ← EXCEDE el límite continuo ❌

Potencia en resistencia: P = 10.8 mA² × 1kΩ = 116 mW → ¼W (250 mW) da margen ×2 ✅
```

### Por qué 4.7 kΩ (no 10 kΩ) en los sensores de rueda:
El pull-up interno del STM32 (~40 kΩ) es demasiado lento para los flancos del
sensor inductivo con cable de hasta 5 m. El 4.7 kΩ externo (en paralelo con el
interno = 4.2 kΩ efectivo) garantiza t_flanco ≤ 5 µs con hasta 1 nF de capacidad
de cable. Ver cálculo completo en `docs/VALIDACION_CAN_PULLUP_PC817.md §2`.

```
τ = 4.2 kΩ × 1 nF = 4.2 µs   → t_to_VIH ≈ 5.1 µs con cable ≤ 5 m ✅
τ = 10 kΩ  × 1 nF = 10 µs    → t_to_VIH ≈ 12 µs → flancos lentos, ruido ❌
```

### Por qué 10 kΩ (no 4.7 kΩ) para la llave de contacto (ESP32 A7):
La llave de contacto es una señal de baja frecuencia (~1 Hz). No se requiere
flanco rápido. La 10 kΩ reduce la corriente de reposo cuando el transistor está
en corte, ahorrando ~0.3 mA por canal respecto a 4.7 kΩ. Compatible con la
resistencia `INPUT_PULLUP` interna del ESP32 (~45 kΩ).

---

## PROTECCIÓN AUTOMOTRIZ — DIODO 1N4148 *(RECOMENDADO)*

> ⚠️ **RECOMENDADO para entornos automotrices.** Los transitorios negativos en
> el bus de 12V (arranque de motor, cargas inductivas) pueden superar −1V en la
> línea de señal del sensor, lo que dañaría el LED del PC817 por corriente inversa.

Añadir **1 diodo 1N4148 en antiparalelo** con el LED del PC817 por canal:

```
       Ánodo LED (IN+)
           │
           ▼  LED
           │  PC817
           │
       Cátodo LED (IN-)
           │

Diodo 1N4148 antiparalelo:
  Cátodo 1N4148 → IN+ (mismo nodo que ánodo LED)
  Ánodo  1N4148 → IN- (mismo nodo que cátodo LED)

  Corriente directa PC817: normal (LED conduce)
  Transición negativa:     1N4148 conduce → limita V_inversa a ~0.7V → LED protegido ✅
```

| Componente | Valor | Montaje | Propósito |
|-----------|-------|---------|-----------|
| 1N4148 × 7 | Diodo de señal 100V/200mA | Antiparalelo con el LED PC817 | Protección contra picos negativos automotrices |

*Si no se añade: el PC817 admite V_R máx. = 6V (datasheet). En entorno automotriz limpio
esto puede ser suficiente, pero se recomienda el 1N4148 como medida de robustez de bajo costo.*

---

## ESQUEMA DE CONEXIÓN — Sensores LJ12A3 (canales A1–A5)

> ⚠️ **CORRECCIÓN CRÍTICA respecto a versión anterior:**
> El cable NEGRO del LJ12A3 es la **salida NPN**, no el GND.
> El cable AZUL es GND. El error anterior conectaba los colores al revés,
> lo que impediría que el LED del PC817 condujera nunca.

El sensor LJ12A3-4-Z/BX es NPN de colector abierto:
- **MARRÓN** = +12V (alimentación del sensor)
- **AZUL** = GND (masa del sensor)
- **NEGRO** = Salida NPN (colector abierto, se cierra a GND cuando detecta metal)

Topología correcta para excitar el LED del PC817:

```
════════════ LADO 12V (entrada) ════════════

  +12V ──────────────────────────────────────────────► MARRÓN (VCC sensor LJ12A3)
  +12V ──[1kΩ ¼W]──────────────────────────────────► PC817 IN+ (ánodo LED)
                                                         │
                                                         ▼  ←── LED conduce cuando
                                                       PC817      NEGRO está a GND
                                                       IN- (cátodo)
                                                         │
                                                         └────────► NEGRO (salida NPN sensor LJ12A3)
                                                                         │
                                                                         ▼ (transistor NPN interno)
  GND ───────────────────────────────────────────────────────────► AZUL (GND sensor LJ12A3)

  [1N4148 antiparalelo: cátodo → IN+, ánodo → IN-]   ← RECOMENDADO

════════════ LADO 3.3V (salida) ════════════

  +3.3V ──[4.7kΩ 1/8W]──┬──► PC817 OUT (colector)
                          │
                     ──►  STM32 GPIO pin destino (ver tabla de canales)

  PC817 GND_emisor  ──► GND_logic (masa STM32 / ESP32, lado lógico)
```

**Secuencia de funcionamiento:**
1. Metal detectado → transistor NPN del LJ12A3 se satura → NEGRO = GND
2. Corriente: +12V → 1kΩ → IN+ → LED → IN- → NEGRO → GND = **10.8 mA** → LED encendido
3. PC817 salida: transistor de salida satura → colector = LOW (~0.1V)
4. STM32 GPIO lee **LOW** → firmware interpreta como **pulso activo** ✅

**Sin metal:**
1. Transistor NPN del LJ12A3 en corte → NEGRO = flotante (alta impedancia)
2. Sin corriente por el LED → LED apagado
3. PC817 salida: transistor de salida en corte → colector = HIGH (pull-up 4.7kΩ)
4. STM32 GPIO lee **HIGH** → firmware interpreta como **sin pulso** ✅

## ESQUEMA CANAL A6 — Señal Z del encoder (solo referencia de centro de dirección)

> **⚠️ IMPORTANTE — diferencia con los canales A y B del encoder:**
>
> La señal Z del encoder E6B2-CWZ6C se usa **únicamente como referencia de posición
> cero del volante** (evento de baja frecuencia, ocurre ~1 vez por vuelta completa
> de la cremallera de dirección). **No es una señal de alta velocidad.**
>
> - **Canales A y B** (encoder cuadratura) → `6N137` (alta velocidad, TIM2)
> - **Canal Z** → **PC817 es aceptable** (baja frecuencia, solo detecta el centro)
>
> No es necesario sustituir el PC817 por un 6N137 en este canal.

```
════════════ LADO 12V — Señal Z del encoder ════════════

  +12V ──[1kΩ ¼W]──────────────────────────────────► PC817 A6 IN+ (ánodo)
                                                         │
                                                       PC817 A6
                                                       IN- (cátodo)
                                                         │
                                                         └────────► Salida Z encoder (NPN, igual que LJ12A3)
                                                                         │
  GND ──────────────────────────────────────────────────────────────────┘

  [1N4148 antiparalelo: cátodo → IN+, ánodo → IN-]   ← RECOMENDADO

════════════ LADO 3.3V (salida) ════════════

  +3.3V ──[4.7kΩ 1/8W]──┬──► PC817 A6 OUT (colector)
                          │
                     ──►  STM32 PB4 (EXTI4)

  PC817 GND_emisor  ──► GND_logic
```

## ESQUEMA CANAL A7 — Llave de contacto (ESP32)

```
════════════ LADO 12V (entrada) ════════════

  Terminal ON del contacto → [1kΩ ¼W] → PC817 A7 IN+
  GND del vehículo (GND_vehicle) →       PC817 A7 IN-

  [1N4148 antiparalelo: cátodo → IN+, ánodo → IN-]   ← RECOMENDADO

════════════ LADO 3.3V (salida) ════════════

  +3.3V ──[10kΩ 1/8W]──┬──► PC817 A7 OUT (colector)
                         │
                    ──►  ESP32 GPIO 40

  PC817 GND_emisor  ──► GND_logic (masa ESP32 / STM32 — compartida)
```

---

## LÓGICA INVERTIDA (PC817 invierte la señal)

### Sensores de rueda / centro dirección (A1–A6 → STM32):
| Estado sensor | LED PC817 | GPIO STM32 | Interpretación firmware |
|---------------|-----------|------------|------------------------|
| Detecta metal (activo) | Encendido | **LOW** | Pulso detectado ✅ |
| No detecta | Apagado | **HIGH** (pull-up) | Sin pulso |

El firmware configura estos pines como `GPIO_PULLUP` + `EXTI` en flanco de bajada.

### Canal A6 — Señal Z del encoder (STM32 PB4):
| Estado señal Z | LED PC817 | GPIO PB4 STM32 | Uso en firmware |
|---------------|-----------|----------------|-----------------|
| Pulso Z detectado (centro mecánico) | Encendido | **LOW** | Referencia de posición cero del volante |
| Sin pulso | Apagado | **HIGH** (pull-up) | Normal (fuera del punto cero) |

> Esta señal ocurre a baja frecuencia (~1 vez por vuelta completa del volante).
> El PC817 es completamente adecuado para esta velocidad.

### Llave de contacto (A7 → ESP32 GPIO 40):
| Posición llave | LED PC817 | GPIO 40 ESP32 | Interpretación firmware |
|----------------|-----------|---------------|------------------------|
| **ON** (+12V) | Encendido | **LOW** | Llave encendida ✅ |
| **OFF** (0V) | Apagado | **HIGH** (pull-up) | Llave apagada |

```cpp
// power_manager.cpp — lectura de la llave
bool raw = (digitalRead(PIN_IGNITION_SENSE) == LOW);  // LOW = llave ON
```
El ESP32 usa `INPUT_PULLUP` como respaldo de seguridad, pero el 10 kΩ externo
es **obligatorio** para inmunidad al ruido automotriz.

---

## PEDAL ACELERADOR — NO usa PC817

El pedal (SS1324LUA-T, Hall-effect 5V) va directo al STM32 con divisor de tensión:

```
Pedal OUT (0.3V–4.8V a 5V) ──[10kΩ]──┬──► PA3 STM32 (ADC1_IN4)
                                        │
                                     [6.8kΩ]
                                        │
                                       GND

Voltaje en PA3: 0–3.3V (compatible con ADC interno de 3.3V)
```

---

## ENCODER E6B2-CWZ6C — NO usa PC817, usa 6N137

Los canales A y B del encoder usan aisladores **6N137** (no PC817), con:
- Pull-up externo **4.7 kΩ** a 3.3V en la salida (pin 6 del 6N137)
- El firmware usa `GPIO_NOPULL` para ENC_A (PA15/TIM2_CH1) y ENC_B (PB3/TIM2_CH2)
- Si se omite este pull-up, **la entrada del TIM2 flota y se pierde el control de dirección**

Ver `docs/ENCODER_WIRING_6N137.md` para el circuito completo del encoder.

---

## LISTA DE COMPRA DE RESISTENCIAS Y DIODOS (solo PC817)

| Cant. | Valor | Tipo | Para |
|-------|-------|------|------|
| 7 | **1 kΩ ¼W** | Carbón o film | Entrada lado 12V de cada canal (A1–A7) |
| 6 | **4.7 kΩ 1/8W** | Film metal | Pull-up salida A1–A6 (sensores → STM32) |
| 1 | **10 kΩ 1/8W** | Film metal | Pull-up salida A7 (llave → ESP32 GPIO 40) |
| 7 | **1N4148** | Diodo señal 100V/200mA | Antiparalelo LED PC817, protección automotriz *(recomendado)* |

*(Para el divisor del pedal: 1× 10 kΩ + 1× 6.8 kΩ — pedido por separado)*

---

## NOTAS DE MONTAJE Y AISLAMIENTO DE GND

### Regla de GND — CORREGIDA

> ⚠️ La versión anterior decía "no conectar GND de ambos lados al mismo punto".
> Esto era **incompleto y engañoso**. La regla correcta es:

| GND | Qué incluye | Qué NO incluye |
|-----|-------------|----------------|
| **GND_vehicle** (lado 12V del PC817) | Masa del vehículo, sensores LJ12A3 cable AZUL, llave de contacto | Masa lógica del STM32 / ESP32 |
| **GND_logic** (lado 3.3V del PC817) | STM32 GND, ESP32 GND, emisor PC817 salida | Masa del vehículo / motor |

**El aislamiento es SOLO entre el dominio de 12V y el dominio lógico de 3.3V.**

**STM32 y ESP32 DEBEN compartir GND_logic.** El bus CAN ya lo requiere por construcción
(los transceivers TJA1051T/3 de ambos nodos comparten el mismo GND_CAN).

```
GND_vehicle (12V) ─────────────────────────┐
                                            │ ← BARRERA GALVÁNICA (PC817)
GND_logic   (3.3V) ──────────── STM32 GND ─┤
                   └──────────── ESP32 GND ─┘
                   (mismo nodo, obligatorio para CAN)
```

### Otras notas de montaje

1. Las resistencias de pull-up (4.7 kΩ / 10 kΩ) se pueden soldar directamente
   sobre los pines OUT del módulo o en una placa de prototipos aparte.
2. La resistencia de entrada 1 kΩ puede ir en el módulo o en el extremo del cable
   junto al sensor (reduce ruido en el cable).
3. El módulo necesita **alimentación 3.3V en el lado lógico** (no 5V).
   Usar el LDO externo AMS1117-3.3, no el pin 3.3V de la Nucleo-64.
4. Los diodos 1N4148 antiparalelo se sueldan junto a cada resistencia de 1 kΩ
   en el lado 12V del módulo.

## BUS CAN (PA11/PA12) — NO usa PC817 ni 6N137

El bus CAN del STM32 (FDCAN1) **no se aisla con optoacopladores PC817 ni 6N137**.
Los pines PA11 (RX) y PA12 (TX) van al transceiver **TJA1051T/3** directamente,
o a través del módulo **ADuM1201** si se requiere barrera galvánica completa:

```
STM32 PA12 ──► ADuM1201 AI │══════│ AO ──► TJA1051T/3 TXD ──► Bus CAN
STM32 PA11 ◄── ADuM1201 BO │══════│ BI ◄── TJA1051T/3 RXD ◄── Bus CAN
```

El módulo ADuM1201 necesita:
- **V1/G1** → 3.3V_STM32 / GND_logic (lado STM32)
- **V2/G2** → 3.3V_aislada / GND_CAN (del DC-DC aislado, lado CAN)
- DC-DC aislado de **5V** adicional para alimentar el TJA1051T/3 (VCC = 4.5–5.5 V)

**Ningún cambio de firmware.** Ver `docs/CONEXIONES_COMPLETAS.md §9` para el esquema
completo y `docs/CABLEADO_AISLAMIENTO_DEFINITIVO.md §9` para la especificación técnica.

---

*Documento creado a partir de `power_manager.h`, `project_config.h` y*
*`docs/VALIDACION_CAN_PULLUP_PC817.md`. Revisado y corregido: 2026-04-28.*
