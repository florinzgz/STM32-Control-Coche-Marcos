# Llave de Contacto — Encendido y Apagado del Sistema

**Documento de referencia para taller — Circuito completo de la llave de contacto**

> **Fuente:** Extraído directamente del firmware ESP32-S3 (`power_manager.h`,
> `power_manager.cpp`, `main.cpp`) y STM32G474RE (`safety_system.c`,
> `safety_system.h`, `main.h`, `main.c`), y de la documentación existente
> (`POWER_DISTRIBUTION.md`, `HARDWARE_WIRING_MANUAL.md`, `LISTADO_PINES_COMPLETO.md`).
> No se ha inventado ningún componente ni conexión.

---

## Índice

1. [Resumen General](#1-resumen-general)
2. [Conexión Física de la Llave de Contacto](#2-conexión-física-de-la-llave-de-contacto)
3. [Circuito de Alimentación Permanente y Retención](#3-circuito-de-alimentación-permanente-y-retención)
4. [Secuencia Completa de Encendido](#4-secuencia-completa-de-encendido)
5. [Flujo de Corriente hacia los Motores de Tracción](#5-flujo-de-corriente-hacia-los-motores-de-tracción)
6. [Secuencia Completa de Apagado](#6-secuencia-completa-de-apagado)
7. [Diagrama Eléctrico Completo](#7-diagrama-eléctrico-completo)
8. [Lista de Componentes del Circuito de Llave](#8-lista-de-componentes-del-circuito-de-llave)
9. [Preguntas Frecuentes](#9-preguntas-frecuentes)

---

## 1. Resumen General

La llave de contacto es el **interruptor maestro** que inicia y detiene todo el
sistema del vehículo. Su señal la lee el **ESP32-S3** (no el STM32).

```
 Llave de           ESP32-S3               CAN Bus              STM32G474RE
 Contacto         (HMI / Control)       (500 kbps)          (Autoridad de Seguridad)
 ─────────         ──────────────        ──────────           ───────────────────────
                                         
  Posición  ──►  GPIO 40 detecta   ──►  Heartbeat   ──►   Estado ACTIVE
  ON / OFF       estado de llave        0x011 cada          │
                      │                 100 ms              ▼
                      │                                  Relay_PowerUp()
                      ▼                                     │
                  GPIO 41 activa                            ▼
                  retención de                         RELAY_MAIN → RELAY_TRAC → RELAY_DIR
                  alimentación                              │
                                                            ▼
                                                      4× motores tracción (24 V)
                                                      1× motor dirección (12 V)
```

**Roles de cada microcontrolador:**

| Función | Responsable | Pin / Mecanismo |
|---------|-------------|-----------------|
| Detectar posición de la llave | **ESP32-S3** | GPIO 40 (entrada con pull-down) |
| Mantener alimentación durante apagado | **ESP32-S3** | GPIO 41 (salida, activo HIGH) |
| Reproducir audio de bienvenida / despedida | **ESP32-S3** | DFPlayer Mini (UART2) |
| Activar relés de potencia | **STM32G474RE** | PC10, PC11, PC12 (GPIO salida) |
| Controlar motores | **STM32G474RE** | TIM1/TIM8/TIM3 (PWM 20 kHz) |

---

## 2. Conexión Física de la Llave de Contacto

### 2.1 ¿Dónde se conecta la llave?

La llave de contacto se conecta **al ESP32-S3**, en el pin **GPIO 40**
(`PIN_IGNITION_SENSE`). El ESP32 la lee como entrada digital con pull-down
interno.

```
                           ┌─────────────────────────────────┐
  LLAVE DE CONTACTO        │        ESP32-S3 DevKitC-1       │
  (interruptor ON/OFF)     │                                 │
                           │                                 │
  Terminal (+) ─── R1 ──┬──┤ GPIO 40 (IGNITION_SENSE)       │
                        │  │   INPUT_PULLDOWN                │
                       R2  │                                 │
                        │  │ GPIO 41 (POWER_HOLD) ──────────►│── A relé de retención
                       GND │                                 │
                           │                                 │
  Terminal (−) ──── GND ───┤ GND                             │
                           └─────────────────────────────────┘
```

### 2.2 Adaptación de nivel de tensión

La llave de contacto puede estar conectada a la batería de 12 V o 24 V.
Como el ESP32-S3 opera a **3.3 V**, es necesario un **divisor de tensión
resistivo** para reducir la señal:

**Si la llave conmuta 12 V:**

> ⚠️ **EJEMPLO INCORRECTO — NO USAR estos valores:**

| Componente | Valor | Función |
|------------|-------|---------|
| R1 | 10 kΩ | Resistencia superior del divisor |
| R2 | 10 kΩ | Resistencia inferior (a GND) + pull-down |

Cálculo: V_GPIO40 = 12 V × 10k / (10k + 10k) = **6 V** → ❌ **PELIGROSO: supera 3.3 V, destruiría el pin GPIO del ESP32.**

> ⚠️ **EJEMPLO INCORRECTO — NO USAR estos valores:**

| Componente | Valor | Función |
|------------|-------|---------|
| R1 | 22 kΩ | Resistencia superior del divisor |
| R2 | 10 kΩ | Resistencia inferior (a GND) |

Cálculo: V_GPIO40 = 12 V × 10k / (22k + 10k) = **3.75 V** → ❌ **INSEGURO: sigue superando 3.3 V máximo absoluto del ESP32.**

**✅ Valores recomendados (margen seguro):**

| Componente | Valor | Función |
|------------|-------|---------|
| R1 | 33 kΩ | Resistencia superior del divisor |
| R2 | 10 kΩ | Resistencia inferior (a GND) |

Cálculo: V_GPIO40 = 12 V × 10k / (33k + 10k) = **2.79 V** ✅ (< 3.3 V máximo absoluto; > 2.48 V umbral HIGH típico del ESP32-S3, ~0.75 × VDD según datasheet)

> **Referencia firmware:** `power_manager.h` línea 10: `PIN_IGNITION_SENSE = 40`  
> **Referencia docs:** `LISTADO_PINES_COMPLETO.md` líneas 145-153

### 2.3 Esquema de cableado físico de la llave

```
  Batería 12 V (+)
       │
       │  Cable rojo (min 0.5 mm²)
       ▼
  ┌──────────┐
  │  LLAVE   │  ← Interruptor de contacto (llave física del vehículo)
  │ CONTACTO │     Tipo: SPST (Single Pole, Single Throw)
  │          │     Posición ON: circuito cerrado
  │          │     Posición OFF: circuito abierto
  └────┬─────┘
       │
       │  Cable naranja (señal, 0.5 mm²)
       ▼
  ┌─────────┐
  │  R1     │  33 kΩ, ¼ W
  └────┬────┘
       │
       ├──────────────────► ESP32-S3 GPIO 40
       │
  ┌────┴────┐
  │  R2     │  10 kΩ, ¼ W
  └────┬────┘
       │
      GND ────────────────► ESP32-S3 GND
```

**Lógica de la señal:**
- **Llave ON** → GPIO 40 = HIGH (≈ 2.79 V con divisor) → ESP32 detecta encendido
- **Llave OFF** → GPIO 40 = LOW (pull-down interno a GND) → ESP32 detecta apagado

---

## 3. Circuito de Alimentación Permanente y Retención

### 3.1 ¿Cómo se alimenta el ESP32-S3?

El ESP32-S3 necesita alimentación **antes** de que la llave se gire y
**después** de que se apague (para reproducir el audio de despedida y guardar
datos). Esto se consigue con un **relé de retención** (delay relay) externo:

```
  Batería 12 V (+)
       │
       ├──────────────────────────────────────────────────────────┐
       │                                                          │
       ▼                                                          ▼
  ┌──────────┐                                              ┌──────────┐
  │  LLAVE   │                                              │  RELÉ DE │
  │ CONTACTO │                                              │RETENCIÓN │
  └────┬─────┘                                              │ (externo)│
       │                                                    └────┬─────┘
       │  Señal ON/OFF                                           │
       │  (vía divisor)                                          │ 12 V controlado
       ▼                                                         ▼
  ESP32 GPIO 40                                          ┌──────────────┐
  (IGNITION_SENSE)                                       │  Regulador   │
                                                         │  12V → 5V    │
  ESP32 GPIO 41 ──────────► Bobina del relé              │  (LM7805 o   │
  (POWER_HOLD)               de retención               │   buck)      │
                                                         └──────┬───────┘
                                                                │ 5 V
                                                                ▼
                                                         ESP32-S3 (5V pin)
                                                         STM32 Nucleo (5V→3.3V LDO)
```

### 3.2 Funcionamiento del circuito de retención

1. **Llave ON** → La llave conecta 12 V directamente al relé de retención,
   que cierra y alimenta al ESP32 y STM32.
2. **ESP32 arranca** → GPIO 41 (`POWER_HOLD`) se pone HIGH → mantiene el
   relé de retención energizado **independientemente** de la llave.
3. **Llave OFF** → La llave se abre, pero GPIO 41 sigue HIGH → el relé
   permanece cerrado → el sistema sigue con alimentación.
4. **Audio de despedida** → El ESP32 reproduce el sonido de apagado (3 s).
5. **GPIO 41 LOW** → El ESP32 libera `POWER_HOLD` → el relé se abre →
   alimentación cortada → sistema completamente apagado.

> **Referencia firmware:** `power_manager.cpp` líneas 84, 127-128

### 3.3 Esquema del relé de retención

> **⚠️ IMPORTANTE:** El diodo flyback D2 (1N4007) debe estar en **PARALELO** con
> la bobina del relé (cátodo a bobina+, ánodo a bobina−/GND), **nunca en serie**.
> Un diodo en serie NO protege contra el pico inductivo al desconectar la bobina.
> Ver [IGNITION_KEY_CIRCUIT_VALIDATION.md](IGNITION_KEY_CIRCUIT_VALIDATION.md)
> para la validación eléctrica completa y las correcciones de diseño.

```
  Bat 12V (+) ──┬─── Llave contacto ────────────────────────────┐
                │                                                │
                │   ┌──────────────────────────────────┐         │
                │   │   RELÉ RETENCIÓN                 │         │
                │   │   (12V DC, contactos ≥ 5A)       │         │
                │   │                                  │         │
                │   │         ┌────────┐               │         │
                │   │  Bobina(+) ◄──┤ D2     │◄── Bobina(−)│    │
                │   │     │         │ 1N4007 │       │     │     │
                │   │     │         │flyback │       │     │     │
                │   │     │         │PARALELO│       │     │     │
                │   │     │         └────────┘       │     │     │
                │   │     │                          │     │     │
                │   │     │◄───── D_OR1 ◄── R4 ◄────┤─────┘     │
                │   │     │       (1N4148)  (10kΩ)   │  (llave)  │
                │   │     │                          │           │
                │   │     │◄───── D_OR2 ◄── Q2_col   │           │
                │   │     │       (1N4148)     ▲     │           │
                │   │     │                    │     │           │
                │   │     │              Q1 NPN(BC547)│           │
                │   │     │              Base ◄── R_base (1kΩ) ◄── ESP32 GPIO 41
                │   │     │              Emisor ──► GND│           │
                │   │     │                          │           │
                │   │     └──── bobina(−) ──► GND    │           │
                │   │                                  │           │
                │   │  COM ◄───────────────────────────┘ (Bat 12V+)
                │   │  NO  ──────────────────► Regulador 5V → ESP32 + STM32
                │   │  NC  ── (sin conectar)  │
                │   └──────────────────────────────────┘
                │
               GND
```

**Nota:** La bobina del relé se activa por **dos caminos en paralelo** (diodo OR):
1. Desde la llave de contacto vía **R4 (10kΩ) + Q1 transistor NPN** (encendido inicial).
   La llave **no acciona la bobina directamente** — solo alimenta la base del transistor
   (~1.13 mA), evitando arco eléctrico y desgaste de los contactos de la llave.
2. Desde el GPIO 41 del ESP32 vía transistor NPN Q2 (retención durante apagado).

Ambos caminos se combinan con diodos OR (1N4148 × 2) para evitar retroalimentación.
El diodo flyback D2 (1N4007) en **paralelo** con la bobina absorbe los picos inductivos
al desconectar.

---

## 4. Secuencia Completa de Encendido

### 4.1 Línea temporal del encendido

```
  t=0        t~100ms      t~300ms       t~600ms       t~800ms      t~850ms    t~870ms
   │            │            │             │             │            │          │
   ▼            ▼            ▼             ▼             ▼            ▼          ▼
 LLAVE ON → ESP32      → ESP32         → ESP32       → STM32      → STM32    → STM32
             detecta     POWER_HOLD     RUNNING        recibe       Relay      relés
             GPIO 40     HIGH           Audio          heartbeat    PowerUp()  completos
             HIGH        (retención)    bienvenida     ESP32                   ACTIVE
                                                       → STANDBY
                                                       → ACTIVE
```

### 4.2 Detalle paso a paso

| Paso | Tiempo | Componente | Acción | Referencia código |
|------|--------|------------|--------|-------------------|
| 1 | t=0 | **Llave** | Usuario gira llave a ON → 12 V al relé de retención | Hardware |
| 2 | t=0 | **Relé retención** | Cierra contactos → 12 V llega al regulador 5 V | Hardware |
| 3 | t=0 | **Regulador 5 V** | Convierte 12 V → 5 V → alimenta ESP32 y STM32 | Hardware |
| 4 | t~50 ms | **ESP32-S3** | Arranca, ejecuta `setup()` (510 ms de inicialización) | `main.cpp:285-330` |
| 5 | t~50 ms | **STM32G474RE** | Arranca simultáneamente, ejecuta `HAL_Init()`, `SystemClock_Config()` | `main.c:117-130` |
| 6 | t~60 ms | **STM32** | Lee causa de reset, inicia periféricos (ADC, CAN, I2C, timers) | `main.c:120-180` |
| 7 | t~100 ms | **ESP32** | `power_mgr::init()` → lee GPIO 40 (HIGH) → llave detectada | `power_manager.cpp:52-73` |
| 8 | t~100 ms | **ESP32** | Estado → `POWER_HOLD` → GPIO 41 HIGH (retiene alimentación) | `power_manager.cpp:83-88` |
| 9 | t~100 ms | **ESP32** | Estado → `STARTING` (inmediato desde POWER_HOLD) | `power_manager.cpp:91-95` |
| 10 | t~300 ms | **ESP32** | Estado → `RUNNING` (tras 200 ms de STARTUP_DELAY) | `power_manager.cpp:98-104` |
| 11 | t~300 ms | **ESP32** | Reproduce audio de bienvenida vía DFPlayer | `main.cpp:593-596` |
| 12 | t~560 ms | **ESP32** | `setup()` completo → entra en `loop()` → envía heartbeat 0x011 | `main.cpp:560-590` |
| 13 | continuo | **ESP32** | Heartbeat CAN 0x011 cada 100 ms al STM32 | `main.cpp` |
| 14 | — | **STM32** | Recibe primer heartbeat → estado BOOT → STANDBY → ACTIVE | `safety_system.c:252-264` |
| 15 | — | **STM32** | `Relay_PowerUp()` invocado → inicia secuencia de relés | `safety_system.c:264` |
| 16 | t+0 ms | **STM32** | `RELAY_MAIN` ON (PC10 → HIGH) | `safety_system.c:467` |
| 17 | t+50 ms | **STM32** | Espera 50 ms (asentamiento corriente inrush) → `RELAY_TRAC` ON (PC11) | `safety_system.c:482-484` |
| 18 | t+70 ms | **STM32** | Espera 20 ms (supresión de arco) → `RELAY_DIR` ON (PC12) | `safety_system.c:490-492` |
| 19 | — | **STM32** | Sistema en estado ACTIVE → motores habilitados, esperando demanda | — |
| 20 | — | **STM32** | `startup_inhibit` activo → espera pedal < 3% durante 400 ms para desbloquear | `main.c:94-96` |

### 4.3 Protecciones durante el encendido

- **Inhibición de pedal al arranque:** Si el pedal está pisado al encender, los
  motores permanecen bloqueados hasta que el pedal se suelte (< 3 %) durante
  400 ms continuos.
- **Validación de boot:** 6 chequeos obligatorios de sensores antes de permitir
  estado ACTIVE.
- **Centrado de dirección:** El motor de dirección busca la posición central
  usando el sensor inductivo LJ12A3 + encoder.

---

## 5. Flujo de Corriente hacia los Motores de Tracción

### 5.1 Cadena de potencia completa

Una vez que los relés están cerrados, la corriente fluye así:

```
 ┌─────────────┐                                                     ┌──────────┐
 │ Batería 24V │                                                     │ 4× Motor │
 │   Tracción  │                                                     │ Tracción │
 └──────┬──────┘                                                     └────▲─────┘
        │                                                                  │
        │ 4 mm²                                                           │ 2.5 mm²
        ▼                                                                  │
   INA226 ch4 ← mide tensión de batería (incluso con relé abierto)       │
   (0.75 mΩ)                                                               │
        │                                                                  │
        ▼                                                                  │
   ┌─────────┐     ┌─────────┐     ┌────┬────┬────┬────┐     ┌──────────┐│
   │RELAY_MAIN│ →  │RELAY_TRAC│ →  │INA │INA │INA │INA │ →  │4×BTS7960 │┘
   │ (PC10)  │     │ (PC11)  │     │ch0 │ch1 │ch2 │ch3 │     │ drivers  │
   │ 60A fuse│     │ 50A fuse│     │1.5m│1.5m│1.5m│1.5m│     │          │
   └─────────┘     └─────────┘     └────┴────┴────┴────┘     └──────────┘
                                                                    ▲
                                                                    │ PWM 20 kHz
                                                               STM32 TIM1/TIM8
                                                               (PA8-PA10+PC3, PC6-PC9)
```

### 5.2 Cadena de potencia de dirección

```
 ┌─────────────┐
 │ Batería 12V │
 │  Dirección  │
 └──────┬──────┘
        │ 4 mm²
        ▼
   ┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐
   │RELAY_DIR │ →  │INA226 ch5│ →  │ BTS7960  │ →  │  Motor   │
   │ (PC12)   │     │(1.5 mΩ) │     │ steering │     │Dirección │
   │ 20A fuse │     │          │     │          │     │          │
   └──────────┘     └──────────┘     └──────────┘     └──────────┘
                                          ▲
                                          │ PWM 20 kHz
                                     STM32 TIM3
                                     (PA6, PA7)
```

### 5.3 Alimentación lógica (3.3 V / 5 V)

```
 Regulador 5 V (desde relé de retención / 12 V)
        │
        ├──► ESP32-S3 (pin 5V)
        │
        ├──► DC-DC 24→5V (LM2596) ──► Lógica BTS7960 (VCC) + WS2812B LEDs
        │
        └──► LDO 3.3 V (integrado en Nucleo-64)
                │
                ├──► STM32G474RE (MCU, 170 MHz)
                ├──► TCA9548A + 6× INA226 (I²C)
                ├──► Transceiver CAN TJA1051T/3 (VCC=5V, VIO=3.3V)
                └──► Señales digitales (PWM, GPIO, encoder)
```

---

## 6. Secuencia Completa de Apagado

### 6.1 Línea temporal del apagado

```
  t=0          t~50ms        t~3000ms       t~3050ms       t~3100ms
   │              │              │              │              │
   ▼              ▼              ▼              ▼              ▼
 LLAVE OFF →  ESP32          → ESP32         → ESP32        → Relé de
              detecta          3 s audio       GPIO 41        retención
              GPIO 40 LOW      despedida       LOW            se abre
              → SHUTTING_DOWN  completado     (POWER_HOLD    → SIN
                               guardar datos   liberado)      ALIMENTACIÓN
```

### 6.2 Detalle paso a paso

| Paso | Tiempo | Componente | Acción | Referencia código |
|------|--------|------------|--------|-------------------|
| 1 | t=0 | **Usuario** | Gira la llave a OFF → 12 V cortado a la llave | Hardware |
| 2 | t~50 ms | **ESP32** | GPIO 40 lee LOW (debounce 50 ms) → `SHUTTING_DOWN` | `power_manager.cpp:115-118` |
| 3 | t~50 ms | **ESP32** | `config_store::flush()` → guarda configuración en flash | `main.cpp:602` |
| 4 | t~50 ms | **ESP32** | Reproduce audio de despedida (`FAREWELL`) | `main.cpp:603` |
| 5 | t~50 ms | **ESP32** | GPIO 41 sigue HIGH → relé de retención mantiene 12 V | `power_manager.cpp` |
| 6 | t~3050 ms | **ESP32** | Han pasado 3000 ms (`SHUTDOWN_DELAY_MS`) | `power_manager.cpp:126` |
| 7 | t~3050 ms | **ESP32** | GPIO 41 → LOW → `POWER_HOLD` liberado | `power_manager.cpp:127` |
| 8 | t~3050 ms | **ESP32** | Estado → `OFF` | `power_manager.cpp:128-129` |
| 9 | t~3100 ms | **Relé retención** | Se abre (ya no tiene ni llave ni POWER_HOLD) | Hardware |
| 10 | t~3100 ms | **Regulador 5V** | Sin entrada → 5 V cae a 0 V | Hardware |
| 11 | inmediato | **STM32** | Pierde alimentación → periféricos se apagan | Hardware |
| 12 | inmediato | **ESP32** | Pierde alimentación → se apaga | Hardware |

### 6.3 ¿Qué pasa con los relés de potencia del STM32?

Cuando el STM32 pierde alimentación, **todos los GPIOs caen a LOW**, lo que
significa que los relés RELAY_MAIN, RELAY_TRAC y RELAY_DIR se abren
automáticamente por pérdida de señal. Este es un comportamiento **fail-safe**:

- Los motores se desconectan de las baterías instantáneamente.
- No hay riesgo de que un relé quede "pegado" en ON por software.
- Los diodos flyback (1N4007) en las bobinas de los relés absorben los picos
  inductivos generados al cortar.

### 6.4 Apagado por software (estados SAFE/ERROR del STM32)

Si el STM32 detecta una condición peligrosa **antes** de que se apague la llave,
realiza su propia secuencia de apagado por software:

```c
void Safety_PowerDown(void)       // safety_system.c:1343
{
    Traction_EmergencyStop();     // Todos los PWM a 0%
    Relay_PowerDown();            // Relés en orden inverso
}

void Relay_PowerDown(void)        // safety_system.c:503
{
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_DIR,  GPIO_PIN_RESET);  // Dirección OFF
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_TRAC, GPIO_PIN_RESET);  // Tracción OFF
    HAL_GPIO_WritePin(GPIOC, PIN_RELAY_MAIN, GPIO_PIN_RESET);  // Principal OFF
}
```

Orden de apagado: **DIR → TRAC → MAIN** (inverso al encendido).

---

## 7. Diagrama Eléctrico Completo

### 7.1 Circuito completo desde la llave hasta los motores

```
                                    ┌────────────────────────────────────────────────────────────┐
                                    │                    STM32G474RE                              │
                                    │                                                            │
                   ┌────────────────┤ PC10 (RELAY_MAIN) ──► Opto ──► Relé MAIN (60A fuse)       │
                   │                │ PC11 (RELAY_TRAC) ──► Opto ──► Relé TRAC (50A fuse)       │
  ┌─────────┐      │                │ PC12 (RELAY_DIR)  ──► Opto ──► Relé DIR  (20A fuse)       │
  │Bat 12 V │──┐   │                │                                                            │
  └─────────┘  │   │                │ TIM1 CH1-CH3 (PA8-PA10) + CH4 (PC3) ──► BTS7960 FL/FR          │
               │   │                │ TIM8 CH1-CH4 (PC6-PC9)  ──► BTS7960 RL/RR                 │
               │   │                │ TIM3 CH1-CH2 (PA6-PA7)  ──► BTS7960 Dirección              │
               │   │                └────────────────────────────────────────────────────────────┘
               │   │                                    ▲ 3.3V (LDO)
               │   │                                    │
               │   │                              ┌─────┴──────┐
               │   │           ┌─────────────────►│ Regulador  │◄─── DC-DC 24→5V (LM2596)
               │   │           │                  │  5V → 3.3V │     desde bus 24V (tras RELAY_MAIN)
               │   │           │                  └────────────┘
               │   │           │
               │   │    ┌──────┴────────────────────────────────────────────────────────┐
               │   │    │                      ESP32-S3                                  │
               │   │    │                                                                │
               │   │    │  GPIO 40 (IGNITION_SENSE) ◄── R1=33kΩ ── Llave contacto      │
               │   │    │  GPIO 41 (POWER_HOLD)     ──► Transistor ──► Relé retención   │
               │   │    │  GPIO 4/5 (CAN TX/RX)     ──► TJA1051T/3 ──► CAN Bus ──► STM32  │
               │   │    └───────────────────────────────────────────────────────────────┘
               │   │                                    ▲ 5V
               │   │                                    │
               │   │                              ┌─────┴──────┐
               │   └────── Llave contacto ──────►│ RELÉ DE    │
               │                                  │ RETENCIÓN  │
               └──── (alimentación directa) ────►│ (12V→5V    │
                                                  │  regulador)│
                                                  └────────────┘
                                                        │ 5V
                                                        ▼
                                                  ESP32 + STM32
                                                  alimentados

  ┌─────────┐
  │Bat 24 V │──► INA226 ch4 ──► RELAY_MAIN ──► RELAY_TRAC ──┬── INA226 ch0 ── BTS7960 ── Motor FL
  └─────────┘    (0.75mΩ)      (PC10)         (PC11)        ├── INA226 ch1 ── BTS7960 ── Motor FR
                                                              ├── INA226 ch2 ── BTS7960 ── Motor RL
                                                              └── INA226 ch3 ── BTS7960 ── Motor RR

  ┌─────────┐
  │Bat 12 V │──► RELAY_DIR ──► INA226 ch5 ── BTS7960 ── Motor Dirección
  └─────────┘    (PC12)        (1.5mΩ)
```

---

## 8. Lista de Componentes del Circuito de Llave

### 8.1 Circuito de detección de llave (ESP32 GPIO 40)

| Componente | Valor | Función |
|------------|-------|---------|
| Llave de contacto | SPST, rated ≥ 12 V / 1 A | Interruptor maestro ON/OFF (2 terminales) |
| R1 (divisor) | 33 kΩ, ¼ W, metal film | Resistencia superior del divisor de tensión |
| R2 (divisor) | 10 kΩ, ¼ W, metal film | Resistencia inferior + pull-down |
| R3 (serie GPIO) | 10 kΩ, ¼ W | Protección anti-transitorio + parte del filtro RC |
| C1 (filtro) | 100 nF (0.1 µF), 50 V, X7R | Filtro RC anti-ruido (τ = R3×C1 = 1 ms) |
| Cable señal | 0.5 mm², apantallado recomendado | De la llave al divisor |

> **Mejora opcional:** añadir zener 3.3 V (BZX55C3V3) entre GPIO 40 y GND para
> clamp de protección contra transitorios automotrices.
> Ver [IGNITION_KEY_CIRCUIT_VALIDATION.md](IGNITION_KEY_CIRCUIT_VALIDATION.md) §8.

### 8.2 Circuito de retención de alimentación (ESP32 GPIO 41)

| Componente | Valor | Función |
|------------|-------|---------|
| Q1 (transistor NPN) | BC547B o 2N2222 (camino llave) | Driver de bobina desde señal de llave |
| Q2 (transistor NPN) | BC547B o 2N2222 (camino GPIO 41) | Driver de bobina desde POWER_HOLD |
| R4 (base Q1) | 10 kΩ, ¼ W | Limita corriente de base Q1 (~1.13 mA) desde llave |
| R_base (Q2) | 1 kΩ, ¼ W | Limita corriente de base Q2 desde GPIO 41 (3.3V) |
| D2 (flyback) | **1N4007** — en **PARALELO** con bobina | Protección contra pico inductivo (cátodo→bobina+, ánodo→bobina−) |
| D_OR1, D_OR2 | 1N4148 × 2 | Combina señal de llave + POWER_HOLD sin retroalimentación |
| Relé de retención | 12 V DC, contactos ≥ 5 A | Mantiene alimentación durante apagado |
| Regulador 5 V | LM7805 o módulo buck 12 V→5 V | Alimentación de ESP32-S3 y STM32 |

> ⚠️ **CRÍTICO:** El diodo flyback D2 debe estar en **PARALELO** con la bobina
> del relé, **no en serie**. Un diodo en serie no protege contra kickback inductivo.
> Ver [IGNITION_KEY_CIRCUIT_VALIDATION.md](IGNITION_KEY_CIRCUIT_VALIDATION.md) §3.1 y §7.1.

### 8.3 Relés de potencia (controlados por STM32)

| Componente | Pin STM32 | Fusible | Función |
|------------|-----------|---------|---------|
| RELAY_MAIN | PC10 | 60 A | Alimentación general 24 V |
| RELAY_TRAC | PC11 | 50 A | Motores de tracción 24 V |
| RELAY_DIR | PC12 | 20 A | Motor de dirección 12 V |

Cada relé incluye: optoacoplador HY-M158 + R 330 Ω + diodo flyback 1N4007 + snubber RC (100 Ω + 100 nF / 250 V).

---

## 9. Preguntas Frecuentes

### ¿Por qué la llave va al ESP32 y no al STM32?

La llave de contacto es una función de **interfaz de usuario** (encender/apagar
el vehículo). El ESP32-S3 es el controlador de HMI que gestiona la pantalla,
el audio y la interacción con el usuario. El STM32 es la **autoridad de
seguridad** que controla motores y relés, pero no necesita leer la llave
directamente — recibe la información del estado del sistema a través del bus CAN.

### ¿Qué pasa si el ESP32 se desconecta o falla?

Si el STM32 deja de recibir heartbeats CAN del ESP32 durante más de 250 ms,
entra en modo **LIMP_HOME** (20 % de potencia máxima, 5 km/h). El vehículo
permanece operable a velocidad de peatón usando solo el pedal local, sin
necesidad del ESP32.

### ¿Qué pasa si giro la llave a OFF mientras conduzco?

1. El ESP32 detecta llave OFF y entra en `SHUTTING_DOWN`.
2. Deja de enviar heartbeats CAN.
3. El STM32 detecta timeout CAN (250 ms) y entra en LIMP_HOME.
4. Tras 3 segundos, el ESP32 libera `POWER_HOLD` y el sistema se apaga.
5. El STM32 pierde alimentación → todos los GPIOs a LOW → relés se abren →
   motores desconectados.

### ¿Se puede encender sin la llave (bypass)?

Sí, para pruebas en banco: conectar GPIO 40 del ESP32 directamente a 3.3 V
(sin divisor). Esto simula la llave en posición ON permanente.

### ¿Qué corriente consume la llave?

La corriente por el divisor es mínima: I = 12 V / (33 kΩ + 10 kΩ) ≈ 0.28 mA.
La llave no necesita soportar más que esa corriente para la señal de detección.
La corriente de potencia pasa por el relé de retención, no por la llave
directamente (excepto para accionar la bobina del relé: ~70 mA).

---

> **Documento generado a partir del firmware (`esp32/src/power_manager.h`,
> `esp32/src/power_manager.cpp`, `esp32/src/main.cpp`, `Core/Src/safety_system.c`,
> `Core/Inc/main.h`, `Core/Src/main.c`) y de la documentación existente
> (`POWER_DISTRIBUTION.md`, `HARDWARE_WIRING_MANUAL.md`,
> `LISTADO_PINES_COMPLETO.md`). No contiene hardware inventado.**
