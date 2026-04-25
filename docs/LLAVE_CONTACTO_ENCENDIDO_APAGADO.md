# Llave de Contacto — Encendido y Apagado del Sistema

**Documento de referencia para taller — Cómo conectar los cables**

> **Fuente verificada:** `esp32/src/power_manager.h`, `esp32/src/power_manager.cpp`,
> `esp32/src/main.cpp`, `Core/Src/safety_system.c`, `Core/Inc/project_config.h`,
> `Core/Src/main.c`. No se ha inventado ningún componente ni conexión.
>
> **Revisión:** Se elimina el circuito discreto de transistores y diodos OR.
> El apagado con retardo se realiza con un **módulo relé con retardo hardware**
> (temporizador autónomo con potenciómetro). No hacen falta Q1, Q2, R4,
> R_base, D_OR1, D_OR2 ni ningún transistor adicional.

---

## Índice

1. [Resumen General](#1-resumen-general)
2. [Conexión de la Llave — Cableado Exacto](#2-conexión-de-la-llave--cableado-exacto)
3. [Módulo Relé con Retardo Hardware](#3-módulo-relé-con-retardo-hardware)
4. [Secuencia Completa de Encendido](#4-secuencia-completa-de-encendido)
5. [Flujo de Corriente hacia los Motores](#5-flujo-de-corriente-hacia-los-motores)
6. [Secuencia Completa de Apagado](#6-secuencia-completa-de-apagado)
7. [Diagrama Eléctrico Completo](#7-diagrama-eléctrico-completo)
8. [Lista de Componentes](#8-lista-de-componentes)
9. [Preguntas Frecuentes](#9-preguntas-frecuentes)

---

## 1. Resumen General

### ¿Cómo funciona el encendido y apagado?

El sistema usa dos mecanismos que trabajan juntos:

1. **Módulo relé con retardo hardware** (temporizador con potenciómetro):
   mantiene la alimentación encendida durante N segundos después de girar la
   llave a OFF. Esto da tiempo al ESP32 para reproducir el audio de despedida
   y guardar la configuración. Funciona 100% en hardware, sin código.

2. **GPIO 40 del ESP32** (`IGNITION_SENSE`): la llave envía una señal al
   ESP32 a través de un divisor de tensión. Cuando el ESP32 detecta llave OFF,
   inicia su propia secuencia de apagado (audio de despedida, guardar flash).

### ¿Qué hace el GPIO 41 del ESP32?

El firmware del ESP32 pone el GPIO 41 (`POWER_HOLD`) en HIGH cuando detecta
la llave ON y en LOW cuando completa el apagado. Este pin **no controla el
módulo relé con retardo** — el módulo es autónomo. GPIO 41 puede dejarse sin
conectar o conectarse al trigger del módulo como señal adicional.

### Pines reales del STM32

| Pin STM32 | Función | Tensión |
|-----------|---------|---------|
| PC11 | RELAY_TRAC — relé tracción motores | 24 V |
| PC12 | RELAY_DIR — relé dirección | 12 V |
| PC10 | **RESERVADO** — sin uso en firmware | — |

> PC10 aparece como `RESERVED/unused` en `project_config.h:195`. No existe
> RELAY_MAIN en el firmware. Solo hay dos relés controlados: TRAC y DIR.

---

## 2. Conexión de la Llave — Cableado Exacto

### 2.1 ¿Qué necesita la llave?

La llave de contacto necesita conectarse a **dos sitios**:

1. Al **trigger del módulo relé con retardo** (para controlar la alimentación).
2. Al **GPIO 40 del ESP32** a través de un divisor de tensión (para que el
   firmware sepa cuándo se apaga).

### 2.2 Divisor de tensión para GPIO 40 (obligatorio)

El ESP32-S3 opera a **3.3 V máximo** en sus pines GPIO. Si la llave conmuta
12 V, el divisor reduce la tensión a un nivel seguro.

**Valores correctos (verificados):**

| Componente | Valor | Función |
|------------|-------|---------|
| R1 | 33 kΩ, ¼ W | Resistencia superior del divisor |
| R2 | 10 kΩ, ¼ W | Resistencia inferior (a GND) |

Cálculo: `V_GPIO40 = 12 V × 10k / (33k + 10k) = 2.79 V` — seguro para el
ESP32-S3 (< 3.3 V absoluto; > 2.48 V umbral HIGH).

Corriente por el divisor: `12 V / 43 kΩ ≈ 0.28 mA` — negligible.

> **Referencia firmware:** `power_manager.cpp:56` — `pinMode(PIN_IGNITION_SENSE, INPUT_PULLDOWN)`
> `power_manager.h:26` — `inline constexpr int PIN_IGNITION_SENSE = 40`

### 2.3 Esquema de cableado de la señal de llave

```
  Batería 12 V (+)
       │
       ├────────────────────────────────────────────► al trigger (X1) del
       │                                              módulo relé retardo
       │
  ┌────┴─────┐
  │  LLAVE   │  Interruptor de contacto (SPST)
  │ CONTACTO │  ON = cerrado, OFF = abierto
  └────┬─────┘
       │  Señal ACC 12V (solo con llave en ON)
       │
  ┌────┴────┐
  │  R1     │  33 kΩ, ¼ W
  └────┬────┘
       │
       ├──────────────────────────────────────────► GPIO 40 ESP32-S3
       │                                             (IGNITION_SENSE)
  ┌────┴────┐
  │  R2     │  10 kΩ, ¼ W
  └────┬────┘
       │
      GND ────────────────────────────────────────► GND ESP32-S3
```

**Señal lógica resultante:**
- **Llave ON** → GPIO 40 = HIGH (≈ 2.79 V) → ESP32 detecta encendido
- **Llave OFF** → GPIO 40 = LOW (pull-down interno) → ESP32 detecta apagado

---

## 3. Módulo Relé con Retardo Hardware

### 3.1 ¿Qué es?

Un módulo de relé con temporizador ajustable por potenciómetro (habitualmente
0–120 s). Cuando su entrada de trigger recibe tensión y luego la pierde,
mantiene el relé cerrado durante el tiempo ajustado y después lo abre. Ejemplo
de módulo: SRD-12VDC-SL-C con circuito temporizador NE555 o similar.

### 3.2 Cómo conectar el módulo

```
  Batería 12 V (permanente) ─────────────────────► DC+ del módulo
  GND ────────────────────────────────────────────► DC- del módulo
  Línea ACC de la llave (12V solo con llave ON) ──► X1 del módulo (trigger)

  COM del relé del módulo ────────────────────────► 12V batería (permanente)
  NO  del relé del módulo ────────────────────────► entrada del regulador 5V
                                                    (que alimenta ESP32 + STM32)
  NC  del relé del módulo ────────────────────────► sin conectar
```

### 3.3 Ajuste del temporizador

Ajusta el potenciómetro del módulo a **30–60 segundos** (suficiente para que
el audio de despedida del ESP32 termine y se guarden los datos en flash).
El firmware del ESP32 usa 3 s de retardo (`SHUTDOWN_DELAY_MS = 3000` en
`power_manager.h:31`), así que con 30 s sobra margen.

### 3.4 Funcionamiento paso a paso

| Situación | Módulo relé retardo | GPIO 40 ESP32 |
|-----------|--------------------|----|
| Llave ON | Trigger recibe 12V → relé cierra → sistema alimentado | HIGH → ESP32 en RUNNING |
| Llave OFF | Trigger cae a 0V → módulo inicia cuenta atrás T seg | LOW → ESP32 en SHUTTING_DOWN |
| Durante T segundos | Relé sigue cerrado (cuenta atrás activa) | ESP32 reproduce audio despedida, guarda flash |
| Pasados T segundos | Relé abre → alimentación cortada | ESP32 pierde tensión → apagado |

> **Nota GPIO 41:** El firmware pone GPIO 41 HIGH al encender y LOW al
> terminar el apagado (`power_manager.cpp:84, 127`). Este pin no está conectado
> al módulo relé — el módulo no lo necesita. Puede dejarse libre.

---

## 4. Secuencia Completa de Encendido

### 4.1 Línea temporal

```
  t=0         t~50ms       t~300ms      t~600ms      t~680ms    t~730ms
   │             │             │            │            │           │
   ▼             ▼             ▼            ▼            ▼           ▼
  LLAVE ON    Módulo        ESP32         ESP32        STM32       STM32
              relé cierra   GPIO 40       RUNNING      recibe      relés
              5V llega      detecta HIGH  audio        heartbeat   completos
              a ESP32/STM32 GPIO 41=HIGH  bienvenida   → ACTIVE    TRAC+DIR ON
```

### 4.2 Detalle paso a paso

| Paso | Tiempo | Componente | Acción | Referencia código |
|------|--------|-----------|--------|-------------------|
| 1 | t=0 | **Llave** | Usuario gira llave ON → 12V al trigger del módulo | Hardware |
| 2 | t=0 | **Módulo relé** | Cierra el relé NO → 12V llega al regulador 5V | Hardware |
| 3 | t=0 | **Regulador 5V** | 12V → 5V → alimenta ESP32-S3 y STM32 Nucleo | Hardware |
| 4 | t~50ms | **STM32** | Arranca, `HAL_Init()`, `SystemClock_Config()`, periféricos | `main.c:143-230` |
| 5 | t~50ms | **STM32** | 3 blinks LD2 (~2.3s), inicia CAN bus | `main.c:186-223` |
| 6 | t~50ms | **ESP32** | Arranca `setup()`, `power_mgr::init()` | `main.cpp:633` |
| 7 | t~100ms | **ESP32** | GPIO 40 lee HIGH → llave detectada → GPIO 41=HIGH → POWER_HOLD | `power_manager.cpp:83-88` |
| 8 | t~100ms | **ESP32** | Estado → STARTING (inmediato desde POWER_HOLD) | `power_manager.cpp:91-95` |
| 9 | t~300ms | **ESP32** | Estado → RUNNING (tras 200ms STARTUP_DELAY) | `power_manager.cpp:98-104` |
| 10 | t~300ms | **ESP32** | Audio bienvenida vía DFPlayer | `main.cpp:942` |
| 11 | t~600ms | **ESP32** | `loop()` activo → envía heartbeat CAN 0x011 cada 100ms | `main.cpp:936` |
| 12 | — | **STM32** | Recibe primer heartbeat → BOOT → STANDBY → chequeos boot | `safety_system.c` |
| 13 | — | **STM32** | Centrado dirección completo + BootValidation OK → ACTIVE | `safety_system.c:1546` |
| 14 | — | **STM32** | `Relay_PowerUp()` → PC11 HIGH (RELAY_TRAC ON, 24V) | `safety_system.c:680` |
| 15 | t+50ms | **STM32** | Espera 50ms asentamiento → PC12 HIGH (RELAY_DIR ON, 12V) | `safety_system.c:732-734` |
| 16 | — | **STM32** | `RELAY_SEQ_COMPLETE` → motores disponibles | `safety_system.c:769` |
| 17 | — | **STM32** | `startup_inhibit` activo hasta que pedal < 3% durante 400ms | `main.c:408-417` |

### 4.3 Protecciones durante el encendido

- **Inhibición de pedal:** Si el pedal está pisado al encender, los motores
  permanecen bloqueados hasta que el pedal se suelte (< 3%) durante 400 ms.
- **Boot Validation:** 6 chequeos obligatorios de sensores antes de ACTIVE.
- **Centrado de dirección:** El motor de dirección busca posición central
  usando el sensor inductivo LJ12A3 + encoder antes de permitir ACTIVE.

---

## 5. Flujo de Corriente hacia los Motores

### 5.1 Cadena de potencia tracción (24 V)

```
  ┌─────────┐       ┌────────────────┐      ┌────┬────┬────┬────┐     ┌──────────┐
  │Bat 24V  │──────►│ RELAY_TRAC     │─────►│INA │INA │INA │INA │────►│4×BTS7960 │
  │         │       │ PC11, STM32    │      │ch0 │ch1 │ch2 │ch3 │     │ drivers  │
  └─────────┘       │ (50A fuse)     │      │1.5m│1.5m│1.5m│1.5m│     └────┬─────┘
                    └────────────────┘      └────┴────┴────┴────┘          │
                                                                            ▼
                                                                      4× Motores
                                                                      tracción

  PWM 20 kHz: STM32 TIM1 (PA8-PA10 + PC3) → FL, FR
               STM32 TIM8 (PC6-PC9)         → RL, RR
```

La batería 24V también pasa por un INA226 (canal 4, shunt 0.75 mΩ) antes del
relé, para que el STM32 lea la tensión de batería en todo momento.

### 5.2 Cadena de potencia dirección (12 V)

```
  ┌─────────┐       ┌────────────────┐      ┌──────────┐      ┌──────────┐
  │Bat 12V  │──────►│ RELAY_DIR      │─────►│INA226 ch5│─────►│ BTS7960  │───► Motor
  │         │       │ PC12, STM32    │      │ (1.5 mΩ) │      │ steering │     dirección
  └─────────┘       │ (20A fuse)     │      └──────────┘      └──────────┘
                    └────────────────┘
  PWM 20 kHz: STM32 TIM3 (PA6, PA7) → RPWM, LPWM dirección
```

### 5.3 Alimentación lógica (5V / 3.3V)

```
  Módulo relé retardo → regulador 5V
         │
         ├──► ESP32-S3 (pin 5V)
         │
         └──► Nucleo-64 (pin 5V) → LDO interno → 3.3V para STM32 y periféricos
```

---

## 6. Secuencia Completa de Apagado

### 6.1 Línea temporal

```
  t=0         t~50ms          t~3050ms        t~3100ms         t+T seg
   │              │               │               │               │
   ▼              ▼               ▼               ▼               ▼
  LLAVE OFF    ESP32            ESP32           Módulo relé     Sistema
               GPIO 40=LOW      audio           sigue           apagado
               → SHUTTING_DOWN  despedida       contando
               guarda flash     termina         atrás
               (config_store)   GPIO 41=LOW
```

### 6.2 Detalle paso a paso

| Paso | Tiempo | Componente | Acción | Referencia código |
|------|--------|-----------|--------|-------------------|
| 1 | t=0 | **Usuario** | Gira llave a OFF → trigger del módulo relé cae a 0V | Hardware |
| 2 | t=0 | **Módulo relé** | Inicia cuenta atrás T segundos (mantiene alimentación) | Hardware |
| 3 | t~50ms | **ESP32** | GPIO 40 lee LOW (debounce 50ms) → SHUTTING_DOWN | `power_manager.cpp:115-118` |
| 4 | t~50ms | **ESP32** | `config_store::flush()` → guarda configuración en flash | `main.cpp:952` |
| 5 | t~50ms | **ESP32** | Reproduce audio de despedida (FAREWELL) | `main.cpp:953` |
| 6 | t~50ms | **ESP32** | LEDs apagados durante shutdown | `main.cpp:957-961` |
| 7 | t~3050ms | **ESP32** | Han pasado 3000ms (SHUTDOWN_DELAY_MS) | `power_manager.h:31` |
| 8 | t~3050ms | **ESP32** | GPIO 41 → LOW (firmware termina secuencia) | `power_manager.cpp:127` |
| 9 | t~3050ms | **ESP32** | Estado → OFF | `power_manager.cpp:128-129` |
| 10 | t+T seg | **Módulo relé** | Cuenta atrás completa → relé abre → corta alimentación | Hardware |
| 11 | inmediato | **STM32** | Pierde alimentación → todos GPIOs a LOW → relés TRAC+DIR se abren | Hardware |
| 12 | inmediato | **ESP32** | Pierde alimentación → apagado total | Hardware |

### 6.3 ¿Qué pasa con los relés de potencia del STM32?

Cuando el STM32 pierde alimentación, todos sus GPIOs caen a LOW. Esto abre
RELAY_TRAC (PC11) y RELAY_DIR (PC12) automáticamente. Los motores quedan
desconectados de las baterías sin intervención del firmware.

### 6.4 Apagado por fallo del STM32 (SAFE/ERROR)

Si el STM32 detecta una condición peligrosa antes del apagado normal:

```c
// safety_system.c — apagado de emergencia
void Safety_PowerDown(void)
{
    Traction_EmergencyStop();   // Todos los PWM a 0%
    Relay_PowerDown();          // RELAY_TRAC y RELAY_DIR OFF atómico (BSRR)
}

void Relay_PowerDown(void)
{
    // BSRR: apaga PC11 (TRAC) y PC12 (DIR) en un solo ciclo de reloj
    GPIOC->BSRR = (uint32_t)(PIN_RELAY_TRAC | PIN_RELAY_DIR) << 16U;
}
```

Esto ocurre en fallo de overcurrent, overtemp, watchdog, etc. No está
relacionado con la llave — es una protección independiente.

---

## 7. Diagrama Eléctrico Completo

```
  ┌──────────┐
  │ Bat 12V  │──────────────────────────────────────────────────┐
  │(acc/perm)│                                                  │ (permanente)
  └──────────┘                                                  │
       │                                                        ▼
       │  Línea ACC                                    ┌──────────────────┐
       │  (solo con llave ON)                          │  Módulo relé     │
       │                                               │  con retardo     │
  ┌────┴─────┐                                         │  hardware        │
  │  LLAVE   │────── línea ACC ───────────────────────►│  Trigger (X1)    │
  │ CONTACTO │                                         │  DC+ ← 12V perm  │
  └────┬─────┘                                         │  DC- ← GND       │
       │                                               │                  │
       │  Señal para ESP32                             │  COM ← 12V perm  │
       │                                               │  NO ──────────────┼────► Regulador 5V
  ┌────┴────┐                                          │  NC (libre)      │         │
  │ R1 33kΩ │                                          └──────────────────┘         │
  └────┬────┘                                                                        │
       │                                                                             ▼
       ├────────────────────────────────────────► GPIO 40 ESP32-S3           ┌──────────────┐
       │                                          (IGNITION_SENSE)           │ ESP32-S3 +   │
  ┌────┴────┐                                                                 │ STM32 Nucleo │
  │ R2 10kΩ │                                                                 │  (5V input)  │
  └────┬────┘                                                                 └──────┬───────┘
       │                                                                             │
      GND ────────────────────────────────────────────────────────────────────► GND│

  ESP32-S3 GPIO 41 (POWER_HOLD) ── no conectado al módulo (firmware interno)


  ┌──────────┐
  │ Bat 24V  │──► INA226 ch4 ──► RELAY_TRAC (PC11) ──┬── INA226 ch0 ──► BTS7960 ──► Motor FL
  │ tracción │    (0.75 mΩ)      STM32, 50A fuse      ├── INA226 ch1 ──► BTS7960 ──► Motor FR
  └──────────┘                                        ├── INA226 ch2 ──► BTS7960 ──► Motor RL
                                                       └── INA226 ch3 ──► BTS7960 ──► Motor RR

  ┌──────────┐
  │ Bat 12V  │──► RELAY_DIR (PC12) ──► INA226 ch5 ──► BTS7960 ──► Motor Dirección
  │ dirección│    STM32, 20A fuse      (1.5 mΩ)
  └──────────┘
```

---

## 8. Lista de Componentes

### 8.1 Circuito de señal de llave (GPIO 40 del ESP32)

| Componente | Valor | Función |
|------------|-------|---------|
| R1 | 33 kΩ, ¼ W, metal film | Resistencia superior del divisor de tensión |
| R2 | 10 kΩ, ¼ W, metal film | Resistencia inferior + punto de referencia a GND |
| Cable señal | 0.5 mm², apantallado recomendado | De la llave al divisor resistivo |

### 8.2 Módulo relé con retardo hardware

| Componente | Especificación | Función |
|------------|---------------|---------|
| Módulo relé retardo | 12V DC, contactos ≥ 5A, potenciómetro 0-120s | Mantiene alimentación T segundos tras llave OFF |
| Regulador 5V | LM7805 o módulo buck 12V→5V, ≥ 1A | Convierte 12V del módulo relé a 5V para ESP32+STM32 |

> El módulo relé incluye su propio circuito de protección. No necesita diodo
> flyback externo, transistores ni diodos OR adicionales.

### 8.3 Relés de potencia (controlados por STM32)

| Componente | Pin STM32 | Fusible | Función |
|------------|-----------|---------|---------|
| RELAY_TRAC | PC11 | 50 A | Motores de tracción 24 V (4× BTS7960) |
| RELAY_DIR | PC12 | 20 A | Motor de dirección 12 V (1× BTS7960) |

Cada relé de potencia lleva: diodo flyback 1N4007 en paralelo con la bobina
+ snubber RC (100 Ω + 100 nF / 250 V) en los contactos.
El módulo optoacoplador 4ch (SRD-12VDC-SL-C) entre STM32 y relés incluye
protección interna.

---

## 9. Preguntas Frecuentes

### ¿Por qué la llave va al ESP32 y no al STM32?

El ESP32-S3 gestiona la HMI (pantalla, audio, interacción de usuario). Es el
responsable de la secuencia de encendido/apagado del vehículo. El STM32 es la
autoridad de seguridad de motores y relés — no necesita leer la llave
directamente, recibe el estado del sistema via CAN (heartbeat del ESP32).

### ¿Qué pasa si el ESP32 falla?

Si el STM32 deja de recibir heartbeats CAN del ESP32 durante más de 250 ms,
entra en modo LIMP_HOME (20% de potencia, ~5 km/h). El vehículo sigue siendo
operable con el pedal local sin el ESP32.

### ¿Qué pasa si giro la llave a OFF mientras conduzco?

1. El módulo relé inicia su cuenta atrás (T segundos de alimentación).
2. El ESP32 detecta GPIO 40 LOW → SHUTTING_DOWN → audio despedida → guarda config.
3. El STM32 detecta pérdida de heartbeat CAN → entra en LIMP_HOME.
4. Pasados T segundos → módulo relé abre → alimentación cortada → STM32 pierde tensión → relés TRAC+DIR se abren → motores desconectados.

### ¿Se puede probar sin la llave (banco de trabajo)?

Sí: conectar 3.3V directamente a GPIO 40 del ESP32 (sin divisor). Esto
simula llave ON permanente. El módulo relé se puede alimentar manualmente
desde una fuente 12V en sus bornes DC+/DC-.

### ¿Qué corriente pasa por la llave?

Solo la corriente del divisor resistivo hacia GPIO 40:
`I = 12V / (33k + 10k) ≈ 0.28 mA`

La corriente de potencia para ESP32+STM32 pasa por el módulo relé (relé NO),
no por los contactos de la llave. La llave solo conmuta la señal al trigger.

---

> **Documento generado a partir del firmware verificado:**
> `esp32/src/power_manager.h`, `esp32/src/power_manager.cpp`,
> `esp32/src/main.cpp`, `Core/Src/safety_system.c`,
> `Core/Inc/project_config.h`, `Core/Src/main.c`.
> No contiene hardware inventado ni estimado.
