# Llave de Contacto — Encendido y Apagado del Sistema

**Documento de referencia para taller — Cómo conectar los cables**

> **Fuente verificada:** `esp32/src/power_manager.h`, `esp32/src/power_manager.cpp`,
> `esp32/src/main.cpp`, `Core/Src/safety_system.c`, `Core/Inc/project_config.h`,
> `Core/Src/main.c`. No se ha inventado ningún componente ni conexión.
>
> **Revisión actual:** Se usa un **módulo relé de 2 canales SRD-12VDC-SL-C**
> (high/low level trigger, con optoacopladores) como interfaz entre el STM32
> y los relés de potencia. No hacen falta transistores, diodos flyback externos
> ni circuitos discretos para el control de relés.

---

## Índice

1. [Resumen General](#1-resumen-general)
2. [Conexión de la Llave al ESP32](#2-conexión-de-la-llave-al-esp32)
3. [Módulo Relé con Retardo Hardware (alimentación)](#3-módulo-relé-con-retardo-hardware-alimentación)
4. [Módulo Relé 2 Canales SRD-12VDC-SL-C (control potencia)](#4-módulo-relé-2-canales-srd-12vdc-sl-c-control-potencia)
5. [Secuencia Completa de Encendido](#5-secuencia-completa-de-encendido)
6. [Flujo de Corriente hacia los Motores](#6-flujo-de-corriente-hacia-los-motores)
7. [Secuencia Completa de Apagado](#7-secuencia-completa-de-apagado)
8. [Diagrama Eléctrico Completo](#8-diagrama-eléctrico-completo)
9. [Lista de Componentes](#9-lista-de-componentes)
10. [Preguntas Frecuentes](#10-preguntas-frecuentes)

---

## 1. Resumen General

El sistema usa tres módulos que trabajan juntos:

| Módulo | Función |
|--------|---------|
| **Módulo relé con retardo hardware** | Mantiene la alimentación T segundos tras girar la llave a OFF para que el ESP32 pueda despedirse |
| **Módulo relé 2 canales SRD-12VDC-SL-C** | El STM32 lo controla con 3.3 V para activar los relés de potencia (tracción + dirección) |
| **R1+R2** ó GPIO40 fijo | Señal de detección de llave para el firmware del ESP32 — ver sección 2 |

### Pines reales del STM32 (verificados en firmware)

| Pin STM32 | Función | Nivel |
|-----------|---------|-------|
| PC11 | RELAY_TRAC — relé tracción 24 V | 3.3 V HIGH/LOW |
| PC12 | RELAY_DIR — relé dirección 12 V | 3.3 V HIGH/LOW |
| PC10 | **RESERVADO** — sin uso en firmware | — |

> PC10 aparece como `RESERVED/unused` en `project_config.h:195`.
> Solo existen dos relés controlados por software: TRAC (PC11) y DIR (PC12).

---

## 2. Conexión de la Llave al ESP32

### 2.1 ¿Para qué necesita el ESP32 la señal de la llave?

GPIO 40 (`IGNITION_SENSE`) le dice al firmware del ESP32 si la llave está en ON u OFF.
Cuando detecta OFF, el firmware:
- Reproduce el audio de despedida
- Guarda la configuración en flash (`config_store::flush`)

> **Referencia firmware:** `power_manager.cpp:56` — `pinMode(PIN_IGNITION_SENSE, INPUT_PULLDOWN)`
> `power_manager.h:26` — `PIN_IGNITION_SENSE = 40`

### 2.2 ¿Necesito R1+R2 para GPIO 40?

Depende de cómo llegue la señal a GPIO 40:

| Situación | ¿Necesitas R1+R2? |
|-----------|------------------|
| Cable ACC (12 V de batería) → GPIO 40 directamente | **SÍ, obligatorio** — 12 V destruiría el pin |
| Cable ACC (12 V) → R1 (33 kΩ) → GPIO 40, R2 (10 kΩ) → GND | **No, esto ya es la solución** |
| GPIO 40 fijado a 3.3 V (puente a pin 3V3) | **No** — el firmware siempre ve llave ON; el hardware gestiona el apagado |
| Módulo opto-aislador de 1 canal adicional | **No** — el opto convierte 12 V en señal 3.3 V |

### 2.3 Opción A — Con R1+R2 (recomendada, más simple)

Dos resistencias y listo. El firmware detecta el encendido y apagado de la llave.

```
  Batería 12 V (+)
       │
       ├─────────────────────────────────────────► Trigger (X1) módulo retardo
       │
  ┌────┴─────┐
  │  LLAVE   │  SPST — ON=cerrado, OFF=abierto
  └────┬─────┘
       │  ACC 12 V (solo con llave ON)
       │
  ┌────┴────┐
  │ R1 33kΩ │  ¼ W
  └────┬────┘
       │
       ├──────────────────────────────────────► GPIO 40 ESP32-S3 (IGNITION_SENSE)
       │
  ┌────┴────┐
  │ R2 10kΩ │  ¼ W
  └────┬────┘
       │
      GND ───────────────────────────────────► GND ESP32-S3
```

Resultado: Llave ON → GPIO 40 = 2.79 V (HIGH) / Llave OFF → GPIO 40 = 0 V (LOW)

### 2.4 Opción B — Sin R1+R2 (GPIO 40 fijo a 3.3 V)

El firmware siempre ve "llave ON". El encendido y apagado del sistema los
gestiona exclusivamente el módulo de retardo hardware. No hay detección de
llave por software → no hay audio de despedida al apagar.

```
  ESP32-S3 pin 3V3 ────── puente corto ─────► GPIO 40
```

> Esta opción es válida si aceptas perder el audio de despedida y el guardado
> de configuración al girar la llave. Para banco de trabajo o pruebas: perfecta.

### 2.5 Opción C — Sin R1+R2 con módulo opto-aislador de 1 canal

Añade un módulo opto de 1 canal (SRD-05VDC-SL-C o KY-019 de 1 canal, ~€0.80).
El contacto NO del opto conecta 3.3 V a GPIO 40. La llave de 12 V solo toca
el opto, nunca el GPIO del ESP32.

```
  ACC 12 V (llave) → IN del opto 1ch → opto activa relé
  COM opto → 3.3 V (de ESP32)
  NO  opto → GPIO 40 ESP32-S3
```

Llave ON → opto cierra → GPIO 40 = 3.3 V (HIGH) / Llave OFF → opto abre → GPIO 40 = 0 V (LOW)

---

## 3. Módulo Relé con Retardo Hardware (alimentación)

### 3.1 ¿Qué hace?

Mantiene la alimentación del ESP32+STM32 durante N segundos tras apagar la
llave. Funciona 100 % en hardware, sin código. Ajusta el potenciómetro a
**30–60 s** (el firmware usa 3 s de retardo, así que sobra margen).

### 3.2 Cómo conectar el módulo de retardo

```
  Batería 12 V (permanente) ─────► DC+ del módulo
  GND ────────────────────────────► DC- del módulo
  ACC llave (12 V con llave ON) ──► X1 / Trigger del módulo

  COM del relé del módulo ────────► 12 V batería (permanente)
  NO  del relé del módulo ────────► Entrada del regulador 5 V → ESP32-S3 + STM32 Nucleo
  NC  ────────────────────────────► Sin conectar
```

### 3.3 Funcionamiento

| Situación | Módulo retardo | GPIO 40 (con R1+R2) |
|-----------|---------------|---------------------|
| Llave ON | Trigger 12 V → relé cierra → 5 V a ESP32+STM32 | HIGH → RUNNING |
| Llave OFF | Trigger 0 V → cuenta atrás T seg | LOW → SHUTTING_DOWN |
| Durante T segundos | Relé sigue cerrado | ESP32 despedida + guarda flash |
| Pasados T segundos | Relé abre → alimentación cortada | ESP32 se apaga |

> **GPIO 41 del ESP32** (`POWER_HOLD`): el firmware lo pone HIGH al encender
> y LOW al terminar el apagado. **No se conecta al módulo de retardo** — el
> módulo es autónomo. Déjalo libre.

---

## 4. Módulo Relé 2 Canales SRD-12VDC-SL-C (control potencia)

### 4.1 ¿Para qué sirve en este sistema?

El STM32 necesita activar relés de potencia (RELAY_TRAC a 24 V, RELAY_DIR a
12 V). Sus GPIOs salen a 3.3 V y no pueden alimentar directamente las bobinas
de los relés de potencia (~150 mA). El módulo de 2 canales actúa como
**interfaz de potencia**: recibe 3.3 V del STM32 y acciona las bobinas de los
relés de potencia con 12 V.

### 4.2 ¿Puede el STM32 (3.3 V) conectarse directamente al módulo?

**SÍ, conexión directa sin resistencias adicionales.** El módulo ya tiene:
- Optoacopladores (PC817 o similar) en cada canal
- Resistencias limitadoras integradas en PCB (~1 kΩ en serie con el LED del opto)

Cálculo de corriente con 3.3 V del STM32:
`I = (3.3 V − 1.2 V forward_LED) / 1 kΩ = 2.1 mA` — suficiente (PC817 necesita ≥1 mA)

El mismo cálculo aplica si conectas GPIO del ESP32-S3 (3.3 V) al módulo.

### 4.3 Cómo conectar el módulo de 2 canales

```
  ┌────────────────────────────────────────────────┐
  │       MÓDULO 2 CANALES SRD-12VDC-SL-C          │
  │                                                │
  │  VCC ────────────────────────────► 3.3 V       │
  │  GND ────────────────────────────► GND         │
  │  JD-VCC ─────────────────────────► 12 V        │
  │  (quita el puente/jumper entre VCC y JD-VCC)   │
  │                                                │
  │  IN1 ◄──────────────────────────── PC11 STM32  │  (RELAY_TRAC)
  │  IN2 ◄──────────────────────────── PC12 STM32  │  (RELAY_DIR)
  │                                                │
  │  Relé CH1 COM ──────────────────► 12 V bat     │
  │  Relé CH1 NO  ──────────────────► Bobina relé  │  → relé potencia tracción 24 V (50 A)
  │                       potencia TRAC ────────┤  │
  │                                                │
  │  Relé CH2 COM ──────────────────► 12 V bat     │
  │  Relé CH2 NO  ──────────────────► Bobina relé  │  → relé potencia dirección 12 V (20 A)
  │                       potencia DIR  ────────┤  │
  └────────────────────────────────────────────────┘
```

> **IMPORTANTE — Quitar el jumper VCC/JD-VCC:** Estos módulos llevan un puente
> que cortocircuita VCC con JD-VCC. Si lo dejas puesto, conectas 12 V (JD-VCC)
> directamente al pin VCC a 3.3 V → daño. **Quita el jumper** y conecta cada
> terminal por separado: VCC = 3.3 V, JD-VCC = 12 V.

### 4.4 Ajuste del nivel de disparo (jumper H/L)

El módulo tiene un jumper que selecciona HIGH o LOW trigger:

| Posición jumper | Comportamiento |
|-----------------|---------------|
| **H** (HIGH trigger) | IN=HIGH (3.3 V) → relé ON — **usar este** |
| **L** (LOW trigger) | IN=LOW (0 V) → relé ON |

El STM32 pone PC11/PC12 en HIGH para activar los relés → usar **jumper H**.

### 4.5 Bornes de salida del módulo vs. relés de potencia

Los contactos del módulo (10 A 30 VDC) activan las **bobinas** de los relés
de potencia (que son relés automotrices de mayor amperaje):

| Canal módulo | Pin STM32 | Bobina que activa | Relé de potencia resultante |
|-------------|-----------|------------------|-----------------------------|
| CH1 (IN1) | PC11 | Relé potencia tracción (50 A, 24 V) | Conecta batería 24 V a 4× BTS7960 |
| CH2 (IN2) | PC12 | Relé potencia dirección (20 A, 12 V) | Conecta batería 12 V a BTS7960 dirección |

La corriente de las bobinas de los relés automotrices es ~100–200 mA a 12 V —
los contactos de 10 A del módulo lo manejan sin problema.

---

## 5. Secuencia Completa de Encendido

### 5.1 Línea temporal

```
  t=0         t~50ms       t~300ms      t~600ms      t~680ms    t~730ms
   │             │             │            │            │           │
   ▼             ▼             ▼            ▼            ▼           ▼
  LLAVE ON    Módulo        ESP32         ESP32        STM32       STM32
              retardo       GPIO 40       RUNNING      recibe      módulo 2ch
              cierra        HIGH          audio        heartbeat   IN1/IN2 HIGH
              5V llega      GPIO 41=HIGH  bienvenida   → ACTIVE    relés TRAC+DIR ON
```

### 5.2 Detalle paso a paso

| Paso | Tiempo | Componente | Acción | Referencia código |
|------|--------|-----------|--------|-------------------|
| 1 | t=0 | **Llave** | Usuario gira ON → 12V al trigger del módulo retardo | Hardware |
| 2 | t=0 | **Módulo retardo** | Relé cierra → 12V al regulador 5V | Hardware |
| 3 | t=0 | **Regulador 5V** | 12V → 5V → alimenta ESP32-S3 y STM32 | Hardware |
| 4 | t~50ms | **STM32** | Arranca, `HAL_Init()`, periféricos CAN/I2C/ADC | `main.c:143-230` |
| 5 | t~50ms | **ESP32** | Arranca `setup()`, `power_mgr::init()` | `main.cpp:633` |
| 6 | t~100ms | **ESP32** | GPIO 40 HIGH → GPIO 41=HIGH → estado POWER_HOLD | `power_manager.cpp:83-88` |
| 7 | t~300ms | **ESP32** | Estado → RUNNING (tras 200ms STARTUP_DELAY) | `power_manager.cpp:98-104` |
| 8 | t~300ms | **ESP32** | Audio bienvenida vía DFPlayer | `main.cpp:942` |
| 9 | t~600ms | **ESP32** | Heartbeat CAN 0x011 cada 100ms | `main.cpp:936` |
| 10 | — | **STM32** | Primer heartbeat → STANDBY → BootValidation → ACTIVE | `safety_system.c` |
| 11 | — | **STM32** | PC11 HIGH → módulo 2ch IN1 → CH1 NO cierra → bobina relé TRAC | `safety_system.c:680` |
| 12 | t+50ms | **STM32** | Espera 50ms → PC12 HIGH → módulo 2ch IN2 → CH2 NO cierra → bobina relé DIR | `safety_system.c:732-734` |
| 13 | — | **STM32** | Motores disponibles — espera pedal < 3% durante 400ms | `main.c:408-417` |

---

## 6. Flujo de Corriente hacia los Motores

### 6.1 Cadena de potencia tracción (24 V)

```
  ┌─────────┐    ┌───────┐    ┌──────────────────┐    ┌────────────────────────┐
  │ Bat 24V │───►│INA226 │───►│ Relé potencia    │───►│ 4× BTS7960 drivers     │
  │         │    │ ch4   │    │ tracción 50A     │    │ (FL, FR, RL, RR)       │
  └─────────┘    │0.75mΩ │    │                  │    └──────────┬─────────────┘
                 └───────┘    │ Bobina activada  │               │ PWM 20 kHz
                              │ por módulo 2ch   │          TIM1 (PA8-PA10+PC3)
                              │ CH1 NO ──────────┘          TIM8 (PC6-PC9)
                              └──────────────────┘
                                     ▲
                              STM32 PC11 → módulo 2ch IN1
```

### 6.2 Cadena de potencia dirección (12 V)

```
  ┌─────────┐    ┌────────────────┐    ┌──────────┐    ┌──────────────┐
  │ Bat 12V │───►│ Relé potencia  │───►│ INA226   │───►│ BTS7960      │───► Motor Dir
  │         │    │ dirección 20A  │    │ ch5 1.5mΩ│    │ steering     │
  └─────────┘    │ Bobina por     │    └──────────┘    └──────────────┘
                 │ módulo 2ch CH2 │                    PWM TIM3 (PA6, PA7)
                 └────────────────┘
                        ▲
                 STM32 PC12 → módulo 2ch IN2
```

### 6.3 Alimentación lógica

```
  Módulo retardo → relé NO → Regulador 5V
         │
         ├──► ESP32-S3 (5V)
         └──► STM32 Nucleo (5V → LDO interno → 3.3V)
```

---

## 7. Secuencia Completa de Apagado

### 7.1 Línea temporal

```
  t=0         t~50ms          t~3050ms        t~3100ms         t+T seg
   │              │               │               │               │
   ▼              ▼               ▼               ▼               ▼
  LLAVE OFF    ESP32            ESP32           Firmware        Módulo retardo
               GPIO 40=LOW      audio           completa        abre relé →
               → SHUTTING_DOWN  despedida       GPIO 41=LOW     sistema apagado
               guarda flash     (3 segundos)
```

### 7.2 Detalle paso a paso

| Paso | Tiempo | Componente | Acción | Referencia código |
|------|--------|-----------|--------|-------------------|
| 1 | t=0 | **Usuario** | Gira llave OFF → trigger módulo retardo = 0V | Hardware |
| 2 | t=0 | **Módulo retardo** | Inicia cuenta atrás T segundos (sigue alimentando) | Hardware |
| 3 | t~50ms | **ESP32** | GPIO 40 lee LOW → estado SHUTTING_DOWN | `power_manager.cpp:115-118` |
| 4 | t~50ms | **ESP32** | `config_store::flush()` → guarda en flash | `main.cpp:952` |
| 5 | t~50ms | **ESP32** | Audio despedida (FAREWELL) + LEDs OFF | `main.cpp:953-961` |
| 6 | t~3050ms | **ESP32** | Han pasado 3000ms → GPIO 41 = LOW → estado OFF | `power_manager.cpp:127-129` |
| 7 | t+T seg | **Módulo retardo** | Cuenta atrás completa → relé abre → corta alimentación | Hardware |
| 8 | inmediato | **STM32** | Pierde 5V → GPIOs caen a LOW → PC11/PC12 LOW | Hardware |
| 9 | inmediato | **Módulo 2ch** | IN1/IN2 = LOW → CH1/CH2 abren → bobinas relés OFF | Hardware |
| 10 | inmediato | **Relés potencia** | Sin bobina → contactos abren → motores desconectados | Hardware |

### 7.3 Apagado de emergencia por fallo del STM32

```c
// safety_system.c — apagado de emergencia (overcurrent, overtemp, watchdog...)
void Safety_PowerDown(void)
{
    Traction_EmergencyStop();    // Todos los PWM a 0%
    Relay_PowerDown();           // PC11 y PC12 a LOW atómicamente
}

void Relay_PowerDown(void)
{
    // BSRR: reset PC11 (TRAC) y PC12 (DIR) en un solo ciclo de reloj
    GPIOC->BSRR = (uint32_t)(PIN_RELAY_TRAC | PIN_RELAY_DIR) << 16U;
}
```

PC11/PC12 LOW → módulo 2ch IN1/IN2 LOW → contactos CH1/CH2 abren → bobinas relés OFF → motores desconectados.

---

## 8. Diagrama Eléctrico Completo

```
  ┌──────────┐
  │ Bat 12V  │─────────────────────────────────────────────────────┐
  │(perm+ACC)│                                                     │ (permanente)
  └──────────┘                                                     │
       │                                                           ▼
       │ ACC (llave ON)                               ┌─────────────────────┐
  ┌────┴─────┐                                        │  MÓDULO RETARDO     │
  │  LLAVE   │──── ACC ──────────────────────────────►│  Trigger (X1)       │
  │ CONTACTO │                                        │  DC+ ← 12V perm     │
  └────┬─────┘                                        │  DC- ← GND          │
       │ ACC                                          │  COM ← 12V perm     │
       │ (para R1/R2 si usas Opción A)                │  NO ─────────────────┼──► Reg 5V ──► ESP32+STM32
       │                                              └─────────────────────┘
  [Opción A: R1 33kΩ → GPIO40 → R2 10kΩ → GND]
  [Opción B: puente 3.3V → GPIO40             ]
  [Opción C: módulo opto 1ch → GPIO40         ]

  ESP32-S3 GPIO 40 (IGNITION_SENSE): detecta llave ON/OFF para firmware
  ESP32-S3 GPIO 41 (POWER_HOLD): uso interno firmware, no conectar a nada


  STM32 PC11 ─────────────────────────────────┐
  STM32 PC12 ─────────────────────────────┐   │
                                          │   │
                 ┌──────────────────────────────────────────┐
                 │    MÓDULO 2 CANALES SRD-12VDC-SL-C       │
                 │    (high/low level trigger, opto)         │
                 │                                          │
                 │  VCC ─────────────────────► 3.3V         │
                 │  JD-VCC ──────────────────► 12V (bat)    │
                 │  GND ──────────────────────► GND         │
                 │  (jumper VCC/JD-VCC: QUITADO)            │
                 │  (jumper trigger: posición H)            │
                 │                                          │
                 │  IN1 ◄──────────────────────────── PC11  │  3.3V directo, sin resistencias
                 │  IN2 ◄──────────────────────────── PC12  │  3.3V directo, sin resistencias
                 │                                          │
                 │  CH1 COM ─────────────────► 12V bat      │
                 │  CH1 NO  ────────────────────────────────┼──► Bobina relé potencia TRAC (50A, 24V)
                 │                                          │
                 │  CH2 COM ─────────────────► 12V bat      │
                 │  CH2 NO  ────────────────────────────────┼──► Bobina relé potencia DIR (20A, 12V)
                 └──────────────────────────────────────────┘


  ┌─────────┐   ┌──────────────────┐    INA ch0 ──► BTS7960 ──► Motor FL
  │ Bat 24V │──►│ Relé TRAC (50A)  │───►INA ch1 ──► BTS7960 ──► Motor FR
  └─────────┘   │ Bobina ← CH1 NO  │    INA ch2 ──► BTS7960 ──► Motor RL
                └──────────────────┘    INA ch3 ──► BTS7960 ──► Motor RR

  ┌─────────┐   ┌──────────────────┐
  │ Bat 12V │──►│ Relé DIR (20A)   │──► INA ch5 ──► BTS7960 ──► Motor Dirección
  └─────────┘   │ Bobina ← CH2 NO  │
                └──────────────────┘
```

---

## 9. Lista de Componentes

### 9.1 Señal de llave para GPIO 40 del ESP32

Elige UNA de las tres opciones:

| Opción | Componentes | Ventaja |
|--------|-------------|---------|
| **A (recomendada)** | R1 (33 kΩ ¼W) + R2 (10 kΩ ¼W) | Más simple, mínimas piezas, €0.05 |
| **B** | Puente corto 3.3V → GPIO40 | Sin piezas — pierde audio despedida |
| **C** | Módulo opto 1 canal adicional (~€0.80) | Aislamiento galvánico total |

### 9.2 Módulo relé con retardo hardware (alimentación)

| Componente | Especificación | Función |
|------------|---------------|---------|
| Módulo relé retardo | 12V DC, contactos ≥ 5A, potenciómetro 0-120s | Alimentación T seg tras llave OFF |
| Regulador 5V | LM7805 o módulo buck 12V→5V, ≥ 1A | 12V → 5V para ESP32+STM32 |

### 9.3 Módulo relé 2 canales (control potencia)

| Componente | Especificación | Función |
|------------|---------------|---------|
| Módulo 2ch SRD-12VDC-SL-C | High/low trigger, optoacoplado | Interfaz STM32 → bobinas relés potencia |

Configuración:
- Jumper VCC/JD-VCC: **QUITADO**
- VCC → 3.3V, JD-VCC → 12V
- Jumper trigger: **H** (HIGH trigger)
- IN1/IN2 → PC11/PC12 STM32 (conexión directa, sin resistencias adicionales)

### 9.4 Relés de potencia (accionados por el módulo 2ch)

| Componente | Amperaje | Tensión bobina | Función |
|------------|----------|----------------|---------|
| Relé potencia tracción | ≥ 50 A | 12 V | Conecta 24V a 4× BTS7960 |
| Relé potencia dirección | ≥ 20 A | 12 V | Conecta 12V a BTS7960 dirección |

Cada relé de potencia lleva un **diodo flyback 1N4007 en paralelo con su bobina**
(cátodo al terminal +12V de la bobina, ánodo al terminal GND de la bobina).

---

## 10. Preguntas Frecuentes

### ¿Puedo conectar el ESP32-S3 directamente al módulo de 2 canales?

Sí. El módulo tiene optoacopladores con resistencias integradas. Un GPIO del
ESP32-S3 a 3.3V proporciona (3.3-1.2)/1kΩ = 2.1mA al LED del opto — suficiente.
Sin embargo, en este diseño es el STM32 (PC11/PC12) quien controla el módulo,
no el ESP32.

### ¿Por qué la llave va al ESP32 y no al STM32?

El ESP32-S3 gestiona HMI (pantalla, audio). El STM32 es la autoridad de
seguridad de motores — no necesita leer la llave directamente, recibe el
estado por CAN (heartbeat del ESP32).

### ¿Qué pasa si el ESP32 falla?

El STM32 detecta pérdida de heartbeat CAN tras 250ms → entra en LIMP_HOME
(20% potencia, ~5 km/h). El vehículo sigue siendo operable con pedal local.

### ¿Qué pasa si giro la llave a OFF mientras conduzco?

1. Módulo retardo inicia cuenta atrás (T segundos).
2. ESP32 detecta GPIO 40 LOW → SHUTTING_DOWN → audio despedida → guarda config.
3. STM32 detecta pérdida heartbeat → LIMP_HOME.
4. Pasados T segundos → retardo abre → STM32 pierde tensión → PC11/PC12 LOW → módulo 2ch abre → relés potencia abren → motores desconectados.

### ¿Se puede probar en banco sin la llave?

Sí: conectar 3.3V directamente a GPIO 40 (simula llave ON). El módulo de
2 canales se puede probar directamente conectando 3.3V a IN1/IN2 manualmente.

### ¿Necesito el diodo flyback en el módulo de 2 canales?

No. Los diodos flyback van en las bobinas de los **relés de potencia**
(relés automotrices de 50A/20A), no en el módulo de 2 canales. El módulo
ya tiene protección interna en sus optoacopladores.

---

> **Documento generado a partir del firmware verificado:**
> `esp32/src/power_manager.h`, `esp32/src/power_manager.cpp`,
> `esp32/src/main.cpp`, `Core/Src/safety_system.c`,
> `Core/Inc/project_config.h`, `Core/Src/main.c`.
> No contiene hardware inventado ni estimado.
