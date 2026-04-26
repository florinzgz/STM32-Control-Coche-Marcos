# Llave de Contacto — Encendido y Apagado del Sistema

**Documento de referencia para taller — Cómo conectar los cables**

> **Fuente verificada:** `esp32/src/power_manager.h`, `esp32/src/power_manager.cpp`,
> `esp32/src/main.cpp`, `Core/Src/safety_system.c`, `Core/Inc/project_config.h`,
> `Core/Src/main.c`, `Core/Src/can_handler.c`.
> No se ha inventado ningún componente ni conexión.
>
> **Revisión actual:** Cuatro módulos de relé en total:
> - **Módulo retardo** (12 V): mantiene alimentación al apagar la llave
> - **Módulo 2ch SRD-12VDC-SL-C**: STM32 controla relés de potencia (tracción + dirección)
> - **Módulo 2ch SRD-05VDC-SL-C**: STM32 controla relés alimentación tiras LED WS2812B
>
> No hacen falta transistores, resistencias adicionales ni circuitos discretos.

---

## Índice

1. [Resumen General](#1-resumen-general)
2. [Conexión de la Llave al ESP32](#2-conexión-de-la-llave-al-esp32)
3. [Módulo Relé con Retardo Hardware (alimentación)](#3-módulo-relé-con-retardo-hardware-alimentación)
4. [Módulo Relé 2 Canales 12 V — Relés de Potencia](#4-módulo-relé-2-canales-12-v--relés-de-potencia)
5. [Módulo Relé 2 Canales 5 V — Alimentación Tiras LED](#5-módulo-relé-2-canales-5-v--alimentación-tiras-led)
6. [Secuencia Completa de Encendido](#6-secuencia-completa-de-encendido)
7. [Flujo de Corriente hacia los Motores y LEDs](#7-flujo-de-corriente-hacia-los-motores-y-leds)
8. [Secuencia Completa de Apagado](#8-secuencia-completa-de-apagado)
9. [Diagrama Eléctrico Completo](#9-diagrama-eléctrico-completo)
10. [Lista de Componentes](#10-lista-de-componentes)
11. [Preguntas Frecuentes](#11-preguntas-frecuentes)

---

## 1. Resumen General

El sistema usa cuatro módulos de relé que trabajan juntos:

| Módulo | Tipo | Función |
|--------|------|---------|
| **Módulo retardo hardware** | 12 V, temporizador | Mantiene alimentación T seg tras llave OFF |
| **Módulo 2ch SRD-12VDC-SL-C** | 12 V bobinas, opto | STM32 PC11/PC12 → relés potencia tracción+dirección |
| **Módulo 2ch SRD-05VDC-SL-C** | 5 V bobinas, opto | STM32 PB10/PB11 → alimentación tiras LED WS2812B |

> El módulo de 5 V es igual al de 12 V de la foto — mismo PCB, misma disposición
> de pines — pero con relés marcados SRD-**05**VDC-SL-C en lugar de SRD-**12**VDC-SL-C.

### Pines STM32 verificados en firmware

| Pin STM32 | Puerto | Función | Tensión control | Referencia código |
|-----------|--------|---------|-----------------|-------------------|
| PC11 | GPIOC | RELAY_TRAC — relé tracción 24 V | 3.3 V HIGH/LOW | `project_config.h:199` |
| PC12 | GPIOC | RELAY_DIR — relé dirección 12 V | 3.3 V HIGH/LOW | `project_config.h:200` |
| PB10 | GPIOB | RELAY_LED — alimentación tira LED frontal (28 LEDs WS2812B) | 3.3 V HIGH/LOW | `project_config.h:210` |
| PB11 | GPIOB | RELAY_LED_REAR — alimentación tira LED trasera (16 LEDs WS2812B) | 3.3 V HIGH/LOW | `project_config.h:211` |
| PC10 | GPIOC | **RESERVADO** — sin uso en firmware | — | `project_config.h:195` |

---

## 2. Conexión de la Llave al ESP32

### 2.1 ¿Para qué necesita el ESP32 la señal de la llave?

GPIO 40 (`IGNITION_SENSE`) informa al firmware del ESP32 si la llave está ON u OFF.
Cuando detecta OFF, el firmware reproduce el audio de despedida y guarda config en flash.

> `power_manager.cpp` — `pinMode(PIN_IGNITION_SENSE, INPUT_PULLUP)`
> `power_manager.h`   — `PIN_IGNITION_SENSE = 40`

### 2.2 Opción implementada — Canal A7 de la placa PC817 de 8 canales

Se utiliza un canal libre (A7) de la placa PC817 de 8 canales que ya está montada en el
sistema para los sensores de rueda. **No se necesita ningún módulo adicional.**

La señal de la llave (+12 V ACC) pasa por el optoacoplador → el colector del PC817 se
conecta a GPIO 40. La lógica es **INVERTIDA** respecto a un divisor resistivo directo:

| Llave | LED PC817 | Fototransistor | GPIO 40 | Estado firmware |
|-------|-----------|----------------|---------|-----------------|
| **ON** | Conduce (10.8 mA) | Saturado → colector a GND | **LOW** | → RUNNING |
| **OFF** | Apagado | Abierto | **HIGH** (pull-up externo 10 kΩ + INPUT_PULLUP ~45 kΩ) | → SHUTTING_DOWN |

> ⚠️ **La placa PC817 usada NO tiene pull-up onboard** (verificado con polímetro: circuito abierto).
> Son obligatorios dos pull-ups: **10 kΩ externo** entre GPIO 40 y 3.3 V (soldar en PCB) +
> **INPUT_PULLUP** en firmware (ya configurado en `power_manager.cpp` — red de seguridad ~45 kΩ).

### 2.3 Cableado GPIO 40 via PC817 (canal A7)

```
  +12V ACC (llave ON) ──[1 A]── 1 kΩ ──→ IN+ (ánodo LED)
  GND vehículo        ─────────────────→ IN- (cátodo LED)

                                          3.3V ──[10 kΩ ext.]──┐
                                          OUT (colector) ───────┤──→ GPIO 40
                                          GND (3.3V side) ──────────→ GND ESP32
```

- **Resistencia serie LED: 1 kΩ** (no 330 Ω):
  `I_LED = (12 V − 1.2 V) / 1 kΩ = 10.8 mA` ← seguro (<20 mA nominal PC817, ciclo de trabajo ~100%)
- **Pull-up externo 10 kΩ ¼ W**: soldar entre GPIO 40 y pin 3.3 V del ESP32 — **obligatorio**

## 3. Módulo Relé con Retardo Hardware (alimentación)

Mantiene la alimentación 5 V del ESP32+STM32 durante N segundos tras girar la llave OFF.
Funciona 100 % en hardware, sin código.

**Ajusta el potenciómetro a 30–60 s** (firmware usa 3 s de retardo interno).

### Conexionado

```
  Batería 12 V permanente ──────► DC+ del módulo retardo
  GND ──────────────────────────► DC- del módulo retardo
  ACC llave (12V, llave ON) ────► X1 / Trigger del módulo

  COM del relé ─────────────────► 12 V batería (permanente)
  NO  del relé ─────────────────► Entrada regulador 5 V → ESP32-S3 + STM32 Nucleo
  NC  ──────────────────────────► Sin conectar
```

| Situación | Módulo retardo | GPIO 40 (via PC817) |
|-----------|---------------|---------------------|
| Llave ON | Trigger 12 V → relé cierra → 5V llega a ESP32+STM32 | LOW → RUNNING |
| Llave OFF | Trigger 0 V → cuenta atrás T seg | HIGH → SHUTTING_DOWN |
| Durante T seg | Relé sigue cerrado | ESP32 despedida + guarda flash |
| Pasados T seg | Relé abre → alimentación cortada | ESP32 se apaga |

> **GPIO 41 ESP32 (POWER_HOLD):** uso interno firmware. **No conectar al módulo retardo.**

---

## 4. Módulo Relé 2 Canales 12 V — Relés de Potencia

### 4.1 ¿Para qué sirve?

Permite que los 3.3 V del STM32 accionen las bobinas de los relés de potencia
(tracción 24 V, 50 A y dirección 12 V, 20 A).

### 4.2 Conexión directa sin resistencias adicionales

El módulo ya tiene optoacopladores con resistencias integradas (~1 kΩ).
Con 3.3 V del STM32: `I = (3.3 − 1.2) / 1000 = 2.1 mA` — suficiente (PC817 ≥ 1 mA).

### 4.3 Cableado del módulo 12 V

```
  MÓDULO 2 CANALES SRD-12VDC-SL-C
  ─────────────────────────────────────
  VCC    → 3.3 V   (del regulador 3.3V del STM32 Nucleo)
  JD-VCC → 12 V    (batería — QUITA el jumper entre VCC y JD-VCC)
  GND    → GND

  Jumper trigger → posición H  (HIGH = relé ON)

  IN1 ←── PC11 STM32  (RELAY_TRAC, 3.3V directo)
  IN2 ←── PC12 STM32  (RELAY_DIR,  3.3V directo)

  CH1 COM → 12 V batería
  CH1 NO  → Bobina relé potencia tracción 50A/24V
  CH2 COM → 12 V batería
  CH2 NO  → Bobina relé potencia dirección 20A/12V
```

> ⚠️ **IMPORTANTE — Quita el jumper VCC/JD-VCC** antes de conectar.
> Si lo dejas puesto, los 12 V entran al pin VCC de 3.3 V → daño.

### 4.4 ¿Qué activan esos relés de potencia?

| Canal | STM32 | Relé de potencia | Carga |
|-------|-------|-----------------|-------|
| CH1 | PC11 HIGH | Contactor tracción 50A | Batería 24V → 4× BTS7960 (motores FL/FR/RL/RR) |
| CH2 | PC12 HIGH | Contactor dirección 20A | Batería 12V → BTS7960 motor volante |

Cada relé de potencia (contactos principales) lleva un **diodo flyback 1N4007**
en paralelo con su bobina (cátodo al +12V, ánodo a GND de bobina).

---

## 5. Módulo Relé 2 Canales 5 V — Alimentación Tiras LED

### 5.1 ¿Para qué sirve?

El STM32 controla la alimentación eléctrica de dos tiras LED WS2812B mediante
dos relés independientes. El ESP32 envía órdenes al STM32 via CAN 0x120;
el STM32 activa/desactiva PB10 y PB11.

| Canal | STM32 | Tira LED | LEDs |
|-------|-------|----------|------|
| CH1 | PB10 | Frontal (datos: ESP32) | 28 WS2812B |
| CH2 | PB11 | Trasera (datos: ESP32) | 16 WS2812B |

> **CAN 0x120** (ESP32 → STM32): Byte 0 = frontal (0=OFF / 1=ON), Byte 1 = trasera.
> Referencia: `can_handler.c:1477-1494`.

### 5.2 Conexión directa sin resistencias adicionales

Mismo razonamiento que el módulo 12 V: el PC817 interno del módulo 5 V necesita
≥ 1 mA. Con 3.3 V del STM32: `I = (3.3 − 1.2) / 1000 = 2.1 mA` → OK.

**Conexión directa PB10/PB11 → IN1/IN2. No hace falta ninguna resistencia.**

### 5.3 Cableado del módulo 5 V

```
  MÓDULO 2 CANALES SRD-05VDC-SL-C
  ─────────────────────────────────────
  VCC    → 3.3 V   (del STM32 Nucleo — mismo raíl que el módulo 12V)
  JD-VCC → 5 V     (del regulador 5V — QUITA el jumper entre VCC y JD-VCC)
  GND    → GND

  Jumper trigger → posición H  (HIGH = relé ON)

  IN1 ←── PB10 STM32  (RELAY_LED,      3.3V directo)
  IN2 ←── PB11 STM32  (RELAY_LED_REAR, 3.3V directo)

  CH1 COM → 5 V (del regulador)
  CH1 NO  → + de la tira LED frontal  (28× WS2812B)
  CH1 NC  → Sin conectar

  CH2 COM → 5 V (del regulador)
  CH2 NO  → + de la tira LED trasera  (16× WS2812B)
  CH2 NC  → Sin conectar

  GND del módulo ─── GND tiras LED ─── GND ESP32 (dato WS2812B)
```

> ⚠️ **IMPORTANTE — Quita el jumper VCC/JD-VCC** antes de conectar.
> Si lo dejas puesto, los 5 V entran al pin VCC de 3.3 V → daño.

### 5.4 Corriente que consumen las tiras

| Tira | LEDs | Consumo máximo (todos blancos al 100%) |
|------|------|---------------------------------------|
| Frontal | 28× WS2812B | 28 × 60 mA = 1.68 A a 5 V |
| Trasera | 16× WS2812B | 16 × 60 mA = 0.96 A a 5 V |

Los contactos del módulo (10 A) soportan sin problema estas corrientes.
El regulador 5 V debe ser capaz de suministrar la corriente total de LEDs
+ ESP32+STM32: **usa un módulo buck 12V→5V de ≥ 3 A**.

---

## 6. Secuencia Completa de Encendido

| Paso | Tiempo | Componente | Acción | Referencia código |
|------|--------|-----------|--------|-------------------|
| 1 | t=0 | **Llave** | Usuario gira ON → 12V al trigger módulo retardo | Hardware |
| 2 | t=0 | **Módulo retardo** | Relé cierra → 12V al regulador 5V → ESP32+STM32 | Hardware |
| 3 | t~50ms | **STM32** | Arranca, `HAL_Init()`, configura GPIO PC11/PC12/PB10/PB11 → LOW | `main.c:744-747` |
| 4 | t~50ms | **ESP32** | Arranca `setup()`, `power_mgr::init()` | `main.cpp:633` |
| 5 | t~100ms | **ESP32** | GPIO 40 HIGH → GPIO 41=HIGH → estado POWER_HOLD | `power_manager.cpp:83-88` |
| 6 | t~300ms | **ESP32** | Estado RUNNING → audio bienvenida | `power_manager.cpp:98-104` |
| 7 | t~600ms | **ESP32** | Heartbeat CAN 0x011 cada 100 ms | `main.cpp:936` |
| 8 | — | **STM32** | Heartbeat → ACTIVE → PC11 HIGH | `safety_system.c:680` |
| 9 | t+50ms | **STM32** | PC12 HIGH (retardo 50ms) | `safety_system.c:732-734` |
| 10 | — | **STM32** | PC11/PC12 HIGH → módulo 12V IN1/IN2 → bobinas cierran → motores disponibles | Módulo 12V |
| 11 | — | **ESP32** | CAN 0x120 Byte0=1,Byte1=1 → STM32 activa PB10+PB11 | `can_handler.c:1477` |
| 12 | — | **STM32** | PB10/PB11 HIGH → módulo 5V IN1/IN2 → tiras LED alimentadas | Módulo 5V |

---

## 7. Flujo de Corriente hacia los Motores y LEDs

### 7.1 Tracción (24 V)

```
  Bat 24V → [INA226 ch4] → [Relé potencia 50A] → [4× BTS7960] → Motores FL/FR/RL/RR
                                  ↑
                 Bobina activada por módulo 2ch 12V CH1 (STM32 PC11)
```

### 7.2 Dirección / Volante (12 V)

```
  Bat 12V → [Relé potencia 20A] → [INA226 ch5] → [BTS7960 steering] → Motor volante
                    ↑
       Bobina activada por módulo 2ch 12V CH2 (STM32 PC12)
```

### 7.3 Tiras LED WS2812B (5 V)

```
  Reg 5V → [Módulo 2ch 5V CH1 NO] → + tira LED frontal (28 LEDs)
                                      - tira LED frontal → GND
                                      Dato WS2812B ← ESP32 GPIO (pin WS2812)

  Reg 5V → [Módulo 2ch 5V CH2 NO] → + tira LED trasera (16 LEDs)
                                      - tira LED trasera → GND
                                      Dato WS2812B ← ESP32 GPIO (pin WS2812)
```

### 7.4 Alimentación lógica

```
  Módulo retardo → Reg 5V buck (≥3A)
       ├──► ESP32-S3 (5V)
       ├──► STM32 Nucleo (5V → LDO interno → 3.3V)
       └──► JD-VCC módulo LED 5V + COM CH1/CH2 tiras
```

---

## 8. Secuencia Completa de Apagado

| Paso | Tiempo | Componente | Acción | Referencia código |
|------|--------|-----------|--------|-------------------|
| 1 | t=0 | **Usuario** | Gira llave OFF → trigger módulo retardo = 0V | Hardware |
| 2 | t=0 | **Módulo retardo** | Inicia cuenta atrás T seg | Hardware |
| 3 | t~50ms | **ESP32** | GPIO 40 LOW → SHUTTING_DOWN | `power_manager.cpp:115-118` |
| 4 | t~50ms | **ESP32→STM32** | CAN 0x130 SYSTEM_SHUTDOWN (DLC 0) → handshake pre-corte | `main.cpp` `sendSystemShutdown()` |
| 5 | t~50ms | **STM32** | `Safety_RequestShutdown()`: PWM=0, EN=LOW, relés TRAC+DIR OFF (BSRR atómico), `system_state=SAFE` | `safety_system.c` `Safety_RequestShutdown()` |
| 6 | t~150ms | **ESP32** | Pausa 100 ms tras 0x130 (ventana de reacción del STM32) | `main.cpp` SHUTTING_DOWN |
| 7 | t~150ms | **ESP32** | CAN 0x120 Byte0=0,Byte1=0 → STM32 apaga PB10+PB11 | `can_handler.c:1477` |
| 8 | t~150ms | **STM32** | PB10/PB11 LOW → módulo 5V abre → tiras LED sin alimentación | Módulo 5V |
| 9 | t~150ms | **ESP32** | `config_store::flush()` → flash / audio despedida | `main.cpp` SHUTTING_DOWN |
| 10 | t~3150ms | **ESP32** | 3000 ms (`SHUTDOWN_DELAY_MS`) cumplidos → GPIO 41=LOW → estado OFF | `power_manager.cpp:127-129` |
| 11 | t+T seg | **Módulo retardo** | Cuenta atrás completa → relé abre → 5V cortado | Hardware |
| 12 | inmediato | **STM32** | Pierde 5V → PC11/PC12 LOW → módulo 12V abre → bobinas relés potencia OFF | Hardware |
| 13 | inmediato | **Relés potencia** | Contactos abren → motores desconectados | Hardware |

> **Nota sobre el handshake CAN 0x130:** es aditivo y no rompe compatibilidad. Si la trama se pierde en el bus, la secuencia continúa exactamente como antes — el módulo retardo hardware sigue cortando la alimentación con el mismo timing. La única diferencia funcional es que el STM32 entra en estado seguro de forma determinista ~3 s antes del corte físico de potencia, en lugar de hacerlo pasivamente al perder los 5 V.

### Apagado de emergencia STM32 (fallo seguridad)

```c
// safety_system.c
void Relay_PowerDown(void)
{
    // Apaga PC11 (TRAC) y PC12 (DIR) en un solo ciclo de reloj
    GPIOC->BSRR = (uint32_t)(PIN_RELAY_TRAC | PIN_RELAY_DIR) << 16U;
}

// main.c / stm32g4xx_it.c
void LED_Relay_Emergency_Off(void)
{
    // Apaga PB10 (LED frontal) y PB11 (LED trasero)
    GPIOB->BSRR = (uint32_t)(PIN_RELAY_LED | PIN_RELAY_LED_REAR) << 16U;
}
```

---

## 9. Diagrama Eléctrico Completo

```
╔══════════════════════════════════════════════════════════════════════════════════╗
║                         SISTEMA COMPLETO — VISTA GENERAL                        ║
╚══════════════════════════════════════════════════════════════════════════════════╝

  ┌──────────┐
  │ Bat 12V  │──────────────────────────────────────────────── (permanente) ─────┐
  │(acc+perm)│                                                                   │
  └──────────┘                                                                   ▼
       │ ACC (solo llave ON)                                     ┌───────────────────────┐
  ┌────┴─────┐                                                   │  MÓDULO RETARDO 12V   │
  │  LLAVE   │──── ACC 12V ─────────────────────────────────────►│  X1 / Trigger         │
  │ CONTACTO │                                                   │  DC+ ← 12V permanente │
  └────┬─────┘                                                   │  DC- ← GND            │
       │                                                         │                       │
  [Opción A]                                                     │  COM ← 12V permanente │
  ├── R1 33kΩ ── GPIO40 ESP32-S3 ── R2 10kΩ ── GND             │  NO ──────────────────┼─► Reg 5V (≥3A)
  [Opción B: puente 3.3V → GPIO40]                              └───────────────────────┘        │
                                                                                                  │
                                    ┌─────────────────────────────────────────────────────────────┤
                                    │                                                             │
                                    ▼                                                             ▼
                             ┌────────────┐                                               ┌────────────┐
                             │  ESP32-S3  │                                               │ STM32      │
                             │  5V input  │◄────────────────────── 5V ──────────────────►│ Nucleo     │
                             │  GPIO 40   │ IGNITION_SENSE                                │ 5V → 3.3V  │
                             │  GPIO 41   │ POWER_HOLD (interno)                         └─────┬──────┘
                             └────────────┘                                                    │ 3.3V GPIOs
                                                                     ┌──────────────┬──────────┼───────────────┐
                                                                     │              │          │               │
                                                                    PC11           PC12       PB10            PB11
                                                                     │              │          │               │
                            ┌────────────────────────────┐          │              │          │               │
                            │  MÓDULO 2CH SRD-12VDC-SL-C │          │              │          │               │
                            │  VCC → 3.3V                │          │              │          │               │
                            │  JD-VCC → 12V bat          │          │              │          │               │
                            │  GND → GND                 │          │              │          │               │
                            │  Jumper VCC/JD: QUITADO    │          │              │          │               │
                            │  Jumper trigger: H         │          │              │          │               │
                            │                            │          │              │          │               │
                            │  IN1 ◄─────────────────────┼──────────┘              │          │               │
                            │  IN2 ◄─────────────────────┼─────────────────────────┘          │               │
                            │                            │                                     │               │
                            │  CH1 COM ← 12V bat         │                                     │               │
                            │  CH1 NO ──────────────────────────────────────────┐             │               │
                            │                            │                      │             │               │
                            │  CH2 COM ← 12V bat         │                      │             │               │
                            │  CH2 NO ──────────────────────────────────────────┼──────────┐  │               │
                            └────────────────────────────┘                      │          │  │               │
                                                                                │          │  │               │
                            ┌────────────────────────────┐                      │          │  │               │
                            │  MÓDULO 2CH SRD-05VDC-SL-C │                      │          │  │               │
                            │  VCC → 3.3V                │                      │          │  │               │
                            │  JD-VCC → 5V (reg)         │                      │          │  │               │
                            │  GND → GND                 │                      │          │  │               │
                            │  Jumper VCC/JD: QUITADO    │                      │          │  │               │
                            │  Jumper trigger: H         │                      │          │  │               │
                            │                            │                      │          │  │               │
                            │  IN1 ◄─────────────────────┼──────────────────────┼──────────┼──┘               │
                            │  IN2 ◄─────────────────────┼──────────────────────┼──────────┼──────────────────┘
                            │                            │                      │          │
                            │  CH1 COM ← 5V (reg)        │                      │          │
                            │  CH1 NO ──► +LED frontal   │                      │          │
                            │                            │                      │          │
                            │  CH2 COM ← 5V (reg)        │                      │          │
                            │  CH2 NO ──► +LED trasera   │                      │          │
                            └────────────────────────────┘                      │          │
                                                                                ▼          ▼
                                                                   ┌────────────────┐  ┌────────────────┐
                                                                   │ Relé potencia  │  │ Relé potencia  │
                                                                   │ TRACCIÓN 50A   │  │ DIRECCIÓN 20A  │
                                                                   │ + 1N4007 flyb. │  │ + 1N4007 flyb. │
                                                                   └───────┬────────┘  └───────┬────────┘
                                                                           │                   │
                                                                     Bat 24V                Bat 12V
                                                                           │                   │
                                                                    4×BTS7960            BTS7960
                                                                    FL/FR/RL/RR           volante


  TIRAS LED WS2812B:
  ┌──────────────────────────────────────────────────────────────┐
  │  Frontal (28 LEDs): + ← CH1 NO módulo 5V / GND / Dato ← ESP32
  │  Trasera (16 LEDs): + ← CH2 NO módulo 5V / GND / Dato ← ESP32
  └──────────────────────────────────────────────────────────────┘
```

---

## 10. Lista de Componentes

### 10.1 Señal de llave para GPIO 40 del ESP32 (Opción A)

| Componente | Valor | Función |
|------------|-------|---------|
| R1 | 33 kΩ, ¼ W | Resistencia superior del divisor |
| R2 | 10 kΩ, ¼ W | Resistencia inferior a GND |

### 10.2 Módulo relé con retardo hardware

| Componente | Especificación | Función |
|------------|---------------|---------|
| Módulo retardo | 12 V DC, contactos ≥ 5A, potenciómetro 0-120 s | Alimentación T seg tras llave OFF |
| Regulador 5V buck | 12V → 5V, **≥ 3 A** | Alimenta ESP32 + STM32 + tiras LED |

> El buck debe suministrar ESP32 (~0.5A) + STM32 (~0.2A) + LEDs (~2.64A max) = ~3.4A pico.
> Con LEDs al 30% de brillo (uso típico) ≈ 1.5A total — un módulo buck 3A vale.

### 10.3 Módulo relé 2 canales 12 V (control potencia)

| Componente | Especificación | Configuración |
|------------|---------------|---------------|
| Módulo 2ch SRD-12VDC-SL-C | High/low trigger, opto, 10A/250VAC | VCC=3.3V, JD-VCC=12V, jumper H, jumper VCC/JD-VCC QUITADO |
| IN1 | PC11 STM32 (3.3V directo) | Activa bobina relé tracción 24V |
| IN2 | PC12 STM32 (3.3V directo) | Activa bobina relé dirección 12V |

### 10.4 Módulo relé 2 canales 5 V (alimentación LEDs)

| Componente | Especificación | Configuración |
|------------|---------------|---------------|
| Módulo 2ch SRD-05VDC-SL-C | High/low trigger, opto, 10A/250VAC | VCC=3.3V, JD-VCC=5V, jumper H, jumper VCC/JD-VCC QUITADO |
| IN1 | PB10 STM32 (3.3V directo) | Conecta 5V a tira LED frontal (28× WS2812B) |
| IN2 | PB11 STM32 (3.3V directo) | Conecta 5V a tira LED trasera (16× WS2812B) |

### 10.5 Relés de potencia (accionados por módulo 12V)

| Componente | Amperaje | Tensión bobina | Función |
|------------|----------|----------------|---------|
| Relé potencia tracción | ≥ 50 A | 12 V | 24V → 4× BTS7960 motores tracción |
| Relé potencia dirección | ≥ 20 A | 12 V | 12V → BTS7960 motor volante |
| Diodo flyback | 1N4007 × 2 | — | En paralelo con cada bobina de relé de potencia |

### 10.6 Resumen: ¿qué necesitas comprar?

| # | Módulo/componente | Cantidad |
|---|-------------------|----------|
| 1 | Módulo relé con retardo 12V (temporizador potenciómetro) | 1 |
| 2 | Módulo relé 2 canales SRD-**12**VDC-SL-C (high/low trigger) | 1 |
| 3 | Módulo relé 2 canales SRD-**05**VDC-SL-C (high/low trigger) | 1 |
| 4 | Módulo buck 12V→5V ≥ 3A | 1 |
| 5 | Relé automotriz 50A 12V (tracción) | 1 |
| 6 | Relé automotriz 20A 12V (dirección) | 1 |
| 7 | Diodo 1N4007 | 2 |
| 8 | Resistencia 33 kΩ ¼W (R1) | 1 |
| 9 | Resistencia 10 kΩ ¼W (R2) | 1 |

---

## 11. Preguntas Frecuentes

### ¿Puedo conectar PB10/PB11 directo al módulo 5V sin resistencias?

**Sí.** El módulo SRD-05VDC-SL-C tiene optoacopladores con resistencias integradas.
Con 3.3V del STM32: 2.1mA al opto → suficiente. Sin resistencias adicionales.

### ¿Por qué el módulo LED es de 5V y el de potencia de 12V?

Los relés SRD-05VDC-SL-C tienen bobinas optimizadas para 5V (menor resistencia).
JD-VCC = 5V para activar las bobinas correctamente.
Los relés SRD-12VDC-SL-C necesitan 12V en JD-VCC para sus bobinas.

### ¿Qué pasa si dejo el jumper VCC/JD-VCC puesto?

En el módulo 5V: 5V entran al pin VCC de 3.3V → el STM32 puede dañarse.
En el módulo 12V: 12V entran al pin VCC de 3.3V → daño seguro.
**Siempre quita ese jumper y cablea VCC y JD-VCC por separado.**

### ¿Qué pasa si el ESP32 falla y no manda CAN 0x120?

El STM32 detecta pérdida de heartbeat CAN tras 250ms → LIMP_HOME. Los relés LED
(PB10/PB11) quedan en su último estado. En el apagado de emergencia el firmware
fuerza PB10/PB11 LOW vía BSRR (`stm32g4xx_it.c:61`).

### ¿Qué pasa si giro la llave a OFF mientras los LEDs están encendidos?

1. ESP32 detecta GPIO 40 LOW → envía CAN 0x120 con Byte0=0, Byte1=0 → STM32 apaga PB10/PB11 → LEDs sin alimentación.
2. Módulo retardo mantiene el sistema vivo durante T seg.
3. Pasados T seg → retardo abre → todo se apaga.

### ¿Qué corriente consume el circuito de señal de llave (R1+R2)?

`I = 12V / (33k + 10k) ≈ 0.28 mA` — negligible para los contactos de la llave.

---

> **Documento generado a partir del firmware verificado:**
> `esp32/src/power_manager.h`, `esp32/src/power_manager.cpp`,
> `esp32/src/main.cpp`, `Core/Src/safety_system.c`,
> `Core/Inc/project_config.h`, `Core/Src/main.c`, `Core/Src/can_handler.c`.
> No contiene hardware inventado ni estimado.
