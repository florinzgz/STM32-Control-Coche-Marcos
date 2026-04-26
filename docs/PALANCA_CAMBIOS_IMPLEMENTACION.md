# Palanca de Cambios — Implementación Eléctrica y por qué NO trabaja a 12 V

**Documento de referencia para taller — Circuito completo del selector de marchas**

> **Fuente:** Extraído directamente del firmware ESP32-S3 (`shifter_input.h`,
> `shifter_input.cpp`, `traction_switch.h`, `traction_switch.cpp`, `main.cpp`),
> STM32G474RE (`motor_control.h`, `motor_control.c`, `can_handler.c`), y de la
> documentación existente (`SENSOR_INTERFACE.md`, `HARDWARE_WIRING_MANUAL.md`,
> `TRACTION_GEAR_SHIFTER_AUDIT.md`).
> No se ha inventado ningún componente ni conexión.

---

## Índice

1. [Pregunta: ¿La palanca trabaja a 12 V?](#1-pregunta-la-palanca-trabaja-a-12-v)
2. [Arquitectura del Selector de Marchas](#2-arquitectura-del-selector-de-marchas)
3. [Circuito Eléctrico Completo — MCP23017](#3-circuito-eléctrico-completo--mcp23017)
4. [Cableado Físico Paso a Paso](#4-cableado-físico-paso-a-paso)
5. [Interruptor de Tracción 2WD/4WD](#5-interruptor-de-tracción-2wd4wd)
6. [Comunicación CAN — De la Palanca al Motor](#6-comunicación-can--de-la-palanca-al-motor)
7. [Escalado de Potencia por Marcha](#7-escalado-de-potencia-por-marcha)
8. [Protecciones y Seguridad](#8-protecciones-y-seguridad)
9. [¿Qué pasa si mi palanca original es de 12 V?](#9-qué-pasa-si-mi-palanca-original-es-de-12-v)
10. [Lista de Componentes](#10-lista-de-componentes)
11. [Preguntas Frecuentes](#11-preguntas-frecuentes)

---

## 1. Pregunta: ¿La palanca trabaja a 12 V?

**NO.** La palanca de cambios en este sistema **NO** trabaja a 12 V. Todo el
circuito del selector de marchas opera a **3.3 V**, que es la tensión del
ESP32-S3 y del chip expansor de I/O MCP23017.

### ¿Por qué la confusión con 12 V?

El vehículo usa baterías de **12 V** (dirección) y **24 V** (tracción). Sin
embargo, la palanca de cambios es un componente de **señal**, no de **potencia**.
No necesita manejar corrientes grandes ni tensiones altas.

```
  ┌────────────────────────────────────────────────────────────────┐
  │                CIRCUITO DE POTENCIA (12V / 24V)                │
  │                                                                │
  │  Batería 24V → Relés → BTS7960 → Motores tracción             │
  │  Batería 12V → Relé  → BTS7960 → Motor dirección              │
  │                                                                │
  │  ⚡ Aquí sí hay 12V y 24V con corrientes de decenas de amperios │
  └────────────────────────────────────────────────────────────────┘

  ┌────────────────────────────────────────────────────────────────┐
  │                CIRCUITO DE SEÑAL (3.3V)                        │
  │                                                                │
  │  Palanca de cambios → MCP23017 (3.3V) → I2C → ESP32 (3.3V)   │
  │  Interruptor 2WD/4WD → GPIO 15 ESP32 (3.3V)                   │
  │                                                                │
  │  📡 Aquí todo es 3.3V con corrientes de microamperios          │
  └────────────────────────────────────────────────────────────────┘
```

**La palanca solo cierra contactos a GND.** No transporta corriente de los
motores ni necesita manejar voltajes altos. Es un conjunto de interruptores
simples (contactos secos) que se leen con lógica digital a 3.3 V.

---

## 2. Arquitectura del Selector de Marchas

### 2.1 Resumen del sistema

```
  Palanca Física         MCP23017           ESP32-S3           CAN Bus         STM32G474RE
  (4 contactos+común)    (I2C, 3.3V)        (HMI)              (500 kbps)      (Control)
  ─────────────────      ──────────         ─────────          ──────────      ───────────

  ┌─ SW P  ──┐           GPA0 ◄──┐         GPIO 8 (SDA)
  ├─ SW D2 ──┤   común   GPA1 ◄──┤ I2C     GPIO 9 (SCL)       0x102           Traction_SetGear()
  ├─ SW D1 ──┼── GND →   GPA2 ◄──┼────►    shifter::          CMD_MODE   →    motor_control.c
  └─ SW R  ──┘           GPA3 ◄──┘         update()           byte 1          Aplica escalado
                          (pull-ups                            getGearRaw()    de potencia
                           100kΩ int.)                                          por marcha

  N (Neutral) NO TIENE CONTACTO FÍSICO — se detecta cuando ningún SW está cerrado
  (todos los GPA0-GPA3 quedan en HIGH por los pull-ups internos del MCP23017).
```

### 2.2 ¿Por qué un MCP23017 en vez de GPIOs directos?

| Criterio | GPIO directo | MCP23017 (elegido) |
|----------|-------------|-------------------|
| GPIOs ESP32 consumidos | 5 pines | 2 pines (I2C SDA/SCL) |
| Distancia de cable | Sensible a ruido | I2C robusto hasta 1 m |
| Protección | Cada pin necesita protección | Solo 2 líneas I2C |
| Escalabilidad | Limitada | 16 I/O disponibles (solo se usan 5) |

El ESP32-S3 tiene **restricciones severas de pines** porque la pantalla TFT,
el CAN bus, el sensor de obstáculos, el audio DFPlayer y la llave de contacto
ya consumen la mayoría de los GPIOs disponibles. Usar un MCP23017 ahorra 3
pines netos (5 − 2 = 3 pines liberados).

> **Referencia firmware:** `shifter_input.h` líneas 34-39 — Config struct

### 2.3 Roles de cada microcontrolador

| Función | Responsable | Referencia |
|---------|-------------|------------|
| **Leer posición física de la palanca** | ESP32-S3 (vía MCP23017 I2C) | `shifter_input.cpp:108-117` |
| **Decodificar posición (P/R/N/D1/D2)** | ESP32-S3 | `shifter_input.cpp:66-83` |
| **Enviar comando CAN con la marcha** | ESP32-S3 (CAN ID 0x102, byte 1) | `main.cpp`, `can_ids.h:30` |
| **Validar comando de marcha** | STM32G474RE | `can_handler.c:433-443` |
| **Aplicar escalado de potencia** | STM32G474RE | `motor_control.c:521-531` |
| **Controlar motores con PWM** | STM32G474RE (TIM1/TIM8/TIM3) | `motor_control.c` |

---

## 3. Circuito Eléctrico Completo — MCP23017

### 3.1 Esquema completo

```
  3.3V (del regulador ESP32 o fuente 3.3V)
    │
    ├────────────────────────────────────────────────────────────┐
    │                                                            │
  4.7kΩ    4.7kΩ                                                 │
    │        │                                                   │
    │        │      ┌─────────────────────────────┐              │
  GPIO 8 ──SDA──┬───┤  MCP23017 (DIP-28 o SO-28)  │              │
  GPIO 9 ──SCL──┘   │  Dirección I2C: 0x20         │              │
                     │                              │              │
                     │  VDD (pin 9)  ───────────────┼───── 3.3V   │
                     │  VSS (pin 10) ───────────────┼───── GND    │
                     │  RESET (pin 18) ─────────────┼───── 3.3V   │ (siempre activo)
                     │                              │              │
                     │  A0 (pin 15)  ───────────────┼───── GND    │
                     │  A1 (pin 16)  ───────────────┼───── GND    │ → dirección 0x20
                     │  A2 (pin 17)  ───────────────┼───── GND    │
                     │                              │              │
                     │  GPA0 (pin 21) ──────────────┼──── SW P  ──── COM ──── GND
                     │  GPA1 (pin 22) ──────────────┼──── SW D2 ──── COM ──── GND
                     │  GPA2 (pin 23) ──────────────┼──── SW D1 ──── COM ──── GND
                     │  GPA3 (pin 24) ──────────────┼──── SW R  ──── COM ──── GND
                     │  GPA4 (pin 25) ── (sin uso, dejar al aire)
                     │                              │
                     │  GPPU (reg 0x0C):            │
                     │  Pull-ups internos 100kΩ     │
                     │  habilitados en GPA0-GPA3    │
                     └──────────────────────────────┘

  Desacoplo: 100 nF cerámico entre VDD (pin 9) y VSS (pin 10)
```

### 3.2 Lógica de las señales

| Posición de la palanca | Cable de la palanca | Estado GPA | Lectura (invertida) | Valor CAN |
|------------------------|---------------------|------------|---------------------|-----------|
| **Park (P)** | azul + morado (SW P cerrado) | GPA0 = LOW | bit 0 = 1 | 0 |
| **Drive 2 (D2)** | azul + verde (SW D2 cerrado) | GPA1 = LOW | bit 1 = 1 | 4 |
| **Drive 1 (D1)** | azul + amarillo (SW D1 cerrado) | GPA2 = LOW | bit 2 = 1 | 3 |
| **Reverse (R)** | azul + blanco (SW R cerrado) | GPA3 = LOW | bit 3 = 1 | 1 |
| **Neutral (N)** | *(sin contacto físico)* | Todos HIGH | ningún bit | 2 |
| **Varios a la vez** | — | Varios LOW | múltiples bits | 2 (Neutral) |

> **Cable común (azul):** terminal compartido por los 4 contactos; va siempre a GND del MCP23017.
> **Neutral:** es el estado de reposo de la palanca — no existe un cable "SW N". Cuando la palanca está en N, ningún contacto está cerrado y los 4 pines GPA permanecen en HIGH por los pull-ups internos.

**Lógica activa-baja (active-low):**
- Sin pulsar: pull-up interno → GPA = HIGH (3.3 V)
- Pulsado: interruptor cierra a GND → GPA = LOW (0 V)

> **Referencia firmware:** `shifter_input.cpp` líneas 84-106 — función `decodeGear()`

### 3.3 Tensiones en cada punto

| Punto del circuito | Tensión | Corriente |
|---------------------|---------|-----------|
| VDD del MCP23017 | **3.3 V** | ~1 mA (quiescent) |
| GPA0-GPA3 (sin pulsar) | **3.3 V** (pull-up) | ~0 µA (estático) |
| GPA0-GPA3 (pulsado) | **0 V** (GND) | ~33 µA (3.3V / 100kΩ) |
| SDA/SCL (activo) | **0–3.3 V** | ~0.7 mA (4.7kΩ pull-up) |
| Interruptores de la palanca | **0 V** (contacto a GND) | ~33 µA |

**Conclusión: En ningún punto del circuito de la palanca hay 12 V ni 24 V.**

---

## 4. Cableado Físico Paso a Paso

### 4.1 Materiales necesarios

1. **MCP23017** — chip expansor I2C (DIP-28 o módulo breakout)
2. **2× resistencias 4.7 kΩ** — pull-ups I2C (si no están en módulo)
3. **1× condensador 100 nF cerámico** — desacoplo VDD
4. **5 interruptores SPST** — contactos de la palanca (o microswitch)
5. **Cable 6 hilos** — 0.5 mm² para señal, longitud <1 m recomendado
6. **Conector** — JST o Dupont para la palanca

### 4.2 Instrucciones de cableado

**Paso 1: Alimentar el MCP23017**
```
3.3V del ESP32 (o regulador) ──► VDD (pin 9) del MCP23017
GND                           ──► VSS (pin 10) del MCP23017
3.3V                          ──► RESET (pin 18) del MCP23017
```
> ⚠️ Alimentar con **3.3 V**, NO con 5 V ni 12 V.

**Paso 2: Configurar dirección I2C (0x20)**
```
GND ──► A0 (pin 15)
GND ──► A1 (pin 16)
GND ──► A2 (pin 17)
```

**Paso 3: Conectar I2C al ESP32-S3**
```
ESP32 GPIO 8  ──► SDA (pin 13) del MCP23017
ESP32 GPIO 9  ──► SCL (pin 12) del MCP23017

4.7kΩ entre GPIO 8 y 3.3V  (pull-up SDA)
4.7kΩ entre GPIO 9 y 3.3V  (pull-up SCL)
```
> Si se usa un módulo breakout, los pull-ups suelen estar incluidos.

**Paso 4: Conectar los cables de la palanca**

La palanca tiene **5 cables** en total: 1 común (azul) + 4 cables de señal.
Verificación con multímetro (continuidad palanca → cable común al mover la palanca):

| Posición | Cable de señal | Conectar a |
|---|---|---|
| P  | azul + morado    | GPA0 (pin 21) |
| D2 | azul + verde     | GPA1 (pin 22) |
| D1 | azul + amarillo  | GPA2 (pin 23) |
| R  | azul + blanco    | GPA3 (pin 24) |
| N  | *(sin cable)*    | — (estado de reposo, ningún contacto) |

```
Cable común AZUL ────────────────────────────────────── GND del MCP23017
GPA0 (pin 21) ──── cable ──── SW P  ──── (cable azul+morado)
GPA1 (pin 22) ──── cable ──── SW D2 ──── (cable azul+verde)
GPA2 (pin 23) ──── cable ──── SW D1 ──── (cable azul+amarillo)
GPA3 (pin 24) ──── cable ──── SW R  ──── (cable azul+blanco)
GPA4 (pin 25) ──── sin uso (dejar al aire o a GND mediante 10 kΩ opcional)
```

> **N no tiene cable propio** — si al medir continuidad no encuentras un par
> de cables que cierre cuando la palanca está en N, **es normal**. La posición
> Neutral se detecta por ausencia de señal en los otros cuatro contactos.

**Paso 5: Desacoplo**
```
100 nF cerámico entre VDD (pin 9) y VSS (pin 10) — lo más cerca posible del chip
```

### 4.3 Verificación con multímetro

| Medición | Valor esperado | Si no coincide |
|----------|---------------|----------------|
| VDD − VSS | 3.3 V ± 5% | Verificar alimentación |
| GPA0-GPA3 sin pulsar − GND | ~3.3 V | Pull-ups no habilitados (revisar firmware) |
| GPA0-GPA3 pulsado − GND | <0.3 V | OK: interruptor cerrado a GND |
| SDA sin actividad − GND | ~3.3 V | Pull-up I2C OK |
| SCL sin actividad − GND | ~3.3 V | Pull-up I2C OK |

---

## 5. Interruptor de Tracción 2WD/4WD

El interruptor de tracción es aún más simple que la palanca: un solo
interruptor SPST conectado directamente a un GPIO del ESP32-S3.

### 5.1 Circuito

```
  ESP32-S3 GPIO 15
    (pull-up interno ~45 kΩ a 3.3 V)
         │
         │
         ├──── Interruptor DPDT (usado como SPST) ──── GND
         │
    HIGH = 2WD (contacto abierto, pull-up activo)
    LOW  = 4WD (contacto cerrado a GND)
```

### 5.2 Lógica

| Posición interruptor | GPIO 15 | Modo | Seguridad |
|----------------------|---------|------|-----------|
| Abierto (pull-up) | HIGH | 2WD | ✅ Modo seguro por defecto |
| Cerrado a GND | LOW | 4WD | — |
| Cable roto | HIGH | 2WD | ✅ Fail-safe a 2WD |

### 5.3 Parámetros de debounce

| Parámetro | Valor | Referencia |
|-----------|-------|------------|
| Intervalo de muestreo | 10 ms | `traction_switch.h:35` — `pollMs` |
| Debounce mínimo | 50 ms | `traction_switch.h:33` — `debounceMs` |
| Lecturas estables requeridas | 3 consecutivas | `traction_switch.h:34` — `stableCount` |
| Puerta de velocidad | ≤ 0.5 km/h | `traction_switch.h:36` — `speedThreshKmh` |

> **Referencia firmware:** `traction_switch.cpp` líneas 39-43, 72-123

---

## 6. Comunicación CAN — De la Palanca al Motor

### 6.1 Flujo completo

```
  ┌────────────────────────────────────────────────────────────────────────┐
  │                              ESP32-S3                                  │
  │                                                                        │
  │  1. shifter::update()    → Lee MCP23017 vía I2C cada 50 ms            │
  │  2. decodeGear(portVal)  → Decodifica: P=0, R=1, N=2, D1=3, D2=4     │
  │  3. traction_sw::update()→ Lee GPIO 15, aplica debounce + speed gate  │
  │  4. Construye trama CAN:                                               │
  │       CAN ID = 0x102 (CMD_MODE)                                        │
  │       Byte 0 = mode flags (bit 0 = 4WD)                               │
  │       Byte 1 = gear (0-4)                                              │
  │  5. Envía trama CAN                                                    │
  └───────────────┬────────────────────────────────────────────────────────┘
                  │ CAN Bus (500 kbps)
                  ▼
  ┌────────────────────────────────────────────────────────────────────────┐
  │                            STM32G474RE                                 │
  │                                                                        │
  │  1. Recibe 0x102 en FDCAN1 RX FIFO                                    │
  │  2. Valida DLC ≥ 2                                                     │
  │  3. Byte 0: procesa flags de modo 4x4/tank-turn                       │
  │  4. Byte 1: valida rango (≤ 4) + velocidad (≤ 1.0 km/h)              │
  │  5. Traction_SetGear(gear)  → Almacena marcha                         │
  │  6. Traction_Update() cada 10 ms:                                      │
  │       - Aplica escalado de potencia según marcha                       │
  │       - Genera PWM 20 kHz para BTS7960                                 │
  │       - Controla dirección de giro (RPWM/LPWM)                        │
  └────────────────────────────────────────────────────────────────────────┘
```

### 6.2 Trama CAN 0x102 (CMD_MODE)

| Byte | Campo | Valores | Descripción |
|------|-------|---------|-------------|
| 0 | Mode flags | bit 0: 0=2WD, 1=4WD; bit 1: tank turn | Configuración de tracción |
| 1 | Gear | 0=P, 1=R, 2=N, 3=D1, 4=D2 | Posición de la palanca |

> **Referencia ESP32:** `can_ids.h` línea 30 — `CAN_ID_CMD_MODE = 0x102`  
> **Referencia STM32:** `can_handler.h` línea 25 — `#define CAN_ID_CMD_MODE 0x102`  
> **Referencia STM32:** `can_handler.c` líneas 430-443 — handler de recepción

---

## 7. Escalado de Potencia por Marcha

El STM32 aplica un factor de escala a la demanda del pedal según la marcha
seleccionada:

### 7.1 Tabla de escalado

| Marcha | Factor de potencia | Velocidad máxima | Uso previsto |
|--------|-------------------|-------------------|--------------|
| **P (Park)** | 30% hold brake | 0 km/h (detenido) | Freno activo de retención |
| **R (Reverse)** | 60% | Limitada | Marcha atrás |
| **N (Neutral)** | 0% (coast) | — | Rueda libre, sin tracción |
| **D1 (Drive 1)** | 60% | Media | Conducción normal / niños |
| **D2 (Drive 2)** | 100% | Máxima | Conducción con potencia completa |

### 7.2 Código de escalado

```c
/* motor_control.c líneas 82-84 */
#define GEAR_POWER_FORWARD_PCT      0.60f   /* D1: 60% máximo */
#define GEAR_POWER_FORWARD_D2_PCT   1.00f   /* D2: 100% máximo */
#define GEAR_POWER_REVERSE_PCT      0.60f   /* R:  60% máximo */
```

**Orden de aplicación:**
```
Pedal → Filtro EMA → Rampa → Demanda base
  → Frenado dinámico → Demanda efectiva
  → Escalado por marcha (×0.60 o ×1.00) → Demanda escalada
  → base_pwm = |demanda| × PWM_PERIOD / 100
  → Dirección (RPWM/LPWM según marcha)
  → Per-wheel: base_pwm × wheel_scale[i] → Motor_SetPWM
```

> **Referencia firmware:** `motor_control.c` líneas 521-531

---

## 8. Protecciones y Seguridad

### 8.1 Puerta de velocidad (Speed Gate)

El STM32 **rechaza** cambios de marcha si el vehículo se está moviendo:

| Cambio | Velocidad máxima permitida | Referencia |
|--------|---------------------------|------------|
| Cualquier cambio de marcha | ≤ 1.0 km/h | `can_handler.c:441` |
| Cambio 2WD ↔ 4WD | ≤ 0.5 km/h | `traction_switch.h:36` |

### 8.2 Validaciones del STM32

```c
/* can_handler.c líneas 433-443 */
if (msg_len >= 2) {                                    // ① DLC ≥ 2
    uint8_t gear_raw = rx_payload[1];
    if (gear_raw <= (uint8_t)GEAR_FORWARD_D2) {        // ② Rango 0-4
        GearPosition_t requested = (GearPosition_t)gear_raw;
        float avg_spd = (Wheel_GetSpeed_FL() + ...);
        if (avg_spd <= 1.0f) {                         // ③ Speed gate
            Traction_SetGear(requested);
        }
    }
}
```

### 8.3 Fallos de la palanca y respuestas

| Fallo | Detección | Respuesta |
|-------|-----------|-----------|
| MCP23017 no responde (I2C NACK) | `readReg()` devuelve 0xFF | Marcha congelada en última válida |
| Múltiples marchas activas | `popcount != 1` | Fuerza Neutral |
| Ninguna marcha activa | `popcount == 0` | Fuerza Neutral |
| Cable de interruptor roto | Pin queda HIGH (pull-up) | La marcha afectada no se puede seleccionar |
| Cable de I2C roto | MCP23017 no responde | Marcha congelada en última válida |
| Ruido eléctrico en I2C | Lectura corrupta | Decoded como Neutral (< 2 bits activos) |

> **Referencia firmware:** `shifter_input.cpp` líneas 60, 66-83

### 8.4 Inhibición al arranque

Tras cada encendido o reset del MCU, los motores están **bloqueados** hasta
que el pedal se mantenga por debajo del 3% durante 400 ms continuos. Esto evita
movimiento inesperado si la palanca estaba en D1/D2 al arrancar.

> **Referencia firmware:** `main.c` líneas 49-50 — `STARTUP_PEDAL_REST_PCT`,
> `STARTUP_PEDAL_CLEAR_MS`

---

## 9. ¿Qué pasa si mi palanca original es de 12 V?

Muchos vehículos eléctricos infantiles y de juguete vienen con una palanca que
está cableada directamente a la batería de 12 V. Si tu palanca original tiene
este tipo de cableado, **NO la puedes conectar directamente al MCP23017 ni al
ESP32**.

### 9.1 Caso 1: La palanca tiene interruptores simples (contactos secos)

**Solución: conexión directa al MCP23017.** No hay problema.

La mayoría de las palancas tipo "joystick" son simplemente microswitches que
cierran contactos. No importa que estuvieran cableadas a 12 V en el sistema
original — los interruptores en sí mismos son contactos secos (sin
polaridad ni tensión propia).

```
  Palanca original:
  
  ¿Tiene cables que van a interruptores/microswitches? → SÍ → Desconectar
  de la batería 12V y reconectar entre GPA0-GPA3 y el cable común a GND del MCP23017.
  Los interruptores son idénticos sin importar la tensión del sistema original.
```

### 9.2 Caso 2: La palanca tiene electrónica integrada (LED, solenoide, PCB)

**Solución: aislar la señal.**

Si la palanca tiene componentes activos que necesitan 12 V (LEDs indicadores,
solenoides de bloqueo, placa de circuito impreso), hay dos opciones:

**Opción A — Separar señales de potencia:**
- Alimentar los LEDs/solenoide desde 12 V por un circuito independiente.
- Conectar solo los contactos de posición al MCP23017 (GPA0-GPA3 → SW → cable común → GND).

**Opción B — Optoacoplador (aislamiento galvánico):**
```
  12V señal palanca ──── R 1kΩ ──── LED del optoacoplador ──── GND
                                          │
                                    ┌─────┴─────┐
                                    │ PC817     │
                                    │ o 6N137   │
                                    └─────┬─────┘
                                          │
  MCP23017 GPA0 ◄────────────────────────┘
  (pull-up 100kΩ interno)
```
Cuando la señal de 12 V activa el LED del optoacoplador, el fototransistor
conecta GPA0 a GND → lectura LOW → marcha detectada.

### 9.3 Caso 3: Usar la palanca como interruptor simple de 2 posiciones

Si la palanca original solo tiene "Forward / Reverse" (sin P/N/D2):

| Terminal original | Conectar a |
|-------------------|------------|
| Forward | GPA2 (D1) — y al cable común (a GND) |
| Reverse | GPA3 (R)  — y al cable común (a GND) |
| GPA0 (P) | Interruptor independiente (botón) — segundo terminal a GND |
| GPA1 (D2) | No conectar (se ignora) |
| Neutral | No requiere cable: estado por defecto cuando ningún SW está cerrado |

---

## 10. Lista de Componentes

### 10.1 Circuito del selector de marchas

| Qty | Componente | Valor / Modelo | Función |
|-----|-----------|----------------|---------|
| 1 | MCP23017 | DIP-28 (o módulo breakout) | Expansor I/O I2C |
| 2 | Resistencia | 4.7 kΩ, ¼ W | Pull-ups I2C (SDA, SCL) |
| 1 | Condensador | 100 nF cerámico | Desacoplo VDD MCP23017 |
| 5 | Interruptor SPST | Microswitch o contacto de palanca | P, R, N, D1, D2 |
| 1 | Cable | 6 hilos × 0.5 mm², <1 m | SDA, SCL, VDD, GND, señales |

### 10.2 Circuito del interruptor de tracción

| Qty | Componente | Valor / Modelo | Función |
|-----|-----------|----------------|---------|
| 1 | Interruptor DPDT | Rocker ON-ON latching | 2WD / 4WD |
| 1 | Cable | 2 hilos × 0.5 mm² | GPIO 15 + GND |

### 10.3 Solo si la palanca original es de 12 V con electrónica

| Qty | Componente | Valor / Modelo | Función |
|-----|-----------|----------------|---------|
| 5 | Optoacoplador | PC817 o 6N137 | Aislamiento 12 V → 3.3 V |
| 5 | Resistencia | 1 kΩ, ¼ W | Limitación corriente LED opto |

---

## 11. Preguntas Frecuentes

### ¿Por qué no leer la palanca directamente con los GPIOs del ESP32?

Porque el ESP32-S3 ya tiene **la mayoría de sus GPIOs ocupados** por:
- Pantalla TFT ST7796: 8 pines (GPIO 10, 12, 13, 14, 21, 38, 39, 42)
- CAN bus TJA1051: 2 pines (GPIO 4, 5)
- Sensor obstáculos: 1 pin (GPIO 18)
- Audio DFPlayer: 2 pines (GPIO 43, 44)
- Llave de contacto: 2 pines (GPIO 40, 41)
- Interruptor 2WD/4WD: 1 pin (GPIO 15)
- MCP23017 I2C: 2 pines (GPIO 8, 9)

Total: 18 pines. Usando GPIOs directos para la palanca serían 23.
El MCP23017 permite leer las 4 posiciones físicas (P, D2, D1, R) con solo 2 pines I2C.
La posición N es implícita (estado de reposo, no requiere pin).

### ¿El STM32 lee la palanca?

**NO.** El STM32 no tiene ningún GPIO configurado para la palanca.
Recibe la información exclusivamente vía CAN (ID 0x102, byte 1).

> **Referencia:** `TRACTION_GEAR_SHIFTER_AUDIT.md` sección 4.1

### ¿Puedo cambiar de marcha en movimiento?

**NO.** El STM32 rechaza cambios de marcha si la velocidad promedio
de las 4 ruedas supera 1.0 km/h. El comando CAN se ignora silenciosamente.

### ¿Qué marcha se usa al arrancar?

Al encender, el ESP32 lee la posición actual de la palanca e inmediatamente
envía el valor por CAN. Sin embargo, los motores están bloqueados por la
**inhibición de arranque** (`startup_inhibit`) hasta que el pedal se suelte
durante 400 ms.

### ¿Puedo usar la palanca sin el MCP23017 (conexión directa a GPIOs)?

Sí, pero requiere:
1. Tener 5 GPIOs libres en el ESP32-S3 (difícil con la configuración actual).
2. Modificar `shifter_input.cpp` para leer GPIOs directos en vez del MCP23017.
3. Usar pull-ups internos del ESP32 en cada pin.

No se recomienda porque el firmware actual está diseñado para MCP23017.

### ¿Hay alguna relación entre la marcha y el modo 2WD/4WD?

**NO.** Son completamente independientes:
- Marcha (P/R/N/D1/D2) controla **potencia y dirección** de los motores.
- Modo (2WD/4WD) controla **qué ruedas** reciben tracción.
- Ambos se envían en la misma trama CAN (0x102) pero en bytes diferentes
  (byte 0 = modo, byte 1 = marcha).

> **Referencia:** `TRACTION_GEAR_SHIFTER_AUDIT.md` sección 1.5

---

> **Documento generado a partir del firmware (`esp32/src/shifter_input.h`,
> `esp32/src/shifter_input.cpp`, `esp32/src/traction_switch.h`,
> `esp32/src/traction_switch.cpp`, `esp32/src/main.cpp`,
> `Core/Src/motor_control.c`, `Core/Inc/motor_control.h`,
> `Core/Src/can_handler.c`) y de la documentación existente
> (`SENSOR_INTERFACE.md`, `HARDWARE_WIRING_MANUAL.md`,
> `TRACTION_GEAR_SHIFTER_AUDIT.md`). No contiene hardware inventado.**
